const uint16_t FLAG_NETA_LONG_CONTROL = 1;

// bool controls_allowed = false;
bool neta_longitudinal = true;
bool neta_set_button_prev = false;
bool neta_resume_button_prev = false;

uint8_t neta_crc8_lut_8h2f[256]; // Static lookup table for CRC8 poly 0x2F, aka 8H2F/AUTOSAR
bool neta_brake_pedal_switch = false;
bool brake_pressure_detected = false;

// 车速-rate 限制
//  angle_deg_to_can: 本地变量是int，转为can的degree的1/factor
const SteeringLimits NETA_STEERING_LIMITS = {
  .angle_deg_to_can = 10,
  .angle_rate_up_lookup = {
    {0., 5., 15.},
    {10., 1.6, .3}
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {10., 7.0, .8}
  },
};

// longitudinal limits
// acceleration in m/s2 * 100 to avoid floating point math
const LongitudinalLimits NETA_LONG_LIMITS = {
  .max_accel = 200, // 1/100 m/s2
  .min_accel = -350, // 1/100 m/s2
};

#define IDB_Fr04_0C7    0x0C7   // RX from ABS, for wheel speeds
#define EPS_Fr01_0B1    0x0B1   // RX from EPS, for driver steering torque
#define EPS_Fr02_0B2    0x0B2   // RX from EPS, for driver steering torque
#define IDB_Fr01_0E5    0x0E5   // RX from ABS, for brake switch state
#define VCU_Fr05_0E3    0x0E3   // RX from ECU, for driver throttle input
#define IDB_Fr03_0C5    0x0C5   // RX from ECU, speed
// #define MSG_MOTOR_14    0x3BE   // RX from ECU, for brake switch status

#define BDCS_Fr16_114   0x114   // RX from HMI, ACC control buttons for cancel/resume

// OP TX
#define ADCS_Fr02_08E   0x08E   // TX by OP, ACC control and LCC control
#define ADCS_Fr08_193   0x193   // TX by OP, status
#define ADCS_F12_136    0x136   // TX by OP, HMI


// Transmit of GRA_ACC_01 is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
// cansend都会检查是否可以发送 can0和can2报文互通
//  trans 2 ---> 0
const CanMsg NETA_STOCK_TX_MSGS[] = { {0x8f, 0, 32}, {0xfe, 0, 32}, {0x190, 0, 64}, {0x191, 0, 64}, {0x192, 0, 64}, {0x194, 0, 64}, 
                                      {0x210, 0, 8}, {0x255, 0, 8}, {0x265, 0, 8}, {0x301, 0, 8}, {0x607, 0, 8}, 
                                      {0x8f, 2, 32}, {0xfe, 2, 32}, {0x190, 2, 64}, {0x191, 2, 64}, {0x192, 2, 64},  {0x194, 2, 64},
                                      {0x210, 2, 8}, {0x255, 2, 8}, {0x265, 2, 8}, {0x301, 2, 8}, {0x607, 2, 8} };

const CanMsg NETA_LONG_TX_MSGS[] = {{ADCS_Fr02_08E, 0, 16}, {ADCS_Fr08_193, 0, 64}, {ADCS_F12_136, 0, 16}};

RxCheck neta_rx_checks[] = {
  // {.msg = {{IDB_Fr04_0C7, 0, 32, .check_checksum = false, .max_counter = 0U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  // {.msg = {{EPS_Fr01_0B1, 0, 8, .check_checksum = false, .max_counter = 0U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  // {.msg = {{IDB_Fr01_0E5, 0, 8, .check_checksum = false, .max_counter = 0U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  // // {.msg = {{MSG_TSK_06, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  // {.msg = {{VCU_Fr05_0E3, 0, 8, .check_checksum = false, .max_counter = 0U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  // // {.msg = {{MSG_MOTOR_14, 0, 8, .check_checksum = false, .max_counter = 0U, .expected_timestep = 100000U}, { 0 }, { 0 }}},
};
// #define NETA_ADDR_CHECKS_LEN (sizeof(neta_addr_checks) / sizeof(neta_addr_checks[0]))
// addr_checks neta_rx_checks = {neta_addr_checks, NETA_ADDR_CHECKS_LEN};


// 函数 根据 START_BIT DATA_LENGTH 用于解析CAN总线数据
static int parse_can_data(const unsigned char *can_data, int start_bit, int data_length) {
  // 计算所需的字节数
  int start_byte = start_bit / 8;
  int end_byte = (start_bit + data_length - 1) / 8;

  // 由于CAN总线数据是低位在前，因此需要反转字节顺序
  int reversed_data = 0;
  for (int i = end_byte; i >= start_byte; i--) {
      reversed_data = (reversed_data << 8) | can_data[i];
  }
  // 计算偏移位数
  int offset_bits = start_bit % 8;
  // 移位以获取解析后的数据
  int parsed_data = (reversed_data >> offset_bits) & ((1 << data_length) - 1);

  return parsed_data;
}

static uint32_t neta_get_checksum(const CANPacket_t *to_push) {
  return (uint8_t)GET_BYTE(to_push, 0);
}

static uint8_t neta_get_counter(const CANPacket_t *to_push) {
  // MQB message counters are consistently found at LSB 8.
  return (uint8_t)GET_BYTE(to_push, 1) & 0xFU;
}

static uint32_t neta_compute_crc(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);

  // This is CRC-8H2F/AUTOSAR with a twist. See the OpenDBC implementation
  // of this algorithm for a version with explanatory comments.

  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc ^= (uint8_t)GET_BYTE(to_push, i);
    crc = neta_crc8_lut_8h2f[crc];
  }

  uint8_t counter = neta_get_counter(to_push);
  if (addr == MSG_LH_EPS_03) {
    crc ^= (uint8_t[]){0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5,0xF5}[counter];
  } else if (addr == MSG_ESP_05) {
    crc ^= (uint8_t[]){0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07}[counter];
  } else if (addr == MSG_TSK_06) {
    crc ^= (uint8_t[]){0xC4,0xE2,0x4F,0xE4,0xF8,0x2F,0x56,0x81,0x9F,0xE5,0x83,0x44,0x05,0x3F,0x97,0xDF}[counter];
  } else if (addr == MSG_MOTOR_20) {
    crc ^= (uint8_t[]){0xE9,0x65,0xAE,0x6B,0x7B,0x35,0xE5,0x5F,0x4E,0xC7,0x86,0xA2,0xBB,0xDD,0xEB,0xB4}[counter];
  } else {  // Undefined CAN message, CRC check expected to fail
  }
  crc = neta_crc8_lut_8h2f[crc];

  return (uint8_t)(crc ^ 0xFFU);
}

static safety_config neta_init(uint16_t param) {
  UNUSED(param);

  // 先默认设置为都可控
  controls_allowed = true;
  
  neta_set_button_prev = false;
  neta_resume_button_prev = false;
  neta_brake_pedal_switch = false;
  brake_pressure_detected = false;

  gen_crc_lookup_table_8(0x2F, neta_crc8_lut_8h2f);

  neta_longitudinal = true;

  return BUILD_SAFETY_CFG(neta_rx_checks,NETA_LONG_TX_MSGS);
}

static void neta_rx_hook(const CANPacket_t *to_push) {
  // 输入输出做校验
  // TODO
  // bool valid = addr_safety_check(to_push, &neta_rx_checks,
  //                                neta_get_checksum, neta_compute_crc, neta_get_counter, NULL);
  bool valid = true;

  if (valid && (GET_BUS(to_push) == 0U)) {
    int addr = GET_ADDR(to_push);
    if (addr == EPS_Fr02_0B2){
      // eps 没报错的情况下
      int eps_avaiable = parse_can_data(to_push->data, 0, 1);
      if(eps_avaiable == 1 ) {
        controls_allowed = true;
      }
    }

    // speed
    // ret.vEgoRaw = pt_cp.vl["IDB_Fr03_0C5"]["IDB3_VehicleSpd"] * CV.KPH_TO_MS
    if (addr == IDB_Fr03_0C5) {
      float speed = parse_can_data(to_push->data, 5, 14)*0.03125;
      vehicle_moving = speed > 0.5;
    }

    // if (addr == ADCS_Fr08_193){
    //   // ADCS8_ACCState : 26|3@0+ (1,0) [0|7] "NoUnit"  ACU,IDB1,ADCU_FD3,GW_FD4,IDB2,EPS2,GW_FD1,ICU,RCU1,CDCS
    //   int acc_status = parse_can_data(to_push->data, 26, 3);
    //   if(acc_status == 2 || acc_status == 4) {
    //     controls_allowed = true;
    //   }
    // }

    if (addr == VCU_Fr05_0E3) {
      int gas = parse_can_data(to_push->data, 39, 8);
      gas_pressed = gas > 5;
    }
    
    // if (addr == BDCS_Fr16_114) {
    //   // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
    //   // Always exit controls on main switch off
    //   // CS1_GearPositionReqSt : 19|3@0+ (1,0) [0|7] "NoUnit"  ADAS,FLC_FD3
    //   int gear_state = parse_can_data(to_push->data, 19, 3);
    //   if (gear_state == 0x5) {
    //     controls_allowed = true;
    //   }
    // }

    // 如果刹车踏板被踩下，不允许发送ACC控制报文
    if(addr == IDB_Fr01_0E5){
      // IDB1_BrakePedalApplied : 0|1@0+ (1,0) [0|1] "NoUnit"  ADAS,FLC_FD3
      brake_pressure_detected = parse_can_data(to_push->data, 0, 1);
      if(brake_pressure_detected){
        controls_allowed = false;
      }
    }

    brake_pressed = brake_pressure_detected;
    // TODO 2024.02.14
    generic_rx_checks((addr == ADCS_Fr02_08E));
  }
}

// 这个是panda主动发送部分，panda接收到的数据转发给其他can的会由fwd_hook处理
static bool neta_tx_hook(const CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);
  bool tx = true;
  // steering and ACC check
  if (addr == ADCS_Fr02_08E) {
    bool violation_lcc = false;
    // Steering control
    // ADCS2_ADAS_EPSAngleReq : 29|14@0+ (0.1,-720) [-720|720] "degree"  ACU,EPS1,EPS2,GW_FD1
    int raw_angle_can = parse_can_data(to_send->data, 29, 14);
    int desired_angle = raw_angle_can - 720*10;

    // ADCS2_ADAS_EPSLateralCtrlType : 14|4@0+ (1,0) [0|15] "NoUnit"  ACU,EPS1,EPS2,GW_FD1
    // VAL_ 142 ADCS2_ADAS_EPSLateralCtrlType 7 "Reserved" 5 "Reserved" 4 " ELK Mode" 3 "NNP Mode" 2 "Pilot Mode" 1 "LDP Mode" 0 "None" ;
    int steer_control_type = parse_can_data(to_send->data, 14, 4);
    bool steer_control_enabled = (steer_control_type != 0);  // NONE

    if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, NETA_STEERING_LIMITS)) {
      violation_lcc = true;
    }

    // ACC control
    bool violation_acc = false;
    // ADCS2_LongitudCtrlTargetAccel : 111|11@0+ (0.01,-10.23) [-10.23|10.24] "m/s^2"  ACU,IDB1,IDB2,EPS2,GW_FD1,RCU1
    int desired_accel_raw = parse_can_data(to_send->data, 111, 11);
    int desired_accel = desired_accel_raw - 1023;
    violation_acc = longitudinal_accel_checks(desired_accel, NETA_LONG_LIMITS);

    // 不允许发送ACC控制报文
    if (!violation_lcc || !violation_acc || !controls_allowed) {
      tx = false;
    }
  }

  // op控制本身只会发送部分报文到can0
  bool tx1 = false;
  if(bus == 0){
    tx1 = msg_allowed(to_send, NETA_LONG_TX_MSGS, sizeof(NETA_LONG_TX_MSGS) / sizeof(NETA_LONG_TX_MSGS[0]));
  }

  tx = tx && tx1;
  // 1 allows the message through
  return tx ;
}

// receive can data , trans to bus
static int neta_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;
  int tx=0, tx1=0, tx2=0;
  switch (bus_num) {
    case 0:
      // Forward all traffic from the Extended CAN onward
      // 这个就是从0转发到2
      tx1 = msg_allowed_yj(2, addr, NETA_LONG_TX_MSGS, sizeof(NETA_LONG_TX_MSGS) / sizeof(NETA_LONG_TX_MSGS[0]));
      tx2 = msg_allowed_yj(2, addr, NETA_STOCK_TX_MSGS, sizeof(NETA_STOCK_TX_MSGS) / sizeof(NETA_STOCK_TX_MSGS[0]));
       if (tx1 || tx2) {
        // openpilot takes over acc and lcc
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway
        bus_fwd = 2;
      }

      bus_fwd = 2;
      break;
    case 2:
      tx = msg_allowed_yj(0, addr, NETA_LONG_TX_MSGS, sizeof(NETA_LONG_TX_MSGS) / sizeof(NETA_LONG_TX_MSGS[0]));
      // tx1 = msg_allowed_yj(bus_num, addr, NETA_STOCK_TX_MSGS, sizeof(NETA_STOCK_TX_MSGS) / sizeof(NETA_STOCK_TX_MSGS[0]));
      if (tx) {
        // openpilot takes over acc and lcc
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway
        bus_fwd = 0;
      }
      break;
    default:
      // No other buses should be in use; fallback to do-not-forward
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

const safety_hooks neta_hooks = {
  .init = neta_init,
  .rx = neta_rx_hook,
  .tx = neta_tx_hook,
  .fwd = neta_fwd_hook,
  .get_counter = neta_get_counter,
  .get_checksum = neta_get_checksum,
  .compute_checksum = neta_compute_crc,
};
