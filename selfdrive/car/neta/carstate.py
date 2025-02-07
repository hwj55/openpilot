import copy
import time
import numpy as np
from cereal import car, custom
from typing import Dict, Optional
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.neta.values import DBC, CanBus, GearShifter, CarControllerParams,  NETA_CARS, CruiseButtons
from collections import deque
from openpilot.system.hardware import PC
from openpilot.common.simple_kalman import KF1D, get_kalman_gain

PREV_BUTTON_SAMPLES = 8

class FirstOrderLowPassFilter:
  def __init__(self, cutoff_freq, sample_freq):
    self.a = cutoff_freq / sample_freq
    self.b = 1 - self.a
    self.prev_value = 0

  def filter(self, value):
    output = self.a * value + self.b * self.prev_value
    self.prev_value = output
    return output


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)

    self.cruise_buttons = CruiseButtons.CANCEL
    self.prev_cruise_buttons = CruiseButtons.CANCEL
    self.cruise_setting = 0
    # self.cruise_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    # self.main_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)

    self.esp_hold_confirmation = False
    self.upscale_lead_car_signal = False

    # debug
    self.debug = False
    self.debug1 = False
    self.enable_op_acc_state_manage = True # 是否使用OP本地acc状态机控制  一般用原车
    self.enable_op_lcc_state_manage = True # 是否使用OP本地lcc状态机控制   默认用op
    # -YJ-  acc status: 1:ON  0:OFF
    self.curise_speed_set_cool_counter = 0
    self.acc_enable_cool_counter = 0
    self.acc_en = 0
    self.eps_error_time_pre = 0  # EPS报错后不立马恢复
    self.gear_D_time_pre = 0 # 上一次拨杆D的时间
    self.no_gear_time = 0 # 没有拨杆时间
    self.trigger_gear_D_time_pre = 0 # 拨杆D的时间
    self.gear_shifter_raw = 0 # 当前档位信号

    self.cruise_speed = 0
    self.acc_state = 0
    self.acc_enabled = False
    self.acc_enabled_pre = False
    self.acc_available = False
    # enum LongitudinalPersonality {
    #   aggressive @0;
    #   standard @1;
    #   relaxed @2;
    # }
    #  neta  1,2,3是有效，0 是无效
    self.acc_distance_list = [1.25, 1.5, 1.75]
    self.acc_distance_index_set = 2

    self.lccButton = False
    self.lccButton_pre = False
    self.lcc_raw_steer_req = 0
    self.steer_takeover = False # 驾驶员接管

    # neta_s stock values
    self.Fr08_193_raw_msg = {}
    self.Fr02_08E_raw_msg = {}
    self.F12_136_raw_msg = {}

    self.steer_torque_lpf = FirstOrderLowPassFilter(12, 100)

    # torque kf
    DT_CarState = 0.01
    Q = [[5.0, 0.0], [0.0, 100.0]]
    R = 0.5
    A = [[1.0, DT_CarState], [0.0, 1.0]]
    C = [[1.0, 0.0]]
    x0=[[0.0], [0.0]]
    K = get_kalman_gain(DT_CarState, np.array(A), np.array(C), np.array(Q), R)
    self.steer_torque_kf = KF1D(x0=x0, A=A, C=C[0], K=K)

  def update_steer_torque_kf(self, x_raw):
    if abs(x_raw - self.steer_torque_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.steer_torque_kf.set_x([[x_raw], [0.0]])

    x_filter = self.steer_torque_kf.update(x_raw)
    return float(x_filter[0]), float(x_filter[1])


  def parse_gear_shifter_neta_s(self, gear: Optional[str]) -> car.CarState.GearShifter:
    if gear is None:
      return GearShifter.unknown

    d: Dict[str, car.CarState.GearShifter] = {
      'Park': GearShifter.park, 'PARK': GearShifter.park,
      'Reverse': GearShifter.reverse, 'REVERSE': GearShifter.reverse,
      'Neutral': GearShifter.neutral, 'NEUTRAL': GearShifter.neutral,
      # 'E': GearShifter.eco, 'ECO': GearShifter.eco,
      # 'T': GearShifter.manumatic, 'MANUAL': GearShifter.manumatic,
      'Drive': GearShifter.drive, 'DRIVE': GearShifter.drive,
      'S': GearShifter.sport, 'SPORT': GearShifter.sport,
      # 'L': GearShifter.low, 'LOW': GearShifter.low,
      # 'B': GearShifter.brake, 'BRAKE': GearShifter.brake,
    }
    return d.get(gear.upper(), GearShifter.unknown)

  def create_button_events(self, pt_cp, buttons):
    button_events = []

    for button in buttons:
      state = pt_cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events


  def update(self, pt_cp, cam_cp):
    if self.CP.carFingerprint in NETA_CARS:
      # TODO
      return self.update_neta_s(pt_cp, cam_cp)
    else:
      return self.update_neta_s(pt_cp, cam_cp)


  # # --------------------------
  def update_neta_s(self, pt_cp, cam_cp):
    time_cur = time.monotonic()

    # Messages needed by carcontroller
    self.Fr08_193_raw_msg = copy.copy(cam_cp.vl["ADCS_Fr08_193"])
    self.Fr02_08E_raw_msg = copy.copy(cam_cp.vl["ADCS_Fr02_08E"])
    self.F12_136_raw_msg = copy.copy(cam_cp.vl["ADCS_F12_136"])

    ret = car.CarState.new_message()
    ## car speed
    # Update vehicle speed and acceleration from ABS wheel speeds.
    # fl, fr, rl, rr,  (km/h)
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["IDB_Fr04_0C7"]["IDB4_FLWhlVelocity"],
      pt_cp.vl["IDB_Fr04_0C7"]["IDB4_FRWhlVelocity"],
      pt_cp.vl["IDB_Fr04_0C7"]["IDB4_RLWhlVelocity"],
      pt_cp.vl["IDB_Fr04_0C7"]["IDB4_RRWhlVelocity"],
    )
    # vEgo obtained from Bremse_1 vehicle speed rather than Bremse_3 wheel speeds because Bremse_3 isn't present on NSF
    ret.vEgoRaw = pt_cp.vl["IDB_Fr03_0C5"]["IDB3_VehicleSpd"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # IDB5_VehicleStanstill 3 "reserved" 2 "invalid (short unavailability, max 3min)" 1 "standstill" 0 "not standstill";
    # ret.standstill = ret.vEgo <= 0.1
    ret.standstill = pt_cp.vl["IDB_Fr05_129"]["IDB5_VehicleStanstill"] == 1
    ret.yawRate = pt_cp.vl["ACU_Fr02_0C4"]["ACU2_VehicleDynYawRate"] # rad/s

    ## steering wheel
    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    ret.steeringAngleDeg = pt_cp.vl["EPS_Fr01_0B1"]["EPS1_SteeringAngle"]
    ret.steeringRateDeg = pt_cp.vl["EPS_Fr01_0B1"]["EPS1_SteerAngleSpd"]
    ret.steeringTorque = pt_cp.vl["EPS_Fr01_0B1"]["EPS1_TorsionBarTorque"]  # Nm
    # ret.steeringTorqueEps = pt_cp.vl["EPS_1"]["EPS_StrngWhlTorq"]  # Nm
    # 颠簸的话steeringTorque > 6.0
    # kf 动态性太好了，不适合用于steeringTorque
    # ret.cruiseState.steerTorqueKf , steer_torque_rate = self.update_steer_torque_kf(ret.steeringTorque)

    # data 是你的原始数据 cutoff_freq 是你的低通滤波器的截止频率  sampling_freq 是你的数据采样频率
    ret.cruiseState.steerTorqueKf = self.steer_torque_lpf.filter(ret.steeringTorque)
    self.steer_takeover = abs(ret.cruiseState.steerTorqueKf) > self.CCP.STEER_DRIVER_TAKEROVER
    ret.cruiseState.steerTakeover = self.steer_takeover
    # press还是用原始数据
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE

    # Verify EPS readiness to accept steering commands
    # External Require or EPS Temporary Failed
    # EPS_Fr02_0B2 ::  EPS2_PA_CtrlAbortFeedback 9 "Parking activation error" 8 "Request Value over range" 7 "Request Slope over range"
    # 6 "APA transmition signal error" 5 "EPS Temporary error" 4 "EPS Permanently error" 3 "Over speed" 2 "Vehicle Speed Invalid" 1 "Driver override" 0 "No error";
    EPS2_PA_CtrlAbortFeedback = pt_cp.vl["EPS_Fr02_0B2"]["EPS2_PA_CtrlAbortFeedback"]
    ret.steerFaultTemporary = EPS2_PA_CtrlAbortFeedback == 5
    ret.steerFaultPermanent = EPS2_PA_CtrlAbortFeedback == 4

    # 获取原车控制量
    ret.cruiseState.lccCmdRaw = cam_cp.vl["ADCS_Fr02_08E"]["ADCS2_ADAS_EPSAngleReq"]
    ret.cruiseState.lccCmdRawStatus = int(cam_cp.vl["ADCS_Fr02_08E"]["ADCS2_ADAS_EPSAngleReqSt"])
    ret.cruiseState.accCmdRaw = cam_cp.vl["ADCS_Fr02_08E"]["ADCS2_LongitudCtrlTargetAccel"]
    ret.cruiseState.accCmdRawStatus = int(cam_cp.vl["ADCS_Fr02_08E"]["ADCS2_LongitudCtrlAccelCtrlReq"])

    # VAL_ 178 EPS2_ADAS_Available 1 "Available" 0 "Not  Available";
    # VAL_ 178 EPS2_ADAS_Active 1 "active" 0 "not active";
    # EPS报错后不立马恢复, OP其他模块，cruiseState.epsAvailable一旦为true，就会直接发送控制指令
    # 后需要重新建立握手，需等待异常恢复（EPS2_ADASavailable=0x1 available）至少 5 个周期后(50ms)，ADAS 才可以请求建立握手
    # if EPS2_PA_CtrlAbortFeedback != 0:
    #   self.eps_error_time_pre = time.monotonic()
    # TODO
    # if EPS2_PA_CtrlAbortFeedback == 0 and EPS2_ADAS_Available and time_cur - self.eps_error_time_pre > 0.1:
    ret.cruiseState.epsAvailable = pt_cp.vl["EPS_Fr02_0B2"]["EPS2_ADAS_Available"] == 1
    ret.cruiseState.epsActive = pt_cp.vl["EPS_Fr02_0B2"]["EPS2_ADAS_Active"] == 1

    NpilotSysInfo_hmi = cam_cp.vl["ADCS_F12_136"].get("ADCS12_NPilotSysInfo", -1)

    ## gas pedal
    # Update gas, brakes, and gearshift. 0-1
    ret.gas = pt_cp.vl["VCU_Fr05_0E3"]["VCU5_AccelPosition"] / 100.0
    ret.gasPressed = ret.gas > 0.05

    ## brake pedal
    ret.brake = pt_cp.vl["IDB_Fr01_0E5"]["IDB1_BrakePedalApplied"] # TODO pt_cp.vl["EBB_2_A"]["EBB_BrkPedPst"] / 100.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["IDB_Fr01_0E5"]["IDB1_BrakePedalApplied"])
    # EPB状态
    # IDB6_EPBStatus 7 "Not Initialized (NEW)" 6 "Full Released完全释放" 5 "Dynamic braking via EPB动态驻车"
    # 4 "Releasing in progress卡钳释放中" 3 "Clamping in progress卡钳夹紧中" 2 "Clamped卡钳夹紧" 1 "Released卡钳释放" 0 "Unknown未知" ;
    ret.cruiseState.idbEPBActive = pt_cp.vl["IDB_Fr06_12B"]["IDB6_EPBStatus"] in (2, 3, 4, 5)

    # IDB7_DynParkBrkByIDBActive 1 "Active" 0 "Inactive";
    ret.parkingBrake = pt_cp.vl["IDB_Fr07_12D"]["IDB7_DynParkBrkByIDBActive"] == 1

    ## gear
    # Update gear and/or clutch position data.
    # VCU5_ActGear 7 "Invalid" 5 "S" 4 "Drive" 3 "Neutral" 2 "Reverse" 1 "Park" 0 "Default" ;
    ret.gearShifter = self.parse_gear_shifter_neta_s(self.CCP.shifter_values.get(pt_cp.vl["VCU_Fr05_0E3"]["VCU5_ActGear"]))
    self.gear_shifter_raw = pt_cp.vl["VCU_Fr05_0E3"]["VCU5_ActGear"]

    ## blink
    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    turn_left = pt_cp.vl["BDCS_Fr01_110"]["BDCS1_LeftTurnLightSt"]
    turn_right = pt_cp.vl["BDCS_Fr01_110"]["BDCS1_RightTurnLightSt"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(300, turn_left,turn_right)

    ## lock info
    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["DDCU_Fr01_1A2"]["DDCU1_FLDoorAjar"],
                        0,
                        pt_cp.vl["DDCU_Fr01_1A2"]["DDCU1_RLDoorAjar"],
                       0])
    # Update seatbelt fastened status.
    # ACU1_Driver_Buckle 3 "Not Avaliable(安全带未配置)" 2 "Fault(安全带有错)" 1 "Unbuckled(安全带未系上)" 0 "Buckled(安全带系上)" ;
    ret.seatbeltUnlatched = pt_cp.vl["ACU_Fr01_102"]["ACU1_Driver_Buckle"] == 1

    # 用户在D档时下拨怀挡两次，激活Pilot功能
    # 报文BDCS_Fr16_114，信号CS1_GearPositionReqSt---拨杆信号,连着拨两次ACC和LCC都激活
    # （拨一次CS1_GearPositionReqSt会跳变一次0X5），车辆换挡也是这个信号，复用的
    # CS1_GearPositionReqSt 7 "Gear hold（Reserved）" 6 "S（Reserved）" 5 "D" 4 "N2" 3 "N1" 2 "R" 1 "P（Reserved）" 0 "No request";
    gear_pos = pt_cp.vl["BDCS_Fr16_114"]["CS1_GearPositionReqSt"]

    # 是否使用OP本地lcc状态机控制
    if not self.enable_op_lcc_state_manage:
      ret.cruiseState.lccButton = cam_cp.vl["ADCS_Fr08_193"]["ADCS8_NPilot_SysState"] == 2
    else:
      if self.gear_shifter_raw == 4:
        if gear_pos == 0x0:
          self.no_gear_time = time_cur
        elif gear_pos == 0x5 and self.no_gear_time > 0:
          dt_0_5 = time_cur - self.no_gear_time
          # 判断是否触发了一次D档trigger
          if dt_0_5 > 0 and dt_0_5 < 0.1:
            dt_trigger_D = time_cur - self.trigger_gear_D_time_pre
            # 判断是否触发了两次D档trigger，激活功能
            if dt_trigger_D < 0.8 and dt_trigger_D > 0:
              self.lccButton = True
              if self.debug1:
                print("time_cur: ", time_cur, "dt_trigger_D: ", dt_trigger_D)
                print("time_cur: ", time_cur, "gear_pos: ", gear_pos, "lccEnbaled: 1")

            self.trigger_gear_D_time_pre = time_cur
            self.gear_D_time_pre = time_cur
            self.no_gear_time = 0 # 重置 准备下一次D档位触发
            if self.debug1:
              print("time_cur: ", time_cur, "trigger_gear_D_time: ", self.trigger_gear_D_time_pre)

        # D档下，上拨R, 取消pilot
        if gear_pos == 2 and self.lccButton:
          self.lccButton = False
          if self.debug1:
            print("gear_pos: ", gear_pos, "lccEnbaled: 0")


    # ACC状态 沿用原车的状态机
    if not self.enable_op_acc_state_manage:
      # 如果原车ADAS开启，则用原状态机
      ## ADCS8_ACCState 7 "Passive" 6 "Fault" 5 "Standstill Wait" 4 "Standstill Active" 3 "Override" 2 "Active" 1 "Standby" 0 "Off" ;
      # ADCS8_ACCState是ACC的状态，简单的话就是控车且自车运动的时候发2（Active），
      # 控车且跟前车刹停的时候也就是自车车速位0的时候发4（statndstillActive），没有控车的时候发1就可以（Standby）
      # Passive表示现在有抑制条件，拨杆功能不能激活。功能只能在standby 情况下拨杆激活
      # Standstill wait 指ACC跟停1分钟后，不能自动跟起，可以通过踩油门或者按键跟起。对应有个stand active 是跟停1分钟以内，可以自动跟前车跟起
      acc_state_can =cam_cp.vl["ADCS_Fr08_193"]["ADCS8_ACCState"]
      self.acc_available = acc_state_can in (1, 2, 3, 4, 5)
      self.acc_enabled = acc_state_can in (2, 4, 5)

      ret.cruiseState.speed = cam_cp.vl["ADCS_Fr08_193"]["ADCS8_longitudCtrlSetSpeed"] * CV.KPH_TO_MS
      self.cruise_speed = ret.cruiseState.speed

      tt_set = int(cam_cp.vl["ADCS_Fr08_193"]["ADCS8_longitudCtrlSetDistance"])
      self.acc_distance_index_set = max(0, tt_set - 1)  # 转化为  0-2

    else:
      # ACC状态机本地控制  # 报文BDCS_Fr16_114，信号CS1_GearPositionReqSt---拨杆信号，拨一次ACC激活
      if self.gear_shifter_raw == 4 and gear_pos == 0x5:
        # 如果没有初始化，那就设置当前速度为ACC速度
        # if self.cruise_speed == 0:
        if ret.vEgoRaw < ret.vEgoRaw*CV.KPH_TO_MS:
          self.cruise_speed = 30*CV.KPH_TO_MS
        else:
          self.cruise_speed = ret.vEgoRaw

        # # 速度差不能大于50km/h
        # if abs(ret.vEgoRaw - self.cruise_speed) > 50*CV.KPH_TO_MS:
        #   self.cruise_speed = ret.vEgoRaw
        self.acc_available = True
        self.acc_enabled = True
        # self.lccButton = True
        if self.debug1:
          print("acc enable, cruise_speed: ", self.cruise_speed)
        self.acc_enable_cool_counter = 0

      # ACC 速度设置
      self.curise_speed_set_cool_counter += 1
      if self.curise_speed_set_cool_counter >= 10:
        acc_speed_add = pt_cp.vl["GW_Fr01_15B"]["SWSM_A_CruiseSpeed_Add"]
        if  acc_speed_add == 1:
          self.cruise_speed += 1*CV.KPH_TO_MS
          self.curise_speed_set_cool_counter = 0
          if self.debug:
            print("acc_speed_add 1 kph ")
        elif acc_speed_add == 2:
          self.cruise_speed += 5*CV.KPH_TO_MS
          self.curise_speed_set_cool_counter = 0
          if self.debug:
            print("acc_speed_add 5 kph ")

        acc_speed_minus = pt_cp.vl["GW_Fr01_15B"]["SWSM_A_CruiseSpeed_Minus"]
        if  acc_speed_minus == 1:
          self.cruise_speed -= 1*CV.KPH_TO_MS
          self.curise_speed_set_cool_counter = 0
          if self.debug:
            print("acc_speed_minus 1 kph ")
        elif acc_speed_minus == 2:
          self.cruise_speed -= 5*CV.KPH_TO_MS
          self.curise_speed_set_cool_counter = 0
          if self.debug:
            print("acc_speed_minus 5 kph ")

      # ACC 时距
      # 1，1.25，1.5，1.75，2
      acc_distance_index_set_cur = self.acc_distance_index_set
      acc_distance_add = pt_cp.vl["GW_Fr01_15B"]["SWSM_A_CruiseDistance_Add"]
      if acc_distance_add == 1:
        acc_distance_index_set_cur +=1

      acc_distance_minux = pt_cp.vl["GW_Fr01_15B"]["SWSM_A_CruiseDistance_Minus"]
      if acc_distance_minux == 1:
        acc_distance_index_set_cur -=1

      if acc_distance_index_set_cur < 1:
        acc_distance_index_set_cur = 1
      if acc_distance_index_set_cur > 5:
        acc_distance_index_set_cur = 5

      self.acc_distance_index_set = acc_distance_index_set_cur


    if self.cruise_speed > 64 or self.cruise_speed <= 0:  # 150 kph no current setpoint
      self.cruise_speed = 0

    # ACC 时距
    if self.acc_distance_index_set < 0:
      self.acc_distance_index_set = 0

    if self.acc_distance_index_set <= len(self.acc_distance_list) - 1:
       ret.cruiseState.accDistance = self.acc_distance_list[self.acc_distance_index_set]
    else:
      ret.cruiseState.accDistance = 0

    ret.cruiseState.accDistanceLevel = self.acc_distance_index_set

    if self.steer_takeover:
      self.lccButton = False
      if self.debug1 and self.lccButton != self.lccButton_pre:
        print("steer_takeover: ", self.steer_takeover, "lccButton: ", self.lccButton)

    if ret.brakePressed:
      self.lccButton = False
      self.acc_available = False
      self.acc_enabled = False
      if self.debug1 and self.acc_enabled!= self.acc_enabled_pre:
        print("brakePressed: ", ret.brakePressed)

    ## blindspot sensors   -???-
    # if self.CP.enableBsm:
    # VAL_ 403 ADCS8_LCALeftWarnSt 3 "Reserved" 2 "LV2 Warning(LCA)" 1 "LV1 Warning(BSD)" 0 "No warning";
    # VAL_ 403 ADCS8_LCARightWarnSt 3 "Reserved" 2 "LV2 Warning(LCA)" 1 "LV1 Warning(BSD)" 0 "No warning";
    ret.leftBlindspot = cam_cp.vl["ADCS_Fr08_193"]["ADCS8_LCALeftWarnSt"] == 1 or cam_cp.vl["ADCS_Fr08_193"]["ADCS8_LCALeftWarnSt"] == 2
    ret.rightBlindspot = cam_cp.vl["ADCS_Fr08_193"]["ADCS8_LCARightWarnSt"] == 1 or cam_cp.vl["ADCS_Fr08_193"]["ADCS8_LCARightWarnSt"] == 2

    # # button逻辑还有问题
    # self.prev_cruise_buttons = self.cruise_buttons
    # if self.acc_enabled:
    #   self.cruise_buttons = CruiseButtons.SET
    #   # print("cruise_buttons SET")
    # else:
    #   self.cruise_buttons = CruiseButtons.CANCEL

    # -----------------------------------------------
    # 传递给控制器的ACC状态
    ret.cruiseState.available = self.acc_available
    ret.cruiseState.enabled = self.acc_enabled
    ret.cruiseState.speed = self.cruise_speed
    ret.cruiseState.lccButton = self.lccButton

    self.lccButton_pre = self.lccButton
    self.acc_enabled_pre = self.acc_enabled

    ## safety
    ret.stockFcw = False
    ret.stockAeb = False

    return ret

  @staticmethod
  def get_can_parser(CP):
    if CP.carFingerprint in NETA_CARS:
      # print("get_can_parser_neta_s_new !!!!")
      return CarState.get_can_parser_neta_s(CP)

    else:
      return CarState.get_can_parser_neta_s(CP)

  # 2023.11.22 -YJ-
  def get_can_parser_neta_s(CP):
    messages = [
      ## sig_address, frequency
      ("EPS_Fr01_0B1", 100),    # From J104 ABS/ESP controller
      ("EPS_Fr02_0B2", 100),    # From J104 ABS/ESP controller
      ("ACU_Fr02_0C4", 100),    # From J104 ABS/ESP controller
      ("IDB_Fr01_0E5", 100),
      ("IDB_Fr03_0C5", 100),    # From J104 ABS/ESP controller
      ("IDB_Fr04_0C7", 100),    # From J104 ABS/ESP controller
      ("IDB_Fr05_129", 50),    # From J104 ABS/ESP controller
      ("IDB_Fr06_12B", 50),
      ("IDB_Fr07_12D", 100),
      ("ACU_Fr01_102", 50),    # From J104 ABS/ESP controller
      ("VCU_Fr05_0E3", 50),     # From J623 Engine control module

      ("BDCS_Fr01_110", 50),

      ("DDCU_Fr01_1A2", 20),     #
      # ("PDCU_Fr01_1A3", 20),     #

      # HMI交互
      ("BDCS_Fr16_114", 50),  # adas使能信号 ARS410车型
      ("GW_Fr01_15B", 20), # Resume的信号也是ACC设定速度加的复用信号
      # ("CDCS_Fr11_1F5", 20), # HMI(事件型)  信号CDCS11_ACCOnOffSet---大屏上的PILOT巡航开关（包括了ACC,LCC）

      # #控制报文
      # ("ADCS_F12_136", 50),  # not necessary  no
      # ("ADCS_Fr02_08E", 100),
      # ("ADCS_Fr08_193", 20),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).ECAN)


  @staticmethod
  def get_cam_can_parser(CP):
    messages = []
    if CP.carFingerprint in NETA_CARS:
      # TODO: 2023.08.14
      return CarState.get_cam_can_parser_neta_s(CP)
    else:
       return CarState.get_cam_can_parser_neta_s(CP)


  def get_cam_can_parser_neta_s(CP):
    messages = []
    messages += [
      # sig_address, frequency
      ("ADCS_Fr08_193", 20),
      ("ADCS_Fr02_08E", 100),
      ("ADCS_F12_136", 0),  # not necessary  no
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).CAM)
