# reference: http://sunshine2k.de/articles/coding/crc/understanding_crc.html#ch3
crc8_lut_8h2f = []

def init_lut_crc8_8h2f():
  poly = 0x2F

  for i in range(256):
    crc = i
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
    crc8_lut_8h2f.append(crc)

def get_crc8_8h2f(dat):
  crc = 0xFF    # initial value for crc8_8h2f
  for i in range(len(dat)):
    crc ^= dat[i]
    crc = crc8_lut_8h2f[crc]

  return crc ^ 0xFF

#  SG_ ADCS2_MsgCounter0 : 51|4@0+ (1,0) [0|15] "NoUnit"  HNS2,HNS1,EPS1,EPS2
def get_checksum_data( data):
  dataID = [158,161,150,33,238,151,203,19,139,104,55,2,184,16,105,67]
  checksumData = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  counter = data[6]&0x0f
  Len=0
  checksumData[Len] = counter
  Len = Len + 1
  for i in range(len(data)):
    if i != 7:
      checksumData[Len]= data[i]
      Len = Len + 1
  checksumData[Len] = dataID[counter]
  return checksumData

def cal_crc8(dat):
  crc = 0xFF
  poly = 0x2F
  for i in range(len(dat)):
    crc ^= dat[i]
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  
  return crc ^ 0xFF

# ADCS_Fr02_08E
# ADCS2_longitudCtrlType 3 "NNP Mode" 2 "NPilot Mode" 1 "ACC Mode" 0 "None";
# ADCS2_ADAS_EPSLateralCtrlType 7 "Reserved" 5 "Reserved" 4 "ELK Mode" 3 "NNP Mode" 2 "Pilot Mode" 1 "LDP Mode" 0 "None";
def create_steer_and_accel_command_Fr02_08E(packer, CAN, raw_msg, 
                                            lcc_req=False, steer_angle_des=0, 
                                            acc_req=False, accel_des=0, 
                                            acc_CtrlDecToStopReq=0, acc_CtrlDriveOff=0,
                                            counter=0, bypass=False):
  values = raw_msg
  if not bypass:
    values.update({
      'ADCS2_LongitudCtrlAccelCtrlReq' : 1 if acc_req else 0,
      "ADCS2_LongitudCtrlTargetAccel" : accel_des,
      "ADCS2_LongitudCtrlDecToStopReq" : acc_CtrlDecToStopReq,
      "ADCS2_LongitudCtrlDriveOff" : acc_CtrlDriveOff,
      
      "ADCS2_ADAS_EPSAngleReqSt" : 1 if lcc_req else 0,
      "ADCS2_ADAS_EPSAngleReq" : steer_angle_des,
      "ADCS2_ADAS_EPSLateralCtrlType" : 2 if lcc_req else 0,
      # counter 可能是不连续的，所以不用；直接用原车的
      # "ADCS2_MsgCounter0" : counter,
    })

    dat = packer.make_can_msg("ADCS_Fr02_08E", CAN.ECAN, values)[1]
    checksum_data = get_checksum_data(dat)
    # crc = get_crc8_8h2f(dat)
    crc = cal_crc8(checksum_data)
    values.update({
      "ADCS2_Checksum0" : crc,
      })
    
  # print("raw_crc: ", raw_msg["ADCS2_Checksum0"], "crc: ", crc)
  return packer.make_can_msg("ADCS_Fr02_08E", CAN.ECAN, values)

# ADCS_Fr08_193
# ADCS8_longitudCtrlSetDistance 7 "Reserved2" 6 "Reserved1" 5 "Level#5" 4 "Level#4" 3 "Level#3" 2 "Level#2" 1 "Level#1" 0 "No time gap set";
# ADCS8_NPilot_SysState 5 "passive" 4 "Fault" 3 "suspend" 2 "Active" 1 "Standby" 0 "OFF"
# 脱手告警
# ADCS8_lateralCtrHandsEyesOffWarning 3 "eyes off Warning" 2 "Automatic-Off Warning(Display)" 1 "Hands-off Warning(Display)" 0 "No Warning";
# 方向盘转动状态
# ADCS8_HandShakeSts 4 "Reserved" 3 "Invalid" 2 "Hands-on" 1 "Hands-off" 0 "No Detection";
def create_control_ADCS_Fr08_193(packer, CAN, raw_msg, lead, set_speed, acc_distance_index, NPilot_SysState = 0, bypass=False):
  values = raw_msg
  if not bypass:
    values.update({
      'ADCS8_longitudCtrlTargetValidity' : 1 if lead else 0,
      'ADCS8_longitudCtrlSetDistance' : acc_distance_index+1,
      'ADCS8_longitudCtrlSetSpeed' : set_speed,
      # "ADCS8_ACCState": acc_state,
      # "ADCS8_longitudCtrlTakeOverReq": 0,
      "ADCS8_NPilot_SysState": NPilot_SysState,
      "ADCS8_lateralCtrHandsEyesOffWarning": 0,   # 0: No Warning
    })

  return packer.make_can_msg("ADCS_Fr08_193", CAN.ECAN, values)

# ADCS_F12_136
# ADCS12_NPilotSysInfo 31 "Reserved" 30 "Reserved" 29 "Reserved" 28 "Reserved" 27 "Reserved" 26 "Reserved" 25 "Reserved" 24 "Reserved" 23 "Reserved" 22 "Reserved" 21 "reason 5" 20 "reason 4" 19 "reason 3" 18 "reason 2" 17 "reason 1" 
# 16 "开关关闭" 15 "不在ODD内" 14 "GPS信号差" 13 "无法开启" 12 "Racing mode" 11 "EPB拉起" 10 "前舱盖" 9 "后备箱" 
# 8 "door open" 7 "reservedseatbelt untied" 6 "Disable to engage NPilot Cuz lane" 5 "Disable to engage NPilot Cuz braking" 
# 4 "Disable to engage NPilot Cuz speed" 3 "Disable to engage NPilot cuz fault" 2 "NPilot Off" 1 "NPilot On" 0 "Disable to engage NPilot";
# ADCS12_PilotAudioPlay 3 "Pilot cancel" 2 "Pilot resumed" 1 "Pilot Override" 0 "None";
def create_ADCS_F12_136(packer, CAN, raw_msg, NPilotSysInfo=31, bypass=False):
  values = raw_msg

  if not bypass:
    values.update({
      # 'ADCS8_longitudCtrlTargetValidity' : 1 if lead else 0,
      # 'ADCS8_longitudCtrlSetDistance' : acc_distance_index+1,
      # 'ADCS8_longitudCtrlSetSpeed' : set_speed,
      # "ADCS8_ACCState": acc_state,
      # "ADCS8_longitudCtrlTakeOverReq": 0,
      "ADCS12_NPilotSysInfo": NPilotSysInfo,
      "ADCS12_PilotAudioPlay": 0,
    })

  return packer.make_can_msg("ADCS_F12_136", CAN.ECAN, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, counter, cancel=False, resume=False):
  values = gra_stock_values.copy()

  values.update({
    "COUNTER": counter,
    "GRA_Abbrechen": cancel,
    "GRA_Recall": resume,
  })

  return packer.make_can_msg("GRA_Neu", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if long_active:
    acc_control = 1
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control

# run main
init_lut_crc8_8h2f()