#!/usr/bin/env python3
from opendbc.can.parser import CANParser
from cereal import car
from openpilot.selfdrive.car.byd.values import DBC, RADAR, CanBus
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase
import numpy as np

## ARS410
# left is +
ARS410_RADAR_A_MSGS = list(range(0x20, 0x47))
ARS410_RADAR_B_MSGS = list(range(0x50, 0x77))

# ## ARS513
# # 0x200 是header信息； 0x21F是最后一条报文
# ARS513_RADAR_MSGS = list(range(0x200, 0x214))
# ARS513_RADAR_MSGS.append(0x21f)
# ARS513_RADAR_MSGS_OBJ = list(range(0x201, 0x214))
# ARS513_RADAR_START_ADDR = 0x200
# ARS513_RADAR_END_ADDR = 0x21F
# # ARS513_RADAR_MSG_COUNT = 30 # 报文数量(一条里面2个目标)

# # chuhang513
# # 目标报文是0x201到0x21e
# ARS513CH_RADAR_MSGS = list(range(0x200, 0x21e))
# ARS513CH_RADAR_MSGS.append(0x21f)
# ARS513CH_RADAR_MSGS_OBJ = list(range(0x201, 0x21e))
# ARS513CH_RADAR_START_ADDR = 0x200
# ARS513CH_RADAR_END_ADDR = 0x21F

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    # print("openpilot_dev")
    super().__init__(CP)
    self.CP = CP
    self.CAN=CanBus(CP)
    self.updated_messages = set()
    self.track_id = 0
    self.radar_ts = CP.radarTimeStep
    self.radar = DBC[CP.carFingerprint]['radar']

    print("byd radar type: ", self.radar, " !!!")

    self.install_lat_offset = 0 #相对车头中心横向安装位置 # left is +
    self.install_lon_offset = 0 #相对车头中心纵向向安装位置 # 前 is +
    self.install_yaw = 0/57.3
    self.obj_nums = 0
    self.yawrate = 0
    self.speed = 0
    self.acc = 0
    self.curvature = 0

    if CP.radarUnavailable or  self.radar is None:
      self.rcp = None
    elif self.radar == RADAR.ARS410:
      self.trigger_msg = ARS410_RADAR_B_MSGS[-1]
      self.valid_cnt = {key: 0 for key in ARS410_RADAR_A_MSGS}

      self.install_lon_offset = -3.5
      self.rcp = self.create_radar_can_parser_ars410(CP.carFingerprint)
    elif self.radar == RADAR.ARS513:
      # # 0x200 是header信息； 0x21F是最后一条报文,报文数量(一条里面2个目标)
      self.RADAR_START_ADDR = 0x200
      self.trigger_msg = 0x21F
      self.RADAR_MSGS_OBJ = list(range(0x201, 0x214))
      self.RADAR_MSGS = list(range(0x200, 0x214))
      self.RADAR_MSGS.append(0x21f)
      self.valid_cnt = {key: 0 for key in self.RADAR_MSGS}

      self.install_lon_offset = 0
      self.rcp = self.create_radar_can_parser_ars513(CP.carFingerprint)
    elif self.radar == RADAR.ChuHangRadar:
      self.RADAR_START_ADDR = 0x200
      self.trigger_msg = 0x21F
      self.RADAR_MSGS_OBJ = list(range(0x201, 0x21e))
      self.RADAR_MSGS = list(range(0x200, 0x21e))
      self.RADAR_MSGS.append(0x21f)
      self.valid_cnt = {key: 0 for key in self.RADAR_MSGS}

      self.install_lat_offset = -0.43 #
      self.install_lon_offset = 0

      self.rcp = self.create_radar_can_parser_ars513(CP.carFingerprint)
    else:
      raise ValueError(f"Unsupported radar: {self.radar}")

    # No radar dbc for cars without DSU which are not TSS 2.0
    # TODO: make a adas dbc file for dsu-less models
    # self.no_radar = CP.carFingerprint in NO_DSU_CAR and CP.carFingerprint not in TSS2_CAR

  def create_radar_can_parser_ars410(self, car_fingerprint):
    if DBC[car_fingerprint]['radar'] is None:
      return None
    msg_a_n = len(ARS410_RADAR_A_MSGS)
    msg_b_n = len(ARS410_RADAR_B_MSGS)

    # 20Hz (0.05s)
    messages = list(zip(ARS410_RADAR_A_MSGS + ARS410_RADAR_B_MSGS, [20] * (msg_a_n + msg_b_n)))
    return CANParser(DBC[car_fingerprint]['radar'], messages, self.CAN.ACAN)

  def create_radar_can_parser_ars513(self, car_fingerprint):
    if DBC[car_fingerprint]['radar'] is None:
      return None

    msg_a_n = len(self.RADAR_MSGS)
    # 20Hz (0.05s)
    messages = list(zip(self.RADAR_MSGS, [20] * (msg_a_n)))
    return CANParser(DBC[car_fingerprint]['radar'], messages,  self.CAN.ACAN) #NETAS_CanBus.ACAN)
  
  def update(self, can_strings):
    # if self.rcp is None or self.CP.radarUnavailable:
    if self.rcp is None :
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    if self.radar == RADAR.ARS410:
      rr =  self._update_ars410()
    elif self.radar == RADAR.ARS513:
      rr = self._update_ars513()
    elif self.radar == RADAR.ChuHangRadar:
      rr = self._update_ars513()
    else:
      rr = None

    return rr

  def _update_ars410(self):
    ret = car.RadarData.new_message()
    errors = []
    msg_A_B_offset = 0x30
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for can_id in sorted(self.updated_messages):
      if can_id in ARS410_RADAR_A_MSGS:
        cpt = self.rcp.vl[can_id]
        cpt_B = self.rcp.vl[can_id+msg_A_B_offset]
        if cpt is None or cpt_B is None:
          continue

        # if cpt['FRS_Obj_XPos'] > 0.00001 or not cpt_B['FRS_Obj_UpdateFlag']:
        #   self.valid_cnt[can_id] = 0    # reset counter
        # if cpt_B['FRS_Obj_ValidFlag'] and cpt['FRS_Obj_XPos'] < 255:
        #   self.valid_cnt[can_id] += 1
        # else:
        #   self.valid_cnt[can_id] = max(self.valid_cnt[can_id] - 1, 0)
        obstacle_prob = cpt_B['FRS_Obj_ObstacleProb']
        # if cpt_B['FRS_Obj_ValidFlag'] and (obstacle_prob > 20 and cpt['FRS_Obj_XPos'] < 255 and self.valid_cnt[can_id] > 0) and (cpt_B['FRS_ObjMotionPattern'] != 1):
        if cpt['FRS_Obj_XPos'] > 0.00001 and (cpt_B['FRS_ObjMotionPattern'] != 1):
          if can_id not in self.pts or not cpt_B['FRS_Obj_UpdateFlag']:
            self.pts[can_id] = car.RadarData.RadarPoint.new_message()
            self.pts[can_id].trackId = cpt_B['Obj_ID']
            # self.track_id += 1

          # fix install yaw and install_lat_offset
          x_t = cpt['FRS_Obj_XPos'] + self.install_lon_offset
          offset_t = x_t * np.cos(self.install_yaw)
          y_new = cpt['FRS_Obj_YPos'] + self.install_lat_offset
          self.pts[can_id].dRel = x_t # from front of car
          self.pts[can_id].yRel = y_new # ars410 install offset
          self.pts[can_id].vRel = cpt['FRS_Obj_XVelRel']
          self.pts[can_id].aRel = cpt_B['FRS_Obj_XAccRel']
          self.pts[can_id].yvRel = cpt['FRS_Obj_YVelRel']
          self.pts[can_id].measured = bool(cpt_B['FRS_Obj_ValidFlag'])
        else:
          if can_id in self.pts:
            del self.pts[can_id]

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
  
  def decompose_velocity_acceleration(self, v, a, omega, curvature):
    # 横向速度
    v_horizontal = v * np.cos(omega)
    # 纵向速度
    v_longitudinal = v * np.sin(omega)
    # 横向加速度
    a_horizontal = a * np.cos(omega) - v * omega * np.sin(omega)
    # 纵向加速度
    a_longitudinal = a * np.sin(omega) + v * omega * np.cos(omega)
    return v_longitudinal, v_horizontal, a_longitudinal, a_horizontal
  
  def _update_ars513(self):
    ret = car.RadarData.new_message()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for can_id in sorted(self.updated_messages):
      # header信息
      if can_id == 0x200:
        cpt = self.rcp.vl[can_id]
        self.obj_nums = cpt[f'ARS_OD_NumOfObjects']
        self.yawrate = cpt[f'ARS_OD_EgoYawRate']
        self.speed = cpt[f'ARS_OD_EgoVelocity']
        self.acc = cpt[f'ARS_OD_EgoAcceleration']
        self.curvature = cpt[f'ARS_OD_EgoCurvature']
        vx_ego, vy_ego, ax_ego, ay_ego = self.decompose_velocity_acceleration(self.speed, self.acc, self.yawrate, self.curvature)

      if can_id in self.RADAR_MSGS_OBJ:
        cpt = self.rcp.vl[can_id]
        # 每条报文里面有2个目标  0x201开始为目标报文
        canid_msg_index = can_id-1 - self.RADAR_START_ADDR
        for index in [canid_msg_index*2, canid_msg_index*2+1]:
          id = index #cpt[f"ARS_OD_ID_{index:02d}"]
          x = cpt[f"ARS_OD_DistX_Obj_{index:02d}"]
          y = cpt[f"ARS_OD_DistY_Obj_{index:02d}"]
          vx_abs = cpt[f"ARS_OD_VabsX_Obj_{index:02d}"]
          vy_abs = cpt[f"ARS_OD_VabsY_Obj_{index:02d}"]
          ax_abs = cpt[f"ARS_OD_AabsX_Obj_{index:02d}"]
          ay_abs = cpt[f"ARS_OD_AabsY_Obj_{index:02d}"]
          length = cpt[f"ARS_OD_Length_Obj_{index:02d}"]
          width = cpt[f"ARS_OD_Width_Obj_{index:02d}"]
          obstacle_prob = cpt[f"ARS_OD_ObstacleProb_Obj_{index:02d}"]
          # 0 "MOVING" 1 "STATIONARY" 2 "ONCOMING" 3 "CROSSING_LEFT2RIGHT" 4 "CROSSING_RIGHT2LEFT" 5 "UNKNOWN" 6 "STOPPED" 7 "RESERVED" ;
          motion_status = cpt[f"ARS_OD_DynProp_Obj_{index:02d}"]
          # 0 "INVALID" 1 "NEW" 2 "MEASURED" 3 "PREDICTED" 
          track_status = cpt[f"ARS_OD_MaintenanceState_Obj_{index:02d}"]

          # 释放的雷达目标纵向静止距离跟本车速度相关，速度小于60 kph时，释放60m内的目标，速度大于60 kph时，释放80m内的目标
          x_filter = 60 if self.speed*3.6 < 60 else 80
          is_STL = (abs(x) < x_filter and abs(y)<1.75)

          # if cpt_B['FRS_Obj_ValidFlag'] and (obstacle_prob > 20 and cpt['FRS_Obj_XPos'] < 255 and self.valid_cnt[can_id] > 0) and (cpt_B['FRS_ObjMotionPattern'] != 1):
          if x > -100 and (motion_status != 1 and motion_status != 5) or is_STL:
            self.pts[index] = car.RadarData.RadarPoint.new_message()
            self.pts[index].trackId =id

            # fix install yaw and install_lat_offset
            x_t = x + self.install_lon_offset
            offset_t = x_t * np.cos(self.install_yaw)
            y_new = float(y + self.install_lat_offset)
            self.pts[index].dRel = x_t # from front of car
            self.pts[index].yRel = float(y_new)
            self.pts[index].vRel = vx_abs - self.speed
            self.pts[index].aRel = ax_abs - self.acc
            self.pts[index].yvRel = vy_abs
            self.pts[index].measured = (track_status == 1 or track_status == 2)
          else:
            if index in self.pts:
              del self.pts[index]
    # print("obj nums: ", self.obj_nums)
    # print("self.pts: ", len(self.pts))
    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret

