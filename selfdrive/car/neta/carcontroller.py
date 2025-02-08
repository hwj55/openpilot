from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.neta import neta_s_canfd
from openpilot.selfdrive.car.neta.values import CanBus, CarControllerParams, NETA_CARS

from openpilot.selfdrive.controls.lib.latcontrol import MIN_LATERAL_CONTROL_SPEED


VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CAN = CanBus(CP)
    self.CCP = CarControllerParams(CP)

    # 只转发原车控制信号
    self.bypass_raw_msg = False
    # -YJ- 0: disable control can sends, 1: enable control can sends
    self.enable_control_can_sends = True
    self.lat_active_pre = None
    self.lat_active = None
    self.lcc_button_pre = None
    self.lcc_button = None

    self.pilot_sys_info_pre = 100  # 无效值
    self.msg_0x136_counter = 0 # 发送计数器

    self.Fr02_08E_counter=0

	# -YJ-
    if CP.carFingerprint in NETA_CARS:
      self.CCS = neta_s_canfd
    else:
      self.CCS = neta_s_canfd

    self.packer_pt = CANPacker(dbc_name)
    self.apply_steer_last = 0
    self.apply_angle_last = 0
    self.last_angle = 0
    self.gra_acc_counter_last = None
    self.frame = 0
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0

  def update(self, CC, CS, now_nanos):
    self.frame += 1
    actuators = CC.actuators
    can_sends = []
    if self.CP.carFingerprint in NETA_CARS:
      if self.enable_control_can_sends:
        return self.update_control_neta_s(CC, CS, now_nanos)
      else:
        return actuators, can_sends, False
    else:
      return self.update_control_neta_s(CC, CS, now_nanos)

  ###### neta s
  def update_control_neta_s(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Controls ******************************************** #
    # if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
    if self.CP.openpilotLongitudinalControl:
      # EPS状态
      EPS_available = CS.out.cruiseState.epsAvailable
      EPS_active = CS.out.cruiseState.epsActive
      # IDB状态
      IDB_EPB_active = CS.out.cruiseState.idbEPBActive

      ##### *** acc and lcc ***
      # acc
      acc_control_state = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      accel_cmd = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX)

      # EPB 激活的话 退出acc
      # # 如果用户踩油门，则不响应ACC请求
      acc_req = False
      acc_CtrlDecToStopReq = 0
      acc_CtrlDriveOff = 0
      if CC.longActive and not CS.out.cruiseState.idbEPBActive and not CS.out.gasPressed:
        acc_req = True
        # 自车车速<1kph && 请求减速度<于0,  acc_CtrlDecToStopReq = 1
        # ACC已经跟前车刹停的时候也要置1(静止的时候，如果前车没动，是不是应该acc=0？)
        if CS.out.vEgo <= 0.3 or CS.out.standstill:
          if accel_cmd < 0:
            acc_CtrlDecToStopReq = 1
            acc_CtrlDriveOff = 0
          elif accel_cmd > 0:
            if CS.out.standstill:
              # IDB5_Fr05_129报文里面有个VehicleStandstill的信号 = 1&& 请求减速度>0,  acc_CtrlDriveOff  = 1
              acc_CtrlDriveOff = 1
              acc_CtrlDecToStopReq = 0

      # lcc  低速下模型输出的期望角度误差太大，不适合用于低速控制
      lcc_req = self.lat_active
      standstill_for_lat_control = CS.out.vEgo <= MIN_LATERAL_CONTROL_SPEED or CS.out.standstill
      if self.lat_active  and not standstill_for_lat_control:
        # EPS uses the torque sensor angle to control with, offset to compensate
        # steeringAngleOffsetDeg 还没有估计
        apply_angle_cmd = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg
        # Angular rate limit based on speed
        apply_angle_cmd = apply_std_steer_angle_limits(apply_angle_cmd, self.last_angle, CS.out.vEgo, self.CCP)
        apply_angle_cmd = clip(apply_angle_cmd, CS.out.steeringAngleDeg - 20, CS.out.steeringAngleDeg + 20)
        self.last_angle = apply_angle_cmd
      else:
        apply_angle_cmd = CS.out.steeringAngleDeg

      self.apply_angle_last = apply_angle_cmd
      self.Fr02_08E_counter += 1
      msg_2 = self.CCS.create_steer_and_accel_command_Fr02_08E(self.packer_pt, self.CAN, CS.Fr02_08E_raw_msg,
                                                lcc_req=lcc_req, steer_angle_des=apply_angle_cmd,
                                                acc_req=acc_req, accel_des=accel_cmd,
                                                acc_CtrlDecToStopReq=acc_CtrlDecToStopReq, acc_CtrlDriveOff=acc_CtrlDriveOff,
                                                counter=self.Fr02_08E_counter, bypass=self.bypass_raw_msg
                                                )
      can_sends.append(msg_2)

      #######  *** state ***
      lead = hud_control.leadVisible
      # HMI显示的速度比CAN上的会大一点
      set_speed = CS.out.cruiseState.speed*CV.MS_TO_KPH*1.03
      set_time_distance = 1.0
      # ADCS8_ACCState是ACC的状态，简单的话就是控车且自车运动的时候发2（Active），
      # 控车且跟前车刹停的时候也就是自车车速位0的时候发4（statndstillActive），没有控车的时候发1就可以（Standby）
      self.lat_active = CC.latActive
      if self.lat_active_pre is None:
        self.lat_active_pre = self.lat_active
      self.lcc_button = CS.out.cruiseState.lccButton
      if self.lcc_button_pre is None:
        self.lcc_button_pre = self.lcc_button


      # 关键状态量
      # ADCS8_NPilot_SysState 5 "passive" 4 "Fault" 3 "suspend" 2 "Active" 1 "Standby" 0 "OFF"
      if self.lat_active:
        if EPS_active:
          # 功能已经正常激活
          pilot_sys_state = 2
        else:
          # standby  灰色图标
          pilot_sys_state = 1
      else:
        if EPS_available:
          # standby  灰色图标
          pilot_sys_state = 1
        else:
          # OFF
          pilot_sys_state = 0

      acc_distance_index = CS.out.cruiseState.accDistance
      msg_t = self.CCS.create_control_ADCS_Fr08_193(self.packer_pt, self.CAN, CS.Fr08_193_raw_msg,
                                                    lead, set_speed, acc_distance_index,
                                                    NPilot_SysState=pilot_sys_state, bypass=self.bypass_raw_msg)
      can_sends.append(msg_t)

      ##### *** HMI交互 事件型 变化触发，触发一次***
      # ADCS12_NPilotSysInfo 31 "Reserved" 30 "Reserved" 29 "Reserved" 28 "Reserved" 27 "Reserved" 26 "Reserved" 25 "Reserved" 24 "Reserved" 23 "Reserved" 22 "Reserved" 21 "reason 5" 20 "reason 4" 19 "reason 3" 18 "reason 2" 17 "reason 1"
      # 16 "开关关闭" 15 "不在ODD内" 14 "GPS信号差" 13 "无法开启" 12 "Racing mode" 11 "EPB拉起" 10 "前舱盖" 9 "后备箱"
      # 8 "door open" 7 "reservedseatbelt untied" 6 "Disable to engage NPilot Cuz lane" 5 "Disable to engage NPilot Cuz braking"
      # 4 "Disable to engage NPilot Cuz speed" 3 "Disable to engage NPilot cuz fault" 2 "NPilot Off" 1 "NPilot On" 0 "Disable to engage NPilot";
      if self.lcc_button and not self.lcc_button_pre:
        # 激活
        pilot_sys_info = 1
      elif not self.lcc_button and self.lcc_button_pre:
        # 退出
        pilot_sys_info = 2
      else:
        pilot_sys_info = 31

      # if pilot_sys_info != self.pilot_sys_info_pre:
      # 定周期发送 loop是10ms, 目前100ms发送一次
      self.msg_0x136_counter += 1
      if self.msg_0x136_counter >= 10:
        msg = self.CCS.create_ADCS_F12_136(self.packer_pt, self.CAN, CS.F12_136_raw_msg, NPilotSysInfo=pilot_sys_info, bypass=self.bypass_raw_msg)
        can_sends.append(msg)
        self.msg_0x136_counter = 0

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.latActive_pre = self.lat_active
    self.lcc_button_pre = CS.out.cruiseState.lccButton
    self.pilot_sys_info_pre = pilot_sys_info

    # self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    return new_actuators, can_sends
