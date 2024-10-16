# carstate.py
import copy
from cereal import car
import cereal.messaging as messaging
from cereal.messaging import SubMaster
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.byd.values import DBC, CAR, HUD_MULTIPLIER, CanBus, GearShifter, CarControllerParams
from typing import Dict, Optional

import time
import numpy as np

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

        can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
        self.is_cruise_latch = False

        # stock msg
        self.ID0x316_lcc_raw_msg = {}
        self.ID0x32e_acc_raw_msg = {}
        self.MPC_0x34B_lane_raw_msg = {}

        self.always_enable_lcc = False

        self.lccButton_pre = False
        self.lccButton_cur = False
        self.enable_op_lcc_state_manage = True # 是否使用OP本地lcc状态机控制   默认用op
        self.eps_error_time_pre = 0  # EPS报错后不立马恢复
        self.gear_D_time_pre = 0 # 上一次拨杆D的时间
        self.no_gear_time = 0 # 没有拨杆时间
        self.trigger_gear_D_time_pre = 0 # 拨杆D的时间
        self.gear_shifter_raw = 0 # 当前档位信号

        # self.set_acc_available = False  # 临时设置ACC可用
        self.steer_torque_lpf = FirstOrderLowPassFilter(12, 100)
        self.steer_takeover = False # 驾驶员接管

        self.debug = False

        self.sm = messaging.SubMaster(['modelV2'])
        self.sm_counter = 0
        self.left_coeff = []
        self.right_coeff = []
        self.lane_central_coeff = []

    
    def parse_gear_shifter_byd_s(self, gear: Optional[str]) -> car.CarState.GearShifter:
        if gear is None:
          return GearShifter.unknown

        d: Dict[str, car.CarState.GearShifter] = {
          'PGear': GearShifter.park, 'PARK': GearShifter.park,
          'RGea': GearShifter.reverse, 'REVERSE': GearShifter.reverse,
          'NGea': GearShifter.neutral, 'NEUTRAL': GearShifter.neutral,
          # 'E': GearShifter.eco, 'ECO': GearShifter.eco,
          # 'T': GearShifter.manumatic, 'MANUAL': GearShifter.manumatic,
          'Dgea': GearShifter.drive, 'DRIVE': GearShifter.drive,
        #   'S': GearShifter.sport, 'SPORT': GearShifter.sport,
          # 'L': GearShifter.low, 'LOW': GearShifter.low,
          # 'B': GearShifter.brake, 'BRAKE': GearShifter.brake,
        }
        return d.get(gear.upper(), GearShifter.unknown)

    def get_x_index(self, line, max_range=60):
        index = 0
        for x_t in line.x:
            index += 1
            if x_t >= 60:
                break
        return index
    
    #  x: 纵向  y: 横向
    def lane_process(self):
        left_coeff, right_coeff, lane_central_coeff= [], [], []
        model_v2 = self.sm['modelV2']
        lane_lines = model_v2.laneLines
        if len(lane_lines) > 3:
            left_line = lane_lines[1]
            right_line = lane_lines[2]
            # 取
            left_index = self.get_x_index(left_line, max_range=80)
            left_lane_x, left_lane_y = np.array(left_line.x)[0:left_index], np.array(left_line.y)[0:left_index]

            right_index = self.get_x_index(right_line, max_range=80)
            right_lane_x, right_lane_y = np.array(right_line.x)[0:right_index], np.array(right_line.y)[0:right_index]
            # 使用2阶多项式拟合（拟合成二次曲线，适合一般道路曲线）
            left_coeff = np.polyfit(left_lane_x, left_lane_y, 2)
            right_coeff = np.polyfit(right_lane_x, right_lane_y, 2)
            lane_central_coeff = (left_lane_y + right_lane_y) / 2
            
            # print("left_coeff", left_coeff)
            # print("right_coeff", right_coeff)
        
        return left_coeff, right_coeff, lane_central_coeff

    def update(self, cp, cp_cam):
        time_cur = time.monotonic()

        ret = car.CarState.new_message()
        # Messages needed by carcontroller
        self.ID0x316_lcc_raw_msg = copy.copy(cp_cam.vl["MRR_0x316"])
        self.ID0x32e_acc_raw_msg = copy.copy(cp_cam.vl["MRR_0x32E"])
        self.MPC_0x34B_lane_raw_msg = copy.copy(cp_cam.vl["MPC_0x34B"])
        
        # EV irrelevant messages
        ret.brakeHoldActive = False

        # 车速  m/s
        ret.wheelSpeeds = self.get_wheel_speeds(
              cp.vl["IPB_0x1F0"]['Wheel_Speed_FL_1F0_S'],
              cp.vl["IPB_0x1F0"]['Wheel_Speed_FR_1F0_S'],
              cp.vl["IPB_0x1F0"]['Wheel_Speed_RL_1F0_S'],
              cp.vl["IPB_0x1F0"]['Wheel_Speed_RR_1F0_S'], # TODO: why would BR make the value wrong? Wheelspeed sensor prob?
            )
        ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.

        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
        ret.vEgoCluster = ret.vEgo
        ret.standstill = ret.vEgoRaw < 0.1

        
        # 转向灯刹车灯状态
        turn_signal_can = int(cp.vl["CS_0x133"]['Turn_Signal_Switch_S'])
        ret.leftBlinker = turn_signal_can == 2 or turn_signal_can == 3
        ret.rightBlinker = turn_signal_can == 4 or turn_signal_can == 5
        # ret.genericToggle = bool(cp.vl["CS_0x133"]["GENERIC_TOGGLE"])
        ret.doorOpen = any([cp.vl["Left_BCM_0x294"]['Left_Front_Door_Status_294_S']==2,
                             cp.vl["Left_BCM_0x294"]['Right_Front_Door_Status_294_S'] ==2,
                             cp.vl["Left_BCM_0x294"]['Left_Back_Door_Status_294_S']==2,
                             cp.vl["Left_BCM_0x294"]['Right_Back_Door_Status_294_S'] == 2])

        # 方向盘角度，以度为单位。
        ret.steeringAngleDeg = cp.vl["EPS_0x11F"]["Steering_Wheel_Angle_S"]
        # 方向盘转角速率
        ret.steeringRateDeg = cp.vl["EPS_0x11F"]["Steering_Wheel_Rotation_Speed_S"]

        ret.steeringTorqueEps =  cp.vl["EPS_0x318"]["LKA_Control_Delivered_Value_S"] 
        # 驾驶员手力矩
        ret.steeringTorque = cp.vl["EPS_0x318"]["StrngWhlTorq_S"]
        # 方向盘是否被按下，如果绝对扭矩值大于6则被认为是被按下
        ret.steeringPressed = bool(abs(ret.steeringTorque) >  self.CCP.STEER_DRIVER_ALLOWANCE)

        # data 是你的原始数据 cutoff_freq 是你的低通滤波器的截止频率  sampling_freq 是你的数据采样频率
        ret.cruiseState.steerTorqueKf = self.steer_torque_lpf.filter(ret.steeringTorque)
        self.steer_takeover = abs(ret.cruiseState.steerTorqueKf) > self.CCP.STEER_DRIVER_TAKEROVER

        ret.espDisabled = False

        # 0 "EPSExternalSteeringTorqueRequireControlNotReady" 1 "EPSExternalSteeringTorqueRequireControlReady" 2 "EPSExternalSteeringTorqueRequireControlActive" 3 "ExternalRequireorEPSTemporaryFailed" 4 "ExternalRequireorEPSPermanentlyFailed" ;
        eps_state = cp.vl["EPS_0x318"]["EPS_LKA_Control_State_S"]
        ret.cruiseState.epsAvailable = eps_state in [1, 2]
        ret.cruiseState.epsActive = eps_state == 2

        if self.always_enable_lcc:
            ret.cruiseState.lccButton = True
            ret.cruiseState.epsAvailable = True
            ret.cruiseState.epsActive = True

        # 获取原车控制量
        ret.cruiseState.lccCmdRaw = cp_cam.vl["MRR_0x316"]["Output_LKS_torque_request_S"]
        ret.cruiseState.lccCmdRawStatus = int(cp_cam.vl["MRR_0x316"]["TJA_ICA_function_status_S"])
        # ret.cruiseState.accCmdRaw = cp_cam.vl["ADCS_Fr02_08E"]["ADCS2_LongitudCtrlTargetAccel"]
        # ret.cruiseState.accCmdRawStatus = int(cp_cam.vl["ADCS_Fr02_08E"]["ADCS2_LongitudCtrlAccelCtrlReq"])

        # lcc激活型号，正常要响应开关 0-->1 用原车的值
        # TJA TJA_ICA_function_status_S 0 "OFF" 1 "Passive" 2 "Active1" 3 "Active2" 4 "Fault" 5 "StandBy" ;
        tja_state = cp_cam.vl["MRR_0x316"]["TJA_ICA_function_status_S"] 
        lcc_on_raw = tja_state in [2, 3]
        if not self.lccButton_pre and lcc_on_raw:
            self.lccButton_cur = True
            self.lccButton_pre = True
        
        # 退出LCC的条件
        # 1. MPC_Take_Over_Req1_S >=3时，就得退出LCC
        # MPC_Take_Over_Req1_S 0 "Inactive" 1 "WarningLevel1" 2 "WarningLevel2" 3 "WarningLevel3" ;
        # 2. EPS not available时，就得退出LCC
        # 3. 驾驶员手动接管了方向盘，就得退出LCC
        mpc_take_over_req = cp_cam.vl["MRR_0x316"]["MPC_Take_Over_Req1_S"]
        if not ret.cruiseState.epsAvailable or self.steer_takeover:
            if self.debug:
                print("mapc_take_over_req", mpc_take_over_req, "epsAvailable", ret.cruiseState.epsAvailable, "steer_takeover", self.steer_takeover)
            self.lccButton_cur = False
            self.lccButton_pre = False
        
        ret.cruiseState.lccButton = self.lccButton_cur
        
         # 0 "Reserved" 1 "PGear" 2 "RGea" 3 "NGea" 4 "Dgea" ;
        can_gear_raw = cp.vl["VCU_0x342"]['VCU_Goal_Gear_S']
        ret.gearShifter = self.parse_gear_shifter_byd_s(self.CCP.shifter_values.get(can_gear_raw))

        # # Actual_Shift_Gear_Mode_S 0 "Invalid" 1 "P" 2 "R" 3 "N" 4 "D" 5 "MFuel" 6 "Sfuel" 7 "MPositiveFuel" 8 "MNegativeFuel" ;
        # gear_pos = cp.vl["TCU_0x212"]["Actual_Shift_Gear_Mode_S"]

        # 安全带
        ret.seatbeltUnlatched = cp.vl["Left_BCM_0x294"]['Driver_Belt_Status_S'] == 1
        # 油门踏板
        ret.gas = cp.vl["VCU_0x342"]['VCU_Accelerograph_Depth_S']/100.0
        ret.gasPressed = ret.gas > 0.01
        # 刹车踏板
        ret.brake = cp.vl["VCU_0x342"]['VCU_Brake_Depth_S']/100.0
        ret.brakePressed = ret.brake > 0.01
        disengage = ret.doorOpen or ret.seatbeltUnlatched or ret.brakeHoldActive

        # TODO: get the real value
        ret.stockAeb = False
        ret.stockFcw = False
        
        # ACC设置 速度  ?* CV.KPH_TO_MS 换算成米每秒
        ret.cruiseState.speed = cp_cam.vl["MRR_0x32D"]["Set_Speed_S"] * CV.KPH_TO_MS

        set_distance = cp.vl["SWS_0x3B0"]["SWS_Dist_Set_S"]
        set_speed = cp.vl["SWS_0x3B0"]["SWS_Speed_Set_S"]
        # ACC_Mode_S 0 "OFF" 1 "Passivemode" 2 "StandBymode" 3 "ActiveControlmode" 4 "BrakeOnlymode" 5 "Override" 6 "Standstill" 7 "Failuremode" ;
        ACC_model_raw = int(cp_cam.vl["MRR_0x32E"]["ACC_Mode_S"])
        if set_distance != 0 or set_speed != 0 or ACC_model_raw in [2,3,4]:
            self.is_cruise_latch = True

        # 将巡航状态中的nonAdaptive属性设置为False，表示巡航系统处于自适应模式
        ret.cruiseState.nonAdaptive = False
        ret.cruiseState.enabled = not disengage and self.is_cruise_latch
        ret.cruiseState.available = not ACC_model_raw in [0, 7]

        # blindspot sensors
        # if self.CP.enableBsm:
        #     # used for lane change so its okay for the chime to work on both side.
        #     ret.leftBlindspot = bool(cp.vl["BSM"]["LEFT_APPROACH"])
        #     ret.rightBlindspot = bool(cp.vl["BSM"]["RIGHT_APPROACH"])

        # # 将车道保持辅助系统的状态存储在self.lss_state中
        # self.lss_state = cp_cam.vl["LKAS_HUD_ADAS"]["LKAS_ACTIVE"]
        # # 将车道保持辅助系统的警报信息存储在self.lss_alert中
        # self.lss_alert = cp_cam.vl["LKAS_HUD_ADAS"]["MPCErr"]

        # lane
        self.sm_counter += 1
        if self.sm_counter % 5 == 0:
            self.sm.update()
            self.left_coeff, self.right_coeff, self.lane_central_coeff = self.lane_process()
            self.sm_counter = 0
        if len(self.left_coeff):
            ret.cruiseState.leftLaneCoeff.c0 = float(self.left_coeff[2])
            ret.cruiseState.leftLaneCoeff.c1 = float(self.left_coeff[1])
            ret.cruiseState.leftLaneCoeff.c2 = float(self.left_coeff[0])

            ret.cruiseState.rightLaneCoeff.c0 = float(self.right_coeff[2])
            ret.cruiseState.rightLaneCoeff.c1 = float(self.right_coeff[1])
            ret.cruiseState.rightLaneCoeff.c2 = float(self.right_coeff[0])

            ret.cruiseState.centerLaneCoeff.c0 = float(self.lane_central_coeff[2])
            ret.cruiseState.centerLaneCoeff.c1 = float(self.lane_central_coeff[1])
            ret.cruiseState.centerLaneCoeff.c2 = float(self.lane_central_coeff[0])

        return ret

    @staticmethod
    def get_can_parser(CP):
        messages = [
            # ("VCU_0x342", 20), # 0x242
            
            ("VCU_0x342", 50), # 0x342
            ("IPB_0x1F0", 50),
            ("Left_BCM_0x294", 50),
            ("EPS_0x11F", 100),
            ("EPS_0x318", 50),
            ("SWS_0x3B0", 20), # 0x3b0
            
            # ("TCU_0x212", 50), # 0x342

            ("CS_0x133", 0),

            
            # ("BSM", 20),
            # ("MRR_0x32E", 50),
        ]

        # 从地盘网获取数据
        return CANParser(DBC[CP.carFingerprint]['pt'], messages, CanBus(CP).ECAN)

    @staticmethod
    def get_cam_can_parser(CP):
        messages = [
            ("MRR_0x316", 50),
            ("MRR_0x32D", 50), # 0x32D
            ("MRR_0x32E", 50),
            # HMI  lane
            ("MPC_0x34B", 20), # 
        ]
        return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).CAM)
