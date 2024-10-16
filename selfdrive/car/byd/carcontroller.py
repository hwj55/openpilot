from cereal import car
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car import apply_std_steer_angle_limits, apply_driver_steer_torque_limits_float, common_fault_avoidance
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.byd import bydcan
from openpilot.selfdrive.car.byd.values import CarControllerParams, DBC, CanBus
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
import cereal.messaging as messaging


# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 10  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
#  Nm
MAX_USER_TORQUE = 5

# # 转向角度限制
# # 应用的角度（apply_angle）、实际角度（actual_angle）、车辆速度（v_ego）、以及限制（LIMITS）。
# def apply_byd_steer_angle_limits(apply_angle, actual_angle, v_ego, LIMITS):
#   # pick angle rate limits based on wind up/down
#   steer_up = actual_angle * apply_angle >= 0. and abs(apply_angle) > abs(actual_angle)
#   # 选择角度速率限制，如果需要选择上限，则使用LIMITS中的ANGLE_RATE_LIMIT_UP，否则使用ANGLE_RATE_LIMIT_DOWN。
#   rate_limits = LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else LIMITS.ANGLE_RATE_LIMIT_DOWN

#   return clip(apply_angle, actual_angle - rate_limits, actual_angle + rate_limits)


class CarController(CarControllerBase):
    def __init__(self, dbc_name, CP, VM):
        # self.params = CarControllerParams(CP)
        self.CAN = CanBus(CP)
        self.CP = CP
        self.CCP = CarControllerParams(CP)
        self.CCS = bydcan

        self.frame = 0
        self.apply_steer=0
        self.apply_steer_last = 0
        self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
        # self.packer_pt = CANPacker(dbc_name)
        self.steer_rate_counter = 0
        self.lat_active_pre = False

        self.lcc_counter = None
        self.acc_counter = 0

        self.bypass_raw_msg = False # 直接透传原车CAN报文
        self.bypass_raw_acc = True #透传ACC控制量
        self.bypass_raw_lcc = False #透传使用原车LCC扭矩控制量
        

    def update(self, CC, CS, now_nanos):
        self.frame += 1
        actuators = CC.actuators
        can_sends = []  # 初始化一个空列表，用于存储生成的 CAN 消息

        # lane hmi  20hz
        if self.frame % 5 == 0:
          left_lane_coeff = CS.out.cruiseState.leftLaneCoeff
          right_lane_coeff = CS.out.cruiseState.rightLaneCoeff
          msg = self.CCS.create_lane_hmi(self.packer_pt, self.CAN, CS.MPC_0x34B_lane_raw_msg, left_lane_coeff, right_lane_coeff)
          can_sends.append(msg)

        # LCC  --torque
        if (self.frame % self.CCP.STEER_STEP) == 0: 
            # >100 degree/sec steering fault prevention
            self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, CC.latActive,
                                                                self.steer_rate_counter, MAX_STEER_RATE_FRAMES)
            
            # 如果EPS not active，steer = 0; 但是都是得逐步变小
            eps_active = CS.out.cruiseState.epsActive
            new_steer = CC.actuators.steer
            
            if not eps_active or not CC.latActive:
              new_steer = 0
            
            apply_steer = apply_driver_steer_torque_limits_float(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
            self.apply_steer = apply_steer
            

            raw_counter = int(CS.ID0x316_lcc_raw_msg.get("Message_Alive_Counter316_S", 0))
            if self.lcc_counter is not  None:
                self.lcc_counter += 1
            else:
                self.lcc_counter = raw_counter
            # self.lcc_counter滚动计数累加 0~15
            self.lcc_counter = self.lcc_counter % 16
            
            msg1 = self.CCS.create_lcc_cmd(self.packer_pt, self.CAN, CS.ID0x316_lcc_raw_msg, self.lcc_counter, apply_steer, apply_steer_req, eps_active, self.bypass_raw_lcc, self.bypass_raw_msg)
            can_sends.append(msg1)

            self.apply_steer_last = apply_steer
            self.lat_active_pre = CC.latActive

        # ACC
        if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
        #   if frogpilot_variables.sport_plus:
        #     accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX_PLUS) if CC.longActive else 0
        #   else:
          accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
          
          self.acc_counter += 1
          msg2 = self.CCS.create_acc_cmd(self.packer_pt, self.CAN, CS.ID0x32e_acc_raw_msg, self.acc_counter, accel, CC.longActive, self.bypass_raw_acc)
          # han 22 ACC 不能转发给相机，因为踏实ACC看起来是雷达直接发给底盘的
          # can_sends.append(msg2)
      
        # # frequency doesn't matter, but the counter must match + 1 else it will fault
        # if CS.out.standstill and CC.latActive and (self.frame % 100 == 0):  # 如果车辆处于静止状态且控制器处于激活状态，并且帧数能被100整除
        #     # 生成用于发送控制按钮状态的CAN消息
        #     can_sends.append(self.CCS.send_buttons(self.packer_pt, (CS.counter_pcm_buttons + 1) % 16))

        new_actuators = actuators.as_builder()
        new_actuators.steer = self.apply_steer
        new_actuators.steerOutputCan = self.apply_steer

        return new_actuators, can_sends  # 返回更新后的actuators对象和生成的CAN消息列表
