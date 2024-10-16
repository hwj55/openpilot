#!/usr/bin/env python3
# interface.py
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.byd.values import CAR, HUD_MULTIPLIER,NetworkLocation,CanBus, CANFD_CAR

from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret = CarInterfaceBase.get_std_params(candidate)
    ret.carName = "byd"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.byd)]
    ret.radarUnavailable = True
    ret.safetyConfigs[0].safetyParam = 1
    ret.transmissionType = car.CarParams.TransmissionType.automatic

    ret.steerLimitTimer = 0.4              # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.1          # Steering wheel actuator delay in seconds

    if candidate == CAR.BYD_QIN_2021:
      ret.wheelbase = 2.72
      ret.steerRatio = 16.0
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.9871
      ret.mass = 1650. + STD_CARGO_KG
      ret.wheelSpeedFactor = HUD_MULTIPLIER           # the HUD odo is exactly 1 to 1 with gps speed

      # currently not in use, byd is using stock long
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.5, 1.3, 1.0]
      ret.longitudinalActuatorDelayLowerBound = 0.3
      ret.longitudinalActuatorDelayUpperBound = 0.4

    elif candidate == CAR.BYD_HAN_DMP_22:
      ret.radarUnavailable = True
      # for CANFD
      if candidate in CANFD_CAR:
        cfgs = [get_safety_config(car.CarParams.SafetyModel.byd), get_safety_config(car.CarParams.SafetyModel.byd)]
        ret.safetyConfigs = cfgs

      ret.wheelbase = 2.72
      ret.steerRatio = 16.0
      ret.centerToFront = ret.wheelbase * 0.44
      # tire_stiffness_factor = 0.9871
      ret.mass = 1650. + STD_CARGO_KG
      ret.wheelSpeedFactor = 1.038           # the HUD odo is exactly 1 to 1 with gps speed

      ret.steerControlType = car.CarParams.SteerControlType.torque
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      # for testing
      ret.lateralTuning.torque.kp = 1.0
      ret.lateralTuning.torque.kf = 1.0
      ret.lateralTuning.torque.ki = 0.0

    #   no ff
    #   ret.lateralTuning.torque.kp = 0.8
    #   ret.lateralTuning.torque.kf = 5.0
    #   ret.lateralTuning.torque.ki = 0.05

    elif candidate == CAR.BYD_HAN_EV_2020:
      ret.radarUnavailable = False

      ret.wheelbase = 2.72
      ret.steerRatio = 16.0
      ret.centerToFront = ret.wheelbase * 0.44
      # tire_stiffness_factor = 0.9871
      ret.mass = 1650. + STD_CARGO_KG
      ret.wheelSpeedFactor = 1.038           # the HUD odo is exactly 1 to 1 with gps speed

      ret.steerControlType = car.CarParams.SteerControlType.torque
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      # for testing
      # ret.lateralTuning.torque.kp = 1.1
      # ret.lateralTuning.torque.kf = 1.0
      # ret.lateralTuning.torque.ki = 0.05

      ret.lateralTuning.torque.kp = 0.8
      ret.lateralTuning.torque.kf = 5.0
      ret.lateralTuning.torque.ki = 0.05

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

      ret.steerActuatorDelay = 0.1
    #   ret.lateralTuning.pid.kpBP = [0.]
    #   ret.lateralTuning.pid.kiBP = [0.]
    #   ret.lateralTuning.pid.kf = 0.00006
    #   ret.lateralTuning.pid.kpV = [0.6]
    #   ret.lateralTuning.pid.kiV = [0.2]

    # # currently not in use, byd is using stock long
    # ret.openpilotLongitudinalControl = True
    # ret.longitudinalTuning.kpBP = [0., 5., 20.]
    # ret.longitudinalTuning.kpV = [1.5, 1.3, 1.0]
    # ret.longitudinalTuning.deadzoneBP = [0., 8.05, 20]
    # ret.longitudinalTuning.deadzoneV = [0., 0., 0.]
    # ret.longitudinalTuning.kiBP = [0., 5., 20.]
    # ret.longitudinalTuning.kiV = [0.32, 0.23, 0.12]
    # ret.longitudinalActuatorDelayLowerBound = 0.3
    # ret.longitudinalActuatorDelayUpperBound = 0.4

    ret.minEnableSpeed = -1
    ret.enableBsm = True

    # ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    # ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    events = self.create_common_events(ret, c)

    ret.events = events.to_msg()
    # print("当前档位",ret)
    return ret

  def apply(self, c, now_nanos):
    # isLdw = c.hudControl.leftLaneDepart or c.hudControl.rightLaneDepart
    return  self.CC.update(c, self.CS, now_nanos)


