from cereal import car
from panda import Panda
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import STD_CARGO_KG, get_safety_config, create_button_events
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.neta.values import CAR, CanBus, NetworkLocation, TransmissionType, GearShifter,CruiseButtons

from selfdrive.controls.lib.events import Events

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
SteerControlType = car.CarParams.SteerControlType

# TODO
# ENABLE_BUTTONS = (CruiseButtons.RES_ACCEL, CruiseButtons.SET_DECEL, CruiseButtons.CANCEL)

ButtonType = car.CarState.ButtonEvent.Type
BUTTONS_DICT = {CruiseButtons.ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL: ButtonType.decelCruise,
                CruiseButtons.SET: ButtonType.setCruise, CruiseButtons.CANCEL: ButtonType.cancel,
                CruiseButtons.RESUME: ButtonType.resumeCruise}

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.CAN = CanBus(CP)
    if CP.networkLocation == NetworkLocation.fwdCamera:
      self.ext_bus = self.CAN.pt
      self.cp_ext = self.cp
    else:
      self.ext_bus = self.CAN.cam
      self.cp_ext = self.cp_cam

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "neta"
    ret.radarUnavailable = True
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.neta)]
    ret.enableBsm = False

    ret.transmissionType = TransmissionType.manual
    ret.networkLocation = NetworkLocation.fwdCamera

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4
    ret.steerRatio = 15.6  # Let the params learner figure this out
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kf = 0.00006
    ret.lateralTuning.pid.kpV = [0.6]
    ret.lateralTuning.pid.kiV = [0.2]

    # Global longitudinal tuning defaults, can be overridden per-vehicle
    ret.experimentalLongitudinalAvailable = ret.networkLocation == NetworkLocation.gateway or docs
    if experimental_long:
      # Proof-of-concept, prep for E2E only. No radar points available. Panda ALLOW_DEBUG firmware required.
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_NETA_LONG_CONTROL
      if ret.transmissionType == TransmissionType.manual:
        ret.minEnableSpeed = 4.5

    ret.pcmCruise = True
    ret.stoppingControl = True
    ret.startingState = True
    ret.startAccel = 1.0
    ret.stopAccel = -0.55
    ret.vEgoStarting = 1.0
    ret.vEgoStopping = 1.0
    ret.longitudinalTuning.kpV = [0.1]
    ret.longitudinalTuning.kiV = [0.0]

    if candidate == CAR.NETA_L_ARS513 or candidate == CAR.NETA_L_CH or candidate == CAR.NETA_L_VISION:
      ret.radarUnavailable = False
      ret.openpilotLongitudinalControl = True
      ret.steerControlType = SteerControlType.angle

      # -YJ- red panda, two panda
      # cfgs = [get_safety_config(car.CarParams.SafetyModel.neta), ]
      # cfgs.insert(0, get_safety_config(car.CarParams.SafetyModel.noOutput))

      # wheelbase @18 :Float32;       # [m] distance from rear axle to front axle
      # centerToFront @19 :Float32;   # [m] distance from center of mass to front axle

      cfgs = [get_safety_config(car.CarParams.SafetyModel.neta), get_safety_config(car.CarParams.SafetyModel.neta)]
      ret.safetyConfigs = cfgs
      ret.mass = 2070 + STD_CARGO_KG
      ret.wheelbase = 2.81
      ret.steerRatio = 15.84
      tire_stiffness_factor = 0.5533
      ret.steerActuatorDelay = 0 #0.25
      ret.steerLimitTimer = 0.8

    else:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
      ret.steerLimitTimer = 0.4
      raise ValueError(f"unsupported car {candidate}")

    ret.autoResumeSng = ret.minEnableSpeed == -1
    ret.centerToFront = ret.wheelbase * 0.45
    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)


    ret.buttonEvents = [
      *create_button_events(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT),
      # *create_button_events(self.CS.cruise_setting, self.CS.prev_cruise_setting, {1: ButtonType.altButton1}),
    ]

    events = self.create_common_events(ret, extra_gears=[GearShifter.eco, GearShifter.sport, GearShifter.manumatic],
                                       pcm_enable=False,
                                       enable_buttons=(ButtonType.setCruise, ButtonType.resumeCruise))

    # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
    # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
    # Main button also can trigger an engagement on these cars
    # allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    # events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise, allow_enable=True)

    # Low speed steer alert hysteresis logic
    if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = True
    elif ret.vEgo > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(EventName.belowSteerSpeed)

    if self.CS.CP.openpilotLongitudinalControl:
      if ret.vEgo < self.CP.minEnableSpeed + 0.5:
        events.add(EventName.belowEngageSpeed)
      if c.enabled and ret.vEgo < self.CP.minEnableSpeed:
        events.add(EventName.speedTooLow)

    if self.CC.eps_timer_soft_disable_alert:
      events.add(EventName.steerTimeLimit)

    ret.events = events.to_msg()

    return ret

  # def apply(self, c, now_nanos, frogpilot_toggles):
  #   new_actuators, can_sends, self.eps_timer_soft_disable_alert = self.CC.update(c, self.CS, self.ext_bus, now_nanos, frogpilot_toggles)
  #   return new_actuators, can_sends
