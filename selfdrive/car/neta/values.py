# flake8: noqa
from dataclasses import dataclass, field
from selfdrive.car import dbc_dict
from cereal import car
from dataclasses import dataclass
from enum import StrEnum
from typing import Dict, List, Union
from enum import IntFlag
# from openpilot.selfdrive.car import (
#     CarSpecs,
#     DbcDict,
#     PlatformConfig,
#     Platforms,
#     dbc_dict,
# )
from openpilot.selfdrive.car import AngleRateLimit, CarSpecs, PlatformConfig, Platforms, dbc_dict, CanBusBase,DbcDict
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarDocs
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.can.can_define import CANDefine

Ecu = car.CarParams.Ecu
NetworkLocation = car.CarParams.NetworkLocation
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter

HUD_MULTIPLIER = 0.718

class CruiseButtons:
  RESUME = 5
  ACCEL = 4
  DECEL = 3
  CANCEL = 2
  SET = 1

class RADAR:
    ARS410 = 'ars410_radar'
    ARS513 = 'ars513_radar'
    ChuHangRadar = 'ars513ch_radar'

class CarControllerParams:
    STEER_STEP = 1                           # HCA_01/HCA_1 message frequency 100 hz
    ACC_CONTROL_STEP = 1                     # ACC_06/ACC_07/ACC_System frequency 100hHz

    # Documented lateral limits: 3.00 Nm max, rate of change 5.00 Nm/sec.
    # MQB vs PQ maximums are shared, but rate-of-change limited differently
    # based on safety requirements driven by lateral accel testing.

    STEER_MAX = 300                          # Max heading control assist degree 300
    STEER_DRIVER_MULTIPLIER = 3              # weight driver torque heavily
    STEER_DRIVER_FACTOR = 1                  # from dbc

    STEER_TIME_MAX = 360                     # Max time that EPS allows uninterrupted HCA steering control
    STEER_TIME_ALERT = STEER_TIME_MAX - 10   # If mitigation fails, time to soft disengage before EPS timer expires
    STEER_TIME_STUCK_TORQUE = 1.9            # EPS limits same torque to 6 seconds, reset timer 3x within that period

    ACC_limit_scale=1.0
    ACCEL_MAX = 2.0 * ACC_limit_scale                         # 2.0 m/s max acceleration
    ACCEL_MIN = -3.5  * ACC_limit_scale                         # 3.5 m/s max deceleration

    # Lane Tracing Assist (LTA) control limits
    # Assuming a steering ratio of 13.7:
    # Limit to ~2.0 m/s^3 up (7.5 deg/s), ~3.5 m/s^3 down (13 deg/s) at 75 mph
    # Worst case, the low speed limits will allow ~4.0 m/s^3 up (15 deg/s) and ~4.9 m/s^3 down (18 deg/s) at 75 mph,
    # however the EPS has its own internal limits at all speeds which are less than that
    ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.3, 0.15])
    ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.36, 0.26])
    # LKAS_MAX_TORQUE = 1               # A value of 1 is easy to overpower
    # LTA limits
    # EPS ignores commands above this angle and causes PCS to fault
    MAX_STEER_ANGLE = 94.9461  # deg

    def __init__(self, CP):
        can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
        self.STEER_DRIVER_TAKEROVER = 2.0   # Driver intervention threshold 2.0 Nm

        if CP.carFingerprint in NETA_CARS:
          self.LDW_STEP = 5                   # LDW_1 message frequency 20Hz
          self.ACC_HUD_STEP = 4               # ACC_GRA_Anziege frequency 25Hz
          self.STEER_DRIVER_ALLOWANCE = 0.8    # Driver intervention threshold 0.8 Nm
          self.STEER_DRIVER_TAKEROVER = 2.5   # Driver intervention threshold 2.0 Nm
          self.STEER_DELTA_UP = 6             # Max HCA reached in 1.00s (STEER_MAX / (50Hz * 1.00))
          self.STEER_DELTA_DOWN = 10          # Min HCA reached in 0.60s (STEER_MAX / (50Hz * 0.60))

          self.shifter_values = can_define.dv["VCU_Fr05_0E3"]["VCU5_ActGear"]

class BydFlags(IntFlag):
    # Static flags
    # Gen 1 hardware: same CAN messages and same camera
    GEN1 = 1

@dataclass
class NetaPlatformConfig(PlatformConfig):
    dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('neta_s_canfd', 'ars513_radar'))
    flags: int = BydFlags.GEN1

@dataclass(frozen=True, kw_only=True)
class NetaCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.45
  steerRatio: float = 15.6
  mass: float = 2100.0


class CAR(StrEnum):
  NETA_L_ARS513 = "NETA L ARS513"
  NETA_L_CH = "NETA L CH"
  NETA_L_VISION = "NETA L VISION"

class CAR(Platforms):
    NETA_L_ARS513 = PlatformConfig(
        [CarDocs("NETA L ARS513", "All")],
        CarSpecs(mass=2070, wheelbase=2.81, steerRatio=15.84),
        dbc_dict('neta_s_canfd', RADAR.ARS513),
    )
    NETA_L_CH = PlatformConfig(
        [CarDocs("NETA L CH", "All")],
        CarSpecs(mass=2070, wheelbase=2.81, steerRatio=15.84),
        dbc_dict('neta_s_canfd', RADAR.ChuHangRadar),
    )
    NETA_L_VISION = PlatformConfig(
        [CarDocs("NETA L VISION", "All")],
        CarSpecs(mass=2070, wheelbase=2.81, steerRatio=15.84),
        dbc_dict('neta_s_canfd', None),
    )
    

DBC = {
    CAR.NETA_L_ARS513:  dbc_dict('neta_s_canfd', RADAR.ARS513),
    CAR.NETA_L_CH: dbc_dict('neta_s_canfd', RADAR.ChuHangRadar),
    CAR.NETA_L_VISION: dbc_dict('neta_s_canfd', None),
}

NETA_CARS = {CAR.NETA_L_ARS513, CAR.NETA_L_CH, CAR.NETA_L_VISION}
# NETA_CARS = {CAR.NETA_L_ARS513, CAR.NETA_L_VISION}
CANFD_CAR = NETA_CARS

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
      # rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.adas, Ecu.electricBrakeBooster, Ecu.fwdRadar],
      # rx_offset=0x10,
      bus=0,
    ),
  ]
)


class CanBus(CanBusBase):
  def __init__(self,  CP=None) -> None:
    super().__init__(CP, CP.carFingerprint)

    self._e = 0
    self._a = 1
    self._cam = 2

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. HDA2 cars
    # have a different harness than the HDA1 and non-HDA variants in order to split
    # a different bus, since the steering is done by different ECUs.
    if CP.carFingerprint in CANFD_CAR:
      # -YJ- set num = 2 for CANFD car  
      num = 2
    else:
      num = 1
    self.offset = 4*(num - 1)

    self._e += self.offset  
    self._a += self.offset
    self._cam += self.offset

  # vehicle can
  @property
  def ECAN(self):
    # print("vm ECAN: ", self._e)
    return self._e

  # radar can
  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    # print("vm CAM: ", self._cam)
    return self._cam