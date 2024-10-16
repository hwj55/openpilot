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


class RADAR:
  ARS410 = 'ars410_radar'
  ARS513 = 'ars513_radar'
  ChuHangRadar = 'ars513ch_radar'

class CarControllerParams:
  STEER_STEP = 2                           # HCA_01/HCA_1 message frequency 50Hz
  ACC_CONTROL_STEP = 2                     # ACC_06/ACC_07/ACC_System frequency 50Hz

  # Documented lateral limits: 3.00 Nm max, rate of change 5.00 Nm/sec.
  # MQB vs PQ maximums are shared, but rate-of-change limited differently
  # based on safety requirements driven by lateral accel testing.

  STEER_MAX = 3                          # Max heading control assist torque 3.00 Nm
  STEER_DRIVER_MULTIPLIER = 3              # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1                  # from dbc

  STEER_TIME_MAX = 360                     # Max time that EPS allows uninterrupted HCA steering control
  STEER_TIME_ALERT = STEER_TIME_MAX - 10   # If mitigation fails, time to soft disengage before EPS timer expires
  STEER_TIME_STUCK_TORQUE = 1.9            # EPS limits same torque to 6 seconds, reset timer 3x within that period

  ACCEL_MAX = 2.0                          # 2.0 m/s max acceleration
  ACCEL_MAX_PLUS = 4.0                     # 4.0 m/s max acceleration
  ACCEL_MIN = -3.5                         # 3.5 m/s max deceleration
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[5., .8, .15])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[5., 3.5, 0.4])
  # LKAS_MAX_TORQUE = 1               # A value of 1 is easy to overpower
  # STEER_THRESHOLD = 1.0

  def __init__(self, CP):
    print("vm CP.carFingerprint: ", CP.carFingerprint)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.STEER_TORQUE_RATE_MAX = 10                    # Max rate of change 10.00 Nm/sec

    # 刚激活时，横向需要从0NM开始发，横向请求扭矩的rate需要限制（10NM/S，即每个can报文发送周期，最多增加或者减少0.2NM）
    self.STEER_DRIVER_ALLOWANCE = 0.8    # Driver intervention threshold 0.8 Nm
    self.STEER_DRIVER_TAKEROVER = 2.5   # Driver intervention threshold 2.0 Nm
    self.STEER_DELTA_UP = 0.12 #self.STEER_TORQUE_RATE_MAX/50.0             # Max HCA reached in 1.00s (STEER_MAX / (50Hz * 1.00))
    self.STEER_DELTA_DOWN = 0.12 #self.STEER_TORQUE_RATE_MAX/50.0          # Min HCA reached in 0.60s (STEER_MAX / (50Hz * 0.60))
    self.shifter_values = can_define.dv["VCU_0x342"]["VCU_Goal_Gear_S"]

class BydFlags(IntFlag):
    # Static flags
    # Gen 1 hardware: same CAN messages and same camera
    GEN1 = 1

@dataclass
class BydPlatformConfig(PlatformConfig):
    dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("byd_general_pt", None))
    flags: int = BydFlags.GEN1

@dataclass(frozen=True, kw_only=True)
class BydCarSpecs(CarSpecs):
  centerToFrontRatio: float = 0.45
  steerRatio: float = 15.6
  mass: float = 2100.0


CANFD_CAR = {"BYD_HAN_DMP_22"}

class CAR(StrEnum):
  BYD_QIN_2021 = "BYD_QIN_2021"
  BYD_HAN_DMP_22 = "BYD_HAN_DMP_22"
  BYD_HAN_EV_2020 = "BYD_HAN_EV_2020"
  BYD_SONG_Plus_21 = "BYD_SONG_Plus_21"

class CAR(Platforms):
    BYD_QIN_2021 = PlatformConfig(
        [CarDocs("BYD QIN 2021", "All")],
        CarSpecs(mass=2100.0, wheelbase=2.959, steerRatio=15.0),
        dbc_dict("byd_general_pt", None),
    )
    BYD_SONG_Plus_21 = PlatformConfig(
        [CarDocs("BYD SONG PLUS 21", "All")],
        CarSpecs(mass=2100.0, wheelbase=2.959, steerRatio=15.0),
        dbc_dict("tang_song", None),
    )
    BYD_HAN_DMP_22 = PlatformConfig(
        [CarDocs("BYD HAN DMP 22", "All")],
        CarSpecs(mass=2100, wheelbase=2.959, steerRatio=15.6),
        dbc_dict("byd_han_2021", None),
        
    )
    BYD_HAN_EV_2020 = PlatformConfig(
        [CarDocs("BYD HAN EV 2020", "All")],
        CarSpecs(mass=2100.0, wheelbase=2.92, steerRatio=15.0),
        dbc_dict("byd_han_ev_2020", RADAR.ARS410),
    )
    

DBC = {
    CAR.BYD_QIN_2021: dbc_dict("byd_general_pt", None),
    CAR.BYD_HAN_DMP_22: dbc_dict("byd_han_2021", None),
    CAR.BYD_HAN_EV_2020: dbc_dict("byd_han_ev_2020", None),
}


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

    self._a = 1
    self._e = 0
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