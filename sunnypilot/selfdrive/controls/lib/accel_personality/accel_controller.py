"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from cereal import messaging
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import \
  NORMAL, PERSONALITY_MIN, PERSONALITY_MAX, A_CRUISE_MAX_BP, A_CRUISE_MAX_V, RISE_RATE, T_FOLLOW_OFFSET, JERK_FACTOR_SCALE


class AccelController:
  def __init__(self, CP: structs.CarParams, mpc, params=None):
    self._CP = CP
    self._mpc = mpc
    self._params = params or Params()
    self._frame = 0
    self._enabled: bool = self._params.get_bool("AccelPersonalityEnabled")
    self._personality = NORMAL  # cereal AccelerationPersonality ordinal
    self._v_ego = 0.0
    self._jerk_scale = 1.0
    self._t_follow_offset = 0.0
    self._read_params()

  def _read_params(self) -> None:
    self._enabled = self._params.get_bool("AccelPersonalityEnabled")
    # When disabled, resolve to normal so the disabled path and the normal tier share one numeric path
    # that is identical to stock openpilot.
    if not self._enabled:
      self._personality = NORMAL
      return

    self._personality = get_sanitize_int_param("AccelPersonality", PERSONALITY_MIN, PERSONALITY_MAX, self._params)

  def update(self, sm: messaging.SubMaster) -> None:
    if self._frame % int(1. / DT_MDL) == 0:
      self._read_params()
    self._v_ego = sm['carState'].vEgo
    self._frame += 1

  def get_max_accel(self, v_ego: float) -> float:
    return float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_V[self._personality]))

  def get_rise_rate(self) -> float:
    return RISE_RATE[self._personality]

  def decel_mod(self, is_e2e: bool) -> tuple[float, float]:
    # Decel-side comfort shaping applies only to the comfort MPC (ACC) path. When disabled or when the
    # plan is e2e/blended, fall back to stock (no offset/scale) so e2e and blended braking are untouched.
    # The MPC's ACCEL_MIN floor and FCW crash detection do not depend on these comfort weights, so
    # emergency braking authority is preserved regardless of tier.
    if not self._enabled or is_e2e:
      self._jerk_scale = 1.0
      self._t_follow_offset = 0.0
    else:
      self._jerk_scale = JERK_FACTOR_SCALE[self._personality]
      self._t_follow_offset = T_FOLLOW_OFFSET[self._personality]
    return self._jerk_scale, self._t_follow_offset

  def jerk_factor_scale(self) -> float:
    return self._jerk_scale

  def t_follow_offset(self) -> float:
    return self._t_follow_offset

  def enabled(self) -> bool:
    return self._enabled

  def personality(self):
    return self._personality  # cereal AccelerationPersonality ordinal

  def max_accel(self) -> float:
    # Cached value for publishing; publish_longitudinal_plan_sp has no v_ego in scope.
    return self.get_max_accel(self._v_ego)
