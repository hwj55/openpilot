"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL

_PARAM_REFRESH_FRAMES = max(1, int(1.0 / DT_CTRL))

# Cap how fast the longitudinal command may DEEPEN a brake at onset, so braking eases in gradually
# instead of biting sharply (the planner aTarget is already smooth; the PID proportional term reacting
# to the aEgo lag at onset is what makes the command jerk). Comfort only:
#  - applied solely while the brake command is getting more negative (onset / deepening),
#  - bypassed entirely when aTarget demands a real hard brake, so emergencies are never delayed.
BRAKE_ONSET_JERK = 2.5      # m/s^3, max rate the brake command may deepen at onset
HARD_BRAKE_THRESH = -1.8    # m/s^2, aTarget at/below this = real hard brake -> pass through unmodified


class BrakeOnsetSmoother:
  def __init__(self, params=None):
    self._params = params or Params()
    self._enabled = self._params.get_bool("SmoothBraking")
    self._frame = 0

  def apply(self, output_accel: float, last_output_accel: float, a_target: float) -> float:
    if self._frame % _PARAM_REFRESH_FRAMES == 0:
      self._enabled = self._params.get_bool("SmoothBraking")
    self._frame += 1

    if not self._enabled:
      return output_accel

    # Only ease the onset of a comfort brake: command deepening AND the plan isn't asking for a hard brake.
    if output_accel < last_output_accel and a_target > HARD_BRAKE_THRESH:
      return max(output_accel, last_output_accel - BRAKE_ONSET_JERK * DT_CTRL)

    return output_accel
