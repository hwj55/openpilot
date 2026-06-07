from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.brake_smoother import BrakeOnsetSmoother, BRAKE_ONSET_JERK, HARD_BRAKE_THRESH

MAX_STEP = BRAKE_ONSET_JERK * DT_CTRL


class MockParams:
  def __init__(self, enabled):
    self._v = {"SmoothBraking": enabled}

  def get_bool(self, k):
    return bool(self._v.get(k, False))


def test_disabled_passthrough():
  s = BrakeOnsetSmoother(params=MockParams(False))
  assert s.apply(-1.0, 0.0, -0.5) == -1.0


def test_comfort_onset_rate_limited():
  s = BrakeOnsetSmoother(params=MockParams(True))
  # deepening comfort brake (a_target above the hard threshold) -> clamped to last - max_step
  out = s.apply(-0.5, 0.0, -0.5)
  assert out == -MAX_STEP


def test_hard_brake_passthrough():
  s = BrakeOnsetSmoother(params=MockParams(True))
  # a_target at/below hard threshold -> no limiting, full brake passes through
  assert s.apply(-1.0, 0.0, HARD_BRAKE_THRESH - 0.1) == -1.0


def test_release_passthrough():
  s = BrakeOnsetSmoother(params=MockParams(True))
  # output less negative than last (releasing brake) -> not limited
  assert s.apply(-0.2, -0.5, -0.3) == -0.2


def test_already_within_rate():
  s = BrakeOnsetSmoother(params=MockParams(True))
  # small deepening within the cap is unchanged
  out = s.apply(-MAX_STEP * 0.5, 0.0, -0.3)
  assert out == -MAX_STEP * 0.5
