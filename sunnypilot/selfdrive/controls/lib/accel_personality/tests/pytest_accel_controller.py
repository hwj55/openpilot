import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.accel_controller import AccelController
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import \
  AccelerationPersonality, ECO, NORMAL, SPORT, STOCK_RISE_RATE

# Stock openpilot accel ceiling, duplicated independently here so the test fails if the normal tier ever drifts.
STOCK_BP = [0., 10., 25., 40.]
STOCK_V = [1.6, 1.2, 0.8, 0.6]
V_EGO_GRID = [0.0, 5.0, 10.0, 17.0, 25.0, 33.0, 40.0, 50.0]


class MockParams:
  def __init__(self, enabled=False, personality=NORMAL):
    self._vals = {"AccelPersonalityEnabled": enabled, "AccelPersonality": int(personality)}

  def get_bool(self, key):
    return bool(self._vals.get(key, False))

  def get(self, key, return_default=False):
    return self._vals.get(key, 0)

  def put(self, key, val, block=False):
    self._vals[key] = int(val)


class MockCarState:
  def __init__(self, vEgo=0.0):
    self.vEgo = vEgo


def make_sm(v_ego=0.0):
  return {'carState': MockCarState(vEgo=v_ego)}


@pytest.fixture
def mock_cp():
  class CP:
    radarUnavailable = False
  return CP()


@pytest.fixture
def mock_mpc():
  class MPC:
    crash_cnt = 0
  return MPC()


def stock_max_accel(v_ego):
  return float(np.interp(v_ego, STOCK_BP, STOCK_V))


def test_disabled_matches_stock(mock_cp, mock_mpc):
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=False, personality=SPORT))
  for v in V_EGO_GRID:
    assert c.get_max_accel(v) == pytest.approx(stock_max_accel(v))
  assert c.get_rise_rate() == STOCK_RISE_RATE
  assert c.personality() == AccelerationPersonality.normal
  assert not c.enabled()


def test_normal_matches_stock(mock_cp, mock_mpc):
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=True, personality=NORMAL))
  for v in V_EGO_GRID:
    assert c.get_max_accel(v) == pytest.approx(stock_max_accel(v))
  assert c.get_rise_rate() == STOCK_RISE_RATE
  assert c.personality() == AccelerationPersonality.normal


def test_eco_is_gentler(mock_cp, mock_mpc):
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=True, personality=ECO))
  for v in V_EGO_GRID:
    assert c.get_max_accel(v) <= stock_max_accel(v) + 1e-6
  assert c.get_rise_rate() < STOCK_RISE_RATE
  assert c.personality() == AccelerationPersonality.eco


def test_sport_is_brisker(mock_cp, mock_mpc):
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=True, personality=SPORT))
  for v in V_EGO_GRID:
    assert c.get_max_accel(v) >= stock_max_accel(v) - 1e-6
  assert c.get_rise_rate() > STOCK_RISE_RATE
  assert c.personality() == AccelerationPersonality.sport


def test_param_clamp(mock_cp, mock_mpc):
  # Out-of-range int must clamp to the max tier (sport), not raise.
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=True, personality=9))
  assert c.personality() == AccelerationPersonality.sport


def test_decel_mod_disabled_is_stock(mock_cp, mock_mpc):
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=False, personality=SPORT))
  assert c.decel_mod(is_e2e=False) == (1.0, 0.0)
  assert c.jerk_factor_scale() == 1.0
  assert c.t_follow_offset() == 0.0


def test_decel_mod_e2e_bypass_is_stock(mock_cp, mock_mpc):
  # Even enabled + sport, an e2e/blended plan must keep stock braking.
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=True, personality=SPORT))
  assert c.decel_mod(is_e2e=True) == (1.0, 0.0)


def test_decel_mod_eco_gentler(mock_cp, mock_mpc):
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=True, personality=ECO))
  jerk_scale, t_follow_offset = c.decel_mod(is_e2e=False)
  assert jerk_scale > 1.0          # smoother decel jerk
  assert t_follow_offset > 0.0     # earlier / gentler decel (bigger gap)


def test_decel_mod_sport_crisper(mock_cp, mock_mpc):
  c = AccelController(mock_cp, mock_mpc, params=MockParams(enabled=True, personality=SPORT))
  jerk_scale, t_follow_offset = c.decel_mod(is_e2e=False)
  assert jerk_scale < 1.0          # crisper decel jerk
  assert t_follow_offset < 0.0     # firmer / later decel


def test_frame_gated_read(mock_cp, mock_mpc):
  params = MockParams(enabled=True, personality=NORMAL)
  c = AccelController(mock_cp, mock_mpc, params=params)
  c.update(make_sm())  # frame 0 -> reads, picks up normal
  params._vals["AccelPersonality"] = int(SPORT)
  for _ in range(19):  # frames 1..19, none on the 20-frame gate boundary
    c.update(make_sm())
  assert c.personality() == AccelerationPersonality.normal
  c.update(make_sm())  # frame 20 -> reads, picks up sport
  assert c.personality() == AccelerationPersonality.sport
