"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import custom

# Single source of truth for the tiers: the cereal enum (eco @0, normal @1, sport @2).
AccelerationPersonality = custom.LongitudinalPlanSP.AccelerationPersonality
ECO = AccelerationPersonality.eco
NORMAL = AccelerationPersonality.normal
SPORT = AccelerationPersonality.sport

# Valid range for the AccelPersonality int param (used for clamping).
PERSONALITY_MIN = min(AccelerationPersonality.schema.enumerants.values())
PERSONALITY_MAX = max(AccelerationPersonality.schema.enumerants.values())

# Speed-dependent max acceleration ceiling (m/s^2), interpolated over A_CRUISE_MAX_BP (m/s).
A_CRUISE_MAX_BP = [0., 10., 25., 40.]

# Stock openpilot values (selfdrive/controls/lib/longitudinal_planner.py). Single source of truth so the
# normal tier (and the disabled path, which resolves to normal) is identical to stock by construction.
STOCK_A_CRUISE_MAX_V = [1.6, 1.2, 0.8, 0.6]
STOCK_RISE_RATE = 0.05  # m/s^2 per planner cycle (DT_MDL=0.05s -> 1.0 m/s^2/s); stock accel_clip upper-bound slew

# Eco launch (v<=10) is kept ~stock so departing a stop/green light is not sluggish (no getting honked at);
# the eco character is in the cruise/highway roll-on (>=25 m/s), not off-the-line.
A_CRUISE_MAX_V = {
  ECO:    [1.6, 1.10, 0.55, 0.40],
  NORMAL: STOCK_A_CRUISE_MAX_V,
  SPORT:  [1.8, 1.40, 1.00, 0.75],
}

# How fast the max-accel ceiling is allowed to rise, in m/s^2 per planner cycle (only the upward slew;
# the downward/braking slew stays at the stock rate for safety).
RISE_RATE = {
  ECO:    0.025,
  NORMAL: STOCK_RISE_RATE,
  SPORT:  0.10,
}

# --- Decel-side comfort, applied to the comfort MPC (non-e2e ACC) path only ---
# e2e/blended keep stock braking, and the MPC's ACCEL_MIN floor + FCW crash detection are unaffected
# by these comfort weights (they only shape gentle decel onset/smoothness when following/cruising).

# Offset (s) added to the Driving Personality's T_FOLLOW: + = earlier and gentler decel (bigger gap).
T_FOLLOW_OFFSET = {
  ECO:    0.15,
  NORMAL: 0.0,
  SPORT:  -0.10,
}

# Multiplies the Driving Personality's jerk factor: >1 = smoother decel/accel (more jerk cost), <1 = crisper.
JERK_FACTOR_SCALE = {
  ECO:    1.2,
  NORMAL: 1.0,
  SPORT:  0.85,
}
