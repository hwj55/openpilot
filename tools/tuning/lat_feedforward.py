
from math import fabs


def get_steer_feedforward_han2022(v_ego, lateral_accel_value):
    # ANGLE_COEF = 3.25722272
    # ANGLE_COEF2 = 0.66571096
    # ANGLE_OFFSET = -0.00178209
    # SPEED_OFFSET = 0.00000000
    # SIGMOID_COEF_RIGHT = 0.20355202
    # SIGMOID_COEF_LEFT = 0.17411275
    # SPEED_COEF = 1.32113873
    # SPEED_COEF2 = 0.00000000
    # SPEED_OFFSET2 = 94.19442068

    # 0926
    ANGLE_COEF = 2.93759992
    ANGLE_COEF2 = 0.65750147
    ANGLE_OFFSET = -0.00275237
    SPEED_OFFSET = 0.00000000
    SIGMOID_COEF_RIGHT = 0.20661008
    SIGMOID_COEF_LEFT = 0.17490651
    SPEED_COEF = 1.28704177
    SPEED_COEF2 = 0.00000000
    SPEED_OFFSET2 = 139.98851042

    x = ANGLE_COEF * (lateral_accel_value + ANGLE_OFFSET) * (40.23 / (max(1.0,v_ego + SPEED_OFFSET))**SPEED_COEF)
    sigmoid_factor = (SIGMOID_COEF_RIGHT if (lateral_accel_value + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT)
    sigmoid = x / (1. + fabs(x))
    sigmoid *= sigmoid_factor * sigmoid_factor
    sigmoid *= max(0.2, 40.23 / (max(1.0,v_ego + SPEED_OFFSET2))**SPEED_COEF2)
    linear = ANGLE_COEF2 * (lateral_accel_value + ANGLE_OFFSET)
    return sigmoid + linear