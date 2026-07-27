"""Build the 10-state continuous model used by the MCU LQR.

State order:
    [x, dx, yaw, dyaw,
     theta_l_l, dtheta_l_l,
     theta_l_r, dtheta_l_r,
     theta_b, dtheta_b]

Input order:
    [T_wl, T_wr, T_bl, T_br]
"""

import numpy as np

from amatrix import amatrix
from bmatrix import bmatrix
from leg_data import MODEL, equivalent_leg
from leg_data import MODEL, left_leg_data, right_leg_data


STATE_NAMES = (
    "x",
    "dx",
    "yaw",
    "dyaw",
    "theta_l_l",
    "dtheta_l_l",
    "theta_l_r",
    "dtheta_l_r",
    "theta_b",
    "dtheta_b",
)

INPUT_NAMES = ("T_wl", "T_wr", "T_bl", "T_br")


def build_continuous_model(left_length: float, right_length: float) -> tuple[np.ndarray, np.ndarray]:
    left = left_leg_data(left_length)
    right = right_leg_data(right_length)

    accel_a = amatrix(
        left.length,
        right.length,
        left.l_w,
        left.l_b,
        left.inertia,
        right.l_w,
        right.l_b,
        right.inertia,
    )
    accel_b = bmatrix(
        left.length,
        right.length,
        left.l_w,
        left.l_b,
        left.inertia,
        right.l_w,
        right.l_b,
        right.inertia,
    )

    rw = MODEL.wheel_radius
    rl = MODEL.half_track

    # Convert [ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b]
    # to [ddx, ddyaw, ddtheta_ll, ddtheta_lr, ddtheta_b].
    acceleration_map = np.array(
    [
        # x is the average wheel rolling displacement.
        [
            0.5 * rw,
            0.5 * rw,
            0.0,
            0.0,
            0.0,
        ],

        # Positive yaw convention used by the runtime total_yaw.
        [
            -rw / (2.0 * rl),
            rw / (2.0 * rl),
            -left.length / (2.0 * rl),
            right.length / (2.0 * rl),
            0.0,
        ],

        [0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)
    generalized_a = acceleration_map @ accel_a
    generalized_b = acceleration_map @ accel_b

    a_matrix = np.zeros((10, 10), dtype=float)
    b_matrix = np.zeros((10, 4), dtype=float)

    a_matrix[0, 1] = 1.0
    a_matrix[2, 3] = 1.0
    a_matrix[4, 5] = 1.0
    a_matrix[6, 7] = 1.0
    a_matrix[8, 9] = 1.0

    acceleration_rows = (1, 3, 5, 7, 9)
    angle_columns = (4, 6, 8)
    a_matrix[np.ix_(acceleration_rows, angle_columns)] = generalized_a
    b_matrix[list(acceleration_rows), :] = generalized_b

    return a_matrix, b_matrix


def controllability_matrix(a_matrix: np.ndarray, b_matrix: np.ndarray) -> np.ndarray:
    blocks = []
    power = np.eye(a_matrix.shape[0], dtype=float)
    for _ in range(a_matrix.shape[0]):
        blocks.append(power @ b_matrix)
        power = power @ a_matrix
    return np.hstack(blocks)


def controllability_rank(a_matrix: np.ndarray, b_matrix: np.ndarray) -> int:
    return int(np.linalg.matrix_rank(controllability_matrix(a_matrix, b_matrix)))
