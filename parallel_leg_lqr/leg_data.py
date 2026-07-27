"""Parallel-leg model parameters and equivalent-leg data.

State order used by Q:
    [x, dx, yaw, dyaw,
     theta_l_l, dtheta_l_l,
     theta_l_r, dtheta_l_r,
     theta_b, dtheta_b]

Input order used by R:
    [T_wl, T_wr, T_bl, T_br]

Leg table columns:
    [leg_length, l_w, l_b, I_l]

Units:
    length: m
    mass: kg
    inertia: kg*m^2
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RobotModel:
    wheel_radius: float
    half_track: float
    body_com_offset: float

    wheel_mass: float
    leg_mass: float
    body_mass: float

    wheel_inertia: float
    body_pitch_inertia: float
    body_yaw_inertia: float

    gravity: float


@dataclass(frozen=True)
class EquivalentLeg:
    length: float
    l_w: float
    l_b: float
    inertia: float


# ---------------------------------------------------------------------------
# Robot rigid-body parameters
# ---------------------------------------------------------------------------

R_W = 0.095
R_L = 0.5 / 2.0
L_C = 0.037

M_W = 1.405
M_L = 0.7682
M_B = 22.3

I_W = M_W * R_W**2
I_B = M_B * (0.50**2 + 0.16**2) / 12.0
I_Z = M_B * (0.50**2 + 0.35**2) / 12.0

G = 9.78

MODEL = RobotModel(
    wheel_radius=R_W,
    half_track=R_L,
    body_com_offset=L_C,
    wheel_mass=M_W,
    leg_mass=M_L,
    body_mass=M_B,
    wheel_inertia=I_W,
    body_pitch_inertia=I_B,
    body_yaw_inertia=I_Z,
    gravity=G,
)


# ---------------------------------------------------------------------------
# LQR scheduling range
# ---------------------------------------------------------------------------

# Keep these synchronized with the MCU-side LQR configuration.
#
# The table contains a 0.14 m row, but the current gain table is generated over
# 0.15~0.35 m. Change L_MIN to 0.14 only when the MCU runtime range is also
# changed to 0.14 m.
L_MIN = 0.14
L_MAX = 0.35
L_STEP = 0.01


# ---------------------------------------------------------------------------
# LQR weights
# ---------------------------------------------------------------------------

Q_DIAG = np.array(
    [
        80.0,  # x
        8.0,   # dx
        400.0,   # yaw
        80.0,   # dyaw
        1000.0,   # theta_l_l
        2.0,    # dtheta_l_l
        1000.0,   # theta_l_r
        2.0,    # dtheta_l_r
        10000.0,  # theta_b
        4.0,   # dtheta_b
    ],
    dtype=float,
)

R_DIAG = np.array(
    [
        1.5,  # T_wl
        1.5,  # T_wr
        0.5,  # T_bl
        0.5,  # T_br
    ],
    dtype=float,
)


# ---------------------------------------------------------------------------
# Equivalent-leg data
# ---------------------------------------------------------------------------

# Columns:
#   leg length, wheel-side COM distance l_w,
#   body-side COM distance l_b, equivalent inertia I_l
#
# I_l is used directly as kg*m^2. Confirm that the source table uses this unit
# before generating gains for hardware.
LEG_DATA_LEFT = np.array(
    [
        [0.14, 0.03448, 0.10552, 0.126],
        [0.15, 0.04230, 0.10770, 0.130],
        [0.16, 0.05012, 0.10988, 0.134],
        [0.17, 0.05794, 0.11206, 0.138],
        [0.18, 0.06576, 0.11424, 0.142],
        [0.19, 0.07358, 0.11642, 0.146],
        [0.20, 0.08140, 0.11860, 0.150],
        [0.21, 0.08922, 0.12078, 0.154],
        [0.22, 0.09704, 0.12296, 0.158],
        [0.23, 0.10486, 0.12514, 0.162],
        [0.24, 0.11268, 0.12732, 0.166],
        [0.25, 0.12050, 0.12950, 0.170],
        [0.26, 0.12832, 0.13168, 0.174],
        [0.27, 0.13614, 0.13386, 0.178],
        [0.28, 0.14396, 0.13604, 0.182],
        [0.29, 0.15178, 0.13822, 0.186],
        [0.30, 0.15960, 0.14040, 0.190],
        [0.31, 0.16742, 0.14258, 0.194],
        [0.32, 0.17524, 0.14476, 0.198],
        [0.33, 0.18306, 0.14694, 0.202],
        [0.34, 0.19088, 0.14912, 0.206],
        [0.35, 0.19870, 0.15130, 0.210],
    ],
    dtype=float,
)

# The current mechanism data are symmetric.
LEG_DATA_RIGHT = LEG_DATA_LEFT.copy()


def equivalent_leg_from_table(
    length: float,
    table: np.ndarray,
) -> EquivalentLeg:
    """Interpolate equivalent-leg parameters at the requested leg length."""
    min_length = float(table[0, 0])
    max_length = float(table[-1, 0])

    if length < min_length or length > max_length:
        raise ValueError(
            f"leg length {length:.6f} is outside "
            f"[{min_length:.6f}, {max_length:.6f}]"
        )

    return EquivalentLeg(
        length=length,
        l_w=float(np.interp(length, table[:, 0], table[:, 1])),
        l_b=float(np.interp(length, table[:, 0], table[:, 2])),
        inertia=float(np.interp(length, table[:, 0], table[:, 3])),
    )


def left_leg_data(length: float) -> EquivalentLeg:
    return equivalent_leg_from_table(length, LEG_DATA_LEFT)


def right_leg_data(length: float) -> EquivalentLeg:
    return equivalent_leg_from_table(length, LEG_DATA_RIGHT)


def equivalent_leg(length: float) -> EquivalentLeg:
    """Compatibility helper for code that assumes identical left/right legs."""
    return left_leg_data(length)


def length_grid() -> np.ndarray:
    count = int(round((L_MAX - L_MIN) / L_STEP)) + 1
    return np.linspace(L_MIN, L_MAX, count, dtype=float)
