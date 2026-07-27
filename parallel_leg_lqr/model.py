"""Five-equation linearized parallel-leg dynamics.

The equations are transcribed from the supplied MATLAB model in the form

    M(q) * q_ddot + K * theta + N * u = 0

therefore

    q_ddot = -M^-1 K * theta - M^-1 N * u.
"""

import numpy as np

from leg_data import MODEL


def mass_matrix(
    l_l: float,
    l_r: float,
    l_wl: float,
    l_bl: float,
    i_ll: float,
    l_wr: float,
    l_br: float,
    i_lr: float,
) -> np.ndarray:
    p = MODEL
    rw = p.wheel_radius
    rl = p.half_track
    lc = p.body_com_offset
    mw = p.wheel_mass
    ml = p.leg_mass
    mb = p.body_mass
    iw = p.wheel_inertia
    ib = p.body_pitch_inertia
    iz = p.body_yaw_inertia

    matrix = np.zeros((5, 5), dtype=float)

    matrix[0, 0] = iw * l_l / rw + mw * rw * l_l + ml * rw * l_bl
    matrix[0, 2] = ml * l_wl * l_bl - i_ll

    matrix[1, 1] = iw * l_r / rw + mw * rw * l_r + ml * rw * l_br
    matrix[1, 3] = ml * l_wr * l_br - i_lr

    wheel_term = mw * rw**2 + iw + ml * rw**2 + 0.5 * mb * rw**2
    matrix[2, 0] = -wheel_term
    matrix[2, 1] = -wheel_term
    matrix[2, 2] = -(ml * rw * l_wl + 0.5 * mb * rw * l_l)
    matrix[2, 3] = -(ml * rw * l_wr + 0.5 * mb * rw * l_r)

    body_coupling = mw * rw * lc + iw * lc / rw + ml * rw * lc
    matrix[3, 0] = body_coupling
    matrix[3, 1] = body_coupling
    matrix[3, 2] = ml * l_wl * lc
    matrix[3, 3] = ml * l_wr * lc
    matrix[3, 4] = -ib

    yaw_wheel_coupling = iz * rw / (2.0 * rl) + iw * rl / rw
    matrix[4, 0] = yaw_wheel_coupling
    matrix[4, 1] = -yaw_wheel_coupling
    matrix[4, 2] = iz * l_l / (2.0 * rl)
    matrix[4, 3] = -iz * l_r / (2.0 * rl)

    return matrix


def gravity_matrix(
    l_l: float,
    l_r: float,
    l_wl: float,
    l_wr: float,
) -> np.ndarray:
    """Jacobian of the five equations with respect to [theta_ll, theta_lr, theta_b]."""
    p = MODEL
    matrix = np.zeros((5, 3), dtype=float)
    matrix[0, 0] = (p.leg_mass * l_wl + 0.5 * p.body_mass * l_l) * p.gravity
    matrix[1, 1] = (p.leg_mass * l_wr + 0.5 * p.body_mass * l_r) * p.gravity
    matrix[3, 2] = p.body_mass * p.gravity * p.body_com_offset
    return matrix


def input_equation_matrix(l_l: float, l_r: float) -> np.ndarray:
    """Jacobian of the five equations with respect to [T_wl, T_wr, T_bl, T_br]."""
    p = MODEL
    rw = p.wheel_radius
    rl = p.half_track
    lc = p.body_com_offset

    matrix = np.zeros((5, 4), dtype=float)
    matrix[0, :] = [-(1.0 + l_l / rw), 0.0, 1.0, 0.0]
    matrix[1, :] = [0.0, -(1.0 + l_r / rw), 0.0, 1.0]
    matrix[2, :] = [1.0, 1.0, 0.0, 0.0]
    matrix[3, :] = [-lc / rw, -lc / rw, -1.0, -1.0]
    matrix[4, :] = [-rl / rw, rl / rw, 0.0, 0.0]
    return matrix


def acceleration_jacobians(
    l_l: float,
    l_r: float,
    l_wl: float,
    l_bl: float,
    i_ll: float,
    l_wr: float,
    l_br: float,
    i_lr: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return the MATLAB-equivalent J_A (5x3) and J_B (5x4)."""
    mass = mass_matrix(l_l, l_r, l_wl, l_bl, i_ll, l_wr, l_br, i_lr)
    gravity = gravity_matrix(l_l, l_r, l_wl, l_wr)
    inputs = input_equation_matrix(l_l, l_r)

    try:
        a_matrix = -np.linalg.solve(mass, gravity)
        b_matrix = -np.linalg.solve(mass, inputs)
    except np.linalg.LinAlgError as exc:
        raise ValueError("parallel-leg mass matrix is singular") from exc

    return a_matrix, b_matrix
