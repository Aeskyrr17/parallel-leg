"""Compute and fit the scheduled 4x10 continuous-time LQR gain."""

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are

from dynamics import build_continuous_model, controllability_rank
from leg_data import Q_DIAG, R_DIAG, length_grid


@dataclass(frozen=True)
class FitResult:
    coefficients: np.ndarray
    max_abs_error: float
    mean_abs_error: float
    rms_error: float
    worst_closed_loop_real_part: float
    sample_count: int


def quadratic_features(left_length: float, right_length: float) -> np.ndarray:
    return np.array(
        [
            1.0,
            left_length,
            right_length,
            left_length**2,
            left_length * right_length,
            right_length**2,
        ],
        dtype=float,
    )


def stored_lqr_gain(left_length: float, right_length: float) -> tuple[np.ndarray, float]:
    """Return the gain stored by the MCU and the largest closed-loop real eigenvalue.

    Standard LQR gives u = -K(x-r). The MCU evaluates
        u = K_stored @ (observed-reference),
    hence K_stored = -K.
    """
    a_matrix, b_matrix = build_continuous_model(left_length, right_length)

    rank = controllability_rank(a_matrix, b_matrix)
    if rank != a_matrix.shape[0]:
        raise ValueError(
            f"model is not controllable at L={left_length:.3f}, R={right_length:.3f}: "
            f"rank={rank}"
        )

    q_matrix = np.diag(Q_DIAG)
    r_matrix = np.diag(R_DIAG)
    riccati = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
    standard_gain = np.linalg.solve(r_matrix, b_matrix.T @ riccati)
    stored_gain = -standard_gain

    closed_loop = a_matrix + b_matrix @ stored_gain
    worst_real_part = float(np.max(np.real(np.linalg.eigvals(closed_loop))))
    if worst_real_part >= 0.0:
        raise ValueError(
            f"unstable closed loop at L={left_length:.3f}, R={right_length:.3f}: "
            f"max(real(lambda))={worst_real_part:.6g}"
        )

    return stored_gain, worst_real_part


def fit_gain_surface() -> FitResult:
    features = []
    gains = []
    worst_closed_loop = -np.inf

    lengths = length_grid()
    for left_length in lengths:
        for right_length in lengths:
            gain, closed_loop_real = stored_lqr_gain(
                float(left_length), float(right_length)
            )
            features.append(quadratic_features(float(left_length), float(right_length)))
            gains.append(gain.reshape(-1))
            worst_closed_loop = max(worst_closed_loop, closed_loop_real)

    design = np.asarray(features, dtype=float)
    samples = np.asarray(gains, dtype=float)

    # design @ coefficients.T ~= samples
    coefficients = np.linalg.lstsq(design, samples, rcond=None)[0].T
    predicted = design @ coefficients.T
    error = predicted - samples

    return FitResult(
        coefficients=coefficients,
        max_abs_error=float(np.max(np.abs(error))),
        mean_abs_error=float(np.mean(np.abs(error))),
        rms_error=float(np.sqrt(np.mean(error**2))),
        worst_closed_loop_real_part=float(worst_closed_loop),
        sample_count=int(samples.shape[0]),
    )


def evaluate_fitted_gain(
    coefficients: np.ndarray, left_length: float, right_length: float
) -> np.ndarray:
    flat_gain = coefficients @ quadratic_features(left_length, right_length)
    return flat_gain.reshape((4, 10))
