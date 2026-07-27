"""Small host-side verification for the parallel-leg LQR model."""

import numpy as np

from dynamics import STATE_NAMES, build_continuous_model, controllability_rank
from fit_lqr import fit_gain_surface, stored_lqr_gain
from leg_data import L_MAX, L_MIN


def main() -> None:
    test_points = (
        (L_MIN, L_MIN),
        ((L_MIN + L_MAX) * 0.5, (L_MIN + L_MAX) * 0.5),
        (L_MAX, L_MAX),
        (L_MIN, L_MAX),
        (L_MAX, L_MIN),
    )

    for left, right in test_points:
        a_matrix, b_matrix = build_continuous_model(left, right)
        gain, worst_real = stored_lqr_gain(left, right)
        print(
            f"L={left:.3f}, R={right:.3f}: "
            f"A={a_matrix.shape}, B={b_matrix.shape}, "
            f"rank={controllability_rank(a_matrix, b_matrix)}, "
            f"max_real={worst_real:.6f}, |K|max={np.max(np.abs(gain)):.6f}"
        )

    fit = fit_gain_surface()
    print(f"fit samples: {fit.sample_count}")
    print(f"fit max abs error: {fit.max_abs_error:.9g}")
    print(f"fit mean abs error: {fit.mean_abs_error:.9g}")
    print(f"fit RMS error: {fit.rms_error:.9g}")


if __name__ == "__main__":
    main()
