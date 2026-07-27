"""MATLAB Bmatrix-compatible wrapper."""

import numpy as np

from model import acceleration_jacobians


def bmatrix(
    l_l: float,
    l_r: float,
    l_wl: float,
    l_bl: float,
    I_ll: float,
    l_wr: float,
    l_br: float,
    I_lr: float,
) -> np.ndarray:
    _, matrix = acceleration_jacobians(
        l_l, l_r, l_wl, l_bl, I_ll, l_wr, l_br, I_lr
    )
    return matrix
