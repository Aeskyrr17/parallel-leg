# Parallel-leg offline LQR generator

This folder converts the supplied five-equation MATLAB model into the 10-state,
4-input model used by the MCU controller and fits every gain entry with

```text
a0 + a1*Ll + a2*Lr + a3*Ll^2 + a4*Ll*Lr + a5*Lr^2
```

## Files

- `model.py`: direct numerical form of the five MATLAB equations.
- `amatrix.py`: MATLAB-compatible 5x3 acceleration Jacobian.
- `bmatrix.py`: MATLAB-compatible 5x4 input Jacobian.
- `dynamics.py`: builds the 10x10 `A` and 10x4 `B`.
- `leg_data.py`: robot parameters, Q/R, scheduling range, equivalent-leg model.
- `fit_lqr.py`: CARE solve and two-dimensional quadratic fitting.
- `generate_lqr_header.py`: writes `lqr_coeffs.hpp`.
- `verify_model.py`: controllability/stability smoke test.
- `lqr_codegen.cmake`: CMake integration snippet.

## State and input order

```text
x = [x, dx, yaw, dyaw,
     theta_l_l, dtheta_l_l,
     theta_l_r, dtheta_l_r,
     theta_b, dtheta_b]

u = [T_wl, T_wr, T_bl, T_br]
```

The generated table already contains the negative sign required by the current runtime:

```text
u = K_stored * (observed - reference)
K_stored = -K_standard_LQR
```

## Critical model gap

The MATLAB equations require these properties for each equivalent leg:

```text
l_w, l_b, I_l
```

The supplied parallel-leg MATLAB file does not define how they vary with leg length.
`leg_data.py` therefore contains a temporary uniform-equivalent-leg approximation:

```text
l_w = l/2
l_b = l/2
I_l = m_l*l^2/3
```

Replace `equivalent_leg()` with CAD-derived composite-body data or an identified
polynomial/table before using the generated gains on hardware. The default output is
only a pipeline verification result.

## Run

```bash
python -m pip install -r requirements.txt
python verify_model.py
python generate_lqr_header.py --output ../control/include/lqr_coeffs.hpp
```

## CMake

Example:

```cmake
set(LQR_TOOL_DIR "${CMAKE_SOURCE_DIR}/parallel_leg_lqr")
set(LQR_COEFFS_HPP "${CMAKE_SOURCE_DIR}/control/include/lqr_coeffs.hpp")
set(FIRMWARE_TARGET your_actual_target)
include("${LQR_TOOL_DIR}/lqr_codegen.cmake")
```

Keep the following synchronized with the MCU configuration:

- `L_MIN`, `L_MAX`, `L_STEP`;
- state and actuator order;
- runtime polynomial basis;
- robot mass/inertia parameters;
- `Q_DIAG` and `R_DIAG`;
- equivalent-leg properties.
