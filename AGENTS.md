# Parallel WBR Agent Notes

Last updated: 2026-07-25

This is the persistent handoff for agents working on the parallel wheel-legged robot. Update it
whenever a control boundary changes, a blocker is resolved, or a new control state becomes
hardware-validated.

## Repository boundaries

- Writable target repository:
  `C:\01_Workspace\RM\infantry\parallel_wbr`
- Read-only validated old real-robot reference:
  `C:\01_Workspace\RM\infantry\wbr_2026`
- Read-only MuJoCo architecture reference:
  `C:\01_Workspace\RM\infantry\wbr_mujoco`
- `docs/reference/mujoco_control/`, if present, is also read-only and is not part of the embedded
  build.
- Never modify, format, generate files in, or commit changes to either reference repository.
- Hardware APIs, motor/CAN management, ThreadX, and directories follow the current repository.
- Active `wbr_2026` behavior is the real-robot behavior source. MuJoCo is only an organization and
  interface reference unless the user explicitly says otherwise.
- Do not add `control/include/config.hpp`; it can shadow `configs/generated/config.hpp`.

## Current application architecture

- `demo/` is excluded from CMake; `control/` is included recursively.
- `control/app.cpp::app_start()` calls `wbr::control::start_control_task()`.
- AHRS and remoter are initialized with their module-default, no-argument `init()` calls. Their
  libraries were not modified.
- There is one high-frequency business-control thread, `wbr_control`, at priority 5.
- A separate priority-7 `wbr_input` adapter waits for the AHRS heartbeat, consumes the existing
  AHRS/remoter topics, and updates one mutex-protected latest snapshot.
- `wbr_control` copies that snapshot with `TX_NO_WAIT`. It never calls `msg::read()` and never uses
  `TX_WAIT_FOREVER`.
- The Control cycle directly calls `Function`, `Odometry`, both `Pendulum` objects,
  `ChassisController`, `LQR`, `VMCsolver`, limits, six motor-buffer writes, and one final LK batch
  commit. Intermediate control products do not use the message bus.
- All six LK feedback values are copied into a named `motor_feedback_frame` inside one short
  interrupt-disabled section before the cycle uses them.
- Four LK8016 hips and two LK9025 wheels are constructed from generated device-tree configs and
  registered with `lkmotorhandler`.
- The only application-level `send_control()` call is at the end of `control_entry()`.
- `k_default_chassis.runtime.actuation_enabled` is still `false`. Every cycle therefore computes
  requests but sends relaxed/zero-current commands.

Current cycle:

```text
six-motor feedback snapshot
    -> latest AHRS/remoter snapshot
    -> Function
    -> Odometry
    -> lpendulum/rpendulum solve
    -> health/current-cycle state
    -> ChassisController state switch
    -> link_force + wheel torque
    -> VMC
    -> power/safety insertion point and limits
    -> six command buffers
    -> one lkmotorhandler::send_control()
```

Control file map:

```text
control/include/vmc.hpp + control/src/vmc.cpp
    VMCsolver pure five-bar/VMC math
control/include/pendulum.hpp + control/src/pendulum.cpp
    Pendulum motor binding, signs/zeros, PID/slope, and command-buffer writes
control/include/odometry.hpp + control/src/odometry.cpp
    Odometry state estimation
control/include/lqr.hpp + control/src/lqr.cpp
    LQR evaluation
control/include/lqr_coeffs.hpp
    the only 40x6 LQR coefficient source
control/include/function.hpp + control/src/function.cpp
    Function remote-command logic
control/include/chassis.hpp + control/src/chassis.cpp
    chassis_state/jump_stage and the single visible state switch
control/src/control_task.cpp
    input snapshot, cycle ordering, safety gate, motor buffers, and single send
```

## Source behavior used from `wbr_2026`

The active old configuration is `CHASSIS_ONLY + SJTU_MODEL`; `STAIRUP` is disabled.

- `TaskFunctions.cpp` supplies Relax/Normal/Spin remote behavior, velocity/yaw slopes, manual leg
  length, longitudinal position hold, and the old jump command sequence.
- `TaskPendulum.cpp` supplies the 10-state observation/reference layout, length and roll PID,
  normal/off-ground LQR selection, jump force overrides, and four actuator-domain outputs.
- `TaskSolver.cpp` supplies six-motor ownership, wheel velocity, odometry ordering, left/right motor
  signs, VMC, torque limits, relax/reset policy, and one final send.
- `User/Task/Inc/lqr.hpp` supplies the exact 40x6 SJTU_MODEL coefficient table.
- `vmc.hpp`, `odometry.hpp`, and `config_chassis.hpp` supply the old formulas and tuned values.
- There is no `pendulum.hpp` and no `Pendulum<hiptype, wheeltype>` class in the inspected old
  `User` tree. Pendulum control is implemented directly in `TaskPendulum.cpp`.

The old Function/Pendulum/Solver thread boundaries and latest-value topic timing were not reproduced.
This was already the current repository structure when the naming-convergence work began. That work
did not merge, split, or otherwise redesign the existing `wbr_input`/`wbr_control` threads.

## Completed control objects

- `Function`
  - nested left/right switch interpretation
  - one-update control-switch transition behavior
  - one-shot jump prepare/start events
  - `dt`-scaled velocity/yaw slopes and manual leg-length rate, plus position hold
- `Odometry`
  - fixed-size `[x, v, a]` Kalman filter
  - quaternion body-to-world acceleration rotation
  - DWT-derived explicit `dt`
- `VMCsolver`
  - five-bar forward solve, Jacobian, velocity mapping, VMC both ways
  - spring and support-force estimate
  - discriminant, singularity, `dt`, and finite guards
- `Pendulum`
  - non-template binding to two existing `motors::api&` objects
  - named motor-feedback snapshot input
  - zero/direction conversion
  - length/phi PID and phi slope
  - VMC torque resolution and motor-buffer write
- `LQR`
  - exact single-copy 40x6 old coefficient source in `control/include/lqr_coeffs.hpp`
  - normal and old sparse off-ground gain behavior
- `ChassisController`
  - one enum/switch class; no state classes or factories
  - state time in seconds
  - Relax/Normal/Spin/Offground and Jump control paths
  - safe-zero placeholders for Recover/Flatten/Neutral/Gostair
- `control_entry`
  - same-cycle composition, freshness checks, coherent feedback copy, safety gate, and one commit

No dynamic memory, new controller interface hierarchy, factory, direct CAN call, or motor-manager
call was added to the math/control objects. `Pendulum` reuses the existing `motors::api`; it does
not introduce another runtime-polymorphic hierarchy.

## Actual FSM transitions

```text
invalid / stale / motor offline / Relax command
    -> RELAX

RELAX
    -> NORMAL
    -> SPIN

NORMAL <-> SPIN
    -> OFFGROUND when support force is below the configured threshold

OFFGROUND
    -> NORMAL or SPIN after contact is restored

NORMAL
    -> JUMP on the one-shot prepare event

JUMP::dont
    -> JUMP::extending on the one-shot start event
    -> NORMAL/SPIN when preparation is cancelled

JUMP::extending
    -> JUMP::inair on mean leg length
    -> RELAX on timeout/invalid input

JUMP::inair
    -> JUMP::landing on mean leg length
    -> RELAX on timeout/invalid input

JUMP::landing
    -> NORMAL on support force
    -> RELAX on timeout/invalid input
```

`RECOVER`, `FLATTEN`, `NEUTRAL`, and `GOSTAIR` exist in the enum and switch, but have no reachable
entry transition. If invoked after future changes, their current invalid output falls back to
RELAX. This is deliberate: the required real-robot targets and thresholds are not validated.

## State control status

| State | Current implementation |
| --- | --- |
| Relax | No LQR/VMC request; resets leg PID/slope/solver and odometry; six motors relax |
| Recover | Frozen safe placeholder; no normal LQR; target phi/IMU guards unresolved |
| Flatten | Frozen safe placeholder; no normal LQR; direction/target/wheel assist unresolved |
| Neutral | Frozen safe placeholder; length/phi control targets and exits unresolved |
| Normal | Verified 10-state LQR, left/right length PID, roll PID, wheel + hip request |
| Spin | Same verified LQR table with spin yaw reference, length PID, roll PID |
| Offground | Sparse hip LQR + length PID; wheel torque forced to zero; odometry reset request |
| Jump | `dont/extending/inair/landing`; old 400 N/-200 N semantics, sensed-offground override, wheel zero in sensed-offground and inair, 1.5 s new safety timeout |
| Gostair | Frozen safe placeholder; old `STAIRUP` was disabled and current TOF chain is absent |

Sensed `OFFGROUND` and commanded `JUMP::inair` are separate variables. During any Jump stage, a
sensed off-ground condition forces wheel torque to zero and uses the off-ground leg/hip branch.

## Central parameter source

All current control-layer tuning enters through:

```text
control/include/leg_config.hpp
    -> inline constexpr chassis_config k_default_chassis
```

It owns:

- five-bar geometry/bounds, wheel radius/mass, gravity, spring, alpha equilibrium
- hip/wheel torque limits
- raw encoder offsets, leg directions, wheel directions
- command scales/slopes/reference limits
- odometry covariance/noise
- LQR length range/resolution
- leg and roll PID values
- off-ground, landing, jump force/length/time guards
- singularity/flat/neutral thresholds
- AHRS/command freshness, `dt`, alive-check period, thread priorities, and actuation gate

The 40x6 generated/tuned LQR table remains only in `control/include/lqr_coeffs.hpp`. Structural
constants such as vector dimensions, matrix indices, zero, one, and pi remain in algorithm code.

Device-owned values are intentionally not duplicated into `chassis_config`: LK torque constants
and gear behavior stay in the motor driver; AHRS installation offset stays in the AHRS default
config; motor CAN IDs stay in generated `robot_config.hpp`.

## Safety properties

- Negative closure discriminant is rejected before `sqrt()`.
- Jacobian/spring denominators and final state/VMC values are checked.
- Link acceleration differences use `(current - previous) / dt`; initial history is guarded.
- AHRS/command freshness, motor online/error state, odometry, both links, and all request values
  must be valid.
- Invalid/stale/offline data produces RELAX and all six command buffers are relaxed.
- Offground and Jump inair wheel requests are explicitly zero.
- Jump has a seconds-based abnormal timeout and invalid/Relax clears its stage.
- FSM never accesses CAN, a motor manager, or commit.
- Power limiting has one explicit insertion point after VMC and before motor buffers.
- The actuation gate remains false.

## Unresolved blockers

Do not invent or silently choose these values.

1. **Five-bar geometry conflict**
   - Current migrated solver uses `l1=0.220 m`, `l2=0.260 m`, coincident mathematical pivots, and
     the `+sqrt(discriminant)` assembly branch.
   - `wbr_2026` uses `VMC_L1=0.150 m`, `VMC_L2=0.270 m`, and a nonzero motor-pivot distance.
   - Confirm measured geometry and assembly branch before enabling output.

2. **LK8016 units and gear ratio**
   - Current driver position does not apply the old `1/6` factor; velocity does.
   - Confirm whether position, velocity, and torque are motor-shaft or output-shaft values.

3. **Torque feedback meaning**
   - It is currently estimated from raw current and driver torque constant.
   - Confirm gearbox scaling and whether it is suitable for reverse-VMC/support-force estimation.

4. **Mechanical zero ownership**
   - Old raw encoder offsets are installed in the LK driver.
   - Leg radian zero fields remain zero.
   - Confirm that zero is applied exactly once and whether persistent calibration belongs in the
     driver or control layer.

5. **Left/right signs**
   - Old behavior implies left hip `-1/-1`, right hip `+1/+1`, left wheel `+1`, right wheel `-1`.
   - Confirm position, velocity, feedback torque, and command signs with a low-torque test.

6. **Spring/support-force model**
   - Validate spring constants/geometry, AHRS acceleration axes and gravity content, wheel-side
     mass, support-force sign, and the 20 N off-ground threshold.

7. **Recover/Flatten/Neutral**
   - Required behavior is known, but entry/exit thresholds, target phi, slope/PD gains, kick torque,
     confirmation times, flatten wheel assist, and timeout values are not.
   - These states remain frozen rather than borrowing unverified MuJoCo tuning.

8. **GOSTAIR/TOF**
   - The verified old build had `STAIRUP` disabled.
   - No current TOF latest-snapshot chain or validated state logic exists.

9. **Jump safety timeout**
   - The 1.5 s timeout is a new conservative software guard, not a `wbr_2026` tuning value.
   - Validate it against captured jump timing before enabling output.

10. **Feedback timestamps**
    - The six-value copy is coherent against ISR writes, but individual CAN feedback has no sample
      timestamp. `alive_check()` detects activity only at the configured 10 ms interval.
    - A true per-motor timeout/sample-age check still requires lower-layer timestamp support.

11. **Power, acceleration, and torque-rate limits**
    - The post-VMC insertion point exists; no unverified allocator, acceleration clamp, or torque
      slew limiter is implemented.

12. **LQR applicability**
    - Coefficients and state ordering match the old SJTU model.
    - Confirm current geometry, mass, axes, and actuator signs before treating the output as valid.

13. **Inverse kinematics**
    - Target leg pose to joint angles remains intentionally unimplemented.

14. **Actuation gate**
    - Keep `runtime.actuation_enabled=false` until geometry, units, zero positions, signs, and
      captured-data comparisons are reviewed together.

## Verification

Latest completed checks:

```text
cmake --preset Debug
cmake --build --preset Debug --parallel
cmake --preset Release
cmake --build --preset Release --parallel
```

- Debug and Release both link successfully.
- Debug uses 81,800 B DTCM and 206,012 B flash.
- Release uses 81,744 B DTCM and 104,884 B flash.
- The old and migrated 40x6 LQR tables compare exactly across all 240 coefficients.
- Obsolete `link_solver`, `leg_controller`, `lqr_solver`, `command_fusion`, and `chassis_fsm`
  names have no remaining control-layer matches.
- No dynamic allocation was added under `control/`.
- No staged internal-control topic chain remains.
- Only the input adapter contains `TX_WAIT_FOREVER`; Control uses a no-wait snapshot copy.
- Only Control accesses the two leg objects.
- Only one application-level LK `send_control()` exists.
- All modified C/C++ control files were passed explicitly to clang-format; no whole-tree format was
  run and `AGENTS.md` was not formatted.
