# Parallel WBR Agent Notes

Last updated: 2026-07-27

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
- Keep `pnx_devices` and other `pnx_` modules on their native interfaces. Do not rename or modify
  them as part of Control cleanup.

## Current application architecture

- `demo/` is excluded from CMake; `control/` is included recursively.
- `control/app.cpp::app_start()` calls `wbr::control::start_control_task()`.
- AHRS and remoter are initialized with their module-default, no-argument `init()` calls. Their
  libraries were not modified.
- There is one high-frequency business-control thread, `wbr_control`, at priority 5.
- There is no application-level input adapter thread or shared input mutex. `wbr_control` directly
  reads the AHRS/remoter subscribers each cycle into local latest-value objects.
- These direct `msg::read()` calls use the message-bus topic mutex semantics; the separate adapter
  no longer isolates the control cycle from a blocked topic mutex.
- The control path consumes module-owned `ahrs::message`, `remoter::state`, and
  `motors::feedback` directly. Control does not maintain duplicate freshness, usable, or per-field
  input-validation state.
- The Control cycle directly calls `Function`, `Odometry`, both `leg_controller` objects,
  `ChassisController`, limits, six motor-buffer writes, and one final LK batch commit.
  Each `leg_controller` privately owns its `link_solver`; intermediate control products do not use
  the message bus.
- All six module-owned `motors::feedback` values are copied through the unchanged native
  `get_feedback()` interface into one `motor_fdb_frame` before the cycle uses them. Control
  adds no capture wrapper, interrupt-disabled section, per-motor usable flag, or duplicate
  feedback validation. `lkmotorhandler::alive_check()` runs once per control cycle to maintain the
  motor-owned status; Control does not duplicate its aggregate result.
- Four LK8016 hips and two LK9025 wheels are constructed from generated device-tree configs and
  registered with `lkmotorhandler`.
- Each cycle calls `send_control()` once at the end of `control_entry()`.
- Debug builds expose the single POD `wbr::control::control_debug_data` IDE-Watch snapshot. Its
  definition is in `control_task.cpp`; it owns no thread, lock, pointer, or control behavior.
- `k_default_control.chassis.runtime.actuation_enabled` is still `false`. Every cycle therefore
  computes requests but sends relaxed/zero-current commands.

Current cycle:

```text
six-motor feedback snapshot
    -> direct AHRS/remoter subscriber reads
    -> Function
    -> Odometry
    -> left_leg/right_leg solve
    -> current-cycle validity
    -> ChassisController state switch
    -> left/right virtual_force targets + wheel torque
    -> leg_controller/link_solver VMC
    -> safety insertion point and torque limits
    -> six command buffers
    -> one lkmotorhandler::send_control()
```

Control file map:

```text
control/include/leg.hpp + control/src/leg.cpp
    leg_controller motor binding, directions, length PID, VMC output, and command-buffer writes
control/include/control_config.hpp
    leg_config, chassis_config, control_config, and the single k_default_control object
control/include/vmc.hpp
    header-only link_solver five-bar kinematics, Jacobian, support-force estimate, and VMC math
control/include/odometry.hpp
    header-only Odometry configuration and state estimation
control/include/lqr.hpp
    lqr_state/lqr_output types and header-only LQR evaluation
control/include/lqr_coeffs.hpp
    the only 40x6 LQR coefficient source
control/include/function.hpp + control/src/function.cpp
    Function remote-command logic
control/include/chassis.hpp + control/src/chassis.cpp
    chassis_state/jump_stage and the single visible state switch
control/include/control_debug.hpp
    DEBUG-only cycle snapshot type; its sole global POD definition is in control_task.cpp
control/src/control_task.cpp
    direct input reads, cycle ordering, safety gate, motor buffers, and single send
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
The current application has one `wbr_control` thread; the AHRS and remoter keep their module-owned
threads.

## Completed control objects

- `Function`
  - nested left/right switch interpretation
  - one-update control-switch transition behavior
  - right-switch jump request mapping
  - remote axes produce velocity/yaw-rate commands
  - moving x reference uses one-step feedback prediction `odom.x + cmd.v * dt`; zero velocity
    captures and holds the current odometry position
  - active yaw reference uses one-step continuous-angle prediction
    `total_yaw + cmd.yaw_rate * dt`; zero yaw rate captures and holds `total_yaw`
- `Odometry`
  - fixed-size `[x, v, a]` Kalman filter
  - quaternion body-to-world acceleration rotation
  - fixed configured `dt`
  - legacy `Q(0,0) = dt^3 * q / 20` and gravity-preserving world `a_z`
- `link_solver`
  - five-bar forward solve, Jacobian velocity mapping, feedback-force reconstruction, and VMC
    output mapping
  - spring and support-force estimate
  - discriminant, Jacobian/leg-length singularity, and spring-denominator guards
  - VMC forward/reverse operations return `bool`; `joint_torque` carries no duplicate validity flag
  - the only single-leg mathematical solver; it owns no motor or controller state and is private
    to `leg_controller`
- `leg_controller`
  - non-template binding to two existing `motors::api&` objects
  - direct `motors::feedback` snapshot input
  - no duplicate feedback-valid boolean or motor feedback validation
  - zero/direction conversion
  - one active old-real-robot length PID
  - VMC torque resolution and motor-buffer write
  - no unused equivalent-leg-angle controller
- `LQR`
  - exact single-copy 40x6 old coefficient source in `control/include/lqr_coeffs.hpp`
  - normal and old sparse off-ground gain behavior
  - typed `lqr_state` input and four direct torque outputs
  - active Normal control calls `LQR::solve()` with the verified 10-state ordering
- `ChassisController`
  - one enum/switch class; no state classes or factories
  - `step(chassis_context)` with explicit Relax/Normal/Spin/Offground/Jump methods
  - Normal applies LQR wheel/leg torque, roll-compensated length PD, old `+0.03 m` length preload,
    optional spring-force subtraction, and the verified 20 N off-ground transition
  - Normal and Spin map the Function-owned continuous yaw/yaw-rate command directly; the observed
    LQR yaw state uses `ahrs::message::total_yaw`
  - Spin/Offground/Jump currently keep safe-relax placeholder output
- `control_entry`
  - same-cycle composition, direct input reads, fixed configured `dt`, actuator gate, and one
    commit

No dynamic memory, new controller interface hierarchy, factory, direct CAN call, or motor-manager
call was added to the math/control objects. `leg_controller` reuses the existing `motors::api`; it
does not introduce another runtime-polymorphic hierarchy.

`control/include/leg_math.hpp` now contains only the Control-specific `slope`. Control reuses
`pnx_libs/math` for `math::pi`, `math::clamp`, `math::clamp_loop`, and `math::limit_abs`.

## Actual chassis transitions

```text
invalid module result / Relax command
    -> RELAX

RELAX
    -> NORMAL for a valid Normal or Jump command
    -> SPIN for a valid Spin command

NORMAL
    -> SPIN for a Spin command
    -> OFFGROUND when combined support force is below 20 N
```

`RECOVER`, `FLATTEN`, `NEUTRAL`, and `GOSTAIR` are not declared states. They were removed because
the active `wbr_2026` branch does not provide validated behavior for them.

## State control status

| State | Current implementation |
| --- | --- |
| Relax | Resets both legs, requests odometry/position-hold reset, keeps safe relaxed output, and routes valid Normal/Jump/Spin commands |
| Normal | Active 10-state LQR, roll-compensated length PD, spring-force subtraction, and wheel/leg outputs |
| Spin | Safe-relax placeholder; observed/reference scaffolding only |
| Offground | Safe-relax placeholder; observed/reference scaffolding only |
| Jump | Safe-relax placeholder |

## Central parameter source

All current control-layer tuning enters through:

```text
control/include/control_config.hpp
    -> inline const control_config k_default_control
        -> leg_config leg
        -> chassis_config chassis
```

It owns:

- leg_config: legacy five-bar geometry, explicit spring enable/model, solver numerics, length PID,
  and hip limit
- chassis_config: wheel radius/mass, gravity, roll PID, wheel limit, directions, LQR fit range,
  command mapping, Normal-state preload/off-ground threshold, and runtime
- hip/wheel torque limits
- leg directions and wheel directions
- command scales/slopes/reference limits
- LQR length range/resolution
- leg and roll PID values
- singularity thresholds
- fixed `dt`, control-thread priority, and actuation gate

The 40x6 generated/tuned LQR table remains only in `control/include/lqr_coeffs.hpp`. Structural
constants such as vector dimensions, matrix indices, zero, one, and pi remain in algorithm code.

`link_solver` and `leg_controller` retain only `const leg_config&`. `ChassisController` retains
`const chassis_config&` so state logic can read chassis-owned targets such as `cmd.min_len`; it
also copies the LQR configuration and constructs its roll PID.
Wheel-side mass and gravity remain chassis-owned and are passed as scalar cycle inputs to the leg
solve path for the existing support-force calculation. `runtime_config::dt` is fixed at `0.001 s`
and is passed through Function, Odometry, leg_controller, and link_solver without runtime range
validation.

ThreadX is configured for 1000 ticks/s. The control thread uses the configured `0.001 s` step and
sleeps one tick after each cycle. Control does not use DWT for cycle timing and has no
application-level release deadline or period-conversion state.

Device-owned values are intentionally not duplicated into either control config: LK torque
constants and gear behavior stay in the motor driver; AHRS installation offset stays in the AHRS
default config; motor CAN IDs stay in generated `robot_config.hpp`.

## Safety properties

- Negative closure discriminant is rejected before `sqrt()`.
- Jacobian, leg-length, spring, and innovation-matrix denominators are checked only where the
  corresponding division or square root requires them.
- The restored legacy geometry uses `l1=0.150 m`, `l2=0.270 m`, and a `0.150 m` motor-pivot
  distance. Software spring compensation is explicitly disabled by default.
- The spring model's `phi2` implicit derivatives use `sin(phi2 - phi3)`; the ordinary VMC
  Jacobian continues to use its original `sin(phi3 - phi2)` form.
- Link acceleration differences use `(current - previous) / dt`; initial history is guarded.
- Remoter offline behavior is handled by `Function`; Odometry failure, invalid links, or an invalid
  command produces RELAX through the current-cycle `control_ok` value.
- The actuator writer directly clamps both wheel requests to `chassis_config::max_wheel_tau`;
  the native motor layer owns torque-to-current conversion.
- Unimplemented motion states return safe-relax output.
- `ChassisController` never accesses CAN, a motor manager, or commit.
- Power limiting has one explicit insertion point after VMC and before motor buffers.
- The actuation gate remains false.

## Unresolved blockers

Do not invent or silently choose these values.

1. **Five-bar geometry validation**
   - The solver now matches `wbr_2026`: `l1=0.150 m`, `l2=0.270 m`, `motor_distance=0.150 m`, and
     the `+sqrt(discriminant)` assembly branch.
   - Confirm these values against measured hardware before enabling output.

2. **LK8016 units and gear ratio**
   - Current driver position does not apply the old `1/6` factor; velocity does.
   - Confirm whether position, velocity, and torque are motor-shaft or output-shaft values.

3. **Torque feedback meaning**
   - It is currently estimated from raw current and driver torque constant.
   - Confirm gearbox scaling and whether it is suitable for reverse-VMC/support-force estimation.

4. **Mechanical zero ownership**
   - Control no longer assigns LK raw encoder offsets or subtracts joint-radian zero values.
   - The unchanged LK driver still owns its generic `offset` field, which remains at its default
     zero value in this application.
   - Confirm the motor-side mechanical-zero procedure before enabling output.

5. **Left/right signs**
   - Old behavior implies left hip `-1/-1`, right hip `+1/+1`, left wheel `+1`, right wheel `-1`.
   - Confirm position, velocity, feedback torque, and command signs with a low-torque test.

6. **Spring/support-force model**
   - Software spring compensation defaults off to match active `wbr_2026`.
   - The `phi2` implicit-derivative sign has been corrected and verified by central differences.
     Before enabling compensation, validate spring constants/geometry. Independently validate AHRS
     acceleration axes and gravity content, wheel-side mass, and support-force sign.

7. **Recover/Flatten/Neutral**
   - Required behavior is known, but entry/exit thresholds, target phi, slope/PD gains, kick torque,
     confirmation times, flatten wheel assist, and timeout values are not.
   - These states are absent rather than declared as frozen placeholders or populated with
     unverified MuJoCo tuning.

8. **GOSTAIR/TOF**
   - The verified old build had `STAIRUP` disabled.
   - No current TOF latest-snapshot chain or validated state logic exists; `GOSTAIR` is not a
     declared state.

9. **Feedback timestamps**
    - The six values are copied without disabling interrupts, and individual CAN feedback has no
      sample timestamp. `alive_check()` updates motor-owned status once per control cycle.
    - A true per-motor timeout/sample-age check still requires lower-layer timestamp support.

10. **Power, acceleration, and torque-rate limits**
    - The post-VMC insertion point exists; no unverified allocator, acceleration clamp, or torque
      slew limiter is implemented.

11. **LQR applicability**
    - Coefficients and state ordering match the old SJTU model.
    - Confirm current geometry, mass, axes, and actuator signs before treating the output as valid.

12. **Inverse kinematics**
    - Target leg pose to joint angles remains intentionally unimplemented.

13. **Actuation gate**
    - Keep `runtime.actuation_enabled=false` until geometry, units, zero positions, signs, and
      captured-data comparisons are reviewed together.

## Verification

Latest completed checks after moving continuous-yaw command generation into Function:

```text
cmake --build --preset Debug --target pnx_embedded --clean-first --parallel 4
cmake --build --preset Release --target pnx_embedded --parallel 4
```

- Debug and Release both link successfully.
- Debug uses 77,056 B DTCM and 197,512 B flash.
- Release uses 77,000 B DTCM and 99,944 B flash.
- The old and migrated 40x6 LQR tables compare exactly across all 240 coefficients.
- `chassis_state` declarations and switch cases are synchronized. The only declared states are
  Relax, Normal, Spin, Offground, and Jump.
- `command_action`, `command_event`, `fsm_input`, `fsm_output`, `function_feedback`,
  `health_state`, and `power_state` have no remaining control-layer matches.
- VMC forward/reverse transformations use direct `bool + output` operations. Velocity mapping is
  internal to `link_solver::solve()`; `leg_controller` exposes only the torque-resolution operation
  needed by upper-layer control.
- Duplicate `joint_state.valid`, `joint_torque.valid`, `motor_fdb_frame.valid`,
  `odometry_state.valid`, `reachable`, and `near_singularity` states were removed.
- No dynamic allocation was added under `control/`.
- No staged internal-control topic chain remains.
- The former C-linkage task telemetry remains removed. Debug builds expose only
  `wbr::control::control_debug_data`, populated directly by the existing control thread.
- Control-layer motor sign configuration uses `leg_dir`, `motor_dir_config`, and
  `chassis_config::motor_dir`; no legacy calibration naming remains under `control/`.
- Single-leg targets use only `virtual_force`; motor input uses only module-owned
  `motors::feedback`. The duplicate `Pendulum` files/class and old `VMCsolver` class are removed;
  header-only `vmc.hpp` now contains the single `link_solver` mathematical layer.
- Odometry-specific tuning and its header-only implementation live in `odometry.hpp`;
  `chassis_config` no longer owns a duplicate odometry configuration block.
- Three finite joint-angle comparisons against the legacy geometry matched `u2/u3/phi/len`,
  `J`, `JT`, and `JT_inv` to at most `2.22e-16` in a double-precision script.
- At `phi1=1.3`, `phi4=0.3`, and `delta=1e-4 rad`, central differences for the spring model's
  `dphi2/dphi1` and `dphi2/dphi4` matched the `sin(phi2 - phi3)` analytic expressions within
  `1.1e-9`.
- There is no `wbr_input` thread, input stack, shared input snapshot, or input mutex. `wbr_control`
  directly reads the module-owned AHRS/remoter topics.
- Only Control accesses the two leg objects.
- A normal cycle contains one LK `send_control()`.
- `git diff --check` passes. No `clang-format` executable is available in the current environment,
  so formatting was kept consistent manually and no whole-tree formatter was run.
