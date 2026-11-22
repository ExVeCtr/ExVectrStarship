# ExVectrControl

> Guidance and control for ExVectr vehicles: cascaded position → acceleration → attitude → TVC control, Starship bellyflop flap control, a simple PID, and a rigid-body physics integrator. Repo: https://github.com/ExVeCtr/ExVectrControl.git. `Last verified against commit: 084bea1 (pinned now: 6e07d38 = 084bea1 plus a mechanical NOW() -> NowNs() port for ExVectrCore 18bbb09 (8d6efab) and resetControlState() (6e07d38, 2026-11-20, branch feature/bellyflop-abort-gnss); content otherwise as verified; feature/sim-aero c8d1ffb, 2026-11-21: PhysicsBodySim integrator rewritten, re-verified for that file; feature/descent-s1 1902fa4, 2026-11-22: descent guidance, flap descent law, `Descent` flap mode, `FlapModeMapper`, re-verified for `ControlAttitudeFlaps` and `starship/`)`

## Purpose & scope

- Converts state estimates (`[v,p]`, `[ω,q]` topics from ExVectrDSP or `BodySimulator`) and setpoints into actuator commands:
  - a TVC body-frame force plus roll torque (`Vector<4>`);
  - four flap angles (`ControlAttitudeFlapSetting`).
- Descent (belly-flop) control for the freefall: `descent::DescentGuidance` and `descent::FlapDescentLaw` plus the `Descent` mode of `ControlAttitudeFlaps` (since 1902fa4, **not wired to the vehicle yet**).
- Control laws are cascaded P/PI with hand-tuned gains. There is **no LQR, MPC or trajectory optimisation**. Guidance is minimal: it comes from mission setpoints in `src/mission/*` and `FlapModeObserverTask` in `src/main.cpp:1258`.
- `PhysicsBodySim` is the integrator used by `DSP::BodySimModel` (inside `DSP::BodySimulator`).
- Commit `084bea1` removed the old monolithic `control_rocket.*`. The parent repo pins `9ab8379`, one commit behind.
- The README is stale ("Implement PID control" TODO).

## Dependencies (ExVectr libs + external)

- **ExVectrCore**: `Task_Periodic`, `Topic`, `Simple_Subscriber`, `Callback_Subscriber`, `Topic_Publisher`, `Timestamped`, `NowNs()`, `LOG_MSG`.
- **ExVectrMath**: quaternions and vectors. Conventions are in `docs/libs/ExVectrMath.md`.
- **ExVectrDSP**: only `value_covariance.hpp` is included. Note the reverse dependency: `DSP/body_simulator.hpp` includes `physics_bodysim.hpp` and `starship/control_attitude_flaps.hpp`, which makes the pair circular.
- External: `<math.h>`.

## Key concepts & API

- All controllers are `Core::Task_Periodic` subclasses that add themselves to `Core::getSystemScheduler()` in the constructor, at **20 ms**. All three are named `"Control Rocket"`.
- `taskInit()` calls `setInitialised(false)` until the required inputs have produced data, which makes the scheduler retry init.
- Frames follow the ExVectrMath conventions: q = body→world, world = X-North/Y-West/Z-Up, body +Z = nose, TVC at body z = −0.35 m.

### `ControlPositionStandard` (`control_position_standard.hpp/.cpp`)
- **Inputs:**
  - `subscribePositionMeasurement(Topic<Timestamped<Vector<6>>>)` with layout `[vN,vW,vU,pN,pW,pU]`.
  - `subscribeSetpoint(Topic<Vector<6>>)` with layout `[v_ff(3), p_sp(3)]`. If no new setpoint arrives, `p_sp += v_ff·dt`.
- **Output:** `getAccelTopic()` publishes `Topic<Vector<3>>`, the wanted world-frame acceleration in m/s², **including +g on z**.
- **Law:**
  - `v_cmd = Kp·(p_sp − p)`. The horizontal command is clamped by vector norm. The vertical command has an upper clamp only.
  - `e_v = v_cmd + v_ff − v`.
  - `a = Kv·e_v + I + [0,0,g]`.
  - The integral `I += e_v·Ki·dt` is clamped so that `Kv·e_v + I` stays within the accel limits (conditional anti-windup; no sign reversal), then clamped to ±3. `I` is reset while `enableControl(false)`.
  - `resetControlState()` (since 6e07d38): `I = 0`, the delta-time reference is set to now, and the setpoint state is set to "hold the latest measured position with zero velocity" until the next setpoint arrives (a setpoint already received and not yet read is still used). Gains and the enable flag are unchanged. The flight software calls it on every enable of the TVC controller set and on a mission switch while enabled (missions.md 6.2).
  - **The output itself is not clamped.**
- **Tuning (defaults; setters exist for the position and velocity gains and position limits only):**

  | Parameter | Value |
  |---|---|
  | `positionHZTGain_` | 0.5 1/s |
  | `positionVRTGain_` | 2 1/s |
  | `positionHZTLimit_ms_` | 3 m/s |
  | `positionVRTLimit_ms_` | 15 m/s (up only) |
  | `velocityHZTGain_` | 2.2 1/s |
  | `velocityVRTGain_` | 2.5 1/s |
  | `velocityHZTIntegral_` | 0.2 |
  | `velocityVRTIntegral_` | 0.8 |
  | `velocityIntegralLimit_ms_` | 3 |
  | `velocityHZTLimit_mss_` | 2 m/s² |
  | `velocityVRTLimit_mss_` | 8 m/s² |

### `ControlMappingAccToAtt` (`control_mapping_acctoatt.hpp/.cpp`)
- Not a task. A `Callback_Subscriber` on `subscribeAccelSetpoint(Topic<Vector<3>>)`.
- On every accel sample it publishes on `getAttitudeTopic()` (`Topic<Vector<7>>`) the setpoint `[0,0,0, q]`, where `q = Quat(normalize(e_z × a), min(∠(e_z,a), 35°))`.
- The result is the minimal rotation tilting body +Z toward `a`, with zero yaw intent. Tilt limit `velocityLimit_Rad_ = 35°` (no setter). An accel along +Z gives the identity.

### `ControlAttitudeTvc` (`control_attitude_tvc.hpp/.cpp`)
- **Constructor:** `(mass_kg=1, thrustLimit_N=20, tvcAngleLimit_Rad=15°)`. Main passes `(1.1, 14, 45°/15 = 3°)`.
- **Inputs:**
  - `subscribeAttitudeMeasurement(Topic<Timestamped<Vector<7>>>)` with `[ω_body, q]`.
  - `subscribeAttitudeSetpoint(Topic<Vector<7>>)` with `[ω_sp, q_sp]`. `unsubscribeAttitudeSetpoint()` and `setAttitudeStateSetpoint(v7)` exist for direct override, used by `MissionBellyflop`.
  - `subscribeAccelerationSetpoint(Topic<Vector<3>>)`: world accel including g.
- **Output:** `getTvcTopic()` publishes `Topic<Vector<4>>` = `[Fx,Fy,Fz (N, body), T (roll torque)]`. When disabled it publishes `[0,0,0.1,0]`.
- `resetControlState()` (since 6e07d38): `I = 0` and the delta-time reference set to now. Setpoints, gains and the enable flag are unchanged. (`I` is also zeroed every run while disabled.)
- **Law:**
  1. `q_e = q* ⊗ q_sp` (body frame, w ≥ 0).
  2. `z_d = q_e.rotate(e_z)`. Tilt error vector `= normalize(e_z × z_d)·∠(e_z, z_d)`.
  3. If tilt < 10°: `err.z = atan2(q_e.z, q_e.w)`, which is half the yaw angle. Otherwise `err.z = 0`.
  4. `τ = Kp∘err + Kd∘(ω_sp − ω) + I`, where `I += Ki∘err·dt` is clamped.
  5. Lateral force `F_xy = τ × (0,0,1/L)` with `L = tvcCGOffset_m_ = −0.35`. This gives `Fx = τy/L`, `Fy = −τx/L`.
  6. `Fz = |m·Rᵀa_sp|·cos∠(Rᵀa_sp, e_z)`.
  7. If `∠(F, e_z) > limit`, F is replaced by the unit vector at the limit angle. It is scaled by `|F_xy|/sin(limit)` only when `setTVCLimitCompensation(true)` (main enables this).
- **Setpoint propagation:** if no new setpoint arrived this tick, `q_sp ← q_sp ⊗ Δq(ω_measured·dt)`.
- **Tuning (defaults; setters for all gains):**

  | Parameter | Value |
  |---|---|
  | `attitudeGain_` (x,y) | 1.2 N·m/rad |
  | `attitudeZGain_` | 0.4 |
  | `attitudeRateGain_` | 0.5 N·m·s/rad |
  | `attitudeRateZGain_` | 0.08 |
  | `attitudeIntegralGain_` | 0 |
  | `attitudeIntegralZGain_` | 0 |
  | `integralLimit_` | 0.5 |
  | `integralZLimit_` | 0 |
  | `tvcCGOffset_m_` | −0.35 m |
  | `tiltLimit_Rad_` | 35° (unused) |
  | `compensateTVCAngle_` | false (main: true) |

  The gains are torques: there is no inertia scaling.

### `ControlAttitudeFlaps` (`starship/control_attitude_flaps.hpp/.cpp`)
- **Inputs:**
  - `subscribeAttitudeMeasurement` with `[ω, q]`.
  - `subscribePositionMeasurement` with `[v, p]`. Currently read but unused by the law.
  - `subscribeControlSetting(Topic<ControlAttitudeBellyFlopSetting>)` = `{azimuthAngle_Rad, pitchAngle_Rad, rollAngle_Rad (unused), extentFlapsOnAscent, pitchSplit_Rad (Descent only), bellyFlopMode ∈ {Acsent, Stabilize, Upright, Descent}}`. The settings structs live in `starship/flap_types.hpp` (included by `control_attitude_flaps.hpp`); `Descent` is appended (value 3, the others keep 0..2).
- **Output:** `subscribeFlapSettingOutputTopic(Topic<ControlAttitudeFlapSetting>)`, a `Topic_Publisher`, publishes `{flapTL, flapTR, flapBL, flapBR (rad), enableFlaps (always true)}`. Convention: **0 rad = extended, 90° = retracted**. The simulator clamps to 10–90°; the controller does not clamp.
- **Mode mapping** is the pure, header-only `FlapModeMapper::map(setting, [ω,q], FlapStabilizeParams, FlapDescentLawParams, FlapDescentLawMem&)` (`starship/flap_mode_mapper.hpp`, host-tested). `taskThread()` only reads the topics, calls it and publishes. The Acsent, Stabilize and Upright code moved into it unchanged (same expressions and order); `test/test_descent_unit` proves bit-identical outputs against a verbatim copy of the old code. The Descent memory (turn latch) is reset on every call in another mode.
- **Modes:**
  - **Acsent:** all flaps at 90°, or 0° if `extentFlapsOnAscent`.
  - **Upright:** top 0°, bottom 90°.
  - **Stabilize:**
    - Body Z expressed in world gives `azimuth = atan2(zW, zN)` (counter-clockwise from North toward West) and `pitch = atan2(zU, |z_horiz|)`.
    - `u = [clamp(Kaz·wrap(az_sp − az), ±0.5), Kp·(pitch_sp − pitch), 0] − Kω∘ω_body`.
    - Mixer, with s = fade factor:
      - TL = 50° + (−u0 − u1 + u2)·s
      - TR = 50° + (u0 − u1 − u2)·s
      - BL = 40° + (u0 + u1 + u2)·s
      - BR = 40° + (−u0 + u1 − u2)·s
    - Fade factor `s` = 1 while `∠(body X, world up) ≤ 35°`, falling linearly to 0 at 55°.
  - **Descent** (new, unused until the flap task selects it): `descent::FlapDescentLaw` with `azimuthAngle_Rad` as the heading setpoint and `pitchSplit_Rad` as the glide split (+ nose-up/slower, − forward glide, clamped −80°..+12°). Neutral **30/78°** (top/bottom). Yaw is the bottom-pair differential (`kHeading` 1.5 1/s → yaw-rate command ≤ 75°/s → `kYawRate` 0.40), roll is held at 0 with the top-pair differential (`kRoll` 2.0, `kRollRate` 0.25), a 2×2 decoupler (`dYY 1.05, dYR −0.07, dRY −0.82, dRR 1.05`) separates them, the split goes on the top pair first (30 → 90°, then the bottom pair 78 → 58°) and the brake retracts the bottom pair, pitch damping `u1 = −0.6·θ − 0.4·ω_y` acts on both pairs, flaps are clamped to 15..90° and fade to 30/78 for belly angles 35..55°. Heading error beyond 150° keeps the turn direction (latch). Design, derivation and all numbers: workspace `docs/descent-control-design.md` sections 3 and 5.
  - **Descent input guards:** non-finite attitude → neutral 30/78 (a vertical fall) and the latch is cleared; non-finite heading setpoint → no heading loop (yaw-rate damping only); non-finite split or roll setpoint → 0; corrupt memory is reset.
- **Tuning:**

  | Parameter | Value |
  |---|---|
  | `flapTopNeutralAngle_Rad_` | 50° |
  | `flapBottomNeutralAngle_Rad_` | 40° |
  | `attitudeAzimuGain_` | 0.3 |
  | `attitudePitchGain_` | 1.2 |
  | `attitudeRateXGain_` | 0.2 |
  | `attitudeRateYGain_` | 0.4 |
  | `attitudeRateZGain_` | 0.1 |
  | `attitudeAzimuLimit_Perc_` | 0.5 rad |
  | `enableThresAngle_Rad_` | 35° |
  | `enableThresX_` | 0.1 (unused) |

  Setters exist for the rate gains only.

### `descent::DescentGuidance` and `descent::FlapDescentLaw` (`starship/descent_guidance.hpp`, `flap_descent_law.hpp`, shared helpers in `descent_common.hpp`)
- Header-only, pure (no scheduler, clock, globals, heap), namespace `VCTR::CTRL::descent`, plain float arrays in the ExVectr conventions. Moved from the study prototype (`feature/descent-control-study`), host-tested by `test/test_descent_guidance` (closed loop against `DSP::BodySimModel`) and `test/test_descent_unit` (guards, mode mapping).
- `DescentGuidance::step(params, DescentTarget{x,y,endAltitude,valid}, pos[6]=[V,P], att[7]=[ω,q], dt, DescentGuidanceMem&)` → `{headingSp_rad, glideCmd_mps, pitchSplit_rad, vDes, vAirCmd}`: funnel (v_des = e/3 s, limit 4.5 → 1.5 m/s over the last 20 m above `endAltitude`) → velocity PI (`velKp` 0.8, `velKi` 0.25, integral ≤ 7 m/s, anti-windup along the nose) → heading follows the air-velocity command above 1.2 m/s (holds below 0.6) → glide = command projected on the nose (0 beyond 60° heading error, ≤ 3.3 m/s beyond 20°, −1.1..4.9 m/s) → split from the glide map, slewed at 40°/s. First 1.5 s after `init`: heading held, split −20°. `DescentGuidanceMem` is 28 B and is owned by the caller.
- **Invalid input** (target invalid, non-finite position, attitude, target or `endAltitude`, `dt` ≤ 0 or non-finite, also corrupted memory): the heading setpoint is held, the glide command is 0, the split slews to 0 (neutral 30/78, a vertical fall at about −0.24 m/s of glide), the velocity integral is frozen and stays finite, and guidance resumes when the input is valid again. With a bad `dt` the split cannot slew and stays where it was.
- Model-based results (closed loop, calm, 2 seeds): miss at the freefall stop trigger median 1.2 m, p90 1.6 m, belly ≤ 13°. These rest on the unvalidated `BodySimModel` (fold shield 0.7, calibration 4.64); see the design document section 9.

### `SimplePID` (`simple_pid.hpp/.cpp`)
- 1-D PID: `SimplePID(kp, ki, kd, maxOutput)`, `setSetPoint`, `update(state, stateTime_ns)` returns a clamped output, `reset()`.
- Derivative on error. The integral is zeroed on saturation. Unused in the project.

### `PhysicsBodySim` (`physics_bodysim.hpp/.cpp`)
- State: `positionState_` `[v(3), p(3)]` world, and `attitudeState_` `[ω_body(3), q(4)]`.
- `addForce(F, r_body=0, bodyFrame=true, isAccel=false)` accumulates the force and `r×F` torque into body or nav sums. `addTorque(...)`, `clearForces()`.
- `step(dt_s)` (since 2026-11-21, `feature/sim-aero`, c8d1ffb; `simulateForTime(dt_ns)` forwards to it), semi-implicit Euler with the force and torque sums held over the step:
  - `v += (R(q)F_body + F_nav)/m·dt`, then `p += v_new·dt`.
  - `ω += I⁻¹(τ_body + R(q)ᵀτ_nav − ω × Iω)·dt` (full Euler equation, full inertia tensor with products of inertia, inverse cached by `setInertiaTensor`), then `q ← normalize(q ⊗ Δq(ω_new·dt))`, w ≥ 0.
  - Pure code (no scheduler or wall clock, no print): host-built in the flight software's native harness (`test/test_sim_aero`, torque-free spin conserves |L| within 1.3 % over 4 s at 2 ms steps).
- No damping terms (aerodynamics are the caller's job: `DSP::BodySimModel`).

## Usage

As in `src/main.cpp:168-172`, `:1939-1961`:

```cpp
CTRL::ControlPositionStandard posCtrl;
CTRL::ControlMappingAccToAtt  accToAtt;
CTRL::ControlAttitudeTvc      attTvc(VEHICLE_MASS_KG, TVC_THRUST_LIMIT_N, TVC_ANGLE_LIMIT_RAD);
CTRL::ControlAttitudeFlaps    flaps;

posCtrl.subscribePositionMeasurement(positionTopicSwitch.getTopic());
posCtrl.subscribeSetpoint(mission->getSetpointTopic());          // [v_ff, p_sp]
accToAtt.subscribeAccelSetpoint(posCtrl.getAccelTopic());
attTvc.subscribeAttitudeMeasurement(attitudeTopicSwitch.getTopic());
attTvc.subscribeAttitudeSetpoint(accToAtt.getAttitudeTopic());
attTvc.subscribeAccelerationSetpoint(posCtrl.getAccelTopic());
attTvc.setTVCLimitCompensation(true);
starshipTVC.setTVCInputTopic(attTvc.getTvcTopic());              // app-side servo/motor mapping

flaps.subscribeControlSetting(bellyFlopControlTopic);           // published by FlapModeObserverTask
flaps.subscribeAttitudeMeasurement(attitudeTopicSwitch.getTopic());
flaps.subscribePositionMeasurement(positionTopicSwitch.getTopic());
flaps.subscribeFlapSettingOutputTopic(flapSettingTopic);         // -> StarshipFlaps

posCtrl.enableControl(mission->getActuatorsEnabled());
attTvc.enableControl(mission->getActuatorsEnabled());

// Direct attitude hold (MissionBellyflop, src/mission/mission_bellyflop.hpp:87-104)
attTvc.unsubscribeAttitudeSetpoint();
auto qSp = Math::Quat_F(Math::Vector_F{0, 1, 0}, -90 * DEGREES);   // nose to world -X, belly down
attTvc.setAttitudeStateSetpoint({0, 0, 0, qSp(0), qSp(1), qSp(2), qSp(3)});   // re-sent every tick
```

## Conventions

- See **`docs/libs/ExVectrMath.md` → Conventions**, which is authoritative: Hamilton product since `b221829`, `[w,x,y,z]`, q = body→world, world NWU.
- Accel setpoints are world-frame and **include gravity compensation** (+g on Up).
- TVC output: body-frame force in N. A positive Fz pushes along the nose. `T` is the roll (body Z) torque command.
- Attitude errors are in body frame (`q* ⊗ q_sp`). Torques are body frame.
- Flap angles: rad; 0 = extended (normal ±X, maximum drag in bellyflop), 90° = retracted. TL/BL are on +Y, TR/BR on −Y; top is at z = +0.26 and bottom at z = −0.26.
- Azimuth (`ControlAttitudeBellyFlopSetting`) is measured counter-clockwise from North toward West (NWU `atan2(y,x)`). `src/main.cpp:1297-1307` computes the azimuth toward the origin consistently with that.
- `lastRunTimestamp_` comments say "microseconds"; it is Core ns.

## How this project uses it

- `src/main.cpp:168-172`: instantiates all four controllers. `CTRL::StarshipTVC` and `CTRL::StarshipFlaps` (actuator mapping) live in `src/starship_tvc.*` and `src/starship_flaps.*`, not in the library.
- `src/main.cpp:1939-1961`: topic wiring. The position setpoint comes from the active mission (`:536`).
- `src/main.cpp` `applyControlPhase()`: `enableControl` and `resetControlState()` of the position and TVC attitude controllers from the explicit phase table `Safety::controlPhaseFor` (`src/safety/control_phase.*`), every tick (since 2026-11-20; before, on mission-mode edges). See missions.md 6.2.
- Nothing in `src/` uses `BellyFlopMode_Descent` or `pitchSplit_Rad` yet (S3 of the descent plan); `main.cpp` and the flap behaviour are unchanged.
- `src/main.cpp` `FlapModeObserverTask` (10 Hz) chooses the bellyflop mode and sets azimuth/pitch targets (freefall and the bellyflop after its powered kick → Stabilize (bellyflop since 2026-11-20, after the kick only since 2026-11-21), RTH stabilising → Upright, otherwise Ascent), published on `bellyFlopControlTopic`.
- `src/mission/mission_bellyflop.hpp`: takes over the TVC attitude setpoint directly (`unsubscribeAttitudeSetpoint()` + `setAttitudeStateSetpoint()` every 10 ms) for the powered bellyflop kick (2026-11-21), and calls `enableControl(false)` itself at the kick end so the motors go neutral without waiting for the 10 Hz phase table.
- `CTRL::PhysicsBodySim` is used inside `DSP::BodySimModel` / `DSP::BodySimulator`. `SimplePID` is unused.

## Known issues & limitations

"Quat-semantics" marks call sites whose behaviour likely changed with ExVectrMath `b221829`. This library was tuned on 2026-01-25 against the old reversed product and inverse `rotate()`. With the **current** Math, the Control call sites below read as correct, which suggests the fix was made for them. But the gains were tuned in a simulator whose own quaternion code changed at the same time, so **re-validate the tuning in sim and on the bench**.

| Severity | Location (path:line) | Issue | Impact |
|---|---|---|---|
| High | `src/control_attitude_tvc.cpp:236-250` | When saturated, `forceVectorNew` is a **unit** vector. Without compensation, thrust collapses to 1 N. With compensation (main's setting) it is scaled by `|F_xy|/sin(limit)` with no upper bound. `tvcThrustLimit_N_` is never applied in the controller. | Without compensation: loss of thrust and a fall. With it: unbounded thrust demand, which the header itself warns can run away. Clamping is left to `StarshipTVC` and the simulator. |
| High | `src/control_attitude_tvc.cpp:128-136` | With no new setpoint, `q_sp` is propagated by the **measured** body rate (`ω`, not `ω_sp`). | The setpoint rotates with the vehicle, so the attitude error freezes and the vehicle never converges. Hit whenever setpoints are slower than 20 ms, e.g. after `setAttitudeStateSetpoint` if the mission ticks slower than the controller. |
| Med | `src/control_attitude_tvc.cpp:157-161` | Yaw is controlled only when tilt < 10°, and with `atan2(q_z,q_w)`, which is half the yaw angle. | Yaw control switches on and off discontinuously around 10° tilt (a torque step). The effective yaw gain is half the nominal value. |
| Med | `src/control_position_standard.cpp:150-152` | The vertical velocity command has only an upper clamp. There is no descent-rate limit. | A large altitude error commands an unbounded descent speed. |
| Med | `src/control_position_standard.cpp:184-237` | Anti-windup bounds only the integral. The published acceleration (P + I + g) is never clamped to `velocity*Limit_mss_`. | Large velocity errors give large accel, which the 35° tilt limit then saturates. Thrust demand can exceed the motor limit. |
| Med | `src/control_attitude_tvc.cpp:213-224` | `Fz` is the projection of the wanted force onto body Z. The lateral TVC force (step 5) is added without reducing Fz or accounting for its translational effect. There is no inertia scaling (gains are N·m/rad). | Coupling between attitude and translation. Gains must be re-tuned for any mass or inertia change. |
| Med | `src/control_attitude_tvc.cpp:148`, `:153`, `:214` (Quat-semantics) | `q*⊗q_sp`, `q_e.rotate(e_z)`, `q.conjugate().rotate(a)`. | Correct with current Math (body-frame error; world→body). Under the old Math they were a world-frame error and the inverse transform. Gains were tuned against the old behaviour. |
| Med | `src/control_attitude_tvc.cpp:130`, `:238` (Quat-semantics) | Setpoint propagation `q_sp ⊗ Δq` and limit rotation `Quat(e_z×F, limit).rotate(e_z)`. | Correct with current Math. Both behaved inversely before. |
| Med | `src/physics_bodysim.cpp:113`, `:121` (Quat-semantics) | `q.rotate(F_body)` and `q ⊗ Δq(ω)`. | Correct now. Under the old Math the simulator used the inverse attitude for force rotation and a world-frame rate integration, so tuning done in sim before `b221829` is suspect. |
| High | `src/starship/control_attitude_flaps.cpp` (Stabilize law, neutral 50/40°) + `src/main.cpp` `FlapModeObserverTask` (descent guidance) | Descent study 2026-11-22 (`feature/descent-control-study`, `test/test_descent_guidance`, design: workspace `docs/descent-control-design.md`), simulation model with the fold shield: (1) the guidance is not a position controller: pitch setpoint from the distance only (+20° nose-up inside 21 m, which glides away from the nose), active only above 100 m; below 100 m the azimuth is frozen and the pitch setpoint is 0. (2) 50/40° does not trim: the free body glides 8.4 m/s nose-first at −7.3°, and the P-only pitch loop cannot hold 0° (needs +0.72 N m), so the ship keeps gliding 3–4 m/s along the frozen azimuth. (3) The mixer drives yaw on all four flaps (yaw +0.0038 N m/deg with the shield, −0.0073 without), roll is only rate-damped, and the roll tilt makes a yaw moment of −0.007..−0.010 N m per degree of roll: a ±30° roll / ±35° heading limit cycle (the "snapping" about the frozen azimuth). | Reproduced: the ship flies to home, past it and oscillates; miss at the stop trigger 13.9 m median in calm air, 43.9 m in wind 3/5 m/s (20 seeds). **Still present in flight:** the legacy Stabilize law and the 10 Hz observer guidance are unchanged. The replacement (`descent::DescentGuidance` + `FlapDescentLaw`, mode `Descent`, neutral 30/78) exists in this library since 1902fa4 (steps S1/S2 of the design document) but nothing selects it yet (S3 wires the guidance into the 50 Hz flap task and `FlapModeObserverTask`). |
| Med | `src/starship/control_attitude_flaps.cpp:77-82` (Quat-semantics) | `q.rotate(body axis)` gives the body axes in world. | Correct now; inverted before. The azimuth/pitch signs and the mixer signs were tuned under the old behaviour, so verify the flap sign mapping. |
| Low | `src/control_mapping_acctoatt.cpp:66-74` | Built from axis-angle (independent of the product order). Under current `rotate()`, body +Z maps to the accel direction. | Correct now. Under the old Math the tilt pointed the mirrored direction. No yaw handling (yaw intent is always 0). |
| Low | `src/starship/control_attitude_flaps.cpp:83-84`, `include/ExVectrControl/starship/control_attitude_flaps.hpp:84` | `velXAxis` and `enableThresX_` are computed or declared but unused. `rollAngle_Rad` is unused. The comment says "10 deg" but the fade uses 20° (`:89-91`). | Dead code and misleading docs. |
| Low | `src/starship/control_attitude_flaps.cpp:150-165`, `:192` | No clamping of flap commands. `enableFlaps` is always true. | Relies on `StarshipFlaps` and the simulator for limits. |
| Low | `src/control_attitude_tvc.cpp:18`, `src/control_position_standard.cpp:19`, `src/starship/control_attitude_flaps.cpp:16` | All three tasks are named `"Control Rocket"`. | Ambiguous scheduler diagnostics. |
| Low | `include/ExVectrControl/control_attitude_tvc.hpp:20-26`, `:13-14` | Doc comments describe the setpoint as `[V,P]` (it is `[ω,q]`) and the class as a "Simple one dimensional PID". `tiltLimit_Rad_` is unused. | Misleading docs. |
| Low | `src/simple_pid.cpp:43`, `:53-59` | `(stateTime − lastTime_)` can be 0, giving a divide by zero. There is a derivative kick on the first call. The integral is reset to 0 (not held) on saturation. | Unused, but unsafe if adopted. |
| ~~Low~~ Fixed | `src/physics_bodysim.cpp` | `hdtsq = 0`. No gyroscopic `ω×Iω` term. `torqueSumNav_` is ignored. Only diagonal inertia. | Fixed 2026-11-21 (`feature/sim-aero`, c8d1ffb): full Euler equation with the full tensor, nav torques rotated into the body frame, semi-implicit Euler. |

## Extending

To add a controller:
1. Create `include/ExVectrControl/<name>.hpp` and `src/<name>.cpp` in `namespace VCTR::CTRL`. Vehicle-specific controllers go under `starship/`.
2. Subclass `Core::Task_Periodic`. In the constructor: `Task_Periodic("<Unique Name>", 20 * Core::MILLISECONDS)` followed by `Core::getSystemScheduler().addTask(*this)`.
3. Inputs: `Core::Simple_Subscriber` on the **standard topics**, so it works with both estimator and simulator through the `Topic_Switch`es:
   - attitude `Timestamped<Vector<7>> [ω,q]`;
   - position `Timestamped<Vector<6>> [v,p]`;
   - setpoints `Vector<6>` / `Vector<7>` / `Vector<3>` (world accel including g).
4. In `taskInit()`, call `setInitialised(false)` until required inputs have arrived, and reset integrators. In `taskThread()`:
   - compute dt from `Core::NowNs()` (ns) and guard dt ≤ 0;
   - read with `getItem()`;
   - publish on an owned `Core::Topic<…>`, or a `Topic_Publisher` if the consumer owns the topic.
   - Provide `enableControl(bool)` that zeroes integrators and outputs a safe command when disabled.
5. Compute attitude errors as `q.conjugate() * q_sp` (body frame). Use `q.rotate()` for body→world and `q.conjugate().rotate()` for world→body, per `docs/libs/ExVectrMath.md`.
6. Clamp outputs to actuator limits inside the controller. Don't rely on the downstream mapping.
7. Wire it in `src/main.cpp` `initialiseTopicConnections()` (`:1920`) and gate it in the vehicle-mode logic (`:620-690`). Test it with `DSP::BodySimulator` by switching the topic switches to the simulator outputs (`:920-960`).
