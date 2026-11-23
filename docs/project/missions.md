# Missions

How the mission framework in `src/mission/` works, each mission's state machine, and how `src/main.cpp` sequences missions, maps them to telemetry phases and picks flap modes.

Last verified against commit: b073d07 (chain redirect to RTH added on `feature/sim-fixes-3`, 2026-11-21: §1, §2, §6.2, §7 and Known issues) (§1, §5, §6.2 and Known issues re-verified on `feature/reset-chain-gnss`, 2026-11-20; §3, §5, §6.2, §8 and Known issues updated on `feature/bellyflop-abort-gnss`, 2026-11-20)

Conventions used here: time is `int64_t` nanoseconds (`Core::SECONDS = 1e9`). The reference frame is NWU (X north, Y west, Z up). Position state and setpoint vectors are `[Vx, Vy, Vz, Px, Py, Pz]`. The attitude state is `[ωx, ωy, ωz, qw, qx, qy, qz]` with q rotating body to reference (see [../codebase-structure.md](../codebase-structure.md)). "Safety task" means `VehicleSafetyAndControlTask` in `main.cpp`.

---

## 1. Framework: `MissionAbstract` (`src/mission/mission_abstract.hpp`)

Every mission derives from `MissionAbstract`, and every concrete mission also derives from `Core::Task_Periodic`, registering itself with the system scheduler in its constructor. So all missions exist and tick from boot. Being "active" only means the safety task's `mission_` points at that mission.

| Member | Meaning |
|---|---|
| `positionSetpointTopic_` (`Topic<Vector<float,6>>`) | Setpoint `[Vx,Vy,Vz,Px,Py,Pz]` in the reference frame. `switchMissionTo()` subscribes `controlPositionStandard` to the active mission's topic (`main.cpp:536`). |
| `missionTime_` (`Time_Source`) | Mission clock. `beginMission(startTime)` sets it to `startTime`, so a negative start time gives a countdown. |
| `missionState_` (`MissionState`, see `telemetry.hpp`) | Mode, mission time and setpoint copy. The telemetry task maps it into `VEHICLE_STATUS` and `MISSION` (`src/comms/telemetry_mapping.cpp`). |
| `nextMission_` / `nextMission()` | Chaining: the mission to switch to when this one ends. `nullptr` means fall back to the default (RTH). |
| `actuatorsEnabled_` | The safety task reads it every tick to gate the TVC, motors and flaps (`main.cpp:1195-1202`). |
| `disableKinematicSafety_` | When true, the safety task skips the position and tilt out-of-bounds checks for this mission (`main.cpp:1108`). |
| `missionEnd_` / `missionEnd()` | When true **and the mission is not a reset one**, the safety task switches to `nextMission()` or to RTH (§6.2). Every `resetMission()` also sets it to true, so `missionEnd()` alone does not mean "finished": the chain decision (`Safety::MissionChain`, `src/safety/mission_chain.hpp`) treats "ended and mode Idle" as a reset, which never chains. |
| `clearMissionState()` | Since 2026-11-20 called by every `resetMission()`: mode Idle, mission time 0 (clock and reported), setpoint copy zero, so a reset mission reports its boot state. |

Virtuals: `beginMission(int64_t startTime)`, `resetMission()`, `missionEnd()`, `nextMission()`. `MissionType` (`mission_abstract.hpp:19`) declares RTH, Remote, Waypoint and FlapDescent, but nothing uses it, and Remote and FlapDescent have no implementation.

`MissionMode` (`telemetry.hpp:71`) values: `Idle`, `Initialisation`, `Startup`, `Running`, `Finished`. The safety task reacts to changes of the active mission's mode, not to its level (see §6.2).

### Mission instances (main.cpp:437-449)

| Instance | Class | Constructor arguments as passed |
|---|---|---|
| `defaultMission_` | `MissionRTH` | att = `attitudeTopicSwitch`, pos = `positionTopicSwitch`, ground = `groundTopicSwitch`, groundCov = `posEstTask.getGroundDistanceCovTopic()`, **home = `{0, 0, 3}`** |
| `missionWaypointTask` | `MissionWaypoint` | att, pos topic switches. Waypoints are added in the safety task's `taskInit` (§7). |
| `missionFreefallTask` | `MissionFreefall` | att, pos, `vehicleMass_kg = VEHICLE_MASS_KG` (1.1), `tvcThrustLimit_N = TVC_THRUST_LIMIT_N` (14.0), **`stopAlt = 20`** m |
| `missionBellyflop` | `MissionBellyflop` | att, `controlAttitudeTvc`, `controlMappingAccToAtt`, **`transitionTimeLimit = 1 s`**, **`angleThreshold = 50°`**, **`angVelThreshold = 150°/s`** |

### Configured chain (main.cpp:556-558)

```
SA go ─► Waypoint ──end──► Bellyflop ──end──► Freefall ──end──► RTH (default) ──end──► shutdown
                       │                 │                  │
                       └──── SA released in flight (any phase) / mag timeout (mission mode Running) ────► RTH
```

SA released on the ground (before liftoff, by mission mode only: countdown, Idle mission, RTH landed) resets and stops instead (§6.2).

**RTH instead of the bellyflop (owner decisions 2026-11-21, X6 of the flight executive design applied to the current chain, `feature/sim-fixes-3`):** the Waypoint → Bellyflop step goes to RTH (default mission, hot start) instead when (a) the waypoint mission ended because the waypoint with the cancel flag (the 1 m hover check) was not reached in its time limit (`MissionWaypoint::endReason() == Cancelled`), or (b) the altitude (position estimate z, NWU, above the position reference) is below `Safety::kMinKickAltitudeM` = **90 m** or not finite, for any reason the waypoint mission ended (for example the ascent waypoint's 25.5 s time limit at 50 m). The normal flight reaches the 120 m apogee and is unchanged. Decision in `Safety::MissionChain::step()` (§6.2); log `Mission ended: RTH instead of the next mission (<reason>, altitude <z> m, minimum 90 m)`.

---

## 2. `MissionWaypoint` (`src/mission/mission_waypoint.hpp`)

**Purpose:** flies a list of 3-D waypoints by moving the position setpoint along a velocity and acceleration profile. It is used for take-off, hover checks and the climb to apogee.

- Task: 10 Hz (`0.1 s`). The task name is the copy-pasted `"Mission RTH"` (`:76`).
- `disableKinematicSafety_ = false`. This is the only mission that keeps the out-of-bounds checks.
- `nextMission_` = `missionBellyflop` (main.cpp:556).

### `addWaypoint` signature (`:104`)

```cpp
void addWaypoint(const Vector<float,3>& position, float velocity = 0,
                 float startAccel = 3, float stopAccel = 3,
                 float thresholdDistance = 1, int64_t loiterTime = 0,
                 int64_t timeLimit = Core::END_OF_TIME,
                 bool cancelIfNotReachable = false);
```

| Arg | Unit | Meaning |
|---|---|---|
| `position` | m, ref frame | Target point. |
| `velocity` | m/s | Cruise speed of the setpoint. `< 0.0001` means **instant jump** of the setpoint (applied when the waypoint becomes current, `:346-353`). |
| `startAccel` | m/s² | Ramp-up rate of the setpoint speed. `< 0.001` means jump straight to cruise speed. |
| `stopAccel` | m/s² | Braking limit: the speed is capped at `sqrt(d · stopAccel)` (`:424`), where d is the distance from the **measured** position. That is equivalent to a constant deceleration of `stopAccel/2`, not `stopAccel`. |
| `thresholdDistance` | m | Reached when \|target − measured position\| < threshold. |
| `loiterTime` | **ns** (the doc comment says seconds) | Time the vehicle must stay continuously inside the threshold before advancing. |
| `timeLimit` | ns | If the waypoint is not reached within this time of becoming current, it is skipped or cancelled. Default: `distance_from_previous / velocity × 1.5 s` (`:119`, `:125`). For the first waypoint the distance is measured from the origin. |
| `cancelIfNotReachable` | bool | On timeout, end the whole mission instead of skipping to the next waypoint. **The end still follows `nextMission()`, which is Bellyflop** (see Known issues). |

### State machine (`missionState_.missionMode`)

| From | To | Trigger | Actions |
|---|---|---|---|
| any | `Initialisation` | `beginMission(t0)` | `resetMission()` first, then `missionTime = t0`, actuators off, `missionEnd_ = false`, index 0. |
| `Initialisation` | `Startup` | `missionTime > −10 s` (`:249`) | actuators off |
| `Startup` | `Running` | `missionTime > 0` (`:292`) | Every tick in Startup: actuators **on**, setpoint = `[0,0,0, measured P]`, index reset to 0. If the list is empty, adds a default `{0,0,1}, v=0.5, startAccel=1`. |
| `Running` | (stays `Running`) | last waypoint reached, or timeout on the last waypoint, or cancel | `missionEnd_ = true`. The mode **never becomes `Finished`**, and the mission keeps publishing setpoints toward the last waypoint. |
| any | `Idle` | `resetMission()` | actuators off, `missionEnd_ = true`, setpoint zeroed, mission time 0 |

In `Idle` (`missionIdle`) the mission sets actuators off, mission time 0 and `missionEnd_ = true` every tick.

### Running logic (`missionHover`, `:299-447`), per 10 Hz tick

1. `d = target − measured position`.
2. If `|d| < threshold`: set `waypointReached_`. If the vehicle has been inside the threshold for longer than `loiterTime`, snap the setpoint to the target and advance the index (reset speed to 0, apply the instant jump if the next waypoint has v ≈ 0). If this was the last waypoint, set `missionEnd_`.
3. Otherwise restart the loiter timer. If the waypoint was **never** reached and `NOW − waypointStartTime > timeLimit`: snap the setpoint to the target, then cancel (`missionEnd_`, `endReason() = Cancelled`), advance to the next waypoint, or finish if this was the last one (`LastTimedOut`). A normal end is `Completed`; `begin`/`reset` set `None`. The chain reads the reason (§1 "RTH instead of the bellyflop"). Once a waypoint has been reached its time limit no longer applies, so an overshoot followed by drifting out can stall forever.
4. Speed profile: brake to `sqrt(|d|·stopAccel)`, otherwise ramp up by `startAccel·dt` up to `velocity`.
5. Move the setpoint position toward the target by `min(remaining, v·dt)`. Set the velocity setpoint to `unit(target − setpointPos) · v` (uses `normalize()`, which returns zero for a zero vector, so no NaN).

**Setpoint published (every tick in every mode):** `[Vx,Vy,Vz,Px,Py,Pz]` = the profiled velocity and the moving setpoint position, also copied into `missionState_.positionSetpoint`.

---

## 3. `MissionBellyflop` (`src/mission/mission_bellyflop.hpp`)

**Purpose:** the powered **bellyflop kick** (user decision 2026-11-21): a short transition at the start of the bellyflop in which the motors stay on and the TVC attitude controller pitches the vehicle from vertical towards the belly-down attitude; then the motors go to zero and the flap controller (freefall) takes over.

- Task: 100 Hz (`0.01 s`), name `"Mission Bellyflop"`. **Paused** unless the mission is active. **No `taskInit` override** since 2026-11-20 (Fixed entry in Known issues).
- `disableKinematicSafety_ = true` (the tilt leaves the kinematic bounds by design).
- `nextMission_` = `missionFreefallTask`.
- **All tuning is in named constants** in `src/safety/bellyflop_kick.hpp` (pure logic, host-tested in `test/test_bellyflop_kick`):

| Constant | Value | Meaning | Origin |
|---|---|---|---|
| `kKickPitchAngleRad` | 90° | pitch about body +Y from the start attitude (body +X up, belly = body −X down) | old firmware: fixed `Quat(y, −90°)` |
| `kKickTimeLimitNs` | 1 s | **hard limit**: motors to zero at the latest this long after the bellyflop began, attitude reached or not | old firmware: 1 s |
| `kKickEndRemainingAngleRad` | 15° | normal end: remaining angle between body +X and its target below this | old firmware: 50°; lowered after the bench (below) |
| `kKickEndPitchRateRadPs` | 250°/s | spin guard: \|body pitch rate\| above this ends the kick | old firmware: 150°/s as an early-end trigger |
| `kKickThrustLimitN` | 11 N | thrust cap during the kick (close to the weight 10.8 N at 1.1 kg; motor limit 14 N). The kick is **not meant to fly** (owner 2026-11-21): it only gives the rotation impulse that gets the ship roughly into the right attitude faster, so the cap is not tuned for height | new; the old firmware ran the kick at full thrust (TVC angle compensation asked far above 14 N) |
| `kKickGuardMarginNs` | 100 ms | 10 Hz guard in the phase tracker: TVC set off if the bellyflop is still active past limit + margin | new (defence in depth) |

| From | To | Trigger | Actions |
|---|---|---|---|
| any | `Running` | `beginMission` | actuators **on** (motors stay on), unpause, `controlTvc_.unsubscribeAttitudeSetpoint()`, compute the kick target from the current attitude (`Safety::kickTargetAttitude`) and write it to the TVC controller at once. |
| `Running` | `Finished` | `Safety::kickEndTrigger`, every 10 ms: elapsed > 1 s (time), else remaining angle < 15° (angle), else \|ω_y\| > 250°/s (rate) | `controlTvc_.enableControl(false)` **at once** (its next 50 Hz run publishes the neutral `[0,0,0.1 N,0]`), `missionEnd_ = true`, pause. The next 10 Hz safety tick switches to freefall (TVC actuators off: motors 0, flap attitude control on). The TVC setpoint stays on the kick target until `resetMission`. |
| any | `Idle` | `resetMission` | actuators off, `missionEnd_ = true`, pause, resubscribe the TVC attitude setpoint to `controlMappingAccToAtt`, mission time 0. |

**Old firmware:** the same mission (b765e1b, May 2025) was written as a powered kick: fixed −90° pitch setpoint into the TVC attitude controller (the first version also fed −90°/s as the rate setpoint "to really kick it there"), 1 s / 50° / 150°/s triggers, thrust from the controller (full 14 N in practice). On the first bellyflop after boot, however, the old `taskInit` set the mode to `Idle` and the mode-edge logic switched the TVC controllers off (Fixed 2026-11-20), so the real flights (flight4, flight5: TVC thrust 0 in the logs) flew the pitch-over unpowered by accident; only later bellyflops after a reset got the powered kick. The 2026-11-20 phase table first kept it unpowered (user decision), which made the simulated vehicle fall vertically; the 2026-11-21 decision restores the powered kick with a thrust cap and a hard limit.

**Target:** `kickTargetAttitude(q0) = Rz(ψ) · Ry(−90°)`, where ψ keeps the start heading: the horizontal direction of the belly (body −X) at kick start. The rotation is a pure pitch about body +Y; the nose ends horizontal towards the start belly side. For q0 = identity (the ascent holds yaw 0) it is exactly the old fixed `Quat(y, −90°)` (nose south). If the belly direction is nearly vertical (vehicle already flat) ψ = 0. **Remaining angle** = angle between body +X now and body +X of the target (the flap controller's belly-down metric for a 90° kick). The position setpoint topic is never published by this mission; the position controller (part of the TVC set) holds the measured position from its reset at the switch, its acceleration only adds to the axial thrust request, which the cap limits.

**Controllers (phase table §6.2):** in `Running` the TVC set is **on** (reset at the waypoint → bellyflop switch), thrust capped at `kKickThrustLimitN` by `StarshipTVC::thrustCap()`; the flaps stay in `Ascent` (retracted), the flap attitude control is **off**, so the two sets never act at the same time. After the kick (Finished) and in freefall the TVC set is off and the flap attitude control on.

**Timing of the motor cut:** the end trigger is checked at 100 Hz, so the kick ends at most 10 ms after a trigger; the TVC controller output goes neutral (0.1 N, motors at the idle floor) within a further 20 ms, and the motors are off (TVC actuators disabled) at the next 10 Hz safety tick (≤ 100 ms). Worst case if the mission task stalled: the phase-table guard switches the TVC set off at the first safety tick after 1.1 s.

**Bench (simulation, 2026-11-21, `tools/bench_vehicle_usb.py`, full chain waypoint → bellyflop → freefall → RTH → landing, three runs with the final constants, no Failure bits):** kick ended by the angle trigger after 0.94–0.96 s at 14.4–14.6° remaining and 50–51°/s; thrust 11.0 N (cap) and vane deflection +3° (limit) from the first sample, reversing to −3° from about 40° tilt (the controller's rate term brakes); tilt 2° → 75° at the motor cut, peak body rate 134°/s, horizontal speed 3.6 m/s, altitude −0.4 m. After the cut the vehicle coasted to belly-down (tilt 87–95°, remaining 2–5° about 0.3 s after the cut) and stayed near horizontal (77–95° tilt) in freefall. `setpoint_q` = the kick target during the kick. **Old triggers (50°, 150°/s), same bench:** the kick ended after 0.59 s at 36° tilt and 133°/s; the momentum carried the vehicle through belly-down to 140° tilt (50° nose-down past horizontal), which is why the angle trigger was lowered. The simulator's aerodynamics are known to be wrong (no aerodynamic pitch damping or pitch-over), so only the powered kick and the handover are meaningful; real aerodynamics will add damping and pitch-over moment.

---

## 4. `MissionFreefall` (`src/mission/mission_freefall.hpp`)

**Purpose:** unpowered, flap-stabilised belly-down fall until the altitude at which a full-thrust stop is still possible.

- Task: 100 Hz. `disableKinematicSafety_ = true` (`:42`, re-asserted every tick at `:146`).
- `nextMission_` = `defaultMission_` (RTH). The `ctrlSubr_` member is unused.

| From | To | Trigger | Actions |
|---|---|---|---|
| any | `Running` | `beginMission` | `actuatorsEnabled_ = false`, `missionEnd_ = false` |
| `Running` | `Finished` | `Pz < stopDist(Vz) + stopAlt` (`:119`) | `missionEnd_ = true`. Afterwards `taskThread` returns immediately (`:83`), so nothing more is published. |
| any | `Idle` | `resetMission`, or `taskInit` (which also sets `missionEnd_ = true`) | actuators off |

`stopDist = 0.5·Vz² / (T/m − 9.81)` (`:157`). With T = 14 N and m = 1.1 kg (the measured mass; a 1.2 kg value was in the code only between 2026-11-21 commits and was reverted by the owner): `T/m − g = 2.917 m/s²`, so the trigger altitude is `0.171·Vz² + 20` m. For example, the trigger is at 88 m when falling at 20 m/s. A drag-free fall from 120 m triggers at about 96 m and 21 m/s; drag lowers both numbers. The formula uses `Vz²`, so it ignores the sign of the velocity and also triggers while climbing fast. It also ignores the time needed to flip upright. `getStopAltitude()` returns `stopDist + stopAlt` and is used by the flap observer.

**Setpoint:** publishes `[0,0,0,0,0,0]` every tick while active. **TVC actuators are off** (`actuatorsEnabled_ = false`), but the safety task force-enables the **flaps** for this mission (`main.cpp:1195-1196`).

---

## 5. `MissionRTH` (default mission) (`src/mission/mission_rth.hpp`)

**Purpose:** fallback and recovery. It kills velocity, flies straight to `home` (`{0,0,3}`) and descends to touchdown. It runs after freefall, after an in-flight `MissionAbort` (from any phase, §6.2) and after a magnetometer timeout.

**Starting from any phase (bench, simulation, 2026-11-20):** the hot start uses only the measured state, and the TVC controller set is enabled and reset in the same tick (§6.2), so RTH can start from hover, ascent, the bellyflop transition (it re-subscribes the TVC attitude setpoint to the acc-to-att mapping in `MissionBellyflop::resetMission`) and freefall. Freefall at 72.8 m, tilt 80°, 16 m/s down: upright (tilt 11°) within 1 s at full thrust, stopped at 40 m after 5 s, then translation, landing and auto-disarm. Limits that remain: a start low and fast in freefall may not stop in time (the freefall stop altitude assumes full thrust at once, Known issues), and the translation is a straight 3-D line (no climb).

**Without GNSS (today, before T9b):** RTH never runs without GNSS. GNSS data older than 0.5 s latches `Failure` (`sensorFailure`) and cuts all actuators (failsafe), whatever the mission. A magnetometer timeout (> 2.5 s) in a Running mission switches to this normal RTH, which then navigates with the GNSS/baro position and the gyro-propagated heading. The "no-position RTH" (point up, descend on the baro) of safety.md 4.3 is not implemented (T9b / failure policy).

- Task: 10 Hz. `disableKinematicSafety_ = true` (`:83`).
- `nextMission()` is overridden to always return `nullptr` (`:115`), so `setNextMission` has no effect.
- `beginMission(startTime)` **ignores `startTime`**: it sets mission time to 0 (`:129`), actuators on immediately, setpoint = measured position with zero velocity, then calls `missionStartup()`, which sets mode to `Running` and `runningState_ = Stabilize`. There is no Init or Startup dwell (a "hot start").

### Outer modes

| From | To | Trigger |
|---|---|---|
| any | `Running` (via `Startup` in the same call) | `beginMission` |
| `Running` | `Finished` | landing detected in Descent (below) |
| any | `Idle` | `resetMission` (`missionEnd_ = true`, actuators off, mission time 0, setpoint zero, `runningState_ = Stabilize`) |

In `Finished` (`missionLanded`) the mission sets actuators off, `missionEnd_ = true` and the setpoint to **all zeros**. Since 2026-11-20 `resetMission` also resets `runningState_` to `Stabilize` and the setpoint to zero (the boot values); `beginMission` sets both again, so a running RTH is unaffected.

### Running sub-states (`RunningState`)

| From | To | Trigger | Setpoint behaviour |
|---|---|---|---|
| `Stabilize` | `Ascent` | measured \|V\| < 0.5 m/s (`:268`) | `[0,0,0, measured P]` every tick, so the controller acts as a pure velocity damper. |
| `Ascent` | `Translation` | immediately, on the next tick (`:278`) | unchanged. The "ascend to translation altitude" step is **not implemented**. |
| `Translation` | `Descent` | \|home − measured P\| < 0.5 m (`:337`) | Straight 3-D line from the setpoint to `home`, so altitude changes at the same time. Speed ramps at 0.5 m/s² to at most 2 m/s, braking cap `sqrt(0.5·d)` (`:296-311`). The velocity setpoint is `unit·v`, using `travelDistance / mag` (**NaN if the setpoint reaches home exactly**). On the transition the setpoint Z is set to the measured Z. |
| `Descent` | (outer `Finished`) | (a) vehicle more than 0.2 m **above** the Z setpoint for > 2 s (`:400-415`), or (b) ground estimate `g(1)` in (−0.02, 0.04) m with `cov(1,1) < 7` and fresh (< 100 ms) (`:417`) | XY setpoint = home XY. The Z setpoint descends at 0.2 m/s, but only while the vehicle is less than 0.3 m above the setpoint (`:390`). |

Trigger (a) detects ground contact, when the vehicle stops following the setpoint. It fires on **any** 2 s period of tracking error above 0.2 m, at any altitude (see Known issues). Trigger (b) is **dead in real flight**: `groundTopicSwitch` is only subscribed inside the simulation-mode toggle (`main.cpp:933`, `:955`), never at boot (`main.cpp:1971-1972` subscribe only the position and attitude switches).

**Setpoint published:** `[Vx,Vy,Vz,Px,Py,Pz]` every tick in every mode.

---

## 6. Mission control in `VehicleSafetyAndControlTask` (`main.cpp:456-1255`)

10 Hz. Each tick (`taskThread`, `:570-615`):

1. `applyRcCommands()` (T9b-4: SA abort level, go, SC, SE reset and hold from the RC state), `handleTelecommands()` (the non-safety commands only), then `superviseMotorInterlock()` (T9b-2: control-source latch input, D4-A re-arm while in Failure, gate log line; before anything can return early).
2. If `!systemsInitialised()`: disable everything and return. Phase mapping and mission control are skipped.
3. Map the active mission to a `MissionPhase` (§6.1).
4. `checkForFailures()`.
5. `handleVehicleMissionControl()` (§6.2).
6. `vehicleShutdownControl(!vehicleIsReady())` (§6.3).
7. *(removed in T9b-4: `vehicleArmingControl()` and the 10 % power limit; the limit is 1.0 and the SD interlock is the gate.)*

### 6.1 `MissionPhase` mapping (`:579-600`)

| Active mission | Condition (first match) | `MissionPhase` |
|---|---|---|
| RTH | `missionEnd()` | `Completed` |
| RTH | `stabilising()` | `Deceleration` |
| RTH | `translation()` | `RTH` |
| RTH | `descent()` | `Landing` |
| RTH | (Ascent sub-state, 1 tick) | unchanged (previous value kept) |
| Waypoint | `currentWaypointIndex() > 1` | `Ascent` |
| Waypoint | otherwise (including Init and Startup) | `Hover` |
| Bellyflop or Freefall | always | `Descent` |
| other | always | `Idle` |

At boot `mission_` is RTH in `Idle` with `missionEnd_ = true`, so the phase reads `Completed`. The telemetry task (`TelemetryProvider` / `VehicleComms`, `src/comms/`) sends the current phase in `VEHICLE_STATUS` and `MISSION`.

### 6.2 Mode-edge actions and switching (`handleVehicleMissionControl`, `:617-726`)

Mode-edge actions fire only when the active mission's mode **differs from `lastMissionMode_`** (shared across missions). Since 2026-11-20 they no longer switch the mission controllers (see "Controller phase table" below):

| New mode | posEst / IMU / bodySim zeroing | Other |
|---|---|---|
| `Idle` | off | flaps disabled |
| `Initialisation` | **on** | flaps enabled (then overridden, §6.3) |
| `Startup` | off | `starshipTVC.beginActuatorTest(0)` (4 s servo test). Flap test only with `DO_FLAP_TEST_STARTUP` defined (undefined). |
| `Running` | off | flaps enabled |
| `Finished` | off | flaps disabled |

Since 2026-11-20 the zeroing column sets `missionZeroing_`; `applyEstimatorZeroing()` puts the three estimators into zeroing mode while `missionZeroing_` **or** the `ZeroEstimators` / boot-zero window is active (overview.md, "Estimator zeroing"). The mission's own zeroing is unchanged and does not set `VEHICLE_STATUS` `ZEROING` (the window cannot overlap a mission: `MissionBegin` is refused during a window, and `ZeroEstimators` is refused while a mission is active).

#### Controller phase table (user decision 2026-11-20)

The mission controllers are switched in **one place**, `applyControlPhase()` (main.cpp), from the pure table `Safety::controlPhaseFor` (`src/safety/control_phase.*`, host-tested in `test/test_control_phase` and inside the chain model of `test/test_mission_chain`). It is evaluated **every tick** from the active mission and its mode (a level, not an edge), after all switches of that tick (telecommands, failure checks, chain step), and also in the not-initialised path. So the first and every later run of a mission get the same controllers, and no task-init callback or edge history can change them.

| Active mission and mode | TVC set (`controlPositionStandard` + `controlAttitudeTvc`) | Bellyflop set (flap attitude control, `Stabilize`) |
|---|---|---|
| Waypoint `Running` (hover, ascent), RTH `Running` (deceleration, translation, landing) | **on** (with the mission's actuator flag, which is true in these phases) | off (observer: `Ascent`, flaps retracted; RTH stabilising: `Upright`) |
| Bellyflop `Running` (the kick, at most `kKickTimeLimitNs` + guard) | **on** (`kTvcDuringBellyflopTransition = true`, user decision 2026-11-21), thrust capped at `kKickThrustLimitN` | **off** (flaps `Ascent`, retracted) |
| Bellyflop after the kick (`Finished`) or past the guard | **off** | **on** |
| Freefall (any mode) | **off** | **on** |
| Any mission `Idle`, `Initialisation`, `Startup`, `Finished` (boot, countdown, landed) | off | off (except bellyflop after the kick, and freefall) |

- **Exclusive in the bellyflop:** `bellyflopFlaps = freefall || (bellyflop && !tvcEnabled)`, so the TVC set and the flap attitude control never drive the vehicle at the same time (they act on different actuators anyway: motors and vanes vs flaps).
- **Thrust cap:** `ControlPhase::tvcThrustCapN` = `kKickThrustLimitN` in the bellyflop, otherwise `kNoTvcThrustCapN` (none). `applyControlPhase()` applies it every tick with `StarshipTVC::thrustCap()`; `resetMissionsToBootState()` clears it. The cap scales the commanded thrust vector (direction kept), the motor output scale stays relative to the 14 N limit. Bench: 11.0 N maximum in the kick, 14.0 N again in RTH.
- **Kick guard:** `ControlPhaseTracker::update(..., nowNs)` records the switch time of the bellyflop; past `kKickTimeLimitNs + kKickGuardMarginNs` (or for a bellyflop it never saw being switched to) the TVC set is off regardless of the mission mode.
- **Neutral TVC output when the TVC set is off:** a disabled `ControlAttitudeTvc` publishes `[0,0,0.1,0]` (0.1 N axial, `control_attitude_tvc.cpp`), never a stale deflection. With the TVC actuators enabled (countdown Startup, the ≤ 100 ms between the kick end and the freefall switch) that is vanes centred and the motors at the idle floor (`0.1/14·limit + 0.04`). With the TVC actuators disabled (freefall, Idle, shutdown) the servo PWM and the motors are off (`starship_tvc.cpp`).
- **Reset:** the TVC set is reset (`resetControlState()`: integrals, delta-time reference, position setpoint held at the measured position until the next mission setpoint, ExVectrControl 6e07d38) whenever it is enabled after being off in the previous tick, or the active mission was switched (`switchMissionTo` counter) while it stays on. In the configured chain: waypoint Startup→Running, waypoint→bellyflop (kick, stays on), RTH start after freefall, RTH start from an abort (also waypoint→RTH and bellyflop kick→RTH, where it stays on). Both controllers also zero their integrals every run while disabled (unchanged). The flap controller has no integrator or filter state (P + rate only), so it needs no reset.
- The flap controller is not paused in the powered phases: its mode is selected by `FlapModeObserverTask` (§8), and RTH uses `Upright` during stabilisation.
- A log line `Control phase: TVC controllers on/off [(reset)], bellyflop flap control on/off` is written on every change or reset.

Switching, after the edge handling, is decided by `Safety::MissionChain::step()` (`src/safety/mission_chain.*`, pure, host-tested in `test/test_mission_chain`). A chain is **pending** only from an accepted `MissionBegin` until RTH ends or a reset (`SystemReset`, or a `MissionAbort` that resets and stops on the ground):

| Condition | Action |
|---|---|
| active == RTH and `missionEnd()` (pending or not; also the steady state after boot and after SystemReset) | `enableMotors(false)`, flaps off, `vehicleShutdownControl(true)`; the chain is no longer pending |
| chain not pending | nothing |
| pending, `missionEnd()` and mode `Idle` (= the mission was reset) | nothing; the chain is cancelled |
| pending, mode `Idle` but not `missionEnd()` | nothing; the chain stays pending. Defence in depth only: the old bellyflop `taskInit` override produced this state on its first run after boot; it was removed on 2026-11-20, and no mission reports it today. |
| pending, `missionEnd()`, not Idle, the active mission ended by a failed cancel-flag waypoint (`activeCancelled`, the hover check) | RTH: `switchMissionTo(&defaultMission_)` (`redirect() = CancelledWaypoint`; owner 2026-11-21) |
| pending, `missionEnd()`, not Idle, `nextMission()` is the bellyflop and altitude < `kMinKickAltitudeM` (90 m) or not finite | RTH: `switchMissionTo(&defaultMission_)` (`redirect() = BelowKickAltitude`; owner 2026-11-21) |
| pending, `missionEnd()`, not Idle, `nextMission()` ∈ {RTH, nullptr} | `missionSelection_ = 0`, `switchMissionTo(&defaultMission_)` |
| pending, `missionEnd()`, not Idle, `nextMission()` is another mission | `switchMissionTo(nextMission())` |

The two RTH rows only change the Waypoint → Bellyflop step; bellyflop → freefall → RTH and MissionAbort (which does not go through `step()`) are unchanged. The RTH start is the same hot start as after an abort (§5). Host tests (`test/test_mission_chain`): `test_kick_gate_step_rules` (cancel → RTH at 2 m and at 120 m; 89.99 / 50 / 0 / −1 m / NaN → RTH; 90 / 117 / 120 / 150 m → bellyflop; no gate for other steps), `test_hover_check_failure_goes_to_rth` (hover check fails at 2 m: no bellyflop or freefall begin, RTH runs to its end), `test_ascent_timeout_low_goes_to_rth` (50 m), `test_ascent_end_at_apogee_goes_to_bellyflop` (120 m, full chain unchanged), `test_abort_low_unchanged_by_kick_gate`. Bench (simulation, 2026-11-21): the nominal chain at 120 m still went waypoint → bellyflop → freefall → RTH → landing, no Failure bits. The two RTH paths themselves were not reproduced on the bench (they need a hover failure or a low ascent end in the simulator).

Before 2026-11-20 the switch used `missionEnd()` alone. Because `resetMission()` sets it, a SystemReset (which reset only the active mission and left it active) and a `MissionAbort` during the countdown made the next tick start the next mission: waypoint → bellyflop → freefall → RTH hot start (bench, simulation).

`resetMissionsToBootState()` (SystemReset): chain reset, `resetMission()` on all four mission objects, RTH (Idle) active with the position controller subscribed to its setpoint topic, `missionSelection_ = 0`, `missionPhase_ = Idle` (the phase mapping then shows `Completed` once initialised, as after boot), mission zeroing off, `controlAttitudeTvc` and `controlPositionStandard` disabled and the phase tracker reset (the next enable resets the controllers), flaps off, `lastMissionMode_ = Idle` (no edge pending). Not reset (not mission state): the flap observer keeps its last `ControlAttitudeBellyFlopSetting` until the next Running mission; the flap telemetry shows the last angles while the flap outputs are disabled.

`switchMissionTo(m, t0 = 0)`: `reset()` on the old mission, `reset()` on the new one, `m->beginMission(t0)`, resubscribe the position controller's setpoint to `m`, count the switch (for the controller reset).

Controller state in the configured chain (phase table; the same on the first and every later run):

| Step | TVC set | Bellyflop set |
|---|---|---|
| Waypoint Init / Startup | off | off |
| Waypoint Running | **on, reset** | off |
| Bellyflop kick (Running, ≤ 1 s) | **on, reset**, thrust ≤ 11 N | off |
| Bellyflop Finished (≤ 100 ms) | off | on |
| Freefall | off | on |
| RTH Running | **on, reset** | off |
| RTH Finished | off | off |

Before 2026-11-20 the enable was evaluated on mission-mode edges with a shared `lastMissionMode_`, which made the first bellyflop after boot differ from later ones and left RTH without controllers after an abort or mag timeout from freefall (both Fixed, Known issues).

#### MissionAbort (user decision 2026-11-20)

Two kinds of stop: a **failsafe** (a latched `Failure`, for example an IMU failure) cuts everything and the vehicle falls, unchanged. An **abort** (SA released, T9b-4; was the `MissionAbort` telecommand, and later the abort outcomes of the failure policy) means "stop and return home". The decision is `Safety::decideAbort` (`src/safety/mission_abort.*`, pure, host-tested), accepted only while the vehicle mode is `Running`. Since T9b-4 it is evaluated **every 10 Hz tick while SA is released** and a chain is pending or the mode is Running (idempotent: RTH is started once and then kept; reset and stop runs once, then the vehicle is no longer Running):

| Active mission / mode | Action |
|---|---|
| vehicle not `Running` | ignored |
| any mission `Idle` | reset and stop |
| Waypoint `Initialisation` or `Startup` (countdown) | **reset and stop** |
| Waypoint `Running` (any phase after the countdown: hover, ascent) or `Finished` | **start RTH now** (hot start) |
| Bellyflop, Freefall (any mode except `Idle`) | **start RTH now** (hot start) |
| RTH `Running` | keep RTH (no restart) |
| RTH `Finished` (landed) | reset and stop |

- **On the ground** means **before liftoff** and is decided by the mission mode alone (user decision 2026-11-20, review fix): the countdown phases above, any `Idle` mission, RTH landed. In the countdown the TVC controllers are off and the motors at most at idle, so the vehicle cannot fly there. There is **no altitude/speed ground check** any more: it was removed because an abort just after liftoff (waypoint `Running`, 0.3 m, 0.2 m/s) would have reset the mission and cut the motors, dropping the vehicle, instead of starting RTH. Consequence: an abort right at T0 with the vehicle still on the pad starts RTH too; it climbs to the home point (0, 0, 3 m), descends and shuts down. Bench (simulation, 2026-11-20, feature/review-fixes): abort in Init and in Startup → `on the ground: reset and stop`, Ready, mission Idle within 0.2 s; abort in hover at 0.84 m, 0.89 m/s → `in flight: start RTH`, RTH ran (phase Deceleration → Landing), landed 16 s later, auto-disarm and RTH-end shutdown. A `Finished` waypoint, bellyflop or freefall mission lasts at most one safety tick before the chain switches on, so it is in the air and starts RTH.
- **Calibration abort of the countdown (2026-11-21):** the gyro run that closes the waypoint countdown (Initialisation window, deadline end − 1 s) re-checks all three sensor calibrations; if one is bad (or the run failed) the mission is aborted before liftoff with the same **reset and stop** (shared method `resetAndStopOnGround`, `Safety::decideAbort` is not consulted: the countdown is on the ground by construction) and the log says `MISSION ABORTED before liftoff: calibration invalid (<reason>)`, for example `gyro: last run failed, accel: ok, mag: ok`. Simulation mode is exempt. Details: [calibration.md](calibration.md). A manual `MissionAbort` in the countdown cancels the gyro run.
- **Reset and stop:** TVC actuators and motors off, shutdown, chain reset, `resetMission()` of the active mission (it stays active, Idle; the phase still reads Hover for the waypoint mission), the sensor states are re-initialised (so the vehicle goes Startup → Ready) **without clearing any failure flag or latch** (T9b-4: only the SE short press clears latches; the vehicle is Running here, so no failure flag is set anyway). Nothing follows, and the interlock needs a new SD toggle.
- **Start RTH:** `switchMissionTo(&defaultMission_)`; the chain stays pending until RTH ends (RTH-end shutdown and auto-disarm as after a normal RTH); the TVC set is enabled and reset in the same tick.
- Log: `SA released: <action> (alt <m> m, speed <m/s> m/s)` (only when something is done, not for the idempotent repeats)
- *(previously `Mission abort telecommand: ...`)*  (altitude and speed are logged for information only; they do not enter the decision).

Other ways a mission gets switched:

| Source | Condition | Action |
|---|---|---|
| SA go edge (T9b-4, `applyRcCommands` -> `beginMissionFromGo`; was `Telecommand_MissionBegin`) | observed NOT_GO -> GO of the active source, vehicle mode `Ready`, no latched failsafe, no zeroing window and a valid zero, SC valid and equal to the applied mode, SB = 2 (the only implemented profile), no calibration session or gyro run, then all three calibrations valid (real and simulation alike, [calibration.md](calibration.md)); the interlock is **not** a precondition | `switchMissionTo(&missionWaypointTask, -15 s)` (fixed countdown, `Safety::kGoCountdownParamInt`), vehicle mode → `Running`, `enableMotors(true)` **in simulation and real mode alike**. Init lasts until −10 s, Startup until 0 s. |
| SA released (T9b-4; was `Telecommand_MissionAbort`) | chain pending or vehicle mode `Running` | In flight: RTH hot start from any phase; on the ground: reset and stop (table above); every tick, idempotent. |
| SE short press (T9b-4; was `Telecommand_SystemReset`) | on the ground, interlock closed (D12) | `resetMissionsToBootState()` plus the system state reset (overview.md), latches cleared. |
| Mag data older than 2.5 s | vehicle mode `Running` and mission mode `Running` | switch to RTH (the phase table enables and resets the TVC set; unchanged trigger, not yet the abort decision) |
| SB (T9b-4; was `Telecommand_MissionSelect`, a no-op) | read at the go edge | profile 2 starts the configured chain; SB 0 and 1 refuse the go until T9b-8. |

`missionSelection_` is written into a **local copy** of `MissionState` at `:619` that is then discarded, so the telemetry `missionIndex` never carries it. The latched SB profile is reported in `VEHICLE_STATUS.mission_profile` (T9b-4).

### 6.3 Actuator gating (`vehicleShutdownControl`, `:1187-1203`), every tick

- `shutdown` (not ready, a failure latched, or RTH ended): TVC actuators off, flaps off, simulator TVC off.
- Otherwise: flaps on if the mission is Freefall, else `mission_->getActuatorsEnabled()`. TVC actuators and simulator TVC follow `getActuatorsEnabled()`. Simulator flaps follow the flap state.
- So the per-mode flap enables in §6.2 are overridden every tick, and the effective rule is "Freefall: on, else mission actuators".
- Motors additionally need `enableMotors_`, which is set at the go and cleared at RTH end and at the reset. Within `StarshipTVC` they also need `enableActuators_` and an idle actuator test. **There is no arm state or power limit any more (T9b-4: the motors run at full power when the interlock is open)**, see [hardware-and-drivers.md](hardware-and-drivers.md#3-starshiptvc). **Since T9b-2 (2026-11-22) every mission also needs the hard motor interlock open** (observed Pocket SD off -> on, ENABLED state at most 1 s old, 100 Hz watchdog; no mission, RTH or failure handler can bypass it; it never reads the simulation flag). A mission chain that ends (RTH landed or cancelled) re-arms the interlock: the next flight needs a new SD toggle. If SD goes down or the RC stops for over 1 s in flight the motors stop and the vehicle falls (a 1 to 2 s gap reopens by itself).

Kinematic safety (only for missions with `disableKinematicSafety_ == false`, which in practice means Waypoint) applies when mode ≠ Idle:

| Mission mode | Horizontal distance limit | Tilt limit |
|---|---|---|
| Init or Startup ("stationary") | 2 m | 20° |
| Running | 100 m | 120° |

Distance is `position.magnitude(3,5)`, a half-open range, so only **Px, Py** count. There is **no altitude bound**. Tilt is the angle between `q.rotate(Z)` and Z. A violation latches `VehicleMode_Failure`, which cuts all actuators.

---

## 7. Configured flight profile (`main.cpp:552-558`)

```cpp
missionWaypointTask.addWaypoint({0, 0, 0.3}, 0,   3, 1, 5,   0 * Core::SECONDS);
missionWaypointTask.addWaypoint({0, 0, 1},   0.5, 3, 1, 0.5, 5 * Core::SECONDS, 10 * Core::SECONDS, true);
missionWaypointTask.addWaypoint({0, 0, 120}, 7,   3, 2, 3,   5 * Core::SECONDS);
```

| # | position (m) | velocity | startAccel | stopAccel | threshold | loiter | timeLimit | cancel | Effect |
|---|---|---|---|---|---|---|---|---|---|
| 0 | (0,0,0.3) | 0 (jump) | 3 | 1 | 5 m | 0 | default = 0.3/0 × 1.5 s = **+inf → int64 (UB)** | no | Reached on the first Running tick (vehicle within 5 m, loiter 0). Net effect: the setpoint jumps to 0.3 m altitude. |
| 1 | (0,0,1) | 0.5 m/s | 3 | 1 (effective 0.5 m/s² braking) | 0.5 m | 5 s | 10 s | **yes** | Slow lift to 1 m and a 5 s hover check. If 1 m ± 0.5 is not reached in 10 s, the mission ends (`Cancelled`) and the chain goes to **RTH** (since 2026-11-21; before: Bellyflop at about 1 m). |
| 2 | (0,0,120) | 7 m/s | 3 | 2 (effective 1 m/s²) | 3 m | 5 s | default = 119/7 × 1.5 = **25.5 s** | no | Climb to 120 m and hover 5 s within 3 m. Reached or timed out, the mission ends → Bellyflop → Freefall → RTH; if it ends below 90 m (`kMinKickAltitudeM`), → RTH directly. |

During the climb, `MissionPhase` = `Ascent` (index 2). The commented-out alternative profile (`:560-567`) is a low-altitude hop to (0,−5,1.5) that ends directly in RTH.

Nominal timeline (MissionBegin with `paramInt = P < −10`):

| Mission time | Mission / mode | What happens |
|---|---|---|
| P … −10 s | Waypoint `Initialisation` | Estimator zeroing on, controllers off, TVC actuators off. |
| −10 … 0 s | Waypoint `Startup` | TVC actuators on. 4 s TVC servo test (2 s circle + 2 s twist). Then motors at idle: the controller is disabled, so it publishes 0.1 N → `0.1/14·limit + 0.04`. A 2 s motor-start delay applies only to the first enable after boot. |
| 0 s → | Waypoint `Running` | Controllers enabled at the edge; WP0 → WP1 → WP2. |
| apogee + 5 s | Bellyflop | ≤ 1 s pitch-over, nose south. |
| — | Freefall | TVC off, flaps on (§8). |
| `Pz < 0.171·Vz² + 20` | RTH | Stabilise → straight line to (0,0,3) at ≤ 2 m/s → descend at 0.2 m/s → land → shutdown and auto-disarm (`:1148`). |

If `P > −10`, Init is skipped, and if `P ≥ 0`, Startup lasts one tick. The 10 Hz safety task can then miss those edges, so zeroing or the actuator test is skipped, or the 4 s TVC test (motors gated off) overlaps take-off.

---

## 8. `FlapModeObserverTask` (`main.cpp:1258-1335`)

10 Hz. Publishes a `ControlAttitudeBellyFlopSetting` on `bellyFlopControlTopic`, which `controlAttitudeFlaps` consumes. It updates the setting only when vehicle mode == `Running` **and** the active mission's mode == `Running`. Otherwise it republishes the last setting unchanged.

| Active mission | Condition | `bellyFlopMode` | Other fields |
|---|---|---|---|
| Freefall or Bellyflop (`bellyflopFlapsActive()`, phase table; bellyflop since 2026-11-20) | always, **default build (since S3)** | `Descent` | Only the mode is set here; the heading setpoint and the pitch split come from `DescentGuidance`, which runs in the 50 Hz `ControlAttitudeFlaps` task (section 8.1). |
| Freefall or Bellyflop, **legacy build** (`-DDESCENT_LEGACY_STABILIZE=1`, one release) | always | `Stabilize` | `pitchAngle = 0` each tick, then: |
| | `Pz > 100` | | `azimuth = −atan2(Py, −Px)` (back toward the origin), `pitch = (clamp(−(dXY − 1), −40, 0) + 20)°`, so +20° near the origin down to −20° at ≥ 41 m off. |
| | `Pz < getStopAltitude() + 20` (= stopDist + 40) | | `pitch = 30°`. The azimuth keeps its last value. |
| | otherwise | | pitch 0, azimuth keeps its last value |
| RTH | `stabilising()` | `Upright` | (controller: top flaps 0 = extended, bottom 90° = retracted) |
| Waypoint | `defaultMission_.stabilising()` (**checks RTH's state, not Waypoint's**) | `Acsent` | `extentFlapsOnAscent = EXTEND_FLAPS_ASCENT` (false) |
| anything else (RTH translate/descend, Waypoint after any RTH run) | | `Acsent` | `extentFlapsOnAscent = false` |

In `Acsent` mode the flap controller commands 90° (retracted) unless extend is set. The flap angle convention and servo mapping are in [hardware-and-drivers.md](hardware-and-drivers.md#4-starshipflaps).

### 8.1 Descent control wiring (S3, `feature/descent-s3`; design: workspace `docs/descent-control-design.md`)

- **Target (every observer tick, `descentTargetTopic`):** `Safety::makeDescentTarget` (`src/safety/descent_target.hpp`): `DescentTarget{x = 0, y = 0 (home XY = the position reference, the world origin), endAltitude = missionFreefallTask.getStopAltitude() (27.6 m fallback if not finite), valid = Descent mode selected AND position usable}`. Position usable (`Safety::descentPositionUsable`): estimate finite, and in real mode GNSS `positionValid` with a sample at most 0.5 s old, filtered by `Safety::DescentGnssHysteresis` (invalid only after more than 0.3 s of loss, valid again after 0.5 s of good GNSS, so a 100 ms flicker never toggles the guidance; finite check immediate); in simulation mode the (absent) GNSS is not required and nothing is filtered.
- **Guidance** runs in `ControlAttitudeFlaps` (50 Hz, measured `dt`): init on the first tick in Descent (heading setpoint = current nose heading, 1.5 s entry hold with split -20 deg, flaps deploy from the kick), `DescentGuidance::step` every tick, additionally invalid when the position estimate is older than 0.5 s. Details: libs/ExVectrControl.md.
- **Leaving Descent** (flip = Freefall mission end -> `bellyflopFlaps` off -> `Acsent`/`Upright`, abort/RTH, failure, system reset): the observer stops selecting Descent, the guidance memory is reset on the next flap tick, the target is published invalid. The kick, the freefall stop trigger and the abort/RTH/failure behaviour are unchanged (T9b owns them).
- **Failure behaviour (design section 7):** GNSS invalid/silent, stale or non-finite position -> target invalid -> heading held, glide 0, split slews to 0 (neutral 30/78, vertical fall, about -0.24 m/s of glide) until the stop-altitude guard fires and the flip follows (then the no-position arrest, T9b). GNSS/mag loss before the bellyflop is the T9b no-position RTH, not this code.
- **Telemetry (no schema change):** `setpoint_q` (MISSION / AIR_MISSION) in Descent is the belly-down attitude with the **heading setpoint** of the guidance (`flapPointingSetpoint(headingSp, pitch 0, q)`: body +Z on the horizon at the setpoint azimuth, roll follows the real vehicle). The **pitch split is not transmitted**; the GS derives it from the actual flap angles in `ACTUATORS` / `AIR_ACTUATORS` (split = common-mode of the top pair against 30 deg, then the bottom pair against 78 deg; neutral 30/78 means split 0). A dedicated field is a later, cross-project change (S4). Legacy build: `setpoint_q` from the Stabilize azimuth/pitch as before.
- **Compile-time switch:** `-DDESCENT_LEGACY_STABILIZE=1` (e.g. `PLATFORMIO_BUILD_FLAGS`) selects the legacy Stabilize observer path (azimuth/pitch semantics unchanged); default 0 = Descent. Remove after one release.

**Owner's SIMULATION BENCH PROCEDURE for S3 (flight-safety item: bench before any flight).** Motors electrically disconnected (ESCs unpowered) and no props for every item; the simulation drives the motors exactly as real mode does. The flaps must be visible and free to move on the bench (the servos run the real commands). GS in master with the virtual radio, or the Pocket; Router and ship node on Serial4; Teensy on USB. Wind 0 and gusts 0 (the GS wind panel; they reset to 0 on every simulation switch). Watch `POSE` (position, `vel`), `MISSION.setpoint_q` (heading setpoint), `ACTUATORS` (true flap angles), `VEHICLE_STATUS`, the console.
  1. **Flash the default build** (`pio run -t upload`, no extra flags). Boot, wait for Ready, enter simulation mode (SC 0 -> 1 -> 0), SB 2, SD up (interlock open, motors disconnected), SA up: go, 15 s countdown, the simulated climb, bellyflop kick, freefall.
  2. **Full chain, wind 0:** let the whole flight run to the flip and RTH. Expected: after the kick (flaps 90 deg) the flaps deploy within about 1 s to top about 40 to 50 deg, bottom about 80 to 86 deg (entry hold, 1.5 s), then settle near **30/78** with small differentials; the heading setpoint (from `setpoint_q`) turns toward home if the ship is off home and then stays constant; the ship glides toward home (`vel` horizontal up to about 4.5 m/s, less than 1.5 m/s in the last 20 m above the stop altitude); at the freefall stop trigger the horizontal distance to home in `POSE` is **< 3 m** (the offset at the flip start is what the run produced; with no wind the climb offset is small, create 5 to 10 m by setting the wind to 2 to 3 m/s for the climb and back to 0 at about 100 m, and write the offset down). Then flip and RTH as before.
  3. **What to watch (must hold):** no heading oscillation late (the heading in `POSE` stays within 10 deg of the setpoint in the last 5 s before the flip, except a single turn); the flaps never alternate: the left/right flaps of a pair do not swing against each other by more than about 10 deg after the first 3 s, no limit cycle with a period of about 2 s; belly angle (body +X against up) stays below 35 deg after the first 2 s (< 15 deg expected); all four flaps stay within 15..90 deg; no console warnings about NaN.
  4. **Failure behaviour (optional, in simulation mode):** GNSS loss is not simulated (the simulation does not need the GNSS), so test the real-mode logic only on the bench with the real GNSS: a fix lost in real mode cannot be exercised without flight. The host test `test/test_descent_wiring` covers it (a straight fall).
  5. **Legacy comparison (optional):** reflash with `PLATFORMIO_BUILD_FLAGS=-DDESCENT_LEGACY_STABILIZE=1` and repeat item 2: the old behaviour (fly past home, flaps alternating 20..70 deg) comes back; the telemetry `setpoint_q` then follows azimuth/pitch.
  - **Abort criteria (stop the run, SA down or the GS FAILSAFE, and report with the log):** any flap angle outside 15..90 deg or a flap that does not move; belly angle above 45 deg after 3 s; the ship glides AWAY from home for more than 3 s above 60 m; a heading oscillation (swings above 10 deg both ways) or alternating flaps for more than 2 s; miss at the flip start above 5 m in calm air; a console line about invalid inputs while the position display looks fine; vehicle mode Failure.
  - Do not fly this before the pivot test in an airflow (design section 9 item 1): the model (fold shield 0.7, aero factor 4.64) is not validated against the ship.


---

## Known issues & limitations

| Severity | Location | Issue | Impact |
|---|---|---|---|
| High | `mission_rth.hpp:400-415` | The landing detector fires when the vehicle stays > 0.2 m above the Z setpoint for 2 s, at any altitude. | A poor vertical-tracking period during descent (gust, low thrust margin, estimator jump) is declared a landing, the actuators are cut and the vehicle falls. |
| High | `main.cpp:933`, `:955`, `:1971-1972` | `groundTopicSwitch` is not subscribed at boot, only in the simulation toggle. | The RTH rangefinder landing trigger (`mission_rth.hpp:417`) never fires in real flight. Only the tracking-error trigger works. |
| Info | main.cpp `applyRcCommands`, `Safety::kGoCountdownParamInt` | The go countdown is the fixed **15 s** (D8, owner decision 2026-11-22); the countdown gyro run uses a 3 s still window so it fits the 5 s Initialisation phase (calibration.md). | None. |
| Fixed T9b-4 | main.cpp | (was High, the arm state is removed) MissionBegin did not require `vehicleArmed_`. Original text: MissionBegin does not require `vehicleArmed_`. Arming only scales the motor power limit (0.1 disarmed). | A disarmed vehicle can start a mission with motors running at 10 % authority. |
| High | main.cpp `checkForFailures`, `vehicleIsReady` | Control-source loss (T9b-3: accepted RC_STATE of the latched source older than 2 s while committed, overview.md) or any sensor older than 0.5 s (GNSS, gyro, baro, posKF, attKF) latches `Failure`, and `vehicleShutdownControl(true)` cuts all actuators. | The response to link loss or a sensor hiccup in flight is flight termination, not RTH. Failure is latched until SystemReset. |
| High | `mission_freefall.hpp:155-159`, main.cpp:445 | Stop altitude = `0.5·Vz²/(T/m − g) + 20`. It assumes full 14 N immediately, ignores flip-up time and drag changes, and is sign-blind. It goes negative or infinite if `T/m ≤ g`. | Margin is only the fixed 20 m. A thrust-to-weight change (mass, battery sag) silently breaks it. |
| Medium | `main.cpp:1119`, `:1134` | Out-of-bounds uses `magnitude(3,5)`, which covers elements 3 and 4 only. | No altitude limit; the vehicle may climb without bound. The horizontal-only limit is 100 m. |
| Medium | `mission_rth.hpp:292-294` | `travelDistance / travelDistanceMag` with no zero guard. | If the setpoint converges exactly onto home before the vehicle is within 0.5 m (or starts there), the velocity setpoint becomes NaN. |
| Medium | `mission_rth.hpp:277-280`, `:284-319` | The Ascent state is a no-op, and Translation is a straight 3-D line to (0,0,3). | No climb to a safe transit altitude. From freefall exit (tens of metres) the path descends diagonally at ≤ 2 m/s, which is slow and low over the terrain on the way. |
| Medium | `mission_waypoint.hpp:119`, `:125`, main.cpp:552 | Default time limit with `velocity = 0` gives `x/0 → inf → int64` (undefined behaviour). | WP0 is harmless today only because it is reached instantly (threshold 5 m). |
| Medium | `mission_waypoint.hpp:424`, `mission_rth.hpp:308` | Braking cap `sqrt(d·a)` gives an effective deceleration of `a/2`. | The configured stopAccel values are twice the real braking. Arrival is gentler and slower than configured. |
| Medium | `mission_waypoint.hpp:380-420` | The time limit applies only until the waypoint is first reached. Leaving the threshold afterwards has no timeout. | The mission can hang forever on overshoot or drift. |
| Medium | `mission_bellyflop.hpp:102`, `:87` | The pitch-over direction is hard-coded to nose-south (ref −X). The position controller stays subscribed to a never-published setpoint topic. | Bellyflop heading ignores wind and home. The freefall then commands the azimuth toward home, so a vehicle that drifted south of home starts the freefall up to 180° off its heading setpoint (bench 2026-11-21: 174°, turned in about 5 s). The position-loop thrust during the manoeuvre is whatever stale input it holds (verify in ExVectrControl). |
| Medium | main.cpp:1316-1321 | Waypoint flap branch tests `defaultMission_.stabilising()`. RTH's `runningState_` persists after a run. | Wrong branch after any RTH run. Harmless only while `EXTEND_FLAPS_ASCENT == false`. |
| Medium | main.cpp:874-880, `mission_waypoint.hpp:249`, `:292` | The MissionBegin `paramInt` is the initial mission time, not "seconds in the future" as `telecommand.hpp:159` says. Values > −10 skip Init and Startup. | Operator error can skip estimator zeroing or overlap the TVC test with take-off. |
| Medium | `src/safety/bellyflop_kick.hpp`, `mission_bellyflop.hpp` | **Bellyflop kick tuned only in the simulator** (2026-11-21). It was tuned with the old simulator (no aerodynamic pitch damping or pitch-over moment, 9000°/s equivalent vane servo); the inertia (0.044 kg m², measured) and the 3° vane authority are model values. With the redesigned simulator (`feature/sim-aero`) the kick still ends by the angle trigger (0.91 s, 14.8° remaining, 93°/s). With the real aerodynamics the angle trigger may be reached earlier (pitch-over moment, as in the old real flights: tilt 8° → 103° in 0.9 s with TVC thrust 0) or the rotation may be damped so that the 1 s limit ends the kick. The old 50° / 150°/s triggers overshot to 140° tilt in the simulator. The TVC controller propagates its setpoint with the measured body rate between mission writes (≤ 20 ms, ≤ 3° at 134°/s), which slightly softens the kick. | Review the kick in the first flight data (tilt, rate, thrust, end trigger and time in the log line `Bellyflop kick finished (...)`); tune the constants in one place. |
| High | TVC attitude controller (`ControlAttitudeTvc` gains), RTH flip | **The TVC attitude loop is tuned for an unrealistically fast vane model** (the old actual-thrust model slewed 600°/s of thrust angle = 9000°/s of servo). With the 9 g servo speed (300°/s servo = 20°/s thrust angle, `feature/sim-aero` 2026-11-21) the simulated RTH flip from the freefall overshoots and limit-cycles: from belly-down at 27 m (7 m/s down) tilt 71° → 27° in 0.5 s at 177-200°/s, then 27-105° oscillation until ground contact at 7 m/s and 65° tilt (bench, simulation, full chain). Same with the aerodynamic calibration set to 1 (13 m/s fall, flip from 44 m): 51° → 5.5° then tumbling; so the cause is mainly the loop against the slower actuator, made worse by aerodynamic moments up to 0.43 N m against 0.26 N m of TVC authority. Hover and ascent stay stable (no disturbance). This matches the reviewer's note that the real flip never reached upright. | Retune the TVC attitude gains (rate limit / lower P, more rate damping) for the real servo speed in simulation, then check the flip start altitude (the freefall stop altitude ignores the flip time). Not changed in `feature/sim-aero` (owner: no gain changes in that task). |
| Medium | Flap controller (`ControlAttitudeFlaps`) in freefall | **Heading sign fixed in the simulator 2026-11-21** (`feature/sim-fixes-3`): the fold shield (`flapFoldShieldFactor` 0.7, owner-approved) is on by default, so all three axis signs match the controller at 50/40° over AoA −20..+20° (ExVectrDSP Known issues). Bench full chain (simulation, no wind, no faults): after the kick the nose pointed South (heading −174°) while the freefall setpoint pointed home (North, 0..+10°); the nose turned toward the setpoint (error 174° → 0° in about 5 s, 20-45°/s), then held within a yaw/roll limit cycle of about ±34° (period about 4 s, belly up to 31°), as the harness replica does; fall speed 6.2-6.7 m/s; pitch held at −4..−10° against a +14..+19° setpoint above 100 m (the 50/40° neutral angles do not trim belly-down). | Owner: bench pivot test in an airflow to confirm the fold shield; retune the flap controller (yaw/roll limit cycle, pitch trim). Controller gains and neutral angles were not changed. |
| ~~Medium~~ Fixed | `src/safety/mission_abort.*` | The abort ground check used the estimated altitude and speed (0.5 m, 0.5 m/s): a vehicle hovering below 0.5 m and slower than 0.5 m/s (for example just after liftoff) was treated as on the ground, reset and dropped; baro noise on the pad could turn a stop into an RTH. | Fixed 2026-11-20 (feature/review-fixes): "on the ground" is decided by mission mode only (before liftoff); waypoint `Running` always starts RTH. An abort at T0 on the pad now starts RTH (climb to 3 m, land). |
| Low | main.cpp `checkForFailures` (mag timeout) | The magnetometer-timeout switch to RTH still uses its own trigger (mission mode Running), not `Safety::decideAbort`. | Unchanged behaviour. The failure policy (T9b) should route it through the abort decision. |
| Low | `mission_waypoint.hpp:340-370` | Waypoint never enters `Finished` when it ends. Switching relies on `missionEnd_` alone. | Cosmetic since 2026-11-20 (the controllers no longer depend on mode edges). |
| Low | main.cpp:619 | `missionIndex` is set on a discarded local copy. | Telemetry `missionIndex` is always 0. |
| Low | main.cpp:588-600 | No phase for the RTH Ascent tick or for RTH `Idle` before the first mission (reads `Completed`). Waypoint Init and Startup report `Hover`. | The ground display can mislead. |
| Low | `mission_waypoint.hpp:76` | Copy-pasted task name ("Mission RTH"). The bellyflop task is named "Mission Bellyflop" since 2026-11-21. | Confusing scheduler and debug output. |
| Low | `mission_waypoint.hpp:55`, `:39`, comments | Comments say µs and seconds where values are ns. "Set the mission end to false" sits next to `= true`. | Misleading when editing. |
| Low | `mission_abstract.hpp:19-25` | `MissionType` (Remote, FlapDescent) is unused and unimplemented. MissionSelect is a no-op. | Dead API surface. |
| Low | `mission_bellyflop.hpp:112` | `LOG_MSG` every 10 ms during bellyflop. | Serial load in a time-critical phase. |
| Info | main.cpp `handleTelecommands`, `vehicleIsReady` | After a mission, the vehicle mode stays `Running` (only Startup can return to Ready). | A new MissionBegin needs SystemReset first (since 2026-11-20 it no longer re-zeroes; the waypoint mission's Initialisation still does). |
| Fixed 2026-11-21 | `src/safety/mission_chain.*`, `mission_waypoint.hpp` (`endReason`), main.cpp `handleVehicleMissionControl` | (was Critical) WP1 has `cancelIfNotReachable = true`, but a cancelled waypoint mission still ended through `nextMission()` = **Bellyflop**: a failed 1 m hover check pitched the vehicle 90° at about 1 m (owner wind test 2026-11-21: hover check failed 3 m from the waypoint at about 2 m height, the vehicle went into the bellyflop). Now a cancelled waypoint mission goes to RTH, and the chain never starts the bellyflop below `kMinKickAltitudeM` = 90 m (owner decisions, §1). Host-tested; the nominal chain was re-run on the bench. | |
| Fixed 2026-11-20 | `mission_bellyflop.hpp` (`taskInit`), main.cpp `handleVehicleMissionControl` | (was Critical) **The first bellyflop after boot differed from later ones.** The task is paused from construction, so the scheduler ran its `taskInit` after `beginMission`; it set mode `Idle`, and the shared mode-edge handling disabled the TVC controllers for that bellyflop only (later runs: no edge, controllers stayed on). Bench before the fix (simulation, d29ec72): first run mission mode Idle, thrust 0.10 N and TVC 0 after a single 14 N kick of < 100 ms, tilt 4° → 40° in 1.01 s (time limit, remaining angle 55.8°); run after SystemReset mission mode Running, 14 N with 0.052 rad TVC for 0.4 s, tilt → 104°, angle trigger at 0.54 s. Fixed by removing the `taskInit` override and the explicit phase table (§6.2, `test_bellyflop_first_and_later_runs_identical`, `test_root_cause_bellyflop_first_run_differs_legacy`). The old firmware's logs do not show this bug (mission mode stays Running there). | |
| Fixed 2026-11-20 | main.cpp mode-edge controller enable | (was Critical) An abort or mag-timeout switch to RTH while Freefall was Running was Running→Running, so no edge fired and RTH flew with the TVC controllers disabled (vehicle falls). Now the phase table enables and resets them in the same tick. Bench (simulation): abort in freefall at 72.8 m, tilt 80°, 16 m/s down → TVC on (reset) at once, upright within 1 s, landed and auto-disarmed. | |
| Fixed 2026-11-20 | main.cpp SystemReset case, `handleVehicleMissionControl` | (was High) SystemReset during a mission continued the mission chain (waypoint → bellyflop → freefall → RTH hot start in Running while the vehicle reported Ready with TVC/flap actuators enabled). Root cause: switching on `missionEnd()` alone, which `resetMission()` sets. Fixed by `Safety::MissionChain` and `resetMissionsToBootState()`. Bench (simulation, `feature/reset-chain-gnss`): SystemReset in waypoint Init, Startup, hover, ascent, bellyflop, freefall, RTH deceleration, RTH translation and RTH landing: mission mode Idle within one tick, vehicle Ready, actuator telemetry constant and no mission start or switch for 30 s each; a full mission afterwards ran normally. | |
| Fixed 2026-11-20 | main.cpp MissionAbort case | `MissionAbort` while the mission was not Running (countdown: Init or Startup) reset the mission, and its `missionEnd()` started bellyflop, then freefall (which ends at once on the ground), then an **RTH hot start with actuators and (outside simulation) motors enabled**. Now the chain is reset and nothing follows. | |
