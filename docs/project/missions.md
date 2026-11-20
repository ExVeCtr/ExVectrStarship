# Missions

How the mission framework in `src/mission/` works, each mission's state machine, and how `src/main.cpp` sequences missions, maps them to telemetry phases and picks flap modes.

Last verified against commit: b073d07 (§1, §5, §6.2 and Known issues re-verified on `feature/reset-chain-gnss`, 2026-11-20)

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
MissionBegin TC ─► Waypoint ──end──► Bellyflop ──end──► Freefall ──end──► RTH (default) ──end──► shutdown
                       │                 │                  │
                       └──── abort TC / mag timeout (only when mode == Running) ────► RTH
```

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
3. Otherwise restart the loiter timer. If the waypoint was **never** reached and `NOW − waypointStartTime > timeLimit`: snap the setpoint to the target, then cancel (`missionEnd_`), advance to the next waypoint, or finish if this was the last one. Once a waypoint has been reached its time limit no longer applies, so an overshoot followed by drifting out can stall forever.
4. Speed profile: brake to `sqrt(|d|·stopAccel)`, otherwise ramp up by `startAccel·dt` up to `velocity`.
5. Move the setpoint position toward the target by `min(remaining, v·dt)`. Set the velocity setpoint to `unit(target − setpointPos) · v` (uses `normalize()`, which returns zero for a zero vector, so no NaN).

**Setpoint published (every tick in every mode):** `[Vx,Vy,Vz,Px,Py,Pz]` = the profiled velocity and the moving setpoint position, also copied into `missionState_.positionSetpoint`.

---

## 3. `MissionBellyflop` (`src/mission/mission_bellyflop.hpp`)

**Purpose:** a short transition that pitches the vehicle from vertical into the belly-down attitude using TVC attitude control, before freefall.

- Task: 100 Hz (`0.01 s`). The task name is the copy-pasted `"Mission Freefall"` (`:51`). The task is **paused** unless the mission is active (`:66`, `:86`, `:97`, `:143`).
- `disableKinematicSafety_ = true` (`:64`).
- `nextMission_` = `missionFreefallTask`.

| From | To | Trigger | Actions |
|---|---|---|---|
| any | `Running` | `beginMission` | actuators **on**, unpause, `controlTvc_.unsubscribeAttitudeSetpoint()` (`:87`) so the mission feeds the attitude controller directly. |
| `Running` | `Finished` | first true of: **angle < 50°**, then **\|ωy\| > 150°/s**, then **elapsed > 1 s** (`:119-132`, checked in that order) | Resubscribe TVC attitude setpoint to `controlMappingAccToAtt`, `missionEnd_ = true`, pause. `actuatorsEnabled_` stays **true**. |
| any | `Idle` | `resetMission` | actuators off, `missionEnd_ = true`, pause, resubscribe attitude setpoint, mission time 0. |
| (first run after boot) | `Idle` while running | `taskInit` (`:71-74`) | The task is paused from construction, so the scheduler runs its `taskInit` on the first unpaused tick, **after** `beginMission`, and overwrites `Running` with `Idle` (`missionEnd_` stays false). The transition still runs and ends in `Finished`. See Known issues. |

**Setpoint:** each tick, `controlTvc_.setAttitudeStateSetpoint([0,0,0, q])` where `q = Quat(axis (0,1,0), −90°)` (`:102`). With q body→ref, body +Z maps to ref −X, so the nose points **south** whatever the home position is. The position setpoint topic is **never published** by this mission. The position controller stays subscribed to it, so its last input is stale (whatever `Simple_Subscriber` holds).

**Angle metric** (`:109-110`): `angle(q*.rotate(X_ref), −Z_body)` is the angle between body −Z and ref +X, i.e. the remaining error to the setpoint attitude. The rate trigger uses the attitude topic's `ω(1)` (body pitch rate). Because of the rate trigger, the transition can end on momentum long before the attitude is reached.

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

`stopDist = 0.5·Vz² / (T/m − 9.81)` (`:157`). With T = 14 N and m = 1.1 kg: `T/m − g = 2.917 m/s²`, so the trigger altitude is `0.171·Vz² + 20` m. For example, the trigger is at 89 m when falling at 20 m/s. A drag-free fall from 120 m triggers at about 97 m and 21 m/s; drag lowers both numbers. The formula uses `Vz²`, so it ignores the sign of the velocity and also triggers while climbing fast. It also ignores the time needed to flip upright. `getStopAltitude()` returns `stopDist + stopAlt` and is used by the flap observer.

**Setpoint:** publishes `[0,0,0,0,0,0]` every tick while active. **TVC actuators are off** (`actuatorsEnabled_ = false`), but the safety task force-enables the **flaps** for this mission (`main.cpp:1195-1196`).

---

## 5. `MissionRTH` (default mission) (`src/mission/mission_rth.hpp`)

**Purpose:** fallback and recovery. It kills velocity, flies straight to `home` (`{0,0,3}`) and descends to touchdown. It runs after freefall, after an abort and after a magnetometer timeout.

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

1. `handleTelecommands()`.
2. If `!systemsInitialised()`: disable everything and return. Phase mapping and mission control are skipped.
3. Map the active mission to a `MissionPhase` (§6.1).
4. `checkForFailures()`.
5. `handleVehicleMissionControl()` (§6.2).
6. `vehicleShutdownControl(!vehicleIsReady())` (§6.3).
7. `vehicleArmingControl()`: motor power limit 1.0 if armed, otherwise **0.1** (`:1233-1237`).

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

Actions fire only when the active mission's mode **differs from `lastMissionMode_`**. That variable is shared across missions:

| New mode | posEst / IMU / bodySim zeroing | `controlAttitudeTvc` & `controlPositionStandard` | Other |
|---|---|---|---|
| `Idle` | off | disabled | flaps disabled |
| `Initialisation` | **on** | disabled | flaps enabled (then overridden, §6.3) |
| `Startup` | off | disabled | `starshipTVC.beginActuatorTest(0)` (4 s servo test). Flap test only with `DO_FLAP_TEST_STARTUP` defined (undefined). |
| `Running` | off | `enableControl(mission_->getActuatorsEnabled())`, evaluated **once at the edge** | flaps enabled |
| `Finished` | off | disabled | flaps disabled |

Since 2026-11-20 the zeroing column sets `missionZeroing_`; `applyEstimatorZeroing()` puts the three estimators into zeroing mode while `missionZeroing_` **or** the `ZeroEstimators` / boot-zero window is active (overview.md, "Estimator zeroing"). The mission's own zeroing is unchanged and does not set `VEHICLE_STATUS` `ZEROING` (the window cannot overlap a mission: `MissionBegin` is refused during a window, and `ZeroEstimators` is refused while a mission is active).

Disabled controllers do not go silent: `ControlAttitudeTvc` publishes `[0,0,0.1,0]` (0.1 N axial) when disabled (`lib/ExVectrControl/src/control_attitude_tvc.cpp:262`).

Switching, after the edge handling, is decided by `Safety::MissionChain::step()` (`src/safety/mission_chain.*`, pure, host-tested in `test/test_mission_chain`). A chain is **pending** only from an accepted `MissionBegin` until RTH ends or a reset (`SystemReset`, or `MissionAbort` of a mission that is not Running):

| Condition | Action |
|---|---|
| active == RTH and `missionEnd()` (pending or not; also the steady state after boot and after SystemReset) | `enableMotors(false)`, flaps off, `vehicleShutdownControl(true)`; the chain is no longer pending |
| chain not pending | nothing |
| pending, `missionEnd()` and mode `Idle` (= the mission was reset) | nothing; the chain is cancelled |
| pending, `missionEnd()`, not Idle, `nextMission()` ∈ {RTH, nullptr} | `missionSelection_ = 0`, `switchMissionTo(&defaultMission_)` |
| pending, `missionEnd()`, not Idle, `nextMission()` is another mission | `switchMissionTo(nextMission())` |

Before 2026-11-20 the switch used `missionEnd()` alone. Because `resetMission()` sets it, a SystemReset (which reset only the active mission and left it active) and a `MissionAbort` during the countdown made the next tick start the next mission: waypoint → bellyflop → freefall → RTH hot start (bench, simulation).

`resetMissionsToBootState()` (SystemReset): chain reset, `resetMission()` on all four mission objects, RTH (Idle) active with the position controller subscribed to its setpoint topic, `missionSelection_ = 0`, `missionPhase_ = Idle` (the phase mapping then shows `Completed` once initialised, as after boot), mission zeroing off, `controlAttitudeTvc` and `controlPositionStandard` disabled, flaps off, `lastMissionMode_ = Idle` (no edge pending). Not reset (not mission state): the flap observer keeps its last `ControlAttitudeBellyFlopSetting` until the next Running mission; the flap telemetry shows the last angles while the flap outputs are disabled.

`switchMissionTo(m, t0 = 0)` (`:531`): `reset()` on the old mission, `reset()` on the new one, `m->beginMission(t0)`, then resubscribe the position controller's setpoint to `m`.

Because the edge handling is shared, the controller enable state in the real chain is:

| Step | Mode seen | Controllers |
|---|---|---|
| Waypoint Running | edge Startup→Running | enabled (waypoint actuators = true) |
| Bellyflop Running | no edge (Running→Running) | stay enabled. Required for the TVC attitude manoeuvre. |
| Bellyflop Finished | edge | disabled |
| Freefall Running | edge Finished→Running | `enableControl(false)` |
| Freefall Finished | edge | disabled |
| RTH Running | edge Finished→Running | enabled |

An **abort or mag-timeout switch to RTH while Freefall is Running is Running→Running**, so no edge fires and the controllers stay disabled while RTH enables the actuators (see Known issues).

Other ways a mission gets switched:

| Source | Condition | Action |
|---|---|---|
| `Telecommand_MissionBegin` (`:871`) | vehicle mode `Ready` (armed **not** required) | `switchMissionTo(&missionWaypointTask, paramInt × 1 s)`, vehicle mode → `Running`, `enableMotors(!simulationMode_)`. The mission starts at `missionTime = paramInt` s, so a countdown needs a **negative** paramInt: Init lasts until −10 s, Startup until 0 s. |
| `Telecommand_MissionAbort` | vehicle mode `Running` | Mission mode `Running`: switch to RTH (the chain stays pending until RTH ends). Otherwise: actuators off, shutdown, chain reset, `resetMission()` (the mission stays the active one, Idle; the phase still reads `Hover`), `clearFailures()` (which also clears the initialisation flag, so the vehicle goes Startup → Ready; the arm state is unchanged); nothing follows (before 2026-11-20 the chain continued to bellyflop). Bench (simulation): abort in Startup → Ready, mission Idle, no switch for 30 s. |
| `Telecommand_SystemReset` | 0xC5 | `resetMissionsToBootState()` plus the system state reset (overview.md). |
| Mag data older than 2.5 s (`:1059-1068`) | vehicle mode `Running` and mission mode `Running` | switch to RTH |
| `Telecommand_MissionSelect` (`:855`) | — | no-op (only logs) |

`missionSelection_` is written into a **local copy** of `MissionState` at `:619` that is then discarded, so the telemetry `missionIndex` never carries it.

### 6.3 Actuator gating (`vehicleShutdownControl`, `:1187-1203`), every tick

- `shutdown` (not ready, a failure latched, or RTH ended): TVC actuators off, flaps off, simulator TVC off.
- Otherwise: flaps on if the mission is Freefall, else `mission_->getActuatorsEnabled()`. TVC actuators and simulator TVC follow `getActuatorsEnabled()`. Simulator flaps follow the flap state.
- So the per-mode flap enables in §6.2 are overridden every tick, and the effective rule is "Freefall: on, else mission actuators".
- Motors additionally need `enableMotors_`, which is set at MissionBegin and cleared at RTH end, reset and simulation toggle. Within `StarshipTVC` they also need `enableActuators_` and an idle actuator test. **Arming only changes the power limit (1.0 vs 0.1)**, see [hardware-and-drivers.md](hardware-and-drivers.md#3-starshiptvc).

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
| 1 | (0,0,1) | 0.5 m/s | 3 | 1 (effective 0.5 m/s² braking) | 0.5 m | 5 s | 10 s | **yes** | Slow lift to 1 m and a 5 s hover check. If 1 m ± 0.5 is not reached in 10 s, the mission ends, and the **chain continues to Bellyflop**. |
| 2 | (0,0,120) | 7 m/s | 3 | 2 (effective 1 m/s²) | 3 m | 5 s | default = 119/7 × 1.5 = **25.5 s** | no | Climb to 120 m and hover 5 s within 3 m. Reached or timed out, the mission ends → Bellyflop → Freefall → RTH. |

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
| Freefall | always | `Stabilize` | `pitchAngle = 0` each tick, then: |
| | `Pz > 100` | | `azimuth = −atan2(Py, −Px)` (back toward the origin), `pitch = (clamp(−(dXY − 1), −40, 0) + 20)°`, so +20° near the origin down to −20° at ≥ 41 m off. |
| | `Pz < getStopAltitude() + 20` (= stopDist + 40) | | `pitch = 30°`. The azimuth keeps its last value. |
| | otherwise | | pitch 0, azimuth keeps its last value |
| RTH | `stabilising()` | `Upright` | (controller: top flaps 0 = extended, bottom 90° = retracted) |
| Waypoint | `defaultMission_.stabilising()` (**checks RTH's state, not Waypoint's**) | `Acsent` | `extentFlapsOnAscent = EXTEND_FLAPS_ASCENT` (false) |
| anything else (Bellyflop, RTH translate/descend, Waypoint after any RTH run) | | `Acsent` | `extentFlapsOnAscent = false` |

In `Acsent` mode the flap controller commands 90° (retracted) unless extend is set. The flap angle convention and servo mapping are in [hardware-and-drivers.md](hardware-and-drivers.md#4-starshipflaps).

---

## Known issues & limitations

| Severity | Location | Issue | Impact |
|---|---|---|---|
| Critical | `main.cpp:556`, `mission_waypoint.hpp:394-400`, `main.cpp:718-724` | WP1 has `cancelIfNotReachable = true`, but a cancelled waypoint mission still ends through `nextMission()` = **Bellyflop**. | A failed 1 m hover check pitches the vehicle 90° at about 1 m altitude. Freefall then ends at once (stop altitude ≥ 20 m), then RTH. |
| Critical | `main.cpp:622-698`, `:897-903`, `:1063-1066` | Controller enable is evaluated only on mission-mode edges, and `lastMissionMode_` is shared across missions. An abort or mag-timeout from Freefall (Running) to RTH (Running) produces no edge. | RTH flies with `controlAttitudeTvc` and `controlPositionStandard` disabled. TVC gets `[0,0,0.1,0]`, so the motors sit at idle and the vehicle falls. The same applies to any future Running→Running switch from a mission with actuators off. |
| High | `mission_rth.hpp:400-415` | The landing detector fires when the vehicle stays > 0.2 m above the Z setpoint for 2 s, at any altitude. | A poor vertical-tracking period during descent (gust, low thrust margin, estimator jump) is declared a landing, the actuators are cut and the vehicle falls. |
| High | `main.cpp:933`, `:955`, `:1971-1972` | `groundTopicSwitch` is not subscribed at boot, only in the simulation toggle. | The RTH rangefinder landing trigger (`mission_rth.hpp:417`) never fires in real flight. Only the tracking-error trigger works. |
| High | `main.cpp:873-893`, `:1233-1237` | MissionBegin does not require `vehicleArmed_`. Arming only scales the motor power limit (0.1 disarmed). | A disarmed vehicle can start a mission with motors running at 10 % authority. |
| High | main.cpp `checkForFailures`, `vehicleIsReady` | Radio loss (no radio packet from node 0/2 on the UART for 2 s, overview.md) or any sensor older than 0.5 s (GNSS, gyro, baro, posKF, attKF) latches `Failure`, and `vehicleShutdownControl(true)` cuts all actuators. | The response to link loss or a sensor hiccup in flight is flight termination, not RTH. Failure is latched until SystemReset. |
| High | `mission_freefall.hpp:155-159`, main.cpp:445 | Stop altitude = `0.5·Vz²/(T/m − g) + 20`. It assumes full 14 N immediately, ignores flip-up time and drag changes, and is sign-blind. It goes negative or infinite if `T/m ≤ g`. | Margin is only the fixed 20 m. A thrust-to-weight change (mass, battery sag) silently breaks it. |
| Medium | `main.cpp:1119`, `:1134` | Out-of-bounds uses `magnitude(3,5)`, which covers elements 3 and 4 only. | No altitude limit; the vehicle may climb without bound. The horizontal-only limit is 100 m. |
| Medium | `mission_rth.hpp:292-294` | `travelDistance / travelDistanceMag` with no zero guard. | If the setpoint converges exactly onto home before the vehicle is within 0.5 m (or starts there), the velocity setpoint becomes NaN. |
| Medium | `mission_rth.hpp:277-280`, `:284-319` | The Ascent state is a no-op, and Translation is a straight 3-D line to (0,0,3). | No climb to a safe transit altitude. From freefall exit (tens of metres) the path descends diagonally at ≤ 2 m/s, which is slow and low over the terrain on the way. |
| Medium | `mission_waypoint.hpp:119`, `:125`, main.cpp:552 | Default time limit with `velocity = 0` gives `x/0 → inf → int64` (undefined behaviour). | WP0 is harmless today only because it is reached instantly (threshold 5 m). |
| Medium | `mission_waypoint.hpp:424`, `mission_rth.hpp:308` | Braking cap `sqrt(d·a)` gives an effective deceleration of `a/2`. | The configured stopAccel values are twice the real braking. Arrival is gentler and slower than configured. |
| Medium | `mission_waypoint.hpp:380-420` | The time limit applies only until the waypoint is first reached. Leaving the threshold afterwards has no timeout. | The mission can hang forever on overshoot or drift. |
| Medium | `mission_bellyflop.hpp:102`, `:87` | The pitch-over direction is hard-coded to nose-south (ref −X). The position controller stays subscribed to a never-published setpoint topic. | Bellyflop heading ignores wind and home. The position-loop thrust during the manoeuvre is whatever stale input it holds (verify in ExVectrControl). |
| Medium | main.cpp:1316-1321 | Waypoint flap branch tests `defaultMission_.stabilising()`. RTH's `runningState_` persists after a run. | Wrong branch after any RTH run. Harmless only while `EXTEND_FLAPS_ASCENT == false`. |
| Medium | main.cpp:874-880, `mission_waypoint.hpp:249`, `:292` | The MissionBegin `paramInt` is the initial mission time, not "seconds in the future" as `telecommand.hpp:159` says. Values > −10 skip Init and Startup. | Operator error can skip estimator zeroing or overlap the TVC test with take-off. |
| Low | `mission_waypoint.hpp:340-370`, main.cpp:700-725 | Waypoint never enters `Finished` when it ends. Switching relies on `missionEnd_` alone. | The Bellyflop start is not a mode edge, which is relied upon to keep the controllers enabled. This is fragile coupling. |
| Low | main.cpp:619 | `missionIndex` is set on a discarded local copy. | Telemetry `missionIndex` is always 0. |
| Low | main.cpp:588-600 | No phase for the RTH Ascent tick or for RTH `Idle` before the first mission (reads `Completed`). Waypoint Init and Startup report `Hover`. | The ground display can mislead. |
| Low | `mission_waypoint.hpp:76`, `mission_bellyflop.hpp:51` | Copy-pasted task names ("Mission RTH", "Mission Freefall"). | Confusing scheduler and debug output. |
| Low | `mission_waypoint.hpp:55`, `:39`, comments | Comments say µs and seconds where values are ns. "Set the mission end to false" sits next to `= true`. | Misleading when editing. |
| Low | `mission_abstract.hpp:19-25` | `MissionType` (Remote, FlapDescent) is unused and unimplemented. MissionSelect is a no-op. | Dead API surface. |
| Low | `mission_bellyflop.hpp:112` | `LOG_MSG` every 10 ms during bellyflop. | Serial load in a time-critical phase. |
| Info | main.cpp `handleTelecommands`, `vehicleIsReady` | After a mission, the vehicle mode stays `Running` (only Startup can return to Ready). | A new MissionBegin needs SystemReset first (since 2026-11-20 it no longer re-zeroes; the waypoint mission's Initialisation still does). |
| Critical | `mission_bellyflop.hpp:66` (`setPaused(true)` in the constructor), `:71-74` (`taskInit`), main.cpp `handleVehicleMissionControl` | **Bellyflop reports `Idle` during its first run after boot** (bench, simulation, 2026-11-20). The scheduler runs a task's `taskInit` on its first run; the bellyflop task is paused until `beginMission`, so `taskInit` runs after `beginMission` and sets mode `Idle` over `Running`. The shared mode-edge handling then sees Running→Idle and **disables `controlAttitudeTvc` and `controlPositionStandard` and the flaps** for the whole first bellyflop, while the mission's TVC actuators stay enabled (the disabled attitude controller outputs `[0,0,0.1,0]`). Telemetry shows mission mode Idle with phase Descent. From the second bellyflop since boot it reports Running. `MissionChain` keeps the chain pending for an Idle mission that has not ended, so freefall and RTH still follow. | The first (in practice every) flight's pitch-over has no TVC attitude control, contrary to §6.2 "controllers stay enabled". Fix candidates (needs the user's decision, flight-safety): drop the mode assignment from `MissionBellyflop::taskInit`, or do not start the task paused. Not changed on `feature/reset-chain-gnss`. |
| Fixed 2026-11-20 | main.cpp SystemReset case, `handleVehicleMissionControl` | (was High) SystemReset during a mission continued the mission chain (waypoint → bellyflop → freefall → RTH hot start in Running while the vehicle reported Ready with TVC/flap actuators enabled). Root cause: switching on `missionEnd()` alone, which `resetMission()` sets. Fixed by `Safety::MissionChain` and `resetMissionsToBootState()`. Bench (simulation, `feature/reset-chain-gnss`): SystemReset in waypoint Init, Startup, hover, ascent, bellyflop, freefall, RTH deceleration, RTH translation and RTH landing: mission mode Idle within one tick, vehicle Ready, actuator telemetry constant and no mission start or switch for 30 s each; a full mission afterwards ran normally. | |
| Fixed 2026-11-20 | main.cpp MissionAbort case | `MissionAbort` while the mission was not Running (countdown: Init or Startup) reset the mission, and its `missionEnd()` started bellyflop, then freefall (which ends at once on the ground), then an **RTH hot start with actuators and (outside simulation) motors enabled**. Now the chain is reset and nothing follows. | |
