# Flight executive

Implementation reference for the flight executive: one pure engine that turns each step's inputs into **one** control command, driven by a declarative flight plan (phases as data). It replaces the mission system of [missions.md](missions.md) in steps E0 to E7.

Design and decisions: workspace `docs/flight-executive-design.md` (sections 2 to 8; the owner decisions of its section 1.3 override its tables).

Last verified against commit: `feature/executive-core` (parent), ExVectrControl `fc93d24` (`feature/executive`), 2026-11-21.

## Status

| Step | State |
|---|---|
| E0 types, native test access, include boundary | done (this branch) |
| E1 engine, behaviours, guards, sanitiser, validator, kick functions moved | done (this branch) |
| E2 Starship plan profile 2, follower-plant scenarios, golden sequence | done (this branch) |
| E3 shadow on the vehicle (`InputGatherer`, `FlightExecutiveTask`, comparator) | open |
| E4 to E7 | open |

**Nothing of the executive runs on the vehicle yet.** The core lives in ExVectrControl, but only its kick functions are linked (by the legacy bellyflop mission through `src/safety/bellyflop_kick.hpp`). `src/executive/` is excluded from the `teensy40` build (`build_src_filter = ... -<executive/>`) until E3. The legacy mission system ([missions.md](missions.md)) stays authoritative until E6.

## Where things live

| What | Where |
|---|---|
| Engine, behaviours, guards, plan types, validator, sanitiser, kick geometry (reusable, vehicle-free) | `lib/ExVectrControl/include/ExVectrControl/executive/*.hpp`, `lib/ExVectrControl/src/executive/*.cpp`, umbrella `ExVectrControl/executive.hpp`; namespace `VCTR::Exec` |
| Starship constants (kick, minimum kick altitude) | `src/executive/starship_constants.hpp` (namespace `VCTR::Starship`) |
| Starship flight plan (phase rows and parameters) | `src/executive/starship_plan.hpp/.cpp` |
| Legacy kick names for the old mission code | `src/safety/bellyflop_kick.hpp` (re-export, removed in E6) |
| Host tests | `test/test_executive_*`, shared helpers in `test/executive_support/` (fake clock, follower plant, closed-loop harness) |
| Adapters (input gatherer, command applier, 100 Hz task, telemetry) | not yet (E3/E5), they go in `src/executive/` |

### Include boundary

The core may include only `<stdint.h>`, `<stddef.h>`, `<math.h>` and its own headers (`ExVectrControl/executive.hpp`, `ExVectrControl/executive/*.hpp`), and must not allocate dynamically. No hardware, no ExVectrCore, no scheduler, no globals, no vehicle knowledge. `test/test_executive_includes` greps every core file. Vectors are plain `float[3]`/`float[4]`/`float[6]`/`float[7]` arrays in the project layouts.

ExVectrMath builds natively unchanged (checked in E0). The core still does not use it: the include rule above keeps the plan serialisable and the core free of template types, and the few vector operations it needs are in `exec_math.hpp` and `kick.hpp`. The native env has ExVectrMath on the include path only so that `test_executive_behaviours` can run verbatim copies of the legacy mission code as references.

## Interfaces

### Step input and output (`exec_types.hpp`)

- `ExecInputs`: `nowNs`, `dtNs` (time since the previous step, chosen by the caller; `<= 0` counts as 0), `EstimateSnapshot est`, `OperatorCommands cmd`, `SupervisorLatches sup`.
  - `EstimateSnapshot`: `att[7]` `[ω, q]`, `pos[6]` `[V, P]`, `attValid`, `posValid`, ground distance, variance and freshness. The gatherer decides validity and freshness.
  - `OperatorCommands`: `goEdge`, `legacyStartS` (mission clock at go, the legacy `MissionBegin` paramInt), `abortRequest` (a level), `resetEdge`.
  - `SupervisorLatches`: `failsafe`, `abortNoPos` (T9b-7, inactive), `magTimeout`, `supervisorReady` (read by the applier, not the core).
- `ControlCommand`: phase and `phaseSeq`, `missionTimeNs`, `tvc` (`Off`, `Position`, `Attitude`) and `resetTvc`, `posSp[6]`, `attSp[4]` and `attSpSource`, `thrustN`, `thrustCapN` (`kNoThrustCapN` = 1e9 when none), `flaps` (`Disabled`, `Retracted`, `Extended`, `Descent`, `Upright`) and the descent guidance `flapAzimuthRad`/`flapPitchRad`, the requests `tvcActuators`, `flapActuators` (= `flaps != Disabled`), `motors`, `estimatorZeroing`, `actuatorTestSeq`, `bounds`, `onGround`, `sanitized`.
  - `phaseSeq` and `actuatorTestSeq` are monotonic (they survive a reset), so the applier and telemetry can detect edges.
- `ExecStatus` (per step, telemetry and logs): go result, abort outcome, `missionComplete` (one step on entering the plan's complete phase), `entryRedirected`, `transition` (the reason of this step's phase change).

### Flight plan (`plan.hpp`)

A `FlightPlan` is a table of `PhaseSpec` rows plus typed parameter tables (`BehaviourTables`: go-to, transit, land, kick, descent; `GuardParams`), and the ids of its special phases: `idle` (boot, reset target, go accepted here), `first` (entered on go), `failsafe`, `rth` (abort target), `noPos` (T9b-7, may be `kNoPhase`), `complete` (landed). It holds no function pointers.

A `PhaseSpec` row: `tvc` and `thrustCapN`, `flaps`, `tvcActuators`, `motors`, `behaviour` + parameter index + `behaviourEntry` (`Init`, or `Continue` = keep the memory when the previous phase ran the same behaviour with the same parameters), `ctrlGroup`, `bounds`, `onGround`, `estimatorZeroing`, `entryActions` (bit `kEntryTvcActuatorTest`), `timeoutNs`/`onTimeout` (`kNoTimeout` = none), `abort` and `abortNoPos` rules (`Ignore`, `ResetAndStop`, `ToRth`, `Stay`, `ToNoPos`), an optional `entryGuard` (if false when the phase is targeted, its `fallback` is entered instead), and up to four exits (guard, parameter index, target) in priority order. Rows are written with the constexpr builder (`PhaseSpec::make(id, name).withTvc(...).withExit(...)`); every field not set keeps its safe default (everything off, no timeout, abort ignored).

### Behaviours (`behaviours.hpp`, `behaviour_*.hpp`)

Deterministic step functions over an explicit memory POD (`BehaviourMem`). Initialised at phase entry from the estimate and the parameters, stepped once per step with the estimate and dt. No other inputs, no hidden state.

| Behaviour | Output | Legacy source |
|---|---|---|
| `Idle` | zero setpoint | idle missions |
| `HoldMeasured` | `[0,0,0, P_meas]` (velocity damper) | waypoint Startup, RTH Stabilize |
| `GoTo` | profiled `[V, P]` towards one target; entry at the measured position or a jump point; instant jump for speed < 0.0001 | `MissionWaypoint::missionHover` steps 4-5, **bit for bit** (test) |
| `Transit` | straight line from the measured position to home | RTH Translation (within 1e-6; no zero guard, the sanitiser catches the NaN) |
| `Land` | XY home, Z ramp while < hold distance above the setpoint | RTH Descent |
| `KickAttitude` | `attSp` = `kickTargetAttitude(q at entry, pitch)`, constant `thrustN`, `posSp` holds the entry position (telemetry only) | `MissionBellyflop` kick |
| `DescentGuidance` | flap azimuth and pitch, zero `posSp`; azimuth kept between steps (starts at 0) | `FlapModeObserverTask` Stabilize branch |

### Guards (`guards.hpp`)

`MissionTimeAbove` (strict), `InsideFor` (timer restarts outside; starts at phase entry), `Reached`, `NotReachedWithin` (outside now, never inside since entry, phase time > limit: the legacy waypoint time limit), `SpeedBelow`, `StopAltitude` (legacy freefall formula), `KickAngleBelow`, `PitchRateAbove`, `LandingDetected` (tracking error OR ground window, both legacy RTH triggers), `AltitudeAtLeast` (needs a valid position). `InsideFor`, `Reached` and `NotReachedWithin` measure to the phase behaviour's target (`behaviourTarget`). Guards see the new estimate and the behaviour memory of the previous step. `GuardParams` has one named constexpr factory per guard.

### Validator (`validate_plan.hpp`)

`validatePlan(plan, PlanLimits)` returns the first `PlanError` and its row. Checks: ids unique and special phases present, all targets and parameter indices valid, numbers finite and within the vehicle limits, timeouts consistent, motors only with TVC actuators, attitude-mode phases with a finite timeout `<= maxAttitudePhaseNs` and a cap `<= maxThrustN` (and the kick thrust within its phase's cap), no airborne phase (except failsafe) that ignores an abort, idle/complete/failsafe request nothing (idle and failsafe also cannot leave by themselves), target guards only with target behaviours, kick and landing guards only with their behaviours, entry guards with a valid fallback that has no entry guard itself, and every phase reachable from idle.

## Step semantics (`FlightExecutive::step`)

1. Global transitions, first match:
   1. `sup.failsafe` → `failsafe` phase. Nothing else is evaluated while the latch is set, not even a reset: the supervisor clears the latch before sending the reset edge.
   2. `cmd.resetEdge` → `idle` (also from idle: re-entry clears the mission state).
   3. `sup.abortNoPos` → the phase's `abortNoPos` rule.
   4. `cmd.abortRequest` or `sup.magTimeout` → the phase's `abort` rule.
   5. `cmd.goEdge` → `first`, only in idle and only while no abort level (3 or 4) is active.

   An abort rule `Ignore` or `Stay` makes no transition; the phase's own timeout and exits still run in that step. A repeated abort level is therefore idempotent.
2. Phase timeout: time in phase `>= timeoutNs` → `onTimeout`.
3. Exits in table order; the first true guard wins.
4. At most one transition per step. On entry: entry guard (fallback if false), `phaseSeq + 1`, guard memories and (unless `Continue` applies) the behaviour initialised from this step's estimate, entry actions. Entering idle clears the mission state (clock, behaviour and guard memory). A go starts the mission clock at `legacyStartS`.
5. The (new) phase's behaviour steps, the command is assembled from the row, then sanitised. So a new phase's command is produced in the same step.
6. TVC reset rule: `resetTvc` is true for one step when the TVC mode becomes non-`Off` after `Off`, or stays non-`Off` and the `ctrlGroup` differs from the previous step's.

Mission clock: 0 in idle (and after any return to idle), otherwise `legacyStartS + (now − go time)`, monotonic through all phases.

Output sanitiser (`sanitize.hpp`): a non-finite position setpoint becomes hold (velocity 0 at the measured position, or the last finite setpoint position); a non-finite or zero attitude target switches `TvcMode::Attitude` off for that step (thrust 0); non-finite thrust, cap or flap guidance become 0; `sanitized` is set. The scenarios assert that it never fires in a normal flight.

## Starship plan profile 2 (`src/executive/starship_plan.cpp`)

Phase id = wire value of the telemetry phase. Values 0 to 7 keep their wire names (in brackets where the new code uses another name); new values are appended. 13 and 14 are reserved for the T9b-7 no-position RTH.

| Id | Phase | TVC (cap) | Flaps | TVC act / motors | Behaviour | Group | Bounds | Ground | Exits (priority) / timeout | Abort / no-pos |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | `Idle` | Off | Disabled | off/off | Idle | 0 | None | yes | (go) | Ignore / Ignore |
| 8 | `Countdown` | Off | Disabled | off/off | HoldMeasured; estimator zeroing | 0 | Stationary | yes | mission time > −10 s → 9 | ResetAndStop / ResetAndStop |
| 9 | `PreLaunch` | Off | Retracted | on/on | HoldMeasured; entry: TVC actuator test | 0 | Stationary | yes | mission time > 0 → 1 | ResetAndStop / ResetAndStop |
| 1 | `Hover` | Position | Retracted | on/on | GoTo: jump to (0,0,0.3), to (0,0,1) at 0.5 m/s, accel 3, brake 1 | 1 | Flight | no | InsideFor(0.5 m, 5 s) → 2; NotReachedWithin(0.5 m, 10 s) → 4 | ToRth / ToRth |
| 2 | `Ascent` | Position | Retracted | on/on | GoTo: from (0,0,1) to (0,0,120) at 7 m/s, accel 3, brake 2 | 1 | Flight | no | Reached(3 m) → 10; timeout 25.5 s → 11 | ToRth / ToRth |
| 10 | `ApogeeHold` | Position | Retracted | on/on | the Ascent go-to, continued | 1 | Flight | no | InsideFor(3 m, 5 s) → 11 | ToRth / ToRth |
| 11 | `Kick` ("Transition") | Attitude (11 N) | Retracted | on/on | KickAttitude 90°, 11 N; **entry guard: altitude ≥ 90 m, else → 4** | 2 | None | no | KickAngleBelow(15°) → 3; PitchRateAbove(250°/s) → 3; timeout 1.0 s → 3 | ToRth / ToRth |
| 3 | `Descent` | Off | Descent | off/off | DescentGuidance | 0 | None | no | StopAltitude(1.1 kg, 14 N, 20 m) → 12 | ToRth / ToRth |
| 12 | `Flip` | Position | Upright | on/on | HoldMeasured | 3 | None | no | SpeedBelow(0.5 m/s) → 5 | Stay / Stay |
| 4 | `RthBrake` [Deceleration] | Position | Upright | on/on | HoldMeasured | 3 | None | no | SpeedBelow(0.5 m/s) → 5 | Stay / Stay |
| 5 | `RthTransit` [RTH] | Position | Retracted | on/on | Transit to (0,0,3), 2 m/s, 0.5 m/s² | 3 | None | no | Reached(0.5 m) → 6 | Stay / Stay |
| 6 | `Landing` | Position | Retracted | on/on | Land at (0,0,3), 0.2 m/s, hold 0.3 m | 3 | None | no | LandingDetected(0.2 m for 2 s, or ground in (−0.02, 0.04) m, var < 7, fresh) → 7 | Stay / Stay |
| 7 | `Landed` [Completed] | Off | Disabled | off/off | Idle | 0 | None | yes | – (terminal until reset) | ResetAndStop / ResetAndStop |
| 15 | `Failsafe` | Off | Disabled | off/off | Idle | 0 | None | no | – (reset only) | Ignore / Ignore |

Plan specials: idle 0, first 8, failsafe 15, rth 4, noPos none, complete 7. Limits for the validator: thrust 14 N (`TVC_THRUST_LIMIT_N`), attitude-mode phases at most 1.0 s (the kick's hard limit), 15 m/s, 5 m/s².

### Differences from the design table and from the legacy system

Listed parity differences of the design (section 6.4) that this plan implements: **P1** (hover check failed → `RthBrake`, asserted in `test_executive_scenarios`), **P2** (ascent timeout below 90 m → `RthBrake`, asserted), P3 (8/9 and 12 as phase values), P4 (no RTH Ascent tick), P5 (an abort or magnetometer timeout in the RTH phases keeps the phase), P6 (kick thrust commanded as 11 N; kick end goes straight to `Descent` with TVC actuators and motors off), P7 (each countdown phase lasts at least one step, so the actuator test is never skipped; asserted).

Implementation choices that differ from the design table (reasons in brackets):

| # | Design | Implemented | Reason |
|---|---|---|---|
| D1 | `Hover`: phase timeout 10 s → 4 | exit `NotReachedWithin(0.5 m, 10 s)` → 4, no phase timeout | parity (X1): the legacy WP1 limit applied only until the waypoint was first reached |
| D2 | `ApogeeHold`: timeout "rest of 25.5 s" → 11 | no timeout | parity (X1): the legacy WP2 limit stopped once reached (known issue "waypoint timeout after reach", E7). **Open question for the owner.** |
| D3 | `Ascent` timeout → 11 if altitude ≥ min, else 4 | timeout → `Kick`, whose entry guard sends every entry below 90 m to `RthBrake` | one generic mechanism; also protects the `ApogeeHold` → `Kick` path (normally at 120 m, so no change in practice) |
| D4 | `PreLaunch` flaps Disabled | flaps Retracted (flap actuators on) | parity (X1): the legacy Startup enabled the flap actuators with the mission's actuator flag; the observer's default setting is retracted |
| D5 | `ApogeeHold` "GoTo held at the target" | the Ascent go-to continued (`BehaviourEntry::Continue`) | parity: the legacy WP2 profile kept running (setpoint and speed) during the loiter |
| D6 | `abortNoPos` = `ToNoPos` in flight | `ToRth` in flight, `Stay` in recovery, `ResetAndStop` on the ground; phases 13/14 not in the plan | T9b-7 is not approved; the input stays false until then |
| D7 | mission clock (design silent) | one clock from go, not reset at phase changes | the legacy clock restarted at 0 at every mission switch (bellyflop, freefall, RTH); telemetry `mission_time` will differ from E4/E5 on. **Open question for the owner.** |
| D8 | timeout `elapsed ≥ timeoutNs`; legacy kick `> 1 s` | `≥` | design 2.4; the kick can end one step (10 ms) earlier than the legacy `>` |
| D9 | `PhaseSpec::flapActuators`, `kinematicTiltTolerant` | not fields: `flapActuators = flaps != Disabled`; tilt tolerance is the `bounds` profile | fewer inconsistent states |
| D10 | `Descent` command carries the flap pointing setpoint as `attSp` (X7) | `attSpSource = None` in `Descent` | the flap pointing quaternion needs the flap controller's convention; decided with the telemetry in E4/E5 |

Known behaviour inherited on purpose (X1, fixed in E7): braking at a/2, the waypoint timeout after reach, the landing tracking detector at any altitude, the sign-blind stop altitude, the RTH straight line without climb, the RTH unit vector without a zero guard (now caught by the sanitiser), the kick heading.

## How to

- **Add a phase:** a row in `kPhases` (`starship_plan.cpp`) with its exits, a new id appended to `Starship::Phase` (and later to the ExVectrPackets `MissionPhase` schema enum and the ground-station names). The validator, the property tests (they iterate the table) and the golden test cover it; the property test's abort table (`abortTarget`) and the golden table need the new expectations.
- **Add a behaviour:** `behaviour_<name>.hpp/.cpp` in the core (params, memory, init, step), an enum value and a table in `behaviours.hpp`, dispatch cases in `behaviours.cpp`, parameter checks in `validate_plan.cpp`, a unit test in `test_executive_behaviours`.
- **Add a guard:** an enum value and its fields/factory in `guards.hpp`, its case in `guards.cpp`, checks in `validate_plan.cpp`, a unit test.
- **Add a plan** (T9b-8 profiles): another `FlightPlan` built from the same phase ids and parameter tables; validate it in `test_executive_plan`.
- **Change the golden sequence:** run `test_executive_golden`; on a mismatch it prints the actual table. Paste it only with an explanation in the commit message; the main session reviews it like code.

## Tests

`~/.platformio/penv/bin/pio test -e native -f "test_executive_*"`

| Test | Content |
|---|---|
| `test_executive_engine` | step semantics with a test-only plan: priority order, go refusal, failsafe latch, abort idempotence and `Stay`, exact timeout and timeout before exits, one transition per step, TVC reset rule, entry-guard redirect, `Continue`, reset to the boot command, mission clock, sanitiser |
| `test_executive_behaviours` | go-to bit for bit against the legacy code (4 parameter sets, 4000 steps each, varying dt), transit and land against the legacy code, hold, idle, descent guidance, kick geometry (moved from `test_bellyflop_kick`), every guard |
| `test_executive_plan` | the Starship plan validates; one-defect copies are rejected (unreachable, attitude phase without timeout or cap, airborne abort ignored, non-finite and out-of-limit numbers, unknown targets and ids, unsafe actuator requests, guard misuse, exit overflow, bad entry guard, missing no-pos target); the owner's constants (90 m, 250°/s) |
| `test_executive_scenarios` | follower plant: nominal flight countdown → landed, countdown timing and P7, hover-check timeout (P1), ascent timeout above and below 90 m (P2), kick ended by angle, rate and time, landing by ground sensor vs tracking error, non-finite estimate |
| `test_executive_properties` | for every phase: abort outcome, repeated abort, magnetometer timeout, reset → idle with the boot command, failsafe and its latch; attitude phases bounded; 10^5-step seeded fuzz of events, estimates (also non-finite) and dt with the command invariants |
| `test_executive_golden` | the nominal flight's phase sequence (phase, entry time ± 10 ms, TVC mode and reset, flaps, actuator and motor requests, cap) against a checked-in table |
| `test_executive_includes` | include boundary and no dynamic allocation in the core |

The follower plant (`test/executive_support/follower_plant.hpp`) is not a vehicle model: a point mass whose velocity follows the commanded setpoint with a first-order lag (acceleration and climb rate limited), a scripted pitch rate during the kick, a ballistic fall with drag otherwise, and the ground at z = 0. Controller dynamics, ESC behaviour and aerodynamics are verified only in the simulator and in flight (design 6.5).

Not yet: the single-writer grep for actuator calls (design 3.5; it applies once the `CommandApplier` exists, E5) and the comparison with the legacy system on the vehicle (E3 shadow).

## Known issues & limitations

| Severity | Location | Issue | Impact |
|---|---|---|---|
| Info | whole executive | Host-tested only; the parity with the legacy missions is shown for the go-to profile (bit for bit) and transit/land (1e-6), not for whole flights on the vehicle. | E3 shadow mode on the simulation bench must compare it with the old system before anything drives (design 6.2). |
| Low | `starship_plan.cpp` (`ApogeeHold`) | No timeout in `ApogeeHold` (parity, D2): a vehicle that drifts out of the 3 m window and never holds it for 5 s stays there. | Same as the legacy "waypoint timeout after reach" known issue; E7 or an owner decision. |
| Low | `guards.cpp`, `flight_executive.cpp` (`enter`) | `InsideFor` starts its timer at phase entry. In `ApogeeHold` the legacy loiter counted from the last tick outside, one legacy tick (100 ms) earlier. | Within the E3 tolerance (one old tick). |
| Low | `missions.md` section 3 | Still names `src/safety/bellyflop_kick.hpp` as the home of the kick constants; they now live in `src/executive/starship_constants.hpp` and are re-exported under the old names. | Left unchanged on purpose: `missions.md` stays authoritative and untouched until E6. |
