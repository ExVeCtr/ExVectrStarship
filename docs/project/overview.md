# Starship Vehicle: System Overview

> What the vehicle does, how a flight works end to end, how it is controlled and kept safe, and how to operate it. Details per mission: [missions.md](missions.md). Hardware, drivers, wire formats: [hardware-and-drivers.md](hardware-and-drivers.md).

## Purpose

An autonomous model of SpaceX's Starship (1.1 kg). It serves as the flagship application and hardware-in-the-loop testbed for the ExVectr embedded framework. Flight goals:

1. Lift off and hover using **thrust-vector control (TVC)**.
2. Ascend to altitude.
3. Perform the **bellyflop**: rotate to horizontal, belly down.
4. Descend in a **flap-stabilised freefall**.
5. Flip back upright, decelerate, **return to home** and land.

## Vehicle

| Item | Value | Source |
|---|---|---|
| Mass | 1.1 kg (measured; owner decision 2026-11-21) | `VEHICLE_MASS_KG`, `starship_hardware.h` |
| Max thrust | 14 N (T/W ≈ 1.3) | `TVC_THRUST_LIMIT_N` |
| Propulsion | 2 contra-rotating motors (CW pin 8, CCW pin 7); differential speed gives roll torque | `starship_tvc.*` |
| TVC | 4 vane servos (±45° servo travel), mechanical ratio 15 → ±3° thrust deflection | `TVC_SERVO_LIMIT`, `TVC_ANGLE_FACTOR` |
| Aero surfaces | 4 flaps (upper L/R, lower L/R), one servo each | `starship_flaps.*` |
| Flight computer | Teensy 4.0; MPU9250 IMU (SPI, interrupt-driven), BME280 baro (I2C), QMC5883 mag (I2C), VL53L0X ToF range (I2C), u-blox NEO-M8Q GNSS (Serial5), ExVectrNetwork links: USB serial (bench ground station) and a 460800 UART (Serial4) to the ExVectrLink RX module (no on-board radio since T9a), SSD1306 OLED, EEPROM | `board_v_1_0.h` |
| TVC lever arm (sim) | 0.35 m below CoM | `main.cpp` `setTVCInputTopic(..., {0,0,-0.35})` |
| Inertia (sim) | diag(0.044, 0.044, 0.004) kg·m² | `BodySimulator` constructor in `main.cpp` |

## Flight profile (current configuration)

Configured in `VehicleSafetyAndControlTask::taskInit()` in `main.cpp`. A go (SA, T9b-4) starts the waypoint mission with the waypoint list of the SB profile: 2 is the full flight below, 0 a hover test (1 m, hold 30 s, RTH landing), 1 a waypoint test (1 m, then +2 m x, then +2 m x and y, RTH landing); see [missions.md](missions.md) section 7a. Other SB values are refused.

```
MissionWaypoint ──► MissionBellyflop ──► MissionFreefall ──► MissionRTH (default) ──► landed, auto-disarm
 1. 0.3 m   (settle/init)        rotate to belly-down     flaps stabilise,        flip upright, decelerate,
 2. 1.0 m   hover check          (≤1 s, or ≥50° reached,  steer toward home,      translate home, descend,
 3. 120 m   ascent at 7 m/s       or ≥150°/s rate)        stop altitude 20 m      land at {0,0,·}
```

Any mission ending without a successor, SA released in flight (the abort, from any phase, including the bellyflop and freefall), or a magnetometer timeout in flight switches to **RTH**, the universal fallback. A `MissionAbort` on the ground (before liftoff, by mission mode only: countdown, Idle mission, RTH landed) resets and stops instead; the waypoint mission after the countdown always counts as in flight. Waypoint arguments, phase logic and the abort rules: [missions.md](missions.md) 6.2.

## Guidance, navigation and control

**Navigation** (ExVectrDSP, [libs/ExVectrDSP.md](../libs/ExVectrDSP.md)):
- `IMUAttitudeEKF` at 1 kHz: gyro propagation, accelerometer tilt correction, magnetometer yaw correction. Outputs `[ω, q]`.
- `IMUGPSPositionKalman` at 100 Hz: accelerometer-driven prediction in NWU, updated by baro altitude, GNSS horizontal position/velocity and a separate ground-distance filter from the ToF rangefinder (0.4–1.5 m window). Outputs `[V, P]`.
- Sensor calibration: per-sensor `TopicCoordTransform` (`T·(v − bias)`) loaded from persistent memory at boot (keys in `memory_keys.hpp`). The gyro bias is measured automatically (boot zero, `ZeroEstimators` windows of at least 7 s, mission countdown); the hard-coded override is gone. **All three calibrations must be valid to Arm or MissionBegin** (in simulation mode too, owner 2026-11-21): [calibration.md](calibration.md).

**Control cascade** (ExVectrControl, [libs/ExVectrControl.md](../libs/ExVectrControl.md)):

```
mission setpoint [V,P] → ControlPositionStandard (P on position → PI on velocity, +g) → accel_ref (world)
accel_ref → ControlMappingAccToAtt (tilt toward accel, ≤35°, yaw 0) → q_ref
q_ref, accel_ref, [ω,q] → ControlAttitudeTvc (P + rate P + I) → [Fx, Fy, Fz, τz] body
→ StarshipTVC mixer → vane angles + motor speeds
```

- **Controller phases (user decisions 2026-11-20, 2026-11-21):** one explicit table switches the controllers every tick (missions.md 6.2): TVC set (position + TVC attitude) on in the powered phases (waypoint, RTH, and the bellyflop kick, ≤ 1 s, thrust capped at 11 N), off in the countdown, after the kick and in freefall (neutral TVC command `[0,0,0.1 N,0]`); bellyflop set (flap attitude control, `Stabilize`) on in freefall and in the bellyflop after the kick, never together with the TVC set. The TVC set is reset (integrals, setpoint hold) on every enable and on a mission switch.

| Phase | TVC set (motors, vanes) | Flap attitude control |
|---|---|---|
| Countdown (Init, Startup) | off | off |
| Waypoint hover / ascent | on | off (flaps Ascent) |
| Bellyflop kick (Running, ≤ 1 s hard limit) | on, thrust ≤ 11 N, pitch-over setpoint | off (flaps Ascent) |
| Bellyflop after the kick, freefall | off (motors 0 in freefall) | on (Stabilize) |
| RTH | on | off (Upright during stabilisation) |
| Landed, boot, SE reset | off | off |

- **Bellyflop:** `MissionBellyflop` is the powered kick (user decision 2026-11-21): the TVC attitude controller pitches the vehicle 90° about body +Y towards belly down with the motors on (thrust capped), then cuts the motors at 15° remaining angle, a 250°/s spin guard or the 1 s hard limit, and freefall's flap control takes over. All values are named constants in `src/safety/bellyflop_kick.hpp` ([missions.md](missions.md) §3).
- **Flaps:** `FlapModeObserverTask` (10 Hz) picks the `BellyFlopMode` (Ascent / Upright / Stabilize) and target azimuth/pitch per mission. `ControlAttitudeFlaps` turns it into four flap angles. In freefall above 100 m it steers the belly toward home; below stop altitude + 20 m it pitches to 30°.

## Modes and safety

### Vehicle mode machine (`VehicleSafetyAndControlTask`, 10 Hz)

```
Startup ──all inputs published AND valid zero since boot──► Ready ──SA go──► Running
   │  (no zero yet: automatic 10 s boot zero, ZEROING flag)    │                       │
   ▲                                                           │                       │
   └──────────── SE short press (reset) ◄────────── Failure ◄──── any failure ───────────┘
SensorCalib (CalibStart: gyro / accel / mag session) ── disarms, blocks Ready, aborts a zeroing window and clears the valid zero
```

- **Startup → Ready:** every input (GNSS, gyro, mag, baro, both estimators) has published at least once **and** a valid estimator zero exists since boot, with no zeroing window active (`EstimatorZeroing::readyAllowed()`, `src/safety/estimator_zeroing.hpp`). Without a valid zero the vehicle stays in Startup.
- **Failure latches.** Once in Failure, only the **SE short press (the ground station's `reset_button`, from the Pocket SE or the virtual SE)** on the ground with the interlock closed clears it (T9b-4; the `SystemReset` telecommand is removed); after a **GS failsafe** the request must be cleared first (owner Q6). The reset resets the system state only (failures and latches, mode, actuators and motors off, the interlock needs a new motors toggle) and **resets the whole mission chain to the state after boot** (user decision 2026-11-20): every mission Idle with mission time 0 and setpoint zero, RTH active and Idle, no pending next mission, mission controllers and flaps off (`resetMissionsToBootState()`, main.cpp; `Safety::MissionChain`, `src/safety/mission_chain.hpp`). Nothing starts again without a new go. Same code path in simulation and real mode, with one documented difference: in simulation mode the simulated body is put back to its start pose ("Simulation mode"). It does **not** change simulation mode. The estimators keep running untouched. With a valid zero since boot and no zeroing window active the vehicle returns to Ready as soon as all inputs have published again (within 0.5 s on the bench), without zeroing; a reset during a zeroing window invalidates the zero and the automatic 10 s zero runs first ("Estimator zeroing").
- **Boot:** since 2026-11-20 the GNSS configuration runs as a non-blocking sequence (one bounded u-blox call per 1 ms task run, the old `delay(1000)` is a timed wait state; [hardware-and-drivers.md](hardware-and-drivers.md#6-ubloxserialgnss-srcublox_gnsshppcpp)). The vehicle reaches Ready after the 10 s boot zero without latching Failure (bench: 3 boots, longest vehicle-side POSE gap 64-83 ms at t ≈ 6.7 s; before: about 1.3 s stall and Failure (sensorFailure) at every boot).

### Estimator zeroing (user decision 2026-11-20)

Zeroing and the system reset are separate. Zeroing runs the existing `enableZeroingMode` of the position KF, the attitude EKF and the body simulator for a **window timed on the vehicle** (`EstimatorZeroing`, `src/safety/estimator_zeroing.*`, applied by `VehicleSafetyAndControlTask::applyEstimatorZeroing()`):

| Estimator | What zeroing does ([libs/ExVectrDSP.md](../libs/ExVectrDSP.md)) |
|---|---|
| Position KF | State `[V, P]` decays to 0 (`x *= 0.95` per 10 ms update), the barometric reference altitude and the GNSS lat/lon reference and the range zero point are IIR-slewed to the current readings (factor 0.1 per sample), and the accelerometer bias is learned assuming the vehicle is at rest. So altitude, vertical velocity and horizontal position go to 0 relative to the new origin. |
| Attitude EKF | The gyro bias is learned (IIR 0.005 per 1 kHz sample, only while \|ω\| < 10°/s). The attitude itself is **not** reset: it is absolute (tilt from gravity, heading from the magnetometer in the NWU frame), so it keeps its physical value. |
| Body simulator | Holds the simulated body at its zeroing pose (`p_N = 1 m`, q = (0,0,0,1): a known DSP bug, Known issues there). Applied in simulation and flight alike, as before. |

**The vehicle must be kept still** for the whole window (it learns biases assuming zero motion). `VEHICLE_STATUS` `ZEROING` (flags bit 7) is high for the window so the ground station can show it.

- **Automatic boot zero:** once all inputs have published after boot (the point where the old Startup went to Ready) and no valid zero exists, the vehicle zeroes for 10 s and only then becomes Ready. It also restarts after the SE reset or a mag calibration aborted any zeroing window (boot zero or `ZeroEstimators`), because that clears the valid zero. It is identical in simulation and flight.
- **`ZERO_ESTIMATORS` request** (`REQUEST` action 12, `arg_f` = window in s, NaN or 0 = 10 s, clamped 1..60 s; session-checked, answered with `REQUEST_ACK`; replaces the legacy telecommand type 10 since T9b-5 part 1): accepted only when initialised, **disarmed**, motors off, mode Startup or Ready and the active mission `Idle` or `Finished` ("on the ground": no mode in which the vehicle can fly and no mission running). Otherwise ignored, counted and logged (at most once per second). A second one during a window restarts it with the new duration. One-shot ground action (safety.md 2.5): the end time lives on the vehicle, nothing depends on the sender staying connected.
- **During a window:** a go (SA) is refused (`go_refusal` 3); actuators stay shut down (`vehicleShutdownControl(true)` while the window is active, in addition to the normal gating); the mode does not change (Ready stays Ready). No failure check depends on the zeroing: the kinematic (tilt / distance) checks run only while a mission is active, which a window excludes, and the estimators keep publishing, so the data-age checks are unaffected. Failure detected during a window latches as usual; the window continues and still records its zero.
- **End:** a window that runs to its end records a **valid zero since boot**. The SE reset and `CalibStart` (mag calibration) **abort** an active window without recording a zero, and since the review fix of 2026-11-20 they also **clear an earlier valid zero**: the estimators were already in zeroing mode for part of the window and hold a partial zero. Ready (and so a go) then needs a new complete window. An SE hold shorter than 1 s (or aborted by the interlock opening) clears the valid zero the same way; one of at least 1 s records a valid zero. The vehicle is not left waiting: in Startup the automatic 10 s boot-style zero starts again (after the SE reset at once, after a mag calibration when it ends and the vehicle returns to Startup), so keep the vehicle still after such a reset. An SE reset with no window active keeps the valid zero (only a reboot or an aborted window clears it).
- The waypoint mission's own Initialisation phase still zeroes the estimators before every flight, as before (missions.md 6.2).
- **After the SE reset the estimators keep the old origin.** To start a flight from a new spot, or after the vehicle was carried around, send `ZeroEstimators` (or rely on the mission's Initialisation zeroing). In simulation mode the SE reset also puts the simulated body back to its start pose (upright, origin, at rest; a documented sim/flight difference, see "Simulation mode"); the estimators are not touched in either mode.

**Why the boot zero matters (bench 2026-11-20):** the position KF starts predicting before the attitude EKF publishes (the EKF task starts at t = 5 s); with a zero quaternion it integrates −g, so at the first estimate the altitude is already about −300 m at −89 m/s. Without a zero the barometer only slowly pulls it back: on 7ac9b96 the bench vehicle read about −427 m with +0.8 to +1.2 m/s while standing still (the "−440 m" symptom). After the boot zero: altitude within ±0.03 m at the end of the window. 60 s stationary after a `ZeroEstimators`: altitude mean +0.06 m, σ 0.25 m, range −0.54..+0.52 m, linear trend +3.5 mm/s; vertical velocity mean −0.09 m/s (σ 0.07), horizontal position < 7 mm. The remaining vertical-velocity offset is the accelerometer bias learned with the known `updateAccBias` frame bug (DSP Known issues), estimator not changed.

### Command mapping for go, abort, simulation/real, reset and zeroing (T9b-4, input changed by the GS-control cutover 2026-11-23)

**Status:** **the ground station is the only control source** (workspace `docs/gs-control-design.md`, `docs/safety.md`). The vehicle accepts command state only from node 0 as `CMD_STATE` (port 0x13) on USB and UART (one stream, one latched session); the Radiomaster Pocket only feeds the ground station, and its `RC_STATE` is not interpreted by the vehicle at all. The vehicle acts on the fields of the accepted `CMD_STATE`: **go** (SA: observed NOT_GO -> GO, Ready, no latch, no zeroing, `vehicle_mode` = applied mode, profile 0, 1 or 2, no request in progress; the interlock is not a precondition) and abort (go NOT_GO while a chain is pending or Running: `decideAbort` every tick, on the ground reset and stop without clearing latches, in flight RTH), **profile** (SB) latched at go, **vehicle_mode** (SC: REAL 0 / SIMULATION 1, an observed change, on the ground, not committed; the vehicle boots real), **motors** (SD, "motors armed / disarmed": the interlock), **reset_button** (SE) short press = system reset (on the ground, interlock closed, GS failsafe request cleared) and hold >= 1 s = zero the estimators while held, **failsafe** (level 1 latches a failsafe in any state, failure bit 14). The safety telecommands (Arm, SimulationMode, MissionBegin, MissionSelect, MissionAbort, SystemReset) are removed. The full table, the rules and the owner's bench procedure: [hardware-and-drivers.md](hardware-and-drivers.md#5-exvectrnetwork-links-srccomms) "Command ingress" and "Command path (T9b-4)". The control failsafe (table below) uses the latched session's `CMD_STATE` age (T9b-3). **Consequence (owner decision): if the ground-station app or PC dies in flight, the vehicle falls after 1 to 2 s.**

`reset_button` (the Pocket switch **SE** through the ground station; agreed 2026-11-20, implemented in T9b-4):
- **Short press** (released within 1 s): the system reset (state reset, no zeroing).
- **Held longer than 1 s:** starts zeroing and continues while SE is held; zeroing ends on release.
- The hold is a **hold state on the vehicle** (`EstimatorZeroing::holdStart/holdRefresh/holdRelease`): it starts after SE has been pressed for 1 s, zeroes while fresh pressed states arrive plus a 300 ms grace, and ends on release, staleness, the interlock opening (aborted) or 60 s after its start; it records a valid zero if it ran for at least 1 s, otherwise the vehicle returns to Startup for a complete zero. It shares one `EstimatorZeroing` object with the fixed `ZeroEstimators` window (the window end is the later of the two). The same preconditions apply (interlock closed, on the ground).
- The workspace `docs/safety.md` 4.2/4.3 was aligned 2026-11-20: a latched condition is cleared by the short press (reset system state), the hold (> 1 s) zeroes and clears nothing.

### Failure detection (`checkForFailures`)

| Check | Threshold | Effect |
|---|---|---|
| **Control source lost** (T9b-3, the one control failsafe rule): while committed (`Safety::committed`: motor interlock open, mission chain pending, or mode Running and not landed on the ground), the accepted `CMD_STATE` of the latched ground-station session (node 0, USB or UART) is older than 2 s, or there is none | 2 s (`VehicleNetwork::controlSourceLost`, 10 Hz check, so 2.0-2.1 s) | Failure (latched until SystemReset), failure bit 5 `CONTROL_SOURCE_LOST`. Heartbeats, `REQUEST`s, telecommands, link statistics, handset `RC_STATE` and rejected frames never keep it alive. Not committed (ground, disarmed, nothing pending, including a landed vehicle still in mode Running): only `ctrl_source` NONE, no failure (D2-B). Details: [hardware-and-drivers.md](hardware-and-drivers.md#5-exvectrnetwork-links-srccomms) |
| **GS failsafe request** (design D-GS6): `failsafe = 1` in the last accepted `CMD_STATE` | next 10 Hz tick, any state | Failure (latched), failure bit 14 `GS_FAILSAFE`; the SE reset is refused while the request is 1 (Q6) |
| GNSS, gyro, baro, attitude-EKF or position-KF data age | > 0.5 s | Failure, `sensorFailure` |
| Magnetometer data age | > 2.5 s | Mag marked failed; if in a running mission, **switch to RTH** (only graceful degradation) |
| Horizontal distance from origin, on ground (Init/Startup) | > 2 m | Failure, `positionOutOfBounds` |
| Tilt, on ground | > 20° | Failure, `attitudeOutOfBounds` |
| Horizontal distance, in flight | > 100 m | Failure |
| Tilt, in flight | > 120° | Failure |

- Kinematic checks are skipped when the mission sets `disableKinematicSafety` (bellyflop, freefall, RTH).
- The distance is horizontal only: `position.magnitude(3,5)` is end-exclusive and covers Px, Py.

### Actuator gating

- `vehicleIsReady()` false → `vehicleShutdownControl(true)`: TVC, flaps and simulated actuators are disabled. **In flight this means the vehicle falls.** This is the current policy for every latched failure, including control-source loss.
- Otherwise actuators follow `mission->getActuatorsEnabled()`. Flaps are always on in freefall.
- **Arming (T9b-4):** the legacy arm state, the `Arm` telecommand, the 10 % power limit and every auto-disarm rule are **removed**. "Armed" in the telemetry means the motor interlock is open. The motors need the control system to command them (a go) **and** the interlock (motors armed, SD) open; with the interlock open they run at **full power**. A go is **refused while a calibration is invalid** (all three calibrations must be good, [calibration.md](calibration.md)) and has a fixed 15 s countdown (`kGoCountdownParamInt`; the countdown gyro bias run uses a 3 s still window so it fits the 5 s Initialisation phase, calibration.md). The interlock is not a go precondition.
- Motors are enabled only by a go (`enableMotors(true)`), **in simulation mode exactly as in real mode** (T9b-4, owner decision 2026-11-21, safety.md rule 9: the simulation gates nothing). **For every simulation or bench test the motors are electrically disconnected (ESCs unpowered); SD is only an additional measure.**
- **Hard motor interlock (T9b-2, 2026-11-22):** in addition to the above, the motor outputs are driven only while `Safety::MotorInterlock` is open: an **observed motors DISABLED -> ENABLED transition** of the latched GS session **and** an accepted ENABLED motors state at most **1 s** old. It is a pure gate at the last point before the PWM (`StarshipTVC::taskThread`, `Safety::evaluateMotorGate`) plus a **100 Hz watchdog task** (priority 120, above the controllers) that forces both motor outputs to duty 0 while the gate is closed; the watchdog itself is monitored (not run for 100 ms: failsafe, `failure_bits` bit 13). It never reads the simulation flag and simulation does not change it; the vehicle boots in real mode. **Flight-safety consequence:** with no valid ENABLED state for 1 s (Pocket off, link loss, SD down) the motors stop and the vehicle falls; a gap of 1 to 2 s reopens by itself when ENABLED states return (D3-B), a longer gap, a source change, boot, the SE reset, an SC change, the end of a mission chain, mode Failure and a ground reset-and-stop each need a fresh SD off -> on (D4-A). Telemetry: `MOTORS_ENABLED`, `ctrl_flags` `INTERLOCK_NEED_TOGGLE`; details and the owner's bench procedure in [hardware-and-drivers.md](hardware-and-drivers.md#5-exvectrnetwork-links-srccomms).
- **Simulated thrust (D16 stays REJECTED, owner decision 2026-11-21, design 1.3):** the "actual thrust" that the body simulator consumes is computed exactly as before T9b-2, independent of the interlock; simulation never takes the interlock into account. A simulated RC gap therefore does not make the simulated vehicle fall. The interlock gates only the motor PWM.
- **Done in T9b-4 (owner decision 2026-11-21):** the legacy `enableMotors(!simulationMode_)` simulation gate is removed, so the simulation never gates the motors (safety.md rule 9).

### Known safety gaps (decide before the next flight)

- Control-source loss (CMD_STATE older than 2 s while committed, which now includes the ground-station app or PC dying) or a single 0.5 s sensor dropout in flight cuts all actuators instead of attempting RTH or a controlled descent.
- (Fixed 2026-11-20: a SystemReset during a mission no longer lets the chain continue, and the vehicle no longer latches Failure at every boot.)
- (Fixed 2026-11-20: the first bellyflop after boot no longer differs from later ones; RTH after an abort or mag timeout from freefall now has its controllers.)
- The bellyflop kick (2026-11-21) is tuned in the simulator only, whose aerodynamics are known to be wrong (bench: tilt 2° → 75° in 0.95 s at 11 N, then belly-down and near horizontal). Review the kick in the first flight data ([missions.md](missions.md) Known issues).
- A go does not need the interlock (owner decision); the motors need the interlock (motors armed toggled and fresh), so a vehicle without a toggled SD cannot spin the motors. **With the interlock open the motors run at full power (no 10 % limit any more), also in simulation:** motors disconnected for every bench test.
- T9b-2: the interlock watchdog failsafe (100 ms) is an open item, see the Known issues of [hardware-and-drivers.md](hardware-and-drivers.md#known-issues--limitations).
- (Gone with T9b-4: the leftover arm state after leaving simulation; there is no arm state any more.)
- Altitude is not bounded by the out-of-bounds check.
- Estimator and controller bugs that affect flight safety are listed in the Known issues sections of [libs/ExVectrDSP.md](../libs/ExVectrDSP.md), [libs/ExVectrControl.md](../libs/ExVectrControl.md) and [missions.md](missions.md).

## Operating the vehicle

The safety commands are the ground station's command state (`CMD_STATE`), which it derives from the Pocket switches (or its own virtual switches and buttons): SA go / abort, SB profile, SC vehicle mode, SD **motors armed / disarmed**, SE reset / zero, plus the GS FAILSAFE, ABORT and MOTORS OFF buttons (table and rules: [hardware-and-drivers.md](hardware-and-drivers.md#5-exvectrnetwork-links-srccomms), "Command path (T9b-4)"). The vehicle ignores the Pocket itself. The temporary telecommand on port 0x7F (calibration, the simulation-only commands) stays for the rest of T9b-5; `ZERO_ESTIMATORS` and `SET_STOP_ALTITUDE` are session-checked `REQUEST`s; the safety telecommands are removed.

Typical sequence:
1. Start the ground station and power on with every switch in its safe position (SA down, SC 0 = real, SD disarmed, SE released) -> Startup. **Keep the vehicle still**: after initialisation it zeroes its estimators for 10 s (`ZEROING` shown), then **Ready**. The vehicle boots in **real mode**.
2. SC: flip SC to select simulation (SC 1 or 2) or real (SC 0); the vehicle must be on the ground and not committed. The field must equal the applied mode (`SC_MISMATCH` clear) for a go.
3. SB to the profile (2 flight, 0 hover test, 1 waypoint test; the test missions are for low-altitude control testing, missions.md section 7a).
4. SD down -> up (motors armed): the motor interlock opens (`ARMED`); required for the motors, not for the go.
5. SA down -> up: go. The waypoint mission runs through Initialisation (estimator zeroing, the countdown gyro run) -> Startup (TVC actuator test) -> Running; the countdown is 15 s. A refused go shows `go_refusal`.
6. SA down at any time: abort. In flight it starts RTH at once from any phase (hover, ascent, bellyflop, freefall; if RTH is already running it continues); on the ground (before liftoff: countdown, Idle mission, RTH landed; decided by mission mode only, the waypoint mission after T0 always counts as in flight) the mission is reset, motors and actuators go off, the vehicle returns to Ready and nothing follows; latches are never cleared by an abort.
   SE short press on the ground with the interlock closed: the system reset (state and mission chain reset to the boot state, no zeroing, simulation mode unchanged, in simulation mode the simulated body back to its start pose, latches cleared). To re-zero on the ground (new start spot, vehicle moved): hold SE >= 1 s (zeroes while held) or the `ZERO_ESTIMATORS` request (seconds), interlock closed, vehicle still.
7. After landing the RTH finishes, the chain ends and the interlock needs a new motors toggle.

Sensor calibration (gyro automatic; accel 12 poses, mag tumble): `CalibStart` (subsystem 0/1/2, paramInt 1) → poses / tumble → `CalibStart` with 0 → `CalibApply` (PASS or WARN only) → `Save` (EEPROM write and read-back). Procedure, validity rules and log cues: [project/calibration.md](calibration.md).

## Simulation mode

Selected by an observed change of the `vehicle_mode` field of the ground station's command state (SC; T9b-4; the `SimulationMode` telecommand is removed): on the ground, not committed, and not in mode Failure. The vehicle boots in real mode. Simulation gates nothing (safety.md rule 9).
- The three topic switches route attitude, position and ground distance from `BodySimulator` instead of the estimators.
- The simulator consumes the TVC *actual thrust* topic and the true flap angles, so the full control and mission stack runs unchanged. **The motors are NOT disabled by the simulation (T9b-4):** they are driven exactly as in flight when a go has been accepted and the interlock is open, so the motors are electrically disconnected for every simulation.
- **Model (redesigned 2026-11-21, `feature/sim-aero`, [libs/ExVectrDSP.md](../libs/ExVectrDSP.md) "Body simulator"):** the owner's geometry and measured mass/inertia, strip crossflow drag with rotation damping, slender-body potential normal force (AoA-dependent centre of pressure), flat-plate flap normal force with stall, full rigid-body equations (`ω × Iω`), fixed 2 ms steps (bounded catch-up, no step over a pause). One documented calibration factor (4.64 on all aerodynamic forces with the fold shield 0.7, 2026-11-21; ExVectrDSP.md) makes the belly-down fall at neutral flaps 6.4 m/s (real 21..25 km/h); the geometry alone gives 13.5 m/s. Geometry: bottom flaps 1 cm above the bottom of the ship (21 cm long), top flaps canted 5° inward about their lower hinge point. Actuators: modelled flap angle 300°/s, TVC thrust-angle slew 20°/s (= 300°/s vane servo / gain 15), thrust instantaneous. Off-target harness: `pio test -e native -f test_sim_aero` (before/after table in the library doc).
- **Wind and gusts (simulation only, owner-approved exception to safety.md rule 7, approved 2026-11-21):** steady wind and a deterministic seeded gust process, off by default; set at run time with the telecommands `SimWindSpeed` (11), `SimWindDirection` (12), `SimGustStrength` (13, 0..10 m/s), `SimGustPeriod` (14, 0.5..30 s) and `SimGustSeed` (15) (hardware-and-drivers.md section 10), refused outside simulation mode, reset to zero (wind and gusts) when simulation mode switches on or off. **Wind and gusts persist across the SE reset** on purpose (a series of runs in the same wind); send wind 0 / gust 0 or toggle simulation mode to clear them. The gust model is described in ExVectrDSP.md. They change only the simulated air, nothing in the control, mission or safety path.
- **Flap servo faults (simulation only, owner-approved exception to safety.md rules 7 and 9, approved 2026-11-21):** `SimFlapStuck` (16), `SimFlapSlow` (17) and `SimFaultClear` (18) (hardware-and-drivers.md section 10) make a flap's modelled TRUE angle stick (immediately, at the next 20 ms flap tick, flaps enabled or not) or slew slowly, to reproduce the real-flight failure of a stuck top-left servo in the descent. In simulation mode only the physical servo PWM shows the fault too (owner decision 2026-11-22, `Safety::ServoOutputStage::servoCommandRad`, the one place where flap angles become PWM in `StarshipFlaps`): a stuck flap's PWM uses the forced angle (clamped to the 15..90 deg servo range), a slowed flap's PWM slews at the normal servo rate x factor, release slews back at the normal rate (no jump); the flag is read on every call, real-mode output is bit-identical with and without any fault state, motors are never affected. The true angle is altered at its one publication point (`TrueFlapModel::publishAngles` in `StarshipFlaps`, published every tick since 2026-11-21), which is what the body simulator and the ACTUATORS / AIR_ACTUATORS telemetry read. Guards: the command decision (`Safety::decideSimFault`, `src/safety/sim_flap_fault.*`) refuses everything unless simulation mode is on; `SimFlapFaults::set` refuses again without the flag; the model (`StarshipFlaps::taskThread`) reads the live simulation flag at every update and returns the unmodified angle and rate factor 1.0 (bit-identical, host-tested) when it is false; all faults are cleared on every simulation mode switch, by the SE reset (review 2026-11-21: they used to carry over into the next run) and by the clear command; a released stuck fault also drops the servo model's forced angle at once. The former "arm for the descent" mode (subsystem bit 0x10) was removed on 2026-11-21 and is refused. A log line states each set and clear.
- Toggling back resets to Startup (then Ready without zeroing if a valid zero exists since boot).
- Estimator zeroing (boot zero, `ZeroEstimators`, mission Initialisation) acts on the estimators **and** the body simulator in both modes. While a zeroing window is active the simulated body is held at its start pose (`BodySimulator::resetToStartPose()`: upright, q = identity, at the origin, at rest); since 2026-11-20 (feature/review-fixes) that is the same pose as at simulation start (it was 1 m north and yawed 180°, DSP Known issues).
- Entering simulation (SC) puts the simulated body to its start pose.
- **Sim/flight difference (review fix 2026-11-20, user-approved): the SE reset in simulation mode also puts the simulated body back to its start pose.** The simulated body is the world, not an estimator: a run that ended tilted, fallen or away from the origin would otherwise carry over into the next run and re-trigger `attitudeOutOfBounds` / `positionOutOfBounds` at once. The estimators are still not touched by `SystemReset` in either mode. In real mode nothing physical can be reset: the estimators keep their origin until `ZeroEstimators` (or the boot zero, or the mission's Initialisation zeroing), so **a vehicle that was carried needs `ZeroEstimators`** before the next flight. Bench (simulation, 2026-11-20): a run ended at pN 0.035 m, pW −0.122 m, yaw 9° (q = [0.9966, −0.0198, −0.0054, −0.0798]); after `SystemReset` the reported pose was the origin and q = [1, 0, 0, 0] within one status line. A drop from 19 m (disarm in the ascent) ended at (0.277, 0.066, 0) m with 3.6° tilt; after `SystemReset` origin and identity, Ready, and the next `MissionBegin` started in Init at the origin, upright, no Failure.

Intended sim/flight differences, complete list (safety.md rule 7): (1) attitude, position and ground distance come from the body simulator instead of the estimators; (2) *(removed in T9b-4: the simulation no longer keeps the motors off; nothing gates on the simulation flag, safety.md rule 9)*; (3) `SystemReset` also resets the simulated body to its start pose; (4) *(removed 2026-11-21: the calibration check applies in simulation mode exactly as in real mode, the calibration commands work in both; owner decision, [calibration.md](calibration.md). No arm or go logic reads the simulation flag.)* (5) The sim wind and gust telecommands (11..15) act only in simulation mode (they change only the simulated air: steady wind, and deterministic seeded gusts; **an owner-approved exception to safety.md rule 7, approved 2026-11-21**). (6) The flap servo fault telecommands (16..18) alter only the modelled true flap angle and the flap servo PWM, and only in simulation mode (**owner-approved 2026-11-21, PWM part 2026-11-22**; see "Flap servo faults" above).

Use it on the bench for every Development verification (see [../development-workflow.md](../development-workflow.md#flight-release-development--master)). The simulator's remaining limits (calibration factor, CoM position, uncertain flap azimuth authority, no sensor model) are in the Known issues of [libs/ExVectrDSP.md](../libs/ExVectrDSP.md). A passing simulation is therefore necessary but not sufficient. **Bench (simulation, 2026-11-21, `feature/sim-aero`, full chain):** ascent and hover stable (tilt 0°); bellyflop kick ended by the angle trigger after 0.91 s (77° tilt at the cut); freefall held belly-down (belly angle 2-22°) at 7.1-7.5 m/s down, gliding 3-4 m/s toward the nose, with the flaps saturating 15/90° in a roll/azimuth limit cycle (body rates ±50-95°/s); the RTH flip from 27 m overshot and limit-cycled (tilt 27-105°, rates up to 200°/s) and the simulated vehicle hit the ground tilted at 7 m/s; no failure bits, no NaN; no RTH auto-disarm (still armed after 4 min). With the aerodynamic calibration set to 1 (experiment, not committed) the flip still overshot: the TVC attitude loop is tuned for the old, unrealistically fast vane model (missions.md Known issues).

## Telemetry and display

- Downlink rates and ports: [codebase-structure.md](../codebase-structure.md#data-flow-topic-graph).
- USB carries the full telemetry set (pose 100 Hz, actuators incl. flap angles 50 Hz, status/mission 20 Hz, GNSS, debug packets) and the console as `LOG_TEXT`; the UART to the RX module carries the radio set (status 5 Hz, `AIR_POSE` 25 Hz, `HANDSET_TELEM` 2 Hz). USB serial is binary: use the ground station, not a serial monitor.
- The 128×32 OLED shows velocity/position per axis, satellite count, position accuracy, tilt, heading and link status.
