# Starship Vehicle: System Overview

> What the vehicle does, how a flight works end to end, how it is controlled and kept safe, and how to operate it. Details per mission: [missions.md](missions.md). Hardware, drivers, wire formats: [hardware-and-drivers.md](hardware-and-drivers.md).

## Purpose

An autonomous model of SpaceX's Starship (≈1.1 kg). It serves as the flagship application and hardware-in-the-loop testbed for the ExVectr embedded framework. Flight goals:

1. Lift off and hover using **thrust-vector control (TVC)**.
2. Ascend to altitude.
3. Perform the **bellyflop**: rotate to horizontal, belly down.
4. Descend in a **flap-stabilised freefall**.
5. Flip back upright, decelerate, **return to home** and land.

## Vehicle

| Item | Value | Source |
|---|---|---|
| Mass | 1.1 kg | `VEHICLE_MASS_KG`, `starship_hardware.h` |
| Max thrust | 14 N (T/W ≈ 1.3) | `TVC_THRUST_LIMIT_N` |
| Propulsion | 2 contra-rotating motors (CW pin 8, CCW pin 7); differential speed gives roll torque | `starship_tvc.*` |
| TVC | 4 vane servos (±45° servo travel), mechanical ratio 15 → ±3° thrust deflection | `TVC_SERVO_LIMIT`, `TVC_ANGLE_FACTOR` |
| Aero surfaces | 4 flaps (upper L/R, lower L/R), one servo each | `starship_flaps.*` |
| Flight computer | Teensy 4.0; MPU9250 IMU (SPI, interrupt-driven), BME280 baro (I2C), QMC5883 mag (I2C), VL53L0X ToF range (I2C), u-blox NEO-M8Q GNSS (Serial5), ExVectrNetwork links: USB serial (bench ground station) and a 460800 UART (Serial4) to the ExVectrLink RX module (no on-board radio since T9a), SSD1306 OLED, EEPROM | `board_v_1_0.h` |
| TVC lever arm (sim) | 0.35 m below CoM | `main.cpp` `setTVCInputTopic(..., {0,0,-0.35})` |
| Inertia (sim) | diag(0.044, 0.044, 0.004) kg·m² | `BodySimulator` constructor in `main.cpp` |

## Flight profile (current configuration)

Configured in `VehicleSafetyAndControlTask::taskInit()` in `main.cpp`. MissionBegin always starts the waypoint mission; `MissionSelect` is currently a no-op.

```
MissionWaypoint ──► MissionBellyflop ──► MissionFreefall ──► MissionRTH (default) ──► landed, auto-disarm
 1. 0.3 m   (settle/init)        rotate to belly-down     flaps stabilise,        flip upright, decelerate,
 2. 1.0 m   hover check          (≤1 s, or ≥50° reached,  steer toward home,      translate home, descend,
 3. 120 m   ascent at 7 m/s       or ≥150°/s rate)        stop altitude 20 m      land at {0,0,·}
```

Any mission ending without a successor, an abort telecommand, or a magnetometer timeout in flight switches to **RTH**, the universal fallback. Waypoint arguments and phase logic: [missions.md](missions.md).

## Guidance, navigation and control

**Navigation** (ExVectrDSP, [libs/ExVectrDSP.md](../libs/ExVectrDSP.md)):
- `IMUAttitudeEKF` at 1 kHz: gyro propagation, accelerometer tilt correction, magnetometer yaw correction. Outputs `[ω, q]`.
- `IMUGPSPositionKalman` at 100 Hz: accelerometer-driven prediction in NWU, updated by baro altitude, GNSS horizontal position/velocity and a separate ground-distance filter from the ToF rangefinder (0.4–1.5 m window). Outputs `[V, P]`.
- Sensor calibration: per-sensor `TopicCoordTransform` (`T·(v − bias)`) loaded from persistent memory at boot (keys in `memory_keys.hpp`). Gyro bias is currently **hard-coded** in `initialiseMemory()` and overrides the stored value.

**Control cascade** (ExVectrControl, [libs/ExVectrControl.md](../libs/ExVectrControl.md)):

```
mission setpoint [V,P] → ControlPositionStandard (P on position → PI on velocity, +g) → accel_ref (world)
accel_ref → ControlMappingAccToAtt (tilt toward accel, ≤35°, yaw 0) → q_ref
q_ref, accel_ref, [ω,q] → ControlAttitudeTvc (P + rate P + I) → [Fx, Fy, Fz, τz] body
→ StarshipTVC mixer → vane angles + motor speeds
```

- **Bellyflop:** `MissionBellyflop` unsubscribes the TVC controller from the mapper and commands a fixed −90° pitch attitude directly, then hands back.
- **Flaps:** `FlapModeObserverTask` (10 Hz) picks the `BellyFlopMode` (Ascent / Upright / Stabilize) and target azimuth/pitch per mission. `ControlAttitudeFlaps` turns it into four flap angles. In freefall above 100 m it steers the belly toward home; below stop altitude + 20 m it pitches to 30°.

## Modes and safety

### Vehicle mode machine (`VehicleSafetyAndControlTask`, 10 Hz)

```
Startup ──all inputs published AND valid zero since boot──► Ready ──MissionBegin──► Running
   │  (no zero yet: automatic 10 s boot zero, ZEROING flag)    │                       │
   ▲                                                           │                       │
   └──────────── SystemReset (0xC5) ◄──────────── Failure ◄──── any failure ───────────┘
SensorCalib (mag calibration telecommand) ── disarms, blocks Ready, aborts a zeroing window
```

- **Startup → Ready:** every input (GNSS, gyro, mag, baro, both estimators) has published at least once **and** a valid estimator zero exists since boot, with no zeroing window active (`EstimatorZeroing::readyAllowed()`, `src/safety/estimator_zeroing.hpp`). Without a valid zero the vehicle stays in Startup.
- **Failure latches.** Once in Failure, only a `SystemReset` telecommand clears it. SystemReset resets the system state only (failures, mode, mission, disarm, actuators off); the estimators keep running untouched. With a valid zero since boot the vehicle returns to Ready as soon as all inputs have published again (within 0.5 s on the bench), without zeroing.
- **Boot:** the GNSS driver blocks the scheduler for about 1.3 s at t ≈ 5 s, so today the vehicle latches Failure (sensorFailure) right after initialisation at every boot (Known issues in [hardware-and-drivers.md](hardware-and-drivers.md#known-issues--limitations)). The automatic boot zero still runs and completes; a SystemReset afterwards gives Ready.

### Estimator zeroing (user decision 2026-11-20)

Zeroing and the system reset are separate. Zeroing runs the existing `enableZeroingMode` of the position KF, the attitude EKF and the body simulator for a **window timed on the vehicle** (`EstimatorZeroing`, `src/safety/estimator_zeroing.*`, applied by `VehicleSafetyAndControlTask::applyEstimatorZeroing()`):

| Estimator | What zeroing does ([libs/ExVectrDSP.md](../libs/ExVectrDSP.md)) |
|---|---|
| Position KF | State `[V, P]` decays to 0 (`x *= 0.95` per 10 ms update), the barometric reference altitude and the GNSS lat/lon reference and the range zero point are IIR-slewed to the current readings (factor 0.1 per sample), and the accelerometer bias is learned assuming the vehicle is at rest. So altitude, vertical velocity and horizontal position go to 0 relative to the new origin. |
| Attitude EKF | The gyro bias is learned (IIR 0.005 per 1 kHz sample, only while \|ω\| < 10°/s). The attitude itself is **not** reset: it is absolute (tilt from gravity, heading from the magnetometer in the NWU frame), so it keeps its physical value. |
| Body simulator | Holds the simulated body at its zeroing pose (`p_N = 1 m`, q = (0,0,0,1): a known DSP bug, Known issues there). Applied in simulation and flight alike, as before. |

**The vehicle must be kept still** for the whole window (it learns biases assuming zero motion). `VEHICLE_STATUS` `ZEROING` (flags bit 7) is high for the window so the ground station can show it.

- **Automatic boot zero:** once all inputs have published after boot (the point where the old Startup went to Ready) and no valid zero exists, the vehicle zeroes for 10 s and only then becomes Ready. It also restarts after a SystemReset or mag calibration aborted it before any zero completed. It is identical in simulation and flight.
- **`ZeroEstimators` telecommand** (type 10, `paramInt = 0x2E`, `paramFloat` = window in s, NaN or 0 = 10 s, clamped 1..60 s): accepted only when initialised, **disarmed**, motors off, mode Startup or Ready and the active mission `Idle` or `Finished` ("on the ground": no mode in which the vehicle can fly and no mission running). Otherwise ignored, counted and logged (at most once per second). A second one during a window restarts it with the new duration. One-shot ground action (safety.md 2.5): the end time lives on the vehicle, nothing depends on the sender staying connected.
- **During a window:** `Arm` (0xA5) and `MissionBegin` are refused (disarm is always accepted); actuators stay shut down (`vehicleShutdownControl(true)` while the window is active, in addition to the normal gating); the mode does not change (Ready stays Ready). No failure check depends on the zeroing: the kinematic (tilt / distance) checks run only while a mission is active, which a window excludes, and the estimators keep publishing, so the data-age checks are unaffected. Failure detected during a window latches as usual; the window continues and still records its zero.
- **End:** a window that runs to its end records a **valid zero since boot** (only a reboot clears it). `SystemReset` and `CalibStart` (mag calibration) **abort** an active window without recording a zero; an earlier valid zero is kept.
- The waypoint mission's own Initialisation phase still zeroes the estimators before every flight, as before (missions.md 6.2).
- **After SystemReset the estimators keep the old origin.** To start a flight from a new spot, or after the vehicle was carried around, send `ZeroEstimators` (or rely on the mission's Initialisation zeroing). In simulation, the simulated body stays where the last run left it after SystemReset; toggle `SimulationMode` off and on to reset it to the origin (as before).

**Why the boot zero matters (bench 2026-11-20):** the position KF starts predicting before the attitude EKF publishes (the EKF task starts at t = 5 s); with a zero quaternion it integrates −g, so at the first estimate the altitude is already about −300 m at −89 m/s. Without a zero the barometer only slowly pulls it back: on 7ac9b96 the bench vehicle read about −427 m with +0.8 to +1.2 m/s while standing still (the "−440 m" symptom). After the boot zero: altitude within ±0.03 m at the end of the window. 60 s stationary after a `ZeroEstimators`: altitude mean +0.06 m, σ 0.25 m, range −0.54..+0.52 m, linear trend +3.5 mm/s; vertical velocity mean −0.09 m/s (σ 0.07), horizontal position < 7 mm. The remaining vertical-velocity offset is the accelerometer bias learned with the known `updateAccBias` frame bug (DSP Known issues), estimator not changed.

### Intended RC mapping for reset and zeroing (T9b requirement, not implemented)

Agreed 2026-11-20 for task T9b (the RC-state to command mapping), Pocket switch **SE**:
- **Short press** (released within 1 s): `SystemReset` behaviour (state reset, no zeroing).
- **Held longer than 1 s:** starts zeroing and continues while SE is held; zeroing ends on release.
- The hold variant needs a **hold state on the vehicle**: the vehicle zeroes while it keeps receiving the held state (fresh RC state) plus a short grace, which differs from the fixed window of the `ZeroEstimators` telecommand. T9b must reconcile the two, for example: zero while the hold state is fresh, and keep the timer as the ground-station variant. The same preconditions apply (disarmed, on the ground).
- Note for the workspace `docs/safety.md` 4.2, which says a latched condition is cleared by "SE held at least 1 s": with this mapping the latch is cleared by the short press, and the hold zeroes. The main session must align that text.

### Failure detection (`checkForFailures`)

| Check | Threshold | Effect |
|---|---|---|
| Radio link lost: no radio packet = DATA frame from node 0 or node 2 arriving on the UART (normally the Pocket's `RC_STATE`); heartbeats, node 3 frames and USB never count | 2 s (`VehicleNetwork::isRadioLinkAlive`); before the first radio packet since boot, 10 s after power-on | Failure, `radioConnectionLoss`. The ground station on USB does not influence it (since 2026-11-20). Details and the boot grace: [hardware-and-drivers.md](hardware-and-drivers.md#5-exvectrnetwork-links-srccomms) |
| GNSS, gyro, baro, attitude-EKF or position-KF data age | > 0.5 s | Failure, `sensorFailure` |
| Magnetometer data age | > 2.5 s | Mag marked failed; if in a running mission, **switch to RTH** (only graceful degradation) |
| Horizontal distance from origin, on ground (Init/Startup) | > 2 m | Failure, `positionOutOfBounds` |
| Tilt, on ground | > 20° | Failure, `attitudeOutOfBounds` |
| Horizontal distance, in flight | > 100 m | Failure |
| Tilt, in flight | > 120° | Failure |

- Kinematic checks are skipped when the mission sets `disableKinematicSafety` (bellyflop, freefall, RTH).
- The distance is horizontal only: `position.magnitude(3,5)` is end-exclusive and covers Px, Py.

### Actuator gating

- `vehicleIsReady()` false → `vehicleShutdownControl(true)`: TVC, flaps and simulated actuators are disabled. **In flight this means the vehicle falls.** This is the current policy for every latched failure, including radio loss.
- Otherwise actuators follow `mission->getActuatorsEnabled()`. Flaps are always on in freefall.
- **Arming** (`Arm`, 0xA5) only sets the motor power limit: 100 % when armed, 10 % when disarmed. It is *not* required for `MissionBegin`. Disarm is forced in Failure, SensorCalib, Startup and SensorInit, and automatically after RTH finishes. The forced-disarm log line ("Vehicle disarmed due to ...") is logged once when the reason changes and then every 5 s while it holds (`disarmLogDue`, main.cpp:335); the disarm itself still runs every 100 ms tick.
- Motors are switched on only by `MissionBegin`, and never in simulation mode.

### Known safety gaps (decide before the next flight)

- Radio loss or a single 0.5 s sensor dropout in flight cuts all actuators instead of attempting RTH or a controlled descent.
- A SystemReset during a mission lets the mission chain continue to an RTH hot start in Ready (servos active, motors off) until a second SystemReset ([missions.md](missions.md) Known issues).
- The vehicle latches Failure at every boot (GNSS driver blocking delay); a SystemReset after boot is part of the procedure until fixed.
- Arming does not gate mission start.
- Altitude is not bounded by the out-of-bounds check.
- Estimator and controller bugs that affect flight safety are listed in the Known issues sections of [libs/ExVectrDSP.md](../libs/ExVectrDSP.md), [libs/ExVectrControl.md](../libs/ExVectrControl.md) and [missions.md](missions.md).

## Operating the vehicle

Telecommands arrive on port 0x7F (TEMPORARY adapter of the old port-1000 `Telecommand`, from node 0 on USB or radio, until T9b replaces it with the RC-state command path). Full list and byte layout: [hardware-and-drivers.md](hardware-and-drivers.md#10-telecommands-srctelecommandhpp-temporary-adapter-on-port-0x7f).

Typical sequence:
1. Power on → Startup. **Keep the vehicle still**: after initialisation it zeroes its estimators for 10 s (`ZEROING` shown). Today it then sits in Failure (boot issue above): send `SystemReset` → **Ready** (no new zero).
2. Optional: `SimulationMode` (0xA9) to fly the mission against the body simulator.
3. `Arm` (0xA5).
4. `MissionBegin` (paramInt = start delay in s). The waypoint mission runs through Initialisation (estimator zeroing) → Startup (TVC actuator test) → Running.
5. Abort at any time with `MissionAbort` (in Running → RTH; otherwise shutdown), or `SystemReset` (0xC5) as an emergency stop (state reset only, no zeroing).
   To re-zero on the ground (new start spot, vehicle moved): `ZeroEstimators` (type 10, 0x2E, seconds), disarmed, vehicle still.
6. After landing, RTH finishes and the vehicle auto-disarms.

Magnetometer calibration: `CalibStart` (subsystem Magneto, paramInt > 0) → rotate the vehicle → `CalibStart` with 0 to end → `CalibApply` → `Save` (writes EEPROM).

## Simulation mode

Toggled by `SimulationMode` (0xA9).
- The three topic switches route attitude, position and ground distance from `BodySimulator` instead of the estimators.
- The simulator consumes the TVC *actual thrust* topic and the true flap angles, so the full control and mission stack runs unchanged with motors disabled.
- Toggling back resets to Startup (then Ready without zeroing if a valid zero exists since boot).
- Estimator zeroing (boot zero, `ZeroEstimators`, mission Initialisation) acts on the estimators **and** the body simulator in both modes, so there is no new simulation/flight difference. The simulator needs no zero of its own to start a simulation: `SimulationMode` on resets the simulated body to the origin (as before). Since 2026-11-20 `SystemReset` no longer resets the simulated body; that is the same rule as for the real estimators, not a mode difference.

Use it on the bench for every Development verification (see [../development-workflow.md](../development-workflow.md#flight-release-development--master)). The simulator has known model bugs: TVC clamp direction, wind frame, zeroing yaw. See [libs/ExVectrDSP.md](../libs/ExVectrDSP.md). A passing simulation is therefore necessary but not sufficient.

## Telemetry and display

- Downlink rates and ports: [codebase-structure.md](../codebase-structure.md#data-flow-topic-graph).
- USB carries the full telemetry set (pose 100 Hz, actuators incl. flap angles 50 Hz, status/mission 20 Hz, GNSS, debug packets) and the console as `LOG_TEXT`; the UART to the RX module carries the radio set (status 5 Hz, `AIR_POSE` 10 Hz, `HANDSET_TELEM` 2 Hz). USB serial is binary: use the ground station, not a serial monitor.
- The 128×32 OLED shows velocity/position per axis, satellite count, position accuracy, tilt, heading and link status.
