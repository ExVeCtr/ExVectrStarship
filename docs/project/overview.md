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
Startup ──all sensors+filters publishing──► Ready ──MissionBegin──► Running
   ▲                                          │                        │
   └──────────── SystemReset (0xC5) ◄──── Failure ◄── any failure ─────┘
SensorCalib (mag calibration telecommand) ── disarms, blocks Ready
```

- **Startup → Ready:** every input (GNSS, gyro, mag, baro, both estimators) has published at least once.
- **Failure latches.** Once in Failure, only a `SystemReset` telecommand clears it.

### Failure detection (`checkForFailures`)

| Check | Threshold | Effect |
|---|---|---|
| Ground station (node 0) lost: no valid frame (heartbeat or data) from node 0 on USB or UART | 2 s (`VehicleComms::isGroundStationPresent`, same 2 s as the old NetworkNode) | Failure, `radioConnectionLoss`. **Until T9b:** over the radio path only node 0 data frames reach the vehicle (the modules drop its heartbeats), see [hardware-and-drivers.md](hardware-and-drivers.md#5-exvectrnetwork-links-srccomms) |
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
- **Arming** (`Arm`, 0xA5) only sets the motor power limit: 100 % when armed, 10 % when disarmed. It is *not* required for `MissionBegin`. Disarm is forced in Failure, SensorCalib, Startup and SensorInit, and automatically after RTH finishes.
- Motors are switched on only by `MissionBegin`, and never in simulation mode.

### Known safety gaps (decide before the next flight)

- Radio loss or a single 0.5 s sensor dropout in flight cuts all actuators instead of attempting RTH or a controlled descent.
- Arming does not gate mission start.
- Altitude is not bounded by the out-of-bounds check.
- Estimator and controller bugs that affect flight safety are listed in the Known issues sections of [libs/ExVectrDSP.md](../libs/ExVectrDSP.md), [libs/ExVectrControl.md](../libs/ExVectrControl.md) and [missions.md](missions.md).

## Operating the vehicle

Telecommands arrive on port 0x7F (TEMPORARY adapter of the old port-1000 `Telecommand`, from node 0 on USB or radio, until T9b replaces it with the RC-state command path). Full list and byte layout: [hardware-and-drivers.md](hardware-and-drivers.md#10-telecommands-srctelecommandhpp-temporary-adapter-on-port-0x7f).

Typical sequence:
1. Power on → Startup. Wait for **Ready** (all sensors initialised, ground link up).
2. Optional: `SimulationMode` (0xA9) to fly the mission against the body simulator.
3. `Arm` (0xA5).
4. `MissionBegin` (paramInt = start delay in s). The waypoint mission runs through Initialisation (estimator zeroing) → Startup (TVC actuator test) → Running.
5. Abort at any time with `MissionAbort` (in Running → RTH; otherwise shutdown), or `SystemReset` (0xC5) as an emergency stop.
6. After landing, RTH finishes and the vehicle auto-disarms.

Magnetometer calibration: `CalibStart` (subsystem Magneto, paramInt > 0) → rotate the vehicle → `CalibStart` with 0 to end → `CalibApply` → `Save` (writes EEPROM).

## Simulation mode

Toggled by `SimulationMode` (0xA9).
- The three topic switches route attitude, position and ground distance from `BodySimulator` instead of the estimators.
- The simulator consumes the TVC *actual thrust* topic and the true flap angles, so the full control and mission stack runs unchanged with motors disabled.
- Toggling back resets to Startup.

Use it on the bench for every Development verification (see [../development-workflow.md](../development-workflow.md#flight-release-development--master)). The simulator has known model bugs: TVC clamp direction, wind frame, zeroing yaw. See [libs/ExVectrDSP.md](../libs/ExVectrDSP.md). A passing simulation is therefore necessary but not sufficient.

## Telemetry and display

- Downlink rates and ports: [codebase-structure.md](../codebase-structure.md#data-flow-topic-graph).
- USB carries the full telemetry set (pose 100 Hz, actuators incl. flap angles 50 Hz, status/mission 20 Hz, GNSS, debug packets) and the console as `LOG_TEXT`; the UART to the RX module carries the radio set (status 5 Hz, `AIR_POSE` 10 Hz, `HANDSET_TELEM` 2 Hz). USB serial is binary: use the ground station, not a serial monitor.
- The 128×32 OLED shows velocity/position per axis, satellite count, position accuracy, tilt, heading and link status.
