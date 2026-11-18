# Codebase Structure

> What lives where, what belongs where, and how the pieces connect at runtime.

## Repository layout

```
flight-software/            (repo ExVectr-Testing; lives in ~/Development/ExVectrStarship/)
├── platformio.ini            Single env: teensy40 (Teensy 4.0, IMXRT1062 600 MHz), Arduino framework
├── CLAUDE.md                 Entry point for Claude sessions → points here
├── docs/                     Versioned developer documentation (this folder)
├── .claude/skills/           Claude Code workflow skills (feature, release, docs upkeep)
├── src/                      Vehicle application ("Starship"): everything specific to THIS vehicle
│   ├── main.cpp              Object graph, topic wiring, system tasks, setup()/loop()
│   ├── mission/              Mission framework + missions (waypoint, bellyflop, freefall, RTH)
│   ├── starship_tvc.*        TVC mixer/actuator layer (4 vane servos + 2 contra-rotating motors)
│   ├── starship_flaps.*      Flap actuator layer (4 flap servos) + ControlAttitudeFlaps settings types
│   ├── datalink_sx1280.*     SX1280 2.4 GHz radio as an ExVectrNetwork datalink
│   ├── ublox_gnss.*          u-blox NEO-M8Q GNSS driver → SNSR::GNSS topic
│   ├── vl53l0x/              VL53L0X ToF rangefinder driver → SNSR::RangeFinder topic
│   ├── telemetry.hpp         Packed downlink structs (VehicleState, MissionState, enums)
│   ├── telecommand.hpp       Packed uplink struct + command list
│   ├── subsystems.hpp        Subsystem enum used by telecommands
│   ├── memory_keys.hpp       Persistent-memory keys (calibration, transforms)
│   ├── board_v_1_0.h         Flight-computer board pinout
│   ├── starship_hardware.h   Vehicle pinout + physical limits (mass, thrust, TVC angles)
│   ├── gnssdriver/           Vendored SparkFun u-blox library (gitignored)
│   └── sx12xxAL/             Vendored SX12xx library (gitignored)
├── lib/
│   ├── ExVectr*/             The ExVectr framework, one git submodule per library
│   └── Adafruit_*/           Display libraries (SSD1306 OLED), committed as plain folders
├── include/, test/           PlatformIO defaults, currently unused
└── utils/                    Helper scripts
```

## What belongs where

| If the code is… | Put it in | Examples |
|---|---|---|
| Hardware-agnostic infrastructure (time, scheduling, pub/sub, containers) | `lib/ExVectrCore` | new container, scheduler feature |
| An abstract hardware interface | `lib/ExVectrHAL` | new bus type interface |
| An implementation of a HAL interface for a platform | `lib/ExVectrArduinoPlatform` (or a new platform lib) | Teensy PWM, I2C, EEPROM |
| Pure math with no hardware/time dependency | `lib/ExVectrMath` | matrix ops, quaternion helpers |
| Generic estimation / filtering / calibration / simulation | `lib/ExVectrDSP` | EKF, calibrators, body simulator |
| Generic controllers usable on other vehicles | `lib/ExVectrControl` | PID, attitude controller |
| A reusable sensor driver | `lib/ExVectrSensor` | IMU, baro, magnetometer drivers |
| A reusable actuator abstraction | `lib/ExVectrActuator` | PWM protocols, H-bridge |
| Networking layers and protocol | `lib/ExVectrNetwork` | node, transport |
| Shared wire packet formats (ground station must match) | `lib/ExVectrPackets` | attitude/position/GPS packets |
| Persistent storage | `lib/ExVectrData` | memory manager, EEPROM backends |
| **Anything specific to this vehicle** (geometry, pinout, mixing, missions, safety policy, telemetry layout) | `src/` | flap mixer, mission profile, failure checks |

Rules of thumb:
- **Libraries must not include from `src/`** and must not know about Starship. If a library needs a vehicle parameter, take it as a constructor argument or setter.
- Dependencies point downward only: `Core ← HAL ← Platform`, `Math`, then `DSP/Control/Sensor/Actuator/Network/Data` on top. A known violation exists: `ExVectrDSP/body_simulator` includes ExVectrControl headers ([libs/ExVectrDSP.md](libs/ExVectrDSP.md)). Don't add more.
- **Drivers start in `src/` and move to a lib once stable.** The u-blox and VL53L0X drivers are at this stage.
- Physical constants and pins go in `starship_hardware.h` / `board_*.h`, not inline.
- `main.cpp` is currently a ~2000-line monolith that holds the system tasks (`VehicleSafetyAndControlTask`, `FlapModeObserverTask`, `CommsSystemStateTask`, `DisplayTask`, `MagnetometerCalibrationTask`). When touching one substantially, move it into its own `src/<name>.hpp/.cpp`; leave `main.cpp` with only object construction and wiring.

## Runtime model

- **One cooperative scheduler, no RTOS.** `loop()` calls `Core::getSystemScheduler().tick()`. Each tick runs **at most one** due task, chosen by `misses*10 + priority`. A task that blocks stalls everything, so never `delay()` or busy-wait in a task.
- **Tasks** derive from `Core::Task_Periodic` (interval in ns), register themselves with `Core::getSystemScheduler().addTask(*this)` in their constructor, and implement `taskInit()` (once) and `taskThread()` (each period). Most objects are global statics, constructed before `setup()`.
- **Topics** (`Core::Topic<T>`) are synchronous: `publish()` calls every subscriber's `receive()` immediately, in the publisher's context. Keep callbacks short. Never publish from an ISR unless every subscriber is ISR-safe (the MPU9250 interrupt path is the existing exception; see [libs/ExVectrCore.md](libs/ExVectrCore.md)).
- **Time** is `int64_t` nanoseconds (`Core::NOW()`, `Core::SECONDS`, `Core::MILLISECONDS` …). Data travels as `Core::Timestamped<T>`; sensor data as `Timestamped<DSP::ValueCov<float,N>>`.
- **Priorities in use:** MPU9250 driver 110, attitude EKF and position KF 105, everything else default 100.

### Conventions (summary; authoritative details in [libs/ExVectrMath.md](libs/ExVectrMath.md))

- Reference frame: **X-North, Y-West, Z-Up** (NWU), origin at the position reference set at reset.
- Attitude state: `Vector<float,7>` = `[ωx, ωy, ωz, qw, qx, qy, qz]`. The quaternion rotates body → reference (`q.rotate(v_body) = v_ref`).
- Position state: `Vector<float,6>` = `[Vx, Vy, Vz, Px, Py, Pz]` in the reference frame (velocity first).
- SI units throughout (m, m/s, rad, rad/s, N, N·m, Pa). Exception: magnetometer units are inconsistent between drivers; see [libs/ExVectrSensor.md](libs/ExVectrSensor.md).
- Naming: namespaces `VCTR::{Core,HAL,Platform,Math,DSP,CTRL,SNSR,ACTR,Net,Data}`; members end in `_`; units as suffixes where ambiguous (`angle_Rad`, `thrust_N`).

## Data flow (topic graph)

```
MPU9250 ──gyro──► gyroTransformTopic ─┐
        ──acc───► accTransformTopic ──┼──► IMUAttitudeEKF (1 kHz) ──[ω,q]──► attitudeTopicSwitch ─┐
QMC5883 ──mag───► magTransformTopic ──┘           │                                   ▲          │
                                                  ▼                                   │ (sim)    │
BME280 ──baro──┐                        IMUGPSPositionKF (100 Hz) ──[V,P]──► positionTopicSwitch ┤
u-blox ──GNSS──┼──────────────────────────────►  │  ──ground dist──► groundTopicSwitch           │
VL53L0X ─range─┘                                  │                         ▲                     │
                                           BodySimulator (sim mode only) ───┘                     │
                                                                                                  ▼
Mission (setpoint [V,P]) ──► ControlPositionStandard ──accel──► ControlMappingAccToAtt ──[0,q_d]──┐
                                       │                                                          ▼
                                       └───────────accel────────────────────────────► ControlAttitudeTvc
                                                                                         │ [Fx,Fy,Fz,τz]
FlapModeObserverTask ──bellyflop setting──► ControlAttitudeFlaps ──flap angles──► StarshipFlaps ──► servos
                                                                                 StarshipTVC ──► vanes + motors
                                                                                         │ actual thrust
                                                                                         └──► BodySimulator, telemetry
```

The three `Topic_Switch`es let simulation mode replace the estimators with the body simulator without rewiring anything downstream.

Radio: `NetworkNode(address 1)` over `Datalink_SX1280`, with `TransportTopic` bridges on these ports:

| Port | Topic | Direction |
|---|---|---|
| 10 | `PacketAttitude` (5 Hz) | down |
| 11 | `PacketPosition` (10 Hz) | down |
| 12 | `PacketGPS` (1 Hz) | down |
| 50 | `VehicleState` (5 Hz) | down |
| 51 | `MissionState` (5 Hz) | down |
| 100 | connection counter | down |
| 1000 | `Telecommand` | up |

The ground station is node 0. Losing its heartbeat sets `radioConnectionLoss`.

## Build

- `pio run` builds, `pio run -t upload` flashes (`upload_port` in `platformio.ini` is a Windows COM port; override it locally with `--upload-port`), `pio device monitor -b 115200` shows serial output.
- PlatformIO binary on this machine: `~/.platformio/penv/bin/pio`.
- Debug serial output is controlled by `DEBUG_TELEMETRY_SERIAL` and the other `#define`s at the top of `main.cpp`, and by `LOG_MSG`/`VRBS_MSG` switches in `ExVectrCore/print.hpp`.
- Resource baseline (Nov 2026): RAM1 ≈ 56 KB variables, 243 KB code, ≈ 205 KB free for stack.
