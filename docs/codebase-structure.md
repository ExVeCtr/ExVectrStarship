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
│   ├── comms/                ExVectrNetwork links (USB + UART to the RX module), telemetry task, packet mapping
│   ├── ublox_gnss.*          u-blox NEO-M8Q GNSS driver → SNSR::GNSS topic
│   ├── vl53l0x/              VL53L0X ToF rangefinder driver → SNSR::RangeFinder topic
│   ├── telemetry.hpp         Vehicle/mission state enums and structs (wire formats: ExVectrPackets, comms/)
│   ├── telecommand.hpp       Telecommand struct + command list (TEMPORARY wire adapter until T9b)
│   ├── subsystems.hpp        Subsystem enum used by telecommands
│   ├── memory_keys.hpp       Persistent-memory keys (calibration, transforms)
│   ├── board_v_1_0.h         Flight-computer board pinout
│   ├── starship_hardware.h   Vehicle pinout + physical limits (mass, thrust, TVC angles)
│   ├── gnssdriver/           Vendored SparkFun u-blox library (gitignored)
│   └── sx12xxAL/             Vendored SX12xx library (gitignored, unused, excluded from the build)
├── lib/
│   ├── ExVectr*/             The ExVectr framework, one git submodule per library
│   └── Adafruit_*/           Display libraries (SSD1306 OLED), committed as plain folders
├── test/                     Host tests of src/comms (pio test -e native)
├── include/                  PlatformIO default, unused
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
- `main.cpp` is currently a ~2000-line monolith that holds the system tasks (`VehicleSafetyAndControlTask`, `FlapModeObserverTask`, `DisplayTask`, `MagnetometerCalibrationTask`) and the `TelemetryProvider`. The network and telemetry task lives in `src/comms/`. When touching one substantially, move it into its own `src/<name>.hpp/.cpp`; leave `main.cpp` with only object construction and wiring.

## Runtime model

- **One cooperative scheduler, no RTOS.** `loop()` calls `Core::getSystemScheduler().tick()`. Each tick runs **at most one** due task, chosen by `misses*10 + priority`. A task that blocks stalls everything, so never `delay()` or busy-wait in a task.
- **Tasks** derive from `Core::Task_Periodic` (interval in ns), register themselves with `Core::getSystemScheduler().addTask(*this)` in their constructor, and implement `taskInit()` (once) and `taskThread()` (each period). Most objects are global statics, constructed before `setup()`.
- **Topics** (`Core::Topic<T>`) are synchronous: `publish()` calls every subscriber's `receive()` immediately, in the publisher's context. Keep callbacks short. Never publish from an ISR unless every subscriber is ISR-safe (the MPU9250 interrupt path is the existing exception; see [libs/ExVectrCore.md](libs/ExVectrCore.md)).
- **Time** is `int64_t` nanoseconds (`Core::NowNs()`, `Core::SECONDS`, `Core::MILLISECONDS` …). Data travels as `Core::Timestamped<T>`; sensor data as `Timestamped<DSP::ValueCov<float,N>>`.
- **Priorities in use:** MPU9250 driver 110, attitude EKF and position KF 105, the telemetry/network task `VehicleComms` 50 (100 Hz), the platform task 10, everything else default 100.

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

### Radio and USB network

ExVectrNetwork (workspace `docs/network-design.md`, [libs/ExVectrNetwork.md](libs/ExVectrNetwork.md)): the vehicle is **node 1**, an end node, on two stream links (COBS + CRC-16): **USB Serial** (bench ground station, node 0) and **Serial4 at 460800** to the ExVectrLink RX module (node 3), which routes over the air to the TX module (node 2) and the ground station. There is no on-board radio. Code: `src/comms/`, details in [project/hardware-and-drivers.md](project/hardware-and-drivers.md#5-exvectrnetwork-links-srccomms) sections 5, 9, 10.

| Port | Packet | Direction | USB | UART (radio set) |
|---|---|---|---|---|
| – | heartbeat (node 1) | down | 1 Hz | 1 Hz |
| 0x02 | `LOG_TEXT` (console) | down | as printed | – |
| 0x03 | `HANDSET_TELEM` (dst 0xFF) | down | 2 Hz | 2 Hz |
| 0x20 | `VEHICLE_STATUS` | down | 20 Hz | 5 Hz |
| 0x21 | `AIR_POSE` | down | – | 10 Hz |
| 0x7F | legacy `Telecommand`, **TEMPORARY until T9b**, from node 0 only | up | yes | yes |
| 0x80 | `POSE` | down | 100 Hz | – |
| 0x81 | `ACTUATORS` | down | 50 Hz | – |
| 0x82 | `MISSION` | down | 20 Hz | – |
| 0x83 | `GNSS` | down | 5 Hz | – |
| 0x90 | `IMU_RAW` | down | 100 Hz | – |
| 0x91 | `ESTIMATOR` | down | 20 Hz | – |
| 0x92 | `TASK_TIMING` | down | 5 Hz | – |

Packet layouts: ExVectrPackets schema (hash 0x3AFA3CCE), [libs/ExVectrPackets.md](libs/ExVectrPackets.md). The old ports 10/11/12/50/51/100/1000 are retired. Ports >= 0x80 never leave on the UART.

Losing the ground station (no valid frame from node 0 on either link for 2 s) sets `radioConnectionLoss`, as before. Until T9b this includes the radio path, where only node 0 data frames (not its heartbeats) reach the vehicle.

## Build

- `pio run` builds, `pio run -t upload` flashes (`upload_port` in `platformio.ini` is a Windows COM port; override it locally with `--upload-port`).
- PlatformIO binary on this machine: `~/.platformio/penv/bin/pio`.
- **USB Serial carries only ExVectrNetwork stream frames**: `pio device monitor` shows binary. Console text (`LOG_MSG`, Core print) arrives as `LOG_TEXT` packets in the ground station. Never `Serial.print` in `src/`. `LOG_MSG`/`VRBS_MSG` switches are in `ExVectrCore/print.hpp`.
- `platformio.ini`: `-Ilib/ExVectrPackets/gen` (generated packet headers), `-DEXVECTR_ARDUINO_NO_SERIAL_CONSOLE` (platform console off), `build_src_filter` excludes `src/sx12xxAL`. `default_envs = teensy40`, so `pio run` builds only the firmware.
- Host tests: `~/.platformio/penv/bin/pio test -e native` (Unity) builds `src/comms/telemetry_mapping.cpp`, `src/comms/vehicle_network.cpp` and the ExVectrNetwork stack natively (`test/test_telemetry_mapping`, `test/test_vehicle_network`).
- Resource baseline (Nov 2026, after the network migration T9a): FLASH code 229080 B, RAM1 variables 52384 B, code 226920 B, padding 2456 B, 242528 B free for local variables; RAM2 variables 22528 B (of which `vehicleComms` 10096 B in DMAMEM), 501760 B free for malloc/new. Before T9a (ExVectrCore 18bbb09 update): FLASH code 245400 B, RAM1 variables 56448 B, 205696 B free; RAM2 variables 12416 B.
