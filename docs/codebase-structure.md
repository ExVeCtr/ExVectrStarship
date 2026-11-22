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
│   ├── comms/                ExVectrNetwork links (USB + UART to the RX module), telemetry task, packet mapping, radio link rule
│   ├── calibration/          Sensor calibration: CalibrationService (session engine, pure, host-tested), validity and command rules (pure), CalibrationTask (feeds the raw sensor topics)
│   ├── safety/               Pure safety decision logic (estimator zeroing window, "valid zero since boot" rule, mission chain switching, controller phase table, bellyflop kick constants and triggers, MissionAbort decision), host-tested
│   ├── ublox_gnss.*          u-blox NEO-M8Q GNSS driver → SNSR::GNSS topic
│   ├── gnss_config_sequencer.*  Pure, host-tested non-blocking u-blox configuration sequence used by ublox_gnss
│   ├── vl53l0x/              VL53L0X ToF rangefinder driver → SNSR::RangeFinder topic
│   ├── telemetry.hpp         Vehicle/mission state enums and structs (wire formats: ExVectrPackets, comms/)
│   ├── telecommand.hpp       Telecommand struct + command list (TEMPORARY wire adapter until T9b)
│   ├── subsystems.hpp        Subsystem enum used by telecommands
│   ├── memory_keys.hpp       Persistent-memory keys (calibration, transforms)
│   ├── board_v_1_0.h         Flight-computer board pinout
│   ├── starship_hardware.h   Vehicle pinout + physical limits (mass, thrust, TVC angles)
│   ├── gnssdriver/           Vendored SparkFun u-blox library (gitignored)
│   └── sx12xxAL/             Leftover vendored SX12xx library (gitignored, not used, kept out of the build by a filter)
├── lib/
│   ├── ExVectr*/             The ExVectr framework, one git submodule per library
│   └── Adafruit_*/           Display libraries (SSD1306 OLED), committed as plain folders
├── test/                     Host tests of src/comms, src/safety, src/calibration, the GNSS config sequencer and the body simulator model (pio test -e native)
├── tools/                    Bench tools (UART pin scan, bench_vehicle_usb.py temporary USB session)
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
- `main.cpp` is currently a ~2000-line monolith that holds the system tasks (`VehicleSafetyAndControlTask`, `FlapModeObserverTask`, `DisplayTask`; `MagnetometerCalibrationTask` was retired 2026-11-21 for `src/calibration/`) and the `TelemetryProvider`. The network and telemetry task lives in `src/comms/`. When touching one substantially, move it into its own `src/<name>.hpp/.cpp`; leave `main.cpp` with only object construction and wiring.

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
| 0x21 | `AIR_POSE` | down | – | 25 Hz |
| 0x22 | `AIR_ACTUATORS` (2 samples per packet) | down | – | 25 Hz |
| 0x23 | `AIR_MISSION` | down | – | 5 Hz |
| 0x24 | `AIR_GNSS` | down | – | 10 Hz |
| 0x25 | `AIR_GNSS_INFO` | down | – | 1 Hz |
| 0x7F | legacy `Telecommand`, **TEMPORARY until T9b**, from node 0 only | up | yes | yes |
| 0x80 | `POSE` | down | 100 Hz | – |
| 0x81 | `ACTUATORS` | down | 50 Hz | – |
| 0x82 | `MISSION` | down | 20 Hz | – |
| 0x83 | `GNSS` | down | 10 Hz | – |
| 0x90 | `IMU_RAW` | down | 100 Hz | – |
| 0x91 | `ESTIMATOR` | down | 20 Hz | – |
| 0x92 | `TASK_TIMING` | down | 5 Hz | – |
| 0x93 | `CALIBRATION` (applied sensor calibration) | down | 0.5 Hz | – |

Packet layouts: ExVectrPackets schema (hash 0x414DE27A), [libs/ExVectrPackets.md](libs/ExVectrPackets.md). The old ports 10/11/12/50/51/100/1000 are retired. Ports >= 0x80 never leave on the UART.

`radioConnectionLoss` (since 2026-11-20): no **radio packet** for 2 s, where a radio packet is a DATA frame arriving on the UART from node 0 or node 2 (the far side of the air link; normally the Pocket's `RC_STATE` at 25 Hz). Heartbeats, node 3 frames and anything on USB never count; before the first radio packet, a 10 s boot grace applies. The ground station on USB no longer influences it ([project/hardware-and-drivers.md](project/hardware-and-drivers.md#5-exvectrnetwork-links-srccomms)).

## Build

- `pio run` builds, `pio run -t upload` flashes (`upload_port` in `platformio.ini` is a Windows COM port; override it locally with `--upload-port`).
- PlatformIO binary on this machine: `~/.platformio/penv/bin/pio`.
- **USB Serial carries only ExVectrNetwork stream frames**: `pio device monitor` shows binary. Console text (`LOG_MSG`, Core print) arrives as `LOG_TEXT` packets in the ground station. Never `Serial.print` in `src/`. `LOG_MSG`/`VRBS_MSG` switches are in `ExVectrCore/print.hpp`.
- `platformio.ini`: `-Ilib/ExVectrPackets/gen` (generated packet headers), `-DEXVECTR_ARDUINO_NO_SERIAL_CONSOLE` (platform console off), `build_src_filter` keeps a leftover local `src/sx12xxAL` out of the build. `default_envs = teensy40`, so `pio run` builds only the firmware.
- Host tests: `~/.platformio/penv/bin/pio test -e native` (Unity) builds `src/comms/telemetry_mapping.cpp`, `src/comms/vehicle_network.cpp`, `src/safety/estimator_zeroing.cpp`, `src/safety/mission_chain.cpp`, `src/safety/control_phase.cpp`, `src/safety/bellyflop_kick.cpp`, `src/safety/mission_abort.cpp`, `src/safety/sim_flap_fault.cpp`, `src/gnss_config_sequencer.cpp` and the ExVectrNetwork stack natively (`test/test_telemetry_mapping`, `test/test_vehicle_network`, `test/test_estimator_zeroing`, `test/test_mission_chain`, `test/test_control_phase`, `test/test_bellyflop_kick`, `test/test_sim_flap_fault`, `test/test_gnss_config_sequencer`). The calibration code (`src/calibration/`, `ExVectrDSP` `src/calibration/`) is covered by `test/test_calibration_validity` (gates and flow model), `test/test_calibration_service`, `test/test_cal_*` and `test/test_calibration_memory`, which runs the real `ExVectrData` `Memory_Manager` natively (`memory_manager.cpp` plus the Core log stubs in `test/support/native_core_stubs.cpp`, both in the native `build_src_filter`). The body simulator harness `test/test_sim_aero` builds the pure `ExVectrDSP` `body_sim_model.cpp` and `ExVectrControl` `physics_bodysim.cpp` with ExVectrMath (scenarios, before/after measurements with `-v | grep SIMAERO`, CSV time series with `SIM_AERO_CSV=<dir>`). The descent control harness `test/test_descent_guidance` (2026-11-22, report only except the calm-air acceptance: median < 2 m, p90 < 3 m, belly < 35°) runs the same model closed loop from the kick end at 120 m to the freefall stop trigger with the legacy flap guidance replica and `DescentGuidance` / `FlapDescentLaw` from `ExVectrControl` `starship/descent_guidance.hpp`, `flap_descent_law.hpp` (header-only, pure, since 1902fa4); `-v | grep DESCENT`, `DESCENT_SEEDS=<n>` seeds per scenario cell (default 2). `test/test_descent_unit` (fast) covers the NaN guards of both classes and the `ControlAttitudeFlaps` mode mapping (`FlapModeMapper`, header-only) including bit-identical Stabilize/Upright/Acsent outputs against a verbatim copy of the old code.
- Resource baseline (Nov 2026, after the network migration T9a): FLASH code 229080 B, RAM1 variables 52384 B, code 226920 B, padding 2456 B, 242528 B free for local variables; RAM2 variables 22528 B (of which `vehicleComms` 10096 B in DMAMEM), 501760 B free for malloc/new. Before T9a (ExVectrCore 18bbb09 update): FLASH code 245400 B, RAM1 variables 56448 B, 205696 B free; RAM2 variables 12416 B. After the calibration telemetry (2026-11-20): FLASH code 230744 B, RAM1 code 228584 B, padding 792 B, 242528 B free for local variables (unchanged). After `feature/split-reset-zero` (2026-11-20; baseline 7ac9b96: code 228712 B, variables 52384 B, 242528 B free): FLASH code 234392 B, RAM1 variables 53504 B (+1120), code 232232 B (+3520), padding 29912 B, **208640 B free** (-33888 B). The drop is mostly ITCM granularity: RAM1 code is allocated in 32 KB banks and the code crossed 7 x 32 KB = 229376 B, so a new bank was reserved (padding). About 26 KB of code can be added before the next bank. After `feature/reset-chain-gnss` (2026-11-20): FLASH code 236824 B, RAM1 variables 53536 B (+32), code 234664 B (+2432), padding 27480 B, **208608 B free** (-32 B against `feature/split-reset-zero`). After `feature/bellyflop-abort-gnss` (2026-11-20): RAM1 variables 53856 B (+320, mostly the 276 B UBX payload buffer), code 237544 B (+2880), padding 24600 B, **208288 B free** (-320 B). After `feature/review-fixes` (2026-11-20): RAM1 variables 54880 B (+1024, the first use of `Core::printW` for the GNSS warning line), code 237928 B (+384), padding 24216 B, **207264 B free** (-1024 B). After `feature/bellyflop-kick` (2026-11-21, on top of `feature/setpoint-attitude`): FLASH code 242072 B, RAM1 variables 54976 B, code 239912 B, padding 22232 B, **207168 B free** (-96 B against `feature/review-fixes`; `feature/setpoint-attitude` added no entry). After `feature/calibration-service` (calibration C2, 2026-11-21): FLASH code 266776 B, RAM1 variables 61792 B (+6816, not itemised: task and subscriber objects, legacy records, printf state), code 264616 B (+24704: the calibration engine, its fits and `snprintf` of doubles), padding 30296 B (the code crossed 8 x 32 KB = 262144 B, a new ITCM bank), **167584 B free** (-39584 B); RAM2 variables 36928 B (+14016: `CalibrationService` and its 7200 B mag buffer in DMAMEM). About 26 KB of RAM1 code fit before the next bank. After `feature/sim-aero` (2026-11-21, on top of `feature/calibration-service`): FLASH code 268124 B, RAM1 variables 64192 B (+2400: the `BodySimModel` state, of which 2048 B is the optional flap dead-time buffer), code 265960 B (+1344), padding 28952 B, **165184 B free** (-2400 B); RAM2 unchanged (36928 B). After the sim-aero follow-up (geometry, calibration 4.23, gust model and telecommands 13..15, 2026-11-21): RAM1 variables 64352 B (+160), code 267240 B, padding 27672 B, **165024 B free** (-160 B); RAM2 unchanged. After the effective flap angle and the candidate flap/body effects (2026-11-21): FLASH code 269916 B, RAM1 variables 64384 B (+32), code 267752 B, padding 27160 B, **164992 B free** (-32 B); RAM2 unchanged. After `feature/sim-faults` (2026-11-21, simulation flap servo faults): RAM1 variables 65472 B (+1088), code 269096 B, padding 25816 B, **163904 B free** (-1088 B); RAM2 unchanged. After `feature/review-fixes-2` (2026-11-21): FLASH code 271644 B, RAM1 variables 63264 B (-2208), code 269480 B, padding 25432 B, **166112 B free** (+2208 B); RAM2 variables 40384 B (+3456). The simulator object `bodySimulator` (3232 B, of which 2048 B is the flap dead-time buffer) moved to DMAMEM (RAM2), plus the pending-result copy in `CalibrationService` (about 300 B). Largest remaining RAM1 variables: `posEstTask` 1336 B, `vehicleSafetyAndControlTask` 992 B, `gnss` 960 B, `imuTask` 888 B (`__pow_log_data`/`__exp_data` 4168/2160 B and the Teensy vector table 704 B are runtime). The calibration engine (7032 B), its mag buffer (7200 B) and `vehicleComms` (10488 B) are in RAM2. About 26 KB of RAM1 code fit before the next 32 KB ITCM bank. After `feature/sim-fixes-3` (2026-11-21: true flap model `Safety::TrueFlapModel`, chain redirect, fold shield default): FLASH code 271580 B, RAM1 variables 63232 B (-32), code 269416 B (-64), padding 25496 B, **166144 B free** (+32 B); RAM2 unchanged (40384 B). After `feature/calib-offset-hint` (2026-11-21): RAM1 variables 63232 B (unchanged), code 269928 B (+512), padding 24984 B, **166144 B free** (unchanged); RAM2 variables 40416 B (+32). After `feature/calib-fit-quality` (2026-11-22): RAM1 variables 63232 B (unchanged), code 270696 B (+768), padding 24216 B, **166144 B free** (unchanged); RAM2 variables 40448 B (+32). After `feature/mag-ellipsoid` (2026-11-22): FLASH code 278428 B, RAM1 variables 65280 B (+2048), code 276264 B (+5568, the ellipsoid fit and its Gauss-Newton), padding 18648 B, **164096 B free** (-2048 B); RAM2 variables 44448 B (+4000: the fit scratch arrays and sums inside `CalibrationService`). After `feature/schema-integration` (2026-11-22: ExVectrPackets 29db156, `AIR_GNSS`, `AIR_GNSS_INFO`, `home_epoch`, `calib_state`): FLASH code 282460 B, RAM1 variables 65376 B (+96), code 280296 B, padding 14616 B, **164000 B free** (-96 B); RAM2 variables 44640 B (+192). After `feature/t9b-0` (2026-11-22: ExVectrPackets 7b7285d, hash 0x65E4E6E3, schema only): FLASH code 282524 B, RAM1 variables 65376 B (unchanged), code 280360 B (+64), padding 14552 B, **164000 B free** (unchanged); RAM2 variables 44640 B (unchanged). The four `src/main.cpp` unused-but-set warnings (`velEst`, `attData`, `biasData`, `magData`) predate this branch. After `feature/descent-s1` (2026-11-22: ExVectrControl 1902fa4, `Descent` flap mode and the pure `FlapModeMapper`, nothing selects it): FLASH code 284636 B, RAM1 variables 65504 B (+128: the Stabilize/Descent parameter and memory members of `ControlAttitudeFlaps`), code 282472 B (+2112), padding 12440 B, **163872 B free** (-128 B against `feature/t9b-0`); RAM2 variables 44640 B (unchanged). No new `src/` warnings.
