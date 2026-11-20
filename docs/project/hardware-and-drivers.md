# Hardware and Drivers

Pinout, vehicle constants and the application-level drivers in `src/` (TVC, flaps, ExVectrNetwork links, u-blox GNSS, VL53L0X), plus memory keys, telemetry and telecommand wire formats.

Last verified against commit: b073d07 (sections 1, 5, 9, 10 and the Known issues re-verified on `feature/network-migration`, 2026-11-19)

Target: Teensy 4.0 (`platformio.ini` `[env:teensy40]`), little-endian. Frames and units follow [../codebase-structure.md](../codebase-structure.md): NWU reference, body +Z = thrust axis (nose), SI units, time in ns. Library internals (PWM_Output, controllers, KF) are documented in [../libs/](../libs/). This file covers only how `src/` uses them.

Vendored third-party code (gitignored, not ExVectr):

| Folder | What it is |
|---|---|
| `src/gnssdriver/` | SparkFun u-blox GNSS Arduino Library v2.2.27 (`SFE_UBLOX_GNSS`), used by `ublox_gnss.*`. |
| `src/sx12xxAL/` | Leftover vendored StuartsProjects SX12XX-LoRa library, gitignored. **Not part of the firmware** (the on-board SX1280 was physically removed from the board on 2026-11-20 and its datalink in T9a). If the folder is still present locally, `build_src_filter` in `platformio.ini` keeps it out of the build (without the filter its sources are compiled into the image). It can be deleted locally, after which the filter is harmless.

---

## 1. Pin map

Sources: `src/starship_hardware.h` (actuators, vehicle constants), `src/board_v_1_0.h` (board peripherals), and defines and constructors in `src/main.cpp`.

| Pin | Define | Function | Used by |
|---|---|---|---|
| 0 | `TVC_SERVO_PIN_1` | TVC vane servo **XN** | `servoTVCXNPin` (main.cpp:89) |
| 1 | `TVC_SERVO_PIN_3` | TVC vane servo **XP** | `servoTVCXPPin` (main.cpp:88) |
| 23 | `TVC_SERVO_PIN_2` | TVC vane servo **YP** | main.cpp:90 |
| 22 | `TVC_SERVO_PIN_4` | TVC vane servo **YN** | main.cpp:91 |
| 8 | `MOTOR_PIN_CW` | ESC, clockwise motor | main.cpp:92 |
| 7 | `MOTOR_PIN_CCW` | ESC, counter-clockwise motor | main.cpp:93 |
| 25 | `FLAP_SERVO_PIN_UL` | Flap servo top-left (TL) | main.cpp:95 |
| 24 | `FLAP_SERVO_PIN_UR` | Flap servo top-right (TR) | main.cpp:96 |
| 29 | `FLAP_SERVO_PIN_DL` | Flap servo bottom-left (BL) | main.cpp:97 |
| 28 | `FLAP_SERVO_PIN_DR` | Flap servo bottom-right (BR) | main.cpp:98 |
| 2 | `MPU9250_INT_PIN` | MPU9250 data-ready interrupt | `mpuIntHandler` |
| 3 | `MPU9250_NCS_PIN` | MPU9250 SPI chip select | `spiMPU` (SPI) |
| 6 | `BME280_NCS_PIN` | BME280 SPI CS (**unused**: the BME280 runs on I2C 0x76, main.cpp:79) | — |
| 16 | `QMC5883L_INT_PIN` | Magnetometer interrupt (unused in software) | **Now RX4**: wired to the ship node TX (GPIO1). Pin 17 is TX4, wired to the ship node RX (GPIO3); it has no `board_v_1_0.h` define. |
| 5, 9, 10, 14, 15, 17 | (none) | Formerly the SX1280 pins. The module is physically removed (2026-11-20) and the defines are deleted | Free. 16/17 are the UART to the ship node (Serial4); the others are unused |
| 4 | `RGBLED_PIN` | RGB LED (**unused**) | — |
| 26 | `INPUT_VOLTAGE_ADC_PIN` | Battery ADC, factor 1.0 (**unused**) | — |
| Serial4 | `RX_MODULE_SERIALPORT` | ExVectrLink RX module, ExVectrNetwork stream link, 460800 8N1 (pin 16 RX4 / pin 17 TX4, bench-confirmed 2026-11-20, see section 5) | `vehicleComms` (section 5) |
| Serial5 | `NEO_M8Q_SERIALPORT` (define unused; main.cpp:82 hard-codes `Serial5`) | u-blox GNSS UART | `gnss` |

Buses (`initialiseHardware`, main.cpp): `SPI` for the MPU9250. `Wire` at 100 kHz for the BME280 (0x76), QMC5883 (0x0D), VL53L0X (0x29) and SSD1306 128×32 OLED (0x3C). The VL53L0X driver requests 400 kHz on its bus device (`vl53l0x_driver.cpp:72-73`), and that bus is the same `Wire`.

## 2. Vehicle constants (`starship_hardware.h`)

| Define | Value | Notes |
|---|---|---|
| `TVC_SERVO_LIMIT` | `45*DEG_TO_RAD` | Servo half-range. **Not parenthesised.** Safe in its only use as a constructor argument. |
| `TVC_ANGLE_FACTOR` | 15 | Servo angle ÷ thrust-deflection angle. Not passed to `StarshipTVC`, which uses its own default `tvcFactor = 15`. |
| `TVC_ANGLE_LIMIT_RAD` | `TVC_SERVO_LIMIT/15` = **3°** (0.0524 rad) | The stale comment says 8°. Also passed to `ControlAttitudeTvc` and `BodySimulator`. |
| `VEHICLE_MASS_KG` | 1.1 | Used by controller, simulator and freefall stop calculation. |
| `TVC_THRUST_LIMIT_N` | 14.0 | Total thrust of both motors. T/W = 1.30. |

Simulator-only parameters in main.cpp: inertia `{0.044, 0.044, 0.004}` kg·m² (`:117`), TVC lever arm `(0,0,−0.35)` m (`:1967`).

---

## 3. StarshipTVC

Files: `src/starship_tvc.hpp/.cpp`.

Maps the attitude controller's force/torque demand to four exhaust vanes (XP, XN, YP, YN, named by their position on the body axes) and two counter-rotating motors driven with identical commands.

**Construction (main.cpp:174-177):** pins as in §1, `tvcAngleLimit = 3°`, `servoAngleLimit = 45°`, `tvcThrustLimit = 14 N`, `tvcFactor = 15` (default). All six outputs use `PWM_Output_Protocol::STANDARD` (400 Hz, 1–2 ms pulse, value 0..1 clamped by `PWM_Output::setValue`).

**Scheduling:** `Task_Periodic` at 20 ms, but `setRelease(END_OF_TIME)` (`.cpp:27`) plus `taskCheck` → `setDeadline(0)` on new input (`.cpp:36`). It is therefore **event-driven at the publish rate of `controlAttitudeTvc.getTvcTopic()`**. Nothing (enable changes, actuator test) takes effect unless the controller publishes. It publishes even when disabled (`[0,0,0.1,0]`).

**Topics:**

| Direction | Topic | Format |
|---|---|---|
| in | `setTVCInputTopic(controlAttitudeTvc.getTvcTopic())` (main.cpp:1961) | `[Fx, Fy, Fz, τz]`: force in N (body), roll torque in N·m |
| out | `getTVCActualThrustTopic()` | `[X, Y, Z, T]`: modelled achieved thrust vector (N, body) and roll term. Consumed by `bodySimulator` and the telemetry task (`ACTUATORS`). |

**Mixing (`taskThread`, `.cpp:61-290`):**

1. `|F|` = norm of elements 0..2, clamped to 14 N. The vector `tvcVector` is clamped to the 3° cone around +Z. This vector is used **only for the "actual" output**, not for the servos.
2. Roll: `twist = τz · 45 / |F|` (`twistFactor = 45`, `.cpp:99`). Zero if `|F| < 0.01`. Clamped to **±0.5** normalised servo units (= ±22.5° vane, `.cpp:100`).
3. Vane angles, computed from the **unclamped** input: `xAngle = atan2(Fy, Fz)` (rotation about body X, so the X-vanes produce Y force) and `yAngle = atan2(Fx, Fz)` (`.cpp:116-119`). Each is clamped to ±3°, then multiplied by 15 → ±45° (`.cpp:132`), then clamped to ±45°.
4. Normalised servo commands (`.cpp:191-198`), with S = 45°:
   - `XP = ( xAngle + offXP)/S + twist`
   - `XN = −(xAngle − offXN)/S + twist`
   - `YP = ( yAngle + offYP)/S + twist`
   - `YN = −(yAngle − offYN)/S + twist`

   Opposite vanes are mirrored, so a common `twist` produces roll torque. PWM value = `cmd·0.5 + 0.5` (`.cpp:255-258`), neutral 1.5 ms. Offsets plus twist plus full deflection can exceed ±1 and are then clipped by PWM_Output.
5. Fin trim offsets, set in main.cpp:1771-1772: **XP +5°, XN −2°, YP 0°, YN +5°**. The way they enter the formulas above means every offset is added to that vane's normalised command with a positive sign.
6. Motors: `cmd = |F|/14 · motorPowerLimit`, capped at `motorPowerLimit`, then `+ 0.04` idle (`.cpp:200-205`, `:272-274`). Both motors get the same value, so there is no differential yaw from the motors. The full range is 0.04 … 1.04, clipped to 1.0.

**Gating:**

| Output | Enabled when |
|---|---|
| Servos | `enableActuators_` **or** actuator test active (`.cpp:141`) |
| Motors | `enableMotors_ && enableActuators_ && test idle` (`.cpp:247-250`) |
| Motors, first start | Additionally 2 s after the first enable since boot (`.cpp:268`). `firstMotorEnable_` is never reset, so later enables have no delay. |
| `motorPowerLimit_` | Default 0.05. The safety task sets **1.0 when armed, 0.1 when disarmed** every 100 ms (main.cpp:1233-1237). |

**Actuator test (`beginActuatorTest(offset)`):** starts at `NOW + offset` and hijacks the servos regardless of enable. For `TVC_TEST_SPIN_TIME = 2 s` it traces a circle: `x = sin`, `y = cos` at 45° amplitude, 2 revolutions. For the next `TVC_TEST_TWIST_TIME = 2 s` it applies roll-only `twist = sin(...)` at amplitude 1.0 (unclamped, ±45°), 2 cycles. Motors are held off throughout. It is triggered at every mission `Startup` edge (main.cpp:663).

**"Actual" thrust model** (`.cpp:211-240`, only when actuators are enabled and the test is idle): direction = clamped `tvcVector`, magnitude = mean motor command × 14 N. The vane angles are rate-limited to 600°/s (`60°/0.1 s`). `T = twist·|F|/45`, i.e. the roll torque reconstructed after clamping. The header calls `T` an "angle in radians".

---

## 4. StarshipFlaps

Files: `src/starship_flaps.hpp/.cpp`.

Drives four flap servos from `ControlAttitudeFlapSetting` (`lib/ExVectrControl/include/ExVectrControl/starship/control_attitude_flaps.hpp:13`) and publishes simulated true flap positions.

- Construction (main.cpp:178): `(UL=25, UR=24, DL=29, DR=28)` are mapped to `TL, TR, BL, BR`. STANDARD PWM (400 Hz). Periodic task at **20 ms** (`.cpp:41`).
- Input: `setFlapSettingTopic(flapSettingTopic)`, the output of `controlAttitudeFlaps` (main.cpp:1959, 1962). Output: `setTrueFlapSettingTopic(trueFlapSettingTopic)` (main.cpp:1963), consumed by `bodySimulator` and the flap debug print.

**Angle convention:** radians. **90° = retracted (folded), 0° = fully extended.** This follows from the flap controller: `Acsent` without extend commands 90° on all flaps, and `Upright` commands top 0° and bottom 90° (`control_attitude_flaps.cpp:174-189`). The driver clamps every angle to **[15°, 90°]** (`.hpp:36-37`, `:93-99`), so full extension (0°) is unreachable.

**Servo mapping (`.cpp:158-165`):** `value = angle/(π/2)` for TR and BL. `value = 1 − angle/(π/2)` for TL and BR, which are mounted mirrored. 0..1 maps to 1–2 ms. All per-flap trim offsets are 0 (`.cpp:146-150`).

**Gating:** `enable = enableActuators_ && setting.enableFlaps` (`.cpp:95`). `enableActuators_` defaults to **true** and is then driven every tick by the safety task: on during Freefall, otherwise the mission's actuator flag (main.cpp:1195-1198). `controlAttitudeFlaps` always sets `enableFlaps = true`.

**Actuator test (`beginActuatorTest(offset)`):** a 4 s window in which each flap follows `(sin(φ + 2πt/4) + 1)·π/2` (0..π before the [15°, 90°] clamp), with phase offsets BL 0, TR π/4, TL π/2, BR 3π/4. Output is forced on. It is only called under `#ifdef DO_FLAP_TEST_STARTUP` (main.cpp:664, undefined). Note that the hold-off before start (`WaitBegin`) does not force-enable the output.

**True flap topic:** `ServoAngleSimulator` per flap, slew-limited to 200°/s (`60°/0.3 s`, `.cpp:60-63`), towards the **clamped** commanded angle. Published only while enabled. Its `enableFlaps` field carries `enableActuators_`, not the combined enable.

---

## 5. ExVectrNetwork links (`src/comms/`)

Since the network migration (T9a) the vehicle has **no on-board radio**. `Datalink_SX1280` and the on-board SX1280 (physically removed from the board on 2026-11-20) are gone. The vehicle talks ExVectrNetwork (library doc: [../libs/ExVectrNetwork.md](../libs/ExVectrNetwork.md); wire format and design: workspace `docs/network-design.md`) over two byte-stream links:

| Link | Port | Settings | Peer |
|---|---|---|---|
| USB | `Serial` (Teensy USB CDC) | baud ignored | ground station (node 0) on the bench |
| UART | `RX_MODULE_SERIALPORT` = **Serial4** (pin 16 RX4, pin 17 TX4), `starship_hardware.h` | **460800 8N1**, extra buffers 1024 B RX / 512 B TX (`addMemoryFor*`) | ExVectrLink RX module (node 3), router to the TX module (node 2) and the ground station over the air |

**USB Serial carries only stream frames** (`COBS(0x01 ‖ frame ‖ CRC-16) ‖ 0x00`). Console text is never printed to it: the platform console is disabled with `-DEXVECTR_ARDUINO_NO_SERIAL_CONSOLE` (ExVectrArduinoPlatform), and Core print output (`printM/D/W/E`, `LOG_MSG`) is collected into lines and sent as `LOG_TEXT` (port 0x02) on USB only (`LogTextQueue`: 8 lines of up to 200 characters, at most 4 lines per tick, overflow drops lines). The old `DEBUG_TELEMETRY_SERIAL` prints are removed: their content is in the USB telemetry set. Do not add `Serial.print` anywhere (`CalibrationTesting`, not instantiated, and the u-blox `usbPassthrough_`, off, still touch `Serial`).

**Structure:**
- `VehicleNetwork` (`vehicle_network.hpp/.cpp`, platform-free, host-tested): `StreamFrameLink<2048>` (USB) and `StreamFrameLink<1024>` (UART); **two `Node`s, both address 1**, end nodes (never forward), one per link, so each link gets its own telemetry set and rates; one shared `PortMux<2>` (a frame that arrives on both links is delivered once). UART egress filter: ports >= 0x80 dropped.
- `VehicleComms` (`vehicle_comms.hpp/.cpp`): a `Core::Task_Periodic` at **100 Hz, priority 50** (below all control tasks, 100-110). Per tick: read both links, heartbeats, sample the vehicle (`TelemetryProvider` in main.cpp: latest-value subscribers and getters, read-only), queue the telemetry due on this tick, push bytes out.
- `telemetry_mapping.hpp/.cpp` (pure, host-tested): snapshot -> packets, flight-software bit fields, legacy telecommand codec.
- The `VehicleComms` object (links, rings, nodes, log queue, UART buffers: about 10 KB) is in **DMAMEM** (RAM2).

**Constant cost:** every tick builds and encodes the same packets whatever is connected. Writes are bounded by `availableForWrite()` and never block (the Teensy USB write would otherwise wait up to 120 ms for a host that does not read). With nothing draining a link its TX ring fills and further frames are dropped (`stats.txDropped`); encoding happens before that check. Telemetry is no longer produced inside the estimator topic callbacks (the old 5/10/1 Hz attitude, position and GNSS callbacks ran in the EKF/KF publish context): the control path does less work than before.

**Heartbeats:** each node sends the vehicle heartbeat (role 1, schema hash `Packets::kSchemaHash`) at 1 Hz on its link. UART peer: `UART_PEER_SEEN` (see section 9, `VEHICLE_STATUS.flags`) reports whether node 3 was heard on the UART in the last 3 s. Heartbeats and `LINK_STATS` of the modules are informational: they fill the node table and the `INCOMPATIBLE` flag (a present node with another network version or schema hash), nothing else.

**Ground-station presence and `radioConnectionLoss`:** `VehicleComms::isGroundStationPresent()` is true while any valid frame (heartbeat or data) from node 0 arrived on USB or UART within **2 s** (`presenceTimeoutNs`). `checkForFailures` uses it exactly where it used `NetworkNode::isNodeReachable(0)` (old timeout 2 s). Consequences on the new topology:
- **USB (bench):** the ground station's 1 Hz heartbeat keeps node 0 present, as the old radio heartbeat did.
- **Radio path:** the TX and RX modules drop node 0 heartbeats (and `RC_STATE`) before the vehicle, by design. Only node 0 *data* frames reach the vehicle over the radio (for example `REQUEST`, or the temporary telecommand). Until T9b, a ground station connected only through the radio must send such data at least every 2 s, or the vehicle enters Failure (`radioConnectionLoss`) after initialisation. Frames from nodes 2 and 3 never count.
- T9b replaces this with the `RC_STATE` control-source timeout.

**Port table:** [../codebase-structure.md](../codebase-structure.md#radio-and-usb-network).

**Wiring (bench-confirmed end to end, 2026-11-20):** the ship node (ESP8285 "RX module", node 3) is wired to **Serial4**: ship node TX (GPIO1) to **Teensy pin 16 (RX4)**, **Teensy pin 17 (TX4)** to ship node RX (GPIO3), 3.3 V, 460800 8N1. Verified with the ExVectrLink Router: it lists node 1 as present and the vehicle telemetry arrives over the radio. History: the module was first found on pads 30/31 (GPIO_EMC_37/36), which are not pins of any Teensy 4.0 hardware UART, so no `SerialN` could read them; the wires were moved to 16/17. During the SX1280 removal the TX pin (17) had been shorted to ground; it was repaired.

**Finding the wiring again (bench tool):** `tools/uart_pin_scan.py` (run with the ground-station venv python, which has pyserial; needs the Router attached) disables the ground-station USB source, flashes the bench-only scan firmware (`pio run -e teensy40_pinscan`, `tools/pinscan/pinscan.cpp`, links no flight code), listens on every UART RX pin and scans all pads for edges, then transmits node 1 heartbeats on candidate TX pins one at a time while polling node 3's `NetworkStatus` through the Router (age[1] about 0 means the ship node heard node 1). It finishes by rebuilding and flashing the flight firmware of the checked-out branch and re-enabling the USB source (`--no-restore` leaves the scan firmware on). `--quick` re-checks only the known candidates (460800 listen, TX on pin 17, bit-bang pad 31). Use it after any rewiring or hardware change near the UART.

---

## 6. `UbloxSerialGNSS` (`src/ublox_gnss.hpp/.cpp`)

u-blox (NEO-M8Q per the define name) on `Serial5`, wrapping SparkFun `SFE_UBLOX_GNSS`. It implements `SNSR::GNSS` and publishes `Timestamped<SNSR::GNSSData>` on `getGNSSTopic()`.

- Task: `Task_Periodic("Ublox GNSS", 1 ms, start = 5 s)` (`.hpp:28`), so the **first run is at t = 5 s**. `taskCheck` unpauses the task when UART bytes are available. Each `readGNSS()` call pauses it again.
- **Init (`.cpp:138-209`):** tries 115200, then 9600, then 38400 baud (the log message always says 9600). Then `setSerialRate(115200)`, reopens, and configures: UART1 output **UBX only**, **nav rate 10 Hz**, `setAutoPVT(true)`, dynamic model **AIRBORNE <2g**. GNSS selection: Galileo on; GLONASS, BeiDou, SBAS, QZSS and IMES off; then after `delay(1000)` **GPS off** (`.cpp:199-200`). Finally `saveConfiguration()` persists all of this in the receiver. The `delay(1000)` blocks the cooperative scheduler.
- **Read:** `getPVT(0)` (non-blocking). If no PVT arrives for ≥ 500 ms, it logs only (no recovery action).

**Output conversion (`_getData`, `.cpp:7-48`):**

| Field | Source | Unit / frame |
|---|---|---|
| `position(0,1)` | lat, lon × 1e-7 → rad | geodetic, double |
| `position(2)` | `getAltitude()` / 1000 | m above **ellipsoid** (not MSL) |
| `velocity` | `[N, −E, −D]` / 1000 | m/s, **NED → NWU** (the position KF converts LLA to the local frame) |
| `positionCov` | `[hAcc, hAcc, vAcc]` / 1000 | **metres (1σ), not variance**, despite the name |
| `velocityCov` | `sAcc/1000` in all three axes | m/s (1σ) |
| `positionValid = velocityValid` | `numSats ≥ 5` | The fix-OK check is commented out (`.cpp:37`). `minNumSats_` and `maxError_` are unused. |
| `numSats` | `getSIV()` | |
| timestamp | `Core::NowNs()` at read | Not GNSS time; includes UART latency. |

Messages containing NaN are dropped (self-equality test, `.cpp:41`).

---

## 7. VL53L0X rangefinder (`src/vl53l0x/`)

`vl53l0x_driver.*` is a port of the Pololu VL53L0X Arduino library (the original files are kept in `src/vl53l0x/template/`) onto `HAL::DigitalIO` and `SNSR::RangeFinder`.

- Instance (main.cpp:89-90): I2C 0x29 on `Wire`, **`longRange = true`, `highSpeed = true`**.
- `taskInit` (`.cpp:1068-1106`): checks model ID 0xEE, switches I/O to 2V8, runs the standard ST init. Long range: signal-rate limit 0.1 MCPS, VCSEL pre/final 18/14 PCLK. High speed: timing budget 20 ms (otherwise 200 ms). I/O timeout 500, then **continuous back-to-back** mode.
- Task period: **25 ms** with highSpeed (205 ms otherwise, `.cpp:1060`). Each run polls `RESULT_INTERRUPT_STATUS`, reads the range and clears the interrupt.
- Output (`rangeTopic_`, `Timestamped<ValueCov<float,1>>`): `val = mm/1000` (m, sensor axis), `cov = 0.15 · range` (`RELATIVE_ERROR`, `.hpp:110`). That is a 1σ-style value in m, not m². The timestamp is the read time. The range status is **not checked**, so out-of-range or "no target" readings (typically about 8.19 m) are published as values.
- Watchdog: if no sample arrives for > 1 s, `setInitialised(false)`, and the scheduler re-runs `taskInit`. `initialisationRetries_` (max 5) only counts while `initialised_` is false.
- Consumer: `posEstTask.setRangeInput(range, minRange = 0.4, maxRange = 1.5)` (main.cpp:1932). Readings outside 0.4–1.5 m are ignored by the KF. The ground-distance KF state is `[rate, distance]`, see [../libs/ExVectrDSP.md](../libs/ExVectrDSP.md).

---

## 8. Persistent memory keys (`src/memory_keys.hpp`)

`Data::Memory_Internal<512>` is a RAM image of EEPROM, managed by `Memory_Manager` (main.cpp:105-107). At boot: EEPROM → RAM. A manager version mismatch clears memory. Missing items are created with identity defaults. Then RAM → EEPROM (main.cpp:1775-1871). The image is saved again only by `Telecommand_Save` with subsystem `Magneto`.

| Key | Define | Type | Content |
|---|---|---|---|
| 100 | `MEMORY_KEY_GYROCALIB` | `DSP::ValueCov<float,3>` | `val` = bias (sensor frame), `cov` = scale/misalignment 3×3 |
| 101 | `MEMORY_KEY_MAGCALIB` | `ValueCov<float,3>` | `val` = hard-iron bias, `cov` = scale. Written by `CalibApply` (main.cpp:769). |
| 102 | `MEMORY_KEY_ACCCALIB` | `ValueCov<float,3>` | bias / scale |
| 200 | `MEMORY_KEY_GYROTRANSFORM` | `Matrix<float,3,3>` | sensor → body |
| 201 | `MEMORY_KEY_MAGTRANSFORM` | `Matrix<float,3,3>` | sensor → body |
| 202 | `MEMORY_KEY_ACCTRANSFORM` | `Matrix<float,3,3>` | sensor → body |

Applied as `TopicCoordTransform::setTransform(calib.cov * transform, calib.val)` through `applySensorCalibration` (main.cpp:117, calls at :1804-1812), which also keeps a copy for the `CALIBRATION` telemetry. **The gyro bias is overridden in RAM at every boot** with hard-coded `(0.92, 2.0, 0.6) °/s` (main.cpp:1789-1792). It is never written to memory, so the stored key 100 bias is ignored. The exact formulas are in section 8a.

## 8a. Sensor calibration pipeline (what `CALIBRATION` reproduces)

Each of the three sensor streams passes one `DSP::TopicCoordTransform` before the estimators (`lib/ExVectrDSP/include/ExVectrDSP/topic_coord_transform.hpp:94-99`):

```
cal = M * (raw - b)        M = C * T  (3x3),  b = offset (3)
```

| Sensor | raw (driver output, `IMU_RAW`) | b | C | T | Consumers of `cal` |
|---|---|---|---|---|---|
| Gyro | MPU9250 gyro, rad/s | hard-coded `(0.92, 2.0, 0.6)°/s` in rad/s (key 100 `.val` ignored) | key 100 `.cov` | key 200 | attitude EKF |
| Accel | MPU9250 accel, m/s² | key 102 `.val` | key 102 `.cov` | key 202 | attitude EKF, position KF |
| Mag | QMC5883, driver gauss | key 101 `.val` | key 101 `.cov` | key 201 | attitude EKF |

- Order: the offset is subtracted in the **raw (sensor) frame**, then `M` scales and rotates into the body frame in one step. There is no temperature term, no other remap and no clamping. Missing EEPROM items are created as `b = 0`, `C = T = I` (identity). `Telecommand_CalibApply` replaces the mag `b` and `C` (uniform scale `1/fitScale`, normalising |B| to 0.05 gauss-units) with `T` = key 201 re-read (main.cpp:606-664); it is RAM only until `Telecommand_Save`.
- The "sensor frame" is the driver output frame. The MPU9250 driver already remaps the chip axes: driver `(x, y, z)` = chip `(y, x, −z)` for accel and gyro (`lib/ExVectrSensor/src/mpu9250.cpp:886-898`). The QMC5883 driver does not remap.
- The EKF additionally multiplies the calibrated accel by `accTiltCompensation_` in `updateAcc` only; it is the identity (main.cpp:1922-1924). The EKF subtracts its own gyro bias state from the calibrated gyro (`POSE.omega = gyro cal − gyro_bias_est`).
- Covariances are not transformed (DSP Known issues).

**`CALIBRATION` (port 0x93, USB only, every 2 s)** carries, per sensor, `*_offset` = `b` and `*_matrix` = `M` row-major (`M[r][c] = matrix[3r + c]`), so the ground station computes from `IMU_RAW`:

```
gyro_cal  [rad/s] = gyro_matrix  * (IMU_RAW.gyro  - gyro_offset)
accel_cal [m/s^2] = accel_matrix * (IMU_RAW.accel - accel_offset)
mag_cal   [T]     = mag_matrix   * (IMU_RAW.mag   - mag_offset)
omega_est [rad/s] = gyro_cal - gyro_bias_est          (equals POSE.omega up to timing)
```

The vehicle computes the mag in gauss (`IMU_RAW.mag` = driver gauss × 1e-4, `mag_offset` = key 101 `.val` × 1e-4, `mag_matrix` unchanged), so `mag_cal` is the vehicle's value × 1e-4. The EKF only uses the mag direction, and drops samples with |vehicle value| outside 0.01–0.1, i.e. |`mag_cal`| outside 1e-6–1e-5 T (`lib/ExVectrDSP/src/imu_attitude_ekf.cpp:327`); with the in-flight calibration |`mag_cal`| ≈ 5e-6 T, which is not the physical field strength.

`raw_frame` is `SENSOR` (0). Source bits per sensor (`*_source`): `CALIB_EEPROM` (key 10x read from EEPROM), `TRANSFORM_EEPROM` (key 20x read), `OFFSET_OVERRIDE` (gyro: hard-coded bias), `RAM_ONLY` (set by `CalibApply`, cleared by a successful `Save`), `MEMORY_CLEARED` (memory manager version mismatch at boot, everything is defaults). Both bits clear means identity defaults.

**Bench values (vehicle on the bench 2026-11-20, read with `CALIBRATION`):** all six items loaded from EEPROM (gyro source 0x07, accel 0x03, mag 0x03).

| Sensor | b | M |
|---|---|---|
| Gyro | `(0.01606, 0.03491, 0.01047)` rad/s (override) | `diag(−1, 1, −1)` |
| Accel | `(0.42849, 0.55833, 2.14256)` m/s² | `diag(−0.9984, 1.003, −0.9673)` |
| Mag | `(1.0491e-6, −1.467e-7, 8.154e-7)` T | `0.16220 · [[0,0,−1],[1,0,0],[0,−1,0]]` |

These are the values of the `if (false)` block in `initialiseMemory` for the accel (offset/scale, `T = diag(−1,1,−1)`) and gyro (`C = diag(−1,1,−1)`), and a `CalibApply` mag fit with the permutation transform in key 201. With them the bench vehicle at rest gives |accel cal| = 10.3 m/s² (raw 8.44): the accel calibration no longer fits the sensor (Known issues).

---

## 9. Telemetry: generated packets (`src/comms/telemetry_mapping.cpp`)

The packets are the generated ExVectrPackets classes (schema hash 0x4FE838D2, byte layouts in the library's `docs/packet-reference.md`). Decoded values are SI; fixed-point scaling, saturation and NaN handling are done by the generated `encode()`. The old raw structs `VehicleState` (port 50) and `Net::Packet*` (ports 10-12) are removed; `MissionState` (`telemetry.hpp`) stays as the missions' internal state.

**Sets and rates** (100 Hz tick, rates from the generated `kUsbRateHz` / `kAirRateHz`):

| Port | Packet | USB | UART (radio set) | dst |
|---|---|---|---|---|
| 0x80 | `POSE` | 100 Hz | – | 0 |
| 0x90 | `IMU_RAW` | 100 Hz | – | 0 |
| 0x81 | `ACTUATORS` | 50 Hz | – | 0 |
| 0x20 | `VEHICLE_STATUS` | 20 Hz | 5 Hz | 0 |
| 0x82 | `MISSION` | 20 Hz | – | 0 |
| 0x91 | `ESTIMATOR` | 20 Hz | – | 0 |
| 0x83 | `GNSS` | 5 Hz | – | 0 |
| 0x92 | `TASK_TIMING` | 5 Hz | – | 0 |
| 0x93 | `CALIBRATION` | 0.5 Hz (every 2 s, `VehicleNetwork::kCalibrationPeriodTicks`; the schema rate is 0) | – | 0 |
| 0x03 | `HANDSET_TELEM` | 2 Hz | 2 Hz | 0xFF |
| 0x21 | `AIR_POSE` | – | 10 Hz | 0 |
| 0x02 | `LOG_TEXT` | as printed | – | 0 |
| – | heartbeat | 1 Hz | 1 Hz | 0xFF |

UART total about 0.65 kB/s including framing (the air budget is 6.2 kB/s); USB about 18 kB/s plus log text.

**Sources** (`TelemetryProvider`, main.cpp; read-only, in the telemetry task): attitude and position from the topic switches (estimators, or the body simulator in simulation mode, as everything downstream sees them), ground distance from the ground topic switch, GNSS from `gnss`, flap angles from `trueFlapSettingTopic`, TVC from `StarshipTVC::getTVCActualThrustTopic()`, raw sensors from the MPU9250, QMC5883 and BME280 topics, standard deviations from the estimators' covariances, mode/mission/failures from `VehicleSafetyAndControlTask`.

**Field mapping:**

| Packet | Field | From |
|---|---|---|
| all | `t_us` | `(NowNs() / 1000) mod 2^32` |
| `VEHICLE_STATUS` | `t_ns` | `NowNs()` |
| | `mode` | `VehicleMode`: 0 Startup, 1 SensorInit, 2 SensorCalib, 3 Ready, 4 Running, 5 Failure |
| | `mission_mode`, `mission_phase` | `MissionMode` (0 Idle … 4 Finished), `MissionPhase` (0 Idle … 7 Completed) |
| | `ctrl_source` | 0 (NONE): no RC control source until T9b |
| | `flags` | ARMED = `vehicleArmed_`; SIMULATION; MOTORS_ENABLED = `StarshipTVC` motors enabled **and** actuators enabled (the existing logic; there is no hard motor gate before T9b); GS_PRESENT = ground-station presence (above); INCOMPATIBLE; FAILSAFE = mode is Failure (actuators cut); UART_PEER_SEEN (bit 6) = a valid frame from the ship node (node 3) arrived on the UART within 3 s (`VehicleNetwork::isUartPeerSeen()`, from the UART node's node table, so USB frames and other nodes never set it). **Information only:** nothing in the vehicle reads it (safety.md 4.1). It exists because `NetworkStatus` seen over USB reflects only the USB node, so a dead UART is otherwise invisible |
| | `failure_bits` | bit 0 sensor, 1 attitude, 2 position, 3 position out of bounds, 4 attitude out of bounds, 5 radio connection loss |
| | `sensor_state` | 2 bits per sensor, `TelemetrySensor` (0 Init, 1 Calib, 2 Ready, 3 Failure): bits 0-1 GNSS, 2-3 IMU, 4-5 baro, 6-7 mag, 8-9 attitude KF, 10-11 position KF |
| | `last_req_id`, `last_req_status`, `rc_age` | 0, 0, NaN (no `REQUEST` / `RC_STATE` until T9b) |
| `POSE` | `omega`, `q`, `vel`, `pos` | attitude state `[ω, q]`, position state `[V, P]` (f32) |
| `AIR_POSE` | `q` (i16, 1/32767), `pos` (**i32, 0.001 m = 1 mm**, ±2147483 m; was i16 0.1 m until 2026-11-20), `vel` (i16, 0.01 m/s, ±327 m/s) | as `POSE`, fixed point, saturating; 30 byte payload. USB `POSE` is f32 and unchanged |
| `ACTUATORS` | `flaps` | true flap angles `[TL, TR, BL, BR]`, rad |
| | `tvc` | `[atan2(Fx, Fz), atan2(Fy, Fz)]` of the actual thrust vector (body frame), rad |
| | `thrust` | `|[Fx, Fy, Fz]|`, N. The roll torque `T` has no field (Known issues) |
| | `ground_distance` | ground topic switch `data(1)` (KF state `[rate, distance]`), m; NaN before the first sample |
| `MISSION` | `setpoint_pos`, `setpoint_vel` | `positionSetpoint[3..5]`, `[0..2]` |
| | `mission_time` | `missionTime` ns -> s (1 ms wire resolution; negative in the countdown) |
| | `index`, `phase` | `MissionState.missionIndex` (always 0, see missions.md), current `MissionPhase` |
| `GNSS` | `latitude`, `longitude` | rad (f64) |
| | `altitude`, `vel` | m above the ellipsoid; NWU m/s |
| | `h_acc`, `v_acc`, `s_acc` | the driver's `positionCov(0)`, `positionCov(2)`, `velocityCov(0)`: 1σ in m, m, m/s |
| | `fix_type` | 3 if `positionValid` (>= 5 satellites), else 0 |
| `HANDSET_TELEM` | `flags` | armed, failure (mode Failure), simulation, GNSS fix |
| | `mode_text` | `INIT` (Startup, SensorInit), `CALIB`, `RDY` / `ARMD`, `FAIL`; in Running by phase: `HOVR`, `ASCN`, `DESC`, `DECL`, `RTH`, `LAND`, `DONE`, else `RUN`. `" S"` is appended in simulation mode (for example `HOVR S`) |
| | `latitude`, `longitude`, `satellites` | GNSS |
| | `ground_speed`, `heading` | from the estimated velocity: `|[Vx, Vy]|`, course over ground clockwise from North |
| | `altitude`, `vertical_speed` | estimated `Pz`, `Vz` (above the reference) |
| | `roll`, `pitch`, `yaw` | ZYX Euler angles of q (NWU; yaw counterclockwise about +Z, 0 = body X North) |
| | `battery` | NaN -> 0 = unknown (no battery monitoring) |
| `IMU_RAW` | `gyro`, `accel` | MPU9250 driver outputs (`mpuDriver` topics, not the transform outputs), sensor frame, **uncalibrated** (before offset, scale and the body transform; section 8a) |
| | `mag` | QMC5883 driver output (`qmc` topic), gauss × 1e-4 -> T, sensor frame, **uncalibrated** (no hard/soft iron). The driver scale is 0.73x the physical field (Known issues) |
| | `pressure`, `temperature` | BME280 Pa; MPU9250 °C + 273.15 -> K |
| `ESTIMATOR` | `att_std` | 1σ small rotations about the **world axes [North, West, Up]**, rad: `sqrt(diag(4 Ωᵀ P Ω))` with `P` the EKF 4x4 quaternion covariance and `Ω(q)` the map of a world-frame rotation to dq (`attitudeStdWorld`, `src/comms/telemetry_mapping.cpp:52`), using the EKF's own q (also in simulation). At q = identity it equals the former `2·sqrt(P_vector)`; away from identity the former diagonal mixed the axes. Only as meaningful as the EKF covariance (Known issues) |
| `CALIBRATION` | all | section 8a: per sensor `offset`, `matrix` (row-major `C·T`), `source` bits; `raw_frame` = SENSOR; `gyro_bias_est` = EKF state `x(0..2)`. Mag offset gauss × 1e-4 -> T (`makeCalibration`) |
| | `vel_std`, `pos_std` | `sqrt` of the position-KF covariance diagonal |
| `TASK_TIMING` | `count`, `avg` | first 8 scheduler tasks in registration order, average run time (EMA), s; `max` NaN |

## 10. Telecommands (`src/telecommand.hpp`): TEMPORARY adapter on port 0x7F

**Temporary (network migration T9a, removed in T9b).** The new command path (`RC_STATE` mapping, control-source arbitration, `REQUEST`) is task T9b and needs the user's approval. Until then the old `Telecommand` is carried unchanged through the new stack, so the vehicle stays commandable:

- **Port 0x7F** (`LegacyTelecommandPacket::kPort`, `src/comms/telemetry_mapping.hpp`). The old port 1000 does not fit the 8-bit port field. 0x7F is below 0x80, so it passes the RX and TX modules' filters and works over USB and over the radio. It is **not in the ExVectrPackets schema** (the ground station has no encoder for it yet).
- Accepted only from **node 0** (the ground station, the only sender on the old link), exactly **8 bytes**, on either link. Frames from other nodes are counted and dropped.
- Delivered once even if it arrives on both links (shared sequence filter), then published on `telecommandTopic` and handled by `VehicleSafetyAndControlTask::handleTelecommands` exactly as before (same struct, same handler, newest one per 100 ms tick).

### `Telecommand`: 8 bytes, explicit little-endian (identical to the old packed struct)

| Off | Size | Field |
|---|---|---|
| 0 | 1 | `type` (`TelecommandType`) |
| 1 | 1 | `subsystem` (`Subsystem`, `subsystems.hpp`: 0 Gyro, 1 Accel, 2 Magneto, 3 Baro, 4 GNSS, 5 AttFilter, 6 PosFilter) |
| 2 | 2 | `int16 paramInt` |
| 4 | 4 | `float paramFloat` (unused by every handler) |

Network frame from the ground station: CTRL `0x48` (DATA, QoS EVENT, not latest), TTL 3, DST 1, SRC 0, PORT 0x7F, SEQ (+1 per command), then the 8 bytes.

### Command list (handler: `VehicleSafetyAndControlTask::handleTelecommands`, main.cpp:537-787)

Only the newest telecommand per 100 ms tick is processed (`Simple_Subscriber`).

| # | Type | Magic / params | Precondition | Implemented behaviour |
|---|---|---|---|---|
| 0 | `CalibStart` | `subsystem == Magneto`; `paramInt > 0` starts, ≤ 0 stops | none | Start: mag least-squares calibration task, TVC actuators off, mode `SensorCalib`. Stop: end calibration, mode `Startup`. The duration in the header comment is not implemented. |
| 1 | `CalibApply` | `subsystem == Magneto` | none | bias = fit, scale = 1/fitScale·I. Write key 101, read key 201, apply to `magTransformTopic` (RAM only until Save). |
| 2 | `Load` | — | — | **Not handled.** |
| 3 | `Save` | `subsystem == Magneto` | none | EEPROM ← RAM image (all keys). |
| 4 | `SystemReset` | `paramInt == 0xC5` | none | Mode `Startup`, re-init sensors, actuators/motors/flaps off, `resetMission()`, `clearFailures()`, reset the position reference, reset the simulator state, **disarm**. Does **not** leave simulation mode, although the header says so. |
| 5 | `Arm` | `paramInt == 0xA5` arms; **any other value disarms** | none | Sets `vehicleArmed_`. Auto-disarm on Failure, SensorCalib, Startup or SensorInit, and after RTH finishes. |
| 6 | `SimulationMode` | `paramInt == 0xA9` | none | **Toggles.** On: motors off, topic switches → `bodySimulator`, simulator reset and unpaused. Off: switches → estimators, simulator paused, mode `Startup`, re-init, actuators off. |
| 7 | `MissionBegin` | `paramInt` = initial mission time in s (use ≤ −10 for the full countdown) | mode `Ready` | `switchMissionTo(waypoint, paramInt·1 s)`, mode `Running`, motors enabled unless in simulation. **Arming not required.** |
| 8 | `MissionSelect` | — | mode `Ready` | No-op (log only). |
| 9 | `MissionAbort` | none | mode `Running` | Mission mode `Running`: switch to RTH. Else: actuators off, reset mission, clear failures. |

There is no authentication beyond the three magic values and the source address 0 (which any node on a link can claim). The vehicle subscribes to no other port: telemetry ports are send-only now.

---

## Body frame and actuator layout

Purpose: everything a 3D model or ground-station view needs to place the body axes, flaps, TVC vanes and motors. Each statement is tagged **[code]** (derivable from code, cited) or **[sim]** (only from the simulator's geometry, which is a model, not a measurement) or **UNKNOWN** (needs the user). Line numbers refer to the pinned sources at the time of writing (ExVectrControl `084bea1`, ExVectrSensor `6bd4b01`).

### Body frame

- **[code]** Right-handed, **+Z = long axis toward the nose**, TVC/motors at the aft end at body `(0,0,−0.35)` m (`src/main.cpp:1967`, `lib/ExVectrDSP/src/body_simulator.cpp:34`). Top flaps at z = +0.26 m, bottom flaps at z = −0.26 m.
- **[code]** **±Y are the flap sides**: TL and BL at +Y, TR and BR at −Y (`body_simulator.cpp`, flap block: `mountY = 0.08`, `mountZ = 0.26`).
- **[code]** **+X completes the frame** (X = Y × Z). The belly of the vehicle is **−X** and the back is **+X**: `ControlAttitudeFlaps` measures `bellyDownAngle` as the angle between body +X and world +Z, so "belly down" means body +X pointing up (`lib/ExVectrControl/src/starship/control_attitude_flaps.cpp:80-84`).
- **[code]** Roll axis = body **Z**. `StarshipTVC` roll torque `τz` is about Z. Pitch = rotation about body **Y**, yaw/azimuth for the flap controller is defined from the world projection of body Z (see below). Rotation sign is right-handed (`docs/libs/ExVectrMath.md`).
- **UNKNOWN (needs the user):** which physical side of the vehicle is "left" vs "right" (i.e. whether +Y, the TL/BL side, is physically the left or the right when seen from behind or from the front), and which way the physical "belly" really faces relative to +X. The code only knows the names TL/TR/BL/BR and the signs above. The names suggest left = +Y when looking along +Z from the tail toward the nose (X up/back, Y to the left makes X × Y = Z pointing away from the viewer: consistent), but this is an inference from naming, not from code.

### Sensor axes into the body frame

| Sensor | Transform | Source |
|---|---|---|
| MPU9250 accel | Driver remap, chip `(x,y,z)` to driver/sensor frame `(y, x, −z)`, in m/s² | `lib/ExVectrSensor/src/mpu9250.cpp:886-891` |
| MPU9250 gyro | Driver remap, chip `(x,y,z)` to driver/sensor frame `(y, x, −z)`, rad/s | `mpu9250.cpp:893-898` |
| QMC5883 magnetometer | No axis remap in driver | |
| All three, app level | `cal = C·T · (raw − b)` (section 8a), from EEPROM keys 100-102 (`C`, `b`) and 200-202 (`T`). Read live with `CALIBRATION` | `src/main.cpp:1804-1812`, `:654` |

**Bench vehicle EEPROM (read with `CALIBRATION`, 2026-11-20):** sensor -> body is `diag(−1, 1, −1)` for gyro and accel (with the accel scale `diag(0.9984, 1.003, 0.9673)`), and `[[0,0,−1],[1,0,0],[0,−1,0]]` (times the scale 0.1622) for the mag, i.e. the values written once by the now-disabled `if (false)` block in `initialiseMemory` plus a later `CalibApply` mag fit. With these, the resting vehicle's calibrated accel points along body +Z and the calibrated mag gives a plausible field direction (north and down, 51° inclination in the vehicle's units), and the EKF quaternion agrees with both. Whether body +Z is the physical nose (vehicle standing upright) is for the user to confirm. **UNKNOWN (needs the user):** how the IMU board is physically mounted in the airframe. An accel tilt compensation hook exists, set to 0° (`main.cpp:1922-1924`).

### Attitude quaternion

- **[code]** `[w,x,y,z]`, Hamilton, **body to world**: `v_world = q.rotate(v_body)` (`docs/libs/ExVectrMath.md`). The state is `[ω(3), q(4)]`.
- **[code]** The identity quaternion `[1,0,0,0]` means body axes = world axes: **upright, body +Z (nose) pointing world Up**, body +X pointing North, +Y West. The simulator starts from identity (`body_simulator.cpp:28`) and the tilt check uses body Z rotated into world versus world Z (`main.cpp:1115-1120`).
- **[code]** Bellyflop setpoint `Quat(Y axis, −90°)` (`src/mission/mission_bellyflop.hpp:102`): body +X (back) points world Up, belly down, nose points world **−X (South)**, horizontal.
- **Pitch/azimuth convention (flap controller, `control_attitude_flaps.cpp:105-120`):** azimuth = `atan2(zBody.y, zBody.x)` in world (counter-clockwise from North toward West, not a compass heading). **Pitch = elevation of body +Z above the world horizontal plane**, so **positive pitch = nose up, negative pitch = nose tipped down toward the ground**. Body-frame rotation about +Y by a positive angle tips the nose toward body/world +X (right-handed).

### Flaps

Four flap servos, names from the code: **TL** top-left, **TR** top-right, **BL** bottom-left, **BR** bottom-right. In the code "top" and "bottom" (UL/UR, DL/DR in `starship_hardware.h`) mean the **forward (+Z, z = +0.26 m)** and **aft (−Z, z = −0.26 m)** flap pairs **[sim]**.

| Flap | PWM pin (`src/starship_hardware.h:15-18`) | Body position (m) [sim] | Servo value (`starship_flaps.cpp:158-165`) |
|---|---|---|---|
| TL | 25 (`FLAP_SERVO_PIN_UL`) | `(0, +0.08, +0.26)` | `1 − angle/(π/2)` |
| TR | 24 (`FLAP_SERVO_PIN_UR`) | `(0, −0.08, +0.26)` | `angle/(π/2)` |
| BL | 29 (`FLAP_SERVO_PIN_DL`) | `(0, +0.08, −0.26)` | `angle/(π/2)` |
| BR | 28 (`FLAP_SERVO_PIN_DR`) | `(0, −0.08, −0.26)` | `1 − angle/(π/2)` |

- **Hinge axis [sim]:** all four hinge axes are **parallel to body Z** (the long axis). The simulator rotates each flap about Z by `∓angle` (TL, BL: `−angle`; TR, BR: `+angle`) with the centre of pressure `0.03` m out from the hinge in ±Y (`body_simulator.cpp`, flap block). At angle 0 the flap points straight out sideways (±Y, away from the body). At 90° it has swung to point toward body **+X (the back/top of the vehicle, away from the belly)** on all four. **UNKNOWN (needs the user):** whether the real flaps fold toward the back (+X) or toward the belly (−X), their chord/span dimensions, and the real hinge location. The simulator's model is the only source.
- **Angle meaning [code]:** radians. **0° = fully extended**, **90° = retracted/folded** against the body. Evidence: ascent without the extend command commands 90° on all four, `Upright` commands top 0° and bottom 90° (`control_attitude_flaps.cpp:174-189`). So during bellyflop the flaps are extended into the airflow.
- **Sign convention [code]:** one commanded angle per flap, same sense for the left and right flap of a pair (mirroring is done in the servo mapping above). Larger angle = more folded.
- **Clamp [code]:** the driver limits every angle to **[`FLAP_MIN_ANGLE` = 15°, `FLAP_MAX_ANGLE` = 90°]** (`starship_flaps.hpp:36-37`, `:93-99`). 0° is never commanded in flight. The `Upright` mode's 0° becomes 15°.
- **Servo [code]:** STANDARD PWM, value 0..1 maps to 1–2 ms, 90° of travel mapped over the full range; trim offsets all zero (`starship_flaps.cpp:146-150`). Slew limited to 200°/s in the simulated "true" position (`:60-63`).
- **Controller mixing [code]** (Stabilize mode, `control_attitude_flaps.cpp:150-167`): with controller output `u = [azimuth+rates (x), pitch+rates (y), roll rate (z)]`:
  TL = neutral + (−u0 − u1 + u2)·k, TR = neutral + (+u0 − u1 − u2)·k, BL = neutral + (+u0 + u1 + u2)·k, BR = neutral + (−u0 + u1 − u2)·k (neutral top / bottom set by `flapTopNeutralAngle_Rad_` / `flapBottomNeutralAngle_Rad_`).
- **Telemetry [code]:** the true flap angles `[TL, TR, BL, BR]` (rad) are in `ACTUATORS` (port 0x81, 50 Hz) on **USB only**. They are not in the radio set ("bare minimum on the radio", section 9).

### TVC vanes and motors

- **Vanes [code]:** four exhaust vanes in the thrust flow at the aft end, named by their position on the body axes. XP at +X, XN at −X, YP at +Y, YN at −Y (`starship_tvc.hpp:37-40,73-80`). Servo pins (`src/main.cpp:92-95`, macros in `starship_hardware.h:11-14`): **XP = pin 1 (`TVC_SERVO_PIN_3`), XN = pin 0 (`TVC_SERVO_PIN_1`), YP = pin 23 (`TVC_SERVO_PIN_2`), YN = pin 22 (`TVC_SERVO_PIN_4`)**. The macro numbering is not the XP..YN order.
- **Vane axis and sign [code, partly inferred]:** the X-pair vanes rotate about body X and deflect the exhaust in **Y**, the Y-pair rotate about body Y and deflect in **X**: `xAngle = atan2(Fy, Fz)` drives the X-pair, `yAngle = atan2(Fx, Fz)` the Y-pair (`starship_tvc.cpp:116-119`). Opposite vanes are mirrored (`XN = −(…)`, `YN = −(…)`), a common offset on all four (`twist`) gives roll torque about Z. Servo travel ±45° maps to ±3° thrust deflection (factor 15). Vane geometry (size, offset from the axis) is **UNKNOWN (needs the user)**. The sign of the physical vane rotation versus the servo direction cannot be confirmed from code.
- **Trim [code]:** XP +5°, XN −2°, YP 0°, YN +5° (`main.cpp:1771-1772`).
- **Motors [code]:** two contra-rotating motors on the thrust axis at the aft end. `MOTOR_PIN_CW` = **8** (clockwise), `MOTOR_PIN_CCW` = **7** (counter-clockwise) (`starship_hardware.h:20-21`). **UNKNOWN (needs the user):** clockwise as seen from which end (assumed: looking down the thrust axis from the nose end). They receive identical commands, so no differential yaw. Total thrust limit 14 N.
- **Lever arm [code]:** the thrust point is **0.35 m aft of the origin**, at body `(0, 0, −0.35)`. Thrust acts along +Z (pure thrust is `+Z` body force) (`main.cpp:1967`).
- **Origin [code]:** the body origin is the centre of mass in the simulator. **UNKNOWN (needs the user):** the real CoM position and total vehicle length/diameter (needed to scale a 3D model).

### Diagram

```
 Side view, vehicle upright, looking along +Y (from the -Y side), +X to the right
 (belly = -X on the left, back = +X on the right)

              +Z (nose, world Up when q = identity)
               ^
               |            TL / TR  top (forward) flaps   z = +0.26
        belly  |  back      hinge parallel to Z, at y = +-0.08
         -X <--+--> +X      
               |            origin (CoM)                    z =  0
               |            BL / BR  bottom (aft) flaps    z = -0.26
               |            vanes XP(+X) XN(-X)  at aft end
               |            motors CW(8) CCW(7), thrust along +Z at z = -0.35
               v   -Z (tail)

 Top view, looking down -Z (from the nose toward the tail), X up the page:

                 +X (back)
                  ^
                  |
        TL/BL     |     TR/BR
        (+Y)  [flap]   [flap]  (-Y)
     +Y <---------+---------> -Y
        (left?)   |          (right?)    left/right: UNKNOWN, see above
                  |
        vanes: XP at +X, XN at -X, YP at +Y, YN at -Y
                  v
                 -X (belly)
```

(Looking down −Z from the nose with X up, +Y is on the viewer's **left**, because X × Y = Z points toward the viewer. This is the right-handed arrangement; it is shown as-is and is correct for the frame as defined.)

### Open items for the user

1. Physical left/right: which side of the real vehicle is +Y (TL/BL)?
2. Which way does the real belly face (assumed −X)?
3. Real flap hinge location, fold direction at 90° (toward back or belly), and dimensions.
4. IMU and magnetometer mounting (the EEPROM matrices are now readable via `CALIBRATION`, section 8a).
5. Vane geometry and physical rotation sign per servo, motor spin direction viewpoint, CoM position and vehicle size.
6. Whether flap angles should be sent over radio (today USB only, `ACTUATORS`).

---

## Known issues & limitations

| Severity | Location | Issue | Impact |
|---|---|---|---|
| High | `ublox_gnss.cpp:199-200`, `:208` | GPS is disabled (Galileo only), and the setting is saved to the receiver's non-volatile config. | Far fewer satellites, worse fix, slower TTFF. It persists even after the code is changed, until the receiver is reconfigured. |
| High | main.cpp:1233-1237, `starship_tvc.cpp:247-250` | Arming does not gate the motors. Disarmed only sets `motorPowerLimit = 0.1` (+0.04 idle). | Motors spin on a disarmed vehicle whenever `enableMotors_` and the actuators are enabled. |
| Medium | `starship_tvc.cpp:27`, `:36` | TVC output is event-driven on controller publications only. | If the controller stops publishing, the servo, motor, enable and test state freezes at the last values. Motors keep their last command. |
| Medium | `starship_tvc.cpp:191-198` | Trim (up to 5°/45° = 0.11), roll twist (±0.5) and full deflection (±1) are summed with no prioritisation. PWM clips at 0/1. | Pitch/yaw authority is lost asymmetrically under roll demand. Saturation is not reported to the controller (the "actual" topic does not model it). |
| Medium | `starship_tvc.cpp:116-119` vs `:76-92` | Vane angles use the unclamped force direction, while the "actual" topic uses the 3°-cone-clamped vector. The rate-limit model (600°/s) is not applied to the real servos. | Simulator and telemetry thrust direction can differ from what the vanes do near the limits. |
| Medium | `ublox_gnss.cpp:23-31`, `:37` | `positionCov` and `velocityCov` hold 1σ accuracies (m, m/s), not variances. Validity is only `numSats ≥ 5` with the fix-OK check disabled. | Wrong KF weighting unless the consumer knows it gets σ. A 2D or invalid fix with 5 satellites is accepted as valid. |
| Medium | `vl53l0x_driver.cpp:834-856` | Range status is not checked. `cov = 0.15·range` is σ-like, not m². | Out-of-range readings (~8.19 m) are published. The KF window 0.4–1.5 m (main.cpp:1932) masks this today. |
| Medium | `vl53l0x_driver.cpp:72-73`, main.cpp:1732 | The VL53L0X sets its I2C device to 400 kHz on the shared `Wire`, while main sets 100 kHz. | Whether the bus clock changes per transaction depends on the HAL. The BME280, QMC5883 and OLED may run at 400 kHz. |
| Medium | `PWM_Output` (lib) used by both drivers | `outputEnabled_` is never assigned in `enableOutput()` (`lib/ExVectrActuator/src/pwm_output.cpp:103-114`). Disable writes 0 duty once, but the next `setValue()` drives the pin again. | The drivers only work because they call `setValue` exclusively while enabled. Any future `setValue` on a disabled output re-energises it. |
| Medium | `ublox_gnss.cpp:199` | `delay(1000)` (plus up to three blocking `begin()` attempts) inside `taskInit`. | Stalls the cooperative scheduler, including IMU processing, for at least 1 s at t ≈ 5 s. |
| Low | `starship_flaps.hpp:37` | `FLAP_MIN_ANGLE = 15°` clamps the controller's 0° (full extension, used by `Upright`). | Flaps never fully extend. The true-flap topic shows 15°. |
| Low | main.cpp:1789-1792 | Gyro bias hard-coded at boot overrides the stored calibration. | Stale bias when the IMU or temperature changes. The key 100 bias is silently ignored. `CALIBRATION` shows it (`OFFSET_OVERRIDE`). |
| Medium | EEPROM key 102 (bench vehicle) | The stored accel calibration (`b = (0.428, 0.558, 2.143)` m/s², `C·T = diag(−0.9984, 1.003, −0.9673)`) maps the resting raw accel (\|raw\| 8.44 m/s²) to \|cal\| 10.3 m/s², not 9.81. The z offset no longer matches the sensor (it may have shifted, or the values were fitted in another frame). | The tilt estimate can be off by a few degrees (the x/y offset error is not observable at rest in one orientation). Needs a new accel calibration (six-position) before flight; there is no accel calibration command. |
| Medium | `lib/ExVectrSensor/src/qmc5883.cpp:99-101`, `:170` | The QMC5883 is configured for the 8 G range (3000 LSB/G per datasheet) but converted with 8/32767 G/LSB (4096 LSB/G). | `IMU_RAW.mag` is 0.73x the physical field (0.35 G read for about 0.48 G). Direction and the existing mag calibration are unaffected; do not "fix" it without recalibrating the mag (the stored offset is in these units). |
| Medium | attitude EKF (`lib/ExVectrDSP`), seen in `ESTIMATOR` | The EKF quaternion covariance does not converge on the bench: `att_std` stays at about 0.3-1 rad tilt and several rad heading. The mag correction is a fixed-gain complementary step that never reduces `P`, and the accel update uses the driver variance × 5000. | `att_std` is not a usable confidence today, although the attitude itself is consistent with the calibrated accel and mag (2026-11-20 bench check: accel to world up within 1.6°, mag heading error 1.5°). Estimator change: needs a decision and simulation verification. |
| Low | main.cpp `systemsInitialised` | "GNSS data not new" (and the other "... not new" lines) are logged every 100 ms until all sensors deliver. | About 10 `LOG_TEXT` lines/s on the bench without GNSS. Not rate-limited (only the auto-disarm messages are, 2026-11-20). |
| Low | `starship_hardware.h:25-26`, `starship_tvc.hpp:92-94` | `TVC_SERVO_LIMIT` is unparenthesised. `TVC_ANGLE_FACTOR` is duplicated by the hard default `tvcFactor = 15`. | Changing the define does not change the TVC mixing. Macro expansion hazard. |
| Low | `ublox_gnss.cpp:150-170` | Baud fallback messages are wrong ("15200", always "9600"). A failed 115200 reconfig returns without `setInitialised(false)`. | Misleading logs. The task may run unconfigured. |
| Low | `telecommand.hpp:152`, `:156`, `:159` | Header comments disagree with the code: CalibStart duration, SystemReset leaving sim mode, MissionBegin "seconds in the future". | Operator and ground-software confusion. |
| Info | `starship_hardware.h` `RX_MODULE_SERIALPORT` | Resolved 2026-11-20: the ship node is wired to Serial4 (pin 16 RX4 / pin 17 TX4), confirmed end to end (Router shows node 1, vehicle telemetry arrives over the radio). It was first on pads 30/31 and was moved. | None. If the harness is rewired, re-check with `tools/uart_pin_scan.py` (section 5). |
| High | main.cpp `checkForFailures`, section 5 | **TEMPORARY until T9b:** `radioConnectionLoss` = no valid frame from node 0 for 2 s on USB or UART. Over the radio path the modules drop node 0 heartbeats by design, so a ground station reached only by radio keeps the vehicle out of Failure only if it sends node 0 data (telecommand, `REQUEST`) at least every 2 s. The ground station has no telecommand encoder yet. | Over the radio alone the vehicle enters Failure after initialisation unless the ground station sends data. Bench use over USB behaves as before. T9b replaces it with the `RC_STATE` control-source timeout. |
| Medium | section 10 | **TEMPORARY until T9b:** the legacy telecommand on port 0x7F is accepted from any frame claiming source 0 on either link, with only the three magic values as protection (as before on port 1000). It is not in the ExVectrPackets schema. | Same exposure as before; any device on USB or behind the RX module can command the vehicle. |
| Low | section 9 | `ACTUATORS` has no field for the TVC roll torque `T`. | Roll torque is no longer telemetered (it was `VehicleState.tvcTorque`, mis-scaled). Needs a schema field. |
| Low | `ExVectrNetwork` (library) | The legacy SX1280 sources of the library are compiled (unused, dropped at link time). | Library warnings in the build log. |
| Info | altitude resolution (`POSE.pos[2]`, `AIR_POSE.pos[2]`) | The telemetry itself no longer limits it: `POSE` is f32 (sub-mm at a few m) and `AIR_POSE` is 1 mm since 2026-11-20 (was 0.1 m). What remains: BME280 pressure at 16x oversampling has about 0.2 Pa noise (about 1.5-2 cm of altitude), IIR filter setting 1, and the f32 pressure (Q24.8 compensation output, resolution 0.0078 Pa near 100 kPa, about 0.7 mm) and `calcAltitudeFromPressure` in f32 (about 0.5 mm per ULP of p/p0). The position KF fuses baro, GNSS and range with large baro covariance (`cov*1000`), and the reference is a 10 % low-pass of the baro during zeroing, so the altitude is smoothed, not quantised. No casts to integer on the vehicle before the packet. | Expect centimetre-level noise and slow drift (weather, temperature), not steps. Estimator not changed. |
| Info | `board_v_1_0.h` | BME280 SPI CS, RGB LED, battery ADC are defined but unused (QMC INT pin 16 is now RX4). There is no battery monitoring. | No low-voltage protection. |
