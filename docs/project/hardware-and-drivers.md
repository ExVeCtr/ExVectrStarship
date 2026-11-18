# Hardware and Drivers

Pinout, vehicle constants and the application-level drivers in `src/` (TVC, flaps, SX1280 datalink, u-blox GNSS, VL53L0X), plus memory keys, telemetry and telecommand wire formats.

Last verified against commit: b073d07

Target: Teensy 4.0 (`platformio.ini` `[env:teensy40]`), little-endian. Frames and units follow [../codebase-structure.md](../codebase-structure.md): NWU reference, body +Z = thrust axis (nose), SI units, time in ns. Library internals (PWM_Output, controllers, KF) are documented in [../libs/](../libs/). This file covers only how `src/` uses them.

Vendored third-party code (gitignored, not ExVectr):

| Folder | What it is |
|---|---|
| `src/gnssdriver/` | SparkFun u-blox GNSS Arduino Library v2.2.27 (`SFE_UBLOX_GNSS`), used by `ublox_gnss.*`. |
| `src/sx12xxAL/` | StuartsProjects SX12XX-LoRa library (`SX128XLT`), "modified for use with ExVectr" per its ReadMe, used by `datalink_sx1280.*`. |

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
| 16 | `QMC5883L_INT_PIN` | Magnetometer interrupt (**unused**) | — |
| 9 | `SX1280_NSS_PIN` | SX1280 SPI CS | `datalinkSX1280` |
| 5 | `SX1280_NRESET_PIN` | SX1280 reset | 〃 |
| 15 | `SX1280_RFBUSY_PIN` | SX1280 BUSY | 〃 |
| 14 | `SX1280_DIO1_PIN` | SX1280 DIO1 (IRQ, polled) | 〃 |
| 10 | `SX1280_RXEN_PIN` | RF switch RX enable | 〃. Also set HIGH in `initialiseHardware` (main.cpp:1744-1745). |
| 17 | `SX1280_TXEN_PIN` | RF switch TX enable | 〃. Also set LOW (main.cpp:1747-1748). |
| 4 | `RGBLED_PIN` | RGB LED (**unused**) | — |
| 26 | `INPUT_VOLTAGE_ADC_PIN` | Battery ADC, factor 1.0 (**unused**) | — |
| Serial5 | `NEO_M8Q_SERIALPORT` (define unused; main.cpp:82 hard-codes `Serial5`) | u-blox GNSS UART | `gnss` |

Buses (main.cpp:1729-1732): `SPI` for the MPU9250 and SX1280. `Wire` at 100 kHz for the BME280 (0x76), QMC5883 (0x0D), VL53L0X (0x29) and SSD1306 128×32 OLED (0x3C). The VL53L0X driver requests 400 kHz on its bus device (`vl53l0x_driver.cpp:72-73`), and that bus is the same `Wire`.

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
| out | `getTVCActualThrustTopic()` | `[X, Y, Z, T]`: modelled achieved thrust vector (N, body) and roll term. Consumed by `bodySimulator`, `CommsSystemStateTask` and debug print. |

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

## 5. `Datalink_SX1280` (`src/datalink_sx1280.hpp/.cpp`)

Implements `Net::Datalink_Interface` on an SX1280 at 2.4 GHz for `Net::NetworkNode networkNode(1, datalinkSX1280, 2 s)` (main.cpp:131): vehicle node address 1, ground station node 0, peer timeout 2 s.

**Radio configuration (`taskInit`, `.cpp:39-71`):**

| Parameter | Value |
|---|---|
| Modem | **LoRa** (FLRC is not used) |
| Frequency | 2 445 000 000 Hz, offset 0 (`.cpp:62`) |
| SF / BW / CR | SF5 / 1600 kHz / 4:6 |
| Packet params (library `setupLoRa`) | 12-symbol preamble, explicit (variable-length) header, max 255, LoRa CRC on, IQ normal |
| Regulator | LDO. `setHighSensitivity()` |
| TX power | 12 dBm (`transmit(..., 12, NO_WAIT)`, `.cpp:353`) |
| RF switch | `lora_.begin(..., RX_EN_PIN_, TX_EN_PIN_, DEVICE_SX1280)`. The library toggles RXEN (10) and TXEN (17) itself. The pins start LOW. |

**Framing:** upper layers hand frames of ≤ `dataLinkMaxFrameLength = 200` B to `dataframeReceiveFunc`. These are queued as `[len][bytes…]` in a 1000 B ring (`transmitBuffer_`); a full buffer drops the frame. One radio packet packs whole frames while `size + len + 2 ≤ 250` (`.cpp:301-309`) and appends a 1-byte **additive checksum** over all preceding bytes:

```
| len0 | frame0 ... | len1 | frame1 ... | ... | sum8 |
```

RX accepts a packet only if the IRQ shows `RX_DONE && HEADER_VALID && !CRC_ERROR` and the sum8 matches. It then splits the packet into frames and publishes each one on `receiveTopic_`.

**Half-duplex state machine** (polled: `taskCheck` releases the 10 ms task immediately when DIO1 is high or TX is pending while Idle):

| State | Entered when | Leaves on |
|---|---|---|
| Idle | IRQ done, error or timeout | Immediately `beginReceive()` → Receiving (single RX, timeout 0) |
| Receiving | `beginReceive` | RX_DONE, error or timeout → Idle. Re-armed every 500 ms. TX pending and ≥ 1 ms since the last TX end → ActivityDetection. |
| ActivityDetection | `startCAD(LORA_CAD_08_SYMBOL)` (`.cpp:243`), listen-before-talk | `CAD_DONE` → transmit → Transmitting. `CAD_ACTIVITY_DETECTED` → `beginReceive(1000)`. 500 ms timeout → Idle. |
| ChannelBusy | `PREAMBLE_DETECTED` | RX completes, or 500 ms → Idle |
| Transmitting | after `transmit()` | `TX_DONE`, `TX_TIMEOUT`, or 500 ms → Idle |

RSSI and SNR of the last packet: `lastPacketRSSI()`, `lastPacketSNR()`.

**Network ports in use** (`Net::TransportTopic(port, dst = UINT16_MAX, networkNode, topic)`, main.cpp:145-161). All are bidirectional (`noReceive = false`).

| Port | Type | Direction (by use) | Source and rate |
|---|---|---|---|
| 10 | `Net::PacketAttitude` | TX | `attitudeTelemetryCallback`, ≤ 5 Hz (main.cpp:189-206) |
| 11 | `Net::PacketPosition` | TX | `positionTelemetryCallback`, ≤ 10 Hz (main.cpp:225-243) |
| 12 | `Net::PacketGPS` | TX | `gnssTelemetryCallback`, ≤ 1 Hz (main.cpp:262-274) |
| 50 | `VehicleState` | TX | `CommsSystemStateTask`, 5 Hz (main.cpp:1343-1378) |
| 51 | `MissionState` | TX | `CommsSystemStateTask`, 5 Hz (main.cpp:1385-1389) |
| 100 | `uint8_t` connection counter | TX | `DisplayTask`, 10 Hz (main.cpp:1559). **Only if the OLED init succeeds**; otherwise the task removes itself (main.cpp:1424-1427). |
| 1000 | `Telecommand` | RX | → `telecommandTopic` → safety task |

The `Net::Packet*` layouts belong to ExVectrPackets ([../libs/ExVectrPackets.md](../libs/ExVectrPackets.md)).

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
| timestamp | `Core::NOW()` at read | Not GNSS time; includes UART latency. |

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

Applied as `TopicCoordTransform::setTransform(calib.cov * transform, calib.val)` (main.cpp:1862-1867). **The gyro bias is overridden in RAM at every boot** with hard-coded `(0.92, 2.0, 0.6) °/s` (main.cpp:1848-1850). It is never written to memory, so the stored key 100 bias is ignored.

---

## 9. Telemetry structs (`src/telemetry.hpp`)

All structs are `__attribute__((packed))`, little-endian, and sent raw by `TransportTopic`.

### `VehicleState`: 27 bytes, port 50

| Off | Size | Field | Encoding (filled at main.cpp:1360-1377) |
|---|---|---|---|
| 0 | 1 | `mode` | `VehicleMode`: 0 Startup, 1 SensorInit, 2 SensorCalib, 3 Ready, 4 Running, 5 Failure |
| 1 | 1 | `sensoryState.gnss` | `TelemetrySensor`: 0 Init, 1 Calib, 2 Ready, 3 Failure |
| 2 | 1 | `sensoryState.imu` | 〃 |
| 3 | 1 | `sensoryState.baro` | 〃 |
| 4 | 1 | `sensoryState.mag` | 〃 |
| 5 | 1 | `sensoryState.attitudeKF` | 〃 |
| 6 | 1 | `sensoryState.positionKF` | 〃 |
| 7 | 1 | `failureState.sensorFailure` | bool |
| 8 | 1 | `failureState.attitudeFailure` | bool (never set) |
| 9 | 1 | `failureState.positionFailure` | bool (never set) |
| 10 | 1 | `failureState.positionOutOfBounds` | bool |
| 11 | 1 | `failureState.attitudeOutOfBounds` | bool |
| 12 | 1 | `failureState.radioConnectionLoss` | bool |
| 13 | 1 | `simulationModeEnabled` | bool |
| 14 | 1 | `armed` | bool |
| 15 | 2 | `tvcThrustX` | int16 = `actual.X / 50 · 32767` (N) |
| 17 | 2 | `tvcThrustY` | int16, same scale |
| 19 | 2 | `tvcThrustZ` | int16, same scale |
| 21 | 2 | `tvcTorque` | int16 = `actual.T / 50 · 32767` |
| 23 | 2 | `groundDistance` | int16 = `g(1) / 2 · 32767` (m, full scale 2 m) |
| 25 | 2 | `groundDistanceCov` | int16 = `P(1,1) · 32767` (full scale 1) |

### `MissionState`: 35 bytes, port 51

| Off | Size | Field | Notes |
|---|---|---|---|
| 0 | 24 | `float positionSetpoint[6]` | `[Vx,Vy,Vz,Px,Py,Pz]`, ref frame |
| 24 | 1 | `missionMode` | `MissionMode`: 0 Idle, 1 Initialisation, 2 Startup, 3 Running, 4 Finished |
| 25 | 8 | `int64 missionTime` | **ns** (`Time_Source::NOW()`). Negative during the countdown. |
| 33 | 1 | `missionIndex` | Effectively always 0 (see [missions.md](missions.md) §6.2) |
| 34 | 1 | `missionPhase` | `MissionPhase`: 0 Idle, 1 Hover, 2 Ascent, 3 Descent, 4 Deceleration, 5 RTH, 6 Landing, 7 Completed |

## 10. Telecommands (`src/telecommand.hpp`), port 1000

### `Telecommand`: 8 bytes, packed

| Off | Size | Field |
|---|---|---|
| 0 | 1 | `type` (`TelecommandType`) |
| 1 | 1 | `subsystem` (`Subsystem`, `subsystems.hpp`: 0 Gyro, 1 Accel, 2 Magneto, 3 Baro, 4 GNSS, 5 AttFilter, 6 PosFilter) |
| 2 | 2 | `int16 paramInt` |
| 4 | 4 | `float paramFloat` (unused by every handler) |

### Command list (handler: `VehicleSafetyAndControlTask::handleTelecommands`, main.cpp:728-978)

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

There is no authentication beyond the three magic values. Every port is also receivable, so a ground node can inject data on ports 10-100. Those topics feed only telemetry and debug paths today.

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
| MPU9250 accel | Driver remap, chip `(x,y,z)` to body `(y, x, −z)`, in m/s² | `lib/ExVectrSensor/src/mpu9250.cpp:886-891` |
| MPU9250 gyro | Driver remap, chip `(x,y,z)` to body `(y, x, −z)`, rad/s | `mpu9250.cpp:893-898` |
| QMC5883 magnetometer | No axis remap in driver. App transform `sensorToBody_` is the **identity** (`src/main.cpp:786`), then calibration `cov * sensorToBody_` (`:799-801`) | |
| All three, app level | `TopicCoordTransform` with `calib.cov * stored transform` and `calib.val` offset (`src/main.cpp:1862-1866`, `:1921-1923`). The stored transform matrices (`MEMORY_KEY_ACC/GYRO/MAGTRANSFORM`) default to identity but are read from EEPROM, so **the real values are unknown from code**. | |

The `if (false)` test block (`main.cpp:1886-1917`) documents earlier tried values (`diag(−1,1,−1)` for acc and gyro, a permutation for the mag) that are **not active**. The magnetometer needs its own axis mapping that code alone cannot confirm. **UNKNOWN (needs the user):** contents of the EEPROM transform and calibration matrices, and how the IMU board is physically mounted (the driver remap above says the chip is rotated 90 degrees about Z and flipped, but not how that sits in the airframe). An accel tilt compensation hook exists, set to 0° (`main.cpp:1976-1978`).

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
- **Telemetry [code]:** **flap angles are not on the radio link.** `VehicleState` (port 50) carries only TVC thrust and ground distance, and no other packet carries flap data. The only report is the USB debug print `Flap Debug [TL, TR, BL, BR]` in **radians, unscaled floats**, of the simulated true position, when `DEBUG_TELEMETRY_SERIAL` is defined (`main.cpp:278-287`). A ground-station 3D model therefore has to either infer flap angles from the mission/phase, or a new packet has to be added (cross-project change, not done here).

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
4. IMU and magnetometer mounting, and the EEPROM transform/calibration matrices.
5. Vane geometry and physical rotation sign per servo, motor spin direction viewpoint, CoM position and vehicle size.
6. Whether flap angles should be sent over radio (not currently telemetered).

---

## Known issues & limitations

| Severity | Location | Issue | Impact |
|---|---|---|---|
| High | `ublox_gnss.cpp:199-200`, `:208` | GPS is disabled (Galileo only), and the setting is saved to the receiver's non-volatile config. | Far fewer satellites, worse fix, slower TTFF. It persists even after the code is changed, until the receiver is reconfigured. |
| High | main.cpp:1233-1237, `starship_tvc.cpp:247-250` | Arming does not gate the motors. Disarmed only sets `motorPowerLimit = 0.1` (+0.04 idle). | Motors spin on a disarmed vehicle whenever `enableMotors_` and the actuators are enabled. |
| Medium | `starship_tvc.cpp:27`, `:36` | TVC output is event-driven on controller publications only. | If the controller stops publishing, the servo, motor, enable and test state freezes at the last values. Motors keep their last command. |
| Medium | `starship_tvc.cpp:191-198` | Trim (up to 5°/45° = 0.11), roll twist (±0.5) and full deflection (±1) are summed with no prioritisation. PWM clips at 0/1. | Pitch/yaw authority is lost asymmetrically under roll demand. Saturation is not reported to the controller (the "actual" topic does not model it). |
| Medium | `starship_tvc.cpp:116-119` vs `:76-92` | Vane angles use the unclamped force direction, while the "actual" topic uses the 3°-cone-clamped vector. The rate-limit model (600°/s) is not applied to the real servos. | Simulator and telemetry thrust direction can differ from what the vanes do near the limits. |
| Medium | `datalink_sx1280.cpp:183-226` | CAD_DONE is always raised at the end of CAD, including when CAD_ACTIVITY_DETECTED is also set. Both branches run in the same pass, and the CAD_DONE branch transmits anyway. | Listen-before-talk is ineffective: the vehicle transmits over an active channel. |
| Medium | `datalink_sx1280.cpp:239-243`, `:274` | Entering ActivityDetection does not update `channelBusyStart_`, so the 500 ms busy timeout usually fires in the same pass, and the state returns to Idle. CAD is re-issued on every pass until DIO1. | Busy-loop of CAD restarts. TX starts when a CAD_DONE happens to be seen. |
| Medium | `datalink_sx1280.cpp:399-413` | The RX parser trusts `segLen` without checking it against `packetL`, protected only by an 8-bit additive sum (plus LoRa CRC). | Malformed or colliding packets can read past the buffer and publish garbage frames. |
| Medium | `ublox_gnss.cpp:23-31`, `:37` | `positionCov` and `velocityCov` hold 1σ accuracies (m, m/s), not variances. Validity is only `numSats ≥ 5` with the fix-OK check disabled. | Wrong KF weighting unless the consumer knows it gets σ. A 2D or invalid fix with 5 satellites is accepted as valid. |
| Medium | `vl53l0x_driver.cpp:834-856` | Range status is not checked. `cov = 0.15·range` is σ-like, not m². | Out-of-range readings (~8.19 m) are published. The KF window 0.4–1.5 m (main.cpp:1932) masks this today. |
| Medium | `vl53l0x_driver.cpp:72-73`, main.cpp:1732 | The VL53L0X sets its I2C device to 400 kHz on the shared `Wire`, while main sets 100 kHz. | Whether the bus clock changes per transaction depends on the HAL. The BME280, QMC5883 and OLED may run at 400 kHz. |
| Medium | `PWM_Output` (lib) used by both drivers | `outputEnabled_` is never assigned in `enableOutput()` (`lib/ExVectrActuator/src/pwm_output.cpp:103-114`). Disable writes 0 duty once, but the next `setValue()` drives the pin again. | The drivers only work because they call `setValue` exclusively while enabled. Any future `setValue` on a disabled output re-energises it. |
| Medium | `ublox_gnss.cpp:199` | `delay(1000)` (plus up to three blocking `begin()` attempts) inside `taskInit`. | Stalls the cooperative scheduler, including IMU processing, for at least 1 s at t ≈ 5 s. |
| Low | `starship_flaps.hpp:37` | `FLAP_MIN_ANGLE = 15°` clamps the controller's 0° (full extension, used by `Upright`). | Flaps never fully extend. The true-flap topic shows 15°. |
| Low | main.cpp:1848-1850 | Gyro bias hard-coded at boot overrides the stored calibration. | Stale bias when the IMU or temperature changes. The key 100 bias is silently ignored. |
| Low | main.cpp:1375-1377 | `groundDistance` (full scale 2 m) and `groundDistanceCov` (full scale 1) overflow int16 when exceeded. Float → int16 out of range is undefined behaviour. | Garbage ground-distance telemetry above 2 m. |
| Low | main.cpp:232-237 | Position telemetry `hAcc = sqrt(P33² + P44²)/1000`, `vAcc = P55/1000`. Variances are squared and scaled by 1/1000. | The hAcc and vAcc values in `PacketPosition` have no physical unit. |
| Low | main.cpp:196-200 | `attitudeTelemetryCallback` prints a quaternion self-test line to Serial at 5 Hz, unconditionally (not under `DEBUG_TELEMETRY_SERIAL`). | Serial spam. |
| Low | main.cpp:1424-1427, `:1559` | The connection counter (port 100) is published only by `DisplayTask`. | No connection counter if the OLED is absent. |
| Low | `starship_tvc.hpp:118-122` | The doc says `T` is an "angle in radians", but it is the reconstructed roll torque. `VehicleState.tvcTorque` scales it as "50 N". | Misinterpretation of telemetry. |
| Low | `starship_hardware.h:25-26`, `starship_tvc.hpp:92-94` | `TVC_SERVO_LIMIT` is unparenthesised. `TVC_ANGLE_FACTOR` is duplicated by the hard default `tvcFactor = 15`. | Changing the define does not change the TVC mixing. Macro expansion hazard. |
| Low | `ublox_gnss.cpp:150-170` | Baud fallback messages are wrong ("15200", always "9600"). A failed 115200 reconfig returns without `setInitialised(false)`. | Misleading logs. The task may run unconfigured. |
| Low | `datalink_sx1280.cpp:309`, `:351-360` | A zero-length queued frame is never dequeued. The "clear buffer" branch is unreachable because the checksum byte makes `bufferSize ≥ 1`. | A 1-byte packet is sent forever and the TX queue is stuck. |
| Low | `telecommand.hpp:152`, `:156`, `:159` | Header comments disagree with the code: CalibStart duration, SystemReset leaving sim mode, MissionBegin "seconds in the future". | Operator and ground-software confusion. |
| Info | `board_v_1_0.h` | BME280 SPI CS, QMC INT, RGB LED and battery ADC are defined but unused. There is no battery monitoring. | No low-voltage protection. |
