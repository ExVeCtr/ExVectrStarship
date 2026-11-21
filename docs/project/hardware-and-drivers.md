# Hardware and Drivers

Pinout, vehicle constants and the application-level drivers in `src/` (TVC, flaps, ExVectrNetwork links, u-blox GNSS, VL53L0X), plus memory keys, telemetry and telecommand wire formats.

Last verified against commit: b073d07 (section 9 `MISSION`/`AIR_MISSION` `setpoint_q` rows on `feature/setpoint-attitude`, 2026-11-21; sections 1, 5, 9, 10 and the Known issues re-verified on `feature/network-migration`, 2026-11-19)

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
| `VEHICLE_MASS_KG` | 1.1 (measured; a change to 1.2 on 2026-11-21 was reverted by the owner the same day) | Used by controller, simulator and freefall stop calculation. |
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

**"Actual" thrust model** (`.cpp:211-240`, only when actuators are enabled and the test is idle): direction = clamped `tvcVector`, magnitude = mean motor command × 14 N. The modelled thrust angles are rate-limited to the vane servo rate divided by the servo-to-thrust gain: 300°/s / `tvcFactor_` (15) = **20°/s**, so a full ±3° reversal takes 0.3 s (since 2026-11-21, `feature/sim-aero`, review F6; before: 600°/s in thrust-angle space = 9000°/s of servo). A non-finite command holds the previous angles and reports 0 thrust for that tick, so NaN can no longer latch into the integrated angles (it stayed NaN until reboot). `T = twist·|F|/45`, i.e. the roll torque reconstructed after clamping; `T` is zeroed when the actuators are disabled (review F13). The header calls `T` an "angle in radians".

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

**True flap topic:** `Safety::TrueFlapModel` (`src/safety/sim_flap_fault.hpp`, pure, host-tested), one slew model per flap, **300°/s** (`FLAP_SERVO_RATE_RADPS` = 60°/0.2 s, generic 9 g servo, owner 2026-11-21; was 200°/s), towards the **clamped** commanded angle while the flaps are enabled; while disabled the angles do not move (an unpowered servo stays where it is). **Published every tick (20 ms), enabled or not** (since 2026-11-21, `feature/sim-fixes-3`; before, only while enabled, so a simulation fault set on the ground never reached the topic). The simulation-only flap faults are applied at exactly this point (`TrueFlapModel::publishAngles`). Consumers (the only two subscribers): `BodySimulator` (flap input of the simulated aerodynamics) and `TelemetryProvider` (`TelemetrySnapshot.flaps` → ACTUATORS.flaps and AIR_ACTUATORS.flaps; the ground station's FlapState is decoded from those). No controller or observer reads a flap angle. Its `enableFlaps` field carries `enableActuators_`, not the combined enable.

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

**Radio link and `radioConnectionLoss` (user decision 2026-11-20):** `VehicleNetwork::isRadioLinkAlive()` (`src/comms/vehicle_network.cpp`) drives `radioConnectionLoss` in `checkForFailures`.
- A **radio packet** is a valid DATA frame that arrives **on the UART** with source **node 0** (ground station, relayed by the Router) or **node 2** (Router / Pocket: `RC_STATE`, `LINK_STATS`, ...), that is, from the far side of the air link. A small tap (`VehicleNetwork::RadioTap`) sits between the UART link and the UART node, records the receive time of such frames and passes every frame on unchanged.
- **Never counted:** heartbeats of any node (link-status information only, no timeout of their own), frames from node 3 (the ship node's own `LINK_STATS` and heartbeat prove only that the UART works), and everything on USB. The ground station on USB therefore no longer influences `radioConnectionLoss`.
- **Timeout:** alive while the last radio packet is at most **2 s** old (`kRadioTimeoutNs`). The Pocket's `RC_STATE` at 25 Hz is the normal keep-alive; any other node 0/2 data frame counts too.
- **Boot:** before the first radio packet since boot the link counts as alive for **10 s** after boot (`kRadioStartupGraceNs`, measured from the construction of `VehicleNetwork`, i.e. power-on). Why: the RX module and the air link need a few seconds after power-on, and failing at 2 s would latch Failure at every boot; never firing before the first packet would let a vehicle that never had a radio link become Ready. After the first radio packet the grace no longer applies.
- **SystemReset** clears the latched failure; the radio timestamp is kept, so with the radio still absent the failure latches again on the next check. **Simulation mode:** same rule (safety.md rule 7).
- **Ground-station presence** (`isGroundStationPresent()`: any valid frame from node 0 on USB or UART within 2 s) is unchanged but **information only**: `VEHICLE_STATUS` `GS_PRESENT`.
- **"Connected" is a separate predicate (review 2026-11-20):** `isRadioPacketRecent()` is true only if a radio packet really arrived within 2 s, **without** the boot grace. The OLED link indicator (`connected`, "OK") and its connection counter use it; before, they used `isRadioLinkAlive()` and reported a connection during the 10 s boot grace before any packet. `isRadioLinkAlive()` (with the grace) is used only for the `radioConnectionLoss` failure.
- What remains for T9b: control-source arbitration (which source may command) and the `RC_STATE` stale failsafe from the accepted control source (safety.md 4.1). Until then any node 0/2 data counts as radio activity, including a ground-station telecommand.

**Port table:** [../codebase-structure.md](../codebase-structure.md#radio-and-usb-network).

**Wiring (bench-confirmed end to end, 2026-11-20):** the ship node (ESP8285 "RX module", node 3) is wired to **Serial4**: ship node TX (GPIO1) to **Teensy pin 16 (RX4)**, **Teensy pin 17 (TX4)** to ship node RX (GPIO3), 3.3 V, 460800 8N1. Verified with the ExVectrLink Router: it lists node 1 as present and the vehicle telemetry arrives over the radio. History: the module was first found on pads 30/31 (GPIO_EMC_37/36), which are not pins of any Teensy 4.0 hardware UART, so no `SerialN` could read them; the wires were moved to 16/17. During the SX1280 removal the TX pin (17) had been shorted to ground; it was repaired.

**Finding the wiring again (bench tool):** `tools/uart_pin_scan.py` (run with the ground-station venv python, which has pyserial; needs the Router attached) disables the ground-station USB source, flashes the bench-only scan firmware (`pio run -e teensy40_pinscan`, `tools/pinscan/pinscan.cpp`, links no flight code), listens on every UART RX pin and scans all pads for edges, then transmits node 1 heartbeats on candidate TX pins one at a time while polling node 3's `NetworkStatus` through the Router (age[1] about 0 means the ship node heard node 1). It finishes by rebuilding and flashing the flight firmware of the checked-out branch and re-enabling the USB source (`--no-restore` leaves the scan firmware on). `--quick` re-checks only the known candidates (460800 listen, TX on pin 17, bit-bang pad 31). Use it after any rewiring or hardware change near the UART.

---

## 6. `UbloxSerialGNSS` (`src/ublox_gnss.hpp/.cpp`)

u-blox (NEO-M8Q per the define name) on `Serial5`, wrapping SparkFun `SFE_UBLOX_GNSS`. It implements `SNSR::GNSS` and publishes `Timestamped<SNSR::GNSSData>` on `getGNSSTopic()`.

- Task: `Task_Periodic("Ublox GNSS", 1 ms, start = 5 s)` (`.hpp:30`), so the **first run is at t = 5 s**. After the configuration, `taskCheck` unpauses the task when UART bytes are available and each `readGNSS()` call pauses it again.
- **Init, non-blocking since 2026-11-20 (`GnssConfigSequencer`, `src/gnss_config_sequencer.hpp/.cpp`, pure, host-tested in `test/test_gnss_config_sequencer`):** `taskInit` no longer talks to the receiver. While unconfigured the task stays unpaused and `taskThread` → `runConfiguration()` performs **at most one SparkFun call per run** (`performConfigAction`), each with its ACK wait bounded to **250 ms** (`kStepMaxWaitMs`; the SparkFun header documents 250 as safe, the default is 1100). Waits are timed states, nothing sleeps. Sequence: `begin` at 115200, else 9600, else 38400 baud (one attempt each; none answers: Failed, task paused for good, as before); `setSerialRate(115200)` (result ignored); reopen at 115200; UART1 output **UBX only**; **nav rate 10 Hz**; `setAutoPVT(true)`; dynamic model **AIRBORNE <2g**; **constellations GPS + Galileo** (user decision 2026-11-20): poll UBX-CFG-GNSS, then one CFG-GNSS write with GPS, Galileo and QZSS on and SBAS, BeiDou, IMES, GLONASS off (QZSS follows GPS: the receiver NACKs GPS on with QZSS off, bench 2026-11-20; QZSS has no satellites in view in Europe); the configuration found at boot is logged from that poll, BEFORE the write (`GNSS config found at boot (before write): GPS .. Galileo ..; GPS+Galileo as intended: yes/NO`; it shows a persisted save only after a receiver power cycle, a Teensy-only reboot leaves the receiver's RAM configuration); **1 s wait state** (GNSS restart after the CFG-GNSS change); save (UBX-CFG-CFG) after the wait (review 2026-11-20: it used to follow the write at once, during the restart); read back UBX-CFG-GNSS, UBX-MON-GNSS and UBX-CFG-RATE and log one line `GNSS read-back: CFG-GNSS ... MON-GNSS ... in use ... nav rate ... GPS+Galileo as intended: yes/NO` (once per boot; `UbloxSerialGNSS::constellationsAsIntended()` exposes the flag, no telemetry field). **Failure visibility (review 2026-11-20, information only):** while the sequence has ended (done or failed) and is not OK (failed, a step skipped, or the read-back not GPS + Galileo), `VehicleSafetyAndControlTask` sends a Warning LOG_TEXT line every 10 s: `GNSS configuration not as intended: configured yes/NO, GPS+Galileo yes/NO` (`SNSR::GnssConfigWarning`, host-tested). Nothing acts on it: Ready, Arm and the mode machine are unchanged. VehicleStatus carries no bit for it (the `flags` byte is full; a bit would be a schema change). In every wait state the driver drains the UART (`checkUblox`), because auto-PVT otherwise overflows the RX buffer and the first exchange after the wait fails (bench: the save timed out 3x after the 1 s wait before this). A failure log carries the SparkFun status (`ubx status 3` timeout, `4` NACK). A failed step is retried after 100 ms, up to 3 attempts, then skipped (the old code ignored every result); a failed reopen ends in Failed. The end is logged as `GNSS Module configured (<s>, longest step <ms>, <n> steps skipped)`.
- **Bench (2026-11-20, 3 boots):** configuration 1.70 s from t = 5.0 s, longest single step 61-65 ms (around `saveConfiguration`), longest vehicle-side gap between consecutive `POSE` packets 64-83 ms (at t ≈ 6.7 s), "All systems initialised" at t = 6.8 s, Ready at t = 16.8 s after the boot zero, **no Failure**. Before: about 1.3 s stall and Failure (sensorFailure) at every boot. Every boot: `QZSS off` failed once (then accepted) and **`GPS off` was rejected 3 times** within about 10 ms each (a fast refusal, not an ACK timeout), so it is skipped: see Known issues.
- **Bench (2026-11-20, `feature/bellyflop-abort-gnss`):** read-back `CFG-GNSS GPS on Galileo on GLONASS off BeiDou off SBAS off QZSS on IMES off; MON-GNSS in use GPS+Galileo (supported GPS+GLONASS+BeiDou+Galileo); nav rate 10.0 Hz (meas 100 ms x 1)`. No sky view on the bench, so fix and satellite count with GPS + Galileo are not verified.
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

`Data::Memory_Internal<512>` is a RAM image of EEPROM, managed by `Memory_Manager` (main.cpp:105-107). At boot: EEPROM → RAM. A manager version mismatch clears memory. Missing items are created with identity defaults. Then RAM → EEPROM (`initialiseMemory`). The image is saved again only by `Telecommand_Save` (any of the subsystems Gyro, Accel, Magneto; EEPROM write then read-back compare; [calibration.md](calibration.md)).

| Key | Define | Type | Content |
|---|---|---|---|
| 100 | `MEMORY_KEY_GYROCALIB` | `DSP::ValueCov<float,3>` | `val` = bias (sensor frame), `cov` = scale/misalignment 3×3 |
| 101 | `MEMORY_KEY_MAGCALIB` | `ValueCov<float,3>` | `val` = hard-iron bias, `cov` = scale. Written by `CalibApply` (mag candidate, `C = s I`). |
| 102 | `MEMORY_KEY_ACCCALIB` | `ValueCov<float,3>` | bias / scale. Written by `CalibApply` (accel candidate, `C = T A T⁻¹`). |
| 200 | `MEMORY_KEY_GYROTRANSFORM` | `Matrix<float,3,3>` | sensor → body |
| 201 | `MEMORY_KEY_MAGTRANSFORM` | `Matrix<float,3,3>` | sensor → body |
| 202 | `MEMORY_KEY_ACCTRANSFORM` | `Matrix<float,3,3>` | sensor → body |

Applied as `TopicCoordTransform::setTransform(calib.cov * transform, calib.val)` through `applySensorCalibration` (main.cpp:117, calls at :1804-1812), which also keeps a copy for the `CALIBRATION` telemetry. **The hard-coded gyro bias `(0.92, 2.0, 0.6) °/s` is gone (2026-11-21, calibration C2):** the boot gyro offset is key 100 `.val` until the automatic gyro bias run of the boot zeroing window applies the measured bias (RAM transform only, never written to the image unless a manual gyro `CalibApply`). Calibration behaviour, validity rules and the commands: [calibration.md](calibration.md). The exact formulas are in section 8a.

## 8a. Sensor calibration pipeline (what `CALIBRATION` reproduces)

Each of the three sensor streams passes one `DSP::TopicCoordTransform` before the estimators (`lib/ExVectrDSP/include/ExVectrDSP/topic_coord_transform.hpp:94-99`):

```
cal = M * (raw - b)        M = C * T  (3x3),  b = offset (3)
```

| Sensor | raw (driver output, `IMU_RAW`) | b | C | T | Consumers of `cal` |
|---|---|---|---|---|---|
| Gyro | MPU9250 gyro, rad/s | key 100 `.val` at boot, then the measured bias of the automatic run (RAM) | key 100 `.cov` | key 200 | attitude EKF |
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

`raw_frame` is `SENSOR` (0). Source bits per sensor (`*_source`): `CALIB_EEPROM` (key 10x read from EEPROM), `TRANSFORM_EEPROM` (key 20x read), `OFFSET_OVERRIDE` (no longer set since 2026-11-21: the hard-coded gyro bias is removed), `RAM_ONLY` (set by `CalibApply` and by the automatic gyro run, cleared by a successful `Save` for records whose value was written to the image), `MEMORY_CLEARED` (memory manager version mismatch at boot, everything is defaults). Both bits clear means identity defaults.

**Bench values (vehicle on the bench 2026-11-20, read with `CALIBRATION`). These are the MANUAL ROLLBACK values for the first calibration increment (calibration C2 has no rollback record: there is no tool that writes arbitrary values, so restoring means re-entering these via firmware or a future ground-station rollback):** all six items loaded from EEPROM (gyro source 0x07 with the override, accel 0x03, mag 0x03).

| Sensor | b | M |
|---|---|---|
| Gyro | `(0.01606, 0.03491, 0.01047)` rad/s (the hard-coded override of that time; the stored key 100 value, read 2026-11-21 as the boot offset after the override was removed, is `(0.00323, 0.02011, −0.00744)` rad/s) | `diag(−1, 1, −1)` |
| Accel | `(0.42849, 0.55833, 2.14256)` m/s² | `diag(−0.9984, 1.003, −0.9673)` |
| Mag | `(1.0491e-6, −1.467e-7, 8.154e-7)` T | `0.16220 · [[0,0,−1],[1,0,0],[0,−1,0]]` |

These are the values of the `if (false)` block in `initialiseMemory` for the accel (offset/scale, `T = diag(−1,1,−1)`) and gyro (`C = diag(−1,1,−1)`), and a `CalibApply` mag fit with the permutation transform in key 201. With them the bench vehicle at rest gives |accel cal| = 10.3 m/s² (raw 8.44): the accel calibration no longer fits the sensor (Known issues). On 2026-11-21 that stored offset (`b_z = 2.14`) was above the 1.5 m/s² limit and Arm/MissionBegin refused; the limit is now 2.5 m/s² (owner decision), but the live check still fails with this calibration, so recalibrate ([calibration.md](calibration.md)).

---

## 9. Telemetry: generated packets (`src/comms/telemetry_mapping.cpp`)

The packets are the generated ExVectrPackets classes (schema hash 0xBFF82392, byte layouts in the library's `docs/packet-reference.md`). Decoded values are SI; fixed-point scaling, saturation and NaN handling are done by the generated `encode()`. The old raw structs `VehicleState` (port 50) and `Net::Packet*` (ports 10-12) are removed; `MissionState` (`telemetry.hpp`) stays as the missions' internal state.

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
| 0x22 | `AIR_ACTUATORS` (two samples per packet) | – | 25 Hz | 0 |
| 0x23 | `AIR_MISSION` | – | 5 Hz | 0 |
| 0x02 | `LOG_TEXT` | as printed | – | 0 |
| – | heartbeat | 1 Hz | 1 Hz | 0xFF |

UART total about 2.0 kB/s including framing (air downlink: about 2.14 kB/s of 6.15 kB/s entry capacity, 35 % since `AIR_MISSION.setpoint_q`, see ExVectrPackets `docs/telemetry-sets.md`); USB about 18 kB/s plus log text. Bench check 2026-11-20 over the air: 24.6 `AIR_ACTUATORS`/s and 4.8 `AIR_MISSION`/s arrived at the ground station, `prev_age` 17.5-22.5 ms (mean 20.0 ms).

**Sources** (`TelemetryProvider`, main.cpp; read-only, in the telemetry task): attitude and position from the topic switches (estimators, or the body simulator in simulation mode, as everything downstream sees them), ground distance from the ground topic switch, GNSS from `gnss`, flap angles from `trueFlapSettingTopic`, TVC from `StarshipTVC::getTVCActualThrustTopic()`, raw sensors from the MPU9250, QMC5883 and BME280 topics, standard deviations from the estimators' covariances, mode/mission/failures from `VehicleSafetyAndControlTask`.

**Field mapping:**

| Packet | Field | From |
|---|---|---|
| all | `t_us` | `(NowNs() / 1000) mod 2^32` |
| `VEHICLE_STATUS` | `t_ns` | `NowNs()` |
| | `mode` | `VehicleMode`: 0 Startup, 1 SensorInit, 2 SensorCalib, 3 Ready, 4 Running, 5 Failure |
| | `mission_mode`, `mission_phase` | `MissionMode` (0 Idle … 4 Finished), `MissionPhase` (0 Idle … 7 Completed) |
| | `ctrl_source` | 0 (NONE): no RC control source until T9b |
| | `flags` | ARMED = `vehicleArmed_`; SIMULATION; MOTORS_ENABLED = `StarshipTVC` motors enabled **and** actuators enabled (the existing logic; there is no hard motor gate before T9b); GS_PRESENT = ground-station presence (section 5; information only, not the failure condition); INCOMPATIBLE; FAILSAFE = mode is Failure (actuators cut); ZEROING (bit 7) = an estimator zeroing window is active (`VehicleSafetyAndControlTask::estimatorZeroingActive()`: boot zero or `ZeroEstimators`; the mission's own Initialisation zeroing does not set it); UART_PEER_SEEN (bit 6) = a valid frame from the ship node (node 3) arrived on the UART within 3 s (`VehicleNetwork::isUartPeerSeen()`, from the UART node's node table, so USB frames and other nodes never set it). **Information only:** nothing in the vehicle reads it (safety.md 4.1). It exists because `NetworkStatus` seen over USB reflects only the USB node, so a dead UART is otherwise invisible |
| | `failure_bits` | bit 0 sensor, 1 attitude, 2 position, 3 position out of bounds, 4 attitude out of bounds, 5 radio connection loss |
| | `sensor_state` | 2 bits per sensor, `TelemetrySensor` (0 Init, 1 Calib, 2 Ready, 3 Failure): bits 0-1 GNSS, 2-3 IMU, 4-5 baro, 6-7 mag, 8-9 attitude KF, 10-11 position KF |
| | `last_req_id`, `last_req_status`, `rc_age` | 0, 0, NaN (no `REQUEST` / `RC_STATE` until T9b) |
| `POSE` | `omega`, `q`, `vel`, `pos` | attitude state `[ω, q]`, position state `[V, P]` (f32) |
| `AIR_POSE` | `q` (i16, 1/32767), `pos` (**i32, 0.001 m = 1 mm**, ±2147483 m; was i16 0.1 m until 2026-11-20), `vel` (i16, 0.01 m/s, ±327 m/s) | as `POSE`, fixed point, saturating; 30 byte payload. USB `POSE` is f32 and unchanged |
| `ACTUATORS` | `flaps` | true flap angles `[TL, TR, BL, BR]`, rad. **Modelled, not measured**: the output of the servo model in `StarshipFlaps` (300 °/s slew; in simulation also the injected faults); no position feedback exists. |
| | `tvc` | `[atan2(Fx, Fz), atan2(Fy, Fz)]` of the actual thrust vector (body frame), rad. **Modelled, not measured**: servo-space slew model (20 °/s on the thrust angle) in `StarshipTVC`, not the measured vane position. |
| | `thrust` | `|[Fx, Fy, Fz]|`, N. The roll torque `T` has no field (Known issues) |
| | `ground_distance` | ground topic switch `data(1)` (KF state `[rate, distance]`), m; NaN before the first sample |
| `AIR_ACTUATORS` | `t_us`, `prev_age` | time of the newest sample; newest minus previous sample time, s (nominally 0.020; NaN with no previous sample) |
| | `tvc[4]`, `roll_torque[2]`, `thrust[2]`, `flaps[8]` | newest first, then previous: same definitions as `ACTUATORS` (`tvc` = `[atan2(Fx,Fz), atan2(Fy,Fz)]` of the actual thrust, `thrust` = its magnitude, `flaps` = true angles `[TL,TR,BL,BR]`), plus `roll_torque` = the actual thrust topic's `T` (the twist actuation as roll torque, N·m; the vane twist fraction is `T·45/thrust`, limited to ±0.5). Fixed point: angles 0.1 mrad, torque 0.01 mN·m, thrust 1 mN, saturating, NaN -> sentinel. Samples are taken every 20 ms (`VehicleNetwork::kActuatorSampleTicks`) into `ActuatorHistory` (4 entries) and the packet is sent at 25 Hz on a sampling tick, so the previous sample is exactly the one before the newest. The values are the modelled actual output (commands after the mixer limits and the rate model), not servo feedback; the vehicle has no servo feedback |
| `AIR_MISSION` | `mission_time`, `mission_mode`, `phase`, `index`, `setpoint_pos`, `setpoint_vel`, `setpoint_q` | as `MISSION` (+ `mission_mode` as in `VEHICLE_STATUS`), fixed point: time 1 ms, position 1 mm, velocity 0.01 m/s, `setpoint_q` i16 `limit = 1.0` as `AIR_POSE.q` (raw = round(v * 32767), 0 stays exactly 0) |
| | `waypoint` | `missionWaypointTask.currentWaypointIndex()` while the waypoint mission is active, else 255 |
| `MISSION` | `setpoint_pos`, `setpoint_vel` | `positionSetpoint[3..5]`, `[0..2]` |
| | `mission_time` | `missionTime` ns -> s (1 ms wire resolution; negative in the countdown) |
| | `index`, `phase` | `MissionState.missionIndex` (always 0, see missions.md), current `MissionPhase` |
| | `setpoint_q` (also in `AIR_MISSION`) | Attitude setpoint of the **currently active attitude controller**, `[w,x,y,z]` body -> world, NWU (as `POSE.q`), unit with w >= 0; **all four 0 = no attitude setpoint**. Pure mapping `Comms::attitudeSetpoint()` (telemetry_mapping.hpp, host-tested). Selection by the controller phase table as applied (`Safety::controlPhaseFor`): (1) bellyflop set on (`bellyflopFlapsActive()`, freefall and the bellyflop after its kick) **and** the flap setting on `bellyFlopControlTopic` is `BellyFlopMode_Stabilize` (FlapModeObserverTask: vehicle and mission Running) -> flap controller; (2) else TVC set on (`tvcControlActive()`, waypoint/RTH Running with actuators, and the bellyflop kick) -> during the kick (`MissionBellyflop::kickActive()`, input `tvcOverride`) the kick target the mission writes into the controller (`kickSetpoint()`), otherwise the TVC attitude controller's setpoint topic `controlMappingAccToAtt.getAttitudeTopic()` (the topic `controlAttitudeTvc.subscribeAttitudeSetpoint` is wired to, main.cpp setup), q part `[3..6]`, normalised. The phase table makes (1) and (2) exclusive since 2026-11-21; (3) else zero (idle, ground, countdown, Finished, RTH stabilisation with flaps Upright). The flap controller (ControlAttitudeFlaps, Stabilize) has no quaternion setpoint: it tracks only the direction of body +Z (`azimuthAngle_Rad` = atan2(z_W, z_N), `pitchAngle_Rad` = elevation of body +Z), roll about body +Z is uncontrolled. The published q is the current attitude rotated by the shortest world-frame rotation that puts body +Z on that direction (`flapPointingSetpoint`), so its roll follows the real vehicle. Non-finite or near-zero inputs give 0. Read through extra `Simple_Subscriber`s on the same topics and two const getters: no controller input, subscription or timing changes. Bench (simulation, full chain, 2026-11-21, `tools/bench_vehicle_usb.py`): 0 in Startup, Ready, countdown (Init, Startup) and after the RTH landing; unit (norm 1 +- 6e-5) in waypoint hover/ascent, bellyflop, freefall and RTH; control phase log lines, bellyflop time-limit end (1.01 s) and RTH auto-disarm as before. Bench 2026-11-21 (bellyflop kick, `feature/bellyflop-kick`): the kick target `[0.707, -0.001, -0.707, -0.001]` for the whole kick, then the flap setpoint; unit in every bellyflop and freefall sample after the `flapPointingSetpoint` fix (Known issues). |
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

### Command list (handler: `VehicleSafetyAndControlTask::handleTelecommands`, main.cpp:603-902)

Only the newest telecommand per 100 ms tick is processed (`Simple_Subscriber`).

| # | Type | Magic / params | Precondition | Implemented behaviour |
|---|---|---|---|---|
| 0 | `CalibStart` | `subsystem` 0 gyro, 1 accel, 2 mag; `paramInt` 1 (or > 2) calibrate, 2 verify, 0 or ≤ −2 stop and fit, −1 discard | initialised, disarmed, motors off, mode Startup/Ready/SensorCalib, mission Idle/Finished, **not simulation mode** (else refused, logged 1/s) | Starts the `CalibrationService` session (accel 12-pose fit, mag sphere fit, gyro run), TVC actuators off, mode `SensorCalib`, aborts a zeroing window and clears the valid zero. When no session is collecting or fitting the mode returns to `Startup` (automatic zero). [calibration.md](calibration.md). |
| 1 | `CalibApply` | `subsystem` 0 / 1 / 2 | as `CalibStart`, and a candidate with verdict **PASS or WARN** | Writes the legacy key (accel `C = T A T⁻¹`, mag `C = s I`, gyro bias) to the RAM image, applies the transform (RAM only until Save), invalidates the estimator zero (Ready → Startup). |
| 2 | `Load` | — | — | **Not handled.** |
| 3 | `Save` | `subsystem` 0 / 1 / 2 | as `CalibStart`, no sensor collecting | EEPROM ← RAM image (all keys), then read back and compared (`saved ... verified` / `EEPROM save FAILED`). |
| 4 | `SystemReset` | `paramInt == 0xC5` | none | **System state only (since 2026-11-20):** mode `Startup`, re-init (wait for all inputs), actuators/motors/flaps off, **whole mission chain reset to the boot state** (`resetMissionsToBootState()`: all missions Idle, RTH active, no pending next mission, never continued), `clearFailures()`, **disarm**, aborts an active zeroing window. Then `Ready` as soon as the inputs are back, without a new zero, if a valid zero exists since boot. It does **not** zero or re-reference the estimators or reset the body simulator (use `ZeroEstimators`), and does **not** change simulation mode (it stays on or off as it was). |
| 5 | `Arm` | `paramInt == 0xA5` arms; **any other value disarms** | `0xA5`: no zeroing window active (else refused, rate-limited log) **and all three calibrations valid** (not required in simulation mode; refused with the reason, log every 10 s) | Sets `vehicleArmed_`. Auto-disarm on Failure, SensorCalib, Startup or SensorInit, after RTH finishes, and in mode Ready if the calibration check fails (real mode). Disarm is always accepted. |
| 6 | `SimulationMode` | `paramInt == 0xA9` | none | **Toggles.** On: motors off, topic switches → `bodySimulator`, simulator reset and unpaused. Off: switches → estimators, simulator paused, mode `Startup`, re-init, actuators off. |
| 7 | `MissionBegin` | `paramInt` = initial mission time in s (use ≤ −10 for the full countdown) | mode `Ready`, no zeroing window active (else refused, rate-limited log), **all three calibrations valid and `paramInt <= −18`** (not required in simulation mode; refused with the reason, log every 10 s) | `switchMissionTo(waypoint, paramInt·1 s)`, mode `Running`, motors enabled unless in simulation. **Arming not required.** |
| 8 | `MissionSelect` | — | mode `Ready` | No-op (log only). |
| 9 | `MissionAbort` | none | mode `Running` (else ignored) | **Abort = stop and return home** (user decision 2026-11-20, `Safety::decideAbort`, missions.md 6.2). In flight (waypoint Running or Finished, bellyflop, freefall; by mission mode only, no altitude/speed check): RTH hot start at once; RTH already running: kept. On the ground = before liftoff (waypoint Init/Startup, any Idle mission, RTH landed): TVC actuators and motors off, chain reset, reset mission (Idle), clear failures → Startup → Ready, nothing restarts. The old "only valid in flight" note is replaced by this rule. |
| 10 | `ZeroEstimators` | `paramInt == 0x2E`; `paramFloat` = window in s (NaN or 0: 10 s; clamped to 1..60 s) | initialised (all inputs published), disarmed, motors off, mode `Startup` or `Ready`, active mission `Idle` or `Finished`. Otherwise ignored, counted, rate-limited log (1/s) | Starts the zeroing window (overview.md, "Estimator zeroing"); a second one during the window restarts it with the new duration. One-shot ground action, timed on the vehicle. **The vehicle must be still.** |
| 11 | `SimWindSpeed` | `paramInt == 0x57`; `paramFloat` = wind speed in m/s (finite; clamped to 0..20) | **simulation mode only** (else refused, logged with the reason; also refused for a bad magic or a non-finite value) | Sets the steady wind of the body simulator (gusts: 13..15; only the simulated air changes). The wind is reset to 0 whenever `SimulationMode` switches on or off; `SystemReset` keeps it. Owner request 2026-11-21 (`feature/sim-aero`). |
| 12 | `SimWindDirection` | `paramInt == 0x57`; `paramFloat` = direction the air moves **toward**, rad, counterclockwise from North toward West (NWU: 0 = toward North, π/2 = toward West; wrapped to −π..π) | as 11 | Sets the wind direction (speed unchanged). Send 11 and 12 at least 100 ms apart: only the newest telecommand per 100 ms tick is handled. |
| 13 | `SimGustStrength` | `paramInt == 0x57`; `paramFloat` = peak gust speed (bound of the extra wind speed along the mean direction) in m/s (finite; clamped to 0..10; 0 = no gusts) | as 11 | Sets the gust strength (period and seed unchanged). Owner-approved simulation-only exception to safety.md rule 7 (2026-11-21). Reset to 0 with the wind when `SimulationMode` switches. |
| 14 | `SimGustPeriod` | `paramInt == 0x57`; `paramFloat` = gust timescale in s (finite; clamped to 0.5..30) | as 11 | Sets the gust period (default 5 s). |
| 15 | `SimGustSeed` | `paramInt == 0x57`; `paramFloat` = integer-valued seed 0..16777215 (finite; rounded to the nearest integer, clamped; exact in a float) | as 11 | Sets the gust seed (default 1). Same seed and settings give an identical gust series (model: ExVectrDSP.md). |
| 16 | `SimFlapStuck` | `paramInt == 0x57`; `subsystem` = flap 0..3 (0 TL, 1 TR, 2 BL, 3 BR, the telemetry ACTUATORS flaps order), **no other bits** (bit 4 `0x10`, the removed "arm for the descent" mode, is refused since 2026-11-21); `paramFloat` = stuck angle in rad (finite; clamped 0..π/2), **−1.0 = hold the true angle it has when the fault is applied** | **simulation mode only** (else refused and logged with the reason; also refused for a bad magic, bad subsystem bits or a non-finite value) | **Immediately** (next flap task tick, 20 ms; flaps enabled or not) the flap's modelled TRUE angle is forced to that angle whatever the controller commands, at the one publication point of the true flap topic, so the body simulator, ACTUATORS / AIR_ACTUATORS telemetry and the ground station's FlapState all show it. The servo PWM output still follows the commanded angle. Log `Sim fault: flap TL STUCK at 42.0 deg (now)`. Owner request 2026-11-21 (redesign the same day: the arm-for-descent mode is gone; send the command when the fault should start). |
| 17 | `SimFlapSlow` | `paramInt == 0x57`; `subsystem` = flap index (0..3, no other bits); `paramFloat` = servo rate factor (finite; clamped 0.05..1.0, 1 = normal) | as 16 | Scales the modelled servo slew rate (300 deg/s) of that flap from then on. Effective at once. |
| 18 | `SimFaultClear` | `paramInt == 0x57`; subsystem and `paramFloat` ignored | as 16 (magic and simulation mode) | Clears every flap fault (a stuck flap snaps back to its commanded angle). Faults are also cleared on every `SimulationMode` switch (on and off) and by `SystemReset` (review 2026-11-21). |

**Encoding of 11..18** (8 bytes, little-endian, subsystem 0 = unused; `paramInt` 0x57 = bytes `57 00`; send at least 100 ms apart, only the newest command per 100 ms tick is handled). Examples:

| Command | Hex bytes |
|---|---|
| 11 wind 5.0 m/s | `0B 00 57 00 00 00 A0 40` |
| 12 direction π/2 (toward West) | `0C 00 57 00 DB 0F C9 3F` |
| 13 gust strength 3.0 m/s | `0D 00 57 00 00 00 40 40` |
| 14 gust period 4.0 s | `0E 00 57 00 00 00 80 40` |
| 15 gust seed 42 | `0F 00 57 00 00 00 28 42` |
| 16 TL stuck at 42° now (0.7330 rad) | `10 00 57 00 66 A8 3B 3F` |
| 16 BR stuck at its current angle (subsystem 0x03, value −1.0) | `10 03 57 00 00 00 80 BF` |
| 17 BL servo rate factor 0.25 | `11 02 57 00 00 00 80 3E` |
| 18 clear all faults | `12 00 57 00 00 00 00 00` |

There is no authentication beyond the three magic values and the source address 0 (which any node on a link can claim). The vehicle subscribes to no other port: telemetry ports are send-only now.

---

## Body frame and actuator layout

Purpose: everything a 3D model or ground-station view needs to place the body axes, flaps, TVC vanes and motors. Each statement is tagged **[code]** (derivable from code, cited) or **[sim]** (only from the simulator's geometry, which is a model, not a measurement) or **UNKNOWN** (needs the user). Line numbers refer to the pinned sources at the time of writing (ExVectrControl `084bea1`, ExVectrSensor `6bd4b01`).

### Body frame

- **[code]** Right-handed, **+Z = long axis toward the nose**, TVC/motors at the aft end at body `(0,0,−0.35)` m (`src/main.cpp:1967`, `lib/ExVectrDSP/src/body_simulator.cpp:34`). Flap pair centroids (owner geometry 2026-11-21, simulator `BodySimModel`, origin at the CoM 0.37 m above the bottom): top z = +0.220 m (span +0.16..+0.30), bottom z = −0.264 m (span −0.36..−0.15; bottom flaps 1 cm above the bottom, corrected 2026-11-21); top flaps canted 5° inward about the lower hinge point (centroid 5 mm closer to the axis); the old simulator used ±0.26 m.
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

| Flap | PWM pin (`src/starship_hardware.h:15-18`) | Hinge line, area centroid height (m) [owner geometry, sim] | Servo value (`starship_flaps.cpp:158-165`) |
|---|---|---|---|
| TL | 25 (`FLAP_SERVO_PIN_UL`) | `y = +0.065`, `z = +0.220` (7 × 14 cm, 8 cm chamfer, 82 cm²) | `1 − angle/(π/2)` |
| TR | 24 (`FLAP_SERVO_PIN_UR`) | `y = −0.065`, `z = +0.220` | `angle/(π/2)` |
| BL | 29 (`FLAP_SERVO_PIN_DL`) | `y = +0.065`, `z = −0.264` (8 × 21 cm, 8 cm chamfer, 152 cm²) | `angle/(π/2)` |
| BR | 28 (`FLAP_SERVO_PIN_DR`) | `y = −0.065`, `z = −0.264` | `1 − angle/(π/2)` |

- **Hinge axis [sim]:** all four hinge axes are **parallel to body Z** (the long axis). The simulator rotates each flap about Z by `∓angle` (TL, BL: `−angle`; TR, BR: `+angle`) (`body_sim_model.cpp`, `addFlap`), hinge line on the tube surface (r = 0.065 m), the plate extending radially; the chordwise centre of pressure moves from 1/4 to 1/2 of the width with the flap angle of attack. At angle 0 the flap points straight out sideways (±Y, away from the body). At 90° it has swung to point toward body **+X (the back/top of the vehicle, away from the belly)** on all four. Owner 2026-11-21: hinges parallel to the axis, the fully extended flaps lie in a plane through the axis (not swept back); dimensions as in the table (workspace `docs/system-architecture.md`, owner update 2026-11-21: bottom flaps start 1 cm above the bottom (corrected 2026-11-21, was 10 cm), top flaps' lower edge 53 cm and canted 5° inward about their lower hinge point, `topFlapCantRad`). The owner wrote "the body X-Z plane" for the extended flaps; with the confirmed frame (flaps on ±Y, belly −X) the extended flap lies in the **Y-Z** plane, which the model uses (open question to the owner).
- **Angle meaning [code]:** radians. **0° = fully extended**, **90° = retracted/folded** against the body. Evidence: ascent without the extend command commands 90° on all four, `Upright` commands top 0° and bottom 90° (`control_attitude_flaps.cpp:174-189`). So during bellyflop the flaps are extended into the airflow.
- **Sign convention [code]:** one commanded angle per flap, same sense for the left and right flap of a pair (mirroring is done in the servo mapping above). Larger angle = more folded.
- **Clamp [code]:** the driver limits every angle to **[`FLAP_MIN_ANGLE` = 15°, `FLAP_MAX_ANGLE` = 90°]** (`starship_flaps.hpp:36-37`, `:93-99`). 0° is never commanded in flight. The `Upright` mode's 0° becomes 15°.
- **Servo [code]:** STANDARD PWM, value 0..1 maps to 1–2 ms, 90° of travel mapped over the full range; trim offsets all zero (`starship_flaps.cpp:146-150`). Slew limited to 300°/s in the simulated "true" position (since 2026-11-21; was 200°/s).
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
| Fixed 2026-11-20 | `ublox_gnss.cpp` `performConfigAction`, `gnss_config_sequencer.*` | (was High) The code asked for "Galileo only" with per-constellation toggles ending in "GPS off", which the receiver refused at every boot, so "Galileo only" was never in effect. Now GPS + Galileo (user decision) in one CFG-GNSS write plus a read-back log; QZSS stays on with GPS (the receiver NACKs QZSS off while GPS is on). | Sky-view check of fix and satellite count still open. |
| High | main.cpp:1233-1237, `starship_tvc.cpp:247-250` | Arming does not gate the motors. Disarmed only sets `motorPowerLimit = 0.1` (+0.04 idle). | Motors spin on a disarmed vehicle whenever `enableMotors_` and the actuators are enabled. |
| Medium | `starship_tvc.cpp:27`, `:36` | TVC output is event-driven on controller publications only. | If the controller stops publishing, the servo, motor, enable and test state freezes at the last values. Motors keep their last command. |
| Medium | `starship_tvc.cpp:191-198` | Trim (up to 5°/45° = 0.11), roll twist (±0.5) and full deflection (±1) are summed with no prioritisation. PWM clips at 0/1. | Pitch/yaw authority is lost asymmetrically under roll demand. Saturation is not reported to the controller (the "actual" topic does not model it). |
| Medium | `starship_tvc.cpp:116-119` vs `:76-92` | Vane angles use the unclamped force direction, while the "actual" topic uses the 3°-cone-clamped vector. The rate-limit model (20°/s of thrust angle = 300°/s of servo) is not applied to the real servos. | Simulator and telemetry thrust direction can differ from what the vanes do near the limits. |
| Medium | `ublox_gnss.cpp:23-31`, `:37` | `positionCov` and `velocityCov` hold 1σ accuracies (m, m/s), not variances. Validity is only `numSats ≥ 5` with the fix-OK check disabled. | Wrong KF weighting unless the consumer knows it gets σ. A 2D or invalid fix with 5 satellites is accepted as valid. |
| Medium | `vl53l0x_driver.cpp:834-856` | Range status is not checked. `cov = 0.15·range` is σ-like, not m². | Out-of-range readings (~8.19 m) are published. The KF window 0.4–1.5 m (main.cpp:1932) masks this today. |
| Medium | `vl53l0x_driver.cpp:72-73`, main.cpp:1732 | The VL53L0X sets its I2C device to 400 kHz on the shared `Wire`, while main sets 100 kHz. | Whether the bus clock changes per transaction depends on the HAL. The BME280, QMC5883 and OLED may run at 400 kHz. |
| Medium | `PWM_Output` (lib) used by both drivers | `outputEnabled_` is never assigned in `enableOutput()` (`lib/ExVectrActuator/src/pwm_output.cpp:103-114`). Disable writes 0 duty once, but the next `setValue()` drives the pin again. | The drivers only work because they call `setValue` exclusively while enabled. Any future `setValue` on a disabled output re-energises it. |
| Fixed 2026-11-20 | `ublox_gnss.cpp` (was `:199`) | (was High) `delay(1000)` and the whole blocking configuration in `taskInit` stalled the scheduler for about 1.3 s at t ≈ 5 s, so the vehicle latched Failure (sensorFailure) at every boot. Replaced by the non-blocking `GnssConfigSequencer` (section 6). Bench: Ready without Failure, longest POSE gap 64-83 ms. | |
| Low | `ublox_gnss.cpp` `performConfigAction` (`Begin`) | Residual blocking: one `begin()` attempt can still block up to 3 x 250 ms (the SparkFun `isConnected` retries) when the receiver does not answer at that baud (first-ever setup, wrong baud, receiver missing). | Only before the first GNSS data, so no sensor-age check runs yet (initialisation waits for GNSS), but the estimators are starved for up to 0.75 s per baud attempt in that case. Normal boots (receiver saved at 115200) do not hit it. |
| Low | `lib/ExVectrSensor/src/mpu9250.cpp` (`WriteRegister` 2 x 10 ms per register write, AK8963 waits of 100 ms), `qmc5883.cpp:164` (10 ms), `src/vl53l0x/vl53l0x_driver.cpp` (init loops without timeout, `io_timeout = 0`) | Other blocking waits in the start-up path (checked 2026-11-20). All run in the drivers' `taskInit` at t ≈ 0, before the estimators publish and long before the first sensor-age check, so they cannot latch sensorFailure. `MPU9250::WriteRegister` delays 20 ms per call; it is also used by the AK8963 read path, but the mag is disabled on this vehicle (`MPU9250Driver(spiMPU, true)`). | Slower boot only. A missing VL53L0X could hang its init loop (not observed). Not changed. |
| Low | `starship_flaps.hpp:37` | `FLAP_MIN_ANGLE = 15°` clamps the controller's 0° (full extension, used by `Upright`). | Flaps never fully extend. The true-flap topic shows 15°. |
| Fixed 2026-11-21 | `initialiseMemory` | (was Low) Gyro bias hard-coded at boot overrode the stored calibration. | Removed: the boot offset is key 100 until the automatic gyro run applies the measured bias ([calibration.md](calibration.md)). |
| Medium | EEPROM key 102 (bench vehicle) | The stored accel calibration (`b = (0.428, 0.558, 2.143)` m/s², `C·T = diag(−0.9984, 1.003, −0.9673)`) maps the resting raw accel (\|raw\| 8.44 m/s²) to \|cal\| 10.3 m/s², not 9.81. The z offset no longer matches the sensor (it may have shifted, or the values were fitted in another frame). | The tilt estimate can be off by a few degrees (the x/y offset error is not observable at rest in one orientation). Needs a new accel calibration (six-position) before flight; there is no accel calibration command. | **Since 2026-11-21 the calibration check refuses Arm and MissionBegin until the accelerometer is recalibrated** (first because the offset was above 1.5 m/s², now because the live `|a_cal|` check fails; the offset limit is 2.5 m/s²) ([calibration.md](calibration.md)).
| Medium | `lib/ExVectrSensor/src/qmc5883.cpp:99-101`, `:170` | The QMC5883 is configured for the 8 G range (3000 LSB/G per datasheet) but converted with 8/32767 G/LSB (4096 LSB/G). | `IMU_RAW.mag` is 0.73x the physical field (0.35 G read for about 0.48 G). Direction and the existing mag calibration are unaffected; do not "fix" it without recalibrating the mag (the stored offset is in these units). |
| Medium | attitude EKF (`lib/ExVectrDSP`), seen in `ESTIMATOR` | The EKF quaternion covariance does not converge on the bench: `att_std` stays at about 0.3-1 rad tilt and several rad heading. The mag correction is a fixed-gain complementary step that never reduces `P`, and the accel update uses the driver variance × 5000. | `att_std` is not a usable confidence today, although the attitude itself is consistent with the calibrated accel and mag (2026-11-20 bench check: accel to world up within 1.6°, mag heading error 1.5°). Estimator change: needs a decision and simulation verification. |
| Low | main.cpp `systemsInitialised` | "GNSS data not new" (and the other "... not new" lines) are logged every 100 ms until all sensors deliver. | About 10 `LOG_TEXT` lines/s on the bench without GNSS. Not rate-limited (only the auto-disarm messages are, 2026-11-20). |
| Low | `starship_hardware.h:25-26`, `starship_tvc.hpp:92-94` | `TVC_SERVO_LIMIT` is unparenthesised. `TVC_ANGLE_FACTOR` is duplicated by the hard default `tvcFactor = 15`. | Changing the define does not change the TVC mixing. Macro expansion hazard. |
| Low | `telecommand.hpp` | Header comments disagree with the code: CalibStart duration, MissionBegin "seconds in the future". (The SystemReset comment was corrected 2026-11-20: it never changes simulation mode, and it resets the mission chain to the boot state.) | Operator and ground-software confusion. |
| Medium | main.cpp `handleTelecommands`, section 10 | Only the newest telecommand per 100 ms tick is handled (`Simple_Subscriber`). Seen on the bench 2026-11-20: a `ZeroEstimators` sent within the same tick as a `SystemReset` from another sender was lost. | Two senders (or a fast sequence) can silently drop a command. Senders must space commands > 100 ms (the ground station enforces 500 ms). T9b replaces the path. |
| Info | `starship_hardware.h` `RX_MODULE_SERIALPORT` | Resolved 2026-11-20: the ship node is wired to Serial4 (pin 16 RX4 / pin 17 TX4), confirmed end to end (Router shows node 1, vehicle telemetry arrives over the radio). It was first on pads 30/31 and was moved. | None. If the harness is rewired, re-check with `tools/uart_pin_scan.py` (section 5). |
| Medium | main.cpp `checkForFailures`, section 5 | **Until T9b:** `radioConnectionLoss` counts any DATA frame from node 0 or 2 on the UART as radio activity (normally the Pocket's `RC_STATE`). There is no control-source arbitration and no `RC_STATE` stale failsafe yet. Fixed 2026-11-20: the former rule (node 0 frames on USB or UART) failed 2 s after boot over the radio alone. | A ground-station telecommand over the radio also keeps the link alive. T9b adds the control-source timeout (safety.md 4.1). The 2 s timeout itself was verified on the host only (the Pocket cannot be switched off from the bench). |
| Medium | section 10 | **TEMPORARY until T9b:** the legacy telecommand on port 0x7F is accepted from any frame claiming source 0 on either link, with only the three magic values as protection (as before on port 1000). It is not in the ExVectrPackets schema. | Same exposure as before; any device on USB or behind the RX module can command the vehicle. |
| Low | section 9 | `ACTUATORS` has no field for the TVC roll torque `T`. | Roll torque is no longer telemetered (it was `VehicleState.tvcTorque`, mis-scaled). Needs a schema field. |
| Low | `ExVectrNetwork` (library) | The legacy SX1280 sources of the library are compiled (unused, dropped at link time). | Library warnings in the build log. |
| Info | altitude resolution (`POSE.pos[2]`, `AIR_POSE.pos[2]`) | The telemetry itself no longer limits it: `POSE` is f32 (sub-mm at a few m) and `AIR_POSE` is 1 mm since 2026-11-20 (was 0.1 m). What remains: BME280 pressure at 16x oversampling has about 0.2 Pa noise (about 1.5-2 cm of altitude), IIR filter setting 1, and the f32 pressure (Q24.8 compensation output, resolution 0.0078 Pa near 100 kPa, about 0.7 mm) and `calcAltitudeFromPressure` in f32 (about 0.5 mm per ULP of p/p0). The position KF fuses baro, GNSS and range with large baro covariance (`cov*1000`), and the reference is a 10 % low-pass of the baro during zeroing, so the altitude is smoothed, not quantised. No casts to integer on the vehicle before the packet. | Expect centimetre-level noise and slow drift (weather, temperature), not steps. Estimator not changed. |
| Info | `board_v_1_0.h` | BME280 SPI CS, RGB LED, battery ADC are defined but unused (QMC INT pin 16 is now RX4). There is no battery monitoring. | No low-voltage protection. |
| Low | `src/comms/telemetry_mapping.cpp` `attitudeSetpoint()` | `setpoint_q` in the waypoint/RTH TVC phases is the latest value of the acc-to-att mapping topic, not `ControlAttitudeTvc`'s internal `stateSetpoint_` (no getter in ExVectrControl). They differ only between mapping updates, where the controller propagates its setpoint with the measured body rate. During the bellyflop kick the published value is the mission's kick target (`tvcOverride`), which the controller propagates the same way for ≤ 20 ms between mission writes. | Telemetry display only (ground station drawing); no effect on control. |
| Fixed (2026-11-21) | `src/comms/telemetry_mapping.cpp` `flapPointingSetpoint()` | `setpoint_q` read 0 whenever the flap correction exceeded about 151° (the unnormalised half-angle quaternion, norm `sqrt(2(1+d))`, fell under `sanitizeSetpointQuat`'s 0.5 "invalid" threshold); bench: 0 for about 1 s at the start of freefall. The correction is now normalised directly; regression test in `test_flap_pointing_setpoint`. | Telemetry display only. |
