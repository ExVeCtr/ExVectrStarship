# Hardware and Drivers

Pinout, vehicle constants and the application-level drivers in `src/` (TVC, flaps, ExVectrNetwork links, u-blox GNSS, VL53L0X), plus memory keys, telemetry and telecommand wire formats.

Last verified against commit: b073d07 (section 9 `AIR_GNSS`, `AIR_GNSS_INFO`, `home_epoch`, `calib_state`, `roll_torque`, `MISSION.mission_mode`/`waypoint` on `feature/schema-integration`, 2026-11-22; section 9 `MISSION`/`AIR_MISSION` `setpoint_q` rows on `feature/setpoint-attitude`, 2026-11-21; sections 1, 5, 9, 10 and the Known issues re-verified on `feature/network-migration`, 2026-11-19)

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
| Motors | **`evaluateMotorGate`** (`src/safety/motor_interlock.hpp`): hard interlock open (T9b-2) **and** not forced off by the watchdog **and** `enableMotors_ && enableActuators_ && test idle`. A closed gate never calls `motor*_.setValue()` (the `PWM_Output` bug, Known issues in the library doc) and calls `enableOutput(false)` (duty 0) on every run. See "Hard motor interlock (T9b-2)" in section 5. |
| Motors, first start | Additionally 2 s after the first enable since boot (`.cpp:268`). `firstMotorEnable_` is never reset, so later enables have no delay. |
| `motorPowerLimit_` | Default 0.05. **T9b-4:** the legacy arm state and its 10 % limit are removed; `main.cpp` sets the limit to **1.0 once at boot**, so with the interlock open the motors run at full power. The motors need the control system (`enableMotors_`, `enableActuators_`), and the interlock (SD) open; there is no simulation condition anywhere on that path. |

**Actuator test (`beginActuatorTest(offset)`):** starts at `NOW + offset` and hijacks the servos regardless of enable. For `TVC_TEST_SPIN_TIME = 2 s` it traces a circle: `x = sin`, `y = cos` at 45° amplitude, 2 revolutions. For the next `TVC_TEST_TWIST_TIME = 2 s` it applies roll-only `twist = sin(...)` at amplitude 1.0 (unclamped, ±45°), 2 cycles. Motors are held off throughout. It is triggered at every mission `Startup` edge (main.cpp:663).

**"Actual" thrust model** (only when actuators are enabled and the test is idle; **independent of the interlock gate** and of `enableMotors_`: design D16 stays REJECTED, owner decision 2026-11-21, so the body simulator consumes the commanded thrust whatever the gate does): direction = clamped `tvcVector`, magnitude = mean motor command × 14 N. The modelled thrust angles are rate-limited to the vane servo rate divided by the servo-to-thrust gain: 300°/s / `tvcFactor_` (15) = **20°/s**, so a full ±3° reversal takes 0.3 s (since 2026-11-21, `feature/sim-aero`, review F6; before: 600°/s in thrust-angle space = 9000°/s of servo). A non-finite command holds the previous angles and reports 0 thrust for that tick, so NaN can no longer latch into the integrated angles (it stayed NaN until reboot). `T = twist·|F|/45`, i.e. the roll torque reconstructed after clamping; `T` is zeroed when the actuators are disabled (review F13). The header calls `T` an "angle in radians".

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

**Servo PWM stage (2026-11-22, owner decision):** the flap angle that is converted to PWM goes through `Safety::ServoOutputStage::servoCommandRad(faults, flap, clampedCommand, simulationNow, dt, normalRate, 15 deg, 90 deg)` (`src/safety/sim_flap_fault.*`), called once per flap per tick in `StarshipFlaps::taskThread` after `publishAngles`, the single place where flap angles become PWM. It returns the command unchanged (bit-identical, host-tested) unless simulation mode is on at THAT call (`simulationActive()` is read on every call, never cached) and a fault is active: stuck, the held angle clamped to the servo range; slow, the output slews from its previous value at 300 deg/s x factor; after a clear it slews back at the normal rate (no jump) and then returns the exact command. Why safe: real mode never reads the fault state; faults can only be set in simulation (decision, `set` re-check), are cleared on every simulation switch and SystemReset; the motors/TVC are not touched; flap outputs are only driven while the flaps are enabled, as before (`reset()` when disabled). Verification: `pio test -e native -f test_sim_flap_fault`; physical check by the owner on the simulation bench.

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

**General link health (information only) and the control failsafe (T9b-3, `feature/t9b-3`):**
- **General link health never decides anything** (safety.md 4.1): `VehicleNetwork::isRadioPacketRecent()` is true while a valid DATA frame from node 0 or node 2 arrived on the UART within 2 s (`RadioTap`: heartbeats of any node, node 3 frames and USB never count; no boot grace). It feeds only the OLED link indicator and the connection counter. `isRadioLinkAlive()`, `kRadioStartupGraceNs` and the 10 s boot grace are **removed**. Ground-station presence (`isGroundStationPresent()`, any valid node 0 frame within 2 s) is information only (`GS_PRESENT`); the ground station is never a failure condition.
- **The control failsafe is one rule** (`VehicleNetwork::controlSourceLost(committed)`, on a seqlock snapshot of the `ControlSourceTable`, pure logic `ControlSourceSnapshot::failsafeDue`): while **committed**, the accepted `RC_STATE` of the latched control source is older than **2 s** (or there is no source) -> failure bit 5 `CONTROL_SOURCE_LOST` (renamed from `radioConnectionLoss`; `FailureState::controlSourceLost`), mode Failure, `vehicleShutdownControl(true)`: actuators cut, the vehicle falls. Latched until `SystemReset`. **Committed** = the motor interlock is open, or a mission chain is pending, or the mode is Running (`VehicleSafetyAndControlTask::isCommitted()`, the same expression that latches the source via `setCommitted()`), evaluated at the moment of the check.
- **Only accepted `RC_STATE` keeps it alive:** heartbeats, `LINK_STATS`, a GS `REQUEST` or legacy telecommand over the radio, RC frames that are rejected (`SOURCE_OK` clear, node 0 on the UART, stale sequence, bad length) and the ignored (non-latched) source never refresh it. Pocket off with the link still up (heartbeats only) therefore fails after 2 s. The check runs in the 10 Hz safety task, so it fires 2.0 to 2.1 s after the last accepted state; the interlock (1 s) closes the motors earlier.
- **D2-B (not committed: on the ground, disarmed, nothing pending):** losing every source only sets `ctrl_source` NONE; no failure, nothing latched. No boot grace is needed: nothing is committed at boot. A mission started with no control source at all (committed with source NONE) is a failsafe at once; a go needs an active source by construction (it comes from its state).
- **The SE reset** (short press, T9b-4; was the `SystemReset` telecommand) clears the failure (`clearFailures()`); with the Pocket still absent it latches again only if the vehicle is committed again (after a reset it is not). **Simulation mode:** the same rule, no simulation flag is read (safety.md rule 7); the bench source is the GS as BENCH.
- **Degraded (information only, clears by itself):** `VEHICLE_STATUS.degraded_bits` bit 2 `UART_HEARTBEAT_LOST` is set while RADIO is fresh and no valid frame from the ship node (3) arrived on the UART for 3 s (`!isUartPeerSeen()`; any node 3 frame counts, so it is "ship node silent"). Bit 0 `TOF_LOST` and bit 1 `LINK_STATS_LOST` stay 0 until T9b-6. Degraded never triggers a failsafe, abort or motor cut.

**Control sources and RC ingress (T9b-1, `feature/t9b-1`; the control failsafe above acts on it since T9b-3, the commands since T9b-4):** the vehicle decodes `RC_STATE` (port 0x10) itself, per link. The interlock (T9b-2), the control failsafe (T9b-3) and the safety commands (T9b-4, "RC command path" below) act on it.
- **`Comms::RcIngressTap`** (`src/comms/rc_ingress_tap.*`): one per link, between the `StreamFrameLink` and its `Node` (USB link -> tap -> USB node; UART link -> RC tap -> `RadioTap` -> UART node). It knows its link by construction (the shared `PortMux` cannot: `RxInfo.ingressLink` is 0 for both nodes). It looks only at DATA frames for this vehicle (dst 1 or broadcast) on port 0x10. A payload that is not exactly 34 bytes, or does not decode, is rejected (`BadLength`). Every frame is forwarded unchanged, so node presence and the radio counters behave as before. Port 0x10 is not subscribed on the `PortMux`, so the shared sequence state is never touched by RC (an `RC_STATE` of node 0 on the UART cannot make the bench copy on USB look like a duplicate).
- **`Safety::ControlSourceTable`** (`src/safety/control_source.*`, pure, host-tested): two sources only, **RADIO = (node 2, UART)** and **BENCH = (node 0, USB)**. Accepted means: key in the table, sequence newer per source (the transport rule: `(seq - last) mod 256` in 1..127, or the source was silent for 1 s), `SOURCE_OK` set. Rejects are counted per reason (`[0]` bad length or undecodable, `[1]` (src, link) not in the table, `[2]` stale or repeated sequence, `[3]` `SOURCE_OK` clear) and in a total (`VEHICLE_STATUS.rc_rejected`, wrapping u8). Heartbeats and other ports are never RC data. Fresh = last accepted state at most **2 s** old. Arbitration (`update()`, run every comms tick from `VehicleNetwork::pollNodes()`): not committed -> RADIO while fresh, else BENCH while fresh, else NONE; a handover back to RADIO from BENCH needs **0.5 s of continuous radio states** (a gap over 250 ms restarts the run) unless BENCH is itself stale. The `committed` input (interlock open, mission pending or Running) latches the active source (`setCommitted()`, set by the safety task every run since T9b-2). `failsafeDue()` is the control failsafe rule and is wired since T9b-3 (see above). Every source change writes one console line (`Control source: RADIO|BENCH|NONE`) and resets the mapper's safe-seen bits. A seqlock `snapshot()` gives other tasks a consistent copy (the 64-bit timestamps are not written atomically on the 32-bit Teensy); the comms task writes; the safety task (failsafe) and the interlock read through it.
- **`Safety::RcCommandMapper`** (`src/safety/rc_mapper.*`, pure, host-tested; the edge events and the command-intent layer are the "RC command path (T9b-4)" section below). Decode part: channels 1-based **5 SA, 6 SB, 7 SC, 8 SD, 9 SE** (design 6.1). SA, SD, SE: v <= -500 -> 0, v >= 500 -> 1, else invalid. SB, SC: v <= -500 -> 0, -200..200 -> 1, v >= 500 -> 2, else invalid. `CH_INVALID` and any value outside -1000..1000 are invalid. For action purposes invalid is the safe value (NOT_GO, DISABLED, RELEASED). It also keeps the four **safe-seen bits** (GO, SC_REAL, SD_ENABLE, SE_PRESS): set by an accepted state showing the safe value explicitly (a valid position 0; an invalid channel does not set it), cleared by an active-source change and by a gap over 2 s between two accepted states of a source.
- **Bench check procedure for the owner** (no props, USB to the PC, ship node on Serial4, Pocket on): start the GS and watch the RC_APPLIED display (or `pio device monitor` is useless, use the GS). Expected with only the Pocket on: `ctrl_source` RADIO, `rc_age` < 0.1 s, `radio_age` about 0.04 s, `bench_age` NaN or growing, `RC_APPLIED.rejects` not increasing. Then flip each switch and compare:
  1. **SA** up/down: `sa` 1 / 0 (SA is the 2-position switch; the lowest position = NOT_GO = 0).
  2. **SB** three positions, low / middle / high: `sb` 0 / 1 / 2.
  3. **SC** low / middle / high: `sc` 0 / 1 / 2 (position 2 is reserved, shown as 2).
  4. **SD** down / up: `sd` 0 / 1 (down = disabled).
  5. **SE** (momentary, spring return) released / hold: `se` 0 / 1.
  6. `safe_seen`: bit 0 (GO) set while SA has been down, bit 1 (SC_REAL) while SC has been low, bit 2 (SD) while SD has been down, bit 3 (SE) while SE has been released since the last source change. After flipping SA up the bit stays set (the edge is the go event of T9b-4); to clear all bits, switch the Pocket off for over 2 s and back on (or force a source change as below).
  Expected switch-to-channel result: if any switch shows another switch's number, the Pocket's EdgeTX model does not output SA..SE on ch5..ch9 as required (decision D18): fix the model, not the firmware.
  7. **Source arbitration:** Pocket off for more than 2 s and the GS in master with its virtual radio on: `ctrl_source` USB, `bench_age` small; the console shows `Control source: BENCH`. Pocket on again: `ctrl_source` stays USB for about 0.5 s and then RADIO (`Control source: RADIO`); both present from the start gives RADIO.
  8. **Rejects:** with the Router and ship node connected and the GS virtual radio on in master, the vehicle must show nothing from node 0 over the radio path (`rejects[1]` may rise only if something else sends a wrong-key `RC_STATE`). A USB-serial adapter in place of the ship node sending a node 0 `RC_STATE` on Serial4 must raise `rejects[1]` and never change `ctrl_source`. A frame with `SOURCE_OK` clear raises `rejects[3]`.

**Port table:** [../codebase-structure.md](../codebase-structure.md#radio-and-usb-network).

**Wiring (bench-confirmed end to end, 2026-11-20):** the ship node (ESP8285 "RX module", node 3) is wired to **Serial4**: ship node TX (GPIO1) to **Teensy pin 16 (RX4)**, **Teensy pin 17 (TX4)** to ship node RX (GPIO3), 3.3 V, 460800 8N1. Verified with the ExVectrLink Router: it lists node 1 as present and the vehicle telemetry arrives over the radio. History: the module was first found on pads 30/31 (GPIO_EMC_37/36), which are not pins of any Teensy 4.0 hardware UART, so no `SerialN` could read them; the wires were moved to 16/17. During the SX1280 removal the TX pin (17) had been shorted to ground; it was repaired.

**Finding the wiring again (bench tool):** `tools/uart_pin_scan.py` (run with the ground-station venv python, which has pyserial; needs the Router attached) disables the ground-station USB source, flashes the bench-only scan firmware (`pio run -e teensy40_pinscan`, `tools/pinscan/pinscan.cpp`, links no flight code), listens on every UART RX pin and scans all pads for edges, then transmits node 1 heartbeats on candidate TX pins one at a time while polling node 3's `NetworkStatus` through the Router (age[1] about 0 means the ship node heard node 1). It finishes by rebuilding and flashing the flight firmware of the checked-out branch and re-enabling the USB source (`--no-restore` leaves the scan firmware on). `--quick` re-checks only the known candidates (460800 listen, TX on pin 17, bit-bang pad 31). Use it after any rewiring or hardware change near the UART.


**Hard motor interlock (T9b-2, `feature/t9b-2`):** a pure gate at the last point before the motor commands. Design: `docs/t9b-design.md` 3.4, 6.3, 6.6; rules: workspace `docs/safety.md` section 3 and rule 9.
- **`Safety::MotorInterlock`** (`src/safety/motor_interlock.*`, pure, no heap, no simulation input, host-tested with a fake clock: `test/test_motor_interlock`). `open(now)` = an observed DISABLED -> ENABLED transition **and** an accepted ENABLED SD state of the **active** control source at most **1 s** old. Fed by the RC taps (`RcIngressTap::onFrame`: every accepted `RC_STATE`, SD = channel 8 decoded by `RcCommandMapper::decodeTwoPos`, only value 1 = ENABLED; invalid decodes as DISABLED) and by `VehicleNetwork::pollNodes()` (`update(now, active source)` every 10 ms comms tick). Frames from a source that is not the active one, frames with `SOURCE_OK` clear, wrong keys and stale sequences never reach it (the control-source table rejects them), so they count as "no data" and the gate closes by the 1 s age.
- **Observed transition:** two consecutive accepted states of the active source, DISABLED then ENABLED, at most 1 s apart. Forgotten (needs a new off -> on) by: boot, an active-source change, a silence or gap of **more than 2 s** (D3-B; a gap of 1 to 2 s only closes the gate and a fresh ENABLED reopens it without a toggle), and the D4-A triggers below. The first frame after a (re)connect only makes the source active (the interlock sees states from the next frame on), which is conservative.
- **D4-A re-arm triggers** (`requireRearm`, close at once and need a new toggle even if SD stays ENABLED): chain no longer pending (RTH landed, or the pending chain was cancelled; `handleVehicleMissionControl`), mode **Failure** (every safety-task run while it lasts), the SE reset, an SC change (simulation/real), and the ground "reset and stop" (SA released on the ground, calibration countdown abort). **Not yet available:** the latched abort (an in-flight abort latch arrives with T9b-6); a failure-mode abort already re-arms through Failure.
- **Where it acts:** `StarshipTVC::taskThread` evaluates `Safety::evaluateMotorGate` once per run (interlock open, watchdog forced-off flag, `enableMotors_`, `enableActuators_`, test idle): motor `enableOutput(...)` and the `setValue()` call are conditioned on its result. A null interlock pointer is closed. **`MotorInterlockWatchdog`** (main.cpp, 100 Hz, **priority 120**, above every controller) calls `StarshipTVC::forceMotorsOff()` every run while the gate is closed (both motor outputs duty 0, sticky forced-off flag that `taskThread` also honours, the modelled thrust is untouched (D16 rejected)) and `releaseMotorsForce()` while open. This covers the event-driven TVC task (it does not run if the controller stops publishing). Nothing in mission code, the failure handler or RTH can bypass it: they only set `enableMotors_` and `enableActuators_`, which come before the AND.
- **Watchdog-stale failsafe:** `checkForFailures` latches mode Failure with `failure_bits` bit 13 (`interlockWatchdog`) if the watchdog task has not run for **100 ms** (or never ran). Cleared by the SE reset like the other failures.
- **Control-source latch:** the safety task now passes `committed = interlock open || mission chain pending || mode Running` to `VehicleComms::setCommitted()` every run (it was always false in T9b-1), so the active source stays latched while the vehicle is committed (`ctrl_flags` SOURCE_LATCHED).
- **Telemetry:** `VEHICLE_STATUS.flags` **MOTORS_ENABLED** = interlock open **and** `enableMotors_` **and** actuators enabled (TVC task run, i.e. the same AND as the motor output without the watchdog flag). **ARMED = interlock open since T9b-4** (the legacy arm state is gone). `ctrl_flags` **INTERLOCK_NEED_TOGGLE** (bit 0) = SD shows ENABLED in a state at most 1 s old but no off -> on transition was observed. `failure_bits` bit 13 as above. The vehicle logs `Motor interlock OPEN ...` and `Motor interlock CLOSED: <reason>` (no control source / SD enabled but needs a toggle / SD disabled / no valid enabled state within 1 s) on every gate change (10 Hz sampling, so a glitch shorter than 100 ms is not logged).
- **Simulation:** the interlock never reads the simulation flag and simulation never changes it; the vehicle boots in real mode. D16 stays rejected: the simulated thrust is not gated. **Since T9b-4 the legacy `enableMotors(!simulationMode_)` gate is gone** (owner decision 2026-11-21, safety.md rule 9): a go enables the motors in simulation exactly as in real mode, so in a bench simulation the motor PWM runs whenever the interlock is open. **Bench rule: the motors are electrically disconnected (ESCs unpowered) for every simulation or bench test; SD is only an additional measure.**

**Interlock bench procedure for the owner (T9b-2).** Motors electrically disconnected (ESCs unpowered), Teensy on USB with the GS, Router and ship node on Serial4, Pocket on, no props. Watch `VEHICLE_STATUS` (`ctrl_flags`, `flags`) and the console. Since T9b-4 the simulation no longer holds the motor PWM off (the gate is the only thing that does): the motors are electrically disconnected, the **gate** is observed through the log lines `Motor interlock OPEN/CLOSED`, `INTERLOCK_NEED_TOGGLE`, `MOTORS_ENABLED`, and the PWM pins.
  1. **Boot with SD in each position** (Pocket SD up, then down, power-cycle the Teensy each time with the Pocket on): SD up at boot: log `CLOSED: SD enabled but needs a toggle`, `INTERLOCK_NEED_TOGGLE` set, the gate never opens however long you wait. SD down: closed, no flag. Flip SD down -> up: `Motor interlock OPEN` within one RC period, flag cleared.
  2. **Stale RC with SD enabled** (simulation, SD up and the gate open): switch the Pocket off. About 1.0 s later the log shows `Motor interlock CLOSED: no valid enabled state within 1 s`. (The simulated vehicle does not fall: the simulated thrust is not gated. **Since T9b-3** an open gate means committed, so about 1 s later (2.0-2.1 s after the last RC state) the control failsafe also latches Failure, bit 5; clear it with the SE short press before the next step.) Repeat with the GS as control source (Pocket off, GS virtual radio master, virtual SD enabled behind its lock, then release the control lease or stop the GS virtual radio).
  3. **Reconnect with SD already enabled never enables:** after step 2 with the SD switch still up, switch the Pocket on again. After a gap of up to 2 s the gate reopens by itself (D3-B, logged OPEN). After a gap of more than 2 s (leave it off for 3 s or more) it must stay closed with `INTERLOCK_NEED_TOGGLE` set until SD is flipped down and up. Also a Pocket that is switched on with SD up in a fresh source (for example after the GS took over, see step 5) must not open.
  4. **Stuck SD:** SD up and never toggled (with the Pocket on for a minute): never opens. SD down (never raised): never opens.
  5. **Source change:** with the gate open on the Pocket and the vehicle not committed (no mission, mode Ready), switch the Pocket off for more than 2 s so the GS virtual radio becomes the source (`Control source: BENCH`, SD enabled on the GS): closed, needs a toggle on the GS SD.
  6. **D4-A re-arm:** with the gate open and SD still up: press SE briefly (reset, closed at once, needs a toggle), cycle SC (same), let a simulated RTH finish (same), and induce a Failure (same: the interlock re-arms while the mode is Failure).
  7. **SD down:** gate open, flip SD down: log `CLOSED: SD disabled` at once. Flipping it up again re-opens (a fresh edge). In real mode on the bench (no props, ESCs unpowered, scope on pins 7 and 8) the pulses stop at once.
  8. **Real mode pins (scope on the motor pins 7 and 8, ESCs unpowered, no props), I10:** in real mode (SC) with a go accepted (so `enableMotors_` is set), there must be no pulses on pins 7 and 8 whenever `MOTORS_ENABLED` is false or the log says CLOSED, in particular right after boot with SD up, after stale RC, and after reconnect with SD up. Only with SD toggled and the gate OPEN do pulses appear (idle pulse about 1.04 ms plus the first-enable delay of 2 s after boot).
  9. **I5, controller stopped:** not testable on this build (it needs a test hook that pauses `controlAttitudeTvc`); covered by the design of the watchdog (the TVC task is bypassed) and left to T10.
  10. **Watchdog:** nothing to do on the bench; a watchdog that does not run for 100 ms latches Failure with `failure_bits` bit 13 (look for it in the heartbeat after heavy load). If it ever appears on the bench without a fault, report it: the 100 ms limit may be too tight for the scheduler.

**Control failsafe bench procedure for the owner (T9b-3).** Simulation mode (selected from the GS), motors electrically disconnected, ESCs unpowered, no props, Teensy on USB with the GS, Router and ship node on Serial4, Pocket on. Watch `VEHICLE_STATUS` (`ctrl_source`, `rc_age`, `failure_bits`, `ctrl_flags`, `degraded_bits`, `flags` FAILSAFE) and the console. Nothing here needs real motors.
  1. **Pocket off while idle on the ground (D2-B):** vehicle Ready, disarmed, no mission, SD down. Switch the Pocket off. About 2 s later `ctrl_source` becomes NONE (console `Control source: NONE`, or `BENCH` if the GS virtual radio is master). `failure_bits` stays 0, the mode stays Ready, no FAILSAFE flag. Switch the Pocket on: `Control source: RADIO` (a handover needs no latch). Wait 1 minute with the Pocket off: still no failure.
  2. **Pocket off while committed (mission running):** SD down, SC to simulation, SA down then up (go; a 18 s countdown), SD up (toggle, interlock OPEN). Let it run into the hover. Switch the Pocket off (SD state irrelevant). About 1 s: console `Motor interlock CLOSED: no valid enabled state within 1 s`. 2.0 to 2.1 s after the last state: console `Failsafe: control source lost (no accepted RC_STATE for 2 s)`, `failure_bits` bit 5 (0x20) set, mode Failure (FAILSAFE flag), simulated actuators off. Repeat from other phases (ascent, bellyflop, freefall); the same each time.
  3. **Heartbeats and GS do not keep it alive:** repeat step 2 with the GS connected and sending `REQUEST`s or telecommands over the radio path, with the Router and ship node heartbeats flowing (they do while the Pocket is off). The failsafe must still fire at 2 s. `ctrl_flags` RADIO_FRESH drops after 2 s while the GS data still arrives.
  4. **GS USB unplugged while RADIO is active:** mission running on the Pocket, unplug the GS USB (or close the GS): nothing happens (no failure, no source change; `GS_PRESENT` is informational and the vehicle ignores it).
  5. **GS as the bench source and lease release:** Pocket off, GS virtual radio master, the GS control lease taken so RC_STATE reaches the vehicle on USB (`Control source: BENCH`), mission as in step 2, then release the lease or stop the GS virtual radio: the same 1 s interlock close and 2 s failsafe.
  6. **Reconnect and reset:** after a failsafe switch the Pocket on: the failure stays latched (mode Failure). The SE short press on the ground clears it; with the Pocket on the vehicle returns to Startup then Ready. With the Pocket still off at the time of the reset it does not latch again (not committed) and `ctrl_source` is NONE.
  7. **Degraded:** `degraded_bits` bit 2 needs RC arriving over the UART while the ship node itself sends nothing for 3 s, which the bench cannot produce (unplugging the UART also stops the RC). Host-tested only (`test_vehicle_network`); on the bench `degraded_bits` must simply stay 0 in steps 1 to 6.

**RC command path (T9b-4, `feature/t9b-4`): the safety commands are RC state.** Design: `docs/t9b-design.md` 3.2, 3.3, 6.1, 6.4 with the owner decisions of 1.3 (they win); rules: workspace `docs/safety.md` sections 1 to 4. The telecommands Arm, SimulationMode, MissionBegin, MissionSelect, MissionAbort and SystemReset are **removed** (section 10); the same RC state is read from the Pocket (RADIO) and from the ground station's virtual switches (BENCH) by the same code, only the **active source** (section 5, arbitration) is looked at.
- **Layers:** `Safety::RcCommandMapper` (decode, safe-seen bits, **latched edge events** of the active source, SE classifier; `rc_mapper.*`) -> `Safety::RcCommandLogic` (pure intent layer: go preconditions and refusal codes, SC decision, D12 "on the ground", SE actions, hold intents; `rc_command.*`) -> `VehicleSafetyAndControlTask::applyRcCommands()` (main.cpp, every 10 Hz safety tick before anything else) applies the intents with the existing vehicle code. All pure parts are host-tested (`test/test_rc_mapper`, `test_estimator_zeroing`, `test_mission_chain`); none reads the simulation flag as a gate.
- **Switch semantics (all unsafe directions need an observed transition: two consecutive accepted states of the active source at most 1 s apart, the safe value seen first; every handover, boot and gap over 2 s forgets it, so every switch position at boot does nothing until that switch is cycled):**

| Switch | Safe direction (acts on the state) | Unsafe direction (observed edge plus preconditions) |
|---|---|---|
| **SA** go / abort | NOT_GO or invalid while a mission chain is pending or the mode is Running: `Safety::decideAbort` **every tick** (idempotent): on the ground reset and stop (no latch cleared), in flight RTH (kept while RTH runs). A stale source does not abort (the control failsafe covers it). | NOT_GO -> GO, SA still GO when the tick looks at it, and: mode Ready, no latched failsafe, no estimator zeroing window and a valid zero, SC valid and equal to the applied mode, SB valid with an implemented profile (**only SB 2**, D9), no calibration session or gyro run in progress. **The interlock is not a precondition** (owner 2026-11-21). Then the existing calibration gates and the countdown gyro run, then mission begin with the SB profile, **fixed countdown** (compiled in, 18 s, see Known issues), mode Running, `enableMotors(true)` in simulation and real mode alike. A refused go keeps its reason in `go_refusal` and logs `GO refused: <reason>` (2 s rate limit); it must be cycled to try again. A held SA never starts a second go. |
| **SB** profile | none | read at the go edge only and latched (`mission_profile`); changes after go are not looked at. SB 0 and 1 refuse the go (`go_refusal` 5) until T9b-8. |
| **SC** simulation / real | none | an **observed change between positions 0 (simulation) and 1 (real)**, only while on the ground (D12), not committed (interlock open, chain pending or Running) and not in mode Failure; refused otherwise (log line, SC_MISMATCH shows the difference). 0 -> 1 needs position 0 seen first. Position 2 and invalid are no position: no edge into or out of them, SC_MISMATCH set, go refused. **The vehicle boots in real mode**; an SC at 0 at boot leaves it real (mismatch, go refused) until SC is cycled. The change runs `setSimulationMode()` (topic switches as before, body simulator reset when entering, mode Startup when leaving) and needs a new SD toggle (D4-A). Simulation gates nothing. |
| **SD** interlock | DISABLED, invalid or stale: gate closed at once | the interlock section above, unchanged. Motors run only when the control system commands them **and** the gate is open; `ARMED` in the telemetry = the gate is open. |
| **SE** reset / zero | none | **short press** (>= 2 pressed states, no gap over 200 ms, released within 1 s of the first pressed state, RELEASED seen first): the system reset (`systemResetFromSe()`: mission chain to the boot state, failures **and latches cleared**, Startup and re-init, actuators and motors off, an active zeroing window aborted, simulated body to its start pose in simulation, the interlock needs a new toggle, `mission_profile` back to 255), **only on the ground with the interlock closed** (D12: interlock closed, motors not enabled, and mode Startup/Ready/SensorCalib with the mission Idle or Finished, or mode Failure, or mode Running with the chain ended); refused in flight (SE_REFUSED for 2 s). **Hold** (pressed for 1 s): zeroes the estimators **while held** (`EstimatorZeroing::holdStart`, same conditions as `ZeroEstimators`: initialised, interlock closed, motors off, mode Startup or Ready, mission Idle or Finished); continues while fresh pressed states arrive, ends 300 ms after the last one, on release, when the interlock opens (aborted) or 60 s after the start. It records a valid zero if it zeroed for at least 1 s; a shorter hold (or an aborted one) leaves a partial zero: the valid zero is cleared and the vehicle returns to Startup (automatic zero). A gap over 200 ms between pressed states, or an invalid SE, cancels a press with no action, so a lost release sample can never turn a hold into a reset. A stuck SE: zeroing stops at 60 s, no reset. |

- **Logs:** `GO: mission begin, profile 2, countdown 18 s`, `GO refused: ...`, `SA released: on the ground: reset and stop | in flight: start RTH ...`, `Simulation mode ON|OFF (SC)`, `SC change refused ...`, `System reset (SE short press)`, `SE reset refused: only on the ground with the interlock closed`, `SE hold: estimator zeroing while held`, `SE hold zeroing ended: valid zero recorded (x s)`, `Telecommand N ignored: removed in T9b-4 ...`.
- **Open points for the owner** (all in Known issues): the countdown is 18 s (15 s would refuse every go); SC acts only on observed changes (not as a level); D12 includes a landed vehicle in mode Running; the arm state is gone entirely (the brief of this step removes it, D5-B asked for a separate arm from the Pocket: with the interlock open the motors are at full power, no further condition).

**RC command bench procedure for the owner (T9b-4).** **Motors electrically disconnected (ESCs unpowered) and no props for every item: the simulation no longer holds the motors off, SD is only an additional measure.** Teensy on USB with the GS, Router and ship node on Serial4, Pocket on. Watch `VEHICLE_STATUS` (`flags`, `ctrl_flags`, `go_refusal`, `mission_profile`), `RC_APPLIED`, the console and the motor pins with a scope or by the ESCs being unpowered. Repeat A and B and the countdown case of E from the GS virtual switches (Pocket off, GS virtual radio master) for the same results.
  A. **A full mission from the Pocket switches only (real vehicle on the bench, simulation):** boot with all switches in their safe positions (SA down, SB 2, SC 0, SD down, SE released) and wait for Ready. SC is at 0 and the vehicle is real, so `SC_MISMATCH` is set and a go is refused (`GO refused: SC ...`): flip SC 0 -> 1 -> 0 (the second change enters simulation, console `Simulation mode ON (SC)`, `SIMULATION` flag, mode back to Startup then Ready). SB to 2, SD up (toggle, `Motor interlock OPEN`, `ARMED`), SA up: `GO: mission begin, profile 2, countdown 18 s`, mode Running, `mission_profile` 2. Let the simulated flight run to the hover, SA down: `SA released: in flight: start RTH` and the vehicle lands and ends (`RTH mission ended`, interlock closed, needs a toggle). SE short press: `System reset (SE short press)`, mode back to Ready, `mission_profile` 255. Repeat and release SA in the countdown (Initialisation and Startup phase): `on the ground: reset and stop`, no latch cleared.
  B. **Go refusals:** repeat the go with, one at a time: SB 0 and 1 (`go_refusal` 5), SC on the wrong position or SC at 2 (4), during a zeroing hold (3), right after boot before Ready (1), with a calibration session running (7). Each refused go needs SA down and up again; a held SA never gives a second go.
  C. **Boot with every switch combination:** power-cycle the Teensy with (SA up, SC 1, SD up, SE pressed) and with other combinations (each switch at its unsafe position alone, and all together). Nothing may happen: no go, no SC change, no reset, no zeroing, the interlock stays closed with `INTERLOCK_NEED_TOGGLE`. Then cycle each switch (SA down and up: go; SC 1 -> 0 -> 1; SD down and up; SE released and pressed) and confirm only that switch acts.
  D. **Stuck switches:** SA held up after a go: no second go; SE (the GS holds it pressed) for 70 s: zeroing starts at 1 s, stops at 60 s (`SE hold zeroing ended`), the vehicle is not reset; SD stuck up never opens the gate (interlock section); SC stuck at 0 or 2: go refused.
  E. **SE:** a short press on the ground with SD down resets; with SD up (gate open) it is refused (`SE_REFUSED` for 2 s, no reset); in a simulated flight it is refused and nothing changes. Hold SE for 3 s on the ground: zeroing starts after 1 s, ends at release (`valid zero recorded (~2 s)`), `ZEROING` flag only meanwhile; release after 1.3 s (zeroing < 1 s): `ended after ... (< 1 s or aborted)`, the vehicle returns to Startup and the automatic 10 s zero runs again. Open SD during a hold: aborted.
  F. **Stale RC in the countdown:** start a simulated go with SD up, then switch the Pocket off (or release the GS lease): about 1 s later `Motor interlock CLOSED: no valid enabled state within 1 s`, 2.0-2.1 s later the control failsafe (`Failsafe: control source lost`, mode Failure, `FAILSAFE_LATCHED`); the abort does not run on a stale source. Switch the Pocket on: the failure stays latched, SA does nothing until the SE short press on the ground clears it.
  G. **Removed telecommands:** sending the old Arm / SimulationMode / MissionBegin / MissionSelect / MissionAbort / SystemReset telecommands (a stale GS) only logs `Telecommand N ignored` (at most every 5 s) and changes nothing.

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

The packets are the generated ExVectrPackets classes (schema hash 0x65E4E6E3, byte layouts in the library's `docs/packet-reference.md`). Decoded values are SI; fixed-point scaling, saturation and NaN handling are done by the generated `encode()`. The old raw structs `VehicleState` (port 50) and `Net::Packet*` (ports 10-12) are removed; `MissionState` (`telemetry.hpp`) stays as the missions' internal state.

**Sets and rates** (100 Hz tick, rates from the generated `kUsbRateHz` / `kAirRateHz`):

| Port | Packet | USB | UART (radio set) | dst |
|---|---|---|---|---|
| 0x80 | `POSE` | 100 Hz | – | 0 |
| 0x90 | `IMU_RAW` | 100 Hz | – | 0 |
| 0x81 | `ACTUATORS` | 50 Hz | – | 0 |
| 0x20 | `VEHICLE_STATUS` | 20 Hz | 5 Hz | 0 |
| 0x82 | `MISSION` | 20 Hz | – | 0 |
| 0x91 | `ESTIMATOR` | 20 Hz | – | 0 |
| 0x83 | `GNSS` | 10 Hz | – | 0 |
| 0x92 | `TASK_TIMING` | 5 Hz | – | 0 |
| 0x93 | `CALIBRATION` | 0.5 Hz (every 2 s, `VehicleNetwork::kCalibrationPeriodTicks`; the schema rate is 0) | – | 0 |
| 0x03 | `HANDSET_TELEM` | 2 Hz | 2 Hz | 0xFF |
| 0x21 | `AIR_POSE` | – | 25 Hz (schema `Packets::AirPose::kAirRateHz`, every 4 ticks; a `static_assert` in `vehicle_network.cpp` checks it) | 0 |
| 0x22 | `AIR_ACTUATORS` (two samples per packet) | – | 25 Hz | 0 |
| 0x23 | `AIR_MISSION` | – | 5 Hz | 0 |
| 0x24 | `AIR_GNSS` (22 B, ticks = 3 mod 10) | – | 10 Hz | 0 |
| 0x25 | `AIR_GNSS_INFO` (10 B, tick 31 of each second) | – | 1 Hz | 0 |
| 0x02 | `LOG_TEXT` | as printed | – | 0 |
| – | heartbeat | 1 Hz | 1 Hz | 0xFF |

UART total 3.16 kB/s including framing and heartbeat (6.9 % of the 46.08 kB/s of 460800 baud 8N1; measured with the vehicle-network test dump of one second of the radio set, 3161 B, with `AIR_GNSS` and `AIR_GNSS_INFO`; 2.8 kB/s, 6.1 %, before them; was 2.19 kB/s with AIR_POSE at 10 Hz). Air downlink: about 2.7 kB/s of entries of 6.15 kB/s entry capacity (about 44 %; AIR_POSE 25 Hz since 2026-11-22; before 35 % since `AIR_MISSION.setpoint_q`, see ExVectrPackets `docs/telemetry-sets.md`); USB about 18 kB/s plus log text. Bench check 2026-11-20 over the air: 24.6 `AIR_ACTUATORS`/s and 4.8 `AIR_MISSION`/s arrived at the ground station, `prev_age` 17.5-22.5 ms (mean 20.0 ms).

**Sources** (`TelemetryProvider`, main.cpp; read-only, in the telemetry task): attitude and position from the topic switches (estimators, or the body simulator in simulation mode, as everything downstream sees them), ground distance from the ground topic switch, GNSS from `gnss`, flap angles from `trueFlapSettingTopic`, TVC from `StarshipTVC::getTVCActualThrustTopic()`, raw sensors from the MPU9250, QMC5883 and BME280 topics, standard deviations from the estimators' covariances, mode/mission/failures from `VehicleSafetyAndControlTask`.

**Field mapping:**

| Packet | Field | From |
|---|---|---|
| all | `t_us` | `(NowNs() / 1000) mod 2^32` |
| `VEHICLE_STATUS` | `t_ns` | `NowNs()` |
| | `mode` | `VehicleMode`: 0 Startup, 1 SensorInit, 2 SensorCalib, 3 Ready, 4 Running, 5 Failure |
| | `mission_mode`, `mission_phase` | `MissionMode` (0 Idle … 4 Finished), `MissionPhase` (0 Idle … 7 Completed) |
| | `ctrl_source` | the active control source (T9b-1, `Safety::ControlSourceTable`): 0 NONE, 1 RADIO (node 2 on the UART), 2 USB (BENCH, node 0 on USB). Observe only: nothing acts on it yet. NONE = no accepted `RC_STATE` within 2 s |
| | `flags` | ARMED = the motor interlock is open (**T9b-4: the legacy `vehicleArmed_` and the `Arm` telecommand are gone**, `VehicleSafetyAndControlTask::isVehicleArmed()` = `MotorInterlock::open(now)`); SIMULATION (set by SC through the active source, section 5 "RC command path"); MOTORS_ENABLED = `StarshipTVC` motors enabled **and** actuators enabled **and** the hard motor interlock open (T9b-2, `starshipTVC.interlockOpen(now)`; the SD switch of the active source, section 5); GS_PRESENT = ground-station presence (section 5; information only, not the failure condition); INCOMPATIBLE; FAILSAFE = mode is Failure (actuators cut); ZEROING (bit 7) = an estimator zeroing window is active (`VehicleSafetyAndControlTask::estimatorZeroingActive()`: boot zero or `ZeroEstimators`; the mission's own Initialisation zeroing does not set it); UART_PEER_SEEN (bit 6) = a valid frame from the ship node (node 3) arrived on the UART within 3 s (`VehicleNetwork::isUartPeerSeen()`, from the UART node's node table, so USB frames and other nodes never set it). **Information only:** nothing in the vehicle reads it (safety.md 4.1). It exists because `NetworkStatus` seen over USB reflects only the USB node, so a dead UART is otherwise invisible |
| | `failure_bits` | bit 0 sensor, 1 attitude, 2 position, 3 position out of bounds, 4 attitude out of bounds, 5 `CONTROL_SOURCE_LOST` (T9b-3: the latched control source's accepted `RC_STATE` older than 2 s while committed; was `radioConnectionLoss`). The schema also reserves bits 6-13 (T9b-6: 6 GNSS, 7 magnetometer, 8 baro, 9 gyro, 10 accel, 11 attitude estimate, 12 position estimate); **bit 13 (interlock watchdog) is set since T9b-2** (the 100 Hz motor-interlock watchdog did not run for 100 ms, latches Failure); the vehicle never sets bits 6-12 yet |
| | `sensor_state` | 2 bits per sensor, `TelemetrySensor` (0 Init, 1 Calib, 2 Ready, 3 Failure): bits 0-1 GNSS, 2-3 IMU, 4-5 baro, 6-7 mag, 8-9 attitude KF, 10-11 position KF |
| | `last_req_id`, `last_req_status` | 0, 0 (no `REQUEST` handling until T9b-5) |
| | `rc_age` | age of the last accepted `RC_STATE` of the **active** source, s; NaN when the active source is NONE (T9b-1) |
| | `ctrl_flags` | T9b-2 sets INTERLOCK_NEED_TOGGLE (bit 0: SD shows ENABLED in a state at most 1 s old but no off -> on transition was observed; the pilot must toggle SD), RADIO_FRESH (bit 7), BENCH_FRESH (bit 8, an accepted state of that source at most 2 s old, also for the ignored source) and SOURCE_LATCHED (bit 9: the source is latched because the interlock is open, a mission chain is pending or the mode is Running); **T9b-4** adds the bits the vehicle logic owns (`TelemetrySnapshot::cmdFlags`, OR-ed in by `makeVehicleStatus`): SC_MISMATCH (bit 4: an active source whose SC is invalid, reserved (position 2) or not equal to the applied mode), SE_HOLD_ZEROING (bit 5: an SE hold is zeroing the estimators), SE_REFUSED (bit 6: an SE press or hold was refused, shown 2 s), FAILSAFE_LATCHED (bit 2: mode Failure, latched until the SE reset). ABORT_LATCHED (bit 1), NOPOS_RTH (bit 3) and REQ_QUEUE_FULL (bit 10) stay 0 until T9b-6/7/5 |
| | `rc_rejected` | wrapping u8 count of rejected `RC_STATE` frames, all reasons, both sources (T9b-1) |
| | `go_refusal`, `degraded_bits`, `mission_profile` | `degraded_bits`: bit 2 `UART_HEARTBEAT_LOST` since T9b-3 (info only, see section 5), else 0; **T9b-4:** `go_refusal` = the reason the last go edge was refused (0 none, 1 vehicle not Ready or calibration invalid, 2 latched failsafe (mode Failure), 3 zeroing window or no valid zero, 4 SC invalid / reserved / not equal to the applied mode, 5 SB invalid or profile not implemented (only SB 2 flies, D9), 6 reserved (the interlock is not a precondition), 7 calibration session or gyro run in progress); kept until the next go is accepted or the SE reset. `mission_profile` = the SB position latched at the last accepted go (255 none, reset by the SE reset). Appended fields of the schema hash 0x65E4E6E3 (payload 36 B); real values arrive with T9b-4 to T9b-8 (definitions: library `docs/packet-reference.md`, design `docs/t9b-design.md` 3.8) |
| `RC_APPLIED` (0x84, USB only, 10 Hz, 18 B, tick phase 5) | `sa`, `sb`, `sc`, `sd`, `se` | the decoded switch positions of the **active** source (`RcCommandMapper::decode`): `sa` 0 NOT_GO / 1 GO, `sb` 0/1/2 or 255 invalid, `sc` 0 sim / 1 real / 2 reserved or 255 invalid, `sd` 0 DISABLED / 1 ENABLED, `se` 0 RELEASED / 1 PRESSED (invalid SA, SD, SE report 0). With no active source: 0, 255, 255, 0, 0 |
| | `safe_seen` | the mapper's four safe-seen bits of the active source (0 with none): bit 0 GO, 1 SC_REAL, 2 SD_ENABLE, 3 SE_PRESS |
| | `radio_age`, `bench_age` | age of the last accepted `RC_STATE` of each source, s, NaN = never (keeps counting after the source is lost, saturating at 65.5 s) |
| | `rejects[4]` | wrapping reject counters per reason (see section 5): bad length, source not in the table, stale sequence, `SOURCE_OK` clear. Never sent on the UART (port >= 0x80). Bench check procedure: section 5 |
| | `home_epoch` | `Comms::HomeEpoch` in `VehicleSafetyAndControlTask`: **0 until the vehicle has a position reference**, then a counter that increments each time the reference is re-set (first value 1, wraps 255 -> 1, never back to 0). The reference is the GNSS reference point `gnssRefPos_` of the position KF, which only moves while the estimators are in zeroing mode, so `referenceSet()` is called on the **falling edge of the applied zeroing mode** in `applyEstimatorZeroing()`: end of the boot zero, of a `ZeroEstimators` window, of the mission's Initialisation zeroing, and of a window aborted by SystemReset or a calibration start (that window also moved the reference; an extra increment only makes the ground station request HOME_POSITION once more). `SystemReset` without an active window does not change it (the estimators keep their origin). `HOME_POSITION` (0x50) itself is not sent yet (the request channel comes later). Telemetry only |
| | `calib_state` | bit0 gyro valid, bit1 accel valid, bit2 mag valid (`Calibration::Validity` per sensor, `src/calibration/calibration_validity.hpp`, `Reason::Ok`), bit3 a calibration session is active (`CalibrationService::anyBusy()`, any sensor collecting or fitting, or the automatic gyro run), bit4 ready to arm = `Calibration::readyToArm()` = mode Ready, no zeroing window and `decideArm() == None` (the same gate function the Arm command uses; Arm is only useful in Ready because the task disarms outside Ready), bits 5-7 zero. Cached by the safety task every 100 ms run (`updateTelemetryCache()`), so it lags a mode change by up to 0.1 s; the telemetry task does not evaluate the validity. Same in simulation mode (safety.md rule 9: telemetry only, no gate reads it) |
| `POSE` | `omega`, `q`, `vel`, `pos` | attitude state `[ω, q]`, position state `[V, P]` (f32) |
| `AIR_POSE` | `q` (i16, 1/32767), `pos` (**i32, 0.001 m = 1 mm**, ±2147483 m; was i16 0.1 m until 2026-11-20), `vel` (i16, 0.01 m/s, ±327 m/s) | as `POSE`, fixed point, saturating; 30 byte payload. USB `POSE` is f32 and unchanged |
| `ACTUATORS` | `flaps` | true flap angles `[TL, TR, BL, BR]`, rad. **Modelled, not measured**: the output of the servo model in `StarshipFlaps` (300 °/s slew; in simulation also the injected faults); no position feedback exists. |
| | `tvc` | `[atan2(Fx, Fz), atan2(Fy, Fz)]` of the actual thrust vector (body frame), rad. **Modelled, not measured**: servo-space slew model (20 °/s on the thrust angle) in `StarshipTVC`, not the measured vane position. |
| | `thrust` | `|[Fx, Fy, Fz]|`, N |
| | `roll_torque` | the actual thrust topic's `T` (`s.tvcThrust[3]`), f32 N·m: the same source and definition as `AIR_ACTUATORS.roll_torque`, unquantised |
| | `ground_distance` | ground topic switch `data(1)` (KF state `[rate, distance]`), m; NaN before the first sample |
| `AIR_ACTUATORS` | `t_us`, `prev_age` | time of the newest sample; newest minus previous sample time, s (nominally 0.020; NaN with no previous sample) |
| | `tvc[4]`, `roll_torque[2]`, `thrust[2]`, `flaps[8]` | newest first, then previous: same definitions as `ACTUATORS` (`tvc` = `[atan2(Fx,Fz), atan2(Fy,Fz)]` of the actual thrust, `thrust` = its magnitude, `flaps` = true angles `[TL,TR,BL,BR]`), plus `roll_torque` = the actual thrust topic's `T` (the twist actuation as roll torque, N·m; the vane twist fraction is `T·45/thrust`, limited to ±0.5). Fixed point: angles 0.1 mrad, torque 0.01 mN·m, thrust 1 mN, saturating, NaN -> sentinel. Samples are taken every 20 ms (`VehicleNetwork::kActuatorSampleTicks`) into `ActuatorHistory` (4 entries) and the packet is sent at 25 Hz on a sampling tick, so the previous sample is exactly the one before the newest. The values are the modelled actual output (commands after the mixer limits and the rate model), not servo feedback; the vehicle has no servo feedback |
| `AIR_GNSS` | `latitude`, `longitude`, `altitude`, `vel` | the u-blox driver's values as `GNSS` (radians, m above the ellipsoid, NWU m/s). Wire: lat/lon i32 1e-7 deg, altitude i32 mm, velocity i16 0.01 m/s, done by the generated codec; sentinels decode as NaN. Before the first GNSS measurement (`gnssSubr_` timestamp 0) every value is NaN. An invalid fix still sends the receiver's values; `AIR_GNSS_INFO.fix_type` says whether to trust them. 10 Hz on the UART only (USB has `GNSS`) |
| `AIR_GNSS_INFO` | `fix_type`, `satellites`, `h_acc`, `v_acc` | the REAL UBX-NAV-PVT `fixType` (`UbloxSerialGNSS::fixType()`, cached in `_getData()`), numSV (`getSIV()`), `positionCov(0)` and `positionCov(2)` (from `getHorizontalAccEst()` / `getVerticalAccEst()` mm -> m; u16 in 1 cm, NaN -> 65535 = unknown, NaN before the first measurement). 1 Hz on the UART |
| `AIR_MISSION` | `mission_time`, `mission_mode`, `phase`, `index`, `setpoint_pos`, `setpoint_vel`, `setpoint_q` | as `MISSION` (+ `mission_mode` as in `VEHICLE_STATUS`), fixed point: time 1 ms, position 1 mm, velocity 0.01 m/s, `setpoint_q` i16 `limit = 1.0` as `AIR_POSE.q` (raw = round(v * 32767), 0 stays exactly 0) |
| | `waypoint` | `missionWaypointTask.currentWaypointIndex()` while the waypoint mission is active, else 255 |
| `MISSION` | `setpoint_pos`, `setpoint_vel` | `positionSetpoint[3..5]`, `[0..2]` |
| | `mission_time` | `missionTime` ns -> s (1 ms wire resolution; negative in the countdown) |
| | `index`, `phase` | `MissionState.missionIndex` (always 0, see missions.md), current `MissionPhase` |
| | `mission_mode`, `waypoint` | exactly as `AIR_MISSION` (`MissionState.missionMode`; waypoint index of the waypoint mission, 255 = another mission is active) |
| | `setpoint_q` (also in `AIR_MISSION`) | Attitude setpoint of the **currently active attitude controller**, `[w,x,y,z]` body -> world, NWU (as `POSE.q`), unit with w >= 0; **all four 0 = no attitude setpoint**. Pure mapping `Comms::attitudeSetpoint()` (telemetry_mapping.hpp, host-tested). Selection by the controller phase table as applied (`Safety::controlPhaseFor`): (1) bellyflop set on (`bellyflopFlapsActive()`, freefall and the bellyflop after its kick) **and** the flap setting on `bellyFlopControlTopic` is `BellyFlopMode_Stabilize` (FlapModeObserverTask: vehicle and mission Running) -> flap controller; (2) else TVC set on (`tvcControlActive()`, waypoint/RTH Running with actuators, and the bellyflop kick) -> during the kick (`MissionBellyflop::kickActive()`, input `tvcOverride`) the kick target the mission writes into the controller (`kickSetpoint()`), otherwise the TVC attitude controller's setpoint topic `controlMappingAccToAtt.getAttitudeTopic()` (the topic `controlAttitudeTvc.subscribeAttitudeSetpoint` is wired to, main.cpp setup), q part `[3..6]`, normalised. The phase table makes (1) and (2) exclusive since 2026-11-21; (3) else zero (idle, ground, countdown, Finished, RTH stabilisation with flaps Upright). The flap controller (ControlAttitudeFlaps, Stabilize) has no quaternion setpoint: it tracks only the direction of body +Z (`azimuthAngle_Rad` = atan2(z_W, z_N), `pitchAngle_Rad` = elevation of body +Z), roll about body +Z is uncontrolled. The published q is the current attitude rotated by the shortest world-frame rotation that puts body +Z on that direction (`flapPointingSetpoint`), so its roll follows the real vehicle. Non-finite or near-zero inputs give 0. Read through extra `Simple_Subscriber`s on the same topics and two const getters: no controller input, subscription or timing changes. Bench (simulation, full chain, 2026-11-21, `tools/bench_vehicle_usb.py`): 0 in Startup, Ready, countdown (Init, Startup) and after the RTH landing; unit (norm 1 +- 6e-5) in waypoint hover/ascent, bellyflop, freefall and RTH; control phase log lines, bellyflop time-limit end (1.01 s) and RTH auto-disarm as before. Bench 2026-11-21 (bellyflop kick, `feature/bellyflop-kick`): the kick target `[0.707, -0.001, -0.707, -0.001]` for the whole kick, then the flap setpoint; unit in every bellyflop and freefall sample after the `flapPointingSetpoint` fix (Known issues). |
| `GNSS` | `latitude`, `longitude` | rad (f64) |
| | `altitude`, `vel` | m above the ellipsoid; NWU m/s |
| | `h_acc`, `v_acc`, `s_acc` | the driver's `positionCov(0)`, `positionCov(2)`, `velocityCov(0)`: 1σ in m, m, m/s |
| | `fix_type` | the real UBX-NAV-PVT `fixType` (0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + DR, 5 time only), also in `AIR_GNSS_INFO`. `positionValid` stays the vehicle's own rule (>= 5 satellites) |
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

**Temporary (network migration T9a, removed in T9b).** The schema already defines `RequestAction.ZERO_ESTIMATORS = 12` (T9b-0); the vehicle does not handle `REQUEST` yet (T9b-5). The new command path (`RC_STATE` mapping, control-source arbitration, `REQUEST`) is task T9b and needs the user's approval. Until then the old `Telecommand` is carried unchanged through the new stack, so the vehicle stays commandable:

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

### Command list (handler: `VehicleSafetyAndControlTask::handleTelecommands`)

Only the newest telecommand per 100 ms tick is processed (`Simple_Subscriber`). **T9b-4:** the safety telecommands (4 to 9) are removed and no longer act; types 4 to 9 and unknown types are counted and ignored with a rate-limited log line (`Telecommand N ignored: removed in T9b-4`). The port and the non-safety commands (calibration, `ZeroEstimators`, the simulation-only commands 11 to 18) stay until T9b-5.

| # | Type | Magic / params | Precondition | Implemented behaviour |
|---|---|---|---|---|
| 0 | `CalibStart` | `subsystem` 0 gyro, 1 accel, 2 mag; `paramInt` 1 (or > 2) calibrate, 2 verify, 0 or ≤ −2 stop and fit, −1 discard | initialised, disarmed, motors off, mode Startup/Ready/SensorCalib, mission Idle/Finished, **not simulation mode** (else refused, logged 1/s) | Starts the `CalibrationService` session (accel 12-pose fit, mag sphere fit, gyro run), TVC actuators off, mode `SensorCalib`, aborts a zeroing window and clears the valid zero. When no session is collecting or fitting the mode returns to `Startup` (automatic zero). [calibration.md](calibration.md). |
| 1 | `CalibApply` | `subsystem` 0 / 1 / 2 | as `CalibStart`, and a candidate with verdict **PASS or WARN** | Writes the legacy key (accel `C = T A T⁻¹`, mag `C = s I`, gyro bias) to the RAM image, applies the transform (RAM only until Save), invalidates the estimator zero (Ready → Startup). |
| 2 | `Load` | — | — | **Not handled.** |
| 3 | `Save` | `subsystem` 0 / 1 / 2 | as `CalibStart`, no sensor collecting | EEPROM ← RAM image (all keys), then read back and compared (`saved ... verified` / `EEPROM save FAILED`). |
| 4 | `SystemReset` | **REMOVED in T9b-4** | — | Ignored, counted, rate-limited log line. Replaced by the SE short press (section 5 "RC command path"): only on the ground with the interlock closed; same reset body (`systemResetFromSe()`), which also clears the latches and needs a new SD toggle. |
| 5 | `Arm` | **REMOVED in T9b-4** | — | Ignored (see 4). "Armed" in the telemetry now means the motor interlock (SD) is open; the 10 % power limit and the auto-disarm rules are gone (motors run at full power when the gate is open). |
| 6 | `SimulationMode` | **REMOVED in T9b-4** | — | Ignored (see 4). Simulation or real is selected by an observed SC change of the active source (section 5 "RC command path"); the vehicle boots in real mode. The topic-switch code is unchanged (`setSimulationMode()`); the old `enableMotors(false)` in simulation is removed (the simulation gates nothing). |
| 7 | `MissionBegin` | **REMOVED in T9b-4** | — | Ignored (see 4). Replaced by the SA go edge: Ready, no latch, no zeroing, SC = applied mode, SB profile 2, no request in progress; then `beginMissionFromGo()` (the calibration gates and the countdown gyro run stay), countdown fixed at 18 s (see the open decision in section 5), `Running`, motors enabled in simulation and real mode alike. |
| 8 | `MissionSelect` | **REMOVED in T9b-4** | — | Ignored (see 4). SB is read at the go edge. |
| 9 | `MissionAbort` | **REMOVED in T9b-4** | — | Ignored (see 4). Replaced by SA released (level, every 10 Hz tick while a chain is pending or the mode is Running): `Safety::decideAbort` unchanged (in flight RTH, on the ground reset and stop), but ResetAndStop no longer clears latches. |
| 10 | `ZeroEstimators` | `paramInt == 0x2E`; `paramFloat` = window in s (NaN or 0: 10 s; clamped to 1..60 s) | initialised (all inputs published), disarmed, motors off, mode `Startup` or `Ready`, active mission `Idle` or `Finished`. Otherwise ignored, counted, rate-limited log (1/s) | Starts the zeroing window (overview.md, "Estimator zeroing"); a second one during the window restarts it with the new duration. One-shot ground action, timed on the vehicle. **The vehicle must be still.** |
| 11 | `SimWindSpeed` | `paramInt == 0x57`; `paramFloat` = wind speed in m/s (finite; clamped to 0..20) | **simulation mode only** (else refused, logged with the reason; also refused for a bad magic or a non-finite value) | Sets the steady wind of the body simulator (gusts: 13..15; only the simulated air changes). The wind is reset to 0 whenever the simulation mode (SC) switches on or off; the SE reset keeps it. Owner request 2026-11-21 (`feature/sim-aero`). |
| 12 | `SimWindDirection` | `paramInt == 0x57`; `paramFloat` = direction the air moves **toward**, rad, counterclockwise from North toward West (NWU: 0 = toward North, π/2 = toward West; wrapped to −π..π) | as 11 | Sets the wind direction (speed unchanged). Send 11 and 12 at least 100 ms apart: only the newest telecommand per 100 ms tick is handled. |
| 13 | `SimGustStrength` | `paramInt == 0x57`; `paramFloat` = peak gust speed (bound of the extra wind speed along the mean direction) in m/s (finite; clamped to 0..10; 0 = no gusts) | as 11 | Sets the gust strength (period and seed unchanged). Owner-approved simulation-only exception to safety.md rule 7 (2026-11-21). Reset to 0 with the wind when `SimulationMode` switches. |
| 14 | `SimGustPeriod` | `paramInt == 0x57`; `paramFloat` = gust timescale in s (finite; clamped to 0.5..30) | as 11 | Sets the gust period (default 5 s). |
| 15 | `SimGustSeed` | `paramInt == 0x57`; `paramFloat` = integer-valued seed 0..16777215 (finite; rounded to the nearest integer, clamped; exact in a float) | as 11 | Sets the gust seed (default 1). Same seed and settings give an identical gust series (model: ExVectrDSP.md). |
| 16 | `SimFlapStuck` | `paramInt == 0x57`; `subsystem` = flap 0..3 (0 TL, 1 TR, 2 BL, 3 BR, the telemetry ACTUATORS flaps order), **no other bits** (bit 4 `0x10`, the removed "arm for the descent" mode, is refused since 2026-11-21); `paramFloat` = stuck angle in rad (finite; clamped 0..π/2), **−1.0 = hold the true angle it has when the fault is applied** | **simulation mode only** (else refused and logged with the reason; also refused for a bad magic, bad subsystem bits or a non-finite value) | **Immediately** (next flap task tick, 20 ms; flaps enabled or not) the flap's modelled TRUE angle is forced to that angle whatever the controller commands, at the one publication point of the true flap topic, so the body simulator, ACTUATORS / AIR_ACTUATORS telemetry and the ground station's FlapState all show it. In simulation mode the flap's servo PWM also uses the forced angle (clamped to the 15..90 deg servo range) instead of the commanded one, so the physical servo sits at the stuck position (owner decision 2026-11-22, see "Servo PWM stage" in section 4). Log `Sim fault: flap TL STUCK at 42.0 deg (now)`. Owner request 2026-11-21 (redesign the same day: the arm-for-descent mode is gone; send the command when the fault should start). |
| 17 | `SimFlapSlow` | `paramInt == 0x57`; `subsystem` = flap index (0..3, no other bits); `paramFloat` = servo rate factor (finite; clamped 0.05..1.0, 1 = normal) | as 16 | Scales the modelled servo slew rate (300 deg/s) of that flap from then on. Effective at once, also on the PWM: the flap's PWM slews at normal rate x factor (2026-11-22). |
| 18 | `SimFaultClear` | `paramInt == 0x57`; subsystem and `paramFloat` ignored | as 16 (magic and simulation mode) | Clears every flap fault (a stuck flap snaps back to its commanded angle). Faults are also cleared on every simulation mode switch (on and off) and by the SE reset (review 2026-11-21). |

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
| Fixed T9b-4 | main.cpp, `starship_tvc.cpp` | (was High) Arming did not gate the motors, only a 10 % power limit. The arm state and the limit are removed; the SD interlock is the gate and the motors run at full power when it is open. | **Consequence: with the interlock open there is no power reduction any more.** Bench rule: motors electrically disconnected. |
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
| Low | `home_epoch` (section 9) | The counter follows the end of estimator zeroing mode, not the GNSS reference itself: it is 1 after the boot zero even if no valid GNSS fix existed during the window (the reference then stays at its default), and an aborted window counts too. | The ground station may request `HOME_POSITION` once more than needed, or get a home without a fix (check `AIR_GNSS_INFO.fix_type`). Revisit when `HOME_POSITION` is sent (needs a position-KF accessor for the reference). |
| Low | `calib_state` bit 3/4 (section 9) | Cached by the 10 Hz safety task: lags a mode, validity or session change by up to 0.1 s. `currentCalibrationValidity()` now also runs every task run (previously only on Arm, MissionBegin and in Ready); its run time on the Teensy has not been measured. | Information only. Check the safety task's `TASK_TIMING` average on the bench. |
| Low | `telecommand.hpp` | Header comments disagree with the code: CalibStart duration, MissionBegin "seconds in the future". (The SystemReset comment was corrected 2026-11-20: it never changes simulation mode, and it resets the mission chain to the boot state.) | Operator and ground-software confusion. |
| Medium | main.cpp `handleTelecommands`, section 10 | Only the newest telecommand per 100 ms tick is handled (`Simple_Subscriber`). Seen on the bench 2026-11-20: a `ZeroEstimators` sent within the same tick as a `SystemReset` from another sender was lost (SystemReset has since moved to SE). | Two senders (or a fast sequence) can silently drop a command. Senders must space commands > 100 ms (the ground station enforces 500 ms). T9b replaces the path. |
| Info | `starship_hardware.h` `RX_MODULE_SERIALPORT` | Resolved 2026-11-20: the ship node is wired to Serial4 (pin 16 RX4 / pin 17 TX4), confirmed end to end (Router shows node 1, vehicle telemetry arrives over the radio). It was first on pads 30/31 and was moved. | None. If the harness is rewired, re-check with `tools/uart_pin_scan.py` (section 5). |
| Info | main.cpp `checkForFailures`, section 5 | **T9b-3 (resolved):** the former rule (any node 0/2 data frame, 10 s boot grace) is replaced by the control-source rule (accepted `RC_STATE` of the latched source older than 2 s while committed, bit 5 `CONTROL_SOURCE_LOST`). A GS `REQUEST` or telecommand no longer keeps it alive. Verified on the host only (fake clock); the Pocket-off cases are in the T9b-3 bench procedure. | A vehicle committed with no source (mission started over USB without RC) fails at once. `LINK_STATS_LOST` and `TOF_LOST` degraded bits are not set yet (T9b-6). |
| Medium | section 10 | **TEMPORARY until T9b:** the legacy telecommand on port 0x7F is accepted from any frame claiming source 0 on either link, with only the three magic values as protection (as before on port 1000). It is not in the ExVectrPackets schema. | Same exposure as before; any device on USB or behind the RX module can command the vehicle. |
| High | `src/main.cpp` `MotorInterlockWatchdog`, `checkForFailures` | **T9b-2:** a watchdog that has not run for 100 ms latches Failure (bit 13), which cuts all actuators. The scheduler is cooperative: a long task run (EKF, calibration, flash write) delaying the 100 Hz task by more than 100 ms would make the vehicle fall in flight. Not measured yet. | Bench: watch for `failure_bits` bit 13 under full load before the next flight. |
| Medium | `lib/ExVectrActuator` `PWM_Output` | `outputEnabled_` is never assigned: `enableOutput(false)` only writes duty 0 once per call and a later `setValue()` drives the pin again. T9b-2 works around it (a closed gate never calls `setValue`, and `forceMotorsOff()` writes duty 0 every 10 ms), the library is not fixed. | The interlock's safety depends on no other code calling `setValue` on the motor outputs. |
| Fixed T9b-4 | main.cpp | (was Medium, owner decision safety.md rule 9) the legacy `enableMotors(!simulationMode_)` simulation motor gate is removed. | **A simulation now drives the motors exactly as in flight when a go is accepted and SD is on:** disconnect the motors for every simulation or bench test. |
| High | `src/main.cpp` `applyRcCommands`, `kGoCountdownParamInt` | **T9b-4 open decision:** the design fixes the go countdown at 15 s (D8), but the sensor-calibration rule needs `paramInt <= -18` (Initialisation >= 8 s for the countdown gyro run). The code uses **18 s** (`Safety::kGoCountdownParamInt`), the shortest value the existing rules accept; 15 s would refuse every go (`GO refused`/`go_refusal` 1). | Owner decision needed: keep 18 s, or relax `kMinCountdownParamInt` and use 15 s. |
| Medium | main.cpp `onGroundNow` | **T9b-4:** D12 "on the ground" is extended by one case: mode Running with no pending chain and the active mission Idle or Finished (a landed vehicle: the mode stays Running after an RTH, see missions.md). Without it a landed vehicle could only be reset by a power cycle. | Reviewed in the T9b-4 report. |
| Medium | `Safety::RcCommandMapper` SC edge | **T9b-4:** SC acts only on an observed change 0 <-> 1 in both directions, never as a level, so an SC left at 0 at boot does NOT put the vehicle into simulation (the vehicle boots real; go is refused with `go_refusal` 4 until SC is cycled). D6 said "leaving real acts at once". | Safer and consistent with "boot with every switch combination does nothing"; confirm with the owner. |
| Low | `ExVectrNetwork` (library) | The legacy SX1280 sources of the library are compiled (unused, dropped at link time). | Library warnings in the build log. |
| Info | altitude resolution (`POSE.pos[2]`, `AIR_POSE.pos[2]`) | The telemetry itself no longer limits it: `POSE` is f32 (sub-mm at a few m) and `AIR_POSE` is 1 mm since 2026-11-20 (was 0.1 m). What remains: BME280 pressure at 16x oversampling has about 0.2 Pa noise (about 1.5-2 cm of altitude), IIR filter setting 1, and the f32 pressure (Q24.8 compensation output, resolution 0.0078 Pa near 100 kPa, about 0.7 mm) and `calcAltitudeFromPressure` in f32 (about 0.5 mm per ULP of p/p0). The position KF fuses baro, GNSS and range with large baro covariance (`cov*1000`), and the reference is a 10 % low-pass of the baro during zeroing, so the altitude is smoothed, not quantised. No casts to integer on the vehicle before the packet. | Expect centimetre-level noise and slow drift (weather, temperature), not steps. Estimator not changed. |
| Info | `board_v_1_0.h` | BME280 SPI CS, RGB LED, battery ADC are defined but unused (QMC INT pin 16 is now RX4). There is no battery monitoring. | No low-voltage protection. |
| Low | `src/comms/telemetry_mapping.cpp` `attitudeSetpoint()` | `setpoint_q` in the waypoint/RTH TVC phases is the latest value of the acc-to-att mapping topic, not `ControlAttitudeTvc`'s internal `stateSetpoint_` (no getter in ExVectrControl). They differ only between mapping updates, where the controller propagates its setpoint with the measured body rate. During the bellyflop kick the published value is the mission's kick target (`tvcOverride`), which the controller propagates the same way for ≤ 20 ms between mission writes. | Telemetry display only (ground station drawing); no effect on control. |
| Fixed (2026-11-21) | `src/comms/telemetry_mapping.cpp` `flapPointingSetpoint()` | `setpoint_q` read 0 whenever the flap correction exceeded about 151° (the unnormalised half-angle quaternion, norm `sqrt(2(1+d))`, fell under `sanitizeSetpointQuat`'s 0.5 "invalid" threshold); bench: 0 for about 1 s at the start of freefall. The correction is now normalised directly; regression test in `test_flap_pointing_setpoint`. | Telemetry display only. |
