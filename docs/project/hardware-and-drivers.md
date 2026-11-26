# Hardware and Drivers

Pinout, vehicle constants and the application-level drivers in `src/` (TVC, flaps, ExVectrNetwork links, u-blox GNSS, VL53L0X), plus memory keys, telemetry and the REQUEST actions.

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
   - **Enabled motor at zero thrust (verified 2026-11-23):** `motorPowerLimit` is 1.0 (`main.cpp`), so the output is `0.04 + |F|/14`. A zero thrust command gives 0.04, which the `STANDARD` protocol (400 Hz, `pulse = 1 ms + value · 1 ms`) turns into a 1040 us pulse (idle), and the disabled-controller neutral command 0.1 N gives about 1047 us. Disabled (gate closed) = pulse 0. The first time the motors are enabled after boot the output is 0.0 for 2 s (`firstMotorEnable_`, ESC arming), afterwards the idle floor starts at once. Whether the motors actually spin at that pulse is up to the ESC (not verified on hardware). Used by the freefall pre-spin (missions.md section 4).

**Gating:**

| Output | Enabled when |
|---|---|
| Servos | `enableActuators_` **or** actuator test active (`.cpp:141`) |
| Motors | **`evaluateMotorGate`** (`src/safety/motor_interlock.hpp`): hard interlock open (T9b-2) **and** not forced off by the watchdog **and** `enableMotors_ && enableActuators_ && test idle`. A closed gate never calls `motor*_.setValue()` (the `PWM_Output` bug, Known issues in the library doc) and calls `enableOutput(false)` (duty 0) on every run. See "Hard motor interlock (T9b-2)" in section 5. |
| Motors, first start | Additionally 2 s after the first enable since boot (`.cpp:268`). `firstMotorEnable_` is never reset, so later enables have no delay. |
| `motorPowerLimit_` | Default 0.05. **T9b-4:** the legacy arm state and its 10 % limit are removed; `main.cpp` sets the limit to **1.0 once at boot**, so with the interlock open the motors run at full power. The motors need the control system (`enableMotors_`, `enableActuators_`), and the interlock (SD) open; there is no simulation condition anywhere on that path. |

**Actuator test (`beginActuatorTest(offset)`):** starts at `NOW + offset` and hijacks the servos regardless of enable. For `TVC_TEST_SPIN_TIME = 2 s` it traces a circle: `x = sin`, `y = cos` at 45° amplitude, 2 revolutions. For the next `TVC_TEST_TWIST_TIME = 2 s` it applies roll-only `twist = sin(...)` at amplitude 1.0 (unclamped, ±45°), 2 cycles. Motors are held off throughout. It is triggered at every mission `Startup` edge (main.cpp:663).

**"Actual" thrust model** (only when actuators are enabled and the test is idle; **independent of the interlock gate** and of `enableMotors_`: telemetry copy; **Owner decision 2026-11-23 (reverses the D16 rejection of 2026-11-21): the body simulator now receives a separate copy, `StarshipTVC::getTVCSimThrustTopic()`, that is exactly zero (thrust, vane force and roll torque) while the motor gate output `outputsEnabled` is closed**, via `Safety::gateSimulatedThrust`; the gate result is used, never the simulation flag, and the gate and real outputs are unchanged): direction = clamped `tvcVector`, magnitude = mean motor command × 14 N. The modelled thrust angles are rate-limited to the vane servo rate divided by the servo-to-thrust gain: 300°/s / `tvcFactor_` (15) = **20°/s**, so a full ±3° reversal takes 0.3 s (since 2026-11-21, `feature/sim-aero`, review F6; before: 600°/s in thrust-angle space = 9000°/s of servo). A non-finite command holds the previous angles and reports 0 thrust for that tick, so NaN can no longer latch into the integrated angles (it stayed NaN until reboot). `T = twist·|F|/45`, i.e. the roll torque reconstructed after clamping; `T` is zeroed when the actuators are disabled (review F13). The header calls `T` an "angle in radians".

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
- `telemetry_mapping.hpp/.cpp` (pure, host-tested): snapshot -> packets, flight-software bit fields, the HOME_POSITION builder.
- The `VehicleComms` object (links, rings, nodes, log queue, UART buffers: about 10 KB) is in **DMAMEM** (RAM2).

**Constant cost:** every tick builds and encodes the same packets whatever is connected. Writes are bounded by `availableForWrite()` and never block (the Teensy USB write would otherwise wait up to 120 ms for a host that does not read). With nothing draining a link its TX ring fills and further frames are dropped (`stats.txDropped`); encoding happens before that check. Telemetry is no longer produced inside the estimator topic callbacks (the old 5/10/1 Hz attitude, position and GNSS callbacks ran in the EKF/KF publish context): the control path does less work than before.

**Heartbeats:** each node sends the vehicle heartbeat (role 1, schema hash `Packets::kSchemaHash`) at 1 Hz on its link. UART peer: `UART_PEER_SEEN` (see section 9, `VEHICLE_STATUS.flags`) reports whether node 3 was heard on the UART in the last 3 s. Heartbeats and `LINK_STATS` of the modules are informational: they fill the node table and the `INCOMPATIBLE` flag (a present node with another network version or schema hash), nothing else.

**General link health (information only) and the control failsafe (T9b-3, input changed by the GS-control cutover, 2026-11-23):**
- **General link health never decides anything** (safety.md 4.1): `VehicleNetwork::isRadioPacketRecent()` is true while a valid DATA frame from node 0 or node 2 arrived on the UART within 2 s (`RadioTap`: heartbeats of any node, node 3 frames and USB never count; no boot grace). It feeds only the OLED link indicator and the connection counter. Ground-station presence (`isGroundStationPresent()`, any valid node 0 frame within 2 s) is information only (`GS_PRESENT`).
- **The control failsafe is one rule** (`VehicleNetwork::controlSourceLost(committed)`, on a seqlock snapshot of `Safety::CommandIngress`, pure logic `CmdIngressSnapshot::failsafeDue`): while **committed**, the accepted `CMD_STATE` of the latched ground-station session (node 0, either link) is older than **2 s** (or there is none) -> failure bit 5 `CONTROL_SOURCE_LOST` (`FailureState::controlSourceLost`), mode Failure, `vehicleShutdownControl(true)`: actuators cut, the vehicle falls. Latched until the SE reset. **Owner decision 2026-11-23: if the ground-station app, its PC, USB or the Router/air link dies in flight the vehicle falls after 1 to 2 s (accepted trade-off).** **Committed** = `Safety::committed(GroundContext)` (`src/safety/rc_command.*`, pure, host-tested): the motor interlock is open, or a mission chain is pending, or the mode is Running **and the vehicle is not on the ground** (the D12 `onGround` rule: a landed vehicle in mode Running, no chain pending, mission Idle or Finished, interlock closed, motors not enabled, is not committed; this includes the landed-Running fix 3bd6579). `VehicleSafetyAndControlTask::isCommitted()` is that function on `groundContext()`.
- **Only an accepted `CMD_STATE` keeps it alive:** heartbeats, `LINK_STATS`, a `REQUEST`, frames that are rejected (not node 0, foreign session, stale sequence, out-of-range field) and handset `RC_STATE` never refresh it. The check runs in the 10 Hz safety task, so it fires 2.0 to 2.1 s after the last accepted state; the interlock (1 s) closes the motors earlier.
- **D2-B (not committed: on the ground, motors disarmed, nothing pending):** losing the command stream only sets `ctrl_source` NONE; no failure, nothing latched. A mission started with no accepted state at all (committed with no session) is a failsafe at once; a go needs a session by construction (it comes from its state).
- **GS failsafe request** (design D-GS6): `failsafe = 1` in the last accepted `CMD_STATE` latches a failsafe in **any state** (mode Failure, `failure_bits` bit 14 `GS_FAILSAFE`, `FailureState::gsFailsafe`, `ctrl_flags` `GS_FAILSAFE_REQ` while requested) in the next safety tick (`checkForFailures`, log `Failsafe: GS failsafe requested`). The latch is cleared only by the SE short press, and **the vehicle refuses that reset while the request is still 1** (owner Q6, `RcCommandLogic`: `SE_REFUSED`, log `SE reset refused: ...`); the ground station clears the request first. If the request is set again after a reset it latches again. No simulation dependence.
- **The SE reset** (short press from the ground station's SE / virtual SE) clears the failure (`clearFailures()`); with the stream gone it latches again only if the vehicle is committed again (after a reset it is not).
- **Degraded (information only, clears by itself):** `VEHICLE_STATUS.degraded_bits` bit 2 `UART_HEARTBEAT_LOST` is set while the UART link carries the command state (`ctrl_flags` `UART_FRESH`) and no valid frame from the ship node (3) arrived on the UART for 3 s (`!isUartPeerSeen()`). Bit 0 `TOF_LOST` and bit 1 `LINK_STATS_LOST` stay 0 until T9b-6. Degraded never triggers a failsafe, abort or motor cut.

**Command ingress: the ground station is the only control source (GS-control cutover, `feature/gs-control`, 2026-11-23; design: workspace `docs/gs-control-design.md`).** The vehicle accepts command state **only from node 0** as `CMD_STATE` (port 0x13, 14 B, strict fields) on **both** links (USB on the bench, UART from the ship node in flight), as one stream. The Radiomaster Pocket is an input device of the ground station: its `RC_STATE` (port 0x10) is **not interpreted at all** by the vehicle (the tap only counts it as `rcIgnored()`), the old RADIO/BENCH sources, the node 2 RC path, the 0.5 s handover and `RC_APPLIED` are removed. Owner decisions: no authentication, no separate arm switch, **SD = "motors armed / disarmed"**.
- **`Comms::CmdIngressTap`** (`src/comms/cmd_ingress_tap.*`): one per link, between the `StreamFrameLink` and its `Node` (USB link -> tap -> USB node; UART link -> tap -> `RadioTap` -> UART node). It knows its link by construction (the shared `PortMux` cannot: `RxInfo.ingressLink` is 0 for both nodes). It looks only at DATA frames for the vehicle (dst 1) on ports 0x13 (CMD_STATE), 0x11 (REQUEST, session check only) and 0x10 (counted, ignored). A `CMD_STATE` payload that is not exactly 14 bytes, or whose **strict decode** fails (any field outside its enum, version != 1, session 0, flags without VALID or with bits above 2, reserved != 0), rejects the whole frame (`rejects[0]`). Every frame is forwarded unchanged. Port 0x13 is not subscribed on the `PortMux`, so its sequence state is never shared with other ports.
- **`Safety::CommandIngress`** (`src/safety/command_ingress.*`, pure, host-tested in `test/test_command_ingress`; replaces `ControlSourceTable`): accepted = source node 0 (`rejects[1]` otherwise), then the **session latch before the sequence filter**, then the per-session sequence rule (`(seq - last) mod 256` in 1..127, or the session silent for 1 s; the second copy of a frame on the other link is an exact duplicate: NOT counted as a reject (internal `duplicates` counter), it refreshes the per-link age; the late radio-path copy up to 16 frames behind is likewise not a reject (internal `lateCopies`), only frames older than that count in `rejects[3]`). **Session latch:** the first valid session after boot is latched; another session is latched only while the vehicle is **not committed or in mode Failure** and the latched session has been silent **1 s** (any frame of the session counts, also a duplicate copy); the acceptance resets the mapper and the interlock (`Control source: new GS session <hex>`) before its first state is used. Otherwise a foreign session is rejected (`rejects[2]`) and `ctrl_flags` **MULTI_MASTER** is set for 2 s. The committed/Failure context is fed by the safety task through `VehicleComms::setCommitted(committed, modeFailure)` every run. Consequence for a GS restart: a restarted GS cannot take a committed vehicle; after the control failsafe puts it in Failure (2 s) the new session is accepted and can send the SE reset. A rebooted vehicle has no session and latches whichever session arrives first.
- **Freshness and telemetry:** `ctrl_source` = GS while the latched session's last accepted state is at most 2 s old, else NONE (the session stays latched). `rc_age` = age of the last accepted state. `ctrl_flags` `USB_FRESH` / `UART_FRESH` (bits 7, 8): an accepted or duplicate copy of the latched session on that link within 2 s, **information only** (no link priority, no handover; losing one link while the other is fresh changes nothing). `SOURCE_LATCHED` = committed. `VEHICLE_STATUS.cmd_session` = low 16 bits of the session, `rc_rejected` = wrapping total of rejects. **`CMD_APPLIED`** (0x84, USB only, 10 Hz, 26 B; replaces `RC_APPLIED`): the last accepted fields (go, profile, vehicle_mode, motors, reset_button, failsafe), the info flags (POCKET_LIVE, INPUT_VIRTUAL), the safe-seen bits, the full session, the ages (state, USB, UART) and `rejects[4]` (`[0]` length or field, `[1]` source not node 0, `[2]` foreign session, `[3]` stale sequence, older than the 16-frame late-copy window; duplicates and late copies are not counted).
- **REQUESTs (design Q5):** a vehicle-directed `REQUEST` (target 1) carries the sender's `session`; `CommandIngress::requestAllowed(session)` is true only for the latched, non-zero session (an older sender without the field decodes as 0 and is refused). The tap counts accepted and refused requests (`requestsAccepted()/requestsRefused()`). **The REQUEST executor (T9b-5, part 1) acts on it**, see "REQUESTs (T9b-5)" below; the legacy telecommand on port 0x7F (calibration, simulated wind and faults) still comes from node 0 without a session field: see Known issues. Router and ship-node requests (`LINK_UPLOAD_MODE`, `LINK_REBOOT`, target 2 or 3) are not vehicle-directed and unaffected.
- **`Safety::RcCommandMapper`** (`src/safety/rc_mapper.*`, pure, host-tested; the name is historical): takes the fields of each accepted `CMD_STATE` and keeps the four **safe-seen bits** (GO, SC_REAL = vehicle_mode REAL seen, SD_ENABLE, SE_PRESS), set by an accepted state showing the safe value, cleared by boot, a **session change** and a gap over 2 s between two accepted states. Every field is valid by construction, so there is no "invalid" value any more.
- **Bench check procedure for the owner** (motors electrically disconnected and ESCs unpowered, no props; Teensy on USB with the ground station, Router and ship node optional): see "GS-control bench procedure" at the end of this section.


**Port table:** [../codebase-structure.md](../codebase-structure.md#radio-and-usb-network).

**Wiring (bench-confirmed end to end, 2026-11-20):** the ship node (ESP8285 "RX module", node 3) is wired to **Serial4**: ship node TX (GPIO1) to **Teensy pin 16 (RX4)**, **Teensy pin 17 (TX4)** to ship node RX (GPIO3), 3.3 V, 460800 8N1. Verified with the ExVectrLink Router: it lists node 1 as present and the vehicle telemetry arrives over the radio. History: the module was first found on pads 30/31 (GPIO_EMC_37/36), which are not pins of any Teensy 4.0 hardware UART, so no `SerialN` could read them; the wires were moved to 16/17. During the SX1280 removal the TX pin (17) had been shorted to ground; it was repaired.

**Finding the wiring again (bench tool):** `tools/uart_pin_scan.py` (run with the ground-station venv python, which has pyserial; needs the Router attached) disables the ground-station USB source, flashes the bench-only scan firmware (`pio run -e teensy40_pinscan`, `tools/pinscan/pinscan.cpp`, links no flight code), listens on every UART RX pin and scans all pads for edges, then transmits node 1 heartbeats on candidate TX pins one at a time while polling node 3's `NetworkStatus` through the Router (age[1] about 0 means the ship node heard node 1). It finishes by rebuilding and flashing the flight firmware of the checked-out branch and re-enabling the USB source (`--no-restore` leaves the scan firmware on). `--quick` re-checks only the known candidates (460800 listen, TX on pin 17, bit-bang pad 31). Use it after any rewiring or hardware change near the UART.


**Hard motor interlock (T9b-2, `feature/t9b-2`):** a pure gate at the last point before the motor commands. Design: `docs/t9b-design.md` 3.4, 6.3, 6.6; rules: workspace `docs/safety.md` section 3 and rule 9.
- **`Safety::MotorInterlock`** (`src/safety/motor_interlock.*`, pure, no heap, no simulation input, host-tested with a fake clock: `test/test_motor_interlock`). `open(now)` = an observed DISABLED -> ENABLED transition **and** an accepted **motors = ENABLED** (motors armed, SD) state of the latched ground-station session at most **1 s** old. Fed by the command taps (`CmdIngressTap::onFrame`: every accepted `CMD_STATE`, `onCmdState(motors == ENABLED)`; `onSessionChanged()` first when a new session is latched) and by `VehicleNetwork::pollNodes()` (`update(now)` every 10 ms comms tick). Frames of another source, a foreign session, a stale sequence or with an out-of-range field never reach it (the ingress rejects them), so they count as "no data" and the gate closes by the 1 s age.
- **Observed transition:** two consecutive accepted states, DISABLED then ENABLED, at most 1 s apart. Forgotten (needs a new off -> on) by: boot, a **new session**, a silence or gap of **more than 2 s** (D3-B; a gap of 1 to 2 s only closes the gate and a fresh ENABLED reopens it without a toggle), and the D4-A triggers below. The first state of a session is only the baseline: motors ENABLED from the first state of a session never opens.
- **D4-A re-arm triggers** (`requireRearm`, close at once and need a new toggle even if SD stays ENABLED): chain no longer pending (RTH landed, or the pending chain was cancelled; `handleVehicleMissionControl`), mode **Failure** (every safety-task run while it lasts), the SE reset, an SC change (simulation/real), and the ground "reset and stop" (SA released on the ground, calibration countdown abort). **Not yet available:** the latched abort (an in-flight abort latch arrives with T9b-6); a failure-mode abort already re-arms through Failure.
- **Where it acts:** `StarshipTVC::taskThread` evaluates `Safety::evaluateMotorGate` once per run (interlock open, watchdog forced-off flag, `enableMotors_`, `enableActuators_`, test idle): motor `enableOutput(...)` and the `setValue()` call are conditioned on its result. A null interlock pointer is closed. **`MotorInterlockWatchdog`** (main.cpp, 100 Hz, **priority 120**, above every controller) calls `StarshipTVC::forceMotorsOff()` every run while the gate is closed (both motor outputs duty 0, sticky forced-off flag that `taskThread` also honours, the modelled telemetry thrust is untouched; the simulator copy follows the gate, owner decision 2026-11-23) and `releaseMotorsForce()` while open. This covers the event-driven TVC task (it does not run if the controller stops publishing). Nothing in mission code, the failure handler or RTH can bypass it: they only set `enableMotors_` and `enableActuators_`, which come before the AND.
- **Watchdog-stale failsafe:** `checkForFailures` latches mode Failure with `failure_bits` bit 13 (`interlockWatchdog`) if the watchdog task has not run for **100 ms** (or never ran). Cleared by the SE reset like the other failures.
- **Session latch input:** the safety task now passes `committed = Safety::committed(...)` (interlock open || mission chain pending || mode Running and not on the ground) to `VehicleComms::setCommitted()` every run (it was always false in T9b-1), so the session cannot be taken over while the vehicle is committed (`ctrl_flags` SOURCE_LATCHED).
- **Telemetry:** `VEHICLE_STATUS.flags` **MOTORS_ENABLED** = interlock open **and** `enableMotors_` **and** actuators enabled (TVC task run, i.e. the same AND as the motor output without the watchdog flag). **ARMED = interlock open since T9b-4** (the legacy arm state is gone). `ctrl_flags` **INTERLOCK_NEED_TOGGLE** (bit 0) = the motors field shows ENABLED in a state at most 1 s old but no off -> on transition was observed. `failure_bits` bit 13 as above. The vehicle logs `Motor interlock OPEN ...` and `Motor interlock CLOSED: <reason>` (no GS session / motors armed but needs a toggle / motors disarmed / no valid armed state within 1 s) on every gate change (10 Hz sampling, so a glitch shorter than 100 ms is not logged).
- **Simulation:** the interlock never reads the simulation flag and simulation never changes it; the vehicle boots in real mode. **Owner decision 2026-11-23:** the simulated thrust and TVC forces follow the real gate output (zero while the gate is closed), so disarming the motors (SD) in a simulated flight removes the simulated thrust (failsafe test); the gate itself still never reads the simulation flag. **Since T9b-4 the legacy `enableMotors(!simulationMode_)` gate is gone** (owner decision 2026-11-21, safety.md rule 9): a go enables the motors in simulation exactly as in real mode, so in a bench simulation the motor PWM runs whenever the interlock is open. **Bench rule: the motors are electrically disconnected (ESCs unpowered) for every simulation or bench test; SD is only an additional measure.**

**Command path (T9b-4, input changed by the GS-control cutover): the safety commands are `CMD_STATE` fields.** Design: `docs/t9b-design.md` 3.2, 3.3, 6.4 (the owner decisions of 1.3 win) and workspace `docs/gs-control-design.md`; rules: workspace `docs/safety.md` sections 1 to 4. The telecommands Arm, SimulationMode, MissionBegin, MissionSelect, MissionAbort and SystemReset are **removed** (ignored and counted, `Telecommand N ignored`, at most every 5 s).
- **Layers:** `Safety::RcCommandMapper` (safe-seen bits, **latched edge events**, SE classifier; `rc_mapper.*`) -> `Safety::RcCommandLogic` (pure intent layer: go preconditions and refusal codes, SC decision, D12 "on the ground", SE actions, hold intents; `rc_command.*`) -> `VehicleSafetyAndControlTask::applyRcCommands()` (main.cpp, 10 Hz, first thing in the tick), which applies the intents with the existing vehicle code. The ground station's own pickup rule (design 3.2) makes it send an unsafe value only after the input was seen safe; the vehicle enforces the same rule independently (**two layers**): every unsafe direction needs an **observed** transition in two consecutive accepted states at most 1 s apart, the safe value seen first, and every session change, boot and gap over 2 s forgets it, so **every field combination at boot or at a session start does nothing until that field is cycled**.

| Field (Pocket switch) | Safe direction (acts on the state) | Unsafe direction (observed edge plus preconditions) |
|---|---|---|
| **go** (SA) | NOT_GO while a mission chain is pending or the mode is Running: `Safety::decideAbort` **every tick** (idempotent): on the ground reset and stop (no latch cleared), in flight RTH (kept while RTH runs). A stale stream does not abort (the control failsafe covers it). The GS ABORT button is this level. | NOT_GO -> GO, go still GO when the tick looks at it, and: mode Ready, no latched failsafe, no estimator zeroing window and a valid zero, `vehicle_mode` equal to the applied mode, a known profile (**0 hover test, 1 waypoint test, 2 flight**, `Safety::isKnownProfile`), no calibration session or gyro run in progress. **The interlock is not a precondition** (owner 2026-11-21). Then the calibration gates and the countdown gyro run, then mission begin with the profile, **fixed 15 s countdown**, mode Running, `enableMotors(true)` in simulation and real mode alike. A refused go keeps its reason in `go_refusal` and logs `GO refused: <reason>` (2 s rate limit); it must be cycled to try again. A held go never starts a second one. |
| **profile** (SB) | none | read at the go edge only and latched (`mission_profile`); selects the waypoint list and chain (0 hover test, 1 waypoint test, 2 flight; missions.md section 7a); a value other than 0, 1, 2 refuses the go (`go_refusal` 5, text `profile (SB) not 0, 1 or 2`). |
| **vehicle_mode** (SC; Pocket 0 = REAL, 1 and 2 = SIMULATION) | none | an **observed change of the field** (REAL <-> SIMULATION), only while on the ground (D12), not committed and not in mode Failure; refused otherwise (log line, SC_MISMATCH shows the difference). **The vehicle boots real**; a SIMULATION field at boot or at a session start leaves it real (mismatch, go refused) until the field is cycled. The change runs `setSimulationMode()` and needs a new motors toggle (D4-A). Simulation gates nothing. |
| **motors** (SD, "motors armed / disarmed") | DISABLED or stale: gate closed at once | the interlock section above, unchanged. `ARMED` in the telemetry = the gate is open. The GS MOTORS OFF button is this level. |
| **reset_button** (SE) | none | **short press** (>= 2 pressed states, no gap over 200 ms, released within 1 s of the first pressed state, RELEASED seen first; the GS stretches every press to >= 160 ms): the system reset (`systemResetFromSe()`: mission chain to the boot state, failures **and latches cleared**, Startup and re-init, actuators and motors off, an active zeroing window aborted, simulated body to its start pose in simulation, the interlock needs a new toggle, `mission_profile` back to 255), **only on the ground with the interlock closed** (D12) **and with the GS failsafe request cleared** (owner Q6); refused otherwise (`SE_REFUSED` for 2 s). **Hold** (pressed for 1 s): zeroes the estimators **while held** (`EstimatorZeroing::holdStart`, same conditions as `ZeroEstimators`); continues while fresh pressed states arrive, ends 300 ms after the last one, on release, when the interlock opens (aborted) or 60 s after the start. A valid zero is recorded if it zeroed for at least 1 s; a shorter or aborted hold clears the valid zero and the vehicle returns to Startup (automatic zero). A gap over 200 ms between pressed states cancels a press with no action. |
| **failsafe** | latched failsafe in any state (control failsafe section above) | - |

- **Logs:** `Control source: new GS session <hex>`, `GO: mission begin, profile 2 (flight), countdown 15 s`, `GO refused: ...`, `SA released: on the ground: reset and stop | in flight: start RTH ...`, `Simulation mode ON|OFF (SC)`, `SC change refused ...`, `System reset (SE short press)`, `SE reset refused: only on the ground with the interlock closed and the GS failsafe request cleared`, `Failsafe: GS failsafe requested`, `SE hold: estimator zeroing while held`, `SE hold zeroing ended: valid zero recorded (x s)`, `Telecommand N ignored`. (The log names SA/SC/SE are the Pocket switch names the ground station maps from.)

**GS-control bench procedure for the owner.** **The Pocket is irrelevant to the vehicle: it only feeds the ground station, which sends all commands.** **Motors electrically disconnected (ESCs unpowered) and no props for every item; the simulation does not hold the motors off, "motors armed" is only an additional measure.** Teensy on USB with the ground station (GS command engine on, Pocket optional or the GS virtual panel), and, where noted, the Router and ship node on Serial4. Watch `VEHICLE_STATUS` (`ctrl_source`, `cmd_session`, `ctrl_flags`, `flags`, `failure_bits`), `CMD_APPLIED` (fields, safe-seen bits, ages, rejects) and the console. The GS software must implement the matching command engine (design section 7, step 2c).
  1. **Session start:** start the GS. The log shows `Control source: new GS session <hex>`; `ctrl_source` GS, `cmd_session` = the low 16 bits of the GS session, `rc_age` < 0.1 s, `CMD_APPLIED.session` equal to it, `rejects` not increasing (the second copy over both links is not counted). Start the GS with each Pocket switch in its unsafe position (go up, motors armed, reset pressed, vehicle_mode SIMULATION): the GS must send the safe values, nothing happens on the vehicle (no go, no mode change, no interlock open); cycle each switch: only that one acts.
  2. **Boot and session start with every field combination:** with the GS stopped, reset the Teensy; start a GS with the unsafe combinations. Nothing may happen without an observed edge: no go, no SC change, no reset, no zeroing, interlock closed with `INTERLOCK_NEED_TOGGLE` if the field is armed.
  3. **Full simulated mission:** vehicle_mode REAL -> SIMULATION (SC), profile 2 (SB; 0 and 1 are the test missions, missions.md 7a), motors disarmed -> armed (SD, `Motor interlock OPEN`, `ARMED`), go (SA up, 15 s countdown), hover, go NOT_GO (SA down: RTH), landing, SE short press = reset (the interlock needs a new motors toggle).
  4. **Go refusals:** with one condition at a time wrong (profile above 2: `go_refusal` 5, vehicle_mode not equal to the applied mode: 4, during a zeroing hold: 3, right after boot before Ready: 1, calibration session running: 7) the go is refused; each needs go down and up again.
  5. **Motors armed and stale stream:** gate open, stop the GS (kill the process). About 1 s later `Motor interlock CLOSED: no valid armed state within 1 s`; if committed, 2.0 to 2.1 s after the last state `Failsafe: control source lost`, mode Failure, `FAILSAFE_LATCHED`. Not committed (Ready, nothing pending): only `ctrl_source` NONE after 2 s, no failure. A landed vehicle (mode Running, nothing pending, interlock closed) also only goes NONE.
  6. **GS restart:** after step 5 in Failure, restart the GS: the vehicle accepts the new session (`new GS session`), the SE short press from the new session resets (with the GS failsafe request 0). Restart the GS on the idle uncommitted vehicle: the new session is latched after 1 s of silence, every field needs its edge again. Restart the GS while committed and before the 2 s failsafe: the new session is rejected (`rejects[2]`, `MULTI_MASTER`) until the failsafe has latched.
  7. **Two links:** GS on USB and Router at once: nothing changes when either link is unplugged (`USB_FRESH` or `UART_FRESH` drops after 2 s, `ctrl_source` stays GS); with both unplugged the 1 s / 2 s behaviour of step 5.
  8. **Second GS instance (another PC or a script with another session):** `MULTI_MASTER` set, `rejects[2]` rising, no effect on the vehicle, the first GS's `cmd_session` still reported.
  9. **FAILSAFE, ABORT and MOTORS OFF buttons** (GS step 3b): ABORT in the countdown, ascent and descent (go NOT_GO acts at once); FAILSAFE (`failsafe = 1`, motors DISABLED, go NOT_GO) in each phase: `Failsafe: GS failsafe requested`, mode Failure, `failure_bits` bit 14, `GS_FAILSAFE_REQ`; with the request still 1 the SE short press is refused (`SE reset refused`); clear the request in the GS, SE short press resets. If the request is set again the latch comes back. MOTORS OFF: `Motor interlock CLOSED: motors disarmed` at once.
  10. **Removed paths:** with the Pocket on and connected to the Router: nothing from node 2 reaches a decision (`rc_ignored` is not telemetry; `ctrl_source` stays NONE without a GS); an `RC_STATE` frame never changes any `CMD_APPLIED` field. Old telecommands Arm / SimulationMode / MissionBegin / MissionSelect / MissionAbort / SystemReset only log `Telecommand N ignored`.
  11. **Latency (design section 9, item 9):** SD flip to `Motor interlock CLOSED` below 100 ms over 20 flips (GS timestamps against the log).


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
| | `ctrl_source` | the control source (GS-control cutover): 0 NONE, 1 GS (a session is latched and its last accepted `CMD_STATE` is at most 2 s old). Old values 1 RADIO and 2 USB are retired. `Safety::CommandIngress` |
| | `flags` | ARMED = the motor interlock is open (**T9b-4: the legacy `vehicleArmed_` and the `Arm` telecommand are gone**, `VehicleSafetyAndControlTask::isVehicleArmed()` = `MotorInterlock::open(now)`); SIMULATION (set by the `vehicle_mode` field, section 5 "Command path"); MOTORS_ENABLED = `StarshipTVC` motors enabled **and** actuators enabled **and** the hard motor interlock open (T9b-2, `starshipTVC.interlockOpen(now)`; the motors field of the latched GS session, section 5); GS_PRESENT = ground-station presence (section 5; information only, not the failure condition); INCOMPATIBLE; FAILSAFE = mode is Failure (actuators cut); ZEROING (bit 7) = an estimator zeroing window is active (`VehicleSafetyAndControlTask::estimatorZeroingActive()`: boot zero or `ZeroEstimators`; the mission's own Initialisation zeroing does not set it); UART_PEER_SEEN (bit 6) = a valid frame from the ship node (node 3) arrived on the UART within 3 s (`VehicleNetwork::isUartPeerSeen()`, from the UART node's node table, so USB frames and other nodes never set it). **Information only:** nothing in the vehicle reads it (safety.md 4.1). It exists because `NetworkStatus` seen over USB reflects only the USB node, so a dead UART is otherwise invisible |
| | `failure_bits` | bit 0 sensor, 1 attitude, 2 position, 3 position out of bounds, 4 attitude out of bounds, 5 `CONTROL_SOURCE_LOST` (T9b-3: the latched session's accepted `CMD_STATE` older than 2 s while committed; was `radioConnectionLoss`). The schema also reserves bits 6-13 (T9b-6: 6 GNSS, 7 magnetometer, 8 baro, 9 gyro, 10 accel, 11 attitude estimate, 12 position estimate); **bit 13 (interlock watchdog) is set since T9b-2** (the 100 Hz motor-interlock watchdog did not run for 100 ms, latches Failure); **bit 14 `GS_FAILSAFE`** (GS-control: an accepted `CMD_STATE` carried failsafe = 1; latched, cleared by the SE reset while the request is 0); the vehicle never sets bits 6-12 yet |
| | `sensor_state` | 2 bits per sensor, `TelemetrySensor` (0 Init, 1 Calib, 2 Ready, 3 Failure): bits 0-1 GNSS, 2-3 IMU, 4-5 baro, 6-7 mag, 8-9 attitude KF, 10-11 position KF |
| | `last_req_id`, `last_req_status` | 0, 0 (the executor keeps `lastReqId()/lastStatus()`, not yet mapped into the telemetry) |
| | `rc_age` | age of the last accepted `CMD_STATE` (either link), s; NaN when none yet (name kept from `RC_STATE`) |
| | `ctrl_flags` | T9b-2 sets INTERLOCK_NEED_TOGGLE (bit 0: the motors field shows ENABLED in a state at most 1 s old but no off -> on transition was observed; the operator must disarm and arm again), USB_FRESH (bit 7), UART_FRESH (bit 8, an accepted or duplicate copy of the latched session on that link at most 2 s old, information only), SOURCE_LATCHED (bit 9: the vehicle is committed, so the session cannot be taken over), MULTI_MASTER (bit 11: a foreign-session `CMD_STATE` was rejected within 2 s) and GS_FAILSAFE_REQ (bit 12: the last accepted `CMD_STATE` carries failsafe = 1); **T9b-4** adds the bits the vehicle logic owns (`TelemetrySnapshot::cmdFlags`, OR-ed in by `makeVehicleStatus`): SC_MISMATCH (bit 4: the `vehicle_mode` field differs from the applied mode), SE_HOLD_ZEROING (bit 5: an SE hold is zeroing the estimators), SE_REFUSED (bit 6: an SE press or hold was refused, shown 2 s), FAILSAFE_LATCHED (bit 2: mode Failure, latched until the SE reset). ABORT_LATCHED (bit 1), NOPOS_RTH (bit 3) and REQ_QUEUE_FULL (bit 10) stay 0 until T9b-6/7/5 |
| | `rc_rejected` | wrapping u8 count of rejected `CMD_STATE` frames, all reasons, both links (duplicate and late copies of a frame that arrives on both links are not counted) |
| | `cmd_session` | low 16 bits of the latched `CMD_STATE` session, 0 = none; the ground station compares it with its own (NOT MASTER if different). Schema hash 0xF1941A87, payload 38 B |
| | `go_refusal`, `degraded_bits`, `mission_profile` | `degraded_bits`: bit 2 `UART_HEARTBEAT_LOST` since T9b-3 (info only, see section 5), else 0; **T9b-4:** `go_refusal` = the reason the last go edge was refused (0 none, 1 vehicle not Ready or calibration invalid, 2 latched failsafe (mode Failure), 3 zeroing window or no valid zero, 4 SC invalid / reserved / not equal to the applied mode, 5 profile other than 0, 1, 2 (SB: 0 hover test, 1 waypoint test, 2 flight), 6 reserved (the interlock is not a precondition), 7 calibration session or gyro run in progress); kept until the next go is accepted or the SE reset. `mission_profile` = the SB position latched at the last accepted go (255 none, reset by the SE reset). Appended fields of the schema hash 0x65E4E6E3 (payload 36 B); real values arrive with T9b-4 to T9b-8 (definitions: library `docs/packet-reference.md`, design `docs/t9b-design.md` 3.8) |
| `CMD_APPLIED` (0x84, USB only, 10 Hz, 26 B, tick phase 5; replaces `RC_APPLIED`) | `go`, `profile`, `vehicle_mode`, `motors`, `reset_button`, `failsafe` | the last accepted `CMD_STATE` fields (all safe/0 with no session; `vehicle_mode` is the field, not the applied mode) |
| | `flags` | POCKET_LIVE (bit 0), INPUT_VIRTUAL (bit 1) of the last accepted `CMD_STATE` (reported, never acted on) |
| | `session` | the full latched session, 0 = none |
| | `age`, `usb_age`, `uart_age` | s since the last accepted state, and since the last accepted, duplicate or late (up to 16 frames behind, the radio path trails USB by tens of ms) copy of the latched session on the USB / UART link; NaN = none |
| | `safe_seen` | the mapper's four safe-seen bits (0 with none): bit 0 GO, 1 SC_REAL (vehicle_mode REAL seen), 2 SD_ENABLE, 3 SE_PRESS |
| | `rejects[4]` | wrapping reject counters per reason (see section 5): `[0]` bad length or field out of range, `[1]` source not node 0, `[2]` foreign session, `[3]` stale or duplicate sequence. Never sent on the UART (port >= 0x80). Bench procedure: section 5 |
| | `home_epoch` | `Comms::HomeEpoch` in `VehicleSafetyAndControlTask`: **0 until the vehicle has a position reference**, then a counter that increments each time the reference is re-set (first value 1, wraps 255 -> 1, never back to 0). The reference is the GNSS reference point `gnssRefPos_` of the position KF, which only moves while the estimators are in zeroing mode, so `referenceSet()` is called on the **falling edge of the applied zeroing mode** in `applyEstimatorZeroing()`: end of the boot zero, of a `ZeroEstimators` window, of the mission's Initialisation zeroing, and of a window aborted by SystemReset or a calibration start (that window also moved the reference; an extra increment only makes the ground station request HOME_POSITION once more). `SystemReset` without an active window does not change it (the estimators keep their origin). `HOME_POSITION` (0x50) itself is not sent yet (the request channel comes later). Telemetry only |
| | `calib_state` | bit0 gyro valid, bit1 accel valid, bit2 mag valid (`Calibration::Validity` per sensor, `src/calibration/calibration_validity.hpp`, `Reason::Ok`), bit3 a calibration session is active (`CalibrationService::anyBusy()`, any sensor collecting or fitting, or the automatic gyro run), bit4 ready to arm = `Calibration::readyToArm()` = mode Ready, no zeroing window and `decideArm() == None` (the same gate function the Arm command uses; Arm is only useful in Ready because the task disarms outside Ready), bits 5-7 zero. Cached by the safety task every 100 ms run (`updateTelemetryCache()`), so it lags a mode change by up to 0.1 s; the telemetry task does not evaluate the validity. Same in simulation mode (safety.md rule 9: telemetry only, no gate reads it) |
| `POSE` | `omega`, `q`, `vel`, `pos` | attitude state `[ω, q]`, position state `[V, P]` (f32) |
| | `ground_distance` | downward lidar distance, m (f32, last member): ground topic switch `data(1)` (KF state `[rate, distance]`); NaN before the first sample and when the sample is older than 100 ms. Moved here from `ACTUATORS` on 2026-11-26 (a sensor, not an actuator; schema hash 0x93EF9C5F). Read-only telemetry, no consumer reads it (`main.cpp` `TelemetryProvider`, `TelemetrySnapshot::groundDistance`) |
| `AIR_POSE` | `q` (i16, 1/32767), `pos` (**i32, 0.001 m = 1 mm**, ±2147483 m; was i16 0.1 m until 2026-11-20), `vel` (i16, 0.01 m/s, ±327 m/s) | as `POSE`, fixed point, saturating; 32 byte payload (30 before `ground_distance`). USB `POSE` is f32 and unchanged |
| | `ground_distance` (u16, 1 mm, 0..65.534 m) | as `POSE.ground_distance`; NaN = wire 65535. Optional member at the end (older senders decode as NaN) |
| `ACTUATORS` | `flaps` | true flap angles `[TL, TR, BL, BR]`, rad. **Modelled, not measured**: the output of the servo model in `StarshipFlaps` (300 °/s slew; in simulation also the injected faults); no position feedback exists. |
| | `tvc` | `[atan2(Fx, Fz), atan2(Fy, Fz)]` of the actual thrust vector (body frame), rad. **Modelled, not measured**: servo-space slew model (20 °/s on the thrust angle) in `StarshipTVC`, not the measured vane position. |
| | `thrust` | `|[Fx, Fy, Fz]|`, N |
| | `roll_torque` | the actual thrust topic's `T` (`s.tvcThrust[3]`), f32 N·m: the same source and definition as `AIR_ACTUATORS.roll_torque`, unquantised |
| `AIR_ACTUATORS` | `t_us`, `prev_age` | time of the newest sample; newest minus previous sample time, s (nominally 0.020; NaN with no previous sample) |
| | `tvc[4]`, `roll_torque[2]`, `thrust[2]`, `flaps[8]` | newest first, then previous: same definitions as `ACTUATORS` (`tvc` = `[atan2(Fx,Fz), atan2(Fy,Fz)]` of the actual thrust, `thrust` = its magnitude, `flaps` = true angles `[TL,TR,BL,BR]`), plus `roll_torque` = the actual thrust topic's `T` (the twist actuation as roll torque, N·m; the vane twist fraction is `T·45/thrust`, limited to ±0.5). Fixed point: angles 0.1 mrad, torque 0.01 mN·m, thrust 1 mN, saturating, NaN -> sentinel. Samples are taken every 20 ms (`VehicleNetwork::kActuatorSampleTicks`) into `ActuatorHistory` (4 entries) and the packet is sent at 25 Hz on a sampling tick, so the previous sample is exactly the one before the newest. The values are the modelled actual output (commands after the mixer limits and the rate model), not servo feedback; the vehicle has no servo feedback |
| `AIR_GNSS` | `latitude`, `longitude`, `altitude`, `vel` | the u-blox driver's values as `GNSS` (radians, m above the ellipsoid, NWU m/s). Wire: lat/lon i32 1e-7 deg, altitude i32 mm, velocity i16 0.01 m/s, done by the generated codec; sentinels decode as NaN. Before the first GNSS measurement (`gnssSubr_` timestamp 0) every value is NaN. An invalid fix still sends the receiver's values; `AIR_GNSS_INFO.fix_type` says whether to trust them. 10 Hz on the UART only (USB has `GNSS`) |
| `AIR_GNSS_INFO` | `fix_type`, `satellites`, `h_acc`, `v_acc` | the REAL UBX-NAV-PVT `fixType` (`UbloxSerialGNSS::fixType()`, cached in `_getData()`), numSV (`getSIV()`), `positionCov(0)` and `positionCov(2)` (from `getHorizontalAccEst()` / `getVerticalAccEst()` mm -> m; u16 in 1 cm, NaN -> 65535 = unknown, NaN before the first measurement). 1 Hz on the UART |
| `AIR_MISSION` | `mission_time`, `mission_mode`, `phase`, `index`, `setpoint_pos`, `setpoint_vel`, `setpoint_q` | as `MISSION` (+ `mission_mode` as in `VEHICLE_STATUS`), fixed point: time 1 ms, position 1 mm, velocity 0.01 m/s, `setpoint_q` i16 `limit = 1.0` as `AIR_POSE.q` (raw = round(v * 32767), 0 stays exactly 0) |
| | `waypoint` | `missionWaypointTask.currentWaypointIndex()` while the waypoint mission is active, else 255 |
| `MISSION` and `AIR_MISSION` | `mission_elapsed` (added 2026-11-26, `feature/mission-elapsed`, schema hash 0xF49ACD01; fixed i32 at 1 ms, INT32_MIN = NaN; MISSION 64 B, AIR_MISSION 46 B) | Seconds since T0 of the whole mission (owner decision 2026-11-26), from `Safety::MissionClock` (`src/safety/mission_clock.hpp`, [missions.md](missions.md#whole-mission-clock-mission_elapsed)): negative during the countdown, 0 at T0, continuous across all mission steps (unlike `mission_time`, which restarts per step), held after the chain ends, NaN before the first begin and after a reset to Idle. Telemetry only. |
| `MISSION` | `setpoint_pos`, `setpoint_vel` | `positionSetpoint[3..5]`, `[0..2]` |
| | `mission_time` | `missionTime` ns -> s (1 ms wire resolution; negative in the countdown) |
| | `index`, `phase` | `MissionState.missionIndex` (always 0, see missions.md), current `MissionPhase` |
| | `mission_mode`, `waypoint` | exactly as `AIR_MISSION` (`MissionState.missionMode`; waypoint index of the waypoint mission, 255 = another mission is active) |
| `MISSION` and `AIR_MISSION` | `stop_altitude`, `flip_start_altitude` (added 2026-11-23, `feature/flip-start`, schema hash 0x0E1E8CAB; MISSION f32 m, AIR_MISSION i16 at 0.1 m, -32768 = NaN) | `stop_altitude` = `MissionFreefall::getStopTargetAltitude()`, always sent (default 20 m, the echo of SetStopAltitude). `flip_start_altitude` = `MissionFreefall::getStopAltitude()` = `stopDist(Vz) + stopAlt`, computed from the same position estimate and `calculateStoppingDistance` the stop trigger compares `Pz` against, evaluated at each telemetry snapshot; sent only while the freefall mission is the active mission and running (not after the trigger fired), NaN otherwise |
| | `setpoint_q` (also in `AIR_MISSION`) | Attitude setpoint of the **currently active attitude controller**, `[w,x,y,z]` body -> world, NWU (as `POSE.q`), unit with w >= 0; **all four 0 = no attitude setpoint**. Pure mapping `Comms::attitudeSetpoint()` (telemetry_mapping.hpp, host-tested). Selection by the controller phase table as applied (`Safety::controlPhaseFor`): (1) bellyflop set on (`bellyflopFlapsActive()`, freefall and the bellyflop after its kick) **and** the flap setting on `bellyFlopControlTopic` is `BellyFlopMode_Stabilize` or, since S3, `BellyFlopMode_Descent` (FlapModeObserverTask: vehicle and mission Running) -> flap controller. **In Descent** (default build) the quaternion is the belly-down attitude at the guidance's **heading setpoint** (`Safety::flapSetpointAngles`: azimuth = `DescentStatus.headingSp_rad` from `controlAttitudeFlaps.getDescentStatus()`, pitch 0, then `flapPointingSetpoint`); the **pitch split is not transmitted** (no schema change: the GS derives it from the actual flap angles in `ACTUATORS` / `AIR_ACTUATORS`, neutral 30/78 = split 0; S4 labels it in the GS). In the legacy Stabilize build: the setting's azimuth/pitch as before; (2) else TVC set on (`tvcControlActive()`, waypoint/RTH Running with actuators, and the bellyflop kick) -> during the kick (`MissionBellyflop::kickActive()`, input `tvcOverride`) the kick target the mission writes into the controller (`kickSetpoint()`), otherwise the TVC attitude controller's setpoint topic `controlMappingAccToAtt.getAttitudeTopic()` (the topic `controlAttitudeTvc.subscribeAttitudeSetpoint` is wired to, main.cpp setup), q part `[3..6]`, normalised. The phase table makes (1) and (2) exclusive since 2026-11-21; (3) else zero (idle, ground, countdown, Finished, RTH stabilisation with flaps Upright). The flap controller (ControlAttitudeFlaps, Stabilize) has no quaternion setpoint: it tracks only the direction of body +Z (`azimuthAngle_Rad` = atan2(z_W, z_N), `pitchAngle_Rad` = elevation of body +Z), roll about body +Z is uncontrolled. The published q is the current attitude rotated by the shortest world-frame rotation that puts body +Z on that direction (`flapPointingSetpoint`), so its roll follows the real vehicle. Non-finite or near-zero inputs give 0. Read through extra `Simple_Subscriber`s on the same topics and two const getters: no controller input, subscription or timing changes. Bench (simulation, full chain, 2026-11-21, `tools/bench_vehicle_usb.py`): 0 in Startup, Ready, countdown (Init, Startup) and after the RTH landing; unit (norm 1 +- 6e-5) in waypoint hover/ascent, bellyflop, freefall and RTH; control phase log lines, bellyflop time-limit end (1.01 s) and RTH auto-disarm as before. Bench 2026-11-21 (bellyflop kick, `feature/bellyflop-kick`): the kick target `[0.707, -0.001, -0.707, -0.001]` for the whole kick, then the flap setpoint; unit in every bellyflop and freefall sample after the `flapPointingSetpoint` fix (Known issues). |
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

## 10. REQUESTs (the legacy telecommand port 0x7F is DELETED)

**`feature/requests-all` (2026-11-23; needs ExVectrPackets 0284392, hash 0x036F9C45, local unpushed: not mergeable until Packets is pushed).** Everything the ground station asks for besides `CMD_STATE` is a `REQUEST` (port 0x11, dst 1). The old `Telecommand` on port 0x7F, its decoder (`LegacyTelecommandPacket`), the `TelecommandSinkI`, the telecommand topic and `src/telecommand.hpp` are removed; nothing subscribes to port 0x7F, so frames on it are ignored (no log, no counter). The old 0x57 magic is gone. The safety commands (arm, simulation switch, mission begin/abort, reset) were already `CMD_STATE` fields (section 5).

`Safety::RequestExecutor` (`src/safety/request_executor.*`, pure, host-tested in `test/test_request_executor`) decides every request that `CmdIngressTap` queues in the 8-entry `RequestInbox` (a full inbox drops the new request and counts it; the sender retries). The safety task drains it every run (`handleRequests()`, at most 8 per run, never blocks), and each copy gets a `REQUEST_ACK` (port 0x12) to the sender node on the **link the copy arrived on** (`VehicleNetwork::sendRequestAck`). Idempotent by (source node, `req_id`) over the last 8 requests: a duplicate copy (same frame on both links, a retry) is not executed again and is re-acknowledged with the stored result; the 9th distinct request evicts the oldest. Refusals are stored too, so **a sender must use a new `req_id` after a refusal**. Checks, first failing wins: target must be 1 (`REJECTED_INVALID` 3, detail 2); action known (7, 8, 12..23; 0..6, 9, 10, 11 and anything else: 3, detail 3); `session` non-zero and equal to the latched `CMD_STATE` session (`REJECTED_STATE` 2, detail 3); then the action's rules. `arg_i` is i32, `arg_f` f32.

| # | Action | Arguments | Rules (refusal) | Effect |
|---|---|---|---|---|
| 7 | `CALIB_APPLY` | `arg_i` subsystem 0 gyro, 1 accel, 2 mag (else INVALID 1) | calibration gates (below) and a candidate with verdict PASS or WARN, else REJECTED_STATE detail 6 | Writes the legacy key, applies the transform (RAM only until SAVE_CONFIG), invalidates the estimator zero (Ready to Startup) |
| 8 | `SAVE_CONFIG` | args ignored | calibration gates, no session collecting, EEPROM read-back equal, else detail 6 | EEPROM from the RAM image (all keys), read back and compared |
| 12 | `ZERO_ESTIMATORS` | `arg_f` window s (NaN or 0: 10 s; clamped 1..60) | on the ground (not committed, interlock closed) else 2/1; zeroing refusal 2/2 | `EstimatorZeroing::request` |
| 13 | `SET_STOP_ALTITUDE` | `arg_f` m, finite and >= 1 | on the ground else 2/1; range 3/1 | `MissionFreefall::setStopTargetAltitude`, echoed in `MISSION.stop_altitude` |
| 14 | `CALIB_CONTROL` | `arg_i` = subsystem*10 + code; code 1 start, 2 verify, 0 stop and fit, 9 discard (11, 12, 10, 19, 21, 22, 20, 29 and so on; others INVALID 1) | calibration gates and **simulation OFF**, and the service accepting it (not busy, collecting for stop): else REJECTED_STATE detail 6 | Same code as the old `CalibStart`: TVC actuators off, mode `SensorCalib`, aborts a zeroing window and clears the valid zero. [calibration.md](calibration.md) |
| 15 | `SIM_WIND_SPEED` | `arg_f` m/s, clamped 0..20 | simulation mode only (else REJECTED_STATE detail 5); non-finite INVALID 1 | Steady wind of the body simulator. Reset to 0 whenever simulation switches on or off; the SE reset keeps it |
| 16 | `SIM_WIND_DIRECTION` | `arg_f` rad the air moves **toward**, CCW from North toward West (NWU), wrapped to -pi..pi | as 15 | Wind direction |
| 17 | `SIM_GUST_STRENGTH` | `arg_f` m/s, clamped 0..10 (0 = none) | as 15 | Gust peak (period and seed unchanged) |
| 18 | `SIM_GUST_PERIOD` | `arg_f` s, clamped 0.5..30 | as 15 | Gust timescale (default 5 s) |
| 19 | `SIM_GUST_SEED` | `arg_f` integer 0..16777215, rounded, clamped | as 15 | Gust seed (default 1); same seed and settings give the same series |
| 20 | `SIM_FLAP_STUCK` | `arg_i` flap 0..3 exactly (0 TL, 1 TR, 2 BL, 3 BR, else INVALID 1); `arg_f` rad clamped 0..pi/2, **-1 = hold the true angle it has when applied** | as 15 | Immediately (next flap tick) the flap's modelled true angle is held; the PWM follows the faulted angle in simulation |
| 21 | `SIM_FLAP_SLOW` | `arg_i` flap 0..3; `arg_f` rate factor clamped 0.05..1 | as 15 | Scales the modelled servo slew rate (300 deg/s) and the PWM slew |
| 22 | `SIM_FAULT_CLEAR` | args ignored | as 15 | Clears every flap fault. Faults are also cleared on every simulation switch and by the SE reset |
| 23 | `GET_HOME_POSITION` | none | no position reference yet (`home_epoch` 0): REJECTED_STATE detail 4 and **no packet** | ACK DONE, then `HOME_POSITION` (0x50, 25 B) on the same link: `flags` (LATLON_VALID and ALT_VALID when the reference holds a GNSS position, SIMULATION, ZEROING), `latitude`/`longitude` rad and `altitude` m of the position estimator's reference (`gnssRefPos_`, the NWU origin), `h_acc` NaN, `epoch` = `home_epoch`, `set_time` = vehicle time of the last reference set |
| 24 | `SET_MISSION_PARAM` | `arg_i` id 1..10, `arg_f` value | on the ground (not committed, interlock closed) else 2/1; unknown id or non-finite / out-of-range value 3/1; session 2/3 | `Safety::MissionParamSet` (`src/safety/mission_params.*`), idempotent, RAM only (defaults after a reboot). Ids (default, range): 1 ascent_speed m/s (7, 0.5..10), 2 apogee_altitude m (120, 2..1000), 3 hover_hold_s (30, 0..600), 4 p1_hold_1_s (10), 5 p1_hold_2_s (15), 6 p1_hold_3_s (15), all hold times 0..600 s, 7 ascent_accel m/s^2 (3, 0.2..20), 8 ascent_decel m/s^2 (2, 0.2..20), 9 apogee_x m north of the position reference (0, -2000..2000), 10 apogee_y m WEST (+Y = west in NWU; 0, -2000..2000). Read at the go edge by `configureWaypointChain`; a change after go is refused (ground only). Echoed in `MISSION_PARAMS` |

Gates. **Calibration gates (7, 8, 14):** initialised, disarmed, motors off, mode Startup/Ready/SensorCalib, mission Idle or Finished (`Calibration::checkCommand`); any failure is REJECTED_STATE detail 6, the console has the reason. They are not tied to "on the ground" otherwise. **15..22** work only in simulation mode and are not tied to the ground or the interlock (wind tuning in a simulated flight); the simulation check comes before the argument check, so a request outside simulation is always 2/5. The flap fault stays inert outside simulation (second guard in `SimFlapFaults`/`StarshipFlaps`). 12 and 13 need the ground (the old rule). 23 and the 15..22 requests are accepted in flight; only the simulation ones change physics, and only the simulated world.

**Owner bench procedure (vehicle on the ground, GS with the session latched, a new `req_id` per request):**
1. Real mode, Ready: `CALIB_CONTROL` 1 (gyro start): DONE, mode `SensorCalib`; `CALIB_CONTROL` 0: DONE, fit logged; `CALIB_CONTROL` 9 discards. `CALIB_CONTROL` 5 or 3: INVALID 1. `CALIB_APPLY` 0 without a passing candidate: REJECTED_STATE 6; with one: DONE, `calib_state` updates. `SAVE_CONFIG`: DONE, `saved ... verified` in the log. Motors armed (SD enabled): all three REJECTED_STATE 6. Switch to simulation (SC): `CALIB_CONTROL` REJECTED_STATE 6.
2. Simulation mode: `SIM_WIND_SPEED` 5, `SIM_WIND_DIRECTION` 1.57, `SIM_GUST_STRENGTH` 3: each DONE, log `Sim wind` / `Sim gusts`, the simulated drift and attitude respond in `AIR_POSE`. `SIM_WIND_SPEED` NaN: INVALID 1. Real mode: all 15..22 REJECTED_STATE 5.
3. Simulation mode: `SIM_FLAP_STUCK` flap 2, -1 (hold): DONE, `ACTUATORS` flap 2 stops following the command; `SIM_FLAP_SLOW` flap 1, 0.25; `SIM_FLAP_STUCK` flap 4: INVALID 1; `SIM_FAULT_CLEAR`: DONE, the flap snaps back.
4. After the boot zero (`home_epoch` not 0): `GET_HOME_POSITION`: ACK DONE and a `HOME_POSITION` packet whose `epoch` equals `VEHICLE_STATUS.home_epoch` and whose lat/lon match the `GNSS` packet at the reference. Before any reference (just after boot, `home_epoch` 0): REJECTED_STATE detail 4, no packet.

**MISSION_PARAMS (0x26, 40 B, `feature/mission-params-2`, needs ExVectrPackets d7ca411, hash 0xC2061E5D, local unpushed: not mergeable until Packets is pushed).** Ten f32 in the order ascent_speed, apogee_altitude, hover_hold_s, p1_hold_1_s, p1_hold_2_s, p1_hold_3_s, ascent_accel, ascent_decel, apogee_x, apogee_y, read from the store (`makeMissionParams`). USB 1 Hz (schema rate, tick phase 19), radio every 5 s (0.2 Hz, `VehicleNetwork::kMissionParamsAirPeriodTicks`, tick phase 21; `rate_air` cannot express it). An apogee_altitude below 90 m (`kMinKickAltitudeM`) ends the flight in RTH without the bellyflop (the chain rule "a kick below 90 m is an abort" is unchanged); a warning line is logged at mission begin. A non-zero apogee X/Y (> 0.5 m) logs `apogee not above home: the descent target is home, expect a lateral transfer in the descent`: the hover check, RTH home and the freefall/kick assumption (vehicle returns above home) are NOT changed.

Owner bench procedure (simulation): set each parameter (`SET_MISSION_PARAM`), watch `MISSION_PARAMS` echo it; with the interlock closed run profile 0 (hover hold = hover_hold_s) and profile 1 (three holds = p1_hold_1..3_s) and time the holds; refused after go (REJECTED_STATE 1); set apogee_altitude 30 m, go with profile 2 in simulation: the log shows `apogee below 90 m: no bellyflop, the flight ends in RTH`.
5. Resend a `req_id` (also on both links at once): the ACK repeats, the action runs once. Wrong session or 0: detail 3. Action 9 or 10: INVALID 3. Frames on port 0x7F do nothing.

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
| Medium | `Safety::CommandIngress` session latch | **GS control:** a rebooted vehicle has no session and latches the first valid one, and an idle uncommitted vehicle accepts a new session after 1 s of silence (design R5, no authentication, owner decision). A spoofer can take an idle vehicle but then still needs an observed edge for every unsafe action. | Accepted by the owner (Q4: no authentication) |
| Low | `Safety::CommandIngress` session silence | **GS control:** "silent" is measured from the last accepted or exactly-duplicate frame of the latched session on either link; a stale (replayed) sequence of the latched session does not count. A session whose sequence restarts after 1 s of silence is resynced (the transport rule), also for a replayed frame. | Matches the design text; noted for the bench review |
| Info | `Safety::RcCommandMapper` SC edge | **GS control:** the SC edge is an observed change of `vehicle_mode` in either direction (REAL 0 / SIMULATION 1); the SC_REAL safe-seen bit means "REAL seen" and is no longer a precondition of an edge (a change itself proves the previous value). The Pocket's SC positions 1 and 2 are both SIMULATION (mapped by the ground station), so 1 <-> 2 is no change. | None |
| Info | main.cpp `applyRcCommands` | **GS control:** the go refusal text for SC and SB changed (no "invalid" values exist); `go_refusal` codes are unchanged (4 vehicle_mode not equal to the applied mode, 5 profile not implemented). | None |
| Info | `starship_hardware.h` `RX_MODULE_SERIALPORT` | Resolved 2026-11-20: the ship node is wired to Serial4 (pin 16 RX4 / pin 17 TX4), confirmed end to end (Router shows node 1, vehicle telemetry arrives over the radio). It was first on pads 30/31 and was moved. | None. If the harness is rewired, re-check with `tools/uart_pin_scan.py` (section 5). |
| Info | main.cpp `checkForFailures`, section 5 | **T9b-3 (resolved, input changed by the GS-control cutover):** the former rule (any node 0/2 data frame, 10 s boot grace) is replaced by the control-source rule (accepted `RC_STATE` of the latched source older than 2 s while committed, bit 5 `CONTROL_SOURCE_LOST`). A GS `REQUEST` or telecommand no longer keeps it alive. Verified on the host only (fake clock); the Pocket-off cases are in the T9b-3 bench procedure. | A vehicle committed with no source (mission started over USB without RC) fails at once. `LINK_STATS_LOST` and `TOF_LOST` degraded bits are not set yet (T9b-6). |
| High | `src/main.cpp` `MotorInterlockWatchdog`, `checkForFailures` | **T9b-2:** a watchdog that has not run for 100 ms latches Failure (bit 13), which cuts all actuators. The scheduler is cooperative: a long task run (EKF, calibration, flash write) delaying the 100 Hz task by more than 100 ms would make the vehicle fall in flight. Not measured yet. | Bench: watch for `failure_bits` bit 13 under full load before the next flight. |
| Medium | `lib/ExVectrActuator` `PWM_Output` | `outputEnabled_` is never assigned: `enableOutput(false)` only writes duty 0 once per call and a later `setValue()` drives the pin again. T9b-2 works around it (a closed gate never calls `setValue`, and `forceMotorsOff()` writes duty 0 every 10 ms), the library is not fixed. | The interlock's safety depends on no other code calling `setValue` on the motor outputs. |
| Fixed T9b-4 | main.cpp | (was Medium, owner decision safety.md rule 9) the legacy `enableMotors(!simulationMode_)` simulation motor gate is removed. | **A simulation now drives the motors exactly as in flight when a go is accepted and SD is on:** disconnect the motors for every simulation or bench test. |
| Info | `Safety::kGoCountdownParamInt`, `Calibration::GyroRunKind` | **T9b-4 (owner decision 2026-11-22):** the go countdown is the fixed 15 s of D8; the Initialisation phase is 5 s and the countdown gyro run uses a **3 s still window** (3 consecutive good 1 s blocks, deadline 4 s) instead of 5 s, every judging limit unchanged. Startup/boot and `ZeroEstimators` runs keep the 5 s window. | The 3 s bias estimate averages fewer samples (noise of the mean about 1.3 times that of 5 s); the bias limit is unchanged. Bench-verify with the countdown gyro run log lines. |
| Fixed (committed fix) | main.cpp `isCommitted`, `Safety::committed` | (was High, bench) "committed" counted every vehicle in mode Running, so after a landed mission (simulation or real, mode stays Running after RTH) a radio gap on the ground latched the control failsafe (Failure). Now committed = interlock open or chain pending or (Running and not on the ground, the D12 rule), one function for the latch and the failsafe. A flying vehicle, a countdown and an open interlock stay committed. | Host tests in test_rc_mapper and test_control_source; bench case A2. |
| Fixed | main.cpp `groundContext`, `applyAbort`, chain tick | 2026-11-23 regression: reading `posEstSubr.getItem()` there cleared the new flag that `systemsInitialised()` needs, so the vehicle stayed in Startup (`Position data not new`). These reads now use `posPeekSubr_`. Rule in codebase-structure.md (conventions). | Audited all `isDataNew()` users; test_subscriber_newflag documents the rule. Needs a boot check on the vehicle. |
| Fixed | `Safety::onGround`, `Safety::GroundRest` (`src/safety/ground_rest.hpp`) | 2026-11-23: D12 also counts a vehicle with the interlock closed, motors not enabled, no chain pending and physically at rest at ground level (altitude < 0.3 m or valid range < 0.15 m, speed < 0.3 m/s, continuously 2 s) as on the ground, in any mission phase, so a Running + RTH that cannot finish no longer blocks SE reset, SC change and abort. Flying, armed, motors enabled, open interlock, pending chain unchanged. | Host tests test_ground_rest, test_rc_mapper. Needs the bench check in missions.md. |
| Medium | main.cpp `onGroundNow` | **T9b-4:** D12 "on the ground" is extended by one case: mode Running with no pending chain and the active mission Idle or Finished (a landed vehicle: the mode stays Running after an RTH, see missions.md). Without it a landed vehicle could only be reset by a power cycle. | Reviewed in the T9b-4 report. |
| Medium | `Safety::RcCommandMapper` SC edge | **T9b-4:** SC acts only on an observed change 0 <-> 1 in both directions, never as a level, so an SC left at 0 at boot does NOT put the vehicle into simulation (the vehicle boots real; go is refused with `go_refusal` 4 until SC is cycled). D6 said "leaving real acts at once". | Safer and consistent with "boot with every switch combination does nothing"; confirm with the owner. |
| Low | `ExVectrNetwork` (library) | The legacy SX1280 sources of the library are compiled (unused, dropped at link time). | Library warnings in the build log. |
| Info | altitude resolution (`POSE.pos[2]`, `AIR_POSE.pos[2]`) | The telemetry itself no longer limits it: `POSE` is f32 (sub-mm at a few m) and `AIR_POSE` is 1 mm since 2026-11-20 (was 0.1 m). What remains: BME280 pressure at 16x oversampling has about 0.2 Pa noise (about 1.5-2 cm of altitude), IIR filter setting 1, and the f32 pressure (Q24.8 compensation output, resolution 0.0078 Pa near 100 kPa, about 0.7 mm) and `calcAltitudeFromPressure` in f32 (about 0.5 mm per ULP of p/p0). The position KF fuses baro, GNSS and range with large baro covariance (`cov*1000`), and the reference is a 10 % low-pass of the baro during zeroing, so the altitude is smoothed, not quantised. No casts to integer on the vehicle before the packet. | Expect centimetre-level noise and slow drift (weather, temperature), not steps. Estimator not changed. |
| Info | `board_v_1_0.h` | BME280 SPI CS, RGB LED, battery ADC are defined but unused (QMC INT pin 16 is now RX4). There is no battery monitoring. | No low-voltage protection. |
| Low | `src/comms/telemetry_mapping.cpp` `attitudeSetpoint()` | `setpoint_q` in the waypoint/RTH TVC phases is the latest value of the acc-to-att mapping topic, not `ControlAttitudeTvc`'s internal `stateSetpoint_` (no getter in ExVectrControl). They differ only between mapping updates, where the controller propagates its setpoint with the measured body rate. During the bellyflop kick the published value is the mission's kick target (`tvcOverride`), which the controller propagates the same way for ≤ 20 ms between mission writes. | Telemetry display only (ground station drawing); no effect on control. |
| Fixed (2026-11-21) | `src/comms/telemetry_mapping.cpp` `flapPointingSetpoint()` | `setpoint_q` read 0 whenever the flap correction exceeded about 151° (the unnormalised half-angle quaternion, norm `sqrt(2(1+d))`, fell under `sanitizeSetpointQuat`'s 0.5 "invalid" threshold); bench: 0 for about 1 s at the start of freefall. The correction is now normalised directly; regression test in `test_flap_pointing_setpoint`. | Telemetry display only. |

## Simulated sensors: ground distance

In simulation the ground distance (downward VL53L0X) and its covariance (constant 0.01, published with each valid reading, switched by `groundCovTopicSwitch`) are simulated by `SimLidar` (`src/sim_lidar.hpp`): the body simulator height is the measured distance only below `kSimLidarMaxRangeM` (1.0 m), otherwise no reading (stale topic, telemetry NaN). Same topic and telemetry path as the real driver; no noise. See missions.md.
