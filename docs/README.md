# ExVectr Starship: Developer Documentation

Versioned with the code, so every commit and flight tag carries matching docs. **Keep it current:** a change isn't finished until the docs it affects are updated (see [development-workflow.md](development-workflow.md#documentation-rule)).

## Start here

| Document | Read it for |
|---|---|
| [development-workflow.md](development-workflow.md) | Branch model (master / Development / feature), submodule rules, flight release, post-flight backup tags, commit conventions |
| [codebase-structure.md](codebase-structure.md) | Repo layout, what belongs where (lib vs `src/`), runtime model, frames/units, topic graph, radio ports, build |
| [project/overview.md](project/overview.md) | The vehicle: flight profile, GNC cascade, mode machine, failure detection, actuator gating, operating procedure, simulation mode |
| [project/missions.md](project/missions.md) | Mission framework and each mission's phases, triggers and parameters |
| [project/calibration.md](project/calibration.md) | Sensor calibration: validity rules that gate Arm and MissionBegin, the automatic gyro run, the accel/mag procedure, commands, log cues, bench evidence |
| [project/hardware-and-drivers.md](project/hardware-and-drivers.md) | Pinout, body frame and actuator layout, TVC/flap mixing, network links (USB, UART to the RX module), GNSS, rangefinder, telemetry/telecommand mapping |
| [flight-log.md](flight-log.md) | Every flight: tag, goals, outcome, anomalies |

## ExVectr framework libraries

| Library | Purpose |
|---|---|
| [ExVectrCore](libs/ExVectrCore.md) | Time, cooperative scheduler, tasks, topics/pub-sub, printing, containers |
| [ExVectrHAL](libs/ExVectrHAL.md) | Abstract hardware interfaces (buses, pins, interrupts) |
| [ExVectrArduinoPlatform](libs/ExVectrArduinoPlatform.md) | Arduino/Teensy implementation of Core platform hooks and HAL |
| [ExVectrMath](libs/ExVectrMath.md) | Matrix/vector/quaternion templates; **authoritative frame and rotation conventions** |
| [ExVectrDSP](libs/ExVectrDSP.md) | Attitude EKF, position KF, calibrators, body simulator, filters |
| [ExVectrControl](libs/ExVectrControl.md) | Position, attitude (TVC) and flap controllers |
| [ExVectrSensor](libs/ExVectrSensor.md) | Sensor interfaces and drivers (MPU9250, BME280, QMC5883) |
| [ExVectrActuator](libs/ExVectrActuator.md) | PWM output protocols, servo and H-bridge abstractions |
| [ExVectrNetwork](libs/ExVectrNetwork.md) | Stream datalink (COBS + CRC-16), network node, transport (ports, sequence filter), packet ports |
| [ExVectrPackets](libs/ExVectrPackets.md) | Schema-generated packets (telemetry, control), shared with ExVectrLink and the ground station |
| [ExVectrData](libs/ExVectrData.md) | Memory backends and key/value memory manager |

## Pinned library versions (feature/exvectrcore-update + feature/network-migration, 2026-11-19)

| Library | Pin | Base | Note |
|---|---|---|---|
| ExVectrCore | 6fe3f8e | 18bbb09 (`origin/Development`) | `NowNs()` API, plus a local `ListBuffer` overload fix (not yet on Core `Development`) |
| ExVectrSensor | bfe9b4e | 6bd4b01 | NowNs port only. `origin/Development` 86a35a3 is **not** taken (functional rewrite, still uses `NOW()`) |
| ExVectrDSP | 8a46c69 | bad2624 (`feature/review-fixes-2`) | **feature/sim-fixes-3** (2026-11-21): fold shield `flapFoldShieldFactor` 0.7 on by default (owner-approved), `aeroCalibration` 4.64. Below it (bad2624) mass 1.1 kg, factor 4.00, effective flap angle and the candidate effects. Below that **feature/sim-aero** (2026-11-21): pure body simulator model `BodySimModel` + thin `BodySimulator` task, resting ground contact. Below it the calibration module (56a998b) on the NowNs port (fc99b99 on 3e1f070). `origin/Development` 402831f (position KF rewrite) is **not** taken |
| ExVectrControl | c8d1ffb | 6e07d38 | **feature/sim-aero** (2026-11-21): `PhysicsBodySim` semi-implicit Euler with `ω × Iω`. Below it `resetControlState()` (6e07d38) and the NowNs port (8d6efab on 084bea1) |
| ExVectrActuator | 4d3c87a | db98892 | NowNs port only |
| ExVectrNetwork | de708ca | `origin/Development` | **feature/network-migration:** the new stack (stream links, Node, PortMux, PacketPort). The old stack and its NowNs port 7072cfe are gone |
| ExVectrPackets | 18b6884 | 53a7020 | **feature/setpoint-attitude:** `MISSION.setpoint_q` (f32) and `AIR_MISSION.setpoint_q` (i16, `limit = 1.0`) appended (attitude setpoint of the active attitude controller, all 0 = none; optional, append-only), `AIR_MISSION` 30 -> 38 B, `MISSION` 34 -> 50 B, schema hash **0xBFF82392** (Router, ship node and ground station must take the same pin to avoid INCOMPATIBLE). Below it, 53a7020 **feature/split-reset-zero:** `VEHICLE_STATUS.flags` bit `ZEROING` (bit 7, the byte is now full), new air packets `AIR_ACTUATORS` (0x22, 25 Hz, two samples) and `AIR_MISSION` (0x23, 5 Hz), schema hash **0x761D5378** (additive, but Router, ship node and ground station should take the same pin to avoid INCOMPATIBLE). Below it, 2c9c121 **feature/uart-cleanup-airpose:** `AIR_POSE.pos` i32 at 1 mm (30 B payload, wire break: Router, ship node, ground station must take the same pin), `VEHICLE_STATUS.flags` bit `UART_PEER_SEEN`, schema hash **0x4FE838D2**. Below it: `CALIBRATION` (0x93), hash 0xCFB7AEED; a96bd7b (`feature/link-config`, ExVectrLink's copy, unpushed) a96bd7b adds `LINK_REBOOT` and `LINK_CONFIG` on top of 4ca8fb1 |
| ExVectrArduinoPlatform | d481e6c | 4f31846 (`origin/Development`) | **feature/network-migration:** `EXVECTR_ARDUINO_NO_SERIAL_CONSOLE` build flag |
| ExVectrMath, ExVectrHAL, ExVectrData | unchanged | | No change needed |

The port commits are local feature-branch commits until the main session pushes them. See [development-workflow.md](development-workflow.md#dependency-updates-and-bench-re-verification) for the bench re-verification checklist that belongs to this update.

## Doc conventions

- Library docs share one structure: Purpose · Dependencies · Key concepts & API · Usage · Conventions · How this project uses it · Known issues · Extending.
- Each library doc states `Last verified against commit:`. If the submodule pin has moved past that commit, treat the doc as possibly stale and re-verify the parts you rely on.
- Cite code as `path:line`. Line numbers drift, so re-check them when editing.
