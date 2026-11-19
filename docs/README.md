# ExVectr Starship: Developer Documentation

Versioned with the code, so every commit and flight tag carries matching docs. **Keep it current:** a change isn't finished until the docs it affects are updated (see [development-workflow.md](development-workflow.md#documentation-rule)).

## Start here

| Document | Read it for |
|---|---|
| [development-workflow.md](development-workflow.md) | Branch model (master / Development / feature), submodule rules, flight release, post-flight backup tags, commit conventions |
| [codebase-structure.md](codebase-structure.md) | Repo layout, what belongs where (lib vs `src/`), runtime model, frames/units, topic graph, radio ports, build |
| [project/overview.md](project/overview.md) | The vehicle: flight profile, GNC cascade, mode machine, failure detection, actuator gating, operating procedure, simulation mode |
| [project/missions.md](project/missions.md) | Mission framework and each mission's phases, triggers and parameters |
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
| ExVectrDSP | fc99b99 | 3e1f070 | NowNs port only. `origin/Development` 402831f (position KF rewrite) is **not** taken |
| ExVectrControl | 8d6efab | 084bea1 | NowNs port only |
| ExVectrActuator | 4d3c87a | db98892 | NowNs port only |
| ExVectrNetwork | de708ca | `origin/Development` | **feature/network-migration:** the new stack (stream links, Node, PortMux, PacketPort). The old stack and its NowNs port 7072cfe are gone |
| ExVectrPackets | 4ca8fb1 | `origin/Development` | **feature/network-migration:** generated packets, schema hash 0x3AFA3CCE |
| ExVectrArduinoPlatform | d481e6c | 4f31846 (`origin/Development`) | **feature/network-migration:** `EXVECTR_ARDUINO_NO_SERIAL_CONSOLE` build flag |
| ExVectrMath, ExVectrHAL, ExVectrData | unchanged | | No change needed |

The port commits are local feature-branch commits until the main session pushes them. See [development-workflow.md](development-workflow.md#dependency-updates-and-bench-re-verification) for the bench re-verification checklist that belongs to this update.

## Doc conventions

- Library docs share one structure: Purpose · Dependencies · Key concepts & API · Usage · Conventions · How this project uses it · Known issues · Extending.
- Each library doc states `Last verified against commit:`. If the submodule pin has moved past that commit, treat the doc as possibly stale and re-verify the parts you rely on.
- Cite code as `path:line`. Line numbers drift, so re-check them when editing.
