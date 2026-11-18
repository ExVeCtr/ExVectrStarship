# ExVectr Starship: Developer Documentation

Versioned with the code, so every commit and flight tag carries matching docs. **Keep it current:** a change isn't finished until the docs it affects are updated (see [development-workflow.md](development-workflow.md#documentation-rule)).

## Start here

| Document | Read it for |
|---|---|
| [development-workflow.md](development-workflow.md) | Branch model (master / Development / feature), submodule rules, flight release, post-flight backup tags, commit conventions |
| [codebase-structure.md](codebase-structure.md) | Repo layout, what belongs where (lib vs `src/`), runtime model, frames/units, topic graph, radio ports, build |
| [project/overview.md](project/overview.md) | The vehicle: flight profile, GNC cascade, mode machine, failure detection, actuator gating, operating procedure, simulation mode |
| [project/missions.md](project/missions.md) | Mission framework and each mission's phases, triggers and parameters |
| [project/hardware-and-drivers.md](project/hardware-and-drivers.md) | Pinout, TVC/flap mixing, radio, GNSS, rangefinder, telemetry/telecommand byte layouts |
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
| [ExVectrNetwork](libs/ExVectrNetwork.md) | Datalink, network node, transport, topic bridging |
| [ExVectrPackets](libs/ExVectrPackets.md) | Fixed-point telemetry packet formats |
| [ExVectrData](libs/ExVectrData.md) | Memory backends and key/value memory manager |

## Doc conventions

- Library docs share one structure: Purpose · Dependencies · Key concepts & API · Usage · Conventions · How this project uses it · Known issues · Extending.
- Each library doc states `Last verified against commit:`. If the submodule pin has moved past that commit, treat the doc as possibly stale and re-verify the parts you rely on.
- Cite code as `path:line`. Line numbers drift, so re-check them when editing.
