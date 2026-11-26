# ExVectrPackets
> Shared packet definitions. Schema-generated packets (TOML schema -> C++ headers and a Python module, one source of truth for the vehicle, ExVectrLink and the ground station), plus the legacy hand-written fixed-point structs and CRSF classes used by ExVectrLink FPV. Repo: https://github.com/ExVeCtr/ExVectrPackets.git `Last verified against commit: 3bdd417 (Development; schema hash 0xF49ACD01)`

The authoritative docs are in the library repo: `docs/schema-format.md`, `docs/telemetry-sets.md`, `docs/versioning.md`, `docs/adding-a-packet.md` and the generated `docs/packet-reference.md` (every field, offset, scaling and unit). This file covers what the flight software uses.

## Purpose & scope
- **Generated packets** (`schema/common.toml`, `schema/starship.toml` -> `gen/ExVectrPackets/gen/*.hpp`, `python/exvectr_packets/`): explicit little-endian `encode()`/`decode()` field by field, no struct copies. Fixed-point fields round half away from zero, saturate, and encode NaN/Inf as an invalid sentinel that decodes to NaN. Decoded values are always SI.
- **Legacy** (`include/ExVectrPackets/packet_vehicle.hpp`, `data_packing.hpp`, CRSF/radiolink): no longer used by the flight software since the network migration (T9a). Kept for ExVectrLink FPV.

## Dependencies (ExVectr libs + external)
- Generated headers: none (standard library only: `<stdint.h>`, `<math.h>`, `<string.h>`, `<limits>`). C++17 (`namespace VCTR::Packets`).
- The generated headers are under `gen/`, not `include/`: the flight software adds `-Ilib/ExVectrPackets/gen` in `platformio.ini`.

## Key concepts & API
- Each packet is a struct with SI members and constants `kPort`, `kQos`, `kLatest`, `kAirEligible`, `kAirRateHz`, `kUsbRateHz`, `kMaxSize`, `kMinSize`, flag masks (`kFlags*`). `size_t encode(uint8_t*, size_t) const` (0 on failure), `bool decode(const uint8_t*, size_t)` (accepts longer payloads, append-only evolution).
- `Packets::kSchemaHash` (`all.hpp`) goes into every heartbeat. The ground station compares it with its own.
- Packets the vehicle sends:

| Port | Packet | Set | Rate on the vehicle | Content (SI) |
|---|---|---|---|---|
| 0x20 | `VehicleStatus` | USB + radio | 20 Hz USB, 5 Hz UART | time, mode, mission mode/phase, flags (incl. `UART_PEER_SEEN`, node 3 heard on the UART within 3 s, information only; `ZEROING`, estimator zeroing window active; the byte is now full), failure bits, sensor states, `home_epoch`, T9b fields (`ctrl_flags`, `go_refusal`, `degraded_bits`, `mission_profile`, `rc_rejected`) and `cmd_session` (low 16 bits of the latched GS session; GS-control cutover; 38 B), `calib_state` (30 B) |
| 0x21 | `AirPose` | radio | 25 Hz UART (schema `kAirRateHz`) | q (i16), position (i32, 1 mm), velocity (i16, 0.01 m/s), fixed point, ground_distance (u16 1 mm, 65535 = NaN), 32 B |
| 0x22 | `AirActuators` | radio | 25 Hz UART, 2 samples each (50 samples/s) | newest and previous (about 20 ms older, `prev_age`) actuator sample: TVC deflection, roll torque, thrust, flaps; i16/u16 fixed point, 38 B |
| 0x23 | `AirMission` | radio | 5 Hz UART | mission time (i32 ms), mission mode, phase, index, waypoint, position setpoint (i32 1 mm), velocity setpoint (i16 0.01 m/s), attitude setpoint `setpoint_q` (i16, as `AirPose.q`; all 0 = none), 38 B |
| 0x24 | `AirGnss` | radio | 10 Hz UART | lat/lon (i32 1e-7 deg, decoded as radians f64), altitude (i32 mm), NWU velocity (i16 0.01 m/s), 22 B |
| 0x25 | `AirGnssInfo` | radio | 1 Hz UART | real u-blox fix type, satellites, h/v accuracy (u16 cm, 65535 = unknown), 10 B |
| 0x03 | `HandsetTelem` | USB + radio | 2 Hz each | display values for the Pocket (dst 0xFF) |
| 0x84 | `CmdApplied` | USB | 10 Hz | last accepted `CMD_STATE` fields, info flags, detector safe-seen bits, session, state/USB/UART ages, reject counters (26 B; replaces `RcApplied`) |
| 0x13 | `CmdState` | up (node 0 -> vehicle) | 25 Hz + on change | the ground station's command state (14 B, strict fields; vehicle decodes it per link, `src/comms/cmd_ingress_tap.*`) |
| 0x80 | `Pose` | USB | 100 Hz | omega, q, velocity, position, ground distance (f32; NaN = unavailable) |
| 0x81 | `Actuators` | USB | 50 Hz | flaps, TVC deflection, thrust, roll torque (36 B; ground distance moved to POSE and AIR_POSE 2026-11-26) |
| 0x82 | `Mission` | USB | 20 Hz | setpoint, mission time (s), index, phase, attitude setpoint `setpoint_q` (f32 [w,x,y,z] body -> world; all 0 = none), `mission_mode`, `waypoint`, 52 B |
| 0x83 | `Gnss` | USB | 10 Hz | lat/lon (rad, f64), altitude, velocity, accuracies |
| 0x90 | `ImuRaw` | USB debug | 100 Hz | uncalibrated sensor-frame gyro, accel, mag (T), pressure, temperature (K) |
| 0x91 | `Estimator` | USB debug | 20 Hz | attitude/velocity/position standard deviations |
| 0x92 | `TaskTiming` | USB debug | 5 Hz | average run time of the first 8 scheduler tasks |
| 0x93 | `Calibration` | USB debug | 0.5 Hz (vehicle constant, schema rate 0) | applied sensor calibration: per sensor offset and 3x3 matrix, source bits, raw frame, EKF gyro bias ([formulas](../project/hardware-and-drivers.md#8a-sensor-calibration-pipeline-what-calibration-reproduces)) |
| 0x02 | `LogText` | USB | as printed | console text (Core print, `LOG_MSG`) |

How the vehicle fills each field: [../project/hardware-and-drivers.md](../project/hardware-and-drivers.md#9-telemetry-generated-packets-srccommstelemetry_mappingcpp).

Vehicle-directed `REQUEST` actions (hash now 0xF49ACD01, Packets 3bdd417): 7, 8, 12..24 are executed by `Safety::RequestExecutor` (table: [../project/hardware-and-drivers.md](../project/hardware-and-drivers.md#10-requests-the-legacy-telecommand-port-0x7f-is-deleted)); action 24 `SET_MISSION_PARAM` takes ids 1..10 and `MISSION_PARAMS` (0x26) is 40 B; `GET_HOME_POSITION` is answered with `HOME_POSITION` (0x50).

## Usage
```cpp
#include "ExVectrPackets/gen/all.hpp"
Packets::Pose p;  p.t_us = ...; p.q[0] = qw; ...
uint8_t buf[Packets::Pose::kMaxSize];
size_t n = p.encode(buf, sizeof buf);          // little-endian, SI -> wire
node.send(0, Packets::Pose::kPort, buf, n, static_cast<network::wire::Qos>(Packets::Pose::kQos), Packets::Pose::kLatest);
```

## Conventions
- Never edit the generated files. Change the schema, regenerate (`tools/gen_packets.py`), commit the output, re-pin in every project that talks to the vehicle (flight software, ExVectrLink, ground station). Any change is a cross-project change.
- Append-only: new fields at the end; a breaking change takes a new port.
- Fields the schema leaves to the flight software (`VehicleStatus.mode`, `failure_bits`, `sensor_state`, `HandsetTelem.mode_text`) are defined in [../project/hardware-and-drivers.md](../project/hardware-and-drivers.md#9-telemetry-generated-packets-srccommstelemetry_mappingcpp).

## How this project uses it
- `src/comms/telemetry_mapping.cpp` builds every packet from a `TelemetrySnapshot` (pure functions, host-tested in `test/test_telemetry_mapping`).
- `src/comms/vehicle_network.cpp` schedules them using the generated `kUsbRateHz` / `kAirRateHz` (100 Hz tick; `static_assert`s check that each rate divides it). Exception: `CALIBRATION` has no schema rate (the rate fields are integer Hz) and is sent every 200 ticks (`kCalibrationPeriodTicks`).
- The fixed-point rules of the schema fix the old telemetry unit bugs: no int16 overflow (ground distance above 2 m, positions above the old limits), no NaN undefined behaviour, GNSS lat/lon declared and sent in radians, accuracies as standard deviations.

## Known issues & limitations

| Severity | Location | Issue | Impact |
|---|---|---|---|
| Low | schema `TASK_TIMING` | `max` is sent as NaN: the ExVectrCore scheduler only keeps an average run time. | Only averages are available. |
| Info | legacy `include/ExVectrPackets.hpp` | Include guard `EXVECTRNETWORK_H_` collides with the old ExVectrNetwork umbrella header. | Irrelevant to the flight software now (neither umbrella is included). |

## Extending
1. Add or change the packet in `schema/starship.toml` (or `common.toml`), regenerate, run the library tests, commit, push.
2. Coordinate the ground-station and ExVectrLink pins (same schema hash everywhere, or the heartbeat raises the incompatibility flag).
3. Re-pin `lib/ExVectrPackets`, add the mapping in `src/comms/telemetry_mapping.cpp`, a test in `test/test_telemetry_mapping`, and schedule it in `VehicleNetwork::sendTelemetry`.
