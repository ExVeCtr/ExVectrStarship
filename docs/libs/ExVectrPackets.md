# ExVectrPackets
> Header-only, fixed-point packed telemetry structs (attitude, position, GPS) plus pack/unpack helpers for sending over ExVectrNetwork. Repo: https://github.com/ExVeCtr/ExVectrPackets.git `Last verified against commit: 8c15c7f`

## Purpose & scope
Standardised, compact wire structs shared by the vehicle and the ground station. Floats are scaled into saturating `int16_t` fixed point against a per-field limit. The structs are `__attribute__((packed))` POD meant to be sent raw by `Net::TransportTopic<T>`.

Only three packet types exist. There is no versioning, framing, CRC or type ID; those come from the transport port number and the transport CRC. README: "under initial development".

## Dependencies (ExVectr libs + external)
- ExVectrMath: `constants.hpp` (`DEGREES` = π/180 as float), `Vector_F`, `Quat_F`, matrix headers.
- External: `<limits>` and `stdint.h`.
- No ExVectrNetwork dependency, although the code lives in namespace `VCTR::Net`.
- Build:
  - Header-only; there is no `src/`.
  - CMakeLists.txt globs `src/*.cpp` into `add_library`, which CMake rejects when there are no sources. PlatformIO doesn't care.

## Key concepts & API

### Fixed-point helpers (include/ExVectrPackets/data_packing.hpp)
```cpp
template<IN, OUT> OUT packFixedPoint(const IN& value, const IN& limit);   // OUT signed, sizeof(OUT) < sizeof(IN)
template<IN, OUT> OUT unpackFixedPoint(const IN& value, const OUT& limit); // IN signed
```
- Packing:
  - `value > limit` → `OUT::max`; `value < -limit` → `OUT::min`
  - otherwise `static_cast<OUT>(value * OUT::max / limit)`, which truncates toward zero
- Unpacking: `value * limit / IN::max`.
- Resolution = `limit / 32767` for int16.
- Asymmetric saturation: `INT16_MIN` unpacks to `-1.00003 * limit`.

### Packet structs (include/ExVectrPackets/packet_vehicle.hpp)
- Fields are private. Construct with physical values and read back with getters.
- All are packed and little-endian (raw memory on ARM Cortex-M7).

`PacketAttitude`: 18 bytes. Constructor `(Vector_F angVel = 0, Quat_F q = (1,0,0,0), float tiltAcc = 0, float northAcc = 0)`. Getters: `getAngularVelocity()`, `getQuaternion()`, `getTiltAccuracy()`, `getNorthAccuracy()`.

| Offset | Type | Field | Unit / frame | Limit | Resolution |
|---|---|---|---|---|---|
| 0 | int16[3] | angularVelocity x,y,z | rad/s | ±1000°/s (17.45 rad/s) | 5.3e-4 rad/s |
| 6 | int16[4] | quaternion w,x,y,z | – | ±1.0 | 3.05e-5 |
| 14 | int16 | tiltAccuracy | rad | ±360° (6.283 rad) | 1.9e-4 rad |
| 16 | int16 | northAccuracy | rad | ±360° | 1.9e-4 rad |

`PacketPosition`: 16 bytes. Constructor `(Vector_F pos = 0, Vector_F vel = 0, float hAcc = 0, float vAcc = 0)`. Getters: `getPosition()`, `getVelocity()`, `getHAccuracy()`, `getVAccuracy()`.

| Offset | Type | Field | Unit | Limit | Resolution |
|---|---|---|---|---|---|
| 0 | int16[3] | position x,y,z | m (local frame of the estimator) | ±1000 m | 0.031 m |
| 6 | int16[3] | velocity x,y,z | m/s | ±100 m/s | 0.0031 m/s |
| 12 | int16 | hAccuracy | m | ±10 m | 3.1e-4 m |
| 14 | int16 | vAccuracy | m | ±10 m | 3.1e-4 m |

`PacketGPS`: 23 bytes. Constructor `(Vector_F vel = 0, double lat = 0, double lon = 0, float alt = 0, uint8_t numSats = 0, float posAcc = 0, float altAcc = 0, float velAcc = 0)`. Note that velocity comes first. Getters: `getLatitude()`, `getLongitude()` (double), `getAltitude()`, `getVelocity()`, `getNumSats()`, `get{Position,Altitude,Velocity}Accuracy()`.

| Offset | Type | Field | Unit (as documented) | Limit | Resolution |
|---|---|---|---|---|---|
| 0 | int32 | latitude | deg × 1e7 (`lat * 1e7`, truncated, no range check) | ±214.7 | 1e-7 |
| 4 | int32 | longitude | deg × 1e7 | ±214.7 | 1e-7 |
| 8 | int16 | altitude | m | ±1000 m | 0.031 m |
| 10 | int16[3] | velocity | m/s | ±100 m/s | 0.0031 m/s |
| 16 | uint8 | numSats | – | 255 | 1 |
| 17 | int16 | positionAccuracy | m | ±10 m | 3.1e-4 m |
| 19 | int16 | altitudeAccuracy | m | ±10 m | 3.1e-4 m |
| 21 | int16 | velocityAccuracy | m/s | ±10 m/s | 3.1e-4 m/s |

## Usage
As in `src/main.cpp:202-205, 239-242, 268-273`:
```cpp
Net::PacketAttitude att({w(0), w(1), w(2)}, {q(0), q(1), q(2), q(3)});   // rad/s, quaternion w,x,y,z
attitudeDataTopic.publish(att);                                       // TransportTopic port 10 sends it

Net::PacketPosition pos({px, py, pz}, {vx, vy, vz}, hAcc, vAcc);
// receiver side
Math::Vector_F p = pos.getPosition();
```

## Conventions
- One struct per transport port. The receiving `TransportTopic` checks only `sizeof(T)`, so two packet types of equal size on the same port would be confused.
- Keep every packet `__attribute__((packed))`, little-endian and ≤ 128 bytes (single transport segment).
- Choose limits for the vehicle envelope. Values outside the limit saturate silently; nothing flags the overflow.
- Namespace `VCTR::Net`. Include `ExVectrPackets/packet_vehicle.hpp` directly. Avoid the umbrella `ExVectrPackets.hpp` (guard collision, and it is empty anyway).

## How this project uses it
- `src/main.cpp:134-150`: topics and `TransportTopic`s for `PacketAttitude` (port 10), `PacketPosition` (port 11) and `PacketGPS` (port 12), all broadcast (`UINT16_MAX`).
- Attitude (`attitudeTelemetryCallback`, main.cpp:189-206, 5 Hz): built from the attitude switch topic `Vector<float,7>`, with rates in `data(0..2)` and quaternion in `data(3..6)`. Accuracies are left at 0.
- Position (`positionTelemetryCallback`, main.cpp:225-243, 10 Hz): position from `data(3..5)`, velocity from `data(0..2)`. `hAcc`/`vAcc` are computed from `posEstTask.getCovariance()` divided by 1000. That app-side scaling is not a standard deviation in metres; verify it.
- GPS (`gnssTelemetryCallback`, main.cpp:262-273, 1 Hz): passes `GNSSData.position(0/1)`, which are **radians** (`SNSR::GNSSData` convention, src/ublox_gnss.cpp:19), into the `lat`/`lon` parameters documented as degrees. On the wire the field holds rad × 1e7 (1e-7 rad ≈ 0.64 m resolution). The ground station must decode it as radians, or the app should convert first.
- In the same call, the accuracies are fed with covariance diagonal entries (variances, m²) rather than accuracies (m), and altitude is height above the ellipsoid, which saturates above 1000 m.

## Known issues & limitations
Paths are relative to `lib/ExVectrPackets/` unless noted.

| Severity | Location | Issue | Impact |
|---|---|---|---|
| Med | include/ExVectrPackets.hpp:1-2 | The include guard is `EXVECTRNETWORK_H_`, the same as `lib/ExVectrNetwork/include/ExVectrNetwork.hpp:1-2`. | Including both umbrella headers silently drops the second one (the ExVectrNetwork umbrella pulls in datalink/node/transport). |
| Low | include/ExVectrPackets/data_packing.hpp:17-22 | NaN or Inf input: NaN fails both comparisons and reaches `static_cast<int16_t>(NaN)`, which is undefined behaviour in C++. ARM VCVT happens to saturate NaN to 0. | An estimator NaN is transmitted as 0 (looks valid) instead of being flagged. |
| Low | include/ExVectrPackets/data_packing.hpp:22 | Truncates instead of rounding. | Bias of up to 1 LSB toward zero. |
| Low | include/ExVectrPackets/data_packing.hpp:20, 30 | Negative saturation is `INT16_MIN`, which unpacks slightly beyond `-limit`. | Cosmetic asymmetry. |
| Low | include/ExVectrPackets/packet_vehicle.hpp:133-134 | `lat * 1e7` / `lon * 1e7` truncate with no range check. | Out-of-range input overflows int32 (UB); not reachable for valid degrees or radians. |
| Low | include/ExVectrPackets/packet_vehicle.hpp:118 | GPS altitude limit is ±1000 m. | Saturates for absolute (ellipsoid/MSL) altitudes at elevated sites; as used, main.cpp passes ellipsoid altitude. |
| Low | include/ExVectrPackets/packet_vehicle.hpp (all) | No packet type ID or version field. | A layout change on one side gives silent misinterpretation if `sizeof` is unchanged. |
| Low | (project) src/main.cpp:268-271 | Passes radians and variances where the packet documents degrees and accuracies (see above). | Ground-station decoding must match the actual units. |
| Low | CMakeLists.txt:4-5 | `add_library` with an empty glob. | CMake/host build of this library fails. |

## Extending
New packet type:
1. Add a `struct PacketX { private: <int fields>; public: PacketX(<physical values> = defaults); <getters>; } __attribute__((packed));` in a header under `include/ExVectrPackets/`.
2. Use `packFixedPoint<float, int16_t>(v, LIMIT)` / `unpackFixedPoint<int16_t, float>(raw, LIMIT)`. Use int32 scaled integers when int16 resolution isn't enough (as for lat/lon). Define each `LIMIT` once (constexpr) so packing and unpacking can't diverge.
3. Give every constructor parameter a default, so `TransportTopic::receiveData` can default-construct `T`.
4. Keep `sizeof ≤ 128`. Document unit, frame and limit per field.
5. Assign a new transport port in `src/main.cpp` and on the ground station.
