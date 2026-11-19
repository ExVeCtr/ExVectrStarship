# ExVectrNetwork
> The ExVectr networking stack: byte-stream datalink (COBS + CRC-16), 8-bit-addressed network node with heartbeats, presence, router forwarding and egress filters, single-frame transport with a sequence filter, and typed packet ports. Plus the legacy SX1280 radio drivers used by ExVectrLink FPV. Repo: https://github.com/ExVeCtr/ExVectrNetwork.git `Last verified against commit: de708ca (Development)`

The authoritative library docs are in the library repo: `docs/node.md` (stack, API), `docs/wire.md` (byte formats), `docs/aggregator.md` (air link, not used on the vehicle). The design and its rationale are in the workspace `docs/network-design.md`. This file covers what the flight software relies on.

## Purpose & scope

```
Application   PacketPort<T>           typed adapter over a generated ExVectrPackets packet
Transport     PortMux<MaxSubs>        port demux + latest-sequence filter
Network       Node                    addresses, TTL, heartbeats, presence, node table,
                                      forwarding (routers only), per-link egress filters
Datalink      FrameLinkI              StreamFrameLink (COBS + CRC-16)  <- the vehicle uses this
                                      AirFrameLink (aggregator)        <- ExVectrLink only
Physical      ByteStreamI             application adapter over USB CDC / UART
```

Replaces the old stack (`NetworkNode`, `TransportCallback`, `TransportTopic`, generic `Datalink`, 16-bit addresses, segmenting transport), which is removed from the library.

## Dependencies (ExVectr libs + external)
- The stack (`wire/`, `link/`, `net/`, `transport/`) has **no** dependency on ExVectrCore, ExVectrHAL or Arduino. Time comes in through `TimeSourceI` (`nowNs()`), bytes through `ByteStreamI`.
- The kept legacy parts (`DataPacket`, `DatalinkI`, `RadioI`, `TopicIO`, `datalink/sx1280/*` with a vendored `sx12xxAL`) depend on ExVectrCore/HAL. They are compiled in the Teensy build (PlatformIO builds the whole library) but nothing in the flight software references them, so the linker drops them. They add a few library warnings to the build log (`-Wreorder`, unused variables in `SX128XLT.cpp`, `Sx1280_2.cpp`).
- No heap after construction. Capacities are compile-time (template parameters, `EXVECTR_NET_MAX_LINKS = 4`, 8 tracked nodes).

## Key concepts & API

### StreamFrameLink<TxCap> (link/StreamFrameLink.hpp)
- Wire: `COBS(0x01 ‖ network frame ‖ CRC16_LE) ‖ 0x00`, CRC-16/CCITT-FALSE.
- `send(frame, len, t)`: encodes into the static TX ring; if the whole encoded frame does not fit it is dropped (`stats.txDropped`), never partially written. The encoding happens before the fit check, so the cost is the same with or without a reader.
- `poll()`: reads all readable bytes (frames delivered synchronously to the Node), then drains the ring into `writable()` bytes. Never blocks if the `ByteStreamI` doesn't.
- RX buffer 256 B; resynchronises at the next `0x00`; bad frames count `stats.rxBad`.

### Node (net/Node.hpp)
- `NodeConfig {address, role, forwarding, defaultTtl 3, schemaHash, heartbeatPeriodNs 1 s, presenceTimeoutNs 3 s}`.
- End nodes (`forwarding = false`, the vehicle) never forward and send every frame on every link their egress filter admits.
- `send(dst, port, payload, len, qos, latest)`: assigns the per-port sequence number.
- `poll()`: heartbeat when due (broadcast, role, network version, schema hash, uptime).
- `isPresent(addr)`: any valid frame (heartbeat or data) from `addr` within `presenceTimeoutNs`. `entry(addr)`: node table (link, last seen, last heartbeat, `incompatible` = other network version or schema hash).
- `EgressFilter`: `portLimit`, `(fromLink, port)` rules, heartbeat-by-source, destination, source and source+port drops.

### PortMux / PacketPort (transport/)
- `PortMux<N>`: one handler per port; accepts a frame from (src, port) if its sequence is newer (`(seq - last) mod 256` in 1..127), or after 1 s of silence, or the first one. Drops duplicates (a frame that arrives on two links).
- `PacketPort<T>(node, mux, port, dst, qos, latest)`: decodes with `T::decode`, keeps the latest value, optional `PacketListenerI<T>`; `send(T)` encodes with `T::encode`. `T` needs `kMaxSize`, `encode(uint8_t*, size_t)`, `decode(const uint8_t*, size_t)`. The generated ExVectrPackets classes have this shape.

## Usage
See `src/comms/vehicle_network.cpp`:
```cpp
network::NodeConfig cfg; cfg.address = 1; cfg.role = network::wire::Role::Vehicle;
cfg.schemaHash = Packets::kSchemaHash; cfg.presenceTimeoutNs = 2000000000LL;
network::StreamFrameLink<2048> usbLink(usbStream, clock);
network::Node usbNode(cfg, clock);
usbNode.addLink(usbLink);
usbNode.setDataSink(mux);                      // shared PortMux
usbLink.poll(); usbNode.poll();                // from a task
usbNode.send(0, Packets::Pose::kPort, buf, n, network::wire::Qos::Telemetry, true);
```

## Conventions
- Addresses: 0 ground station, 1 vehicle, 2 ExVectrLink TX module, 3 ExVectrLink RX module, 0xFF broadcast.
- Ports: 0x01-0x0F link/system, 0x10-0x1F control, 0x20-0x7F vehicle radio set, 0x80-0xEF USB-only (dropped by every air and vehicle-UART egress filter), 0xF0-0xFE local/experimental. The flight software's port table is in [../codebase-structure.md](../codebase-structure.md#radio-and-usb-network).
- Heartbeats report presence only. They never trigger or prevent a failsafe by themselves (workspace `docs/safety.md` section 4). Exception until T9b: see "How this project uses it".
- Frames are delivered synchronously from `poll()`: handlers copy data, they do no control work.

## How this project uses it
`src/comms/` (see [../project/hardware-and-drivers.md](../project/hardware-and-drivers.md#5-exvectrnetwork-links-srccomms)):
- **Links:** `StreamFrameLink<2048>` on USB `Serial`, `StreamFrameLink<1024>` on `RX_MODULE_SERIALPORT` (Serial4, 460800 8N1) to the ExVectrLink RX module.
- **Two Nodes, both address 1** (one per link), so the USB link carries the USB telemetry set and the UART carries the radio set at the air rates. One shared `PortMux<2>`, so a frame that arrives on both links is delivered once. The UART node's egress filter drops ports >= 0x80.
- **Heartbeats:** each node sends the vehicle heartbeat at 1 Hz on its link.
- **Ground-station presence:** `presenceTimeoutNs = 2 s` on both nodes; `isPresent(0)` on either node is the observable of the existing `radioConnectionLoss` failure (same as the old `NetworkNode::isNodeReachable(0)` with its 2 s timeout). This is a deliberate, temporary exception to "heartbeats never keep the vehicle alive", kept until T9b replaces it with the `RC_STATE` control-source timeout.
- **Ports:** the generated telemetry packets, plus the TEMPORARY legacy telecommand port 0x7F (`PacketPort<LegacyTelecommandPacket>`).
- Everything runs in the `VehicleComms` task (100 Hz, priority 50). The whole comms object (links, nodes, buffers, about 10 KB) is in DMAMEM.
- Host tests: `test/test_vehicle_network` builds the stack natively (`pio test -e native`).

## Known issues & limitations

| Severity | Location | Issue | Impact |
|---|---|---|---|
| Info | library `src/sx1280/**` | The legacy SX1280 sources are compiled into the Teensy build although unused. | Extra library warnings and build time only; no code or RAM in the firmware. |
| Info | `Node` | An end node has no per-link send. | The flight software runs one Node per link to give each link its own telemetry set (see above). Each node keeps its own sequence counters and node table. |
| Low | `Node` heartbeat uptime | `startNs_` is taken in the constructor. The vehicle's comms object is a static global, so the heartbeat uptime counts from static initialisation. | Uptime is a few ms longer than `NowNs()`; display only. |

## Extending
- New telemetry packet: add it to the ExVectrPackets schema (cross-project change), regenerate, re-pin, then build it in `src/comms/telemetry_mapping.cpp` and schedule it in `VehicleNetwork::sendTelemetry`.
- New received packet: one more `PacketPort<T>` on the shared mux (raise the `PortMux` capacity).
