# ExVectrNetwork
> Layered (OSI-like) networking: datalink over any `HAL::DigitalIO`, 16-bit-addressed network node with heartbeats, UDP-like segmenting transport and topic-over-network bridging. Repo: https://github.com/ExVeCtr/ExVectrNetwork.git `Last verified against commit: cabddce (pinned now: 7072cfe = cabddce plus a mechanical NOW() -> NowNs() port for ExVectrCore 18bbb09; content otherwise as verified)`

## Purpose & scope
Connects ExVectr devices (vehicle ↔ ground station) by linking `Core::Topic`s across a radio or bus. Layers communicate only through topics:

```
Transport  TransportTopic<T> ─ TransportCallback  (ports, segmentation, CRC)      Topic<TransportData>
Network    NetworkNode : Network_Interface          (addresses, heartbeat, table)  Topic<NetworkPacket>
Datalink   Datalink_Interface → Datalink / app's Datalink_SX1280 (medium access)   Topic<List<uint8_t>>
Physical   HAL::DigitalIO (UART/SPI/radio), TopicIO (loopback over a topic)
```

The README declares it under initial development, says it uses dynamic memory (`Core::ListArray`), and lists DHCP/DNS-like services as plans. Not implemented: routing/forwarding between links, ACK/NACK, retransmission, flow control.

## Dependencies (ExVectr libs + external)
- ExVectrCore:
  - topics/subscribers (`Topic`, `Callback_Subscriber`, `Topic_Publisher`)
  - `Task_Periodic`/`Scheduler`
  - containers (`List`, `ListArray`, `ListBuffer`, `ListExtern`)
  - `computeCrc` (CRC-16-CCITT)
  - `print.hpp`, time
- ExVectrHAL: `HAL::DigitalIO`, `IO_TYPE_t::TOPIC`, `IO_PARAM_t`.
- External: none.
- Heap: `ListArray` is used in the transport payloads, segment buffers and the node table.

## Key concepts & API

### Datalink_Interface (include/ExVectrNetwork/interfaces/datalink_interface.hpp)
- `Topic<List<uint8_t>>& getReceiveTopic()`: frames received from the medium (Datalink → Network). Subscribers must copy the frame.
- `void setTransmitTopic(Topic<List<uint8_t>>&)`: subscribes `transmitSubr_` to a topic of frames to send (Network → Datalink). `removeTransmitTopic()` unsubscribes.
- Implementers override `dataframeReceiveFunc(const List<uint8_t>&)`, which is called once per frame to transmit.

### Datalink (generic, include/ExVectrNetwork/datalink.hpp, src/datalink.cpp)
- `Datalink(HAL::DigitalIO& phy, Scheduler& = getSystemScheduler())` is a `Task_Periodic` with a nominal period of 1 s. `taskCheck()` releases it immediately whenever there is TX data or readable RX bytes.
- `dataLinkMaxFrameLength = 230`. The TX/RX buffers hold `230 * 5` bytes; TX entries are stored as `[len, bytes...]`.
- Medium-access protocol on the wire (sent as raw bytes on `phy`):

| Byte(s) | Meaning |
|---|---|
| `0x00` BLOCK | Sender claims the medium. Receivers set `physicalBlocked_` and timestamp it. |
| `0x02` DATA, `len`, `len` bytes | Payload chunk (≤ 250, ≤ `writable()-2`). One frame may span several DATA chunks. |
| `0x01` FREE | Medium released. The receiver publishes the accumulated RX buffer as one frame. |

- Sequence per frame: BLOCK → DATA … → FREE, one step per task run.
- `setPhysicalReleaseTimeout(ns)` sets the auto-release (default 100 ms) after the last RX activity.
- No CRC, no collision detection, no backoff.

### NetworkNode (include/ExVectrNetwork/network_node.hpp, src/network_node.cpp)
- Constructors:
  - `NetworkNode(uint16_t address, int64_t disconnectTimeout = 1 s)`
  - `NetworkNode(address, Datalink_Interface&, timeout)`
  - Both add the node to the **system** scheduler as a 100 ms periodic task.
- `Network_Interface` (interfaces/network_interface.hpp) provides `getTransmitTopic()`/`getReceiveTopic()` (`Topic<NetworkPacket>`) and `get/setNodeAddress`.
- Addresses: 16-bit. `0xFFFF` (`UINT16_MAX`) is broadcast. The doc comment says 0 means receive-only, but nothing enforces it.
- `sendPacket(const NetworkPacket&)`:
  - Overwrites `srcAddress` with the node's own address.
  - Delivers locally if `dst == own address`.
  - Otherwise packs the packet and publishes it on the datalink transmit topic.
  - Empty payloads are rejected.
- Heartbeat: if nothing was sent for `timeout/10`, it broadcasts `type=HEARTBEAT, hops=0, payload={0}`.
- Reachability: any received packet refreshes `nodeList_[src].lastSeen`. Entries older than `timeout` are dropped. Query with `isNodeReachable(addr)`.
- Receive: unpack → update table → drop heartbeats → deliver to `receiveTopic_` if `dst == own address` or broadcast (decrementing `hops`). No forwarding: `routePacket` is declared but has no definition.
- `NetworkPacket` (structs/network_packet.hpp):
  - fields `type` (DATA/ACK/NACK/HEARTBEAT), `hops` (default 1), `dstAddress`, `srcAddress`, `checksum`
  - `payload` is a `ListBuffer<uint8_t,200>`

Network frame (payload of one datalink frame, big-endian addresses):

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `type` (0 DATA, 1 ACK, 2 NACK, 3 HEARTBEAT) |
| 1 | 1 | `hops` |
| 2 | 2 | `dstAddress` (MSB first) |
| 4 | 2 | `srcAddress` (MSB first) |
| 6 | 1 | checksum = 8-bit sum of all other frame bytes |
| 7 | 1 | payload length N |
| 8 | N | payload |

### TransportCallback (include/ExVectrNetwork/transport_callback.hpp, src/transport_callback.cpp)
- `TransportCallback(uint16_t port[, Network_Interface&])`.
- `TransportData { srcAddress, srcPort, dstAddress, dstPort, ListArray<uint8_t> data }`.
- `send(const TransportData&)` sends one info segment, then `ceil(N/128)` data segments, immediately and back to back.
- Received, CRC-checked messages are published on `getReceiveTopic()`.
- Every node packet passes through every `TransportCallback` attached to the node; each one filters on `dstPort == port_`.
- Delivery is best effort: no ACK, no retransmit. A message is lost if any of its segments is lost.

Segment = `NetworkPacket.payload`, laid out as data followed by an 8-byte trailer:

| Offset | Size | Field |
|---|---|---|
| 0 | k | Segment body: info (see below) or up to 128 data bytes |
| k+0 | 2 | srcPort (MSB first) |
| k+2 | 2 | dstPort |
| k+4 | 2 | order: 0 = info, 1..n = data segment number |
| k+6 | 1 | message ID (uint8, incremented per message, wraps) |
| k+7 | 1 | transport identifier = `transportSimpleVersion + transportSimpleID` = 2 |

Info segment body (k = 5):

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | number of data segments |
| 2 | 2 | total bytes |
| 4 | 1 | low 8 bits of `Core::computeCrc` (CRC-16-CCITT) over the message |

- The receiver treats any packet whose ID differs from the last ID as the info segment; the `order` field is not used for this.
- Data segments are stored until `curSegment_ == numSegments_`, then reassembled, checked (CRC and length) and published.

### TransportTopic\<T\> (include/ExVectrNetwork/transport_topic.hpp)
- `TransportTopic(channel, dstAddress, node, topic, noReceive = false)`.
- Owns a `TransportCallback(channel)`.
- Every item published on `topic` is sent as `sizeof(T)` raw bytes to `dstAddress:channel`.
- A received message is published on `topic` if `srcPort == channel` and its size equals `sizeof(T)`.
- The subscriber excludes itself on publish, so received items are not echoed back out.
- `T` must be trivially copyable and identical (layout and endianness) on both ends. Use `__attribute__((packed))`.

### TopicIO (include/ExVectrNetwork/physicallayers/topic_io.hpp)
- A `HAL::DigitalIO` over a `Topic<const List<uint8_t>&>`: `writeData` publishes and the subscriber buffers received bytes (1024-byte ring buffer).
- Type is `IO_TYPE_t::TOPIC`, `writable()` = 255, parameters are unsupported.
- Used to connect two `Datalink`s in software for testing.

## Usage
As in `src/main.cpp:128-161`:
```cpp
Net::Datalink_SX1280 datalinkSX1280(NSS, NRESET, RFBUSY, DIO1, TXEN, RXEN); // app class, implements Datalink_Interface
Net::NetworkNode networkNode(1, datalinkSX1280, 2 * Core::SECONDS);       // address 1, heartbeat every 200 ms

Core::Topic<Net::PacketAttitude> attitudeDataTopic;
Net::TransportTopic<Net::PacketAttitude>
    attitudeDataTransport(10, UINT16_MAX, networkNode, attitudeDataTopic); // port 10, broadcast

attitudeDataTopic.publish(Net::PacketAttitude(rates, quat));              // goes out over the radio
```
Software loopback with the generic datalink:
```cpp
Core::Topic<const Core::List<uint8_t>&> wire;
Net::TopicIO ioA(wire), ioB(wire);
Net::Datalink linkA(ioA), linkB(ioB);
Net::NetworkNode a(1, linkA), b(2, linkB);
```

## Conventions
- One `TransportTopic` per port. Sender and receiver must use the same port, because the receiver checks `srcPort == own port`.
- Keep a message within one segment (≤ 128 bytes). Multi-segment reassembly is broken (see Known issues).
- Frame budget: 8 (network header) + 8 (transport trailer) + 128 = 144 bytes, within the 200-byte `NetworkPacket` payload, the 200-byte SX1280 frame and the 230-byte generic datalink frame.
- Each message costs 2 frames on air (info + data). Throttle publishers (main.cpp throttles in its telemetry callbacks).
- Namespace `VCTR::Net`. Include specific headers (`ExVectrNetwork/network_node.hpp`, `.../transport_topic.hpp`) rather than the umbrella, because of the guard collision with ExVectrPackets.

## How this project uses it
- Datalink: `Net::Datalink_SX1280` (`src/datalink_sx1280.{hpp,cpp}`, SX128XLT LoRa, max frame 200) implements `Datalink_Interface`. The generic `Datalink` and `TopicIO` are not used.
- Node: `Net::NetworkNode networkNode(1, datalinkSX1280, 2 * Core::SECONDS)` (`src/main.cpp:131`). Vehicle address is 1; heartbeat every 200 ms.
- TransportTopics (`src/main.cpp:145-161`), all to broadcast `UINT16_MAX`:

| Port | Type | sizeof | Producer / direction |
|---|---|---|---|
| 10 | `Net::PacketAttitude` | 18 | `attitudeTelemetryCallback` (main.cpp:189), throttled to 5 Hz, vehicle → ground |
| 11 | `Net::PacketPosition` | 16 | `positionTelemetryCallback` (main.cpp:225), 10 Hz |
| 12 | `Net::PacketGPS` | 23 | `gnssTelemetryCallback` (main.cpp:262), 1 Hz |
| 50 | `VehicleState` (src/telemetry.hpp:98) | small packed | vehicle → ground |
| 51 | `MissionState` (src/telemetry.hpp:122) | 24 + enums | vehicle → ground |
| 100 | `uint8_t` (`connections`) | 1 | |
| 1000 | `Telecommand` (src/telecommand.hpp:33) | packed | ground → vehicle |

- All of these are under 128 bytes, so the multi-segment bug is currently latent.

## Known issues & limitations
Paths are relative to `lib/ExVectrNetwork/`.

| Severity | Location | Issue | Impact |
|---|---|---|---|
| High | src/transport_callback.cpp:228 | Reassembly compares `order == curSeg`, where `order` is the order of the *last received* packet, instead of each buffered segment's order (the correct line is commented out at :226). | Any message > 128 bytes (> 1 segment) is reassembled from a single segment, fails the length/CRC check and is dropped. Single-segment messages work. |
| Med | src/transport_callback.cpp:170-200 | A packet is classified as the info segment by "ID changed", not by `order == 0`. The info body (`payload[0..4]`) is read without checking the payload is ≥ 13 bytes. | If the info segment is lost (LoRa), the first data segment is parsed as info (garbage counts) and that message is lost. A fresh receiver (`rcvID_ = 0`) also misreads a message whose ID is 0 after wrap. |
| Med | src/transport_callback.cpp:170-204 | There is one reassembly state per port, with no sender address in the key. | Two senders on the same port interleave and corrupt each other's messages. |
| Med | src/transport_callback.cpp:30 | `transmitTopicSubr_` gets a callback but is never subscribed to `transmitTopic_`. | Publishing on `TransportCallback::getTransmitTopic()` does nothing. Call `send()` directly. |
| Med | include/ExVectrNetwork/transport_topic.hpp:50-53 | The constructor's `noReceive` argument is never stored (`noReceive_` stays false). | Send-only topics still receive (and republish) incoming data unless `setNoReceive()` is called. |
| Low | src/transport_callback.cpp:71, 242 | The CRC-16-CCITT is truncated to 8 bits. | Weaker corruption detection (1/256 false accept). |
| Med | include/ExVectrNetwork/network_node.hpp:135 | `routePacket` is declared but never defined, and `hops` is never used for forwarding. | No multi-hop or relay; a node only talks to nodes on its own link. |
| Med | src/network_node.cpp:177-186 | `unpackPacket` reads `data[0..7]` before checking `data.size() >= 8`. | Out-of-bounds read on short or garbled frames. |
| Low | src/network_node.cpp:103-105 | Local delivery publishes the original `packet`, not `packetSend`, so `srcAddress` isn't set. Broadcasts are not delivered locally. | Local subscribers see an uninitialised source address; broadcast traffic is not looped back. |
| Low | include/ExVectrNetwork/network_node.hpp:27 | `networkVersion` is documented as part of the checksum but never used. 8-bit sum checksum. | Incompatible network versions aren't rejected; weak integrity check. |
| Low | src/network_node.cpp:28 | Comment says heartbeat every 1/5 of the timeout; the code uses 1/10. | Doc drift only. |
| Low | src/network_node.cpp:30 | The constructor always registers with `getSystemScheduler()`. | You can't put a node on another scheduler. |
| Med | src/datalink.cpp:84-89, 179-189 | No arbitration: a node sends BLOCK whenever it thinks the medium is free. Two nodes can BLOCK simultaneously. | Interleaved, corrupt frames on shared buses (not CRC-protected at this layer). |
| Med | src/datalink.cpp:100-101, 133-145 | After a DATA header the length byte is read without checking it is available. `size` is derived from `readLen` measured *before* the header bytes were consumed, and the `readData` return value is ignored. | On byte-streaming physical layers (UART) garbage lengths or bytes enter the frame. |
| Low | src/datalink.cpp:61 | `taskInit` sets the block timestamp to `NowNs() - 1 s` with the comment "listen for an extra second". With the 100 ms release time the medium is released immediately (it should be `+ 1 s`). | No startup listen period when joining a running network. |
| Med | src/topic_io.cpp:38 | Loop counter is `uint8_t i` while `i < item.size()`, and items up to 1024 bytes are accepted. | Infinite loop (hang) on any item > 255 bytes. |
| Med | lib/ExVectrPackets/include/ExVectrPackets.hpp:1 vs include/ExVectrNetwork.hpp:1 | Both umbrella headers use the include guard `EXVECTRNETWORK_H_`. | Whichever is included second is silently empty. |

## Extending
- **New physical layer**: implement `HAL::DigitalIO` (`readable/readData/writable/writeData`, types, params) and pass it to `Net::Datalink`.
- **New datalink** (radio with its own framing, like `Datalink_SX1280`):
  - Derive from `Datalink_Interface` (plus `Core::Task_Periodic`).
  - Override `dataframeReceiveFunc` to queue a frame.
  - Publish each complete received frame on `receiveTopic_`.
  - Reject frames larger than your MTU (must be ≥ 144 for single-segment transport).
- **New transported message**: define a packed POD (ideally in ExVectrPackets), pick an unused port, and declare `TransportTopic<T>(port, dst, networkNode, topic)` on both ends. Keep `sizeof(T) ≤ 128` until the reassembly bug is fixed.
- **Reliability**: ACK/NACK packet types exist in `NetworkPacketType` but are unused. A reliable transport would be a new class beside `TransportCallback`, using another transport identifier byte.
