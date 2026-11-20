// Host tests of src/comms/vehicle_network.cpp (the vehicle's ExVectrNetwork stack): pio test -e native
//
// Peers are real ExVectrNetwork nodes over in-memory byte pipes: a ground station (node 0) on USB, and on the UART
// nodes standing in for what the RX module forwards (node 0 data, node 3 heartbeats). Covers:
//  - ground-station presence (the radioConnectionLoss observable): 2 s timeout, any valid frame from node 0 on
//    either link, nothing else counts;
//  - the TEMPORARY legacy telecommand adapter: from node 0 only, unchanged struct, delivered once when it arrives
//    on both links;
//  - telemetry sets: USB set at the USB rates on USB, radio set at the air rates on the UART, no USB-only port on
//    the UART, heartbeats on both links;
//  - constant behaviour with nothing draining a link (no blocking, frames dropped and counted);
//  - no heap use after construction.
#include <unity.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <atomic>
#include <new>

#include "comms/vehicle_network.hpp"

using namespace VCTR;
using namespace VCTR::Comms;
using namespace VCTR::network;

// ---- allocation hook ----
static std::atomic<unsigned long> g_allocs{0};
void *operator new(size_t n) {
  g_allocs++;
  void *p = malloc(n ? n : 1);
  if (!p) abort();
  return p;
}
void *operator new[](size_t n) { return operator new(n); }
void operator delete(void *p) noexcept { free(p); }
void operator delete[](void *p) noexcept { free(p); }
void operator delete(void *p, size_t) noexcept { free(p); }
void operator delete[](void *p, size_t) noexcept { free(p); }

// ---- test doubles ----
struct FakeClock : TimeSourceI {
  int64_t t = 5000000000LL;
  int64_t nowNs() override { return t; }
  void advanceMs(int64_t ms) { t += ms * 1000000LL; }
};

struct MemPipe {
  uint8_t d[1 << 16];
  size_t rd = 0, wr = 0;
  size_t size() const { return wr - rd; }
  bool put(const uint8_t *p, size_t n) {
    if (rd == wr) rd = wr = 0;
    if (wr + n > sizeof d) return false;
    memcpy(d + wr, p, n);
    wr += n;
    return true;
  }
};

struct MemStream : ByteStreamI {
  MemPipe *in, *out;
  bool blockWrites = false;
  MemStream(MemPipe *i, MemPipe *o) : in(i), out(o) {}
  size_t readable() override { return in->size(); }
  size_t read(uint8_t *p, size_t n) override {
    if (n > in->size()) n = in->size();
    memcpy(p, in->d + in->rd, n);
    in->rd += n;
    return n;
  }
  size_t writable() override { return blockWrites ? 0 : 4096; }
  size_t write(const uint8_t *p, size_t n) override { return out->put(p, n) ? n : 0; }
};

struct Sink : TelecommandSinkI {
  int count = 0;
  Telecommand last = {};
  void onTelecommand(const Telecommand &tc) override {
    count++;
    last = tc;
  }
};

/// Counts DATA frames per port (and the destination of the last one).
struct PortCounter : DataSinkI {
  uint32_t n[256] = {};
  uint8_t lastDst[256] = {};
  void onData(const RxInfo &info, const uint8_t *, uint8_t) override {
    n[info.port]++;
    lastDst[info.port] = info.dst;
  }
};

/// A peer end node on the far end of one pipe pair.
struct Peer {
  MemStream io;
  StreamFrameLink<4096> link;
  Node node;
  PortCounter counter;
  static NodeConfig cfg(uint8_t addr, wire::Role role) {
    NodeConfig c;
    c.address = addr;
    c.role = role;
    c.schemaHash = Packets::kSchemaHash;
    return c;
  }
  Peer(MemPipe *in, MemPipe *out, TimeSourceI &clk, uint8_t addr, wire::Role role)
      : io(in, out), link(io, clk), node(cfg(addr, role), clk) {
    node.addLink(link);
    node.setDataSink(counter);
  }
  void poll() {
    link.poll();
  }
  bool sendTelecommand(uint8_t type, int16_t param) {
    LegacyTelecommandPacket p;
    p.tc.type = static_cast<TelecommandType>(type);
    p.tc.subsystem = Subsystem_Magneto;
    p.tc.paramInt = param;
    p.tc.paramFloat = 0.5f;
    uint8_t buf[8];
    p.encode(buf, sizeof buf);
    bool ok = node.send(VehicleNetwork::kVehicleNode, LegacyTelecommandPacket::kPort, buf, 8, wire::Qos::Event,
                        false);
    link.poll();
    return ok;
  }
};

struct Rig {
  FakeClock clk;
  MemPipe usbUp, usbDown, uartUp, uartDown; // Up: peer -> vehicle
  MemStream usbIo{&usbUp, &usbDown};
  MemStream uartIo{&uartUp, &uartDown};
  Sink sink;
  VehicleNetwork net{clk, usbIo, uartIo, sink};
  Peer gsUsb{&usbDown, &usbUp, clk, 0, wire::Role::GroundStation};
  Peer gsUart{&uartDown, &uartUp, clk, 0, wire::Role::GroundStation}; // node 0 frames forwarded by the RX module
  void pollAll() {
    net.pollLinks();
    gsUsb.poll();
    gsUart.poll();
    net.pollLinks();
  }
};

static Rig *rig = nullptr;

void setUp() { rig = new Rig(); }
void tearDown() {
  delete rig;
  rig = nullptr;
}

// ---------------------------------------------------------------- presence (radioConnectionLoss observable)

void test_gs_absent_at_boot() {
  rig->pollAll();
  TEST_ASSERT_FALSE(rig->net.isGroundStationPresent());
}

void test_gs_heartbeat_on_usb_gives_presence_for_2s() {
  rig->gsUsb.node.sendHeartbeat();
  rig->pollAll();
  TEST_ASSERT_TRUE(rig->net.isGroundStationPresent());
  rig->clk.advanceMs(2000); // exactly 2 s: still present
  TEST_ASSERT_TRUE(rig->net.isGroundStationPresent());
  rig->clk.t += 1; // just over 2 s: lost (old NetworkNode timeout 2 s)
  TEST_ASSERT_FALSE(rig->net.isGroundStationPresent());
  rig->gsUsb.node.sendHeartbeat(); // and it comes back
  rig->pollAll();
  TEST_ASSERT_TRUE(rig->net.isGroundStationPresent());
}

void test_gs_data_on_uart_gives_presence() {
  // Over the radio path the RX module drops node 0 heartbeats: only node 0 data frames reach the vehicle.
  TEST_ASSERT_TRUE(rig->gsUart.sendTelecommand(Telecommand_MissionSelect, 0));
  rig->pollAll();
  TEST_ASSERT_TRUE(rig->net.isGroundStationPresent());
}

void test_other_nodes_never_give_gs_presence() {
  Peer rx(&rig->uartDown, &rig->uartUp, rig->clk, 3, wire::Role::LinkRx);
  Peer tx(&rig->uartDown, &rig->uartUp, rig->clk, 2, wire::Role::LinkTx);
  for (int i = 0; i < 5; i++) {
    rx.node.sendHeartbeat();
    tx.node.sendHeartbeat();
    rx.poll();
    tx.poll();
    rig->net.pollLinks();
  }
  TEST_ASSERT_TRUE(rig->net.uartNode().isPresent(3));
  TEST_ASSERT_TRUE(rig->net.uartNode().isPresent(2));
  TEST_ASSERT_FALSE(rig->net.isGroundStationPresent());
}

// ---------------------------------------------------------------- UART_PEER_SEEN (information only)

void test_uart_peer_seen_from_node3_on_uart_for_3s() {
  Peer rx(&rig->uartDown, &rig->uartUp, rig->clk, 3, wire::Role::LinkRx);
  TEST_ASSERT_FALSE(rig->net.isUartPeerSeen());
  rx.node.sendHeartbeat();
  rx.poll();
  rig->net.pollLinks();
  TEST_ASSERT_TRUE(rig->net.isUartPeerSeen());
  rig->clk.advanceMs(3000); // exactly 3 s: still seen
  TEST_ASSERT_TRUE(rig->net.isUartPeerSeen());
  rig->clk.t += 1; // just over 3 s: cleared
  TEST_ASSERT_FALSE(rig->net.isUartPeerSeen());
  rx.node.sendHeartbeat(); // and it comes back
  rx.poll();
  rig->net.pollLinks();
  TEST_ASSERT_TRUE(rig->net.isUartPeerSeen());
}

void test_uart_peer_seen_ignores_usb_and_other_nodes() {
  // Node 3 frames on USB never set it (only the UART counts).
  Peer rxOnUsb(&rig->usbDown, &rig->usbUp, rig->clk, 3, wire::Role::LinkRx);
  rxOnUsb.node.sendHeartbeat();
  rxOnUsb.poll();
  rig->net.pollLinks();
  TEST_ASSERT_TRUE(rig->net.usbNode().isPresent(3));
  TEST_ASSERT_FALSE(rig->net.isUartPeerSeen());
  // Frames from the other nodes on the UART (ground station 0, TX module 2) do not either.
  Peer tx(&rig->uartDown, &rig->uartUp, rig->clk, 2, wire::Role::LinkTx);
  tx.node.sendHeartbeat();
  tx.poll();
  rig->gsUart.node.sendHeartbeat();
  rig->pollAll();
  TEST_ASSERT_TRUE(rig->net.uartNode().isPresent(2));
  TEST_ASSERT_TRUE(rig->net.uartNode().isPresent(0));
  TEST_ASSERT_FALSE(rig->net.isUartPeerSeen());
}

void test_uart_peer_seen_from_node3_data_frame() {
  Peer rx(&rig->uartDown, &rig->uartUp, rig->clk, 3, wire::Role::LinkRx);
  const uint8_t b[2] = {1, 2};
  TEST_ASSERT_TRUE(rx.node.send(VehicleNetwork::kVehicleNode, 0x01, b, 2, wire::Qos::Link, true));
  rx.poll();
  rig->net.pollLinks();
  TEST_ASSERT_TRUE(rig->net.isUartPeerSeen());
}

void test_corrupted_bytes_do_not_count() {
  const uint8_t junk[] = {0x12, 0x34, 0x56, 0x00, 0x05, 0x01, 0x40, 0x03, 0xFF, 0x00};
  rig->usbUp.put(junk, sizeof junk);
  rig->net.pollLinks();
  TEST_ASSERT_FALSE(rig->net.isGroundStationPresent());
  TEST_ASSERT_TRUE(rig->net.usbLink().stats.rxBad > 0);
}

// ---------------------------------------------------------------- TEMPORARY legacy telecommand adapter

void test_telecommand_from_gs_on_usb_delivered_unchanged() {
  TEST_ASSERT_TRUE(rig->gsUsb.sendTelecommand(Telecommand_Arm, 0xA5));
  rig->pollAll();
  TEST_ASSERT_EQUAL_INT(1, rig->sink.count);
  TEST_ASSERT_EQUAL_UINT8(Telecommand_Arm, rig->sink.last.type);
  TEST_ASSERT_EQUAL_UINT8(Subsystem_Magneto, rig->sink.last.subsystem);
  TEST_ASSERT_EQUAL_INT16(0xA5, rig->sink.last.paramInt);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, rig->sink.last.paramFloat);
}

void test_telecommand_from_gs_on_uart_delivered() {
  TEST_ASSERT_TRUE(rig->gsUart.sendTelecommand(Telecommand_SystemReset, 0xC5));
  rig->pollAll();
  TEST_ASSERT_EQUAL_INT(1, rig->sink.count);
  TEST_ASSERT_EQUAL_INT16(0xC5, rig->sink.last.paramInt);
}

void test_telecommand_from_other_node_rejected() {
  Peer tx(&rig->uartDown, &rig->uartUp, rig->clk, 2, wire::Role::LinkTx);
  TEST_ASSERT_TRUE(tx.sendTelecommand(Telecommand_Arm, 0xA5));
  rig->net.pollLinks();
  TEST_ASSERT_EQUAL_INT(0, rig->sink.count);
  TEST_ASSERT_EQUAL_UINT32(1, rig->net.telecommandsRejected());
}

void test_telecommand_on_both_links_delivered_once() {
  // The same frame (same sequence number) arriving on USB and over the radio: one delivery. Matters because
  // SimulationMode toggles.
  LegacyTelecommandPacket p;
  p.tc.type = Telecommand_SimulationMode;
  p.tc.paramInt = 0xA9;
  uint8_t payload[8];
  p.encode(payload, sizeof payload);
  // Build one network frame by hand: CTRL (VER 1, DATA, QoS EVENT, not latest) TTL DST SRC PORT SEQ payload
  uint8_t frame[6 + 8] = {0x48, 3, 1, 0, LegacyTelecommandPacket::kPort, 42};
  memcpy(frame + 6, payload, 8);
  uint8_t wireBuf[wire::kMaxStreamWireLen];
  size_t n = 0;
  TEST_ASSERT_TRUE(wire::encodeStreamFrame(frame, sizeof frame, wireBuf, sizeof wireBuf, n));
  rig->usbUp.put(wireBuf, n);
  rig->uartUp.put(wireBuf, n);
  rig->net.pollLinks();
  TEST_ASSERT_EQUAL_INT(1, rig->sink.count);
  TEST_ASSERT_EQUAL_UINT8(Telecommand_SimulationMode, rig->sink.last.type);
}

void test_telecommand_wrong_length_rejected() {
  uint8_t buf[9] = {Telecommand_Arm, 0, 0xA5, 0, 0, 0, 0, 0, 0};
  rig->gsUsb.node.send(1, LegacyTelecommandPacket::kPort, buf, 9, wire::Qos::Event, false);
  rig->gsUsb.node.send(1, LegacyTelecommandPacket::kPort, buf, 7, wire::Qos::Event, false);
  rig->pollAll();
  TEST_ASSERT_EQUAL_INT(0, rig->sink.count);
}

// ---------------------------------------------------------------- telemetry sets and rates

void test_telemetry_sets_and_rates() {
  TelemetrySnapshot s;
  for (uint32_t tick = 0; tick < 100; tick++) { // one second at 100 Hz
    rig->net.pollLinks();
    rig->net.pollNodes();
    rig->net.sendTelemetry(tick, s);
    rig->pollAll();
    rig->clk.advanceMs(10);
  }
  const PortCounter &usb = rig->gsUsb.counter;
  const PortCounter &uart = rig->gsUart.counter;
  // USB set (USB rates)
  TEST_ASSERT_EQUAL_UINT32(100, usb.n[Packets::Pose::kPort]);
  TEST_ASSERT_EQUAL_UINT32(100, usb.n[Packets::ImuRaw::kPort]);
  TEST_ASSERT_EQUAL_UINT32(50, usb.n[Packets::Actuators::kPort]);
  TEST_ASSERT_EQUAL_UINT32(20, usb.n[Packets::VehicleStatus::kPort]);
  TEST_ASSERT_EQUAL_UINT32(20, usb.n[Packets::Mission::kPort]);
  TEST_ASSERT_EQUAL_UINT32(20, usb.n[Packets::Estimator::kPort]);
  TEST_ASSERT_EQUAL_UINT32(5, usb.n[Packets::Gnss::kPort]);
  TEST_ASSERT_EQUAL_UINT32(2, usb.n[Packets::HandsetTelem::kPort]);
  TEST_ASSERT_EQUAL_UINT32(0, usb.n[Packets::AirPose::kPort]);
  TEST_ASSERT_EQUAL_UINT32(1, usb.n[Packets::Calibration::kPort]); // 0.5 Hz: tick 17 of the first 2 s
  TEST_ASSERT_EQUAL_UINT8(0, usb.lastDst[Packets::Calibration::kPort]);
  TEST_ASSERT_EQUAL_UINT8(0xFF, usb.lastDst[Packets::HandsetTelem::kPort]);
  TEST_ASSERT_EQUAL_UINT8(0, usb.lastDst[Packets::Pose::kPort]);
  // Radio set on the UART (air rates), nothing from the USB-only range
  TEST_ASSERT_EQUAL_UINT32(5, uart.n[Packets::VehicleStatus::kPort]);
  TEST_ASSERT_EQUAL_UINT32(10, uart.n[Packets::AirPose::kPort]);
  TEST_ASSERT_EQUAL_UINT32(2, uart.n[Packets::HandsetTelem::kPort]);
  TEST_ASSERT_EQUAL_UINT32(25, uart.n[Packets::AirActuators::kPort]);
  TEST_ASSERT_EQUAL_UINT32(5, uart.n[Packets::AirMission::kPort]);
  TEST_ASSERT_EQUAL_UINT32(0, usb.n[Packets::AirActuators::kPort]); // air set only (USB has ACTUATORS)
  TEST_ASSERT_EQUAL_UINT32(0, usb.n[Packets::AirMission::kPort]);   // air set only (USB has MISSION)
  uint32_t uartTotal = 0, usbOnly = 0;
  for (int p = 0; p < 256; p++) {
    uartTotal += uart.n[p];
    if (p >= 0x80) usbOnly += uart.n[p];
  }
  TEST_ASSERT_EQUAL_UINT32(0, usbOnly);
  TEST_ASSERT_EQUAL_UINT32(47, uartTotal);
  // Vehicle heartbeat on both links
  const NodeEntry *hu = rig->gsUsb.node.entry(1);
  const NodeEntry *hr = rig->gsUart.node.entry(1);
  TEST_ASSERT_TRUE(hu->hasHeartbeat);
  TEST_ASSERT_TRUE(hr->hasHeartbeat);
  TEST_ASSERT_EQUAL_UINT8(1, hu->role);
  TEST_ASSERT_EQUAL_HEX32(Packets::kSchemaHash, hr->schemaHash);
  TEST_ASSERT_FALSE(hu->incompatible);
}

void test_task_timing_rate() {
  int n = 0;
  for (uint32_t t = 0; t < 100; t++) n += VehicleNetwork::taskTimingDue(t) ? 1 : 0;
  TEST_ASSERT_EQUAL_INT(5, n);
}

void test_calibration_rate() {
  int n = 0;
  for (uint32_t t = 0; t < 1000; t++) n += VehicleNetwork::calibrationDue(t) ? 1 : 0; // 10 s
  TEST_ASSERT_EQUAL_INT(5, n);
  TEST_ASSERT_EQUAL_UINT16(0, Packets::Calibration::kUsbRateHz); // rate lives in the vehicle (0.5 Hz)
}

void test_nothing_draining_links_drops_frames_without_blocking() {
  rig->usbIo.blockWrites = true;  // USB not connected / host not reading
  rig->uartIo.blockWrites = true; // RX module absent
  TelemetrySnapshot s;
  for (uint32_t tick = 0; tick < 500; tick++) {
    rig->net.pollLinks();
    rig->net.pollNodes();
    rig->net.sendTelemetry(tick, s);
    rig->clk.advanceMs(10);
  }
  TEST_ASSERT_TRUE(rig->net.usbLink().stats.txDropped > 0);
  TEST_ASSERT_TRUE(rig->net.uartLink().stats.txDropped > 0);
  TEST_ASSERT_EQUAL_UINT(0, rig->usbDown.size());
  // Reconnect: data flows again
  rig->usbIo.blockWrites = false;
  rig->net.pollLinks();
  rig->net.sendTelemetry(500, s);
  rig->pollAll();
  rig->pollAll();
  TEST_ASSERT_TRUE(rig->gsUsb.counter.n[Packets::Pose::kPort] > 0);
}

void test_incompatible_peer_flag() {
  NodeConfig c = Peer::cfg(3, wire::Role::LinkRx);
  c.schemaHash = 0xDEADBEEF;
  MemStream io(&rig->uartDown, &rig->uartUp);
  StreamFrameLink<1024> link(io, rig->clk);
  Node rx(c, rig->clk);
  rx.addLink(link);
  rx.sendHeartbeat();
  link.poll();
  rig->net.pollLinks();
  TEST_ASSERT_TRUE(rig->net.anyIncompatiblePeer());
  rig->clk.advanceMs(2500); // presence timeout (2 s) clears it
  TEST_ASSERT_FALSE(rig->net.anyIncompatiblePeer());
}

void test_no_heap_after_construction() {
  TelemetrySnapshot s;
  unsigned long before = g_allocs.load();
  for (uint32_t tick = 0; tick < 200; tick++) {
    rig->net.pollLinks();
    rig->net.pollNodes();
    rig->net.sendTelemetry(tick, s);
    rig->gsUsb.sendTelecommand(Telecommand_MissionSelect, 0);
    rig->pollAll();
    rig->clk.advanceMs(10);
  }
  TEST_ASSERT_EQUAL_UINT32(before, g_allocs.load());
}

/// With VEHICLE_NET_DUMP=<prefix> set, writes one second of raw USB and UART output to <prefix>_usb.bin and
/// <prefix>_uart.bin (for decoding with the ground station's exvectrnet-1 profile). Otherwise a no-op.
void test_dump_streams_for_ground_station() {
  const char *prefix = getenv("VEHICLE_NET_DUMP");
  if (!prefix) return;
  TelemetrySnapshot s;
  s.mode = VehicleMode::VehicleMode_Ready;
  s.att[0] = 0.01f;
  s.pos[5] = 1.25f;
  s.missionTime_ns = -10000000000LL;
  s.groundDistance = 0.5f;
  for (uint32_t tick = 0; tick < 100; tick++) {
    rig->net.pollNodes();
    rig->net.sendTelemetry(tick, s);
    rig->net.pollLinks();
    rig->clk.advanceMs(10);
  }
  char path[512];
  snprintf(path, sizeof path, "%s_usb.bin", prefix);
  FILE *f = fopen(path, "wb");
  TEST_ASSERT_NOT_NULL(f);
  fwrite(rig->usbDown.d + rig->usbDown.rd, 1, rig->usbDown.size(), f);
  fclose(f);
  snprintf(path, sizeof path, "%s_uart.bin", prefix);
  f = fopen(path, "wb");
  TEST_ASSERT_NOT_NULL(f);
  fwrite(rig->uartDown.d + rig->uartDown.rd, 1, rig->uartDown.size(), f);
  fclose(f);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_dump_streams_for_ground_station);
  RUN_TEST(test_gs_absent_at_boot);
  RUN_TEST(test_gs_heartbeat_on_usb_gives_presence_for_2s);
  RUN_TEST(test_gs_data_on_uart_gives_presence);
  RUN_TEST(test_other_nodes_never_give_gs_presence);
  RUN_TEST(test_uart_peer_seen_from_node3_on_uart_for_3s);
  RUN_TEST(test_uart_peer_seen_ignores_usb_and_other_nodes);
  RUN_TEST(test_uart_peer_seen_from_node3_data_frame);
  RUN_TEST(test_corrupted_bytes_do_not_count);
  RUN_TEST(test_telecommand_from_gs_on_usb_delivered_unchanged);
  RUN_TEST(test_telecommand_from_gs_on_uart_delivered);
  RUN_TEST(test_telecommand_from_other_node_rejected);
  RUN_TEST(test_telecommand_on_both_links_delivered_once);
  RUN_TEST(test_telecommand_wrong_length_rejected);
  RUN_TEST(test_telemetry_sets_and_rates);
  RUN_TEST(test_task_timing_rate);
  RUN_TEST(test_calibration_rate);
  RUN_TEST(test_nothing_draining_links_drops_frames_without_blocking);
  RUN_TEST(test_incompatible_peer_flag);
  RUN_TEST(test_no_heap_after_construction);
  return UNITY_END();
}
