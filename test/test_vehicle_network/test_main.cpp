// Host tests of src/comms/vehicle_network.cpp (the vehicle's ExVectrNetwork stack): pio test -e native
//
// Peers are real ExVectrNetwork nodes over in-memory byte pipes: a ground station (node 0) on USB, and on the UART
// nodes standing in for what the RX module forwards (node 0 data, node 3 heartbeats). Covers:
//  - ground-station presence (gs_present, information only): 2 s timeout, any valid frame from node 0 on
//    either link, nothing else counts;
//  - general link health (isRadioPacketRecent, information only): DATA frames from node 0 or 2 on the UART only,
//    2 s, no boot grace; heartbeats, node 3 and USB never count;
//  - GS control: CMD_STATE (0x13) from node 0 on USB and UART, session latch, per-link freshness, strict field
//    rejection, handset RC_STATE not interpreted at all, REQUEST session acceptance, the interlock fed through the taps;
//  - the control failsafe (controlSourceLost, T9b-3): only accepted CMD_STATE of the latched session keeps it alive;
//  - the deleted legacy telecommand port 0x7F: frames on it are ignored
//    on both links;
//  - telemetry sets: USB set at the USB rates on USB, radio set at the air rates on the UART, no USB-only port on
//    the UART, heartbeats on both links;
//  - constant behaviour with nothing draining a link (no blocking, frames dropped and counted);
//  - no heap use after construction.
#include <unity.h>

#include <math.h>
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

/// Counts DATA frames per port (and the destination of the last one).
struct PortCounter : DataSinkI {
  uint32_t n[256] = {};
  uint8_t lastDst[256] = {};
  void onData(const RxInfo &info, const uint8_t *, uint8_t) override {
    n[info.port]++;
    lastDst[info.port] = info.dst;
  }
};

constexpr uint8_t kLegacyTelecommandPort = 0x7F; // deleted: no subscription

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
  /// A GS data frame on the deleted legacy telecommand port 0x7F (8 bytes): nothing subscribes to it any more, but it
  /// is still a valid DATA frame from node 0 (presence, radio-packet recency).
  bool sendLegacyPortFrame() {
    uint8_t buf[8] = {5, 2, 0xA5, 0, 0, 0, 0, 0x3F};
    bool ok = node.send(VehicleNetwork::kVehicleNode, kLegacyTelecommandPort, buf, 8, wire::Qos::Event, false);
    link.poll();
    return ok;
  }
};

struct Rig {
  FakeClock clk;
  MemPipe usbUp, usbDown, uartUp, uartDown; // Up: peer -> vehicle
  MemStream usbIo{&usbUp, &usbDown};
  MemStream uartIo{&uartUp, &uartDown};
  VehicleNetwork net{clk, usbIo, uartIo};
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

// ---------------------------------------------------------------- ground-station presence (gs_present, information only)

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
  TEST_ASSERT_TRUE(rig->gsUart.sendLegacyPortFrame());
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

// ---------------------------------------------------------------- general link health (information only)

static void sendData(Peer &p, uint8_t port = 0x10) {
  const uint8_t b[4] = {1, 2, 3, 4};
  TEST_ASSERT_TRUE(p.node.send(VehicleNetwork::kVehicleNode, port, b, sizeof b, wire::Qos::Link, true));
  p.poll();
  rig->net.pollLinks();
}

void test_radio_no_boot_grace_nothing_seen_is_not_recent() {
  rig->pollAll();
  TEST_ASSERT_FALSE(rig->net.radioPacketSeen());
  TEST_ASSERT_FALSE(rig->net.isRadioPacketRecent()); // nothing was ever received: not "connected"
}

void test_radio_node2_data_on_uart_keeps_alive_with_exact_timeout() {
  Peer tx(&rig->uartDown, &rig->uartUp, rig->clk, 2, wire::Role::LinkTx); // Router / Pocket RC_STATE
  rig->clk.advanceMs(20000);
  TEST_ASSERT_FALSE(rig->net.isRadioPacketRecent());
  sendData(tx);
  TEST_ASSERT_TRUE(rig->net.radioPacketSeen());
  TEST_ASSERT_TRUE(rig->net.isRadioPacketRecent());
  rig->clk.advanceMs(2000); // exactly 2 s: recent
  TEST_ASSERT_TRUE(rig->net.isRadioPacketRecent());
  rig->clk.t += 1; // just over: not recent
  TEST_ASSERT_FALSE(rig->net.isRadioPacketRecent());
  rig->clk.advanceMs(5000); // restart after a gap
  sendData(tx);
  TEST_ASSERT_TRUE(rig->net.isRadioPacketRecent());
}

void test_radio_node0_data_on_uart_keeps_alive() {
  rig->clk.advanceMs(20000);
  TEST_ASSERT_TRUE(rig->gsUart.sendLegacyPortFrame()); // relayed by the Router
  rig->pollAll();
  TEST_ASSERT_TRUE(rig->net.isRadioPacketRecent());
}

void test_radio_heartbeats_node3_and_usb_never_count() {
  Peer tx(&rig->uartDown, &rig->uartUp, rig->clk, 2, wire::Role::LinkTx);
  Peer rx(&rig->uartDown, &rig->uartUp, rig->clk, 3, wire::Role::LinkRx);
  Peer txOnUsb(&rig->usbDown, &rig->usbUp, rig->clk, 2, wire::Role::LinkTx);
  rig->clk.advanceMs(20000);
  for (int i = 0; i < 5; i++) {
    tx.node.sendHeartbeat();     // node 2 heartbeat on the UART
    rx.node.sendHeartbeat();     // node 3 heartbeat on the UART
    tx.poll();
    rx.poll();
    rig->gsUart.node.sendHeartbeat(); // node 0 heartbeat on the UART
    rig->gsUsb.node.sendHeartbeat();  // ground station on USB
    rig->pollAll();
    sendData(rx);                      // node 3's own data (LINK_STATS): proves the UART only
    sendData(rig->gsUsb);              // node 0 data on USB
    const uint8_t b[2] = {1, 2};
    txOnUsb.node.send(VehicleNetwork::kVehicleNode, 0x10, b, 2, wire::Qos::Link, true); // node 2 data on USB
    txOnUsb.poll();
    rig->net.pollLinks();
  }
  TEST_ASSERT_TRUE(rig->net.uartNode().isPresent(2));
  TEST_ASSERT_TRUE(rig->net.uartNode().isPresent(3));
  TEST_ASSERT_TRUE(rig->net.isGroundStationPresent()); // gs_present (information) still works
  TEST_ASSERT_FALSE(rig->net.radioPacketSeen());
  TEST_ASSERT_FALSE(rig->net.isRadioPacketRecent());
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

// ---------------------------------------------------------------- deleted legacy telecommand port

void test_legacy_telecommand_port_is_not_subscribed() {
  // Frames on the old port 0x7F (any length, from the GS on either link) are valid DATA frames but go nowhere: no
  // listener, no request, no effect on the command state, no crash.
  TEST_ASSERT_TRUE(rig->gsUsb.sendLegacyPortFrame());
  TEST_ASSERT_TRUE(rig->gsUart.sendLegacyPortFrame());
  uint8_t buf[9] = {0};
  rig->gsUsb.node.send(1, kLegacyTelecommandPort, buf, 9, wire::Qos::Event, false);
  rig->pollAll();
  Safety::RequestMsg m;
  TEST_ASSERT_FALSE(rig->net.popRequest(m));
  TEST_ASSERT_EQUAL_UINT32(0, rig->net.requestsDropped());
  TEST_ASSERT_FALSE(rig->net.commandIngress().sessionLatched());
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
  TEST_ASSERT_EQUAL_UINT32(10, usb.n[Packets::Gnss::kPort]);
  TEST_ASSERT_EQUAL_UINT32(2, usb.n[Packets::HandsetTelem::kPort]);
  TEST_ASSERT_EQUAL_UINT32(0, usb.n[Packets::AirPose::kPort]);
  TEST_ASSERT_EQUAL_UINT32(10, usb.n[Packets::CmdApplied::kPort]);  // 10 Hz, USB only
  TEST_ASSERT_EQUAL_UINT32(0, rig->gsUart.counter.n[Packets::CmdApplied::kPort]);
  TEST_ASSERT_EQUAL_UINT32(1, usb.n[Packets::Calibration::kPort]); // 0.5 Hz: tick 17 of the first 2 s
  TEST_ASSERT_EQUAL_UINT8(0, usb.lastDst[Packets::Calibration::kPort]);
  TEST_ASSERT_EQUAL_UINT32(1, usb.n[Packets::MissionParams::kPort]); // 1 Hz on USB
  TEST_ASSERT_EQUAL_UINT32(1, uart.n[Packets::MissionParams::kPort]); // 0.2 Hz on the radio: tick 21 of the first 5 s
  TEST_ASSERT_EQUAL_UINT8(0xFF, usb.lastDst[Packets::HandsetTelem::kPort]);
  TEST_ASSERT_EQUAL_UINT8(0, usb.lastDst[Packets::Pose::kPort]);
  // Radio set on the UART (air rates), nothing from the USB-only range
  TEST_ASSERT_EQUAL_UINT32(5, uart.n[Packets::VehicleStatus::kPort]);
  TEST_ASSERT_EQUAL_UINT32(Packets::AirPose::kAirRateHz, uart.n[Packets::AirPose::kPort]);
  TEST_ASSERT_EQUAL_UINT32(25, uart.n[Packets::AirPose::kPort]);
  TEST_ASSERT_EQUAL_UINT32(2, uart.n[Packets::HandsetTelem::kPort]);
  TEST_ASSERT_EQUAL_UINT32(25, uart.n[Packets::AirActuators::kPort]);
  TEST_ASSERT_EQUAL_UINT32(5, uart.n[Packets::AirMission::kPort]);
  TEST_ASSERT_EQUAL_UINT32(10, uart.n[Packets::AirGnss::kPort]);
  TEST_ASSERT_EQUAL_UINT32(1, uart.n[Packets::AirGnssInfo::kPort]);
  TEST_ASSERT_EQUAL_UINT32(0, usb.n[Packets::AirGnss::kPort]);     // air set only (USB has GNSS)
  TEST_ASSERT_EQUAL_UINT32(0, usb.n[Packets::AirGnssInfo::kPort]);
  TEST_ASSERT_EQUAL_UINT32(0, usb.n[Packets::AirActuators::kPort]); // air set only (USB has ACTUATORS)
  TEST_ASSERT_EQUAL_UINT32(0, usb.n[Packets::AirMission::kPort]);   // air set only (USB has MISSION)
  uint32_t uartTotal = 0, usbOnly = 0;
  for (int p = 0; p < 256; p++) {
    uartTotal += uart.n[p];
    if (p >= 0x80) usbOnly += uart.n[p];
  }
  TEST_ASSERT_EQUAL_UINT32(0, usbOnly);
  TEST_ASSERT_EQUAL_UINT32(74, uartTotal); // 5 + 25 + 25 + 5 + 2 + 10 + 1 + 1 (MISSION_PARAMS, first 2 s)
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

// ---------------------------------------------------------------- GS control: CMD_STATE ingress taps, session, failsafe

static constexpr uint32_t kSessA = 0xA1B2C3D4u, kSessB = 0x0BADF00Du;
static uint8_t cmdSeq = 0;

struct Cmd {
  uint32_t session = kSessA;
  uint8_t go = 0, profile = 2, mode = 0, motors = 0, reset = 0, failsafe = 0;
  uint8_t flags = 1;
};

/// Puts one CMD_STATE network frame (DATA, CONTROL, DST vehicle, SRC `src`, port 0x13, SEQ `seq`) into a link's
/// receive pipe by hand, so the same sequence number can be sent on both links. `len`/`raw` allow malformed frames.
static void putCmd(MemPipe &pipe, uint8_t src, uint8_t seq, const Cmd &c, size_t len = 14, const uint8_t *raw = nullptr) {
  uint8_t payload[16] = {};
  if (raw) {
    memcpy(payload, raw, len < 16 ? len : 16);
  } else {
    Packets::CmdState cs;
    cs.flags = c.flags;
    cs.version = 1;
    cs.session = c.session;
    cs.go = static_cast<Packets::CmdGo>(c.go);
    cs.profile = c.profile;
    cs.vehicle_mode = static_cast<Packets::CmdVehicleMode>(c.mode);
    cs.motors = static_cast<Packets::CmdMotors>(c.motors);
    cs.reset_button = static_cast<Packets::CmdButton>(c.reset);
    cs.failsafe = static_cast<Packets::CmdFailsafe>(c.failsafe);
    TEST_ASSERT_EQUAL(14, (int)cs.encode(payload, sizeof payload));
  }
  uint8_t frame[6 + 16];
  const uint8_t hdr[6] = {0x48, 3, VehicleNetwork::kVehicleNode, src, Packets::CmdState::kPort, seq};
  memcpy(frame, hdr, 6);
  memcpy(frame + 6, payload, len);
  uint8_t wireBuf[wire::kMaxStreamWireLen];
  size_t n = 0;
  TEST_ASSERT_TRUE(wire::encodeStreamFrame(frame, 6 + len, wireBuf, sizeof wireBuf, n));
  TEST_ASSERT_TRUE(pipe.put(wireBuf, n));
}

/// One GS tick: a new frame on the UART (and on USB too if `both`), then a comms tick, then 40 ms.
static void gsTick(const Cmd &c, bool uart = true, bool usb = false) {
  cmdSeq++;
  if (uart) putCmd(rig->uartUp, 0, cmdSeq, c);
  if (usb) putCmd(rig->usbUp, 0, cmdSeq, c);
  rig->net.pollLinks();
  rig->net.pollNodes();
  rig->clk.advanceMs(40);
}
static void gsTicks(const Cmd &c, int n, bool uart = true, bool usb = false) {
  for (int i = 0; i < n; i++) gsTick(c, uart, usb);
}
static const Cmd kSafe;

void test_cmd_state_on_uart_is_accepted_from_node0() {
  cmdSeq = 0;
  gsTick(kSafe);
  TEST_ASSERT_TRUE(rig->net.commandIngress().sessionLatched());
  TEST_ASSERT_EQUAL_HEX32(kSessA, rig->net.commandIngress().session());
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_EQUAL_UINT8(1, s.ctrlSource);
  TEST_ASSERT_EQUAL_UINT16(0xC3D4, s.cmdSession);
  TEST_ASSERT_TRUE(s.rcAgeS >= 0.0f && s.rcAgeS < 0.1f);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsUartFresh);
  TEST_ASSERT_FALSE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsUsbFresh);
  uint32_t sess = 0;
  TEST_ASSERT_TRUE(rig->net.takeSessionChange(sess));
  TEST_ASSERT_EQUAL_HEX32(kSessA, sess);
  TEST_ASSERT_TRUE(rig->net.isRadioPacketRecent()); // node 0 data on the UART is general link health
}

void test_cmd_state_on_both_links_is_one_stream() {
  cmdSeq = 0;
  gsTicks(kSafe, 10, true, true);
  const auto &ing = rig->net.commandIngress();
  TEST_ASSERT_EQUAL_UINT32(10, ing.live().accepted); // the second copy is a duplicate
  TEST_ASSERT_EQUAL_UINT8(0, ing.rejectCounts()[3]); // a duplicate or late second copy is not a reject (feature/reject-count)
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsUartFresh);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsUsbFresh);
  TEST_ASSERT_TRUE(s.cmdApplied.usbAgeS < 0.1f && s.cmdApplied.uartAgeS < 0.1f);
  // unplug the USB: the UART copy keeps the state fresh and nothing else changes
  gsTicks(kSafe, 100, true, false);
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsUartFresh);
  TEST_ASSERT_FALSE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsUsbFresh);
  TEST_ASSERT_EQUAL_UINT8(1, s.ctrlSource);
  TEST_ASSERT_TRUE(s.cmdApplied.usbAgeS > 2.0f); // keeps counting
}

void test_cmd_state_from_other_nodes_is_rejected_and_never_latches() {
  cmdSeq = 0;
  putCmd(rig->uartUp, 2, 1, kSafe); // the Router / Pocket
  putCmd(rig->usbUp, 3, 1, kSafe);  // the ship node
  putCmd(rig->uartUp, 5, 1, kSafe);
  rig->net.pollLinks();
  rig->net.pollNodes();
  TEST_ASSERT_FALSE(rig->net.commandIngress().sessionLatched());
  TEST_ASSERT_EQUAL_UINT8(3, rig->net.commandIngress().rejectCounts()[1]);
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_EQUAL_UINT8(3, s.rcRejected);
  TEST_ASSERT_EQUAL_UINT8(3, s.cmdApplied.rejects[1]);
  TEST_ASSERT_EQUAL_UINT8(0, s.ctrlSource);
  TEST_ASSERT_TRUE(isnan(s.rcAgeS));
  TEST_ASSERT_EQUAL_UINT16(0, s.cmdSession);
}

void test_cmd_state_strict_enum_and_length_rejection_counted() {
  cmdSeq = 0;
  uint8_t good[14];
  Packets::CmdState cs;
  cs.flags = 1;
  cs.version = 1;
  cs.session = kSessA;
  cs.profile = 2;
  cs.encode(good, sizeof good);
  // every field out of range rejects the whole frame (flags, version, session 0, go, profile, mode, motors, reset,
  // failsafe, reserved)
  const struct { int off; uint8_t val; } bad[] = {{0, 0}, {0, 9}, {1, 2}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 2}, {7, 3},
                                                  {8, 2}, {9, 2}, {10, 2}, {11, 2}, {12, 1}, {13, 1}};
  int n = 0;
  for (const auto &b : bad) {
    uint8_t f[14];
    memcpy(f, good, 14);
    if (b.off == 2 || b.off == 3 || b.off == 4 || b.off == 5) { // session = 0 needs all four bytes cleared
      f[2] = f[3] = f[4] = f[5] = 0;
    } else {
      f[b.off] = b.val;
    }
    putCmd(rig->uartUp, 0, ++cmdSeq, kSafe, 14, f);
    n++;
  }
  putCmd(rig->uartUp, 0, ++cmdSeq, kSafe, 13, good); // one byte short
  uint8_t longer[15];
  memcpy(longer, good, 14);
  longer[14] = 0;
  putCmd(rig->uartUp, 0, ++cmdSeq, kSafe, 15, longer); // one byte too long
  n += 2;
  rig->net.pollLinks();
  rig->net.pollNodes();
  TEST_ASSERT_FALSE(rig->net.commandIngress().sessionLatched());
  TEST_ASSERT_EQUAL_UINT8(n, rig->net.commandIngress().rejectCounts()[0]);
  putCmd(rig->uartUp, 0, ++cmdSeq, kSafe, 14, good); // the unmodified one is accepted
  rig->net.pollLinks();
  TEST_ASSERT_TRUE(rig->net.commandIngress().sessionLatched());
}

void test_rc_state_is_not_interpreted_at_all() {
  // A handset RC_STATE from the Router (node 2, UART) or even from node 0 changes nothing on the vehicle.
  Peer tx(&rig->uartDown, &rig->uartUp, rig->clk, 2, wire::Role::LinkTx);
  Packets::RcState rc;
  rc.flags = 1;
  for (int i = 0; i < 16; i++) rc.ch[i] = 0;
  rc.ch[4] = 1000;
  rc.ch[7] = 1000; // SA up, SD up
  uint8_t buf[40] = {};
  rc.encode(buf, sizeof buf);
  for (int i = 0; i < 20; i++) {
    TEST_ASSERT_TRUE(tx.node.send(VehicleNetwork::kVehicleNode, Packets::RcState::kPort, buf, 34, wire::Qos::Control, true));
    tx.poll();
    TEST_ASSERT_TRUE(rig->gsUart.node.send(VehicleNetwork::kVehicleNode, Packets::RcState::kPort, buf, 34, wire::Qos::Control, true));
    rig->gsUart.poll();
    rig->net.pollLinks();
    rig->net.pollNodes();
    rig->clk.advanceMs(40);
  }
  const auto &ing = rig->net.commandIngress();
  TEST_ASSERT_FALSE(ing.sessionLatched());
  TEST_ASSERT_EQUAL_UINT8(0, ing.live().rejectedTotal); // not even a reject: it is not a command at all
  TEST_ASSERT_FALSE(rig->net.rcMapper().seen());
  TEST_ASSERT_FALSE(rig->net.motorInterlock().open(rig->clk.nowNs()));
  TEST_ASSERT_FALSE(rig->net.controlSourceLost(false));
  TEST_ASSERT_TRUE(rig->net.controlSourceLost(true)); // committed with no GS command at all: due
}

void test_heartbeats_and_other_ports_are_not_commands() {
  Peer tx(&rig->uartDown, &rig->uartUp, rig->clk, 2, wire::Role::LinkTx);
  tx.node.sendHeartbeat();
  tx.poll();
  sendData(tx, 0x14);
  rig->gsUart.node.sendHeartbeat();
  rig->gsUart.poll();
  rig->net.pollNodes();
  TEST_ASSERT_FALSE(rig->net.commandIngress().sessionLatched());
  TEST_ASSERT_EQUAL_UINT8(0, rig->net.commandIngress().live().rejectedTotal);
}

void test_source_goes_none_after_2s_and_age_is_reported() {
  cmdSeq = 0;
  gsTick(kSafe);
  rig->clk.advanceMs(1960); // exactly 2.0 s after the frame
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_EQUAL_UINT8(1, s.ctrlSource);
  rig->clk.advanceMs(1);
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_EQUAL_UINT8(0, s.ctrlSource);
  TEST_ASSERT_TRUE(s.rcAgeS > 2.0f);
  TEST_ASSERT_EQUAL_UINT16(0xC3D4, s.cmdSession); // the session stays latched
}

void test_foreign_session_rejected_multi_master_and_requests() {
  cmdSeq = 0;
  gsTicks(kSafe, 5);
  Cmd other;
  other.session = kSessB;
  putCmd(rig->usbUp, 0, 200, other);
  rig->net.pollLinks();
  rig->net.pollNodes();
  TEST_ASSERT_EQUAL_HEX32(kSessA, rig->net.commandIngress().session());
  TEST_ASSERT_EQUAL_UINT8(1, rig->net.commandIngress().rejectCounts()[2]);
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsMultiMaster);
  // REQUEST: only the latched session is allowed (design Q5)
  auto sendReq = [&](uint32_t session, size_t len = 16) {
    Packets::Request rq;
    rq.req_id = 1;
    rq.action = 12;
    rq.target = VehicleNetwork::kVehicleNode;
    rq.session = session;
    uint8_t b[16] = {};
    rq.encode(b, sizeof b);
    TEST_ASSERT_TRUE(rig->gsUsb.node.send(VehicleNetwork::kVehicleNode, Packets::Request::kPort, b, len, wire::Qos::Event, false));
    rig->gsUsb.poll();
    rig->net.pollLinks();
  };
  sendReq(kSessA);
  sendReq(kSessB);
  sendReq(0);
  sendReq(kSessA, 12); // an older sender without the session field decodes as 0
  TEST_ASSERT_EQUAL_UINT16(1, rig->net.commandIngress().requestsAccepted());
  TEST_ASSERT_EQUAL_UINT16(3, rig->net.commandIngress().requestsRefused());
}

/// T9b-5: a REQUEST arriving on a link is queued with that link; the ACK goes back on the same link only.
static void requestRoundTrip(bool overUart) {
  cmdSeq = 0;
  gsTicks(kSafe, 5, true, true); // latches kSessA on both links
  Peer &gs = overUart ? rig->gsUart : rig->gsUsb;
  Peer &other = overUart ? rig->gsUsb : rig->gsUart;
  Packets::Request rq;
  rq.req_id = 42;
  rq.action = 13;
  rq.target = VehicleNetwork::kVehicleNode;
  rq.arg_f = 25.0f;
  rq.session = kSessA;
  uint8_t b[16] = {};
  TEST_ASSERT_EQUAL(16, (int)rq.encode(b, sizeof b));
  TEST_ASSERT_TRUE(gs.node.send(VehicleNetwork::kVehicleNode, Packets::Request::kPort, b, 16, wire::Qos::Event, false));
  gs.poll();
  rig->net.pollLinks();
  Safety::RequestMsg m;
  TEST_ASSERT_TRUE(rig->net.popRequest(m));
  TEST_ASSERT_EQUAL_UINT16(42, m.reqId);
  TEST_ASSERT_EQUAL_UINT8(0, m.src);
  TEST_ASSERT_EQUAL_UINT8(overUart ? 1 : 0, m.link);
  TEST_ASSERT_EQUAL_FLOAT(25.0f, m.argF);
  TEST_ASSERT_EQUAL_HEX32(kSessA, m.session);
  const uint32_t before = gs.counter.n[Packets::RequestAck::kPort];
  TEST_ASSERT_TRUE(rig->net.sendRequestAck(m.link, m.src, m.reqId, 2, 3));
  rig->net.pollLinks();
  rig->gsUsb.poll();
  rig->gsUart.poll();
  TEST_ASSERT_EQUAL_UINT32(before + 1, gs.counter.n[Packets::RequestAck::kPort]);
  TEST_ASSERT_EQUAL_UINT32(0, other.counter.n[Packets::RequestAck::kPort]);
}
void test_request_over_usb_gets_its_ack_on_usb() { requestRoundTrip(false); }
void test_request_over_uart_gets_its_ack_on_uart() { requestRoundTrip(true); }

void test_request_inbox_is_bounded_and_does_not_block() {
  cmdSeq = 0;
  gsTicks(kSafe, 3);
  Packets::Request rq;
  rq.action = 12;
  rq.target = VehicleNetwork::kVehicleNode;
  rq.session = kSessA;
  uint8_t b[16] = {};
  for (uint16_t i = 1; i <= 20; i++) {
    rq.req_id = i;
    rq.encode(b, sizeof b);
    rig->gsUsb.node.send(VehicleNetwork::kVehicleNode, Packets::Request::kPort, b, 16, wire::Qos::Event, false);
    rig->gsUsb.poll();
    rig->net.pollLinks();
  }
  int n = 0;
  Safety::RequestMsg m;
  while (rig->net.popRequest(m)) n++;
  TEST_ASSERT_EQUAL(Safety::RequestInbox::kSize, n);
  TEST_ASSERT_TRUE(rig->net.requestsDropped() > 0);
}

void test_new_session_resets_detectors_and_needs_new_edges() {
  cmdSeq = 0;
  Cmd c;
  gsTicks(c, 5);                       // motors DISABLED
  c.motors = 1;
  gsTick(c);                           // observed DISABLED -> ENABLED
  TEST_ASSERT_TRUE(rig->net.motorInterlock().open(rig->clk.nowNs()));
  // the GS dies and restarts (new session) with motors already ENABLED: not committed, silent > 1 s -> latched
  rig->net.setCommitted(false, false);
  rig->clk.advanceMs(1200);
  rig->net.pollNodes();
  c.session = kSessB;
  cmdSeq = 0;
  gsTick(c);
  TEST_ASSERT_EQUAL_HEX32(kSessB, rig->net.commandIngress().session());
  TEST_ASSERT_FALSE(rig->net.motorInterlock().open(rig->clk.nowNs()));
  gsTicks(c, 10);
  TEST_ASSERT_FALSE(rig->net.motorInterlock().open(rig->clk.nowNs())); // ENABLED from the first state: never opens
  TEST_ASSERT_TRUE(rig->net.motorInterlock().needToggle(rig->clk.nowNs()));
  TEST_ASSERT_FALSE(rig->net.rcMapper().takeEvents().goEdge);
  c.motors = 0;
  gsTick(c);
  c.motors = 1;
  gsTick(c);
  TEST_ASSERT_TRUE(rig->net.motorInterlock().open(rig->clk.nowNs()));
}

void test_committed_vehicle_keeps_its_session_against_a_restarted_gs() {
  cmdSeq = 0;
  Cmd c;
  gsTicks(c, 5);
  rig->net.setCommitted(true, false);
  rig->net.pollNodes();
  rig->clk.advanceMs(1500);
  c.session = kSessB;
  cmdSeq = 0;
  gsTick(c);
  TEST_ASSERT_EQUAL_HEX32(kSessA, rig->net.commandIngress().session()); // rejected: committed
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsMultiMaster);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsSourceLatched);
  rig->net.setCommitted(true, true); // the failsafe put the mode in Failure
  rig->net.pollNodes();
  gsTick(c);
  TEST_ASSERT_EQUAL_HEX32(kSessB, rig->net.commandIngress().session()); // now the restarted GS may command
}

void test_gs_failsafe_level_is_visible_and_cleared_by_the_next_state() {
  cmdSeq = 0;
  Cmd c;
  gsTicks(c, 3);
  TEST_ASSERT_FALSE(rig->net.gsFailsafeRequested());
  c.failsafe = 1;
  gsTick(c);
  TEST_ASSERT_TRUE(rig->net.gsFailsafeRequested());
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsGsFailsafeReq);
  TEST_ASSERT_EQUAL_UINT8(1, s.cmdApplied.failsafe);
  c.failsafe = 0;
  gsTick(c);
  TEST_ASSERT_FALSE(rig->net.gsFailsafeRequested());
}

// ---- the one control failsafe rule

void test_control_failsafe_only_accepted_cmd_state_keeps_it_alive() {
  cmdSeq = 0;
  Peer rx(&rig->uartDown, &rig->uartUp, rig->clk, 3, wire::Role::LinkRx);
  Peer tx(&rig->uartDown, &rig->uartUp, rig->clk, 2, wire::Role::LinkTx);
  gsTick(kSafe);
  rig->net.setCommitted(true, false);
  rig->net.pollNodes();
  TEST_ASSERT_FALSE(rig->net.controlSourceLost(true));
  // the GS command stream stops: everything else keeps arriving for 2.0 s
  for (int i = 0; i < 49; i++) {
    rig->clk.advanceMs(40);
    if (i % 5 == 0) {
      tx.node.sendHeartbeat();
      rx.node.sendHeartbeat();
      tx.poll();
      rx.poll();
      sendData(tx, 0x14);                                          // node 2 non-command data
      rig->gsUart.sendLegacyPortFrame();  // GS telecommand over the radio
      putCmd(rig->uartUp, 2, 77, kSafe);                          // CMD_STATE from the Router: rejected
      Cmd other;
      other.session = kSessB;
      putCmd(rig->uartUp, 0, 77, other);                          // foreign session: rejected
      sendData(tx, Packets::RcState::kPort);                      // handset RC
    }
    rig->net.pollLinks();
    rig->net.pollNodes();
  }
  TEST_ASSERT_TRUE(rig->net.isRadioPacketRecent()); // general link health is fine ...
  TEST_ASSERT_FALSE(rig->net.controlSourceLost(true)); // ... 1.96 s + 40 ms = exactly 2.0 s since the state: not yet
  rig->clk.advanceMs(100);
  TEST_ASSERT_TRUE(rig->net.controlSourceLost(true)); // 2.0-2.1 s: failsafe
  TEST_ASSERT_TRUE(rig->net.isRadioPacketRecent());   // link health never decided anything
  gsTick(kSafe);                                       // a state coming back makes it fresh again (the latch is in main.cpp)
  TEST_ASSERT_FALSE(rig->net.controlSourceLost(true));
}

void test_control_failsafe_not_committed_never_due() { // D2-B
  cmdSeq = 0;
  gsTick(kSafe);
  rig->clk.advanceMs(5000);
  rig->net.pollNodes();
  TEST_ASSERT_FALSE(rig->net.controlSourceLost(false));
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_EQUAL_UINT8(0, s.ctrlSource);
  // committed with no GS command at all (a mission started without any): due at once
  TEST_ASSERT_TRUE(rig->net.controlSourceLost(true));
}

void test_control_failsafe_usb_only_and_uart_only_timing() {
  for (int link = 0; link < 2; link++) {
    delete rig;
    rig = new Rig();
    cmdSeq = 0;
    gsTick(kSafe, link == 1, link == 0);
    rig->net.setCommitted(true, false);
    rig->net.pollNodes();
    rig->clk.advanceMs(1960);
    TEST_ASSERT_FALSE(rig->net.controlSourceLost(true));
    rig->clk.advanceMs(1);
    TEST_ASSERT_TRUE(rig->net.controlSourceLost(true));
  }
}

void test_degraded_uart_heartbeat_lost_is_information_only() {
  cmdSeq = 0;
  Peer rx(&rig->uartDown, &rig->uartUp, rig->clk, 3, wire::Role::LinkRx);
  rx.node.sendHeartbeat();
  rx.poll();
  gsTick(kSafe);
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_EQUAL_HEX8(0, s.degradedBits);
  for (int i = 0; i < 80; i++) gsTick(kSafe); // commands keep arriving, the ship node is silent for 3.2 s
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_EQUAL_HEX8(Packets::VehicleStatus::kDegradedBitsUartHeartbeatLost, s.degradedBits);
  TEST_ASSERT_FALSE(rig->net.controlSourceLost(true)); // never a failsafe
  rx.node.sendHeartbeat(); // recovers by itself
  rx.poll();
  rig->net.pollLinks();
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_EQUAL_HEX8(0, s.degradedBits);
}

// ---------------------------------------------------------------- interlock fed through the real taps

/// One GS state with `motors`, then a comms tick, then 40 ms.
static void motorsTick(uint8_t motors, bool usb = false) {
  Cmd c;
  c.motors = motors;
  gsTick(c, !usb, usb);
}

void test_interlock_toggle_opens_and_boot_enabled_never_opens() {
  cmdSeq = 0;
  auto &il = rig->net.motorInterlock();
  for (int i = 0; i < 10; i++) motorsTick(1); // I1: ENABLED from the first frame
  TEST_ASSERT_FALSE(il.open(rig->clk.nowNs()));
  TelemetrySnapshot s;
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_TRUE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsInterlockNeedToggle);
  motorsTick(0);
  TEST_ASSERT_FALSE(il.open(rig->clk.nowNs()));
  motorsTick(1); // I2: observed off -> on
  TEST_ASSERT_TRUE(il.open(rig->clk.nowNs()));
  s = TelemetrySnapshot();
  rig->net.fillCmdTelemetry(s);
  TEST_ASSERT_FALSE(s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsInterlockNeedToggle);
  motorsTick(0); // the safe direction acts at once
  TEST_ASSERT_FALSE(il.open(rig->clk.nowNs()));
}

void test_interlock_works_the_same_over_usb() {
  cmdSeq = 0;
  auto &il = rig->net.motorInterlock();
  motorsTick(0, true);
  motorsTick(0, true);
  motorsTick(1, true);
  TEST_ASSERT_TRUE(il.open(rig->clk.nowNs()));
}

void test_interlock_rejected_frames_are_no_data() { // I7
  cmdSeq = 0;
  auto &il = rig->net.motorInterlock();
  motorsTick(0);
  motorsTick(0);
  motorsTick(1);
  TEST_ASSERT_TRUE(il.open(rig->clk.nowNs()));
  const int64_t lastGood = rig->clk.nowNs() - 40 * 1000000LL;
  for (int i = 0; i < 30; i++) { // ENABLED frames that are all rejected: foreign session, wrong source, bad field
    Cmd other;
    other.session = kSessB;
    other.motors = 1;
    putCmd(rig->uartUp, 0, ++cmdSeq, other);
    putCmd(rig->uartUp, 2, ++cmdSeq, kSafe);
    uint8_t bad[14] = {};
    putCmd(rig->uartUp, 0, ++cmdSeq, kSafe, 14, bad);
    rig->net.pollLinks();
    rig->net.pollNodes();
    rig->clk.advanceMs(40);
  }
  TEST_ASSERT_TRUE(rig->clk.nowNs() - lastGood > 1000000000LL);
  TEST_ASSERT_FALSE(il.open(rig->clk.nowNs()));
}

void test_spoofer_disarm_and_arm_interleaving_cannot_create_an_edge() {
  // The real GS sends motors DISABLED; a spoofer of session B sends ENABLED in between. The interlock never sees B.
  cmdSeq = 0;
  auto &il = rig->net.motorInterlock();
  motorsTick(0); // the real GS latches its session first
  for (int i = 0; i < 50; i++) {
    Cmd on;
    on.session = kSessB;
    on.motors = 1;
    putCmd(rig->uartUp, 0, 200 + i, on);
    motorsTick(0);
  }
  TEST_ASSERT_FALSE(il.open(rig->clk.nowNs()));
  TEST_ASSERT_FALSE(il.needToggle(rig->clk.nowNs()));
  TEST_ASSERT_EQUAL_HEX32(kSessA, rig->net.commandIngress().session());
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
    rig->gsUsb.sendLegacyPortFrame();
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
  RUN_TEST(test_gs_absent_at_boot);
  RUN_TEST(test_gs_heartbeat_on_usb_gives_presence_for_2s);
  RUN_TEST(test_gs_data_on_uart_gives_presence);
  RUN_TEST(test_other_nodes_never_give_gs_presence);
  RUN_TEST(test_radio_no_boot_grace_nothing_seen_is_not_recent);
  RUN_TEST(test_radio_node2_data_on_uart_keeps_alive_with_exact_timeout);
  RUN_TEST(test_radio_node0_data_on_uart_keeps_alive);
  RUN_TEST(test_radio_heartbeats_node3_and_usb_never_count);
  RUN_TEST(test_uart_peer_seen_from_node3_on_uart_for_3s);
  RUN_TEST(test_uart_peer_seen_ignores_usb_and_other_nodes);
  RUN_TEST(test_uart_peer_seen_from_node3_data_frame);
  RUN_TEST(test_corrupted_bytes_do_not_count);
  RUN_TEST(test_legacy_telecommand_port_is_not_subscribed);
  RUN_TEST(test_telemetry_sets_and_rates);
  RUN_TEST(test_task_timing_rate);
  RUN_TEST(test_calibration_rate);
  RUN_TEST(test_cmd_state_on_uart_is_accepted_from_node0);
  RUN_TEST(test_cmd_state_on_both_links_is_one_stream);
  RUN_TEST(test_cmd_state_from_other_nodes_is_rejected_and_never_latches);
  RUN_TEST(test_cmd_state_strict_enum_and_length_rejection_counted);
  RUN_TEST(test_rc_state_is_not_interpreted_at_all);
  RUN_TEST(test_heartbeats_and_other_ports_are_not_commands);
  RUN_TEST(test_source_goes_none_after_2s_and_age_is_reported);
  RUN_TEST(test_foreign_session_rejected_multi_master_and_requests);
  RUN_TEST(test_request_over_usb_gets_its_ack_on_usb);
  RUN_TEST(test_request_over_uart_gets_its_ack_on_uart);
  RUN_TEST(test_request_inbox_is_bounded_and_does_not_block);
  RUN_TEST(test_new_session_resets_detectors_and_needs_new_edges);
  RUN_TEST(test_committed_vehicle_keeps_its_session_against_a_restarted_gs);
  RUN_TEST(test_gs_failsafe_level_is_visible_and_cleared_by_the_next_state);
  RUN_TEST(test_control_failsafe_only_accepted_cmd_state_keeps_it_alive);
  RUN_TEST(test_control_failsafe_not_committed_never_due);
  RUN_TEST(test_control_failsafe_usb_only_and_uart_only_timing);
  RUN_TEST(test_degraded_uart_heartbeat_lost_is_information_only);
  RUN_TEST(test_interlock_toggle_opens_and_boot_enabled_never_opens);
  RUN_TEST(test_interlock_works_the_same_over_usb);
  RUN_TEST(test_interlock_rejected_frames_are_no_data);
  RUN_TEST(test_spoofer_disarm_and_arm_interleaving_cannot_create_an_edge);
  RUN_TEST(test_nothing_draining_links_drops_frames_without_blocking);
  RUN_TEST(test_incompatible_peer_flag);
  RUN_TEST(test_no_heap_after_construction);
  RUN_TEST(test_dump_streams_for_ground_station);
  return UNITY_END();
}
