#ifndef COMMS_VEHICLE_NETWORK_HPP
#define COMMS_VEHICLE_NETWORK_HPP

/**
 * The vehicle's ExVectrNetwork stack without any platform code (no Arduino, no ExVectrCore), so it is
 * host-tested (test/test_vehicle_network). VehicleComms (vehicle_comms.hpp) wraps it in a Core task with the
 * Teensy USB Serial and UART streams.
 *
 * - Links: StreamFrameLink (COBS + CRC-16) on USB (bench ground station) and on the UART to the ExVectrLink RX
 *   module (node 3), which routes to the TX module (node 2) and the ground station (node 0) over the air.
 * - The vehicle is node 1, an end node (never forwards). It runs one Node per link, both address 1, so each link
 *   gets its own telemetry set at its own rates: the USB set on USB, the radio set on the UART (a single end node
 *   would send every frame on every link). Both nodes share one PortMux, so a frame arriving on both links (ground
 *   station on USB and radio at once) is delivered once (sequence filter).
 * - Each node sends the vehicle heartbeat (1 Hz) on its link. Heartbeats and LINK_STATS of other nodes are
 *   informational only (node table, incompatibility flag).
 * - isUartPeerSeen(): any valid frame from node 3 on the UART within kUartPeerTimeoutNs (3 s), the
 *   VEHICLE_STATUS UART_PEER_SEEN flag. Information only.
 * - isGroundStationPresent(): any valid frame from node 0 on either link within kGsTimeoutNs (2 s). This is the
 *   observable of the existing radioConnectionLoss failure, as NetworkNode::isNodeReachable(0) with its 2 s
 *   timeout was on the old SX1280 link.
 * - TEMPORARY (removed in T9b): the legacy Telecommand on port LegacyTelecommandPacket::kPort, from node 0 only,
 *   is handed unchanged to the TelecommandSinkI (main.cpp publishes it on the existing telecommand topic).
 */

#include <stddef.h>
#include <stdint.h>

#include "ExVectrNetwork/TimeSourceI.hpp"
#include "ExVectrNetwork/link/StreamFrameLink.hpp"
#include "ExVectrNetwork/net/Node.hpp"
#include "ExVectrNetwork/transport/PacketPort.hpp"
#include "ExVectrNetwork/transport/PortMux.hpp"

#include "telemetry_mapping.hpp"

namespace VCTR {
namespace Comms {

/// Receives accepted legacy telecommands (TEMPORARY, T9b).
class TelecommandSinkI {
public:
  virtual ~TelecommandSinkI() = default;
  virtual void onTelecommand(const Telecommand &tc) = 0;
};

class VehicleNetwork : public network::PacketListenerI<LegacyTelecommandPacket> {
public:
  static constexpr uint8_t kVehicleNode = 1;
  static constexpr uint8_t kGroundStationNode = 0;
  static constexpr uint32_t kTickHz = 100; ///< sendTelemetry() call rate: the fastest USB rate
  /// Ground-station presence timeout: the 2 s of the old NetworkNode(1, datalink, 2 s). Information only since
  /// 2026-11-20 (VEHICLE_STATUS gs_present); radioConnectionLoss uses isRadioLinkAlive().
  static constexpr int64_t kGsTimeoutNs = 2000000000LL;
  static constexpr uint8_t kRouterNode = 2; ///< the ExVectrLink TX module (Router, Pocket side of the air link)
  /// Radio link timeout (radioConnectionLoss): no radio packet (see isRadioLinkAlive) for longer than this.
  static constexpr int64_t kRadioTimeoutNs = 2000000000LL;
  /// Boot grace of the radio link check: before the first radio packet the link counts as alive for this long
  /// after construction (boot), so the RX module and the air link can come up.
  static constexpr int64_t kRadioStartupGraceNs = 10000000000LL;
  static constexpr uint8_t kShipNode = 3; ///< the ExVectrLink RX module (ship radio node) on the UART
  /// UART_PEER_SEEN window: any valid frame from node 3 on the UART within this time (information only).
  static constexpr int64_t kUartPeerTimeoutNs = 3000000000LL;
  static constexpr size_t kUsbTxRing = 2048;
  static constexpr size_t kUartTxRing = 1024;

  VehicleNetwork(network::TimeSourceI &time, network::ByteStreamI &usb, network::ByteStreamI &uart,
                 TelecommandSinkI &telecommands);

  /// Reads both links (frames -> node table, telecommands) and drains their TX rings. Never blocks.
  void pollLinks();
  /// Heartbeats (1 Hz per link).
  void pollNodes();

  /// Builds and queues the telemetry due on this tick (call at kTickHz with an incrementing tick):
  /// USB set at the USB rates on USB, radio set at the air rates on the UART. Rates come from the generated
  /// packet metadata (kUsbRateHz, kAirRateHz). TASK_TIMING and LOG_TEXT are sent by the caller (sendUsb).
  void sendTelemetry(uint32_t tick, const TelemetrySnapshot &s);
  /// True on the ticks where TASK_TIMING is due on USB.
  static bool taskTimingDue(uint32_t tick);
  /// CALIBRATION period on USB: 0.5 Hz (the schema's integer rate fields cannot express it, kUsbRateHz is 0).
  static constexpr uint32_t kCalibrationPeriodTicks = 2 * kTickHz;
  /// AIR_ACTUATORS: one actuator sample every 2 ticks (50 Hz, 20 ms), two samples per packet at 25 Hz.
  static constexpr uint32_t kActuatorSampleTicks = 2;
  static constexpr uint32_t kAirActuatorsPhase = 6; ///< send tick phase (a sampling tick, so the newest is fresh)
  /// True on the ticks where CALIBRATION is due on USB (every kCalibrationPeriodTicks, phase 17).
  static bool calibrationDue(uint32_t tick);
  /// True if a packet at `hz` is due on `tick` with phase `phase` (spreads the packets over the ticks).
  static bool due(uint32_t tick, uint16_t hz, uint32_t phase);

  /// Sends a generated packet on the USB link only.
  template <class T>
  bool sendUsb(uint8_t dst, const T &packet) {
    return sendOn(usbNode_, dst, packet);
  }

  /// Any valid frame from node 0 (ground station) on USB or UART within kGsTimeoutNs. Information only
  /// (VEHICLE_STATUS gs_present, display); it no longer drives radioConnectionLoss.
  bool isGroundStationPresent() const;
  /// Radio link rule (user decision 2026-11-20, drives radioConnectionLoss): a "radio packet" is a valid DATA frame
  /// that arrives on the UART with source node 0 (ground station, relayed by the Router) or node 2 (Router /
  /// Pocket: RC_STATE, LINK_STATS ...), i.e. from the far side of the air link. Heartbeats of any node, frames
  /// from node 3 (the ship node itself) and everything on USB never count. Alive while the last radio packet is at
  /// most kRadioTimeoutNs old; before the first radio packet since boot, alive only during the boot grace
  /// (kRadioStartupGraceNs after construction).
  bool isRadioLinkAlive() const;
  /// A radio packet (see isRadioLinkAlive) really arrived within kRadioTimeoutNs; NO boot grace. For telemetry
  /// "connected" and the connection counter (review 2026-11-20: the failure predicate above is true during the
  /// boot grace before any packet, which made the display report a connection that never existed).
  bool isRadioPacketRecent() const;
  /// A radio packet (see isRadioLinkAlive) has arrived since boot.
  bool radioPacketSeen() const { return radioTap_.seen; }
  /// A valid frame from the ship node (node 3) arrived on the UART (not USB) within kUartPeerTimeoutNs. Read from
  /// the UART node's node table (last frame time per source), so no raw byte polling. Information only: the
  /// NetworkStatus the ground station gets over USB reflects the USB node, so a dead UART is invisible there;
  /// VEHICLE_STATUS.flags UART_PEER_SEEN carries this. Vehicle behaviour never depends on it (safety.md 4.1).
  bool isUartPeerSeen() const;
  /// A present node reported another network version or schema hash than ours.
  bool anyIncompatiblePeer() const;

  void onPacket(const LegacyTelecommandPacket &packet, const network::RxInfo &info) override;

  const network::Node &usbNode() const { return usbNode_; }
  const network::Node &uartNode() const { return uartNode_; }
  const network::FrameLinkI &usbLink() const { return usbLink_; }
  const network::FrameLinkI &uartLink() const { return uartLink_; }
  uint32_t telecommandsAccepted() const { return telecommandsAccepted_; }
  uint32_t telecommandsRejected() const { return telecommandsRejected_; }

private:
  static network::NodeConfig nodeConfig();
  static network::EgressFilter uartFilter();

  template <class T>
  bool sendOn(network::Node &node, uint8_t dst, const T &packet) {
    uint8_t buf[T::kMaxSize];
    size_t n = packet.encode(buf, sizeof buf);
    if (n == 0) return false;
    return node.send(dst, T::kPort, buf, n, static_cast<network::wire::Qos>(T::kQos), T::kLatest);
  }

  network::TimeSourceI &time_;
  TelecommandSinkI &telecommands_;
  network::StreamFrameLink<kUsbTxRing> usbLink_;
  network::StreamFrameLink<kUartTxRing> uartLink_;
  network::Node usbNode_;
  network::Node uartNode_;
  network::PortMux<2> mux_;
  network::PacketPort<LegacyTelecommandPacket> telecommandPort_; ///< TEMPORARY (T9b)

  /// Sits between the UART link and the UART node: records radio packets (isRadioLinkAlive), forwards every frame.
  struct RadioTap : network::FrameSinkI {
    network::FrameSinkI *next = nullptr;
    bool seen = false;
    int64_t lastNs = 0;
    void onFrame(const network::FrameView &f, uint8_t linkIndex) override;
  };
  RadioTap radioTap_;
  int64_t startNs_ = 0;
  uint32_t telecommandsAccepted_ = 0, telecommandsRejected_ = 0;
  ActuatorHistory actuatorHistory_; ///< AIR_ACTUATORS samples, one every kActuatorSampleTicks
};

} // namespace Comms
} // namespace VCTR

#endif
