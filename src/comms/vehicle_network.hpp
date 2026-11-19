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
  /// Ground-station presence timeout: the 2 s of the old NetworkNode(1, datalink, 2 s) (radioConnectionLoss).
  static constexpr int64_t kGsTimeoutNs = 2000000000LL;
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
  /// True if a packet at `hz` is due on `tick` with phase `phase` (spreads the packets over the ticks).
  static bool due(uint32_t tick, uint16_t hz, uint32_t phase);

  /// Sends a generated packet on the USB link only.
  template <class T>
  bool sendUsb(uint8_t dst, const T &packet) {
    return sendOn(usbNode_, dst, packet);
  }

  /// Any valid frame from node 0 (ground station) on USB or UART within kGsTimeoutNs.
  bool isGroundStationPresent() const;
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

  TelecommandSinkI &telecommands_;
  network::StreamFrameLink<kUsbTxRing> usbLink_;
  network::StreamFrameLink<kUartTxRing> uartLink_;
  network::Node usbNode_;
  network::Node uartNode_;
  network::PortMux<2> mux_;
  network::PacketPort<LegacyTelecommandPacket> telecommandPort_; ///< TEMPORARY (T9b)
  uint32_t telecommandsAccepted_ = 0, telecommandsRejected_ = 0;
};

} // namespace Comms
} // namespace VCTR

#endif
