#include "vehicle_network.hpp"

namespace VCTR {
namespace Comms {

using network::wire::Qos;

namespace {

constexpr uint32_t kHz = VehicleNetwork::kTickHz;
static_assert(kHz % Packets::Pose::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::ImuRaw::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::Actuators::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::VehicleStatus::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::Mission::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::Estimator::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::Gnss::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::HandsetTelem::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::TaskTiming::kUsbRateHz == 0, "USB rate must divide the tick rate");
static_assert(kHz % Packets::VehicleStatus::kAirRateHz == 0, "air rate must divide the tick rate");
static_assert(kHz % Packets::AirPose::kAirRateHz == 0, "air rate must divide the tick rate");
static_assert(kHz % Packets::HandsetTelem::kAirRateHz == 0, "air rate must divide the tick rate");
static_assert(LegacyTelecommandPacket::kPort < 0x80, "the legacy telecommand must pass the radio-path filters");

} // namespace

network::NodeConfig VehicleNetwork::nodeConfig() {
  network::NodeConfig cfg;
  cfg.address = kVehicleNode;
  cfg.role = network::wire::Role::Vehicle;
  cfg.forwarding = false; // end node: never forwards
  cfg.schemaHash = Packets::kSchemaHash;
  cfg.presenceTimeoutNs = kGsTimeoutNs;
  return cfg;
}

network::EgressFilter VehicleNetwork::uartFilter() {
  network::EgressFilter f;
  f.portLimit = 0x80; // the USB-only debug range never goes towards the radio (design 3.3.3)
  return f;
}

VehicleNetwork::VehicleNetwork(network::TimeSourceI &time, network::ByteStreamI &usb, network::ByteStreamI &uart,
                               TelecommandSinkI &telecommands)
    : telecommands_(telecommands), usbLink_(usb, time), uartLink_(uart, time), usbNode_(nodeConfig(), time),
      uartNode_(nodeConfig(), time),
      telecommandPort_(usbNode_, mux_, LegacyTelecommandPacket::kPort, kGroundStationNode, Qos::Event, false) {
  usbNode_.addLink(usbLink_);
  uartNode_.addLink(uartLink_, uartFilter());
  // One transport for both links: a frame arriving on both is delivered once.
  usbNode_.setDataSink(mux_);
  uartNode_.setDataSink(mux_);
  telecommandPort_.setListener(this);
}

void VehicleNetwork::pollLinks() {
  usbLink_.poll();
  uartLink_.poll();
}

void VehicleNetwork::pollNodes() {
  usbNode_.poll();
  uartNode_.poll();
}

bool VehicleNetwork::due(uint32_t tick, uint16_t hz, uint32_t phase) {
  if (hz == 0) return false;
  const uint32_t d = kTickHz / hz;
  return d != 0 && (tick % d) == (phase % d);
}

bool VehicleNetwork::taskTimingDue(uint32_t tick) { return due(tick, Packets::TaskTiming::kUsbRateHz, 13); }

bool VehicleNetwork::calibrationDue(uint32_t tick) { return (tick % kCalibrationPeriodTicks) == 17; }

void VehicleNetwork::sendTelemetry(uint32_t t, const TelemetrySnapshot &s) {
  constexpr uint8_t gs = kGroundStationNode;
  constexpr uint8_t bc = network::kBroadcastAddr;

  // USB set (ExVectrPackets docs/telemetry-sets.md). AIR_POSE is not sent on USB (POSE supersedes it).
  if (due(t, Packets::Pose::kUsbRateHz, 0)) sendOn(usbNode_, gs, makePose(s));
  if (due(t, Packets::ImuRaw::kUsbRateHz, 0)) sendOn(usbNode_, gs, makeImuRaw(s));
  if (due(t, Packets::Actuators::kUsbRateHz, 1)) sendOn(usbNode_, gs, makeActuators(s));
  if (due(t, Packets::VehicleStatus::kUsbRateHz, 0)) sendOn(usbNode_, gs, makeVehicleStatus(s));
  if (due(t, Packets::Mission::kUsbRateHz, 2)) sendOn(usbNode_, gs, makeMission(s));
  if (due(t, Packets::Estimator::kUsbRateHz, 4)) sendOn(usbNode_, gs, makeEstimator(s));
  if (due(t, Packets::Gnss::kUsbRateHz, 3)) sendOn(usbNode_, gs, makeGnss(s));
  if (due(t, Packets::HandsetTelem::kUsbRateHz, 7)) sendOn(usbNode_, bc, makeHandsetTelem(s));
  if (calibrationDue(t)) sendOn(usbNode_, gs, makeCalibration(s)); // USB only (port 0x93), 0.5 Hz

  // Radio set on the UART (RX module -> air -> TX module -> ground station), air rates.
  if (due(t, Packets::VehicleStatus::kAirRateHz, 0)) sendOn(uartNode_, gs, makeVehicleStatus(s));
  if (due(t, Packets::AirPose::kAirRateHz, 5)) sendOn(uartNode_, gs, makeAirPose(s));
  if (due(t, Packets::HandsetTelem::kAirRateHz, 7)) sendOn(uartNode_, bc, makeHandsetTelem(s));
}

bool VehicleNetwork::isGroundStationPresent() const {
  return usbNode_.isPresent(kGroundStationNode) || uartNode_.isPresent(kGroundStationNode);
}

bool VehicleNetwork::anyIncompatiblePeer() const {
  for (uint8_t a = 0; a < network::kMaxTrackedNodes; a++) {
    if (a == kVehicleNode) continue;
    const network::NodeEntry *u = usbNode_.entry(a);
    const network::NodeEntry *r = uartNode_.entry(a);
    if (u && u->incompatible && usbNode_.isPresent(a)) return true;
    if (r && r->incompatible && uartNode_.isPresent(a)) return true;
  }
  return false;
}

void VehicleNetwork::onPacket(const LegacyTelecommandPacket &packet, const network::RxInfo &info) {
  // TEMPORARY (T9b): only the ground station commanded the vehicle on the old port 1000.
  if (info.src != LegacyTelecommandPacket::kAcceptedSource) {
    telecommandsRejected_++;
    return;
  }
  telecommandsAccepted_++;
  telecommands_.onTelecommand(packet.tc);
}

} // namespace Comms
} // namespace VCTR
