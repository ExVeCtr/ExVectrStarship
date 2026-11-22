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
static_assert(Packets::AirPose::kAirRateHz == 25, "the schema AIR_POSE air rate is 25 Hz (every 4th tick)");
static_assert(kHz % Packets::AirPose::kAirRateHz == 0, "air rate must divide the tick rate");
static_assert(Packets::AirGnss::kAirRateHz == 10 && kHz % Packets::AirGnss::kAirRateHz == 0,
              "AIR_GNSS 10 Hz must divide the tick rate");
static_assert(Packets::AirGnssInfo::kAirRateHz == 1 && kHz % Packets::AirGnssInfo::kAirRateHz == 0,
              "AIR_GNSS_INFO 1 Hz must divide the tick rate");
static_assert(Packets::Gnss::kUsbRateHz == 10, "GNSS USB rate is 10 Hz");
static_assert(kHz % Packets::HandsetTelem::kAirRateHz == 0, "air rate must divide the tick rate");
static_assert(kHz % Packets::AirActuators::kAirRateHz == 0, "air rate must divide the tick rate");
static_assert(kHz % Packets::AirMission::kAirRateHz == 0, "air rate must divide the tick rate");
static_assert((VehicleNetwork::kAirActuatorsPhase % VehicleNetwork::kActuatorSampleTicks) == 0,
              "AIR_ACTUATORS must be sent on a sampling tick");
static_assert(kHz % Packets::RcApplied::kUsbRateHz == 0, "USB rate must divide the tick rate");
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
    : time_(time), telecommands_(telecommands), usbLink_(usb, time), uartLink_(uart, time), usbNode_(nodeConfig(), time),
      uartNode_(nodeConfig(), time),
      telecommandPort_(usbNode_, mux_, LegacyTelecommandPacket::kPort, kGroundStationNode, Qos::Event, false),
      usbRcTap_(Safety::RcLink::Usb, controlSources_, rcMapper_, motorInterlock_, kVehicleNode),
      uartRcTap_(Safety::RcLink::Uart, controlSources_, rcMapper_, motorInterlock_, kVehicleNode) {
  const int usbIndex = usbNode_.addLink(usbLink_);
  const int uartIndex = uartNode_.addLink(uartLink_, uartFilter());
  // Interpose the taps (same link indices): USB link -> usbRcTap_ -> USB node,
  // UART link -> uartRcTap_ -> radioTap_ -> UART node.
  usbRcTap_.next = &usbNode_;
  usbLink_.setSink(usbRcTap_, static_cast<uint8_t>(usbIndex));
  radioTap_.next = &uartNode_;
  uartRcTap_.next = &radioTap_;
  uartLink_.setSink(uartRcTap_, static_cast<uint8_t>(uartIndex));
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
  // Control-source arbitration (T9b-1): a change resets the transition detectors.
  const int64_t now = time_.nowNs();
  controlSources_.update(now, committed_);
  if (controlSources_.sourceChangedSinceLast()) {
    rcMapper_.onActiveChanged(controlSources_.active());
    sourceChangePending_ = true;
  }
  // The interlock follows the table's active source (a change clears its observed transition).
  motorInterlock_.update(now, controlSources_.active());
}

bool VehicleNetwork::activeSourceFresh(int64_t nowNs, int64_t maxAgeNs, Safety::ControlSourceId &id) const {
  Safety::ControlSourceSnapshot cs;
  if (!controlSources_.snapshot(cs)) cs = controlSources_.live();
  id = cs.active;
  return cs.active != Safety::ControlSourceId::None && cs.fresh(cs.active, nowNs, maxAgeNs);
}

bool VehicleNetwork::takeSourceChange(Safety::ControlSourceId &newActive) {
  if (!sourceChangePending_) return false;
  sourceChangePending_ = false;
  newActive = controlSources_.active();
  return true;
}

void VehicleNetwork::fillRcTelemetry(TelemetrySnapshot &s) {
  Safety::ControlSourceSnapshot cs;
  if (!controlSources_.snapshot(cs)) cs = controlSources_.live(); // comms task is the writer
  Comms::fillRcTelemetry(s, cs, rcMapper_, time_.nowNs(), motorInterlock_.needToggle(time_.nowNs()));
  // Degraded (information only, clears by itself, never a failure): the ship node (3) was not heard on the UART
  // for 3 s while RADIO RC_STATE is arriving. Any valid frame from node 3 counts (heartbeat or LINK_STATS), so this
  // is "UART_HEARTBEAT_LOST" in the broad sense. LINK_STATS_LOST (bit 1) and TOF_LOST (bit 0) are T9b-6.
  s.degradedBits = 0;
  if ((s.ctrlFlags & Packets::VehicleStatus::kCtrlFlagsRadioFresh) && !isUartPeerSeen())
    s.degradedBits |= Packets::VehicleStatus::kDegradedBitsUartHeartbeatLost;
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
  if (due(t, Packets::RcApplied::kUsbRateHz, kRcAppliedPhase)) sendOn(usbNode_, gs, makeRcApplied(s)); // USB only (0x84)

  // Radio set on the UART (RX module -> air -> TX module -> ground station), air rates.
  if (t % kActuatorSampleTicks == 0) actuatorHistory_.push(makeActuatorSample(s));
  if (due(t, Packets::VehicleStatus::kAirRateHz, 0)) sendOn(uartNode_, gs, makeVehicleStatus(s));
  if (due(t, Packets::AirPose::kAirRateHz, 5)) sendOn(uartNode_, gs, makeAirPose(s));
  if (due(t, Packets::AirActuators::kAirRateHz, kAirActuatorsPhase)) sendOn(uartNode_, gs, makeAirActuators(actuatorHistory_));
  if (due(t, Packets::AirMission::kAirRateHz, 9)) sendOn(uartNode_, gs, makeAirMission(s));
  if (due(t, Packets::HandsetTelem::kAirRateHz, 7)) sendOn(uartNode_, bc, makeHandsetTelem(s));
  if (due(t, Packets::AirGnss::kAirRateHz, kAirGnssPhase)) sendOn(uartNode_, gs, makeAirGnss(s));
  if (due(t, Packets::AirGnssInfo::kAirRateHz, kAirGnssInfoPhase)) sendOn(uartNode_, gs, makeAirGnssInfo(s));
}

bool VehicleNetwork::isGroundStationPresent() const {
  return usbNode_.isPresent(kGroundStationNode) || uartNode_.isPresent(kGroundStationNode);
}

void VehicleNetwork::RadioTap::onFrame(const network::FrameView &f, uint8_t linkIndex) {
  network::wire::NetHeader h;
  if (network::wire::decodeNetHeader(f.data, f.len, h) &&
      h.type == static_cast<uint8_t>(network::wire::FrameType::Data) &&
      f.len >= network::wire::kNetHeaderLen + network::wire::kDataHeaderLen &&
      (h.src == kGroundStationNode || h.src == kRouterNode)) {
    seen = true;
    lastNs = f.rxTimeNs;
  }
  if (next) next->onFrame(f, linkIndex);
}

bool VehicleNetwork::isRadioPacketRecent() const {
  return radioTap_.seen && time_.nowNs() - radioTap_.lastNs <= kRadioTimeoutNs;
}

bool VehicleNetwork::controlSourceLost(bool committed) const {
  Safety::ControlSourceSnapshot cs;
  if (!controlSources_.snapshot(cs)) return false; // writer busy on all 8 retries (not expected: cooperative scheduler); the next 10 Hz check decides
  return cs.failsafeDue(time_.nowNs(), committed, Safety::ControlSourceTable::kCtrlTimeoutNs);
}

bool VehicleNetwork::isUartPeerSeen() const {
  const network::NodeEntry *e = uartNode_.entry(kShipNode);
  return e && e->valid && (time_.nowNs() - e->lastSeenNs) <= kUartPeerTimeoutNs;
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
