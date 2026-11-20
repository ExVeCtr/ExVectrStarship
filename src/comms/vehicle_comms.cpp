#include "vehicle_comms.hpp"

#include <math.h>
#include <string.h>

#include "ExVectrCore/print.hpp"

namespace VCTR {
namespace Comms {

namespace {

/// The log queue that the Core print callbacks feed (set by the VehicleComms constructor).
LogTextQueue *gLogQueue = nullptr;

void printToLogM(const char *const &t) {
  if (gLogQueue) gLogQueue->append(Packets::Severity::kInfo, t);
}
void printToLogD(const char *const &t) {
  if (gLogQueue) gLogQueue->append(Packets::Severity::kDebug, t);
}
void printToLogW(const char *const &t) {
  if (gLogQueue) gLogQueue->append(Packets::Severity::kWarning, t);
}
void printToLogE(const char *const &t) {
  if (gLogQueue) gLogQueue->append(Packets::Severity::kError, t);
}

} // namespace

int64_t CoreTimeSource::nowNs() { return Core::NowNs(); }

// ---------------------------------------------------------------- LogTextQueue

void LogTextQueue::append(Packets::Severity severity, const char *text) {
  if (text == nullptr) return;
  for (const char *c = text; *c != '\0'; ++c) {
    if (partialLen_ == 0) partialSeverity_ = severity;
    if (*c == '\n') {
      finishLine();
      continue;
    }
    if (*c == '\r') continue;
    partial_[partialLen_++] = *c;
    if (partialLen_ >= kLineLen) finishLine();
  }
}

void LogTextQueue::finishLine() {
  if (partialLen_ == 0) return;
  if (count_ >= kLines) {
    dropped_++;
  } else {
    Line &l = lines_[(head_ + count_) % kLines];
    l.severity = partialSeverity_;
    l.len = static_cast<uint16_t>(partialLen_);
    memcpy(l.text, partial_, partialLen_);
    count_++;
  }
  partialLen_ = 0;
}

bool LogTextQueue::pop(Packets::LogText &out) {
  if (count_ == 0) return false;
  const Line &l = lines_[head_];
  out.severity = l.severity;
  size_t n = l.len < sizeof out.text ? l.len : sizeof out.text;
  memcpy(out.text, l.text, n);
  out.text_len = static_cast<uint16_t>(n);
  head_ = (head_ + 1) % kLines;
  count_--;
  return true;
}

// ---------------------------------------------------------------- VehicleComms

VehicleComms::VehicleComms(usb_serial_class &usb, HardwareSerialIMXRT &uart, TelemetryProviderI &provider,
                           Core::Topic<Telecommand> &telecommandTopic)
    : Task_Periodic("Vehicle Comms", kTickNs), uart_(uart), provider_(provider),
      telecommandTopic_(telecommandTopic), usbIo_(usb), uartIo_(uart), net_(time_, usbIo_, uartIo_, *this) {
  // Console text -> LOG_TEXT on USB. Subscribed at construction so boot messages are queued too (the queue
  // keeps the first kLines lines until the task runs).
  gLogQueue = &log_;
  printSubM_.setCallbackFunction(printToLogM);
  printSubD_.setCallbackFunction(printToLogD);
  printSubW_.setCallbackFunction(printToLogW);
  printSubE_.setCallbackFunction(printToLogE);
  printSubM_.subscribe(Core::getMessageTopic());
  printSubD_.subscribe(Core::getDebugTopic());
  printSubW_.subscribe(Core::getWarningTopic());
  printSubE_.subscribe(Core::getErrorTopic());

  Core::getSystemScheduler().addTask(*this);
  setPriority(kPriority);
}

void VehicleComms::taskInit() {
  // UART to the RX module: larger buffers so whole air packets (255 B bursts) fit between two ticks.
  // addMemoryFor* resets the ring indices, so it goes before begin().
  uart_.addMemoryForRead(uartRxMem_, sizeof uartRxMem_);
  uart_.addMemoryForWrite(uartTxMem_, sizeof uartTxMem_);
  uart_.begin(kUartBaud); // 8N1
  // USB Serial is started by the platform (Serial.begin in Core::initialise).
  provider_.begin();
}

void VehicleComms::taskThread() {
  // Receive (node table, telecommands) and drain what is queued.
  net_.pollLinks();
  net_.pollNodes();

  provider_.fillSnapshot(snap_);
  snap_.t_ns = Core::NowNs();
  snap_.gsPresent = net_.isGroundStationPresent();
  snap_.incompatible = net_.anyIncompatiblePeer();
  snap_.uartPeerSeen = net_.isUartPeerSeen();
  net_.sendTelemetry(tick_, snap_);
  if (VehicleNetwork::taskTimingDue(tick_)) sendTaskTiming();

  Packets::LogText lt;
  for (int i = 0; i < kMaxLogLinesPerTick && log_.pop(lt); i++)
    net_.sendUsb(VehicleNetwork::kGroundStationNode, lt);

  // Push this tick's frames out (bounded by availableForWrite, never blocks).
  net_.pollLinks();
  tick_++;
}

void VehicleComms::sendTaskTiming() {
  Packets::TaskTiming p;
  p.t_us = timeUs(Core::NowNs());
  const auto &tasks = Core::getSystemScheduler().getTasks();
  size_t n = tasks.size();
  if (n > 8) n = 8;
  p.count = static_cast<uint8_t>(n);
  for (size_t i = 0; i < 8; i++) {
    // Scheduler registration order; average (EMA) run time; the scheduler keeps no maximum.
    p.avg[i] = i < n ? static_cast<float>(static_cast<double>(tasks[i]->getRuntime()) * 1e-9) : NAN;
    p.max[i] = NAN;
  }
  net_.sendUsb(VehicleNetwork::kGroundStationNode, p);
}

} // namespace Comms
} // namespace VCTR
