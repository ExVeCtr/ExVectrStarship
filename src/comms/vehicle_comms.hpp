#ifndef COMMS_VEHICLE_COMMS_HPP
#define COMMS_VEHICLE_COMMS_HPP

/**
 * Teensy wrapper of the vehicle network stack (VehicleNetwork, vehicle_network.hpp) as one low-priority
 * periodic task (workspace docs/network-design.md 3.2.2, 3.3, 3.4, 3.5.6).
 *
 * - USB Serial and the UART to the ExVectrLink RX module (RX_MODULE_SERIALPORT, 460800 8N1) as byte streams.
 * - Every tick (100 Hz): read both links, heartbeats, sample the vehicle (TelemetryProviderI), queue the
 *   telemetry due on this tick, push the queued bytes out.
 * - Its cost does not depend on whether USB or the RX module is connected: every tick builds and encodes the
 *   same packets; a full TX ring (nobody draining it) drops the frame. Writes are bounded by availableForWrite()
 *   and never block. The task runs below the control tasks' priority.
 * - Console text (Core print topics, LOG_MSG) is queued as lines and sent as LOG_TEXT on USB only. USB Serial
 *   carries nothing but stream frames (the platform console is off: EXVECTR_ARDUINO_NO_SERIAL_CONSOLE).
 * - TEMPORARY (T9b): accepted legacy telecommands are published on the existing telecommand topic.
 * No heap after construction. main.cpp places the object in DMAMEM (RAM2).
 */

#include <Arduino.h>

#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "vehicle_network.hpp"

namespace VCTR {
namespace Comms {

/// Fills the snapshot (implemented in main.cpp, where the vehicle objects live).
class TelemetryProviderI {
public:
  virtual ~TelemetryProviderI() = default;
  /// Called once from the telemetry task's taskInit (subscribe to topics here).
  virtual void begin() {}
  virtual void fillSnapshot(TelemetrySnapshot &s) = 0;
};

/// Core::NowNs() for the network stack.
class CoreTimeSource : public network::TimeSourceI {
public:
  int64_t nowNs() override;
};

/// Non-blocking ByteStreamI over an Arduino stream (usb_serial_class, HardwareSerialIMXRT).
template <class S>
class ArduinoByteStream : public network::ByteStreamI {
public:
  explicit ArduinoByteStream(S &s) : s_(s) {}
  size_t readable() override {
    int a = s_.available();
    return a > 0 ? static_cast<size_t>(a) : 0;
  }
  size_t read(uint8_t *d, size_t n) override {
    size_t i = 0;
    while (i < n) {
      int c = s_.read();
      if (c < 0) break;
      d[i++] = static_cast<uint8_t>(c);
    }
    return i;
  }
  size_t writable() override {
    int a = s_.availableForWrite();
    return a > 0 ? static_cast<size_t>(a) : 0;
  }
  size_t write(const uint8_t *d, size_t n) override { return s_.write(d, n); }

private:
  S &s_;
};

/// Collects Core print output into lines for LOG_TEXT. The print callbacks only copy text; the telemetry task
/// sends the queued lines. Lines longer than kLineLen are split; a full queue drops lines (counted).
class LogTextQueue {
public:
  static constexpr size_t kLines = 8;
  static constexpr size_t kLineLen = 200;

  void append(Packets::Severity severity, const char *text);
  /// Pops the oldest complete line. False if none.
  bool pop(Packets::LogText &out);
  uint32_t dropped() const { return dropped_; }

private:
  void finishLine();

  char partial_[kLineLen];
  size_t partialLen_ = 0;
  Packets::Severity partialSeverity_ = Packets::Severity::kDebug;
  struct Line {
    Packets::Severity severity;
    uint16_t len;
    char text[kLineLen];
  };
  Line lines_[kLines];
  size_t head_ = 0, count_ = 0;
  uint32_t dropped_ = 0;
};

class VehicleComms : public Core::Task_Periodic, public TelecommandSinkI {
public:
  static constexpr uint32_t kUartBaud = 460800;
  static constexpr int64_t kTickNs = Core::SECONDS / VehicleNetwork::kTickHz; ///< 10 ms
  static constexpr int kPriority = 50; ///< below every control task (100..110)
  static constexpr int kMaxLogLinesPerTick = 4;

  VehicleComms(usb_serial_class &usb, HardwareSerialIMXRT &uart, TelemetryProviderI &provider,
               Core::Topic<Telecommand> &telecommandTopic);

  /// Ground station (node 0) heard on USB or UART within 2 s. Information only (gs_present).
  bool isGroundStationPresent() const { return net_.isGroundStationPresent(); }
  /// Radio link alive (VehicleNetwork::isRadioLinkAlive): the radioConnectionLoss observable.
  bool isRadioLinkAlive() const { return net_.isRadioLinkAlive(); }
  /// A radio packet really arrived within 2 s, no boot grace (VehicleNetwork::isRadioPacketRecent): "connected".
  bool isRadioPacketRecent() const { return net_.isRadioPacketRecent(); }

  void taskInit() override;
  void taskThread() override;

  /// TEMPORARY (T9b): publishes the telecommand on the existing topic (handled by the safety task as before).
  void onTelecommand(const Telecommand &tc) override { telecommandTopic_.publish(tc); }

  const VehicleNetwork &network() const { return net_; }
  const LogTextQueue &logQueue() const { return log_; }

private:
  void sendTaskTiming();

  HardwareSerialIMXRT &uart_;
  TelemetryProviderI &provider_;
  Core::Topic<Telecommand> &telecommandTopic_;

  CoreTimeSource time_;
  ArduinoByteStream<usb_serial_class> usbIo_;
  ArduinoByteStream<HardwareSerialIMXRT> uartIo_;
  VehicleNetwork net_;

  LogTextQueue log_;
  Core::StaticCallback_Subscriber<const char *> printSubM_, printSubD_, printSubW_, printSubE_;
  TelemetrySnapshot snap_;
  uint32_t tick_ = 0;

  uint8_t uartRxMem_[1024]; ///< extra UART RX buffer (whole air-packet bursts between two ticks)
  uint8_t uartTxMem_[512];  ///< extra UART TX buffer
};

} // namespace Comms
} // namespace VCTR

#endif
