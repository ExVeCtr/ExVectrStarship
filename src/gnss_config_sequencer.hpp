#ifndef GNSS_CONFIG_SEQUENCER_HPP
#define GNSS_CONFIG_SEQUENCER_HPP

/**
 * Non-blocking start-up and configuration sequence of the u-blox receiver (UbloxSerialGNSS, src/ublox_gnss.*).
 *
 * Pure logic with injected time: no Arduino, no ExVectrCore, no SparkFun library. Host-tested with
 * `pio test -e native` (test/test_gnss_config_sequencer). The driver's 1 ms periodic task calls next(now) and, if
 * it returns an action, performs exactly that one library call and reports the result with report(ok, now).
 * Waits are states with timestamps, so no call ever sleeps. Each library call is bounded by its own
 * ACK wait (kStepMaxWaitMs, the value the SparkFun library documents as safe), instead of the old taskInit that
 * ran the whole sequence plus a delay(1000) in one go (about 1.3 s scheduler stall at t = 5 s, which latched
 * sensorFailure at every boot, 2026-11-20).
 *
 * Sequence (unchanged from the old taskInit):
 *  1. Begin at 115200, else 9600, else 38400 baud (one attempt each). None answers: Failed (the task stays paused).
 *  2. SetSerialRate(115200) (result ignored: the receiver changes baud), then Reopen at 115200 (begin).
 *  3. UART1 output UBX only, navigation rate 10 Hz, auto PVT on, dynamic model AIRBORNE <2g.
 *  4. GLONASS off, Galileo on, BeiDou off, SBAS off, QZSS off, IMES off.
 *  5. Wait kGnssSwitchWaitNs (was the delay(1000)), then GPS off.
 *  6. Save the configuration to the receiver's non-volatile memory. Done.
 *
 * Failure handling: the old code ignored the result of every step after Reopen. Now a failed step is retried after
 * kRetryWaitNs, up to kMaxAttempts attempts, and the sequence then continues with the next step (as before).
 * A failed Reopen (the old code returned and left the task running unconfigured) is retried the same way and then
 * ends in Failed.
 */

#include <stdint.h>

namespace VCTR {
namespace SNSR {

class GnssConfigSequencer {
public:
  enum class Action : uint8_t {
    None = 0,      ///< nothing to do now (waiting, done or failed)
    Begin,         ///< setupSerial(baud()) and begin(); ok = module answered
    SetSerialRate, ///< setSerialRate(115200); result ignored
    Reopen,        ///< setupSerial(115200) and begin(); ok = module answered
    SetUart1Ubx,
    SetNavRate10Hz,
    SetAutoPvt,
    SetDynModelAirborne2g,
    DisableGlonass,
    EnableGalileo,
    DisableBeidou,
    DisableSbas,
    DisableQzss,
    DisableImes,
    DisableGps,
    SaveConfig,
  };

  static constexpr int64_t kMsNs = 1000000LL;
  static constexpr int64_t kGnssSwitchWaitNs = 1000 * kMsNs; ///< between the other constellations and GPS off
  static constexpr int64_t kRetryWaitNs = 100 * kMsNs;       ///< before retrying a failed step
  static constexpr uint8_t kMaxAttempts = 3;                 ///< per configuration step
  static constexpr uint16_t kStepMaxWaitMs = 250;            ///< ACK wait passed to each library call
  static constexpr uint8_t kBaudCount = 3;
  static constexpr uint32_t kBauds[kBaudCount] = {115200, 9600, 38400};

  /// The action to perform now, or None. Calling it again before report() returns the same action.
  Action next(int64_t nowNs);

  /// Result of the action last returned by next().
  void report(bool ok, int64_t nowNs);

  /// Baud rate for Action::Begin.
  uint32_t baud() const { return kBauds[baudIndex_ < kBaudCount ? baudIndex_ : kBaudCount - 1]; }

  bool done() const { return state_ == State::Done; }
  bool failed() const { return state_ == State::Failed; }

  /// Configuration steps that still failed after kMaxAttempts and were skipped.
  uint8_t skippedSteps() const { return skipped_; }

  /// Short text for logs.
  static const char *toString(Action a);

private:
  enum class State : uint8_t { Running, Done, Failed };

  void advance(int64_t nowNs);

  State state_ = State::Running;
  Action step_ = Action::Begin;
  uint8_t baudIndex_ = 0;
  uint8_t attempts_ = 0;
  uint8_t skipped_ = 0;
  int64_t notBeforeNs_ = 0; ///< no action before this time (wait states)
};

} // namespace SNSR
} // namespace VCTR

#endif
