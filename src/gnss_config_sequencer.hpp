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
 * Sequence:
 *  1. Begin at 115200, else 9600, else 38400 baud (one attempt each). None answers: Failed (the task stays paused).
 *  2. SetSerialRate(115200) (result ignored: the receiver changes baud), then Reopen at 115200 (begin).
 *  3. UART1 output UBX only, navigation rate 10 Hz, auto PVT on, dynamic model AIRBORNE <2g.
 *  4. Constellations GPS + Galileo (user decision 2026-11-20): poll UBX-CFG-GNSS, then ONE CFG-GNSS write (separate
 *     step, so each step stays one bounded call) that enables GPS and Galileo (and QZSS, which the u-blox M8 requires
 *     together with GPS: it NACKs GPS on with QZSS off) and disables SBAS, BeiDou, IMES and GLONASS in the same
 *     message (applyGpsGalileo on the polled payload). The
 *     old per-constellation toggles ended with "GPS off", which the receiver refused at every boot (the old
 *     "Galileo only" was never in effect).
 *     The driver logs the configuration found at boot (the poll, before the write), so a configuration that
 *     persisted in the receiver's non-volatile memory is visible.
 *  5. Wait kGnssSwitchWaitNs (the receiver restarts its GNSS subsystem after a CFG-GNSS change). During every wait
 *     state the driver drains the UART (checkUblox): auto-PVT at 10 Hz otherwise overflows the RX buffer and the
 *     first exchange after the wait fails (bench 2026-11-20: save timed out 3x after the 1 s wait).
 *  6. Save the configuration to the receiver's non-volatile memory (after the wait, review 2026-11-20: before, the
 *     save was sent right after the write, while the receiver restarted).
 *  7. Read back: UBX-CFG-GNSS (configured constellations), UBX-MON-GNSS (constellations in use), UBX-CFG-RATE
 *     (navigation rate). The driver logs one line with the result. Done.
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
    PollGnssForWrite,  ///< CFG-GNSS poll into the driver's buffer
    SetGnssGpsGalileo, ///< applyGpsGalileo() on that buffer, CFG-GNSS write (fails if the poll did not succeed)
    SaveConfig,
    ReadGnssConfig, ///< CFG-GNSS poll -> decodeCfgGnss
    ReadGnssInUse,  ///< MON-GNSS poll -> decodeMonGnss
    ReadNavRate,    ///< CFG-RATE poll -> decodeCfgRate
  };

  static constexpr int64_t kMsNs = 1000000LL;
  static constexpr int64_t kGnssSwitchWaitNs = 1000 * kMsNs; ///< after the CFG-GNSS write, before the save
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

/**
 * Visibility of a failed or incomplete GNSS configuration (review 2026-11-20). INFORMATION ONLY: nothing on the
 * vehicle acts on it (Ready and Arm are not affected). VehicleStatus has no free bit for it (the flags byte is full),
 * so the vehicle repeats a warning LOG_TEXT line while the configuration is not as intended.
 *
 * due(finished, ok, now): finished = the sequence has ended (done or failed); ok = it completed without a skipped
 * step AND the read-back shows GPS + Galileo as intended. True on the first tick that is finished and not ok, then
 * every kPeriodNs while that persists.
 */
class GnssConfigWarning {
public:
  static constexpr int64_t kPeriodNs = 10000LL * 1000000LL; ///< 10 s
  bool due(bool finished, bool ok, int64_t nowNs);

private:
  bool warned_ = false;
  int64_t lastNs_ = 0;
};

// ---- UBX payload helpers (pure, host-tested) -------------------------------------------------------------------

/// u-blox gnssId values (UBX-CFG-GNSS).
enum GnssId : uint8_t { kGnssGps = 0, kGnssSbas = 1, kGnssGalileo = 2, kGnssBeidou = 3, kGnssImes = 4,
                        kGnssQzss = 5, kGnssGlonass = 6 };

/// Decoded UBX-CFG-GNSS: enable bit per constellation (false if its block is missing).
struct GnssConstellations {
  bool valid = false;
  bool gps = false, sbas = false, galileo = false, beidou = false, imes = false, qzss = false, glonass = false;
  uint8_t numTrkChHw = 0, numTrkChUse = 0;
};

/// Decodes a UBX-CFG-GNSS payload (4 + 8 * numConfigBlocks bytes). False if malformed.
bool decodeCfgGnss(const uint8_t *payload, uint16_t len, GnssConstellations &out);

/// Modifies a polled UBX-CFG-GNSS payload in place for GPS + Galileo: enable GPS, Galileo and QZSS (QZSS follows GPS,
/// the M8 refuses GPS without it; each gets its primary signal, sigCfgMask 0x01, if the mask is empty), disable every
/// other block (SBAS, BeiDou, IMES, GLONASS). The payload is then sent back
/// as one CFG-GNSS write. False if malformed or the GPS or Galileo block is missing (nothing should be sent then).
bool applyGpsGalileo(uint8_t *payload, uint16_t len);

/// The intended set: GPS and Galileo on, SBAS, BeiDou, IMES and GLONASS off (QZSS not checked, it follows GPS).
bool constellationsAsIntended(const GnssConstellations &c);

/// Decoded UBX-MON-GNSS (bit 0 GPS, 1 GLONASS, 2 BeiDou, 3 Galileo).
struct GnssInUse {
  bool valid = false;
  uint8_t supported = 0, enabled = 0;
};
bool decodeMonGnss(const uint8_t *payload, uint16_t len, GnssInUse &out);

/// Decoded UBX-CFG-RATE.
struct GnssNavRate {
  bool valid = false;
  uint16_t measRateMs = 0;
  uint16_t navRateCycles = 0;
  /// Navigation solutions per second (0 if invalid).
  float hz() const { return (valid && measRateMs && navRateCycles) ? 1000.0f / (measRateMs * navRateCycles) : 0; }
};
bool decodeCfgRate(const uint8_t *payload, uint16_t len, GnssNavRate &out);

} // namespace SNSR
} // namespace VCTR

#endif
