#ifndef SAFETY_LANDING_DETECTOR_HPP
#define SAFETY_LANDING_DETECTOR_HPP

/**
 * Landing detector of the RTH descent (pure, no heap, no Core dependency, host-tested: test/test_landing_detector).
 * Replaces the old "setpoint more than 0.2 m below the vehicle for 2 s, at any altitude" trigger.
 *
 * Rules:
 *  1. NEVER landed above kLandingMaxAltitudeM (position altitude above the reference), and never while a VALID range
 *     reading is above kLandingMaxRangeM (the range is the stronger cue when present). Both gates are checked first, so
 *     a hover or a slow descent in the air is not a landing.
 *  2. Below the gates, landed when one of two contact cues has held without a break for its time:
 *     a. range contact: valid range < kLandingRangeContactM and speed < kLandingContactSpeed for kLandingRangeHoldNs
 *        (the tilted resting body reads a larger range than an upright one, hence 0.10 m);
 *     b. tracking contact (the old criterion, now only low and slow): the vehicle is held more than
 *        kLandingTrackingErrorM above the descending setpoint, with |vz| < kLandingContactVz and speed
 *        < kLandingContactSpeed, for kLandingTrackingHoldNs. Works without any range data.
 *  3. Invalid range (stale, uncertain, absent) neither vetoes nor triggers: cue b and the altitude gate decide alone.
 *  4. Any lost condition restarts that cue's timer (a bounce, a gust lifting the vehicle, a speed spike).
 * Landed is latched by the caller (the mission goes to Finished); reset() clears the timers on a new descent.
 */

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

constexpr float kLandingMaxAltitudeM = 1.5f;    ///< above this height over the reference: never landed
constexpr float kLandingMaxRangeM = 1.0f;       ///< a valid range above this: never landed
constexpr float kLandingRangeContactM = 0.10f;  ///< valid range below this = ground contact (old: 0.04 m, upright)
constexpr float kLandingTrackingErrorM = 0.2f;  ///< vehicle above the setpoint by more than this = held up
constexpr float kLandingContactSpeed = 0.5f;    ///< m/s, total speed limit of both contact cues
constexpr float kLandingContactVz = 0.2f;       ///< m/s, |vz| limit of the tracking cue
constexpr int64_t kLandingRangeHoldNs = 300000000LL;      ///< 0.3 s
constexpr int64_t kLandingTrackingHoldNs = 2000000000LL;  ///< 2 s (as the old trigger)

struct LandingInput {
  float altitudeM = 0;       ///< position Z above the reference
  float verticalSpeed = 0;   ///< m/s
  float speed = 0;           ///< m/s, magnitude of the velocity
  float trackingErrorM = 0;  ///< vehicle altitude minus the Z setpoint (positive = vehicle above the setpoint)
  bool rangeValid = false;   ///< fresh, confident, plausible range reading
  float rangeM = 0;          ///< only used when rangeValid
  int64_t nowNs = 0;
};

class LandingDetector {
public:
  void reset() { rangeSince_ = trackSince_ = -1; }

  /// Call every tick of the descent. Returns true while the landing condition holds.
  bool update(const LandingInput &in) {
    const bool finite = isfinite(in.altitudeM) && isfinite(in.verticalSpeed) && isfinite(in.speed) &&
                        isfinite(in.trackingErrorM);
    const bool rangeOk = in.rangeValid && isfinite(in.rangeM);
    if (!finite || in.altitudeM > kLandingMaxAltitudeM || (rangeOk && in.rangeM > kLandingMaxRangeM)) {
      reset();
      return false;
    }
    const bool slow = in.speed < kLandingContactSpeed;

    const bool rangeCue = rangeOk && in.rangeM < kLandingRangeContactM && slow;
    const bool trackCue = in.trackingErrorM > kLandingTrackingErrorM && slow && fabsf(in.verticalSpeed) < kLandingContactVz;

    return hold(rangeCue, rangeSince_, in.nowNs, kLandingRangeHoldNs) |
           hold(trackCue, trackSince_, in.nowNs, kLandingTrackingHoldNs);
  }

private:
  static bool hold(bool cue, int64_t &since, int64_t now, int64_t holdNs) {
    if (!cue) {
      since = -1;
      return false;
    }
    if (since < 0)
      since = now;
    return now - since >= holdNs;
  }
  int64_t rangeSince_ = -1;
  int64_t trackSince_ = -1;
};

} // namespace Safety
} // namespace VCTR

#endif
