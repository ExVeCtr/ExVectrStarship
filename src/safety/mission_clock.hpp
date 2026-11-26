#ifndef SAFETY_MISSION_CLOCK_HPP
#define SAFETY_MISSION_CLOCK_HPP

/**
 * Whole-mission clock (owner decision 2026-11-26): seconds since the mission's T0 for MISSION.mission_elapsed and
 * AIR_MISSION.mission_elapsed. Telemetry only: nothing in the mission, safety, control, abort or estimator logic
 * reads it, and it only records events that already happen (it never causes one).
 *
 * Pure logic, header only: no Arduino, no ExVectrCore, no globals. Host-tested in test/test_mission_clock.
 * VehicleSafetyAndControlTask (main.cpp) owns one instance and calls:
 * - begin(now, startTime) when the chain begins (MissionBegin / go; startTime is the countdown start, negative ns,
 *   the same value the first mission gets in beginMission()),
 * - end(now) when the chain ends normally (RTH ended): the value is held,
 * - reset() on a reset to Idle (SystemReset, ResetAndStop abort on the ground, a chain cancelled because the active
 *   mission was reset): the value becomes invalid (NaN).
 *
 * elapsed = (now - beginTime) + startTime: negative (T-minus) during the countdown, 0 at T0, continuous across all
 * mission steps (a step restarting its own mission time does not matter). A new begin() restarts it.
 */

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

class MissionClock {
public:
  /// Chain begun at `nowNs` with the countdown start value `startTimeNs` (negative = T-minus).
  void begin(int64_t nowNs, int64_t startTimeNs) {
    ns_ = startTimeNs - nowNs; // elapsed = now + ns_
    running_ = true;
    valid_ = true;
  }

  /// Chain ended: hold the value at this instant. Ignored unless running.
  void end(int64_t nowNs) {
    if (!running_)
      return;
    ns_ += nowNs; // held elapsed
    running_ = false;
  }

  /// Reset to Idle: no value (NaN) until the next begin().
  void reset() { valid_ = running_ = false; }

  bool valid() const { return valid_; }
  bool running() const { return running_; }

  /// Elapsed time in ns (only meaningful when valid()).
  int64_t elapsedNs(int64_t nowNs) const { return running_ ? nowNs + ns_ : ns_; }

  /// Elapsed time in seconds, NaN when no mission has begun since boot or after a reset.
  float elapsedS(int64_t nowNs) const {
    if (!valid_)
      return NAN;
    return static_cast<float>(static_cast<double>(elapsedNs(nowNs)) * 1e-9);
  }

private:
  int64_t ns_ = 0; ///< running: startTime - beginTime (elapsed = now + ns_); held: the held elapsed
  bool running_ = false;
  bool valid_ = false;
};

} // namespace Safety
} // namespace VCTR

#endif
