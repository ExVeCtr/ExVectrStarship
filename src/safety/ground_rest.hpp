#ifndef SAFETY_GROUND_REST_HPP
#define SAFETY_GROUND_REST_HPP

/**
 * "Physically at rest on the ground" (pure, host-tested: test/test_ground_rest). Input of the D12 on-the-ground
 * definition (Safety::onGround) so a vehicle can never be stuck in a mission phase that cannot finish: true when
 * the altitude is below kGroundRestAltitudeM or a VALID range is below kGroundRestRangeM, and the speed is below
 * kGroundRestSpeed, continuously for kGroundRestHoldNs. Any lost condition (bounce, lift, spike, non-finite input)
 * restarts the timer. Absent range does not veto (the altitude estimate decides then).
 */

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

constexpr float kGroundRestAltitudeM = 0.3f;
constexpr float kGroundRestRangeM = 0.15f;
constexpr float kGroundRestSpeed = 0.3f;
constexpr int64_t kGroundRestHoldNs = 2000000000LL;

class GroundRest {
public:
  /// Call regularly; returns true once the rest condition has held for kGroundRestHoldNs.
  bool update(float altitudeM, bool rangeValid, float rangeM, float speed, int64_t nowNs) {
    const bool rangeOk = rangeValid && isfinite(rangeM);
    const bool low = (isfinite(altitudeM) && altitudeM < kGroundRestAltitudeM) || (rangeOk && rangeM < kGroundRestRangeM);
    const bool cue = low && isfinite(speed) && speed < kGroundRestSpeed;
    if (!cue) {
      since_ = -1;
      return false;
    }
    if (since_ < 0) since_ = nowNs;
    return nowNs - since_ >= kGroundRestHoldNs;
  }
  void reset() { since_ = -1; }

private:
  int64_t since_ = -1;
};

} // namespace Safety
} // namespace VCTR

#endif
