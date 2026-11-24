#ifndef SAFETY_SETPOINT_SANITISER_HPP
#define SAFETY_SETPOINT_SANITISER_HPP

// Pure guard between the mission and the controllers: a non-finite mission setpoint is never passed on. Layout of
// the 6-vector is [V(3), P(3)] (velocity setpoint, position setpoint).

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

class SetpointSanitiser {
public:
  static constexpr uint8_t kBadVelocity = 1;
  static constexpr uint8_t kBadPosition = 2;

  /// in[6] = [V,P]; pos[3] = current vehicle position (used only if no finite setpoint was ever seen). Returns a
  /// bit mask of kBad* (0 = input was fine and passed through unchanged).
  uint8_t sanitise(const float in[6], const float pos[3], float out[6]) {
    uint8_t bad = 0;
    for (int i = 0; i < 3; i++) if (!isfinite(in[i])) bad |= kBadVelocity;
    for (int i = 3; i < 6; i++) if (!isfinite(in[i])) bad |= kBadPosition;
    if (!bad) {
      for (int i = 0; i < 6; i++) { out[i] = in[i]; last_[i] = in[i]; }
      haveLast_ = true;
      return 0;
    }
    count_++;
    out[0] = out[1] = out[2] = 0.0f; // velocity setpoint zero for the tick
    for (int i = 3; i < 6; i++) {
      if (haveLast_) out[i] = last_[i];
      else out[i] = isfinite(pos[i - 3]) ? pos[i - 3] : 0.0f;
    }
    return bad;
  }

  uint32_t count() const { return count_; }

  /// True at most once per second (for the warning log).
  bool takeLogSlot(int64_t nowNs) {
    if (lastLogNs_ >= 0 && nowNs - lastLogNs_ < 1000000000LL) return false;
    lastLogNs_ = nowNs;
    return true;
  }

private:
  float last_[6] = {0, 0, 0, 0, 0, 0};
  bool haveLast_ = false;
  uint32_t count_ = 0;
  int64_t lastLogNs_ = -1;
};

} // namespace Safety
} // namespace VCTR

#endif
