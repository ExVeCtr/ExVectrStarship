#ifndef SAFETY_RTH_TRANSLATION_HPP
#define SAFETY_RTH_TRANSLATION_HPP

// Pure RTH translation step and handover rule (no hardware, host-testable). Fix for the 2026-11 recording where the
// setpoint arrived exactly at the return point: 0/0 gave a NaN direction, a NaN velocity setpoint and zero thrust.

#include <math.h>
#include <stdint.h>

#include "../flashmem.hpp"

namespace VCTR {
namespace Safety {

constexpr float kRthAccel = 0.5f;           ///< m/s^2
constexpr float kRthMaxSpeed = 2.0f;        ///< m/s
constexpr float kRthArriveEps = 1e-5f;      ///< m: below this the direction is undefined (treated as arrived)
constexpr float kRthSpArrivedM = 0.01f;     ///< setpoint counts as AT the return point
constexpr int64_t kRthSpArrivedNs = 3000000000LL; ///< ... for this long
constexpr float kRthVehicleNearM = 0.5f;    ///< normal handover radius (vehicle)
constexpr float kRthVehicleWindM = 1.5f;    ///< handover radius after the setpoint dwelled at the return point

struct RthTranslationState {
  float setpoint[3];  ///< position setpoint (world, m)
  float velocity;     ///< current translation speed (m/s)
};

struct RthTranslationOut {
  float setpoint[3];
  float velSetpoint[3];
  float velocity;
};

inline bool rthAllFinite3(const float *v) { return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }

/// One translation step. Every output is finite for every input (non-finite input: setpoint -> return point when that
/// is finite, else the previous setpoint when finite, else 0; velocity setpoint 0).
FLASHMEM __attribute__((noinline)) static RthTranslationOut rthTranslationStep(const RthTranslationState &s, const float ret[3], float dTime) {
  RthTranslationOut o;
  for (int i = 0; i < 3; i++) {
    o.setpoint[i] = isfinite(s.setpoint[i]) ? s.setpoint[i] : 0.0f;
    o.velSetpoint[i] = 0.0f;
  }
  o.velocity = 0.0f;
  if (!rthAllFinite3(ret)) return o;

  float dt = (isfinite(dTime) && dTime > 0.0f) ? dTime : 0.0f;
  float vel = (isfinite(s.velocity) && s.velocity > 0.0f) ? s.velocity : 0.0f;

  float travel[3];
  for (int i = 0; i < 3; i++) travel[i] = ret[i] - o.setpoint[i];
  const float mag = sqrtf(travel[0] * travel[0] + travel[1] * travel[1] + travel[2] * travel[2]);

  if (!(mag > kRthArriveEps) || !isfinite(mag)) {
    if (!isfinite(mag)) return o; // absurd distance: hold, zero velocity
    for (int i = 0; i < 3; i++) o.setpoint[i] = ret[i]; // arrived: exactly the return point
    return o;
  }

  float norm[3];
  for (int i = 0; i < 3; i++) norm[i] = travel[i] / mag;

  if (vel < kRthMaxSpeed) vel += dt * kRthAccel;
  if (vel > kRthMaxSpeed) vel = kRthMaxSpeed;

  if (mag / dt > vel) {
    for (int i = 0; i < 3; i++) travel[i] = norm[i] * vel * dt;
  }
  const float accelLimit = sqrtf(mag * kRthAccel);
  if (vel > accelLimit) vel = accelLimit;

  float sp[3], vs[3];
  for (int i = 0; i < 3; i++) {
    vs[i] = norm[i] * vel;
    sp[i] = o.setpoint[i] + travel[i];
  }
  if (!rthAllFinite3(sp) || !rthAllFinite3(vs) || !isfinite(vel)) return o;
  for (int i = 0; i < 3; i++) {
    o.setpoint[i] = sp[i];
    o.velSetpoint[i] = vs[i];
  }
  o.velocity = vel;
  return o;
}

enum class RthHandover : uint8_t { None = 0, VehicleNear, SetpointDwell };

/// Decides when the translation stage hands over to the descent.
class RthHandoverRule {
public:
  void reset() { arrivedSinceNs_ = -1; }

  /// spDist: |return point - setpoint|; vehDist: |return point - vehicle|. Non-finite inputs never trigger.
  RthHandover update(int64_t nowNs, float spDist, float vehDist) {
    if (!isfinite(vehDist)) { arrivedSinceNs_ = -1; return RthHandover::None; }
    if (isfinite(spDist) && spDist < kRthSpArrivedM) {
      if (arrivedSinceNs_ < 0) arrivedSinceNs_ = nowNs;
    } else {
      arrivedSinceNs_ = -1;
    }
    if (vehDist < kRthVehicleNearM) return RthHandover::VehicleNear;
    if (arrivedSinceNs_ >= 0 && nowNs - arrivedSinceNs_ > kRthSpArrivedNs && vehDist < kRthVehicleWindM)
      return RthHandover::SetpointDwell;
    return RthHandover::None;
  }

private:
  int64_t arrivedSinceNs_ = -1;
};

} // namespace Safety
} // namespace VCTR

#endif
