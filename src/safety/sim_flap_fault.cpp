#include "safety/sim_flap_fault.hpp"

#include <math.h>

namespace VCTR {
namespace Safety {

const char *simFaultRefusalText(SimFaultRefusal r) {
  switch (r) {
  case SimFaultRefusal::None: return "accepted";
  case SimFaultRefusal::NotSimulation: return "not in simulation mode";
  case SimFaultRefusal::BadMagic: return "bad magic";
  case SimFaultRefusal::NotFinite: return "value not finite";
  case SimFaultRefusal::BadSubsystem: return "bad subsystem bits";
  default: return "unknown fault type";
  }
}

SimFaultDecision decideSimFault(uint8_t type, uint8_t subsystem, int16_t paramInt, float paramFloat,
                                bool simulationMode) {
  SimFaultDecision d;
  if (type != kSimFlapStuckType && type != kSimFlapSlowType && type != kSimFaultClearType) {
    d.reason = SimFaultRefusal::BadType;
    return d;
  }
  if (!simulationMode) {
    d.reason = SimFaultRefusal::NotSimulation;
    return d;
  }
  if (paramInt != kSimFaultMagic) {
    d.reason = SimFaultRefusal::BadMagic;
    return d;
  }
  if (type == kSimFaultClearType) {
    d.action = SimFaultAction::Clear;
    d.reason = SimFaultRefusal::None;
    return d;
  }
  // Only the flap index bits. Bit 0x10 (the removed "arm for the descent" mode) is refused like any other bit.
  if ((subsystem & ~kSimFaultFlapMask) != 0) {
    d.reason = SimFaultRefusal::BadSubsystem;
    return d;
  }
  if (!isfinite(paramFloat)) {
    d.reason = SimFaultRefusal::NotFinite;
    return d;
  }
  d.flap = subsystem & 0x03;
  d.reason = SimFaultRefusal::None;
  if (type == kSimFlapStuckType) {
    d.action = SimFaultAction::Stuck;
    if (paramFloat == kSimFlapCaptureValue) {
      d.captureCurrent = true;
    } else {
      d.value = paramFloat < 0.0f ? 0.0f : (paramFloat > kSimFlapMaxRad ? kSimFlapMaxRad : paramFloat);
    }
  } else {
    d.action = SimFaultAction::Slow;
    d.value = paramFloat < kSimFlapSlowMinFactor ? kSimFlapSlowMinFactor : (paramFloat > 1.0f ? 1.0f : paramFloat);
  }
  return d;
}

bool SimFlapFaults::set(const SimFaultDecision &d, bool simActive) {
  if (!simActive || d.reason != SimFaultRefusal::None || d.flap > 3)
    return false;
  Flap &f = f_[d.flap];
  switch (d.action) {
  case SimFaultAction::Stuck:
    f.stuck = true;
    f.capture = d.captureCurrent;
    f.angle = d.value;
    return true;
  case SimFaultAction::Slow:
    f.rate = d.value;
    return true;
  case SimFaultAction::Clear:
    clear();
    return true;
  default:
    return false;
  }
}

void SimFlapFaults::clear() {
  for (uint8_t i = 0; i < 4; i++) {
    if (f_[i].stuck)
      released_[i] = true;
    f_[i] = Flap();
  }
}

bool SimFlapFaults::takeReleased(uint8_t flap) {
  if (flap > 3 || !released_[flap])
    return false;
  released_[flap] = false;
  return true;
}

float SimFlapFaults::effectiveAngle(uint8_t flap, bool simActive, float modelAngle) {
  if (!simActive || flap > 3 || !f_[flap].stuck)
    return modelAngle;
  Flap &f = f_[flap];
  if (f.capture) {
    f.angle = modelAngle;
    f.capture = false;
  }
  return f.angle;
}

float SimFlapFaults::rateFactor(uint8_t flap, bool simActive) const {
  if (!simActive || flap > 3)
    return 1.0f;
  return f_[flap].rate;
}

bool SimFlapFaults::any() const {
  for (const auto &f : f_)
    if (f.stuck || f.rate != 1.0f)
      return true;
  return false;
}

void TrueFlapModel::advance(int64_t dtNs, const SimFlapFaults &faults, bool simActive) {
  const double dt = double(dtNs) * 1e-9;
  for (uint8_t i = 0; i < 4; i++) {
    const float diff = target_[i] - angle_[i];
    const float maxStep = float(double(rate_ * faults.rateFactor(i, simActive)) * dt);
    if (diff <= maxStep && diff >= -maxStep)
      angle_[i] = target_[i];
    else
      angle_[i] += diff > 0 ? maxStep : -maxStep;
  }
}

void TrueFlapModel::publishAngles(SimFlapFaults &faults, bool simActive, float out[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    if (faults.takeReleased(i))
      angle_[i] = target_[i]; // a released stuck fault must not leave its forced angle in the model
    angle_[i] = faults.effectiveAngle(i, simActive, angle_[i]);
    out[i] = angle_[i];
  }
}

float ServoOutputStage::servoCommandRad(SimFlapFaults &faults, uint8_t flap, float commandedRad, bool simulationNow,
                                        int64_t dtNs, float normalRateRadps, float minRad, float maxRad) {
  if (!simulationNow || flap > 3) {
    if (flap <= 3) {
      last_[flap] = commandedRad;
      valid_[flap] = true;
      engaged_[flap] = false;
    }
    return commandedRad; // bit-identical, no fault state is even read
  }
  float out = commandedRad;
  if (faults.stuck(flap)) {
    float a = faults.stuckAngle(flap);
    out = a < minRad ? minRad : (a > maxRad ? maxRad : a);
    engaged_[flap] = true;
  } else {
    const float factor = faults.rateFactor(flap, true);
    if ((factor != 1.0f || engaged_[flap]) && valid_[flap]) {
      const float rate = normalRateRadps * factor; // slowed, or the normal rate when slewing back after a release
      const float maxStep = float(double(rate) * double(dtNs) * 1e-9);
      const float diff = commandedRad - last_[flap];
      if (diff <= maxStep && diff >= -maxStep) {
        out = commandedRad;
        engaged_[flap] = (factor != 1.0f) ? engaged_[flap] : false;
      } else {
        out = last_[flap] + (diff > 0 ? maxStep : -maxStep);
        engaged_[flap] = true;
      }
    } else {
      engaged_[flap] = false;
    }
  }
  last_[flap] = out;
  valid_[flap] = true;
  return out;
}

} // namespace Safety
} // namespace VCTR
