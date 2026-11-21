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
  const uint8_t allowed = (type == kSimFlapStuckType) ? uint8_t(0x03 | kSimFaultArmBit) : uint8_t(0x03);
  if ((subsystem & ~allowed) != 0) {
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
    d.armForDescent = (subsystem & kSimFaultArmBit) != 0;
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
    if (d.armForDescent) {
      f.armed = true;
      f.armCapture = d.captureCurrent;
      f.armAngle = d.value;
    } else {
      f.stuck = true;
      f.capture = d.captureCurrent;
      f.angle = d.value;
    }
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

int SimFlapFaults::onDescentBegin(bool simActive) {
  int n = 0;
  for (auto &f : f_) {
    if (!f.armed)
      continue;
    if (simActive) {
      f.stuck = true;
      f.capture = f.armCapture;
      f.angle = f.armAngle;
      n++;
    }
    f.armed = false;
    f.armCapture = false;
  }
  return n;
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
    if (f.stuck || f.armed || f.rate != 1.0f)
      return true;
  return false;
}

} // namespace Safety
} // namespace VCTR
