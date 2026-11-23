#include "safety/motor_enable.hpp"

#include <math.h>

namespace VCTR {
namespace Safety {

MotorEnableOutput motorEnableFor(const MotorEnableInput &in) {
  MotorEnableOutput o;
  o.enableMotors = in.motorsEnabledNow;
  if (in.shutdown)
    return o; // TVC actuators, flaps and the simulated TVC off; enableMotors untouched (as before)
  if (in.request == MotorsRequest::Enable)
    o.enableMotors = true;
  else if (in.request == MotorsRequest::Disable)
    o.enableMotors = false;
  o.tvcActuators = in.missionActuatorsEnabled || in.request == MotorsRequest::Enable;
  o.flapActuators = in.freefallActive ? true : in.missionActuatorsEnabled;
  o.simTvc = in.missionActuatorsEnabled;
  return o;
}

static const float kG = 9.81f;

bool freefallStopTriggered(float altitudeM, float vzMps, float stopTargetM, float thrustLimitN, float massKg) {
  const float aSum = thrustLimitN / massKg - kG;
  return altitudeM < 0.5f * vzMps * vzMps / aSum + stopTargetM;
}

bool freefallSpinupDue(const FreefallSpinupInput &in) {
  if (!in.running || !isfinite(in.altitudeM))
    return false;
  const float aSum = in.thrustLimitN / in.massKg - kG;
  if (!isfinite(in.vzMps) || !(aSum > 0.0f))
    return in.altitudeM < in.stopTargetM + kMotorSpinupMarginM;
  if (freefallStopTriggered(in.altitudeM, in.vzMps, in.stopTargetM, in.thrustLimitN, in.massKg))
    return true;
  const float t = kMotorSpinupLeadS;
  const float vzL = in.vzMps - kG * t;
  const float altL = in.altitudeM + in.vzMps * t - 0.5f * kG * t * t;
  return freefallStopTriggered(altL, vzL, in.stopTargetM, in.thrustLimitN, in.massKg);
}

} // namespace Safety
} // namespace VCTR
