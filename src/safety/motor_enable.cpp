#include "safety/motor_enable.hpp"

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

} // namespace Safety
} // namespace VCTR
