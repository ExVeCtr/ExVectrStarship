#ifndef EXECUTIVE_STARSHIP_PLAN_HPP
#define EXECUTIVE_STARSHIP_PLAN_HPP

/**
 * The Starship flight plans for the flight executive (ExVectrControl/executive). Data only: phase rows and their
 * parameters. Reference: docs/project/flight-executive.md (implementation) and the workspace design
 * docs/flight-executive-design.md section 3.2 with the owner decisions of section 1.3.
 *
 * Status (E2): host-tested only, not linked into the firmware (platformio.ini keeps src/executive/ out of the
 * teensy40 build until E3). The legacy mission system (docs/project/missions.md) stays authoritative until E6.
 *
 * Phase ids are the wire values of the telemetry phase (MissionPhase). 0 to 7 keep their wire values and names on
 * the wire (wire names in brackets where the new code uses another name); new values are appended (design X4).
 */

#include <stdint.h>

#include "ExVectrControl/executive/plan.hpp"
#include "ExVectrControl/executive/validate_plan.hpp"

namespace VCTR {
namespace Starship {

enum class Phase : Exec::PhaseId {
  Idle = 0,       ///< nothing running; go accepted here
  Hover = 1,      ///< liftoff and 1 m hover check
  Ascent = 2,     ///< climb to apogee
  Descent = 3,    ///< unpowered, flap-controlled belly-down fall
  RthBrake = 4,   ///< [wire: Deceleration] RTH after an abort: kill the velocity, flaps upright
  RthTransit = 5, ///< [wire: RTH] straight line to the home point
  Landing = 6,    ///< descend at home until touchdown
  Landed = 7,     ///< [wire: Completed] terminal until a reset
  Countdown = 8,  ///< estimator zeroing, everything off
  PreLaunch = 9,  ///< TVC actuators and motors (idle floor) on, TVC actuator test
  ApogeeHold = 10, ///< hold at apogee
  Kick = 11,      ///< powered pitch-over (spoken "Transition"), hard time limit
  Flip = 12,      ///< after the descent: upright and stop (like RthBrake; a separate id for the ground station)
  // 13 NoPosArrest and 14 NoPosDescend are reserved for T9b-7 (no-position RTH).
  Failsafe = 15,  ///< latched failure: everything off until a reset
};

constexpr Exec::PhaseId id(Phase p) { return Exec::PhaseId(p); }

/// Profile 2: today's full flight (countdown, hover check, climb to 120 m, kick, descent, flip, RTH, landing).
const Exec::FlightPlan &flightPlanProfile2();

/// Starship limits for validatePlan().
const Exec::PlanLimits &planLimits();

} // namespace Starship
} // namespace VCTR

#endif
