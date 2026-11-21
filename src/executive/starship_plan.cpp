#include "executive/starship_plan.hpp"

#include "executive/starship_constants.hpp"
#include "starship_hardware.h"

namespace VCTR {
namespace Starship {

namespace {

using Exec::AbortRule;
using Exec::Behaviour;
using Exec::BehaviourEntry;
using Exec::BoundsProfile;
using Exec::FlapMode;
using Exec::GoToEntry;
using Exec::GoToParams;
using Exec::Guard;
using Exec::GuardParams;
using Exec::PhaseSpec;
using Exec::TvcMode;

// ---------------------------------------------------------------------------------------------------------------
// Numbers of the legacy flight profile (docs/project/missions.md section 7), reproduced for parity (design X1).

constexpr float kHome[3] = {0.0f, 0.0f, 3.0f}; ///< RTH home point (legacy MissionRTH home {0,0,3})

/// Legacy WP2 time limit: default distance(WP1 -> WP2) / v * 1.5 = (120 m - 1 m) / 7 m/s * 1.5 = 25.5 s.
constexpr int64_t kAscentTimeLimitNs = int64_t((120.0 - 1.0) / 7.0 * 1.5 * 1e9);

// Behaviour parameter tables (indices are used by the phase rows below).
enum GoToIdx : uint8_t { kGoToHoverCheck = 0, kGoToAscent };
constexpr GoToParams kGoTo[] = {
    // Hover: legacy WP0 (instant jump to 0.3 m, reached at once) + WP1 (to 1 m at 0.5 m/s, accel 3, brake 1).
    {{0.0f, 0.0f, 1.0f}, 0.5f, 3.0f, 1.0f, GoToEntry::JumpTo, {0.0f, 0.0f, 0.3f}},
    // Ascent: legacy WP2 (to 120 m at 7 m/s, accel 3, brake 2), starting at WP1 (the old snap to the reached point).
    {{0.0f, 0.0f, 120.0f}, 7.0f, 3.0f, 2.0f, GoToEntry::JumpTo, {0.0f, 0.0f, 1.0f}},
};

constexpr Exec::TransitParams kTransit[] = {
    {{kHome[0], kHome[1], kHome[2]}, 2.0f, 0.5f}, // legacy RTH Translation: <= 2 m/s, 0.5 m/s^2
};

constexpr Exec::LandParams kLand[] = {
    {{kHome[0], kHome[1], kHome[2]}, 0.2f, 0.3f}, // legacy RTH Descent: 0.2 m/s, paused while >= 0.3 m above
};

constexpr Exec::KickParams kKick[] = {
    {kKickPitchAngleRad, kKickThrustLimitN},
};

constexpr Exec::DescentParams kDescent[] = {
    // Legacy flap observer: homing above 100 m (pitch +20 .. -20 deg, 1 deg per m beyond 1 m), 30 deg below the
    // freefall stop altitude (stop distance at 14 N + 20 m) + 20 m.
    {VEHICLE_MASS_KG, TVC_THRUST_LIMIT_N, 20.0f, 100.0f, 1.0f, 1.0f * kDeg, -40.0f * kDeg, 20.0f * kDeg, 20.0f,
     30.0f * kDeg},
};

// Guard parameter table.
enum GuardIdx : uint8_t {
  kGuardCountdownEnd = 0,
  kGuardLiftoff,
  kGuardHoverCheckPassed,
  kGuardHoverCheckFailed,
  kGuardApogeeReached,
  kGuardApogeeHoldDone,
  kGuardKickAngle,
  kGuardKickRate,
  kGuardStopAltitude,
  kGuardStopped,
  kGuardHomeReached,
  kGuardLanded,
  kGuardKickMinAltitude,
};
constexpr GuardParams kGuards[] = {
    GuardParams::missionTimeAbove(-10 * kSecondNs),           // legacy Init -> Startup at T-10 s
    GuardParams::missionTimeAbove(0),                         // legacy Startup -> Running at T0
    GuardParams::distance(0.5f, 5 * kSecondNs),               // WP1: within 0.5 m for 5 s
    GuardParams::distance(0.5f, 10 * kSecondNs),              // WP1: never within 0.5 m in 10 s
    GuardParams::distance(3.0f),                              // WP2: within 3 m
    GuardParams::distance(3.0f, 5 * kSecondNs),               // WP2: within 3 m for 5 s
    GuardParams::angleBelow(kKickEndRemainingAngleRad),       // kick: remaining angle < 15 deg
    GuardParams::rateAbove(kKickEndPitchRateRadPs),           // kick: spin guard 250 deg/s
    GuardParams::stopAltitude(VEHICLE_MASS_KG, TVC_THRUST_LIMIT_N, 20.0f), // legacy freefall end (stopAlt 20 m)
    GuardParams::speedBelow(0.5f),                            // legacy RTH Stabilize end
    GuardParams::distance(0.5f),                              // legacy RTH Translation end
    GuardParams::landing(0.2f, 2 * kSecondNs, -0.02f, 0.04f, 7.0f), // legacy RTH landing triggers
    GuardParams::altitudeAtLeast(kMinKickAltitudeM),          // owner X6: no kick below 90 m
};

// Controller groups (TVC reset when the group changes while the TVC set stays on).
constexpr uint8_t kGroupClimb = 1;
constexpr uint8_t kGroupKick = 2;
constexpr uint8_t kGroupRecovery = 3;

// ---------------------------------------------------------------------------------------------------------------
// Phase rows, in flight order.

constexpr PhaseSpec kPhases[] = {
    PhaseSpec::make(id(Phase::Idle), "Idle").withOnGround(true),

    PhaseSpec::make(id(Phase::Countdown), "Countdown")
        .withBehaviour(Behaviour::HoldMeasured)
        .withEstimatorZeroing(true)
        .withBounds(BoundsProfile::Stationary)
        .withOnGround(true)
        .withAbort(AbortRule::ResetAndStop, AbortRule::ResetAndStop)
        .withExit(Guard::MissionTimeAbove, kGuardCountdownEnd, id(Phase::PreLaunch)),

    // Flaps: the legacy Startup enabled the flap actuators with the mission's actuator flag (true), retracted.
    PhaseSpec::make(id(Phase::PreLaunch), "PreLaunch")
        .withActuators(true, true)
        .withFlaps(FlapMode::Retracted)
        .withBehaviour(Behaviour::HoldMeasured)
        .withEntryActions(Exec::kEntryTvcActuatorTest)
        .withBounds(BoundsProfile::Stationary)
        .withOnGround(true)
        .withAbort(AbortRule::ResetAndStop, AbortRule::ResetAndStop)
        .withExit(Guard::MissionTimeAbove, kGuardLiftoff, id(Phase::Hover)),

    // Hover check failed -> RTH (parity difference P1: the legacy chain went on to the bellyflop).
    PhaseSpec::make(id(Phase::Hover), "Hover")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withFlaps(FlapMode::Retracted)
        .withBehaviour(Behaviour::GoTo, kGoToHoverCheck)
        .withCtrlGroup(kGroupClimb)
        .withBounds(BoundsProfile::Flight)
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::InsideFor, kGuardHoverCheckPassed, id(Phase::Ascent))
        .withExit(Guard::NotReachedWithin, kGuardHoverCheckFailed, id(Phase::RthBrake)),

    // Timeout -> Kick, whose entry guard sends the vehicle to RTH below kMinKickAltitudeM (parity difference P2).
    PhaseSpec::make(id(Phase::Ascent), "Ascent")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withFlaps(FlapMode::Retracted)
        .withBehaviour(Behaviour::GoTo, kGoToAscent)
        .withCtrlGroup(kGroupClimb)
        .withBounds(BoundsProfile::Flight)
        .withTimeout(kAscentTimeLimitNs, id(Phase::Kick))
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::Reached, kGuardApogeeReached, id(Phase::ApogeeHold)),

    // Same go-to as Ascent, continued (legacy: the same waypoint). No timeout: the legacy WP2 limit stopped
    // applying once the waypoint was reached (known issue, E7).
    PhaseSpec::make(id(Phase::ApogeeHold), "ApogeeHold")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withFlaps(FlapMode::Retracted)
        .withBehaviour(Behaviour::GoTo, kGoToAscent, BehaviourEntry::Continue)
        .withCtrlGroup(kGroupClimb)
        .withBounds(BoundsProfile::Flight)
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::InsideFor, kGuardApogeeHoldDone, id(Phase::Kick)),

    PhaseSpec::make(id(Phase::Kick), "Kick")
        .withTvc(TvcMode::Attitude, kKickThrustLimitN)
        .withActuators(true, true)
        .withFlaps(FlapMode::Retracted)
        .withBehaviour(Behaviour::KickAttitude)
        .withCtrlGroup(kGroupKick)
        .withEntryGuard(Guard::AltitudeAtLeast, kGuardKickMinAltitude, id(Phase::RthBrake))
        .withTimeout(kKickTimeLimitNs, id(Phase::Descent))
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::KickAngleBelow, kGuardKickAngle, id(Phase::Descent))
        .withExit(Guard::PitchRateAbove, kGuardKickRate, id(Phase::Descent)),

    PhaseSpec::make(id(Phase::Descent), "Descent")
        .withFlaps(FlapMode::Descent)
        .withBehaviour(Behaviour::DescentGuidance)
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::StopAltitude, kGuardStopAltitude, id(Phase::Flip)),

    PhaseSpec::make(id(Phase::Flip), "Flip")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withFlaps(FlapMode::Upright)
        .withBehaviour(Behaviour::HoldMeasured)
        .withCtrlGroup(kGroupRecovery)
        .withAbort(AbortRule::Stay, AbortRule::Stay)
        .withExit(Guard::SpeedBelow, kGuardStopped, id(Phase::RthTransit)),

    PhaseSpec::make(id(Phase::RthBrake), "RthBrake")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withFlaps(FlapMode::Upright)
        .withBehaviour(Behaviour::HoldMeasured)
        .withCtrlGroup(kGroupRecovery)
        .withAbort(AbortRule::Stay, AbortRule::Stay)
        .withExit(Guard::SpeedBelow, kGuardStopped, id(Phase::RthTransit)),

    PhaseSpec::make(id(Phase::RthTransit), "RthTransit")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withFlaps(FlapMode::Retracted)
        .withBehaviour(Behaviour::Transit)
        .withCtrlGroup(kGroupRecovery)
        .withAbort(AbortRule::Stay, AbortRule::Stay)
        .withExit(Guard::Reached, kGuardHomeReached, id(Phase::Landing)),

    PhaseSpec::make(id(Phase::Landing), "Landing")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withFlaps(FlapMode::Retracted)
        .withBehaviour(Behaviour::Land)
        .withCtrlGroup(kGroupRecovery)
        .withAbort(AbortRule::Stay, AbortRule::Stay)
        .withExit(Guard::LandingDetected, kGuardLanded, id(Phase::Landed)),

    PhaseSpec::make(id(Phase::Landed), "Landed")
        .withOnGround(true)
        .withAbort(AbortRule::ResetAndStop, AbortRule::ResetAndStop),

    PhaseSpec::make(id(Phase::Failsafe), "Failsafe"),
};

template <typename T, size_t N> constexpr uint8_t countOf(const T (&)[N]) { return uint8_t(N); }

constexpr Exec::FlightPlan makeProfile2() {
  Exec::FlightPlan p;
  p.phases = kPhases;
  p.phaseCount = countOf(kPhases);
  p.idle = id(Phase::Idle);
  p.first = id(Phase::Countdown);
  p.failsafe = id(Phase::Failsafe);
  p.rth = id(Phase::RthBrake);
  p.noPos = Exec::kNoPhase; // T9b-7
  p.complete = id(Phase::Landed);
  p.behaviours.goTo = kGoTo;
  p.behaviours.goToCount = countOf(kGoTo);
  p.behaviours.transit = kTransit;
  p.behaviours.transitCount = countOf(kTransit);
  p.behaviours.land = kLand;
  p.behaviours.landCount = countOf(kLand);
  p.behaviours.kick = kKick;
  p.behaviours.kickCount = countOf(kKick);
  p.behaviours.descent = kDescent;
  p.behaviours.descentCount = countOf(kDescent);
  p.guards = kGuards;
  p.guardCount = countOf(kGuards);
  return p;
}

constexpr Exec::FlightPlan kProfile2 = makeProfile2();

constexpr Exec::PlanLimits kLimits = {
    TVC_THRUST_LIMIT_N, // maxThrustN
    kKickTimeLimitNs,   // maxAttitudePhaseNs: no attitude-mode phase longer than the kick's hard limit
    15.0f,              // maxSpeedMps
    5.0f,               // maxAccelMps2
};

} // namespace

const Exec::FlightPlan &flightPlanProfile2() { return kProfile2; }

const Exec::PlanLimits &planLimits() { return kLimits; }

} // namespace Starship
} // namespace VCTR
