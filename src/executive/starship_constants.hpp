#ifndef EXECUTIVE_STARSHIP_CONSTANTS_HPP
#define EXECUTIVE_STARSHIP_CONSTANTS_HPP

/**
 * Starship flight constants shared by the flight plan (src/executive/starship_plan.*) and, until E6, the legacy
 * mission code (src/safety/bellyflop_kick.hpp re-exports the kick constants). Plain constants only: no includes
 * beyond <stdint.h>, so the host tests and the firmware both use them.
 *
 * Kick (powered pitch-over at the start of the descent, user decision 2026-11-21, confirmed as the starting point by
 * the owner on 2026-11-21, design X5). Starting point: the old firmware's MissionBellyflop parameters (main.cpp at
 * b765e1b..b797a2e: 1 s time limit, 50 deg remaining-angle trigger, 150 deg/s rate trigger, -90 deg pitch setpoint).
 * The old firmware never limited the kick thrust; the limit here is new. Bench (simulation, 2026-11-21) with the old
 * 50 deg / 150 deg/s triggers: the kick ended at 36 deg tilt with 133 deg/s and the momentum carried the vehicle to
 * 140 deg tilt. With 15 deg the TVC attitude controller (P + rate) brakes the rotation itself: kick ended after
 * 0.96 s at 78 deg tilt and 50 deg/s, then the vehicle settled near horizontal. So the angle trigger was lowered to
 * 15 deg and the rate trigger became a spin guard (250 deg/s). See docs/project/missions.md section 3.
 */

#include <stdint.h>

namespace VCTR {
namespace Starship {

constexpr float kPi = 3.14159265358979f;
constexpr float kDeg = kPi / 180.0f;
constexpr int64_t kSecondNs = 1000000000LL;
constexpr int64_t kMillisecondNs = 1000000LL;

/// Pitch-over target: rotation about body +Y from the attitude at kick start, so body +X points up and the belly
/// (body -X) down. 90 deg = horizontal (old firmware: fixed -90 deg about y).
constexpr float kKickPitchAngleRad = 90.0f * kDeg;

/// Hard time limit of the kick: the motors go to zero at the latest this long after the kick began, whether or not
/// the attitude was reached (old firmware: 1 s).
constexpr int64_t kKickTimeLimitNs = 1 * kSecondNs;

/// The kick ends once the remaining angle between body +X and its target direction is below this (old firmware:
/// 50 deg; 15 deg since bench 2026-11-21).
constexpr float kKickEndRemainingAngleRad = 15.0f * kDeg;

/// Spin guard: the kick ends once |body pitch rate| exceeds this (owner 2026-11-21: 250 deg/s; old firmware:
/// 150 deg/s as an early-end trigger; bench peak with the current gains 135 deg/s).
constexpr float kKickEndPitchRateRadPs = 250.0f * kDeg;

/// Thrust during the kick, N: requested and capped at this value (about the vehicle weight 1.1 kg * 9.81 = 10.8 N,
/// so the kick does not climb while vertical; motor limit 14 N).
constexpr float kKickThrustLimitN = 11.0f;

/// Minimum altitude for the kick after an ascent timeout (owner decision X6, 2026-11-21): below it the plan goes
/// to RTH instead of pitching over low over the pad. The plan applies it as the kick's entry condition.
constexpr float kMinKickAltitudeM = 90.0f;

} // namespace Starship
} // namespace VCTR

#endif
