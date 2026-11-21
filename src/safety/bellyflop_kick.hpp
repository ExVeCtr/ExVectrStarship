#ifndef SAFETY_BELLYFLOP_KICK_HPP
#define SAFETY_BELLYFLOP_KICK_HPP

/**
 * Powered bellyflop kick (user decision 2026-11-21): at the start of the bellyflop the motors stay on for a short
 * time and the TVC attitude controller pitches the vehicle towards the belly-down attitude; then the motors go to
 * zero and the flap controller (bellyflop set, freefall) takes over. See docs/project/missions.md section 3.
 *
 * Pure logic: no Arduino, no ExVectrCore, no ExVectrMath. Host-tested in test/test_bellyflop_kick. Quaternions are
 * float[4] = [w, x, y, z], rotating body -> world (NWU), as everywhere in the flight software.
 *
 * All tuning lives in the constants below. Starting point: the old firmware's MissionBellyflop parameters
 * (main.cpp at b765e1b..b797a2e, 2025/2026: 1 s time limit, 50 deg remaining-angle trigger, 150 deg/s rate trigger,
 * -90 deg pitch setpoint). The old firmware never limited the kick thrust (the TVC controller with angle
 * compensation asked for far more than the 14 N motor limit, so the kick ran at full thrust); the limit here is new.
 * Bench (simulation, 2026-11-21) with the old 50 deg / 150 deg/s triggers: the kick ended at 36 deg tilt with
 * 133 deg/s and the momentum carried the vehicle to 140 deg tilt (50 deg past horizontal). With 15 deg the TVC
 * attitude controller (P + rate) brakes the rotation itself: kick ended after 0.96 s at 78 deg tilt and 50 deg/s,
 * then the vehicle settled near horizontal (77..95 deg tilt). So the angle trigger was lowered to 15 deg and the
 * rate trigger became a spin guard (250 deg/s). See docs/project/missions.md section 3.
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

constexpr float kKickPi = 3.14159265358979f;
constexpr float kKickDeg = kKickPi / 180.0f;

/// Pitch-over target: rotation about body +Y from the attitude at kick start, so body +X (the flap side's normal,
/// belly = body -X) points up and the belly down. 90 deg = horizontal (old firmware: fixed -90 deg about y).
constexpr float kKickPitchAngleRad = 90.0f * kKickDeg;

/// Hard time limit of the kick: the motors go to zero at the latest this long after the bellyflop began, whether
/// or not the attitude was reached (old firmware: 1 s).
constexpr int64_t kKickTimeLimitNs = 1000000000LL;

/// The kick ends once the remaining angle between body +X and its target direction is below this (old firmware:
/// 50 deg, i.e. at about 40 deg tilt, relying on momentum; 15 deg since bench 2026-11-21, see above).
constexpr float kKickEndRemainingAngleRad = 15.0f * kKickDeg;

/// Spin guard: the kick ends once |body pitch rate| (about body +Y) exceeds this (old firmware: 150 deg/s as an
/// early-end trigger; bench peak with the current gains 135 deg/s).
constexpr float kKickEndPitchRateRadPs = 250.0f * kKickDeg;

/// Thrust limit during the kick, N (2026-11-21, confirmed 11 N by the owner). The kick is NOT meant to fly: it only
/// gives the rotation impulse that gets the ship roughly into the right attitude faster, then the flaps take over. It
/// is not tuned to hold or gain height (11 N is close to the 10.8 N weight at 1.1 kg, so it neither climbs nor sinks
/// much in the short kick); the full motor limit is 14 N (TVC_THRUST_LIMIT_N). The torque comes from the 3 deg vane
/// deflection, so the pitch acceleration scales with this value.
constexpr float kKickThrustLimitN = 11.0f;

/// Defence in depth in the 10 Hz safety task (ControlPhaseTracker): if the bellyflop mission is still active this
/// long after the time limit (one safety tick), the TVC set is switched off anyway.
constexpr int64_t kKickGuardMarginNs = 100000000LL;

enum class KickEnd : uint8_t { None = 0, Angle, Rate, Time };

/**
 * End trigger of the kick, checked every mission tick. Time first (it always ends the kick), then angle, then rate.
 * Non-finite angle or rate never trigger (the time limit still does).
 */
KickEnd kickEndTrigger(int64_t elapsedNs, float remainingAngleRad, float pitchRateRadPs);

/// Rotates v (body) into the world frame with q (body -> world).
void kickRotate(const float q[4], const float v[3], float out[3]);

/**
 * Pitch-over target from the attitude at kick start q0: q0's heading (world-Z rotation that keeps the belly
 * direction) followed by kKickPitchAngleRad about body +Y towards belly down. For q0 = identity this is exactly the
 * old firmware's fixed Quat(y, -90 deg). If the belly direction is nearly vertical (the vehicle is already lying
 * flat) the heading 0 is used. Output unit, w >= 0.
 */
void kickTargetAttitude(const float q0[4], float pitchRad, float out[4]);

/// Remaining angle of the pitch-over: angle between body +X of q and body +X of the target qTarget (rad, [0, pi]).
/// NaN if either quaternion is not finite.
float kickRemainingAngle(const float q[4], const float qTarget[4]);

} // namespace Safety
} // namespace VCTR

#endif
