#ifndef SAFETY_MISSION_PARAMS_HPP
#define SAFETY_MISSION_PARAMS_HPP

/**
 * Runtime mission parameters (owner request 2026-11-23, REQUEST action 24 SET_MISSION_PARAM, telemetry MISSION_PARAMS
 * 0x26). Pure, no heap. The values live in RAM only (not stored across a reboot, defaults at boot).
 *
 *  id 1 ascent_speed     m/s  default 7    range 0.5..10   climb speed of the apogee waypoint (profile 2)
 *  id 2 apogee_altitude  m    default 120  range 2..1000   altitude of the apogee waypoint (profile 2)
 *  id 3 hover_hold_s     s    default 30   range 0..600    hold of profile 0
 *  id 4 p1_hold_1_s      s    default 10   range 0..600    first hold of profile 1
 *  id 5 p1_hold_2_s      s    default 15   range 0..600
 *  id 6 p1_hold_3_s      s    default 15   range 0..600
 *
 * The mission lists read the set once, when a mission begins (main.cpp configureWaypointChain at the go edge). A
 * change is accepted on the ground only (request executor), so a running mission is never changed. An apogee below
 * kApogeeNoFlipBelowM (the bellyflop kick minimum altitude) ends in RTH without a flip: the chain rule is unchanged.
 */

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

constexpr uint8_t kMissionParamCount = 6;
constexpr float kApogeeNoFlipBelowM = 90.0f; ///< == kMinKickAltitudeM (bellyflop_kick): below it the kick is an abort

struct MissionParamSet {
  float ascentSpeed = 7.0f;
  float apogeeAltitude = 120.0f;
  float hoverHoldS = 30.0f;
  float p1Hold1S = 10.0f;
  float p1Hold2S = 15.0f;
  float p1Hold3S = 15.0f;
};

/// True if id (1..6) is known and value is finite and inside the range of the id.
bool missionParamValid(uint8_t id, float value);

/// Validates and stores the value. False (nothing changed) for an unknown id or a bad value. Idempotent.
bool missionParamApply(MissionParamSet &set, uint8_t id, float value);

} // namespace Safety
} // namespace VCTR

#endif
