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
 *  id 7 ascent_accel     m/s2 default 3    range 0.2..20   start acceleration of the apogee waypoint
 *  id 8 ascent_decel     m/s2 default 2    range 0.2..20   stop acceleration (deceleration) of the apogee waypoint
 *  id 9 apogee_x         m    default 0    range -2000..2000  apogee position, m NORTH of the position reference (+X)
 *  id 10 apogee_y        m    default 0    range -2000..2000  apogee position, m WEST of it (+Y = west in NWU)
 *
 * The mission lists read the set once, when a mission begins (main.cpp configureWaypointChain at the go edge). A
 * change is accepted on the ground only (request executor), so a running mission is never changed. An apogee below
 * kApogeeNoFlipBelowM (the bellyflop kick minimum altitude) ends in RTH without a flip: the chain rule is unchanged.
 */

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

constexpr uint8_t kMissionParamCount = 10;
constexpr float kApogeeNoFlipBelowM = 90.0f; ///< == kMinKickAltitudeM (bellyflop_kick): below it the kick is an abort

struct MissionParamSet {
  float ascentSpeed = 7.0f;
  float apogeeAltitude = 120.0f;
  float hoverHoldS = 30.0f;
  float p1Hold1S = 10.0f;
  float p1Hold2S = 15.0f;
  float p1Hold3S = 15.0f;
  float ascentAccel = 1.0f;
  float ascentDecel = 2.0f;
  float apogeeX = 0.0f;
  float apogeeY = 0.0f;
};

/// True if id (1..10) is known and value is finite and inside the range of the id.
bool missionParamValid(uint8_t id, float value);

/// Validates and stores the value. False (nothing changed) for an unknown id or a bad value. Idempotent.
bool missionParamApply(MissionParamSet &set, uint8_t id, float value);

} // namespace Safety
} // namespace VCTR

#endif
