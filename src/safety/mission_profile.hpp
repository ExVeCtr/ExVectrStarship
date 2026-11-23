#ifndef SAFETY_MISSION_PROFILE_HPP
#define SAFETY_MISSION_PROFILE_HPP

/**
 * Mission profiles (SB, read at the go edge, owner request 2026-11-23): which waypoint list and which chain a go runs.
 *
 *  2 = the flight (waypoint list, bellyflop, freefall, RTH): stays hard-coded in main.cpp (taskInit), unchanged.
 *  0 = hover test: climb to 1 m above home (the position reference), hold 30 s, then RTH (descent and landing).
 *  1 = waypoint test: 1 m / hold 10 s; (2, 0, 1) / hold 15 s; (2, 2, 1) / hold 15 s; then RTH (land at home).
 *      NWU: +X north, +Y west, Z up.
 *
 * The test profiles end in the default mission (RTH, landing detector, motors off), never in the bellyflop: no kick,
 * no freefall. Every waypoint carries the cancel flag, so a waypoint not reached in its time limit ends the test
 * into RTH (it never skips on to the next waypoint). Pure data, host-tested (test/test_mission_profile).
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

/// The profiles that exist. SB values above this are refused at go (GoRefusal::SbProfile).
constexpr uint8_t kProfileHoverTest = 0;
constexpr uint8_t kProfileWaypointTest = 1;
constexpr uint8_t kProfileFlight = 2;

constexpr uint8_t kMaxTestWaypoints = 5;

struct ProfileWaypoint {
  float x, y, z;          ///< m, NWU, above/at the position reference
  float velocity;         ///< m/s
  float startAccel;       ///< m/s^2
  float stopAccel;        ///< m/s^2
  float thresholdM;       ///< reached when closer than this
  float loiterS;          ///< hold time after reaching, s
  float timeLimitS;       ///< not reached within this: the test ends into RTH (cancel flag)
};

struct MissionProfileDesc {
  bool valid = false;        ///< false: unknown profile
  bool flight = false;       ///< profile 2: the hard-coded flight chain (count is 0)
  uint8_t count = 0;         ///< waypoints of a test profile
  ProfileWaypoint wp[kMaxTestWaypoints] = {};
};

/// True for 0, 1 and 2.
bool isKnownProfile(uint8_t profile);

/// Description of a profile: the waypoint list of a test profile, `flight` for 2, `!valid` for anything else.
MissionProfileDesc missionProfileFor(uint8_t profile);

/// Short name for logs and the ground station ("hover test", "waypoint test", "flight").
const char *profileName(uint8_t profile);

} // namespace Safety
} // namespace VCTR

#endif
