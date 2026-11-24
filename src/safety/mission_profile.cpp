#include "safety/mission_profile.hpp"

#ifndef FLASHMEM // Teensy: cold code runs from flash and keeps ITCM (RAM1 code) small; empty on the host
#ifdef __IMXRT1062__
#define FLASHMEM __attribute__((section(".flashmem")))
#else
#define FLASHMEM
#endif
#endif

namespace VCTR {
namespace Safety {

namespace {
// Same shape as the first two waypoints of the flight list (main.cpp taskInit): a 0.3 m pad point passed at once
// (speed 0, threshold 5 m, no hold), then the 1 m point at 0.5 m/s, threshold 0.5 m. The time limit is explicit
// (the flight list lets addWaypoint compute it; the pad point divides by speed 0).
constexpr ProfileWaypoint kPad = {0, 0, 0.3f, 0, 3, 1, 5, 0, 10};
constexpr float kV = 0.5f;
constexpr float kThr = 0.5f;
constexpr float kLegLimitS = 12.0f; // 2 m at 0.5 m/s takes 4 s
constexpr float kClimbLimitS = 10.0f; // as the flight hover check
} // namespace

bool isKnownProfile(uint8_t profile) { return profile <= kProfileFlight; }

FLASHMEM MissionProfileDesc missionProfileFor(uint8_t profile, const MissionParamSet &p) {
  MissionProfileDesc d;
  switch (profile) {
  case kProfileFlight:
    d.valid = true;
    d.flight = true;
    break;
  case kProfileHoverTest:
    d.valid = true;
    d.count = 2;
    d.wp[0] = kPad;
    d.wp[1] = {0, 0, 1, kV, 3, 1, kThr, p.hoverHoldS, kClimbLimitS};
    break;
  case kProfileWaypointTest:
    d.valid = true;
    d.count = 4;
    d.wp[0] = kPad;
    d.wp[1] = {0, 0, 1, kV, 3, 1, kThr, p.p1Hold1S, kClimbLimitS};
    d.wp[2] = {2, 0, 1, kV, 3, 1, kThr, p.p1Hold2S, kLegLimitS};
    d.wp[3] = {2, 2, 1, kV, 3, 1, kThr, p.p1Hold3S, kLegLimitS};
    break;
  default:
    break;
  }
  return d;
}

FLASHMEM ProfileWaypoint flightApogeeWaypoint(const MissionParamSet &p) {
  return {p.apogeeX, p.apogeeY, p.apogeeAltitude, p.ascentSpeed, p.ascentAccel, p.ascentDecel, 3, 5, 0};
}

FLASHMEM const char *profileName(uint8_t profile) {
  switch (profile) {
  case kProfileHoverTest: return "hover test";
  case kProfileWaypointTest: return "waypoint test";
  case kProfileFlight: return "flight";
  }
  return "unknown";
}

} // namespace Safety
} // namespace VCTR
