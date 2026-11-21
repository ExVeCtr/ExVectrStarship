#include "safety/mission_chain.hpp"

namespace VCTR {
namespace Safety {

ChainStep MissionChain::step(const ChainInput &in) {
  redirect_ = ChainRedirect::None;
  if (in.activeIsDefault && in.activeEnded) {
    pending_ = false;
    return ChainStep::RthEnded;
  }
  if (!pending_)
    return ChainStep::None;
  if (!in.activeEnded)
    return ChainStep::None;
  if (in.activeIdle) {
    // Ended and Idle = reset (resetMission() sets missionEnd()). A reset mission never chains; it cancels whatever
    // was pending. (Idle without ended is not treated as a reset and keeps the chain pending. No mission reports
    // that today: the bellyflop taskInit override that did is gone since 2026-11-20. Kept as defence in depth.)
    pending_ = false;
    return ChainStep::None;
  }
  // Owner decisions 2026-11-21 (header): RTH instead of the next mission.
  if (in.activeCancelled) {
    redirect_ = ChainRedirect::CancelledWaypoint;
    return ChainStep::SwitchToDefault;
  }
  if (in.nextIsBellyflop && !in.nextIsDefaultOrNone && !(in.altitudeM >= kMinKickAltitudeM)) { // NaN: no kick
    redirect_ = ChainRedirect::BelowKickAltitude;
    return ChainStep::SwitchToDefault;
  }
  return in.nextIsDefaultOrNone ? ChainStep::SwitchToDefault : ChainStep::SwitchToNext;
}

const char *toString(ChainRedirect r) {
  switch (r) {
  case ChainRedirect::None:
    return "none";
  case ChainRedirect::CancelledWaypoint:
    return "hover check (cancel-flag waypoint) failed";
  case ChainRedirect::BelowKickAltitude:
    return "below the minimum bellyflop altitude";
  }
  return "?";
}

const char *toString(ChainStep s) {
  switch (s) {
  case ChainStep::None:
    return "none";
  case ChainStep::RthEnded:
    return "RTH ended";
  case ChainStep::SwitchToDefault:
    return "switch to default";
  case ChainStep::SwitchToNext:
    return "switch to next";
  }
  return "?";
}

} // namespace Safety
} // namespace VCTR
