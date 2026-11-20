#include "safety/mission_chain.hpp"

namespace VCTR {
namespace Safety {

ChainStep MissionChain::step(const ChainInput &in) {
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
    // was pending. (Idle without ended is not a reset: the bellyflop mission reports Idle while it runs for the
    // first time after boot, because its deferred taskInit overwrites the Running set by beginMission.)
    pending_ = false;
    return ChainStep::None;
  }
  return in.nextIsDefaultOrNone ? ChainStep::SwitchToDefault : ChainStep::SwitchToNext;
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
