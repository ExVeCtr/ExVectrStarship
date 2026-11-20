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
  if (in.activeIdle) {
    // A reset mission never chains (resetMission() sets missionEnd()); it cancels whatever was pending.
    pending_ = false;
    return ChainStep::None;
  }
  if (!in.activeEnded)
    return ChainStep::None;
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
