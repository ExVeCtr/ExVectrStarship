#include "motor_interlock.hpp"

namespace VCTR {
namespace Safety {

void MotorInterlock::clearTransition() {
  seenDisabled_ = false;
  transitionSeen_ = false;
  enabledValid_ = false;
}

void MotorInterlock::onRcState(ControlSourceId source, bool sdEnabled, int64_t rxNs) {
  if (active_ == ControlSourceId::None || source != active_) return;
  if (haveLast_) {
    if (rxNs < lastRxNs_) return; // out of order: never counts
    if (rxNs - lastRxNs_ > kReconnectGapNs) {
      clearTransition(); // D3-B: a gap over 2 s is a reconnect
      haveLast_ = false;
    }
  }

  if (!sdEnabled) {
    // Safe direction: closed at once, the transition memory restarts from "seen DISABLED".
    seenDisabled_ = true;
    transitionSeen_ = false;
    enabledValid_ = false;
  } else {
    if (!transitionSeen_ && seenDisabled_ && haveLast_ && !lastSdEnabled_ &&
        rxNs - lastRxNs_ <= kEnableTimeoutNs)
      transitionSeen_ = true;
    if (transitionSeen_) {
      enabledValid_ = true;
      lastEnabledNs_ = rxNs;
    }
  }
  haveLast_ = true;
  lastSdEnabled_ = sdEnabled;
  lastRxNs_ = rxNs;
}

void MotorInterlock::update(int64_t nowNs, ControlSourceId active) {
  if (active != active_) {
    active_ = active;
    clearTransition();
    haveLast_ = false;
    return;
  }
  if (active_ != ControlSourceId::None && haveLast_ && nowNs - lastRxNs_ > kReconnectGapNs) {
    clearTransition();
    haveLast_ = false;
  }
}

void MotorInterlock::requireRearm(RearmReason why) {
  clearTransition();
  rearmCount_++;
  lastRearm_ = why;
}

bool MotorInterlock::open(int64_t nowNs) const {
  if (active_ == ControlSourceId::None || !transitionSeen_ || !enabledValid_) return false;
  const int64_t age = nowNs - lastEnabledNs_;
  return age >= 0 && age <= kEnableTimeoutNs; // a negative age (clock anomaly) fails closed
}

bool MotorInterlock::needToggle(int64_t nowNs) const {
  if (active_ == ControlSourceId::None || !haveLast_ || !lastSdEnabled_ || transitionSeen_) return false;
  const int64_t age = nowNs - lastRxNs_;
  return age >= 0 && age <= kEnableTimeoutNs;
}

MotorInterlock::CloseReason MotorInterlock::closeReason(int64_t nowNs) const {
  if (open(nowNs)) return CloseReason::None;
  if (active_ == ControlSourceId::None) return CloseReason::NoSource;
  if (needToggle(nowNs)) return CloseReason::NeedToggle;
  if (haveLast_ && !lastSdEnabled_) return CloseReason::SdDisabled;
  return CloseReason::Stale;
}

} // namespace Safety
} // namespace VCTR
