#include "rc_mapper.hpp"

namespace VCTR {
namespace Safety {

RcDecoded RcCommandMapper::decode(const CmdFrame &f) {
  RcDecoded d;
  d.go = f.go;
  d.profile = f.profile;
  d.mode = f.vehicleMode;
  d.motors = f.motors;
  d.reset = f.resetButton;
  d.failsafe = f.failsafe;
  return d;
}

uint8_t RcCommandMapper::safeBits(const RcDecoded &d) {
  uint8_t b = 0;
  if (d.go == 0) b |= kSafeSeenGo;
  if (d.mode == 0) b |= kSafeSeenScReal;
  if (d.motors == 0) b |= kSafeSeenSdEnable;
  if (d.reset == 0) b |= kSafeSeenSePress;
  return b;
}

void RcCommandMapper::clearDetectors(Slot &s) {
  s.safeSeen = 0;
  s.edgeBase = false;
  s.se = SeState::Idle;
}

void RcCommandMapper::classifySe(Slot &s, uint8_t se, int64_t t) { // se: 1 = pressed
  const bool inPress = s.se == SeState::Press1 || s.se == SeState::Pressing || s.se == SeState::Holding;
  if (inPress && t - s.lastNs > kSeGapNs) {
    // a gap while pressed cancels the press, with no action (D10). A running hold has no release event: the
    // consumer ends it by staleness.
    if (s.se == SeState::Holding) hold_.active = false;
    s.se = SeState::Idle;
  }
  if (se == 0) { // RELEASED
    switch (s.se) {
    case SeState::Idle:
    case SeState::Ready:
    case SeState::Press1:
      s.se = SeState::Ready; // Press1: a single pressed sample is ignored
      break;
    case SeState::Pressing:
      if (t - s.seT0 < kSeShortMaxNs) ev_.sePress = true;
      s.se = SeState::Ready;
      break;
    case SeState::Holding:
      ev_.holdReleased = true;
      ev_.holdReleasedSerial = hold_.serial;
      ev_.holdReleasedNs = t;
      hold_.active = false;
      s.se = SeState::Ready;
      break;
    }
    return;
  }
  // PRESSED
  switch (s.se) {
  case SeState::Idle: // RELEASED not seen yet: a stuck or pre-pressed switch does nothing
    break;
  case SeState::Ready:
    s.se = SeState::Press1;
    s.seT0 = t;
    break;
  case SeState::Press1:
    s.se = SeState::Pressing;
    // fallthrough: the second sample may already be past the hold time only after a gap, which cancelled it above
    break;
  case SeState::Pressing:
    if (t - s.seT0 >= kSeHoldStartNs) {
      s.se = SeState::Holding;
      holdSerial_++;
      if (holdSerial_ == 0) holdSerial_ = 1;
      hold_.active = true;
      hold_.serial = holdSerial_;
      hold_.startNs = t;
      hold_.lastNs = t;
    }
    break;
  case SeState::Holding:
    hold_.lastNs = t;
    break;
  }
}

void RcCommandMapper::onAccepted(const CmdFrame &f, int64_t rxNs) {
  Slot &s = s_;
  if (s.seen && rxNs - s.lastNs > kGapClearNs) {
    clearDetectors(s);
    hold_.active = false;
  }
  const RcDecoded d = decode(f);
  const RcDecoded &p = s.d;
  const bool base = s.edgeBase && rxNs - s.lastNs <= kEdgeMaxAgeNs;
  if (base) {
    if ((s.safeSeen & kSafeSeenGo) && p.go == 0 && d.go == 1) {
      ev_.goEdge = true;
      ev_.goProfile = d.profile;
    }
    if (p.mode != d.mode) ev_.scEdgeTo = d.mode;
  }
  classifySe(s, d.reset, rxNs);
  s.seen = true;
  s.lastNs = rxNs;
  s.d = d;
  s.edgeBase = true;
  s.safeSeen |= safeBits(d);
}

void RcCommandMapper::onSessionChanged() {
  clearDetectors(s_);
  s_.seen = false;
  s_.d = RcDecoded();
  ev_ = RcEvents();
  hold_.active = false;
}

void RcCommandMapper::reset() {
  s_ = Slot();
  ev_ = RcEvents();
  hold_ = RcHoldState();
  holdSerial_ = 0;
}

RcEvents RcCommandMapper::takeEvents() {
  const RcEvents e = ev_;
  ev_.goEdge = false;
  ev_.goProfile = 0;
  ev_.scEdgeTo = RcEvents::kNone;
  ev_.sePress = false;
  ev_.holdReleased = false;
  return e;
}

} // namespace Safety
} // namespace VCTR
