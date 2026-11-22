#include "rc_mapper.hpp"

namespace VCTR {
namespace Safety {

uint8_t RcCommandMapper::decodeTwoPos(int16_t v) {
  if (v == RcFrame::kChInvalid || v < -1000 || v > 1000) return RcDecoded::kInvalid;
  if (v <= -500) return 0;
  if (v >= 500) return 1;
  return RcDecoded::kInvalid;
}

uint8_t RcCommandMapper::decodeThreePos(int16_t v) {
  if (v == RcFrame::kChInvalid || v < -1000 || v > 1000) return RcDecoded::kInvalid;
  if (v <= -500) return 0;
  if (v >= -200 && v <= 200) return 1;
  if (v >= 500) return 2;
  return RcDecoded::kInvalid;
}

RcDecoded RcCommandMapper::decode(const RcFrame &f) {
  RcDecoded d;
  d.sa = decodeTwoPos(f.ch[kChSa]);
  d.sb = decodeThreePos(f.ch[kChSb]);
  d.sc = decodeThreePos(f.ch[kChSc]);
  d.sd = decodeTwoPos(f.ch[kChSd]);
  d.se = decodeTwoPos(f.ch[kChSe]);
  return d;
}

uint8_t RcCommandMapper::safeBits(const RcDecoded &d) {
  uint8_t b = 0;
  if (d.sa == 0) b |= kSafeSeenGo;
  if (d.sc == 0) b |= kSafeSeenScReal;
  if (d.sd == 0) b |= kSafeSeenSdEnable;
  if (d.se == 0) b |= kSafeSeenSePress;
  return b;
}

void RcCommandMapper::clearDetectors(Slot &s) {
  s.safeSeen = 0;
  s.edgeBase = false;
  s.se = SeState::Idle;
}

void RcCommandMapper::classifySe(Slot &s, uint8_t se, int64_t t) {
  const bool inPress = s.se == SeState::Press1 || s.se == SeState::Pressing || s.se == SeState::Holding;
  if (inPress && t - s.lastNs > kSeGapNs) {
    // a gap while pressed cancels the press, with no action (D10). A running hold has no release event: the
    // consumer ends it by staleness.
    if (s.se == SeState::Holding) hold_.active = false;
    s.se = SeState::Idle;
  }
  if (se == RcDecoded::kInvalid) {
    if (s.se == SeState::Press1 || s.se == SeState::Pressing || s.se == SeState::Holding) {
      if (s.se == SeState::Holding) hold_.active = false;
      s.se = SeState::Idle;
    }
    return;
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

void RcCommandMapper::onAccepted(ControlSourceId source, const RcFrame &f, int64_t rxNs) {
  if (source != ControlSourceId::Radio && source != ControlSourceId::Bench) return;
  Slot &s = slot_[ControlSourceSnapshot::index(source)];
  if (s.seen && rxNs - s.lastNs > kGapClearNs) {
    clearDetectors(s);
    if (source == active_) hold_.active = false;
  }
  const RcDecoded d = decode(f);
  if (source == active_) {
    const RcDecoded &p = s.d;
    const bool base = s.edgeBase && rxNs - s.lastNs <= kEdgeMaxAgeNs;
    if (base) {
      if ((s.safeSeen & kSafeSeenGo) && p.sa == 0 && d.sa == 1) {
        ev_.goEdge = true;
        ev_.goSb = d.sb;
      }
      const bool pc = p.sc == 0 || p.sc == 1, nc = d.sc == 0 || d.sc == 1;
      if (pc && nc && p.sc != d.sc) {
        if (d.sc == 0)
          ev_.scEdgeTo = 0;
        else if (s.safeSeen & kSafeSeenScReal)
          ev_.scEdgeTo = 1;
      }
    }
    classifySe(s, d.se, rxNs);
  }
  s.seen = true;
  s.lastNs = rxNs;
  s.d = d;
  s.edgeBase = true;
  s.safeSeen |= safeBits(d);
}

void RcCommandMapper::onActiveChanged(ControlSourceId newActive) {
  for (Slot &s : slot_) clearDetectors(s);
  active_ = newActive;
  ev_ = RcEvents();
  hold_.active = false;
}

void RcCommandMapper::reset() {
  for (Slot &s : slot_) s = Slot();
  active_ = ControlSourceId::None;
  ev_ = RcEvents();
  hold_ = RcHoldState();
  holdSerial_ = 0;
}

RcEvents RcCommandMapper::takeEvents() {
  const RcEvents e = ev_;
  ev_.goEdge = false;
  ev_.goSb = RcDecoded::kInvalid;
  ev_.scEdgeTo = RcEvents::kNone;
  ev_.sePress = false;
  ev_.holdReleased = false;
  return e;
}

RcDecoded RcCommandMapper::decoded(ControlSourceId source) const {
  if (source != ControlSourceId::Radio && source != ControlSourceId::Bench) return RcDecoded();
  return slot_[ControlSourceSnapshot::index(source)].d;
}

uint8_t RcCommandMapper::safeSeen(ControlSourceId source) const {
  if (source != ControlSourceId::Radio && source != ControlSourceId::Bench) return 0;
  return slot_[ControlSourceSnapshot::index(source)].safeSeen;
}

} // namespace Safety
} // namespace VCTR
