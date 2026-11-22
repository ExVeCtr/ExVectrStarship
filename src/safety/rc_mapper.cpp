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

void RcCommandMapper::onAccepted(ControlSourceId source, const RcFrame &f, int64_t rxNs) {
  if (source != ControlSourceId::Radio && source != ControlSourceId::Bench) return;
  Slot &s = slot_[ControlSourceSnapshot::index(source)];
  if (s.seen && rxNs - s.lastNs > kGapClearNs) s.safeSeen = 0;
  s.seen = true;
  s.lastNs = rxNs;
  s.d = decode(f);
  s.safeSeen |= safeBits(s.d);
}

void RcCommandMapper::onActiveChanged() {
  for (Slot &s : slot_) s.safeSeen = 0;
}

void RcCommandMapper::reset() {
  for (Slot &s : slot_) s = Slot();
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
