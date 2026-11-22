#include "control_source.hpp"

namespace VCTR {
namespace Safety {

ControlSourceId ControlSourceTable::classify(uint8_t srcNode, RcLink link) {
  if (srcNode == kRadioNode && link == RcLink::Uart) return ControlSourceId::Radio;
  if (srcNode == kBenchNode && link == RcLink::Usb) return ControlSourceId::Bench;
  return ControlSourceId::None;
}

void ControlSourceTable::reject(RcReject r) {
  cur_.rejects[static_cast<size_t>(r)]++;
  cur_.rejectedTotal++;
  publish();
}

void ControlSourceTable::onMalformed() { reject(RcReject::BadLength); }

bool ControlSourceTable::onRcFrame(uint8_t srcNode, RcLink link, uint8_t seq, const RcFrame &frame, int64_t rxNs,
                                   ControlSourceId *accepted) {
  if (accepted) *accepted = ControlSourceId::None;
  const ControlSourceId id = classify(srcNode, link);
  if (id == ControlSourceId::None) {
    reject(RcReject::SourceNotInTable);
    return false;
  }
  const size_t i = ControlSourceSnapshot::index(id);

  // Sequence rule of the transport layer: accept if (seq - last) mod 256 is in [1, 127], or the source was silent
  // for kSeqResyncNs, or nothing was ever accepted. The state advances even if SOURCE_OK is clear below.
  Seq &s = seq_[i];
  if (s.seen && rxNs - s.acceptedNs <= kSeqResyncNs) {
    const uint8_t d = static_cast<uint8_t>(seq - s.last);
    if (d < 1 || d > 127) {
      reject(RcReject::StaleSequence);
      return false;
    }
  }
  s.seen = true;
  s.last = seq;
  s.acceptedNs = rxNs;

  if ((frame.flags & RcFrame::kFlagSourceOk) == 0) {
    reject(RcReject::SourceNotOk);
    return false;
  }

  RcSourceRecord &r = cur_.src[i];
  if (id == ControlSourceId::Radio) {
    if (!radioRunValid_ || !r.seen || rxNs - r.lastRxNs > kRadioGapNs) radioRunStartNs_ = rxNs;
    radioRunValid_ = true;
  }
  r.seen = true;
  r.lastRxNs = rxNs;
  r.accepted++;
  r.frame = frame;
  if (accepted) *accepted = id;
  publish();
  return true;
}

uint8_t ControlSourceTable::freshMask(int64_t nowNs) const {
  uint8_t m = 0;
  if (cur_.fresh(ControlSourceId::Radio, nowNs, kCtrlTimeoutNs)) m |= 1;
  if (cur_.fresh(ControlSourceId::Bench, nowNs, kCtrlTimeoutNs)) m |= 2;
  return m;
}

void ControlSourceTable::update(int64_t nowNs, bool committed) {
  if (committed) {
    // Latched: the source that was active when the vehicle became committed stays active.
    if (!cur_.latched) {
      cur_.latched = true;
      publish();
    }
    return;
  }
  const bool wasLatched = cur_.latched;
  cur_.latched = false;

  const bool radioFresh = cur_.fresh(ControlSourceId::Radio, nowNs, kCtrlTimeoutNs);
  const bool benchFresh = cur_.fresh(ControlSourceId::Bench, nowNs, kCtrlTimeoutNs);
  // continuous run: still ongoing (last radio state within the gap) and at least kHandoverNs long
  const bool radioHeld = radioFresh && radioRunValid_ && cur_.ageNs(ControlSourceId::Radio, nowNs) <= kRadioGapNs &&
                         (nowNs - radioRunStartNs_ >= kHandoverNs);

  ControlSourceId next = cur_.active;
  switch (cur_.active) {
  case ControlSourceId::Radio:
    if (!radioFresh) next = benchFresh ? ControlSourceId::Bench : ControlSourceId::None;
    break;
  case ControlSourceId::Bench:
    if (radioFresh && (radioHeld || !benchFresh)) next = ControlSourceId::Radio;
    else if (!benchFresh) next = ControlSourceId::None;
    break;
  case ControlSourceId::None:
  default:
    if (radioFresh) next = ControlSourceId::Radio;
    else if (benchFresh) next = ControlSourceId::Bench;
    break;
  }
  const bool changed = next != cur_.active;
  if (changed) {
    cur_.active = next;
    changed_ = true;
  }
  if (changed || wasLatched) publish();
}

bool ControlSourceTable::failsafeDue(int64_t nowNs, bool committed) const {
  return cur_.failsafeDue(nowNs, committed, kCtrlTimeoutNs);
}

void ControlSourceTable::publish() {
  const uint32_t v = version_;
  __atomic_store_n(&version_, v + 1, __ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_RELEASE);
  pub_ = cur_;
  __atomic_store_n(&version_, v + 2, __ATOMIC_RELEASE);
}

bool ControlSourceTable::snapshot(ControlSourceSnapshot &out) const {
  for (int attempt = 0; attempt < 8; attempt++) {
    const uint32_t v1 = __atomic_load_n(&version_, __ATOMIC_ACQUIRE);
    if (v1 & 1u) continue;
    ControlSourceSnapshot tmp = pub_;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    const uint32_t v2 = __atomic_load_n(&version_, __ATOMIC_RELAXED);
    if (v1 == v2) {
      out = tmp;
      return true;
    }
  }
  return false;
}

} // namespace Safety
} // namespace VCTR
