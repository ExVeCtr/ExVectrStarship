#include "command_ingress.hpp"

namespace VCTR {
namespace Safety {

void CommandIngress::reject(CmdReject r) {
  cur_.rejects[static_cast<size_t>(r)]++;
  cur_.rejectedTotal++;
  publish();
}

void CommandIngress::onMalformed() { reject(CmdReject::BadLength); }

CommandIngress::Result CommandIngress::onFrame(uint8_t srcNode, CmdLink link, uint8_t seq, const CmdFrame &frame,
                                               int64_t rxNs) {
  Result res;
  if (srcNode != kGsNode) {
    reject(CmdReject::SourceNotNode0);
    return res;
  }

  // Session latch, before the sequence filter (a foreign sender must never touch the latched session's sequence).
  if (!cur_.latched || frame.session != cur_.session) {
    const bool first = !cur_.latched;
    const bool silent = first || rxNs - sessionRxNs_ >= kSessionSilentNs;
    if (!first && !(silent && (!committed_ || modeFailure_))) {
      cur_.foreignSeen = true;
      cur_.foreignLastNs = rxNs;
      reject(CmdReject::ForeignSession);
      return res;
    }
    cur_.latched = true;
    cur_.session = frame.session;
    cur_.seen = false;
    cur_.frame = CmdFrame();
    cur_.linkSeen[0] = cur_.linkSeen[1] = false;
    seqSeen_ = false;
    sessionRxNs_ = rxNs;
    changed_ = true;
    res.sessionChanged = true;
  }

  // Per-session sequence filter. The state advances only on accepted frames.
  const size_t li = static_cast<size_t>(link);
  if (seqSeen_ && rxNs - seqAcceptedNs_ <= kSeqResyncNs) {
    const uint8_t d = static_cast<uint8_t>(seq - seqLast_);
    if (d < 1 || d > 127) {
      if (d == 0) { // the copy of the accepted frame on the other link: alive, but not new
        cur_.linkSeen[li] = true;
        cur_.linkLastNs[li] = rxNs;
        sessionRxNs_ = rxNs;
      }
      reject(CmdReject::StaleSequence);
      return res;
    }
  }
  seqSeen_ = true;
  seqLast_ = seq;
  seqAcceptedNs_ = rxNs;
  sessionRxNs_ = rxNs;

  cur_.seen = true;
  cur_.lastRxNs = rxNs;
  cur_.accepted++;
  cur_.frame = frame;
  cur_.linkSeen[li] = true;
  cur_.linkLastNs[li] = rxNs;
  res.accepted = true;
  publish();
  return res;
}

void CommandIngress::update(int64_t nowNs, bool committed, bool modeFailure) {
  (void)nowNs;
  committed_ = committed;
  modeFailure_ = modeFailure;
  if (cur_.commitLatched != committed) {
    cur_.commitLatched = committed;
    publish();
  }
}

bool CommandIngress::onRequest(uint32_t session) {
  const bool ok = requestAllowed(session);
  if (ok)
    requestsAccepted_++;
  else
    requestsRefused_++;
  return ok;
}

void CommandIngress::publish() {
  const uint32_t v = version_;
  __atomic_store_n(&version_, v + 1, __ATOMIC_RELAXED);
  __atomic_thread_fence(__ATOMIC_RELEASE);
  pub_ = cur_;
  __atomic_store_n(&version_, v + 2, __ATOMIC_RELEASE);
}

bool CommandIngress::snapshot(CmdIngressSnapshot &out) const {
  for (int attempt = 0; attempt < 8; attempt++) {
    const uint32_t v1 = __atomic_load_n(&version_, __ATOMIC_ACQUIRE);
    if (v1 & 1u) continue;
    CmdIngressSnapshot tmp = pub_;
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
