#include "request_executor.hpp"

#include <math.h>

#include "stop_altitude.hpp"

#ifndef FLASHMEM // cold code runs from flash and keeps ITCM (RAM1 code) small; empty on the host
#ifdef __IMXRT1062__
#define FLASHMEM __attribute__((section(".flashmem")))
#else
#define FLASHMEM
#endif
#endif

namespace VCTR {
namespace Safety {

FLASHMEM RequestResult RequestExecutor::handle(const RequestMsg &m, const RequestContext &ctx, RequestActionsI &actions) {
  for (const Entry &e : mem_) {
    if (e.used && e.src == m.src && e.reqId == m.reqId) {
      RequestResult r;
      r.status = e.status;
      r.detail = e.detail;
      r.duplicate = true;
      duplicates_++;
      return r;
    }
  }
  const RequestResult r = decide(m, ctx, actions);
  remember(m, r);
  lastReqId_ = m.reqId;
  lastStatus_ = r.status;
  if (r.status == kStatusDone) executed_++;
  else refused_++;
  return r;
}

FLASHMEM void RequestExecutor::remember(const RequestMsg &m, const RequestResult &r) {
  Entry &e = mem_[next_];
  e.used = true;
  e.src = m.src;
  e.reqId = m.reqId;
  e.status = r.status;
  e.detail = r.detail;
  next_ = static_cast<uint8_t>((next_ + 1) % kMemory);
}

FLASHMEM RequestResult RequestExecutor::decide(const RequestMsg &m, const RequestContext &ctx, RequestActionsI &actions) {
  RequestResult r;
  auto fail = [&r](uint8_t status, uint8_t detail) {
    r.status = status;
    r.detail = detail;
    return r;
  };
  if (m.target != kVehicleNode) return fail(kStatusRejectedInvalid, kDetailWrongTarget);
  if (m.action != kActionZeroEstimators && m.action != kActionSetStopAltitude)
    return fail(kStatusRejectedInvalid, kDetailUnknownAction);
  if (!ctx.sessionLatched || m.session == 0 || m.session != ctx.latchedSession)
    return fail(kStatusRejectedState, kDetailSession);
  if (ctx.committed || ctx.interlockOpen) return fail(kStatusRejectedState, kDetailNotOnGround);

  if (m.action == kActionZeroEstimators) {
    if (!actions.zeroEstimators(m.argF)) return fail(kStatusRejectedState, kDetailZeroingRefused);
  } else {
    StopAltContext sc;
    sc.interlockOpen = ctx.interlockOpen;
    sc.committed = ctx.committed;
    switch (decideSetStopAltitude(m.argF, sc)) {
    case StopAltResult::Accepted:
      break;
    case StopAltResult::NotFinite:
    case StopAltResult::OutOfRange:
      return fail(kStatusRejectedInvalid, kDetailArgRange);
    default:
      return fail(kStatusRejectedState, kDetailNotOnGround);
    }
    actions.setStopAltitude(m.argF);
  }
  r.status = kStatusDone;
  r.detail = 0;
  r.executed = true;
  return r;
}

} // namespace Safety
} // namespace VCTR
