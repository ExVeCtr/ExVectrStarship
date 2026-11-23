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

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static bool knownAction(uint8_t a) { return a == 7 || a == 8 || (a >= 12 && a <= 23); }

FLASHMEM RequestResult RequestExecutor::decide(const RequestMsg &m, const RequestContext &ctx, RequestActionsI &actions) {
  RequestResult r;
  auto fail = [&r](uint8_t status, uint8_t detail) {
    r.status = status;
    r.detail = detail;
    return r;
  };
  if (m.target != kVehicleNode) return fail(kStatusRejectedInvalid, kDetailWrongTarget);
  if (!knownAction(m.action)) return fail(kStatusRejectedInvalid, kDetailUnknownAction);
  if (!ctx.sessionLatched || m.session == 0 || m.session != ctx.latchedSession)
    return fail(kStatusRejectedState, kDetailSession);

  const uint8_t act = m.action;
  if (act == kActionZeroEstimators || act == kActionSetStopAltitude) {
    if (ctx.committed || ctx.interlockOpen) return fail(kStatusRejectedState, kDetailNotOnGround);
    if (act == kActionZeroEstimators) {
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
  } else if (act == kActionCalibApply || act == kActionSaveConfig || act == kActionCalibControl) {
    uint8_t sub = 0, code = 0;
    if (act == kActionCalibApply) {
      if (m.argI < 0 || m.argI > 2) return fail(kStatusRejectedInvalid, kDetailArgRange);
      sub = static_cast<uint8_t>(m.argI);
    } else if (act == kActionCalibControl) {
      const int32_t c = m.argI % 10;
      sub = static_cast<uint8_t>(m.argI / 10);
      if (m.argI < 0 || m.argI > 29 || !(c == 0 || c == 1 || c == 2 || c == 9)) return fail(kStatusRejectedInvalid, kDetailArgRange);
      code = static_cast<uint8_t>(c);
      if (ctx.simulationMode) return fail(kStatusRejectedState, kDetailCalibRefused);
    }
    if (!ctx.calibGatesOk) return fail(kStatusRejectedState, kDetailCalibRefused);
    bool ok;
    if (act == kActionCalibApply) ok = actions.calibApply(sub);
    else if (act == kActionSaveConfig) ok = actions.saveConfig();
    else ok = actions.calibControl(sub, code);
    if (!ok) return fail(kStatusRejectedState, kDetailCalibRefused);
  } else if (act == kActionGetHomePosition) {
    if (!ctx.homeAvailable) return fail(kStatusRejectedState, kDetailNoHome);
    r.sendHome = true;
  } else { // 15..22 simulation world
    if (!ctx.simulationMode) return fail(kStatusRejectedState, kDetailNotSimulation);
    if (act == kActionSimFaultClear) {
      if (!actions.simFault(decideSimFault(kSimFaultClearType, 0, kSimFaultMagic, 0.0f, true)))
        return fail(kStatusRejectedState, kDetailNotSimulation);
    } else {
      if (!isfinite(m.argF)) return fail(kStatusRejectedInvalid, kDetailArgRange);
      const float v = m.argF;
      switch (act) {
      case kActionSimWindSpeed: actions.simWindSpeed(clampf(v, 0.0f, 20.0f)); break;
      case kActionSimWindDirection: actions.simWindDirection(atan2f(sinf(v), cosf(v))); break;
      case kActionSimGustStrength: actions.simGustStrength(clampf(v, 0.0f, 10.0f)); break;
      case kActionSimGustPeriod: actions.simGustPeriod(clampf(v, 0.5f, 30.0f)); break;
      case kActionSimGustSeed: {
        uint32_t seed = uint32_t(clampf(v, 0.0f, 16777215.0f) + 0.5f); // float: 16777215 + 0.5 rounds up
        actions.simGustSeed(seed > 16777215u ? 16777215u : seed);
      } break;
      default: { // 20 stuck, 21 slow
        if (m.argI < 0 || m.argI > 3) return fail(kStatusRejectedInvalid, kDetailArgRange);
        const SimFaultDecision d = decideSimFault(act == kActionSimFlapStuck ? kSimFlapStuckType : kSimFlapSlowType,
                                                  static_cast<uint8_t>(m.argI), kSimFaultMagic, v, true);
        if (d.action == SimFaultAction::Refuse) return fail(kStatusRejectedInvalid, kDetailArgRange);
        if (!actions.simFault(d)) return fail(kStatusRejectedState, kDetailNotSimulation);
      } break;
      }
    }
  }
  r.status = kStatusDone;
  r.detail = 0;
  r.executed = true;
  return r;
}

} // namespace Safety
} // namespace VCTR
