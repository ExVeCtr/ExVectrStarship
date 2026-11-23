#include "rc_command.hpp"

namespace VCTR {
namespace Safety {

GoRefusal decideGo(const GoContext &c) {
  if (c.latched) return GoRefusal::Latched;
  if (!c.modeReady) return GoRefusal::NotReady;
  if (c.zeroingActive || !c.zeroValid) return GoRefusal::Zeroing;
  const bool scOk = (c.mode == 1) == c.simulationApplied;
  if (!scOk) return GoRefusal::ScMismatch;
  if (c.profile != kImplementedProfile) return GoRefusal::SbProfile;
  if (c.requestBusy) return GoRefusal::RequestBusy;
  return GoRefusal::None;
}

const char *toString(GoRefusal r) {
  switch (r) {
  case GoRefusal::None: return "none";
  case GoRefusal::NotReady: return "vehicle not Ready";
  case GoRefusal::Latched: return "latched abort or failsafe (SE reset needed)";
  case GoRefusal::Zeroing: return "estimator zeroing active or no valid zero";
  case GoRefusal::ScMismatch: return "vehicle mode (SC) not equal to the applied mode";
  case GoRefusal::SbProfile: return "profile (SB) not implemented";
  case GoRefusal::Interlock: return "interlock";
  case GoRefusal::RequestBusy: return "request or calibration in progress";
  }
  return "?";
}

ScAction decideSc(uint8_t edgeTo, const ScContext &c) {
  if (edgeTo != 0 && edgeTo != 1) return ScAction::None;
  const bool wantSim = edgeTo == 1;
  if (wantSim == c.simulationApplied) return ScAction::None;
  if (c.committed || !c.onGround || c.modeFailure) return ScAction::Refused;
  return wantSim ? ScAction::EnterSimulation : ScAction::EnterReal;
}

bool onGround(const GroundContext &g) {
  if (g.interlockOpen || g.motorsEnabled) return false;
  if (g.modeFailure) return true;
  if (!g.missionIdleOrFinished) return false;
  if (g.modeGround) return true;
  return g.modeRunning && !g.chainPending;
}

bool committed(const GroundContext &g) {
  return g.interlockOpen || g.chainPending || (g.modeRunning && !onGround(g));
}

RcIntents RcCommandLogic::evaluate(RcCommandMapper &mapper, const RcPollInput &in) {
  RcIntents out;
  const RcEvents ev = mapper.takeEvents();
  const RcHoldState h = mapper.hold();

  // SA released: abort on the level, every tick (safe direction, idempotent in the caller).
  if (in.haveSource && in.chainPendingOrRunning && in.decoded.go == 0) out.abortLevel = true;

  // SA go edge (unsafe direction): needs the edge, SA still at GO now, and every precondition.
  if (ev.goEdge) {
    out.goEdgeSeen = true;
    if (in.haveSource && in.decoded.go == 1) {
      GoContext g = in.go;
      g.mode = in.decoded.mode;
      g.simulationApplied = in.simulationApplied;
      g.profile = ev.goProfile;
      const GoRefusal r = decideGo(g);
      if (r == GoRefusal::None) {
        out.go = true;
        out.goProfile = ev.goProfile;
        lastGoRefusal_ = GoRefusal::None;
      } else {
        out.goRefused = r;
        lastGoRefusal_ = r;
      }
    }
  }

  // SC change (only an observed edge acts).
  if (ev.scEdgeTo != RcEvents::kNone) {
    ScContext c;
    c.onGround = in.onGround;
    c.committed = in.committed;
    c.modeFailure = in.modeFailure;
    c.simulationApplied = in.simulationApplied;
    out.sc = decideSc(ev.scEdgeTo, c);
  }

  // SE short press: reset only on the ground with the interlock closed (onGround includes both) and the GS failsafe
  // request cleared (owner Q6: a reset with the request still 1 would only re-latch at the next tick).
  if (ev.sePress) {
    if (in.onGround && !in.interlockOpen && in.decoded.failsafe == 0) {
      out.se = SeAction::Reset;
    } else {
      out.se = SeAction::Refused;
      noteSeRefused(in.nowNs);
    }
  }

  // SE hold zeroing (D11).
  if (holdRunning_) {
    if (!in.zeroingHoldActive) {
      holdRunning_ = false; // ended by staleness, the cap or a reset
    } else if (in.interlockOpen) {
      out.hold = HoldAction::Abort;
      holdRunning_ = false;
    } else if (ev.holdReleased && ev.holdReleasedSerial == runningSerial_) {
      out.hold = HoldAction::Release;
      out.holdNs = ev.holdReleasedNs;
      holdRunning_ = false;
    } else if (h.active && h.serial == runningSerial_) {
      out.hold = HoldAction::Refresh;
      out.holdNs = h.lastNs;
    }
  } else if (h.active && h.serial != handledSerial_) {
    handledSerial_ = h.serial; // one attempt per hold: a refused hold is not retried every tick
    if (in.nowNs - h.lastNs <= kHoldFreshNs) {
      out.hold = HoldAction::Start;
      out.holdNs = h.startNs;
      runningSerial_ = h.serial;
    }
  }
  return out;
}

void RcCommandLogic::holdStartResult(bool ok, int64_t nowNs) {
  holdRunning_ = ok;
  if (!ok) noteSeRefused(nowNs);
}

} // namespace Safety
} // namespace VCTR
