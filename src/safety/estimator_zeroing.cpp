#include "safety/estimator_zeroing.hpp"

#include <math.h>

namespace VCTR {
namespace Safety {

const char *toString(ZeroRequestResult r) {
  switch (r) {
  case ZeroRequestResult::Started: return "started";
  case ZeroRequestResult::Restarted: return "restarted";
  case ZeroRequestResult::BadParam: return "bad paramInt";
  case ZeroRequestResult::NotInitialised: return "not initialised";
  case ZeroRequestResult::Armed: return "armed";
  case ZeroRequestResult::MotorsEnabled: return "motors enabled";
  case ZeroRequestResult::NotOnGround: return "not on ground";
  case ZeroRequestResult::HoldActive: return "hold already active";
  }
  return "?";
}

int64_t EstimatorZeroing::durationNs(float requestedS) {
  float s = requestedS;
  if (isnan(s) || s == 0.0f) {
    s = kDefaultDurationS;
  } else if (s < kMinDurationS) {
    s = kMinDurationS;
  } else if (s > kMaxDurationS) {
    s = kMaxDurationS;
  }
  return static_cast<int64_t>(static_cast<double>(s) * kSecondNs);
}

ZeroRequestResult EstimatorZeroing::request(int16_t paramInt, float paramFloat, const ZeroRequestContext &ctx,
                                            int64_t nowNs) {
  ZeroRequestResult r;
  if (paramInt != kParamInt) {
    r = ZeroRequestResult::BadParam;
  } else if (!ctx.initialised) {
    r = ZeroRequestResult::NotInitialised;
  } else if (ctx.armed) {
    r = ZeroRequestResult::Armed;
  } else if (ctx.motorsEnabled) {
    r = ZeroRequestResult::MotorsEnabled;
  } else if (!ctx.onGround) {
    r = ZeroRequestResult::NotOnGround;
  } else {
    r = active_ ? ZeroRequestResult::Restarted : ZeroRequestResult::Started;
    start(nowNs, durationNs(paramFloat));
    return r;
  }
  refused_++;
  return r;
}

ZeroRequestResult EstimatorZeroing::holdStart(const ZeroRequestContext &ctx, int64_t nowNs) {
  ZeroRequestResult r;
  if (holdActive_) {
    refused_++;
    return ZeroRequestResult::HoldActive;
  }
  if (!ctx.initialised) {
    r = ZeroRequestResult::NotInitialised;
  } else if (ctx.armed) {
    r = ZeroRequestResult::Armed;
  } else if (ctx.motorsEnabled) {
    r = ZeroRequestResult::MotorsEnabled;
  } else if (!ctx.onGround) {
    r = ZeroRequestResult::NotOnGround;
  } else {
    holdActive_ = true;
    holdStartNs_ = nowNs;
    holdLastNs_ = nowNs;
    return ZeroRequestResult::Started;
  }
  refused_++;
  return r;
}

void EstimatorZeroing::holdRefresh(int64_t sampleNs) {
  if (holdActive_ && sampleNs > holdLastNs_) holdLastNs_ = sampleNs;
}

EstimatorZeroing::HoldEnd EstimatorZeroing::finishHold(int64_t ranNs, bool aborted) {
  holdActive_ = false;
  lastHoldRanNs_ = ranNs < 0 ? 0 : ranNs;
  if (aborted) {
    zeroed_ = false;
    return HoldEnd::Aborted;
  }
  if (ranNs >= kHoldMinNs) {
    zeroed_ = true;
    return HoldEnd::Valid;
  }
  zeroed_ = false; // the estimators hold a partial zero (as abort())
  return HoldEnd::Short;
}

EstimatorZeroing::HoldEnd EstimatorZeroing::holdRelease(int64_t nowNs) {
  if (!holdActive_) return HoldEnd::None;
  int64_t end = nowNs;
  if (end > holdStartNs_ + kHoldCapNs) end = holdStartNs_ + kHoldCapNs;
  return finishHold(end - holdStartNs_, false);
}

EstimatorZeroing::HoldEnd EstimatorZeroing::holdAbort() {
  if (!holdActive_) return HoldEnd::None;
  return finishHold(0, true);
}

bool EstimatorZeroing::update(int64_t nowNs) {
  bool completed = false;
  if (active_ && nowNs >= endNs_) {
    active_ = false;
    zeroed_ = true;
    completed = true;
  }
  if (holdActive_) {
    // The hold ends kHoldGraceNs after the last fresh "held" sample (staleness) or at the cap, whichever is first.
    int64_t end = holdLastNs_ + kHoldGraceNs;
    if (end > holdStartNs_ + kHoldCapNs) end = holdStartNs_ + kHoldCapNs;
    if (nowNs >= end) {
      pendingHoldEnd_ = finishHold(end - holdStartNs_ - (end == holdLastNs_ + kHoldGraceNs ? kHoldGraceNs : 0), false);
      if (pendingHoldEnd_ == HoldEnd::Valid) completed = true;
    }
  }
  return completed;
}

bool EstimatorZeroing::abort() {
  const bool was = active();
  holdActive_ = false;
  if (was) {
    // The estimators were already in zeroing mode for part of the window: whatever zero existed before is no
    // longer what they hold (review 2026-11-20). Invalid until a complete new window ends.
    zeroed_ = false;
  }
  active_ = false;
  return was;
}

} // namespace Safety
} // namespace VCTR
