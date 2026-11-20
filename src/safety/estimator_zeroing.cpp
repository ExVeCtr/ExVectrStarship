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

bool EstimatorZeroing::update(int64_t nowNs) {
  if (active_ && nowNs >= endNs_) {
    active_ = false;
    zeroed_ = true;
    return true;
  }
  return false;
}

bool EstimatorZeroing::abort() {
  const bool was = active_;
  active_ = false;
  return was;
}

} // namespace Safety
} // namespace VCTR
