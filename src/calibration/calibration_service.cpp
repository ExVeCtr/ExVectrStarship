#include "calibration/calibration_service.hpp"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "calibration/calibration_math.hpp"

namespace VCTR {
namespace Calibration {

using DSP::Calib::CalResult;
using DSP::Calib::PoseEvent;

namespace {
constexpr double kRadToDeg = 57.29577951308232;
DSP::Calib::MagEllipsoidConfig magConfig(const ServiceConfig &cfg) {
  DSP::Calib::MagEllipsoidConfig mc;
  mc.softIron = cfg.magSoftIron;
  return mc;
}

constexpr int64_t kUpdateEstimateNs = 1000000000LL; // mag centre refresh (thinning and coverage), 1 Hz
}

const char *toString(Sensor s) {
  switch (s) {
  case Sensor::Gyro: return "gyro";
  case Sensor::Accel: return "accel";
  case Sensor::Mag: return "mag";
  }
  return "?";
}

const char *toString(State s) {
  switch (s) {
  case State::Idle: return "IDLE";
  case State::Collecting: return "COLLECTING";
  case State::Fitting: return "FITTING";
  case State::Review: return "REVIEW";
  case State::Failed: return "FAILED";
  case State::Applied: return "APPLIED";
  case State::Saved: return "SAVED";
  }
  return "?";
}

CalibrationService::CalibrationService(float (*magBuffer)[3], size_t magCapacity, const ServiceConfig &cfg)
    : cfg_(cfg), magFit_(magBuffer, magCapacity, magConfig(cfg)) {
  poses_.reset();
  coverageHintGiven_ = false;
}

void CalibrationService::logf(const char *fmt, ...) {
  if (sink_ == nullptr)
    return;
  char line[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  sink_(sinkCtx_, line);
}

void CalibrationService::setActive(Sensor s, const double offset[3], const double M[9]) {
  ActiveCalibration &a = active_[idx(s)];
  for (int i = 0; i < 3; i++)
    a.offset[i] = offset[i];
  for (int i = 0; i < 9; i++)
    a.M[i] = M[i];
  a.valid = true;
}

bool CalibrationService::anyBusy() const {
  for (int i = 0; i < 3; i++)
    if (state_[i] == State::Collecting || state_[i] == State::Fitting)
      return true;
  return false;
}

bool CalibrationService::applicable(Sensor s) const {
  if (!(state_[idx(s)] == State::Review && DSP::Calib::acceptable(result_[idx(s)])))
    return false;
  if (s == Sensor::Accel)
    return accelCandidateParams(accelRes_) == Reason::Ok;
  if (s == Sensor::Mag)
    return magCandidateParams(magRes_) == Reason::Ok;
  return true;
}

// ---- samples ----

void CalibrationService::addGyro(int64_t tNs, const double w[3]) {
  if (gyro_.collecting())
    gyro_.addGyro(tNs, w);
  if (accelSession())
    still_.addGyro(tNs, w);
}

void CalibrationService::addAccel(int64_t tNs, const double a[3]) {
  if (gyro_.collecting())
    gyro_.addAccel(tNs, a);
  if (accelSession()) {
    still_.addAccel(tNs, a);
    stillNow_ = still_.still();
    const PoseEvent ev = poses_.addAccel(tNs, a, stillNow_);
    if (ev != PoseEvent::None && noteCount_ < kNoteCap) {
      PoseNote &n = notes_[(noteHead_ + noteCount_) % kNoteCap];
      n.ev = ev;
      n.count = static_cast<uint8_t>(poses_.count());
      n.angleDeg = static_cast<float>(poses_.lastNearestAngle() * kRadToDeg);
      noteCount_++;
    }
  }
}

void CalibrationService::addMag(int64_t tNs, const double m[3]) {
  if (gyro_.collecting())
    gyro_.addMag(tNs, m);
  if (magSession()) {
    if (haveMagSample_ && tNs - lastMagNs_ < cfg_.magDecimateNs && tNs >= lastMagNs_)
      return;
    haveMagSample_ = true;
    lastMagNs_ = tNs;
    if (verify_[2]) {
      const ActiveCalibration &a = active_[2];
      if (!a.valid)
        return;
      const double d[3] = {m[0] - a.offset[0], m[1] - a.offset[1], m[2] - a.offset[2]};
      double v[3];
      mat3Vec(a.M, d, v);
      const double n = vec3Norm(v);
      magVerifyN_++;
      magVerifySum_ += n;
      magVerifySum2_ += n * n;
      magVerifyCoverage_.add(d);
    } else {
      magFit_.addSample(m);
    }
  }
}

void CalibrationService::addTemperature(int64_t tNs, double tempC) {
  if (gyro_.collecting())
    gyro_.addTemperature(tNs, tempC);
}

// ---- sessions ----

StartResult CalibrationService::start(Sensor s, bool verify, int64_t nowNs) {
  if (anyBusy())
    return StartResult::Busy;
  if (gyroAuto_ && gyro_.collecting())
    cancelGyroRun();

  const size_t i = idx(s);
  state_[i] = State::Collecting;
  verify_[i] = verify;
  verifyPass_[i] = false;
  result_[i] = CalResult::None;
  startNs_[i] = nowNs;
  lastProgressNs_ = nowNs;
  lastEstimateNs_ = nowNs;

  switch (s) {
  case Sensor::Accel:
    poses_.reset();
    coverageHintGiven_ = false;
    {
      DSP::Calib::StillnessConfig sc;
      if (gyroBiasTrusted_ && active_[0].valid) {
        // stillness of the CALIBRATED rate: |w - bias| < 0.02 rad/s (the default rule)
      } else {
        const DSP::Calib::GyroBiasConfig gc; // bias-free: the limits of the gyro run's 1 s blocks
        sc.maxGyroNorm = 0;
        sc.maxGyroStd = gc.maxBlockStd;
        sc.maxGyroP2P = gc.maxBlockP2P;
      }
      still_ = DSP::Calib::StillnessDetector(sc);
      if (active_[0].valid)
        still_.setGyroBias(active_[0].offset);
    }
    accelRes_ = DSP::Calib::AccelFitResult();
    noteHead_ = noteCount_ = 0;
    stillNow_ = false;
    logf("Accel calibration%s started: put the ship in a new, still pose (>20 deg from the last), at least 9 poses, "
         "12 recommended", verify ? " VERIFY" : "");
    break;
  case Sensor::Mag:
    magFit_.reset();
    haveMagSample_ = false;
    magVerifyN_ = 0;
    magVerifySum_ = magVerifySum2_ = 0;
    magVerifyCoverage_.reset();
    magRes_ = DSP::Calib::MagEllipsoidResult();
    logf("Mag calibration%s started: tumble the ship slowly in all directions (60-120 s), away from metal",
         verify ? " VERIFY" : "");
    break;
  case Sensor::Gyro:
    gyroAuto_ = false;
    gyroRes_ = DSP::Calib::GyroBiasResult();
    gyro_.start(nowNs, nowNs + cfg_.gyroManualDeadlineNs);
    logf("Gyro calibration%s started: keep the ship still for 5 s", verify ? " VERIFY" : "");
    break;
  }
  return StartResult::Started;
}

bool CalibrationService::stop(Sensor s, int64_t nowNs) {
  (void)nowNs;
  const size_t i = idx(s);
  if (state_[i] != State::Collecting)
    return false;
  if (s == Sensor::Gyro) {
    gyro_.cancel();
    state_[i] = State::Idle;
    logf("%s", "Gyro calibration stopped without result");
    return true;
  }
  state_[i] = State::Fitting;
  return true;
}

void CalibrationService::discard(Sensor s) {
  const size_t i = idx(s);
  if (s == Sensor::Gyro && state_[i] == State::Collecting)
    gyro_.cancel();
  if (state_[i] == State::Collecting || state_[i] == State::Fitting || state_[i] == State::Review ||
      state_[i] == State::Failed) {
    state_[i] = State::Idle;
    result_[i] = CalResult::None;
    logf("%s calibration candidate discarded, active calibration unchanged", toString(s));
  }
}

void CalibrationService::markApplied(Sensor s) {
  if (state_[idx(s)] == State::Review)
    state_[idx(s)] = State::Applied;
}

void CalibrationService::markSaved() {
  for (int i = 0; i < 3; i++)
    if (state_[i] == State::Applied)
      state_[i] = State::Saved;
}

// ---- automatic gyro run ----

bool CalibrationService::startGyroRun(int64_t nowNs, int64_t deadlineNs) {
  if (gyroSession() || accelSession() || magSession() || state_[0] == State::Fitting)
    return false;
  gyroAuto_ = true; // a pending (untaken) result stays in autoRes_ with gyroDone_ set
  gyro_.start(nowNs, deadlineNs);
  return true;
}

void CalibrationService::cancelGyroRun() {
  if (gyroAuto_ && gyro_.collecting())
    gyro_.cancel();
}

bool CalibrationService::takeGyroRunDone(GyroRunDone &out) {
  if (!gyroDone_)
    return false;
  gyroDone_ = false;
  out.result = autoRes_;
  return true;
}

void CalibrationService::finishGyroRun(int64_t nowNs) {
  (void)nowNs;
  if (gyroAuto_) {
    autoRes_ = gyro_.result(); // never touches a manual candidate waiting in Review
    gyroDone_ = true;
    return;
  }
  gyroRes_ = gyro_.result();
  // manual session
  const size_t i = 0;
  result_[i] = gyroRes_.result;
  if (verify_[i]) {
    const ActiveCalibration &a = active_[0];
    double worst = 0;
    for (int k = 0; k < 3; k++) {
      const double d = fabs(gyroRes_.bias[k] - (a.valid ? a.offset[k] : 0.0));
      if (d > worst)
        worst = d;
    }
    const bool pass = DSP::Calib::acceptable(gyroRes_.result) && worst < cfg_.verifyGyroTolerance;
    verifyPass_[i] = pass;
    state_[i] = State::Idle;
    logf("Gyro VERIFY %s: run %s, still bias differs from the active one by %.3f deg/s (limit %.2f)",
         pass ? "PASS" : "FAIL", DSP::Calib::toString(gyroRes_.result), worst * kRadToDeg,
         cfg_.verifyGyroTolerance * kRadToDeg);
    return;
  }
  state_[i] = DSP::Calib::acceptable(gyroRes_.result) ? State::Review : State::Failed;
  logf("Gyro calibration %s: %s, bias %.3f %.3f %.3f deg/s, noise %.3f %.3f %.3f deg/s -> %s",
       state_[i] == State::Review ? "result" : "FAILED", DSP::Calib::toString(gyroRes_.result),
       gyroRes_.bias[0] * kRadToDeg, gyroRes_.bias[1] * kRadToDeg, gyroRes_.bias[2] * kRadToDeg,
       gyroRes_.noiseStd[0] * kRadToDeg, gyroRes_.noiseStd[1] * kRadToDeg, gyroRes_.noiseStd[2] * kRadToDeg,
       state_[i] == State::Review ? "apply with CalibApply" : "discard and repeat");
}

// ---- fits ----

void CalibrationService::runAccelFit(int64_t nowNs) {
  (void)nowNs;
  const size_t i = 1;
  const size_t n = poses_.count();
  DSP::Calib::AccelFitConfig fc;
  fc.gRef = cfg_.gRef;

  if (verify_[i]) {
    const ActiveCalibration &a = active_[1];
    double worst = 0;
    for (size_t p = 0; p < n; p++) {
      const double d[3] = {poses_.mean(p)[0] - a.offset[0], poses_.mean(p)[1] - a.offset[1],
                           poses_.mean(p)[2] - a.offset[2]};
      double v[3];
      mat3Vec(a.M, d, v);
      const double e = fabs(vec3Norm(v) - cfg_.gRef);
      if (e > worst)
        worst = e;
    }
    const bool pass = a.valid && n >= cfg_.verifyAccelMinPoses && worst <= cfg_.verifyAccelTolerance;
    verifyPass_[i] = pass;
    result_[i] = pass ? CalResult::Pass : (n < cfg_.verifyAccelMinPoses ? CalResult::FailCoverage : CalResult::FailResidual);
    state_[i] = State::Idle;
    logf("Accel VERIFY %s: %u poses, worst | |a_cal| - %.2f | = %.3f m/s^2 (limit %.2f, at least %u poses)",
         pass ? "PASS" : "FAIL", (unsigned)n, cfg_.gRef, worst, cfg_.verifyAccelTolerance,
         (unsigned)cfg_.verifyAccelMinPoses);
    return;
  }

  accelRes_ = DSP::Calib::fitAccel(poses_.means(), n, fc);
  result_[i] = accelRes_.result;
  state_[i] = DSP::Calib::acceptable(accelRes_.result) ? State::Review : State::Failed;
  const Reason accelParams = accelCandidateParams(accelRes_);
  if (state_[i] == State::Review && accelParams != Reason::Ok) {
    state_[i] = State::Failed;
    logf("Accel candidate rejected: parameters not acceptable for arming (%s)", toString(accelParams));
  }
  logf("Accel fit %s: %u poses%s, residual rms %.4f max %.4f m/s^2, offset %.3f %.3f %.3f m/s^2, scale %.3f..%.3f, "
       "checks 0x%03x",
       DSP::Calib::toString(accelRes_.result), (unsigned)accelRes_.nUsed, accelRes_.diagonal ? " (diagonal fit)" : "",
       accelRes_.residualRms, accelRes_.residualMax, accelRes_.offset[0], accelRes_.offset[1], accelRes_.offset[2],
       accelRes_.scaleEig[0], accelRes_.scaleEig[2], (unsigned)accelRes_.checks);
  if (!(accelRes_.checks & DSP::Calib::kAccelCheckCoverage))
    logWeakParams(accelRes_, fc);
  logf("%s", state_[i] == State::Review ? "Accel candidate ready: apply with CalibApply (then Save), or discard"
                                        : "Accel candidate rejected: discard, move to more different poses and repeat");
}

void CalibrationService::runMagFit(int64_t nowNs) {
  (void)nowNs;
  const size_t i = 2;
  if (verify_[i]) {
    const bool enough = magVerifyN_ >= cfg_.verifyMagMinSamples;
    double mean = 0, sd = 0;
    if (magVerifyN_ > 0) {
      mean = magVerifySum_ / magVerifyN_;
      const double var = magVerifySum2_ / magVerifyN_ - mean * mean;
      sd = var > 0 ? sqrt(var) : 0;
    }
    const double spread = mean > 0 ? sd / mean : 1.0;
    const double cov = magVerifyCoverage_.fractionAtLeast(1);
    const bool pass = active_[2].valid && enough && spread < cfg_.verifyMagMaxSpread && cov >= cfg_.verifyMagMinCoverage;
    verifyPass_[i] = pass;
    result_[i] = pass ? CalResult::Pass : (enough ? CalResult::FailResidual : CalResult::FailCoverage);
    state_[i] = State::Idle;
    logf("Mag VERIFY %s: %u samples, |B_cal| %.4f spread %.2f %% (limit %.1f %%), coverage %.0f %% (min %.0f %%)",
         pass ? "PASS" : "FAIL", (unsigned)magVerifyN_, mean, spread * 100, cfg_.verifyMagMaxSpread * 100, cov * 100,
         cfg_.verifyMagMinCoverage * 100);
    return;
  }
  magRes_ = magFit_.evaluate();
  result_[i] = magRes_.result;
  state_[i] = DSP::Calib::acceptable(magRes_.result) ? State::Review : State::Failed;
  const Reason magParams = magCandidateParams(magRes_);
  if (state_[i] == State::Review && magParams != Reason::Ok) {
    state_[i] = State::Failed;
    logf("Mag candidate rejected: parameters not acceptable for arming (%s)", toString(magParams));
  }
  logf("Mag fit %s: %u stored, coverage %.0f %%, residual rms %.2f %% max %.2f %% (inliers), centre %.4f %.4f %.4f, "
       "radius %.4f, soft-iron axis ratio %.3f (%s), rejected %u of %u, checks 0x%03x",
       DSP::Calib::toString(magRes_.result), (unsigned)magRes_.nStored, magRes_.coverage * 100,
       magRes_.residualRms * 100, magRes_.residualMax * 100, magRes_.offset[0], magRes_.offset[1], magRes_.offset[2],
       magRes_.radius, magRes_.axisRatio, magRes_.softIron ? "ellipsoid" : "sphere", (unsigned)magRes_.nRejected,
       (unsigned)magRes_.nStored, (unsigned)magRes_.checks);
  if (magRes_.result != DSP::Calib::CalResult::Pass)
    logf("Mag fit reason: %s", DSP::Calib::magFailReason(magRes_));
  logf("%s", state_[i] == State::Review ? "Mag candidate ready: apply with CalibApply (then Save), or discard"
                                        : "Mag candidate rejected: discard, tumble through more directions and repeat");
}

void CalibrationService::logWeakParams(const DSP::Calib::AccelFitResult &r, const DSP::Calib::AccelFitConfig &fc) {
  if (r.nUsed < fc.minPosesDiag) {
    logf("Accel not well determined: only %u poses, at least %u needed (%u for the full fit)", (unsigned)r.nUsed,
         (unsigned)fc.minPosesDiag, (unsigned)fc.minPosesFull);
    return;
  }
  if (!r.sigmaValid) {
    logf("%s", "Accel not well determined: the poses do not constrain every parameter (singular), add poses in clearly "
               "different orientations");
    return;
  }
  DSP::Calib::AccelWeakParam w[3];
  const size_t cnt = DSP::Calib::accelWeakParams(r, fc, w, 3);
  char txt[110];
  size_t pos = 0;
  txt[0] = 0;
  for (size_t i = 0; i < cnt && pos < sizeof(txt); i++)
    pos += snprintf(txt + pos, sizeof(txt) - pos, "%s%s sigma %.3g (limit %.3g)", i ? ", " : "",
                    DSP::Calib::accelParamName(w[i].idx), w[i].sigma, w[i].limit);
  logf("Accel not well determined: %s", txt);
}

void CalibrationService::logCaptured(uint8_t count) {
  const DSP::Calib::AccelFitConfig fc;
  if (count < fc.minPosesDiag) {
    logf("Accel: pose %u captured (distribution can be judged from %u poses)", (unsigned)count, (unsigned)fc.minPosesDiag);
    return;
  }
  // Same function as the final check: the full fit with its parameter standard errors.
  const DSP::Calib::AccelFitResult r = DSP::Calib::fitAccel(poses_.means(), count, fc);
  DSP::Calib::AccelWeakParam w[2];
  const size_t cnt = DSP::Calib::accelWeakParams(r, fc, w, 2);
  const bool enoughPoses = count >= fc.minPosesFull;
  const bool well = enoughPoses && r.wellDetermined;
  if (cnt == 0) {
    logf("Accel: pose %u captured (well determined: no; poses do not constrain every parameter, add poses in clearly "
         "different orientations)",
         (unsigned)count);
  } else {
    char weak[64];
    size_t pos = 0;
    weak[0] = 0;
    for (size_t i = 0; i < cnt && pos < sizeof(weak); i++)
      pos += snprintf(weak + pos, sizeof(weak) - pos, "%s%s %.3g", i ? ", " : "", DSP::Calib::accelParamName(w[i].idx),
                      w[i].sigma);
    if (well)
      logf("Accel: pose %u captured (well determined: yes; weakest: %s)", (unsigned)count, weak);
    else if (!enoughPoses)
      logf("Accel: pose %u captured (well determined: no, at least %u poses needed; weakest: %s)", (unsigned)count,
           (unsigned)fc.minPosesFull, weak);
    else
      logf("Accel: pose %u captured (well determined: no; weakest: %s; add poses with %s)", (unsigned)count, weak,
           DSP::Calib::accelParamAdvice(w[0].idx));
  }
  if (well && DSP::Calib::acceptable(r.result) && !coverageHintGiven_) {
    coverageHintGiven_ = true;
    logf("Accel: well determined with %u poses, you can stop and fit now", (unsigned)count);
  }
}

void CalibrationService::drainPoseEvents() {
  while (noteCount_ > 0) {
    const PoseNote n = notes_[noteHead_];
    noteHead_ = (noteHead_ + 1) % kNoteCap;
    noteCount_--;
    switch (n.ev) {
    case PoseEvent::Averaging: logf("%s", "Accel: still, measuring pose (hold 2 s)"); break;
    case PoseEvent::Captured: logCaptured(n.count); break;
    case PoseEvent::Duplicate:
      logf("Accel: same orientation as an earlier pose (%.0f deg), not stored, turn it further", (double)n.angleDeg);
      break;
    case PoseEvent::Aborted: logf("%s", "Accel: moved during the measurement, pose dropped"); break;
    case PoseEvent::Full:
      logCaptured(n.count);
      logf("Accel: all %u poses stored, stop and fit", (unsigned)n.count);
      break;
    case PoseEvent::None: break;
    }
  }
}

void CalibrationService::tick(int64_t nowNs) {
  if (gyro_.collecting() && gyro_.poll(nowNs))
    finishGyroRun(nowNs);

  drainPoseEvents();

  if (state_[1] == State::Collecting && nowNs - startNs_[1] >= cfg_.accelMaxNs) {
    logf("%s", "Accel calibration: time limit reached, fitting");
    state_[1] = State::Fitting;
  }
  if (state_[2] == State::Collecting && nowNs - startNs_[2] >= cfg_.magMaxNs) {
    logf("%s", "Mag calibration: time limit reached, fitting");
    state_[2] = State::Fitting;
  }

  if (state_[1] == State::Fitting)
    runAccelFit(nowNs);
  if (state_[2] == State::Fitting)
    runMagFit(nowNs);

  // keep the mag centre (direction thinning, coverage) fresh at 1 Hz
  if (state_[2] == State::Collecting && !verify_[2] && nowNs - lastEstimateNs_ >= kUpdateEstimateNs) {
    lastEstimateNs_ = nowNs;
    magFit_.updateEstimate();
  }

  // live progress for the operator
  if ((state_[1] == State::Collecting || state_[2] == State::Collecting || state_[0] == State::Collecting) &&
      nowNs - lastProgressNs_ >= cfg_.progressLogNs) {
    lastProgressNs_ = nowNs;
    if (state_[1] == State::Collecting) {
      logf("Accel: %u poses captured (%s)", (unsigned)poses_.count(), stillNow_ ? "still" : "moving");
    } else if (state_[2] == State::Collecting) {
      if (verify_[2]) {
        logf("Mag verify: %u samples, coverage %.0f %%", (unsigned)magVerifyN_,
             magVerifyCoverage_.fractionAtLeast(1) * 100);
      } else {
        magFit_.updateEstimate();
        logf("Mag: %u samples, %u stored, coverage %.0f %% (need 75 %%)", (unsigned)magFit_.accumulatedCount(),
             (unsigned)magFit_.storedCount(), magFit_.coverage().fractionAtLeast(2) * 100);
      }
    } else {
      const auto &b = gyro_.lastBlock();
      if (b.valid)
        logf("Gyro: %.0f %% of the still window, last 1 s block %s: gyro std %.3f p2p %.3f deg/s, accel std %.3f m/s^2",
             gyro_.progress() * 100, b.accepted ? "ok" : "REJECTED (motion)", b.gyroStd * kRadToDeg,
             b.gyroP2P * kRadToDeg, b.accelAxisStd);
      else
        logf("Gyro: %.0f %% of the still window", gyro_.progress() * 100);
    }
  }
}

// ---- legacy-key conversion ----

bool accelCandidateToLegacy(const DSP::Calib::AccelFitResult &fit, const double T[9], double offsetOut[3],
                            double COut[9]) {
  double Ti[9], TA[9];
  if (!mat3Inverse(T, Ti))
    return false;
  mat3Mul(T, fit.A, TA);
  mat3Mul(TA, Ti, COut); // C = T A T^-1
  for (int i = 0; i < 3; i++)
    offsetOut[i] = fit.offset[i];
  return true;
}

Reason accelCandidateParams(const DSP::Calib::AccelFitResult &fit) {
  StoredCalibration st;
  st.keyPresent = true;
  for (int i = 0; i < 3; i++)
    st.offset[i] = fit.offset[i];
  for (int i = 0; i < 9; i++)
    st.C[i] = fit.A[i]; // T = identity: C = T A T^-1 = A
  return checkAccelParams(st);
}

Reason magCandidateParams(const DSP::Calib::MagEllipsoidResult &fit) {
  StoredCalibration st;
  st.keyPresent = true;
  for (int i = 0; i < 3; i++)
    st.offset[i] = fit.offset[i];
  for (int i = 0; i < 9; i++)
    st.C[i] = fit.A[i]; // T = identity: C = T A T^-1 = A
  return checkMagParams(st);
}

bool magCandidateToLegacy(const DSP::Calib::MagEllipsoidResult &fit, const double T[9], double offsetOut[3],
                          double COut[9]) {
  double Ti[9], TA[9];
  if (!mat3Inverse(T, Ti))
    return false;
  mat3Mul(T, fit.A, TA);
  mat3Mul(TA, Ti, COut); // C = T A T^-1
  for (int i = 0; i < 3; i++)
    offsetOut[i] = fit.offset[i];
  return true;
}

} // namespace Calibration
} // namespace VCTR
