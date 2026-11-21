#include "calibration/calibration_validity.hpp"

#include <math.h>
#include <stdio.h>

#include "calibration/calibration_math.hpp"

namespace VCTR {
namespace Calibration {

CommandResult checkCommand(const CommandContext &ctx) {
  if (ctx.simulation)
    return CommandResult::Simulation;
  if (!ctx.initialised)
    return CommandResult::NotInitialised;
  if (ctx.armed)
    return CommandResult::Armed;
  if (ctx.motorsEnabled)
    return CommandResult::MotorsEnabled;
  if (!ctx.modeAllowed || !ctx.missionIdleOrFinished)
    return CommandResult::NotOnGround;
  return CommandResult::Ok;
}

const char *toString(CommandResult r) {
  switch (r) {
  case CommandResult::Ok: return "ok";
  case CommandResult::NotInitialised: return "not initialised";
  case CommandResult::Armed: return "armed";
  case CommandResult::MotorsEnabled: return "motors enabled";
  case CommandResult::NotOnGround: return "not on ground";
  case CommandResult::Simulation: return "simulation mode";
  }
  return "?";
}

const char *toString(Reason r) {
  switch (r) {
  case Reason::Ok: return "ok";
  case Reason::NotCalibrated: return "not calibrated since power-up";
  case Reason::RunFailed: return "last run failed";
  case Reason::KeyMissing: return "not found in memory";
  case Reason::Default: return "default (never calibrated)";
  case Reason::OutOfRange: return "parameters out of range";
  case Reason::LiveCheck: return "still-window check failed";
  case Reason::NoLiveData: return "no still-window data";
  }
  return "?";
}

static bool isIdentityDefault(const StoredCalibration &s) {
  for (int i = 0; i < 3; i++)
    if (fabs(s.offset[i]) > kDefaultEps)
      return false;
  for (int i = 0; i < 9; i++) {
    const double e = (i % 4 == 0) ? 1.0 : 0.0;
    if (fabs(s.C[i] - e) > kDefaultEps)
      return false;
  }
  return true;
}

static bool finite(const StoredCalibration &s) {
  for (int i = 0; i < 3; i++)
    if (!isfinite(s.offset[i]))
      return false;
  for (int i = 0; i < 9; i++)
    if (!isfinite(s.C[i]) || !isfinite(s.T[i]))
      return false;
  return true;
}

double calibratedNorm(const StoredCalibration &s, const double raw[3]) {
  double M[9], d[3] = {raw[0] - s.offset[0], raw[1] - s.offset[1], raw[2] - s.offset[2]}, v[3];
  mat3Mul(s.C, s.T, M);
  mat3Vec(M, d, v);
  return vec3Norm(v);
}

Reason checkAccelParams(const StoredCalibration &s) {
  if (!s.keyPresent)
    return Reason::KeyMissing;
  if (!finite(s))
    return Reason::OutOfRange;
  if (isIdentityDefault(s))
    return Reason::Default;
  for (int i = 0; i < 3; i++)
    if (fabs(s.offset[i]) > kAccelOffsetMax)
      return Reason::OutOfRange;
  // Sensor-frame matrix A = T^-1 C T (M = C T = T A): scale on the diagonal, little coupling off it.
  double Ti[9], A[9];
  if (!mat3Inverse(s.T, Ti))
    return Reason::OutOfRange;
  mat3Mul(Ti, s.C, A);
  mat3Mul(A, s.T, A);
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) {
      const double a = A[3 * r + c];
      if (r == c ? (a < kAccelDiagMin || a > kAccelDiagMax) : fabs(a) > kAccelOffDiagMax)
        return Reason::OutOfRange;
    }
  return Reason::Ok;
}

Reason checkMagParams(const StoredCalibration &s) {
  if (!s.keyPresent)
    return Reason::KeyMissing;
  if (!finite(s))
    return Reason::OutOfRange;
  if (isIdentityDefault(s))
    return Reason::Default;
  // C = s I (uniform scale to kMagRef): s = kMagRef / R with R the raw field radius.
  const double mean = (s.C[0] + s.C[4] + s.C[8]) / 3.0;
  if (!(mean > 0))
    return Reason::OutOfRange;
  for (int i = 0; i < 9; i++) {
    if (i % 4 == 0) {
      if (fabs(s.C[i] - mean) > kMagMatrixDiagSpread * mean)
        return Reason::OutOfRange;
    } else if (fabs(s.C[i]) > kMagMatrixOffDiagRatio * mean) {
      return Reason::OutOfRange;
    }
  }
  const double R = kMagRef / mean;
  if (R < kMagRadiusMin || R > kMagRadiusMax)
    return Reason::OutOfRange;
  if (vec3Norm(s.offset) > kMagOffsetRatioMax * R)
    return Reason::OutOfRange;
  return Reason::Ok;
}

Validity calibrationsValid(const ValidityInput &in) {
  Validity v;
  switch (in.gyro) {
  case GyroRunState::Ok: v.gyro = Reason::Ok; break;
  case GyroRunState::Failed: v.gyro = Reason::RunFailed; break;
  default: v.gyro = Reason::NotCalibrated; break;
  }

  v.accel = checkAccelParams(in.accel);
  if (v.accel == Reason::Ok) {
    if (in.live.haveAccel) {
      if (fabs(calibratedNorm(in.accel, in.live.accelRaw) - kGravity) > kAccelLiveTolerance)
        v.accel = Reason::LiveCheck;
    } else if (v.gyro == Reason::Ok) {
      v.accel = Reason::NoLiveData; // a successful run always supplies samples; if not, do not trust the values
    }
    // else: the gyro run is still outstanding and already blocks; the live check follows with its data
  }

  v.mag = checkMagParams(in.mag);
  if (v.mag == Reason::Ok) {
    if (in.live.haveMag) {
      if (fabs(calibratedNorm(in.mag, in.live.magRaw) - kMagRef) > kMagLiveToleranceRatio * kMagRef)
        v.mag = Reason::LiveCheck;
    } else if (v.gyro == Reason::Ok) {
      v.mag = Reason::NoLiveData;
    }
  }
  return v;
}

size_t describe(const Validity &v, char *buf, size_t n) {
  if (n == 0)
    return 0;
  const int w = snprintf(buf, n, "gyro: %s, accel: %s, mag: %s", toString(v.gyro), toString(v.accel), toString(v.mag));
  if (w < 0) {
    buf[0] = '\0';
    return 0;
  }
  return static_cast<size_t>(w) < n ? static_cast<size_t>(w) : n - 1;
}

} // namespace Calibration
} // namespace VCTR
