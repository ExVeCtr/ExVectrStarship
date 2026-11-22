#ifndef CALIBRATION_SERVICE_HPP
#define CALIBRATION_SERVICE_HPP

/**
 * CalibrationService: the session engine of the sensor calibration (workspace docs/calibration-design.md sections
 * 3.2 and 6.1, step C2). Pure and host-testable (test/test_calibration_service): no Arduino, no ExVectrCore, no
 * globals; time is injected as int64 ns, raw samples are fed by the caller, and every decision lives here or in
 * calibration_validity.hpp. The Arduino side (src/calibration/calibration_task.hpp) feeds raw driver samples and calls
 * tick(); the vehicle task (main.cpp) applies results to the transforms and persists them.
 *
 * Per sensor state (Gyro, Accel, Mag): Idle -> Collecting -> Fitting -> Review | Failed; Review -> Applied (markApplied)
 * -> Saved (markSaved); discard() returns Review/Failed/Collecting to Idle. A candidate is never applied by the
 * service. Only one sensor collects at a time.
 *
 * - Accel: the 12-pose magnitude fit (fitAccel). Poses are captured automatically (StillnessDetector with the active
 *   gyro bias, PoseCollector: still 1.5 s, averaged over 2 s, new if > 20 deg from every earlier pose). Stop (or the
 *   15 min timeout) fits.
 * - Mag: hard + soft iron ellipsoid fit (MagEllipsoidFit, decimated to 10 Hz; ServiceConfig::magSoftIron false gives the
 *   former sphere fit). Stop (or the 5 min timeout) evaluates.
 * - Gyro: the stationary bias run (GyroBiasCollector). Two uses: the AUTOMATIC run (startGyroRun, zeroing windows and
 *   the countdown; the result is reported through takeGyroRunDone and applied by the vehicle task) and a manual
 *   session (start(Gyro), result goes to Review).
 * - Verify mode (start(.., verify = true)): the same collection, but the result checks the ACTIVE calibration (set with
 *   setActive) and the session ends in Idle; no candidate.
 *
 * Samples are in the raw sensor frame: gyro rad/s, accel m/s^2, mag driver gauss, temperature degC.
 */

#include <stddef.h>
#include <stdint.h>

#include "ExVectrDSP/calibration/accel_ellipsoid_fit.hpp"
#include "ExVectrDSP/calibration/gyro_bias_collector.hpp"
#include "ExVectrDSP/calibration/mag_ellipsoid_fit.hpp"
#include "ExVectrDSP/calibration/pose_collector.hpp"
#include "ExVectrDSP/calibration/stillness_detector.hpp"
#include "calibration/calibration_validity.hpp"

namespace VCTR {
namespace Calibration {

enum class Sensor : uint8_t { Gyro = 0, Accel = 1, Mag = 2 };
enum class State : uint8_t { Idle = 0, Collecting, Fitting, Review, Failed, Applied, Saved };
enum class StartResult : uint8_t { Started = 0, Busy };

const char *toString(Sensor s);
const char *toString(State s);

/// The calibration a sensor applies now: cal = M (raw - offset), M = C * T (row-major).
struct ActiveCalibration {
  bool valid = false;
  double offset[3] = {0, 0, 0};
  double M[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

struct ServiceConfig {
  int64_t accelMaxNs = 15LL * 60 * 1000000000LL;   ///< auto stop and fit
  int64_t magMaxNs = 5LL * 60 * 1000000000LL;
  int64_t gyroManualDeadlineNs = 60LL * 1000000000LL; ///< a manual gyro session gives up (Motion or Timeout)
  int64_t magDecimateNs = 100000000LL;             ///< 10 Hz
  int64_t progressLogNs = 2000000000LL;
  double verifyAccelTolerance = 0.05;              ///< |a_cal| = g +- this in every pose, m/s^2
  size_t verifyAccelMinPoses = 3;
  double verifyGyroTolerance = 0.1 * 3.14159265358979 / 180.0; ///< |mean - active bias| per axis, rad/s
  size_t verifyMagMinSamples = 200;
  double verifyMagMaxSpread = 0.03;                ///< std/mean of |B_cal| in a verify tumble
  bool magSoftIron = true;                         ///< false: sphere fit (A = s I), to compare on the same data
  double verifyMagMinCoverage = 0.6;               ///< fraction of bins with at least one sample
  double gRef = 9.81;
};

using LogSink = void (*)(void *ctx, const char *line);

/// An automatic gyro run that finished (success or not). The vehicle task applies it.
struct GyroRunDone {
  DSP::Calib::GyroBiasResult result;
};

class CalibrationService {
public:
  static constexpr size_t kMagBufferCapacity = 600;

  /// magBuffer: storage of kMagBufferCapacity direction-thinned mag samples (the vehicle puts it in DMAMEM).
  CalibrationService(float (*magBuffer)[3], size_t magCapacity, const ServiceConfig &cfg = ServiceConfig());

  void setLogSink(LogSink sink, void *ctx) {
    sink_ = sink;
    sinkCtx_ = ctx;
  }

  /// The applied calibration of a sensor (verify checks it; for Accel and Mag the stillness uses the gyro bias).
  void setActive(Sensor s, const double offset[3], const double M[9]);

  /// Whether the active gyro offset was measured in this power cycle (a successful gyro run). Only a trusted bias is used
  /// for the pose stillness check |w - bias|; otherwise (a stale stored bias, a failed boot run) stillness uses the
  /// bias-free gyro std and peak-to-peak limits of the gyro run, so poses can still be captured.
  void setGyroBiasTrusted(bool trusted) { gyroBiasTrusted_ = trusted; }

  // ---- Raw samples (cheap; the callers skip them when the matching wants*() is false) ----
  bool wantsGyro() const { return gyro_.collecting() || accelSession(); }
  bool wantsAccel() const { return gyro_.collecting() || accelSession(); }
  bool wantsMag() const { return gyro_.collecting() || magSession(); }
  bool wantsTemperature() const { return gyro_.collecting(); }
  void addGyro(int64_t tNs, const double w[3]);
  void addAccel(int64_t tNs, const double a[3]);
  void addMag(int64_t tNs, const double m[3]);
  void addTemperature(int64_t tNs, double tempC);

  // ---- Sessions ----
  /// Starts collecting. Busy if any sensor collects or fits. An automatic gyro run in progress is cancelled.
  StartResult start(Sensor s, bool verify, int64_t nowNs);
  /// Ends collection and queues the fit (done in the next tick). False if the sensor is not collecting.
  bool stop(Sensor s, int64_t nowNs);
  /// Drops a candidate or a collecting session. The active calibration is untouched.
  void discard(Sensor s);
  void markApplied(Sensor s);
  /// After the memory image was written to EEPROM and verified: Applied -> Saved.
  void markSaved();

  // ---- Automatic gyro run (zeroing window, countdown) ----
  /// Starts (or restarts) it with a deadline. Ignored (false) while a session collects. A finished run that was not
  /// taken yet stays pending (it is neither dropped nor overwritten by the start).
  /// `windowNs` > 0: still-window length of this run (DSP::Calib::kGyroWindowCountdownNs for the go countdown).
  bool startGyroRun(int64_t nowNs, int64_t deadlineNs, int64_t windowNs = 0);
  /// Configuration of the current/last gyro run (window length of that run), for the failure log.
  const DSP::Calib::GyroBiasConfig &gyroRunConfig() const { return gyro_.config(); }
  void cancelGyroRun();
  bool gyroRunActive() const { return gyroAuto_ && gyro_.collecting(); }
  /// True once per finished automatic run.
  bool takeGyroRunDone(GyroRunDone &out);

  /// Time-driven work: timeouts, fits, the gyro collector, progress and pose log lines.
  void tick(int64_t nowNs);

  // ---- State and results ----
  State state(Sensor s) const { return state_[idx(s)]; }
  bool collecting(Sensor s) const { return state_[idx(s)] == State::Collecting; }
  bool anyBusy() const;     ///< any sensor Collecting or Fitting
  bool verifyMode(Sensor s) const { return verify_[idx(s)]; }
  DSP::Calib::CalResult result(Sensor s) const { return result_[idx(s)]; }
  /// A candidate with Pass or Warn is waiting (D6: only those may be applied) and, for accel and mag, its legacy-key
  /// parameters also pass checkAccelParams / checkMagParams (so an applied and saved candidate can never be rejected
  /// by the arming check as "out of range").
  bool applicable(Sensor s) const;
  const DSP::Calib::AccelFitResult &accelResult() const { return accelRes_; }
  const DSP::Calib::MagEllipsoidResult &magResult() const { return magRes_; }
  const DSP::Calib::GyroBiasResult &gyroResult() const { return gyroRes_; }
  size_t poseCount() const { return poses_.count(); }
  const DSP::Calib::PoseCollector &poses() const { return poses_; }

private:
  static size_t idx(Sensor s) { return static_cast<size_t>(s); }
  bool accelSession() const { return state_[1] == State::Collecting; }
  bool magSession() const { return state_[2] == State::Collecting; }
  bool gyroSession() const { return state_[0] == State::Collecting; }

  void logf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
  void runAccelFit(int64_t nowNs);
  void runMagFit(int64_t nowNs);
  void finishGyroRun(int64_t nowNs);
  void drainPoseEvents();
  void logCaptured(uint8_t count);
  void logWeakParams(const DSP::Calib::AccelFitResult &r, const DSP::Calib::AccelFitConfig &fc);

  ServiceConfig cfg_;
  LogSink sink_ = nullptr;
  void *sinkCtx_ = nullptr;

  State state_[3] = {State::Idle, State::Idle, State::Idle};
  DSP::Calib::CalResult result_[3] = {DSP::Calib::CalResult::None, DSP::Calib::CalResult::None,
                                      DSP::Calib::CalResult::None};
  bool verify_[3] = {false, false, false};
  bool verifyPass_[3] = {false, false, false};
  ActiveCalibration active_[3];
  int64_t startNs_[3] = {0, 0, 0};
  int64_t lastProgressNs_ = 0;
  int64_t lastEstimateNs_ = 0;

  DSP::Calib::StillnessDetector still_;
  DSP::Calib::PoseCollector poses_;
  DSP::Calib::MagEllipsoidFit magFit_;
  DSP::Calib::GyroBiasCollector gyro_;
  bool gyroBiasTrusted_ = false;
  bool gyroAuto_ = false;
  bool gyroDone_ = false;
  DSP::Calib::AccelFitResult accelRes_;
  DSP::Calib::MagEllipsoidResult magRes_;
  DSP::Calib::GyroBiasResult gyroRes_;     ///< the manual session's result (the candidate)
  DSP::Calib::GyroBiasResult autoRes_;     ///< the finished automatic run, until taken

  // mag decimation and verify statistics
  int64_t lastMagNs_ = 0;
  bool haveMagSample_ = false;
  uint32_t magVerifyN_ = 0;
  double magVerifySum_ = 0, magVerifySum2_ = 0;
  DSP::Calib::SphereCoverage magVerifyCoverage_;
  bool stillNow_ = false;
  bool coverageHintGiven_ = false; ///< the 'well determined, you can stop' line was logged in this session

  // pose events recorded in the sample path, logged in tick()
  struct PoseNote {
    DSP::Calib::PoseEvent ev;
    uint8_t count;
    float angleDeg;
  };
  static constexpr uint8_t kNoteCap = 8;
  PoseNote notes_[kNoteCap];
  uint8_t noteHead_ = 0, noteCount_ = 0;
};

/// Legacy-key form of a candidate (design section 3.6 / owner decision: legacy keys, no new record).
/// Accel: cal_body = T A (raw - b), and the pipeline computes M = C T, so C = T A T^-1; offset = b.
/// Returns false if T is singular.
bool accelCandidateToLegacy(const DSP::Calib::AccelFitResult &fit, const double T[9], double offsetOut[3],
                            double COut[9]);
/// Mag: calibrated = T A (raw - b) like the accelerometer, so C = T A T^-1 (C = s I for a sphere); offset = b.
/// Returns false if T is singular.
bool magCandidateToLegacy(const DSP::Calib::MagEllipsoidResult &fit, const double T[9], double offsetOut[3],
                          double COut[9]);

/// The arming validity range check (checkAccelParams / checkMagParams) applied to a candidate. The accel check is
/// independent of the mounting matrix T (A = T^-1 C T = fit.A), so T = identity is used.
Reason accelCandidateParams(const DSP::Calib::AccelFitResult &fit);
Reason magCandidateParams(const DSP::Calib::MagEllipsoidResult &fit);

} // namespace Calibration
} // namespace VCTR

#endif
