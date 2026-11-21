#ifndef CALIBRATION_TASK_HPP
#define CALIBRATION_TASK_HPP

/**
 * Arduino/ExVectrCore side of the calibration service (workspace docs/calibration-design.md step C2): feeds the raw
 * driver topics (MPU9250 gyro, accel and thermometer, QMC5883 mag) into the CalibrationService ONLY while it wants them
 * (a session or a gyro run is collecting), and runs its time-driven work at 10 Hz in a low-priority task, outside the
 * safety task. The callbacks run synchronously in the publisher's task (the MPU9250 driver publishes from its task
 * thread, not from the interrupt), so they are kept to a conversion and a call.
 *
 * Header-only and platform-bound (ExVectrCore): the host tests cover CalibrationService itself.
 */

#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_source.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrDSP/value_covariance.hpp"

#include "calibration/calibration_service.hpp"

namespace VCTR {
namespace Calibration {

class CalibrationTask : public Core::Task_Periodic {
public:
  using Vec3Item = Core::Timestamped<DSP::ValueCov<float, 3>>;
  using Vec1Item = Core::Timestamped<DSP::ValueCov<float, 1>>;

  explicit CalibrationTask(CalibrationService &service) : Task_Periodic("Calibration", 100 * Core::MILLISECONDS), svc_(service) {
    Core::getSystemScheduler().addTask(*this);
    svc_.setLogSink(&CalibrationTask::logLine, nullptr);
    gyroSubr_.setCallback(this, &CalibrationTask::onGyro);
    accSubr_.setCallback(this, &CalibrationTask::onAccel);
    magSubr_.setCallback(this, &CalibrationTask::onMag);
    thermoSubr_.setCallback(this, &CalibrationTask::onThermo);
  }

  /// Subscribes to the RAW driver topics (before the calibration transforms).
  void connect(Core::Topic<Vec3Item> &gyro, Core::Topic<Vec3Item> &accel, Core::Topic<Vec3Item> &mag,
               Core::Topic<Vec1Item> &thermo) {
    gyroSubr_.subscribe(gyro);
    accSubr_.subscribe(accel);
    magSubr_.subscribe(mag);
    thermoSubr_.subscribe(thermo);
  }

  CalibrationService &service() { return svc_; }

  void taskThread() override { svc_.tick(Core::NowNs()); }

private:
  static void logLine(void *, const char *line) { Core::printM("Calibration: %s\n", line); }

  static void toDouble(const Vec3Item &item, double v[3]) {
    v[0] = item.data.val(0);
    v[1] = item.data.val(1);
    v[2] = item.data.val(2);
  }

  void onGyro(const Vec3Item &item) {
    if (!svc_.wantsGyro())
      return;
    double v[3];
    toDouble(item, v);
    svc_.addGyro(item.timestamp, v);
  }
  void onAccel(const Vec3Item &item) {
    if (!svc_.wantsAccel())
      return;
    double v[3];
    toDouble(item, v);
    svc_.addAccel(item.timestamp, v);
  }
  void onMag(const Vec3Item &item) {
    if (!svc_.wantsMag())
      return;
    double v[3];
    toDouble(item, v);
    svc_.addMag(item.timestamp, v);
  }
  void onThermo(const Vec1Item &item) {
    if (!svc_.wantsTemperature())
      return;
    svc_.addTemperature(item.timestamp, item.data.val(0));
  }

  CalibrationService &svc_;
  Core::Callback_Subscriber<Vec3Item, CalibrationTask> gyroSubr_, accSubr_, magSubr_;
  Core::Callback_Subscriber<Vec1Item, CalibrationTask> thermoSubr_;
};

} // namespace Calibration
} // namespace VCTR

#endif
