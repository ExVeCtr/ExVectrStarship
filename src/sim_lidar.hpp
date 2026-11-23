#ifndef SIM_LIDAR_HPP
#define SIM_LIDAR_HPP

/**
 * Simulated ground distance sensor (downward VL53L0X). Simulation only replaces the SOURCE of the sensor topic: the body
 * simulator's height above flat ground (at the position reference) is republished as the measured distance ONLY below
 * kSimLidarMaxRangeM. At or above it nothing is published, the topic goes stale and the consumers (landing detector,
 * telemetry) treat it as no reading, exactly as for the real sensor out of its useful range. No noise (the other
 * simulated sensors have none). Test: test/test_landing_detector.
 */

#include <math.h>

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrMath/matrix_vector.hpp"

namespace VCTR {

constexpr float kSimLidarMaxRangeM = 1.0f;

/// True and the reading in outM when the simulated sensor has a valid reading at this height.
inline bool simLidarReading(float heightM, float &outM) {
  if (!isfinite(heightM) || heightM >= kSimLidarMaxRangeM)
    return false;
  outM = heightM;
  return true;
}

/// Subscribes to the body simulator ground topic ([vz, height]) and publishes [vz, distance] only when valid.
class SimLidar : public Core::Subscriber<Core::Timestamped<Math::Vector<float, 2>>> {
public:
  Core::Topic<Core::Timestamped<Math::Vector<float, 2>>> &topic() { return out_; }

  void receive(const Core::Timestamped<Math::Vector<float, 2>> &item,
               const Core::Topic<Core::Timestamped<Math::Vector<float, 2>>> *) override {
    float d;
    if (!simLidarReading(item.data(1), d))
      return;
    Core::Timestamped<Math::Vector<float, 2>> o = item;
    o.data(1) = d;
    out_.publish(o);
  }

private:
  Core::Topic<Core::Timestamped<Math::Vector<float, 2>>> out_;
};

} // namespace VCTR

#endif
