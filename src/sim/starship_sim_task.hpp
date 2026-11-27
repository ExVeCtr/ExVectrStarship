#ifndef STARSHIP_SIM_STARSHIP_SIM_TASK_HPP
#define STARSHIP_SIM_STARSHIP_SIM_TASK_HPP

/**
 * The vehicle's simulation task (replaces `DSP::BodySimulator`, step S4 of docs/project/simulation-architecture.md).
 * Simulation mode only: it replaces the estimators' outputs with the state of the `StarshipPlant` and publishes the
 * same topic types as the estimators. External behaviour is identical to the legacy `BodySimulator` (same topics,
 * same API used by main.cpp, 10 ms period, wall-clock integration with the 100 ms gap rule). The logic is in
 * `StarshipSimAdapter` (host tested); this class only does the topic plumbing. In flash (D5).
 *
 * Inputs: the gated simulated thrust topic [X, Y, Z, T] of `StarshipTVC` (zero while the motor gate is closed) and the
 * true flap angles of `Safety::TrueFlapModel` (including the sim-only faults). Neither gate nor faults are in here.
 */

#include "ExVectrControl/starship/control_attitude_flaps.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrMath.hpp"
#include "starship_sim_adapter.hpp"

namespace VCTR {
namespace ShipSim {

class StarshipSimTask : public Core::Task_Periodic {
public:
  static constexpr int64_t kMaxGapNs = StarshipSimAdapter::kMaxGapNs;

  SIM_COLD StarshipSimTask(float mass, Math::Vector<float, 3> inertiaTensor, float tvcThrustLimit_N,
                           float tvcAngleLimit_Rad, int64_t simulationInterval = 0.01 * Core::SECONDS);

  SIM_COLD ~StarshipSimTask() override; // never called, but its code would otherwise sit in ITCM

  /// Subscribes to the TVC actual thrust topic [X, Y, Z, T]. Only the z of `tvcPosition` is used.
  SIM_COLD void setTVCInputTopic(Core::Topic<Math::Vector<float, 4>> &tvcTopic, const Math::Vector<float, 3> &tvcPosition,
                                 float tvcAngleLimit, float tvcForceLimit);
  void setFlapSettingInputTopic(Core::Topic<CTRL::ControlAttitudeFlapSetting> &flapTopic) {
    flapSettingsSubr_.subscribe(flapTopic);
  }

  /// One simulation run: take the inputs, integrate the elapsed time, apply zeroing mode.
  SIM_COLD void update();

  void enableZeroingMode(bool enable) { adapter_.enableZeroingMode(enable); }
  void resetToStartPose() { adapter_.resetToStartPose(); }
  void enableTVC(bool enable) { adapter_.enableTVC(enable); }
  void enableFlaps(bool enable) { adapter_.enableFlaps(enable); }

  void setWind(float speed_mps, float direction_rad) { adapter_.setWind(speed_mps, direction_rad); }
  void setGust(float strength_mps, float period_s, uint32_t seed) { adapter_.setGust(strength_mps, period_s, seed); }
  void clearWind() { adapter_.clearWind(); }
  float gustStrength() const { return adapter_.gustStrength(); }
  float gustPeriod() const { return adapter_.gustPeriod(); }
  uint32_t gustSeed() const { return adapter_.gustSeed(); }
  float windSpeed() const { return adapter_.windSpeed(); }
  float windDirection() const { return adapter_.windDirection(); }

  const Math::Vector<float, 7> &getAttitudeState() const { return adapter_.getAttitudeState(); }
  const Math::Vector<float, 6> &getPositionState() const { return adapter_.getPositionState(); }

  /// @return attitude [W, Q] (body rates, q body -> world).
  Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &getAttitudeEstTopic() { return attitudeTopic_; }
  /// @return [V, P] in the world frame (NWU).
  Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &getStateEstTopic() { return positionTopic_; }
  /// @return ground [V, P]: vertical velocity m/s, height above the ground m.
  Core::Topic<Core::Timestamped<Math::Vector<float, 2>>> &getGroundTopic() { return groundTopic_; }

private:
  SIM_COLD void taskInit() override;
  SIM_COLD void taskThread() override;

  Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> attitudeTopic_;
  Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> positionTopic_;
  Core::Topic<Core::Timestamped<Math::Vector<float, 2>>> groundTopic_;

  Core::Simple_Subscriber<Math::Vector<float, 4>> tvcSubr_;
  Core::Simple_Subscriber<CTRL::ControlAttitudeFlapSetting> flapSettingsSubr_;

  StarshipSimAdapter adapter_;
};

} // namespace ShipSim
} // namespace VCTR

#endif
