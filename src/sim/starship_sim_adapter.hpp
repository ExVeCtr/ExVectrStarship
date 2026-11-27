#ifndef STARSHIP_SIM_STARSHIP_SIM_ADAPTER_HPP
#define STARSHIP_SIM_STARSHIP_SIM_ADAPTER_HPP

/**
 * Vehicle-side logic of the simulator, without any scheduler, topic or clock dependency (so it runs on the host):
 * the part of the legacy `DSP::BodySimulator` that is not topic plumbing. `StarshipSimTask` (starship_sim_task.hpp) is
 * the thin Task_Periodic shell around it. Behaviour is identical to `BodySimulator::update()` (tested bit for bit
 * against a frozen copy in test/test_sim_task):
 *  - the thrust (last RECEIVED value, a non-finite one is treated as zero thrust by the plant), the roll torque and the
 *    TVC enable are fed to the plant on every step;
 *  - flap angles: the last finite value per channel is held here (the plant takes its input inside `step`, and two
 *    set calls between steps would keep only the last one; the legacy model kept the last valid one);
 *  - the wall-clock time since the previous run is integrated in fixed plant steps (carry, at most maxAdvance), a gap
 *    above kMaxGapNs, or none, is not integrated and clears the carry;
 *  - zeroing mode holds the start pose after the integration of every run.
 *
 * Flight safety: simulation mode only (the plant is the world, not an estimator). All of this code is in flash
 * (SIM_COLD, docs/project/simulation-architecture.md D5).
 */

#include <math.h>
#include <stdint.h>

#include "ExVectrDynamics/sim.hpp"
#include "ExVectrMath.hpp"
#include "sim_placement.hpp"
#include "starship_plant.hpp"

namespace VCTR {
namespace ShipSim {

class StarshipSimAdapter {
public:
  using Plant = StarshipPlant;

  /// Gap above which the elapsed wall-clock time is not integrated (pause, restart).
  static constexpr int64_t kMaxGapNs = 100 * 1000 * 1000;

  /// Same parameters as `DSP::BodySimulator`: mass, diagonal inertia, TVC thrust and cone limits; everything else is
  /// the default of StarshipPlantParams. The plant applies no actuator models (the drivers publish actual values).
  SIM_COLD StarshipSimAdapter(float mass, Math::Vector<float, 3> inertiaTensor, float tvcThrustLimit_N,
                              float tvcAngleLimit_Rad);
  StarshipSimAdapter(const StarshipSimAdapter &) = delete;
  StarshipSimAdapter &operator=(const StarshipSimAdapter &) = delete;

  /// Thrust limit, cone limit and application point (z on the axis) of the TVC input.
  SIM_COLD void configureTvc(float tvcAngleLimit, float tvcForceLimit, float tvcFromCg_m);

  /// A new item of the TVC "actual thrust" topic: [X, Y, Z, T] (body N, roll torque N m). Held until the next one.
  void setTvcInput(const Math::Vector<float, 4> &in) {
    tvcForce_ = {in(0), in(1), in(2)};
    tvcRollTorque_ = in(3);
  }
  /// A new item of the true flap angle topic (rad). A non-finite angle keeps the last finite one of that channel.
  void setFlapInput(float tl, float tr, float bl, float br) {
    const float in[4] = {tl, tr, bl, br};
    for (int i = 0; i < 4; i++)
      if (isfinite(in[i]))
        flapIn_[i] = in[i];
  }

  /// First run after the task starts: the elapsed time is measured from here.
  void start(int64_t nowNs) { stateTimestamp_ = nowNs; }
  /// One run at wall-clock time `nowNs` (the inputs are those set before): integrate, then apply zeroing mode.
  SIM_COLD void update(int64_t nowNs);

  /// Timestamp of the published state (time of the last run).
  int64_t stateTimestamp() const { return stateTimestamp_; }

  void enableZeroingMode(bool enable) { zeroingMode_ = enable; }
  void enableTVC(bool enable) { tvcEnabled_ = enable; }
  void enableFlaps(bool enable) { flapsEnabled_ = enable; }
  bool flapsEnabled() const { return flapsEnabled_; }

  /// Upright at the origin, at rest; time and actuators continue; the time carry is cleared.
  SIM_COLD void resetToStartPose();

  SIM_COLD void setWind(float speed_mps, float direction_rad);
  SIM_COLD void setGust(float strength_mps, float period_s, uint32_t seed);
  SIM_COLD void clearWind();
  float gustStrength() const { return cfg_.p.wind.gustStrength_mps; }
  float gustPeriod() const { return cfg_.p.wind.gustPeriod_s; }
  uint32_t gustSeed() const { return cfg_.p.wind.gustSeed; }
  float windSpeed() const { return cfg_.p.wind.speed_mps; }
  float windDirection() const { return cfg_.p.wind.direction_rad; }

  const Math::Vector<float, 7> &getAttitudeState() const { return attitudeState_; }
  const Math::Vector<float, 6> &getPositionState() const { return positionState_; }

  const Dyn::Sim<Plant> &sim() const { return sim_; }
  const Plant::Config &config() const { return cfg_; }

private:
  Plant::Config cfg_;
  Dyn::Sim<Plant> sim_;

  Math::Vector<float, 3> tvcForce_; ///< last thrust vector received, body frame, N
  float tvcRollTorque_ = 0;         ///< last roll torque received, N m
  float flapIn_[4];                 ///< last finite flap angle per channel (NaN until the first: the plant keeps its start value)
  int64_t stateTimestamp_ = 0;
  bool zeroingMode_ = false;
  bool tvcEnabled_ = true;
  bool flapsEnabled_ = false;

  Math::Vector<float, 7> attitudeState_;
  Math::Vector<float, 6> positionState_;

  Plant::Input makeInput() const;
};

} // namespace ShipSim
} // namespace VCTR

#endif
