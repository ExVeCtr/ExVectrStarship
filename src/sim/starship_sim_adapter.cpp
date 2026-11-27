#include "starship_sim_adapter.hpp"

#include <math.h>

namespace VCTR {
namespace ShipSim {

namespace {
StarshipPlantParams vehicleParams(float mass, const Math::Vector<float, 3> &inertia, float thrustLimit,
                                  float angleLimit) {
  StarshipPlantParams p;
  p.mass_kg = mass;
  p.inertiaXX = inertia(0);
  p.inertiaYY = inertia(1);
  p.inertiaZZ = inertia(2);
  p.tvc.thrustLimit_N = thrustLimit;
  p.tvc.angleLimit_rad = angleLimit;
  // On the vehicle the flap driver and the TVC driver already publish rate-limited "actual" values.
  p.applyActuatorModels = false;
  return p;
}
} // namespace

StarshipSimAdapter::StarshipSimAdapter(float mass, Math::Vector<float, 3> inertiaTensor, float tvcThrustLimit_N,
                                       float tvcAngleLimit_Rad)
    : cfg_(Plant::configure(vehicleParams(mass, inertiaTensor, tvcThrustLimit_N, tvcAngleLimit_Rad))), sim_(cfg_) {
  tvcForce_ = {0, 0, 0}; // no thrust until the first TVC message
  for (int i = 0; i < 4; i++)
    flapIn_[i] = NAN;
  attitudeState_ = {0, 0, 0, 1, 0, 0, 0};
  positionState_ = {0, 0, 0, 0, 0, 0};
}

void StarshipSimAdapter::configureTvc(float tvcAngleLimit, float tvcForceLimit, float tvcFromCg_m) {
  cfg_.p.tvc.angleLimit_rad = tvcAngleLimit;
  cfg_.p.tvc.thrustLimit_N = tvcForceLimit;
  cfg_.p.tvc.fromCg_m = tvcFromCg_m;
}

StarshipSimAdapter::Plant::Input StarshipSimAdapter::makeInput() const {
  Plant::Input u;
  u.thrustBody = tvcForce_;
  u.rollTorque = tvcRollTorque_;
  u.thrustEnabled = tvcEnabled_;
  for (int i = 0; i < 4; i++)
    u.flapAngle[i] = flapIn_[i];
  return u;
}

void StarshipSimAdapter::resetToStartPose() {
  Plant::resetPose(sim_.state());
  sim_.clearTimeCarry();
  attitudeState_ = sim_.state().body.att;
  positionState_ = sim_.state().body.pos;
}

void StarshipSimAdapter::update(int64_t nowNs) {
  const int64_t elapsedNs = nowNs - stateTimestamp_;
  stateTimestamp_ = nowNs;

  if (elapsedNs > 0 && elapsedNs <= kMaxGapNs) {
    sim_.advance(float(double(elapsedNs) / 1e9), makeInput());
  } else {
    sim_.clearTimeCarry(); // first run after a pause or restart: continue without a catch-up step
  }

  if (zeroingMode_)
    Plant::resetPose(sim_.state());

  attitudeState_ = sim_.state().body.att;
  positionState_ = sim_.state().body.pos;
}

void StarshipSimAdapter::setWind(float speed_mps, float direction_rad) { Plant::setWind(cfg_, speed_mps, direction_rad); }
void StarshipSimAdapter::setGust(float strength_mps, float period_s, uint32_t seed) {
  Plant::setGust(cfg_, strength_mps, period_s, seed);
}
void StarshipSimAdapter::clearWind() { Plant::clearWind(cfg_); }

} // namespace ShipSim
} // namespace VCTR
