#ifndef TEST_SUPPORT_PLANT_MODEL_HPP
#define TEST_SUPPORT_PLANT_MODEL_HPP

/**
 * Host harness handle on the Starship plant (S5 of docs/project/simulation-architecture.md): one
 * `Dyn::Sim<ShipSim::StarshipPlantHarness>` with its own `Config`, the held inputs, and the convenience calls the
 * harnesses use (set the inputs, step, advance, read the state and the loads, query the plate geometry).
 *
 * It replaces the deleted `DSP::BodySimModel` in the harnesses (test_sim_aero, test_descent_*, test_kick_sweep,
 * test_sim_flap_fault). It adds NO physics: every call forwards to src/sim (StarshipPlant, starshipAero) or to
 * ExVectrDynamics. Input holding is the one the vehicle adapter uses: thrust (vector, roll torque, enable) is held as
 * given, a flap channel keeps its last finite value. Parameters are `ShipSim::StarshipPlantParams`: change them with
 * `params()` and call `applyParams()` (rebuilds the derived config, keeps the state) before the next step.
 *
 * Header only. Copyable (a copy gets its own config and the same state).
 */

#include <math.h>

#include "ExVectrDynamics/flat_plate.hpp"
#include "ExVectrDynamics/sim.hpp"
#include "ExVectrDynamics/wind.hpp"
#include "sim/starship_plant.hpp"

namespace plant_test {

using VCTR::ShipSim::StarshipPlantHarness;
using VCTR::ShipSim::StarshipPlantParams;

/// Output of one force evaluation (body frame), the same fields as the old `BodySimLoads`.
struct PlantLoads {
  VCTR::Math::Vector<float, 3> aeroForce;     ///< Sum of aerodynamic forces, N.
  VCTR::Math::Vector<float, 3> aeroTorque;    ///< Sum of aerodynamic moments about the CoM, N m.
  VCTR::Math::Vector<float, 3> flapForce[4];  ///< Per flap TL, TR, BL, BR, N.
  VCTR::Math::Vector<float, 3> airVelocity;   ///< Air velocity relative to the CoM, body frame, m/s.
};

class PlantModel {
public:
  using Plant = StarshipPlantHarness;

  explicit PlantModel(const StarshipPlantParams &p = StarshipPlantParams())
      : cfg_(Plant::configure(p)), sim_(cfg_), last_() {
    initInput();
  }
  PlantModel(const PlantModel &o) : cfg_(o.cfg_), sim_(cfg_), u_(o.u_), last_(o.last_) { sim_.restore(o.sim_.snapshot()); }
  PlantModel &operator=(const PlantModel &o) {
    cfg_ = o.cfg_;
    sim_.setConfig(cfg_);
    sim_.restore(o.sim_.snapshot());
    u_ = o.u_;
    last_ = o.last_;
    return *this;
  }

  /// Parameters (change, then call applyParams()).
  StarshipPlantParams &params() { return cfg_.p; }
  const StarshipPlantParams &params() const { return cfg_.p; }
  /// Rebuilds the derived config (mass properties, geometry, areas, gust table) from params(); keeps the state.
  void applyParams() { cfg_ = Plant::configure(cfg_.p); }

  /// Upright at the origin, at rest; actuators, time and the time carry keep their state.
  void resetToStartPose() { Plant::resetPose(sim_.state()); }
  void setAttitudeState(const VCTR::Math::Vector<float, 7> &s) { sim_.state().body.att = s; }
  void setPositionState(const VCTR::Math::Vector<float, 6> &s) { sim_.state().body.pos = s; }

  /// Held thrust input: force vector in the body frame, roll torque about body Z, enable.
  void setThrust(const VCTR::Math::Vector<float, 3> &forceBody, float rollTorque, bool enabled) {
    u_.thrustBody = {forceBody(0), forceBody(1), forceBody(2)};
    u_.rollTorque = rollTorque;
    u_.thrustEnabled = enabled;
  }
  /// Flap angle inputs TL, TR, BL, BR, rad; a non-finite value keeps the last finite one of that channel.
  void setFlapInput(float tl, float tr, float bl, float br) {
    const float in[4] = {tl, tr, bl, br};
    for (int i = 0; i < 4; i++)
      if (isfinite(in[i]))
        u_.flapAngle[i] = in[i];
  }
  void setWind(float speed_mps, float direction_rad) { Plant::setWind(cfg_, speed_mps, direction_rad); }
  void setGust(float strength_mps, float period_s, uint32_t seed) { Plant::setGust(cfg_, strength_mps, period_s, seed); }
  void clearWind() { Plant::clearWind(cfg_); }
  void gustAt(double t_s, float &along, float &across) const {
    VCTR::Dyn::gustAt(cfg_.p.wind, cfg_.d.gust, t_s, along, across);
  }

  /// One fixed step of params().step_s with the held inputs.
  void step() {
    sim_.step(u_);
    last_ = sim_.output();
  }
  /// Fixed steps for `elapsed_s` of real time (carry, bounded catch-up). Returns the number of steps.
  int advance(float elapsed_s) {
    const int n = sim_.advance(elapsed_s, u_);
    if (n > 0)
      last_ = sim_.output();
    return n;
  }
  void clearTimeCarry() { sim_.clearTimeCarry(); }

  const VCTR::Math::Vector<float, 7> &attitudeState() const { return sim_.state().body.att; }
  const VCTR::Math::Vector<float, 6> &positionState() const { return sim_.state().body.pos; }
  /// Modelled flap angles TL, TR, BL, BR after the servo model, rad.
  float flapAngle(int i) const { return sim_.state().flaps.angle[i]; }
  /// Modelled thrust angles [about body X, about body Y] after the TVC slew model, rad.
  float tvcAngleX() const { return sim_.state().tvc.angleX; }
  float tvcAngleY() const { return sim_.state().tvc.angleY; }
  PlantLoads loads() const {
    PlantLoads l;
    l.aeroForce = last_.aero.force;
    l.aeroTorque = last_.aero.torque;
    for (int i = 0; i < 4; i++)
      l.flapForce[i] = last_.flapForce[i];
    l.airVelocity = last_.airVelBody;
    return l;
  }
  /// Simulated time since construction, s.
  double time() const { return sim_.state().t_s; }
  /// World wind vector (NWU, air velocity), m/s.
  VCTR::Math::Vector<float, 3> windWorld() const { return VCTR::Dyn::windAt(cfg_.p.wind, cfg_.d.gust, sim_.state().t_s); }

  /// Flat-plate normal-force coefficient at angle of attack alpha (0..pi/2).
  float flapCn(float alpha) const { return VCTR::Dyn::flatPlateCn(cfg_.d.aero.flap, alpha); }
  /// Effective aerodynamic flap angle for the true (servo) angle `a`, rad.
  float flapAeroAngle(float a) const { return VCTR::Dyn::flatPlateAeroAngle(cfg_.d.aero.flap, a); }
  /// Plate geometry of flap `index` (TL, TR, BL, BR) at flap angle `angle`: area centroid and unit normal.
  void flapPlate(int index, float angle, VCTR::Math::Vector<float, 3> &centroid, VCTR::Math::Vector<float, 3> &normal) const {
    const int pair = index < 2 ? 0 : 1;
    const float side = (index == 0 || index == 2) ? 1.0f : -1.0f;
    const VCTR::ShipSim::FlapPairGeometry &g = pair == 0 ? cfg_.p.flapTop : cfg_.p.flapBottom;
    VCTR::Math::Vector<float, 3> n, d, e;
    VCTR::ShipSim::flapFrame(cfg_.d.aero, index, angle, n, d, e);
    const VCTR::Math::Vector<float, 3> low = {0.0f, side * cfg_.p.hingeRadius_m, cfg_.d.aero.zLow_m[pair]};
    centroid = low + e * cfg_.d.aero.sc_m[pair] + d * (0.5f * g.width_m);
    normal = n;
  }
  /// Aerodynamic loads for the current state and flap angles (does not integrate); fills loads().
  void computeAero() {
    VCTR::Dyn::ForceSums sums;
    VCTR::ShipSim::AeroOutput aero;
    Plant::evalSums(cfg_, sim_.state(), sim_.state().t_s, sums, last_, aero);
  }

  /// The underlying instance (state, snapshot) and config, for harnesses that need more.
  const VCTR::Dyn::Sim<Plant> &sim() const { return sim_; }
  const Plant::Config &config() const { return cfg_; }

private:
  void initInput() {
    u_.thrustBody = {0, 0, 0};
    u_.rollTorque = 0.0f;
    u_.thrustEnabled = false;
    for (int i = 0; i < 4; i++)
      u_.flapAngle[i] = 1.5708f; // the default flap input (folded)
  }

  Plant::Config cfg_;
  VCTR::Dyn::Sim<Plant> sim_;
  Plant::Input u_;
  Plant::Output last_;
};

} // namespace plant_test

#endif
