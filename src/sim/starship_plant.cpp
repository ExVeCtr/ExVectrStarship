#include "starship_plant.hpp"

#include "ExVectrDynamics/ground_contact.hpp"

#include <math.h>

namespace VCTR {
namespace ShipSim {

namespace {
constexpr float kGravity = 9.807f; // = Math::GRAVITY (as in body_sim_model.cpp)
}

AeroConfig makeAeroConfig(const StarshipPlantParams &p) {
  AeroConfig a;
  a.hull = Dyn::makeSlenderBodyConfig(p.hull, p.hullAero);
  a.flap = p.flapAero;
  a.pair[0] = p.flapTop;
  a.pair[1] = p.flapBottom;
  a.hingeRadius_m = p.hingeRadius_m;
  a.foldTowardBelly = p.flapFoldTowardBelly;
  a.calibration = p.aeroCalibration;
  const FlapPairGeometry *g[2] = {&p.flapTop, &p.flapBottom};
  for (int k = 0; k < 2; k++) {
    const float w = g[k]->width_m, h = g[k]->height_m, b = g[k]->bottomFromShipBottom_m;
    float c = g[k]->chamfer_m / sqrtf(2.0f); // legs of the 45 deg cut
    c = c < w ? c : w;
    c = c < h ? c : h;
    const float aRect = w * h, aTri = 0.5f * c * c;
    a.area_m2[k] = aRect - aTri;
    // Centroid along the axis: rectangle minus the corner triangle at the nose-side outer corner.
    const float zc = (aRect * (b + 0.5f * h) - aTri * (b + h - c / 3.0f)) / a.area_m2[k];
    a.sc_m[k] = zc - b;
    a.zLow_m[k] = b - p.hull.cgFromBottom_m;
  }
  return a;
}

AeroConfig makeScaledAeroConfig(const StarshipPlantParams &p, const AeroScales &s) {
  StarshipPlantParams q = p;
  q.hullAero.cdCrossflow *= s.hullCrossflow;
  q.hullAero.potentialFactor *= s.hullPotential;
  q.hullAero.cdAxialTailFirst *= s.hullAxial;
  q.hullAero.cdAxialNoseFirst *= s.hullAxial;
  q.flapTop.effectiveness *= s.flapTop;
  q.flapBottom.effectiveness *= s.flapBottom;
  q.aeroCalibration *= s.global;
  return makeAeroConfig(q);
}

AeroConfig makeReducedAeroConfig(const StarshipPlantParams &p, int strips, const AeroScales &s) {
  StarshipPlantParams q = p;
  q.hull.strips = strips;
  return makeScaledAeroConfig(q, s);
}

template <int K> typename StarshipPlantT<K>::Config StarshipPlantT<K>::configure(const Params &p) {
  Config c;
  c.p = p;
  Math::Matrix<float, 3, 3> I = Math::Matrix<float, 3, 3>::eye();
  I(0, 0) = p.inertiaXX;
  I(1, 1) = p.inertiaYY;
  I(2, 2) = p.inertiaZZ;
  I(0, 1) = I(1, 0) = p.inertiaXY;
  I(0, 2) = I(2, 0) = p.inertiaXZ;
  I(1, 2) = I(2, 1) = p.inertiaYZ;
  c.d.body = Dyn::makeRigidBodyParams(p.mass_kg, I);
  c.d.aero = makeAeroConfig(p);
  c.d.gust = Dyn::buildGustTable(p.wind);
  return c;
}

template <int K> void StarshipPlantT<K>::init(const Config &, State &x) {
  x.body = Dyn::makeRigidBodyState();
  x.thrustCmd = {0, 0, 0};
  x.thrustMag = 0.0f;
  x.rollTorque = 0.0f;
  x.thrustEnabled = false;
  Dyn::servoInit(x.flaps, 1.5708f);
  x.tvc = Dyn::TvcState();
  x.t_s = 0.0;
}

template <int K> void StarshipPlantT<K>::resetPose(State &x) { x.body = Dyn::makeRigidBodyState(); }

template <int K> void StarshipPlantT<K>::setWind(Config &c, float speed_mps, float direction_rad) {
  Dyn::setWind(c.p.wind, speed_mps, direction_rad);
}
template <int K> void StarshipPlantT<K>::setGust(Config &c, float strength_mps, float period_s, uint32_t seed) {
  Dyn::setGust(c.p.wind, c.d.gust, strength_mps, period_s, seed);
}
template <int K> void StarshipPlantT<K>::clearWind(Config &c) { Dyn::clearWind(c.p.wind, c.d.gust); }

template <int K> void StarshipPlantT<K>::beginStep(const Config &c, State &x, const Input &u, float dt) {
  const Params &p = c.p;
  // 1. Inputs (BodySimModel::setThrust and setFlapInput): clamp into the held state.
  const Dyn::TvcCommand cmd = Dyn::tvcClamp(p.tvc, u.thrustBody);
  x.thrustCmd = cmd.force;
  x.thrustMag = cmd.mag;
  x.rollTorque = isfinite(u.rollTorque) ? u.rollTorque : 0.0f;
  x.thrustEnabled = u.thrustEnabled;
  for (int i = 0; i < 4; i++)
    Dyn::servoSetInput(p.flapServo, x.flaps, i, u.flapAngle[i]);
  // 2. Actuators: flap servos, then TVC slew.
  Dyn::servoStep(p.flapServo, x.flaps, dt, p.step_s, p.applyActuatorModels);
  Dyn::TvcCommand held;
  held.force = x.thrustCmd;
  held.mag = x.thrustMag;
  Dyn::tvcSlewStep(p.tvc, x.tvc, held, p.applyActuatorModels, dt);
}

template <int K>
void StarshipPlantT<K>::evalSums(const Config &c, const State &x, double t_s, Dyn::ForceSums &sums, Output &y,
                                 AeroOutput &aero) {
  const Params &p = c.p;
  // 3. Air velocity at the CoM, wind of the time before the increment.
  const Math::Quat_F q = x.body.att.template block<4, 1>(3, 0);
  const Vec3 v = x.body.pos.template block<3, 1>(0, 0);
  const Vec3 omega = x.body.att.template block<3, 1>(0, 0);
  AeroInput in;
  in.airVelBody = airVelocityBody(q, v, Dyn::windAt(p.wind, c.d.gust, t_s));
  in.omegaBody = omega;
  for (int i = 0; i < 4; i++)
    in.flapAngle[i] = x.flaps.angle[i];
  in.rho = Dyn::airDensity(p.air, x.body.pos(5));
  // 4. Aero.
  starshipAero(c.d.aero, in, aero);

  // 5. Force sums, in the order of BodySimModel::step.
  sums = Dyn::ForceSums();
  const float m = p.mass_kg;
  Dyn::addForce(sums, m, Vec3{0.0f, 0.0f, -kGravity}, Vec3{0, 0, 0}, false, true); // gravity, world, as acceleration
  if (x.thrustEnabled && x.thrustMag > 1e-6f) {
    Vec3 f = x.thrustCmd;
    if (p.applyActuatorModels) // rebuild the vector from the slewed angles (as StarshipTVC does)
      f = Dyn::tvcForceFromAngles(x.tvc, x.thrustMag);
    Dyn::addForce(sums, m, f, Vec3{0.0f, 0.0f, p.tvc.fromCg_m}, true, false);
    Dyn::addTorque(sums, m, Vec3{0.0f, 0.0f, x.rollTorque}, true, false); // gated with the thrust
  }
  Dyn::addForce(sums, m, aero.total.force, Vec3{0, 0, 0}, true, false);
  Dyn::addTorque(sums, m, aero.total.torque, true, false);

  y.aero = aero.total;
  for (int i = 0; i < 4; i++) {
    y.flapForce[i] = aero.flapForce[i];
    y.flapAngle[i] = x.flaps.angle[i];
  }
  y.airVelBody = in.airVelBody;
  y.tvcAngle[0] = x.tvc.angleX;
  y.tvcAngle[1] = x.tvc.angleY;
}

template <int K>
void StarshipPlantT<K>::forces(const Config &c, const State &x, const Input &, double t_s, Dyn::Wrench &body,
                               Vec3 &world, Output &y) {
  Dyn::ForceSums sums;
  AeroOutput aero;
  evalSums(c, x, t_s, sums, y, aero);
  body.force = sums.forceBody;
  body.torque = sums.torqueBody;
  world = sums.forceWorld;
}

template <int K>
void StarshipPlantT<K>::endStep(const Config &, State &x, const Dyn::RigidBodyState &before, float dt) {
  Dyn::groundContactStep(x.body, before);
  x.t_s += double(dt);
}

template struct StarshipPlantT<1>;
template struct StarshipPlantT<128>;

} // namespace ShipSim
} // namespace VCTR
