#ifndef STARSHIP_SIM_STARSHIP_PLANT_HPP
#define STARSHIP_SIM_STARSHIP_PLANT_HPP

/**
 * The Starship plant: rigid body + hull and flap aerodynamics + TVC and flap actuator models + wind + ground contact,
 * composed from the ExVectrDynamics blocks (simulation architecture, section 4.5). It is a Model in the sense of
 * ExVectrDynamics `model.hpp` and runs in `Dyn::Sim<StarshipPlant>`.
 *
 * It reproduces `DSP::BodySimModel` (ExVectrDSP 5aba4c6) bit for bit on the host (test/test_sim_equivalence), bugs
 * included. Step order (as BodySimModel::step): 1 take inputs (clamped into the held state), 2 actuators (flap servos,
 * TVC slew), 3 air velocity at the CoM with the wind of the time BEFORE the increment, 4 aero (hull strips,
 * potential force, axial drag, flaps TL TR BL BR), 5 force sums (gravity, thrust at z = tvc.fromCg_m, roll torque,
 * aero), 6 semi-implicit Euler step then ground contact with the state from before the step, 7 t += dt.
 *
 * Instances: the Config is read-only and shared (the environment, i.e. wind, is part of it and may be changed between
 * steps with setWind/setGust/clearWind), the State is a small POD (snapshot = copy). No heap, no virtual calls, no
 * globals. NOT used by the vehicle yet (step S4 does that).
 */

#include <stdint.h>

#include "ExVectrDynamics/atmosphere.hpp"
#include "ExVectrDynamics/integrators.hpp"
#include "ExVectrDynamics/rigid_body.hpp"
#include "ExVectrDynamics/servo.hpp"
#include "ExVectrDynamics/tvc.hpp"
#include "ExVectrDynamics/wind.hpp"
#include "sim/starship_aero.hpp"

namespace VCTR {
namespace ShipSim {

/// All parameters of the plant. Defaults are identical to DSP::BodySimParams (owner facts 2026-11-21, textbook
/// coefficients, `aeroCalibration` 4.64 = belly-down terminal speed 6.4 m/s, see docs/libs/ExVectrDSP.md Known issues).
struct StarshipPlantParams {
  // Mass properties (measured, double swing).
  float mass_kg = 1.1f;
  float inertiaXX = 0.044f, inertiaYY = 0.044f, inertiaZZ = 0.004f; ///< kg m^2 about the CoM, body axes.
  float inertiaXY = 0.0f, inertiaXZ = 0.0f, inertiaYZ = 0.0f;      ///< Products of inertia.
  // Geometry and aerodynamics.
  Dyn::SlenderBodyGeometry hull = Dyn::SlenderBodyGeometry();      ///< R 0.065, tube 0.54, nose 0.20, CoM 0.37, 24 strips.
  Dyn::SlenderBodyCoeffs hullAero = Dyn::SlenderBodyCoeffs();      ///< Cdc 1.2, eta 0.65, potential 1.0, Cd axial 0.9/0.35.
  FlapPairGeometry flapTop = {0.07f, 0.14f, 0.08f, 0.53f, 1.0f, 0.17453f};  ///< TL, TR; cant 10 deg.
  FlapPairGeometry flapBottom = {0.08f, 0.21f, 0.08f, 0.01f, 1.0f, 0.0f};   ///< BL, BR.
  Dyn::FlatPlateCoeffs flapAero = Dyn::FlatPlateCoeffs();          ///< C_N_alpha 3.0, C_N90 1.17, stall 30/5 deg, shield 0.7.
  bool flapFoldTowardBelly = false;   ///< false (owner confirmed): folded flaps point toward +X.
  float hingeRadius_m = 0.065f;
  float aeroCalibration = 4.64f;      ///< Global scale on every aero force and moment.
  // Environment and actuators.
  Dyn::AtmosphereParams air = Dyn::AtmosphereParams();
  Dyn::WindParams wind = Dyn::WindParams();
  Dyn::TvcParams tvc = Dyn::TvcParams();                           ///< 14 N, 3 deg, z = -0.35, slew 300 deg/s / 15.
  Dyn::ServoParams flapServo = Dyn::ServoParams();                 ///< 300 deg/s, lag 0, dead time 0, limits 0..90 deg.
  /// True: the plant applies the servo and TVC rate limits itself (host harness). False on the vehicle, where the
  /// drivers already publish rate-limited "actual" values.
  bool applyActuatorModels = true;
  float step_s = 0.002f;
  float maxAdvance_s = 0.05f;
};

/// Builds the read-only aero config (hull, flap geometry, derived areas) from the plant parameters. Use it for a
/// controller's own "belief" config (decision D7).
AeroConfig makeAeroConfig(const StarshipPlantParams &p);

template <int kDelayCap> // 1 on the vehicle (no dead time), 128 in the host harness
struct StarshipPlantT {
  using Params = StarshipPlantParams;
  struct Derived {
    Dyn::RigidBodyParams body;
    AeroConfig aero;
    Dyn::GustTable gust;
  };
  struct Config {
    Params p;
    Derived d;
  };
  struct State {
    Dyn::RigidBodyState body;
    Vec3 thrustCmd;                           ///< Last clamped thrust vector (body axes).
    float thrustMag, rollTorque;
    bool thrustEnabled;
    Dyn::ServoState<4, kDelayCap> flaps;      ///< Flap servos TL, TR, BL, BR (held input, rate-limited, lagged).
    Dyn::TvcState tvc;                        ///< Slewed thrust angles.
    double t_s;                               ///< Simulated time (legacy double accumulator).
  };
  struct Input {
    Vec3 thrustBody;       ///< Thrust force vector in body axes, N (clamped by the plant).
    float rollTorque;      ///< N m about body Z.
    bool thrustEnabled;    ///< Thrust and roll torque act only while enabled.
    float flapAngle[4];    ///< TL, TR, BL, BR, rad (non-finite values keep the last valid one).
  };
  struct Output {
    Dyn::Wrench aero;      ///< Aerodynamic wrench of the last step (body axes).
    Vec3 flapForce[4];
    Vec3 airVelBody;       ///< Air velocity at the CoM of the last step.
    float flapAngle[4];    ///< Modelled flap angles after the servo model.
    float tvcAngle[2];     ///< Modelled thrust angles [about X, about Y].
  };
  using DefaultIntegrator = Dyn::SemiImplicitEuler;

  static Config configure(const Params &p);
  /// Start pose (upright at the origin, at rest), flaps folded (pi/2), thrust off, t = 0.
  static void init(const Config &c, State &x);
  /// Upright at the origin, at rest; actuators and time keep their state (== BodySimModel::resetToStartPose).
  static void resetPose(State &x);
  static float stepSize_s(const Config &c) { return c.p.step_s; }
  static float maxAdvance_s(const Config &c) { return c.p.maxAdvance_s; }

  // Environment changes between steps (same semantics as BodySimModel::setWind/setGust/clearWind).
  static void setWind(Config &c, float speed_mps, float direction_rad);
  static void setGust(Config &c, float strength_mps, float period_s, uint32_t seed);
  static void clearWind(Config &c);

  // ---- Hooks used by the step and by the Rk4 integrator ----
  static const Dyn::RigidBodyParams &bodyParams(const Config &c) { return c.d.body; }
  static double time_s(const State &x) { return x.t_s; }
  /// Steps 1-2: hold the inputs (clamped), flap servos and TVC slew.
  static void beginStep(const Config &c, State &x, const Input &u, float dt);
  /// Steps 3-5: aero and force sums at the state `x` and time `t_s`; fills the aero outputs of `y`.
  static void evalSums(const Config &c, const State &x, double t_s, Dyn::ForceSums &sums, Output &y,
                       AeroOutput &aero);
  /// Same loads as (body, world) for the general integrators.
  static void forces(const Config &c, const State &x, const Input &u, double t_s, Dyn::Wrench &body, Vec3 &world,
                     Output &y);
  /// Step 6 (second half) and 7: ground contact with the state from before the step, time increment.
  static void endStep(const Config &c, State &x, const Dyn::RigidBodyState &before, float dt);

  /// One fixed step (semi-implicit Euler, bit-identical to BodySimModel::step). A probe may define
  /// `void onAero(const AeroOutput&)` to see the aero components; NoProbe ignores it.
  template <class Probe> static void step(const Config &c, State &x, const Input &u, float dt, Output &y, Probe &pr) {
    beginStep(c, x, u, dt);
    Dyn::ForceSums sums;
    AeroOutput aero;
    evalSums(c, x, x.t_s, sums, y, aero);
    probeAero(pr, aero, 0);
    const Dyn::RigidBodyState before = x.body;
    Dyn::semiImplicitEulerStep(c.d.body, x.body, sums, dt);
    endStep(c, x, before, dt);
  }

private:
  template <class P> static auto probeAero(P &p, const AeroOutput &a, int) -> decltype(p.onAero(a), void()) {
    p.onAero(a);
  }
  template <class P> static void probeAero(P &, const AeroOutput &, long) {}
};

using StarshipPlant = StarshipPlantT<1>;          ///< Vehicle variant.
using StarshipPlantHarness = StarshipPlantT<128>; ///< Host harness variant with servo dead time (adds 2 KB).

extern template struct StarshipPlantT<1>;
extern template struct StarshipPlantT<128>;

} // namespace ShipSim
} // namespace VCTR

#endif
