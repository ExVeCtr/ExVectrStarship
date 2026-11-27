// FROZEN verbatim copy of DSP::BodySimModel (ExVectrDSP 5aba4c6, lib/ExVectrDSP/{include,src}/body_sim_model.*), namespace
// renamed to legacy_frozen, member functions made inline (header-only), built on the frozen PhysicsBodySim, and the
// `private:` section opened to `public:` so the block tests can reach the individual legacy functions.
// Golden reference for the ExVectrDynamics equivalence tests (S2 blocks, S3 plant): do NOT edit or "fix" this file;
// bugs found in it are recorded in docs/project/simulation-architecture.md, section 10.
#ifndef FROZEN_LEGACY_BODY_SIM_MODEL_HPP
#define FROZEN_LEGACY_BODY_SIM_MODEL_HPP
#include <math.h>

/**
 * Pure physics model of the Starship body simulator (feature/sim-aero, 2026-11-21).
 *
 * No scheduler, no topics, no wall clock, no random numbers: the caller drives it with a fixed step, so identical
 * inputs give bit-identical runs (host-tested in the flight-software repo, test/test_sim_aero). `BodySimulator`
 * (body_simulator.hpp) is the scheduler task that wraps it on the vehicle.
 *
 * Frames and units: body frame +Z nose, belly -X, flaps TL/BL on +Y and TR/BR on -Y, origin at the centre of
 * mass; world NWU; q = body -> world [w,x,y,z]; attitude state [omega(3) body rad/s, q(4)], position state
 * [V(3), P(3)] world m/s, m. SI throughout. Flap angle: 0 = fully extended along +-Y, 90 deg = folded toward +X.
 *
 * Aerodynamic model (docs/libs/ExVectrDSP.md, "Body simulator model"):
 * - Body crossflow: strip theory over the real length (tube + parabolic nose), each strip
 *   dF = 1/2 rho eta Cdc d(z) dz |u_perp| u_perp with the local air velocity u = u_cg - omega x r, so the
 *   distributed pitch/yaw damping follows from the geometry.
 * - Body potential (slender-body, Jorgensen) normal force 1/2 rho V^2 A_base sin(2 alpha) cos(alpha/2) at the
 *   leading end (nose centre of pressure when the nose leads, the base face when the tail leads): this gives the
 *   angle-of-attack dependent centre of pressure and the belly-down pitch stiffness.
 * - Body axial force 1/2 rho Cd_axial A_base u_z |u_z| (separate coefficients for tail-first and nose-first flow).
 * - Each flap: flat plate normal force 1/2 rho |u|^2 A C_N(alpha) along the plate normal, C_N blended from the
 *   attached (C_N_alpha sin a cos a) to the stalled (C_N90 sin a) branch, centre of pressure moving along the
 *   chord from 1/4 (small alpha) to 1/2 (alpha = 90 deg): x_cp/c = 0.5 -/+ 0.25 cos(alpha). The flap geometry in
 *   the aerodynamics uses the effective angle a_eff = smoothmax(a, 20 deg) (an extended flap still sees a small
 *   effective angle, BodySimParams::flapEffectiveMinAngleRad); the servo angle stays the true angle.
 * - One global calibration factor scales every aerodynamic force (see BodySimParams::aeroCalibration).
 */

#include <stdint.h>

#include "legacy_physics_bodysim.hpp"
#include "ExVectrMath.hpp"

namespace VCTR {
namespace legacy_frozen {

/// One flap pair (left and right flap identical, mirrored about the body X-Z plane).
struct BodySimFlapGeometry {
  float width_m;              ///< Radial extent from the hinge line to the outer edge (the chord for belly-down flow), m.
  float height_m;             ///< Extent along the body axis, m.
  float chamfer_m;            ///< Length of the 45 deg cut at the outer nose-side corner, m (0 = rectangle).
  float bottomFromShipBottom_m; ///< Lower (tail-side) edge above the bottom of the ship, m.
  float effectiveness;        ///< Multiplies this pair's aerodynamic force (body shielding, gaps). 1 = free plate.
};

/**
 * All parameters of the model with units and physical meaning. Defaults: owner's ship facts (2026-11-21) and
 * textbook coefficients, with `aeroCalibration` chosen so that the belly-down steady fall at neutral flaps
 * (top 50 deg, bottom 40 deg) is 6.4 m/s (real flight, 21..25 km/h, owner 2026-11-21).
 */
struct BodySimParams {
  // ---- Mass properties (measured, double swing; owner 2026-11-21) ----
  float mass_kg = 1.1f;   ///< Total mass incl. battery and motors (= VEHICLE_MASS_KG).
  float inertiaXX = 0.044f; ///< kg m^2 about the CoM, body X (pitch/yaw axes are X and Y).
  float inertiaYY = 0.044f; ///< kg m^2, body Y.
  float inertiaZZ = 0.004f; ///< kg m^2, body Z (long axis).
  float inertiaXY = 0.0f, inertiaXZ = 0.0f, inertiaYZ = 0.0f; ///< Products of inertia, kg m^2 (tensor entries).

  // ---- Geometry (owner 2026-11-21). Origin of the body frame = CoM on the axis. ----
  float bodyRadius_m = 0.065f;     ///< Tube radius (diameter 13 cm).
  float tubeLength_m = 0.54f;      ///< Cylindrical tube, bottom face to the nose base.
  float noseLength_m = 0.20f;      ///< Parabolic nose (r = R (2 s - s^2), s = distance from the tip / length).
  float cgFromBottom_m = 0.37f;    ///< CoM above the bottom of the ship ("basically the middle" of 0.74 m).
  float flapHingeRadius_m = 0.065f; ///< Hinge line distance from the axis (on the tube surface).
  BodySimFlapGeometry flapTop = {0.07f, 0.14f, 0.08f, 0.53f, 1.0f};    ///< TL, TR.
  BodySimFlapGeometry flapBottom = {0.08f, 0.21f, 0.08f, 0.01f, 1.0f}; ///< BL, BR: spans 0.01..0.22 m above the bottom.
  /**
   * Inward cant of the TOP flaps (owner 2026-11-21, 10 deg): each top flap is mounted tilted about the
   * horizontal line through its LOWER hinge point so that its upper end moves toward the body axis (the top
   * connection sits close to the nose cone). The lower hinge point stays at (r = flapHingeRadius, z = lower edge);
   * the hinge line, plate normal, centroid, centre of pressure and area direction are rotated with it; the flap
   * angle convention is unchanged (rotation about the tilted hinge line). Bottom flaps are not canted.
   */
  float topFlapCantRad = 0.17453f; ///< 10 deg.
  /**
   * Fold direction of the flaps. false (default, CONFIRMED by the owner 2026-11-21: folded flaps point toward +X,
   * the back): an increasing angle folds the flap toward the back (+X, downstream in the belly-down fall); true:
   * toward the belly (-X, upstream), kept for comparison.
   * Only the sign of the flaps' sideways (Y) force changes, so only the azimuth (tau_x) response to the
   * controller's u0 mixer flips: with false it is reversed against ControlAttitudeFlaps at every angle of attack
   * tested, with true it has the controller's sign; pitch, roll, drag and terminal speed are identical
   * (test_sim_aero "control axis signs", ExVectrDSP.md Known issues).
   */
  bool flapFoldTowardBelly = false;

  // ---- Air ----
  float airDensity_kgm3 = 1.2f; ///< rho.

  // ---- Body aerodynamics ----
  float cdCrossflow = 1.2f;     ///< 2-D circular cylinder crossflow drag coefficient (subcritical Re ~ 7e4).
  float crossflowEta = 0.65f;   ///< Finite-length factor on the crossflow drag (L/D ~ 5.7).
  float potentialFactor = 1.0f; ///< Multiplies the slender-body potential normal force (0 disables it).
  float cdAxialTailFirst = 0.9f; ///< Axial drag coefficient on the base area, flow from the tail (falling upright).
  float cdAxialNoseFirst = 0.35f; ///< Axial drag coefficient on the base area, flow from the nose.
  int bodyStrips = 24;          ///< Number of strips over the body length (crossflow and damping), 4..64.

  // ---- Flap aerodynamics (flat plate) ----
  float flapCnAlpha = 3.0f;           ///< Attached normal-force slope, 1/rad (Helmbold, plate AR ~ 2.5).
  float flapCn90 = 1.17f;             ///< Normal-force coefficient broadside (alpha = 90 deg).
  float flapStallAngle_rad = 0.5236f; ///< 30 deg: centre of the attached -> stalled blend (low-AR plates stall late).
  float flapStallWidth_rad = 0.0873f; ///< 5 deg: width of the blend (logistic).

  /**
   * Effective aerodynamic flap angle (owner 2026-11-21). The flow around the body means a fully extended flap
   * (fold angle 0) still sees a small effective angle instead of producing pure drag. The aerodynamic calculation
   * (plate normal = force direction, C_N argument, centre of pressure) uses
   *   a_eff = smoothmax(a, a_min) = 0.5 (a + a_min + sqrt((a - a_min)^2 + k^2)),  k = flapEffectiveSmoothRad,
   * which is C1 (C-infinity) and converges to a_min (+ k^2 / (4 a_min)) for a -> 0 and to a far above a_min. The
   * servo angle (flapAngle(), telemetry) stays the true angle. flapEffectiveMinAngleRad <= 0 disables it.
   */
  float flapEffectiveMinAngleRad = 0.349066f; ///< a_min, 20 deg.
  float flapEffectiveSmoothRad = 0.087266f;   ///< k, 5 deg: transition width of the smooth max / min.
  /// Same treatment at the retracted end: a_eff = smoothmin(a_eff, 90 deg - margin). 0 = off (a folded flap is
  /// edge-on to the belly-down flow; kept as an option).
  float flapEffectiveRetractMarginRad = 0.0f;

  // ---- Candidate flap/body interaction effects (azimuth sign investigation, 2026-11-21; all off by default) ----
  /**
   * (a) Sidewash at the flap: the local crossflow is turned toward the body axis (gain > 0) or away from it
   * (gain < 0) by delta = gain (R / r_c)^2 sin(alpha_body), r_c = distance of the flap's reference point from the
   * axis, alpha_body = angle between the flow and the body axis (90 deg belly-down). Implemented as an extra
   * velocity delta |u_perp| along -r_hat (radial unit vector of the flap point).
   */
  float flapSidewashGain = 0.0f;
  /**
   * (b) Body-crossflow induced flow: adds gain x the 2-D potential-flow perturbation of a circular cylinder of
   * radius R in the body crossflow, evaluated at the flap's reference point: (u - i v)_p = -V R^2 / z^2 with
   * V = u_x + i u_y (local crossflow) and z = x + i y. 1 = full potential flow, 0 = off.
   */
  float flapBodyCrossflowGain = 0.0f;
  /**
   * (c) Flap/body interference: the effective flap area is multiplied by 1 - factor sin^2(a_eff), i.e. a flap
   * folding against the body side (into its boundary layer / wake) loses area. 0 = off, 1 = no force when folded.
   * ON by default since 2026-11-21 (owner-approved, 0.7): it is the only candidate effect that gives the flap
   * controller's expected yaw (azimuth) sign at the neutral angles 50/40 deg over AoA -20..+20 deg, and the real
   * ship's heading loop works. Fitted to the sign, not measured: the owner plans a bench pivot test in an airflow.
   */
  float flapFoldShieldFactor = 0.7f;

  /**
   * Global calibration factor on every aerodynamic force and moment. Without it (1.0) the model falls belly-down
   * at neutral flaps at about 13.5 m/s; the real ship falls at 6.4 m/s (21..25 km/h, flight position filter), which
   * needs a factor of 4.64 with the fold shield 0.7 (4.00 without it; an effective drag area of 0.43 m^2 instead of
   * 0.097 m^2). That is NOT a plausible
   * coefficient for the documented 1.1 kg ship of 13 x 74 cm: it stands for an unexplained difference (mass, the
   * speed measurement, or missing drag sources) to be resolved with the owner.
   * Because it scales all aerodynamic forces equally, it also scales stiffness, damping and flap authority.
   */
  float aeroCalibration = 4.64f;

  // ---- Actuators ----
  /// True: the model applies the servo rate limits itself (host harness). False on the vehicle, where the flap
  /// driver (ServoAngleSimulator) and the TVC driver already publish rate-limited "actual" values.
  bool applyActuatorRateLimits = true;
  float flapServoRate_radps = 5.23599f;   ///< 300 deg/s (9 g servo, 60 deg per 0.2 s, owner's choice).
  float flapServoLag_s = 0.0f;            ///< Optional first-order lag of the flap angle, s (0 = none).
  float flapServoDeadTime_s = 0.0f;       ///< Optional dead time of the flap angle, s (0 = none, max 0.25 s).
  float flapMinAngle_rad = 0.0f;          ///< Mechanical limits of the modelled flap angle.
  float flapMaxAngle_rad = 1.5708f;       ///< 90 deg.
  float tvcServoRate_radps = 5.23599f;    ///< 300 deg/s vane servo rate (same 9 g servos).
  float tvcServoToThrustGain = 15.0f;     ///< Servo angle per thrust angle (StarshipTVC tvcFactor_ default 15).
  float tvcThrustLimit_N = 14.0f;         ///< |F| limit (TVC_THRUST_LIMIT_N).
  float tvcAngleLimit_rad = 0.05236f;     ///< Thrust angle limit from body +Z (45 deg / 15 = 3 deg).
  float tvcFromCg_m = -0.35f;             ///< Thrust application point on the axis, z (body frame, m).

  // ---- Wind (steady, no gusts; off by default) ----
  float windSpeed_mps = 0.0f;     ///< Horizontal wind speed, m/s (>= 0).
  float windDirection_rad = 0.0f; ///< Direction the air moves TOWARD, counterclockwise from North toward West (NWU).

  // ---- Gusts (simulation only, owner-approved exception 2026-11-21; deterministic, off by default) ----
  /**
   * Gust model: the gust is a deterministic sum of 8 sinusoids with equal amplitude strength/8, frequencies
   * (0.5 .. 2) / gustPeriod_s and phases drawn from a xorshift32 generator seeded with gustSeed (no wall clock, no
   * global random state), evaluated at the simulated time. It is added to the mean wind in the world frame, along
   * the mean wind direction (a head or tail gust); a second independent process of the same kind, scaled by
   * gustSpread, acts horizontally across the mean direction. |gust| along the mean never exceeds gustStrength_mps
   * (hard bound; realised peaks are about 0.6..0.8 of it), RMS = strength / 4, correlation time about the period.
   * Same seed and parameters give a bit-identical gust series.
   */
  float gustStrength_mps = 0.0f; ///< Peak (bound) of the extra speed along the mean direction, m/s (0..10; 0 = no gusts).
  float gustPeriod_s = 5.0f;     ///< Gust timescale, s (0.5..30): the centre period of the sinusoids.
  uint32_t gustSeed = 1;         ///< Seed of the phases and frequency ratios.
  float gustSpread = 0.3f;       ///< Amplitude of the crosswind gust process relative to the along-wind one (0..1).

  // ---- Integration ----
  float step_s = 0.002f;     ///< Fixed integration step, s.
  float maxAdvance_s = 0.05f; ///< advance() integrates at most this much per call (bounded catch-up), s.
};

/// Output of one force evaluation (body frame), kept for logging and tests.
struct BodySimLoads {
  Math::Vector<float, 3> aeroForce;   ///< Sum of aerodynamic forces, N.
  Math::Vector<float, 3> aeroTorque;  ///< Sum of aerodynamic moments about the CoM, N m.
  Math::Vector<float, 3> flapForce[4]; ///< Per flap TL, TR, BL, BR, N.
  Math::Vector<float, 3> airVelocity; ///< Air velocity relative to the CoM, body frame, m/s.
};

class BodySimModel {
 public:
  explicit BodySimModel(const BodySimParams &params = BodySimParams());

  /// Parameters (change, then call applyParams() for mass, inertia or geometry changes).
  BodySimParams &params() { return params_; }
  const BodySimParams &params() const { return params_; }
  void applyParams();

  /// Upright (q = identity) at the origin, at rest; actuators keep their state.
  void resetToStartPose();
  void setAttitudeState(const Math::Vector<float, 7> &s) { body_.setAttitudeState(s); }
  void setPositionState(const Math::Vector<float, 6> &s) { body_.setPositionState(s); }

  /**
   * Thrust input: force vector in the body frame (its magnitude is the thrust, N) and the roll torque about body
   * Z (N m). Both act only while `enabled`. The angle and magnitude limits are applied here.
   */
  void setThrust(const Math::Vector<float, 3> &forceBody, float rollTorque, bool enabled);
  /// Flap angle inputs TL, TR, BL, BR, rad (commanded angles in the harness, "true" angles on the vehicle).
  void setFlapInput(float tl, float tr, float bl, float br);
  void setWind(float speed_mps, float direction_rad);
  /// Gust settings (clamped: strength 0..10 m/s, period 0.5..30 s); non-finite values are ignored. Rebuilds the
  /// gust table; the gust series depends only on these values and the simulated time.
  void setGust(float strength_mps, float period_s, uint32_t seed);
  /// Wind and gusts off, defaults for the period and seed.
  void clearWind();
  /// Gust speed along (index 0) and across (index 1) the mean wind direction at the current simulated time, m/s.
  void gustAt(double t_s, float &along, float &across) const;

  /// One fixed step of params().step_s.
  void step();
  /**
   * Integrates `elapsed_s` of real time in fixed steps (the remainder is carried to the next call). Elapsed time
   * above maxAdvance_s is dropped (bounded catch-up, no huge step after a pause). Returns the number of steps.
   */
  int advance(float elapsed_s);
  /// Drops any carried remainder (call when the simulation is (re)started after a pause).
  void clearTimeCarry() { carry_s_ = 0.0; }

  const Math::Vector<float, 7> &attitudeState() { return body_.getAttitudeState(); }
  const Math::Vector<float, 6> &positionState() { return body_.getPositionState(); }
  /// Modelled flap angles TL, TR, BL, BR after the servo model, rad.
  float flapAngle(int i) const { return flapAngle_[i]; }
  /// Modelled thrust angles [about body X, about body Y] after the TVC slew model, rad.
  float tvcAngleX() const { return tvcAngleX_; }
  float tvcAngleY() const { return tvcAngleY_; }
  const BodySimLoads &loads() const { return loads_; }
  /// Simulated time since construction, s (fixed steps x step_s).
  double time() const { return time_s_; }
  /// World wind vector (NWU, air velocity), m/s.
  Math::Vector<float, 3> windWorld() const;

  /// Flat-plate normal-force coefficient at angle of attack alpha (0..pi/2), exposed for tests.
  float flapCn(float alpha) const;
  /// Effective aerodynamic flap angle for the true (servo) angle `a`, rad (see flapEffectiveMinAngleRad).
  float flapAeroAngle(float a) const;
  /// Plate geometry of flap `index` (TL, TR, BL, BR) at flap angle `angle` in the body frame: area centroid and unit
  /// normal (cant included), exposed for tests.
  void flapPlate(int index, float angle, Math::Vector<float, 3> &centroid, Math::Vector<float, 3> &normal) const;
  /// Computes the aerodynamic loads for the current state and flap angles (does not integrate).
  void computeAero();

 public: // FROZEN COPY: was private, opened for the block tests (the only change besides the namespace)
  BodySimParams params_;
  PhysicsBodySim body_;

  // Thrust
  Math::Vector<float, 3> thrustCmd_;
  float thrustMag_ = 0.0f;
  float rollTorque_ = 0.0f;
  bool thrustEnabled_ = false;
  float tvcAngleX_ = 0.0f, tvcAngleY_ = 0.0f;

  // Flaps
  float flapInput_[4] = {1.5708f, 1.5708f, 1.5708f, 1.5708f};
  float flapRateLimited_[4] = {1.5708f, 1.5708f, 1.5708f, 1.5708f};
  float flapAngle_[4] = {1.5708f, 1.5708f, 1.5708f, 1.5708f};
  static constexpr int kDelayLen = 128;
  float flapDelay_[4][kDelayLen] = {};
  int delayIdx_ = 0;
  bool delayPrimed_ = false;

  // Derived geometry (applyParams)
  float zBottom_ = 0.0f, zNoseBase_ = 0.0f, zTip_ = 0.0f, zNoseCp_ = 0.0f;
  float baseArea_ = 0.0f;
  float flapArea_[2] = {0, 0};
  float flapSc_[2] = {0, 0};   ///< Area centroid distance from the lower edge along the flap (hinge line), m.
  float flapZLow_[2] = {0, 0}; ///< Lower hinge point z (body frame), m.
  float flapCant_[2] = {0, 0}; ///< Inward cant of the pair, rad.
  /// Orthonormal plate frame of flap `index` at `angle`: normal n, radial d (hinge -> outer edge), axial e (along
  /// the hinge line, lower -> upper end). Cant about the horizontal X line through the lower hinge point.
  void flapFrame(int index, float angle, Math::Vector<float, 3> &n, Math::Vector<float, 3> &d,
                 Math::Vector<float, 3> &e) const;

  static constexpr int kGustTerms = 8;
  float gustOmega_[2][kGustTerms] = {};
  float gustPhase_[2][kGustTerms] = {};
  void buildGust();

  BodySimLoads loads_;
  double carry_s_ = 0.0;
  double time_s_ = 0.0;

  void updateActuators(float dt);
  float bodyDiameterAt(float z) const;
  void addFlap(int index, float angle, const Math::Vector<float, 3> &u0, const Math::Vector<float, 3> &omega);
  void groundContact(const Math::Vector<float, 6> &posBefore, const Math::Vector<float, 7> &attBefore);
};




namespace body_sim_frozen_detail {
constexpr float kGravity = 9.807f; // = Math::GRAVITY
constexpr float kPi = 3.14159265f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/// Moves `cur` toward `target` by at most `maxStep`.
inline float slew(float cur, float target, float maxStep) {
  const float d = target - cur;
  if (d > maxStep)
    return cur + maxStep;
  if (d < -maxStep)
    return cur - maxStep;
  return target;
}
} // namespace body_sim_frozen_detail
using namespace body_sim_frozen_detail;

inline BodySimModel::BodySimModel(const BodySimParams &params)
    : params_(params), body_(params.mass_kg, Math::Matrix<float, 3, 3>::eye()) {
  thrustCmd_ = {0, 0, 0};
  loads_.aeroForce = {0, 0, 0};
  loads_.aeroTorque = {0, 0, 0};
  loads_.airVelocity = {0, 0, 0};
  for (int i = 0; i < 4; i++)
    loads_.flapForce[i] = {0, 0, 0};
  applyParams();
  resetToStartPose();
}

inline void BodySimModel::applyParams() {
  const BodySimParams &p = params_;
  body_.setMass(p.mass_kg);
  Math::Matrix<float, 3, 3> I = Math::Matrix<float, 3, 3>::eye();
  I(0, 0) = p.inertiaXX;
  I(1, 1) = p.inertiaYY;
  I(2, 2) = p.inertiaZZ;
  I(0, 1) = I(1, 0) = p.inertiaXY;
  I(0, 2) = I(2, 0) = p.inertiaXZ;
  I(1, 2) = I(2, 1) = p.inertiaYZ;
  body_.setInertiaTensor(I);

  zBottom_ = -p.cgFromBottom_m;
  zNoseBase_ = p.tubeLength_m - p.cgFromBottom_m;
  zTip_ = zNoseBase_ + p.noseLength_m;
  // Slender-body potential normal force of a nose acts at x = L (1 - V_nose / (A_base L)) from the tip; for the
  // parabolic nose r = R (2s - s^2): V_nose = 8/15 A_base L.
  zNoseCp_ = zTip_ - p.noseLength_m * (1.0f - 8.0f / 15.0f);
  baseArea_ = kPi * p.bodyRadius_m * p.bodyRadius_m;

  buildGust();

  const BodySimFlapGeometry *g[2] = {&p.flapTop, &p.flapBottom};
  for (int k = 0; k < 2; k++) {
    const float w = g[k]->width_m, h = g[k]->height_m, b = g[k]->bottomFromShipBottom_m;
    float c = g[k]->chamfer_m / sqrtf(2.0f); // legs of the 45 deg cut
    c = c < w ? c : w;
    c = c < h ? c : h;
    const float aRect = w * h, aTri = 0.5f * c * c;
    flapArea_[k] = aRect - aTri;
    // Centroid along the axis: rectangle minus the corner triangle at the nose-side outer corner.
    const float zc = (aRect * (b + 0.5f * h) - aTri * (b + h - c / 3.0f)) / flapArea_[k];
    flapSc_[k] = zc - b;
    flapZLow_[k] = b - p.cgFromBottom_m;
    flapCant_[k] = k == 0 ? p.topFlapCantRad : 0.0f;
  }
}

inline void BodySimModel::resetToStartPose() {
  body_.setAttitudeState({0, 0, 0, 1, 0, 0, 0});
  body_.setPositionState({0, 0, 0, 0, 0, 0});
}

inline void BodySimModel::setThrust(const Math::Vector<float, 3> &forceBody, float rollTorque, bool enabled) {
  Math::Vector<float, 3> f = forceBody;
  if (!(isfinite(f(0)) && isfinite(f(1)) && isfinite(f(2))))
    f = {0, 0, 0};
  // Magnitude limit.
  float mag = f.magnitude();
  if (mag > params_.tvcThrustLimit_N) {
    f = f * (params_.tvcThrustLimit_N / mag);
    mag = params_.tvcThrustLimit_N;
  }
  // Cone limit about body +Z: keep the azimuth, put the vector on the cone (the old clamp rotated away from +Z).
  const float lat = sqrtf(f(0) * f(0) + f(1) * f(1));
  if (mag > 1e-6f && atan2f(lat, f(2)) > params_.tvcAngleLimit_rad && lat > 1e-9f) {
    const float s = sinf(params_.tvcAngleLimit_rad) * mag / lat;
    f = {f(0) * s, f(1) * s, cosf(params_.tvcAngleLimit_rad) * mag};
  }
  thrustCmd_ = f;
  thrustMag_ = mag;
  rollTorque_ = isfinite(rollTorque) ? rollTorque : 0.0f;
  thrustEnabled_ = enabled;
}

inline void BodySimModel::setFlapInput(float tl, float tr, float bl, float br) {
  const float in[4] = {tl, tr, bl, br};
  for (int i = 0; i < 4; i++)
    if (isfinite(in[i]))
      flapInput_[i] = clampf(in[i], params_.flapMinAngle_rad, params_.flapMaxAngle_rad);
}

inline void BodySimModel::setWind(float speed_mps, float direction_rad) {
  if (isfinite(speed_mps))
    params_.windSpeed_mps = speed_mps < 0.0f ? 0.0f : speed_mps;
  if (isfinite(direction_rad))
    params_.windDirection_rad = direction_rad;
}

inline void BodySimModel::buildGust() {
  uint32_t s = params_.gustSeed ? params_.gustSeed : 0x9E3779B9u;
  auto next = [&s]() { // xorshift32
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  };
  for (int k = 0; k < 12; k++) // warm-up: small seeds start poorly mixed
    next();
  const float period = clampf(params_.gustPeriod_s, 0.5f, 30.0f);
  for (int p = 0; p < 2; p++)
    for (int i = 0; i < kGustTerms; i++) {
      const float u1 = float(next() >> 8) / 16777216.0f, u2 = float(next() >> 8) / 16777216.0f;
      gustOmega_[p][i] = 2.0f * kPi / period * (0.5f + 1.5f * u1);
      gustPhase_[p][i] = 2.0f * kPi * u2;
    }
}

inline void BodySimModel::gustAt(double t_s, float &along, float &across) const {
  const float strength = clampf(params_.gustStrength_mps, 0.0f, 10.0f);
  along = 0.0f;
  across = 0.0f;
  if (strength <= 0.0f)
    return;
  double sum[2] = {0.0, 0.0};
  for (int p = 0; p < 2; p++)
    for (int i = 0; i < kGustTerms; i++)
      sum[p] += sin(double(gustOmega_[p][i]) * t_s + double(gustPhase_[p][i]));
  const float a = strength / float(kGustTerms);
  along = a * float(sum[0]);
  across = a * clampf(params_.gustSpread, 0.0f, 1.0f) * float(sum[1]);
}

inline void BodySimModel::setGust(float strength_mps, float period_s, uint32_t seed) {
  if (isfinite(strength_mps))
    params_.gustStrength_mps = clampf(strength_mps, 0.0f, 10.0f);
  if (isfinite(period_s))
    params_.gustPeriod_s = clampf(period_s, 0.5f, 30.0f);
  params_.gustSeed = seed;
  buildGust();
}

inline void BodySimModel::clearWind() {
  const BodySimParams d;
  params_.windSpeed_mps = 0.0f;
  params_.windDirection_rad = 0.0f;
  setGust(0.0f, d.gustPeriod_s, d.gustSeed);
}

inline Math::Vector<float, 3> BodySimModel::windWorld() const {
  const float cd = cosf(params_.windDirection_rad), sd = sinf(params_.windDirection_rad);
  float ga, gc;
  gustAt(time_s_, ga, gc);
  const float along = params_.windSpeed_mps + ga;
  return {along * cd - gc * sd, along * sd + gc * cd, 0.0f};
}

inline float BodySimModel::flapCn(float alpha) const {
  const float sa = sinf(alpha), ca = cosf(alpha);
  const float sigma = 1.0f / (1.0f + expf(-(alpha - params_.flapStallAngle_rad) / params_.flapStallWidth_rad));
  return (1.0f - sigma) * params_.flapCnAlpha * sa * ca + sigma * params_.flapCn90 * sa;
}

inline float BodySimModel::flapAeroAngle(float a) const {
  const float k = params_.flapEffectiveSmoothRad;
  const float aMin = params_.flapEffectiveMinAngleRad;
  if (aMin > 0.0f)
    a = 0.5f * (a + aMin + sqrtf((a - aMin) * (a - aMin) + k * k));
  if (params_.flapEffectiveRetractMarginRad > 0.0f) {
    const float aMax = 0.5f * kPi - params_.flapEffectiveRetractMarginRad;
    a = 0.5f * (a + aMax - sqrtf((a - aMax) * (a - aMax) + k * k));
  }
  return a;
}

inline float BodySimModel::bodyDiameterAt(float z) const {
  const float d = 2.0f * params_.bodyRadius_m;
  if (z <= zNoseBase_)
    return d;
  float s = (zTip_ - z) / params_.noseLength_m;
  s = clampf(s, 0.0f, 1.0f);
  return d * (2.0f * s - s * s);
}

inline void BodySimModel::updateActuators(float dt) {
  const BodySimParams &p = params_;

  // Flaps: dead time -> rate limit -> first-order lag.
  int delaySteps = int(p.flapServoDeadTime_s / p.step_s + 0.5f);
  delaySteps = delaySteps < 0 ? 0 : (delaySteps > kDelayLen - 1 ? kDelayLen - 1 : delaySteps);
  if (!delayPrimed_) {
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < kDelayLen; j++)
        flapDelay_[i][j] = flapInput_[i];
    delayPrimed_ = true;
  }
  for (int i = 0; i < 4; i++)
    flapDelay_[i][delayIdx_] = flapInput_[i];
  const int readIdx = (delayIdx_ - delaySteps + kDelayLen) % kDelayLen;
  delayIdx_ = (delayIdx_ + 1) % kDelayLen;

  for (int i = 0; i < 4; i++) {
    const float target = flapDelay_[i][readIdx];
    flapRateLimited_[i] = p.applyActuatorRateLimits ? slew(flapRateLimited_[i], target, p.flapServoRate_radps * dt)
                                                    : target;
    if (p.flapServoLag_s > 0.0f)
      flapAngle_[i] += (flapRateLimited_[i] - flapAngle_[i]) * (dt / (p.flapServoLag_s + dt));
    else
      flapAngle_[i] = flapRateLimited_[i];
  }

  // TVC: slew of the thrust angles = servo rate / servo-to-thrust gain (servo space, review F6).
  const float tgtX = atan2f(thrustCmd_(1), thrustCmd_(2));
  const float tgtY = atan2f(thrustCmd_(0), thrustCmd_(2));
  if (p.applyActuatorRateLimits && thrustMag_ > 1e-6f) {
    const float maxStep = p.tvcServoRate_radps / p.tvcServoToThrustGain * dt;
    tvcAngleX_ = slew(tvcAngleX_, tgtX, maxStep);
    tvcAngleY_ = slew(tvcAngleY_, tgtY, maxStep);
  } else if (thrustMag_ > 1e-6f) {
    tvcAngleX_ = tgtX;
    tvcAngleY_ = tgtY;
  }
}

inline void BodySimModel::flapFrame(int index, float angle, Math::Vector<float, 3> &n, Math::Vector<float, 3> &d,
                             Math::Vector<float, 3> &e) const {
  const int pair = index < 2 ? 0 : 1;
  const float side = (index == 0 || index == 2) ? 1.0f : -1.0f; // TL, BL on +Y
  const float ca = cosf(angle), sa = sinf(angle);
  // Uncanted, fold toward +X: TL/BL rotate by -angle about Z, TR/BR by +angle: n = Rz(-+a) x, d = Rz(-+a) (+-y),
  // e = z. Fold toward -X (flapFoldTowardBelly): the rotation sense is mirrored, d_x = -sin a, n_y = +side sin a.
  const float fold = params_.flapFoldTowardBelly ? -1.0f : 1.0f; // sign of d_x
  float ny = -fold * side * sa, nz = 0.0f, dy = side * ca, dz = 0.0f, ey = 0.0f, ez = 1.0f;
  // Cant: rotation about X through the lower hinge point that takes the axial direction z to
  // (0, -side sin c, cos c), i.e. the upper end moves toward the axis (rotation angle phi = side c about X):
  // y' = y cos(phi) - z sin(phi), z' = y sin(phi) + z cos(phi).
  const float cc = cosf(flapCant_[pair]), sc = side * sinf(flapCant_[pair]);
  const float ny2 = ny * cc - nz * sc, nz2 = ny * sc + nz * cc;
  const float dy2 = dy * cc - dz * sc, dz2 = dy * sc + dz * cc;
  const float ey2 = ey * cc - ez * sc, ez2 = ey * sc + ez * cc;
  n = {ca, ny2, nz2};
  d = {fold * sa, dy2, dz2};
  e = {0.0f, ey2, ez2};
}

inline void BodySimModel::flapPlate(int index, float angle, Math::Vector<float, 3> &centroid,
                             Math::Vector<float, 3> &normal) const {
  const int pair = index < 2 ? 0 : 1;
  const float side = (index == 0 || index == 2) ? 1.0f : -1.0f;
  const BodySimFlapGeometry &g = pair == 0 ? params_.flapTop : params_.flapBottom;
  Math::Vector<float, 3> n, d, e;
  flapFrame(index, angle, n, d, e);
  const Math::Vector<float, 3> low = {0.0f, side * params_.flapHingeRadius_m, flapZLow_[pair]};
  centroid = low + e * flapSc_[pair] + d * (0.5f * g.width_m);
  normal = n;
}

inline void BodySimModel::addFlap(int index, float angle, const Math::Vector<float, 3> &u0,
                           const Math::Vector<float, 3> &omega) {
  const BodySimParams &p = params_;
  const int pair = index < 2 ? 0 : 1;
  const BodySimFlapGeometry &g = pair == 0 ? p.flapTop : p.flapBottom;
  const float side = (index == 0 || index == 2) ? 1.0f : -1.0f; // TL, BL on +Y
  Math::Vector<float, 3> n, d, e;
  flapFrame(index, angle, n, d, e);
  // Hinge reference point: on the hinge line at the area centroid height (canted with the flap).
  const Math::Vector<float, 3> hinge = Math::Vector<float, 3>{0.0f, side * p.flapHingeRadius_m, flapZLow_[pair]} +
                                       e * flapSc_[pair];

  const Math::Vector<float, 3> pc = hinge + d * (0.5f * g.width_m);
  Math::Vector<float, 3> u = u0 - omega.cross(pc);
  // Candidate interaction effects (off by default), evaluated at the flap reference point pc.
  const float rc2 = pc(0) * pc(0) + pc(1) * pc(1);
  const float R2 = p.bodyRadius_m * p.bodyRadius_m;
  if (p.flapBodyCrossflowGain != 0.0f && rc2 > 1e-8f) {
    // (u - i v)_p = -V R^2 / z^2, V = ux + i uy, 1/z^2 = (x^2 - y^2 - 2 i x y) / r^4.
    const float x = pc(0), y = pc(1), r4 = rc2 * rc2;
    const float ar = (x * x - y * y) / r4, ai = -2.0f * x * y / r4;      // 1/z^2 = ar + i ai
    const float pr = u(0) * ar - u(1) * ai, pi = u(0) * ai + u(1) * ar; // V / z^2
    // u_p - i v_p = -R^2 (pr + i pi): u_p = -R^2 pr, v_p = +R^2 pi.
    u(0) += p.flapBodyCrossflowGain * (-R2 * pr);
    u(1) += p.flapBodyCrossflowGain * (R2 * pi);
  }
  if (p.flapSidewashGain != 0.0f && rc2 > 1e-8f) {
    const float uPerp = sqrtf(u0(0) * u0(0) + u0(1) * u0(1));
    const float sinAb = uPerp / sqrtf(uPerp * uPerp + u0(2) * u0(2) + 1e-12f);
    const float rc = sqrtf(rc2);
    const float delta = p.flapSidewashGain * (R2 / rc2) * sinAb;
    u(0) -= delta * uPerp * pc(0) / rc;
    u(1) -= delta * uPerp * pc(1) / rc;
  }
  const float speed2 = u.dot(u);
  if (speed2 < 1e-6f) {
    loads_.flapForce[index] = {0, 0, 0};
    return;
  }
  const float speed = sqrtf(speed2);
  const float un = u.dot(n), ud = u.dot(d);
  const float alpha = asinf(clampf(fabsf(un) / speed, 0.0f, 1.0f));
  const float sAng = sinf(angle);
  const float shield = 1.0f - clampf(p.flapFoldShieldFactor, 0.0f, 1.0f) * sAng * sAng;
  const float fmag = 0.5f * p.airDensity_kgm3 * p.aeroCalibration * g.effectiveness * shield * flapArea_[pair] *
                     speed2 * flapCn(alpha);
  const Math::Vector<float, 3> F = n * (un >= 0.0f ? fmag : -fmag);
  // Chordwise centre of pressure from the leading edge: 1/4 chord at small alpha, 1/2 at 90 deg.
  const float xLead = g.width_m * (0.5f - 0.25f * cosf(alpha));
  const float xcp = ud >= 0.0f ? xLead : g.width_m - xLead; // flow hinge -> tip: hinge edge leads
  const Math::Vector<float, 3> pcp = hinge + d * xcp;

  loads_.flapForce[index] = F;
  loads_.aeroForce = loads_.aeroForce + F;
  loads_.aeroTorque = loads_.aeroTorque + pcp.cross(F);
}

inline void BodySimModel::computeAero() {
  const BodySimParams &p = params_;
  const Math::Quat_F q = body_.getAttitude();
  const Math::Vector<float, 3> v = body_.getVelocity();
  const Math::Vector<float, 3> omega = body_.getAngularVelocity();
  const Math::Vector<float, 3> u0 = q.conjugate().rotate(windWorld() - v); // air velocity at the CoM, body
  const float k = 0.5f * p.airDensity_kgm3 * p.aeroCalibration;

  loads_.airVelocity = u0;
  loads_.aeroForce = {0, 0, 0};
  loads_.aeroTorque = {0, 0, 0};

  // Body crossflow strips (with the rotation-induced flow: distributed damping).
  int n = p.bodyStrips < 4 ? 4 : (p.bodyStrips > 64 ? 64 : p.bodyStrips);
  const float len = zTip_ - zBottom_, dz = len / float(n);
  float fx = 0, fy = 0, tx = 0, ty = 0;
  for (int i = 0; i < n; i++) {
    const float z = zBottom_ + (float(i) + 0.5f) * dz;
    // u = u0 - omega x r, r = (0, 0, z): omega x r = (wy z, -wx z, 0)
    const float ux = u0(0) - omega(1) * z;
    const float uy = u0(1) + omega(0) * z;
    const float up = sqrtf(ux * ux + uy * uy);
    const float c = k * p.crossflowEta * p.cdCrossflow * bodyDiameterAt(z) * dz * up;
    const float sfx = c * ux, sfy = c * uy;
    fx += sfx;
    fy += sfy;
    tx += -z * sfy; // r x F = (0,0,z) x (Fx,Fy,0) = (-z Fy, z Fx, 0)
    ty += z * sfx;
  }
  loads_.aeroForce = loads_.aeroForce + Math::Vector<float, 3>{fx, fy, 0.0f};
  loads_.aeroTorque = loads_.aeroTorque + Math::Vector<float, 3>{tx, ty, 0.0f};

  // Slender-body potential normal force at the leading end, and axial force.
  const float uPerp = sqrtf(u0(0) * u0(0) + u0(1) * u0(1));
  const float speed = sqrtf(uPerp * uPerp + u0(2) * u0(2));
  if (speed > 1e-3f && uPerp > 1e-6f && p.potentialFactor != 0.0f) {
    const float alpha = atan2f(uPerp, fabsf(u0(2))); // 0 = axial flow, 90 deg = broadside
    const float fp = k * p.potentialFactor * baseArea_ * speed * speed * sinf(2.0f * alpha) * cosf(0.5f * alpha);
    const float zLead = u0(2) < 0.0f ? zNoseCp_ : zBottom_; // air from the nose (u_z < 0): nose leads
    const float px = fp * u0(0) / uPerp, py = fp * u0(1) / uPerp;
    loads_.aeroForce = loads_.aeroForce + Math::Vector<float, 3>{px, py, 0.0f};
    loads_.aeroTorque = loads_.aeroTorque + Math::Vector<float, 3>{-zLead * py, zLead * px, 0.0f};
  }
  const float cdAx = u0(2) > 0.0f ? p.cdAxialTailFirst : p.cdAxialNoseFirst;
  loads_.aeroForce(2) += k * cdAx * baseArea_ * u0(2) * fabsf(u0(2)); // on the axis: no moment

  for (int i = 0; i < 4; i++) // aerodynamics with the effective angle; the servo angle stays flapAngle_
    addFlap(i, flapAeroAngle(flapAngle_[i]), u0, omega);
}

inline void BodySimModel::groundContact(const Math::Vector<float, 6> &posBefore, const Math::Vector<float, 7> &attBefore) {
  Math::Vector<float, 6> &pos = body_.getPositionState();
  if (pos(5) >= 0.0f)
    return;
  // Ground (height 0 of the CoM): stop, no sliding, no rotation, no tip-over (simplest contact model). A body that
  // was already resting keeps its previous horizontal position and attitude, so wind or residual torques cannot
  // move it along the ground; a body arriving from the air stops where it hits.
  const bool wasResting = posBefore(5) <= 0.0f;
  pos(5) = 0.0f;
  pos(0) = 0.0f;
  pos(1) = 0.0f;
  if (pos(2) < 0.0f)
    pos(2) = 0.0f;
  Math::Vector<float, 7> &att = body_.getAttitudeState();
  if (wasResting) {
    pos(3) = posBefore(3);
    pos(4) = posBefore(4);
    for (int i = 3; i < 7; i++)
      att(i) = attBefore(i);
  }
  att(0) = 0.0f;
  att(1) = 0.0f;
  att(2) = 0.0f;
}

inline void BodySimModel::step() {
  const float dt = params_.step_s;
  updateActuators(dt);
  computeAero();

  body_.clearForces();
  body_.addForce({0.0f, 0.0f, -kGravity}, {0, 0, 0}, false, true); // gravity, world frame, as acceleration
  if (thrustEnabled_ && thrustMag_ > 1e-6f) {
    Math::Vector<float, 3> f = thrustCmd_;
    if (params_.applyActuatorRateLimits) // rebuild the vector from the slewed angles (as StarshipTVC does)
      f = {sinf(tvcAngleY_) * thrustMag_, sinf(tvcAngleX_) * thrustMag_,
           cosf(tvcAngleX_) * cosf(tvcAngleY_) * thrustMag_};
    body_.addForce(f, {0.0f, 0.0f, params_.tvcFromCg_m});
    body_.addTorque({0.0f, 0.0f, rollTorque_}); // gated with the thrust (review F13)
  }
  body_.addForce(loads_.aeroForce);
  body_.addTorque(loads_.aeroTorque);
  const Math::Vector<float, 6> posBefore = body_.getPositionState();
  const Math::Vector<float, 7> attBefore = body_.getAttitudeState();
  body_.step(dt);
  groundContact(posBefore, attBefore);
  time_s_ += double(dt);
}

inline int BodySimModel::advance(float elapsed_s) {
  if (!(elapsed_s > 0.0f) || !isfinite(elapsed_s))
    return 0;
  if (elapsed_s > params_.maxAdvance_s)
    elapsed_s = params_.maxAdvance_s;
  carry_s_ += double(elapsed_s);
  int steps = 0;
  const double h = double(params_.step_s);
  while (carry_s_ >= h - 1e-6) { // 1 us tolerance: float step and float elapsed
    step();
    carry_s_ -= h;
    steps++;
  }
  if (carry_s_ < 0.0)
    carry_s_ = 0.0;
  return steps;
}


} // namespace legacy_frozen
} // namespace VCTR
#endif
