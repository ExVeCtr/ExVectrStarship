#ifndef STARSHIP_SIM_STARSHIP_AERO_HPP
#define STARSHIP_SIM_STARSHIP_AERO_HPP

/**
 * Starship aerodynamics as a standalone, stateless callable (simulation architecture, section 6).
 *
 * `starshipAero` composes the ExVectrDynamics hull (slender body) and flat-plate blocks with the ship geometry (four
 * flaps TL/TR/BL/BR, top cant, fold direction). It is the exact function the plant's step calls, in the exact order of
 * the legacy BodySimModel::computeAero, so the simulator and any other caller (observers, feedforward) never
 * disagree; they differ only in the config they pass. Pure: no state, no allocation, loop counts depend only on the
 * config. It does not write into any plant state.
 *
 * Frames and units: SI, body axes +Z nose, belly -X, TL/BL on +Y, TR/BR on -Y, origin at the CoM; flap angle 0 =
 * fully extended along +-Y, pi/2 = folded toward +X (or toward -X if foldTowardBelly).
 */

#include "ExVectrDynamics/flat_plate.hpp"
#include "ExVectrDynamics/slender_body.hpp"
#include "ExVectrDynamics/types.hpp"
#include "sim_placement.hpp"

namespace VCTR {
namespace ShipSim {

using Vec3 = Dyn::Vec3;

/// One flap pair (left and right identical, mirrored about the body X-Z plane).
struct FlapPairGeometry {
  float width_m;                ///< Radial extent from the hinge line to the outer edge (chord for belly-down flow).
  float height_m;               ///< Extent along the body axis.
  float chamfer_m;              ///< 45 deg cut at the outer nose-side corner (0 = rectangle).
  float bottomFromShipBottom_m; ///< Lower (tail-side) edge above the bottom of the ship.
  float effectiveness;          ///< Multiplies this pair's aerodynamic force.
  float cant_rad;               ///< Inward cant of the pair about the horizontal line through its lower hinge point.
};

/// Everything starshipAero needs (read-only, shareable). Built by makeAeroConfig(StarshipPlantParams).
struct AeroConfig {
  Dyn::SlenderBodyConfig hull;
  Dyn::FlatPlateCoeffs flap;
  FlapPairGeometry pair[2]; ///< [0] top (TL, TR), [1] bottom (BL, BR).
  float hingeRadius_m;
  bool foldTowardBelly;
  float calibration;        ///< Global scale on every aerodynamic force and moment.
  // Derived per pair:
  float area_m2[2];         ///< Flap area (rectangle minus the chamfer triangle).
  float sc_m[2];            ///< Area centroid distance from the lower edge along the hinge line.
  float zLow_m[2];          ///< Lower hinge point z (body frame).
};

struct AeroInput {
  Vec3 airVelBody;     ///< m/s: air velocity relative to the CoM, body axes = R(q)^T (w_world - v_world).
  Vec3 omegaBody;      ///< rad/s, body axes.
  float flapAngle[4];  ///< TL, TR, BL, BR true servo angles, rad.
  float rho;           ///< kg/m^3.
};

struct AeroOutput {
  Dyn::Wrench total;   ///< Body axes, N and N m about the CoM.
  Dyn::Wrench hull, flaps; ///< Split of the total.
  Vec3 flapForce[4];
  float alpha_rad;     ///< Hull angle of attack (0 axial, pi/2 broadside).
  bool valid;          ///< False if an input was non-finite: then every output is 0.
};

/// Pure, stateless. Equal to BodySimModel::computeAero bit for bit for finite inputs.
SIM_COLD void starshipAero(const AeroConfig &cfg, const AeroInput &in, AeroOutput &out);

/// Air velocity at the CoM in body axes: R(q)^T (wind - v).
SIM_COLD Vec3 airVelocityBody(const Math::Quat_F &qBodyToWorld, const Vec3 &vWorld, const Vec3 &windWorld);

/// Orthonormal plate frame of flap `index` at `angle`: normal n, radial d (hinge -> outer edge), axial e (along the
/// hinge line, lower -> upper end). Port of BodySimModel::flapFrame.
SIM_COLD void flapFrame(const AeroConfig &cfg, int index, float angle, Vec3 &n, Vec3 &d, Vec3 &e);

} // namespace ShipSim
} // namespace VCTR

#endif
