#include "starship_aero.hpp"

#include <math.h>

namespace VCTR {
namespace ShipSim {

Vec3 airVelocityBody(const Math::Quat_F &q, const Vec3 &vWorld, const Vec3 &windWorld) {
  return q.conjugate().rotate(windWorld - vWorld);
}

void flapFrame(const AeroConfig &cfg, int index, float angle, Vec3 &n, Vec3 &d, Vec3 &e) {
  const int pair = index < 2 ? 0 : 1;
  const float side = (index == 0 || index == 2) ? 1.0f : -1.0f; // TL, BL on +Y
  const float ca = cosf(angle), sa = sinf(angle);
  // Uncanted, fold toward +X: TL/BL rotate by -angle about Z, TR/BR by +angle: n = Rz(-+a) x, d = Rz(-+a) (+-y),
  // e = z. Fold toward -X (foldTowardBelly): the rotation sense is mirrored, d_x = -sin a, n_y = +side sin a.
  const float fold = cfg.foldTowardBelly ? -1.0f : 1.0f; // sign of d_x
  float ny = -fold * side * sa, nz = 0.0f, dy = side * ca, dz = 0.0f, ey = 0.0f, ez = 1.0f;
  // Cant: rotation about X through the lower hinge point that takes the axial direction z to
  // (0, -side sin c, cos c), i.e. the upper end moves toward the axis (rotation angle phi = side c about X):
  // y' = y cos(phi) - z sin(phi), z' = y sin(phi) + z cos(phi).
  const float cc = cosf(cfg.pair[pair].cant_rad), sc = side * sinf(cfg.pair[pair].cant_rad);
  const float ny2 = ny * cc - nz * sc, nz2 = ny * sc + nz * cc;
  const float dy2 = dy * cc - dz * sc, dz2 = dy * sc + dz * cc;
  const float ey2 = ey * cc - ez * sc, ez2 = ey * sc + ez * cc;
  n = {ca, ny2, nz2};
  d = {fold * sa, dy2, dz2};
  e = {0.0f, ey2, ez2};
}

void starshipAero(const AeroConfig &cfg, const AeroInput &in, AeroOutput &out) {
  out.total.force = {0, 0, 0};
  out.total.torque = {0, 0, 0};
  out.hull = out.total;
  out.flaps = out.total;
  for (int i = 0; i < 4; i++)
    out.flapForce[i] = {0, 0, 0};
  out.alpha_rad = 0.0f;
  out.valid = false;

  bool finite = isfinite(in.rho);
  for (int i = 0; i < 3; i++)
    finite = finite && isfinite(in.airVelBody(i)) && isfinite(in.omegaBody(i));
  for (int i = 0; i < 4; i++)
    finite = finite && isfinite(in.flapAngle[i]);
  if (!finite)
    return;
  out.valid = true;

  const Vec3 &u0 = in.airVelBody;
  const Vec3 &omega = in.omegaBody;

  // Hull: strips, potential force, axial drag.
  out.hull = Dyn::slenderBodyAero(cfg.hull, in.rho, cfg.calibration, u0, omega);
  out.total = out.hull;
  out.alpha_rad = atan2f(sqrtf(u0(0) * u0(0) + u0(1) * u0(1)), fabsf(u0(2)));

  // Flaps in the order TL, TR, BL, BR, accumulated onto the hull sums (float addition is not associative).
  for (int i = 0; i < 4; i++) { // aerodynamics with the effective angle; the servo angle stays in.flapAngle
    const float angle = Dyn::flatPlateAeroAngle(cfg.flap, in.flapAngle[i]);
    const int pair = i < 2 ? 0 : 1;
    const float side = (i == 0 || i == 2) ? 1.0f : -1.0f; // TL, BL on +Y
    Vec3 n, d, e;
    flapFrame(cfg, i, angle, n, d, e);
    Dyn::FlatPlateGeom g;
    // Hinge reference point: on the hinge line at the area centroid height (canted with the flap).
    g.hinge = Vec3{0.0f, side * cfg.hingeRadius_m, cfg.zLow_m[pair]} + e * cfg.sc_m[pair];
    g.n = n;
    g.d = d;
    g.width_m = cfg.pair[pair].width_m;
    g.area_m2 = cfg.area_m2[pair];
    g.effectiveness = cfg.pair[pair].effectiveness;
    g.bodyRadius_m = cfg.hull.geo.radius_m;
    const Dyn::FlatPlateResult r = Dyn::flatPlateFlap(cfg.flap, g, in.rho, cfg.calibration, angle, u0, omega);
    out.flapForce[i] = r.force;
    if (r.active) {
      out.total.force = out.total.force + r.force;
      out.total.torque = out.total.torque + r.cp.cross(r.force);
      out.flaps.force = out.flaps.force + r.force;
      out.flaps.torque = out.flaps.torque + r.cp.cross(r.force);
    }
  }
}

} // namespace ShipSim
} // namespace VCTR
