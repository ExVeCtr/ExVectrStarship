// Replica of the body simulator BEFORE feature/sim-aero (ExVectrDSP 56a998b body_simulator.cpp, ExVectrControl
// 6e07d38 physics_bodysim.cpp), used only to measure the old model in the native harness. Forces, geometry and the
// explicit Euler integrator are copied 1:1 (old flap driver slew 200 deg/s and old TVC thrust-angle slew 600 deg/s
// emulated by the harness). The random wind is replaced by a fixed body-frame vector (default 0).
#pragma once

#include <math.h>

#include "ExVectrMath.hpp"

namespace legacy {

using namespace VCTR::Math;
using VCTR::DEGREES;

struct LegacyModel {
  float mass = 1.1f;
  float I[3] = {0.044f, 0.044f, 0.004f};
  Vector<float, 6> pos;
  Vector<float, 7> att;
  Vector_F windBody;
  Vector_F thrust;
  float flaps[4] = {1.5708f, 1.5708f, 1.5708f, 1.5708f};
  double t = 0;

  LegacyModel() {
    pos = {0, 0, 0, 0, 0, 0};
    att = {0, 0, 0, 1, 0, 0, 0};
    windBody = {0, 0, 0};
    thrust = {0, 0, 0};
  }

  static Vector_F cylForce(const Vector_F &af) {
    const float cylSideArea = 0.12 * 0.5;
    const float cylCoeff = 1.1;
    auto velProj = af.getProjectionOn(Vector_F{0, 0, 1}, true);
    return velProj * (0.5 * 1.2 * cylCoeff * cylSideArea * velProj.magnitude());
  }
  static Vector_F capForce(const Vector_F &af, const Vector_F &normal) {
    const float cylFlatArea = 2 * M_PI * (0.12 / 2) * (0.12 / 2);
    const float flatCoeff = 1.17;
    if (af.getAngleTo(normal) > 90 * DEGREES)
      return Vector_F{0, 0, 0};
    auto velProj = af.getProjectionOn(normal, false);
    return velProj * (0.5 * 1.2 * flatCoeff * cylFlatArea * velProj.magnitude());
  }
  static Vector_F flapForce(const Vector_F &sn, const Vector_F &af, float basis) {
    auto velSurface = af.getProjectionOn(sn);
    return velSurface * (velSurface.magnitude() * 0.5 * basis);
  }

  void step(float dt) {
    Quat_F q = att.block<4, 1>(3, 0);
    Vector_F v = pos.block<3, 1>(0, 0);
    Vector_F w = att.block<3, 1>(0, 0);
    Vector_F vb = q.conjugate().rotate(v);
    Vector_F F = {0, 0, 0}, T = {0, 0, 0};
    auto add = [&](const Vector_F &f, const Vector_F &r) {
      F = F + f;
      T = T + r.cross(f);
    };
    add(thrust, Vector_F{0, 0, -0.35f});
    Vector_F af = -vb - windBody;
    const float off = 0.1f;
    auto afT = af - w.cross(Vector_F{0, 0, off});
    auto afB = af - w.cross(Vector_F{0, 0, -off});
    Vector_F shift = {0, 0, -af(2) / 200.0f};
    add(cylForce(afT) / 2, shift + Vector_F{0, 0, off});
    add(cylForce(afB) / 2, shift + Vector_F{0, 0, -off});
    add(capForce(af, Vector_F{0, 0, 1}), Vector_F{0, 0, 0.35f});
    add(capForce(af, Vector_F{0, 0, -1}), Vector_F{0, 0, -0.35f});
    const float mountY = 0.08, mountZ = 0.26, copY = 0.03;
    const float sgn[4] = {-1, 1, -1, 1};
    const float my[4] = {mountY, -mountY, mountY, -mountY};
    const float mz[4] = {mountZ, mountZ, -mountZ, -mountZ};
    const float cy[4] = {copY, -copY, copY, -copY};
    const float area[4] = {7.49e-3f, 7.49e-3f, 9.12e-3f, 9.12e-3f};
    for (int i = 0; i < 4; i++) {
      Quat_F rotRaw(Vector_F{0, 0, 1}, sgn[i] * flaps[i]); // CoP uses the unclamped angle
      Vector_F p = rotRaw.rotate(Vector_F{0, cy[i], 0}) + Vector_F{0, my[i], mz[i]};
      float a = flaps[i] < 10 * DEGREES ? 10 * DEGREES : (flaps[i] > 90 * DEGREES ? 90 * DEGREES : flaps[i]);
      Quat_F rot(Vector_F{0, 0, 1}, sgn[i] * a);
      Vector_F sn = rot.rotate(Vector_F{1, 0, 0});
      add(flapForce(sn, af - w.cross(p), area[i] * 1.44f), p);
    }
    // Old PhysicsBodySim::simulateForTime: p += v_old dt, v += a dt, q *= dq(w_old), w += tau/I dt.
    Vector_F Fw = q.rotate(F) + Vector_F{0, 0, -9.807f * mass};
    for (int i = 0; i < 3; i++) {
      pos(i + 3) += pos(i) * dt;
      pos(i) += Fw(i) / mass * dt;
    }
    q = q * Quat_F(w.normalize(), w.magnitude() * dt);
    for (int i = 0; i < 3; i++)
      w(i) += T(i) / I[i] * dt;
    q = q.normalize();
    if (q(0) < 0)
      q = -q;
    att.block(w, 0, 0);
    att.block(q, 3, 0);
    if (pos(5) < 0) {
      if (pos(2) < 0)
        pos(2) = 0;
      pos(0) = pos(1) = pos(5) = 0;
      att(0) = att(1) = att(2) = 0;
    }
    t += dt;
  }
};

} // namespace legacy
