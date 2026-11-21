#ifndef TEST_EXECUTIVE_SUPPORT_FOLLOWER_PLANT_HPP
#define TEST_EXECUTIVE_SUPPORT_FOLLOWER_PLANT_HPP

// Follower plant for the executive scenario tests: a point mass whose "estimate" follows the commanded setpoints.
// It is not a vehicle model (no controller or aerodynamic dynamics; design 6.5): it only makes the guards fire the way
// a vehicle following the commands would.
//
// - TvcMode::Position with TVC actuators and motors on: velocity first-order lag towards
//   v_cmd = v_sp + posGain * (p_sp - p), climb rate limited, |a| <= maxAccel; attitude upright.
// - TvcMode::Attitude (kick): a scripted body pitch rate rotates the attitude; the velocity decays.
// - Otherwise in the air: ballistic fall with quadratic drag (terminal speed ~20 m/s), attitude kept.
// - The ground (z = 0) stops the vehicle.

#include <math.h>

#include "ExVectrControl/executive/exec_types.hpp"

namespace ExecTest {

struct PlantConfig {
  float tauVelS = 0.4f;
  float posGain = 1.0f;           ///< 1/s
  float maxAccel = 3.0f;          ///< m/s^2 (about the 14 N stop margin of the vehicle)
  float maxClimbRate = 100.0f;    ///< m/s (lower it to make the climb time out)
  float kickPitchRateRadPs = -1.75f; ///< body +Y rate during the kick (negative pitches towards belly down)
  float dragK = 9.81f / 400.0f;   ///< terminal speed 20 m/s
  bool groundSensor = false;      ///< publish a fresh ground distance (ToF landing trigger)
};

class FollowerPlant {
public:
  explicit FollowerPlant(const PlantConfig &cfg = PlantConfig()) : cfg_(cfg) {}

  float pos[6] = {0, 0, 0, 0, 0, 0};
  float att[7] = {0, 0, 0, 1, 0, 0, 0};
  PlantConfig cfg_;

  VCTR::Exec::EstimateSnapshot snapshot(int64_t nowNs) const {
    VCTR::Exec::EstimateSnapshot s;
    s.tNs = nowNs;
    for (int i = 0; i < 7; i++)
      s.att[i] = att[i];
    for (int i = 0; i < 6; i++)
      s.pos[i] = pos[i];
    s.attValid = true;
    s.posValid = true;
    if (cfg_.groundSensor) {
      s.groundDist = pos[5];
      s.groundCov = 1.0f;
      s.groundFresh = true;
    }
    return s;
  }

  void step(const VCTR::Exec::ControlCommand &c, float dt) {
    using VCTR::Exec::TvcMode;
    const bool powered = c.tvcActuators && c.motors;
    if (c.tvc == TvcMode::Position && powered) {
      setUpright();
      float a[3];
      for (int i = 0; i < 3; i++) {
        float vCmd = c.posSp[i] + cfg_.posGain * (c.posSp[3 + i] - pos[3 + i]);
        if (i == 2 && vCmd > cfg_.maxClimbRate)
          vCmd = cfg_.maxClimbRate;
        a[i] = (vCmd - pos[i]) / cfg_.tauVelS;
      }
      const float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      if (an > cfg_.maxAccel)
        for (float &ai : a)
          ai *= cfg_.maxAccel / an;
      for (int i = 0; i < 3; i++)
        pos[i] += a[i] * dt;
    } else if (c.tvc == TvcMode::Attitude && powered) {
      att[0] = 0;
      att[1] = cfg_.kickPitchRateRadPs;
      att[2] = 0;
      rotateBody(0, 1, 0, cfg_.kickPitchRateRadPs * dt);
      for (int i = 0; i < 3; i++)
        pos[i] *= (1.0f - 0.5f * dt);
    } else if (pos[5] > 0.0f || pos[2] > 0.0f) {
      att[0] = att[1] = att[2] = 0;
      pos[2] += (-9.81f + cfg_.dragK * pos[2] * pos[2]) * dt;
      pos[0] *= (1.0f - 0.2f * dt);
      pos[1] *= (1.0f - 0.2f * dt);
    }
    for (int i = 0; i < 3; i++)
      pos[3 + i] += pos[i] * dt;
    if (pos[5] <= 0.0f) {
      pos[5] = 0.0f;
      for (int i = 0; i < 3; i++)
        pos[i] = 0.0f;
    }
  }

private:
  void setUpright() {
    att[0] = att[1] = att[2] = 0;
    att[3] = 1;
    att[4] = att[5] = att[6] = 0;
  }
  // q = q * dq(axis, angle) (body-frame rotation), renormalised.
  void rotateBody(float ax, float ay, float az, float angle) {
    const float s = sinf(0.5f * angle), c = cosf(0.5f * angle);
    const float b[4] = {c, ax * s, ay * s, az * s};
    const float a[4] = {att[3], att[4], att[5], att[6]};
    float o[4];
    o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    const float n = sqrtf(o[0] * o[0] + o[1] * o[1] + o[2] * o[2] + o[3] * o[3]);
    for (int i = 0; i < 4; i++)
      att[3 + i] = o[i] / n;
  }
};

} // namespace ExecTest

#endif
