// Host tests of the flight executive behaviours and guards (ExVectrControl/executive/behaviours.*, guards.*,
// kick.*): pio test -e native -f test_executive_behaviours
//
// The go-to, transit and land behaviours are compared with verbatim copies of the legacy mission code
// (src/mission/mission_waypoint.hpp missionHover steps 4-5, mission_rth.hpp Translation and Descent) running on the
// real ExVectrMath types: the go-to must match bit for bit, the others within 1e-6. The kick geometry tests moved
// here from test_bellyflop_kick (which stays until E6 for the legacy re-export).
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "ExVectrMath.hpp"

#include "ExVectrControl/executive/behaviours.hpp"
#include "ExVectrControl/executive/exec_math.hpp"
#include "ExVectrControl/executive/guards.hpp"
#include "ExVectrControl/executive/kick.hpp"

using namespace VCTR;
using namespace VCTR::Exec;

void setUp() {}
void tearDown() {}

static constexpr int64_t MS = 1000000LL;
static constexpr int64_t S = 1000000000LL;
static const float kDeg = kDegToRad;

static float frand(float lo, float hi) { return lo + (hi - lo) * float(rand()) / float(RAND_MAX); }

// ---------------------------------------------------------------------------------------------------------------
// Legacy references (copied from the old missions; only member names adapted to locals)

struct LegacyWaypoint {
  Math::Vector<float, 3> position;
  float velocity, startAccel, stopAccel;
};

struct LegacyWaypointProfile { // MissionWaypoint::missionHover, steps 4 and 5
  Math::Vector<float, 6> positionSetpoint_;
  float currentVelocity_ = 0;
  void step(const LegacyWaypoint &waypoint, const Math::Vector<float, 6> &positionIs_, float dTime) {
    auto distance = waypoint.position - positionIs_.block<3, 1>(3);
    auto distanceMagnitude = distance.magnitude();
    auto accelVelocityLimit = sqrtf(distanceMagnitude * waypoint.stopAccel);
    if (waypoint.stopAccel > 0.001 && currentVelocity_ > accelVelocityLimit) {
      currentVelocity_ = accelVelocityLimit;
    } else if (waypoint.velocity > currentVelocity_) {
      currentVelocity_ += waypoint.startAccel * dTime;
      if (currentVelocity_ > waypoint.velocity || waypoint.startAccel < 0.001) {
        currentVelocity_ = waypoint.velocity;
      }
    }
    auto travelDistance = waypoint.position - positionSetpoint_.block<3, 1>(3);
    auto travelDistanceNorm = travelDistance.normalize();
    if (travelDistance.magnitude() / dTime > currentVelocity_) {
      travelDistance = travelDistance.normalize() * currentVelocity_ * dTime;
    }
    positionSetpoint_(0) = travelDistanceNorm(0) * currentVelocity_;
    positionSetpoint_(1) = travelDistanceNorm(1) * currentVelocity_;
    positionSetpoint_(2) = travelDistanceNorm(2) * currentVelocity_;
    positionSetpoint_(3) += travelDistance(0);
    positionSetpoint_(4) += travelDistance(1);
    positionSetpoint_(5) += travelDistance(2);
  }
};

struct LegacyTranslation { // MissionRTH RunningState_Translation (setpoint part)
  Math::Vector<float, 6> positionSetpoint_;
  Math::Vector<float, 3> homePosition_;
  float translationVelocity_ = 0;
  void step(float dTime) {
    auto travelDistance = homePosition_ - positionSetpoint_.block<3, 1>(3);
    auto travelDistanceMag = travelDistance.magnitude();
    auto travelDistanceNorm = travelDistance / travelDistanceMag;
    const float accel = 0.5;
    if (translationVelocity_ < 2)
      translationVelocity_ += dTime * accel;
    if (translationVelocity_ > 2) {
      translationVelocity_ = 2;
    }
    if (travelDistanceMag / dTime > translationVelocity_) {
      travelDistance = travelDistanceNorm * translationVelocity_ * dTime;
    }
    auto accelVelocityLimit = sqrtf(travelDistanceMag * accel);
    if (translationVelocity_ > accelVelocityLimit) {
      translationVelocity_ = accelVelocityLimit;
    }
    positionSetpoint_(0) = travelDistanceNorm(0) * translationVelocity_;
    positionSetpoint_(1) = travelDistanceNorm(1) * translationVelocity_;
    positionSetpoint_(2) = travelDistanceNorm(2) * translationVelocity_;
    positionSetpoint_(3) += travelDistance(0);
    positionSetpoint_(4) += travelDistance(1);
    positionSetpoint_(5) += travelDistance(2);
  }
};

static EstimateSnapshot estAt(float x, float y, float z, float vx = 0, float vy = 0, float vz = 0) {
  EstimateSnapshot e;
  e.pos[0] = vx;
  e.pos[1] = vy;
  e.pos[2] = vz;
  e.pos[3] = x;
  e.pos[4] = y;
  e.pos[5] = z;
  e.posValid = e.attValid = true;
  return e;
}

// ---------------------------------------------------------------------------------------------------------------
// GoTo

static void runGoToParity(const GoToParams &p, unsigned seed) {
  srand(seed);
  LegacyWaypoint wp{{p.target[0], p.target[1], p.target[2]}, p.speedMps, p.startAccelMps2, p.stopAccelMps2};
  LegacyWaypointProfile ref;
  GoToMem mem;
  float meas[3] = {frand(-2, 2), frand(-2, 2), frand(0, 2)};
  EstimateSnapshot est = estAt(meas[0], meas[1], meas[2]);
  goToInit(p, est, mem);
  for (int i = 0; i < 3; i++)
    ref.positionSetpoint_(3 + i) = mem.spPos[i];
  for (int k = 0; k < 4000; k++) {
    // The measured position follows the setpoint with lag and noise; dt varies (10 ms nominal, also 100 ms).
    const float dt = (k % 97 == 0) ? 0.1f : (k % 13 == 0 ? 0.0137f : 0.01f);
    for (int i = 0; i < 3; i++)
      meas[i] += (ref.positionSetpoint_(3 + i) - meas[i]) * 0.05f + frand(-0.01f, 0.01f);
    est = estAt(meas[0], meas[1], meas[2]);
    Math::Vector<float, 6> posIs;
    for (int i = 0; i < 6; i++)
      posIs(i) = est.pos[i];
    ref.step(wp, posIs, dt);
    BehaviourOut out;
    goToStep(p, est, dt, mem, out);
    for (int i = 0; i < 6; i++) {
      if (out.posSp[i] != ref.positionSetpoint_(i)) {
        char msg[96];
        snprintf(msg, sizeof(msg), "step %d elem %d: %.9g vs legacy %.9g", k, i, double(out.posSp[i]),
                 double(ref.positionSetpoint_(i)));
        TEST_FAIL_MESSAGE(msg);
      }
    }
    TEST_ASSERT_TRUE(mem.speedMps == ref.currentVelocity_);
  }
}

static void test_goto_matches_legacy_bit_for_bit() {
  // The Starship hover check and ascent waypoints, plus edge cases (no braking cap, instant ramp, slow target).
  const GoToParams cases[] = {
      {{0, 0, 1}, 0.5f, 3, 1, GoToEntry::JumpTo, {0, 0, 0.3f}},
      {{0, 0, 120}, 7, 3, 2, GoToEntry::JumpTo, {0, 0, 1}},
      {{5, -3, 2}, 2, 0, 0, GoToEntry::FromMeasured, {0, 0, 0}},
      {{-1, 4, 0.5f}, 1, 0.5f, 3, GoToEntry::FromMeasured, {0, 0, 0}},
  };
  unsigned seed = 1;
  for (const GoToParams &p : cases)
    runGoToParity(p, seed++);
}

static void test_goto_init() {
  GoToParams p{{0, 0, 1}, 0.5f, 3, 1, GoToEntry::JumpTo, {0, 0, 0.3f}};
  GoToMem m;
  m.speedMps = 5;
  goToInit(p, estAt(1, 2, 3), m);
  TEST_ASSERT_EQUAL_FLOAT(0.3f, m.spPos[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, m.spPos[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, m.speedMps);
  p.entry = GoToEntry::FromMeasured;
  goToInit(p, estAt(1, 2, 3), m);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, m.spPos[0]);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, m.spPos[2]);
  p.speedMps = 0; // instant-jump waypoint
  goToInit(p, estAt(1, 2, 3), m);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, m.spPos[2]);
  BehaviourOut out;
  goToStep(p, estAt(1, 2, 3), 0.01f, m, out);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, out.posSp[5]); // stays at the target
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.posSp[2]);
}

// ---------------------------------------------------------------------------------------------------------------
// Transit, Land, Hold, Idle

static void test_transit_matches_legacy() {
  const TransitParams p{{0, 0, 3}, 2.0f, 0.5f};
  TransitMem m;
  EstimateSnapshot est = estAt(12, -7, 25);
  transitInit(p, est, m);
  LegacyTranslation ref;
  ref.homePosition_ = Math::Vector<float, 3>({0, 0, 3});
  for (int i = 0; i < 3; i++)
    ref.positionSetpoint_(3 + i) = est.pos[3 + i];
  for (int k = 0; k < 1500; k++) { // until just before the setpoint reaches home
    const float dt = (k % 10 == 0) ? 0.1f : 0.01f;
    ref.step(dt);
    BehaviourOut out;
    transitStep(p, est, dt, m, out);
    if (!isfinite(ref.positionSetpoint_(0)))
      break;
    for (int i = 0; i < 6; i++)
      TEST_ASSERT_FLOAT_WITHIN(1e-6f, ref.positionSetpoint_(i), out.posSp[i]);
  }
  // The legacy unit vector has no zero guard: at home exactly the output is not finite (the sanitiser's job).
  TransitMem atHome;
  transitInit(p, estAt(0, 0, 3), atHome);
  BehaviourOut out;
  transitStep(p, estAt(0, 0, 3), 0.01f, atHome, out);
  TEST_ASSERT_FALSE(allFinite(out.posSp, 6));
}

static void test_land() {
  const LandParams p{{1, 2, 3}, 0.2f, 0.3f};
  LandMem m;
  landInit(p, estAt(0.9f, 2.1f, 3.4f), m);
  TEST_ASSERT_EQUAL_FLOAT(3.4f, m.spZ);
  BehaviourOut out;
  landStep(p, estAt(0.9f, 2.1f, 3.4f), 0.1f, m, out); // vehicle at the setpoint: ramp
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.38f, out.posSp[5]);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, out.posSp[3]); // XY at home
  TEST_ASSERT_EQUAL_FLOAT(2.0f, out.posSp[4]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.posSp[2]);
  landStep(p, estAt(1, 2, 3.68f + 0.001f), 0.1f, m, out); // 0.3 m above: the ramp pauses
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.38f, out.posSp[5]);
}

static void test_hold_and_idle() {
  const BehaviourTables t;
  BehaviourMem mem;
  behaviourInit(Behaviour::HoldMeasured, 0, t, estAt(1, 2, 3, 4, 5, 6), mem);
  BehaviourOut out;
  behaviourStep(t, estAt(1, 2, 3, 4, 5, 6), 0.01f, mem, out);
  const float hold[6] = {0, 0, 0, 1, 2, 3};
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(hold, out.posSp, 6);
  behaviourInit(Behaviour::Idle, 0, t, estAt(1, 2, 3), mem);
  BehaviourOut idle;
  behaviourStep(t, estAt(1, 2, 3), 0.01f, mem, idle);
  const float zero[6] = {0, 0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(zero, idle.posSp, 6);
  float target[3];
  TEST_ASSERT_FALSE(behaviourTarget(Behaviour::HoldMeasured, 0, t, target));
}

// ---------------------------------------------------------------------------------------------------------------
// Descent guidance (legacy FlapModeObserverTask Stabilize branch)

static DescentParams starshipDescent() {
  return DescentParams{1.1f, 14.0f, 20.0f, 100.0f, 1.0f, 1.0f * kDeg, -40.0f * kDeg, 20.0f * kDeg, 20.0f, 30.0f * kDeg};
}

static void test_descent_guidance() {
  const DescentParams p = starshipDescent();
  DescentMem m;
  descentInit(p, estAt(0, 0, 0), m);
  BehaviourOut out;
  // High, near the origin: pitch +20 deg - 0 offset; azimuth back to the origin.
  descentStep(p, estAt(0.5f, 0, 120, 0, 0, -10), m, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 20.0f * kDeg, out.flapPitchRad);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -atan2f(0.0f, -0.5f), out.flapAzimuthRad);
  // High, 21 m off: offset -20 deg -> pitch 0; at >= 41 m: -20 deg.
  descentStep(p, estAt(0, 21, 120, 0, 0, -10), m, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, out.flapPitchRad);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -atan2f(21.0f, -0.0f), out.flapAzimuthRad);
  const float azHigh = out.flapAzimuthRad;
  descentStep(p, estAt(10, 60, 110, 0, 0, -10), m, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -20.0f * kDeg, out.flapPitchRad);
  // Middle band: pitch 0, azimuth kept.
  descentStep(p, estAt(5, 5, 90, 0, 0, -10), m, out);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out.flapPitchRad);
  TEST_ASSERT_TRUE(out.flapAzimuthRad != azHigh); // it was updated at the 110 m step
  const float azKept = out.flapAzimuthRad;
  // Low: below stop altitude + 20 m (20 m/s: 0.5*400/(14/1.1-9.81) + 20 = 88.6 m; + 20 = 108.6 m ... use 80 m).
  descentStep(p, estAt(5, 5, 80, 0, 0, -20), m, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 30.0f * kDeg, out.flapPitchRad);
  TEST_ASSERT_EQUAL_FLOAT(azKept, out.flapAzimuthRad);
  const float zero[6] = {0, 0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL_FLOAT_ARRAY(zero, out.posSp, 6);
  // Stop distance: legacy numbers (missions.md section 4: 0.171 * Vz^2, 89 m trigger at 20 m/s with the 20 m margin).
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 68.6f, descentStopDistanceM(-20.0f, 14.0f, 1.1f));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 68.6f, descentStopDistanceM(20.0f, 14.0f, 1.1f)); // sign-blind (known issue)
}

// ---------------------------------------------------------------------------------------------------------------
// Kick (moved from test_bellyflop_kick)

static void quatAxisAngle(float ax, float ay, float az, float ang, float q[4]) {
  const float s = sinf(0.5f * ang);
  q[0] = cosf(0.5f * ang);
  q[1] = ax * s;
  q[2] = ay * s;
  q[3] = az * s;
}

static void quatMul(const float a[4], const float b[4], float o[4]) {
  o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void axis(const float q[4], float x, float y, float z, float out[3]) {
  const float v[3] = {x, y, z};
  kickRotate(q, v, out);
}

static void test_kick_end_trigger() {
  const KickEndLimits lim = {1000 * MS, 15 * kDeg, 250 * kDeg};
  TEST_ASSERT_TRUE(kickEndTrigger(100 * MS, 90 * kDeg, 0, lim) == KickEnd::None);
  TEST_ASSERT_TRUE(kickEndTrigger(1000 * MS, 90 * kDeg, 0, lim) == KickEnd::None);
  TEST_ASSERT_TRUE(kickEndTrigger(1000 * MS + 1, NAN, NAN, lim) == KickEnd::Time);
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, 14 * kDeg, 0, lim) == KickEnd::Angle);
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, 80 * kDeg, -251 * kDeg, lim) == KickEnd::Rate);
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, INFINITY, -INFINITY, lim) == KickEnd::None);
}

static void test_kick_target_identity_is_old_setpoint() {
  const float q0[4] = {1, 0, 0, 0};
  float t[4], old[4];
  kickTargetAttitude(q0, 90 * kDeg, t);
  quatAxisAngle(0, 1, 0, -90 * kDeg, old);
  for (int i = 0; i < 4; i++)
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, old[i], t[i]);
  float b[3], n[3];
  axis(t, -1, 0, 0, b);
  axis(t, 0, 0, 1, n);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, b[2]); // belly down
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, n[0]); // nose south
}

static void test_kick_target_keeps_heading() {
  for (int k = -6; k <= 6; k++) {
    float qYaw[4], qTilt[4], q0[4], t[4];
    quatAxisAngle(0, 0, 1, k * 30.0f * kDeg, qYaw);
    quatAxisAngle(1, 0, 0, 3 * kDeg, qTilt);
    quatMul(qYaw, qTilt, q0);
    kickTargetAttitude(q0, 90 * kDeg, t);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, t[0] * t[0] + t[1] * t[1] + t[2] * t[2] + t[3] * t[3]);
    TEST_ASSERT_TRUE(t[0] >= 0);
    float bt[3], b0[3], nt[3];
    axis(t, -1, 0, 0, bt);
    axis(q0, -1, 0, 0, b0);
    axis(t, 0, 0, 1, nt);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, bt[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, nt[2]);
    const float bh = sqrtf(b0[0] * b0[0] + b0[1] * b0[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, (nt[0] * b0[0] + nt[1] * b0[1]) / bh);
  }
  // Already flat (degenerate heading) and non-finite start attitude: still a finite unit target.
  float flat[4], t[4], b[3];
  quatAxisAngle(0, 1, 0, -90 * kDeg, flat);
  kickTargetAttitude(flat, 90 * kDeg, t);
  axis(t, -1, 0, 0, b);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, b[2]);
  const float bad[4] = {NAN, 0, 0, 0};
  kickTargetAttitude(bad, 90 * kDeg, t);
  TEST_ASSERT_TRUE(allFinite(t, 4));
}

static void test_kick_remaining_angle_and_behaviour() {
  const float q0[4] = {1, 0, 0, 0};
  float t[4];
  kickTargetAttitude(q0, 90 * kDeg, t);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90 * kDeg, kickRemainingAngle(q0, t));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, kickRemainingAngle(t, t));
  float q40[4];
  quatAxisAngle(0, 1, 0, -40 * kDeg, q40);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50 * kDeg, kickRemainingAngle(q40, t));
  const float bad[4] = {NAN, 0, 0, 0};
  TEST_ASSERT_TRUE(isnan(kickRemainingAngle(bad, t)));
  // Behaviour: target from the attitude at entry, constant thrust, hold position at entry.
  const KickParams p{90 * kDeg, 11.0f};
  KickMem m;
  EstimateSnapshot est = estAt(1, 2, 120);
  kickInit(p, est, m);
  BehaviourOut out;
  kickStep(p, m, out);
  for (int i = 0; i < 4; i++)
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, t[i], out.attSp[i]);
  TEST_ASSERT_EQUAL_FLOAT(11.0f, out.thrustN);
  TEST_ASSERT_EQUAL_FLOAT(120.0f, out.posSp[5]);
}

// ---------------------------------------------------------------------------------------------------------------
// Guards

static GuardContext ctxFor(const EstimateSnapshot &est, const BehaviourMem &mem, int64_t now, int64_t elapsed,
                           const float target[3] = nullptr) {
  GuardContext c;
  c.est = &est;
  c.mem = &mem;
  c.nowNs = now;
  c.elapsedNs = elapsed;
  if (target) {
    c.hasTarget = true;
    for (int i = 0; i < 3; i++)
      c.target[i] = target[i];
  }
  return c;
}

static void test_guard_inside_for_restarts_outside() {
  const GuardParams p = GuardParams::distance(0.5f, 5 * S);
  const float tgt[3] = {0, 0, 1};
  BehaviourMem mem;
  GuardMem g;
  guardInit(Guard::InsideFor, 0, g);
  const EstimateSnapshot in = estAt(0, 0, 0.8f), out = estAt(0, 0, 0.4f);
  TEST_ASSERT_FALSE(guardEval(Guard::InsideFor, p, ctxFor(in, mem, 4 * S, 4 * S, tgt), g));
  TEST_ASSERT_FALSE(guardEval(Guard::InsideFor, p, ctxFor(out, mem, 4 * S + 10 * MS, 0, tgt), g)); // restart
  TEST_ASSERT_FALSE(guardEval(Guard::InsideFor, p, ctxFor(in, mem, 9 * S + 10 * MS, 0, tgt), g)); // == 5 s: no
  TEST_ASSERT_TRUE(guardEval(Guard::InsideFor, p, ctxFor(in, mem, 9 * S + 20 * MS, 0, tgt), g));
  // No target: never.
  GuardMem g2;
  guardInit(Guard::InsideFor, 0, g2);
  TEST_ASSERT_FALSE(guardEval(Guard::InsideFor, p, ctxFor(in, mem, 100 * S, 0), g2));
}

static void test_guard_not_reached_within() {
  const GuardParams p = GuardParams::distance(0.5f, 10 * S);
  const float tgt[3] = {0, 0, 1};
  BehaviourMem mem;
  GuardMem g;
  guardInit(Guard::NotReachedWithin, 0, g);
  const EstimateSnapshot far = estAt(0, 0, 0), near = estAt(0, 0, 0.9f);
  TEST_ASSERT_FALSE(guardEval(Guard::NotReachedWithin, p, ctxFor(far, mem, 10 * S, 10 * S, tgt), g)); // not >
  TEST_ASSERT_TRUE(guardEval(Guard::NotReachedWithin, p, ctxFor(far, mem, 10 * S + 1, 10 * S + 1, tgt), g));
  // Reached once: the limit no longer applies (legacy, known issue).
  guardInit(Guard::NotReachedWithin, 0, g);
  TEST_ASSERT_FALSE(guardEval(Guard::NotReachedWithin, p, ctxFor(near, mem, 3 * S, 3 * S, tgt), g));
  TEST_ASSERT_FALSE(guardEval(Guard::NotReachedWithin, p, ctxFor(far, mem, 30 * S, 30 * S, tgt), g));
}

static void test_guard_simple_predicates() {
  BehaviourMem mem;
  GuardMem g;
  GuardContext c;
  const EstimateSnapshot slow = estAt(0, 0, 50, 0.3f, 0.2f, 0.2f), fast = estAt(0, 0, 50, 0.4f, 0.3f, 0.0f);
  TEST_ASSERT_TRUE(guardEval(Guard::SpeedBelow, GuardParams::speedBelow(0.5f), ctxFor(slow, mem, 0, 0), g));
  TEST_ASSERT_FALSE(guardEval(Guard::SpeedBelow, GuardParams::speedBelow(0.5f), ctxFor(fast, mem, 0, 0), g));
  // Mission time strictly above.
  c = ctxFor(slow, mem, 0, 0);
  c.missionTimeNs = -10 * S;
  TEST_ASSERT_FALSE(guardEval(Guard::MissionTimeAbove, GuardParams::missionTimeAbove(-10 * S), c, g));
  c.missionTimeNs = -10 * S + 1;
  TEST_ASSERT_TRUE(guardEval(Guard::MissionTimeAbove, GuardParams::missionTimeAbove(-10 * S), c, g));
  // Stop altitude: 20 m/s down -> 68.6 m + 20 m.
  const GuardParams stop = GuardParams::stopAltitude(1.1f, 14.0f, 20.0f);
  TEST_ASSERT_FALSE(guardEval(Guard::StopAltitude, stop, ctxFor(estAt(0, 0, 89, 0, 0, -20), mem, 0, 0), g));
  TEST_ASSERT_TRUE(guardEval(Guard::StopAltitude, stop, ctxFor(estAt(0, 0, 88, 0, 0, -20), mem, 0, 0), g));
  // Pitch rate (either sign, finite only).
  EstimateSnapshot spin = estAt(0, 0, 100);
  spin.att[1] = -260 * kDeg;
  TEST_ASSERT_TRUE(guardEval(Guard::PitchRateAbove, GuardParams::rateAbove(250 * kDeg), ctxFor(spin, mem, 0, 0), g));
  spin.att[1] = NAN;
  TEST_ASSERT_FALSE(guardEval(Guard::PitchRateAbove, GuardParams::rateAbove(250 * kDeg), ctxFor(spin, mem, 0, 0), g));
  // Altitude at least: needs a valid position.
  EstimateSnapshot high = estAt(0, 0, 95);
  TEST_ASSERT_TRUE(guardEval(Guard::AltitudeAtLeast, GuardParams::altitudeAtLeast(90), ctxFor(high, mem, 0, 0), g));
  high.posValid = false;
  TEST_ASSERT_FALSE(guardEval(Guard::AltitudeAtLeast, GuardParams::altitudeAtLeast(90), ctxFor(high, mem, 0, 0), g));
  TEST_ASSERT_FALSE(
      guardEval(Guard::AltitudeAtLeast, GuardParams::altitudeAtLeast(90), ctxFor(estAt(0, 0, 89.9f), mem, 0, 0), g));
}

static void test_guard_kick_angle_needs_kick_behaviour() {
  BehaviourTables t;
  const KickParams kp{90 * kDeg, 11.0f};
  t.kick = &kp;
  t.kickCount = 1;
  BehaviourMem mem;
  EstimateSnapshot est = estAt(0, 0, 120);
  behaviourInit(Behaviour::KickAttitude, 0, t, est, mem);
  GuardMem g;
  const GuardParams p = GuardParams::angleBelow(15 * kDeg);
  TEST_ASSERT_FALSE(guardEval(Guard::KickAngleBelow, p, ctxFor(est, mem, 0, 0), g));
  quatAxisAngle(0, 1, 0, -80 * kDeg, &est.att[3]); // 10 deg remaining
  TEST_ASSERT_TRUE(guardEval(Guard::KickAngleBelow, p, ctxFor(est, mem, 0, 0), g));
  BehaviourMem other; // not a kick: never
  TEST_ASSERT_FALSE(guardEval(Guard::KickAngleBelow, p, ctxFor(est, other, 0, 0), g));
}

static void test_guard_landing_detected() {
  const GuardParams p = GuardParams::landing(0.2f, 2 * S, -0.02f, 0.04f, 7.0f);
  BehaviourMem mem;
  mem.active = Behaviour::Land;
  mem.land.spZ = 0.0f;
  GuardMem g;
  guardInit(Guard::LandingDetected, 0, g);
  // Tracking: vehicle 0.25 m above the Z setpoint for > 2 s.
  EstimateSnapshot above = estAt(0, 0, 0.25f);
  TEST_ASSERT_FALSE(guardEval(Guard::LandingDetected, p, ctxFor(above, mem, 1 * S, 0), g));
  TEST_ASSERT_FALSE(guardEval(Guard::LandingDetected, p, ctxFor(above, mem, 2 * S, 0), g));
  TEST_ASSERT_TRUE(guardEval(Guard::LandingDetected, p, ctxFor(above, mem, 2 * S + 1, 0), g));
  // Below the threshold the timer restarts.
  guardInit(Guard::LandingDetected, 0, g);
  TEST_ASSERT_FALSE(guardEval(Guard::LandingDetected, p, ctxFor(above, mem, 1500 * MS, 0), g));
  TEST_ASSERT_FALSE(guardEval(Guard::LandingDetected, p, ctxFor(estAt(0, 0, 0.1f), mem, 1600 * MS, 0), g));
  TEST_ASSERT_FALSE(guardEval(Guard::LandingDetected, p, ctxFor(above, mem, 3500 * MS, 0), g));
  // Ground window: fresh, within (-0.02, 0.04), confident.
  EstimateSnapshot ground = estAt(0, 0, 0.1f);
  ground.groundDist = 0.03f;
  ground.groundCov = 1.0f;
  ground.groundFresh = true;
  guardInit(Guard::LandingDetected, 0, g);
  TEST_ASSERT_TRUE(guardEval(Guard::LandingDetected, p, ctxFor(ground, mem, 10 * MS, 0), g));
  ground.groundFresh = false;
  TEST_ASSERT_FALSE(guardEval(Guard::LandingDetected, p, ctxFor(ground, mem, 20 * MS, 0), g));
  ground.groundFresh = true;
  ground.groundCov = 8.0f;
  TEST_ASSERT_FALSE(guardEval(Guard::LandingDetected, p, ctxFor(ground, mem, 30 * MS, 0), g));
  ground.groundCov = 1.0f;
  ground.groundDist = 0.05f;
  TEST_ASSERT_FALSE(guardEval(Guard::LandingDetected, p, ctxFor(ground, mem, 40 * MS, 0), g));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_goto_matches_legacy_bit_for_bit);
  RUN_TEST(test_goto_init);
  RUN_TEST(test_transit_matches_legacy);
  RUN_TEST(test_land);
  RUN_TEST(test_hold_and_idle);
  RUN_TEST(test_descent_guidance);
  RUN_TEST(test_kick_end_trigger);
  RUN_TEST(test_kick_target_identity_is_old_setpoint);
  RUN_TEST(test_kick_target_keeps_heading);
  RUN_TEST(test_kick_remaining_angle_and_behaviour);
  RUN_TEST(test_guard_inside_for_restarts_outside);
  RUN_TEST(test_guard_not_reached_within);
  RUN_TEST(test_guard_simple_predicates);
  RUN_TEST(test_guard_kick_angle_needs_kick_behaviour);
  RUN_TEST(test_guard_landing_detected);
  return UNITY_END();
}
