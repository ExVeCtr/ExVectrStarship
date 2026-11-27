// Equivalence test of the vehicle simulation adapter (step S4 of docs/project/simulation-architecture.md):
// src/sim/starship_sim_adapter.* (the logic of StarshipSimTask) against a FROZEN copy of the legacy wiring
// DSP::BodySimulator (ExVectrDSP 5aba4c6, body_simulator.cpp: constructor parameters, setTVCInputTopic, resetToStartPose,
// update(), the published ground pair), driving the frozen legacy BodySimModel (test/frozen/). Topics are replaced by
// direct calls in the order the legacy update() used (`isDataNew` -> item). Criterion: BIT-IDENTICAL (memcmp) after
// every run: attitude [w,q], position [V,P], ground pair, timestamp, modelled flap angles, simulated time, wind and gust
// getters, and the simulated lidar reading. Run: pio test -e native -f test_sim_task
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "../frozen/legacy_body_sim_model.hpp"
#include "sim/starship_sim_adapter.hpp"
#include "sim_lidar.hpp"

using namespace VCTR;
using Legacy = legacy_frozen::BodySimModel;
using LegacyParams = legacy_frozen::BodySimParams;
using Adapter = ShipSim::StarshipSimAdapter;

void setUp() {}
void tearDown() {}

namespace {

constexpr int64_t kMs = 1000000LL;
constexpr float kDeg = 3.14159265f / 180.0f;
// Vehicle constants (src/starship_hardware.h): mass, TVC thrust limit, angle limit 45 deg / 15, thrust point z.
constexpr float kMass = 1.1f;
constexpr float kThrustLimit = 14.0f;
const float kAngleLimit = float(45 * 0.017453292519943295 / 15);
constexpr float kTvcZ = -0.35f;

struct Rng {
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed ? seed : 1) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  float uni(float lo, float hi) { return lo + (hi - lo) * (float(next() >> 8) / 16777216.0f); }
  bool coin() { return next() & 1; }
  int pick(int n) { return int(next() % uint32_t(n)); }
};

template <class T> bool sameBits(const T &a, const T &b) { return memcmp(&a, &b, sizeof(T)) == 0; }

/// FROZEN copy of DSP::BodySimulator (body_simulator.cpp, ExVectrDSP 5aba4c6) without scheduler and topics. The
/// statements are verbatim; the `isDataNew`/`getItem` pairs became the pointer arguments of update(). Do not "fix" it.
struct LegacyWiring {
  static constexpr int64_t kMaxGapNs = 100 * 1000 * 1000;

  Legacy model_;
  Math::Vector<float, 3> tvcForce_;
  float tvcRollTorque_ = 0;
  int64_t stateTimestamp_ = 0;
  bool zeroingMode_ = false;
  bool tvcEnabled_ = true;
  bool flapsEnabled_ = false;
  Math::Vector<float, 7> attitudeState_;
  Math::Vector<float, 6> positionState_;

  static LegacyParams vehicleParams(float mass, const Math::Vector<float, 3> &inertia, float thrustLimit,
                                    float angleLimit) {
    LegacyParams p;
    p.mass_kg = mass;
    p.inertiaXX = inertia(0);
    p.inertiaYY = inertia(1);
    p.inertiaZZ = inertia(2);
    p.tvcThrustLimit_N = thrustLimit;
    p.tvcAngleLimit_rad = angleLimit;
    p.applyActuatorRateLimits = false;
    return p;
  }

  LegacyWiring(float mass, Math::Vector<float, 3> inertiaTensor, float tvcThrustLimit_N, float tvcAngleLimit_Rad)
      : model_(vehicleParams(mass, inertiaTensor, tvcThrustLimit_N, tvcAngleLimit_Rad)) {
    tvcForce_ = {0, 0, 0};
    attitudeState_ = {0, 0, 0, 1, 0, 0, 0};
    positionState_ = {0, 0, 0, 0, 0, 0};
  }

  void setTVCInputTopic(float tvcZ, float tvcAngleLimit, float tvcForceLimit) {
    model_.params().tvcAngleLimit_rad = tvcAngleLimit;
    model_.params().tvcThrustLimit_N = tvcForceLimit;
    model_.params().tvcFromCg_m = tvcZ;
  }

  void resetToStartPose() {
    model_.resetToStartPose();
    model_.clearTimeCarry();
    attitudeState_ = model_.attitudeState();
    positionState_ = model_.positionState();
  }

  void update(int64_t now, const Math::Vector<float, 4> *tvcNew, const float *flapNew /*[4] or null*/) {
    const int64_t elapsedNs = now - stateTimestamp_;
    stateTimestamp_ = now;

    if (tvcNew) {
      const auto in = *tvcNew;
      tvcForce_ = {in(0), in(1), in(2)};
      tvcRollTorque_ = in(3);
    }
    model_.setThrust(tvcForce_, tvcRollTorque_, tvcEnabled_);

    if (flapNew)
      model_.setFlapInput(flapNew[0], flapNew[1], flapNew[2], flapNew[3]);

    if (elapsedNs > 0 && elapsedNs <= kMaxGapNs) {
      model_.advance(float(double(elapsedNs) / 1e9));
    } else {
      model_.clearTimeCarry();
    }

    if (zeroingMode_) {
      model_.resetToStartPose();
    }

    attitudeState_ = model_.attitudeState();
    positionState_ = model_.positionState();
  }
};

/// Legacy wiring and the adapter side by side, driven identically. Compares after every run.
struct Duo {
  LegacyWiring lg;
  Adapter ad;
  int64_t now = 0;
  long runs = 0, steps = 0, lastSteps = 0, lidarReadings = 0, lidarMisses = 0;
  const char *name;

  explicit Duo(const char *n)
      : lg(kMass, {0.044, 0.044, 0.004}, kThrustLimit, kAngleLimit),
        ad(kMass, {0.044, 0.044, 0.004}, kThrustLimit, kAngleLimit), name(n) {
    lg.setTVCInputTopic(kTvcZ, kAngleLimit, kThrustLimit);
    ad.configureTvc(kAngleLimit, kThrustLimit, kTvcZ);
    lg.stateTimestamp_ = 0; // taskInit sets the time of the first run: both start() at 0
    ad.start(0);
    expectSame("construct");
  }

  void fail(const char *what, const char *field) {
    char b[200];
    snprintf(b, sizeof(b), "%s: run %ld t=%.3f s: %s differs (%s)", name, runs, double(now) / 1e9, field, what);
    TEST_FAIL_MESSAGE(b);
  }

  void expectSame(const char *what) {
    if (!sameBits(lg.attitudeState_, ad.getAttitudeState()))
      fail(what, "attitude");
    if (!sameBits(lg.positionState_, ad.getPositionState()))
      fail(what, "position");
    if (lg.stateTimestamp_ != ad.stateTimestamp())
      fail(what, "timestamp");
    // Ground pair as StarshipSimTask::taskThread publishes it: [vertical velocity, height].
    const Math::Vector<float, 2> g0 = {lg.positionState_(2), lg.positionState_(5)};
    const Math::Vector<float, 2> g1 = {ad.getPositionState()(2), ad.getPositionState()(5)};
    if (!sameBits(g0, g1))
      fail(what, "ground");
    float d0 = -1, d1 = -2;
    const bool v0 = simLidarReading(g0(1), d0), v1 = simLidarReading(g1(1), d1);
    if (v0 != v1 || (v0 && !sameBits(d0, d1)))
      fail(what, "lidar");
    (v0 ? lidarReadings : lidarMisses)++;
    // Deeper state: actuators, time, wind/gust settings, and the held inputs.
    const auto &x = ad.sim().state();
    for (int i = 0; i < 4; i++) {
      const float a = lg.model_.flapAngle(i), b = x.flaps.angle[i];
      if (!sameBits(a, b))
        fail(what, "flap angle");
      // The plant takes its input inside a step: compare the held input only after a run that stepped (the legacy
      // model took it in every run, unobservable without a step).
      if (steps != lastSteps && !sameBits(lg.model_.flapInput_[i], x.flaps.input[i]))
        fail(what, "flap input");
    }
    if (!sameBits(lg.model_.time(), x.t_s))
      fail(what, "time");
    if (!sameBits(lg.model_.params().windSpeed_mps, ad.windSpeed()) ||
        !sameBits(lg.model_.params().windDirection_rad, ad.windDirection()) ||
        !sameBits(lg.model_.params().gustStrength_mps, ad.gustStrength()) ||
        !sameBits(lg.model_.params().gustPeriod_s, ad.gustPeriod()) || lg.model_.params().gustSeed != ad.gustSeed())
      fail(what, "wind/gust getters");
  }

  // ---- identical actions on both sides ----
  void enableTVC(bool e) {
    lg.tvcEnabled_ = e;
    ad.enableTVC(e);
  }
  void enableFlaps(bool e) {
    lg.flapsEnabled_ = e;
    ad.enableFlaps(e);
  }
  void enableZeroing(bool e) {
    lg.zeroingMode_ = e;
    ad.enableZeroingMode(e);
  }
  void reset() {
    lg.resetToStartPose();
    ad.resetToStartPose();
    expectSame("reset");
  }
  void setWind(float s, float d) {
    lg.model_.setWind(s, d);
    ad.setWind(s, d);
    expectSame("setWind");
  }
  void setGust(float s, float p, uint32_t seed) {
    lg.model_.setGust(s, p, seed);
    ad.setGust(s, p, seed);
    expectSame("setGust");
  }
  void clearWind() {
    lg.model_.clearWind();
    ad.clearWind();
    expectSame("clearWind");
  }

  /// One run of both tasks at wall clock `now + dtNs`. `tvc` / `flap` are the new topic items (null = none).
  void run(int64_t dtNs, const Math::Vector<float, 4> *tvc, const float *flap) {
    now += dtNs;
    lg.update(now, tvc, flap);
    if (tvc)
      ad.setTvcInput(*tvc);
    if (flap)
      ad.setFlapInput(flap[0], flap[1], flap[2], flap[3]);
    ad.update(now);
    runs++;
    steps = ad.sim().ticks();
    expectSame("update");
    lastSteps = steps;
  }
  void runMany(int n, int64_t dtNs, const Math::Vector<float, 4> *tvc, const float *flap) {
    for (int i = 0; i < n; i++)
      run(dtNs, i == 0 ? tvc : nullptr, i == 0 ? flap : nullptr);
  }
};

const float kFolded[4] = {1.5708f, 1.5708f, 1.5708f, 1.5708f};

} // namespace

// Thrust below the gate, gate closed (zero sim thrust), gate open, hover lift-off, thrust cut, fall and landing; the
// lidar must be valid below 1 m and absent above, on both sides.
void test_gate_thrust_landing_lidar() {
  Duo d("gate/landing");
  Math::Vector<float, 4> zero = {0, 0, 0, 0};
  d.run(10 * kMs, &zero, kFolded);
  d.enableTVC(false); // motor gate closed: the simulated thrust never acts
  Math::Vector<float, 4> t = {0, 0, 12.0f, 0.01f};
  d.runMany(100, 10 * kMs, &t, nullptr);
  TEST_ASSERT_TRUE(d.ad.getPositionState()(5) == 0.0f); // nothing moved: on the ground
  d.enableTVC(true);                                     // gate open
  Math::Vector<float, 4> tiny = {0, 0, 1e-7f, 0};       // below the 1e-6 N threshold of the plant
  d.runMany(50, 10 * kMs, &tiny, nullptr);
  Math::Vector<float, 4> lift = {0, 0, 13.0f, 0.02f}; // lifts off (13 N > m g = 10.8 N)
  d.runMany(300, 10 * kMs, &lift, nullptr);
  TEST_ASSERT_TRUE(d.ad.getPositionState()(5) > 0.5f);
  d.runMany(100, 10 * kMs, &zero, nullptr); // thrust cut: falls back
  d.runMany(300, 10 * kMs, nullptr, nullptr);
  TEST_ASSERT_TRUE(d.ad.getPositionState()(5) <= 0.01f); // landed and resting
  TEST_ASSERT_TRUE(d.lidarReadings > 20 && d.lidarMisses > 20);
  TEST_ASSERT_TRUE(d.steps > 1000);
}

// Reading below 1 m: every height below the range reproduces, the cut-off itself, and non-finite heights.
void test_lidar_function_unchanged() {
  float d = 0;
  TEST_ASSERT_TRUE(simLidarReading(0.999f, d) && d == 0.999f);
  TEST_ASSERT_FALSE(simLidarReading(1.0f, d));
  TEST_ASSERT_FALSE(simLidarReading(NAN, d));
}

// Flap faults on the true flap topic (stuck 42 deg, others moving), folded start, out-of-range values and NaN per channel.
void test_flap_inputs() {
  Duo d("flaps");
  Math::Vector<float, 4> hover = {0, 0, 10.8f, 0};
  d.run(10 * kMs, &hover, nullptr);       // flaps never received: the plant keeps the folded start value
  d.enableTVC(true);
  d.runMany(30, 10 * kMs, nullptr, nullptr);
  float f1[4] = {42 * kDeg, 0.3f, 1.2f, 1.5f};
  d.runMany(40, 10 * kMs, nullptr, f1);
  float f2[4] = {42 * kDeg, 2.0f, -0.4f, 0.0f}; // out of range: clamped to 0..pi/2
  d.runMany(40, 10 * kMs, nullptr, f2);
  float f3[4] = {NAN, 0.7f, NAN, INFINITY}; // non-finite channels keep the previous value
  d.runMany(40, 10 * kMs, nullptr, f3);
  TEST_ASSERT_EQUAL_FLOAT(42 * kDeg, d.ad.sim().state().flaps.input[0]); // NaN kept the last valid value
  TEST_ASSERT_EQUAL_FLOAT(1.5707963f, d.ad.sim().state().flaps.input[3] < 1.5708f ? 1.5707963f : d.ad.sim().state().flaps.input[3]);
  float f4[4] = {0.9f, NAN, 0.1f, 1.0f};
  d.runMany(40, 10 * kMs, nullptr, f4);
}

// "Invalid input then valid input": the last valid flap angle per channel is held between and across runs, several set
// calls between two runs keep the last valid per channel; a NaN thrust is zero thrust until the next valid one.
void test_last_valid_input_held() {
  Duo d("hold");
  d.enableTVC(true);
  Math::Vector<float, 4> hover = {0, 0, 11.0f, 0};
  float a[4] = {0.2f, 0.4f, 0.6f, 0.8f};
  d.run(10 * kMs, &hover, a);
  // Two items between two runs: the second has NaN channels, the plant must see the first's values there.
  float b1[4] = {0.3f, 0.5f, 0.7f, 0.9f};
  float b2[4] = {NAN, 0.1f, NAN, 0.2f};
  d.ad.setFlapInput(b1[0], b1[1], b1[2], b1[3]);
  d.ad.setFlapInput(b2[0], b2[1], b2[2], b2[3]);
  d.lg.model_.setFlapInput(b1[0], b1[1], b1[2], b1[3]);
  d.lg.model_.setFlapInput(b2[0], b2[1], b2[2], b2[3]);
  d.run(10 * kMs, nullptr, nullptr);
  TEST_ASSERT_EQUAL_FLOAT(0.3f, d.ad.sim().state().flaps.input[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.1f, d.ad.sim().state().flaps.input[1]);
  TEST_ASSERT_EQUAL_FLOAT(0.7f, d.ad.sim().state().flaps.input[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.2f, d.ad.sim().state().flaps.input[3]);
  float c[4] = {NAN, NAN, NAN, NAN};
  d.runMany(20, 10 * kMs, nullptr, c);
  TEST_ASSERT_EQUAL_FLOAT(0.3f, d.ad.sim().state().flaps.input[0]);
  // NaN and infinite thrust: zero thrust (no motion from it), then valid again.
  Math::Vector<float, 4> bad = {NAN, 0, 11.0f, 0};
  d.runMany(30, 10 * kMs, &bad, nullptr);
  Math::Vector<float, 4> bad2 = {0, 0, INFINITY, NAN};
  d.runMany(30, 10 * kMs, &bad2, nullptr);
  d.runMany(30, 10 * kMs, &hover, nullptr);
  float e[4] = {0.5f, 0.5f, 0.5f, 0.5f};
  d.runMany(30, 10 * kMs, nullptr, e);
}

// Wind and gust changes mid-run, non-finite values (leave the setting unchanged), clearWind (keeps the gust spread, a
// legacy quirk), and the getters used by main.cpp.
void test_wind_and_gust_midrun() {
  Duo d("wind");
  d.enableTVC(true);
  Math::Vector<float, 4> lift = {0, 0, 12.0f, 0};
  float fl[4] = {0.5f, 0.5f, 0.5f, 0.5f};
  d.run(10 * kMs, &lift, fl);
  d.runMany(50, 10 * kMs, nullptr, nullptr);
  d.setWind(3.0f, 0.7f);
  d.runMany(50, 10 * kMs, nullptr, nullptr);
  d.setGust(3.0f, 4.0f, 7);
  d.runMany(100, 10 * kMs, nullptr, nullptr);
  d.setWind(NAN, 1.9f);
  d.setWind(-2.0f, NAN);
  d.setGust(NAN, NAN, 9);
  d.setGust(50.0f, 100.0f, 0); // clamped
  d.runMany(50, 10 * kMs, nullptr, nullptr);
  d.clearWind();
  d.runMany(50, 10 * kMs, nullptr, nullptr);
  d.setGust(2.0f, 1.0f, 123456789u);
  d.setWind(5.0f, 3.0f);
  d.runMany(100, 10 * kMs, nullptr, nullptr);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, d.ad.windSpeed());
  TEST_ASSERT_EQUAL_UINT32(123456789u, d.ad.gustSeed());
}

// Reset to the start pose (SE press, simulation start) and zeroing mode, also while moving; time and actuators
// continue; carry cleared.
void test_reset_and_zeroing() {
  Duo d("reset");
  // Start pose before any run.
  TEST_ASSERT_EQUAL_FLOAT(1.0f, d.ad.getAttitudeState()(3));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.ad.getPositionState()(5));
  d.enableTVC(true);
  Math::Vector<float, 4> lift = {0.3f, 0.2f, 12.5f, 0.1f};
  float fl[4] = {0.3f, 0.6f, 0.9f, 1.2f};
  d.run(10 * kMs, &lift, fl);
  d.runMany(150, 10 * kMs, nullptr, nullptr);
  TEST_ASSERT_TRUE(d.ad.getPositionState()(5) > 0.1f);
  d.reset();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.ad.getPositionState()(5));
  d.runMany(80, 10 * kMs, nullptr, nullptr);
  d.enableZeroing(true); // held at the start pose every run
  d.runMany(80, 10 * kMs, nullptr, nullptr);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.ad.getPositionState()(5));
  d.enableZeroing(false);
  d.runMany(80, 10 * kMs, nullptr, nullptr);
  d.reset();
  d.runMany(20, 10 * kMs, nullptr, nullptr);
}

// Timing: irregular periods, zero elapsed, a gap above 100 ms (not integrated), exactly 100 ms, time going backwards,
// and the first run after start.
void test_timing_rules() {
  Duo d("timing");
  d.enableTVC(true);
  Math::Vector<float, 4> lift = {0, 0, 12.0f, 0};
  float fl[4] = {0.5f, 0.5f, 0.5f, 0.5f};
  d.run(10 * kMs, &lift, fl);
  const int64_t seq[] = {10, 11, 9, 13, 8, 10, 0, 10, 99, 100, 101, 250, 10, 10, 1, 2, 3, 5, 7, 49, 50, 51, 55, 60, 10};
  for (int64_t ms : seq)
    d.run(ms * kMs, nullptr, nullptr);
  d.run(-5 * kMs, nullptr, nullptr); // clock going backwards: not integrated
  d.run(10 * kMs, nullptr, nullptr);
  d.runMany(100, 10 * kMs, nullptr, nullptr);
  // Several runs without a start(): a fresh pair whose first run has a huge elapsed time (task never started).
  Duo e("timing-first");
  e.run(5000 * kMs, nullptr, nullptr);
  e.run(10 * kMs, nullptr, nullptr);
}

// Seeded random fuzz: all inputs, toggles, wind, reset, zeroing and timing mixed.
void test_random_scenarios() {
  for (uint32_t seed = 1; seed <= 24; seed++) {
    Rng r(seed * 7919u);
    char nm[32];
    snprintf(nm, sizeof(nm), "fuzz seed %u", unsigned(seed));
    Duo d(nm);
    d.enableTVC(r.coin());
    for (int k = 0; k < 1500; k++) {
      Math::Vector<float, 4> tvc;
      float fl[4];
      bool haveTvc = r.pick(4) == 0, haveFl = r.pick(5) == 0;
      if (haveTvc) {
        const int kind = r.pick(10);
        const float mag = kind == 0 ? 20.0f : r.uni(0.0f, 13.0f);
        tvc = {r.uni(-0.6f, 0.6f) * mag * 0.3f, r.uni(-0.6f, 0.6f) * mag * 0.3f, mag, r.uni(-0.05f, 0.05f)};
        if (kind == 1)
          tvc(r.pick(4)) = NAN;
        if (kind == 2)
          tvc = {0, 0, 1e-7f, 0};
        if (kind == 3)
          tvc = {0, 0, 0, 0};
        if (kind == 4)
          tvc(2) = INFINITY;
      }
      if (haveFl) {
        for (int i = 0; i < 4; i++) {
          fl[i] = r.uni(-0.2f, 1.9f);
          if (r.pick(12) == 0)
            fl[i] = NAN;
        }
      }
      switch (r.pick(60)) {
      case 0: d.enableTVC(r.coin()); break;
      case 1: d.enableFlaps(r.coin()); break;
      case 2: d.enableZeroing(r.coin()); break;
      case 3: d.reset(); break;
      case 4: d.setWind(r.pick(6) == 0 ? NAN : r.uni(-1.0f, 8.0f), r.pick(6) == 0 ? NAN : r.uni(-4.0f, 4.0f)); break;
      case 5: d.setGust(r.pick(6) == 0 ? NAN : r.uni(0.0f, 12.0f), r.pick(6) == 0 ? NAN : r.uni(0.1f, 40.0f), r.next()); break;
      case 6: d.clearWind(); break;
      default: break;
      }
      const int tk = r.pick(30);
      const int64_t dt = tk == 0 ? 0 : tk == 1 ? 130 * kMs : tk == 2 ? -3 * kMs : tk == 3 ? 100 * kMs : int64_t(r.uni(7.0f, 14.0f) * 1e6f);
      d.run(dt, haveTvc ? &tvc : nullptr, haveFl ? fl : nullptr);
    }
  }
}

// The adapter keeps the vehicle's limits set through configureTvc, and its defaults equal BodySimParams.
void test_vehicle_configuration() {
  Adapter a(kMass, {0.044, 0.044, 0.004}, kThrustLimit, kAngleLimit);
  a.configureTvc(0.04f, 9.0f, -0.4f);
  TEST_ASSERT_EQUAL_FLOAT(0.04f, a.config().p.tvc.angleLimit_rad);
  TEST_ASSERT_EQUAL_FLOAT(9.0f, a.config().p.tvc.thrustLimit_N);
  TEST_ASSERT_EQUAL_FLOAT(-0.4f, a.config().p.tvc.fromCg_m);
  TEST_ASSERT_FALSE(a.config().p.applyActuatorModels);
  Legacy lg(LegacyWiring::vehicleParams(kMass, {0.044, 0.044, 0.004}, kThrustLimit, kAngleLimit));
  TEST_ASSERT_EQUAL_FLOAT(lg.params().aeroCalibration, a.config().p.aeroCalibration);
  TEST_ASSERT_EQUAL_FLOAT(lg.params().step_s, a.config().p.step_s);
  TEST_ASSERT_EQUAL_FLOAT(lg.params().maxAdvance_s, a.config().p.maxAdvance_s);
  TEST_ASSERT_TRUE(sizeof(Adapter) < 1600);
  printf("SIMSIZE StarshipSimAdapter %u B (config %u, sim %u)\n", unsigned(sizeof(Adapter)),
         unsigned(sizeof(Adapter::Plant::Config)), unsigned(sizeof(Dyn::Sim<Adapter::Plant>)));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_gate_thrust_landing_lidar);
  RUN_TEST(test_lidar_function_unchanged);
  RUN_TEST(test_flap_inputs);
  RUN_TEST(test_last_valid_input_held);
  RUN_TEST(test_wind_and_gust_midrun);
  RUN_TEST(test_reset_and_zeroing);
  RUN_TEST(test_timing_rules);
  RUN_TEST(test_random_scenarios);
  RUN_TEST(test_vehicle_configuration);
  return UNITY_END();
}
