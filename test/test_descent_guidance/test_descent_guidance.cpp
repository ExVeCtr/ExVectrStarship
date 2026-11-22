// Descent (belly-down freefall) control study: pio test -e native -f test_descent_guidance -v | grep DESCENT
//
// Closed loop: DSP::BodySimModel (lib/ExVectrDSP, flap servos 300 deg/s, wind and seeded gusts) + estimator noise +
// a flap controller at 50 Hz, from the kick end at 120 m down to the freefall stop trigger (MissionFreefall formula:
// z < 0.5 vz^2 / (14/1.1 - 9.81) + 20). Target (0,0). Strategies:
//   L  legacy: FlapModeObserverTask guidance (10 Hz) + ControlAttitudeFlaps Stabilize (verbatim replica, 50/40 deg)
//   A  legacy attitude law, fixed guidance (bearing with hold radius, pitch from distance and closing speed, all
//      altitudes)
//   B  recommended: DescentGuidance (funnel -> velocity PI -> heading + glide) + FlapDescentLaw (30/78 deg)
//   Bs B limited to the yaw-safe glide envelope (bottom flaps >= 70 deg, glide <= 3.3 m/s)
//   Bx B with the nose axis taken mod 180 deg (fly backward when the target is behind)
//   C  B's guidance, heading free: lateral command through roll (tilt only)
// Metrics: miss distance at the stop trigger (flip start), horizontal speed there, heading/tilt rate rms, flap
// saturation and activity, max belly angle. DESCENT_SEEDS=<n> (default 2, about 20 s; the design doc uses 20, about 3.5 min) sets the seeds per
// matrix cell. Report-only except the acceptance of B (asserted).
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unity.h>

#include <algorithm>
#include <vector>

#include "ExVectrDSP/body_sim_model.hpp"
#include "descent_control.hpp"

using namespace VCTR;
using namespace VCTR::Math;
using DSP::BodySimModel;
using DSP::BodySimParams;
namespace D = VCTR::CTRL::descent;

void setUp() {}
void tearDown() {}

static constexpr float kDeg = 3.14159265f / 180.0f;
static constexpr float kPi = 3.14159265f;

// ---------------------------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------------------------
static Vector<float, 7> attOf(const Quat_F &q) { return {0, 0, 0, q(0), q(1), q(2), q(3)}; }
static float wrapDeg(float a) {
  while (a > 180)
    a -= 360;
  while (a < -180)
    a += 360;
  return a;
}
static Quat_F bellyDown(float pitchDeg = 0, float rollDeg = 0) { // nose toward world -X (azimuth 180 deg)
  Quat_F q = Quat_F(Vector_F{0, 1, 0}, -90 * kDeg) * Quat_F(Vector_F{0, 1, 0}, pitchDeg * kDeg);
  return q * Quat_F(Vector_F{0, 0, 1}, rollDeg * kDeg);
}

struct Rng {
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) {
    if (!s)
      s = 1;
  }
  uint32_t next() {
    s ^= s << 13, s ^= s >> 17, s ^= s << 5;
    return s;
  }
  float uni() { return (next() >> 8) * (1.0f / 16777216.0f); }
  float gauss() {
    float u1 = uni(), u2 = uni();
    if (u1 < 1e-7f)
      u1 = 1e-7f;
    return sqrtf(-2 * logf(u1)) * cosf(2 * kPi * u2);
  }
};
/// First-order Gauss-Markov process with stationary sigma and correlation time tau.
struct GaussMarkov {
  float x = 0, sigma = 0, tau = 1;
  void init(Rng &r, float sig, float t) {
    sigma = sig, tau = t;
    x = sig * r.gauss();
  }
  float step(Rng &r, float dt) {
    const float a = expf(-dt / tau);
    x = a * x + sigma * sqrtf(1 - a * a) * r.gauss();
    return x;
  }
};

// ---------------------------------------------------------------------------------------------------------------
// Legacy replica: FlapModeObserverTask (src/main.cpp, 10 Hz) + ControlAttitudeFlaps Stabilize (50 Hz)
// ---------------------------------------------------------------------------------------------------------------
struct LegacyAttitude { // verbatim law of control_attitude_flaps.cpp (gains 6e07d38, neutral 50/40)
  float top = 50 * kDeg, bottom = 40 * kDeg;
  float azSet = 0, pitchSet = 0;
  float out[4];
  void update(const float att[7]) {
    D::BellyAttitude b = D::bellyAttitude(att);
    float ff = (b.belly - 35 * kDeg) / (20 * kDeg);
    ff = ff < 0 ? 0 : (ff > 1 ? 1 : ff);
    ff = 1 - ff;
    float ea = azSet - b.heading;
    if (ea > kPi)
      ea -= 2 * kPi;
    else if (ea < -kPi)
      ea += 2 * kPi;
    float u0 = ea * 0.3f, u1 = (pitchSet - b.pitch) * 1.2f;
    u0 = u0 > 0.5f ? 0.5f : (u0 < -0.5f ? -0.5f : u0);
    u0 -= att[0] * 0.2f;
    u1 -= att[1] * 0.4f;
    float u2 = -att[2] * 0.1f;
    out[0] = top + (-u0 - u1 + u2) * ff;
    out[1] = top + (u0 - u1 - u2) * ff;
    out[2] = bottom + (u0 + u1 + u2) * ff;
    out[3] = bottom + (-u0 + u1 - u2) * ff;
    for (int i = 0; i < 4; i++)
      out[i] = D::clampf(out[i], 15 * kDeg, 90 * kDeg);
  }
};
static float stopAltitude(float vz) { return 0.5f * vz * vz / (14.0f / 1.1f - 9.81f) + 20.0f; } // incl. stopAlt 20
struct LegacyGuidance {
  void update(const float pos[6], LegacyAttitude &a) {
    a.pitchSet = 0;
    const float azTowardsZero = atan2f(pos[4], -pos[3]);
    const float dist = sqrtf(pos[3] * pos[3] + pos[4] * pos[4]);
    if (pos[5] > 100) {
      float angle = -(dist - 1) * 1;
      angle = angle > 0 ? 0 : (angle < -40 ? -40 : angle);
      a.azSet = -azTowardsZero;
      a.pitchSet = (angle + 20) * kDeg;
    } else if (pos[5] < stopAltitude(pos[2]) + 20) {
      a.pitchSet = 30 * kDeg;
    }
  }
};
/// Strategy A: legacy attitude law, guidance fixed (no altitude gate, hold radius, closing-speed damping).
struct FixedGuidance {
  bool hold = false;
  void update(const float pos[6], LegacyAttitude &a) {
    const float ex = -pos[3], ey = -pos[4];
    const float d = sqrtf(ex * ex + ey * ey);
    if (d < 3)
      hold = true;
    else if (d > 5)
      hold = false;
    if (!hold)
      a.azSet = atan2f(ey, ex);
    const float vToward = d > 0.1f ? (pos[0] * ex + pos[1] * ey) / d : 0;
    const float along = hold ? (ex * cosf(a.azSet) + ey * sinf(a.azSet)) : d; // signed distance along the az
    float p = -(2.0f * along - 4.0f * vToward); // deg: negative = nose down = glide toward the nose
    a.pitchSet = D::clampf(p, -20, 20) * kDeg;
  }
};

// ---------------------------------------------------------------------------------------------------------------
// Scenario runner
// ---------------------------------------------------------------------------------------------------------------
enum class Strat { L, A, B, Bx, Bs, C };
static const char *stratName(Strat s) {
  switch (s) {
  case Strat::L: return "L ";
  case Strat::A: return "A ";
  case Strat::B: return "B ";
  case Strat::Bx: return "Bx";
  case Strat::Bs: return "Bs";
  default: return "C ";
  }
}

struct Scenario {
  float offset = 10, headingErrDeg = 0, wind = 0, gust = 3;
  uint32_t seed = 1;
  float aeroScale = 1.0f; ///< multiplies aeroCalibration
  bool shield = true;     ///< false: fold shield off, calibration 4.00 (the other sign-consistent model)
  bool noise = true;
  float apogee = 120;
  float targetX = 0, targetY = 0;
  float betaDeg = NAN; ///< bearing vehicle -> target; NAN = from the seed
};
struct Metrics {
  float miss = 0, vhEnd = 0, zEnd = 0, tEnd = 0;
  float headRateRms = 0, tiltRateRms = 0, satFrac = 0, flapAct = 0, bellyMax = 0;
  float headRateLate = 0, tiltRateLate = 0; ///< rms over the second half of the descent (oscillation, not turns)
  bool timeout = false;
};

static bool isLegacyLaw(Strat s) { return s == Strat::L || s == Strat::A; }

static Metrics runScenario(Strat st, const Scenario &sc, bool trace = false) {
  BodySimParams p;
  p.applyActuatorRateLimits = true;
  if (!sc.shield) {
    p.flapFoldShieldFactor = 0;
    p.aeroCalibration = 4.00f;
  }
  p.aeroCalibration *= sc.aeroScale;
  BodySimModel m(p);
  Rng rng(sc.seed);
  float beta = rng.uni() * 2 * kPi; // bearing from the vehicle to the target
  if (isfinite(sc.betaDeg))
    beta = sc.betaDeg * kDeg;
  const float windDir = rng.uni() * 2 * kPi;
  const float psi0 = beta + sc.headingErrDeg * kDeg;
  const float px = sc.targetX - sc.offset * cosf(beta), py = sc.targetY - sc.offset * sinf(beta);
  Quat_F q = Quat_F(Vector_F{0, 0, 1}, psi0 - kPi) * bellyDown(5, 0);
  Vector<float, 7> a0 = attOf(q);
  a0(1) = 0.3f; // kick end: residual pitch rate
  m.setAttitudeState(a0);
  m.setPositionState({3.6f * cosf(psi0), 3.6f * sinf(psi0), 0, px, py, sc.apogee});
  if (sc.wind > 0) {
    m.setWind(sc.wind, windDir);
    if (sc.gust > 0)
      m.setGust(sc.gust, 5.0f, sc.seed + 1000);
  }
  GaussMarkov np[3], nv[3], nh;
  for (int i = 0; i < 3; i++) {
    np[i].init(rng, sc.noise ? 0.3f : 0, 2.0f);
    nv[i].init(rng, sc.noise ? 0.2f : 0, 0.3f);
  }
  nh.init(rng, sc.noise ? 2 * kDeg : 0, 5.0f);

  LegacyAttitude la;
  LegacyGuidance lg;
  FixedGuidance fg;
  D::DescentGuidanceParams gp;
  if (st == Strat::Bx)
    gp.forwardOnlyHeading = false;
  D::FlapDescentLawParams lp;
  if (st == Strat::Bs) { // yaw-safe glide envelope only: bottom pair >= 70 deg, glide <= 3.3 m/s
    gp.mapN = 12;
    gp.glideMax_mps = 3.3f;
    gp.vMax_mps = 3.0f;
    lp.splitMin_rad = -68 * kDeg;
  }
  D::DescentGuidanceMem gm;
  D::FlapDescentLawMem lm;
  D::DescentTarget tgt;
  tgt.x_m = sc.targetX, tgt.y_m = sc.targetY, tgt.valid = true;
  bool guidInit = false;

  float cmd[4] = {90 * kDeg, 90 * kDeg, 90 * kDeg, 90 * kDeg}, prevCmd[4];
  memcpy(prevCmd, cmd, sizeof cmd);
  Metrics r;
  double sh = 0, stl = 0, act = 0;
  std::vector<double> secH, secT; // per-second sums of squared rates (for the late window)
  std::vector<int> secN;
  int nRate = 0, nCtl = 0, nSat = 0;
  const float dt = p.step_s;
  const int maxSteps = int(60.0f / dt);
  float estPos[6] = {0}, estAtt[7] = {0};
  D::DescentGuidanceOut go;
  for (int i = 0; i < maxSteps; i++) {
    const float t = i * dt;
    // estimator (every 10 ms): truth + coloured noise; heading noise as a rotation about world up
    if (i % 5 == 0) {
      auto ps = m.positionState();
      auto as = m.attitudeState();
      for (int k = 0; k < 3; k++) {
        estPos[k] = ps(k) + nv[k].step(rng, 0.01f);
        estPos[3 + k] = ps(3 + k) + np[k].step(rng, 0.01f);
      }
      const float hn = nh.step(rng, 0.01f);
      Quat_F qt(as(3), as(4), as(5), as(6));
      Quat_F qn = Quat_F(Vector_F{0, 0, 1}, hn) * qt;
      for (int k = 0; k < 3; k++)
        estAtt[k] = as(k) + (sc.noise ? 0.01f * rng.gauss() : 0);
      for (int k = 0; k < 4; k++)
        estAtt[3 + k] = qn(k);
      tgt.endAltitude_m = stopAltitude(estPos[2]);
      // freefall stop trigger (MissionFreefall, every 10 ms)
      if (t > 1.0f && estPos[5] < stopAltitude(estPos[2])) {
        const float ex = ps(3) - sc.targetX, ey = ps(4) - sc.targetY;
        r.miss = sqrtf(ex * ex + ey * ey);
        r.vhEnd = sqrtf(ps(0) * ps(0) + ps(1) * ps(1));
        r.zEnd = ps(5), r.tEnd = t;
        break;
      }
    }
    // legacy guidance at 10 Hz
    if (i % 50 == 0) {
      if (st == Strat::L)
        lg.update(estPos, la);
      else if (st == Strat::A)
        fg.update(estPos, la);
    }
    // controller at 50 Hz
    if (i % 10 == 0) {
      if (isLegacyLaw(st)) {
        la.update(estAtt);
        memcpy(cmd, la.out, sizeof cmd);
      } else {
        if (!guidInit) {
          D::DescentGuidance::init(gp, estAtt, gm);
          guidInit = true;
        }
        go = D::DescentGuidance::step(gp, tgt, estPos, estAtt, 0.02f, gm);
        D::FlapDescentCmd c;
        c.headingSp_rad = go.headingSp_rad;
        c.pitchSplit_rad = go.pitchSplit_rad;
        if (st == Strat::C) {
          // heading free: along-nose part -> split, perpendicular part -> roll (body +Y drift for + roll)
          D::BellyAttitude b = D::bellyAttitude(estAtt);
          const float nx = cosf(b.heading), ny = sinf(b.heading);
          const float yx = ny, yy = -nx;
          const float along = go.vAirCmd[0] * nx + go.vAirCmd[1] * ny;
          const float perp = go.vAirCmd[0] * yx + go.vAirCmd[1] * yy;
          c.headingFree = true;
          c.pitchSplit_rad = D::DescentGuidance::glideToSplit(gp, D::clampf(along, gp.glideMin_mps, gp.glideMax_mps));
          c.rollSp_rad = D::clampf(0.1f * perp, -0.4f, 0.4f);
        }
        D::FlapDescentLawOut o = D::FlapDescentLaw::step(lp, estAtt, c, lm);
        memcpy(cmd, o.flap, sizeof cmd);
      }
      bool sat = false;
      for (int k = 0; k < 4; k++) {
        sat = sat || cmd[k] <= 15.01f * kDeg || cmd[k] >= 89.99f * kDeg;
        if (t > 2.0f)
          act += fabsf(cmd[k] - prevCmd[k]) / 0.02f / kDeg / 4;
        prevCmd[k] = cmd[k];
      }
      if (t > 2.0f) {
        nSat += sat;
        nCtl++;
      }
      m.setFlapInput(cmd[0], cmd[1], cmd[2], cmd[3]);
    }
    m.step();
    auto as = m.attitudeState();
    if (t > 2.0f) {
      sh += as(0) * as(0);
      stl += as(1) * as(1) + as(2) * as(2);
      const size_t sec = size_t(t);
      if (secH.size() <= sec)
        secH.resize(sec + 1, 0), secT.resize(sec + 1, 0), secN.resize(sec + 1, 0);
      secH[sec] += as(0) * as(0), secT[sec] += as(1) * as(1) + as(2) * as(2), secN[sec]++;
      nRate++;
      float a7[7];
      for (int k = 0; k < 7; k++)
        a7[k] = as(k);
      r.bellyMax = std::max(r.bellyMax, D::bellyAttitude(a7).belly / kDeg);
    }
    if (trace && i % 500 == 0) {
      auto ps = m.positionState();
      float a7[7];
      for (int k = 0; k < 7; k++)
        a7[k] = as(k);
      D::BellyAttitude b = D::bellyAttitude(a7);
      float azSp = isLegacyLaw(st) ? la.azSet : go.headingSp_rad;
      float pSp = isLegacyLaw(st) ? la.pitchSet / kDeg : go.pitchSplit_rad / kDeg;
      printf("DESCENT trace %s t %5.1f z %6.1f x %+6.1f y %+6.1f d %5.1f vN %+5.1f vW %+5.1f vU %+5.1f | nose az %+5.0f "
             "sp %+5.0f | pitch %+5.1f %s %+5.1f | belly %3.0f roll %+4.0f | flaps %2.0f %2.0f %2.0f %2.0f\n",
             stratName(st), t, ps(5), ps(3), ps(4), sqrtf(ps(3) * ps(3) + ps(4) * ps(4)), ps(0), ps(1), ps(2),
             b.heading / kDeg, azSp / kDeg, b.pitch / kDeg, isLegacyLaw(st) ? "sp" : "split", pSp, b.belly / kDeg,
             b.roll / kDeg, m.flapAngle(0) / kDeg, m.flapAngle(1) / kDeg, m.flapAngle(2) / kDeg,
             m.flapAngle(3) / kDeg);
    }
    if (i == maxSteps - 1) {
      r.timeout = true;
      auto ps = m.positionState();
      r.miss = sqrtf(ps(3) * ps(3) + ps(4) * ps(4));
    }
  }
  {
    double h = 0, tl = 0;
    int n = 0;
    for (size_t k = secH.size() / 2; k < secH.size(); k++)
      h += secH[k], tl += secT[k], n += secN[k];
    r.headRateLate = n ? float(sqrt(h / n)) / kDeg : 0;
    r.tiltRateLate = n ? float(sqrt(tl / n)) / kDeg : 0;
  }
  r.headRateRms = nRate ? float(sqrt(sh / nRate)) / kDeg : 0;
  r.tiltRateRms = nRate ? float(sqrt(stl / nRate)) / kDeg : 0;
  r.satFrac = nCtl ? float(nSat) / nCtl : 0;
  r.flapAct = nCtl ? float(act / nCtl) : 0;
  return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Aggregation
// ---------------------------------------------------------------------------------------------------------------
struct Agg {
  std::vector<float> miss;
  double vh = 0, hr = 0, tr = 0, sat = 0, act = 0, hl = 0, tl = 0;
  float bellyMax = 0;
  int n = 0, timeouts = 0;
  void add(const Metrics &m) {
    miss.push_back(m.miss);
    vh += m.vhEnd, hr += m.headRateRms, tr += m.tiltRateRms, sat += m.satFrac, act += m.flapAct;
    hl += m.headRateLate, tl += m.tiltRateLate;
    bellyMax = std::max(bellyMax, m.bellyMax);
    n++;
    timeouts += m.timeout;
  }
  float pct(float q) const {
    std::vector<float> v = miss;
    std::sort(v.begin(), v.end());
    if (v.empty())
      return 0;
    size_t k = size_t(q * (v.size() - 1) + 0.5f);
    return v[k];
  }
  void print(const char *label) const {
    printf("DESCENT %-28s n %4d | miss med %5.1f p90 %5.1f max %5.1f m | vh %4.1f | rate rms head %5.1f tilt %5.1f, "
           "late %5.1f %5.1f deg/s | sat %3.0f %% | act %4.0f deg/s | belly max %3.0f%s\n",
           label, n, pct(0.5f), pct(0.9f), pct(1.0f), vh / n, hr / n, tr / n, hl / n, tl / n, 100 * sat / n, act / n,
           bellyMax,
           timeouts ? " TIMEOUT" : "");
  }
};

static int seedsPerCell() {
  const char *e = getenv("DESCENT_SEEDS");
  int n = e ? atoi(e) : 2;
  return n < 1 ? 1 : (n > 50 ? 50 : n);
}

static const float kOffsets[] = {0, 10, 30};
static const float kHeadings[] = {0, 45, 90, 135, 180};
static const float kWinds[] = {0, 3, 5};

struct MatrixResult {
  Agg all, calm, windy;
};
/// Full matrix for one strategy and model variant; prints one line per (offset, wind) and the totals.
static MatrixResult runMatrix(Strat st, float aeroScale, bool shield, bool noise, const char *variant, bool perCell = true) {
  const int seeds = seedsPerCell();
  MatrixResult r;
  Agg &all = r.all, &calm = r.calm, &windy = r.windy;
  for (float off : kOffsets)
    for (float w : kWinds) {
      Agg cell;
      for (float h : kHeadings)
        for (int s = 1; s <= seeds; s++) {
          Scenario sc;
          sc.offset = off, sc.headingErrDeg = h, sc.wind = w;
          sc.seed = uint32_t(s * 7919 + int(h) * 31 + int(off) * 977 + int(w) * 13);
          sc.aeroScale = aeroScale, sc.shield = shield, sc.noise = noise;
          Metrics m = runScenario(st, sc);
          cell.add(m);
          all.add(m);
          (w > 0 ? windy : calm).add(m);
        }
      if (perCell) {
        char lab[96];
        snprintf(lab, sizeof lab, "%s %s off %2.0f wind %1.0f", stratName(st), variant, off, w);
        cell.print(lab);
      }
    }
  char lab[96];
  snprintf(lab, sizeof lab, "%s %s ALL calm", stratName(st), variant);
  calm.print(lab);
  snprintf(lab, sizeof lab, "%s %s ALL wind", stratName(st), variant);
  windy.print(lab);
  snprintf(lab, sizeof lab, "%s %s ALL", stratName(st), variant);
  all.print(lab);
  return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------------------------------
/// Reproduction of the owner's report with the legacy replica: offset 10 m and 30 m, calm, no noise, trace.
static void test_reproduce_legacy() {
  for (float off : {10.0f, 30.0f}) {
    Scenario sc;
    sc.offset = off, sc.headingErrDeg = 0, sc.noise = false, sc.seed = 3;
    printf("DESCENT --- legacy reproduction, offset %.0f m, nose toward the target, calm, no noise ---\n", off);
    Metrics m = runScenario(Strat::L, sc, true);
    printf("DESCENT legacy offset %.0f: miss %.1f m at z %.1f m, t %.1f s, vh %.1f m/s, belly max %.0f\n", off, m.miss,
           m.zEnd, m.tEnd, m.vhEnd, m.bellyMax);
  }
  {
    // Owner's case: kick from yaw 0 ends with the nose South and 3.6 m/s toward South, 3 m South of home.
    Scenario sc;
    sc.offset = 3, sc.betaDeg = 0, sc.headingErrDeg = 180, sc.noise = false, sc.seed = 3;
    printf("DESCENT --- legacy reproduction, owner's case: 3 m South of home, nose South, calm, no noise ---\n");
    Metrics m = runScenario(Strat::L, sc, true);
    printf("DESCENT legacy owner case: miss %.1f m at z %.1f m, t %.1f s, vh %.1f m/s, belly max %.0f\n", m.miss, m.zEnd,
           m.tEnd, m.vhEnd, m.bellyMax);
    printf("DESCENT --- recommended B, owner's case ---\n");
    m = runScenario(Strat::B, sc, true);
    printf("DESCENT B owner case: miss %.1f m at z %.1f m, t %.1f s, vh %.1f m/s, belly max %.0f\n", m.miss, m.zEnd,
           m.tEnd, m.vhEnd, m.bellyMax);
  }
  Scenario sc;
  sc.offset = 10, sc.headingErrDeg = 0, sc.noise = false, sc.seed = 3;
  printf("DESCENT --- recommended B, offset 10 ---\n");
  Metrics m = runScenario(Strat::B, sc, true);
  printf("DESCENT B offset 10: miss %.1f m at z %.1f m, t %.1f s, vh %.1f m/s\n", m.miss, m.zEnd, m.tEnd, m.vhEnd);
}

/// Attitude law alone: heading steps of 90 and 175 deg at zero glide (split 0), calm, no noise, with and without
/// the fold shield. Prints the heading response (deg) and the peak belly angle.
static void headingStep(bool shield, float stepDeg, float split, bool print) {
  BodySimParams p;
  if (!shield) {
    p.flapFoldShieldFactor = 0;
    p.aeroCalibration = 4.00f;
  }
  BodySimModel m(p);
  m.setAttitudeState(attOf(bellyDown(0, 0)));
  m.setPositionState({0, 0, -6.4f, 0, 0, 500});
  D::FlapDescentLawParams lp;
  D::FlapDescentLawMem lm;
  D::FlapDescentCmd c;
  c.headingSp_rad = kPi + stepDeg * kDeg;
  c.pitchSplit_rad = split * kDeg;
  float bellyMax = 0, t90 = -1, over = 0;
  for (int i = 0; i < 5000; i++) {
    auto as = m.attitudeState();
    float a7[7];
    for (int k = 0; k < 7; k++)
      a7[k] = as(k);
    D::BellyAttitude b = D::bellyAttitude(a7);
    if (i % 10 == 0) {
      D::FlapDescentLawOut o = D::FlapDescentLaw::step(lp, a7, c, lm);
      m.setFlapInput(o.flap[0], o.flap[1], o.flap[2], o.flap[3]);
      if (print && i % 100 == 0)
        printf("DESCENT hstep %s step %+4.0f t %4.1f heading err %+6.1f pitch %+5.1f roll %+5.1f belly %4.1f w %+5.2f %+5.2f "
               "%+5.2f u %+5.1f %+5.1f %+5.1f\n",
               shield ? "shield" : "noshld", stepDeg, i * 0.002f, o.headingErr / kDeg, b.pitch / kDeg, b.roll / kDeg,
               b.belly / kDeg, a7[0], a7[1], a7[2], o.u[0] / kDeg, o.u[1] / kDeg, o.u[2] / kDeg);
    }
    const float e = wrapDeg((c.headingSp_rad - b.heading) / kDeg);
    if (t90 < 0 && fabsf(e) < 0.1f * fabsf(stepDeg))
      t90 = i * 0.002f;
    if (t90 >= 0)
      over = std::max(over, fabsf(e));
    bellyMax = std::max(bellyMax, b.belly / kDeg);
    m.step();
  }
  printf("DESCENT heading step %+4.0f deg, split %+3.0f, %s: 90 %% at %.2f s, residual max after %.1f deg, belly max %.0f\n",
         stepDeg, split, shield ? "shield 0.7" : "no shield", t90, over, bellyMax);
}

/// Free-body glide map along the law's pitch-split schedule (top = t0 - split; bottom = b0 + bottomSplit), both models.
static void test_glide_map() {
  D::FlapDescentLawParams lp;
  for (bool sh : {true, false}) {
    printf("DESCENT glide map %s, neutral %.0f/%.0f:", sh ? "shield 0.7" : "no shield", lp.neutralTop_rad / kDeg,
           lp.neutralBottom_rad / kDeg);
    const float sps[] = {-83, -80, -76, -72, -68, -64, -60, -50, -40, -30, -20, -12, -4, 0, 4, 8, 12};
    for (float sp : sps) {
      BodySimParams p;
      p.applyActuatorRateLimits = false;
      if (!sh) {
        p.flapFoldShieldFactor = 0;
        p.aeroCalibration = 4.00f;
      }
      BodySimModel m(p);
      const float t = D::clampf(lp.neutralTop_rad - sp * kDeg, 15 * kDeg, 90 * kDeg);
      const float b = D::clampf(lp.neutralBottom_rad + D::FlapDescentLaw::bottomSplit(lp, sp * kDeg), 15 * kDeg, 90 * kDeg);
      m.setFlapInput(t, t, b, b);
      m.setAttitudeState(attOf(bellyDown(0)));
      m.setPositionState({0, 0, -6.4f, 0, 0, 3000});
      for (int i = 0; i < 10000; i++)
        m.step();
      auto a = m.attitudeState();
      float a7[7];
      for (int k = 0; k < 7; k++)
        a7[k] = a(k);
      D::BellyAttitude ba = D::bellyAttitude(a7);
      auto v = m.positionState();
      printf(" [%+.1f: %+.2f m/s p%+.1f]", sp, v(0) * cosf(ba.heading) + v(1) * sinf(ba.heading), ba.pitch / kDeg);
    }
    printf("\n");
  }
}

static void test_heading_step() {
  const bool pr = getenv("DESCENT_DEBUG") != nullptr;
  for (bool sh : {true, false})
    for (float st : {90.0f, 175.0f})
      for (float sp : {0.0f, -20.0f, 8.0f})
        headingStep(sh, st, sp, pr && sp == 0 && st == 90);
}

static void flapJacobian(float t, float b, bool shield, float pitchDeg) {
  for (int f = 0; f < 4; f++) {
    float tq[2][3];
    for (int sgn = 0; sgn < 2; sgn++) {
      BodySimParams p;
      p.applyActuatorRateLimits = false;
      if (!shield) {
        p.flapFoldShieldFactor = 0;
        p.aeroCalibration = 4.0f;
      }
      BodySimModel mm(p);
      float a[4] = {t, t, b, b};
      a[f] += (sgn ? 1 : -1) * 2 * kDeg;
      mm.setFlapInput(a[0], a[1], a[2], a[3]);
      mm.step();
      mm.setAttitudeState(attOf(bellyDown(pitchDeg)));
      mm.setPositionState({0, 0, -6.4f, 0, 0, 100});
      mm.computeAero();
      for (int k = 0; k < 3; k++)
        tq[sgn][k] = mm.loads().aeroTorque(k);
    }
    printf("DESCENT jac %2.0f/%2.0f %s pitch %+3.0f flap %d: dtau/deg x %+.4f y %+.4f z %+.4f\n", t / kDeg, b / kDeg,
           shield ? "sh" : "ns", pitchDeg, f, (tq[1][0] - tq[0][0]) / 4, (tq[1][1] - tq[0][1]) / 4, (tq[1][2] - tq[0][2]) / 4);
  }
}

/// Yaw (bottom differential) and roll (top differential) moment derivatives along candidate glide schedules, at
/// flow angles (AoA) -30/0/+30 deg about body Y (glide from the nose side is AoA < 0 here), N m/deg.
static void yawAuthority(float t, float b, bool shield) {
  printf("DESCENT auth %2.0f/%2.0f %s:", t, b, shield ? "sh" : "ns");
  for (float aoa : {-30.f, -15.f, 0.f, 15.f, 30.f}) {
    float tq[2][3];
    for (int sgn = 0; sgn < 2; sgn++) {
      BodySimParams p;
      p.applyActuatorRateLimits = false;
      if (!shield) {
        p.flapFoldShieldFactor = 0;
        p.aeroCalibration = 4.0f;
      }
      BodySimModel mm(p);
      const float d = (sgn ? 1 : -1) * 2 * kDeg;
      mm.setFlapInput(t * kDeg, t * kDeg, b * kDeg + d, b * kDeg - d);
      mm.step();
      mm.setAttitudeState(attOf(bellyDown(aoa)));
      mm.setPositionState({0, 0, -6.4f, 0, 0, 100});
      mm.computeAero();
      for (int k = 0; k < 3; k++)
        tq[sgn][k] = mm.loads().aeroTorque(k);
    }
    printf(" [aoa %+3.0f yaw %+.4f roll %+.4f]", aoa, (tq[1][0] - tq[0][0]) / 4, (tq[1][2] - tq[0][2]) / 4);
  }
  printf("\n");
}

/// Physics evidence (report only): per-flap moment Jacobian at the legacy and new neutrals, and the bottom-pair yaw
/// authority along the glide schedule over AoA, both aero models.
static void test_physics_tables() {
  for (bool sh : {true, false})
    for (int k = 0; k < 2; k++) {
      const float tt[2] = {50, 30}, bb[2] = {40, 78};
      flapJacobian(tt[k] * kDeg, bb[k] * kDeg, sh, 0);
    }
  const float sch[][2] = {{30, 78}, {60, 78}, {90, 78}, {90, 70}, {90, 66}, {90, 62}, {90, 58}, {50, 40}};
  for (bool sh : {true, false})
    for (auto &c : sch)
      yawAuthority(c[0], c[1], sh);
}

/// DESCENT_DEBUG=1: B, offset 10, heading error 90, wind 5 (trace); =2: same, calm, no fold shield.
static void test_debug() {
  const char *e = getenv("DESCENT_DEBUG");
  if (!e)
    return;
  Scenario sc;
  sc.offset = 10, sc.headingErrDeg = 90, sc.wind = 5, sc.seed = 11;
  sc.shield = atoi(e) != 2;
  if (atoi(e) == 2)
    sc.wind = 0;
  Metrics m = runScenario(Strat::B, sc, true);
  printf("DESCENT debug: miss %.1f\n", m.miss);
}

static void test_strategy_matrix() {
  if (getenv("DESCENT_DEBUG"))
    return;
  printf("DESCENT seeds per cell %d (DESCENT_SEEDS), headings 0/45/90/135/180, offsets 0/10/30 m, wind 0/3/5 m/s "
         "(gust 3 m/s with wind), noise pos 0.3 m vel 0.2 m/s heading 2 deg\n",
         seedsPerCell());
  runMatrix(Strat::L, 1, true, true, "nominal");
  runMatrix(Strat::A, 1, true, true, "nominal");
  runMatrix(Strat::C, 1, true, true, "nominal");
  runMatrix(Strat::Bx, 1, true, true, "nominal");
  runMatrix(Strat::Bs, 1, true, true, "nominal");
  MatrixResult b = runMatrix(Strat::B, 1, true, true, "nominal");
  // Acceptance of the prototype (nominal model): calm median within 2 m and p90 within 3 m at the stop trigger, no
  // loss of belly-down (fade starts at 35 deg) in calm, no timeout. Wind 3/5 m/s with 3 m/s gusts is beyond the
  // model's glide authority for station keeping (design doc), so it is reported, not asserted.
  TEST_ASSERT_TRUE(b.calm.pct(0.5f) < 2.0f);
  TEST_ASSERT_TRUE(b.calm.pct(0.9f) < 3.0f);
  TEST_ASSERT_TRUE(b.calm.bellyMax < 35.0f);
  TEST_ASSERT_EQUAL_INT(0, b.all.timeouts);
}

static void test_robustness_B() {
  if (getenv("DESCENT_DEBUG"))
    return;
  runMatrix(Strat::B, 0.7f, true, true, "aero x0.7", false);
  runMatrix(Strat::B, 1.3f, true, true, "aero x1.3", false);
  runMatrix(Strat::B, 1.0f, false, true, "no shield", false);
  runMatrix(Strat::B, 1.0f, true, false, "no noise", false);
  runMatrix(Strat::L, 1.0f, false, true, "no shield", false);
  runMatrix(Strat::Bs, 1.0f, false, true, "no shield", false);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_reproduce_legacy);
  RUN_TEST(test_glide_map);
  RUN_TEST(test_heading_step);
  RUN_TEST(test_physics_tables);
  RUN_TEST(test_debug);
  RUN_TEST(test_strategy_matrix);
  RUN_TEST(test_robustness_B);
  return UNITY_END();
}
