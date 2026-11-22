// Descent (belly-down) guidance and flap attitude law: STUDY PROTOTYPE (feature/descent-control-study).
//
// Pure, deterministic, header-only, no scheduler, topics, clock, globals or heap. Plain float arrays in the
// ExVectr conventions (NWU world, attitude [w(3), q(4)] with q = [w,x,y,z] body -> world, position [V(3), P(3)],
// SI units, angles in rad). Body frame: +Z nose, belly -X, flaps TL/BL on +Y, TR/BR on -Y (ExVectrDSP BodySimModel).
//
// Intended home after review: lib/ExVectrControl/include/ExVectrControl/starship/descent_guidance.hpp (guidance) and
// .../starship/flap_descent_law.hpp (flap law), namespace VCTR::CTRL. Not wired into main.cpp. Design and numbers:
// ExVectrStarship docs/descent-control-design.md.
//
// Structure (cascade):
//   DescentGuidance (target above the landing point, position + velocity estimate)
//       -> heading setpoint (nose direction) + glide command (air speed along the nose) -> pitch split
//   FlapDescentLaw (attitude estimate, heading setpoint, pitch split)
//       -> heading-rate loop (u0), pitch split + pitch damping (u1), roll attitude hold (u2) -> four flap angles
// The vehicle glides essentially only along its nose (forward up to ~8 m/s, backward at most ~1 m/s in the model),
// so lateral control is "turn the nose toward the needed air velocity, then set the glide speed".
#ifndef EXVECTR_STUDY_DESCENT_CONTROL_HPP
#define EXVECTR_STUDY_DESCENT_CONTROL_HPP

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace CTRL {
namespace descent {

static constexpr float kPi = 3.14159265f;
static constexpr float kDegF = kPi / 180.0f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float wrapPi(float a) {
  while (a > kPi)
    a -= 2 * kPi;
  while (a < -kPi)
    a += 2 * kPi;
  return a;
}
/// v_world = q * v_body * q^-1, q = [w,x,y,z].
inline void rotate(const float q[4], const float v[3], float out[3]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  out[0] = v[0] + w * tx + (y * tz - z * ty);
  out[1] = v[1] + w * ty + (z * tx - x * tz);
  out[2] = v[2] + w * tz + (x * ty - y * tx);
}

/// Belly-down attitude quantities from [w, q]: nose azimuth (CCW from North), nose elevation (pitch, + nose up),
/// roll about the nose (+ = positive rotation about body Z), belly angle (body +X vs world up).
struct BellyAttitude {
  float heading, pitch, roll, belly;
  bool headingValid; ///< false when the nose is within 30 deg of vertical (azimuth meaningless)
};
inline BellyAttitude bellyAttitude(const float att[7]) {
  const float ex[3] = {1, 0, 0}, ey[3] = {0, 1, 0}, ez[3] = {0, 0, 1};
  float xw[3], yw[3], zw[3];
  rotate(att + 3, ex, xw);
  rotate(att + 3, ey, yw);
  rotate(att + 3, ez, zw);
  BellyAttitude b;
  const float horiz = sqrtf(zw[0] * zw[0] + zw[1] * zw[1]);
  b.heading = atan2f(zw[1], zw[0]);
  b.pitch = atan2f(zw[2], horiz);
  b.roll = -asinf(clampf(yw[2], -1, 1));
  b.belly = acosf(clampf(xw[2], -1, 1));
  b.headingValid = horiz > 0.5f;
  return b;
}

// =====================================================================================================================
// Target interface
// =====================================================================================================================
/// "TargetAbove": the point the descent flies down above (set by the mission/executive above the landing point).
struct DescentTarget {
  float x_m = 0, y_m = 0;     ///< world NWU horizontal target, m
  float endAltitude_m = 27;   ///< altitude at which the descent ends (flip start estimate, world z), m, for the funnel
  bool valid = false;         ///< false: no target (hold heading, zero glide = fall straight down)
};

// =====================================================================================================================
// Guidance: position/velocity -> heading setpoint + pitch split
// =====================================================================================================================
struct DescentGuidanceParams {
  float posTimeConst_s = 3.0f;   ///< v_des = e / T (horizontal position error -> desired ground velocity)
  float vMax_mps = 4.5f;         ///< desired ground speed limit, far above the end altitude
  float vMaxEnd_mps = 1.5f;      ///< limit at the end altitude (linear between, over funnelHeight_m)
  float funnelHeight_m = 20.0f;
  float velKp = 0.8f;            ///< air-velocity command per ground-velocity error, (m/s)/(m/s)
  float velKi = 0.25f;           ///< integral (wind, trim and model error), 1/s
  float windIntLimit_mps = 7.0f; ///< |integral| limit, m/s (wind authority)
  float turnOnSpeed_mps = 1.2f;  ///< |air-velocity command| above which the heading follows it
  float turnOffSpeed_mps = 0.6f; ///< below which the heading setpoint is held (hysteresis)
  float glideMin_mps = -1.1f;    ///< backward glide limit along the nose (model: -1.15 m/s at split +12 deg)
  float glideMax_mps = 4.9f;     ///< forward glide limit (model: 4.95 m/s at split -80, top 90 / bottom 58 deg)
  float glideYawSafe_mps = 3.3f; ///< glide limit while |heading error| > fastGlideGate (bottom pair >= 70 deg: full
                                 ///< yaw authority in both aero models; below 65 deg it is model-dependent)
  float fastGlideGate_rad = 20 * kDegF;
  /// Glide map (model, shield 0.7, neutral 30/78 deg, the law's split schedule, free body; test_glide_map): air
  /// speed along the nose -> pitch split, rad. Feed-forward only; the velocity integral removes its error.
  static constexpr int kMapMax = 16;
  int mapN = 15; ///< points used (12 = yaw-safe envelope only: bottom pair >= 70 deg)
  float mapGlide_mps[kMapMax] = {-1.15f, -0.99f, -0.68f, -0.24f, 0.13f, 0.87f, 1.42f, 1.80f,
                                 2.19f,  2.59f,  2.78f,  3.37f,  3.79f, 4.29f, 4.95f};
  float mapSplit_rad[kMapMax] = {12 * kDegF,  8 * kDegF,   4 * kDegF,   0,           -4 * kDegF,
                                 -12 * kDegF, -20 * kDegF, -30 * kDegF, -40 * kDegF, -50 * kDegF,
                                 -60 * kDegF, -68 * kDegF, -72 * kDegF, -76 * kDegF, -80 * kDegF};
  float glideHeadingGate_rad = 60 * kDegF; ///< |heading error| above which the glide command is 0 (turn first)
  float splitRate_radps = 40 * kDegF;      ///< slew limit of the pitch split (smooth glide changes)
  float entryHold_s = 1.5f;      ///< after phase entry: heading held, entry split (flaps deploy from the kick)
  float entrySplit_rad = -20 * kDegF; ///< split during the entry hold: top pair 50 deg (damps the deploy pitch-up)
  bool forwardOnlyHeading = true; ///< true: nose toward the command (B). false: nose axis mod 180 deg (study)
};

/// Explicit memory (the flight executive's behaviour memory POD).
struct DescentGuidanceMem {
  float windInt[2] = {0, 0}; ///< velocity integral, world N/W, m/s
  float headingSp = 0;       ///< held heading setpoint, rad
  bool following = false;    ///< heading follows the command (hysteresis state)
  float elapsed_s = 0;       ///< time since phase entry
  float split = 0;           ///< slewed pitch split, rad
};

struct DescentGuidanceOut {
  float headingSp_rad = 0;  ///< nose azimuth setpoint (CCW from North)
  float glideCmd_mps = 0;   ///< air speed command along the measured nose
  float pitchSplit_rad = 0; ///< pitch split for FlapDescentLaw (+ = nose-up / slower)
  float vDes[2] = {0, 0};   ///< desired ground velocity (telemetry)
  float vAirCmd[2] = {0, 0};///< air-velocity command (telemetry)
};

class DescentGuidance {
 public:
  /// Phase entry: integrator zero, heading setpoint = current nose heading.
  static void init(const float att[7], DescentGuidanceMem &m) {
    m = DescentGuidanceMem();
    m.headingSp = bellyAttitude(att).heading;
    m.split = 0;
  }
  /// Phase entry with an initial split (the entry hold value applies at once, not slewed).
  static void init(const DescentGuidanceParams &p, const float att[7], DescentGuidanceMem &m) {
    init(att, m);
    m.split = p.entrySplit_rad;
  }

  static float slew(float x, float target, float maxStep) { return x + clampf(target - x, -maxStep, maxStep); }

  static float glideToSplit(const DescentGuidanceParams &p, float g) {
    if (g <= p.mapGlide_mps[0])
      return p.mapSplit_rad[0];
    for (int i = 1; i < p.mapN; i++)
      if (g <= p.mapGlide_mps[i]) {
        const float f = (g - p.mapGlide_mps[i - 1]) / (p.mapGlide_mps[i] - p.mapGlide_mps[i - 1]);
        return p.mapSplit_rad[i - 1] + f * (p.mapSplit_rad[i] - p.mapSplit_rad[i - 1]);
      }
    return p.mapSplit_rad[p.mapN - 1];
  }

  static DescentGuidanceOut step(const DescentGuidanceParams &p, const DescentTarget &t, const float pos[6],
                                 const float att[7], float dt, DescentGuidanceMem &m) {
    DescentGuidanceOut o;
    const BellyAttitude b = bellyAttitude(att);
    bool finite = true;
    for (int i = 0; i < 6; i++)
      finite = finite && isfinite(pos[i]);
    m.elapsed_s += dt > 0 ? dt : 0;
    if (!t.valid || !finite || !(dt > 0) || m.elapsed_s < p.entryHold_s) { // no target or no position: hold heading, zero glide (fall straight)
      o.headingSp_rad = m.headingSp;
      const float hold = m.elapsed_s < p.entryHold_s && t.valid ? p.entrySplit_rad : glideToSplit(p, 0);
      m.split = slew(m.split, hold, p.splitRate_radps * (dt > 0 ? dt : 0));
      o.pitchSplit_rad = m.split;
      return o;
    }
    // 1. Funnel: desired ground velocity toward the target, limit shrinking toward the end altitude.
    const float ex = t.x_m - pos[3], ey = t.y_m - pos[4];
    const float hGo = pos[5] - t.endAltitude_m;
    const float vLim = p.vMaxEnd_mps + (p.vMax_mps - p.vMaxEnd_mps) * clampf(hGo / p.funnelHeight_m, 0, 1);
    float vdx = ex / p.posTimeConst_s, vdy = ey / p.posTimeConst_s;
    const float vd = sqrtf(vdx * vdx + vdy * vdy);
    if (vd > vLim)
      vdx *= vLim / vd, vdy *= vLim / vd;
    // 2. Velocity PI -> air-velocity command (the integral learns the wind).
    const float evx = vdx - pos[0], evy = vdy - pos[1];
    float cx = vdx + p.velKp * evx + m.windInt[0], cy = vdy + p.velKp * evy + m.windInt[1];
    // 3. Heading: follow the command direction above turnOn, hold below turnOff.
    const float c = sqrtf(cx * cx + cy * cy);
    if (c > p.turnOnSpeed_mps)
      m.following = true;
    else if (c < p.turnOffSpeed_mps)
      m.following = false;
    if (m.following) {
      float h = atan2f(cy, cx);
      if (!p.forwardOnlyHeading && fabsf(wrapPi(h - b.heading)) > 0.5f * kPi)
        h = wrapPi(h + kPi);
      m.headingSp = h;
    }
    // 4. Glide command = projection of the air-velocity command on the measured nose direction.
    const float nx = cosf(b.heading), ny = sinf(b.heading);
    float g = cx * nx + cy * ny;
    const float ehAbs = fabsf(wrapPi(m.headingSp - b.heading));
    if (ehAbs > p.glideHeadingGate_rad)
      g = 0; // turn first: no glide (and no brake) while the nose is far from the setpoint
    else if (ehAbs > p.fastGlideGate_rad && g > p.glideYawSafe_mps)
      g = p.glideYawSafe_mps; // fast glide (bottom pair < 70 deg) only when aligned
    const float gSat = clampf(g, p.glideMin_mps, p.glideMax_mps);
    // 5. Integrate, with anti-windup: no integration of the along-nose part while the glide is saturated in that
    //    direction, and a hard magnitude limit.
    float ix = p.velKi * evx * dt, iy = p.velKi * evy * dt;
    if (g != gSat) {
      const float along = ix * nx + iy * ny;
      if ((g > gSat && along > 0) || (g < gSat && along < 0))
        ix -= along * nx, iy -= along * ny;
    }
    m.windInt[0] += ix, m.windInt[1] += iy;
    const float wi = sqrtf(m.windInt[0] * m.windInt[0] + m.windInt[1] * m.windInt[1]);
    if (wi > p.windIntLimit_mps)
      m.windInt[0] *= p.windIntLimit_mps / wi, m.windInt[1] *= p.windIntLimit_mps / wi;
    o.headingSp_rad = m.headingSp;
    o.glideCmd_mps = gSat;
    m.split = slew(m.split, glideToSplit(p, gSat), p.splitRate_radps * dt);
    o.pitchSplit_rad = m.split;
    o.vDes[0] = vdx, o.vDes[1] = vdy;
    o.vAirCmd[0] = cx, o.vAirCmd[1] = cy;
    return o;
  }
};

// =====================================================================================================================
// Flap attitude law: heading + pitch split + roll hold -> four flap angles (TL, TR, BL, BR)
// =====================================================================================================================
struct FlapDescentLawParams {
  float neutralTop_rad = 30 * kDegF;    ///< zero-glide trim (model): pitch moment ~0, glide about -0.2 m/s
  float neutralBottom_rad = 78 * kDegF; ///< bottom >= 70 deg: yaw sign robust with and without the fold shield
  float kHeading = 1.5f;                ///< heading error -> yaw-rate command, 1/s
  float yawRateMax = 75 * kDegF;        ///< yaw-rate command limit, rad/s
  float kYawRate = 0.40f;               ///< virtual yaw command Y per yaw-rate error, rad/(rad/s)
  float yawMax_rad = 12 * kDegF;        ///< bottom differential limit (78 + 12 = 90)
  float kPitch = 0.60f;                 ///< pitch command per pitch error (setpoint 0), rad/rad
  float kPitchRate = 0.40f;             ///< pitch command per pitch rate, rad/(rad/s)
  float kRoll = 2.0f;                   ///< virtual roll command R per roll error, rad/rad
  float kRollRate = 0.25f;              ///< R per roll rate, rad/(rad/s)
  float rollMax_rad = 15 * kDegF;       ///< top differential limit (30 - 15 = 15 deg clamp)
  /// Yaw/roll decoupler: bottom diff = dYY*Y + dYR*R, top diff = dRY*Y + dRR*R, from the inverse of the model's
  /// 2x2 moment matrix at 30/78 deg (shield 0.7): [tau_x; tau_z] = [[0.0154 0.0010] [0.0028 0.0036]] N m/deg
  /// times [bottom diff; top diff], normalised so Y gives 0.0154 N m/deg of yaw and R 0.0036 N m/deg of roll.
  float dYY = 1.05f, dYR = -0.07f, dRY = -0.82f, dRR = 1.05f;
  float splitMin_rad = -80 * kDegF; ///< forward: top pair 30 -> 90 deg, then bottom 78 -> 58 deg
  float splitMax_rad = 12 * kDegF;  ///< brake: top 30 -> 18, bottom 78 -> 90 deg
  float flapMin_rad = 15 * kDegF, flapMax_rad = 90 * kDegF; ///< StarshipFlaps clamp
  float fadeStart_rad = 35 * kDegF, fadeWidth_rad = 20 * kDegF;
  float turnLatch_rad = 150 * kDegF;    ///< beyond this heading error the turn direction is latched
};

struct FlapDescentCmd {
  float headingSp_rad = 0;
  float pitchSplit_rad = 0;
  float rollSp_rad = 0;      ///< roll setpoint (strategy C only; 0 in the recommended law)
  bool headingFree = false;  ///< true: no heading loop (yaw-rate damping only), strategy C
};

struct FlapDescentLawMem {
  float lastHeadingErr = 0;
};

struct FlapDescentLawOut {
  float flap[4];             ///< TL, TR, BL, BR, rad (clamped)
  float u[3];                ///< yaw diff (bottom), pitch common, roll diff (top), rad; telemetry
  float headingErr;
};

/**
 * Allocation (from the model's per-flap moment Jacobian at 25/72 deg, 6.4 m/s, N m/deg; shield 0.7 / off):
 *   bottom flap: tau_x (yaw) +-0.0070 / +-0.0105, tau_y +0.0048 / +0.0094, tau_z +-0.0019 / +-0.0035
 *   top flap:    tau_x +-0.0010 / +-0.0018,         tau_y -0.0041 / -0.0025, tau_z +-0.0016 / +-0.0010
 * so yaw = bottom differential (BL +, BR -), roll = top differential (TL +, TR -), pitch = top -, bottom +.
 * The legacy mixer drove yaw on all four flaps, and its roll command made 1.35x more yaw moment than its yaw
 * command (u2 -> tau_x +0.016 vs u0 -> tau_x +0.012 N m/deg).
 */
class FlapDescentLaw {
 public:
  /// Bottom-pair part of the split schedule: brake retracts the bottom pair; beyond the top travel (split below
  /// -60 deg) the bottom pair extends down to 70 deg for the last bit of forward glide.
  static float bottomSplit(const FlapDescentLawParams &p, float split) {
    if (split > 0)
      return split;
    const float topTravel = p.flapMax_rad - p.neutralTop_rad;
    return split < -topTravel ? split + topTravel : 0;
  }

  static FlapDescentLawOut step(const FlapDescentLawParams &p, const float att[7], const FlapDescentCmd &c,
                                FlapDescentLawMem &m) {
    const BellyAttitude b = bellyAttitude(att);
    FlapDescentLawOut o;
    float eh = 0;
    if (!c.headingFree && b.headingValid) {
      eh = wrapPi(c.headingSp_rad - b.heading);
      // Turn-direction latch: near 180 deg keep turning the way we started (no snapping between +-180).
      if (fabsf(eh) > p.turnLatch_rad && eh * m.lastHeadingErr < 0)
        eh += eh < 0 ? 2 * kPi : -2 * kPi;
      m.lastHeadingErr = eh;
    }
    const float rCmd = c.headingFree ? 0 : clampf(p.kHeading * eh, -p.yawRateMax, p.yawRateMax);
    // Virtual commands: Y (yaw moment), R (roll moment), decoupled into the bottom (yaw) and top (roll) pairs.
    // Roll has priority: if the top differential would saturate, Y is reduced first (without this the yaw command's
    // roll moment wins over the weak top-pair roll authority and the vehicle spins, no-shield model).
    float Y = p.kYawRate * (rCmd - att[0]);
    const float R = p.kRoll * (c.rollSp_rad - b.roll) - p.kRollRate * att[2];
    float topD = p.dRY * Y + p.dRR * R;
    if (fabsf(topD) > p.rollMax_rad && fabsf(p.dRY) > 1e-6f) {
      const float lim = topD > 0 ? p.rollMax_rad : -p.rollMax_rad;
      const float yNew = (lim - p.dRR * R) / p.dRY;
      Y = (yNew * Y > 0) ? (fabsf(yNew) < fabsf(Y) ? yNew : Y) : 0;
      topD = p.dRY * Y + p.dRR * R;
    }
    const float roll = clampf(topD, -p.rollMax_rad, p.rollMax_rad);
    const float yaw = clampf(p.dYY * Y + p.dYR * R, -p.yawMax_rad, p.yawMax_rad);
    const float split = clampf(c.pitchSplit_rad, p.splitMin_rad, p.splitMax_rad);
    // Pitch: the split is scheduled so that the bottom flaps stay >= 70 deg (yaw authority, see the class comment):
    // forward glide (split < 0) retracts only the top pair, braking (split > 0) extends the top pair and retracts
    // the bottom pair. The pitch attitude/rate damping u1 acts on both pairs (small).
    const float u1 = -p.kPitch * b.pitch - p.kPitchRate * att[1];
    const float ff = 1 - clampf((b.belly - p.fadeStart_rad) / p.fadeWidth_rad, 0, 1);
    float topBase = p.neutralTop_rad - split - u1;
    float botBase = p.neutralBottom_rad + bottomSplit(p, split) + u1;
    // Priority: the common parts are clamped so that every flap keeps its differential inside the travel (a pinned
    // flap turns the split into a yaw/roll moment).
    const float dT = fabsf(roll), dB = fabsf(yaw);
    topBase = clampf(topBase, p.flapMin_rad + dT, fmaxf(p.flapMin_rad + dT, p.flapMax_rad - dT));
    botBase = clampf(botBase, p.flapMin_rad + dB, fmaxf(p.flapMin_rad + dB, p.flapMax_rad - dB));
    // Fade (belly angle beyond 35..55 deg): blend toward the neutral, as the legacy law does.
    const float tb = p.neutralTop_rad + (topBase - p.neutralTop_rad) * ff;
    const float bb = p.neutralBottom_rad + (botBase - p.neutralBottom_rad) * ff;
    o.flap[0] = tb + roll * ff;
    o.flap[1] = tb - roll * ff;
    o.flap[2] = bb + yaw * ff;
    o.flap[3] = bb - yaw * ff;
    for (int i = 0; i < 4; i++)
      o.flap[i] = clampf(o.flap[i], p.flapMin_rad, p.flapMax_rad);
    o.u[0] = yaw, o.u[1] = u1, o.u[2] = roll;
    o.headingErr = eh;
    return o;
  }
};

} // namespace descent
} // namespace CTRL
} // namespace VCTR

#endif
