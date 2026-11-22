// Unit tests of the descent control (ExVectrControl starship/): NaN guards of DescentGuidance and FlapDescentLaw,
// and the ControlAttitudeFlaps mode mapping (FlapModeMapper), including proof that Stabilize/Upright/Acsent are
// bit-identical to the pre-Descent implementation (verbatim reference copy below).
//   pio test -e native -f test_descent_unit
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "ExVectrControl/starship/descent_guidance.hpp"
#include "ExVectrControl/starship/flap_descent_law.hpp"
#include "ExVectrControl/starship/flap_mode_mapper.hpp"

using namespace VCTR;
using namespace VCTR::CTRL;
namespace D = VCTR::CTRL::descent;

void setUp() {}
void tearDown() {}

static const float kNaN = NAN;
static const float kInf = INFINITY;
static const float kDeg = D::kDegF;

/// Belly-down attitude, q = Rz(psi) * Ry(-90 deg). The nose then points along psi + 180 deg (Ry(-90) puts it South).
static void bellyQuat(float psi, float q[4]) {
  const float cz = cosf(psi / 2), sz = sinf(psi / 2);
  const float cy = cosf(-D::kPi / 4), sy = sinf(-D::kPi / 4);
  // (cz, 0, 0, sz) x (cy, 0, sy, 0)
  q[0] = cz * cy;
  q[1] = -sz * sy;
  q[2] = cz * sy;
  q[3] = sz * cy;
}

/// Belly-down attitude with the nose heading `heading` (CCW from North).
static void makeAtt(float heading, float att[7]) {
  for (int i = 0; i < 3; i++)
    att[i] = 0;
  bellyQuat(heading - D::kPi, att + 3);
}

static bool allFiniteArr(const float *v, int n) { return D::allFinite(v, n); }

// ---------------------------------------------------------------------------------------------------------------------
// FlapDescentLaw
// ---------------------------------------------------------------------------------------------------------------------
static void test_law_neutral_at_rest() {
  D::FlapDescentLawParams p;
  D::FlapDescentLawMem m;
  float att[7];
  makeAtt(0.7f, att);
  D::FlapDescentCmd c;
  c.headingSp_rad = 0.7f;
  D::FlapDescentLawOut o = D::FlapDescentLaw::step(p, att, c, m);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 30 * kDeg, o.flap[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 30 * kDeg, o.flap[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 78 * kDeg, o.flap[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 78 * kDeg, o.flap[3]);
}

static void test_law_nan_attitude_gives_neutral() {
  D::FlapDescentLawParams p;
  D::FlapDescentLawMem m;
  D::FlapDescentCmd c;
  c.headingSp_rad = 1.0f;
  c.pitchSplit_rad = -40 * kDeg; // a strong forward glide with a valid attitude
  for (int bad = 0; bad < 7; bad++) {
    float att[7];
    makeAtt(0.0f, att);
    att[bad] = (bad & 1) ? kNaN : kInf;
    m.lastHeadingErr = 1.0f;
    D::FlapDescentLawOut o = D::FlapDescentLaw::step(p, att, c, m);
    TEST_ASSERT_EQUAL_FLOAT(30 * kDeg, o.flap[0]);
    TEST_ASSERT_EQUAL_FLOAT(30 * kDeg, o.flap[1]);
    TEST_ASSERT_EQUAL_FLOAT(78 * kDeg, o.flap[2]);
    TEST_ASSERT_EQUAL_FLOAT(78 * kDeg, o.flap[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.lastHeadingErr); // latch state cleared
  }
}

static void test_law_nan_command_finite() {
  D::FlapDescentLawParams p;
  float att[7];
  makeAtt(0.3f, att);
  att[0] = 0.2f;
  const float bad[3] = {kNaN, kInf, -kInf};
  for (int k = 0; k < 3; k++) {
    D::FlapDescentLawMem m;
    D::FlapDescentCmd c;
    c.headingSp_rad = bad[k];
    c.pitchSplit_rad = bad[(k + 1) % 3];
    c.rollSp_rad = bad[(k + 2) % 3];
    D::FlapDescentLawOut o = D::FlapDescentLaw::step(p, att, c, m);
    TEST_ASSERT_TRUE(allFiniteArr(o.flap, 4));
    TEST_ASSERT_TRUE(std::isfinite(m.lastHeadingErr));
    for (int i = 0; i < 4; i++)
      TEST_ASSERT_TRUE(o.flap[i] >= p.flapMin_rad - 1e-6f && o.flap[i] <= p.flapMax_rad + 1e-6f);
    // Invalid heading = no heading loop, invalid split = 0, invalid roll setpoint = 0: the same result as a valid
    // command with zero heading error, split 0 and roll setpoint 0.
    D::FlapDescentLawMem m2;
    D::FlapDescentCmd c2;
    c2.headingSp_rad = D::bellyAttitude(att).heading;
    D::FlapDescentLawOut o2 = D::FlapDescentLaw::step(p, att, c2, m2);
    for (int i = 0; i < 4; i++)
      TEST_ASSERT_EQUAL_FLOAT(o2.flap[i], o.flap[i]);
  }
}

static void test_law_corrupt_memory_recovers() {
  D::FlapDescentLawParams p;
  D::FlapDescentLawMem m;
  m.lastHeadingErr = kNaN;
  float att[7];
  makeAtt(0.0f, att);
  D::FlapDescentCmd c;
  c.headingSp_rad = 1.0f;
  D::FlapDescentLawOut o = D::FlapDescentLaw::step(p, att, c, m);
  TEST_ASSERT_TRUE(allFiniteArr(o.flap, 4));
  TEST_ASSERT_TRUE(std::isfinite(m.lastHeadingErr));
}

// ---------------------------------------------------------------------------------------------------------------------
// DescentGuidance
// ---------------------------------------------------------------------------------------------------------------------
static D::DescentTarget target() {
  D::DescentTarget t;
  t.x_m = 0, t.y_m = 0, t.endAltitude_m = 27, t.valid = true;
  return t;
}

static const float kPosFar[6] = {0, 0, 0, 20, 0, 80}; // 20 m North of the target, 80 m up

/// Run the guidance for 3 s with valid input at an offset, so that it has a heading and a glide.
static void warmUp(const D::DescentGuidanceParams &p, D::DescentGuidanceMem &m, float att[7]) {
  makeAtt(D::kPi, att); // nose already toward the target (South)
  D::DescentGuidance::init(p, att, m);
  for (int i = 0; i < 150; i++)
    D::DescentGuidance::step(p, target(), kPosFar, att, 0.02f, m);
}

static void test_guidance_valid_flies_toward_target() {
  D::DescentGuidanceParams p;
  D::DescentGuidanceMem m;
  float att[7];
  warmUp(p, m, att);
  D::DescentGuidanceOut o = D::DescentGuidance::step(p, target(), kPosFar, att, 0.02f, m);
  TEST_ASSERT_TRUE(fabsf(D::wrapPi(o.headingSp_rad - D::kPi)) < 0.3f); // target is South of the vehicle
  TEST_ASSERT_TRUE(o.pitchSplit_rad < -1 * kDeg);                      // forward glide
}

/// Invalid input: heading held, integral frozen and finite, outputs finite. Position/attitude/target faults: the
/// split slews to 0 (neutral 30/78, vertical fall). A bad dt freezes the split (no time base to slew with).
static void test_guidance_invalid_input_holds_heading() {
  D::DescentGuidanceParams p;
  for (int what = 0; what < 7; what++) { // 0 pos NaN, 1 vel inf, 2 att NaN, 3 target x NaN, 4 end alt NaN, 5 dt NaN, 6 dt inf
    D::DescentGuidanceMem m;
    float att[7];
    warmUp(p, m, att);
    const float hBefore = m.headingSp;
    const float w0 = m.windInt[0], w1 = m.windInt[1];
    TEST_ASSERT_TRUE(m.split < -1 * kDeg); // gliding before the fault
    float pos[6] = {0, 0, 0, 20, 0, 80};
    D::DescentTarget t = target();
    float dt = 0.02f;
    switch (what) {
    case 0: pos[3] = kNaN; break;
    case 1: pos[1] = kInf; break;
    case 2: att[4] = kNaN; break;
    case 3: t.x_m = kNaN; break;
    case 4: t.endAltitude_m = kNaN; break;
    case 5: dt = kNaN; break;
    default: dt = kInf; break;
    }
    D::DescentGuidanceOut o{};
    for (int i = 0; i < 400; i++) // 8 s: the split slews to 0 at 40 deg/s
      o = D::DescentGuidance::step(p, t, pos, att, dt, m);
    TEST_ASSERT_EQUAL_FLOAT(hBefore, o.headingSp_rad);
    TEST_ASSERT_EQUAL_FLOAT(hBefore, m.headingSp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.glideCmd_mps);
    TEST_ASSERT_TRUE(std::isfinite(o.pitchSplit_rad));
    TEST_ASSERT_TRUE(allFiniteArr(m.windInt, 2));
    TEST_ASSERT_EQUAL_FLOAT(w0, m.windInt[0]); // integral frozen, not poisoned
    TEST_ASSERT_EQUAL_FLOAT(w1, m.windInt[1]);
    if (what < 5)
      TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.pitchSplit_rad);
  }
}

static void test_guidance_recovers_after_invalid_input() {
  D::DescentGuidanceParams p;
  D::DescentGuidanceMem m;
  float att[7];
  warmUp(p, m, att);
  const float bad[6] = {kNaN, kNaN, kNaN, kNaN, kNaN, kNaN};
  for (int i = 0; i < 100; i++)
    D::DescentGuidance::step(p, target(), bad, att, 0.02f, m);
  D::DescentGuidanceOut o{};
  for (int i = 0; i < 200; i++)
    o = D::DescentGuidance::step(p, target(), kPosFar, att, 0.02f, m);
  TEST_ASSERT_TRUE(o.pitchSplit_rad < -1 * kDeg); // gliding again
  TEST_ASSERT_TRUE(std::isfinite(o.headingSp_rad));
}

static void test_guidance_invalid_target_and_corrupt_memory() {
  D::DescentGuidanceParams p;
  D::DescentGuidanceMem m;
  float att[7];
  warmUp(p, m, att);
  D::DescentTarget t = target();
  t.valid = false;
  D::DescentGuidanceOut o{};
  for (int i = 0; i < 400; i++)
    o = D::DescentGuidance::step(p, t, kPosFar, att, 0.02f, m);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, o.pitchSplit_rad);
  // Corrupt memory
  D::DescentGuidanceMem mc;
  D::DescentGuidance::init(p, att, mc);
  mc.windInt[0] = kNaN;
  mc.headingSp = kInf;
  mc.split = kNaN;
  mc.elapsed_s = kNaN;
  for (int i = 0; i < 200; i++)
    o = D::DescentGuidance::step(p, target(), kPosFar, att, 0.02f, mc);
  TEST_ASSERT_TRUE(std::isfinite(o.headingSp_rad));
  TEST_ASSERT_TRUE(std::isfinite(o.pitchSplit_rad));
  TEST_ASSERT_TRUE(allFiniteArr(mc.windInt, 2));
}

static void test_guidance_init_with_nan_attitude() {
  D::DescentGuidanceParams p;
  D::DescentGuidanceMem m;
  float att[7];
  makeAtt(0.0f, att);
  att[3] = kNaN;
  D::DescentGuidance::init(p, att, m);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, m.headingSp);
}

// ---------------------------------------------------------------------------------------------------------------------
// Mode mapping: ControlAttitudeFlaps (FlapModeMapper)
// ---------------------------------------------------------------------------------------------------------------------

/// REFERENCE: the ControlAttitudeFlaps::taskThread mode switch as it was before the Descent mode (lib/ExVectrControl
/// c8d1ffb, copied verbatim with the class members as local constants).
static ControlAttitudeFlapSetting legacyMap(const ControlAttitudeBellyFlopSetting &controlSettings,
                                            const Math::Vector<float, 7> &attitudeEstimation_data) {
  const float flapTopNeutralAngle_Rad_ = 50 * DEGREES;
  const float flapBottomNeutralAngle_Rad_ = 40 * DEGREES;
  const float enableThresAngle_Rad_ = 35 * DEGREES;
  const float attitudeAzimuGain_ = 0.3;
  const float attitudePitchGain_ = 1.2;
  const float attitudeRateXGain_ = 0.2;
  const float attitudeRateYGain_ = 0.4;
  const float attitudeRateZGain_ = 0.1;
  const float attitudeAzimuLimit_Perc_ = 0.5;

  Math::Quat_F attitude = attitudeEstimation_data.block<4, 1>(3, 0);
  auto angularVelocity = attitudeEstimation_data.block<3, 1>(0, 0);
  auto bodyXAxis = attitude.rotate(Math::Vector_F({1, 0, 0}));
  auto bodyZAxis = attitude.rotate(Math::Vector_F({0, 0, 1}));
  auto bellyDownAngle = bodyXAxis.getAngleTo(Math::Vector_F({0, 0, 1}));
  float flapFactor = (bellyDownAngle - enableThresAngle_Rad_) / (20 * DEGREES);
  if (flapFactor < 0)
    flapFactor = 0;
  if (flapFactor > 1)
    flapFactor = 1;
  flapFactor = 1 - flapFactor;

  ControlAttitudeFlapSetting flapSetting;
  if (controlSettings.bellyFlopMode == ControlAttitudeBellyFlopSetting::BellyFlopMode::BellyFlopMode_Stabilize) {
    auto azimuthIs = atan2f(bodyZAxis(1), bodyZAxis(0));
    auto pitchIs = atan2f(bodyZAxis(2), sqrtf(bodyZAxis(0) * bodyZAxis(0) + bodyZAxis(1) * bodyZAxis(1)));
    Math::Vector<float, 3> attRateCtrlOutput = 0;
    float azimuthOutput = controlSettings.azimuthAngle_Rad - azimuthIs;
    float pitchOutput = controlSettings.pitchAngle_Rad - pitchIs;
    if (azimuthOutput > M_PI) {
      azimuthOutput -= 2 * M_PI;
    } else if (azimuthOutput < -M_PI) {
      azimuthOutput += 2 * M_PI;
    }
    azimuthOutput = azimuthOutput * attitudeAzimuGain_;
    pitchOutput = pitchOutput * attitudePitchGain_;
    if (azimuthOutput > attitudeAzimuLimit_Perc_) {
      azimuthOutput = attitudeAzimuLimit_Perc_;
    } else if (azimuthOutput < -attitudeAzimuLimit_Perc_) {
      azimuthOutput = -attitudeAzimuLimit_Perc_;
    }
    attRateCtrlOutput = attRateCtrlOutput + Math::Vector<float, 3>({azimuthOutput, pitchOutput, 0});
    attRateCtrlOutput = attRateCtrlOutput + Math::Vector<float, 3>{
                                                -(angularVelocity(0)) * attitudeRateXGain_,
                                                -(angularVelocity(1)) * attitudeRateYGain_,
                                                -(angularVelocity(2)) * attitudeRateZGain_,
                                            };
    flapSetting.flapTLAngle_Rad =
        flapTopNeutralAngle_Rad_ + (-attRateCtrlOutput(0) - attRateCtrlOutput(1) + attRateCtrlOutput(2)) * flapFactor;
    flapSetting.flapTRAngle_Rad =
        flapTopNeutralAngle_Rad_ + (+attRateCtrlOutput(0) - attRateCtrlOutput(1) - attRateCtrlOutput(2)) * flapFactor;
    flapSetting.flapBLAngle_Rad = flapBottomNeutralAngle_Rad_ +
                                  (+attRateCtrlOutput(0) + attRateCtrlOutput(1) + attRateCtrlOutput(2)) * flapFactor;
    flapSetting.flapBRAngle_Rad = flapBottomNeutralAngle_Rad_ +
                                  (-attRateCtrlOutput(0) + attRateCtrlOutput(1) - attRateCtrlOutput(2)) * flapFactor;
  } else if (controlSettings.bellyFlopMode ==
             ControlAttitudeBellyFlopSetting::BellyFlopMode::BellyFlopMode_Upright) {
    flapSetting.flapTLAngle_Rad = 0;
    flapSetting.flapTRAngle_Rad = 0;
    flapSetting.flapBLAngle_Rad = 90 * DEGREES;
    flapSetting.flapBRAngle_Rad = 90 * DEGREES;
  } else {
    flapSetting.flapTLAngle_Rad = 90 * DEGREES * !controlSettings.extentFlapsOnAscent;
    flapSetting.flapTRAngle_Rad = 90 * DEGREES * !controlSettings.extentFlapsOnAscent;
    flapSetting.flapBLAngle_Rad = 90 * DEGREES * !controlSettings.extentFlapsOnAscent;
    flapSetting.flapBRAngle_Rad = 90 * DEGREES * !controlSettings.extentFlapsOnAscent;
  }
  flapSetting.enableFlaps = true;
  return flapSetting;
}

static uint32_t lcg(uint32_t &s) {
  s = s * 1664525u + 1013904223u;
  return s >> 8;
}
static float uni(uint32_t &s, float lo, float hi) { return lo + (hi - lo) * (lcg(s) / float(1u << 24)); }

static bool sameBits(const ControlAttitudeFlapSetting &a, const ControlAttitudeFlapSetting &b) {
  return memcmp(&a.flapTLAngle_Rad, &b.flapTLAngle_Rad, sizeof(float)) == 0 &&
         memcmp(&a.flapTRAngle_Rad, &b.flapTRAngle_Rad, sizeof(float)) == 0 &&
         memcmp(&a.flapBLAngle_Rad, &b.flapBLAngle_Rad, sizeof(float)) == 0 &&
         memcmp(&a.flapBRAngle_Rad, &b.flapBRAngle_Rad, sizeof(float)) == 0 && a.enableFlaps == b.enableFlaps;
}

/// Stabilize, Upright and Acsent outputs are bit-identical to the pre-Descent code over random attitudes (belly
/// angles 0..180 deg, so the fade is exercised), rates and setpoints, whatever the Descent fields hold.
static void test_legacy_modes_bit_identical() {
  using Mode = ControlAttitudeBellyFlopSetting::BellyFlopMode;
  const FlapStabilizeParams stab;
  const D::FlapDescentLawParams dp;
  D::FlapDescentLawMem dm;
  uint32_t s = 12345;
  int fadeCases = 0, stabCases = 0;
  for (int n = 0; n < 3000; n++) {
    float q[4] = {uni(s, -1, 1), uni(s, -1, 1), uni(s, -1, 1), uni(s, -1, 1)};
    if (n % 3 != 1) { // near belly-down (belly angle small to moderate)
      float qb[4];
      bellyQuat(uni(s, -3.1f, 3.1f), qb);
      for (int i = 0; i < 4; i++)
        q[i] = qb[i] + uni(s, -0.2f, 0.2f);
    }
    const float nq = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    Math::Vector<float, 7> att;
    att(0) = uni(s, -3, 3), att(1) = uni(s, -3, 3), att(2) = uni(s, -3, 3);
    for (int i = 0; i < 4; i++)
      att(3 + i) = q[i] / nq;
    ControlAttitudeBellyFlopSetting set;
    set.azimuthAngle_Rad = uni(s, -4, 4);
    set.pitchAngle_Rad = uni(s, -1, 1);
    set.rollAngle_Rad = uni(s, -1, 1);
    set.extentFlapsOnAscent = lcg(s) & 1;
    set.pitchSplit_Rad = uni(s, -1.4f, 0.2f); // must have no influence
    const Mode modes[3] = {Mode::BellyFlopMode_Acsent, Mode::BellyFlopMode_Stabilize, Mode::BellyFlopMode_Upright};
    set.bellyFlopMode = modes[n % 3];
    const ControlAttitudeFlapSetting ref = legacyMap(set, att);
    const ControlAttitudeFlapSetting out = FlapModeMapper::map(set, att, stab, dp, dm);
    TEST_ASSERT_TRUE(sameBits(ref, out));
    if (set.bellyFlopMode == Mode::BellyFlopMode_Stabilize) {
      stabCases++;
      // Fade region reached: belly angle between 35 and 55 deg (body X vs world up)
      const float bx = 2 * (att(4) * att(6) - att(3) * att(5)); // R(q) e_x, world z component
      const float belly = acosf(fmaxf(-1.0f, fminf(1.0f, bx)));
      if (belly > 35 * kDeg && belly < 55 * kDeg)
        fadeCases++;
    }
  }
  TEST_ASSERT_TRUE(stabCases > 900);
  TEST_ASSERT_TRUE(fadeCases > 20); // the fade-to-neutral region was exercised
}

static void test_mode_values_unchanged() {
  using Mode = ControlAttitudeBellyFlopSetting::BellyFlopMode;
  TEST_ASSERT_EQUAL_INT(0, (int)Mode::BellyFlopMode_Acsent);
  TEST_ASSERT_EQUAL_INT(1, (int)Mode::BellyFlopMode_Stabilize);
  TEST_ASSERT_EQUAL_INT(2, (int)Mode::BellyFlopMode_Upright);
  TEST_ASSERT_EQUAL_INT(3, (int)Mode::BellyFlopMode_Descent);
}

static Math::Vector<float, 7> attVec(float psi) {
  float a[7];
  makeAtt(psi, a);
  Math::Vector<float, 7> v;
  for (int i = 0; i < 7; i++)
    v(i) = a[i];
  return v;
}

static void test_descent_mode_mapping() {
  using Mode = ControlAttitudeBellyFlopSetting::BellyFlopMode;
  const FlapStabilizeParams stab;
  const D::FlapDescentLawParams dp;
  D::FlapDescentLawMem dm;
  ControlAttitudeBellyFlopSetting set;
  set.bellyFlopMode = Mode::BellyFlopMode_Descent;
  set.azimuthAngle_Rad = 0.5f;
  set.pitchSplit_Rad = 0;
  // Aligned, no rates, zero split: neutral 30/78 (TL, TR, BL, BR).
  ControlAttitudeFlapSetting o = FlapModeMapper::map(set, attVec(0.5f), stab, dp, dm);
  TEST_ASSERT_TRUE(o.enableFlaps);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 30 * kDeg, o.flapTLAngle_Rad);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 30 * kDeg, o.flapTRAngle_Rad);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 78 * kDeg, o.flapBLAngle_Rad);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 78 * kDeg, o.flapBRAngle_Rad);
  // Equals a direct call of the law, for a heading error and a split.
  set.azimuthAngle_Rad = 1.2f;
  set.pitchSplit_Rad = -30 * kDeg;
  D::FlapDescentLawMem dm2, dmDirect;
  o = FlapModeMapper::map(set, attVec(0.5f), stab, dp, dm2);
  float att[7];
  makeAtt(0.5f, att);
  D::FlapDescentCmd c;
  c.headingSp_rad = 1.2f, c.pitchSplit_rad = -30 * kDeg;
  D::FlapDescentLawOut ref = D::FlapDescentLaw::step(dp, att, c, dmDirect);
  TEST_ASSERT_EQUAL_FLOAT(ref.flap[0], o.flapTLAngle_Rad);
  TEST_ASSERT_EQUAL_FLOAT(ref.flap[1], o.flapTRAngle_Rad);
  TEST_ASSERT_EQUAL_FLOAT(ref.flap[2], o.flapBLAngle_Rad);
  TEST_ASSERT_EQUAL_FLOAT(ref.flap[3], o.flapBRAngle_Rad);
  // Forward split: the top pair is more retracted (larger angle) than at neutral.
  TEST_ASSERT_TRUE(o.flapTLAngle_Rad > 45 * kDeg || o.flapTRAngle_Rad > 45 * kDeg);
  // Differs from Stabilize for the same input (Stabilize neutral is 50/40).
  set.bellyFlopMode = Mode::BellyFlopMode_Stabilize;
  ControlAttitudeFlapSetting st = FlapModeMapper::map(set, attVec(0.5f), stab, dp, dm);
  TEST_ASSERT_TRUE(fabsf(st.flapBLAngle_Rad - o.flapBLAngle_Rad) > 10 * kDeg);
  // Invalid attitude in Descent mode: neutral 30/78.
  set.bellyFlopMode = Mode::BellyFlopMode_Descent;
  Math::Vector<float, 7> bad = attVec(0.5f);
  bad(4) = kNaN;
  o = FlapModeMapper::map(set, bad, stab, dp, dm);
  TEST_ASSERT_EQUAL_FLOAT(30 * kDeg, o.flapTLAngle_Rad);
  TEST_ASSERT_EQUAL_FLOAT(78 * kDeg, o.flapBRAngle_Rad);
}

static void test_descent_memory_reset_on_mode_change() {
  using Mode = ControlAttitudeBellyFlopSetting::BellyFlopMode;
  const FlapStabilizeParams stab;
  const D::FlapDescentLawParams dp;
  D::FlapDescentLawMem dm;
  dm.lastHeadingErr = 2.0f;
  ControlAttitudeBellyFlopSetting set;
  set.bellyFlopMode = Mode::BellyFlopMode_Stabilize;
  FlapModeMapper::map(set, attVec(0.0f), stab, dp, dm);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, dm.lastHeadingErr);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_law_neutral_at_rest);
  RUN_TEST(test_law_nan_attitude_gives_neutral);
  RUN_TEST(test_law_nan_command_finite);
  RUN_TEST(test_law_corrupt_memory_recovers);
  RUN_TEST(test_guidance_valid_flies_toward_target);
  RUN_TEST(test_guidance_invalid_input_holds_heading);
  RUN_TEST(test_guidance_recovers_after_invalid_input);
  RUN_TEST(test_guidance_invalid_target_and_corrupt_memory);
  RUN_TEST(test_guidance_init_with_nan_attitude);
  RUN_TEST(test_legacy_modes_bit_identical);
  RUN_TEST(test_mode_values_unchanged);
  RUN_TEST(test_descent_mode_mapping);
  RUN_TEST(test_descent_memory_reset_on_mode_change);
  return UNITY_END();
}
