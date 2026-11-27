// S6 / decision D7: the reduced 8-strip aero config against the 24-strip config over the envelope
// (airspeed 0-15 m/s, alpha 0-180 deg, flaps 0-90 deg, with and without body rates).
// Criterion: force error <= 5 % and moment error <= 10 %, each relative to the peak 24-strip magnitude at the same
// airspeed (a pointwise ratio is meaningless where a component crosses zero). The pointwise relative error over the
// points above 10 % of that peak is reported as well. Run: pio test -e native -f test_aero_reduced -v
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "sim/starship_plant.hpp"

using namespace VCTR;
using Dyn::Vec3;

void setUp() {}
void tearDown() {}

static float norm3(const Vec3 &v) { return sqrtf(v(0) * v(0) + v(1) * v(1) + v(2) * v(2)); }

static void sweep(int strips, float &maxF, float &maxM, float &maxFp, float &maxMp) {
  const ShipSim::StarshipPlantParams p;
  const ShipSim::AeroConfig full = ShipSim::makeAeroConfig(p);
  const ShipSim::AeroConfig red = ShipSim::makeReducedAeroConfig(p, strips);
  const float speeds[] = {0.5f, 1.0f, 2.0f, 4.0f, 6.4f, 10.0f, 15.0f};
  const float rates[][3] = {{0, 0, 0}, {2.0f, -1.0f, 0.5f}, {-4.0f, 3.0f, 1.0f}};
  const float kDeg = 3.14159265f / 180.0f;
  const float flaps[][4] = {{0, 0, 0, 0},         {0.5f, 0.5f, 0.5f, 0.5f}, {1.0f, 1.0f, 1.0f, 1.0f},
                            {1.5708f, 1.5708f, 1.5708f, 1.5708f}, {0.3f, 1.2f, 0.9f, 0.1f}, {1.3f, 0.2f, 0.4f, 1.0f}};
  maxF = maxM = maxFp = maxMp = 0.0f;
  for (float sp : speeds)
    for (const auto &w : rates) {
      struct Pt { float df, dm, f, m; };
      static Pt pts[40000];
      int n = 0;
      float peakF = 0, peakM = 0;
      for (int ai = 0; ai <= 12; ai++)
        for (int az = 0; az < 4; az++)
          for (const auto &fl : flaps) {
            const float al = ai * 15.0f * kDeg, ph = az * 45.0f * kDeg;
            ShipSim::AeroInput in;
            in.airVelBody = {sp * sinf(al) * cosf(ph), sp * sinf(al) * sinf(ph), sp * cosf(al)};
            in.omegaBody = {w[0], w[1], w[2]};
            for (int i = 0; i < 4; i++) in.flapAngle[i] = fl[i];
            in.rho = 1.2f;
            ShipSim::AeroOutput a, b;
            ShipSim::starshipAero(full, in, a);
            ShipSim::starshipAero(red, in, b);
            Pt q;
            q.f = norm3(a.total.force);
            q.m = norm3(a.total.torque);
            q.df = norm3(b.total.force - a.total.force);
            q.dm = norm3(b.total.torque - a.total.torque);
            if (q.f > peakF) peakF = q.f;
            if (q.m > peakM) peakM = q.m;
            if (n < 40000) pts[n++] = q;
          }
      for (int i = 0; i < n; i++) {
        if (pts[i].df / peakF > maxF) maxF = pts[i].df / peakF;
        if (pts[i].dm / peakM > maxM) maxM = pts[i].dm / peakM;
        if (pts[i].f > 0.1f * peakF && pts[i].df / pts[i].f > maxFp) maxFp = pts[i].df / pts[i].f;
        if (pts[i].m > 0.1f * peakM && pts[i].dm / pts[i].m > maxMp) maxMp = pts[i].dm / pts[i].m;
      }
    }
}

void test_reduced_strips_within_tolerance() {
  const int counts[] = {4, 6, 8, 12, 16};
  for (int s : counts) {
    float f, m, fp, mp;
    sweep(s, f, m, fp, mp);
    printf("AEROREDUCED strips=%d force_peak_rel=%.3f%% moment_peak_rel=%.3f%% force_pointwise=%.2f%% moment_pointwise=%.2f%%\n",
           s, 100 * f, 100 * m, 100 * fp, 100 * mp);
    if (s == 8) {
      TEST_ASSERT_TRUE_MESSAGE(f <= 0.05f, "8 strips: force error above 5 %");
      TEST_ASSERT_TRUE_MESSAGE(m <= 0.10f, "8 strips: moment error above 10 %");
    }
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_reduced_strips_within_tolerance);
  return UNITY_END();
}
