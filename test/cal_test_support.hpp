// Shared helpers of the calibration host tests (test_cal_*): deterministic random numbers and synthetic sensor data.
#ifndef CAL_TEST_SUPPORT_HPP
#define CAL_TEST_SUPPORT_HPP

#include <math.h>
#include <stdint.h>

namespace CalTest {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

/// Deterministic LCG with Box-Muller normal deviates.
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed = 12345) : s(seed) {}
  double uniform() { // (0, 1)
    s = s * 6364136223846793005ULL + 1442695040888963407ULL;
    return (double((s >> 11) & ((1ULL << 53) - 1)) + 0.5) / double(1ULL << 53);
  }
  double normal() {
    const double u1 = uniform(), u2 = uniform();
    return sqrt(-2.0 * log(u1)) * cos(2 * kPi * u2);
  }
  /// Uniformly distributed unit vector.
  void unit(double v[3]) {
    const double z = 2 * uniform() - 1, phi = 2 * kPi * uniform(), r = sqrt(1 - z * z);
    v[0] = r * cos(phi);
    v[1] = r * sin(phi);
    v[2] = z;
  }
};

inline void normalise(double v[3]) {
  const double n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] /= n;
  v[1] /= n;
  v[2] /= n;
}

/// Direction from nose angle about the vertical and roll: the 12-pose table of calibration-design.md 2.4
/// (sensor z = nose). tiltDeg 0 = nose up, 90 = horizontal, 180 = nose down; rollDeg turns about the long axis.
inline void poseDirection(double tiltDeg, double rollDeg, double u[3]) {
  const double t = tiltDeg * kDeg, r = rollDeg * kDeg;
  u[0] = sin(t) * cos(r);
  u[1] = sin(t) * sin(r);
  u[2] = cos(t);
}

/// The 12 poses of the design (with deliberately imprecise angles when jitterDeg > 0).
inline void twelvePoses(double (*u)[3], Rng &rng, double jitterDeg) {
  const double table[12][2] = {{0, 0},    {180, 0},  {90, 0},   {90, 90},  {90, 180}, {90, 270},
                               {45, 0},   {45, 90},  {45, 180}, {45, 270}, {135, 0},  {135, 180}};
  for (int i = 0; i < 12; i++) {
    const double jt = jitterDeg * (2 * rng.uniform() - 1), jr = jitterDeg * (2 * rng.uniform() - 1);
    poseDirection(table[i][0] + jt, table[i][1] + jr, u[i]);
  }
}

/// raw = Ainv * (g u) + b, the inverse of the model a_cal = A (raw - b).
inline void rawFromTrue(const double Ainv[9], const double b[3], const double v[3], double raw[3]) {
  for (int r = 0; r < 3; r++)
    raw[r] = Ainv[3 * r] * v[0] + Ainv[3 * r + 1] * v[1] + Ainv[3 * r + 2] * v[2] + b[r];
}

} // namespace CalTest

#endif
