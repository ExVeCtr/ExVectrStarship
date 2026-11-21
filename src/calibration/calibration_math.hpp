#ifndef CALIBRATION_MATH_HPP
#define CALIBRATION_MATH_HPP

/**
 * Tiny row-major 3x3 double helpers of the calibration code (host-testable, no dependencies).
 */

#include <math.h>
#include <stddef.h>

namespace VCTR {
namespace Calibration {

inline void mat3Identity(double m[9]) {
  for (int i = 0; i < 9; i++)
    m[i] = (i % 4 == 0) ? 1.0 : 0.0;
}

inline void mat3Mul(const double a[9], const double b[9], double out[9]) {
  double t[9];
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      t[3 * r + c] = a[3 * r] * b[c] + a[3 * r + 1] * b[3 + c] + a[3 * r + 2] * b[6 + c];
  for (int i = 0; i < 9; i++)
    out[i] = t[i];
}

inline void mat3Vec(const double m[9], const double v[3], double out[3]) {
  double t[3];
  for (int r = 0; r < 3; r++)
    t[r] = m[3 * r] * v[0] + m[3 * r + 1] * v[1] + m[3 * r + 2] * v[2];
  for (int i = 0; i < 3; i++)
    out[i] = t[i];
}

/// Inverse by cofactors. Returns false (out untouched) if singular or non-finite.
inline bool mat3Inverse(const double m[9], double out[9]) {
  const double c00 = m[4] * m[8] - m[5] * m[7];
  const double c01 = m[5] * m[6] - m[3] * m[8];
  const double c02 = m[3] * m[7] - m[4] * m[6];
  const double det = m[0] * c00 + m[1] * c01 + m[2] * c02;
  if (!(fabs(det) > 1e-12) || !isfinite(det))
    return false;
  const double id = 1.0 / det;
  double t[9];
  t[0] = c00 * id;
  t[1] = (m[2] * m[7] - m[1] * m[8]) * id;
  t[2] = (m[1] * m[5] - m[2] * m[4]) * id;
  t[3] = c01 * id;
  t[4] = (m[0] * m[8] - m[2] * m[6]) * id;
  t[5] = (m[2] * m[3] - m[0] * m[5]) * id;
  t[6] = c02 * id;
  t[7] = (m[1] * m[6] - m[0] * m[7]) * id;
  t[8] = (m[0] * m[4] - m[1] * m[3]) * id;
  for (int i = 0; i < 9; i++)
    out[i] = t[i];
  return true;
}

inline double vec3Norm(const double v[3]) { return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

} // namespace Calibration
} // namespace VCTR

#endif
