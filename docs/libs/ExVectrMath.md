# ExVectrMath

> Header-only, fixed-size linear algebra (matrix, vector, quaternion) for the ExVectr framework. Repo: https://github.com/ExVeCtr/ExVectrMath.git. `Last verified against commit: b221829`

## Purpose & scope

- Compile-time-sized dense matrices, column vectors and unit quaternions, all templated on the scalar type. No heap allocation and no dynamic sizes.
- Used by every estimator (ExVectrDSP), every controller (ExVectrControl) and the application (`src/`) for frame transforms and Kalman algebra.
- Not provided: decompositions (LU/QR/Cholesky), pseudo-inverse, solvers, Euler-angle conversions, quaternion-from-rotation-matrix (the constructor is commented out), slerp. The README roadmap lists "Solvers (Gauss)" as TODO.
- `src/ExVectrMath.cpp` is empty. Everything lives in `include/ExVectrMath/*.hpp`.

## Dependencies (ExVectr libs + external)

- ExVectr: none. The README claims Core and HAL are needed; the code includes neither.
- External: `<stdint.h>`, `<math.h>` functions (`sqrt`, `sin`, `cos`, `acos`, `::pow`), `std::initializer_list`, and `M_PI` (POSIX/Arduino, not ISO C++).
- Umbrella header `ExVectrMath.hpp` includes `matrix_base.hpp`, `specialised_types.hpp`, `matrix_vector.hpp`, `matrix_quaternion.hpp` and `constants.hpp`.

## Key concepts & API

All types are in `namespace VCTR::Math`, except `VCTR::DEGREES`.

### `Matrix<TYPE, ROWS, COLS>` (`matrix_base.hpp`)
- Storage: public `TYPE r[ROWS][COLS]`, **row-major**. `m[row][col]` is unchecked. `m(row, col)` wraps indices modulo the size; this is not bounds-checked, it silently aliases. `m(i)` is a linear row-major index, also modulo.
- Constructors:
  - `Matrix()` zero-fills.
  - **`Matrix(TYPE v)` sets the diagonal to `v` and everything else to 0** (`:349`). So `Matrix<float,3,3>(1)` is identity, `Matrix<float,3,1>(1)` is `(1,0,0)`, and `= 0` zeroes.
  - `Matrix({a,b,...})` takes a row-major initializer list. If the list size is not exactly ROWS*COLS the constructor returns early and leaves the matrix **uninitialised** (`:371`).
  - Converting constructor from another `TYPE2`.
  - `Matrix(otherMat, rowStart, colStart)` zero-fills and then pastes `otherMat` as a block.
- Static: `eye(val=1)`, `fill(val)`.
- Algebra: `+ - *` (matrix product, scalar product), `/ scalar`, unary `-`. Free operators `scalar*M`, `scalar/M` (elementwise reciprocal) and **`scalar - M`, which means `scalar*I - M`**. `transpose()`, `inverse()`, `diagonal()`, `pow(n)`, `ePow(iter)`, `determinant()`, `==` / `!=` (exact compare). Matrix products require the same `TYPE` on both sides.
- Vector-only (COLS==1, enforced by `static_assert`):
  - `magnitude(startRow=0, endRow=ROWS)` uses a half-open row range.
  - `normalize(startRow, endRow, zeroReturn=true)` returns the **zero vector** if the magnitude is below 1e-4.
  - `cross(v)` is for 3x1 only.
- Blocks:
  - `block<R2,C2>(rowStart, colStart)` extracts a block.
  - `block(mat, rowStart, colStart, rowFromStart, colFromStart, rowFromNum, colFromNum)` pastes into `*this` (clipped to bounds) and returns `*this&`.
- `printTo(printf-like fn)`.

### `Vector<TYPE, N>` (`matrix_vector.hpp`)
- Nx1 `Matrix`.
- **`Vector(TYPE v)` fills every element with `v`** (`:89`). This differs from `Matrix(TYPE)`.
- The `Vector({..})` initializer list has **no size check** (`:96`). A list longer than N writes out of bounds.
- `dot(v)`.
- `getAngleTo(v)` returns [0, π], clamped, and 0 if either vector is near zero.
- `getProjectionOn(v, isSurfaceNormal=false)`: with `false`, the projection onto `v`; with `true`, the projection onto the plane whose normal is `v`. NaN if `v` is zero.
- Aliases: `Vector_F = Vector<float,3>`, `Vector_D = Vector<double,3>`.

### `Quat<TYPE>` (`matrix_quaternion.hpp`)
- A 4x1 `Matrix` stored **`[w, x, y, z]`** (`r[0][0]` = w).
- Constructors:
  - `Quat()` is identity.
  - `Quat(w,x,y,z)`.
  - `Quat(axis, angle)` (`:170`). The axis must be unit length; it is not normalised.
  - `Quat(Matrix<4,1>)`.
  - `Quat(Matrix<3,1> v)` gives the pure quaternion `[0, v]`.
- `fromRotVec(rv)` treats rv as axis*angle. It returns identity when |rv| = 0, detected by a NaN self-compare (`:210`).
- `toRotVec()` (`:230`).
- `conjugate()`.
- `normalize()` returns **identity** if |q| < 1e-4 (`:268`).
- `rotate(v)` (`:247`).
- `operator*(Quat)` is the Hamilton product (`:280`). `* scalar` and `/ scalar` are elementwise.
- `to3x3RotMat()` (`:329`) gives the active rotation matrix R(q), so that `R*v == q.rotate(v)`. Implicit conversions exist to `Matrix<3,3>` (R), `Vector<3>` (xyz part) and `Matrix<4,1>`.
- Unary `-q` returns a `Matrix<4,1>`; assign it back to a `Quat`.
- Aliases: `Quat_F`, `Quat_D`.

### Other
- `Matrix_F<R,C>`, `Matrix_D<R,C>` (`specialised_types.hpp`).
- `constants.hpp`:
  - `Math::GRAVITY = 9.807f` m/s².
  - `Math::GRAVITY_3F = {0,0,+9.807}`. It points **up**: it is the specific force an accelerometer at rest reads, in world frame.
  - `VCTR::DEGREES = π/180`, used as `30*DEGREES`.

## Usage

```cpp
#include "ExVectrMath.hpp"
using namespace VCTR;

Math::Quat_F q(Math::Vector_F{0, 1, 0}, -90 * DEGREES);  // axis must be unit
Math::Vector_F vWorld = q.rotate(vBody);                 // body -> world
Math::Vector_F vBody2 = q.conjugate().rotate(vWorld);    // world -> body
auto R = q.to3x3RotMat();                                // R * vBody == vWorld
Math::Quat_F qErrBody = q.conjugate() * qWanted;         // error in body frame

// Kalman-style algebra (as in imu_gps_position_kf.cpp)
Math::Matrix<float, 6, 6> P = Math::Matrix<float, 6, 6>::eye();
auto H = Math::Matrix<float, 2, 6>({0,0,1,0,0,0, 0,0,0,0,0,1});
auto K = P * H.transpose() * (H * P * H.transpose() + R2).inverse();
P = (Math::Matrix<float, 6, 6>(1) - K * H) * P;

// state slicing (as everywhere in DSP/Control)
Math::Vector<float, 7> att;                      // [w_body(3), q(4)]
Math::Quat_F qa = att.block<4, 1>(3, 0);
att.block(qa, 3, 0);                             // write back
```

## Conventions

**This section is the authoritative convention reference for the whole project.** Other docs defer to it.

- **Storage:** row-major `r[row][col]`. Initializer lists are row-major. Vectors are columns (Nx1).
- **Quaternion layout:** `[w, x, y, z]`, scalar first. Attitude state vectors carry it at indices 3..6.
- **Multiplication (since commit `b221829`, "Fixed quaternion multiplication", 2026-01-29):** `a * b` is the standard **Hamilton** product `a⊗b`.
  - **Before** (≤ `8b14fea`), `a * b` computed `b⊗a`. As a result `q.rotate(v)` computed `q* v q`, the *inverse* rotation.
  - ExVectrDSP `3e1f070` and ExVectrControl `084bea1` were written and tuned on 2026-01-25, i.e. against the **old** semantics. See their Known-issues tables for the affected call sites.
  - `src/main.cpp:196-199` contains a sanity print: `Quat(z, 90°).rotate(x)` should be `[0,1,0]`. That holds with the current code.
- **`rotate()`:** `q.rotate(v) = q ⊗ [0,v] ⊗ q* = R(q)·v`. This is an **active** rotation of `v` by `q`.
- **Attitude quaternion meaning (as the maths actually behaves):** `q` maps **body → world**, i.e. `v_world = q.rotate(v_body) = R(q) v_body`. The doc comments throughout DSP/Control say "rotation from the reference frame to body frame"; treat that as a naming convention only.
  - Consistent consequences, which the code-independent explicit formulas in the EKF and position KF use:
    - Gyro kinematics are `q̇ = ½ q ⊗ [0, ω_body]`.
    - The expected gravity direction in body frame is `Rᵀ e_z`.
    - A body-frame increment right-multiplies: `q ← q ⊗ Δq`.
    - A world-frame increment left-multiplies: `q ← Δq ⊗ q`.
    - The body-frame attitude error is `q* ⊗ q_d`.
- **Rotation direction:** right-handed. A positive angle about an axis rotates counter-clockwise when you look down the axis toward the origin.
- **World/reference frame:** **X = North, Y = West, Z = Up** (NWU, right-handed), with the origin at the reference/launch point. Gravity acceleration is `-GRAVITY_3F`.
  - GNSS lat/lon are radians (`ExVectrSensor/gnss.hpp:22`).
  - Azimuths computed with `atan2(y, x)` in this frame increase counter-clockwise, toward West. That is *not* a compass heading.
- **Vehicle body frame**, as used by Control and the simulator:
  - **+Z** is the long axis toward the nose. The TVC/engine is at `(0,0,-0.35)` m (`src/main.cpp:1967`, `body_simulator.cpp:34`). The top flaps are at z = +0.26 and the bottom flaps at z = -0.26 (`body_simulator.cpp:152`).
  - **±Y** are the flap sides: TL/BL at +Y, TR/BR at −Y.
  - **+X** completes the right-handed frame.
  - In bellyflop, body +X points world-up and the belly is −X. The bellyflop setpoint `Quat(y, −90°)` (`src/mission/mission_bellyflop.hpp:102`) maps body +X to world +Z and the nose to world −X.
  - TVC output vectors are body-frame force; `+Z` is pure thrust.
- **Units:** SI (m, m/s, m/s², rad, rad/s, N, N·m, kg). Time is Core `int64_t` nanoseconds (`Core::SECONDS = 1e9`).
- **Angles:** use `x * DEGREES` to convert to radians.

## How this project uses it

- Used everywhere: all of DSP and Control, plus `src/main.cpp`, `src/starship_tvc.cpp`, `src/starship_flaps.cpp` and `src/mission/*`.
- Quaternion call sites in `src/` whose behaviour changed with `b221829` (every `rotate()` now applies the opposite rotation to before, and every `a*b` now composes in the opposite order):
  - `src/main.cpp:198` is a sanity test and is correct now.
  - `src/main.cpp:1117`: tilt check `attitude.rotate(z)`. Body z in world is correct now.
  - `src/main.cpp:1516`, `:1520`: `attQuat.rotate(...)` used for down angle and heading.
  - `src/main.cpp:1976`: `(accTiltX * accTiltY).to3x3RotMat()`. The composition order is flipped. Both angles are currently 0, so there is no effect yet.
  - `src/starship_tvc.cpp:88-91`: TVC angle clamp. The double conjugate gives `Quat(f×z, α−limit).rotate(f)`, which is correct now: it moves `f` toward +Z.
  - `src/mission/mission_bellyflop.hpp:109`: `attIs.conjugate().rotate(x)` is now *world-X expressed in body*, not body-X in world. Verify that this is intended for the 50°/150° transition angles.

## Known issues & limitations

| Severity | Location (path:line) | Issue | Impact |
|---|---|---|---|
| High | `include/ExVectrMath/matrix_quaternion.hpp:280`, `:247` | The `operator*` order and therefore `rotate()` changed in `b221829` (it used to be `b⊗a` and inverse-rotate). The parent repo pins `b221829` since commit 6b5134a (Nov 2026). | Every DSP/Control/src call site built on the old semantics may now be wrong. See the per-lib tables. |
| High | `include/ExVectrMath/matrix_base.hpp:493`, `:524`, `:539` | `inverse()` is Gauss-Jordan with no partial pivoting. It only bubble-sorts rows by column 0, then divides by `matrix[i][i]`. | Divides by zero or produces garbage for valid but badly ordered matrices, e.g. any covariance with a zero leading pivot. Every KF `S.inverse()` depends on it. |
| Med | `include/ExVectrMath/matrix_base.hpp:349` vs `include/ExVectrMath/matrix_vector.hpp:89` | `Matrix(T v)` is `v*I`, but `Vector(T v)` fills every element. | `Matrix<T,N,1>(x)` sets only element 0. Easy to get wrong in defaults such as `ValueCov`. |
| Med | `include/ExVectrMath/matrix_base.hpp:640-642` | `ePow`: the factorial loop starts at `j=0`, so the factorial is 0 and the code divides by zero for every term `i ≥ 1`. | Returns inf/NaN. Currently unused. |
| Med | `include/ExVectrMath/matrix_quaternion.hpp:230` | `toRotVec()` divides by `|xyz|` (NaN at identity), calls `acos(w)` with no clamp (NaN if \|w\| > 1 from rounding), and ignores w<0 (angle > π). | NaN propagation. Currently unused. |
| Low | `include/ExVectrMath/matrix_base.hpp:676` | `determinant()` instantiates `Matrix<ROWS-1,COLS-1>` recursively down to `Matrix<0,0>`, which hits a `static_assert`. | Does not compile for any size if called. Unused. |
| Low | `include/ExVectrMath/matrix_base.hpp:371` | The initializer-list constructor returns early when the size is wrong and leaves `r` uninitialised. | Silent garbage, no diagnostic. |
| Low | `include/ExVectrMath/matrix_vector.hpp:96` | The `Vector` initializer list has no size check. | Out-of-bounds write if the list has more than N elements. |
| Low | `include/ExVectrMath/matrix_quaternion.hpp:210` | `fromRotVec` detects angle = 0 via a NaN self-compare. | The check is optimised away under `-ffast-math`, which then gives NaN. |
| Low | `include/ExVectrMath/matrix_quaternion.hpp:268`; `matrix_base.hpp` `normalize` | Quat `normalize()` silently returns identity near zero. Vector `normalize()` returns the zero vector. | Masks upstream errors. E.g. EKF accel normalisation in free fall yields a zero vector. |
| Low | `include/ExVectrMath/matrix_base.hpp` `operator()(row,col)` | Indices wrap modulo the size. | Out-of-range access aliases silently instead of failing. |
| Low | `include/ExVectrMath/matrix_base.hpp:1003` | `operator=` returns by value instead of by reference. | Extra copy; chained assignment semantics are odd. |
| Low | `include/ExVectrMath/constants.hpp:23` | `DEGREES` relies on the non-standard `M_PI`. | Portability only. |
| Low | `README.md` | Lists Core/HAL as dependencies; the roadmap is stale. | Documentation only. |

## Extending

- Add free functions or members in the relevant header. Keep everything header-only and fixed-size, and add `static_assert`s for shape requirements, as the existing code does.
- New quaternion utilities must follow the Conventions above: Hamilton product, `[w,x,y,z]`, `rotate = R(q)v`, attitude q = body→world. Add a sanity check like `src/main.cpp:196-199` for anything frame-related.
- Priority gaps:
  - A pivoting `inverse()`, or a Cholesky solve for SPD innovation covariances.
  - A Joseph-form covariance-update helper.
  - Fixes for `ePow`/`determinant`.
  - `Quat(R)` and Euler conversions.
- After changing semantics, bump the submodule pointer in the parent repo and re-verify every call site listed in the DSP and Control Known-issues tables.
