# ExVectrDSP

> State estimation and signal processing for ExVectr: IMU attitude EKF, IMU/baro/GNSS/range position KF, sensor-frame transforms and calibration, generic KF and filters, plus a rigid-body flight simulator. Repo: https://github.com/ExVeCtr/ExVectrDSP. `Last verified against commit: 3e1f070 (pinned now: fc99b99 = 3e1f070 plus a mechanical NOW() -> NowNs() port for ExVectrCore 18bbb09; content otherwise as verified; feature/sim-aero a5c21af, 2026-11-21: body simulator section re-verified against the redesign)`

## Purpose & scope

- Turns raw sensor topics into the two estimate topics the rest of the system consumes:
  - **attitude** `[ω_body(3), q(4)]`
  - **position** `[v(3), p(3)]` in NWU
  - plus a ground-distance estimate.
- Provides `TopicCoordTransform` (sensor→body plus calibration) and calibrators for gyro and magnetometer.
- Contains `BodySimulator`, a simulator task that publishes the same topic types as the estimators, so the application can switch between real and simulated state, around the pure, host-tested physics model `BodySimModel` (2026-11-21).
- Generic helpers: `KalmanFilter<LEN>`, `ValueCov`, first-order `LowPassFilter` and `HighPassFilter`.
- Not implemented:
  - `FullStateEKF` is declared only, with no `.cpp`.
  - `gnss_data.hpp` is empty.
  - FFT (README TODO).
  - `imu_attitude_cf` is not a complementary filter; it is an EKF variant.
- The local branch is 1 commit behind `origin/Development` (`402831f`). The parent repo pins `15512dc`, which is 1 commit behind the checked-out HEAD.

## Dependencies (ExVectr libs + external)

- **ExVectrCore**: `Topic`, `Simple_Subscriber`, `Buffer_Subscriber`, `Callback_Subscriber`, `Task_Periodic`, `Timestamped`, `ListBuffer`, `NowNs()`, `LOG_MSG` (`randGen` no longer used since the simulator redesign).
- **ExVectrMath**: all algebra. Frame conventions come from `docs/libs/ExVectrMath.md`.
- **ExVectrSensor**: `SNSR::GNSSData` (`gnss.hpp`). Lat/lon in rad, alt in m above the ellipsoid, velocity documented as NED (`imu_gps_position_kf.hpp:128`).
- **ExVectrData**: `Data::Memory_Interface` and memory manager, used by `Calibrator_Manager`.
- **ExVectrControl** (**circular**): `body_sim_model.hpp` includes `ExVectrControl/physics_bodysim.hpp` and `body_simulator.hpp` includes `starship/control_attitude_flaps.hpp`, while Control includes `ExVectrDSP/value_covariance.hpp`. PlatformIO resolves this because all headers are on the include path. The CMake build would need both libraries.
- External: `<math.h>` only.

## Key concepts & API

Common pattern:
- Each estimator class holds `Simple_Subscriber`s on input topics and exposes `get*Topic()` outputs.
- Its `*Task` subclass is a `Core::Task_Periodic` that registers itself in the constructor. `taskCheck()` un-pauses when any input is new; `taskThread()` runs `update()` and then re-pauses.
- Sensor inputs are `Topic<Timestamped<ValueCov<float,N>>>`. `getItem()` clears the "new" flag.
- Time is Core `int64_t` ns. Frames follow ExVectrMath Conventions: q = body→world, world = X-North/Y-West/Z-Up.

### `ValueCov<TYPE, N>` (`value_covariance.hpp`)
- Struct `{Vector<TYPE,N> val; Matrix<TYPE,N,N> cov;}`.
- Constructor arguments default to 0. Note that `cov = 1` means identity and `val = 1` means all ones (see the Math constructor asymmetry).
- Also abused to carry a calibration: `val` = bias, `cov` = transform matrix.

### `TopicCoordTransform<TYPE, DIM=3>` (`topic_coord_transform.hpp`)
- A `Callback_Subscriber` on the input topic, republishing `val = T * (val - bias)` with the same timestamp.
- **The covariance is passed through untransformed.**
- API: `setTransform(T, bias)`, `setInputTopic(topic)`, `getOutputTopic()`.

### `IMUAttitudeEKF` / `IMUAttitudeEKFTask` (`imu_attitude_ekf.hpp/.cpp`)
- **State** `x_` (7):

  | idx | 0..2 | 3..6 |
  |---|---|---|
  | content | gyro bias b (rad/s, body) | q = [w,x,y,z], body→world |

  `p_` is 7x7, but **only the 4x4 quaternion block `p_(3..6,3..6)` is propagated or updated**. The bias is not a filter state; it is set by heuristics.
- **Inputs** (all body frame, after `TopicCoordTransform`):
  - `setGyroInput` in rad/s.
  - `setAccInput` in m/s². Only the direction is used.
  - `setMagInput`, scaled so that |B| ≈ 0.05. In practice this is mT: the acceptance window is 0.01–0.1, even though the docs say tesla.
  - `setAccTiltCompensation(R3x3)` is applied in `updateAcc` only.
- **Outputs:**
  - `getAttitudeEstTopic()` publishes `Timestamped<Vector<7>>` = `[ω_meas − b (rad/s, body), q]`, timestamp `NowNs()`.
  - `getBiasEstTopic()` publishes `Timestamped<Vector<3>>` = b.
  - Both are published only when a gyro, accel or mag update ran.
- **`update()` order:**
  1. New gyro → `updateGyro`.
  2. `predict(NOW)`, which re-runs `updateGyro` with the last gyro sample.
  3. Accel: on first use `initialiseAcc`, which is gated on raw |gyro| < 0.1 rad/s; after that `updateAcc`.
  4. Mag, only after the accelerometer is initialised: on first use `initialiseMag`, after that `updateMag`.
  5. Symmetrise P, then publish.
- **Process model:** `q ← normalize(q + ½dt·q⊗[0,ω−b])`. Jacobians F (4x4) and W (4x3) are explicit and verified correct. `P = F P Fᵀ + W·(Σ_gyro·1000)·Wᵀ`.
- **Accel update:**
  - Measurement `h = Rᵀ e_z = [2(xz−wy), 2(wx+yz), 1−2(x²+y²)]`, with the matching 3x4 H.
  - `z = normalize(T_tilt·a)`, `R = Σ_acc·5000`.
  - Standard EKF update, renormalise with w ≥ 0.
  - No thrust or free-fall gating.
- **Mag update:** not an EKF. The field is rotated to world, projected onto the horizontal, `yaw = atan2(m_y, m_x)`, then `q ← q ⊗ Rz(−0.005·yaw)`.
- **Bias heuristics** in `updateGyro`, active only when raw |ω| < 10°/s:
  - With `enableZeroingMode(true)`: IIR `b ← 0.995 b + 0.005 ω_raw`.
  - Startup: the first 500 samples are meant to average the bias, but see the integer-division bug.
- **Tuning (hard-coded):**

  | Parameter | Value | Location |
  |---|---|---|
  | initial P | I | `:13` |
  | gyro cov inflation | ×1000 | `:126` |
  | accel cov inflation | ×5000 | `:245` |
  | bias IIR factor | 0.005 | `:135` |
  | bias motion gate | 10°/s | `:129` |
  | startup bias samples | 500 | `:142` |
  | accel-init gyro gate | 0.1 rad/s | `:51` |
  | mag window | 0.01–0.1 | `:327` |
  | mag yaw gain | 0.005 per update | `:347` |

  `setProcessNoise()` is a no-op.
- **Task:** `IMUAttitudeEKFTask(period)` uses `Task_Periodic("IMUAttitudeEKFTask", period, 5*SECONDS)`, so its start time is absolute t = 5 s. Main runs it at **1 ms**.

### `IMUAttitudeCF` / `IMUAttitudeCFTask` (`imu_attitude_cf.hpp/.cpp`)
- Same state, interfaces and topics as the EKF.
- Differences:
  - Gyro integration is by axis-angle quaternion, left-multiplied.
  - Accel R is scaled by `5000·(1 + |Rᵀa − g|/10)`.
  - Mag window 0.01–0.5, cov ×1000.
  - `initialiseAcc` uses `quat.rotate(g)`.
- **Unused by the project.** Several frame inconsistencies, listed below.

### `IMUGPSPositionKalman` / `IMUGPSPositionKalmanTask` (`imu_gps_position_kf.hpp/.cpp`)
- **State** `x_` (6), world NWU:

  | idx | 0 | 1 | 2 | 3 | 4 | 5 |
  |---|---|---|---|---|---|---|
  | content | v_N | v_W | v_U | p_N | p_W | p_U (m, relative to reference) |

  `p_` is 6x6, initially I.
- **Ground-distance sub-filter:** `groundDistanceEst_` = `[v_ground (m/s), d_ground (m)]`, with a 2x2 covariance.
- **Inputs:**
  - `setAttitudeInput`: `[ω, q]` topic. Only q is used (`att_`).
  - `setAccInput`: raw body accelerometer. Buffered with `Buffer_Subscriber<…,5>` and averaged per update. The `accMedian*` members are unused.
  - `setBaroInput(topic, seaLevelPressure=101325)`: Pa; the covariance is interpreted in m.
  - `setRangeInput(topic, min, max)`: m. The range is assumed to be vertical, with no tilt compensation.
  - `setGNSSInput(topic, refPos=0)`: if `refPos` is non-zero, GNSS counts as initialised.
- **Outputs:**
  - `getStateEstTopic()` (`Vector<6>`) and `getStateCovTopic()` (`Matrix<6,6>`), published after any acc/baro/GNSS update.
  - `getGroundDistanceEstTopic()` and `getGroundDistanceCovTopic()`, published after a range update.
- **Predict (`updateAcc`):**
  - `a_w = R(q)(a − b_acc) − GRAVITY_3F`.
  - `F` = constant-velocity model. `B = [dt·I; ½dt²·I]`.
  - `P = F P Fᵀ + B(R Σ_acc Rᵀ·1000)Bᵀ`. This is the only process noise.
  - The ground filter is propagated with `a_w.z`.
- **Updates:**
  - Baro: altitude from `44330(1−(p/p0)^0.1903)` minus the reference altitude, plus a finite-difference vertical velocity. `R = diag(σ²·1000·2/dt·10, σ²·1000)`. H selects indices 2 and 5.
  - GNSS: horizontal velocity (`velocity(0)`, `velocity(1)`) and relative position from `calcRelPosFromGNSS` (spherical, N = Δlat·b, W = −Δlon·a·cos lat). `R = diag(cov)·200`. H selects indices 0, 1, 3, 4. Rejected if the cov "magnitude" is above 20 or 5, or below 0.01.
  - Range: updates the ground filter. If `setUseRangeForAltitude(true)`, it also updates indices 2 and 5 and slews the reference altitude.
- **Zeroing mode** (`enableZeroingMode`):
  - `x_ *= 0.95` on every update.
  - The accelerometer bias is learned assuming the vehicle is at rest.
  - The reference altitude and lat/lon and the range zero point are IIR-slewed with factor 0.1.
- **Without a GNSS fix:** horizontal states are forced to 0 whenever GNSS data arrives.
- **Init gating:** GNSS initialises only after the baro has, and only with `positionValid` and `positionCov(0) < 20`. It then re-estimates sea-level pressure.
- **Tuning (hard-coded):** acc cov ×1000; baro cov ×1000 and velocity cov ×2/dt×10; range cov ×1000 and velocity cov ×2/dt×10; GNSS cov ×200; acc-bias IIR 0.1; zeroing IIR 0.1. `setProcessNoise()` is a no-op.
- **Task:** `IMUGPSPositionKalmanTask(period)` runs `predict()` then `update()`. Main runs it at **10 ms**.

### Body simulator (`body_sim_model.hpp/.cpp` + `body_simulator.hpp/.cpp`, redesigned 2026-11-21, `feature/sim-aero`)

Two parts:
- **`BodySimModel`** (pure: no scheduler, topics, wall clock or random numbers; host-built and tested in the flight software, `test/test_sim_aero`). Holds `CTRL::PhysicsBodySim` (semi-implicit Euler with `ω × Iω`, ExVectrControl), the geometry, the aerodynamics, the actuator models and the wind. All coefficients are in **`BodySimParams`** (table below). `step()` integrates one fixed step (2 ms); `advance(elapsed)` integrates elapsed time in fixed steps with the remainder carried, at most `maxAdvance_s` (50 ms) per call. Identical inputs give bit-identical runs.
- **`BodySimulator`** (`Task_Periodic`, 10 ms, registered on the system scheduler): feeds the TVC actual-thrust topic `[Fx,Fy,Fz,T]` and the true flap angles into the model, integrates the wall-clock time since its previous run (a gap above `kMaxGapNs` = 100 ms, i.e. a pause or a restart, is not integrated: the simulation continues from where it stopped), applies zeroing mode, publishes `[ω,q]`, `[v,p]`, ground `[v_U, p_U]`. On the vehicle `applyActuatorRateLimits = false`: the flap driver (`ServoAngleSimulator`, 300°/s) and the TVC driver (thrust-angle slew 20°/s = 300°/s servo / 15) already publish rate-limited actual values.

**Model (body frame +Z nose, belly −X, flaps TL/BL on +Y, origin at the CoM):**
- **Body crossflow (strip theory):** `bodyStrips` (24) strips over the real length (tube 0.54 m + parabolic nose 0.20 m, `r = R(2s − s²)`), each `dF = ½ρ·η·Cdc·d(z)·dz·|u⊥|u⊥` with the local air velocity `u = u_cg − ω × r`. The rotation term gives the distributed pitch/yaw damping.
- **Body potential normal force (slender body, Jorgensen):** `½ρV²·A_base·sin 2α·cos(α/2)` along the crossflow, at the **leading end** (nose: x_cp = L(1 − V_nose/(A_base L)) = 0.467 L from the tip; tail first: the base face). It gives the angle-of-attack dependent centre of pressure and the belly-down pitch stiffness. Replaces the old ad hoc `z −= u_z/200` CoP shift.
- **Axial force:** `½ρ·Cd_axial·A_base·u_z|u_z|` on the axis (tail-first 0.9, nose-first 0.35).
- **Flaps (flat plates):** hinge on the tube surface, parallel to Z (top flaps: tilted inward by `topFlapCantRad` about the lower hinge point, see the table); uncanted normal `n = (cos a, ∓sin a, 0)`, radial direction `d = (sin a, ±cos a, 0)` (upper sign TL/BL). Normal force `½ρ|u|²·A·C_N(α)` along `n` with `α` the angle between the local flow and the plate; `C_N` blends the attached branch `C_Nα sin α cos α` into the stalled branch `C_N90 sin α` (logistic, centre `flapStallAngle_rad`). Chordwise centre of pressure from the leading edge `x/c = 0.5 − 0.25 cos α` (1/4 at small α, 1/2 broadside); the hinge edge leads when the flow goes hinge → tip.
- **Wind:** steady, world frame, `u_cg = Rᵀ(w − v)` (correct frame and sign); speed and direction (toward, CCW from North) in the parameters, off by default. Vehicle: telecommands `SimWindSpeed` / `SimWindDirection` (simulation only, hardware-and-drivers.md).
- **Gusts:** deterministic, no wall clock and no global random state. The gust is a sum of 8 sinusoids of equal amplitude `strength/8` with frequencies (0.5 .. 2)/`gustPeriod_s` and phases from a xorshift32 generator seeded with `gustSeed`, evaluated at the simulated time and added to the mean wind in the world frame **along the mean direction** (head or tail gust). A second independent process, scaled by `gustSpread` (0.3), acts horizontally across it. So `|gust| ≤ gustStrength_mps` (hard bound; realised peaks 0.6..0.9 of it, 3.5 of 4 m/s in the test), RMS = strength/4, correlation time about the period; the mean is zero. The same seed and settings reproduce the series bit-identically; 16 `sin` per step, only while strength > 0.
- **Thrust:** force at `(0,0,tvcFromCg_m)`, magnitude and cone limits (the cone clamp keeps the azimuth and puts the vector on the 3° cone toward +Z), roll torque `T` about Z; both act only while TVC is enabled. Non-finite inputs are ignored (no NaN state).
- **Actuators (harness only, `applyActuatorRateLimits = true`):** flap dead time → 300°/s rate limit → first-order lag (lag and dead time default 0); TVC thrust-angle slew `tvcServoRate / tvcServoToThrustGain` (20°/s).
- **Ground:** CoM height 0. Arriving: stop (horizontal and downward velocity and body rates zeroed, attitude kept). Resting: position and attitude held (no sliding, no tip-over).

**`BodySimParams`** (defaults; units SI):

| Group | Parameter | Default | Meaning / source |
|---|---|---|---|
| Mass | `mass_kg` | 1.1 | measured (owner; = `VEHICLE_MASS_KG`) |
| | `inertiaXX/YY/ZZ`, `inertiaXY/XZ/YZ` | 0.044 / 0.044 / 0.004, 0 | kg m² about the CoM, measured (double swing, owner). Main passes the diagonal |
| Geometry | `bodyRadius_m` | 0.065 | tube diameter 13 cm |
| | `tubeLength_m`, `noseLength_m` | 0.54, 0.20 | tube without nose; parabolic nose (total 0.74 m) |
| | `cgFromBottom_m` | 0.37 | CoM above the bottom ("basically the middle", owner). Neutral flaps trim exactly belly-down at 0.33 m (see Known issues) |
| | `flapHingeRadius_m` | 0.065 | hinge line on the tube surface |
| | `flapTop` | 0.07 × 0.14 m, chamfer 0.08, lower edge 0.53 m, eff. 1 | TL/TR: 82 cm², centroid z = +0.220 m (before the cant) |
| | `topFlapCantRad` | 5° | **inward cant of the top flaps** (owner 2026-11-21): tilted about the horizontal line (body X) through the lower hinge point, upper end toward the axis. The hinge line, normal, radial direction, centroid, hinge reference point and centre of pressure rotate rigidly with it (flap angle convention unchanged: rotation about the tilted hinge line). Centroid moves inward by ≈ s·sin(cant) = 5.4 mm, outer edge rises by w·sin(cant), the normal at 90° is tilted 5° out of the Y axis (axial force component `−sin a·sin(cant)`). Bottom flaps: no cant |
| | `flapBottom` | 0.08 × 0.21 m, chamfer 0.08, lower edge **0.01 m** (spans 0.01..0.22 m above the bottom; owner correction 2026-11-21, was 0.10), eff. 1 | BL/BR: 152 cm², centroid 0.106 m above the bottom = z −0.264 m (chamfer-aware; the plain rectangle gives 0.115 m / −0.255 m) |
| Air | `airDensity_kgm3` | 1.2 | |
| Body aero | `cdCrossflow`, `crossflowEta` | 1.2, 0.65 | 2-D cylinder Cd (Re ≈ 7·10⁴), finite-length factor (L/D ≈ 5.7) |
| | `potentialFactor` | 1.0 | scale of the slender-body potential term |
| | `cdAxialTailFirst`, `cdAxialNoseFirst` | 0.9, 0.35 | blunt base first / parabolic nose first, on πR² |
| | `bodyStrips` | 24 | 4..64 |
| Flap aero | `flapCnAlpha` | 3.0 /rad | attached slope (Helmbold, AR ≈ 2.5) |
| | `flapCn90` | 1.17 | broadside normal-force coefficient |
| | `flapStallAngle_rad`, `flapStallWidth_rad` | 30°, 5° | blend centre and width |
| Calibration | `aeroCalibration` | **4.23** | **one global factor on every aerodynamic force and moment**: without it the belly-down fall at neutral flaps is about 13.5 m/s; 4.23 gives the real 6.4 m/s (21..25 km/h, flight position filter; harness 6.40 m/s, tolerance 3 %). Implied drag area CdA = 2mg/(ρv²) = **0.44 m²** against 0.097 m² from the geometry (Known issues). Was 3.08 for 7.7 m/s |
| Actuators | `applyActuatorRateLimits` | true (vehicle: false) | model applies the servo rates itself |
| | `flapServoRate_radps`, `flapServoLag_s`, `flapServoDeadTime_s` | 300°/s, 0, 0 | 9 g servo 60°/0.2 s (owner); optional lag and dead time (≤ 0.25 s) |
| | `flapMinAngle_rad`, `flapMaxAngle_rad` | 0, 90° | modelled travel (the driver clamps commands to 15..90°) |
| | `tvcServoRate_radps`, `tvcServoToThrustGain` | 300°/s, 15 | vane servo rate; `StarshipTVC::tvcFactor_` (default 15, main does not pass it) |
| | `tvcThrustLimit_N`, `tvcAngleLimit_rad`, `tvcFromCg_m` | 14, 3°, −0.35 | main: `TVC_THRUST_LIMIT_N`, `TVC_ANGLE_LIMIT_RAD`, `setTVCInputTopic` position z |
| Wind | `windSpeed_mps`, `windDirection_rad` | 0, 0 | steady; direction the air moves toward, CCW from North (NWU) |
| Gusts | `gustStrength_mps`, `gustPeriod_s`, `gustSeed`, `gustSpread` | 0, 5, 1, 0.3 | simulation only (owner-approved exception to safety.md rule 7, 2026-11-21), off by default. Telecommand range: strength 0..10 m/s, period 0.5..30 s, seed 0..16777215. Model below |
| Integration | `step_s`, `maxAdvance_s` | 0.002, 0.05 | fixed step; bounded catch-up per call |

**Measured with the harness** (`pio test -e native -f test_sim_aero -v | grep SIMAERO`; old = 1:1 replica of the model before this branch, `test/test_sim_aero/legacy_model.hpp`):

| Scenario | Old model | New model |
|---|---|---|
| Hover, T = m·g, 5 s | balanced | balanced (v_U 0, tilt 0) |
| Belly-down fall, attitude held, flaps 50/40° | **14.9 m/s** | **6.40 m/s** (target 6.4; was 7.71 with the 3.08 factor before the correction of 2026-11-21) |
| same, flaps folded 90° / out 15° | 16.5 / 13.3 m/s | 7.9 / 5.9 m/s (was 9.2 / 7.0) |
| Belly-down free, +10° pitch, flaps fixed neutral | period 1.62 s, ζ −0.02 (slowly growing), trim pitch −6°, 16 m/s | **changed with the corrected geometry:** trim pitch −14.6°, slow oscillation (period 7.95 s, ζ 0.29, max belly angle 83° during the start), 5.5 m/s down **gliding 12.9 m/s at AoA 10°** (before the correction: period 1.64 s, ζ 0.22, trim −0.8°, glide 4.2 m/s at AoA 58°) |
| Free fall from rest, tilt +5° (nose toward the back) | belly < 20° after 3.5 s, then tumbling (end tilt 65°) | belly < 20° after 4.6 s, ends belly 15° (before the correction: 2.0 s, 1°) |
| Free fall from rest, tilt −5° (nose toward the belly) | ends back-down-ish (belly 155°) | ends belly 16° with the corrected geometry (before it: back-down, 179°), 14° with a 5° roll asymmetry |
| Roll 20° about the axis from belly-down, free | settles at 17° | settles at 13° (peak 83°) |
| Flap step, top pair out 10° (pitch up) | +2.65 rad/s² | +5.49 rad/s² (was 3.76) |
| Flap step, controller u0 = 10° (azimuth) | ω̇x +1.43 rad/s² (correct sign) | **ω̇x −2.73 rad/s² (reversed, as before: −2.37)**, with ω̇y +5.1 and ω̇z +4.5 coupling |
| Flap step, controller u2 = 10° (roll) | ω̇z +11.6 | ω̇z +9.2 (was 9.9) |
| Static ∂τ/∂u per degree at 50/40°: u0 (x), u1 (y), u2 (z) | — | −0.0080, +0.0196, +0.0068 N m/deg at 6.4 m/s; trim moment at neutral **−0.60 N m** (was −0.43 at 7.7 m/s). At top 50° the pitch moment only reaches ≈ 0 with the bottom flaps near 90° (−0.998 / −0.599 / −0.018 N m at bottom 0 / 40 / 90°): the neutral angles 50/40° no longer trim belly-down |
| Vane step 0 → 3° | 0.010 s (thrust-angle slew 600°/s) | 0.148 s (servo 300°/s / 15) |
| Flip from belly-down, 14 N, fixed 3° vane | upright (< 15°) after 1.94 s | **does not flip any more**: min tilt 31°, peak rate 53 deg/s, never upright in 3 s (before: 1.61 s). Not asserted |
| Flap-controller replica (gains of 6e07d38), start pitch +20°, yaw +20° | holds, azimuth error 0 | **no longer holds with the unchanged neutral angles and gains**: pitch −9.8 ± 6.2°, belly max 133°, v 4.8 m/s (before: holds, max 34°, pitch −1.8 ± 0.2°, azimuth drifts 177° in both). The test no longer asserts it. Cause: the bottom flaps' 9 cm lower position (lever −0.264 instead of −0.174 m), see Known issues |
| Wind 3 m/s toward West | body-fixed, sign inverted, random 0–3 m/s steps | drift exactly toward West at any yaw (3.0 m/s broadside); with the canted top flaps a crosswind adds a small axial drift (1.4 m/s in 20 s at yaw 0, attitude held), so the test checks the symmetry with the cant switched off |
| Torque-free spin 4 s | (no ω × Iω) | |L| +1.3 %, direction 0.03° |

### Calibration module (2026-11-21, branch `feature/calibration-service`, `include/ExVectrDSP/calibration/`, `src/calibration/`)

Pure code (no Arduino, no ExVectrCore), host-tested from the flight-software repo (`pio test -e native`: `test_cal_accel_fit`, `test_cal_mag_fit`, `test_cal_stillness`; the session engine on top is `src/calibration/`, [project/calibration.md](../project/calibration.md)). Time is injected as int64 ns. Samples are raw sensor frame.

| Piece | What it does |
|---|---|
| `fitAccel` (`accel_ellipsoid_fit`) | 12-pose magnitude fit (9 parameters: offset `b`, symmetric `A`; Levenberg-Marquardt on pose means, `a_cal = A (x − b)`), diagonal fallback with 6-8 poses (at best `Warn`), verdict `CalResult` + check bits (poses, coverage, condition, residual rms/max, scale, off-diagonal, offset). |
| `MagSphereFit` | Hard-iron sphere fit (the former `MagnetometerCalibrationTask` maths in double, accumulated online) plus a direction-thinned sample buffer (caller-provided, 600 samples) for residual and 64-bin coverage; `b = centre`, `M = s·I`, `s = 0.05 / R`. |
| `SphereCoverage` | 64 equal-area bins on a Fibonacci lattice (shared definition with the ground station). |
| `StillnessDetector` | Still when the last windowBlocks 0.1 s blocks are inside the gyro and accel limits (calibrated-rate norm with a gyro bias, or bias-free std / peak-to-peak). |
| `PoseCollector` | Automatic static pose capture: still 1.5 s, averaged 2 s, new if > 20° from every stored pose (up to 24). |
| `GyroBiasCollector` | Stationary gyro bias run: 5 s of 1 s blocks inside the motion limits (restart otherwise), deadline, bias/noise/temperature acceptance, mean raw accel and mag of the window, **block diagnostics** (worst and quietest block, last block) used for tuning the limits. |
| `CalResult` | `None, Pass, Warn, FailResidual, FailCoverage, FailRange, Motion, Timeout, Numeric`; `acceptable()` = Pass or Warn. |

Not wired anywhere else: the vehicle uses it only through `src/calibration/`. The old `Calibrator_*` classes below are still unused.

### Calibrators (`calibrator_*.hpp/.cpp`)
- `Calibrator_Interface<T,DIM>` holds `calibrationData_` (`Timestamped<ValueCov>`: val = bias, cov = transform), `sensorToBodyMat_` and `isCalibrated_`.
- `Calibrator_Gyroscope(duration)` averages raw gyro over `duration` after `initCalibration()`. It outputs `val = mean` and `cov = sensorToBody`.
- `Calibrator_Magnetometer` tracks min/max per axis. It outputs `val = (max+min)/2` and `cov = sensorToBody·diag(scale)`.
- `Calibrator_Manager<T,DIM>` (a Task) persists `ValueCov` via `Data::Memory_Interface` under `memoryKey` and pushes it into a `TopicCoordTransform`. It has `saveCalibrationData()`, `loadCalibrationData()` and `applyCalibrationData()`.
- **Unused** (the commented-out use in `src/main.cpp` was removed 2026-11-21). The vehicle calibrates through the calibration module above and `src/calibration/`; its former `MagnetometerCalibrationTask` is retired.

### Generic
- `KalmanFilter<LEN,T>`: `predict(B,u)`, `update(H, ValueCov z)` (standard form plus symmetrisation), `setTransitionMatrix`, `setProcessNoise`, `setState`. Initial P = 1000·I. Unused.
- `LowPassFilter<T>(fc)` / `HighPassFilter<T>(fc)`: RC first order with dt from timestamps. Unused, and the low-pass formula is wrong (see the table).

## Usage

Wiring as in `src/main.cpp:110-118` and `:1920-1976`:

```cpp
DSP::TopicCoordTransform<float> gyroXf, accXf, magXf;        // T*(v-bias)
DSP::IMUAttitudeEKFTask imuTask(1 * Core::MILLISECONDS);
DSP::IMUGPSPositionKalmanTask posEst(10 * Core::MILLISECONDS);

gyroXf.setTransform(gyroCal.cov * gyroSensorToBody, gyroCal.val);
accXf.setTransform(accCal.cov * Math::Matrix<float,3,3>({-1,0,0, 0,1,0, 0,0,-1}), accCal.val);
gyroXf.setInputTopic(mpu.getGyroTopic());
accXf.setInputTopic(mpu.getAccelTopic());
magXf.setInputTopic(qmc.getMagTopic());

imuTask.setGyroInput(gyroXf.getOutputTopic());
imuTask.setAccInput(accXf.getOutputTopic());
imuTask.setMagInput(magXf.getOutputTopic());

posEst.setAccInput(accXf.getOutputTopic());
posEst.setAttitudeInput(imuTask.getAttitudeEstTopic());
posEst.setBaroInput(bme.getBaroTopic());
posEst.setRangeInput(vl53.getRangeTopic(), 0.4, 1.5);
posEst.setGNSSInput(gnss.getGNSSTopic());

// On the pad: learn biases/reference, then release
imuTask.enableZeroingMode(true);  posEst.enableZeroingMode(true);
posEst.setPositionReference();    // current position becomes the origin

// Read the estimate
Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> att(imuTask.getAttitudeEstTopic());
Math::Quat_F q = att.getItem().data.block<4, 1>(3, 0);   // body -> world
```

Simulator in place of the estimators (main.cpp `bodySimulator`, `SimulationMode` handler):

```cpp
DSP::BodySimulator sim(1.1f, {0.044, 0.044, 0.004}, 14.0f, TVC_ANGLE_LIMIT_RAD);
sim.setTVCInputTopic(starshipTVC.getTVCActualThrustTopic(), {0, 0, -0.35},
                     TVC_ANGLE_LIMIT_RAD, TVC_THRUST_LIMIT_N);
sim.setFlapSettingInputTopic(trueFlapSettingTopic);
attitudeTopicSwitch.subscribe(sim.getAttitudeEstTopic());   // instead of imuTask
positionTopicSwitch.subscribe(sim.getStateEstTopic());
```

## Conventions

- Frames, quaternion order and multiplication semantics: see **`docs/libs/ExVectrMath.md` → Conventions**, which is authoritative. In short: `q` = body→world, `[w,x,y,z]`, Hamilton since Math `b221829`, world = X-North/Y-West/Z-Up.
- Attitude topic: `Vector<7>` = `[ωx, ωy, ωz (rad/s, body, bias-removed), qw, qx, qy, qz]`.
- Position topic: `Vector<6>` = `[vN, vW, vU, pN, pW, pU]` (m/s, m, relative to the reference).
- Ground topic: `Vector<2>` = `[v_vertical, distance]`.
- Sensor inputs go into the estimators **already in body frame** (via `TopicCoordTransform`). Accelerometer convention: specific force, reading +g up at rest.
- The doc comments say "Q is a rotation from the reference frame to body frame". The explicit formulas (process model, `h = Rᵀe_z`, `R(q)·a` in the position KF) all implement body→world. Several comments also mislabel `[V,Q]` / `[V,P]`.

## How this project uses it

- `src/main.cpp:110-112`: three `TopicCoordTransform<float>` (gyro, acc, mag). Their transforms are set at `:1862-1866` / `:1909-1913` from stored calibration, and the mag transform also from the in-app calibration at `:799`.
- `src/main.cpp:114`: `IMUAttitudeEKFTask imuTask(1 ms)`.
- `src/main.cpp:115`: `IMUGPSPositionKalmanTask posEstTask(10 ms)`. Range limits 0.4–1.5 m. Range-for-altitude is disabled (`:1932`, `:1934`).
- `src/main.cpp:117`: `BodySimulator`.
- `Topic_Switch`es `attitudeTopicSwitch`, `positionTopicSwitch` and `groundTopicSwitch` (`:182-186`) select estimator or simulator output for the controllers and missions (`:920-960`, default `:1971-1972`).
- Zeroing modes: `VehicleSafetyAndControlTask::applyEstimatorZeroing()` sets `enableZeroingMode` on `posEstTask`, `imuTask` and `bodySimulator` together, while the waypoint mission's Initialisation mode or the zeroing window (automatic boot zero, `ZeroEstimators` telecommand) is active ([../project/overview.md](../project/overview.md), "Estimator zeroing"). Since 2026-11-20 `setPositionReference()` is no longer called by SystemReset (only commented-out code in `DisplayTask` still references it); re-referencing happens only through zeroing mode.
- Not used: `IMUAttitudeCF`, `KalmanFilter`, `LowPass/HighPassFilter`, `FullStateEKF`, the calibrators.

## Known issues & limitations

"Quat-semantics" marks call sites whose behaviour likely changed with ExVectrMath `b221829`: `rotate()` now applies the opposite rotation, and `a*b` now composes in the opposite order. The verdict given is relative to the current (Hamilton) Math.

| Severity | Location (path:line) | Issue | Impact |
|---|---|---|---|
| High | `src/imu_attitude_ekf.cpp:144-145` | `float factor = 1 / (1 + gyroBiasReadings_);` is integer division. The factor is 1 on the first sample and 0 afterwards. | The startup gyro bias equals the first raw sample (noise). The intended 500-sample average never happens. It also logs 500 times. |
| High | `src/imu_attitude_ekf.cpp:350` (Quat-semantics) | Mag yaw correction `quat * rotQuat` is now a rotation about **body** z. With the old Math it was about world z. | In bellyflop (body z horizontal) the mag "yaw" correction perturbs pitch/roll continuously. |
| High | `src/imu_attitude_ekf.cpp:243-264` | No accelerometer gating for thrust, drag or free fall. `normalize()` of a near-zero vector returns zero, giving residual `-h`. | Attitude is pulled toward the specific-force direction during boost and the bellyflop transition. Free fall gives a strong wrong correction. |
| High | `src/imu_gps_position_kf.cpp:365-366` | GNSS velocity (documented NED) is fed directly into the NWU state. `velocity(1)` (East) goes into the West state without negation. | The horizontal west velocity has the wrong sign, fighting the integrated accelerometer and position updates. Check the actual `ublox_gnss.cpp` output frame. |
| High | `src/imu_gps_position_kf.cpp:579-582`, `:81`, `:197` | `taskThread` calls `predict()` up to NOW, then `update()` integrates buffered accelerometer samples stamped earlier (`accSubr_[0].timestamp`). That gives a negative dt and backward integration with the new sample. | Velocity and position integrate over overlapping intervals with mismatched accelerometer values: timing-dependent drift and bias. |
| Med | `src/imu_attitude_ekf.cpp:427-431` (Quat-semantics) | `initialiseAcc` right-multiplies by `Quat(h×acc, angle)`. With current Math this rotates the expected gravity *away* from the measurement. The correct form is axis `acc×h`, or a left-multiply in the matching frame. | The initial tilt error is doubled. The EKF later converges, slowly given acc R ×5000. |
| Med | `src/imu_attitude_ekf.cpp:451-453`, `:462` (Quat-semantics) | `initialiseMag` uses `conjugate().rotate` (world→body) on a body vector, then rotates back, then right-multiplies. `updateMag` (`:316`) uses `R(q)`, the opposite direction. | The initial heading is computed in the wrong frame unless level and near identity. The two functions are inconsistent. |
| Med | `src/imu_attitude_ekf.cpp:185` | The w ≥ 0 sign normalisation tests `x_[0][0]` (bias x) instead of `x_[3][0]` (w). | The published q flips sign depending on the sign of the bias. Any consumer relying on w ≥ 0 or on continuity breaks. The rotation itself is unaffected. |
| Med | `src/imu_attitude_ekf.cpp:114-116` | The first `dTime` is measured from `lastGyroData_.timestamp = 0`, i.e. time since boot. | The first predict integrates one gyro sample over about 5 s and inflates P. Masked by the accelerometer init. |
| Med | `src/imu_attitude_ekf.cpp:126`, `:245`; `src/imu_attitude_ekf.cpp:472-475` | Noise comes only from fixed inflation factors. There is no additive Q and `setProcessNoise` is a no-op. The bias is not in the filter state; only the 4x4 q block of P is used. | Untunable through the API. There is no bias observability in flight, so gyro bias drifts uncorrected unless zeroing is on. |
| Med | `src/imu_attitude_ekf.cpp:327` vs `include/ExVectrDSP/imu_attitude_ekf.hpp:91` | The mag acceptance window 0.01–0.1 implies mT (|B| ≈ 0.05). The header says tesla. | A wrong-unit mag topic is silently rejected forever. |
| Med | `src/imu_attitude_ekf.cpp:417` | `initialiseAcc` ignores `accTiltCompensation_`; `updateAcc` (`:243`) applies it. | Inconsistent if the tilt compensation is ever non-identity. |
| Med | `src/imu_gps_position_kf.cpp:419` (Quat-semantics) | `updateAccBias` uses `att_.rotate(world)`, which is now body→world applied to a world vector. It should be `conjugate().rotate`. | The accelerometer bias learned in zeroing mode is wrong whenever the vehicle is not level. On the pad it is near identity, so the effect is small. |
| High | `src/imu_gps_position_kf.cpp:181-196`, `:571-582` | The position KF runs `predict()` from boot (the task is woken by buffered accelerometer samples) before the attitude EKF publishes (its task starts at t = 5 s). `att_` is then the zero quaternion, `to3x3RotMat()` gives a zero matrix and `a_w = -g`: the state integrates -9.81 m/s² for several seconds. Bench 2026-11-20: about -300 m and -89 m/s at the first published estimate. Without a zero the barometer pulls it back only slowly (7ac9b96: about -427 m, +0.8..1.2 m/s while stationary, the "-440 m" symptom). | Masked since 2026-11-20 by the automatic 10 s boot zero (state decays to 0, baro reference re-slewed) before Ready. The KF itself is unchanged (estimator change needs a decision): gating `predict()` on `attInitialised_` would remove the cause. |
| Med | `src/imu_gps_position_kf.cpp:348-351`, `:453` | `positionCov.magnitude(true)`: the bool becomes `startRow = 1`, so element 0 is excluded. | The GNSS validity gating ignores the north/first covariance component. |
| Med | `src/imu_gps_position_kf.cpp:415-418` | `updateAccBias` treats the GNSS velocity difference as world NWU and adds the up-gravity, while GNSS velocity is NED. The non-zero path also divides by `dTime`, which can be 0. | Wrong bias if ever called with real motion. Currently only called with `assumeZero`. |
| Med | `src/imu_gps_position_kf.cpp:276-343` | The range is used as a vertical distance with no tilt compensation. | Ground distance is wrong when tilted. The VL53L0X window is 0.4–1.5 m, so this matters during touchdown. |
| ~~Med~~ Fixed | `src/body_simulator.cpp` (TVC clamp) | The TVC angle clamp pushed the vector further from +Z (to `2α − limit`). | Fixed 2026-11-21 (`feature/sim-aero`): `BodySimModel::setThrust` puts the vector on the cone toward +Z (harness `test_thrust_gating_and_nan_inputs`). |
| ~~Med~~ Fixed | `src/body_simulator.cpp` (wind) | A world-frame wind was subtracted from the body-frame airflow without rotation and with the wrong sign, and was a random, time-seeded body-fixed 0–3 m/s step every 1 s. | Fixed 2026-11-21 (`feature/sim-aero`): steady world-frame wind, `u = Rᵀ(w − v)`, off by default, set by `setWind` / the sim wind telecommands (harness `test_wind_frame`). |
| ~~Med~~ Fixed | `src/body_simulator.cpp` (zeroing mode) | Zeroing mode set `positionState_ = {0,0,0,1,0,0}` (pN = 1 m) and `attitudeState_ = {0,0,0, 0,0,0,1}` (q = (0,0,0,1), a 180° yaw). | Fixed 2026-11-20 (feature/review-fixes): zeroing mode holds `resetToStartPose()` (origin, identity), the same pose as simulation start and the sim `SystemReset`. |
| Low | `src/body_sim_model.cpp` (`addFlap`) | Flap geometry and fold direction (toward +X) follow the documented frame and the owner's dimensions, but the force directions have not been checked against hardware. | Verify flap torque signs on the bench (e.g. pivot test) before trusting the sim's azimuth behaviour. |
| ~~Low~~ Fixed | `src/body_simulator.cpp` | End-cap area `2πr²`. | Fixed 2026-11-21 (`feature/sim-aero`): axial force on πR². |
| ~~Low~~ Fixed | `src/body_simulator.cpp` | Dead `calcTorqueFromTVC`, wrong comments. | Fixed 2026-11-21 (`feature/sim-aero`): removed. |
| ~~High~~ Fixed | `src/body_simulator.cpp` | Belly-down fall 14.9 m/s (real 7.7), no physically based pitch stiffness or damping (ad hoc `z −= u_z/200` CoP shift), Newtonian flap law with a sign-reversing azimuth term, no `ω × Iω`, wall-clock dt (a pause gave one huge step), phantom +m·g thrust before the first TVC message, roll torque not gated with the TVC (review F1-F3, F8-F11, F13-F15). | Fixed 2026-11-21 (`feature/sim-aero`): redesign (section "Body simulator"); measured before/after table there. |
| High | `include/ExVectrDSP/body_sim_model.hpp` `aeroCalibration` = 4.23 (was 3.08, 2026-11-21) | The documented ship (1.1 kg, 13 × 74 cm, flaps 2×82 + 2×152 cm²) gives a belly-down fall of 13.5 m/s with textbook coefficients; the real fall of 6.4 m/s (21..25 km/h, flight position filter of barometer plus accelerometer, owner 2026-11-21) needs a drag area of **0.44 m²** (CdA = 2mg/ρv²) instead of 0.097 m²: 4.5 times, which no drag coefficient of this shape explains (a 13 × 74 cm body has a frontal area of 0.096 m²; at CdA 0.097 the ship would have to weigh 0.24 kg or fall at 13.5 m/s). The single factor scales **all** aerodynamic forces and moments, so it also triples stiffness, damping and flap authority relative to the geometry. | Owner to check: mass, the 7.7 m/s figure (vertical speed, steady, which sensor), missing drag sources. The aero moments are now larger than the TVC authority (0.26 N m at 14 N; trim moment 0.43 N m at 7.7 m/s), which matters for the RTH flip. |
| Med | `BodySimParams::cgFromBottom_m` = 0.37 | With the CoM at the owner's "middle" the neutral flaps (50/40°) leave a 0.43 N m nose-down moment at belly-down; the free body trims by gliding nose-first (4.2 m/s horizontal at 7 m/s down, AoA 58°) while the body stays horizontal. Neutral flaps trim exactly belly-down with the CoM at 0.33 m. | Measure the CoM; check flight logs for horizontal drift in the belly-down descent. |
| High | `src/body_sim_model.cpp`, `ControlAttitudeFlaps` neutral angles | With the corrected geometry (bottom flaps 1 cm above the bottom, top flaps canted, calibration 4.23) the neutral flaps 50/40° leave −0.60 N m at belly-down (6.4 m/s); the free body glides at 13 m/s with AoA 10° instead of falling belly-down, the 14 N / 3° flip does not complete in 3 s and the flap-controller replica (unchanged gains, neutral angles) leaves the belly-down pose (max 133°). The aligned fall from ±5° still ends belly-down within 15°. | **Owner retunes the controller** (neutral angles and gains were NOT changed in this task): at top 50° the bottom flaps need ≈ 90° to null the moment. Re-run `test_sim_aero` afterwards and restore the replica assertion (`test_closed_loop_flap_hold`, now report-only). |
| Med | `src/body_sim_model.cpp` (`flapCn`, `addFlap`) | The azimuth authority of the controller's u0 mixer comes only from the flaps' sideways (Y) force, which depends on the slope of C_N(α) near α = 40-50°. With the default low-AR plate curve, both pairs are **reversed** at the neutral angles (∂τx/∂u0 = −0.0069 N m/deg); the old model had it positive. In the harness replica the azimuth then drifts 180° while belly-down holds. | Prediction with a medium-confidence coefficient: verify on the real vehicle (azimuth response to u0) before retuning. Controller gains are unchanged in this branch. |
| Low | `src/body_sim_model.cpp` | A gliding back-down fall (body +X down) is also a stable equilibrium in the model once reached (from a tilt toward the belly side); it is unstable at zero glide. | Real behaviour unknown. |
| Low | `src/body_sim_model.cpp` | Not modelled: sensors (perfect, zero-latency state, review F12), motor spool-up (owner: instantaneous), vane effectiveness vs thrust and crossflow, flap-body interference, ground tip-over, gusts. | Simulation remains necessary but not sufficient. |
| Low | `include/ExVectrDSP/body_simulator.hpp` | `flapsEnabled_` is informational only: the model keeps the last true flap angles (the driver publishes only while enabled), i.e. unpowered flaps hold their position. | Matches the owner's statement that the flaps are always powered in flight. |
| Low | `src/imu_attitude_ekf.cpp:291-299`; `src/imu_attitude_cf.cpp:264-272` | Gyro bias from consecutive accelerometer vectors is computed but never written back. It also reads `lastAccData_` before it is first set. | Dead code. |
| Low | `src/imu_attitude_cf.cpp:145` (Quat-semantics) | CF gyro integration `rotQuat * quat` is now a world-frame increment applied to a body-frame rate. | Wrong attitude propagation. The CF is unused. |
| Low | `src/imu_attitude_cf.cpp:213`, `:395`, `:402`, `:321`, `:422-434` (Quat-semantics) | `accErrorIndex` compares a body-frame vector with world g. Init/mag use frame directions inconsistent with the EKF. | The CF is unusable as-is. Unused. |
| Low | `include/ExVectrDSP/low_pass_filter.hpp:91` | `value_ = lastInput_ + (value - lastInput_)*a` interpolates between the last two inputs. It is not an IIR low-pass; it should be `value_ += (value - value_)*a`. `lastInput_` is uninitialised (`:23`). | Practically no filtering. Unused. |
| Low | `include/ExVectrDSP/low_pass_filter.hpp:71`, `:84`; `include/ExVectrDSP/high_pass_filter.hpp:71`, `:84` | Default arguments are repeated on the out-of-class definitions (ill-formed). The first dt is measured from t = 0. | Compile error on first use. Bad first sample. |
| Low | `include/ExVectrDSP/topic_coord_transform.hpp:69-73`, `:97` | The 3-argument constructor passes a `const Topic&` to the non-const `setInputTopic`, which won't compile. The covariance is not transformed (`T Σ Tᵀ`). | Unusable constructor. Covariance stays in sensor frame and scale. |
| Low | `src/calibrator_magnetometer.cpp:55-57` | `0.05/(max-min)/2` evaluates to `0.025/(max-min)`, which gives \|B\| ≈ 0.0125 instead of the intended 0.05 (should be `0.05/((max-min)/2)`). The first sample divides by zero (max == min). | The scale is 4x low; it only stays inside the EKF window (0.01–0.1) by luck. Unused in favour of main's own calibration. |
| Low | `include/ExVectrDSP/calibrator_manager.hpp:118-132` | Save only happens when a save request coincides with a *new* calibration timestamp. | A save requested after calibration completes never persists. |
| Low | `include/ExVectrDSP/fullstate_ekf.hpp:26-174` | Declared, no implementation. | Link error if instantiated. |
| Low | `src/imu_gps_position_kf.cpp:104-106`, `:352-353` | `LOG_MSG` uses `%d` for float/bool arguments. | Garbage logs (UB). |
| Low | `src/imu_gps_position_kf.cpp:489-492`, `:552-556` | `setProcessNoise` is a no-op. GNSS→NWU uses a spherical approximation with `a = 6371000` (the mean radius, mislabelled semi-major). | Fine for short ranges; untunable. |

## Extending

To add an estimator:
1. Create `include/ExVectrDSP/<name>.hpp` and `src/<name>.cpp` in `namespace VCTR::DSP`. Keep the pattern:
   - a plain class holding `Core::Simple_Subscriber<Core::Timestamped<ValueCov<float,N>>>` inputs, with `set<X>Input(Topic&)` doing `subr.subscribe(topic)`;
   - `Core::Topic<Core::Timestamped<…>>` outputs exposed through `get<X>Topic()`;
   - `update()` that consumes `isDataNew()` / `getItem()`, then publishes;
   - `predict(int64_t t)` with dt taken from timestamps (guard the first sample and dt ≤ 0).
2. Add `class <Name>Task : public <Name>, public Core::Task_Periodic`. The constructor is `Task_Periodic("<Name>Task", period)` followed by `scheduler.addTask(*this)`. `taskCheck()` does `setPaused(false)` on new data. `taskThread()` does `update(); setPaused(true);`.
3. Publish **the existing layouts** (`[ω,q]` as `Vector<7>`, `[v,p]` as `Vector<6>`, `[v,d]` as `Vector<2>`). Then the new estimator can be selected via the `Topic_Switch`es in `src/main.cpp:182-186` without touching the controllers.
4. Follow `docs/libs/ExVectrMath.md` Conventions. Write measurement models explicitly, as the EKF does; don't depend on implicit `rotate()` direction. Add a simulator test through `BodySimulator`.
5. Covariance hygiene: symmetrise after updates, and prefer the Joseph form. Don't rely on the unpivoted `Matrix::inverse()` for poorly conditioned `S`.
6. Wire it in `src/main.cpp` `initialiseTopicConnections()` (`:1920`). Feed inputs from the `TopicCoordTransform` outputs, never from raw sensor topics.
