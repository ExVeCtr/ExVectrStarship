# ExVectrDSP

> State estimation and signal processing for ExVectr: IMU attitude EKF, IMU/baro/GNSS/range position KF, sensor-frame transforms and calibration, generic KF and filters, plus a rigid-body flight simulator. Repo: https://github.com/ExVeCtr/ExVectrDSP. `Last verified against commit: 3e1f070 (pinned now: fc99b99 = 3e1f070 plus a mechanical NOW() -> NowNs() port for ExVectrCore 18bbb09; content otherwise as verified; feature/sim-aero a5c21af, 2026-11-21: body simulator section re-verified against the redesign; feature/sim-fixes-3, 2026-11-21: fold shield default and calibration)`

## Purpose & scope

- Turns raw sensor topics into the two estimate topics the rest of the system consumes:
  - **attitude** `[ω_body(3), q(4)]`
  - **position** `[v(3), p(3)]` in NWU
  - plus a ground-distance estimate.
- Provides `TopicCoordTransform` (sensor→body plus calibration) and calibrators for gyro and magnetometer.
- (Since S4, 2026-11-27, the vehicle no longer uses `BodySimulator`/`BodySimModel`: `ShipSim::StarshipSimTask` in `src/sim` replaced them, they are not linked into the firmware and stay only for the frozen tests and harnesses until S5.) Contains `BodySimulator`, a simulator task that publishes the same topic types as the estimators, so the application can switch between real and simulated state, around the pure, host-tested physics model `BodySimModel` (2026-11-21).
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
- **Flaps (flat plates):** hinge on the tube surface, parallel to Z (top flaps: tilted inward by `topFlapCantRad` about the lower hinge point, see the table); uncanted normal `n = (cos a, ∓sin a, 0)`, radial direction `d = (sin a, ±cos a, 0)` (upper sign TL/BL; fold toward +X). With `flapFoldTowardBelly` the fold goes toward −X: `n = (cos a, ±sin a, 0)`, `d = (−sin a, ±cos a, 0)`. Normal force `½ρ|u|²·A·C_N(α)` along `n` with `α` the angle between the local flow and the plate; `C_N` blends the attached branch `C_Nα sin α cos α` into the stalled branch `C_N90 sin α` (logistic, centre `flapStallAngle_rad`). Chordwise centre of pressure from the leading edge `x/c = 0.5 − 0.25 cos α` (1/4 at small α, 1/2 broadside); the hinge edge leads when the flow goes hinge → tip. All of this uses the **effective flap angle** `a_eff = smoothmax(a, 20°)` (`flapEffectiveMinAngleRad`, 2026-11-21) instead of the servo angle, so a fully extended flap still makes a sideways force.
- **Wind:** steady, world frame, `u_cg = Rᵀ(w − v)` (correct frame and sign); speed and direction (toward, CCW from North) in the parameters, off by default. Vehicle: telecommands `SimWindSpeed` / `SimWindDirection` (simulation only, hardware-and-drivers.md).
- **Gusts:** deterministic, no wall clock and no global random state. The gust is a sum of 8 sinusoids of equal amplitude `strength/8` with frequencies (0.5 .. 2)/`gustPeriod_s` and phases from a xorshift32 generator seeded with `gustSeed`, evaluated at the simulated time and added to the mean wind in the world frame **along the mean direction** (head or tail gust). A second independent process, scaled by `gustSpread` (0.3), acts horizontally across it. So `|gust| ≤ gustStrength_mps` (hard bound; realised peaks 0.6..0.9 of it, 3.5 of 4 m/s in the test), RMS = strength/4, correlation time about the period; the mean is zero. The same seed and settings reproduce the series bit-identically; 16 `sin` per step, only while strength > 0.
- **Thrust:** force at `(0,0,tvcFromCg_m)`, magnitude and cone limits (the cone clamp keeps the azimuth and puts the vector on the 3° cone toward +Z), roll torque `T` about Z; both act only while TVC is enabled. Non-finite inputs are ignored (no NaN state).
- **Actuators (harness only, `applyActuatorRateLimits = true`):** flap dead time → 300°/s rate limit → first-order lag (lag and dead time default 0); TVC thrust-angle slew `tvcServoRate / tvcServoToThrustGain` (20°/s).
- **Ground:** CoM height 0. Arriving: stop (horizontal and downward velocity and body rates zeroed, attitude kept). Resting: position and attitude held (no sliding, no tip-over).

**`BodySimParams`** (defaults; units SI):

| Group | Parameter | Default | Meaning / source |
|---|---|---|---|
| Mass | `mass_kg` | 1.1 | measured (owner 2026-11-21: a 1.2 kg value was tried and reverted, 1.1 kg is right; main passes `VEHICLE_MASS_KG`, which is the single source; the struct default only mirrors it) |
| | `inertiaXX/YY/ZZ`, `inertiaXY/XZ/YZ` | 0.044 / 0.044 / 0.004, 0 | kg m² about the CoM, measured (double swing, owner). Main passes the diagonal |
| Geometry | `bodyRadius_m` | 0.065 | tube diameter 13 cm |
| | `tubeLength_m`, `noseLength_m` | 0.54, 0.20 | tube without nose; parabolic nose (total 0.74 m) |
| | `cgFromBottom_m` | 0.37 | CoM above the bottom ("basically the middle", owner). Neutral flaps trim exactly belly-down at 0.33 m (see Known issues) |
| | `flapHingeRadius_m` | 0.065 | hinge line on the tube surface |
| | `flapTop` | 0.07 × 0.14 m, chamfer 0.08, lower edge 0.53 m, eff. 1 | TL/TR: 82 cm², centroid z = +0.220 m (before the cant) |
| | `topFlapCantRad` | 10° | **inward cant of the top flaps** (owner 2026-11-21): tilted about the horizontal line (body X) through the lower hinge point, upper end toward the axis. The hinge line, normal, radial direction, centroid, hinge reference point and centre of pressure rotate rigidly with it (flap angle convention unchanged: rotation about the tilted hinge line). Centroid moves inward by ≈ s·sin(cant) = 10.9 mm (measured by the harness), outer edge rises by w·sin(cant), the normal at 90° is tilted 10° out of the Y axis (axial force component `−sin a·sin(cant)`). Bottom flaps: no cant |
| | `flapBottom` | 0.08 × 0.21 m, chamfer 0.08, lower edge **0.01 m** (spans 0.01..0.22 m above the bottom; owner correction 2026-11-21, was 0.10), eff. 1 | BL/BR: 152 cm², centroid 0.106 m above the bottom = z −0.264 m (chamfer-aware; the plain rectangle gives 0.115 m / −0.255 m) |
| Air | `airDensity_kgm3` | 1.2 | |
| Body aero | `cdCrossflow`, `crossflowEta` | 1.2, 0.65 | 2-D cylinder Cd (Re ≈ 7·10⁴), finite-length factor (L/D ≈ 5.7) |
| | `potentialFactor` | 1.0 | scale of the slender-body potential term |
| | `cdAxialTailFirst`, `cdAxialNoseFirst` | 0.9, 0.35 | blunt base first / parabolic nose first, on πR² |
| | `bodyStrips` | 24 | 4..64 |
| Flap aero | `flapCnAlpha` | 3.0 /rad | attached slope (Helmbold, AR ≈ 2.5) |
| | `flapCn90` | 1.17 | broadside normal-force coefficient |
| | `flapStallAngle_rad`, `flapStallWidth_rad` | 30°, 5° | blend centre and width |
| | `flapEffectiveMinAngleRad`, `flapEffectiveSmoothRad` | **20°**, 5° | **effective aerodynamic flap angle** (owner 2026-11-21: the body shape and the flow around it give an extended flap a small effective angle, not pure drag). The aerodynamics (plate normal = force direction, C_N argument, centre of pressure, shield factor) use `a_eff = ½(a + a_min + √((a − a_min)² + k²))`, a smooth (C∞) max: a = 0/10/20/40/50/90° → a_eff = 20.3/20.6/22.5/40.3/50.2/90.1°. The servo angle (`flapAngle()`, telemetry) stays the true angle. `a_min ≤ 0` disables it |
| | `flapEffectiveRetractMarginRad` | 0 (off) | same smooth limit at the retracted end: `a_eff = smoothmin(a_eff, 90° − margin)`. Off: a folded flap is edge-on to the belly-down flow |
| Candidate effects | `flapSidewashGain` | 0 | (a) the local crossflow at the flap point is turned toward the axis (> 0) or away (< 0) by δ = gain·(R/r_c)²·sin α_body, added as δ·\|u⊥\| along −r̂. Investigation only, off (Known issues) |
| | `flapBodyCrossflowGain` | 0 | (b) gain × the 2-D potential-flow perturbation of the cylinder (radius R) in the local crossflow at the flap point, `(u − iv)_p = −V R²/z²`. Investigation only, off |
| | `flapFoldShieldFactor` | **0.7** (on since 2026-11-21) | (c) flap/body interference: effective flap area × `1 − f·sin²(a_eff)` (a flap folding against the body side, into its boundary layer and wake, loses area). **Enabled by default, owner-approved 2026-11-21** (the real ship's heading loop works, so the model must give the controller's signs): the only candidate effect that gives + yaw at the neutral 50/40° over AoA −20..+20°. Fitted to the sign, not measured: the owner still wants a bench pivot test in an airflow (Known issues). Was 0 (off) |
| Calibration | `aeroCalibration` | **4.64** | **one global factor on every aerodynamic force and moment**: without it the belly-down fall at neutral flaps is about 13.5 m/s; 4.64 with the fold shield 0.7 gives the real 6.4 m/s (21..25 km/h, flight position filter; harness **6.40 m/s** with 1.1 kg and 10° cant, tolerance 3 %). Implied drag area CdA = 2mg/(ρv²) = 2·1.1·9.807/(1.225·6.4²) = **0.430 m²** against 0.097 m² from the geometry (Known issues). Was 4.00 (no shield, 1.1 kg), 4.38 (1.2 kg, 10° cant, reverted with the mass), 4.23 (1.1 kg, 5° cant), earlier 3.08 for 7.7 m/s |
| Actuators | `applyActuatorRateLimits` | true (vehicle: false) | model applies the servo rates itself |
| | `flapServoRate_radps`, `flapServoLag_s`, `flapServoDeadTime_s` | 300°/s, 0, 0 | 9 g servo 60°/0.2 s (owner); optional lag and dead time (≤ 0.25 s) |
| | `flapMinAngle_rad`, `flapMaxAngle_rad` | 0, 90° | modelled travel (the driver clamps commands to 15..90°) |
| | `tvcServoRate_radps`, `tvcServoToThrustGain` | 300°/s, 15 | vane servo rate; `StarshipTVC::tvcFactor_` (default 15, main does not pass it) |
| | `tvcThrustLimit_N`, `tvcAngleLimit_rad`, `tvcFromCg_m` | 14, 3°, −0.35 | main: `TVC_THRUST_LIMIT_N`, `TVC_ANGLE_LIMIT_RAD`, `setTVCInputTopic` position z |
| Wind | `windSpeed_mps`, `windDirection_rad` | 0, 0 | steady; direction the air moves toward, CCW from North (NWU) |
| Gusts | `gustStrength_mps`, `gustPeriod_s`, `gustSeed`, `gustSpread` | 0, 5, 1, 0.3 | simulation only (owner-approved exception to safety.md rule 7, 2026-11-21), off by default. Telecommand range: strength 0..10 m/s, period 0.5..30 s, seed 0..16777215. Model below |
| Integration | `step_s`, `maxAdvance_s` | 0.002, 0.05 | fixed step; bounded catch-up per call |

**Measured with the harness** (`pio test -e native -f test_sim_aero -v | grep SIMAERO`; old = 1:1 replica of the model before this branch, `test/test_sim_aero/legacy_model.hpp`; new = current defaults, fold shield 0.7 and factor 4.64 since 2026-11-21, `feature/sim-fixes-3`; values without the shield in brackets where they differ):

| Scenario | Old model | New model |
|---|---|---|
| Hover, T = m·g, 5 s | balanced | balanced (v_U 0, tilt 0) |
| Belly-down fall, attitude held, flaps 50/40° | **14.9 m/s** | **6.40 m/s** (target 6.4; 1.1 kg, 10° cant, shield 0.7, factor 4.64) |
| same, flaps folded 90° / out 15° | 16.5 / 13.3 m/s | 7.53 / 5.75 m/s (no shield, factor 4.00: 8.11 / 6.08) |
| Belly-down free, +10° pitch, flaps fixed neutral | period 1.62 s, ζ −0.02 (slowly growing), trim pitch −6°, 16 m/s | trim pitch −7.1°, period 5.25 s, ζ 0.34, max belly angle 72° during the start, 5.0 m/s down **gliding 8.4 m/s at AoA 24°** (no shield: −10.1°, 5.91 s, 79°, glide 10.1 m/s at AoA 16°) |
| Free fall from rest, tilt +5° (nose toward the back) | belly < 20° after 3.5 s, then tumbling (end tilt 65°) | belly < 20° after 3.25 s, ends belly 7° (no shield: 3.6 s, 9°) |
| Free fall from rest, tilt −5° (nose toward the belly) | ends back-down-ish (belly 155°) | belly < 20° after 0.93 s, ends belly 7° (no shield: 0.92 s, 9°); with +5° roll also belly 7° after 20 s |
| Roll 20° about the axis from belly-down, free | settles at 17° | settles at 7° (peak 64°) (no shield: 10°, peak 72°) |
| Flap step, top pair out 10° (pitch up) | +2.65 rad/s² | +5.32 rad/s² (no shield: +4.88) |
| Flap step, controller u0 = 10° (azimuth) | ω̇x +1.43 rad/s² (correct sign) | ω̇x −0.64 rad/s² mean over 0.2 s, with ω̇y +4.5 and ω̇z +6.1 coupling (no shield: −2.56). The 10° step is far outside the linear range (the static derivative below is + but small, and the u0 step drives a large roll acceleration about the low-inertia body Z axis, Iz = 0.004 kg m²); the static derivative is the sign check |
| Flap step, controller u2 = 10° (roll) | ω̇z +11.6 | ω̇z +13.4 (no shield: +8.8) |
| Static ∂τ/∂u per degree at 50/40°: u0 (x), u1 (y), u2 (z) | — | **+0.0038, +0.0244, +0.0088 N m/deg** at 6.4 m/s, AoA 0 (yaw over AoA −20/−10/0/+10/+20°: +0.0027 +0.0038 +0.0038 +0.0025 +0.0005; all three axes + at every AoA, `test_control_axis_signs` asserts it); no shield: −0.0074 (yaw reversed), +0.0182, +0.0062. Trim moment at neutral **−0.56 N m** (unchanged by the shield): the neutral angles 50/40° do not trim belly-down |
| Vane step 0 → 3° | 0.010 s (thrust-angle slew 600°/s) | 0.148 s (servo 300°/s / 15) |
| Flip from belly-down, 14 N, fixed 3° vane | upright (< 15°) after 1.94 s | **upright (< 15°) after 2.03 s**, min tilt 0°, peak rate 121 deg/s (no shield: 2.62 s, 63 deg/s). Not asserted |
| Flap-controller replica (gains of 6e07d38), heading error ±20°, pitch −20/0/+20°, 30 s | holds, azimuth error 0 | **holds belly-down** (belly max 32°, pitch −5.1 ± 2.5°, 6.2-6.35 m/s) and the nose turns toward the setpoint, but **does not align**: a yaw/roll limit cycle of about ±34° (period about 4 s) remains in all five cases (azimuth error at 30 s +13.9 / −13.9 / +15.6 / −15.6 / +34.0°). Report only (`test_closed_loop_flap_hold` prints, does not assert). No shield: does not hold (pitch −25.9 ± 9.2°, belly max 78°, azimuth error +175.9°). Unchanged gains and neutral angles |
| Wind 3 m/s toward West | body-fixed, sign inverted, random 0–3 m/s steps | drift exactly toward West at any yaw (3.0 m/s broadside); with the canted top flaps a crosswind adds a small axial drift (1.4 m/s in 20 s at yaw 0, attitude held), so the test checks the symmetry with the cant switched off |
| Torque-free spin 4 s | (no ω × Iω) | |L| +1.3 %, direction 0.03° |

### Calibration module (2026-11-21, branch `feature/calibration-service`, `include/ExVectrDSP/calibration/`, `src/calibration/`)

Pure code (no Arduino, no ExVectrCore), host-tested from the flight-software repo (`pio test -e native`: `test_cal_accel_fit`, `test_cal_mag_ellipsoid`, `test_cal_stillness`; the session engine on top is `src/calibration/`, [project/calibration.md](../project/calibration.md)). Time is injected as int64 ns. Samples are raw sensor frame.

| Piece | What it does |
|---|---|
| `fitAccel` (`accel_ellipsoid_fit`) | 12-pose magnitude fit (9 parameters: offset `b`, symmetric `A`; Levenberg-Marquardt on pose means, `a_cal = A (x − b)`), diagonal fallback with 6-8 poses (at best `Warn`), verdict `CalResult` + check bits (poses, coverage, condition, residual rms/max, scale, off-diagonal, offset; offset limit `kAccelOffsetLimit` = 2.5 m/s²). Pose distribution is accepted from the fit result: `AccelFitResult::sigma[9]` (standard errors from `sigma^2 (J^T J)^-1`, sigma = max(residual rms, `noiseFloor` 0.005 m/s²); offsets in m/s², scale/skew dimensionless), `wellDetermined` (offset sigma < `maxOffsetSigma` 0.05, scale sigma < `maxScaleSigma` 0.005) sets `kAccelCheckCoverage`; a singular normal matrix gives `sigmaValid = false` and `Numeric`. The old per-axis 25° rule (`accelCoverageMask` etc.) is removed: it rejected valid fits with poses at 45° between sensor axes. Helpers `accelWeakParams`, `accelParamName`, `accelParamLimit`, `accelParamAdvice` are shared by the fit and the vehicle's operator hints. |
| `MagEllipsoidFit` (`mag_ellipsoid_fit`, 2026-11-22, replaces `MagSphereFit`) | Hard + soft iron fit, `m_cal = A (x − b)`, `|m_cal| = 0.05`, `A` symmetric. (1) Online accumulation of all samples in double on data normalised (shifted by the first sample, divided by `normScale` 0.3): 9x9 normal matrix of the algebraic quadric with `trace(Q) = 1`, centre and matrix square root of `Q/k` (Denman-Beavers); (2) Levenberg-Marquardt refinement of the 9 parameters on the direction-thinned buffer (600 samples, caller-provided, 5° thinning) minimising `(|A (x − b)|/rRef − 1)²`; (3) two 3-sigma outlier passes (robust sigma = 1.4826 × median residual, floor 0.2 %), rejected count reported. Result: `offset`, `A`, eigenvalues, `axisRatio`, residual rms/max on the inliers, coverage (`SphereCoverage`), standard errors from `σ² (JᵀJ)⁻¹`, check bits and `CalResult` (order: samples/coverage, singular → `Numeric`, residual or too many rejected → `FailResidual`, not well determined → `FailCoverage`, soft-iron / hard-iron range → `FailRange`, 5-15 % rejected or residual rms 1.5..3.5 % → `Warn`, check bit `kMagCheckResidualNominal`; the reason text then reads `field noise or disturbance during the tumble (rms X %)`). Config `MagEllipsoidConfig` (defaults: rms < 3.5 % (`maxResidualRms`, relaxed from 1.5 % by owner decision 2026-11-22; `warnResidualRms` 1.5 % is the Pass/Warn boundary), max < 10 % (was 5 %), offset sigma < 1 % R, matrix sigma < 2 %, axis ratio < `kMagAxisRatioMax` 1.5, eigenvalues 0.6..1.6 × mean, \|b\| < 0.8 R, >= 300 stored, coverage >= 75 %). `softIron = false` is the old sphere behaviour (`A = s I`, algebraic sphere fit, no refinement or rejection). `magFailReason()` gives the operator text. Memory: 600 stored samples (caller) plus about 3 KB of scratch members (the vehicle keeps the object in DMAMEM). |
| `SphereCoverage` | 64 equal-area bins on a Fibonacci lattice (shared definition with the ground station). |
| `StillnessDetector` | Still when the last windowBlocks 0.1 s blocks are inside the gyro and accel limits (calibrated-rate norm with a gyro bias, or bias-free std / peak-to-peak). |
| `PoseCollector` | Automatic static pose capture: still 1.5 s, averaged 2 s, new if > 20° from every stored pose (up to 24). |
| `GyroBiasCollector` | Stationary gyro bias run (`start(now, deadline, windowNs)`: the window is per run, `kGyroWindowStartupNs` 5 s default, `kGyroWindowCountdownNs` 3 s for the go countdown, T9b-4; limits identical): 5 consecutive 1 s blocks inside the motion limits anywhere before the deadline (a bad block restarts the window; limits `GyroBiasConfig` 2026-11-22: gyro std 1.5 °/s, p2p 8 °/s, accel axis and \|a\| std 0.3 m/s², noise 1.5 °/s; the worst violated limit, `bestRun` and `rejectedBlocks` are kept in the result and `describeGyroRun()` formats them for the failure log), deadline, bias/noise/temperature acceptance, mean raw accel and mag of the window, **block diagnostics** (worst and quietest block, last block) used for tuning the limits. |
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
| Low | `src/body_sim_model.cpp` (`addFlap`) | Flap geometry and fold direction (toward +X, confirmed by the owner 2026-11-21) follow the documented frame and the owner's dimensions, but the force directions and magnitudes have not been checked against hardware. | Verify flap torque signs on the bench (e.g. pivot test) before trusting the sim's azimuth behaviour. |
| ~~Low~~ Fixed | `src/body_simulator.cpp` | End-cap area `2πr²`. | Fixed 2026-11-21 (`feature/sim-aero`): axial force on πR². |
| ~~Low~~ Fixed | `src/body_simulator.cpp` | Dead `calcTorqueFromTVC`, wrong comments. | Fixed 2026-11-21 (`feature/sim-aero`): removed. |
| ~~High~~ Fixed | `src/body_simulator.cpp` | Belly-down fall 14.9 m/s (real 7.7), no physically based pitch stiffness or damping (ad hoc `z −= u_z/200` CoP shift), Newtonian flap law with a sign-reversing azimuth term, no `ω × Iω`, wall-clock dt (a pause gave one huge step), phantom +m·g thrust before the first TVC message, roll torque not gated with the TVC (review F1-F3, F8-F11, F13-F15). | Fixed 2026-11-21 (`feature/sim-aero`): redesign (section "Body simulator"); measured before/after table there. |
| High | `include/ExVectrDSP/body_sim_model.hpp` `aeroCalibration` = 4.64 with the fold shield 0.7 (was 4.00 without it, 4.38, 4.23, 3.08; 2026-11-21) | The documented ship (1.1 kg, the measured mass; the owner reverted a 1.2 kg change, 13 × 74 cm, flaps 2×82 + 2×152 cm²) gives a belly-down fall of 13.5 m/s with textbook coefficients; the real fall of 6.4 m/s (21..25 km/h, flight position filter of barometer plus accelerometer, owner 2026-11-21) needs a drag area of **0.43 m²** (CdA = 2mg/ρv²) instead of 0.097 m²: 4.4 times, which no drag coefficient of this shape explains (a 13 × 74 cm body has a frontal area of 0.096 m²; at CdA 0.097 the ship would have to weigh 0.25 kg or fall at 13.5 m/s). The single factor scales **all** aerodynamic forces and moments, so it also triples stiffness, damping and flap authority relative to the geometry. | Owner to check: the 6.4 m/s figure (vertical speed, steady, which sensor), missing drag sources. The aero moments are now larger than the TVC authority (0.26 N m at 14 N; trim moment 0.56 N m at 6.4 m/s), which matters for the RTH flip. |
| Med | `BodySimParams::cgFromBottom_m` = 0.37 | With the CoM at the owner's "middle" the neutral flaps (50/40°) leave a 0.43 N m nose-down moment at belly-down; the free body trims by gliding nose-first (4.2 m/s horizontal at 7 m/s down, AoA 58°) while the body stays horizontal. Neutral flaps trim exactly belly-down with the CoM at 0.33 m. | Measure the CoM; check flight logs for horizontal drift in the belly-down descent. |
| High | `src/body_sim_model.cpp`, `ControlAttitudeFlaps` neutral angles and gains | With the corrected geometry (bottom flaps 1 cm above the bottom, top flaps canted 10°, 1.1 kg) the neutral flaps 50/40° leave −0.56 N m at belly-down (6.4 m/s); the free body glides (8.4 m/s at AoA 24° with the shield). With the fold shield on (2026-11-21) the flap-controller replica holds belly-down (max 32°, pitch −5 ± 2.5°) and turns toward the heading setpoint, but stays in a yaw/roll limit cycle of about ±34° (period about 4 s) instead of aligning; the bench full chain shows the same (±34° after the initial 174° turn). The 14 N / 3° flip completes (upright after 2.0 s). | **Owner retunes the controller** (neutral angles and gains were NOT changed): pitch trim (at top 50° the bottom flaps need ≈ 90° to null the moment) and the yaw/roll limit cycle. Re-run `test_sim_aero` afterwards and make `test_closed_loop_flap_hold` assert again. |
| Medium | `src/body_sim_model.cpp` (`flapFrame`), `flapFoldTowardBelly`, `flapFoldShieldFactor` | **Azimuth (heading) sign was reversed in the sim; fixed 2026-11-21 by enabling the fold shield (`flapFoldShieldFactor` = 0.7, owner-approved, recalibrated `aeroCalibration` 4.64)**: all three axis derivatives now have the controller's sign at 50/40° over AoA −20..+20° (yaw +0.0005..+0.0038, pitch +0.020..+0.024, roll +0.007..+0.009 N m/deg), the bench full chain turns the nose toward the heading setpoint (174° → 0° in about 5 s), and `test_control_axis_signs` asserts the signs. The value 0.7 and the `sin²` shape are fitted to the sign, not measured, and the yaw margin is small at AoA +20° (+0.0005). History of the analysis (owner 2026-11-21: heading turns away from the target in freefall): Analysis 2026-11-21 (test `test_control_axis_signs`): no sign error in the mixer replica, flap index order (controller → `StarshipFlaps` → `BodySimulator` → `setFlapInput`), normals, lever arms, cant direction (upper end inward, checked) or force direction; the ground station draws all four flaps and the attitude the same way as the model. Belly-down, the controller's u0 must give +τx (about world up). τx of a flap comes only from its sideways force, τx = −z·F_y; with the fold toward the back F_y is inward, |F_y| = ½ρV²A·C_N(α)·cos α, α = 90° − a. At the neutral angles both pairs are past the maximum of C_N cos α (α = 40°/50°), so extending a flap LOWERS its inward force (TL: F = [+0.474 −0.556 −0.098] N, extend 1° → ΔF_y = +0.0078 N, Δτx = −0.0016 N m; BL: ΔF_y = +0.0088 N at z = −0.264 m, Δτx = +0.0023 N m): ∂τx/∂u0 = −0.0081 N m/deg (−0.0078 to −0.0129 for AoA ±20°). This holds for any flat-plate C_N curve (attached, stalled, low-AR); the old Newtonian law (C_N ∝ sin²α, maximum of C_N cos α at a = 35°) had the right sign by accident. With the fold toward the belly (`flapFoldTowardBelly = true`) the sideways force points outward and every derivative keeps its magnitude with the controller's sign (+0.0081 N m/deg; pitch, roll, drag, terminal speed unchanged). **Fold direction confirmed by the owner (2026-11-21): toward the back (+X), `flapFoldTowardBelly` stays false.** **Effective flap angle (2026-11-21, owner's insight, `flapEffectiveMinAngleRad` 20°):** implemented, but it changes the angles 40–50° by only 0.2–0.3°, so ∂τx/∂u0 at 50/40 stays −0.0080 N m/deg (−0.0077 to −0.0129 over AoA ±20°). Neutral-angle sweep (`test_neutral_sweep_effective_angle`, top/bottom 0..90° in 10° steps, AoA 0, 1e-4 N m/deg): yaw is + at every AoA −20..+20° only with bottom ≥ 70° (any top: +95..+638) or top ≥ 80° (most bottoms; top 70° with bottom 0–10°); everywhere else, including 50/40, it is −3..−241. Pitch (∂τy/∂u1, +3..+295) and roll (∂τz/∂u2, +1..+104) are + over the whole grid (one exception: pitch at top 90°/bottom 0° at some AoA). **Candidate effects (one at a time, at 50/40, calibration fixed; `test_candidate_effects_yaw_sign`):** (a) sidewash `flapSidewashGain` ±0.5..±1: stays − (−0.005..−0.025); only −2 restores the sign, with an unphysical turning angle (δ ≈ 1.2 rad) and 5× pitch authority. (b) cylinder potential crossflow `flapBodyCrossflowGain` 0.5/1: worse (−0.010/−0.020; the flow converges behind the body, raising the folded flaps' α). (c) fold shield `flapFoldShieldFactor`: 0.3 −0.003, 0.5 ≈ 0 (sign varies with AoA), **0.7 restores + at every AoA (+0.0004..+0.0036)** with pitch +17 % and roll +23 % at fixed calibration, terminal 6.89 m/s; recalibrated (factor 5.06) the yaw derivative is +0.0041, pitch +0.0266, roll +0.0096 N m/deg and the flap-controller replica holds belly-down (pitch −5.3 ± 2.4°, belly max 31°, azimuth error +21.8° after 15 s from +20°) instead of diverging. (c) is the physically plausible one (a flap folding toward +X lies along the downstream half of the cylinder, in its separated wake and boundary layer, so its dynamic pressure falls as it folds), but the value 0.7 and the `sin²` shape are fitted to the sign, not measured. | **Owner: still verify on the real ship** (bench pivot test in an airflow, belly-down: yaw moment vs u0 at 50/40°, and the sideways force of one flap vs its angle). If the sideways force does not fall clearly as a flap folds toward the body, set `flapFoldShieldFactor` back to 0, recalibrate (4.00) and look for another effect. Sidewash and body crossflow stay off. Real-flight evidence that the heading loop works: a late flight aligned its heading well. |
| Low | `src/body_sim_model.cpp` | A gliding back-down fall (body +X down) is also a stable equilibrium in the model once reached (from a tilt toward the belly side); it is unstable at zero glide. | Real behaviour unknown. |
| Low | `src/body_sim_model.cpp` | Not modelled: sensors (perfect, zero-latency state, review F12), motor spool-up (owner: instantaneous), vane effectiveness vs thrust and crossflow, flap-body interference (only as the effective flap angle; the candidate effects are off), ground tip-over, gusts. | Simulation remains necessary but not sufficient. |
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


Note (2026-11-21, `feature/sim-faults`): simulated flap servo faults (stuck, slow; telecommands 16..18) are applied in `src/starship_flaps.cpp` to the modelled true angle that feeds this model, not inside the library; the model itself is unchanged. See overview.md "Simulation mode" and `src/safety/sim_flap_fault.hpp`. The closed-loop reproduction with the top-left flap stuck at 42 deg is `test_sim_fault_reproduction_tl_stuck` in `test/test_sim_aero` (`-v | grep SIMFAULT`).
