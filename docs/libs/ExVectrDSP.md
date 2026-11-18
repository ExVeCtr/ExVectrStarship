# ExVectrDSP

> State estimation and signal processing for ExVectr: IMU attitude EKF, IMU/baro/GNSS/range position KF, sensor-frame transforms and calibration, generic KF and filters, plus a rigid-body flight simulator. Repo: https://github.com/ExVeCtr/ExVectrDSP. `Last verified against commit: 3e1f070`

## Purpose & scope

- Turns raw sensor topics into the two estimate topics the rest of the system consumes:
  - **attitude** `[ω_body(3), q(4)]`
  - **position** `[v(3), p(3)]` in NWU
  - plus a ground-distance estimate.
- Provides `TopicCoordTransform` (sensor→body plus calibration) and calibrators for gyro and magnetometer.
- Contains `BodySimulator`, a hardware-in-the-loop simulator task that publishes the same topic types as the estimators, so the application can switch between real and simulated state.
- Generic helpers: `KalmanFilter<LEN>`, `ValueCov`, first-order `LowPassFilter` and `HighPassFilter`.
- Not implemented:
  - `FullStateEKF` is declared only, with no `.cpp`.
  - `gnss_data.hpp` is empty.
  - FFT (README TODO).
  - `imu_attitude_cf` is not a complementary filter; it is an EKF variant.
- The local branch is 1 commit behind `origin/Development` (`402831f`). The parent repo pins `15512dc`, which is 1 commit behind the checked-out HEAD.

## Dependencies (ExVectr libs + external)

- **ExVectrCore**: `Topic`, `Simple_Subscriber`, `Buffer_Subscriber`, `Callback_Subscriber`, `Task_Periodic`, `Timestamped`, `ListBuffer`, `NOW()`, `LOG_MSG`, `randGen`.
- **ExVectrMath**: all algebra. Frame conventions come from `docs/libs/ExVectrMath.md`.
- **ExVectrSensor**: `SNSR::GNSSData` (`gnss.hpp`). Lat/lon in rad, alt in m above the ellipsoid, velocity documented as NED (`imu_gps_position_kf.hpp:128`).
- **ExVectrData**: `Data::Memory_Interface` and memory manager, used by `Calibrator_Manager`.
- **ExVectrControl** (**circular**): `body_simulator.hpp:4-5` includes `ExVectrControl/physics_bodysim.hpp` and `starship/control_attitude_flaps.hpp`, while Control includes `ExVectrDSP/value_covariance.hpp`. PlatformIO resolves this because all headers are on the include path. The CMake build would need both libraries.
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
  - `getAttitudeEstTopic()` publishes `Timestamped<Vector<7>>` = `[ω_meas − b (rad/s, body), q]`, timestamp `NOW()`.
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

### `BodySimulator` (`body_simulator.hpp/.cpp`, a `Task_Periodic` registered on the system scheduler)
- Wraps `CTRL::PhysicsBodySim`.
- **Forces per tick:**
  - Gravity: world frame, applied as an acceleration.
  - TVC force at `tvcPosition_`, clamped by angle and magnitude, plus a roll torque `T`.
  - Cylinder side drag at z = ±0.1, with a speed-dependent CoP shift of −1/200·airflow_z.
  - End-cap drag at z = ±0.35.
  - 4 flap drag forces. The flap CoP rotates with the flap angle; flap angles are clamped to 10–90° (0° = extended, normal +X; 90° = retracted).
  - Random lateral wind, 0–4 m/s along Y plus a −1 m/s bias along X, refreshed every 1 s.
  - Ground clamp at p_U < 0.
- **Inputs:**
  - `setTVCInputTopic(Topic<Vector<4>> [Fx,Fy,Fz,T] body N / N·m, tvcPosition, angleLimit, forceLimit)`.
  - `setFlapSettingInputTopic(Topic<ControlAttitudeFlapSetting>)`.
  - `enableTVC`, `enableFlaps`, `enableZeroingMode`, `setAttitudeState`, `setPositionState`.
  - Note: the `flapsEnabled_` flag is stored but never read; flap forces are always applied.
- **Outputs**, same layouts as the estimators:
  - `getAttitudeEstTopic()` `[ω,q]`.
  - `getStateEstTopic()` `[v,p]`.
  - `getGroundTopic()` `[v_U, p_U]`.
- **Constructor:** `(mass, inertia diag, thrustLimit, angleLimit, interval = 10 ms)`. Main uses `(1.1 kg, {0.044,0.044,0.004}, 14 N, 45°/15 = 3°)`.

### Calibrators (`calibrator_*.hpp/.cpp`)
- `Calibrator_Interface<T,DIM>` holds `calibrationData_` (`Timestamped<ValueCov>`: val = bias, cov = transform), `sensorToBodyMat_` and `isCalibrated_`.
- `Calibrator_Gyroscope(duration)` averages raw gyro over `duration` after `initCalibration()`. It outputs `val = mean` and `cov = sensorToBody`.
- `Calibrator_Magnetometer` tracks min/max per axis. It outputs `val = (max+min)/2` and `cov = sensorToBody·diag(scale)`.
- `Calibrator_Manager<T,DIM>` (a Task) persists `ValueCov` via `Data::Memory_Interface` under `memoryKey` and pushes it into a `TopicCoordTransform`. It has `saveCalibrationData()`, `loadCalibrationData()` and `applyCalibrationData()`.
- **All are commented out in `src/main.cpp:120-124`.** Main has its own magnetometer least-squares calibration task.

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

Simulator in place of the estimators (`src/main.cpp:117`, `:920-960`):

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
- Zeroing modes are toggled per mission phase at `:620-690`. `setPositionReference` at `:826` and `:1527`. State reset at `:1480-1481`.
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
| Med | `src/imu_gps_position_kf.cpp:348-351`, `:453` | `positionCov.magnitude(true)`: the bool becomes `startRow = 1`, so element 0 is excluded. | The GNSS validity gating ignores the north/first covariance component. |
| Med | `src/imu_gps_position_kf.cpp:415-418` | `updateAccBias` treats the GNSS velocity difference as world NWU and adds the up-gravity, while GNSS velocity is NED. The non-zero path also divides by `dTime`, which can be 0. | Wrong bias if ever called with real motion. Currently only called with `assumeZero`. |
| Med | `src/imu_gps_position_kf.cpp:276-343` | The range is used as a vertical distance with no tilt compensation. | Ground distance is wrong when tilted. The VL53L0X window is 0.4–1.5 m, so this matters during touchdown. |
| Med | `src/body_simulator.cpp:58-65` (Quat-semantics) | The TVC angle clamp uses `Quat(f×z, α−limit).conjugate().rotate(f)`, which pushes the vector further from +Z (to `2α − limit`). The app's `src/starship_tvc.cpp:88-91` uses a double conjugate and is correct. | The simulator lets TVC exceed the angle limit, so the sim is optimistic about control authority. |
| Med | `src/body_simulator.cpp:115` | `airFlowVector = -velocityBody - windVelocity`: a world-frame wind is subtracted from a body-frame vector without rotation. The sign is also inverted for a wind-velocity definition. | Wind acts along body axes and rotates with the vehicle, so the wind disturbance model is wrong. |
| Med | `src/body_simulator.cpp:213-214` | Zeroing mode sets `positionState_ = {0,0,0,1,0,0}` (pN = 1 m) and `attitudeState_ = {0,0,0, 0,0,0,1}` (q = (0,0,0,1), a 180° yaw). | The simulator starts 1 m north and yawed 180°. Likely intended zeros and identity. |
| Low | `src/body_simulator.cpp:105` (Quat-semantics) | `attitude.conjugate().rotate(velocity)` is world→body. | Correct with current Math; it was inverted before. |
| Low | `src/body_simulator.cpp:157-168`, `:321-334` (Quat-semantics) | Flap CoP and normal rotations `Quat(z, ±angle).rotate(...)` change sign vs the old Math. The flaps now fold toward +X at 90°. | Geometry is mirrored relative to the tuning date. Verify the flap force and torque directions in sim against hardware. |
| Low | `src/body_simulator.cpp:385` | End-cap area is `2πr²` instead of `πr²`. | End-cap drag is twice too high. |
| Low | `src/body_simulator.cpp:404-412`; `include/ExVectrDSP/body_simulator.hpp:36-37` | `calcTorqueFromTVC` is unused and returns only the roll torque. The comments "30 degrees" / "1000 N" don't match the values 8° / 12. | Dead code and misleading comments. |
| Low | `include/ExVectrDSP/body_simulator.hpp:80` | The `flapsEnabled_` flag is stored but `update()` always applies flap forces. | `enableFlaps()` has no effect in sim. |
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
