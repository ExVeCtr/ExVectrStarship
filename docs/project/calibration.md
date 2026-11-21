# Sensor calibration (gyro, accelerometer, magnetometer)

> Operator procedure, vehicle behaviour and the rules that gate Arm and MissionBegin. Implements step **C2** of the workspace plan `docs/calibration-design.md` (section 6.1 "Owner decisions" overrides the design where they differ). Code: `src/calibration/` (pure, host-tested engine and rules, plus the platform task), the fit maths in `lib/ExVectrDSP/src/calibration/` ([libs/ExVectrDSP.md](../libs/ExVectrDSP.md#calibration-module-2026-11-21)), the wiring in `VehicleSafetyAndControlTask` (`main.cpp`).

## Rules (owner decisions 2026-11-21)

- **All three calibrations must be good to fly.** If the gyro, the accelerometer or the magnetometer calibration is invalid, the vehicle logs a **warning every 10 s**, **refuses `Arm` (0xA5) and `MissionBegin`** (with the reason), and a vehicle that is armed on the ground is disarmed. **This is identical in simulation and real mode** (owner decision 2026-11-21: "during simulation the real sensors and their data should be calibrated too"; safety.md rule 9): no arm or go logic reads the simulation flag.
- **Gyro: automatic.** The bias is measured at every boot (in the automatic boot zero), in every `ZeroEstimators` window of at least 7 s, and in the mission countdown. There is no hard-coded gyro bias any more.
- **Countdown:** when the gyro run that closes the countdown has finished, **all three calibrations are checked again**; if any is bad the mission is **aborted before liftoff (reset and stop)** with the reason in the log.
- **Accelerometer:** 12-pose magnitude fit, no jig needed: the poses only have to be different and still. **Magnetometer:** hard-iron bias only (sphere fit, scale to 0.05 driver units).
- Accel and mag results are stored in the **legacy EEPROM keys** (no CRC, no rollback): calibrate once, reuse until it is seen to no longer work. Rollback is manual, from the old values in [hardware-and-drivers.md](hardware-and-drivers.md#8a-sensor-calibration-pipeline-what-calibration-reproduces).

## What is checked ("valid")

`Calibration::calibrationsValid` (`src/calibration/calibration_validity.cpp`, host test `test/test_calibration_validity`). Every tolerance is a named constant in `calibration_validity.hpp`; they are proposals from the design (datasheet figures), **not measured on this vehicle**: tune them there after the first session.

| Sensor | Valid when | Constants |
|---|---|---|
| Gyro | A bias run **passed in this power cycle** (a later failed run makes it invalid again until one passes). The stored key 100 value is only the boot offset, never proof. | run limits below |
| Accel | Key 102 was **read from memory** (not created as a default), is **not the identity default**, parameters in range: \|b_k\| <= 1.5 m/s², sensor-frame matrix `A = T⁻¹ C T` with diagonal 0.95..1.05 and off-diagonal <= 0.02. **Live check** in the still window of the gyro run: \|a_cal\| = 9.81 ± 0.25 m/s². | `kAccelOffsetMax`, `kAccelDiag*`, `kAccelOffDiagMax`, `kAccelLiveTolerance` |
| Mag | Key 101 read from memory, not the identity default, `C = s·I` (diagonal within ±20 % of the mean, off-diagonal <= 5 %), raw field radius `R = 0.05/s` in 0.1..1.0, \|b\| < 0.8 R. **Live check:** \|B_cal\| = 0.05 ± 30 %. | `kMag*` |

The reason is logged per sensor, for example `gyro: ok, accel: parameters out of range, mag: ok`. **Current bench vehicle:** the stored accel offset `b_z = 2.14 m/s²` is above the 1.5 limit, so **Arm and MissionBegin are refused until the owner recalibrates the accelerometer** (bench 2026-11-21: `WARNING calibration invalid ... accel: parameters out of range`; gyro and mag were `ok`).

### Gyro run limits (`GyroBiasConfig`, lib)

5 s of continuous stillness measured in 1 s blocks; a block restarts the window if any gyro axis std > 0.25 °/s, peak-to-peak > 1 °/s, any accel axis std > 0.05 m/s² or std of \|a\| > 0.05 m/s². Accept: \|bias\| < 5 °/s, noise std < 0.15 °/s, IMU temperature change < 0.5 K. The run gives up at its deadline (`MOTION` if blocks were rejected, else `TIMEOUT`).

- **Window and deadline:** the run starts with the zeroing window and its deadline is **window end − 1 s**, so the new bias is applied while the attitude EKF is still in zeroing mode (it learns the residual with an IIR of 0.005 per 1 kHz sample, about 0.2 s). A window shorter than **7 s** does not run it (`kGyroRunMinWindowNs`: 5 s window + 1 s reserve + 1 s slack; the plan said 6 s).
- **Countdown:** the waypoint mission's Initialisation phase lasts `−paramInt − 10` s. `MissionBegin` needs **`paramInt <= −18`** (8 s: 5 s window + 3 s slack), otherwise it is refused (`kMinCountdownParamInt`). **The ground-station `MissionBegin` default of −10 is therefore refused** until its config is changed.
- **Bench noise floor (2026-11-21, ship on the bench, USB, servos unpowered by the vehicle):** quietest 1 s blocks gyro std 0.06 °/s, peak-to-peak 0.37 °/s, accel axis std 0.02 m/s² (limits 0.25, 1.0, 0.05); typical gyro bias (1.34, 1.34, −0.49) °/s at 36.5 °C, noise 0.06-0.08 °/s. The environment on the bench was not always quiet: blocks with gyro std up to 0.7 °/s and accel std 0.2 m/s² occurred, some boot runs failed with `MOTION` (5-9 restarts) and a 60 s manual run never completed during a busy period. A run needs **5 s without disturbance**: handling the ship, a person walking on the floor or wind on the pad will make it fail. The log lines `Calibration: worst 1 s block ...` and `quietest of N blocks ...` after every run show the margins.

## No simulation exemption (owner decision 2026-11-21)

An earlier version of this branch exempted simulation mode from the validity check (`kCheckExemptInSimulation`). **It was removed**: in simulation the sensors and the stored calibration keys are real, so the check works the same way, and a rehearsal in simulation must go through the same refusals and the same countdown abort as a real run (safety.md rules 1.2 and 9). The decisions for `Arm`, `MissionBegin`, the countdown re-check and the armed-in-Ready disarm are the pure functions `decideArm`, `decideMissionBegin`, `decideCountdownEnd` and `mustDisarmInReady` in `calibration_validity.hpp`; none of them has a simulation input, so real and simulation behaviour are identical by construction (host-tested in `test/test_calibration_validity`). The calibration commands also work in simulation mode (they act on the real sensors); their preconditions are the same as in real mode. Consequence for rehearsals: a simulated mission needs valid calibrations and `MissionBegin` with `paramInt <= -18` (the ground-station default of -10 is refused).

## Operator procedure (first calibration, vehicle on the bench)

Preparation: flight configuration (battery strapped, servos and wiring as flown, **props off**), power on, USB to the ground station, recording on, **leave it still for 5 min** (warm-up: the MPU9250 heats a few K). The boot zero's gyro run should log `gyro bias run PASS`. Mission Idle, disarmed, motors off.

Commands are the legacy telecommands (port 0x7F) listed below. Until the ground-station config has entries for them (it only has `CalibMag*` and `MagSave`), send them with `tools/bench_vehicle_usb.py` (`tcs <type> <subsystem> <paramInt>`, after `exgs sources disable usb`, which that tool does itself).

1. **Accelerometer (about 5 min).** `CalibStart` accel, 1. Put the ship in **12 different, still poses** (nose up, nose down, four horizontal rolls, four 45° up, two 45° down: design section 2.4; no axis has to be aligned with gravity, each new pose only has to differ by > 20° from every earlier one). Hold each still for about 4 s. Never hold it by hand and never balance it on the nose or flaps: use a padded cradle/box. Log cues: `Accel: still, measuring pose (hold 2 s)`, then `pose N captured, move to the next one`; `same orientation as an earlier pose ..., turn it further`; `moved during the measurement, pose dropped`; every 2 s `Accel: N poses captured (still|moving)`. **If it never says "still"**, the table or floor is vibrating or the gyro bias is stale (see Known issues); wait for a quiet moment. After 12 poses send `CalibStart` accel, 0 (stop and fit). Expect `Accel fit PASS` (or `WARN`: offset above 0.8 m/s² or the 6-pose diagonal fallback) and `Accel candidate ready`. A `FAIL_*` result: `CalibStart` accel, −1 (discard) and repeat with more varied poses.
2. `CalibApply` accel. Log: `accel calibration applied (RAM only until Save)`. The estimator zero is invalidated and the automatic zero runs: **keep the ship still for 10 s**, then Ready.
3. **Magnetometer (60-120 s), outdoors or at least 2 m from steel, PCs, speakers, in flight configuration.** `CalibStart` mag, 1. Hold the ship at its middle and tumble it slowly (about 30°/s): nose to eight compass headings, tip it up and down through vertical while rolling a full turn, then a few random tumbles. Cues: every 2 s `Mag: N samples, M stored, coverage X % (need 75 %)`. When coverage is above 75 % send `CalibStart` mag, 0. Expect `Mag fit PASS` (coverage >= 75 %, >= 300 stored samples, residual rms < 3 %, \|c\| < 0.8 R) and `Mag candidate ready`; a failure says which check; discard (−1) and repeat.
4. `CalibApply` mag, keep still for the automatic zero.
5. **Verify** (optional but recommended): `CalibStart` accel, 2 with three poses not used in the fit, then 0 (`Accel VERIFY PASS`: \|a_cal\| = 9.81 ± 0.05 m/s² in every pose, at least 3 poses); `CalibStart` mag, 2, tumble 30 s, then 0 (`Mag VERIFY PASS`: spread < 3 %, coverage >= 60 %); `CalibStart` gyro, 2 (still 5 s: `Gyro VERIFY PASS`: bias equals the active one within 0.1 °/s).
6. **Save:** `Save` (any of the three subsystems): writes the EEPROM image and reads it back (`saved to EEPROM and verified by read-back`, or `EEPROM save FAILED`). Then **reboot** and check the `CALIBRATION` packet (source bits `0x03` for accel and mag = read from EEPROM; the gyro source shows `0x08` RAM-only after the boot run, because the per-boot gyro bias is deliberately not written to EEPROM) and that the log has no `WARNING calibration invalid`.
7. `Arm` must now be accepted (in real mode). If the live check fails (`still-window check failed`), the reason says which sensor: for the mag it usually means the field at the place of use differs more than 30 % from the calibration place.

Repeat the gyro run only by `ZeroEstimators` (it is automatic at every boot and countdown). Repeat accel/mag calibration after storage > 3 months, a hard landing, an IMU or mounting change, or when verification fails.

## Telecommands (legacy path, port 0x7F, `src/telecommand.hpp`)

All calibration commands require: all inputs initialised, **disarmed, motors off, mode Startup / Ready / SensorCalib, active mission Idle or Finished** (`Calibration::checkCommand`; the same in simulation mode); otherwise `Calibration <what> refused: <reason>` (log, 1/s). Only the newest telecommand per 100 ms tick is handled.

| Type | Subsystem | paramInt | Effect |
|---|---|---|---|
| 0 `CalibStart` | 0 gyro, 1 accel, 2 mag | 1 (or any value > 2) | Calibrate: enters SensorCalib (actuators off, disarmed), aborts an active zeroing window **and clears the valid zero**, starts collecting. Refused if another sensor collects. |
| 0 `CalibStart` | same | 2 | Verify the active calibration, no candidate; ends in Idle. |
| 0 `CalibStart` | same | 0 or any value <= −2 | Stop and fit (accel, mag; a manual gyro run cannot be shortened and is cancelled). Fit in the next 100 ms tick. |
| 0 `CalibStart` | same | −1 | Discard a candidate or end a collection. The active calibration is untouched. |
| 1 `CalibApply` | 0 / 1 / 2 | – | Apply the candidate **only if its verdict is PASS or WARN** (owner decision D6; no way to apply a failing fit). Writes the legacy keys into the RAM image (accel `C = T A T⁻¹`, so `M = C T = T A`, offset `b`; mag `C = s I`, offset = centre), applies the transform, aborts a zeroing window, **invalidates the estimator zero** (Ready goes back to Startup, the automatic zero runs). |
| 3 `Save` | 0 / 1 / 2 | – | RAM image to EEPROM, read back and compared. Refused while a sensor collects. |

A session ends (mode back to Startup, automatic zero) when no sensor is collecting or fitting. A candidate in REVIEW stays until applied, discarded or a reboot.

State per sensor: `IDLE → COLLECTING → FITTING → REVIEW | FAILED`, `REVIEW → APPLIED → SAVED` (`CalibrationService`, `src/calibration/calibration_service.*`). Timeouts: accel 15 min and mag 5 min (auto stop and fit), manual gyro run 60 s.

## Everything the vehicle logs (LOG_TEXT)

`Calibration: ...` lines (session progress, results, `gyro bias run started/PASS/FAILED`, `worst 1 s block`, `quietest of N blocks`), `Arm refused: calibration invalid (...)`, `Mission begin refused: calibration invalid (...)` / `countdown too short`, `WARNING calibration invalid, Arm and MissionBegin are refused (repeats every 10 s): ...` (only outside runs and sessions), `Vehicle disarmed: calibration invalid (...)`, `Gyro bias run NOT started: a calibration session is busy` / `Mission begin refused: the countdown gyro calibration run could not start`, `gyro bias run PASS discarded, the vehicle is no longer on the ground or in its countdown` (a late result is never applied in flight), `Calibration candidate rejected: parameters not acceptable for arming (...)` (accel/mag fit that passes its own checks but fails the arming range check: it cannot be applied), and at the end of a countdown run either `all three calibrations checked ..., countdown continues` or `MISSION ABORTED before liftoff: calibration invalid (<reason>). Reset and stop.` There is no telemetry field for the abort reason (`VEHICLE_STATUS.flags` is full; a field is a packet change): it is in the log only (and in `lastCalibAbortReason_` in RAM).

## Flight-safety notes

- Changes the **estimator inputs**: the boot gyro offset is no longer the hard-coded `(0.92, 2.0, 0.6) °/s` but key 100 until the boot run applies the measured bias (about 5-10 s after boot, inside the boot zero window, deadline window end − 1 s); an accel or mag apply changes `M` and `b` the estimators receive, always followed by a new zero.
- **Adds refusals to Arm and MissionBegin** and an abort inside the countdown (ground, motors at most at idle: the existing "on the ground" reset-and-stop, shared method `resetAndStopOnGround`).
- A passing automatic gyro run changes the applied bias only in Startup, Ready or SensorCalib, or when it is the countdown's run and the countdown is still running (`Calibration::gyroBiasMayApply`); a late result (starved 10 Hz task) is discarded and logged. A finished-but-untaken result is delivered before a new run starts and never lost; if the countdown's run cannot start, `MissionBegin` is refused.
- Calibration can only be applied on the ground (disarmed, motors off, Startup/Ready/SensorCalib, no mission), never in flight (it works in simulation mode too, on the real sensors). Nothing can make a calibration invalid in flight (runs and sessions need the ground state), so the "armed implies valid" disarm acts in mode Ready only.
- Verification in simulation mode (to do by the owner after this change): with invalid calibrations `Arm` and `MissionBegin -20` are refused exactly as in real mode, `MissionBegin -12` is refused as too short, and with valid calibrations `MissionBegin -20` runs the gyro run in its Initialisation phase and reaches Running; MissionAbort in the countdown is `on the ground: reset and stop` (the shared method). Bench evidence below; the real-mode refusals are bench-verified, the real-mode acceptance path (valid calibrations) and the abort at the end of a countdown are covered by host tests only until the owner has valid calibrations (the bench vehicle cannot pass the accel check, and relaxing the limits for a bench build was not done).

## Bench evidence (2026-11-21, bench vehicle, ship not moved, feature/calibration-service)

| Check | Result |
|---|---|
| Boot | `gyro bias run started (window 9.0 s)` at 4.3 s, `PASS` at 9-12 s, bias (1.34, 1.34, −0.49) °/s (was key 100: 0.185, 1.152, −0.426), `CALIBRATION` gyro source gains `RAM_ONLY` (0x0B), no `OFFSET_OVERRIDE`; boot zero completes, Ready. In a noisy period the run failed (`MOTION`, 5-9 restarts) and the log said so. |
| Invalid calibration, real mode | `WARNING calibration invalid ... gyro: ok, accel: parameters out of range, mag: ok` every 10 s; `Arm` → `Arm refused: calibration invalid (...)`; `MissionBegin -20` → `Mission begin refused: calibration invalid (...)`. With a failed run the reason was `gyro: last run failed`. |
| Commands | `CalibStart` gyro verify/calibrate: ran, 60 s give-up with `FAIL` in a noisy period, `PASS` result and `Review` when quiet; `CalibApply` gyro: applied, estimator zero invalidated, automatic zero + run followed, `Arm` during it refused (zeroing active). Accel and mag sessions on the stationary ship ended `FAIL_COVERAGE` (0 poses / 14 % coverage), `CalibApply` refused (`no passing candidate`), discard worked, the mode returned to Startup. `Save` with an unchanged image: `saved to EEPROM and verified by read-back`. |
| Simulation mode (**obsolete since 2026-11-21**: this was measured with the removed simulation exemption) | `CalibStart` was refused in simulation mode and `MissionBegin -12` / `-20` began with invalid calibrations. Now all of this behaves as in real mode (section above). The parts that do not depend on the exemption still hold: the gyro run runs in the −20 Initialisation phase (`PASS`), Init → Startup → Running, `MissionAbort` in the countdown → `on the ground: reset and stop`, in flight → `start RTH`. |
| Not done | Armed refusal and the short-countdown refusal in real mode (needs valid calibrations): host tests only. Abort at the end of a countdown: host-tested decision (`decideCountdownEnd`), execution shares `resetAndStopOnGround` with MissionAbort (bench-verified). A `Save` of a changed value and the accel/mag fits on real motion: not done (owner). |

## Known issues

| Severity | Issue | Impact |
|---|---|---|
| Medium | The motion limits are the design's proposals; on a busy bench 50-70 % of 1 s blocks were rejected (section above). | The boot run, `ZeroEstimators` runs and countdown runs can fail in a noisy place; a failed countdown run aborts the mission. Tune `GyroBiasConfig` after real pad data. |
| Medium | After a failed boot run the gyro offset in use is the stale key-100 value (0.185, 1.152, −0.426) °/s until a run passes. | Pose stillness then uses the bias-free fallback (std and peak-to-peak) so poses can still be captured; the estimators see the stale bias (the EKF learns the residual). |
| Low | The abort reason has no telemetry field. | Log only (`MISSION ABORTED before liftoff: calibration invalid (...)`). Owner decision 2026-11-21: no automatic retry of the countdown run (he will not fly in wind that makes it fail). `LOG_TEXT` goes out on USB only (hardware-and-drivers.md), so over the radio the ground station cannot show the reason without a schema change (a status field or bit); on USB it is in the Console panel. |
| Low | A steady slow rotation about the gravity axis (under 5 °/s, smooth) does not change the accelerometer and could pass the gyro still check as a bias (review finding 7, not changed). | Needs 5 s of steady yaw; the EKF corrects a wrong yaw-rate bias. Optional fix: compare the mag direction of the first and last block. |
| Low | `Save` persists a gyro bias only after a manual gyro `CalibApply` (the automatic per-boot bias is not written to the image). | By design: a Save does not write a per-boot value into the stored calibration. |
| Low | Verification of a changed-value `Save` on hardware was not done (see the task report). | Check source bits and values after the first real Save and reboot. |
