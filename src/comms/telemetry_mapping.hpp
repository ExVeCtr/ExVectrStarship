#ifndef COMMS_TELEMETRY_MAPPING_HPP
#define COMMS_TELEMETRY_MAPPING_HPP

/**
 * Vehicle state -> generated ExVectrPackets telemetry packets (schema starship.toml / common.toml).
 *
 * Pure functions on plain data: no Arduino, no ExVectrCore, no globals. They are host-tested
 * (`pio test -e native`, test/test_telemetry_mapping). The vehicle side fills a TelemetrySnapshot once per
 * telemetry tick (main.cpp, TelemetryProvider) and VehicleComms sends the packets built here.
 *
 * Units: the snapshot holds the vehicle's internal SI values (NWU, attitude [w,x,y,z] body -> world,
 * position [V(3), P(3)], ns). The packets carry SI values; their fixed-point wire scaling and saturation are
 * done by the generated encode(). Definitions owned by the flight software (failure_bits, sensor_state,
 * mode_text, Euler angles) are documented in docs/project/hardware-and-drivers.md section 9.
 */

#include <stdint.h>

#include "ExVectrPackets/gen/all.hpp"

#include "safety/command_ingress.hpp"
#include "safety/rc_mapper.hpp"
#include "telecommand.hpp"
#include "telemetry.hpp"

namespace VCTR {
namespace Comms {

/**
 * One sensor's calibration as the vehicle applies it (DSP::TopicCoordTransform, main.cpp initialiseMemory and
 * CalibApply): cal = matrix * (raw - offset), matrix row-major (matrix[3*r + c]) = calibration scale (key 10x .cov)
 * times the stored sensor->body transform (key 20x), offset = key 10x .val. Vehicle-internal units: gyro rad/s,
 * accel m/s^2, mag gauss (the QMC5883 driver unit). makeCalibration() converts the mag offset to T.
 */
struct SensorCalibration {
  float offset[3] = {0, 0, 0};
  float matrix[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  uint8_t source = 0; ///< Packets::Calibration::k*Source* bits (same bit positions for the three sensors)
};

/// Everything the telemetry sets need, sampled once per telemetry tick.
struct TelemetrySnapshot {
  int64_t t_ns = 0; ///< vehicle time NowNs()

  // Vehicle and mission state (VehicleSafetyAndControlTask)
  VehicleMode mode = VehicleMode::VehicleMode_Startup;
  MissionMode missionMode = MissionMode_Idle;
  MissionPhase missionPhase = MissionPhase_Idle;
  uint8_t missionIndex = 0;
  SensoryState sensors = {};
  FailureState failures = {};
  bool armed = false;
  bool simulation = false;
  bool motorsEnabled = false; ///< motor outputs enabled by the existing logic (StarshipTVC motors && actuators)
  bool gsPresent = false;     ///< ground station node 0 present (information only)
  bool incompatible = false;  ///< a present node reported another network version or schema hash
  bool uartPeerSeen = false;  ///< valid frame from the ship node (3) on the UART within 3 s (information only)
  bool zeroing = false;       ///< estimator zeroing window active (boot zero or ZeroEstimators)
  uint8_t homeEpoch = 0;      ///< HomeEpoch::value(): 0 = no position reference yet, then increments per re-set

  // Calibration state (VEHICLE_STATUS.calib_state), cached by the safety task from Calibration::Validity
  bool calibGyroValid = false, calibAccelValid = false, calibMagValid = false;
  bool calibSessionActive = false; ///< a calibration session or the automatic gyro run is collecting or fitting
  bool readyToArm = false;         ///< Calibration::readyToArm(): the conditions the Arm command accepts

  // Estimates (attitude/position topic switches: estimators, or the body simulator in simulation mode)
  float att[7] = {0, 0, 0, 1, 0, 0, 0}; ///< [wx, wy, wz, qw, qx, qy, qz]
  float pos[6] = {0, 0, 0, 0, 0, 0};    ///< [Vx, Vy, Vz, Px, Py, Pz]

  // Mission
  float setpoint[6] = {0, 0, 0, 0, 0, 0}; ///< [Vx, Vy, Vz, Px, Py, Pz]
  int64_t missionTime_ns = 0;
  float stopAltitudeM = 20.0f;        ///< stop altitude parameter of the freefall mission (always sent)
  float flipStartAltitudeM = NAN;     ///< getStopAltitude() while the freefall phase is active, else NaN
  uint8_t waypointIndex = 0xFF; ///< current waypoint of the waypoint mission, 0xFF = another mission is active
  /// Attitude setpoint of the active attitude controller, [w,x,y,z] body -> world (NWU). All 0 = none
  /// (attitudeSetpoint()).
  float setpointQ[4] = {0, 0, 0, 0};

  // Actuators
  float flaps[4] = {0, 0, 0, 0};     ///< true flap angles [TL, TR, BL, BR], rad
  float tvcThrust[4] = {0, 0, 0, 0}; ///< StarshipTVC actual [Fx, Fy, Fz, T], body frame, N (T: roll torque)
  float groundDistance = 0;          ///< m, NaN if unavailable

  // GNSS
  double latitude = 0, longitude = 0; ///< rad
  float altitude = 0;                 ///< m above the ellipsoid
  float gnssVel[3] = {0, 0, 0};       ///< NWU, m/s
  uint8_t satellites = 0;
  bool gnssValid = false;
  uint8_t gnssFixType = 0;            ///< real u-blox UBX-NAV-PVT fixType (0 none, 2 2D, 3 3D, 4 GNSS+DR, 5 time only)
  bool gnssHaveData = false;          ///< at least one GNSS measurement received (else AIR_GNSS values are NaN)
  float hAcc = 0, vAcc = 0, sAcc = 0; ///< 1 sigma, m, m, m/s

  // Raw sensors (sensor frame)
  float gyro[3] = {0, 0, 0};  ///< rad/s
  float accel[3] = {0, 0, 0}; ///< m/s^2
  float mag[3] = {0, 0, 0};   ///< T
  float pressure = 0;         ///< Pa
  float temperature = 0;      ///< K

  // Applied sensor calibration (CALIBRATION) and the attitude EKF gyro bias state (body frame, rad/s)
  SensorCalibration gyroCal, accelCal, magCal;
  float gyroBiasEst[3] = {0, 0, 0};

  // Estimator standard deviations
  float attStd[3] = {0, 0, 0}; ///< rad, about the world axes (attitudeStdWorld)
  float velStd[3] = {0, 0, 0}; ///< m/s
  float posStd[3] = {0, 0, 0}; ///< m

  // Control source (GS-control cutover; filled by fillCmdTelemetry from the comms task)
  uint8_t ctrlSource = 0;                ///< VehicleStatus.ctrl_source: 0 NONE, 1 GS (latched session, state at most 2 s old)
  float rcAgeS = __builtin_nanf("");     ///< age of the last accepted CMD_STATE, s, NaN = none (VehicleStatus.rc_age)
  uint16_t ctrlFlags = 0;                ///< VehicleStatus.ctrl_flags (INTERLOCK_NEED_TOGGLE, USB_FRESH, UART_FRESH, SOURCE_LATCHED, MULTI_MASTER, GS_FAILSAFE_REQ)
  uint16_t cmdSession = 0;               ///< VehicleStatus.cmd_session: low 16 bits of the latched session, 0 = none
  // Command-path state of the vehicle (T9b-4, filled by the safety task through fillSnapshot): ctrl_flags bits that
  // only the vehicle logic knows (SC_MISMATCH, SE_HOLD_ZEROING, SE_REFUSED, FAILSAFE_LATCHED), OR-ed with ctrlFlags.
  uint16_t cmdFlags = 0;
  uint8_t goRefusal = 0;                 ///< VehicleStatus.go_refusal (0 none)
  uint8_t missionProfile = 255;          ///< VehicleStatus.mission_profile: SB latched at the last go, 255 = none
  uint8_t rcRejected = 0;                ///< wrapping count of rejected CMD_STATE frames (all reasons, both links)
  uint8_t degradedBits = 0;              ///< VehicleStatus.degraded_bits (T9b-3: UART_HEARTBEAT_LOST only; information, never latched)
  /// CMD_APPLIED content (USB only): the last accepted CMD_STATE fields (safe values if none)
  struct CmdAppliedView {
    uint8_t go = 0, profile = 0, vehicleMode = 0, motors = 0, resetButton = 0, failsafe = 0;
    uint8_t flags = 0;    ///< CMD_APPLIED.flags (POCKET_LIVE bit 0, INPUT_VIRTUAL bit 1)
    uint8_t safeSeen = 0;
    uint32_t session = 0;
    float ageS = __builtin_nanf(""), usbAgeS = __builtin_nanf(""), uartAgeS = __builtin_nanf("");
    uint8_t rejects[4] = {0, 0, 0, 0};
  } cmdApplied;
};

// ---- flight-software owned bit fields and helpers ----

/**
 * VEHICLE_STATUS.home_epoch: 0 until the vehicle has a position reference (home), then a counter that increments each
 * time the reference is re-set. The reference is the GNSS reference point of the position KF, which moves only while
 * the estimators are in zeroing mode, so the vehicle calls referenceSet() when the zeroing mode ENDS (falling edge of
 * VehicleSafetyAndControlTask::applyEstimatorZeroing(): boot zero, ZeroEstimators, mission Initialisation zeroing,
 * and an aborted window, which also moved the reference). The first call gives 1; after 255 the counter wraps to 1
 * (never back to 0 = "no home").
 */
class HomeEpoch {
public:
  void referenceSet() { value_ = value_ == 255 ? 1 : static_cast<uint8_t>(value_ + 1); }
  uint8_t value() const { return value_; }

private:
  uint8_t value_ = 0;
};

/// VEHICLE_STATUS.calib_state: bit0 gyro valid, bit1 accel valid, bit2 mag valid, bit3 session active, bit4 ready to
/// arm, bits 5-7 zero.
uint8_t packCalibState(bool gyroValid, bool accelValid, bool magValid, bool sessionActive, bool readyToArm);

/// failure_bits: bit 0 sensor, 1 attitude, 2 position, 3 position out of bounds, 4 attitude out of bounds,
/// 5 radio connection loss.
uint32_t packFailureBits(const FailureState &f);

/// sensor_state: 2 bits per sensor (TelemetrySensor 0 Init, 1 Calib, 2 Ready, 3 Failure): bits 0-1 GNSS,
/// 2-3 IMU, 4-5 baro, 6-7 mag, 8-9 attitude KF, 10-11 position KF.
uint16_t packSensorState(const SensoryState &s);

/// (NowNs / 1000) mod 2^32, the t_us field of every telemetry packet.
uint32_t timeUs(int64_t t_ns);

/// ZYX (yaw-pitch-roll) Euler angles of q = [w,x,y,z] (body -> NWU world), rad. Yaw about world +Z
/// (counterclockwise seen from above, 0 = body X towards North).
void quatToEulerZYX(const float q[4], float &roll, float &pitch, float &yaw);

/// Attitude standard deviation, rad, as small rotations about the world (NWU) axes [North, West, Up], from the
/// attitude EKF's quaternion q = [w,x,y,z] (body -> world) and its 4x4 quaternion covariance P (row-major, same
/// order): with dq = 0.5 * Omega(q) * dtheta for q' = [1, dtheta/2] (x) q, Sigma_theta = 4 * Omega^T P Omega and
/// out[i] = sqrt(Sigma_theta[i][i]) (negative variances give 0). At q = identity this is 2 * sqrt(P[i+1][i+1]).
void attitudeStdWorld(const float q[4], const float P[16], float out[3]);

/// Course over ground from an NWU velocity: clockwise from North, [0, 2 pi).
float courseOverGround(float vNorth, float vWest);

/// Thrust deflection angles of a body-frame thrust vector: [atan2(Fx, Fz), atan2(Fy, Fz)], rad.
void tvcDeflection(const float thrust[3], float out[2]);

/// Short display text for HANDSET_TELEM (<= 8 chars incl. the simulation marker).
void modeText(VehicleMode mode, MissionPhase phase, bool armed, bool simulation, char out[8]);

// ---- packet builders ----

Packets::VehicleStatus makeVehicleStatus(const TelemetrySnapshot &s);
Packets::AirPose makeAirPose(const TelemetrySnapshot &s);
/// AIR_GNSS: position (rad), altitude (m), NWU velocity. Without any GNSS measurement all values are NaN (the
/// generated codec's sentinels); an invalid fix still sends the receiver's values (fix_type in AIR_GNSS_INFO tells).
Packets::AirGnss makeAirGnss(const TelemetrySnapshot &s);
/// AIR_GNSS_INFO: real fix type, satellites, h/v accuracy (NaN = unknown when no measurement was received).
Packets::AirGnssInfo makeAirGnssInfo(const TelemetrySnapshot &s);
Packets::Pose makePose(const TelemetrySnapshot &s);
Packets::Actuators makeActuators(const TelemetrySnapshot &s);
Packets::Mission makeMission(const TelemetrySnapshot &s);
Packets::Gnss makeGnss(const TelemetrySnapshot &s);
Packets::HandsetTelem makeHandsetTelem(const TelemetrySnapshot &s);
Packets::ImuRaw makeImuRaw(const TelemetrySnapshot &s);
Packets::Estimator makeEstimator(const TelemetrySnapshot &s);
/// raw_frame SENSOR; offsets and matrices copied, the mag offset converted gauss -> T (x 1e-4); the matrices are
/// unit-free, so the ground station gets cal = M * (raw - offset) in SI from the IMU_RAW values.
Packets::Calibration makeCalibration(const TelemetrySnapshot &s);
/// CMD_APPLIED (0x84, USB only, 10 Hz) from snapshot.cmdApplied.
Packets::CmdApplied makeCmdApplied(const TelemetrySnapshot &s);
/// Fills snapshot.ctrlSource, rcAgeS, ctrlFlags, cmdSession, rcRejected and cmdApplied from the command ingress
/// state and the mapper's safe-seen bits. Ages come from `nowNs` and the receive times. `interlockNeedToggle` sets
/// ctrl_flags INTERLOCK_NEED_TOGGLE (MotorInterlock::needToggle).
void fillCmdTelemetry(TelemetrySnapshot &s, const Safety::CmdIngressSnapshot &cs, const Safety::RcCommandMapper &m,
                      int64_t nowNs, bool interlockNeedToggle = false);

// ---- AIR_ACTUATORS: two samples per packet ----

/// One actuator sample for AIR_ACTUATORS (same source and definitions as ACTUATORS).
struct ActuatorSample {
  int64_t t_ns = 0;
  float tvc[2] = {0, 0};  ///< thrust deflection [x, y], rad (tvcDeflection of the TVC actual thrust)
  float rollTorque = 0;   ///< TVC actual thrust topic component T (twist / roll actuation), N*m
  float thrust = 0;       ///< |[Fx, Fy, Fz]|, N
  float flaps[4] = {0, 0, 0, 0}; ///< [TL, TR, BL, BR], rad
};

/// The actuator sample of a snapshot.
ActuatorSample makeActuatorSample(const TelemetrySnapshot &s);

/**
 * Ring of the last kSize actuator samples (VehicleNetwork pushes one every kSampleTicks ticks, 50 Hz), so the
 * previous sample in AIR_ACTUATORS is exactly the one taken one sample period before the newest.
 */
class ActuatorHistory {
public:
  static constexpr size_t kSize = 4;
  void push(const ActuatorSample &a) {
    head_ = (head_ + 1) % kSize;
    ring_[head_] = a;
    if (count_ < kSize) count_++;
  }
  size_t count() const { return count_; }
  /// age 0 = newest, 1 = the one before. Requires age < count().
  const ActuatorSample &get(size_t age) const { return ring_[(head_ + kSize - age) % kSize]; }
  void clear() { count_ = 0; }

private:
  ActuatorSample ring_[kSize];
  size_t head_ = kSize - 1;
  size_t count_ = 0;
};

/// AIR_ACTUATORS from the newest and the previous sample; values of a missing sample (and prev_age) are NaN.
/// Requires h.count() >= 1 (otherwise every value is NaN and t_us = 0).
Packets::AirActuators makeAirActuators(const ActuatorHistory &h);

// ---- attitude setpoint (MISSION.setpoint_q, AIR_MISSION.setpoint_q) ----

/// Which attitude controller's setpoint is published.
enum class AttitudeSetpointSource : uint8_t { None = 0, Tvc, Flaps };

/**
 * Selection by the controller phase table (src/safety/control_phase.hpp) as applied on the vehicle:
 * flapsStabilize = phase-table bellyflop set on AND the flap controller setting applied by FlapModeObserverTask is
 * BellyFlopMode Stabilize; tvcEnabled = phase-table TVC set on. Flaps win if both are on, which the phase table
 * excludes since 2026-11-21 (in the bellyflop the kick's TVC set and the flap attitude control are exclusive; see
 * docs/project/hardware-and-drivers.md section 9).
 */
AttitudeSetpointSource selectAttitudeSetpointSource(bool tvcEnabled, bool flapsStabilize);

/// Normalises q to unit length with w >= 0. Non-finite or near-zero (|q| < 0.5) input gives the zero quaternion
/// (= no setpoint).
void sanitizeSetpointQuat(const float in[4], float out[4]);

/**
 * The flap attitude controller (CTRL::ControlAttitudeFlaps, Stabilize) tracks only the direction of body +Z:
 * azimuth = atan2(z_W, z_N) and pitch = elevation of body +Z in the NWU world frame (the controller's own
 * definitions). Roll about body +Z is not controlled. The published quaternion is the current attitude qIs rotated
 * by the shortest world-frame rotation that brings body +Z onto the wanted direction, so the uncontrolled roll
 * follows the real vehicle. Output body -> world, unit, w >= 0; zero if qIs is invalid.
 */
void flapPointingSetpoint(float azimuth, float pitch, const float qIs[4], float out[4]);

struct AttitudeSetpointInputs {
  bool tvcEnabled = false;     ///< phase table: TVC set (ControlAttitudeTvc) enabled
  bool flapsStabilize = false; ///< phase table bellyflop set on and flap setting mode Stabilize
  float tvcQ[4] = {0, 0, 0, 0}; ///< ControlAttitudeTvc setpoint topic (acc-to-att mapping), q part, body -> world
  /// The TVC setpoint is written directly by a mission (bellyflop kick, MissionBellyflop::kickActive()); then
  /// tvcOverrideQ replaces tvcQ when the TVC source is selected.
  bool tvcOverride = false;
  float tvcOverrideQ[4] = {0, 0, 0, 0};
  float flapAzimuth = 0, flapPitch = 0; ///< ControlAttitudeBellyFlopSetting azimuthAngle_Rad / pitchAngle_Rad
  float qIs[4] = {1, 0, 0, 0};          ///< current attitude estimate, body -> world
};

/// The attitude setpoint for telemetry: the selected source's setpoint, sanitised; zero if none is active.
void attitudeSetpoint(const AttitudeSetpointInputs &in, float out[4]);

/// AIR_MISSION, the air version of MISSION.
Packets::AirMission makeAirMission(const TelemetrySnapshot &s);

// ---- TEMPORARY legacy telecommand adapter (removed in T9b) ----

/**
 * TEMPORARY (network migration T9a, removed in T9b): the old `Telecommand` (8 bytes, formerly network port 1000
 * over the SX1280) carried unchanged through the new stack on port kPort, so the vehicle stays commandable until
 * the RC_STATE command path (T9b) replaces it. Layout, explicit little-endian: u8 type, u8 subsystem,
 * i16 paramInt, f32 paramFloat (identical to the old packed struct on the Teensy). Exactly 8 bytes, else rejected.
 * Accepted only from node 0 (ground station). Same handler, same semantics as before
 * (VehicleSafetyAndControlTask::handleTelecommands).
 */
struct LegacyTelecommandPacket {
  static constexpr uint8_t kPort = 0x7F; ///< TEMPORARY, not in the ExVectrPackets schema
  static constexpr size_t kMaxSize = 8;
  static constexpr uint8_t kAcceptedSource = 0; ///< ground station

  Telecommand tc = {};

  size_t encode(uint8_t *out, size_t cap) const;
  bool decode(const uint8_t *in, size_t len);
};

} // namespace Comms
} // namespace VCTR

#endif
