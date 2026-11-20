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
  bool gsPresent = false;     ///< ground station node 0 present (the radioConnectionLoss observable)
  bool incompatible = false;  ///< a present node reported another network version or schema hash
  bool uartPeerSeen = false;  ///< valid frame from the ship node (3) on the UART within 3 s (information only)

  // Estimates (attitude/position topic switches: estimators, or the body simulator in simulation mode)
  float att[7] = {0, 0, 0, 1, 0, 0, 0}; ///< [wx, wy, wz, qw, qx, qy, qz]
  float pos[6] = {0, 0, 0, 0, 0, 0};    ///< [Vx, Vy, Vz, Px, Py, Pz]

  // Mission
  float setpoint[6] = {0, 0, 0, 0, 0, 0}; ///< [Vx, Vy, Vz, Px, Py, Pz]
  int64_t missionTime_ns = 0;

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
};

// ---- flight-software owned bit fields and helpers ----

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
