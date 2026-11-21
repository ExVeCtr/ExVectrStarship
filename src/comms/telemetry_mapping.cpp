#include "telemetry_mapping.hpp"

#include <math.h>
#include <string.h>

namespace VCTR {
namespace Comms {

namespace {

constexpr float kPi = 3.14159265358979323846f;

float nanf_() { return NAN; }

void copy3(const float *in, float *out) {
  out[0] = in[0];
  out[1] = in[1];
  out[2] = in[2];
}

} // namespace

uint32_t packFailureBits(const FailureState &f) {
  uint32_t b = 0;
  if (f.sensorFailure) b |= 1u << 0;
  if (f.attitudeFailure) b |= 1u << 1;
  if (f.positionFailure) b |= 1u << 2;
  if (f.positionOutOfBounds) b |= 1u << 3;
  if (f.attitudeOutOfBounds) b |= 1u << 4;
  if (f.radioConnectionLoss) b |= 1u << 5;
  return b;
}

uint16_t packSensorState(const SensoryState &s) {
  auto v = [](TelemetrySensor t) { return static_cast<uint16_t>(static_cast<uint8_t>(t) & 0x03u); };
  return static_cast<uint16_t>(v(s.gnss) | (v(s.imu) << 2) | (v(s.baro) << 4) | (v(s.mag) << 6) |
                               (v(s.attitudeKF) << 8) | (v(s.positionKF) << 10));
}

uint32_t timeUs(int64_t t_ns) { return static_cast<uint32_t>(static_cast<uint64_t>(t_ns / 1000)); }

void quatToEulerZYX(const float q[4], float &roll, float &pitch, float &yaw) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  float sp = 2.0f * (w * y - z * x);
  if (sp > 1.0f) sp = 1.0f;
  if (sp < -1.0f) sp = -1.0f;
  pitch = asinf(sp);
  yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void attitudeStdWorld(const float q[4], const float P[16], float out[3]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  // Columns of Omega(q): [0, e_i] (x) q, i.e. dq/dtheta_i * 2 for a world-frame rotation about axis i.
  const float om[3][4] = {{-x, w, -z, y}, {-y, z, w, -x}, {-z, -y, x, w}};
  for (int i = 0; i < 3; i++) {
    float v = 0;
    for (int r = 0; r < 4; r++)
      for (int c = 0; c < 4; c++) v += om[i][r] * P[4 * r + c] * om[i][c];
    v *= 4.0f;
    out[i] = v > 0 ? sqrtf(v) : 0.0f;
  }
}

float courseOverGround(float vNorth, float vWest) {
  float c = atan2f(-vWest, vNorth); // east = -west
  if (c < 0.0f) c += 2.0f * kPi;
  if (c >= 2.0f * kPi) c = 0.0f;
  return c;
}

void tvcDeflection(const float thrust[3], float out[2]) {
  out[0] = atan2f(thrust[0], thrust[2]);
  out[1] = atan2f(thrust[1], thrust[2]);
}

void modeText(VehicleMode mode, MissionPhase phase, bool armed, bool simulation, char out[8]) {
  const char *t = "?";
  switch (mode) {
  case VehicleMode::VehicleMode_Startup:
  case VehicleMode::VehicleMode_SensorInit:
    t = "INIT";
    break;
  case VehicleMode::VehicleMode_SensorCalib:
    t = "CALIB";
    break;
  case VehicleMode::VehicleMode_Ready:
    t = armed ? "ARMD" : "RDY";
    break;
  case VehicleMode::VehicleMode_Failure:
    t = "FAIL";
    break;
  case VehicleMode::VehicleMode_Running:
    switch (phase) {
    case MissionPhase_Hover: t = "HOVR"; break;
    case MissionPhase_Ascent: t = "ASCN"; break;
    case MissionPhase_Descent: t = "DESC"; break;
    case MissionPhase_Deceleration: t = "DECL"; break;
    case MissionPhase_RTH: t = "RTH"; break;
    case MissionPhase_Landing: t = "LAND"; break;
    case MissionPhase_Completed: t = "DONE"; break;
    default: t = "RUN"; break;
    }
    break;
  }
  memset(out, 0, 8);
  size_t n = strlen(t);
  if (n > 5) n = 5;
  memcpy(out, t, n);
  if (simulation) {
    out[n] = ' ';
    out[n + 1] = 'S';
  }
}

Packets::VehicleStatus makeVehicleStatus(const TelemetrySnapshot &s) {
  Packets::VehicleStatus p;
  p.t_us = timeUs(s.t_ns);
  p.t_ns = s.t_ns;
  p.mode = static_cast<uint8_t>(s.mode);
  p.mission_mode = static_cast<uint8_t>(s.missionMode);
  p.mission_phase = static_cast<uint8_t>(s.missionPhase);
  p.ctrl_source = Packets::ControlSource::kNone; // T9b: RC control sources do not exist yet
  uint8_t f = 0;
  if (s.armed) f |= Packets::VehicleStatus::kFlagsArmed;
  if (s.simulation) f |= Packets::VehicleStatus::kFlagsSimulation;
  if (s.motorsEnabled) f |= Packets::VehicleStatus::kFlagsMotorsEnabled;
  if (s.gsPresent) f |= Packets::VehicleStatus::kFlagsGsPresent;
  if (s.incompatible) f |= Packets::VehicleStatus::kFlagsIncompatible;
  if (s.uartPeerSeen) f |= Packets::VehicleStatus::kFlagsUartPeerSeen;
  if (s.mode == VehicleMode::VehicleMode_Failure) f |= Packets::VehicleStatus::kFlagsFailsafe;
  if (s.zeroing) f |= Packets::VehicleStatus::kFlagsZeroing;
  p.flags = f;
  p.failure_bits = packFailureBits(s.failures);
  p.sensor_state = packSensorState(s.sensors);
  p.last_req_id = 0;     // T9b: no REQUEST handling yet
  p.last_req_status = 0;
  p.rc_age = nanf_();    // T9b: no RC_STATE yet
  return p;
}

Packets::AirPose makeAirPose(const TelemetrySnapshot &s) {
  Packets::AirPose p;
  p.t_us = timeUs(s.t_ns);
  for (int i = 0; i < 4; i++) p.q[i] = s.att[3 + i];
  copy3(&s.pos[3], p.pos);
  copy3(&s.pos[0], p.vel);
  return p;
}

Packets::Pose makePose(const TelemetrySnapshot &s) {
  Packets::Pose p;
  p.t_us = timeUs(s.t_ns);
  copy3(&s.att[0], p.omega);
  for (int i = 0; i < 4; i++) p.q[i] = s.att[3 + i];
  copy3(&s.pos[0], p.vel);
  copy3(&s.pos[3], p.pos);
  return p;
}

Packets::Actuators makeActuators(const TelemetrySnapshot &s) {
  Packets::Actuators p;
  p.t_us = timeUs(s.t_ns);
  for (int i = 0; i < 4; i++) p.flaps[i] = s.flaps[i];
  tvcDeflection(s.tvcThrust, p.tvc);
  p.thrust = sqrtf(s.tvcThrust[0] * s.tvcThrust[0] + s.tvcThrust[1] * s.tvcThrust[1] +
                   s.tvcThrust[2] * s.tvcThrust[2]);
  p.ground_distance = s.groundDistance;
  return p;
}

ActuatorSample makeActuatorSample(const TelemetrySnapshot &s) {
  ActuatorSample a;
  a.t_ns = s.t_ns;
  tvcDeflection(s.tvcThrust, a.tvc);
  a.rollTorque = s.tvcThrust[3];
  a.thrust = sqrtf(s.tvcThrust[0] * s.tvcThrust[0] + s.tvcThrust[1] * s.tvcThrust[1] +
                   s.tvcThrust[2] * s.tvcThrust[2]);
  for (int i = 0; i < 4; i++) a.flaps[i] = s.flaps[i];
  return a;
}

Packets::AirActuators makeAirActuators(const ActuatorHistory &h) {
  Packets::AirActuators p;
  p.t_us = h.count() > 0 ? timeUs(h.get(0).t_ns) : 0;
  p.prev_age = h.count() > 1 ? static_cast<float>(static_cast<double>(h.get(0).t_ns - h.get(1).t_ns) * 1e-9)
                             : nanf_();
  for (size_t k = 0; k < 2; k++) {
    const bool have = k < h.count();
    const ActuatorSample *a = have ? &h.get(k) : nullptr;
    p.tvc[2 * k + 0] = have ? a->tvc[0] : nanf_();
    p.tvc[2 * k + 1] = have ? a->tvc[1] : nanf_();
    p.roll_torque[k] = have ? a->rollTorque : nanf_();
    p.thrust[k] = have ? a->thrust : nanf_();
    for (int i = 0; i < 4; i++) p.flaps[4 * k + i] = have ? a->flaps[i] : nanf_();
  }
  return p;
}

// ---- attitude setpoint (MISSION / AIR_MISSION setpoint_q) ----

namespace {

void quatMul(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void zeroQ(float out[4]) { out[0] = out[1] = out[2] = out[3] = 0.0f; }

} // namespace

void sanitizeSetpointQuat(const float in[4], float out[4]) {
  const float n2 = in[0] * in[0] + in[1] * in[1] + in[2] * in[2] + in[3] * in[3];
  if (!isfinite(n2) || n2 < 0.25f) { // also catches NaN components
    zeroQ(out);
    return;
  }
  const float k = (in[0] < 0.0f ? -1.0f : 1.0f) / sqrtf(n2);
  for (int i = 0; i < 4; i++) out[i] = in[i] * k;
}

void flapPointingSetpoint(float azimuth, float pitch, const float qIs[4], float out[4]) {
  float q[4];
  sanitizeSetpointQuat(qIs, q);
  if (q[0] == 0 && q[1] == 0 && q[2] == 0 && q[3] == 0) {
    zeroQ(out);
    return;
  }
  // Body +Z in world: third column of R(q).
  const float a[3] = {2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]),
                      1 - 2 * (q[1] * q[1] + q[2] * q[2])};
  // Wanted body +Z in world, inverse of the controller's azimuth/pitch definition.
  const float cp = cosf(pitch);
  const float b[3] = {cp * cosf(azimuth), cp * sinf(azimuth), sinf(pitch)};
  const float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  float dq[4];
  if (d < -0.999999f) {
    // Opposite: rotate pi about any axis perpendicular to a.
    float ax[3] = {0, -a[2], a[1]}; // a x (1,0,0)
    if (ax[1] * ax[1] + ax[2] * ax[2] < 1e-6f) {
      ax[0] = a[2];
      ax[1] = 0;
      ax[2] = -a[0]; // a x (0,1,0)
    }
    dq[0] = 0;
    dq[1] = ax[0];
    dq[2] = ax[1];
    dq[3] = ax[2];
  } else {
    dq[0] = 1 + d;
    dq[1] = a[1] * b[2] - a[2] * b[1];
    dq[2] = a[2] * b[0] - a[0] * b[2];
    dq[3] = a[0] * b[1] - a[1] * b[0];
  }
  float dqn[4], r[4];
  sanitizeSetpointQuat(dq, dqn);
  quatMul(dqn, q, r); // apply the world-frame correction after body -> world
  sanitizeSetpointQuat(r, out);
}

AttitudeSetpointSource selectAttitudeSetpointSource(bool tvcEnabled, bool flapsStabilize) {
  if (flapsStabilize) return AttitudeSetpointSource::Flaps;
  if (tvcEnabled) return AttitudeSetpointSource::Tvc;
  return AttitudeSetpointSource::None;
}

void attitudeSetpoint(const AttitudeSetpointInputs &in, float out[4]) {
  switch (selectAttitudeSetpointSource(in.tvcEnabled, in.flapsStabilize)) {
    case AttitudeSetpointSource::Tvc: sanitizeSetpointQuat(in.tvcQ, out); break;
    case AttitudeSetpointSource::Flaps: flapPointingSetpoint(in.flapAzimuth, in.flapPitch, in.qIs, out); break;
    default: zeroQ(out); break;
  }
}

Packets::AirMission makeAirMission(const TelemetrySnapshot &s) {
  Packets::AirMission p;
  p.t_us = timeUs(s.t_ns);
  p.mission_time = static_cast<float>(static_cast<double>(s.missionTime_ns) * 1e-9);
  p.mission_mode = static_cast<uint8_t>(s.missionMode);
  p.phase = static_cast<uint8_t>(s.missionPhase);
  p.index = s.missionIndex;
  p.waypoint = s.waypointIndex;
  copy3(&s.setpoint[3], p.setpoint_pos);
  copy3(&s.setpoint[0], p.setpoint_vel);
  for (int i = 0; i < 4; i++) p.setpoint_q[i] = s.setpointQ[i];
  return p;
}

Packets::Mission makeMission(const TelemetrySnapshot &s) {
  Packets::Mission p;
  p.t_us = timeUs(s.t_ns);
  copy3(&s.setpoint[3], p.setpoint_pos);
  copy3(&s.setpoint[0], p.setpoint_vel);
  p.mission_time = static_cast<float>(static_cast<double>(s.missionTime_ns) * 1e-9);
  p.index = s.missionIndex;
  p.phase = static_cast<uint8_t>(s.missionPhase);
  for (int i = 0; i < 4; i++) p.setpoint_q[i] = s.setpointQ[i];
  return p;
}

Packets::Gnss makeGnss(const TelemetrySnapshot &s) {
  Packets::Gnss p;
  p.t_us = timeUs(s.t_ns);
  p.latitude = s.latitude;
  p.longitude = s.longitude;
  p.altitude = s.altitude;
  copy3(s.gnssVel, p.vel);
  p.satellites = s.satellites;
  p.fix_type = s.gnssValid ? 3 : 0; // the driver exposes no u-blox fix type: 3 = valid (>= 5 satellites)
  p.h_acc = s.hAcc;
  p.v_acc = s.vAcc;
  p.s_acc = s.sAcc;
  return p;
}

Packets::HandsetTelem makeHandsetTelem(const TelemetrySnapshot &s) {
  Packets::HandsetTelem p;
  uint8_t f = 0;
  if (s.armed) f |= Packets::HandsetTelem::kFlagsArmed;
  if (s.mode == VehicleMode::VehicleMode_Failure) f |= Packets::HandsetTelem::kFlagsFailure;
  if (s.simulation) f |= Packets::HandsetTelem::kFlagsSimulation;
  if (s.gnssValid) f |= Packets::HandsetTelem::kFlagsGnssFix;
  p.flags = f;
  modeText(s.mode, s.missionPhase, s.armed, s.simulation, p.mode_text);
  p.latitude = s.latitude;
  p.longitude = s.longitude;
  p.ground_speed = sqrtf(s.pos[0] * s.pos[0] + s.pos[1] * s.pos[1]);
  p.heading = courseOverGround(s.pos[0], s.pos[1]);
  p.altitude = s.pos[5];
  p.satellites = s.satellites;
  p.vertical_speed = s.pos[2];
  quatToEulerZYX(&s.att[3], p.roll, p.pitch, p.yaw);
  p.battery = nanf_(); // no battery monitoring: encodes as 0 = unknown
  return p;
}

Packets::ImuRaw makeImuRaw(const TelemetrySnapshot &s) {
  Packets::ImuRaw p;
  p.t_us = timeUs(s.t_ns);
  copy3(s.gyro, p.gyro);
  copy3(s.accel, p.accel);
  copy3(s.mag, p.mag);
  p.pressure = s.pressure;
  p.temperature = s.temperature;
  return p;
}

Packets::Estimator makeEstimator(const TelemetrySnapshot &s) {
  Packets::Estimator p;
  p.t_us = timeUs(s.t_ns);
  copy3(s.attStd, p.att_std);
  copy3(s.velStd, p.vel_std);
  copy3(s.posStd, p.pos_std);
  return p;
}

Packets::Calibration makeCalibration(const TelemetrySnapshot &s) {
  Packets::Calibration p;
  p.t_us = timeUs(s.t_ns);
  p.raw_frame = Packets::ImuFrame::kSensor; // IMU_RAW carries the driver outputs, before calibration
  p.gyro_source = s.gyroCal.source;
  p.accel_source = s.accelCal.source;
  p.mag_source = s.magCal.source;
  for (int i = 0; i < 3; i++) {
    p.gyro_offset[i] = s.gyroCal.offset[i];
    p.accel_offset[i] = s.accelCal.offset[i];
    p.mag_offset[i] = s.magCal.offset[i] * 1e-4f; // gauss -> T, as IMU_RAW mag
    p.gyro_bias_est[i] = s.gyroBiasEst[i];
  }
  for (int i = 0; i < 9; i++) {
    p.gyro_matrix[i] = s.gyroCal.matrix[i];
    p.accel_matrix[i] = s.accelCal.matrix[i];
    p.mag_matrix[i] = s.magCal.matrix[i];
  }
  return p;
}

// ---- TEMPORARY legacy telecommand adapter ----

size_t LegacyTelecommandPacket::encode(uint8_t *out, size_t cap) const {
  if (out == nullptr || cap < kMaxSize) return 0;
  Packets::codec::put_u8(out + 0, static_cast<uint8_t>(tc.type));
  Packets::codec::put_u8(out + 1, static_cast<uint8_t>(tc.subsystem));
  Packets::codec::put_i16(out + 2, tc.paramInt);
  Packets::codec::put_f32(out + 4, tc.paramFloat);
  return kMaxSize;
}

bool LegacyTelecommandPacket::decode(const uint8_t *in, size_t len) {
  if (in == nullptr || len != kMaxSize) return false;
  tc.type = static_cast<TelecommandType>(Packets::codec::get_u8(in + 0));
  tc.subsystem = static_cast<Subsystem>(Packets::codec::get_u8(in + 1));
  tc.paramInt = Packets::codec::get_i16(in + 2);
  tc.paramFloat = Packets::codec::get_f32(in + 4);
  return true;
}

} // namespace Comms
} // namespace VCTR
