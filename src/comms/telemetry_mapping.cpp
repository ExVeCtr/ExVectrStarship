#include "telemetry_mapping.hpp"

#include <math.h>
#include <string.h>

#include "flashmem.hpp"

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
  if (f.controlSourceLost) b |= 1u << 5;
  if (f.interlockWatchdog) b |= 1u << 13;
  if (f.gsFailsafe) b |= 1u << 14;
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
  p.ctrl_source = s.ctrlSource == 1 ? Packets::ControlSource::kGs : Packets::ControlSource::kNone;
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
  p.rc_age = s.rcAgeS;
  p.home_epoch = s.homeEpoch;
  p.calib_state = packCalibState(s.calibGyroValid, s.calibAccelValid, s.calibMagValid, s.calibSessionActive,
                                 s.readyToArm);
  // T9b-4: ctrl_flags = control-source bits + INTERLOCK_NEED_TOGGLE (comms task) | the vehicle's command-path bits (cmdFlags)
  p.ctrl_flags = static_cast<uint16_t>(s.ctrlFlags | s.cmdFlags);
  p.go_refusal = s.goRefusal;
  p.degraded_bits = s.degradedBits;
  p.mission_profile = s.missionProfile;
  p.rc_rejected = s.rcRejected;
  p.cmd_session = s.cmdSession;
  return p;
}

Packets::CmdApplied makeCmdApplied(const TelemetrySnapshot &s) {
  Packets::CmdApplied p;
  p.t_us = timeUs(s.t_ns);
  const auto &a = s.cmdApplied;
  p.go = static_cast<Packets::CmdGo>(a.go);
  p.profile = a.profile;
  p.vehicle_mode = static_cast<Packets::CmdVehicleMode>(a.vehicleMode);
  p.motors = static_cast<Packets::CmdMotors>(a.motors);
  p.reset_button = static_cast<Packets::CmdButton>(a.resetButton);
  p.failsafe = static_cast<Packets::CmdFailsafe>(a.failsafe);
  p.flags = a.flags;
  p.safe_seen = a.safeSeen;
  p.session = a.session;
  p.age = a.ageS;
  p.usb_age = a.usbAgeS;
  p.uart_age = a.uartAgeS;
  for (int i = 0; i < 4; i++) p.rejects[i] = a.rejects[i];
  return p;
}

void fillCmdTelemetry(TelemetrySnapshot &s, const Safety::CmdIngressSnapshot &cs, const Safety::RcCommandMapper &m,
                      int64_t nowNs, bool interlockNeedToggle) {
  using Safety::CmdLink;
  using Safety::CommandIngress;
  const auto toS = [](int64_t a) { return a < 0 ? nanf_() : static_cast<float>(static_cast<double>(a) * 1e-9); };
  const bool fresh = cs.fresh(nowNs, CommandIngress::kCtrlTimeoutNs);
  s.ctrlSource = fresh ? 1 : 0;
  s.rcAgeS = toS(cs.ageNs(nowNs));
  s.cmdSession = static_cast<uint16_t>(cs.latched ? (cs.session & 0xFFFFu) : 0u);
  uint16_t f = 0;
  if (cs.linkFresh(CmdLink::Usb, nowNs, CommandIngress::kCtrlTimeoutNs)) f |= Packets::VehicleStatus::kCtrlFlagsUsbFresh;
  if (cs.linkFresh(CmdLink::Uart, nowNs, CommandIngress::kCtrlTimeoutNs)) f |= Packets::VehicleStatus::kCtrlFlagsUartFresh;
  if (cs.commitLatched) f |= Packets::VehicleStatus::kCtrlFlagsSourceLatched;
  if (cs.multiMaster(nowNs, CommandIngress::kMultiMasterShowNs)) f |= Packets::VehicleStatus::kCtrlFlagsMultiMaster;
  if (cs.latched && cs.seen && cs.frame.failsafe == 1) f |= Packets::VehicleStatus::kCtrlFlagsGsFailsafeReq;
  if (interlockNeedToggle) f |= Packets::VehicleStatus::kCtrlFlagsInterlockNeedToggle;
  s.ctrlFlags = f;
  s.rcRejected = cs.rejectedTotal;

  // Last accepted fields (safe values if none yet).
  auto &a = s.cmdApplied;
  const Safety::CmdFrame fr = (cs.latched && cs.seen) ? cs.frame : Safety::CmdFrame();
  a.go = fr.go;
  a.profile = fr.profile;
  a.vehicleMode = fr.vehicleMode;
  a.motors = fr.motors;
  a.resetButton = fr.resetButton;
  a.failsafe = fr.failsafe;
  a.flags = static_cast<uint8_t>(((fr.flags & Safety::CmdFrame::kFlagPocketLive) ? 1u : 0u) |
                                 ((fr.flags & Safety::CmdFrame::kFlagInputVirtual) ? 2u : 0u));
  a.safeSeen = m.safeSeen();
  a.session = cs.latched ? cs.session : 0;
  a.ageS = toS(cs.ageNs(nowNs));
  a.usbAgeS = toS(cs.linkAgeNs(CmdLink::Usb, nowNs));
  a.uartAgeS = toS(cs.linkAgeNs(CmdLink::Uart, nowNs));
  for (int i = 0; i < 4; i++) a.rejects[i] = cs.rejects[i];
}

uint8_t packCalibState(bool gyroValid, bool accelValid, bool magValid, bool sessionActive, bool readyToArm) {
  return static_cast<uint8_t>((gyroValid ? 1u : 0u) | (accelValid ? 2u : 0u) | (magValid ? 4u : 0u) |
                              (sessionActive ? 8u : 0u) | (readyToArm ? 16u : 0u));
}

Packets::AirGnss makeAirGnss(const TelemetrySnapshot &s) {
  Packets::AirGnss p;
  p.t_us = timeUs(s.t_ns);
  if (!s.gnssHaveData) {
    p.latitude = p.longitude = static_cast<double>(nanf_());
    p.altitude = nanf_();
    for (int i = 0; i < 3; i++) p.vel[i] = nanf_();
    return p;
  }
  p.latitude = s.latitude;
  p.longitude = s.longitude;
  p.altitude = s.altitude;
  copy3(s.gnssVel, p.vel);
  return p;
}

Packets::AirGnssInfo makeAirGnssInfo(const TelemetrySnapshot &s) {
  Packets::AirGnssInfo p;
  p.t_us = timeUs(s.t_ns);
  p.fix_type = s.gnssFixType;
  p.satellites = s.satellites;
  p.h_acc = s.gnssHaveData ? s.hAcc : nanf_();
  p.v_acc = s.gnssHaveData ? s.vAcc : nanf_();
  return p;
}

Packets::AirPose makeAirPose(const TelemetrySnapshot &s) {
  Packets::AirPose p;
  p.t_us = timeUs(s.t_ns);
  for (int i = 0; i < 4; i++) p.q[i] = s.att[3 + i];
  copy3(&s.pos[3], p.pos);
  copy3(&s.pos[0], p.vel);
  p.ground_distance = s.groundDistance;
  return p;
}

Packets::Pose makePose(const TelemetrySnapshot &s) {
  Packets::Pose p;
  p.t_us = timeUs(s.t_ns);
  copy3(&s.att[0], p.omega);
  for (int i = 0; i < 4; i++) p.q[i] = s.att[3 + i];
  copy3(&s.pos[0], p.vel);
  copy3(&s.pos[3], p.pos);
  p.ground_distance = s.groundDistance;
  return p;
}

Packets::Actuators makeActuators(const TelemetrySnapshot &s) {
  Packets::Actuators p;
  p.t_us = timeUs(s.t_ns);
  for (int i = 0; i < 4; i++) p.flaps[i] = s.flaps[i];
  p.roll_torque = s.tvcThrust[3]; // same source as AIR_ACTUATORS.roll_torque (makeActuatorSample)
  tvcDeflection(s.tvcThrust, p.tvc);
  p.thrust = sqrtf(s.tvcThrust[0] * s.tvcThrust[0] + s.tvcThrust[1] * s.tvcThrust[1] +
                   s.tvcThrust[2] * s.tvcThrust[2]);
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
  // Normalise here, not with sanitizeSetpointQuat: dq has norm sqrt(2 (1 + d)), which is below that function's 0.5
  // "invalid" threshold for corrections above about 151 deg (fixed 2026-11-21: zero setpoint_q in early freefall).
  const float n = sqrtf(dq[0] * dq[0] + dq[1] * dq[1] + dq[2] * dq[2] + dq[3] * dq[3]);
  if (!(n > 1e-12f) || !isfinite(n)) {
    zeroQ(out);
    return;
  }
  float dqn[4], r[4];
  for (int i = 0; i < 4; i++)
    dqn[i] = dq[i] / n;
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
    case AttitudeSetpointSource::Tvc: sanitizeSetpointQuat(in.tvcOverride ? in.tvcOverrideQ : in.tvcQ, out); break;
    case AttitudeSetpointSource::Flaps: flapPointingSetpoint(in.flapAzimuth, in.flapPitch, in.qIs, out); break;
    default: zeroQ(out); break;
  }
}

// Cold (5 and 20 Hz): runs from flash to keep the ITCM code below the 9 x 32 KB bank boundary.
FLASHMEM Packets::AirMission makeAirMission(const TelemetrySnapshot &s) {
  Packets::AirMission p;
  p.t_us = timeUs(s.t_ns);
  p.mission_time = static_cast<float>(static_cast<double>(s.missionTime_ns) * 1e-9);
  p.mission_elapsed = s.missionElapsedS;
  p.mission_mode = static_cast<uint8_t>(s.missionMode);
  p.phase = static_cast<uint8_t>(s.missionPhase);
  p.index = s.missionIndex;
  p.waypoint = s.waypointIndex;
  p.flip_start_altitude = s.flipStartAltitudeM;
  p.stop_altitude = s.stopAltitudeM;
  copy3(&s.setpoint[3], p.setpoint_pos);
  copy3(&s.setpoint[0], p.setpoint_vel);
  for (int i = 0; i < 4; i++) p.setpoint_q[i] = s.setpointQ[i];
  return p;
}

FLASHMEM Packets::Mission makeMission(const TelemetrySnapshot &s) {
  Packets::Mission p;
  p.t_us = timeUs(s.t_ns);
  copy3(&s.setpoint[3], p.setpoint_pos);
  copy3(&s.setpoint[0], p.setpoint_vel);
  p.mission_time = static_cast<float>(static_cast<double>(s.missionTime_ns) * 1e-9);
  p.index = s.missionIndex;
  p.phase = static_cast<uint8_t>(s.missionPhase);
  for (int i = 0; i < 4; i++) p.setpoint_q[i] = s.setpointQ[i];
  p.mission_elapsed = s.missionElapsedS;
  p.mission_mode = static_cast<uint8_t>(s.missionMode); // as AIR_MISSION
  p.waypoint = s.waypointIndex;                         // 255 = not the waypoint mission
  p.flip_start_altitude = s.flipStartAltitudeM;
  p.stop_altitude = s.stopAltitudeM;
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
  p.fix_type = s.gnssFixType; // real UBX-NAV-PVT fixType
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

Packets::MissionParams makeMissionParams(const TelemetrySnapshot &s) {
  Packets::MissionParams p;
  p.ascent_speed = s.missionParams.ascentSpeed;
  p.apogee_altitude = s.missionParams.apogeeAltitude;
  p.hover_hold_s = s.missionParams.hoverHoldS;
  p.p1_hold_1_s = s.missionParams.p1Hold1S;
  p.p1_hold_2_s = s.missionParams.p1Hold2S;
  p.p1_hold_3_s = s.missionParams.p1Hold3S;
  p.ascent_accel = s.missionParams.ascentAccel;
  p.ascent_decel = s.missionParams.ascentDecel;
  p.apogee_x = s.missionParams.apogeeX;
  p.apogee_y = s.missionParams.apogeeY;
  return p;
}

// ---- HOME_POSITION ----

Packets::HomePosition makeHomePosition(const HomeInput &in) {
  Packets::HomePosition p;
  const bool fix = in.latitude != 0.0 || in.longitude != 0.0;
  p.flags = 0;
  if (fix) p.flags |= Packets::HomePosition::kFlagsLatlonValid | Packets::HomePosition::kFlagsAltValid;
  if (in.simulation) p.flags |= Packets::HomePosition::kFlagsSimulation;
  if (in.zeroing) p.flags |= Packets::HomePosition::kFlagsZeroing;
  p.latitude = fix ? in.latitude : static_cast<double>(NAN);
  p.longitude = fix ? in.longitude : static_cast<double>(NAN);
  p.altitude = fix ? in.altitude : NAN;
  p.h_acc = NAN;
  p.epoch = in.epoch;
  p.set_time = in.setTimeNs;
  p.reserved = 0;
  return p;
}

} // namespace Comms
} // namespace VCTR

namespace VCTR {
namespace Comms {
FLASHMEM __attribute__((flatten)) size_t encodeMission(const TelemetrySnapshot &s, uint8_t *buf, size_t cap) {
  return makeMission(s).encode(buf, cap);
}
FLASHMEM __attribute__((flatten)) size_t encodeAirMission(const TelemetrySnapshot &s, uint8_t *buf, size_t cap) {
  return makeAirMission(s).encode(buf, cap);
}
} // namespace Comms
} // namespace VCTR
