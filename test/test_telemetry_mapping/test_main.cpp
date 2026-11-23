// Host tests of src/comms/telemetry_mapping.cpp: pio test -e native
// Checks the vehicle-state -> packet mapping, the flight-software owned bit fields, SI units through the
// generated encode/decode (round trip), saturation, and the HOME_POSITION builder.
#include <math.h>
#include <string.h>
#include <unity.h>

#include <initializer_list>

#include "comms/telemetry_mapping.hpp"
#include "safety/stop_altitude.hpp"

using namespace VCTR;
using namespace VCTR::Comms;

void setUp() {}
void tearDown() {}

template <class T>
T roundTrip(const T &in) {
  uint8_t buf[T::kMaxSize];
  size_t n = in.encode(buf, sizeof buf);
  TEST_ASSERT_EQUAL_UINT(T::kMaxSize, n);
  T out;
  TEST_ASSERT_TRUE(out.decode(buf, n));
  return out;
}

static TelemetrySnapshot sample() {
  TelemetrySnapshot s;
  s.t_ns = 5000123456789LL; // 5000 s
  s.mode = VehicleMode::VehicleMode_Running;
  s.missionMode = MissionMode_Running;
  s.missionPhase = MissionPhase_Hover;
  s.missionIndex = 1;
  s.armed = true;
  s.simulation = true;
  s.motorsEnabled = false;
  s.gsPresent = true;
  // 90 deg about +Z: q = [cos45, 0, 0, sin45]
  const float c = cosf(M_PI / 4), sn = sinf(M_PI / 4);
  float att[7] = {0.1f, -0.2f, 0.3f, c, 0, 0, sn};
  memcpy(s.att, att, sizeof att);
  float pos[6] = {1.0f, -2.0f, 0.5f, 10.0f, -20.0f, 3.5f};
  memcpy(s.pos, pos, sizeof pos);
  float sp[6] = {0, 0, 0.5f, 0, 0, 120.0f};
  memcpy(s.setpoint, sp, sizeof sp);
  s.missionTime_ns = -7500000000LL; // -7.5 s countdown
  float flaps[4] = {0.3f, 0.4f, 1.0f, 1.5f};
  memcpy(s.flaps, flaps, sizeof flaps);
  float tvc[4] = {1.0f, 0.0f, 10.0f, 0.2f};
  memcpy(s.tvcThrust, tvc, sizeof tvc);
  s.groundDistance = 0.8f;
  s.latitude = 49.945884 * M_PI / 180.0;
  s.longitude = 6.699575 * M_PI / 180.0;
  s.altitude = 324.7f;
  s.satellites = 9;
  s.gnssValid = true;
  s.gnssFixType = 3;
  s.gnssHaveData = true;
  s.hAcc = 1.5f;
  s.vAcc = 2.5f;
  s.sAcc = 0.2f;
  return s;
}

void test_failure_bits() {
  FailureState f = {};
  TEST_ASSERT_EQUAL_HEX32(0, packFailureBits(f));
  f.sensorFailure = true;
  TEST_ASSERT_EQUAL_HEX32(0x01, packFailureBits(f));
  f = {};
  f.controlSourceLost = true;
  TEST_ASSERT_EQUAL_HEX32(0x20, packFailureBits(f));
  f.positionOutOfBounds = true;
  f.attitudeOutOfBounds = true;
  TEST_ASSERT_EQUAL_HEX32(0x38, packFailureBits(f));
  f = {};
  f.interlockWatchdog = true; // T9b-2: bit 13 (motor-interlock watchdog)
  TEST_ASSERT_EQUAL_HEX32(0x2000, packFailureBits(f));
  f = {};
  f.gsFailsafe = true; // GS control: bit 14 (GS_FAILSAFE)
  TEST_ASSERT_EQUAL_HEX32(0x4000, packFailureBits(f));
}

void test_interlock_need_toggle_flag() {
  VCTR::Safety::CommandIngress t;
  VCTR::Safety::RcCommandMapper m;
  VCTR::Safety::CmdIngressSnapshot cs;
  t.snapshot(cs);
  TelemetrySnapshot s = sample();
  fillCmdTelemetry(s, cs, m, 5000000000LL, true);
  auto p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_HEX16(Packets::VehicleStatus::kCtrlFlagsInterlockNeedToggle, p.ctrl_flags);
  s = sample();
  fillCmdTelemetry(s, cs, m, 5000000000LL, false);
  p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_HEX16(0, p.ctrl_flags);
}

void test_sensor_state() {
  SensoryState s;
  s.gnss = TelemetrySensor_Failure;   // 3
  s.imu = TelemetrySensor_Ready;      // 2
  s.baro = TelemetrySensor_Calib;     // 1
  s.mag = TelemetrySensor_Init;       // 0
  s.attitudeKF = TelemetrySensor_Ready;
  s.positionKF = TelemetrySensor_Failure;
  // 3 | 2<<2 | 1<<4 | 0<<6 | 2<<8 | 3<<10
  TEST_ASSERT_EQUAL_HEX16(0x3 | 0x8 | 0x10 | 0x200 | 0xC00, packSensorState(s));
}

void test_time_us_wraps() {
  TEST_ASSERT_EQUAL_UINT32(0, timeUs(0));
  TEST_ASSERT_EQUAL_UINT32(1000, timeUs(1000000));
  // 2^32 us + 5 us wraps to 5
  TEST_ASSERT_EQUAL_UINT32(5, timeUs((4294967296LL + 5) * 1000));
}

void test_vehicle_status() {
  TelemetrySnapshot s = sample();
  s.failures.controlSourceLost = true;
  auto p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_INT64(s.t_ns, p.t_ns);
  TEST_ASSERT_EQUAL_UINT32(timeUs(s.t_ns), p.t_us);
  TEST_ASSERT_EQUAL_UINT8(4, p.mode); // Running
  TEST_ASSERT_EQUAL_UINT8(3, p.mission_mode);
  TEST_ASSERT_EQUAL_UINT8(1, p.mission_phase);
  TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(p.ctrl_source));
  TEST_ASSERT_EQUAL_HEX8(Packets::VehicleStatus::kFlagsArmed | Packets::VehicleStatus::kFlagsSimulation |
                             Packets::VehicleStatus::kFlagsGsPresent,
                         p.flags);
  TEST_ASSERT_EQUAL_HEX32(0x20, p.failure_bits);
  TEST_ASSERT_TRUE(isnan(p.rc_age));
  s.mode = VehicleMode::VehicleMode_Failure;
  p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_TRUE(p.flags & Packets::VehicleStatus::kFlagsFailsafe);
  TEST_ASSERT_FALSE(p.flags & Packets::VehicleStatus::kFlagsUartPeerSeen);
  s.uartPeerSeen = true;
  p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_HEX8(0x40, p.flags & Packets::VehicleStatus::kFlagsUartPeerSeen);
  TEST_ASSERT_FALSE(p.flags & Packets::VehicleStatus::kFlagsZeroing);
  s.zeroing = true;
  p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_HEX8(0x80, p.flags & Packets::VehicleStatus::kFlagsZeroing);
  TEST_ASSERT_EQUAL_HEX8(0x80, Packets::VehicleStatus::kFlagsZeroing);
}

static TelemetrySnapshot actuatorSnap(int64_t t_ns, float base) {
  TelemetrySnapshot s;
  s.t_ns = t_ns;
  // thrust 10 N deflected: Fx = 10 sin(a), Fz = 10 cos(a)
  const float a = 0.05f + base;
  s.tvcThrust[0] = 10.0f * sinf(a);
  s.tvcThrust[1] = 0.0f;
  s.tvcThrust[2] = 10.0f * cosf(a);
  s.tvcThrust[3] = 0.012f + base;
  for (int i = 0; i < 4; i++) s.flaps[i] = 0.3f + 0.1f * i + base;
  return s;
}

void test_air_actuators_two_samples() {
  ActuatorHistory h;
  // empty history: everything NaN
  auto e = roundTrip(makeAirActuators(h));
  TEST_ASSERT_TRUE(isnan(e.prev_age));
  TEST_ASSERT_TRUE(isnan(e.tvc[0]) && isnan(e.flaps[7]) && isnan(e.thrust[0]));

  // one sample: previous is NaN
  h.push(makeActuatorSample(actuatorSnap(1000000000LL, 0.0f)));
  auto one = roundTrip(makeAirActuators(h));
  TEST_ASSERT_EQUAL_UINT32(1000000u, one.t_us);
  TEST_ASSERT_TRUE(isnan(one.prev_age));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.05f, one.tvc[0]);
  TEST_ASSERT_TRUE(isnan(one.tvc[2]) && isnan(one.tvc[3]) && isnan(one.roll_torque[1]) && isnan(one.thrust[1]));
  for (int i = 4; i < 8; i++) TEST_ASSERT_TRUE(isnan(one.flaps[i]));

  // ring: after more pushes the packet holds exactly the newest and the one 20 ms before it
  h.push(makeActuatorSample(actuatorSnap(1020000000LL, 0.01f)));
  h.push(makeActuatorSample(actuatorSnap(1040000000LL, 0.02f)));
  h.push(makeActuatorSample(actuatorSnap(1060000000LL, 0.03f)));
  h.push(makeActuatorSample(actuatorSnap(1080500000LL, 0.04f))); // 20.5 ms later (wraps the 4-entry ring)
  TEST_ASSERT_EQUAL_UINT(ActuatorHistory::kSize, h.count());
  auto p = roundTrip(makeAirActuators(h));
  TEST_ASSERT_EQUAL_UINT32(1080500u, p.t_us);
  TEST_ASSERT_FLOAT_WITHIN(1.5e-6f, 0.0205f, p.prev_age);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.09f, p.tvc[0]); // newest x
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, p.tvc[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.08f, p.tvc[2]); // previous x
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.052f, p.roll_torque[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.042f, p.roll_torque[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, p.thrust[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, p.thrust[1]);
  for (int i = 0; i < 4; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f + 0.1f * i + 0.04f, p.flaps[i]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f + 0.1f * i + 0.03f, p.flaps[4 + i]);
  }
  // same definitions as ACTUATORS (tvc deflection and thrust magnitude)
  auto u = makeActuators(actuatorSnap(1080500000LL, 0.04f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, u.tvc[0], p.tvc[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, u.thrust, p.thrust[0]);
}

void test_air_actuators_saturation_and_nan() {
  ActuatorHistory h;
  TelemetrySnapshot s;
  s.t_ns = 5;
  s.tvcThrust[0] = 0;
  s.tvcThrust[1] = 0;
  s.tvcThrust[2] = 100.0f;       // thrust beyond 65.534 N saturates
  s.tvcThrust[3] = 1.0f;         // roll torque beyond 0.32767 N*m saturates
  s.flaps[0] = 4.0f;             // beyond +-3.2767 rad saturates
  s.flaps[1] = -4.0f;
  s.flaps[2] = NAN;              // NaN stays NaN (sentinel)
  s.flaps[3] = 1.5708f;
  h.push(makeActuatorSample(s));
  s.t_ns = 20000005;
  h.push(makeActuatorSample(s));
  auto p = roundTrip(makeAirActuators(h));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, p.prev_age);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 65.534f, p.thrust[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.32767f, p.roll_torque[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.2767f, p.flaps[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -3.2767f, p.flaps[1]);
  TEST_ASSERT_TRUE(isnan(p.flaps[2]));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.5708f, p.flaps[3]);
  TEST_ASSERT_TRUE(isnan(p.flaps[6]));
  // zero thrust: deflection 0, not NaN
  TelemetrySnapshot z;
  ActuatorHistory hz;
  hz.push(makeActuatorSample(z));
  auto pz = roundTrip(makeAirActuators(hz));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pz.tvc[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pz.thrust[0]);
}

void test_air_mission() {
  TelemetrySnapshot s = sample();
  s.waypointIndex = 2;
  auto p = roundTrip(makeAirMission(s));
  TEST_ASSERT_EQUAL_UINT32(timeUs(s.t_ns), p.t_us);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -7.5f, p.mission_time);
  TEST_ASSERT_EQUAL_UINT8(3, p.mission_mode); // Running
  TEST_ASSERT_EQUAL_UINT8(1, p.phase);        // Hover
  TEST_ASSERT_EQUAL_UINT8(1, p.index);
  TEST_ASSERT_EQUAL_UINT8(2, p.waypoint);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, p.setpoint_pos[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 120.0f, p.setpoint_pos[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.5f, p.setpoint_vel[2]);
  s.waypointIndex = 0xFF;
  s.setpoint[0] = 400.0f; // velocity beyond +-327.67 m/s saturates
  p = roundTrip(makeAirMission(s));
  TEST_ASSERT_EQUAL_UINT8(255, p.waypoint);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 327.67f, p.setpoint_vel[0]);
}

void test_mission_flip_start_and_stop_altitude() {
  TelemetrySnapshot s = sample();
  // Not in the freefall phase: flip start NaN (sentinel), stop altitude echo always present (default 20 m).
  s.flipStartAltitudeM = Safety::flipStartAltitude(false, 96.0f);
  auto am = roundTrip(makeAirMission(s));
  auto m = roundTrip(makeMission(s));
  TEST_ASSERT_TRUE(isnan(am.flip_start_altitude));
  TEST_ASSERT_TRUE(isnan(m.flip_start_altitude));
  TEST_ASSERT_FLOAT_WITHIN(0.06f, 20.0f, am.stop_altitude);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, m.stop_altitude);
  // Freefall phase active: the trigger threshold, 0.1 m resolution on air, exact on USB.
  s.stopAltitudeM = 35.0f;
  s.flipStartAltitudeM = Safety::flipStartAltitude(true, 96.37f);
  am = roundTrip(makeAirMission(s));
  m = roundTrip(makeMission(s));
  TEST_ASSERT_FLOAT_WITHIN(0.06f, 96.4f, am.flip_start_altitude);
  TEST_ASSERT_EQUAL_FLOAT(96.37f, m.flip_start_altitude);
  TEST_ASSERT_FLOAT_WITHIN(0.06f, 35.0f, am.stop_altitude);
  TEST_ASSERT_EQUAL_FLOAT(35.0f, m.stop_altitude);
}

void test_pose_full_precision() {
  TelemetrySnapshot s = sample();
  auto p = roundTrip(makePose(s));
  TEST_ASSERT_EQUAL_FLOAT(0.1f, p.omega[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.3f, p.omega[2]);
  TEST_ASSERT_EQUAL_FLOAT(s.att[3], p.q[0]);
  TEST_ASSERT_EQUAL_FLOAT(s.att[6], p.q[3]);
  // position state is [V, P]: vel from the first three, pos from the last three
  TEST_ASSERT_EQUAL_FLOAT(1.0f, p.vel[0]);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, p.pos[0]);
  TEST_ASSERT_EQUAL_FLOAT(3.5f, p.pos[2]);
}

void test_air_pose_fixed_point_and_saturation() {
  TelemetrySnapshot s = sample();
  auto p = roundTrip(makeAirPose(s));
  TEST_ASSERT_FLOAT_WITHIN(1.0f / 32767, s.att[3], p.q[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.0006f, 10.0f, p.pos[0]); // 1 mm resolution (i32, 0.001 m)
  TEST_ASSERT_FLOAT_WITHIN(0.0006f, -20.0f, p.pos[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, -2.0f, p.vel[1]);
  // Altitude resolves to the millimetre (the old i16 0.1 m field would have given 12.3)
  s.pos[5] = 12.3456f;
  p = roundTrip(makeAirPose(s));
  TEST_ASSERT_FLOAT_WITHIN(0.0006f, 12.346f, p.pos[2]);
  TEST_ASSERT_TRUE(fabsf(p.pos[2] - 12.3f) > 0.04f);
  // Saturates instead of overflowing (old int16 telemetry was undefined behaviour out of range)
  s.pos[3] = 1e10f;
  s.pos[4] = -1e10f;
  s.pos[0] = -1e6f;
  p = roundTrip(makeAirPose(s));
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 2147483.647f, p.pos[0]);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -2147483.647f, p.pos[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -327.67f, p.vel[0]);
  // NaN becomes the invalid sentinel and decodes as NaN
  s.pos[4] = NAN;
  p = roundTrip(makeAirPose(s));
  TEST_ASSERT_TRUE(isnan(p.pos[1]));
}

void test_actuators() {
  TelemetrySnapshot s = sample();
  auto p = roundTrip(makeActuators(s));
  TEST_ASSERT_EQUAL_FLOAT(0.3f, p.flaps[0]);
  TEST_ASSERT_EQUAL_FLOAT(1.5f, p.flaps[3]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, atan2f(1.0f, 10.0f), p.tvc[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, p.tvc[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, sqrtf(101.0f), p.thrust);
  TEST_ASSERT_EQUAL_FLOAT(0.8f, p.ground_distance);
  // Above 2 m the ground distance is still correct (old telemetry overflowed int16 above 2 m)
  s.groundDistance = 8.19f;
  p = roundTrip(makeActuators(s));
  TEST_ASSERT_EQUAL_FLOAT(8.19f, p.ground_distance);
  // Zero thrust: no NaN
  float zero[4] = {0, 0, 0, 0};
  memcpy(s.tvcThrust, zero, sizeof zero);
  p = roundTrip(makeActuators(s));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, p.tvc[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, p.thrust);
}

void test_mission_time_seconds() {
  TelemetrySnapshot s = sample();
  auto p = roundTrip(makeMission(s));
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, -7.5f, p.mission_time);
  TEST_ASSERT_EQUAL_FLOAT(120.0f, p.setpoint_pos[2]);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, p.setpoint_vel[2]);
  TEST_ASSERT_EQUAL_UINT8(1, p.index);
  TEST_ASSERT_EQUAL_UINT8(1, p.phase);
}

void test_gnss_radians() {
  TelemetrySnapshot s = sample();
  auto p = roundTrip(makeGnss(s));
  TEST_ASSERT_EQUAL_DOUBLE(s.latitude, p.latitude);
  TEST_ASSERT_EQUAL_DOUBLE(s.longitude, p.longitude);
  TEST_ASSERT_EQUAL_UINT8(3, p.fix_type);
  TEST_ASSERT_EQUAL_FLOAT(1.5f, p.h_acc);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, p.v_acc);
  s.gnssValid = false;
  s.gnssFixType = 2; // the REAL u-blox fix type is sent, not a 0/3 derived from validity
  TEST_ASSERT_EQUAL_UINT8(2, roundTrip(makeGnss(s)).fix_type);
}

void test_air_gnss() {
  TelemetrySnapshot s = sample();
  s.gnssVel[0] = 1.234f; // north
  s.gnssVel[1] = -2.5f;  // west
  s.gnssVel[2] = -40.0f; // up
  TEST_ASSERT_EQUAL_UINT(22, Packets::AirGnss::kMaxSize);
  TEST_ASSERT_EQUAL_HEX8(0x24, Packets::AirGnss::kPort);
  auto p = roundTrip(makeAirGnss(s));
  TEST_ASSERT_EQUAL_UINT32(timeUs(s.t_ns), p.t_us);
  // 1e-7 deg steps (1.745e-9 rad), altitude 1 mm, velocity 0.01 m/s
  TEST_ASSERT_DOUBLE_WITHIN(1.8e-9, s.latitude, p.latitude);
  TEST_ASSERT_DOUBLE_WITHIN(1.8e-9, s.longitude, p.longitude);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 324.7f, p.altitude);
  TEST_ASSERT_FLOAT_WITHIN(0.006f, 1.234f, p.vel[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.006f, -2.5f, p.vel[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.006f, -40.0f, p.vel[2]);
  // Saturation of the velocity (+-327.66 m/s), exact wire bytes of a known value
  s.gnssVel[0] = 1000.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 327.67f, roundTrip(makeAirGnss(s)).vel[0]);
  uint8_t buf[22];
  s.gnssVel[0] = 1.0f;
  s.latitude = 50.0 * M_PI / 180.0;
  makeAirGnss(s).encode(buf, sizeof buf);
  const int32_t lat = int32_t(uint32_t(buf[4]) | (uint32_t(buf[5]) << 8) | (uint32_t(buf[6]) << 16) | (uint32_t(buf[7]) << 24));
  TEST_ASSERT_EQUAL_INT32(500000000, lat); // 50 deg * 1e7
  TEST_ASSERT_EQUAL_INT16(100, int16_t(uint16_t(buf[16]) | (uint16_t(buf[17]) << 8)));
  // No GNSS measurement yet: NaN sentinels, all values decode as NaN
  s.gnssHaveData = false;
  p = roundTrip(makeAirGnss(s));
  TEST_ASSERT_TRUE(isnan(p.latitude));
  TEST_ASSERT_TRUE(isnan(p.longitude));
  TEST_ASSERT_TRUE(isnan(p.altitude));
  for (int i = 0; i < 3; i++) TEST_ASSERT_TRUE(isnan(p.vel[i]));
}

void test_air_gnss_info() {
  TelemetrySnapshot s = sample();
  s.gnssFixType = 3;
  s.satellites = 17;
  TEST_ASSERT_EQUAL_UINT(10, Packets::AirGnssInfo::kMaxSize);
  TEST_ASSERT_EQUAL_HEX8(0x25, Packets::AirGnssInfo::kPort);
  auto p = roundTrip(makeAirGnssInfo(s));
  TEST_ASSERT_EQUAL_UINT8(3, p.fix_type);
  TEST_ASSERT_EQUAL_UINT8(17, p.satellites);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.5f, p.h_acc); // 1 cm steps
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 2.5f, p.v_acc);
  // Accuracy beyond the range saturates, NaN = unknown (65535)
  s.hAcc = 1000.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 655.34f, roundTrip(makeAirGnssInfo(s)).h_acc);
  s.gnssHaveData = false;
  s.gnssFixType = 0;
  p = roundTrip(makeAirGnssInfo(s));
  TEST_ASSERT_EQUAL_UINT8(0, p.fix_type);
  TEST_ASSERT_TRUE(isnan(p.h_acc));
  TEST_ASSERT_TRUE(isnan(p.v_acc));
  uint8_t buf[10];
  makeAirGnssInfo(s).encode(buf, sizeof buf);
  TEST_ASSERT_EQUAL_HEX8(0xFF, buf[6]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, buf[7]);
}

void test_pack_calib_state() {
  TEST_ASSERT_EQUAL_HEX8(0x00, packCalibState(false, false, false, false, false));
  TEST_ASSERT_EQUAL_HEX8(0x01, packCalibState(true, false, false, false, false));
  TEST_ASSERT_EQUAL_HEX8(0x02, packCalibState(false, true, false, false, false));
  TEST_ASSERT_EQUAL_HEX8(0x04, packCalibState(false, false, true, false, false));
  TEST_ASSERT_EQUAL_HEX8(0x08, packCalibState(false, false, false, true, false));
  TEST_ASSERT_EQUAL_HEX8(0x10, packCalibState(false, false, false, false, true));
  TEST_ASSERT_EQUAL_HEX8(0x07, packCalibState(true, true, true, false, false));
  TEST_ASSERT_EQUAL_HEX8(0x17, packCalibState(true, true, true, false, true));  // all valid and ready
  TEST_ASSERT_EQUAL_HEX8(0x0F, packCalibState(true, true, true, true, false));  // session active, not ready
  TEST_ASSERT_EQUAL_HEX8(0x1F, packCalibState(true, true, true, true, true));
  TEST_ASSERT_EQUAL_HEX8(0x05, packCalibState(true, false, true, false, false)); // accel invalid
  // bits 5-7 are always zero
  TEST_ASSERT_EQUAL_HEX8(0, packCalibState(true, true, true, true, true) & 0xE0);
}

void test_vehicle_status_home_and_calib() {
  TelemetrySnapshot s = sample();
  auto p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_UINT(38, Packets::VehicleStatus::kMaxSize); // +2: cmd_session (GS control)
  TEST_ASSERT_EQUAL_UINT8(0, p.home_epoch); // no position reference yet
  TEST_ASSERT_EQUAL_HEX8(0, p.calib_state);
  s.homeEpoch = 7;
  s.calibGyroValid = s.calibAccelValid = true;
  s.calibMagValid = false;
  s.calibSessionActive = true;
  p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_UINT8(7, p.home_epoch);
  TEST_ASSERT_EQUAL_HEX8(0x0B, p.calib_state);
  s.calibMagValid = true;
  s.calibSessionActive = false;
  s.readyToArm = true;
  p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_HEX8(0x17, p.calib_state);
}

void test_cmd_telemetry_fields_and_cmd_applied() {
  using namespace VCTR::Safety;
  CommandIngress t;
  RcCommandMapper m;
  const int64_t S = 1000000000LL;
  CmdFrame f;
  f.flags = CmdFrame::kFlagPocketLive | CmdFrame::kFlagInputVirtual;
  f.session = 0xDEADBEEFu;
  f.go = 1;
  f.profile = 2;
  f.vehicleMode = 1;
  f.motors = 1;
  f.resetButton = 0;
  f.failsafe = 1;
  m.onSessionChanged();
  const auto r = t.onFrame(0, CmdLink::Usb, 1, f, 10 * S);
  TEST_ASSERT_TRUE(r.accepted);
  m.onAccepted(f, 10 * S);
  t.onFrame(0, CmdLink::Uart, 1, f, 10 * S + 5000000LL); // duplicate copy on the other link
  t.onFrame(1, CmdLink::Usb, 1, f, 10 * S);               // one reject (source)
  t.update(10 * S, true, false);
  TelemetrySnapshot s = sample();
  CmdIngressSnapshot cs;
  TEST_ASSERT_TRUE(t.snapshot(cs));
  fillCmdTelemetry(s, cs, m, 10 * S + 250000000LL);
  auto p = roundTrip(makeVehicleStatus(s));
  TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(p.ctrl_source)); // GS
  TEST_ASSERT_EQUAL_UINT16(0xBEEF, p.cmd_session);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.25f, p.rc_age);
  const uint16_t want = Packets::VehicleStatus::kCtrlFlagsUsbFresh | Packets::VehicleStatus::kCtrlFlagsUartFresh |
                        Packets::VehicleStatus::kCtrlFlagsSourceLatched | Packets::VehicleStatus::kCtrlFlagsGsFailsafeReq;
  TEST_ASSERT_EQUAL_HEX16(want, p.ctrl_flags);
  TEST_ASSERT_EQUAL_UINT8(1, p.rc_rejected); // the source reject only; the duplicate copy is not counted
  auto a = roundTrip(makeCmdApplied(s));
  TEST_ASSERT_EQUAL_UINT(26, Packets::CmdApplied::kMaxSize);
  TEST_ASSERT_EQUAL_UINT8(0x84, Packets::CmdApplied::kPort);
  TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(a.go));
  TEST_ASSERT_EQUAL_UINT8(2, a.profile);
  TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(a.vehicle_mode));
  TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(a.motors));
  TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(a.reset_button));
  TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(a.failsafe));
  TEST_ASSERT_EQUAL_HEX8(Packets::CmdApplied::kFlagsPocketLive | Packets::CmdApplied::kFlagsInputVirtual, a.flags);
  TEST_ASSERT_EQUAL_HEX8(kSafeSeenSePress, a.safe_seen);
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu, a.session);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.25f, a.age);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.25f, a.usb_age);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.245f, a.uart_age);
  TEST_ASSERT_EQUAL_UINT8(1, a.rejects[1]);
  TEST_ASSERT_EQUAL_UINT8(0, a.rejects[3]); // the duplicate copy is not a reject
  // no session: safe values, NaN ages, ctrl_source NONE, cmd_session 0
  CommandIngress t2;
  CmdIngressSnapshot c2;
  t2.snapshot(c2);
  RcCommandMapper m2;
  TelemetrySnapshot s2 = sample();
  fillCmdTelemetry(s2, c2, m2, 5 * S);
  p = roundTrip(makeVehicleStatus(s2));
  a = roundTrip(makeCmdApplied(s2));
  TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(p.ctrl_source));
  TEST_ASSERT_TRUE(isnan(p.rc_age));
  TEST_ASSERT_EQUAL_UINT16(0, p.cmd_session);
  TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(a.go));
  TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(a.motors));
  TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(a.vehicle_mode)); // REAL
  TEST_ASSERT_EQUAL_UINT32(0, a.session);
  TEST_ASSERT_TRUE(isnan(a.age) && isnan(a.usb_age) && isnan(a.uart_age));
}

void test_vehicle_status_t9b_none_values() {
  auto p = roundTrip(makeVehicleStatus(sample()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Packets::ControlSource::kNone), static_cast<uint8_t>(p.ctrl_source));
  TEST_ASSERT_TRUE(isnan(p.rc_age));
  TEST_ASSERT_EQUAL_HEX16(0, p.ctrl_flags);
  TEST_ASSERT_EQUAL_UINT8(0, p.go_refusal);
  TEST_ASSERT_EQUAL_HEX8(0, p.degraded_bits);
  TEST_ASSERT_EQUAL_UINT8(255, p.mission_profile);
  TEST_ASSERT_EQUAL_UINT8(0, p.rc_rejected);
  TEST_ASSERT_EQUAL_UINT8(12, static_cast<uint8_t>(Packets::RequestAction::kZeroEstimators));
}

void test_vehicle_status_t9b4_command_path_fields() {
  TelemetrySnapshot s = sample();
  s.ctrlFlags = Packets::VehicleStatus::kCtrlFlagsUsbFresh | Packets::VehicleStatus::kCtrlFlagsInterlockNeedToggle;
  s.cmdFlags = Packets::VehicleStatus::kCtrlFlagsScMismatch | Packets::VehicleStatus::kCtrlFlagsSeRefused |
               Packets::VehicleStatus::kCtrlFlagsSeHoldZeroing | Packets::VehicleStatus::kCtrlFlagsFailsafeLatched;
  s.goRefusal = 4;
  s.missionProfile = 2;
  auto p = roundTrip(makeVehicleStatus(s));
  // the comms task's bits and the vehicle's bits are both carried
  const uint16_t want = Packets::VehicleStatus::kCtrlFlagsUsbFresh | Packets::VehicleStatus::kCtrlFlagsInterlockNeedToggle |
                        Packets::VehicleStatus::kCtrlFlagsScMismatch | Packets::VehicleStatus::kCtrlFlagsSeRefused |
                        Packets::VehicleStatus::kCtrlFlagsSeHoldZeroing | Packets::VehicleStatus::kCtrlFlagsFailsafeLatched;
  TEST_ASSERT_EQUAL_HEX16(want, p.ctrl_flags);
  TEST_ASSERT_EQUAL_UINT8(4, p.go_refusal);
  TEST_ASSERT_EQUAL_UINT8(2, p.mission_profile);
}

void test_home_epoch_counter() {
  HomeEpoch h;
  TEST_ASSERT_EQUAL_UINT8(0, h.value());
  h.referenceSet();
  TEST_ASSERT_EQUAL_UINT8(1, h.value());
  h.referenceSet();
  TEST_ASSERT_EQUAL_UINT8(2, h.value());
  for (int i = 0; i < 253; i++) h.referenceSet();
  TEST_ASSERT_EQUAL_UINT8(255, h.value());
  h.referenceSet(); // wraps to 1, never back to 0 = "no home"
  TEST_ASSERT_EQUAL_UINT8(1, h.value());
}

void test_actuators_roll_torque_and_mission_fields() {
  TelemetrySnapshot s = sample(); // tvcThrust[3] = 0.2
  auto a = roundTrip(makeActuators(s));
  TEST_ASSERT_EQUAL_UINT(40, Packets::Actuators::kMaxSize);
  TEST_ASSERT_EQUAL_FLOAT(0.2f, a.roll_torque); // unquantised f32
  // Same source as AIR_ACTUATORS.roll_torque
  TEST_ASSERT_EQUAL_FLOAT(makeActuatorSample(s).rollTorque, a.roll_torque);
  ActuatorHistory h;
  h.push(makeActuatorSample(s));
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, a.roll_torque, makeAirActuators(h).roll_torque[0]);
  // MISSION.mission_mode / waypoint exactly as AIR_MISSION
  TEST_ASSERT_EQUAL_UINT(60, Packets::Mission::kMaxSize);
  s.waypointIndex = 2;
  auto m = roundTrip(makeMission(s));
  auto am = roundTrip(makeAirMission(s));
  TEST_ASSERT_EQUAL_UINT8(3, m.mission_mode); // Running
  TEST_ASSERT_EQUAL_UINT8(2, m.waypoint);
  TEST_ASSERT_EQUAL_UINT8(am.mission_mode, m.mission_mode);
  TEST_ASSERT_EQUAL_UINT8(am.waypoint, m.waypoint);
  s.waypointIndex = 0xFF; // another mission is active
  TEST_ASSERT_EQUAL_UINT8(255, roundTrip(makeMission(s)).waypoint);
}

void test_handset_telem() {
  TelemetrySnapshot s = sample();
  auto p = roundTrip(makeHandsetTelem(s));
  TEST_ASSERT_TRUE(p.flags & Packets::HandsetTelem::kFlagsArmed);
  TEST_ASSERT_TRUE(p.flags & Packets::HandsetTelem::kFlagsSimulation);
  TEST_ASSERT_TRUE(p.flags & Packets::HandsetTelem::kFlagsGnssFix);
  TEST_ASSERT_FALSE(p.flags & Packets::HandsetTelem::kFlagsFailure);
  TEST_ASSERT_EQUAL_MEMORY("HOVR S\0\0", p.mode_text, 8);
  // deg*1e7 on the wire: about 1e-7 deg resolution
  TEST_ASSERT_DOUBLE_WITHIN(2e-9, s.latitude, p.latitude);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, sqrtf(5.0f), p.ground_speed);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 4.0f, p.altitude); // 3.5 m rounds to 4 (1 m resolution)
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, p.vertical_speed);
  // 90 deg about +Z
  TEST_ASSERT_FLOAT_WITHIN(0.001f, M_PI / 2, p.yaw);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, p.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, p.pitch);
  TEST_ASSERT_TRUE(isnan(p.battery)); // 0 = unknown on the wire
}

void test_course_over_ground() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, courseOverGround(1, 0));           // north
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, M_PI / 2, courseOverGround(0, -1));      // east (west negative)
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, M_PI, courseOverGround(-1, 0));          // south
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 3 * M_PI / 2, courseOverGround(0, 1));   // west
}

void test_mode_text() {
  char t[8];
  modeText(VehicleMode::VehicleMode_Ready, MissionPhase_Idle, false, false, t);
  TEST_ASSERT_EQUAL_MEMORY("RDY\0\0\0\0\0", t, 8);
  modeText(VehicleMode::VehicleMode_Ready, MissionPhase_Idle, true, false, t);
  TEST_ASSERT_EQUAL_MEMORY("ARMD\0\0\0\0", t, 8);
  modeText(VehicleMode::VehicleMode_Failure, MissionPhase_Hover, true, true, t);
  TEST_ASSERT_EQUAL_MEMORY("FAIL S\0\0", t, 8);
  modeText(VehicleMode::VehicleMode_SensorCalib, MissionPhase_Idle, false, true, t);
  TEST_ASSERT_EQUAL_MEMORY("CALIB S\0", t, 8);
}

void test_imu_raw_and_estimator() {
  TelemetrySnapshot s = sample();
  s.mag[0] = 50e-6f;
  s.temperature = 298.15f;
  s.pressure = 101325.0f;
  s.attStd[2] = 0.05f;
  s.posStd[0] = 1.2f;
  auto r = roundTrip(makeImuRaw(s));
  TEST_ASSERT_EQUAL_FLOAT(50e-6f, r.mag[0]);
  TEST_ASSERT_EQUAL_FLOAT(298.15f, r.temperature);
  TEST_ASSERT_EQUAL_FLOAT(101325.0f, r.pressure);
  auto e = roundTrip(makeEstimator(s));
  TEST_ASSERT_EQUAL_FLOAT(0.05f, e.att_std[2]);
  TEST_ASSERT_EQUAL_FLOAT(1.2f, e.pos_std[0]);
}

void test_calibration() {
  TelemetrySnapshot s = sample();
  // Gyro: hard-coded bias override, diag(-1,1,-1) transform (as the if(false) block in main.cpp would store)
  s.gyroCal.offset[0] = 0.016f;
  s.gyroCal.offset[1] = 0.035f;
  s.gyroCal.offset[2] = 0.0105f;
  const float flip[9] = {-1, 0, 0, 0, 1, 0, 0, 0, -1};
  memcpy(s.gyroCal.matrix, flip, sizeof flip);
  s.gyroCal.source = Packets::Calibration::kGyroSourceCalibEeprom | Packets::Calibration::kGyroSourceOffsetOverride;
  // Accel: non-symmetric matrix to catch a transposed (column-major) layout
  const float a[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  memcpy(s.accelCal.matrix, a, sizeof a);
  s.accelCal.offset[2] = 1.5f;
  s.accelCal.source = Packets::Calibration::kAccelSourceTransformEeprom;
  // Mag: offset in gauss (vehicle unit), scale 0.05/0.47
  s.magCal.offset[0] = 0.12f;
  s.magCal.offset[1] = -0.3f;
  s.magCal.offset[2] = 0.05f;
  for (int i = 0; i < 9; i++) s.magCal.matrix[i] = (i % 4 == 0) ? 0.1064f : 0.0f;
  s.magCal.source = Packets::Calibration::kMagSourceCalibEeprom | Packets::Calibration::kMagSourceRamOnly;
  s.gyroBiasEst[1] = -0.002f;

  auto c = roundTrip(makeCalibration(s));
  TEST_ASSERT_EQUAL_UINT32(timeUs(s.t_ns), c.t_us);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Packets::ImuFrame::kSensor), static_cast<uint8_t>(c.raw_frame));
  TEST_ASSERT_EQUAL_HEX8(0x05, c.gyro_source);
  TEST_ASSERT_EQUAL_HEX8(0x02, c.accel_source);
  TEST_ASSERT_EQUAL_HEX8(0x09, c.mag_source);
  TEST_ASSERT_EQUAL_FLOAT(0.035f, c.gyro_offset[1]);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, c.gyro_matrix[0]);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, c.gyro_matrix[8]);
  for (int i = 0; i < 9; i++) TEST_ASSERT_EQUAL_FLOAT(a[i], c.accel_matrix[i]); // row-major, unchanged
  TEST_ASSERT_EQUAL_FLOAT(1.5f, c.accel_offset[2]);
  // Mag offset gauss -> T; matrix unit-free (unchanged)
  TEST_ASSERT_EQUAL_FLOAT(0.12e-4f, c.mag_offset[0]);
  TEST_ASSERT_EQUAL_FLOAT(-0.3e-4f, c.mag_offset[1]);
  TEST_ASSERT_EQUAL_FLOAT(0.1064f, c.mag_matrix[4]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.mag_matrix[1]);
  TEST_ASSERT_EQUAL_FLOAT(-0.002f, c.gyro_bias_est[1]);

  // The documented ground-station formula cal = M * (raw - offset) on IMU_RAW (SI) equals the vehicle's value
  // M * (raw_internal - offset_internal), with the mag in T instead of gauss.
  s.mag[0] = 0.4e-4f; // T, as IMU_RAW
  s.mag[1] = -0.1e-4f;
  s.mag[2] = 0.3e-4f;
  auto r = roundTrip(makeImuRaw(s));
  for (int row = 0; row < 3; row++) {
    float gs = 0, ship = 0;
    for (int col = 0; col < 3; col++) {
      gs += c.mag_matrix[3 * row + col] * (r.mag[col] - c.mag_offset[col]);
      ship += s.magCal.matrix[3 * row + col] * (s.mag[col] * 1e4f - s.magCal.offset[col]); // vehicle: gauss
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-9f, ship * 1e-4f, gs);
  }
}

void test_attitude_std_world() {
  // Identity: equals the old small-angle mapping 2 * sqrt(P_vector)
  const float qi[4] = {1, 0, 0, 0};
  float P[16] = {0};
  P[0] = 0.5f;           // w variance: no rotation information
  P[5] = 0.0001f;        // x
  P[10] = 0.0004f;       // y
  P[15] = 0.0025f;       // z
  float o[3];
  attitudeStdWorld(qi, P, o);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, o[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.04f, o[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.10f, o[2]);

  // q = 117 deg about +Z with a pure heading uncertainty sigma: dq = 0.5 * [-z, 0, 0, w] * dpsi.
  // The old diagonal mapping would have reported 2*sqrt(P_zz) = sigma*w on "z" and nothing for the w part.
  const float a = 117.0f * (float)M_PI / 180.0f, sigma = 0.3f;
  const float q[4] = {cosf(a / 2), 0, 0, sinf(a / 2)};
  const float d[4] = {-0.5f * q[3], 0, 0, 0.5f * q[0]};
  float Py[16];
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++) Py[4 * r + c] = d[r] * d[c] * sigma * sigma;
  attitudeStdWorld(q, Py, o);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, o[0]); // sqrt of a float rounding residual
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, o[1]); // sqrt of a float rounding residual
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, sigma, o[2]);

  // Same q, tilt uncertainty about world North only: dq = 0.5 * Omega_x(q) * dtheta
  const float dx[4] = {-0.5f * q[1], 0.5f * q[0], -0.5f * q[3], 0.5f * q[2]};
  for (int r = 0; r < 4; r++)
    for (int c = 0; c < 4; c++) Py[4 * r + c] = dx[r] * dx[c] * sigma * sigma;
  attitudeStdWorld(q, Py, o);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, sigma, o[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, o[1]); // sqrt of a float rounding residual
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, o[2]); // sqrt of a float rounding residual

  // Negative variance (numerically broken P) gives 0, not NaN
  float Pn[16] = {0};
  Pn[15] = -1.0f;
  attitudeStdWorld(qi, Pn, o);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, o[2]);
}

void test_home_position_from_a_faked_reference() {
  HomeInput in;
  in.haveReference = true;
  in.latitude = 48.1234 * M_PI / 180.0;
  in.longitude = 11.5678 * M_PI / 180.0;
  in.altitude = 523.25f;
  in.epoch = 3;
  in.setTimeNs = 7000000000LL;
  Packets::HomePosition p = makeHomePosition(in);
  TEST_ASSERT_EQUAL_UINT8(0x03, p.flags);
  TEST_ASSERT_EQUAL_UINT8(0x50, Packets::HomePosition::kPort);
  uint8_t buf[Packets::HomePosition::kMaxSize];
  TEST_ASSERT_EQUAL_UINT(25, p.encode(buf, sizeof buf));
  Packets::HomePosition q;
  TEST_ASSERT_TRUE(q.decode(buf, 25));
  TEST_ASSERT_DOUBLE_WITHIN(1e-8, in.latitude, q.latitude); // 1e-7 deg wire resolution
  TEST_ASSERT_DOUBLE_WITHIN(1e-8, in.longitude, q.longitude);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 523.25f, q.altitude);
  TEST_ASSERT_TRUE(isnan(q.h_acc));
  TEST_ASSERT_EQUAL_UINT8(3, q.epoch);
  TEST_ASSERT_EQUAL_INT64(7000000000LL, q.set_time);
  in.simulation = true;
  in.zeroing = true;
  TEST_ASSERT_EQUAL_UINT8(0x0F, makeHomePosition(in).flags);
}

void test_home_position_without_fix_has_nan_position() {
  HomeInput in;
  in.haveReference = true; // reference set but the estimator never had a GNSS fix (0, 0)
  in.epoch = 1;
  Packets::HomePosition p = makeHomePosition(in);
  TEST_ASSERT_EQUAL_UINT8(0, p.flags);
  TEST_ASSERT_TRUE(isnan(p.latitude) && isnan(p.longitude) && isnan(p.altitude));
  uint8_t buf[25];
  TEST_ASSERT_EQUAL_UINT(25, p.encode(buf, sizeof buf));
}

// ---- attitude setpoint (MISSION / AIR_MISSION setpoint_q) ----

static void rotZ(const float q[4], float z[3]) { // body +Z in world, R(q) third column
  z[0] = 2 * (q[1] * q[3] + q[0] * q[2]);
  z[1] = 2 * (q[2] * q[3] - q[0] * q[1]);
  z[2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
}

static float qnorm(const float q[4]) { return sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); }

void test_attitude_setpoint_selection() {
  TEST_ASSERT_EQUAL(int(AttitudeSetpointSource::None), int(selectAttitudeSetpointSource(false, false)));
  TEST_ASSERT_EQUAL(int(AttitudeSetpointSource::Tvc), int(selectAttitudeSetpointSource(true, false)));
  TEST_ASSERT_EQUAL(int(AttitudeSetpointSource::Flaps), int(selectAttitudeSetpointSource(false, true)));
  TEST_ASSERT_EQUAL(int(AttitudeSetpointSource::Flaps), int(selectAttitudeSetpointSource(true, true)));

  AttitudeSetpointInputs in;
  const float tq[4] = {0.9f, 0.1f, -0.2f, 0.0f}; // not unit: published normalised
  memcpy(in.tvcQ, tq, sizeof tq);
  in.flapAzimuth = 0;
  in.flapPitch = 0;
  float out[4] = {9, 9, 9, 9};
  attitudeSetpoint(in, out); // nothing active (idle, ground, countdown)
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i]);

  in.tvcEnabled = true;
  attitudeSetpoint(in, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, qnorm(out));
  const float n = qnorm(tq);
  for (int i = 0; i < 4; i++) TEST_ASSERT_FLOAT_WITHIN(1e-6f, tq[i] / n, out[i]);

  in.tvcEnabled = false;
  in.flapsStabilize = true;
  attitudeSetpoint(in, out); // identity -> body +Z to North: +90 deg about +Y
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, cosf(M_PI / 4), out[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, sinf(M_PI / 4), out[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[3]);

  in.tvcEnabled = true; // both on (the phase table excludes it since 2026-11-21): the flap setpoint
  float out2[4];
  attitudeSetpoint(in, out2);
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(out[i], out2[i]);

  // Bellyflop kick (2026-11-21): TVC on, flaps not in Stabilize, setpoint written by the mission -> the kick
  // target (here the old fixed Quat(y, -90 deg)), not the mapping topic.
  in.flapsStabilize = false;
  in.tvcEnabled = true;
  in.tvcOverride = true;
  const float kick[4] = {cosf(M_PI / 4), 0.0f, -sinf(M_PI / 4), 0.0f};
  memcpy(in.tvcOverrideQ, kick, sizeof kick);
  attitudeSetpoint(in, out);
  for (int i = 0; i < 4; i++) TEST_ASSERT_FLOAT_WITHIN(1e-6f, kick[i], out[i]);
  // Override without the TVC set on (kick ended, flaps took over) is ignored.
  in.tvcEnabled = false;
  in.flapsStabilize = true;
  attitudeSetpoint(in, out2);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, cosf(M_PI / 4), out2[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, sinf(M_PI / 4), out2[2]);
  in.flapsStabilize = false;
  attitudeSetpoint(in, out2);
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out2[i]);
}

void test_attitude_setpoint_sanitize() {
  float out[4];
  const float nanq[4] = {NAN, 0, 0, 0};
  sanitizeSetpointQuat(nanq, out);
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i]);
  const float zero[4] = {0, 0, 0, 0};
  sanitizeSetpointQuat(zero, out);
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i]);
  const float inf[4] = {INFINITY, 0, 0, 0};
  sanitizeSetpointQuat(inf, out);
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i]);
  const float neg[4] = {-2.0f, 0, 0, 0}; // w < 0 flipped, scaled to unit
  sanitizeSetpointQuat(neg, out);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, out[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, out[1]);
}

void test_flap_pointing_setpoint() {
  // General attitude: the result's body +Z is the wanted direction and the correction is the shortest rotation.
  const float qa[4] = {0.8f, 0.2f, -0.4f, 0.4f};
  float qIs[4];
  sanitizeSetpointQuat(qa, qIs);
  const float az = 2.5f, pitch = -0.3f;
  float out[4];
  flapPointingSetpoint(az, pitch, qIs, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, qnorm(out));
  TEST_ASSERT_TRUE(out[0] >= 0);
  float z[3], z0[3];
  rotZ(out, z);
  rotZ(qIs, z0);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, cosf(pitch) * cosf(az), z[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, cosf(pitch) * sinf(az), z[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, sinf(pitch), z[2]);
  // the controller's own definitions recover the inputs
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, az, atan2f(z[1], z[0]));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, pitch, atan2f(z[2], sqrtf(z[0] * z[0] + z[1] * z[1])));
  // shortest: rotation angle between qIs and out == angle between the two Z axes
  const float dot = fabsf(out[0] * qIs[0] + out[1] * qIs[1] + out[2] * qIs[2] + out[3] * qIs[3]);
  const float relAngle = 2 * acosf(fminf(1.0f, dot));
  const float axisAngle = acosf(z0[0] * z[0] + z0[1] * z[1] + z0[2] * z[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, axisAngle, relAngle);

  // Opposite direction (upright, wanted straight down): a 180 deg rotation, still valid.
  const float id[4] = {1, 0, 0, 0};
  flapPointingSetpoint(0.0f, -M_PI / 2, id, out);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, qnorm(out));
  rotZ(out, z);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, z[2]);

  // Large corrections (> about 151 deg, but not exactly opposite) were published as zero before 2026-11-21 (the
  // unnormalised half-angle quaternion fell under sanitizeSetpointQuat's 0.5 threshold; bench: zero setpoint_q in
  // early freefall). Upright, wanted 160 and 175 deg away: valid, body +Z on the wanted direction.
  for (float deg : {120.0f, 155.0f, 160.0f, 175.0f, 179.5f}) {
    const float pitch = float(M_PI / 2) - deg * float(M_PI) / 180.0f;
    flapPointingSetpoint(0.7f, pitch, id, out);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, qnorm(out));
    rotZ(out, z);
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, sinf(pitch), z[2]);
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, cosf(pitch) * cosf(0.7f), z[0]);
  }

  // Invalid current attitude: no setpoint
  const float bad[4] = {0, 0, 0, 0};
  flapPointingSetpoint(0.0f, 0.0f, bad, out);
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i]);
}

void test_mission_setpoint_q_encoding() {
  TelemetrySnapshot s = sample();
  // default: no setpoint = all four zero, exactly, on both packets
  auto m = roundTrip(makeMission(s));
  auto a = roundTrip(makeAirMission(s));
  for (int i = 0; i < 4; i++) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.setpoint_q[i]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, a.setpoint_q[i]);
  }
  // unit quaternion: f32 on USB exact; air i16 limit 1.0 -> raw = round(v * 32767)
  const float q[4] = {0.5f, 0.5f, -0.5f, 0.5f};
  memcpy(s.setpointQ, q, sizeof q);
  m = roundTrip(makeMission(s));
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(q[i], m.setpoint_q[i]);
  uint8_t buf[Packets::AirMission::kMaxSize];
  TEST_ASSERT_EQUAL_UINT(42, makeAirMission(s).encode(buf, sizeof buf));
  const int16_t want[4] = {16384, 16384, -16384, 16384}; // 16383.5 rounds away from zero
  for (int i = 0; i < 4; i++) {
    const int16_t raw = int16_t(uint16_t(buf[30 + 2 * i]) | (uint16_t(buf[31 + 2 * i]) << 8));
    TEST_ASSERT_EQUAL_INT16(want[i], raw);
  }
  a = roundTrip(makeAirMission(s));
  for (int i = 0; i < 4; i++) TEST_ASSERT_FLOAT_WITHIN(1.0f / 32767, q[i], a.setpoint_q[i]);
  // full scale: +-1.0 -> +-32767 (never the -32768 NaN sentinel)
  const float id[4] = {1.0f, 0, 0, -1.0f};
  memcpy(s.setpointQ, id, sizeof id);
  makeAirMission(s).encode(buf, sizeof buf);
  TEST_ASSERT_EQUAL_INT16(32767, int16_t(uint16_t(buf[30]) | (uint16_t(buf[31]) << 8)));
  TEST_ASSERT_EQUAL_INT16(-32767, int16_t(uint16_t(buf[36]) | (uint16_t(buf[37]) << 8)));
  a = roundTrip(makeAirMission(s));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, a.setpoint_q[0]);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, a.setpoint_q[3]);
}

void test_packet_sizes_fit_budget() {
  // The air set must stay small (design 3.5.6 / telemetry-sets.md)
  TEST_ASSERT_EQUAL_UINT(38, Packets::VehicleStatus::kMaxSize); // +2: cmd_session (GS control)
  TEST_ASSERT_EQUAL_UINT(30, Packets::AirPose::kMaxSize);
  TEST_ASSERT_EQUAL_UINT(34, Packets::HandsetTelem::kMaxSize);
  TEST_ASSERT_EQUAL_UINT(38, Packets::AirActuators::kMaxSize);
  TEST_ASSERT_EQUAL_UINT(42, Packets::AirMission::kMaxSize);
  TEST_ASSERT_EQUAL_UINT(60, Packets::Mission::kMaxSize);
  TEST_ASSERT_EQUAL_UINT(22, Packets::AirGnss::kMaxSize);
  TEST_ASSERT_EQUAL_UINT(10, Packets::AirGnssInfo::kMaxSize);
  TEST_ASSERT_EQUAL_UINT16(25, Packets::AirPose::kAirRateHz);
  TEST_ASSERT_EQUAL_UINT16(10, Packets::AirGnss::kAirRateHz);
  TEST_ASSERT_EQUAL_UINT16(1, Packets::AirGnssInfo::kAirRateHz);
  TEST_ASSERT_EQUAL_UINT16(10, Packets::Gnss::kUsbRateHz);
  TEST_ASSERT_TRUE(Packets::AirActuators::kAirEligible);
  TEST_ASSERT_TRUE(Packets::AirMission::kAirEligible);
  TEST_ASSERT_EQUAL_HEX32(0x036F9C45u, Packets::kSchemaHash);
  TEST_ASSERT_EQUAL_UINT(164, Packets::Calibration::kMaxSize);
  TEST_ASSERT_FALSE(Packets::Calibration::kAirEligible); // USB only
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_failure_bits);
  RUN_TEST(test_interlock_need_toggle_flag);
  RUN_TEST(test_sensor_state);
  RUN_TEST(test_time_us_wraps);
  RUN_TEST(test_vehicle_status);
  RUN_TEST(test_pose_full_precision);
  RUN_TEST(test_air_pose_fixed_point_and_saturation);
  RUN_TEST(test_actuators);
  RUN_TEST(test_mission_time_seconds);
  RUN_TEST(test_gnss_radians);
  RUN_TEST(test_air_gnss);
  RUN_TEST(test_air_gnss_info);
  RUN_TEST(test_pack_calib_state);
  RUN_TEST(test_vehicle_status_home_and_calib);
  RUN_TEST(test_vehicle_status_t9b_none_values);
  RUN_TEST(test_vehicle_status_t9b4_command_path_fields);
  RUN_TEST(test_home_epoch_counter);
  RUN_TEST(test_actuators_roll_torque_and_mission_fields);
  RUN_TEST(test_handset_telem);
  RUN_TEST(test_course_over_ground);
  RUN_TEST(test_mode_text);
  RUN_TEST(test_imu_raw_and_estimator);
  RUN_TEST(test_calibration);
  RUN_TEST(test_attitude_std_world);
  RUN_TEST(test_home_position_from_a_faked_reference);
  RUN_TEST(test_home_position_without_fix_has_nan_position);
  RUN_TEST(test_air_actuators_two_samples);
  RUN_TEST(test_air_actuators_saturation_and_nan);
  RUN_TEST(test_air_mission);
  RUN_TEST(test_mission_flip_start_and_stop_altitude);
  RUN_TEST(test_attitude_setpoint_selection);
  RUN_TEST(test_attitude_setpoint_sanitize);
  RUN_TEST(test_flap_pointing_setpoint);
  RUN_TEST(test_mission_setpoint_q_encoding);
  RUN_TEST(test_cmd_telemetry_fields_and_cmd_applied);
  RUN_TEST(test_packet_sizes_fit_budget);
  return UNITY_END();
}
