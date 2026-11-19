// Host tests of src/comms/telemetry_mapping.cpp: pio test -e native
// Checks the vehicle-state -> packet mapping, the flight-software owned bit fields, SI units through the
// generated encode/decode (round trip), saturation, and the TEMPORARY legacy telecommand layout.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "comms/telemetry_mapping.hpp"

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
  f.radioConnectionLoss = true;
  TEST_ASSERT_EQUAL_HEX32(0x20, packFailureBits(f));
  f.positionOutOfBounds = true;
  f.attitudeOutOfBounds = true;
  TEST_ASSERT_EQUAL_HEX32(0x38, packFailureBits(f));
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
  s.failures.radioConnectionLoss = true;
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
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, p.pos[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, -20.0f, p.pos[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, -2.0f, p.vel[1]);
  // Saturates instead of overflowing (old int16 telemetry was undefined behaviour out of range)
  s.pos[3] = 1e6f;
  s.pos[0] = -1e6f;
  p = roundTrip(makeAirPose(s));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3276.7f, p.pos[0]);
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
  TEST_ASSERT_EQUAL_UINT8(0, roundTrip(makeGnss(s)).fix_type);
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

void test_legacy_telecommand_layout() {
  // Bytes as the old packed struct on the (little-endian) Teensy: Arm (5), subsystem 0, 0xA5, 1.0f
  const uint8_t wire[8] = {0x05, 0x00, 0xA5, 0x00, 0x00, 0x00, 0x80, 0x3F};
  LegacyTelecommandPacket p;
  TEST_ASSERT_TRUE(p.decode(wire, 8));
  TEST_ASSERT_EQUAL_UINT8(Telecommand_Arm, p.tc.type);
  TEST_ASSERT_EQUAL_UINT8(Subsystem_Gyro, p.tc.subsystem);
  TEST_ASSERT_EQUAL_INT16(0xA5, p.tc.paramInt);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, p.tc.paramFloat);
  uint8_t out[8];
  TEST_ASSERT_EQUAL_UINT(8, p.encode(out, sizeof out));
  TEST_ASSERT_EQUAL_MEMORY(wire, out, 8);
  // Negative paramInt (MissionBegin -10 s) and wrong lengths
  const uint8_t begin[8] = {0x07, 0x00, 0xF6, 0xFF, 0, 0, 0, 0};
  TEST_ASSERT_TRUE(p.decode(begin, 8));
  TEST_ASSERT_EQUAL_INT16(-10, p.tc.paramInt);
  TEST_ASSERT_FALSE(p.decode(wire, 7));
  uint8_t nine[9] = {0};
  TEST_ASSERT_FALSE(p.decode(nine, 9));
  TEST_ASSERT_TRUE(LegacyTelecommandPacket::kPort < 0x80); // must pass the radio-path egress filters
}

void test_packet_sizes_fit_budget() {
  // The air set must stay small (design 3.5.6 / telemetry-sets.md)
  TEST_ASSERT_EQUAL_UINT(28, Packets::VehicleStatus::kMaxSize);
  TEST_ASSERT_EQUAL_UINT(24, Packets::AirPose::kMaxSize);
  TEST_ASSERT_EQUAL_UINT(34, Packets::HandsetTelem::kMaxSize);
  TEST_ASSERT_EQUAL_HEX32(0x3AFA3CCEu, Packets::kSchemaHash);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_failure_bits);
  RUN_TEST(test_sensor_state);
  RUN_TEST(test_time_us_wraps);
  RUN_TEST(test_vehicle_status);
  RUN_TEST(test_pose_full_precision);
  RUN_TEST(test_air_pose_fixed_point_and_saturation);
  RUN_TEST(test_actuators);
  RUN_TEST(test_mission_time_seconds);
  RUN_TEST(test_gnss_radians);
  RUN_TEST(test_handset_telem);
  RUN_TEST(test_course_over_ground);
  RUN_TEST(test_mode_text);
  RUN_TEST(test_imu_raw_and_estimator);
  RUN_TEST(test_legacy_telecommand_layout);
  RUN_TEST(test_packet_sizes_fit_budget);
  return UNITY_END();
}
