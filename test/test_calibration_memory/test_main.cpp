// Legacy calibration keys through the real Memory_Manager, natively: pio test -e native -f test_calibration_memory
//
// The calibration policy reads and writes the legacy EEPROM keys 100 (gyro), 101 (mag) and 102 (accel) as
// DSP::ValueCov<float,3> (48 B: val = offset, cov = C). This test runs the same Memory_Manager calls as
// initialiseMemory() / applyLegacyCalibration() in main.cpp on a RAM image: find by key and size, a missing key,
// the identity default created at boot (and seen as Default after a reboot), and the image copy to a second memory
// that stands for the EEPROM (review 2026-11-21, test 4). The Core log stubs live in test/support/native_core_stubs.cpp.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "ExVectrData/memory_internal.hpp"
#include "ExVectrData/memory_manager.hpp"
#include "ExVectrDSP/value_covariance.hpp"
#include "calibration/calibration_validity.hpp"
#include "memory_keys.hpp"

using namespace VCTR;
using namespace VCTR::Calibration;
using Cal = DSP::ValueCov<float, 3>;

void setUp() {}
void tearDown() {}

/// Same conversion as main.cpp storedFrom(): key present, val -> offset, cov -> C.
static StoredCalibration storedFrom(const Cal &c, bool present) {
  StoredCalibration s;
  s.keyPresent = present;
  for (int i = 0; i < 3; i++)
    s.offset[i] = double(c.val(i));
  for (int r = 0; r < 3; r++)
    for (int k = 0; k < 3; k++)
      s.C[3 * r + k] = double(c.cov(r, k));
  return s;
}

static Cal identityDefault() {
  Cal c;
  c.val = Math::Vector<float, 3>({0, 0, 0});
  c.cov = Math::Matrix<float, 3, 3>({1, 0, 0, 0, 1, 0, 0, 0, 1});
  return c;
}

static Cal benchMagRecord() { // bench mag of docs/project/hardware-and-drivers.md: C = 0.1622 I
  Cal c;
  c.val = Math::Vector<float, 3>({1.0491e-2f, -1.467e-3f, 8.154e-3f});
  c.cov = Math::Matrix<float, 3, 3>({0.1622f, 0, 0, 0, 0.1622f, 0, 0, 0, 0.1622f});
  return c;
}

static void test_record_size_is_48_bytes() { TEST_ASSERT_EQUAL_size_t(48, sizeof(Cal)); }

static void test_missing_key_then_default_then_reboot() {
  Data::Memory_Internal<512> mem;
  Data::Memory_Manager mgr(mem);
  mgr.clearMemory();
  Cal c;
  TEST_ASSERT_FALSE(mgr.readItem(c, MEMORY_KEY_MAGCALIB)); // missing key: main creates the default
  Cal def = identityDefault();
  mgr.allocateItem(def, MEMORY_KEY_MAGCALIB);
  mgr.writeItem(def, MEMORY_KEY_MAGCALIB);
  TEST_ASSERT_TRUE(mgr.readItem(c, MEMORY_KEY_MAGCALIB));
  // First boot: the key was created, so keyPresent stays false in main (KeyMissing).
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::KeyMissing), uint8_t(checkMagParams(storedFrom(c, false))));

  // Reboot: a new manager on the same image reads the key, which is now present but still the identity: Default.
  Data::Memory_Manager mgr2(mem);
  Cal c2;
  TEST_ASSERT_TRUE(mgr2.readItem(c2, MEMORY_KEY_MAGCALIB));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Default), uint8_t(checkMagParams(storedFrom(c2, true))));
}

static void test_write_read_round_trip_and_key_isolation() {
  Data::Memory_Internal<512> mem;
  Data::Memory_Manager mgr(mem);
  mgr.clearMemory();
  Cal def = identityDefault();
  mgr.allocateItem(def, MEMORY_KEY_GYROCALIB);
  mgr.allocateItem(def, MEMORY_KEY_MAGCALIB);
  mgr.allocateItem(def, MEMORY_KEY_ACCCALIB);
  const Cal mag = benchMagRecord();
  TEST_ASSERT_TRUE(mgr.writeItem(mag, MEMORY_KEY_MAGCALIB));

  Cal r;
  TEST_ASSERT_TRUE(mgr.readItem(r, MEMORY_KEY_MAGCALIB));
  for (int i = 0; i < 3; i++)
    TEST_ASSERT_EQUAL_FLOAT(mag.val(i), r.val(i));
  TEST_ASSERT_EQUAL_FLOAT(0.1622f, r.cov(1, 1));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(checkMagParams(storedFrom(r, true))));
  // the other keys are untouched
  TEST_ASSERT_TRUE(mgr.readItem(r, MEMORY_KEY_GYROCALIB));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, r.cov(0, 0));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.val(0));
  TEST_ASSERT_TRUE(mgr.readItem(r, MEMORY_KEY_ACCCALIB));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Default), uint8_t(checkAccelParams(storedFrom(r, true))));
  // a record of another size under the same key is not found (the size is part of the match)
  DSP::ValueCov<float, 1> wrongSize;
  TEST_ASSERT_FALSE(mgr.readItem(wrongSize, MEMORY_KEY_MAGCALIB));
}

static void test_save_to_second_memory_and_reboot_from_it() {
  Data::Memory_Internal<512> ram, eeprom;
  Data::Memory_Manager mgr(ram);
  mgr.clearMemory();
  Cal def = identityDefault();
  mgr.allocateItem(def, MEMORY_KEY_MAGCALIB);
  TEST_ASSERT_TRUE(mgr.writeItem(benchMagRecord(), MEMORY_KEY_MAGCALIB));
  eeprom.transferFrom(ram); // Save: RAM image -> EEPROM (main verifies by read-back)
  uint8_t a[64], b[64];
  for (size_t off = 0; off < 512; off += sizeof(a)) {
    ram.readMem(a, sizeof(a), off);
    eeprom.readMem(b, sizeof(b), off);
    TEST_ASSERT_EQUAL_MEMORY(a, b, sizeof(a));
  }
  // next boot: RAM image from the EEPROM
  Data::Memory_Internal<512> ram2;
  ram2.transferFrom(eeprom);
  Data::Memory_Manager mgr2(ram2);
  TEST_ASSERT_EQUAL_UINT8(mgr2.getManagerVersion(), mgr2.getMemoryVersion());
  Cal r;
  TEST_ASSERT_TRUE(mgr2.readItem(r, MEMORY_KEY_MAGCALIB));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(checkMagParams(storedFrom(r, true))));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_record_size_is_48_bytes);
  RUN_TEST(test_missing_key_then_default_then_reboot);
  RUN_TEST(test_write_read_round_trip_and_key_isolation);
  RUN_TEST(test_save_to_second_memory_and_reboot_from_it);
  return UNITY_END();
}
