// Host-only stubs of the two ExVectrCore functions that ExVectrData's memory_manager.cpp logs with. The Core sources are
// not built natively; platformio.ini [env:native] build_src_filter links this file together with memory_manager.cpp
// (used by test/test_calibration_memory). Never part of the teensy40 build.
namespace VCTR {
namespace Core {
double NowS() { return 0.0; }
void printD(const char *, ...) {}
} // namespace Core
} // namespace VCTR
