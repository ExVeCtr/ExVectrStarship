#ifndef TEST_EXECUTIVE_SUPPORT_FAKE_CLOCK_HPP
#define TEST_EXECUTIVE_SUPPORT_FAKE_CLOCK_HPP

// Fake monotonic clock for the executive host tests (header only; included by the test_executive_* suites).

#include <stdint.h>

namespace ExecTest {

constexpr int64_t kMs = 1000000LL;
constexpr int64_t kS = 1000000000LL;

struct FakeClock {
  int64_t nowNs = 1 * kS; ///< non-zero start, so "time 0" bugs show
  int64_t dtNs = 10 * kMs; ///< 100 Hz by default; tests may change it per step
  int64_t advance() {
    nowNs += dtNs;
    return nowNs;
  }
};

} // namespace ExecTest

#endif
