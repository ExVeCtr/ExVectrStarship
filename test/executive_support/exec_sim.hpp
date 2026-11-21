#ifndef TEST_EXECUTIVE_SUPPORT_EXEC_SIM_HPP
#define TEST_EXECUTIVE_SUPPORT_EXEC_SIM_HPP

// Closed loop for the executive host tests: fake clock + follower plant + FlightExecutive. Copyable, so a test can
// branch a scenario (for example abort in a phase reached by the nominal flight).

#include <math.h>
#include <stdint.h>

#include "ExVectrControl/executive/flight_executive.hpp"

#include "fake_clock.hpp"
#include "follower_plant.hpp"

namespace ExecTest {

/// One logged phase entry.
struct PhaseEntry {
  uint8_t phase;
  int64_t tNs; ///< time since go
};

class ExecSim {
public:
  explicit ExecSim(const VCTR::Exec::FlightPlan &plan, const PlantConfig &cfg = PlantConfig())
      : plant(cfg), exec(plan) {}

  FakeClock clock;
  FollowerPlant plant;
  VCTR::Exec::FlightExecutive exec;
  VCTR::Exec::ExecInputs next; ///< operator/supervisor inputs for the next step (edges are cleared after use)
  int64_t goNs = 0;
  bool sanitizedSeen = false;

  /// One step: inputs from the plant, executive step, plant step with the command.
  const VCTR::Exec::ControlCommand &tick() {
    clock.advance();
    VCTR::Exec::ExecInputs in = next;
    in.nowNs = clock.nowNs;
    in.dtNs = clock.dtNs;
    in.est = plant.snapshot(clock.nowNs);
    if (in.cmd.goEdge)
      goNs = clock.nowNs;
    const VCTR::Exec::ControlCommand &c = exec.step(in);
    sanitizedSeen = sanitizedSeen || c.sanitized;
    next.cmd.goEdge = false;
    next.cmd.resetEdge = false;
    plant.step(c, float(clock.dtNs) / 1e9f);
    return c;
  }

  void go(int16_t startS = -15) {
    next.cmd.goEdge = true;
    next.cmd.legacyStartS = startS;
    tick();
  }

  /// Steps until the phase is p (true) or maxS seconds passed (false).
  bool runUntilPhase(uint8_t p, double maxS) {
    const int64_t end = clock.nowNs + int64_t(maxS * 1e9);
    while (clock.nowNs < end) {
      if (exec.phase() == p)
        return true;
      tick();
    }
    return exec.phase() == p;
  }

  void runFor(double s) {
    const int64_t end = clock.nowNs + int64_t(s * 1e9);
    while (clock.nowNs < end)
      tick();
  }

  double sinceGoS() const { return double(clock.nowNs - goNs) / 1e9; }
};

/// All float fields of a command finite.
inline bool commandFinite(const VCTR::Exec::ControlCommand &c) {
  for (float v : c.posSp)
    if (!isfinite(v))
      return false;
  for (float v : c.attSp)
    if (!isfinite(v))
      return false;
  return isfinite(c.thrustN) && isfinite(c.thrustCapN) && isfinite(c.flapAzimuthRad) && isfinite(c.flapPitchRad);
}

/// Field-by-field equality, ignoring the monotonic counters phaseSeq and actuatorTestSeq.
inline bool sameCommand(const VCTR::Exec::ControlCommand &a, const VCTR::Exec::ControlCommand &b) {
  for (int i = 0; i < 6; i++)
    if (a.posSp[i] != b.posSp[i])
      return false;
  for (int i = 0; i < 4; i++)
    if (a.attSp[i] != b.attSp[i])
      return false;
  return a.phase == b.phase && a.missionTimeNs == b.missionTimeNs && a.tvc == b.tvc && a.resetTvc == b.resetTvc &&
         a.attSpSource == b.attSpSource && a.thrustN == b.thrustN && a.thrustCapN == b.thrustCapN &&
         a.flaps == b.flaps && a.flapAzimuthRad == b.flapAzimuthRad && a.flapPitchRad == b.flapPitchRad &&
         a.tvcActuators == b.tvcActuators && a.flapActuators == b.flapActuators && a.motors == b.motors &&
         a.estimatorZeroing == b.estimatorZeroing && a.bounds == b.bounds && a.onGround == b.onGround &&
         a.sanitized == b.sanitized;
}

/// Boot command: all requests off, TVC off.
inline bool allOff(const VCTR::Exec::ControlCommand &c) {
  return c.tvc == VCTR::Exec::TvcMode::Off && c.flaps == VCTR::Exec::FlapMode::Disabled && !c.tvcActuators &&
         !c.flapActuators && !c.motors && !c.estimatorZeroing && !c.resetTvc;
}

} // namespace ExecTest

#endif
