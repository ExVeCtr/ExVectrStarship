#ifndef SAFETY_RC_COMMAND_HPP
#define SAFETY_RC_COMMAND_HPP

/**
 * RC command-intent layer (T9b-4, docs/t9b-design.md 3.2, 3.3, D7, D8, D10 to D12, owner decisions 1.3). Pure, no
 * heap, no clock of its own (every call takes the time), NO dependence on the simulation flag as a gate: the
 * simulation flag appears only as the "applied mode" that SC must match for a go (SC selects it, nothing is gated
 * by it). Host-tested (test/test_rc_mapper).
 *
 * RcCommandMapper (rc_mapper.hpp) turns the accepted RC_STATE of the active source into latched edge events. This
 * layer takes those events plus a description of the vehicle (RcPollInput, filled by VehicleSafetyAndControlTask
 * every 10 Hz tick) and returns what to do (RcIntents). main.cpp applies the intents with the existing vehicle
 * code; nothing here touches an actuator.
 *
 *  SA  NOT_GO while a mission chain is pending or the mode is Running: abortLevel EVERY tick (the caller feeds it to
 *      Safety::decideAbort, idempotent; ResetAndStop does not clear latches). SA GO edge: go if decideGo() says so,
 *      otherwise the refusal reason is kept for telemetry (go_refusal).
 *  SB  read at the GO edge only (profile 2 is the only implemented one, D9); later changes are never looked at.
 *  SC  an observed change 0 <-> 1 selects simulation / real while on the ground and not committed (decideSc); the
 *      vehicle boots in real mode, so an SC position at boot does nothing until the switch is cycled. Refused
 *      while committed, in flight, or in mode Failure (a mode switch re-initialises the vehicle and would clear
 *      a latched failsafe; only SE clears it).
 *  SD  not handled here: the motor interlock (motor_interlock.hpp) is a pure gate on the SD state alone.
 *  SE  a short press = system reset only on the ground with the interlock closed (D12); a hold of 1 s zeroes the
 *      estimators while held (hold intents drive EstimatorZeroing's hold API, D11).
 */

#include <stdint.h>

#include "rc_mapper.hpp"

namespace VCTR {
namespace Safety {

/// go_refusal codes of VEHICLE_STATUS (schema starship.toml).
enum class GoRefusal : uint8_t {
  None = 0,
  NotReady = 1,    ///< vehicle mode is not Ready (also: calibration invalid)
  Latched = 2,     ///< latched abort / failsafe (mode Failure)
  Zeroing = 3,     ///< estimator zeroing window active or no valid zero
  ScMismatch = 4,  ///< SC invalid, reserved, or not equal to the applied mode
  SbProfile = 5,   ///< SB invalid or its profile is not implemented
  Interlock = 6,   ///< reserved: the interlock is NOT a go precondition (owner decision 2026-11-21)
  RequestBusy = 7, ///< a calibration session or request is in progress
};

/// Only SB position 2 (today's full flight) is implemented (D9).
constexpr uint8_t kImplementedProfile = 2;

/**
 * Countdown of a go, a compiled-in constant (D8, owner decision 2026-11-22: fixed 15 s), as the telecommand "start
 * time" parameter (seconds, negative = before mission time zero). The Initialisation phase is then 5 s; the countdown
 * gyro run uses the shorter 3 s still window (Calibration::GyroRunKind::Countdown) so it fits with slack
 * (kMinCountdownParamInt = -15 is derived from it).
 */
constexpr int16_t kGoCountdownParamInt = -15;

struct GoContext {
  bool modeReady = false;
  bool latched = false;        ///< mode Failure or any failure flag: latched abort / failsafe
  bool zeroingActive = false;
  bool zeroValid = false;      ///< a valid estimator zero exists since boot
  bool requestBusy = false;
  uint8_t sc = RcDecoded::kInvalid; ///< SC position of the active source
  bool simulationApplied = false;
  uint8_t sb = RcDecoded::kInvalid; ///< SB position of the GO sample
};

/// D7 as finally decided: Ready, no latch, no zeroing, SC valid and equal to the applied mode, SB valid with an
/// implemented profile, no request in progress. The interlock is not a precondition. First failing rule wins.
GoRefusal decideGo(const GoContext &c);
const char *toString(GoRefusal r);

enum class ScAction : uint8_t { None = 0, EnterSimulation, EnterReal, Refused };
struct ScContext {
  bool onGround = false;
  bool committed = false;
  bool modeFailure = false;
  bool simulationApplied = false;
};
/// `edgeTo` = RcEvents::scEdgeTo. An edge to the applied mode changes nothing.
ScAction decideSc(uint8_t edgeTo, const ScContext &c);

/// D12 "on the ground" (SE reset, SC change): interlock closed and motors not enabled and either the mode is
/// Startup/Ready/SensorCalib with the active mission Idle or Finished, or the mode is Failure (actuators already
/// cut), or the mode is Running but the chain has ended (landed: no chain pending, mission Idle or Finished; after
/// RTH the mode stays Running until a reset).
struct GroundContext {
  bool interlockOpen = false;
  bool motorsEnabled = false;
  bool modeGround = false;  ///< Startup, Ready or SensorCalib
  bool modeFailure = false;
  bool modeRunning = false;
  bool chainPending = false;
  bool missionIdleOrFinished = false;
};
bool onGround(const GroundContext &g);

/**
 * "Committed" (design 3.1.3, one definition for the control-source latch and the control failsafe): the interlock is
 * open, or a mission chain is pending, or the mode is Running and the vehicle is NOT on the ground (onGround above).
 * A landed vehicle (mode Running after an RTH, no chain pending, mission Idle or Finished, interlock closed, motors
 * not enabled) is not committed, so a radio gap on the ground only sets ctrl_source NONE (D2-B). Running with the
 * mission active, motors enabled or the interlock open stays committed.
 */
bool committed(const GroundContext &g);

enum class SeAction : uint8_t { None = 0, Reset, Refused };
enum class HoldAction : uint8_t { None = 0, Start, Refresh, Release, Abort };

/// Everything the layer needs to know this tick.
struct RcPollInput {
  int64_t nowNs = 0;
  bool haveSource = false;        ///< an active source exists and its last state is fresh (at most 1 s old)
  RcDecoded decoded;              ///< decoded switches of the active source
  bool chainPendingOrRunning = false; ///< mission chain pending or vehicle mode Running
  bool onGround = false;          ///< D12
  bool committed = false;         ///< interlock open, chain pending or mode Running
  bool interlockOpen = false;
  bool simulationApplied = false;
  bool zeroingHoldActive = false; ///< EstimatorZeroing::holdActive()
  GoContext go;                   ///< everything but sc, simulationApplied and sb, which the layer fills in
  bool modeFailure = false;
};

struct RcIntents {
  bool abortLevel = false;        ///< SA NOT_GO with a chain pending or Running: decideAbort now
  bool go = false;                ///< begin the mission with `goProfile` and the kGoCountdownParamInt countdown
  uint8_t goProfile = RcDecoded::kInvalid;
  GoRefusal goRefused = GoRefusal::None; ///< a GO edge was consumed and refused (logged by the caller)
  bool goEdgeSeen = false;
  ScAction sc = ScAction::None;
  SeAction se = SeAction::None;
  HoldAction hold = HoldAction::None;
  int64_t holdNs = 0;             ///< Refresh: last held sample, Release: release sample time
};

class RcCommandLogic {
public:
  static constexpr int64_t kSeRefusedShowNs = 2000000000LL; ///< ctrl_flags SE_REFUSED stays set this long
  /// A hold starts zeroing only while its last held sample is at most this old (the zeroing grace, D11).
  static constexpr int64_t kHoldFreshNs = 300000000LL;

  /// Consumes the mapper's events and returns this tick's intents.
  RcIntents evaluate(RcCommandMapper &mapper, const RcPollInput &in);

  /// The caller started (ok) or could not start the hold zeroing that evaluate() asked for.
  void holdStartResult(bool ok, int64_t nowNs);
  /// An SE system reset was applied (or the vehicle reset otherwise): clears the refusal memory.
  void noteReset() { lastGoRefusal_ = GoRefusal::None; }
  /// A go the pure rules accepted was refused by a later gate (calibration, gyro run): kept for go_refusal.
  void noteGoRefused(GoRefusal r) { lastGoRefusal_ = r; }
  /// A press was refused for another reason found by the caller.
  void noteSeRefused(int64_t nowNs) { seRefusedUntilNs_ = nowNs + kSeRefusedShowNs; }

  GoRefusal lastGoRefusal() const { return lastGoRefusal_; }
  bool seRefusedActive(int64_t nowNs) const { return nowNs < seRefusedUntilNs_; }
  bool holdRunning() const { return holdRunning_; }

  /// SC differs from the applied mode (or is invalid / reserved) while a source is active: ctrl_flags SC_MISMATCH.
  static bool scMismatch(bool haveSource, uint8_t sc, bool simulationApplied) {
    if (!haveSource) return false;
    return !((sc == 0 && simulationApplied) || (sc == 1 && !simulationApplied));
  }

private:
  GoRefusal lastGoRefusal_ = GoRefusal::None;
  int64_t seRefusedUntilNs_ = 0;
  bool holdRunning_ = false;
  uint32_t runningSerial_ = 0;
  uint32_t handledSerial_ = 0;
};

} // namespace Safety
} // namespace VCTR

#endif
