#ifndef SAFETY_SIM_FLAP_FAULT_HPP
#define SAFETY_SIM_FLAP_FAULT_HPP

/**
 * Simulation-only flap servo fault injection (owner request and approval 2026-11-21, safety.md rules 7 and 9: another
 * owner-approved simulation-only exception, like the simulated wind), and the modelled TRUE flap angle it acts on.
 *
 * Purpose: reproduce servo failures (the last real flight had the top-left flap servo stuck at 40-45 deg in the
 * descent) in the simulated ship from the ground station. Pure logic: no Arduino, no ExVectrCore, no globals.
 * Host-tested with `pio test -e native -f test_sim_flap_fault`.
 *
 * Telecommands (legacy port 0x7F, paramInt == 0x57 magic, refused outside simulation mode):
 * - 16 SimFlapStuck: subsystem = flap 0..3 (0 TL, 1 TR, 2 BL, 3 BR, the telemetry ACTUATORS order); any other bit is
 *   refused (bit 0x10, the removed "arm for the descent" mode, included). paramFloat = stuck angle in rad clamped
 *   0..pi/2, -1.0 = hold the TRUE angle it has when the fault is applied. Takes effect IMMEDIATELY (redesign
 *   2026-11-21, owner): the next StarshipFlaps tick (20 ms) publishes the forced angle.
 * - 17 SimFlapSlow: subsystem = flap, paramFloat = servo rate factor clamped 0.05..1.0.
 * - 18 SimFaultClear: clears all faults.
 *
 * ONE application point (redesign 2026-11-21): TrueFlapModel::publishAngles() is the modelled true flap angle that
 * StarshipFlaps publishes on the true flap topic, every tick, with the flaps enabled or not. Every consumer reads
 * that topic and nothing else: the body simulator's flap input (BodySimulator::setFlapSettingInputTopic) and the
 * telemetry flaps (TelemetryProvider -> TelemetrySnapshot.flaps -> ACTUATORS.flaps and AIR_ACTUATORS.flaps; the
 * ground station's FlapState is decoded from those). No controller or observer reads a flap angle (the flap
 * controller outputs commands only). The servo PWM outputs follow the COMMANDED angle outside simulation; in simulation the ServoOutputStage (below,
 * owner decision 2026-11-22) lets a stuck or slowed flap show on the physical servo as well.
 *
 * Guards (the fault can never touch real flight): (1) decideSimFault refuses unless simulationMode; (2) the model
 * methods take the CURRENT simulation flag and return the unmodified model value / factor 1.0 when it is false, so a
 * fault state that somehow exists is inert outside simulation; (3) ServoOutputStage reads the live flag on every call and returns the command bit-identical outside simulation;
 * (4) SimFlapFaults::clear() is called on every
 * simulation mode switch, by SystemReset and by the clear command.
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

constexpr int16_t kSimFaultMagic = 0x57;
constexpr uint8_t kSimFlapStuckType = 16;
constexpr uint8_t kSimFlapSlowType = 17;
constexpr uint8_t kSimFaultClearType = 18;
constexpr uint8_t kSimFaultFlapMask = 0x03; ///< the only subsystem bits accepted (flap index)
constexpr float kSimFlapMaxRad = 1.57079633f; ///< pi/2, the flap range is 0..pi/2
constexpr float kSimFlapSlowMinFactor = 0.05f;
constexpr float kSimFlapCaptureValue = -1.0f; ///< paramFloat of SimFlapStuck: stick at the current true angle

enum class SimFaultAction : uint8_t { Refuse = 0, Stuck, Slow, Clear };
enum class SimFaultRefusal : uint8_t { None = 0, NotSimulation, BadMagic, NotFinite, BadSubsystem, BadType };

struct SimFaultDecision {
  SimFaultAction action = SimFaultAction::Refuse;
  SimFaultRefusal reason = SimFaultRefusal::BadType;
  uint8_t flap = 0;
  bool captureCurrent = false;
  float value = 0; ///< stuck angle in rad (clamped) or rate factor (clamped); unused for capture and clear
};

const char *simFaultRefusalText(SimFaultRefusal r);

/// Pure decision for telecommand types 16..18: allowed, and with which validated values. Checked in this order:
/// simulation mode, magic, subsystem bits, finite value (not for Clear).
SimFaultDecision decideSimFault(uint8_t type, uint8_t subsystem, int16_t paramInt, float paramFloat,
                                bool simulationMode);

class SimFlapFaults {
public:
  /// Stores an accepted decision. Returns false (and changes nothing) when simActive is false or the decision is not
  /// an accepted one: a second guard behind decideSimFault.
  bool set(const SimFaultDecision &d, bool simActive);

  /// Clears all faults. Called on every simulation mode switch, by SimFaultClear and by SystemReset. A flap that was
  /// stuck is marked released (takeReleased) so the servo model can drop its forced angle.
  void clear();

  /// True once per flap that was stuck when clear() ran: the caller then snaps its servo model's angle to the commanded
  /// target (the model otherwise keeps the stuck angle and slews back, which only distorts telemetry for 0.3 s). Valid
  /// in and outside simulation, so a fault released by leaving simulation cannot leave a forced angle behind.
  bool takeReleased(uint8_t flap);

  /// Modelled true angle of a flap after the servo model moved it: returns the held angle when stuck, else
  /// modelAngle unchanged. A capture fault takes modelAngle as its held angle at the first call after it was set.
  /// Returns modelAngle bit-identical when simActive is false.
  float effectiveAngle(uint8_t flap, bool simActive, float modelAngle);

  /// Servo rate factor (1.0f exactly when simActive is false or no fault).
  float rateFactor(uint8_t flap, bool simActive) const;

  bool stuck(uint8_t flap) const { return flap < 4 && f_[flap].stuck; }

  /// Held angle of a stuck flap in rad (valid after effectiveAngle resolved a capture fault; 0 when not stuck).
  float stuckAngle(uint8_t flap) const { return (flap < 4 && f_[flap].stuck) ? f_[flap].angle : 0.0f; }
  /// True while a stuck flap still waits for its capture to be resolved by the next effectiveAngle().
  bool capturePending(uint8_t flap) const { return flap < 4 && f_[flap].stuck && f_[flap].capture; }
  bool any() const;

private:
  struct Flap {
    bool stuck = false, capture = false;
    float angle = 0;
    float rate = 1.0f;
  };
  Flap f_[4];
  bool released_[4] = {false, false, false, false};
};

/**
 * Modelled ("true") flap angles of StarshipFlaps: one slew-rate servo model per flap (generic 9 g servo) plus the
 * simulation faults, applied at one point. Order per StarshipFlaps tick:
 *  - flaps powered (enabled): setTargets(clamped commands), then advance(dt) moves each angle toward its target at
 *    rate x rateFactor;
 *  - flaps not powered: nothing moves (an unpowered servo stays where it is);
 *  - always: publishAngles() applies released/stuck faults and returns the angles StarshipFlaps publishes.
 */
class TrueFlapModel {
public:
  explicit TrueFlapModel(float rateRadps = 5.23599f) : rate_(rateRadps) {}

  void setRate(float rateRadps) { rate_ = rateRadps; }
  float rate() const { return rate_; }

  void setTargets(const float target[4]) {
    for (int i = 0; i < 4; i++)
      target_[i] = target[i];
  }

  /// Moves each angle toward its target by at most rate x rateFactor x dt (dt in ns).
  void advance(int64_t dtNs, const SimFlapFaults &faults, bool simActive);

  /// THE application point of the faults: snaps released flaps to their target, forces stuck flaps to the held
  /// angle (also into the servo state, so the servo continues from there when released), and writes the angles to
  /// publish into out[4]. Call every tick, powered or not.
  void publishAngles(SimFlapFaults &faults, bool simActive, float out[4]);

  float angle(int i) const { return angle_[i]; }
  float target(int i) const { return target_[i]; }

private:
  float rate_;
  float angle_[4] = {0, 0, 0, 0};
  float target_[4] = {0, 0, 0, 0};
};

/**
 * Servo PWM command stage (owner decision 2026-11-22): the ONE place where the commanded flap angle (rad) becomes the
 * angle that is converted to PWM. Outside simulation mode, or when no fault is active on the flap, it returns the
 * commanded angle bit-identical. In simulation mode only:
 *  - stuck flap: the held (forced) angle, clamped to [minRad, maxRad], for any commanded angle;
 *  - slow flap: the output slews toward the commanded angle at normalRate x factor (the same rate the TrueFlapModel
 *    uses), starting from the previous output;
 *  - release: the output slews back to the commanded angle at the NORMAL rate (no jump), then returns to the exact
 *    commanded value.
 * The live simulation flag is an argument read by the caller on EVERY call; nothing about it is cached. Turning
 * simulation off returns the commanded angle at the next call. Motors are not part of this class.
 */
class ServoOutputStage {
public:
  /// Pure per-call function. Call after TrueFlapModel::publishAngles() in the same tick (a capture fault is resolved
  /// there). dtNs = time since the previous call.
  float servoCommandRad(SimFlapFaults &faults, uint8_t flap, float commandedRad, bool simulationNow, int64_t dtNs,
                        float normalRateRadps, float minRad, float maxRad);

  /// Forget the output history (flap unpowered): the next call starts from the commanded angle.
  void reset() {
    for (uint8_t i = 0; i < 4; i++)
      valid_[i] = engaged_[i] = false;
  }

private:
  float last_[4] = {0, 0, 0, 0};
  bool valid_[4] = {false, false, false, false};   ///< last_ holds the previous output
  bool engaged_[4] = {false, false, false, false}; ///< output differs from the command (stuck, slowed or slewing back)
};

} // namespace Safety
} // namespace VCTR

#endif
