#ifndef SAFETY_SIM_FLAP_FAULT_HPP
#define SAFETY_SIM_FLAP_FAULT_HPP

/**
 * Simulation-only flap servo fault injection (owner request and approval 2026-11-21, safety.md rules 7 and 9: another
 * owner-approved simulation-only exception, like the simulated wind).
 *
 * Purpose: reproduce the last real flight failure (top-left flap servo stuck at 40-45 deg in the descent) in the
 * simulated ship from the ground station. Pure logic: no Arduino, no ExVectrCore, no globals. Host-tested with
 * `pio test -e native -f test_sim_flap_fault`.
 *
 * Telecommands (legacy port 0x7F, paramInt == 0x57 magic, refused outside simulation mode):
 * - 16 SimFlapStuck: subsystem bits 0..1 = flap (0 TL, 1 TR, 2 BL, 3 BR, the telemetry ACTUATORS order), bit 4 (0x10) =
 *   arm for the descent (the fault takes effect when the bellyflop mission becomes the active mission) instead of
 *   immediately; paramFloat = stuck angle in rad clamped 0..pi/2, -1.0 = hold the TRUE angle at the moment the fault
 *   takes effect (first servo update after it).
 * - 17 SimFlapSlow: subsystem = flap, paramFloat = servo rate factor clamped 0.05..1.0.
 * - 18 SimFaultClear: clears all faults and armed faults.
 *
 * Guards (the fault can never touch real flight): (1) decideSimFault refuses unless simulationMode; (2) the model
 * methods take the CURRENT simulation flag and return the unmodified model value / factor 1.0 when it is false, so a
 * fault state that somehow exists is inert outside simulation; (3) SimFlapFaults::clear() is called on every
 * simulation mode switch and by the clear command. The faults only change the modelled TRUE flap angle (simulator
 * input and telemetry in simulation); the servo PWM outputs still follow the commanded angle.
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

constexpr int16_t kSimFaultMagic = 0x57;
constexpr uint8_t kSimFlapStuckType = 16;
constexpr uint8_t kSimFlapSlowType = 17;
constexpr uint8_t kSimFaultClearType = 18;
constexpr uint8_t kSimFaultArmBit = 0x10;
constexpr float kSimFlapMaxRad = 1.57079633f; ///< pi/2, the flap range is 0..pi/2
constexpr float kSimFlapSlowMinFactor = 0.05f;
constexpr float kSimFlapCaptureValue = -1.0f; ///< paramFloat of SimFlapStuck: stick at the current true angle

enum class SimFaultAction : uint8_t { Refuse = 0, Stuck, Slow, Clear };
enum class SimFaultRefusal : uint8_t { None = 0, NotSimulation, BadMagic, NotFinite, BadSubsystem, BadType };

struct SimFaultDecision {
  SimFaultAction action = SimFaultAction::Refuse;
  SimFaultRefusal reason = SimFaultRefusal::BadType;
  uint8_t flap = 0;
  bool armForDescent = false;
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

  /// Clears all active and armed faults. Called on every simulation mode switch and by SimFaultClear.
  void clear();

  /// The bellyflop (descent) mission became active: armed faults take effect. Returns how many took effect.
  /// Does nothing (and drops the armed faults) when simActive is false.
  int onDescentBegin(bool simActive);

  /// Modelled true angle of a flap after the servo model moved it: returns the held angle when stuck, else
  /// modelAngle unchanged. A capture fault takes modelAngle as its held angle at the first call after taking effect.
  /// Returns modelAngle bit-identical when simActive is false.
  float effectiveAngle(uint8_t flap, bool simActive, float modelAngle);

  /// Servo rate factor (1.0f exactly when simActive is false or no fault).
  float rateFactor(uint8_t flap, bool simActive) const;

  bool stuck(uint8_t flap) const { return flap < 4 && f_[flap].stuck; }
  bool armed(uint8_t flap) const { return flap < 4 && f_[flap].armed; }
  bool any() const;

private:
  struct Flap {
    bool stuck = false, capture = false;
    float angle = 0;
    float rate = 1.0f;
    bool armed = false, armCapture = false;
    float armAngle = 0;
  };
  Flap f_[4];
};

} // namespace Safety
} // namespace VCTR

#endif
