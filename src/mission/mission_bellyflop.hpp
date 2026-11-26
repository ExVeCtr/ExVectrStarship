#ifndef MISSION_BELLYFLOP_HPP
#define MISSION_BELLYFLOP_HPP

#include "../starship_flaps.hpp"
#include "../starship_tvc.hpp"
#include "ExVectrControl/control_attitude_tvc.hpp"
#include "ExVectrControl/control_mapping_acctoatt.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/time_source.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrMath.hpp"
#include "mission_abstract.hpp"
#include "telemetry.hpp"
#include "../safety/bellyflop_kick.hpp"

namespace VCTR {

class MissionBellyflop : public MissionAbstract, public Core::Task_Periodic {
private:
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;

  CTRL::ControlAttitudeTvc &controlTvc_;
  CTRL::ControlMappingAccToAtt &controlMapping_;

  int64_t transitionStartTime_ = 0;

  float kickTarget_[4] = {1, 0, 0, 0}; // pitch-over target, body -> world
  bool kickActive_ = false;

public:
  /**
   * @brief Bellyflop transition: the powered kick (docs/project/missions.md section 3). Timing, trigger thresholds,
   * pitch-over target and thrust limit are the named constants in src/safety/bellyflop_kick.hpp.
   * @param controlTvc TVC attitude controller; the mission writes the pitch-over setpoint into it directly.
   * @param controlMapping Its attitude setpoint topic is resubscribed in resetMission().
   */
  MissionBellyflop(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic,
                   CTRL::ControlAttitudeTvc &controlTvc, CTRL::ControlMappingAccToAtt &controlMapping)
      : Task_Periodic("Mission Bellyflop", 0.01 * Core::SECONDS), controlTvc_(controlTvc),
        controlMapping_(controlMapping) {
    Core::getSystemScheduler().addTask(*this);
    attSubr_.subscribe(attTopic); // attitude estimate [w(3), q(4)]

    disableKinematicSafety_ = true; // Disable the kinematic safety measures

    setPaused(true); // Pause the task initially
  }

  bool missionEnd() override { return missionEnd_; }

  // No taskInit override (2026-11-20). The task is paused from construction, so the scheduler runs taskInit on
  // its first unpaused tick, i.e. AFTER beginMission. The old taskInit set the mode to Idle there, so the first
  // bellyflop after boot reported Idle instead of Running and the mode-edge handling switched the controllers
  // differently from every later run. The constructor (MissionAbstract) already sets the boot mode Idle.

  const MissionState &getMissionState() const {
    return missionState_; // Get the mission state
  }

  void beginMission(int64_t startTime) override {
    missionState_.missionMode = MissionMode::MissionMode_Running;
    missionTime_.setTime(startTime); // Set the mission time to the start time
    actuatorsEnabled_ = true;        // TVC actuators on: motors stay on for the kick
    missionEnd_ = false;             // Set the mission end to false
    setPaused(false); // Unpause the task to start the mission
    controlTvc_.unsubscribeAttitudeSetpoint();
    transitionStartTime_ = Core::NowNs(); // Set the time when the transition started
    // Pitch-over target from the attitude now (heading kept, pure pitch about body +Y, Safety::kickTargetAttitude),
    // written to the TVC controller at once so its next run already uses it.
    float q0[4];
    const auto &att = attSubr_.getItem().data;
    for (int i = 0; i < 4; i++)
      q0[i] = att(3 + i);
    Safety::kickTargetAttitude(q0, Safety::kKickPitchAngleRad, kickTarget_);
    kickActive_ = true;
    writeKickSetpoint();
    LOG_MSG("Started bellyflop kick, target q %.3f %.3f %.3f %.3f\n", kickTarget_[0], kickTarget_[1], kickTarget_[2],
            kickTarget_[3]);
  };

  void resetMission() override {
    clearMissionState();
    actuatorsEnabled_ = false; // Disable actuators
    missionEnd_ = true;        // Set the mission end to true
    kickActive_ = false;
    LOG_MSG("Reset mission bellyflop transition.\n"); // Log the mission reset
    setPaused(true); // Pause the task to stop the mission
    controlTvc_.subscribeAttitudeSetpoint(controlMapping_.getAttitudeTopic());
  }

  /// The powered kick is running (mission Running, before an end trigger). Telemetry: setpoint_q source.
  bool kickActive() const { return kickActive_; }

  /// The kick's attitude setpoint (body -> world, [w,x,y,z]); valid while kickActive().
  void kickSetpoint(float out[4]) const {
    for (int i = 0; i < 4; i++)
      out[i] = kickTarget_[i];
  }

  void taskThread() override {
    writeKickSetpoint();

    float qIs[4];
    const auto &att = attSubr_.getItem().data;
    for (int i = 0; i < 4; i++)
      qIs[i] = att(3 + i);
    const float pitchRate = att(1); // body +Y rate
    const float remaining = Safety::kickRemainingAngle(qIs, kickTarget_);
    const int64_t elapsed = Core::NowNs() - transitionStartTime_;

    const Safety::KickEnd trigger = Safety::kickEndTrigger(elapsed, remaining, pitchRate);
    if (trigger == Safety::KickEnd::None)
      return;

    // End of the kick: motors to zero at once. The TVC controller is disabled here (its next run, <= 20 ms,
    // publishes the neutral command); the phase table confirms "off" on the next 10 Hz safety tick (mode
    // Finished) and the chain then switches to freefall (TVC actuators off, flap attitude control on). The TVC
    // attitude setpoint stays on the kick target until resetMission() resubscribes it to the mapping.
    controlTvc_.enableControl(false);
    kickActive_ = false;
    missionState_.missionMode = MissionMode::MissionMode_Finished;
    missionEnd_ = true;
    setPaused(true);
    static const char *const kNames[] = {"none", "angle", "rate", "time limit", "predicted"};
    LOG_MSG("Bellyflop kick finished (%s): remaining %.1f deg, pitch rate %.1f deg/s, %.3f s\n",
            kNames[uint8_t(trigger)], double(remaining / DEGREES), double(pitchRate / DEGREES),
            double(elapsed) / Core::SECONDS);
  }

private:
  void writeKickSetpoint() {
    controlTvc_.setAttitudeStateSetpoint({0, 0, 0, kickTarget_[0], kickTarget_[1], kickTarget_[2], kickTarget_[3]});
  }

};

} // namespace VCTR

#endif