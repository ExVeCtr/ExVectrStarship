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

namespace VCTR {

class MissionBellyflop : public MissionAbstract, public Core::Task_Periodic {
private:
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;

  CTRL::ControlAttitudeTvc &controlTvc_;
  CTRL::ControlMappingAccToAtt &controlMapping_;

  int64_t transitionTimeLimit_ = 0;
  float angleThreshold_Rad_ = 0;
  float angVelThreshold_RadPs_ = 0;

  int64_t transitionStartTime_ = 0;

public:
  /**
   * @brief Constructor for the MissionBellyflop class.
   * @param controlTvc Reference to the attitude control used to put the vehicle
   * in belly down attitude.
   * @param controlMapping USed to revert back to original config once finished.
   * @param transitionTimeLimit If the transition takes longer than this, then
   * consider it finished.
   * @param angleThreshold_Rad Once the vehicle is within this angle of the
   * belly down position, then we consider the transition finished.
   * @param angVelThreshold_RadPs If the vehicle rotation is greater then this,
   * then we consider the transition fast enough and we good.
   */
  MissionBellyflop(
      Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic,
      CTRL::ControlAttitudeTvc &controlTvc,
      CTRL::ControlMappingAccToAtt &controlMapping, int64_t transitionTimeLimit,
      float angleThreshold_Rad, float angVelThreshold_RadPs)
      : Task_Periodic("Mission Freefall", 0.01 * Core::SECONDS),
        controlTvc_(controlTvc), controlMapping_(controlMapping) {
    Core::getSystemScheduler().addTask(*this);
    // Subscribe to the topics
    attSubr_.subscribe(attTopic); // Subscribe to the attitude topic to get the
                                  // current attitude of the vehicle
    transitionTimeLimit_ = transitionTimeLimit; // Set the transition time limit
    angleThreshold_Rad_ =
        angleThreshold_Rad; // Set the angle threshold in radians
    angVelThreshold_RadPs_ =
        angVelThreshold_RadPs; // Set the angular velocity threshold in radians
                               // per second

    disableKinematicSafety_ = true; // Disable the kinematic safety measures

    setPaused(true); // Pause the task initially
  }

  bool missionEnd() override { return missionEnd_; }

  void taskInit() override {
    missionState_.missionMode =
        MissionMode::MissionMode_Idle; // Set the mission mode to idle
  }

  const MissionState &getMissionState() const {
    return missionState_; // Get the mission state
  }

  void beginMission(int64_t startTime) override {
    missionState_.missionMode = MissionMode::MissionMode_Running;
    missionTime_.setTime(startTime); // Set the mission time to the start time
    actuatorsEnabled_ = true;        // Disable actuators
    missionEnd_ = false;             // Set the mission end to false
    LOG_MSG("Started mission bellyflop transition\n"); // Log the mission start
    setPaused(false); // Unpause the task to start the mission
    controlTvc_.unsubscribeAttitudeSetpoint();
    transitionStartTime_ =
        Core::NOW(); // Set the time when the transition started
  };

  void resetMission() override {
    missionState_.missionMode = MissionMode::MissionMode_Idle;
    actuatorsEnabled_ = false; // Disable actuators
    missionEnd_ = true;        // Set the mission end to true
    LOG_MSG("Reset mission bellyflop transition.\n"); // Log the mission reset
    setPaused(true); // Pause the task to stop the mission
    controlTvc_.subscribeAttitudeSetpoint(controlMapping_.getAttitudeTopic());
  }

  void taskThread() override {
    auto attSet = Math::Quat_F(Math::Vector_F({0, 1, 0}), -90 * DEGREES);
    controlTvc_.setAttitudeStateSetpoint(
        {0, 0, 0, attSet(0), attSet(1), attSet(2), attSet(3)});

    Math::Quat_F attIs = attSubr_.getItem().data.block<4, 1>(3, 0);
    auto velIs = attSubr_.getItem().data.block<3, 1>(0, 0);

    auto xAxis = attIs.conjugate().rotate(Math::Vector_F{1, 0, 0});
    auto angle = xAxis.getAngleTo(Math::Vector_F{0, 0, -1});

    LOG_MSG("Bellyflop debug: att: %.2f %.2f %.2f %.2f, xAxis: %.2f %.2f %.2f, "
            "angle: %.2f deg, angVel: "
            "%.2f deg/s\n",
            attIs(0), attIs(1), attIs(2), attIs(3), xAxis(0), xAxis(1),
            xAxis(2), angle / DEGREES, velIs(1) / DEGREES);

    bool stopTrigger = false;
    auto angleTrigger = angle < angleThreshold_Rad_;
    auto angVelTrigger = abs(velIs(1)) > angVelThreshold_RadPs_;
    auto timeTrigger =
        Core::NOW() - transitionStartTime_ > transitionTimeLimit_;
    if (angleTrigger) {
      LOG_MSG("Bellyflop Transition finished. Trigger was angle\n");
      stopTrigger = true;
    } else if (angVelTrigger) {
      LOG_MSG("Bellyflop Transition finished. Trigger was angular velocity\n");
      stopTrigger = true;
    } else if (timeTrigger) {
      LOG_MSG("Bellyflop Transition finished. Trigger was time limit\n");
      stopTrigger = true;
    }
    if (stopTrigger) {
      controlTvc_.subscribeAttitudeSetpoint(controlMapping_.getAttitudeTopic());
      missionState_.missionMode = MissionMode::MissionMode_Finished;
      missionEnd_ = true;
      LOG_MSG(
          "Transition finished. State: %.2f deg, angular velocity: %.2f deg/s, "
          "Time since start: %.2f s\n",
          angle / DEGREES, velIs(1) / DEGREES,
          double(Core::NOW() - transitionStartTime_) /
              Core::SECONDS); // Log the transition finished
      setPaused(true);        // Pause the task to stop these calculations
    }
  }

private:
};

} // namespace VCTR

#endif