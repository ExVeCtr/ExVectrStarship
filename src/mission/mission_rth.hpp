#ifndef MISSION_RETURNTOHOME_HPP
#define MISSION_RETURNTOHOME_HPP

#include "../starship_flaps.hpp"
#include "../telemetry.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/time_source.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "mission_abstract.hpp"

namespace VCTR {

/**
 * @brief This mission is used to return the vehicle to the home position. This
 * is used for example is emergencies to have the vehicle in a safe state.
 */
class MissionRTH : public MissionAbstract, public Core::Task_Periodic {
private:
  enum class RunningState {
    RunningState_Stabilize =
        0, // Stop all movement and tries to stabilize the vehicle.
    RunningState_Ascent,      // Ascend to the translation altitude.
    RunningState_Translation, // Translate to the home position.
    RunningState_Descent,     // Descent to the home position.
  };

  RunningState runningState_ =
      RunningState::RunningState_Stabilize; // The current running state of the
                                            // vehicle.

  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 2>>>
      groundSubr_;
  Core::Simple_Subscriber<Core::Timestamped<Math::Matrix<float, 2, 2>>>
      groundCovSubr_;
  Math::Vector<float, 6> positionIs_;

  Math::Vector<float, 6> positionSetpoint_;

  // Core::Topic<CTRL::StarshipFlapSettings> flapSettingTopic_;
  // CTRL::StarshipFlapSettings flapSettings_;

  Math::Vector<float, 3> homePosition_ = {
      0, 0, 1}; // The home position of the vehicle in reference frame.
  float translationVelocity_ =
      2; // The translation velocity of the vehicle in m/s.

  int64_t hoverModeLastUpdate_ = 0;

  float landingThresholdDistance_ =
      0.2; // If the vehicle set position is further down than this for a
           // certain time, it will be considered as landed and the mission
           // will be finished.
  int64_t landingThresholdTime_ =
      2 * Core::SECONDS;    // How long the vehicle has to be below the landing
                            // threshold distance to be considered as landed.
  float descentRate_ = 0.2; // How fast the vehicle should go down in m/s.
  int64_t landingThresMetTime_ =
      0; // When the vehicle is below the landing threshold distance.
  int64_t descentModeTimestamp_ = 0; // When the vehicle is in descent mode.

public:
  MissionRTH(
      Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic,
      Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic,
      Core::Topic<Core::Timestamped<Math::Vector<float, 2>>> &groundTopic,
      Core::Topic<Core::Timestamped<Math::Matrix<float, 2, 2>>> &groundCovTopic,
      const Math::Vector<float, 3> &homePosition = {0, 0, 0})
      : Task_Periodic("Mission RTH", 0.1 * Core::SECONDS) {
    // Subscribe to the topics
    attSubr_.subscribe(attTopic);
    posSubr_.subscribe(posTopic);
    groundSubr_.subscribe(groundTopic);
    groundCovSubr_.subscribe(groundCovTopic);
    homePosition_ = homePosition; // Set the home position to the given position
    // homePosition_(2) += 1; // Set the home position to the given position +
    // 1m in z axis
    disableKinematicSafety_ = true; // Disable the kinematic safety measures
    Core::getSystemScheduler().addTask(*this);
    // setPriority(500);
  }

  void setHomePosition(const Math::Vector<float, 3> &homePosition) {
    homePosition_ = homePosition; // Set the home position to the given position
    // homePosition_(2) += 1; // Set the home position to the given position +
    // 1m in z axis
  }

  bool missionEnd() override { return missionEnd_; }

  bool stabilising() const {
    return runningState_ ==
           RunningState::RunningState_Stabilize; // Check if the vehicle is
                                                 // stabilising
  }

  bool translation() const {
    return runningState_ ==
           RunningState::RunningState_Translation; // Check if the vehicle is
                                                   // translating
  }

  bool descent() const {
    return runningState_ ==
           RunningState::RunningState_Descent; // Check if the vehicle is
                                               // descending
  }

  MissionAbstract *nextMission() override {
    return nullptr; // Return the next mission
  }

  void taskInit() override {
    missionState_.missionMode =
        MissionMode::MissionMode_Idle; // Set the mission mode to idle
  }

  const MissionState &getMissionState() const {
    return missionState_; // Get the mission state
  }

  void beginMission(int64_t startTime) override {
    missionState_.missionMode = MissionMode::MissionMode_Startup;
    missionTime_.setTime(0);  // Set the mission time to the start time
    actuatorsEnabled_ = true; // Actuators must be enabled immediately to keep
                              // the vehicle under control
    missionEnd_ = false;      // Set the mission end to false
    positionSetpoint_ =
        posSubr_.getItem()
            .data; // Set the position setpoint to where the vehicle is
    positionSetpoint_(0) = 0; // Set the velocity setpoint to 0
    positionSetpoint_(1) = 0;
    positionSetpoint_(2) = 0;
    LOG_MSG("Started mission ReturnToHome. Hot starting the mission "
            "immedietly\n"); // Log the mission start
    missionStartup();
  };

  void resetMission() override {
    missionState_.missionMode = MissionMode::MissionMode_Idle;
    actuatorsEnabled_ = false;                // Disable actuators
    missionEnd_ = true;                       // Set the mission end to true
    LOG_MSG("Reset mission ReturnToHome.\n"); // Log the mission reset
  }

  void taskThread() override {
    if (posSubr_.isDataNew()) {
      positionIs_ = posSubr_.getItem().data;
    }

    switch (missionState_.missionMode) {
    case MissionMode::MissionMode_Idle:
      missionIdle();
      break;

    case MissionMode::MissionMode_Initialisation:
      missionInit();
      break;

    case MissionMode::MissionMode_Startup:
      missionStartup();
      break;

    case MissionMode::MissionMode_Running:
      missionHover();
      break;

    case MissionMode::MissionMode_Finished:
      missionLanded();
      break;

    default:
      missionState_.missionMode = MissionMode::MissionMode_Idle;
      break;
    }

    missionState_.missionMode = missionState_.missionMode;
    missionState_.missionTime = missionTime_.NOW();
    missionState_.positionSetpoint[0] = positionSetpoint_(0);
    missionState_.positionSetpoint[1] = positionSetpoint_(1);
    missionState_.positionSetpoint[2] = positionSetpoint_(2);
    missionState_.positionSetpoint[3] = positionSetpoint_(3);
    missionState_.positionSetpoint[4] = positionSetpoint_(4);
    missionState_.positionSetpoint[5] = positionSetpoint_(5);

    positionSetpointTopic_.publish(
        positionSetpoint_); // Publish the setpoint to the control system

    // flapSettingTopic_.publish(flapSettings_); // Publish the flap settings to
    // the control system

    // missionTimeTopic.publish(missionTime_.NOW()); // Publish the mission time
    // to the control system
  }

private:
  void missionIdle() {
    actuatorsEnabled_ = false; // Disable actuators
    missionTime_.setTime(0);   // Set the mission time to the start time
    missionEnd_ = true;        // Set the mission end to false
  }

  void missionInit() {
    actuatorsEnabled_ = false; // Disable actuators
    missionEnd_ = false;       // Set the mission end to false
  }

  void missionStartup() {
    actuatorsEnabled_ = true; // Enable actuators
    missionEnd_ = false;      // Set the mission end to false

    // flapSettings_.enableActuators = true; // Enable the actuators
    // flapSettings_.tlAngle = 0; // Move top flaps out fully
    // flapSettings_.trAngle = 0;
    // flapSettings_.blAngle = 90;// Move bottom flaps in fully
    // flapSettings_.brAngle = 90;

    missionState_.missionMode =
        MissionMode::MissionMode_Running;  // Go to hover mode
    missionState_.positionSetpoint[0] = 0; // Set the velocity setpoint to 0
    missionState_.positionSetpoint[1] = 0;
    missionState_.positionSetpoint[2] = 0;
    missionState_.positionSetpoint[3] = posSubr_.getItem().data(
        3); // Set the position setpoint to where the vehicle is
    missionState_.positionSetpoint[4] = posSubr_.getItem().data(4);
    missionState_.positionSetpoint[5] = posSubr_.getItem().data(5);

    // LOG_MSG("Started mission ReturnToHome. Position: (%f, %f, %f)\n",
    // posSubr_.getItem().data(3), posSubr_.getItem().data(4),
    // posSubr_.getItem().data(5)); // Log the mission start

    hoverModeLastUpdate_ =
        Core::NOW(); // Set the time when the hover mode was last updated

    runningState_ = RunningState::RunningState_Stabilize; // Set the running
                                                          // state to stabilise
  }

  void missionHover() {
    float dTime = float(Core::NOW() - hoverModeLastUpdate_) / Core::SECONDS;
    hoverModeLastUpdate_ =
        Core::NOW(); // Set the time when the hover mode was last updated

    actuatorsEnabled_ = true;
    missionEnd_ = false; // Set the mission end to false

    switch (runningState_) {
    case RunningState::RunningState_Stabilize:

      positionSetpoint_(0) = 0;
      positionSetpoint_(1) = 0;
      positionSetpoint_(2) = 0;
      positionSetpoint_(3) = posSubr_.getItem().data(3);
      positionSetpoint_(4) = posSubr_.getItem().data(4);
      positionSetpoint_(5) = posSubr_.getItem().data(5);

      // flapSettings_.enableActuators = true; // Enable the actuators
      // flapSettings_.tlAngle = 0; // Move top flaps out fully
      // flapSettings_.trAngle = 0;
      // flapSettings_.blAngle = 90;// Move bottom flaps in fully
      // flapSettings_.brAngle = 90;

      if (posSubr_.getItem().data.block<3, 1>(0, 0).magnitude() <
          0.5) { // If the vehicle is slower than 0.5 m/s, we consider it as
                 // stopped, and begin translation
        runningState_ = RunningState::RunningState_Ascent; // Go to ascent mode
        translationVelocity_ = 0;
      }

      break;

    case RunningState::RunningState_Ascent:
      runningState_ =
          RunningState::RunningState_Translation; // Go to translation mode
      break;

    case RunningState::RunningState_Translation:

    {
      auto travelDistance =
          homePosition_ -
          positionSetpoint_.block<3, 1>(3); // Get the velocity vector to the
                                            // home position in reference frame
      auto travelDistanceMag =
          travelDistance.magnitude(); // Get the magnitude of the travel
                                      // distance vector
      auto travelDistanceNorm =
          travelDistance /
          travelDistanceMag; // Normalize the travel distance vector

      const float accel = 0.5;
      if (translationVelocity_ < 2)
        translationVelocity_ += dTime * accel;

      if (translationVelocity_ > 2) {
        translationVelocity_ = 2;
      }

      if (travelDistanceMag / dTime > translationVelocity_) {
        travelDistance = travelDistanceNorm * translationVelocity_ * dTime;
      }

      auto accelVelocityLimit = sqrtf(travelDistanceMag * accel);
      if (translationVelocity_ > accelVelocityLimit) {
        translationVelocity_ = accelVelocityLimit;
      }
      positionSetpoint_(0) = travelDistanceNorm(0) * translationVelocity_;
      positionSetpoint_(1) = travelDistanceNorm(1) * translationVelocity_;
      positionSetpoint_(2) = travelDistanceNorm(2) * translationVelocity_;
      positionSetpoint_(3) += travelDistance(
          0); // Update the setpoint position in the reference frame
      positionSetpoint_(4) += travelDistance(1);
      positionSetpoint_(5) += travelDistance(2);

      // flapSettings_.enableActuators = true; // Enable the actuators
      // flapSettings_.tlAngle = 90; // Move top flaps out fully
      // flapSettings_.trAngle = 90;
      // flapSettings_.blAngle = 90;// Move bottom flaps in fully
      // flapSettings_.brAngle = 90;

      auto distance =
          homePosition_ -
          positionIs_.block<3, 1>(
              3); // Get the distance to the home position in reference frame

      // LOG_MSG("Translation mode. Position: (%f, %f, %f)\n",
      // positionSetpoint_(3), positionSetpoint_(4), positionSetpoint_(5)); //
      // Log the mission start LOG_MSG("Distance to home position: (%f, %f,
      // %f)\n", distance(0), distance(1), distance(2)); // Log the mission
      // start

      if (distance.magnitude() < 0.5) { // If the vehicle is within 0.5 m of the
                                        // home position, we begin descent
        runningState_ =
            RunningState::RunningState_Descent; // Go to descent mode
        descentModeTimestamp_ =
            Core::NOW(); // Set the time when the descent mode was started
        positionSetpoint_(5) =
            positionIs_(5); // Set the setpoint position in the reference
                            // frame to the current position
        // missionState_.missionMode = MissionMode::MissionMode_Descent; // Go
        // to hover mode
      }
    }

    break;

    case RunningState::RunningState_Descent:
      missionDescent();
      break;

    default:
      runningState_ =
          RunningState::RunningState_Stabilize; // Go to stabilise mode
      break;
    }
  }

  void missionDescent() {
    float dTime = float(Core::NOW() - descentModeTimestamp_) / Core::SECONDS;
    descentModeTimestamp_ =
        Core::NOW(); // Set the time when the descent mode was last updated

    actuatorsEnabled_ = true;
    missionEnd_ = false; // Set the mission end to falses

    // flapSettings_.enableActuators = true; // Enable the actuators
    // flapSettings_.tlAngle = 90; // Move top flaps out fully
    // flapSettings_.trAngle = 90;
    // flapSettings_.blAngle = 90;// Move bottom flaps in fully
    // flapSettings_.brAngle = 90;

    positionSetpoint_(0) = 0;
    positionSetpoint_(1) = 0;
    positionSetpoint_(2) = 0;
    positionSetpoint_(3) = homePosition_(0);
    positionSetpoint_(4) = homePosition_(1);

    auto groundDistance = groundSubr_.getItem().data(1);
    auto groundCov = groundCovSubr_.getItem().data(1, 1);
    auto groundLastUpdate = groundSubr_.getItem().timestamp;
    LOG_MSG("Ground distance: %.2f, Covariance: %.2f\n", groundDistance,
            groundCov); // Log the ground distance and covariance

    if (positionIs_(5) - positionSetpoint_(5) <
        landingThresholdDistance_ *
            1.5) { // We keep updateting the threshold time. We stop when the
                   // vehicle is above the threshold distance. This triggers
                   // the start of the timer.
      // positionSetpoint_(2) = descentRate_;
      positionSetpoint_(5) -=
          dTime *
          descentRate_; // Update the setpoint position in the reference frame
    }
    if (positionIs_(5) - positionSetpoint_(5) <
        landingThresholdDistance_) { // We keep updateting the threshold time.
                                     // We stop when the vehicle is above the
                                     // threshold distance. This triggers the
                                     // start of the timer.
      landingThresMetTime_ =
          Core::NOW(); // Set the time when the landing threshold was met
    }

    if (Core::NOW() - landingThresMetTime_ >
        landingThresholdTime_) { // If the vehicle is below the landing
                                 // threshold distance for a certain time, we
                                 // consider it as landed.
      missionState_.missionMode =
          MissionMode::MissionMode_Finished; // Go to landed mode
    }

    if (groundDistance < 0.04 && groundDistance > -0.02 && groundCov < 7 &&
        Core::NOW() - groundLastUpdate <
            100 * Core::MILLISECONDS) // If the ground distance is less than 4
                                      // cm and is confident, we consider it as
                                      // landed.
    {
      missionState_.missionMode =
          MissionMode::MissionMode_Finished; // Go to landed mode
    }

    // LOG_MSG("Descent mode. Position: (%f, %f, %f)\n", positionSetpoint_(3),
    // positionSetpoint_(4), positionSetpoint_(5)); // Log the mission start
  }

  void missionLanded() {
    actuatorsEnabled_ = false;
    missionEnd_ = true; // Set the mission end to false

    // flapSettings_.enableActuators = false; // Enable the actuators

    actuatorsEnabled_ = false; // Disable actuators
    positionSetpoint_ = {0, 0, 0, 0,
                         0, 0}; // Set the setpoint to the current position and
                                // velocity of the vehicle

    // LOG_MSG("Landed mode. Position: (%f, %f, %f)\n", positionSetpoint_(3),
    // positionSetpoint_(4), positionSetpoint_(5)); // Log the mission start
  }
};

} // namespace VCTR

#endif // MISSION_ABSTRACT_HPP