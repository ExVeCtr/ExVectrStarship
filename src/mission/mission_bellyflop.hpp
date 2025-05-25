#ifndef MISSION_FREEFALL_HPP
#define MISSION_FREEFALL_HPP

#include "ExVectrCore/print.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/time_source.hpp"

#include "ExVectrMath.hpp"

#include "ExVectrControl/control_mapping_acctoatt.hpp"
#include "ExVectrControl/control_attitude_tvc.hpp"

#include "telemetry.hpp"

#include "mission_abstract.hpp"

#include "../starship_flaps.hpp"
#include "../starship_tvc.hpp"


namespace VCTR
{


class MissionBellyflop : public MissionAbstract, public Core::Task_Periodic
{
private:

    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;

    CTRL::ControlAttitudeTvc& controlTvc_; // Reference to the control TVC system to use for the mission.
    CTRL::ControlMappingAccToAtt& controlMapping_; // Reference to the control mapping system to use for the mission.

    int64_t transitionTimeLimit_ = 0; // If the transition takes longer than this, then consider it finished.
    float angleThreshold_Rad_ = 0; // Once the vehicle is within this angle of the belly down position, then we consider the transition finished.
    float angVelThreshold_RadPs_ = 0; // If the vehicle rotation is greater then this, then we consider the transition fast enough and we good.

    int64_t transitionStartTime_ = 0; // When the transition started.

public:

    /**
     * @brief Constructor for the MissionBellyflop class.
     * @param controlTvc Reference to the attitude control used to put the vehicle in belly down attitude.
     * @param controlMapping USed to revert back to original config once finished.
     * @param transitionTimeLimit If the transition takes longer than this, then consider it finished.
     * @param angleThreshold_Rad Once the vehicle is within this angle of the belly down position, then we consider the transition finished.
     * @param angVelThreshold_RadPs If the vehicle rotation is greater then this, then we consider the transition fast enough and we good.
     */
    MissionBellyflop(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic, CTRL::ControlAttitudeTvc& controlTvc, CTRL::ControlMappingAccToAtt& controlMapping, int64_t transitionTimeLimit, float angleThreshold_Rad, float angVelThreshold_RadPs) :
        Task_Periodic("Mission Freefall", 0.01*Core::SECONDS), controlTvc_(controlTvc), controlMapping_(controlMapping)
    {
        Core::getSystemScheduler().addTask(*this);
        // Subscribe to the topics
        attSubr_.subscribe(attTopic); // Subscribe to the attitude topic to get the current attitude of the vehicle
        
        disableKinematicSafety_ = true; // Disable the kinematic safety measures
        
    }

    bool missionEnd() override {
        return missionEnd_;
    }

    void taskInit() override
    {
        missionState_.missionMode = MissionMode::MissionMode_Idle; // Set the mission mode to idle
        missionEnd_ = true; // Set the mission end to false
    }

    const MissionState& getMissionState() const {
        return missionState_; // Get the mission state
    }

    void beginMission(int64_t startTime) override {
        missionState_.missionMode = MissionMode::MissionMode_Running;
        missionTime_.setTime(startTime); // Set the mission time to the start time
        actuatorsEnabled_ = true; // Disable actuators
        missionEnd_ = false; // Set the mission end to false
        LOG_MSG("Started mission transition\n"); // Log the mission start
        setPaused(false); // Unpause the task to start the mission
        controlTvc_.unsubscribeAttitudeSetpoint();
        transitionStartTime_ = Core::NOW(); // Set the time when the transition started
    };

    void resetMission() override {
        missionState_.missionMode = MissionMode::MissionMode_Idle;
        actuatorsEnabled_ = false; // Disable actuators
        missionEnd_ = true; // Set the mission end to true
        LOG_MSG("Reset mission freefall.\n"); // Log the mission reset
        setPaused(true); // Pause the task to stop the mission
        controlTvc_.subscribeAttitudeSetpoint(controlMapping_.getAttitudeTopic());
    }

    void taskThread() override 
    {

        auto attSet = Math::Quat_F(Math::Vector_F({0, 1, 0}), -90*DEGREES);
        controlTvc_.setAttitudeStateSetpoint({0, -90*DEGREES, attSet(0), attSet(1), attSet(2), attSet(3)}); // Set the attitude setpoint to belly down attitude with angular velocity to really kick it there.

        Math::Quat_F attIs = attSubr_.getItem().data.block<3, 1>(3, 0); // Get the current attitude of the vehicle
        auto velIs = attSubr_.getItem().data.block<3, 1>(0, 0); // Get the current velocity of the vehicle

        auto xAxis = attIs.rotate(Math::Vector_F({1, 0, 0})); // Get the vehicle x axis in world frame
        auto angle = xAxis.getAngleTo(Math::Vector_F({0, 0, 1})); // Get the angle between the vehicle x axis and the world z axis (Angle to belly down position)

        if (angle < angleThreshold_Rad_ && -velIs(1) > angVelThreshold_RadPs_ && Core::NOW() - transitionStartTime_ > transitionTimeLimit_) { // If the vehicle is within the angle threshold and the angular velocity is below the threshold, we consider the transition finished
            controlTvc_.subscribeAttitudeSetpoint(controlMapping_.getAttitudeTopic()); // Subscribe to the attitude setpoint topic to revert back to original configuration
            missionState_.missionMode = MissionMode::MissionMode_Finished; // Set the mission mode to finished
            missionEnd_ = true; // Set the mission end to true
            LOG_MSG("Transition finished\n"); // Log the mission end
            setPaused(true); // Pause the task to stop these calculations
        }

    }


private:


};


}







#endif