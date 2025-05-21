#ifndef MISSION_FREEFALL_HPP
#define MISSION_FREEFALL_HPP


#include "ExVectrCore/print.hpp"
#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/time_source.hpp"

#include "ExVectrControl/control_rocket.hpp"

#include "telemetry.hpp"

#include "mission_abstract.hpp"

#include "../starship_flaps.hpp"
#include "../starship_tvc.hpp"


namespace VCTR
{


class MissionFreefall : public MissionAbstract, public Core::Task_Periodic
{
private:

    Core::Simple_Subscriber<Math::Vector<float, 4>> ctrlSubr_;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;

    Math::Vector<float, 6> positionIs_;

    float vehicleMass_kg_ = 1; // Mass of the vehicle in kg.
    float tvcThrustLimit_N_ = 15; // Maximum thrust in Newtons.
    float stopAlt_ = 0; // How much above the ground to fully stop the vehicle.


public:

    MissionFreefall(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic, Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic, float vehicleMass_kg, float tvcThrustLimit_N, float stopAlt) :
        Task_Periodic("Mission Freefall", 0.01*Core::SECONDS)
    {
        // Subscribe to the topics
        attSubr_.subscribe(attTopic);
        posSubr_.subscribe(posTopic);
        vehicleMass_kg_ = vehicleMass_kg; // Set the vehicle mass
        tvcThrustLimit_N_ = tvcThrustLimit_N; // Set the thrust limit
        Core::getSystemScheduler().addTask(*this);
        disableKinematicSafety_ = true; // Disable the kinematic safety measures
        //setPriority(500);
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
        actuatorsEnabled_ = false; // Disable actuators
        missionEnd_ = false; // Set the mission end to false
        LOG_MSG("Started mission freefall\n"); // Log the mission start
    };

    void resetMission() override {
        missionState_.missionMode = MissionMode::MissionMode_Idle;
        actuatorsEnabled_ = false; // Disable actuators
        missionEnd_ = true; // Set the mission end to true
        LOG_MSG("Reset mission freefall.\n"); // Log the mission reset
    }

    void taskThread() override 
    {

        if (missionEnd_)
            return; // If the mission has ended, do nothing

        if (posSubr_.isDataNew()) {
            positionIs_ = posSubr_.getItem().data;
        }

        actuatorsEnabled_ = false;

        switch (missionState_.missionMode)
        {
        case MissionMode::MissionMode_Idle:
            //actuatorsEnabled_ = false;
            //missionEnd_ = true; // Set the mission end to true
            break;

        case MissionMode::MissionMode_Initialisation:
            //missionEnd_ = false; // Set the mission end to true
            break;

        case MissionMode::MissionMode_Startup:
            //missionEnd_ = false; // Set the mission end to true
            break;
        
        case MissionMode::MissionMode_Running:
            //missionEnd_ = false; // Set the mission end to true
            break;
        
        default:
            missionState_.missionMode = MissionMode::MissionMode_Idle;
            break;
        }

        float stoppingDistance = calculateStoppingDistance(tvcThrustLimit_N_, vehicleMass_kg_, positionIs_); // Calculate the stopping distance

        if (positionIs_(5) < stoppingDistance + stopAlt_ && missionEnd_ == false) { // If the stopping distance is less than the Z position
            missionState_.missionMode = MissionMode::MissionMode_Finished; // Set the mission mode to finished
            missionEnd_ = true; // Set the mission end to true
            // Now we simply trust the next mission to take care of the rest. (Jesus take the wheel)
        }

        LOG_MSG("Freefall mode. Alt: %f, Stopping distance: %f\n", positionIs_(5), stoppingDistance); // Log the mission start

        missionState_.missionMode = missionState_.missionMode;
        missionState_.missionTime = missionTime_.NOW();
        missionState_.positionSetpoint[0] = 0;
        missionState_.positionSetpoint[1] = 0;
        missionState_.positionSetpoint[2] = 0;
        missionState_.positionSetpoint[3] = 0;
        missionState_.positionSetpoint[4] = 0;    
        missionState_.positionSetpoint[5] = 0;
        positionSetpointTopic_.publish({0, 0, 0, 0, 0, 0}); // Publish the setpoint to the control system

        CTRL::StarshipFlapSettings flapSettings_;
        flapSettings_.blAngle = 35*3.14/180; // Retract bottom flaps, extend top flaps
        flapSettings_.brAngle = 35*3.14/180;
        flapSettings_.tlAngle = 55*3.14/180;
        flapSettings_.trAngle = 55*3.14/180;
        flapSettings_.enableActuators = true; // Enable actuators
        flapSettingTopic_.publish(flapSettings_); // Publish the flap settings to the control systems

        disableKinematicSafety_ = true; // Disable the kinematic safety measures


        //missionTimeTopic.publish(missionTime_.NOW()); // Publish the mission time to the control system

    }


private:

    float calculateStoppingDistance(float thrust, float mass, Math::Vector<float, 6> position) {

        //float& pz = position(5); // Get the Z position
        float& vz = position(2); // Get the Z velocity

        float aSum = thrust / mass - 9.81; // Calculate the acceleration sum

        return 0.5 * vz*vz/aSum; // Return the stopping distance

    }



};


}







#endif