#ifndef MISSION_ABSTRACT_HPP
#define MISSION_ABSTRACT_HPP


#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/time_source.hpp"

#include "../starship_flaps.hpp"
#include "../telemetry.hpp"

namespace VCTR
{


enum class MissionType : uint8_t
{
    MissionType_RTH = 0, // Return to home mission. This is the default standard mission that can be used as a fallback to save and return the vehicle.
    MissionType_Remote, // Remote mission. The vehicle setpoint is controlled over the datalink.
    MissionType_Waypoint, // Waypoint mission. This is used to fly a path to a certain point.
    MissionType_FlapDescent // Flap descent mission. This puts the vehicle belly first and uses the flaps to stabilise the vehicle.
};

class MissionAbstract
{
protected:

    Core::Topic<Math::Vector<float, 6>> positionSetpointTopic_;
    //Core::Topic<CTRL::StarshipFlapSettings> flapSettingTopic_;

    Core::Time_Source missionTime_;

    MissionState missionState_;

    MissionAbstract* nextMission_ = nullptr; // The next mission to run after this one is finished.

    bool actuatorsEnabled_ = false;

    //If true, then safety measures for leaving out of bound for position and attitude are disabled. This should only be used by missions that are highly dynamic and ran in safe areas.
    bool disableKinematicSafety_ = false;

    bool missionEnd_ = false; // If true, then the mission is finished and the vehicle should be in a safe state.


public:

    MissionAbstract()
    {
        missionState_.missionMode = MissionMode::MissionMode_Idle; // Set the mission mode to idle
        missionState_.missionTime = 0; // Set the mission time to 0
        missionState_.positionSetpoint[0] = 0; // Set the position setpoint to 0
        missionState_.positionSetpoint[1] = 0; // Set the position setpoint to 0
        missionState_.positionSetpoint[2] = 0; // Set the position setpoint to 0
        missionState_.positionSetpoint[3] = 0; // Set the position setpoint to 0
        missionState_.positionSetpoint[4] = 0; // Set the position setpoint to 0
        missionState_.positionSetpoint[5] = 0; // Set the position setpoint to 0
    }

    Core::Topic<Math::Vector<float, 6>>& getSetpointTopic() {
        return positionSetpointTopic_; // Get the setpoint topic
    }

    //Core::Topic<CTRL::StarshipFlapSettings>& getFlapSettingTopic() {
    //    return flapSettingTopic_; // Get the flap setting topic
    //}

    const MissionState& getMissionState() {
        return missionState_;
    }

    /**
     * @brief Current time of the mission.
     */
    Core::Time_Source& getMissionTime() {
        return missionTime_;
    }

    bool getActuatorsEnabled() {
        return actuatorsEnabled_;
    }

    bool getDisableKinematicSafety() {
        return disableKinematicSafety_;
    }

    virtual bool missionEnd() {
        return missionEnd_;
    }

    /**
     * A pointer to the mission that should come after this one. This is used to chain missions together.
     * @note returns nullptr if there is no next mission.
     */
    virtual MissionAbstract* nextMission() {
        return nextMission_; // Return the next mission
    }

    void setNextMission(MissionAbstract* nextMission) {
        nextMission_ = nextMission; // Set the next mission to run after this one is finished.
    }

    virtual void beginMission(int64_t startTime) {
        missionState_.missionMode = MissionMode::MissionMode_Idle;
        missionTime_.setTime(startTime); // Set the mission time to the start time
        actuatorsEnabled_ = false; // Disable actuators
        LOG_MSG("Attempted to start mission, but mission is not implemented.\n"); // Log the mission start
    };

    virtual void resetMission() {
        missionState_.missionMode = MissionMode::MissionMode_Idle;
        actuatorsEnabled_ = false; // Disable actuators
        LOG_MSG("Attempted to reset mission, but mission is not implemented.\n"); // Log the mission reset
    }


};

}


#endif // MISSION_ABSTRACT_HPP