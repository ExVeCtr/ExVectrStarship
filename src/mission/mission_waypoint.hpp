#ifndef MISSION_WAYPOINT_HPP
#define MISSION_WAYPOINT_HPP


#include "ExVectrCore/print.hpp"

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/timestamped.hpp"
#include "ExVectrCore/time_definitions.hpp"
#include "ExVectrCore/time_source.hpp"
#include "ExVectrCore/task_types.hpp"

#include "../telemetry.hpp"
#include "../starship_flaps.hpp"

#include "mission_abstract.hpp"


namespace VCTR
{

/**
 * @brief This mission is used to return the vehicle to the home position. This is used for example is emergencies to have the vehicle in a safe state.
 */
class MissionWaypoint : public MissionAbstract, public Core::Task_Periodic
{
private:

    struct Waypoint
    {
        Math::Vector<float, 3> position; // The position of the waypoint to travel to.
        float velocity; // The velocity to move to the waypoint in m/s.
        float thresholdDistance; // When the waypoint can be considered as reached if within this distance.
        int64_t loiterTime; // The time the vehicle should loiter at the waypoint in seconds.
    };

    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;
    Math::Vector<float, 6> positionIs_;

    Core::Topic<Math::Vector<float, 6>> positionSetpointTopic_;
    Math::Vector<float, 6> positionSetpoint_;

    Core::Topic<CTRL::StarshipFlapSettings> flapSettingTopic_;
    CTRL::StarshipFlapSettings flapSettings_;

    int64_t hoverModeLastUpdate_ = 0; 

    Core::ListArray<Waypoint> waypoints_; // The list of waypoints to travel to.
    size_t currentWaypointIndex_ = 0; // The index of the current waypoint to travel to.
    int64_t waypointThresholdTime_ = 0; // The time the vehicle should be within the threshold distance to the waypoint to be considered as reached.
    
public:

    MissionWaypoint(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic, Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic) : 
        Task_Periodic("Mission RTH", 0.1*Core::SECONDS)
    {
        // Subscribe to the topics
        attSubr_.subscribe(attTopic);
        posSubr_.subscribe(posTopic);
        Core::getSystemScheduler().addTask(*this);
        //setPriority(500);
    }

    void setSetpointOutput(Core::Subscriber<Math::Vector<float, 6>> &setpointSubscriber) {
        setpointSubscriber.subscribe(positionSetpointTopic_); // Subscribe to the setpoint topic
    }

    void setNextMission(MissionAbstract* nextMission) {
        nextMission_ = nextMission; // Set the next mission to run after this one is finished.
    }

    /**
     * Adds a waypoint to the list of waypoints to travel to. Waypoint is added to the end of the path
     */
    void addWaypoint(const Math::Vector<float, 3> &position, float velocity = 1, float thresholdDistance = 1, int64_t loiterTime = 0) {
        waypoints_.append({position, velocity, thresholdDistance, loiterTime}); // Add the waypoint to the list of waypoints
    }

    void clearWaypoints() {
        waypoints_.clear(); // Clear the list of waypoints
        currentWaypointIndex_ = 0; // Reset the waypoint index
    }

    Core::Topic<CTRL::StarshipFlapSettings>& getFlapSettingTopic() {
        return flapSettingTopic_; // Get the flap setting topic
    }

    bool missionEnd() override {
        return missionEnd_;
    }

    void taskInit() override
    {
        missionState_.missionMode = MissionMode::MissionMode_Idle; // Set the mission mode to idle
    }

    const MissionState& getMissionState() const {
        return missionState_; // Get the mission state
    }

    void beginMission(int64_t startTime) override {
        missionState_.missionMode = MissionMode::MissionMode_Startup;
        missionTime_.setTime(startTime); // Set the mission time to the start time
        actuatorsEnabled_ = false; // Disable actuators
        missionEnd_ = false; // Set the mission end to false
        LOG_MSG("Started mission ReturnToHome\n"); // Log the mission start
    };

    void resetMission() override {
        missionState_.missionMode = MissionMode::MissionMode_Idle;
        actuatorsEnabled_ = false; // Disable actuators
        missionEnd_ = true; // Set the mission end to true
        LOG_MSG("Reset mission ReturnToHome.\n"); // Log the mission reset
    }

    void taskThread() override 
    {

        if (posSubr_.isDataNew()) {
            positionIs_ = posSubr_.getItem().data;
        }

        switch (missionState_.missionMode)
        {
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

        positionSetpointTopic_.publish(positionSetpoint_); // Publish the setpoint to the control system

        flapSettingTopic_.publish(flapSettings_); // Publish the flap settings to the control system


        //missionTimeTopic.publish(missionTime_.NOW()); // Publish the mission time to the control system

    }


private:

    void missionIdle() {

        actuatorsEnabled_ = false; // Disable actuators
        missionTime_.setTime(0); // Set the mission time to the start time
        missionEnd_ = true; // Set the mission end to false

    }

    void missionInit() {

        actuatorsEnabled_ = false; // Disable actuators
        missionEnd_ = false; // Set the mission end to false

    }

    void missionStartup() {

        actuatorsEnabled_ = true; // Enable actuators
        missionEnd_ = false; // Set the mission end to false

        flapSettings_.enableActuators = true; // Enable the actuators
        flapSettings_.tlAngle = 0; // Move top flaps out fully
        flapSettings_.trAngle = 0; 
        flapSettings_.blAngle = 90;// Move bottom flaps in fully
        flapSettings_.brAngle = 90; 

        missionState_.missionMode = MissionMode::MissionMode_Running; // Go to hover mode
        missionState_.positionSetpoint[0] = 0; // Set the velocity setpoint to 0
        missionState_.positionSetpoint[1] = 0; 
        missionState_.positionSetpoint[2] = 0; 
        missionState_.positionSetpoint[3] = posSubr_.getItem().data(3); // Set the position setpoint to where the vehicle is
        missionState_.positionSetpoint[4] = posSubr_.getItem().data(4);
        missionState_.positionSetpoint[5] = posSubr_.getItem().data(5);

        hoverModeLastUpdate_ = Core::NOW(); // Set the time when the hover mode was last updated

        currentWaypointIndex_ = 0; // Reset the waypoint index
        if (waypoints_.size() == 0) {
            addWaypoint({0, 0, 1}, 0.5, 1); // Add a default waypoint to the list of waypoints
        }

    }
    
    void missionHover() {

        float dTime = float(Core::NOW() - hoverModeLastUpdate_)/Core::SECONDS;
        hoverModeLastUpdate_ = Core::NOW(); // Set the time when the hover mode was last updated

        actuatorsEnabled_ = true;   
        //missionEnd_ = false; // Set the mission end to false

        flapSettings_.enableActuators = true; // Enable the actuators
        flapSettings_.tlAngle = 90; // Move top flaps out fully
        flapSettings_.trAngle = 90; 
        flapSettings_.blAngle = 90;// Move bottom flaps in fully
        flapSettings_.brAngle = 90; 

        auto& waypoint = waypoints_[currentWaypointIndex_]; // Get the current waypoint to travel to
        auto distance = waypoint.position - positionIs_.block<3, 1>(3); // Get the distance to the waypoint in reference frame

        if (distance.magnitude() < waypoint.thresholdDistance) { // If the vehicle is within the threshold distance of the waypoint, we consider it as reached
            
            if (Core::NOW() - waypointThresholdTime_ > waypoint.loiterTime) { // If the vehicle is within the threshold distance for a certain time, we consider it as reached
                
                positionSetpoint_(3) = waypoint.position(0); // Set the setpoint position in the reference frame
                positionSetpoint_(4) = waypoint.position(1);
                positionSetpoint_(5) = waypoint.position(2);

                if (currentWaypointIndex_ <= waypoints_.size() - 1) { // If we are at the last waypoint, we go to idle mode
                    currentWaypointIndex_++; // Go to the next waypoint
                } else {
                    missionEnd_ = true; // Set the mission end to true
                }

            }

        } else {
            waypointThresholdTime_ = Core::NOW(); // Set the time when the waypoint was reached
        }

        auto travelDistance = waypoint.position - positionSetpoint_.block<3, 1>(3); // Get the velocity vector to the hover position in reference frame
        if (travelDistance.magnitude() / dTime > waypoint.velocity) {
            travelDistance = travelDistance.normalize() * waypoint.velocity * dTime;
        } 
        positionSetpoint_(3) += travelDistance(0); // Update the setpoint position in the reference frame
        positionSetpoint_(4) += travelDistance(1); 
        positionSetpoint_(5) += travelDistance(2); 

    }


};

}


#endif // MISSION_ABSTRACT_HPP