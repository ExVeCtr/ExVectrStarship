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
            float velocity;                  // The velocity to move to the waypoint in m/s.
            float startAccel;                // The acceleration limit to move the waypoint in m/s^2 at the start of the waypoint travel. This is used to limit the acceleration/deceleration of the vehicle.
            float stopAccel;                 // The deceleration limit to stop at the waypoint in m/s^2 at the end of the waypoint travel. This is used to limit the acceleration/deceleration of the vehicle.
            float thresholdDistance;         // When the waypoint can be considered as reached if within this distance.
            int64_t loiterTime;              // The time the vehicle should loiter at the waypoint in seconds.
            int64_t timeLimit;               // If the vehicle takes more than this time, the waypoint is considered as not reachable and the next waypoint is selected.
            bool cancelIfNotReachable;       // If the waypoint was not reached within the time limit, the mission will be cancelled and ended.
        };

        Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;
        Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;
        Math::Vector<float, 6> positionIs_;

        Math::Vector<float, 6> positionSetpoint_;

        int64_t lastUpdateTimestamp_ = 0; // Timestamp of the last update in microseconds.
        float currentVelocity_ = 0;       // The current velocity of the vehicle in m/s.

        Core::ListArray<Waypoint> waypoints_; // The list of waypoints to travel to.
        size_t currentWaypointIndex_ = 0;     // The index of the current waypoint to travel to.
        int64_t waypointThresholdTime_ = 0;   // The time the vehicle should be within the threshold distance to the waypoint to be considered as reached.
        int64_t waypointStartTime_ = 0;       // The time the vehicle started to travel to the waypoint.
        bool waypointReached_ = false;        // If the waypoint has been reached or not. This is set to true when the vehicle was within the threshold distance to the waypoint.

    public:
        MissionWaypoint(Core::Topic<Core::Timestamped<Math::Vector<float, 7>>> &attTopic, Core::Topic<Core::Timestamped<Math::Vector<float, 6>>> &posTopic) : Task_Periodic("Mission RTH", 0.1 * Core::SECONDS)
        {
            // Subscribe to the topics
            attSubr_.subscribe(attTopic);
            posSubr_.subscribe(posTopic);
            Core::getSystemScheduler().addTask(*this);
            // setPriority(500);
        }

        /**
         * Adds a waypoint to the list of waypoints to travel to. Waypoint is added to the end of the path
         * @param position The position of the waypoint to travel to.
         * @param velocity The velocity to move/lerp to the waypoint in m/s. Set to 0 for an instantaneous move. The velocity output will remain 0.
         * @param acceleration The acceleration limit to move the waypoint in m/s^2. This is used to limit the acceleration/deceleration of the vehicle.
         * @param thresholdDistance When the waypoint can be considered as reached if within this distance.
         * @param loiterTime The time the vehicle should loiter at the waypoint in seconds.
         * @param timeLimit If the vehicle takes more than this time, the waypoint is considered as not reachable and the next waypoint is selected. If not given, the vehicle will calculate the time needed +50% to reach the waypoint.
         * @param cancelIfNotReachable If the waypoint was not reached within the time limit, the mission will be cancelled and ended.
         *
         */
        void addWaypoint(const Math::Vector<float, 3> &position, float velocity = 0, float startAccel = 3, float stopAccel = 3, float thresholdDistance = 1, int64_t loiterTime = 0, int64_t timeLimit = Core::END_OF_TIME, bool cancelIfNotReachable = false)
        {

            if (timeLimit == Core::END_OF_TIME)
            { // If the time limit is not given, calculate the time needed to reach the waypoint

                if (waypoints_.size() > 0)
                {
                    auto lastWaypointPos = waypoints_[waypoints_.size() - 1].position;
                    auto distance = (position - lastWaypointPos).magnitude(); // Calculate the distance to the waypoint
                    timeLimit = (distance / velocity) * 1.5 * Core::SECONDS;  // Calculate the time needed to reach the waypoint +50%
                }
                else
                {
                    auto distance = position.magnitude();                    // Calculate the distance to the waypoint
                    timeLimit = (distance / velocity) * 1.5 * Core::SECONDS; // Calculate the time needed to reach the waypoint +50%
                }
            }

            waypoints_.append({position, velocity, startAccel, stopAccel, thresholdDistance, loiterTime, timeLimit, cancelIfNotReachable}); // Add the waypoint to the list of waypoints
        }

        size_t currentWaypointIndex() const
        {
            return currentWaypointIndex_; // Get the index of the current waypoint
        }

        size_t waypointCount() const
        {
            return waypoints_.size(); // Get the number of waypoints
        }

        void clearWaypoints()
        {
            waypoints_.clear();        // Clear the list of waypoints
            currentWaypointIndex_ = 0; // Reset the waypoint index
        }

        bool missionEnd() override
        {
            return missionEnd_;
        }

        void taskInit() override
        {
            missionState_.missionMode = MissionMode::MissionMode_Idle; // Set the mission mode to idle
            lastUpdateTimestamp_ = Core::NOW();                        // Set the last update timestamp to the current time
        }

        const MissionState &getMissionState() const
        {
            return missionState_; // Get the mission state
        }

        void beginMission(int64_t startTime) override
        {
            resetMission();
            missionState_.missionMode = MissionMode::MissionMode_Initialisation;
            missionTime_.setTime(startTime);       // Set the mission time to the start time
            actuatorsEnabled_ = false;             // Disable actuators
            missionEnd_ = false;                   // Set the mission end to false
            currentWaypointIndex_ = 0;             // Reset the waypoint index
            currentVelocity_ = 0;                  // Reset the current velocity to 0
            waypointStartTime_ = Core::NOW();      // Set the time when the waypoint was started
            lastUpdateTimestamp_ = Core::NOW();    // Reset the last update timestamp
            LOG_MSG("Started mission waypoint\n"); // Log the mission start
        };

        void resetMission() override
        {
            missionState_.missionMode = MissionMode::MissionMode_Idle;
            actuatorsEnabled_ = false;              // Disable actuators
            missionEnd_ = true;                     // Set the mission end to true
            currentWaypointIndex_ = 0;              // Reset the waypoint index
            positionSetpoint_ = {0, 0, 0, 0, 0, 0}; // Set the setpoint to the current position and velocity of the vehicle
            currentVelocity_ = 0;                   // Reset the current velocity to 0
            LOG_MSG("Reset mission waypoint.\n");   // Log the mission reset
        }

        void taskThread() override
        {

            if (posSubr_.isDataNew())
            {
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

            // flapSettingTopic_.publish(flapSettings_); // Publish the flap settings to the control system

            // missionTimeTopic.publish(missionTime_.NOW()); // Publish the mission time to the control system
        }

    private:
        void missionIdle()
        {

            actuatorsEnabled_ = false; // Disable actuators
            missionTime_.setTime(0);   // Set the mission time to the start time
            missionEnd_ = true;        // Set the mission end to false
        }

        void missionInit()
        {

            actuatorsEnabled_ = false; // Disable actuators
            missionEnd_ = false;       // Set the mission end to false

            if (missionTime_.NOW() > -10 * Core::SECONDS)
            {                                                                 // If the mission time is greater than -5 seconds, we consider it as initialised
                missionState_.missionMode = MissionMode::MissionMode_Startup; // Go to startup mode
            }
        }

        void missionStartup()
        {

            actuatorsEnabled_ = true; // Enable actuators
            missionEnd_ = false;      // Set the mission end to false

            // flapSettings_.enableActuators = true; // Enable the actuators
            // flapSettings_.tlAngle = 90; // Move top flaps out fully
            // flapSettings_.trAngle = 90;
            // flapSettings_.blAngle = 90;// Move bottom flaps in fully
            // flapSettings_.brAngle = 90;

            positionSetpoint_(0) = 0; // Set the velocity setpoint to 0
            positionSetpoint_(1) = 0;
            positionSetpoint_(2) = 0;
            positionSetpoint_(3) = posSubr_.getItem().data(3); // Set the position setpoint to where the vehicle is
            positionSetpoint_(4) = posSubr_.getItem().data(4);
            positionSetpoint_(5) = posSubr_.getItem().data(5);

            lastUpdateTimestamp_ = Core::NOW(); // Set the time when the hover mode was last updated
            waypointStartTime_ = Core::NOW();   // Set the time when the waypoint was started

            currentWaypointIndex_ = 0; // Reset the waypoint index
            if (waypoints_.size() == 0)
            {
                addWaypoint({0, 0, 1}, 0.5, 1);                                      // Add a default waypoint to the list of waypoints
                LOG_MSG("No waypoints set, adding default waypoint at (0, 0, 1)\n"); // Log the mission start
            }

            if (missionTime_.NOW() > 0)
                missionState_.missionMode = MissionMode::MissionMode_Running; // Go to hover mode

            currentVelocity_ = 0; // Reset the current velocity to 0
        }

        void missionHover()
        {

            float dTime = float(Core::NOW() - lastUpdateTimestamp_) / Core::SECONDS;
            lastUpdateTimestamp_ = Core::NOW(); // Set the time when the hover mode was last updated

            actuatorsEnabled_ = true;
            // missionEnd_ = false; // Set the mission end to false

            // flapSettings_.enableActuators = true; // Enable the actuators
            // flapSettings_.tlAngle = 90; // Move top flaps out fully
            // flapSettings_.trAngle = 90;
            // flapSettings_.blAngle = 90;// Move bottom flaps in fully
            // flapSettings_.brAngle = 90;

            auto waypoint = waypoints_[currentWaypointIndex_];              // Get the current waypoint to travel to
            auto distance = waypoint.position - positionIs_.block<3, 1>(3); // Get the distance to the waypoint in reference frame
            auto distanceMagnitude = distance.magnitude();                  // Get the magnitude of the distance vector

            if (distanceMagnitude < waypoint.thresholdDistance)
            { // If the vehicle is within the threshold distance of the waypoint, we consider it as reached

                waypointReached_ = true; // Set the waypoint reached to true

                if (Core::NOW() - waypointThresholdTime_ > waypoint.loiterTime)
                { // If the vehicle is within the threshold distance for a certain time, we consider it as reached

                    positionSetpoint_(0) = 0; // Set the velocity setpoint to 0
                    positionSetpoint_(1) = 0;
                    positionSetpoint_(2) = 0;
                    positionSetpoint_(3) = waypoint.position(0); // Set the setpoint position in the reference frame
                    positionSetpoint_(4) = waypoint.position(1);
                    positionSetpoint_(5) = waypoint.position(2);

                    if (currentWaypointIndex_ < waypoints_.size() - 1)
                    {
                        currentWaypointIndex_++;
                        waypoint = waypoints_[currentWaypointIndex_];
                        if (waypoint.velocity < 0.0001)
                        {
                            positionSetpoint_(0) = 0; // Set the velocity setpoint to 0
                            positionSetpoint_(1) = 0;
                            positionSetpoint_(2) = 0;
                            positionSetpoint_(3) = waypoint.position(0); // Set the setpoint position in the reference frame
                            positionSetpoint_(4) = waypoint.position(1);
                            positionSetpoint_(5) = waypoint.position(2);
                        }
                        currentVelocity_ = 0;
                        LOG_MSG("Waypoint reached. Next point: %d\n", currentWaypointIndex_);
                        LOG_MSG("Waypoint position: %.2f %.2f %.2f\n", waypoint.position(0), waypoint.position(1), waypoint.position(2));
                        LOG_MSG("Waypoint velocity: %.2f\n", waypoint.velocity);
                        LOG_MSG("Waypoint threshold distance: %.2f\n", waypoint.thresholdDistance);
                        LOG_MSG("Waypoint loiter time: %.2f\n", double(waypoint.loiterTime) / Core::SECONDS);
                        LOG_MSG("Waypoint time limit: %.2f\n", double(waypoint.timeLimit) / Core::SECONDS);
                        waypointThresholdTime_ = Core::NOW();
                        waypointStartTime_ = Core::NOW();
                        waypointReached_ = false;
                    }
                    else
                    {
                        LOG_MSG("Waypoint mission finished\n"); // Log the mission finished
                        missionEnd_ = true;                     // Set the mission end to true
                    }
                }
            }
            else
            {

                waypointThresholdTime_ = Core::NOW(); // Set the time when the waypoint was reached

                if (!waypointReached_ && Core::NOW() - waypointStartTime_ > waypoint.timeLimit)
                { // If the vehicle is within the threshold distance for a certain time, we consider it as not reachable

                    positionSetpoint_(0) = 0; // Set the velocity setpoint to 0
                    positionSetpoint_(1) = 0;
                    positionSetpoint_(2) = 0;
                    positionSetpoint_(3) = waypoint.position(0); // Set the setpoint position in the reference frame
                    positionSetpoint_(4) = waypoint.position(1);
                    positionSetpoint_(5) = waypoint.position(2);

                    if (waypoint.cancelIfNotReachable)
                    {                                                           // If the waypoint was not reached within the time limit, the mission will be cancelled and ended
                        LOG_MSG("Waypoint not reachable. Mission cancelled\n"); // Log the mission cancelled
                        missionEnd_ = true;                                     // Set the mission end to true
                    }
                    else if (currentWaypointIndex_ < waypoints_.size() - 1)
                    {                                                                               // If we are at the last waypoint, we go to idle mode
                        currentWaypointIndex_++;                                                    // Go to the next waypoint
                        LOG_MSG("Waypoint not reachable. Next point: %d\n", currentWaypointIndex_); // Log the waypoint reached
                        waypointThresholdTime_ = Core::NOW();                                       // Set the time when the waypoint was reached
                        waypointStartTime_ = Core::NOW();                                           // Set the time when the waypoint was started
                        waypointReached_ = false;                                                   // Set the waypoint reached to false
                    }
                    else
                    {
                        LOG_MSG("Waypoint not reachable. Waypoint mission finished\n"); // Log the mission finished
                        missionEnd_ = true;                                             // Set the mission end to true
                    }
                }
            }

            // Calculate the velocity limit to decelerate at the given amount and stop at the waypoint position
            auto accelVelocityLimit = sqrtf(distanceMagnitude * waypoint.stopAccel);

            // Change the velocity to match the waypoint acceleration characteristics
            if (currentVelocity_ > accelVelocityLimit)
            { // The deceleration limit is the strongest limit. We limit the velocity to the deceleration limit
                currentVelocity_ = accelVelocityLimit;
            }
            else if (waypoint.velocity > currentVelocity_)
            {                                                    // If we do not decelerate and we are below the waypoint velocity, we accelerate to the waypoint velocity
                currentVelocity_ += waypoint.startAccel * dTime; // Increase the current velocity by the acceleration multiplied by the time since the last update
                if (currentVelocity_ > waypoint.velocity)
                {                                         // If the current velocity is greater than the waypoint velocity, we set the current velocity to the waypoint velocity
                    currentVelocity_ = waypoint.velocity; // Set the current velocity to the waypoint velocity
                }
            }

            auto travelDistance = waypoint.position - positionSetpoint_.block<3, 1>(3); // Get the velocity vector to the hover position in reference frame
            auto travelDistanceNorm = travelDistance.normalize();                       // Normalize the travel distance vector
            if (travelDistance.magnitude() / dTime > currentVelocity_)
            {
                travelDistance = travelDistance.normalize() * currentVelocity_ * dTime;
            }
            positionSetpoint_(0) = travelDistanceNorm(0) * currentVelocity_; // Set the velocity setpoint in the direction of travel with magnitude of the current velocity
            positionSetpoint_(1) = travelDistanceNorm(1) * currentVelocity_;
            positionSetpoint_(2) = travelDistanceNorm(2) * currentVelocity_;
            positionSetpoint_(3) += travelDistance(0); // Update the setpoint position in the reference frame
            positionSetpoint_(4) += travelDistance(1);
            positionSetpoint_(5) += travelDistance(2);
        }
    };

}

#endif // MISSION_ABSTRACT_HPP