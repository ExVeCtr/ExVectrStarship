#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP


#include "ExVectrMath/matrix_base.hpp"
#include "ExVectrMath/matrix_vector.hpp"

#include "subsystems.hpp"


namespace VCTR {


    /**
     * * @brief The system state of the vehicle.
     */
    enum class VehicleMode : uint8_t
    {
        VehicleMode_Startup = 0,                //Software is starting up and initialising.
        VehicleMode_SensorInit,                 //Sensors are being setup and initialised.
        VehicleMode_SensorCalib,                //Sensors are being calibrated.
        VehicleMode_Safe,                       //Everything is running, but the actuators are currently disabled and in a safe state. Sensors could possibly be trying to calibrate and adjust.
        VehicleMode_Ready,                      //Everything is running and ready to go. Actuators are enabled and the system is ready to go. THIS COULD POSSIBLY BE DANGEROUS.
        VehicleMode_Running,                    //The system is running and doing its thing. E.g. flying and running through the mission.
        VehicleMode_Failure,                    //Some failure has occurred and the system is in a catastrophic failure state. In this state everything shuts down and the system is in a safe state.
    };


    /**
     * * @brief Status of each subsystem (Sensor, Filter etc.)
     */
    enum TelemetrySensor : uint8_t
    {
        TelemetrySensor_Init = 0,       //Software is setting up the system
        TelemetrySensor_Calib,            //System needs time to stabilise, calibrate etc but is not in a failure state.
        TelemetrySensor_Ready,                  //System is ready to go and running.
        TelemetrySensor_Failure,                //System is in a failure state.
    };


    /**
     * * @brief The status of each sensory subsystem.
     */
    struct SensoryState
    {

        TelemetrySensor gnss;
        TelemetrySensor imu;
        TelemetrySensor baro;
        TelemetrySensor mag;
        TelemetrySensor attitudeKF;
        TelemetrySensor positionKF;

    } __attribute__ ((packed));


    /**
     * * @brief What type of failure occurred.
     */
    struct FailureState
    {

        bool sensorFailure;
        bool attitudeFailure;
        bool positionFailure;

        bool positionOutOfBounds;
        bool attitudeOutOfBounds;

    } __attribute__ ((packed));


    /**
     * * @brief The state of the mission.
     */
    enum MissionState : uint8_t {
        MissionState_Idle, //Mission has not started and is waiting for the command.
        MissionState_Startup, //Vehicle sits with everything enabled on the ground. This is used to check if the vehicle is stable and ready for takeoff.
        MissionState_Hover, //Vehicle sets position to hover at a certain height for a given amount of time.
        MissionState_Descent, //Vehicle decends at a given rate until the landing threshold is reached.
        MissionState_Landed //Mission is finished and vehicle is landed.
    };

    /**
     * * @brief The params of each control subsystem axis
     */
    /*struct ControllerParams {
        float pGain;
        float iGain;
        float dGain;
        float iLimit;
    };*/

    /**
     * * @brief The status of each control subsystem.
     */
    /*struct TVCParams
    {

        float angleLimit;

    };*/




}







#endif