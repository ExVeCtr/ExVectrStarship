#ifndef TELECOMMAND_HPP
#define TELECOMMAND_HPP


#include <stdint.h>

#include "subsystems.hpp"


namespace VCTR {


    /**
     * * @brief What the command wants to do.
     */
    enum TelecommandType : uint8_t
    {
        Telecommand_CalibStart = 0, //Calibrates sensors and filters. Subsystem chooses what to calibrate. paramInt gives how long to calibrate for in seconds.
        Telecommand_CalibApply,     //Applies the calibration to the sensors and filters.
        Telecommand_Load,           //Reads the saved state from memory. For sensors this would be settings and calibration. For control this could be PID settings etc.
        Telecommand_Save,           //Writes the current state to memory. For sensors this would be settings and calibration. For control this could be PID settings etc.
        Telecommand_SystemReset,    //Clears errors/failure flags and readys system for another run (mode -> Startup -> Ready, disarm, motors and actuators off). Resets the whole mission chain to the state after boot (all missions Idle, RTH active and Idle, no pending next mission; the chain is never continued). Does NOT zero the estimators. Does NOT change simulation mode: simulation stays on if it was on, off if it was off. Aborts an active estimator zeroing window. paramInt must be equal to 0xC5 to be valid! Can be used as a safety stop.
        Telecommand_Arm,            //Arms the system. paramInt must be equal to 0xA5 to arm. Other values will disarm. This is used to enable the motors and actuators.
        Telecommand_SimulationMode, //Enables simulation mode. paramInt must be equal to 0xA9. This is used for testing and debugging. In this mode the system will not use any sensors and will just run the control loop with simulated data.
        Telecommand_MissionBegin,   //Begins the mission (Real or simulation). paramInt gives the time in seconds to start the mission in the future.
        Telecommand_MissionSelect,  //Selects the mission to run. paramInt gives the mission number to run. Only valid if vehicle is in ready state.
        Telecommand_MissionAbort,   //Aborts the current mission and switches to Return to Home mode. Only valid if the vehicle is in flight.
        Telecommand_ZeroEstimators, //Zeroes all estimators (position origin and baro altitude reference, position/velocity state, accelerometer and gyro bias) for a window of paramFloat seconds (NaN or 0: 10 s, clamped to 1..60 s), timed on the vehicle. paramInt must be equal to 0x2E. Accepted only when initialised, disarmed, motors off and in mode Startup or Ready. The vehicle must be still. Value 10.
        Telecommand_MAX             //Max value for the enum. This is used to check if the command is valid or not.
    };


    struct Telecommand
    {
        TelecommandType type;
        Subsystem subsystem;
        int16_t paramInt;
        float paramFloat;
    } __attribute__ ((packed));


}







#endif