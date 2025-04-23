#ifndef TELECOMMAND_HPP
#define TELECOMMAND_HPP


#include "ExVectrMath/matrix_base.hpp"
#include "ExVectrMath/matrix_vector.hpp"

#include "subsystems.hpp"


namespace VCTR {


    /**
     * * @brief What the command wants to do.
     */
    enum TelecommandType : uint8_t
    {
        Telecommand_CalibStart = 0, //Calibrates sensors and filters.
        Telecommand_CalibApply,     //Applies the calibration to the sensors and filters.
        Telecommand_Load,           //Reads the saved state from memory. For sensors this would be settings and calibration. For control this could be PID settings etc.
        Telecommand_Save,           //Writes the current state to memory. For sensors this would be settings and calibration. For control this could be PID settings etc.
        Telecommand_SystemReset,    //Clears errors/failure flags and readys system for another run. Disables simulation mode. paramInt must be equal to 0xC5 to be valid!
        Telecommand_SimulationMode, //Enables simulation mode. paramInt must be equal to 0xA9. This is used for testing and debugging. In this mode the system will not use any sensors and will just run the control loop with simulated data.
        Telecommand_MissionBegin    //Begins the mission (Real or simulation). paramInt gives the time in milliseconds to start the mission in the future.
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