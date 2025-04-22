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
    };


    struct Telecommand
    {
        TelecommandType type;
        Subsystem subsystem;
        uint16_t paramInt;
        float paramFloat;
    } __attribute__ ((packed));


}







#endif