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
        Telecommand_SimWindSpeed,   //SIMULATION ONLY (refused otherwise): steady wind speed of the simulated world, paramFloat in m/s (finite, clamped to 0..20). paramInt must be equal to 0x57. Reset to 0 whenever simulation mode is switched on or off. Value 11.
        Telecommand_SimWindDirection, //SIMULATION ONLY (refused otherwise): direction the simulated wind blows TOWARD, paramFloat in rad, counterclockwise from North toward West (NWU; 0 = toward North, pi/2 = toward West), finite, wrapped to -pi..pi. paramInt must be equal to 0x57. Value 12.
        Telecommand_SimGustStrength, //SIMULATION ONLY (refused otherwise): peak gust speed (bound of the extra wind speed along the mean direction) of the simulated world, paramFloat in m/s (finite, clamped to 0..10). paramInt must be equal to 0x57. Value 13.
        Telecommand_SimGustPeriod,  //SIMULATION ONLY (refused otherwise): gust timescale, paramFloat in s (finite, clamped to 0.5..30). paramInt must be equal to 0x57. Value 14.
        Telecommand_SimGustSeed,    //SIMULATION ONLY (refused otherwise): gust seed, paramFloat an integer-valued number 0..16777215 (finite; rounded, clamped; exact in float). paramInt must be equal to 0x57. Value 15.
        Telecommand_SimFlapStuck,   //SIMULATION ONLY (refused otherwise): stuck flap servo fault, owner-approved 2026-11-21. subsystem = flap 0..3 (0 TL, 1 TR, 2 BL, 3 BR), any other bit refused (the 'arm for descent' bit 0x10 was removed 2026-11-21). Takes effect immediately on the modelled true flap angle. paramFloat = stuck angle in rad (finite, clamped 0..pi/2), -1.0 = hold the current true angle. paramInt must be equal to 0x57. Cleared on every simulation mode switch. Value 16.
        Telecommand_SimFlapSlow,    //SIMULATION ONLY (refused otherwise): slow flap servo fault. subsystem = flap index, paramFloat = servo rate factor (finite, clamped 0.05..1). paramInt must be equal to 0x57. Value 17.
        Telecommand_SimFaultClear,  //SIMULATION ONLY (refused otherwise): clears all flap faults. paramInt must be equal to 0x57, other fields ignored. Value 18.
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