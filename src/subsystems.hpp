#ifndef SUBSYSTEMS_HPP
#define SUBSYSTEMS_HPP



namespace VCTR {


    /**
     * * @brief The system state of the vehicle.
     */
    enum Subsystem : uint8_t
    {
        Subsystem_Gyro = 0,     
        Subsystem_Accel,
        Subsystem_Magneto,
        Subsystem_Baro,
        Subsystem_GNSS,
        Subsystem_AttFilter,
        Subsystem_PosFilter
    };


}







#endif