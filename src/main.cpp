#include <Arduino.h>

#include "ExVectrMath/specialised_types.hpp"

#include "ExVectrCore.hpp"

#include "ExVectrCore/random.h"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrArduinoPlatform/bus_spi.hpp"
#include "ExVectrArduinoPlatform/bus_i2c.hpp"

#include "ExVectrSensor/Sensors/mpu9250.hpp"
#include "ExVectrSensor/Sensors/bme280.hpp"
#include "ExVectrSensor/Sensors/qmc5883.hpp"

#include "ExVectrArduinoPlatform.hpp"

#include "board_v_1_0.h"

using namespace VCTR;

#define PMW_CS_PIN 29
#define PMW_INT_PIN 28

Platform::PinGPIO mpuPin;
Platform::BusSPIDevice spiMPU(SPI, mpuPin, true);
SNSR::MPU9250Driver mpuDriver(spiMPU, true);

Platform::PinGPIO bmePin;
Platform::BusSPIDevice spiBME(SPI, bmePin, true);
SNSR::BME280Driver bme(spiBME);

Platform::PinGPIO pmwPin;

Platform::PinGPIO rfPin;

Platform::BusI2CDevice qmcBus(Wire, 0x0D);
SNSR::QMC5883Driver qmc(qmcBus);

class SensorReadout : public Core::Task_Periodic
{
public:

    Core::Simple_Subscriber<Core::Timestamped<Data::ValueCov<float, 3>>> subr;
    Core::Simple_Subscriber<Core::Timestamped<Data::ValueCov<float, 1>>> bmeSubr;
    Core::Simple_Subscriber<Core::Timestamped<Data::ValueCov<float, 3>>> qmcSubr;

    SensorReadout() : Task_Periodic("Test MPU driver", Core::MILLISECONDS * 10)
    {
        Core::getSystemScheduler().addTask(*this);
    }

    void taskInit() override
    {

        subr.subscribe(mpuDriver.getGyroTopic());
        bmeSubr.subscribe(bme.getBaroTopic());
        qmcSubr.subscribe(qmc.getMagTopic());

        subr.setTaskToResume(*this);
        bmeSubr.setTaskToResume(*this);
        qmcSubr.setTaskToResume(*this);
    }

    void taskThread() override
    {   

        if (subr.isDataNew())
        {

            auto data = subr.getItem().data.val;

            Core::printM("Gyro X: %f, Y: %f, Z: %f.\n", data(0), data(1), data(2));
        }

        if (bmeSubr.isDataNew())
        {

            auto data = bmeSubr.getItem().data.val;

            Core::printM("Baro: %f.\n", data(0));
        }

        if (qmcSubr.isDataNew())
        {

            auto data = qmcSubr.getItem().data.val;

            Core::printM("Mag X: %f, Y: %f, Z: %f.\n", data(0), data(1), data(2));
        }

    }
};

SensorReadout sensorReadout;

void setup()
{

    SPI.begin();
    Wire.begin();
    Wire.setClock(100000);

    mpuPin.init(MPU9250_NCS_PIN, HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
    bmePin.init(BME280_NCS_PIN, HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
    pmwPin.init(PMW_CS_PIN, HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
    rfPin.init(SX1280_NSS_PIN, HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);

    mpuPin.setPinValue(1);
    bmePin.setPinValue(1);
    pmwPin.setPinValue(1);
    rfPin.setPinValue(1);

    VCTR::Core::initialise();
}

void loop()
{
    VCTR::Core::getSystemScheduler().tick();
    /*const auto& tasks = Core::getSystemScheduler().getTasks();
    Core::printM("Time: %f\n", Core::NOWSeconds());
    for (size_t i = 0; i < tasks.size(); i++) {
        Core::printM("Task %d: %s, rel: %d\n", i, tasks[i].task->taskName(), tasks[i].pseudoPriority);

    }*/
    //delay(1);
}