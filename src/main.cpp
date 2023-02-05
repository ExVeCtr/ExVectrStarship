#include <Arduino.h>

#include "ExVectrCore/scheduler.hpp"
#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrArduinoPlatform/bus_spi.hpp"

#include "board_v_1_0.h"


using namespace VCTR;


Platform::PinGPIO bmePin;
//Platform::BusSPIDevice spiDev(SPI, SPISettings{500000, MSBFIRST, SPI_MODE0}, bmePin, true);


/*class Test: public VCTR::Task_Threading {
public:

    Test() : VCTR::Task_Threading("Test Task", VCTR::eTaskPriority_Realtime, 1*VCTR::SECONDS) {}

    void init() {

        bmePin.init(BME280_NCS_PIN, GPIO_IOMODE_t::IOMODE_OUTPUT);
        bmePin.setPinValue(true);

        SPI.begin();

    }

    void thread() {

        Serial.println("Beginning...");

        uint8_t id = 0;
        uint8_t c = 0xD0 | 0x80;

        bmePin.setPinValue(false);

        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        //SPI.transfer(c);
        id = SPI.transfer(0xff);
        SPI.endTransaction();

        bmePin.setPinValue(true);

        //spiDev.writeByte(0xD0 | 0x80);
        //spiDev.readByte(id);

        Serial.println(String("Time: ") + VCTR::NOWSeconds() + ". ID: " + id);

    }

};

Test t;*/

class Blinky: public VCTR::Task_Threading {
public:

    VCTR::Platform::PinGPIO pinLED;

    Blinky() : VCTR::Task_Threading("Blinky Task", VCTR::eTaskPriority_Realtime, 1*VCTR::SECONDS) {}

    void init() {

        pinLED.init(LED_BUILTIN, VCTR::GPIO_IOMODE_t::IOMODE_OUTPUT);

    }

    void thread() {

        pinLED.setPinValue(!pinLED.getPinValue());

    }

};

Blinky b;

void setup() {
    Serial.begin(115200);
    bmePin.init(BME280_NCS_PIN, GPIO_IOMODE_t::IOMODE_OUTPUT);
        bmePin.setPinValue(true);

        SPI.begin();
}

void loop() {
    VCTR::Task_Threading::schedulerTick();
    Serial.println("Beginning...");

        uint8_t id = 0;
        uint8_t c = 0xD0 | 0x80;

        bmePin.setPinValue(false);

        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        //SPI.transfer(c);
        id = SPI.transfer(0xff);
        SPI.endTransaction();

        bmePin.setPinValue(true);

        //spiDev.writeByte(0xD0 | 0x80);
        //spiDev.readByte(id);

        Serial.println(String("Time: ") + ". ID: " + id);

        delay(1000);
}