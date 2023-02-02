#include <Arduino.h>

#include "ExVectrCore/scheduler.hpp"
#include "ExVectrArduinoPlatform.hpp"

class Test: public VCTR::Task_Threading {
public:

    Test() : VCTR::Task_Threading("Test Task", VCTR::eTaskPriority_Realtime, 1*VCTR::SECONDS) {}

    void thread() {

        Serial.println(String("Time: ") + VCTR::NOWSeconds());

    }

};

class Blinky: public VCTR::Task_Threading {
public:

    VCTR::Platform::HAL_PinGPIO pinLED;

    Blinky() : VCTR::Task_Threading("Blinky Task", VCTR::eTaskPriority_Realtime, 0.3*VCTR::SECONDS) {}

    void init() {

        pinLED.init(LED_BUILTIN, VCTR::GPIO_IOMODE_t::IOMODE_OUTPUT);

    }

    void thread() {

        pinLED.setPinValue(!pinLED.getPinValue());

    }

};

Test t;
Blinky b;

void setup() {
    Serial.begin(115200);
}

void loop() {
    VCTR::Task_Threading::schedulerTick();
}