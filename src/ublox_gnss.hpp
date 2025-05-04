#ifndef EXVECTRSENSOR_UBLOXGNSS_H
#define EXVECTRSENSOR_UBLOXGNSS_H

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrSensor/gnss.hpp"

#include "Arduino.h"

#include "gnssdriver/src/SparkFun_u-blox_GNSS_Arduino_Library.h"


namespace VCTR
{

    namespace SNSR
    {

        
        class UbloxSerialGNSS: public GNSS, public Core::Task_Periodic {
        public:
        
            /**
             * @param serialPort Pointer to serial port to use. If non default pins used then setup before init run.
             * @param usbPassthrough If true then gps wont be setup and serial data will be passed to USB serial.
             */
            UbloxSerialGNSS(HardwareSerial& serialPort, int rxPin = -1, int txPin = -1) : Task_Periodic("Ublox GNSS", 1*Core::MILLISECONDS) {
                serialPort_ = &serialPort;
                rxPin_ = rxPin;
                txPin_ = txPin;
                //usbPassthrough_ = true;// usbPassthrough;
                Core::getSystemScheduler().addTask(*this);
            }
        
            /**
             * @brief Initialises and sets sensors settings.
             * @param ioBus Which bus to use for communications.
             * @return true if successfull and sensor is running, false otherwise.
             */
            bool initSensor(HAL::DigitalIO &ioBus);

            /**
             * @brief Makes the sensor read the values and publish them to the topic.
             * @note Implemented by child class.
             * @return true if reading was successfull. False otherwise.
             */
            bool readGNSS() override;

            /**
             * Will check the uart port if there is data to be read.
             */
            void taskCheck() override;

            /**
             * @brief Initialises sensor and expected to be called once at start by scheduler
             */
            void taskInit() override;

            /**
             * @brief main task thread that reads all sensor data and publishes it. To be called by scheduler.
             */
            void taskThread() override;
        
        
        private:
        
            void setupSerial(uint32_t baudRate);
        
            void _getData();
        
            int rxPin_;
            int txPin_;
        
            uint8_t numSats_ = 0;
        
            //Values used to determine true fix
            uint8_t minNumSats_ = 5;
            float maxError_ = 600;
        
            int64_t lastMeasurement_ = 0;
        
            HardwareSerial* serialPort_;
            //uint32_t serialBaudMulti_ = 1;
            bool usbPassthrough_ = false;
        
            SFE_UBLOX_GNSS gnss_;
        
            
        };

    }

}

#endif