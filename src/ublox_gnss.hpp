#ifndef EXVECTRSENSOR_UBLOXGNSS_H
#define EXVECTRSENSOR_UBLOXGNSS_H

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/scheduler2.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrSensor/gnss.hpp"

#include "Arduino.h"

#include "gnssdriver/src/SparkFun_u-blox_GNSS_Arduino_Library.h"

#include "gnss_config_sequencer.hpp"

namespace VCTR
{

    namespace SNSR
    {

        class UbloxSerialGNSS : public GNSS, public Core::Task_Periodic
        {
        public:
            /**
             * @param serialPort Pointer to serial port to use. If non default pins used then setup before init run.
             * @param usbPassthrough If true then gps wont be setup and serial data will be passed to USB serial.
             */
            UbloxSerialGNSS(HardwareSerial &serialPort, int rxPin = -1, int txPin = -1) : Task_Periodic("Ublox GNSS", 1 * Core::MILLISECONDS, 5 * Core::SECONDS)
            {
                serialPort_ = &serialPort;
                rxPin_ = rxPin;
                txPin_ = txPin;
                // usbPassthrough_ = true;// usbPassthrough;
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
             * @brief Called once at start by the scheduler. Does not talk to the receiver any more: the
             * configuration runs as a non-blocking sequence in taskThread (GnssConfigSequencer).
             */
            void taskInit() override;

            /**
             * @brief Runs one configuration step per call until the receiver is configured, then reads PVT data and
             * publishes it. To be called by scheduler.
             */
            void taskThread() override;

            /// True once the configuration sequence has finished (PVT reading active).
            bool configured() const { return config_.done(); }

            /// Read back after configuration (UBX-CFG-GNSS): GPS and Galileo on, everything else off.
            bool constellationsAsIntended() const { return SNSR::constellationsAsIntended(readGnss_); }

            /// Fix type of the last PVT (UBX-NAV-PVT fixType: 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + DR,
            /// 5 time only). Telemetry only (AIR_GNSS_INFO, GNSS); 0 until the first measurement.
            uint8_t fixType() const { return fixType_; }

            /// The configuration sequence has ended, done or failed (no module, reopen failed).
            bool configFinished() const { return config_.done() || config_.failed(); }

            /// The configuration sequence completed with every step successful (no step skipped).
            bool configComplete() const { return config_.done() && config_.skippedSteps() == 0; }

        private:
            void setupSerial(uint32_t baudRate);

            /// Performs one action of the configuration sequence (one bounded library call).
            bool performConfigAction(GnssConfigSequencer::Action action);

            /// Runs the configuration sequence: at most one library call per task run, waits are timed states.
            void runConfiguration();

            /// One UBX poll (or write, len > 0) of cls/id through a custom packet into ubxPayload_. True on reply.
            bool ubxPoll(uint8_t cls, uint8_t id);
            bool ubxWrite(uint8_t cls, uint8_t id, uint16_t len);

            /// Logs the read-back result once, when the sequence is done.
            void logReadBack();

            // UBX payload buffer for the custom packets (the library writes up to MAX_PAYLOAD_SIZE bytes into it).
            uint8_t ubxPayload_[MAX_PAYLOAD_SIZE];
            uint16_t ubxPayloadLen_ = 0;
            int lastUbxStatus_ = -1; ///< sfe_ublox_status_e of the last ubxPoll/ubxWrite (for the failure log)
            bool gnssPolledForWrite_ = false;
            GnssConstellations readGnss_;
            GnssInUse readInUse_;
            GnssNavRate readRate_;

            GnssConfigSequencer config_;
            int64_t configStartNs_ = 0;
            int64_t configMaxStepNs_ = 0; ///< longest single configuration step (bounded library call)

            void _getData();

            int rxPin_;
            int txPin_;

            uint8_t numSats_ = 0;
            uint8_t fixType_ = 0;

            // Values used to determine true fix
            uint8_t minNumSats_ = 5;
            float maxError_ = 600;

            int64_t lastMeasurement_ = 0;

            HardwareSerial *serialPort_;
            // uint32_t serialBaudMulti_ = 1;
            bool usbPassthrough_ = false;

            SFE_UBLOX_GNSS gnss_;
        };

    }

}

#endif