#ifndef EXVECTRNETWORK_DATALINKSX1280_H_
#define EXVECTRNETWORK_DATALINKSX1280_H_

#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/list.hpp"

#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/interfaces/datalink_interface.hpp"

#include "sx12xxAL/src/SX128XLT.h"


namespace VCTR
{

    namespace Net
    {

        /**
         * @brief A class implementing a datalink layer for the SX1280 LoRa transceiver.
         */
        class Datalink_SX1280 : public Net::Datalink_Interface, public Core::Task_Periodic
        {
        private:

            ///@brief Maximum length a data frame can be.
            static constexpr size_t dataLinkMaxFrameLength = 200; 
            
            SX128XLT lora_; 

            int NSS_PIN_;
            int NRESET_PIN_;
            int RFBUSY_PIN_;
            int DIO1_PIN_;
            int TX_EN_PIN_;
            int RX_EN_PIN_;

            int16_t receivedDataRSSI_;
            int16_t receivedDataSNR_;

            bool isSending_ = false;
            bool channelBusy_ = false;

            Core::ListBuffer<uint8_t, dataLinkMaxFrameLength * 5> transmitBuffer_;

            int64_t lastSendTimestamp_ = 0;
            

        public:

            Datalink_SX1280(int nssPin, int nresetPin, int rfbusyPin, int dio1Pin, int txEnPin, int rxEnPin);
            

            void taskInit() override;

            void taskThread() override;

            void taskCheck() override;


            int16_t lastPacketRSSI() const { return receivedDataRSSI_; }
            int16_t lastPacketSNR() const { return receivedDataSNR_; }


        private:

            void dataframeReceiveFunc(const Core::List<uint8_t> &item) override;
            
        };

    }

}

#endif