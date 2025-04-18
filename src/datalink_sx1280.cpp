#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/list.hpp"

#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/interfaces/datalink_interface.hpp"

#include "sx12xxAL/src/SX128XLT.h"

#include "datalink_sx1280.hpp"


//#define SX1280_DEBUG


namespace VCTR
{

    namespace Net
    {

        Datalink_SX1280::Datalink_SX1280(int nssPin, int nresetPin, int rfbusyPin, int dio1Pin, int txEnPin, int rxEnPin) : 
            Task_Periodic("Datalink_SX1280", 100*Core::MILLISECONDS)
        {       
                
                NSS_PIN_ = nssPin;
                NRESET_PIN_ = nresetPin;
                RFBUSY_PIN_ = rfbusyPin;
                DIO1_PIN_ = dio1Pin;
                TX_EN_PIN_ = txEnPin;
                RX_EN_PIN_ = rxEnPin;
    
                Core::getSystemScheduler().addTask(*this);
        }

        void Datalink_SX1280::taskInit() {

            pinMode(NSS_PIN_, OUTPUT);
            pinMode(NRESET_PIN_, OUTPUT);
            pinMode(RFBUSY_PIN_, INPUT);
            pinMode(DIO1_PIN_, INPUT);
            pinMode(TX_EN_PIN_, OUTPUT);
            pinMode(RX_EN_PIN_, OUTPUT);

            digitalWrite(NSS_PIN_, HIGH);
            digitalWrite(NRESET_PIN_, HIGH);
            digitalWrite(TX_EN_PIN_, LOW);
            digitalWrite(RX_EN_PIN_, LOW);

            if (!lora_.begin(NSS_PIN_, NRESET_PIN_, RFBUSY_PIN_, DIO1_PIN_, RX_EN_PIN_, TX_EN_PIN_, DEVICE_SX1280)) {
                LOG_MSG("Failed to initialize SX1280!\n");
                setInitialised(false);
                setPaused(true);
                return;
            }

            lora_.setupLoRa(2445000000, 0, LORA_SF6, LORA_BW_1600, LORA_CR_4_8);
            lora_.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0, 0);
            lora_.setHighSensitivity();

            lora_.receiveSXBuffer(0, 0, NO_WAIT);

            setPaused(true); //Pause the thread until dio interrupt

        }

        void Datalink_SX1280::taskCheck() {

            if (digitalRead(DIO1_PIN_) == HIGH || (transmitBuffer_.size() > 0 && !isSending_ && !channelBusy_)) {
                setPaused(false);
                setRelease(Core::NOW());
            }

        }
        
        void Datalink_SX1280::taskThread() {

            if (digitalRead(DIO1_PIN_) == HIGH) {

                #ifdef SX1280_DEBUG
                    //Serial.println("DIO1 Pin is high!");
                #endif

                uint16_t irqStatus = lora_.readIrqStatus();
                lora_.clearIrqStatus(IRQ_RADIO_ALL);

                bool beginCad = true;

                if ((irqStatus & IRQ_RX_DONE) && (irqStatus & IRQ_HEADER_VALID)) { // If interrupt says data good then get data

                    #ifdef SX1280_DEBUG
                        Serial.println("Interrupt says data received");
                    #endif
        
                    size_t packetL = lora_.readRXPacketL();
        
                    if (packetL > 0) { // make sure packet is okay
        
                        receivedDataRSSI_ = lora_.readPacketRSSI();
                        receivedDataSNR_ = lora_.readPacketSNR();

                        uint8_t buffer[packetL];
                        //uint8_t crc = 0;
        
                        lora_.startReadSXBuffer(0);
                        lora_.readBuffer(buffer, packetL);
                        lora_.endReadSXBuffer();

                        uint8_t crc = 0;
                        for (int i = 0; i < packetL - 1; i++) crc += buffer[i];
                        uint8_t crcRcv = buffer[packetL - 1];

                        if (crc == crcRcv) { //Only decode if crc is correct.

                            Core::ListBuffer<uint8_t, dataLinkMaxFrameLength> receivedData;
                            for (size_t i = 0; i < packetL - 1;) {

                                auto segLen = buffer[i];
                                i++; //Read the size
                                receivedData.clear(); //Make sure its empty
                                for (int j = 0; j < segLen; j++) { //Place data into list.
                                    receivedData.placeBack(buffer[j + i]);
                                }
                                LOG_MSG("Data segment Received is %d bytes long.\n", segLen);
                                receiveTopic_.publish(receivedData);

                                i += segLen;

                            }

                        } else {
                            LOG_MSG("Received data is corrupt and cant be decoded reliably! Data len %d, crcRcv %d, crc calc %d\n", packetL, crcRcv, crc);
                        }
                        
        
                    } else {
                        LOG_MSG("Data was 0 bytes long! CRITICAL ERROR\n"); 
                    }

                
                }

                if (irqStatus & (IRQ_HEADER_ERROR)) {
        
                    LOG_MSG("Interrupt says header error!\n");
        
                }

                if (irqStatus & (IRQ_CRC_ERROR)) {
        
                    LOG_MSG("Interrupt says crc error!\n");
        
                }

                if (irqStatus & (IRQ_RX_TIMEOUT)) {

                    LOG_MSG("Interrupt says rx timed out!\n");
        
                } 
                
                if (irqStatus & (IRQ_TX_DONE)) {
        
                    isSending_ = false;
                    lastSendTimestamp_ = Core::NOW();
                    //lastSendTimestamp_ = NOW();
        
                    #ifdef SX1280_DEBUG
                        Serial.println("Interrupt says tx done!");
                    #endif
        
                }
        
                if (irqStatus & (IRQ_TX_TIMEOUT)) {
        
                    isSending_ = false;
                    lastSendTimestamp_ = Core::NOW();
                    //lastSendTimestamp_ = NOW();
        
                    LOG_MSG("Interrupt says tx timeout!");
        
                }
        
                if (irqStatus & (IRQ_CAD_ACTIVITY_DETECTED)) {
        
                    channelBusy_ = true;
                    beginCad = false;

                    //lora_.clearIrqStatus(IRQ_RADIO_ALL);
                    lora_.receiveSXBuffer(0, 0, NO_WAIT);
        
                    #ifdef SX1280_DEBUG
                        Serial.println("Channel activity detected! Beginning receive");
                    #endif
        
                }
        
                if (irqStatus & (IRQ_CAD_DONE)) {
                    //lastSendTimestamp_ = NOW();
        
                    if (channelBusy_)
                        LOG_MSG("Channel is free again!\n");

                    channelBusy_ = false;
        
                }

                if (beginCad) {
                    lora_.setMode(MODE_STDBY_RC);
                    lora_.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0, 0); 
                    lora_.rxEnable();
                    lora_.startCAD(LORA_CAD_01_SYMBOL);
                }

                //lora_.receiveSXBuffer(0, 0, NO_WAIT);

            }


            if (transmitBuffer_.size() > 0 && !isSending_ && !channelBusy_ && Core::NOW() - lastSendTimestamp_ > 20*Core::MILLISECONDS) {

                isSending_ = true;

                const uint8_t packetLimit = 250;
                uint8_t buffer[packetLimit];
                uint8_t bufferSize = 0;
                uint8_t crc = 0;

                #ifdef SX1280_DEBUG
                    Serial.printf("There is data to send. Moving segments into buffer...\n");
                #endif
                
                //We keep placing each packet frame into the buffer untill the next one would be too much.
                while (transmitBuffer_.size() > 0 && bufferSize + transmitBuffer_[0] + 2 <= packetLimit && transmitBuffer_[0] != 0) { //The next addition of bytes would be the frame size plus also the frame size number. This number is needed by receiver to decode. An addistion byte for the crc.
                    auto frameLen = transmitBuffer_[0];
                    #ifdef SX1280_DEBUG
                        Serial.printf("Segment is %d bytes long\n", frameLen);
                    #endif
                    //transmitBuffer_.removeFront(); //Dont remove this. This is needed by the 
                    for (int i = 0; i < frameLen + 1; i++) {
                        buffer[i + bufferSize] = transmitBuffer_[i];
                        crc += buffer[i + bufferSize];
                    }
                    transmitBuffer_.removeFront(frameLen + 1);
                    bufferSize += frameLen + 1;
                }
                buffer[bufferSize] = crc;

                #ifdef SX1280_DEBUG
                    Serial.printf("Finished making buffer. Sending data! Buffer length: %d\n", bufferSize);
                #endif
                if (bufferSize > 0)
                    lora_.transmit(buffer, bufferSize + 1, 0, 12, NO_WAIT);
                else {
                    LOG_MSG("Datalink: No data to send. Failure. Why does the buffer contain data, but the data is marked with 0 length?... Buffer will be cleared\n");
                    transmitBuffer_.clear();
                }

            }

        }

        void Datalink_SX1280::dataframeReceiveFunc(const Core::List<uint8_t> &item) {

            //VRBS_MSG("Received %d bytes from topic to send. Pointer %d \n", item.size(), this);

            auto len = item.size();
            if (len > dataLinkMaxFrameLength)
            {
                LOG_MSG("Datalink: Max frame length exceeded. Failure.\n");
                return; // Max frame length exceeded. Failure.
            }
            if (len > transmitBuffer_.sizeMax() - transmitBuffer_.size() - 1)
            {
                LOG_MSG("Datalink: Buffer overflow. Failure.\n");
                return; // Buffer overflow case. Failure.
            }

            #ifdef SX1280_DEBUG
                Serial.println("Data received to send is " + String(len) + " bytes long");
            #endif

            transmitBuffer_.placeBack(len);
            for (size_t i = 0; i < len; i++)
                transmitBuffer_.placeBack(item[i]);

        }

    }

}