#include "ExVectrCore/print.hpp"

#include "ublox_gnss.hpp"

using namespace VCTR;

void SNSR::UbloxSerialGNSS::_getData() {

    auto time = Core::NOW();
    numSats_ = gnss_.getSIV();

    float positionDeviation = (float)gnss_.getHorizontalAccEst()/1000.0f;
    float altitudeDeviation = (float)gnss_.getVerticalAccEst()/1000.0f;
    float velocityError = (float)gnss_.getSpeedAccEst()/1000.0f;

    SNSR::GNSSData gnssData;
    gnssData.numSats = numSats_;
    gnssData.position(0) = (double)gnss_.getLatitude()/1e7*DEG_TO_RAD;
    gnssData.position(1) = (double)gnss_.getLongitude()/1e7*DEG_TO_RAD;
    gnssData.position(2) = (double)gnss_.getAltitude()/1000.0;

    gnssData.positionCov(0) = positionDeviation;
    gnssData.positionCov(1) = positionDeviation;
    gnssData.positionCov(2) = altitudeDeviation;

    gnssData.velocity(0) = (float)gnss_.getNedNorthVel()/1000.0;
    gnssData.velocity(1) = -(float)gnss_.getNedEastVel()/1000.0;
    gnssData.velocity(2) = -(float)gnss_.getNedDownVel()/1000.0;

    gnssData.velocityCov = velocityError;

    //LOG_MSG("NEW GNSS DATA: \n lat: %.10f, \n lon: %.10f, \n Alt: %.2f, \n Sats: %d, pdevh: %.2f, pdevv: %.2f, vdev: %.2f\n", gnssData.latitude, gnssData.longitude, gnssData.altitude, gnssData.numSats, positionDeviation, altitudeDeviation, velocityError);

    //int64_t tow = gnss_.getTimeOfWeek()*MILLISECONDS;

    gnssData.positionValid = gnssData.velocityValid = numSats_ >= 5;// && gnss_.getGnssFixOk();

    gnssTopic_.publish(Core::Timestamped<SNSR::GNSSData>(gnssData, time));

}


bool SNSR::UbloxSerialGNSS::initSensor(HAL::DigitalIO &ioBus) {

    taskInit();

    return getInitialised();

}


void SNSR::UbloxSerialGNSS::taskCheck() {

    if (getInitialised()) {

        if (serialPort_->available() > 0) {
            setPaused(false);
        } 

    }

}


void SNSR::UbloxSerialGNSS::taskThread() {

    readGNSS();

}


bool SNSR::UbloxSerialGNSS::readGNSS() {

    if (usbPassthrough_) {
        
        while (serialPort_->available()) Serial.write(serialPort_->read());
        while (Serial.available()) serialPort_->write(Serial.read());
        
        return false;
    }

    //LOG_MSG("GNSS Driver read update\n");

    bool retVal = false;

    if (gnss_.getPVT(0)) { //Read data but tell library to not waste time waiting.

        _getData();
        lastMeasurement_ = Core::NOW();

        retVal = true;

    } else if (Core::NOW() - lastMeasurement_ >= 500*Core::MILLISECONDS) {

        //gnss_.factoryDefault(100);
        //init();
        //gnss_.flushPVT();
        lastMeasurement_ = Core::NOW();

        LOG_MSG("Long time past sice last measurment. Flushed PVT\n");

    }

    setPaused(true);

    //LOG_MSG("LOOP\n");

    return retVal;

}



void SNSR::UbloxSerialGNSS::setupSerial(uint32_t baudRate) {

    serialPort_->end();

    #ifdef ESP32 

    serialPort_->begin(baudRate, 134217756U, rxPin_, txPin_);

    #else

    serialPort_->begin(baudRate);

    #endif

}



void SNSR::UbloxSerialGNSS::taskInit() {

    if (usbPassthrough_) {
        LOG_MSG("GNSS Driver setup for gnss serial passthrough!...\n");
        serialPort_->begin(115200);
        return;
    }

    VRBS_MSG("Starting up gnss module...\n");

    setupSerial(115200);
    if (!gnss_.begin(*serialPort_)) {

        LOG_MSG("Failed to initialise GNSS module at 15200. Looks like its isnt setup yet. Attempting 9600...\n");

        setupSerial(9600);
        if (!gnss_.begin(*serialPort_)) {
            LOG_MSG("Failed to initialise GNSS module at 9600. Looks like its isnt setup yet. Attempting 38400...\n");
            setupSerial(38400);
            if (!gnss_.begin(*serialPort_)) {
                LOG_MSG("Failed to initialise GNSS module at 38400. GNSS task will be paused and exit.\n");
                setInitialised(false);
                setPaused(true);
                return;
            }
        }

        LOG_MSG("GNSS was initialised at 9600. Will be reconfigured in next step.\n");

    }

    //Serial.begin(String("Staring GNSS module with ") + (moduleStatus_ == eModuleStatus_t::eModuleStatus_NotStarted ? "115200":"9600") + " baud...");

    VRBS_MSG("Configuring GNSS module...\n");

    gnss_.setSerialRate(115200);
    setupSerial(115200);
    if (!gnss_.begin(*serialPort_)) {

        LOG_MSG("GNSS Failed to go into 115200 baud rate. This is very wierd...\n");
        return;

    }

    //LOG_MSG("GNSS Driver setup for gnss serial passthrough!\n");
    gnss_.softwareResetGNSSOnly();
    //gnss_.setUART1Output(COM_TYPE_UBX);
    gnss_.setNavigationFrequency(10);
    gnss_.setAutoPVT(true);
    gnss_.setDynamicModel(DYN_MODEL_AIRBORNE1g);
    //gnss_.getProtocolVersion();
    gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GLONASS);
    gnss_.enableGNSS(true, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GALILEO);
    delay(1000);
    gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_SBAS);
    gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_QZSS);
    gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GPS);
    gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_BEIDOU);

    //gnss_.enableGNSS(true, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GALILEO);
    //gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GPS);
    //gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GLONASS);

    //gnss_.factoryDefault();

    gnss_.saveConfiguration();

    VRBS_MSG("GNSS Module configured.\n");

    //setPriority(150);

}

