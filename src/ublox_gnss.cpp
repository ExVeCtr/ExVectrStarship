#include "ExVectrCore/print.hpp"

#include "ublox_gnss.hpp"

using namespace VCTR;

void SNSR::UbloxSerialGNSS::_getData()
{

    auto time = Core::NowNs();
    numSats_ = gnss_.getSIV();

    float positionDeviation = (float)gnss_.getHorizontalAccEst() / 1000.0f;
    float altitudeDeviation = (float)gnss_.getVerticalAccEst() / 1000.0f;
    float velocityError = (float)gnss_.getSpeedAccEst() / 1000.0f;

    SNSR::GNSSData gnssData;
    gnssData.numSats = numSats_;
    gnssData.position(0) = (double)gnss_.getLatitude() / 1e7 * DEG_TO_RAD;
    gnssData.position(1) = (double)gnss_.getLongitude() / 1e7 * DEG_TO_RAD;
    gnssData.position(2) = (double)gnss_.getAltitude() / 1000.0;

    gnssData.positionCov(0) = positionDeviation;
    gnssData.positionCov(1) = positionDeviation;
    gnssData.positionCov(2) = altitudeDeviation;

    gnssData.velocity(0) = (float)gnss_.getNedNorthVel() / 1000.0;
    gnssData.velocity(1) = -(float)gnss_.getNedEastVel() / 1000.0;
    gnssData.velocity(2) = -(float)gnss_.getNedDownVel() / 1000.0;

    gnssData.velocityCov = velocityError;

    // LOG_MSG("NEW GNSS DATA: \n lat: %.10f, \n lon: %.10f, \n Alt: %.2f, \n Sats: %d, pdevh: %.2f, pdevv: %.2f, vdev: %.2f\n", gnssData.latitude, gnssData.longitude, gnssData.altitude, gnssData.numSats, positionDeviation, altitudeDeviation, velocityError);

    // int64_t tow = gnss_.getTimeOfWeek()*MILLISECONDS;

    gnssData.positionValid = gnssData.velocityValid = numSats_ >= 5; // && gnss_.getGnssFixOk();

    auto gnssStamped = Core::Timestamped<SNSR::GNSSData>(gnssData, time);

    if (gnssStamped.data.position == gnssStamped.data.position && gnssStamped.data.velocity == gnssStamped.data.velocity && gnssStamped.data.positionCov == gnssStamped.data.positionCov && gnssStamped.data.velocityCov == gnssStamped.data.velocityCov)
    {
        gnssTopic_.publish(gnssStamped);
    }
    else
    {
        LOG_MSG("GNSS Data NaN!\n");
    }
}

bool SNSR::UbloxSerialGNSS::initSensor(HAL::DigitalIO &ioBus)
{

    taskInit();

    return getInitialised();
}

void SNSR::UbloxSerialGNSS::taskCheck()
{

    // During the configuration sequence the task stays unpaused (it runs every 1 ms and returns at once in the
    // wait states). Afterwards it only runs when UART bytes are waiting.
    if (getInitialised() && config_.done())
    {

        if (serialPort_->available() > 0)
        {
            setPaused(false);
        }
    }
}

void SNSR::UbloxSerialGNSS::taskThread()
{

    if (usbPassthrough_ || config_.done())
    {
        readGNSS();
        return;
    }

    runConfiguration();
}

void SNSR::UbloxSerialGNSS::runConfiguration()
{

    if (config_.failed())
    {
        setPaused(true);
        return;
    }

    const auto action = config_.next(Core::NowNs());
    if (action == GnssConfigSequencer::Action::None)
        return; // waiting (retry or GNSS switch wait)

    const int64_t stepStart = Core::NowNs();
    const bool ok = performConfigAction(action);
    const int64_t stepEnd = Core::NowNs();
    if (stepEnd - stepStart > configMaxStepNs_)
        configMaxStepNs_ = stepEnd - stepStart;
    config_.report(ok, stepEnd);

    if (!ok && action == GnssConfigSequencer::Action::Begin)
    {
        if (config_.failed())
        {
            LOG_MSG("Failed to initialise GNSS module at 115200, 9600 and 38400 baud. GNSS task will be paused and exit.\n");
        }
        else
        {
            LOG_MSG("Failed to initialise GNSS module, attempting %lu baud...\n", (unsigned long)config_.baud());
        }
    }
    else if (!ok && action != GnssConfigSequencer::Action::SetSerialRate)
    {
        LOG_MSG("GNSS config step '%s' failed%s\n", GnssConfigSequencer::toString(action),
                config_.failed() ? ", GNSS task will be paused and exit" : "");
    }

    if (config_.failed())
    {
        setInitialised(false);
        setPaused(true);
    }
    else if (config_.done())
    {
        LOG_MSG("GNSS Module configured (%.2f s, longest step %.1f ms, %u steps skipped).\n",
                double(Core::NowNs() - configStartNs_) / Core::SECONDS,
                double(configMaxStepNs_) / Core::MILLISECONDS, (unsigned)config_.skippedSteps());
        setPaused(true); // taskCheck unpauses when PVT bytes arrive
    }
}

bool SNSR::UbloxSerialGNSS::performConfigAction(GnssConfigSequencer::Action action)
{

    using A = GnssConfigSequencer::Action;
    constexpr uint16_t w = GnssConfigSequencer::kStepMaxWaitMs;

    switch (action)
    {
    case A::Begin:
        setupSerial(config_.baud());
        return gnss_.begin(*serialPort_, w);
    case A::SetSerialRate:
        gnss_.setSerialRate(115200, COM_PORT_UART1, w);
        return true;
    case A::Reopen:
        setupSerial(115200);
        return gnss_.begin(*serialPort_, w);
    case A::SetUart1Ubx:
        return gnss_.setUART1Output(COM_TYPE_UBX, w);
    case A::SetNavRate10Hz:
        return gnss_.setNavigationFrequency(10, w);
    case A::SetAutoPvt:
        return gnss_.setAutoPVT(true, w);
    case A::SetDynModelAirborne2g:
        return gnss_.setDynamicModel(DYN_MODEL_AIRBORNE2g, w);
    case A::DisableGlonass:
        return gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GLONASS, w);
    case A::EnableGalileo:
        return gnss_.enableGNSS(true, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GALILEO, w);
    case A::DisableBeidou:
        return gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_BEIDOU, w);
    case A::DisableSbas:
        return gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_SBAS, w);
    case A::DisableQzss:
        return gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_QZSS, w);
    case A::DisableImes:
        return gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_IMES, w);
    case A::DisableGps:
        return gnss_.enableGNSS(false, sfe_ublox_gnss_ids_e::SFE_UBLOX_GNSS_ID_GPS, w);
    case A::SaveConfig:
        return gnss_.saveConfiguration(w);
    case A::None:
        break;
    }
    return false;
}

bool SNSR::UbloxSerialGNSS::readGNSS()
{

    if (usbPassthrough_)
    {

        while (serialPort_->available())
            Serial.write(serialPort_->read());
        while (Serial.available())
            serialPort_->write(Serial.read());

        return false;
    }

    // LOG_MSG("GNSS Driver read update\n");

    bool retVal = false;

    if (gnss_.getPVT(0))
    { // Read data but tell library to not waste time waiting.

        _getData();
        lastMeasurement_ = Core::NowNs();

        retVal = true;
    }
    else if (Core::NowNs() - lastMeasurement_ >= 500 * Core::MILLISECONDS)
    {

        // gnss_.factoryDefault(100);
        // init();
        // gnss_.flushPVT();
        lastMeasurement_ = Core::NowNs();

        LOG_MSG("Long time past sice last measurment. Flushed PVT\n");
    }

    setPaused(true);

    // LOG_MSG("LOOP\n");

    return retVal;
}

void SNSR::UbloxSerialGNSS::setupSerial(uint32_t baudRate)
{

    serialPort_->end();

#ifdef ESP32

    serialPort_->begin(baudRate, 134217756U, rxPin_, txPin_);

#else

    serialPort_->begin(baudRate);

#endif
}

void SNSR::UbloxSerialGNSS::taskInit()
{

    if (usbPassthrough_)
    {
        LOG_MSG("GNSS Driver setup for gnss serial passthrough!...\n");
        serialPort_->begin(115200);
        return;
    }

    // Non-blocking (2026-11-20): the start-up and configuration (baud search 115200/9600/38400, 115200, UBX
    // output, 10 Hz, auto PVT, airborne <2g, Galileo only, save) run as GnssConfigSequencer steps in taskThread,
    // one bounded library call per task run. The old blocking sequence with delay(1000) stalled the scheduler for
    // about 1.3 s at t = 5 s and latched sensorFailure at every boot.
    VRBS_MSG("Starting up gnss module...\n");
    configStartNs_ = Core::NowNs();
    setPaused(false);
}
