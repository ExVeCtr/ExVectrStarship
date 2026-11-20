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
    {
        // Waiting (retry or GNSS switch wait): drain the UART without blocking. Auto-PVT (10 Hz) would otherwise
        // overflow the RX buffer and corrupt the first exchange after the wait (bench 2026-11-20).
        gnss_.checkUblox();
        return;
    }
    lastUbxStatus_ = -1;

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
        // ubx status (sfe_ublox_status_e): 3 timeout, 4 NACK, -1 = a library call without a status
        LOG_MSG("GNSS config step '%s' failed (ubx status %d)%s\n", GnssConfigSequencer::toString(action),
                lastUbxStatus_, config_.failed() ? ", GNSS task will be paused and exit" : "");
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
        logReadBack();
        setPaused(true); // taskCheck unpauses when PVT bytes arrive
    }
}

bool SNSR::UbloxSerialGNSS::ubxPoll(uint8_t cls, uint8_t id)
{
    ubxPayloadLen_ = 0;
    ubxPacket pkt = {cls, id, 0, 0, 0, ubxPayload_, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
                     SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
    const auto st = gnss_.sendCommand(&pkt, GnssConfigSequencer::kStepMaxWaitMs);
    lastUbxStatus_ = int(st);
    if (st != SFE_UBLOX_STATUS_DATA_RECEIVED || pkt.cls != cls || pkt.id != id)
        return false;
    ubxPayloadLen_ = pkt.len > MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : pkt.len;
    return true;
}

bool SNSR::UbloxSerialGNSS::ubxWrite(uint8_t cls, uint8_t id, uint16_t len)
{
    ubxPacket pkt = {cls, id, len, 0, 0, ubxPayload_, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
                     SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
    const auto st = gnss_.sendCommand(&pkt, GnssConfigSequencer::kStepMaxWaitMs);
    lastUbxStatus_ = int(st);
    return st == SFE_UBLOX_STATUS_DATA_SENT; // ACK only
}

void SNSR::UbloxSerialGNSS::logReadBack()
{
    // One line, once per boot (the sequence runs once): what the receiver really has, not what was asked for.
    const auto &c = readGnss_;
    const auto bits = [](uint8_t m, char *out) {
        // bit 0 GPS, 1 GLONASS, 2 BeiDou, 3 Galileo (UBX-MON-GNSS)
        out[0] = 0;
        const char *names[4] = {"GPS", "GLONASS", "BeiDou", "Galileo"};
        for (int i = 0; i < 4; i++)
        {
            if (m & (1 << i))
            {
                if (out[0])
                    strcat(out, "+");
                strcat(out, names[i]);
            }
        }
        if (!out[0])
            strcat(out, "none");
    };
    char inUse[40], supported[40];
    bits(readInUse_.enabled, inUse);
    bits(readInUse_.supported, supported);
    LOG_MSG("GNSS read-back: CFG-GNSS %s GPS %s Galileo %s GLONASS %s BeiDou %s SBAS %s QZSS %s IMES %s (trk ch %u/%u); "
            "MON-GNSS %s in use %s (supported %s); nav rate %s %.1f Hz (meas %u ms x %u); GPS+Galileo as intended: %s\n",
            c.valid ? "ok:" : "FAILED", c.gps ? "on" : "off", c.galileo ? "on" : "off", c.glonass ? "on" : "off",
            c.beidou ? "on" : "off", c.sbas ? "on" : "off", c.qzss ? "on" : "off", c.imes ? "on" : "off",
            (unsigned)c.numTrkChUse, (unsigned)c.numTrkChHw, readInUse_.valid ? "ok:" : "FAILED", inUse, supported,
            readRate_.valid ? "ok:" : "FAILED", double(readRate_.hz()), (unsigned)readRate_.measRateMs,
            (unsigned)readRate_.navRateCycles, constellationsAsIntended() ? "yes" : "NO");
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
    case A::PollGnssForWrite:
        gnssPolledForWrite_ = ubxPoll(UBX_CLASS_CFG, UBX_CFG_GNSS);
        if (gnssPolledForWrite_)
        {
            // The configuration the receiver had at boot, BEFORE this sequence writes it: shows whether an earlier
            // save persisted in its non-volatile memory (only after a receiver power cycle; a Teensy-only reboot
            // leaves the receiver's RAM configuration in place).
            GnssConstellations atBoot;
            decodeCfgGnss(ubxPayload_, ubxPayloadLen_, atBoot);
            LOG_MSG("GNSS config found at boot (before write): GPS %s Galileo %s GLONASS %s BeiDou %s SBAS %s QZSS %s "
                    "IMES %s; GPS+Galileo as intended: %s\n",
                    atBoot.gps ? "on" : "off", atBoot.galileo ? "on" : "off", atBoot.glonass ? "on" : "off",
                    atBoot.beidou ? "on" : "off", atBoot.sbas ? "on" : "off", atBoot.qzss ? "on" : "off",
                    atBoot.imes ? "on" : "off", SNSR::constellationsAsIntended(atBoot) ? "yes" : "NO");
        }
        return gnssPolledForWrite_;
    case A::SetGnssGpsGalileo:
        // GPS + Galileo, all others off, in ONE CFG-GNSS message (user decision 2026-11-20). Retries resend the same
        // modified payload (applyGpsGalileo is idempotent).
        if (!gnssPolledForWrite_ || !applyGpsGalileo(ubxPayload_, ubxPayloadLen_))
            return false;
        return ubxWrite(UBX_CLASS_CFG, UBX_CFG_GNSS, ubxPayloadLen_);
    case A::SaveConfig:
        // UBX-CFG-CFG save (same payload as SFE_UBLOX_GNSS::saveConfiguration), through ubxWrite so a failure logs
        // the receiver's answer (NACK or timeout).
        memset(ubxPayload_, 0, 12);
        ubxPayload_[4] = 0xFF;
        ubxPayload_[5] = 0xFF;
        return ubxWrite(UBX_CLASS_CFG, UBX_CFG_CFG, 12);
    case A::ReadGnssConfig:
        return ubxPoll(UBX_CLASS_CFG, UBX_CFG_GNSS) && decodeCfgGnss(ubxPayload_, ubxPayloadLen_, readGnss_);
    case A::ReadGnssInUse:
        return ubxPoll(UBX_CLASS_MON, UBX_MON_GNSS) && decodeMonGnss(ubxPayload_, ubxPayloadLen_, readInUse_);
    case A::ReadNavRate:
        return ubxPoll(UBX_CLASS_CFG, UBX_CFG_RATE) && decodeCfgRate(ubxPayload_, ubxPayloadLen_, readRate_);
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
    // output, 10 Hz, auto PVT, airborne <2g, GPS + Galileo, save, read-back) run as GnssConfigSequencer steps in
    // taskThread, one bounded library call per task run. The old blocking sequence with delay(1000) stalled the scheduler for
    // about 1.3 s at t = 5 s and latched sensorFailure at every boot.
    VRBS_MSG("Starting up gnss module...\n");
    configStartNs_ = Core::NowNs();
    setPaused(false);
}
