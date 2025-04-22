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
#include "ExVectrArduinoPlatform/interrupt_event.hpp"

#include "ExVectrSensor/Sensors/mpu9250.hpp"
#include "ExVectrSensor/Sensors/bme280.hpp"
#include "ExVectrSensor/Sensors/qmc5883.hpp"

#include "ublox_gnss.hpp"

#include "ExVectrNetwork/network_node.hpp"
#include "ExVectrNetwork/transport_topic.hpp"

#include "ExVectrActuator/pwm_output.hpp"
#include "ExVectrActuator/servo_control.hpp"

#include "ExVectrDSP/topic_coord_transform.hpp"
#include "ExVectrDSP/calibrator_gyroscope.hpp"
#include "ExVectrDSP/calibrator_magnetometer.hpp"
#include "ExVectrDSP/calibrator_manager.hpp"
#include "ExVectrDSP/imu_attitude_ekf.hpp"
#include "ExVectrDSP/imu_attitude_cf.hpp"
#include "ExVectrDSP/imu_gps_position_kf.hpp"

#include "ExVectrControl/control_rocket.hpp"


#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrArduinoPlatform/memory_eeprom.hpp"

#include "ExVectrData/memory_internal.hpp"
#include "ExVectrData/memory_manager.hpp"

#include "ExVectrPackets/packet_vehicle.hpp"

#include "subsystems.hpp"
#include "telecommand.hpp"
#include "telemetry.hpp"

#include "starship_tvc.hpp"

#include "board_v_1_0.h"
#include "starship_connections_v_1_0.h"
#include "memory_keys.hpp"

//#include "SX128XLT.h"
#include "Adafruit_SSD1306.h"

#include "datalink_sx1280.hpp"

//#include "sx1280_driver/datalink_sx1280.hpp"

using namespace VCTR;

#define PMW_CS_PIN 29
#define PMW_INT_PIN 28

//auto gnssRef = Math::Vector<double, 3>({49.945884*DEG_TO_RAD, 6.699575*DEG_TO_RAD, 324.7});


//Hardware
Platform::PinGPIO mpuIntPin(MPU9250_INT_PIN);
Platform::Interrupt_GPIO<SNSR::MPU9250Driver> mpuIntHandler(MPU9250_INT_PIN);
Platform::PinGPIO mpuPin(MPU9250_NCS_PIN);
Platform::BusSPIDevice spiMPU(SPI, mpuPin, true);
SNSR::MPU9250Driver mpuDriver(spiMPU, true);

//Platform::PinGPIO bmePin(BME280_NCS_PIN);
//Platform::BusSPIDevice spiBME(SPI, bmePin, true);
Platform::BusI2CDevice i2cBME(Wire, 0x76);
SNSR::BME280Driver bme(i2cBME);

SNSR::UbloxSerialGNSS gnss(Serial5);

Platform::PinGPIO pmwPin(PMW_CS_PIN);

Platform::BusI2CDevice qmcBus(Wire, 0x0D);
SNSR::QMC5883Driver qmc(qmcBus);

Platform::PinPWM servoTVCXPPin(TVC_SERVO_PIN_3);
Platform::PinPWM servoTVCXNPin(TVC_SERVO_PIN_1);
Platform::PinPWM servoTVCYPPin(TVC_SERVO_PIN_2);
Platform::PinPWM servoTVCYNPin(TVC_SERVO_PIN_4);
Platform::PinPWM motorCWPIN(MOTOR_PIN_CW);
Platform::PinPWM motorCCWPIN(MOTOR_PIN_CCW);
//ACTR::PWM_Output servoTest(servoTVCYNPin, ACTR::PWM_Output_Protocol::STANDARD);
//ACTR::Servo_Control servoTest(40, -40, 10, 1, servoTVCXPPin, ACTR::PWM_Output_Protocol::STANDARD);



//Memory managment
Data::Memory_ArduinoEEPROM eeprom(EEPROM, EEPROM.length());
Data::Memory_Internal<512> internalMemory;
Data::Memory_Manager memoryManager(internalMemory);


//Sensor Processing
DSP::TopicCoordTransform<float> gyroTransformTopic;
DSP::TopicCoordTransform<float> accTransformTopic;
DSP::TopicCoordTransform<float> magTransformTopic;

DSP::IMUAttitudeEKFTask imuTask(1*Core::MILLISECONDS);
DSP::IMUGPSPositionKalmanTask posEstTask(100*Core::MILLISECONDS);

//DSP::Calibrator_Magnetometer magCalibrator;
//DSP::Calibrator_Gyroscope gyroCalibrator;
//DSP::Calibrator_Manager<float> gyroCalibratorManager(gyroCalibrator, gyroTransformTopic, internalMemory, 100);
//DSP::Calibrator_Manager<float> magCalibratorManager(magCalibrator, magTransformTopic, internalMemory, 102);


//Telecoms
Core::Topic<Telecommand> telecommandTopic;
//Core::Topic<Telemetry

void telecommandCallback(const Telecommand& cmd);
//Core::StaticCallback_Subscriber<Telecommand> telecommandSubr(telecommandTopic, telecommandCallback);


//Networking

Net::Datalink_SX1280 datalinkSX1280(SX1280_NSS_PIN, SX1280_NRESET_PIN, SX1280_RFBUSY_PIN, SX1280_DIO1_PIN, SX1280_TXEN_PIN, SX1280_RXEN_PIN);
Net::NetworkNode networkNode(1, datalinkSX1280, 2*Core::SECONDS);
//Net::NetworkMonitor networkMonitor(networkNode);

Core::Topic<Net::PacketAttitude> attitudeDataTopic;
Core::Topic<Net::PacketPosition> positionDataTopic;
Core::Topic<uint8_t> connections;

Core::Topic<VehicleMode> vehicleModeTopic;
Core::Topic<MissionState> missionStateTopic;
Core::Topic<SensoryState> sensoryStateTopic;
Core::Topic<FailureState> failureStateTopic;

Core::Topic<Net::G> gpsDataTopic;

Net::TransportTopic<Net::PacketAttitude> attitudeDataTransport(10, UINT16_MAX, networkNode, attitudeDataTopic);
Net::TransportTopic<Net::PacketPosition> positionDataTransport(11, UINT16_MAX, networkNode, positionDataTopic);
Net::TransportTopic<GPSData> gpsDataTransport(12, UINT16_MAX, networkNode, gpsDataTopic);

Net::TransportTopic<VehicleMode> vehicleModeTransport(50, UINT16_MAX, networkNode, vehicleModeTopic);
Net::TransportTopic<MissionState> missionStateTransport(51, UINT16_MAX, networkNode, missionStateTopic);
Net::TransportTopic<SensoryState> sensoryStateTransport(52, UINT16_MAX, networkNode, sensoryStateTopic);
Net::TransportTopic<FailureState> failureStateTransport(53, UINT16_MAX, networkNode, failureStateTopic);

Net::TransportTopic<uint8_t> connectionsTransport(100, UINT16_MAX, networkNode, connections);

Net::TransportTopic<Telecommand> telecommandTransport(5, UINT16_MAX, networkNode, telecommandTopic);


//Control
CTRL::ControlRocket controlRocket(1, 20, 0.5);

CTRL::StarshipTVC starshipTVC(servoTVCXPPin, servoTVCXNPin, servoTVCYPPin, servoTVCYNPin, motorCWPIN, motorCCWPIN);


//Functions

void attitudeTelemetryCallback(const Core::Timestamped<Math::Vector<float, 7>>& data) {

    static int64_t lastSend = 0;
    if (Core::NOW() - lastSend < 0.1*Core::SECONDS) return;
    lastSend = Core::NOW();

    Net::PacketAttitude packet({
        data.data(0), data.data(1), data.data(2)
    }, {
        data.data(3), data.data(4), data.data(5), data.data(6)
    });
    attitudeDataTopic.publish(packet);

    //LOG_MSG("Attitude: %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", data.data(0), data.data(1), data.data(2), data.data(3), data.data(4), data.data(5), data.data(6));
    
}
Core::StaticCallback_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attitudeTeleSubr(imuTask.getAttitudeEstTopic(), attitudeTelemetryCallback);

void positionTelemetryCallback(const Core::Timestamped<Math::Vector<float, 6>>& data) {

    static int64_t lastSend = 0;
    if (Core::NOW() - lastSend < 0.1*Core::SECONDS) return;
    lastSend = Core::NOW();

    Net::PacketPosition packet({
        data.data(0), data.data(1), data.data(2)
    }, {
        data.data(3), data.data(4), data.data(5)
    });
    positionDataTopic.publish(packet);

}
Core::StaticCallback_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> positionTeleSubr(posEstTask.getStateEstTopic(), positionTelemetryCallback);

void gnssTelemetryCallback(const Core::Timestamped<SNSR::GNSSData>& data) {

    static int64_t lastSend = 0;
    static uint8_t counter = 0;
    if (Core::NOW() - lastSend < 0.1*Core::SECONDS) return;
    lastSend = Core::NOW();

    auto position = posEstTask.calcRelPosFromGNSS(data.data.position, posEstTask.getPositionReference());

    GPSData packet;
    packet.px = position(0);
    packet.py = position(1);
    packet.pz = position(2);
    packet.vx = data.data.velocity(0);
    packet.vy = data.data.velocity(1);
    packet.vz = data.data.velocity(2);
    packet.numSats = data.data.numSats;
    packet.counter = counter++;

    gpsDataTopic.publish(packet);

}
//Core::StaticCallback_Subscriber<Core::Timestamped<SNSR::GNSSData>> gnssTeleSubr(gnss.getGNSSTopic(), gnssTelemetryCallback);



class MissionGuidanceTask : public Core::Task_Periodic
{
private:

    Core::Time_Source missionTime_;

    /// @brief In form: [V, P] with V and P as 3D vectors in reference frame.
    Core::Topic<Math::Vector<float, 6>> positionSetpointTopic_;
    Math::Vector<float, 6> positionSetpoint_;

    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> positionIsSubr_;
    Math::Vector<float, 6> positionIs_;

    int64_t missionStartTime_ = 0;

    int64_t startStartupTime_ = 0; // When the vehicle goes into startup. (Actuators enable and ready, vehicle remains at start position. This is usually negative)

    int64_t startDescentTime_ = 0; // When the vehicle goes into descent mode. (descends until landing threshold is met. Usually at time 0)
    float ascentRate_ = 0; // How fast the vehicle should go up in m/s.

    int64_t startHoverTime_ = 0; // When the vehicle goes into hover mode. (flys to position and hovers. Usually at time 0)
    float hoverAltitude_ = 0; // The altitude the vehicle should hover at in m.


    float landingThresholdDistance_ = 0; // If the vehicle set position is further down than this for a certain time, it will be considered as landed and the mission will be finished.
    int64_t landingThresholdTime_ = 0; // How long the vehicle has to be below the landing threshold distance to be considered as landed.
    float descentRate_ = 0; // How fast the vehicle should go down in m/s.
    int64_t landingThresMetTime_ = 0; // When the vehicle is below the landing threshold distance.


    bool missionBeginTrigger_ = false; // Trigger for the mission start. This is set to true when the vehicle can begin the mission.
    
    MissionState missionState_ = MissionState::MissionState_Idle;

    bool actuatorsEnabled_ = false; // If the actuators are enabled or not. This is set to true when the vehicle is in startup or hover mode.

    bool hoverModeInitialised_ = false; // If the hover mode is initialised or not. This is set to true when the vehicle is in hover mode.
    int64_t hoverModeLastUpdate_ = 0; // When the vehicle goes into hover mode. (flys to position and hovers. Usually at time 0)

    bool descentModeInitialised_ = false; // If the descent mode is initialised or not. This is set to true when the vehicle is in descent mode.
    int64_t descentModeLastUpdate_ = 0; // When the vehicle goes into descent mode. (descends until landing threshold is met)


public:

    MissionGuidanceTask() : Task_Periodic("Mission Guidance", 0.1*Core::SECONDS)
    {

        Core::getSystemScheduler().addTask(*this);
        //setPriority(500);

        startStartupTime_ = -5*Core::SECONDS;

        startHoverTime_ = 0*Core::SECONDS;
        ascentRate_ = 0.5; // m/s
        hoverAltitude_ = 1.0; // m

        startDescentTime_ = 10*Core::SECONDS;
        descentRate_ = 0.2; // m/s
        landingThresholdDistance_ = 0.5; // m
        landingThresholdTime_ = 5*Core::SECONDS; // s

        positionSetpoint_ = {0, 0, 0, 0, 0, 0}; // In form: [Vx, Vy, Vz, Px, Py, Pz] in reference frame.

        missionStartTime_ = Core::END_OF_TIME; // Set to end of time to keep the mission from starting

    }

    /**
     * @brief Current state of the mission.
     */
    MissionState getMissionState() {
        return missionState_;
    }

    /**
     * * @brief Used to start the mission at a given time. This will put the vehicle into idle mode 
     */
    void beginMission(int64_t startTime) {
        missionBeginTrigger_ = true;
        missionState_ = MissionState::MissionState_Idle;
        missionStartTime_ = startTime;
    }

    bool getActuatorsEnabled() {
        return actuatorsEnabled_;
    }


    void taskInit() override
    {
        
        controlRocket.subscribeSetpoint(positionSetpointTopic_);
        positionIsSubr_.subscribe(posEstTask.getStateEstTopic());

    }

    void taskThread() override 
    {

        if (positionIsSubr_.isDataNew()) {
            positionIs_ = positionIsSubr_.getItem().data;
        }

        switch (missionState_)
        {
        case MissionState::MissionState_Idle:
            missionIdle();
            break;

        case MissionState::MissionState_Startup:
            missionStartup();
            break;
        
        case MissionState::MissionState_Hover:
            missionHover();
            break;

        case MissionState::MissionState_Descent:
            missionDescent();
            break;

        case MissionState::MissionState_Landed:
            missionLanded();
            break;
        
        default:
            missionState_ = MissionState::MissionState_Idle;
            missionBeginTrigger_ = false;
            break;
        }

    }


private:

    void missionIdle() {

        actuatorsEnabled_ = false; // Disable actuators
        positionSetpointTopic_.publish(positionSetpoint_); // Publish the setpoint to the control system

        posEstTask.enableZeroingMode(true); // Enable zeroing mode for the position estimator

    }

    void missionStartup() {

        actuatorsEnabled_ = true; // Enable actuators
        posEstTask.enableZeroingMode(false); // Disable zeroing mode for the position estimator. We want to see if the system is stable.

        hoverModeInitialised_ = false; // Reset hover mode initialisation
        descentModeInitialised_ = false; // Reset descent mode initialisation

    }
    
    void missionHover() {

        float dTime = Core::NOW() - hoverModeLastUpdate_;

        if (!hoverModeInitialised_) {
            // Set the setpoint to the current position and velocity of the vehicle
            positionSetpoint_ = {0, 0, 0, 0, 0, 0}; // In form: [Vx, Vy, Vz, Px, Py, Pz] in reference frame.
            hoverModeInitialised_ = true; // Set hover mode initialised to true
            dTime = 0; // Reset dTime to 0
        }

        actuatorsEnabled_ = true;   

        if (positionSetpoint_(5) < hoverAltitude_)
            positionSetpoint_(5) += dTime*ascentRate_; // Update the setpoint position in the reference frame
        
        if (positionSetpoint_(5) > hoverAltitude_)
            positionSetpoint_(5) = hoverAltitude_; // Limit the setpoint position to the hover altitude

        if (missionTime_.NOW() > startDescentTime_) {
            missionState_ = MissionState::MissionState_Descent; // Go to descent mode
            descentModeInitialised_ = false; // Reset descent mode initialisation
        }

    }

    void missionDescent() {

        float dTime = Core::NOW() - descentModeLastUpdate_;

        if (!descentModeInitialised_) {
            // Set the setpoint to the current position and velocity of the vehicle
            descentModeInitialised_ = true; // Set descent mode initialised to true
            dTime = 0; // Reset dTime to 0
        }

        actuatorsEnabled_ = true;   

        positionSetpoint_(5) -= dTime*descentRate_; // Update the setpoint position in the reference frame
        
        if (positionSetpoint_(5) - positionIs_(5) > landingThresholdDistance_) { //We keep updateting the threshold time. We stop when the vehicle is above the threshold distance. This triggers the start of the timer.
            landingThresMetTime_ = Core::NOW(); // Set the time when the landing threshold was met
        } 

        if (Core::NOW() - landingThresMetTime_ > landingThresholdTime_) { // If the vehicle is below the landing threshold distance for a certain time, we consider it as landed.
            missionState_ = MissionState::MissionState_Landed; // Go to landed mode
        }

    }

    void missionLanded() {

        actuatorsEnabled_ = false; // Disable actuators
        positionSetpoint_ = {0, 0, 0, 0, 0, 0}; // Set the setpoint to the current position and velocity of the vehicle

    }


};

MissionGuidanceTask missionGuidanceTask;

/**
 * This class takes care of enablign, disabling and setting the actuators for the rocket. It also prepares the system for mission start and signals when something is wrong.
 */
class VehicleSafetyAndControlTask : public Core::Task_Periodic
{
private:

    const int64_t DATA_TIMEOUT = 0.5*Core::SECONDS;

    const float POSITION_OUTOFBOUNDS_STATIONARY = 2.0f; // m
    const float ANGLE_OUTOFBOUNDS_STATIONARY = 10*DEG_TO_RAD; // rad

    const float POSITION_OUTOFBOUNDS_FLIGHT = 20.0f; // m
    const float ANGLE_OUTOFBOUNDS_FLIGHT = 45*DEG_TO_RAD; // rad

    Core::Simple_Subscriber<Core::Timestamped<SNSR::GNSSData>> gnssSubr;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> gyroSubr;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> magSubr;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 1>>> baroSubr;

    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posEstSubr;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attEstSubr;
    
    VehicleMode vehicleMode_ = VehicleMode::VehicleMode_Startup;
    SensoryState sensoryState_;

    FailureState failureState_;

    bool allSystemsInitialised_ = false; // If all systems are initialised or not. This is set to true when all systems are initialised.


public:

    VehicleSafetyAndControlTask() : Task_Periodic("Vehicle Safety and Control", 0.1*Core::SECONDS)
    {
        Core::getSystemScheduler().addTask(*this);
        //setPriority(500);
        sensoryState_.baro = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.imu = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.mag = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.gnss = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.attitudeKF = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.positionKF = TelemetrySensor::TelemetrySensor_Init;

        failureState_.attitudeFailure = false;
        failureState_.positionFailure = false;
        failureState_.sensorFailure = false;
        failureState_.positionOutOfBounds = false;
        failureState_.attitudeOutOfBounds = false;

    }

    void taskInit() override
    {
        
        gnssSubr.subscribe(gnss.getGNSSTopic());
        gyroSubr.subscribe(gyroTransformTopic.getOutputTopic());
        magSubr.subscribe(magTransformTopic.getOutputTopic());
        baroSubr.subscribe(bme.getBaroTopic());
        posEstSubr.subscribe(posEstTask.getStateEstTopic());
        attEstSubr.subscribe(imuTask.getAttitudeEstTopic());

    }

    void taskThread() override {

        if (!systemsInitialised()) {
            starshipTVC.enableActuators(false); // Disable actuators
            vehicleShutdownControl(); // Disable everything
            return; // Wait until all systems are initialised
        }

        checkForFailures();
        
        auto vehicleReady = vehicleIsReady();
        vehicleShutdownControl(!vehicleReady);

    }

    bool systemsInitialised() {

        if (allSystemsInitialised_) {
            return true;
        }

        if (!gnssSubr.isDataNew()) {
            return false;
        }

        if (!gyroSubr.isDataNew()) {
            return false;
        }

        if (!magSubr.isDataNew()) {
            return false;
        }

        if (!baroSubr.isDataNew()) {
            return false;
        }

        if (!posEstSubr.isDataNew()) {
            return false;
        }

        if (!attEstSubr.isDataNew()) {
            return false;
        }

        allSystemsInitialised_ = true; // Set all systems initialised to true

        return true;

    }

    void checkForFailures() {   

        //Check sensor for data timeout which indicates a failure

        if (Core::NOW() - gnssSubr.getItem().timestamp > DATA_TIMEOUT) {
            vehicleMode_ = VehicleMode::VehicleMode_Failure;
            failureState_.sensorFailure = true;
            sensoryState_.gnss = TelemetrySensor::TelemetrySensor_Failure;
        } else if (sensoryState_.gnss != TelemetrySensor::TelemetrySensor_Calib && sensoryState_.gnss != TelemetrySensor::TelemetrySensor_Failure) {
            sensoryState_.gnss = TelemetrySensor::TelemetrySensor_Ready;
        }

        if (Core::NOW() - gyroSubr.getItem().timestamp > DATA_TIMEOUT) {
            vehicleMode_ = VehicleMode::VehicleMode_Failure;
            failureState_.sensorFailure = true;
            sensoryState_.imu = TelemetrySensor::TelemetrySensor_Failure;
        } else if (sensoryState_.imu != TelemetrySensor::TelemetrySensor_Calib && sensoryState_.imu != TelemetrySensor::TelemetrySensor_Failure) {
            sensoryState_.imu = TelemetrySensor::TelemetrySensor_Ready;
        }

        if (Core::NOW() - magSubr.getItem().timestamp > DATA_TIMEOUT) {
            vehicleMode_ = VehicleMode::VehicleMode_Failure;
            failureState_.sensorFailure = true;
            sensoryState_.mag = TelemetrySensor::TelemetrySensor_Failure;
        } else if (sensoryState_.mag != TelemetrySensor::TelemetrySensor_Calib && sensoryState_.mag != TelemetrySensor::TelemetrySensor_Failure) {
            sensoryState_.mag = TelemetrySensor::TelemetrySensor_Ready;
        }

        if (Core::NOW() - baroSubr.getItem().timestamp > DATA_TIMEOUT) {
            vehicleMode_ = VehicleMode::VehicleMode_Failure;
            failureState_.sensorFailure = true;
            sensoryState_.baro = TelemetrySensor::TelemetrySensor_Failure;
        } else if (sensoryState_.baro != TelemetrySensor::TelemetrySensor_Calib && sensoryState_.baro != TelemetrySensor::TelemetrySensor_Failure) {
            sensoryState_.baro = TelemetrySensor::TelemetrySensor_Ready;
        }

        if (Core::NOW() - posEstSubr.getItem().timestamp > DATA_TIMEOUT) {
            vehicleMode_ = VehicleMode::VehicleMode_Failure;
            failureState_.sensorFailure = true;
            sensoryState_.positionKF = TelemetrySensor::TelemetrySensor_Failure;
        } else if (sensoryState_.positionKF != TelemetrySensor::TelemetrySensor_Calib && sensoryState_.positionKF != TelemetrySensor::TelemetrySensor_Failure) {
            sensoryState_.positionKF = TelemetrySensor::TelemetrySensor_Ready;
        }

        if (Core::NOW() - attEstSubr.getItem().timestamp > DATA_TIMEOUT) {
            vehicleMode_ = VehicleMode::VehicleMode_Failure;
            failureState_.sensorFailure = true;
            sensoryState_.attitudeKF = TelemetrySensor::TelemetrySensor_Failure;
        } else if (sensoryState_.attitudeKF != TelemetrySensor::TelemetrySensor_Calib && sensoryState_.attitudeKF != TelemetrySensor::TelemetrySensor_Failure) {
            sensoryState_.attitudeKF = TelemetrySensor::TelemetrySensor_Ready;
        }


        //Check if any of the data is out of bounds which indicates a safety issue

        auto missionMode = missionGuidanceTask.getMissionState();
        bool stationary = false;
        stationary |= (missionMode == MissionState::MissionState_Idle);
        stationary |= (missionMode == MissionState::MissionState_Landed);

        auto position = posEstSubr.getItem().data;
        Math::Quat<float> attitude = attEstSubr.getItem().data.block<4, 1>(3, 0);

        auto zAxisBody = attitude.rotate(Math::Vector<float, 3>({0, 0, 1}));

        auto startDistance = position.magnitude(3);
        auto tilt = zAxisBody.getAngleTo(Math::Vector<float, 3>({0, 0, 1}));

        if (stationary) {

            if (startDistance > POSITION_OUTOFBOUNDS_STATIONARY) {
                failureState_.positionOutOfBounds = true;
                vehicleMode_ = VehicleMode::VehicleMode_Failure;
            } 

            if (tilt > ANGLE_OUTOFBOUNDS_STATIONARY) {
                failureState_.attitudeOutOfBounds = true;
                vehicleMode_ = VehicleMode::VehicleMode_Failure;
            }

        } else {

            if (startDistance > POSITION_OUTOFBOUNDS_FLIGHT) {
                failureState_.positionOutOfBounds = true;
                vehicleMode_ = VehicleMode::VehicleMode_Failure;
            } 

            if (tilt > ANGLE_OUTOFBOUNDS_FLIGHT) {
                failureState_.attitudeOutOfBounds = true;
                vehicleMode_ = VehicleMode::VehicleMode_Failure;
            }

        }

    }



    bool vehicleIsReady() {

        if (
            allSystemsInitialised_ 
            && sensoryState_.imu == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.baro == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.mag == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.gnss == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.attitudeKF == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.positionKF == TelemetrySensor::TelemetrySensor_Ready
            && !failureState_.sensorFailure
            && !failureState_.attitudeFailure
            && !failureState_.positionFailure
            && !failureState_.positionOutOfBounds
            && !failureState_.attitudeOutOfBounds
            && vehicleMode_ != VehicleMode::VehicleMode_Failure
        ) {
            return true;
        }

        return false;
    }

    void vehicleShutdownControl(bool shutdown) {
        
        if (shutdown) {
            starshipTVC.enableActuators(false); // Enable actuators
            return;
        }

        starshipTVC.enableActuators(missionGuidanceTask.getActuatorsEnabled()); //Give mission guidance control over the actuators

    }

    void clearFailures() {
        failureState_.sensorFailure = false;
        failureState_.attitudeFailure = false;
        failureState_.positionFailure = false;
        failureState_.positionOutOfBounds = false;
        failureState_.attitudeOutOfBounds = false;
    }


};
VehicleSafetyAndControlTask vehicleSafetyAndControlTask;





class DisplayTask : public Core::Task_Periodic
{
public:

    static constexpr int SCREEN_WIDTH = 128; // OLED display width, in pixels
    static constexpr int SCREEN_HEIGHT = 32; // OLED display height, in pixels
    static constexpr int SCREEN_ADDRESS = 0x3C; // I2C address for the screen

    Adafruit_SSD1306 display_;

    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> accSub;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSub;

    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> accelSub;

    Core::Simple_Subscriber<Core::Timestamped<SNSR::GNSSData>> gnssSub;

    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> gyroSub;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 3>>> biasSub;

    int8_t counter = 0;

    int connectionCounter = 0;
    bool lastConnectionState = false;

    DisplayTask() : Task_Periodic("Display task", 0.1*Core::SECONDS)
    {
        Core::getSystemScheduler().addTask(*this);
        //setPriority(500);
    }

    void taskInit() override
    {

        auto ret = display_.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
        if (!ret) {
            Core::printM("Display init failed!\n");
            Core::getSystemScheduler().removeTask(*this);
            return;
        }
        display_.setRotation(0);
        //display_.clearDisplay();
        display_.display();

        attSub.subscribe(imuTask.getAttitudeEstTopic());

        accSub.subscribe(posEstTask.getStateEstTopic());

        accelSub.subscribe(accTransformTopic.getOutputTopic());

        gnssSub.subscribe(gnss.getGNSSTopic());

        gyroSub.subscribe(gyroTransformTopic.getOutputTopic());
        biasSub.subscribe(imuTask.getBiasEstTopic());

        //magSub.subscribe(qmc.getMagTopic());


    }

    void taskThread() override
    {   

        /*if (magSub.isDataNew()) {

            auto mag = magSub.getItem().data.val;

            for (size_t i = 0; i < 3; i++)
            {
                if (mag[i][0] < magMin[i][0]) magMin[i][0] = mag[i][0];
                if (mag[i][0] > magMax[i][0]) magMax[i][0] = mag[i][0];
            }

            Core::printM("Time: %f, Mag: ", Core::NOWSeconds());
            mag.printTo(Core::printM);
            Core::printM("\n");

            Core::printM("Mag min: ");
            magMin.printTo(Core::printM);
            Core::printM("\n");

            Core::printM("Mag max: ");
            magMax.printTo(Core::printM);
            Core::printM("\n");

        }*/

        //magSub.getStandardDeviation().printTo(Core::printM);

        //imuTask.getState().printTo(Core::printM);
        //imuTask.getCovariance().printTo(Core::printM);

        /*if (Serial.available()) {
            
            Serial.clear();
            //Serial.flush();
            posEstTask.setState(DSP::ValueCov<float, 6>(0, 1));
            imuTask.setState(DSP::ValueCov<float, 7>({0, 0, 0, 1, 0, 0, 0}, 1));

        }*/

        /*if (accSub.isDataNew()) {

            auto accData = accSub.getItem();
            
            Core::printM("New Data: t: %.2f\n", double(accData.timestamp)/Core::SECONDS);
            accData.data.val.printTo(Core::printM);
            accData.data.cov.printTo(Core::printM);

        }*/

        //return;
        /*Core::Timestamped<SNSR::GNSSData> dummyGNSSData;
        dummyGNSSData.timestamp = Core::NOW();
        dummyGNSSData.data.position = gnssRef;
        dummyGNSSData.data.positionCov = Math::Vector_F({3, 3, 3});
        dummyGNSSData.data.velocity = Math::Vector_F({0, 0, 0});
        dummyGNSSData.data.velocityCov = Math::Vector_F({1, 1, 1});
        dummyGNSSData.data.numSats = 5;
        dummyGNSSData.data.positionValid = true;
        dummyGNSSData.data.velocityValid = true;
        gnss.getGNSSTopic().publish(dummyGNSSData);*/

        float angle = 0;
        float north = 0;

        if (attSub.isDataNew()) {

            auto attData = attSub.getItem();
            
            Math::Quat_F attQuat = attData.data.block<4, 1>(3, 0);

            auto downRot = attQuat.conjugate().rotate(Math::Vector_F({0, 0, -1}));
            angle = Math::Vector_F({0, 0, -1}).getAngleTo(downRot);

            //calulate the compass heading
            auto northRot = attQuat.conjugate().rotate(Math::Vector_F({1, 0, 0}));
            north = atan2(northRot(1), northRot(0));

            if (angle > 80*3.14/180) {
                //gnssRef = gnssSub.getItem().data.position;
                auto newRef = gnssSub.getItem().data.position;
                //newRef(2) = accSub.getItem().data.val(5);
                posEstTask.setPositionReference(newRef);
                //posEstTask.setState(DSP::ValueCov<float, 6>(0, 1));
                //calulate the the new attitude with the same tilt but the new heading as zero
                //auto headingQuat = Math::Quat_F(Math::Vector_F({0, 0, 1}), -north);
                //auto attitude = headingQuat * attQuat;
                //auto attState = Math::Matrix<float, 7, 1>({
                //    0, 0, 0, attitude(0), attitude(1), attitude(2), attitude(3)
                //});
                //imuTask.setState(DSP::ValueCov<float, 7>(attState, 1));
            }

        }

        /*if (accSub.isDataNew() && attSub.isDataNew()) {

            auto accData = accSub.getItem();
            auto attData = attSub.getItem();
            
            Core::printM("New Data: t: %.2f\n", double(accData.timestamp)/Core::SECONDS);
            attData.data.val.printTo(Core::printM);
            accData.data.val.printTo(Core::printM);

        }*/


        if (accSub.isDataNew() || true) {

            bool connected = networkNode.isNodeReachable(0);
            if (!lastConnectionState && connected) {
                connectionCounter++;
            }
            lastConnectionState = connected;

            connections.publish(connectionCounter);


            auto accData = accSub.getItem();
            auto attData = attSub.getItem();
            auto biasData = biasSub.getItem();
            
            display_.clearDisplay();

            display_.setTextSize(1);
            display_.setTextColor(SSD1306_WHITE);
            display_.setTextWrap(false);
            
            display_.setCursor(0, 0);
            //display_.printf("Vel:%.2f,%.2f,%.2f\n", accData.data.val(0), accData.data.val(1), accData.data.val(2));
            //display_.setCursor(0, 9);
            //display_.printf("Pos:%.2f,%.2f,%.2f\n", accData.data.val(3), accData.data.val(4), accData.data.val(5));
            //display_.setCursor(0, 18);
            display_.printf("X:%.2f\t%.2f\n", accData.data(0), accData.data(3));
            display_.printf("Y:%.2f\t%.2f | %d\n", min(accData.data(1),100), min(accData.data(4),100), connectionCounter);
            display_.printf("Z:%.2f\t%.2f | %s\n", min(accData.data(2),100), min(accData.data(5),100), connected ? "OK" : "");
            //display_.printf("w:%.2f,x:%.2f,y:%.2f,z:%.2f\n", attData.data(3), attData.data(4), attData.data(5), attData.data(6));
            display_.printf("S:%d,P:%.1f,A:%.0f,N:%.0f\n", gnssSub.getItem().data.numSats, min(gnssSub.getItem().data.positionCov(0), 99), min(angle*180/3.14,99), north*180/3.14);
            //display_.printf("S:%d,HA:%.2f,VA:%.2f\n", gnssData.data.numSats, gnssData.data.positionCov(0, 0), gnssData.data.positionCov(2, 2));
            //display_.setCursor(0, 9);
            //display_.printf("Lat: %.10f, \nLon: %.10f\nAlt:%.2f, T:%.2f", gnssData.data.latitude*180/3.14, gnssData.data.longitude*180/3.14, gnssData.data.altitude, Core::NOWSeconds()/60);


            display_.display();

            //posEstTask.getCovariance().printTo(Core::printM);

            //LOG_MSG("Bias Est: \tX:%.2f\tY:%.2f\tZ:%.2f\t GyroX:%.2f\tY:%.2f\tZ:%.2f\n", biasData.data(0) * RAD_TO_DEG, biasData.data(1) * RAD_TO_DEG, biasData.data(2) * RAD_TO_DEG, gyroBias(0) * RAD_TO_DEG, gyroBias(1) * RAD_TO_DEG, gyroBias(2) * RAD_TO_DEG);
            //LOG_MSG("Bias:     \tX:%.2f\tY:%.2f\tZ:%.2f\n", gyroBias(0) * RAD_TO_DEG, gyroBias(1) * RAD_TO_DEG, gyroBias(2) * RAD_TO_DEG);
            //LOG_MSG("Attitude: W:%.2f, X:%.2f, Y:%.2f, Z:%.2f\n", attData.data(3), attData.data(4), attData.data(5), attData.data(6));

        }

        if (Core::NOW() > 5*Core::SECONDS) {

        }

        //(magTransform * (magSub.getItem().data.val - magBias)).printTo(Core::printM);
        //magSub.getItem().data.val.printTo(Core::printM);

    }

};
DisplayTask displayTask;



class CalibrationTesting : public Core::Task_Periodic
{
public:



    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> accelSub;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> gyroSub;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> magSub;

    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 1>>> imuTempSub;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 1>>> baroSub;

    Math::Vector<float, 3> magMax, magMin;

    Math::Vector<float, 3> gyroBias;
    Math::Vector<float, 3> accelVal;

    Core::ListBuffer<float, 500> gyroBiasBuffer;

    bool firstAcc = true;


    CalibrationTesting() : Task_Periodic("Calibration task", 0.01*Core::SECONDS)
    {
        Core::getSystemScheduler().addTask(*this);
        //setPriority(500);
    }

    void taskInit() override
    {

        accelSub.subscribe(accTransformTopic.getOutputTopic());
        gyroSub.subscribe(gyroTransformTopic.getOutputTopic());
        magSub.subscribe(magTransformTopic.getOutputTopic());
        imuTempSub.subscribe(mpuDriver.getThermoTopic());

        baroSub.subscribe(bme.getBaroTopic());

    }

    void updateGyroBias() {

        auto gyroData = gyroSub.getItem().data.val;
        auto imuTemp = imuTempSub.getItem().data.val(0);
        
        gyroBias = gyroBias * 0.999 + gyroData * 0.001;

        LOG_MSG("Gyro Bias:  X:%.3f Y:%.3f Z:%.3f, Temp: %.2f\n", gyroBias(0) * RAD_TO_DEG, gyroBias(1) * RAD_TO_DEG, gyroBias(2) * RAD_TO_DEG, imuTemp);

    }

    void updateMagBias() {

        auto magData = magSub.getItem().data.val;

        for (size_t i = 0; i < 3; i++)
        {
            if (magData[i][0] < magMin[i][0]) magMin[i][0] = magData[i][0];
            if (magData[i][0] > magMax[i][0]) magMax[i][0] = magData[i][0];
        }

        LOG_MSG("Mag val: %.2f, %.2f, %.2f, M:%.2f\n", magData(0), magData(1), magData(2), magData.magnitude());

        LOG_MSG("Mag min: ");
        magMin.printTo(Core::printM);
        LOG_MSG("\n");

        LOG_MSG("Mag max: ");
        magMax.printTo(Core::printM);
        LOG_MSG("\n");

    }

    void updateAccBias() {

        if (!accelSub.isDataNew()) return;

        auto accData = accelSub.getItem().data.val;

        accelVal = accelVal * 0.999 + accData * 0.001;

        if (firstAcc) {
            accelVal = accData;
            firstAcc = false;
        }

        LOG_MSG("Acc Bias: \tX:%.3f\tY:%.3f\tZ:%.3f\n", accelVal(0), accelVal(1), accelVal(2));

    }

    void updateBaro() {

        auto baroData = baroSub.getItem().data.val;

        auto altitude = posEstTask.calcAltitudeFromPressure(baroData(0));

        gyroBiasBuffer.placeBack(altitude, true);

        auto average = gyroBiasBuffer.getAverage();
        auto variance = gyroBiasBuffer.getStandardDeviation();
        variance = variance * variance;

        LOG_MSG("Baro: %.2f, Altitude: %.2f, Var: %.2f\n", baroData(0), average, variance);

    }

    void taskThread() override
    {   

        //updateBaro();
        //updateGyroBias();
        //updateMagBias();

        //tvcTopic.publish(tvcSetting);

    }

};
//CalibrationTesting calibrationTask;


void initialiseHardware() {

    SPI.begin();
    Wire.begin();
    Wire.setClock(100000);

    mpuPin.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
    //bmePin.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
    pmwPin.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);

    mpuPin.setPinValue(1);
    //bmePin.setPinValue(1);
    pmwPin.setPinValue(1);

    mpuIntPin.init(HAL::GPIO_IOMODE_t::IOMODE_INPUT);

    pinMode(SX1280_RXEN_PIN, OUTPUT);
    digitalWrite(SX1280_RXEN_PIN, HIGH);

    pinMode(SX1280_TXEN_PIN, OUTPUT);
    digitalWrite(SX1280_TXEN_PIN, LOW);

    //mpuDriver.enablePinInterrupt(mpuIntPin);
    //mpuDriver.ConfigDlpf(mpuDriver.DLPF_BANDWIDTH_250HZ_4kHz);
    mpuDriver.taskInit();
    mpuDriver.ConfigDlpf(SNSR::MPU9250::DlpfBandwidth::DLPF_BANDWIDTH_184HZ);
    mpuDriver.setInterval(250 * Core::MICROSECONDS); // 32kHz rate
    mpuDriver.setInitialised(true);
    mpuIntHandler.setCallback(&SNSR::MPU9250Driver::interruptHandler, mpuDriver);
    mpuDriver.setPriority(110);
    //mpuDriver.DisableDrdyInt();

    imuTask.setInterval(250*Core::MICROSECONDS);
    imuTask.setPriority(105);

    posEstTask.setPriority(105);
    //mpuDriver.setPriority(1000);
    //bme.setPriority(1000);

    //Core::getSystemScheduler().removeTask(gnss);

    //servoTest.setPosition(0);

    starshipTVC.setTVCFinsOffset(0, 0.5*DEG_TO_RAD, -2*DEG_TO_RAD, -1*DEG_TO_RAD);

}

void initialiseMemory() {

    //Initialise the memory and read the EEPROM into the internal memory
    EEPROM.begin();
    internalMemory.transferFrom(eeprom);

    if (memoryManager.getManagerVersion() != memoryManager.getMemoryVersion()) {
        LOG_MSG("Memory manager version mismatch, clearing memory\n");
        memoryManager.clearMemory();
    }
    //memoryManager.clearMemory();

    //Read the calibration and transformation data from the internal memory
    DSP::ValueCov<float, 3> magCalibData;
    DSP::ValueCov<float, 3> gyroCalibData;
    DSP::ValueCov<float, 3> accCalibData;

    if (!memoryManager.readItem(magCalibData, MEMORY_KEY_MAGCALIB)) {

        LOG_MSG("No mag calibration data found, creating new one\n");
        magCalibData.val = Math::Vector<float, 3>({0, 0, 0});
        magCalibData.cov = Math::Matrix<float, 3, 3>({1, 0, 0,
                                                      0, 1, 0,
                                                      0, 0, 1});

        memoryManager.allocateItem(magCalibData, MEMORY_KEY_MAGCALIB);
        memoryManager.writeItem(magCalibData, MEMORY_KEY_MAGCALIB);

    }

    if (!memoryManager.readItem(gyroCalibData, MEMORY_KEY_GYROCALIB)) {

        LOG_MSG("No gyro calibration data found, creating new one\n");
        gyroCalibData.val = Math::Vector<float, 3>({0, 0, 0});
        gyroCalibData.cov = Math::Matrix<float, 3, 3>({1, 0, 0,
                                                      0, 1, 0,
                                                      0, 0, 1});

        memoryManager.allocateItem(gyroCalibData, MEMORY_KEY_GYROCALIB);
        memoryManager.writeItem(gyroCalibData, MEMORY_KEY_GYROCALIB);

    }

    if (!memoryManager.readItem(accCalibData, MEMORY_KEY_ACCCALIB)) {

        LOG_MSG("No acc calibration data found, creating new one\n");
        accCalibData.val = Math::Vector<float, 3>({0, 0, 0});
        accCalibData.cov = Math::Matrix<float, 3, 3>({1, 0, 0,
                                                      0, 1, 0,
                                                      0, 0, 1});

        memoryManager.allocateItem(accCalibData, MEMORY_KEY_ACCCALIB);
        memoryManager.writeItem(accCalibData, MEMORY_KEY_ACCCALIB);

    }

    gyroTransformTopic.setTransform(gyroCalibData.cov, gyroCalibData.val);
    accTransformTopic.setTransform(accCalibData.cov, accCalibData.val);
    magTransformTopic.setTransform(magCalibData.cov, magCalibData.val);

    //Sync the internal memory with the EEPROM. This wont write anything if we ended up only reading.
    eeprom.transferFrom(internalMemory);

    //Lets update the memory with known values for the calibration data. This is only for testing purposes.
    if (false) {

        accCalibData.val = Math::Vector<float, 3>({0.161, -0.045, 0.8835});
        accCalibData.cov = Math::Matrix<float, 3, 3>({-1, 0, 0,
                                                        0, 1, 0,
                                                        0, 0, -0.989});
        //memoryManager.writeItem(accCalibData, MEMORY_KEY_ACCCALIB);

        gyroCalibData.val = Math::Vector<float, 3>({0.185 * DEG_TO_RAD, 1.152 * DEG_TO_RAD, -0.426 * DEG_TO_RAD});
        gyroCalibData.cov = Math::Matrix<float, 3, 3>({-1, 0, 0,
                                                        0, 1, 0,
                                                        0, 0, -1});
        //memoryManager.writeItem(gyroCalibData, MEMORY_KEY_GYROCALIB);
        
        Math::Vector<float, 3> magMin = {-0.3972, -0.3577, -0.2332};
        Math::Vector<float, 3> magMax = {0.3662, 0.4033, 0.4663};
        // Mag bias
        //auto magBias = 
        magCalibData.val = (magMax + magMin) / 2;
        magCalibData.cov = Math::Matrix<float, 3, 3>::eye();
        // Calculate the scale factor
        auto magScale = (magMax - magMin) / 2;
        magCalibData.cov(0, 0) = 1/magScale(0);
        magCalibData.cov(1, 1) = 1/magScale(1);
        magCalibData.cov(2, 2) = 1/magScale(2);
        magCalibData.cov = magCalibData.cov * 0.05;
        magCalibData.cov = magCalibData.cov * Math::Matrix<float, 3, 3>({
            0, 0, -1,
            1, 0, 0,
            0, -1, 0
        });
        //memoryManager.writeItem(magCalibData, MEMORY_KEY_MAGCALIB);

        //gyroTransformTopic.setTransform(gyroCalibData.cov, gyroCalibData.val);
        //accTransformTopic.setTransform(accCalibData.cov, accCalibData.val);
        //magTransformTopic.setTransform(magCalibData.cov, magCalibData.val);

        eeprom.transferFrom(internalMemory);

    }

}

void initialiseTopicConnections() {

    gyroTransformTopic.setInputTopic(mpuDriver.getGyroTopic());
    accTransformTopic.setInputTopic(mpuDriver.getAccelTopic());
    magTransformTopic.setInputTopic(qmc.getMagTopic());

    imuTask.setGyroInput(gyroTransformTopic.getOutputTopic());
    imuTask.setAccInput(accTransformTopic.getOutputTopic());
    imuTask.setMagInput(magTransformTopic.getOutputTopic());

    posEstTask.setAccInput(accTransformTopic.getOutputTopic());
    posEstTask.setAttitudeInput(imuTask.getAttitudeEstTopic());
    posEstTask.setBaroInput(bme.getBaroTopic());
    posEstTask.setGNSSInput(gnss.getGNSSTopic());

    controlRocket.subscribeAttitudeMeasurement(imuTask.getAttitudeEstTopic());
    controlRocket.subscribePositionMeasurement(posEstTask.getStateEstTopic());
    //controlRocket.subscribeSetpoint(positionSetpointTopic);

    starshipTVC.setTVCInputTopic(controlRocket.getTvcTopic());


}


void setup()
{

    //delay(1000);

    VCTR::Core::initialise();
    
    initialiseHardware();

    initialiseMemory();

    initialiseTopicConnections();

}

void loop()
{

    VCTR::Core::getSystemScheduler().tick();

}