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
#include "ExVectrDSP/body_simulator.hpp"

#include "ExVectrControl/control_rocket.hpp"
#include "ExVectrControl/control_position_standard.hpp"
#include "ExVectrControl/control_mapping_acctoatt.hpp"
#include "ExVectrControl/control_attitude_tvc.hpp"

#include "ExVectrArduinoPlatform.hpp"

#include "ExVectrArduinoPlatform/memory_eeprom.hpp"

#include "ExVectrData/memory_internal.hpp"
#include "ExVectrData/memory_manager.hpp"

#include "ExVectrPackets/packet_vehicle.hpp"

#include "subsystems.hpp"
#include "telecommand.hpp"
#include "telemetry.hpp"

#include "starship_tvc.hpp"
#include "starship_flaps.hpp"

#include "board_v_1_0.h"
#include "starship_hardware.h"
#include "memory_keys.hpp"

//#include "SX128XLT.h"
#include "Adafruit_SSD1306.h"

#include "datalink_sx1280.hpp"

#include "mission/mission_abstract.hpp"
#include "mission/mission_rth.hpp"
#include "mission/mission_waypoint.hpp"
#include "mission/mission_freefall.hpp"
#include "mission/mission_bellyflop.hpp"

//#include "sx1280_driver/datalink_sx1280.hpp"

//#define DO_FLAP_TEST_STARTUP

#define EXTEND_FLAPS_ASCENT false

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
//Platform::BusSPIDevice bmeBus(SPI, bmePin, true);
Platform::BusI2CDevice bmeBus(Wire, 0x76);
SNSR::BME280Driver bme(bmeBus);

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

Platform::PinPWM flapServoULPin(FLAP_SERVO_PIN_UL);
Platform::PinPWM flapServoURPin(FLAP_SERVO_PIN_UR);
Platform::PinPWM flapServoDLPin(FLAP_SERVO_PIN_DL);
Platform::PinPWM flapServoDRPin(FLAP_SERVO_PIN_DR);



//Memory managment
Data::Memory_ArduinoEEPROM eeprom(EEPROM, EEPROM.length());
Data::Memory_Internal<512> internalMemory;
Data::Memory_Manager memoryManager(internalMemory);


//Sensor Processing
DSP::TopicCoordTransform<float> gyroTransformTopic;
DSP::TopicCoordTransform<float> accTransformTopic;
DSP::TopicCoordTransform<float> magTransformTopic;

DSP::IMUAttitudeEKFTask imuTask(1*Core::MILLISECONDS);
DSP::IMUGPSPositionKalmanTask posEstTask(10*Core::MILLISECONDS);

DSP::BodySimulator bodySimulator(VEHICLE_MASS_KG, {0.044, 0.044, 0.004}, TVC_THRUST_LIMIT_N, TVC_ANGLE_LIMIT_RAD);

//DSP::Calibrator_Magnetometer magCalibrator;
//DSP::Calibrator_Gyroscope gyroCalibrator;
//DSP::Calibrator_Manager<float> gyroCalibratorManager(gyroCalibrator, gyroTransformTopic, internalMemory, 100);
//DSP::Calibrator_Manager<float> magCalibratorManager(magCalibrator, magTransformTopic, internalMemory, 102);


//Networking

Net::Datalink_SX1280 datalinkSX1280(SX1280_NSS_PIN, SX1280_NRESET_PIN, SX1280_RFBUSY_PIN, SX1280_DIO1_PIN, SX1280_TXEN_PIN, SX1280_RXEN_PIN);
Net::NetworkNode networkNode(1, datalinkSX1280, 2*Core::SECONDS);
//Net::NetworkMonitor networkMonitor(networkNode);

Core::Topic<Net::PacketAttitude> attitudeDataTopic;
Core::Topic<Net::PacketPosition> positionDataTopic;
Core::Topic<uint8_t> connections;

/*Core::Topic<VehicleMode> vehicleModeTopic;
Core::Topic<MissionMode> MissionModeTopic;
Core::Topic<SensoryState> sensoryStateTopic;
Core::Topic<FailureState> failureStateTopic;
Core::Topic<int64_t> missionTimeTopic;*/
Core::Topic<VehicleState> vehicleStateTopic;
Core::Topic<MissionState> missionStateTopic;

Core::Topic<Telecommand> telecommandTopic;

Core::Topic<Net::PacketGPS> gpsDataTopic;

Net::TransportTopic<Net::PacketAttitude> attitudeDataTransport(10, UINT16_MAX, networkNode, attitudeDataTopic);
Net::TransportTopic<Net::PacketPosition> positionDataTransport(11, UINT16_MAX, networkNode, positionDataTopic);
Net::TransportTopic<Net::PacketGPS> gpsDataTransport(12, UINT16_MAX, networkNode, gpsDataTopic);

/*Net::TransportTopic<VehicleMode> vehicleModeTransport(50, UINT16_MAX, networkNode, vehicleModeTopic);
Net::TransportTopic<MissionMode> MissionModeTransport(51, UINT16_MAX, networkNode, MissionModeTopic);
Net::TransportTopic<SensoryState> sensoryStateTransport(52, UINT16_MAX, networkNode, sensoryStateTopic);
Net::TransportTopic<FailureState> failureStateTransport(53, UINT16_MAX, networkNode, failureStateTopic);
Net::TransportTopic<int64_t> missionTimeTransport(54, UINT16_MAX, networkNode, missionTimeTopic);*/
Net::TransportTopic<VehicleState> vehicleStateTransport(50, UINT16_MAX, networkNode, vehicleStateTopic);
Net::TransportTopic<MissionState> missionStateTransport(51, UINT16_MAX, networkNode, missionStateTopic);

Net::TransportTopic<Telecommand> telecommandTransport(1000, UINT16_MAX, networkNode, telecommandTopic);

Net::TransportTopic<uint8_t> connectionsTransport(100, UINT16_MAX, networkNode, connections);


//Control
Core::Topic<CTRL::ControlAttitudeFlapSetting> flapSettingTopic;
Core::Topic<CTRL::ControlAttitudeBellyFlopSetting> bellyFlopControlTopic;

CTRL::ControlPositionStandard controlPositionStandard;
CTRL::ControlMappingAccToAtt controlMappingAccToAtt;
CTRL::ControlAttitudeTvc controlAttitudeTvc(VEHICLE_MASS_KG, TVC_THRUST_LIMIT_N, TVC_ANGLE_LIMIT_RAD);
CTRL::ControlAttitudeFlaps controlAttitudeFlaps;

CTRL::StarshipTVC starshipTVC(servoTVCXPPin, servoTVCXNPin, servoTVCYPPin, servoTVCYNPin, motorCWPIN, motorCCWPIN, TVC_ANGLE_LIMIT_RAD, TVC_SERVO_LIMIT, TVC_THRUST_LIMIT_N);
CTRL::StarshipFlaps starshipFlaps(flapServoULPin, flapServoURPin, flapServoDLPin, flapServoDRPin);


//Topics and data routing
Core::Topic_Switch<Core::Timestamped<Math::Vector<float, 7>>> attitudeTopicSwitch;
Core::Topic_Switch<Core::Timestamped<Math::Vector<float, 6>>> positionTopicSwitch;

//Functions
void attitudeTelemetryCallback(const Core::Timestamped<Math::Vector<float, 7>>& data) {

    static int64_t lastSend = 0;
    if (Core::NOW() - lastSend < 0.2*Core::SECONDS) return;
    lastSend = Core::NOW();

    Net::PacketAttitude packet({
        data.data(0), data.data(1), data.data(2)
    }, {
        data.data(3), data.data(4), data.data(5), data.data(6)
    });
    attitudeDataTopic.publish(packet);

    //LOG_MSG("Attitude: %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", data.data(0), data.data(1), data.data(2), data.data(3), data.data(4), data.data(5), data.data(6));
    
}
Core::StaticCallback_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attitudeTeleSubr(attitudeTopicSwitch.getTopic(), attitudeTelemetryCallback);

void positionTelemetryCallback(const Core::Timestamped<Math::Vector<float, 6>>& data) {

    static int64_t lastSend = 0;
    //LOG_MSG("Time: %f\n", double(data.timestamp)/Core::SECONDS);
    if (Core::NOW() - lastSend < 0.1*Core::SECONDS) return;
    lastSend = Core::NOW();

    Net::PacketPosition packet({
        data.data(3), data.data(4), data.data(5)
    }, {
        data.data(0), data.data(1), data.data(2)
    });
    packet.hAccuracy = sqrt(posEstTask.getCovariance()(3, 3) * posEstTask.getCovariance()(3, 3) + posEstTask.getCovariance()(4, 4) * posEstTask.getCovariance()(4, 4))/1000;
    packet.vAccuracy = posEstTask.getCovariance()(5, 5)/1000;
    positionDataTopic.publish(packet);

}
Core::StaticCallback_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> positionTeleSubr(positionTopicSwitch.getTopic(), positionTelemetryCallback);

void gnssTelemetryCallback(const Core::Timestamped<SNSR::GNSSData>& data) {

    static int64_t lastSend = 0;
    if (Core::NOW() - lastSend < 1*Core::SECONDS) return;
    lastSend = Core::NOW();

    Net::PacketGPS packet;
    packet.latitude = data.data.position(0) * 1e7;
    packet.longitude = data.data.position(1) * 1e7;
    packet.altitude = data.data.position(2);
    packet.velocity[0] = data.data.velocity(0);
    packet.velocity[1] = data.data.velocity(1);
    packet.velocity[2] = data.data.velocity(2);
    packet.numSats = data.data.numSats;
    packet.positionAccuracy = data.data.positionCov(0, 0);
    packet.altitudeAccuracy = data.data.positionCov(2, 2);
    packet.velocityAccuracy = data.data.velocityCov(0, 0);

    gpsDataTopic.publish(packet);

}
Core::StaticCallback_Subscriber<Core::Timestamped<SNSR::GNSSData>> gnssTeleSubr(gnss.getGNSSTopic(), gnssTelemetryCallback);


class MagnetometerCalibrationTask : public Core::Task_Periodic
{
private:

    Core::Buffer_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>, 10> magSubr;

    bool magCalibrationEnabled_ = false; // If the magnetometer calibration has started or not. This is set to true when the magnetometer calibration has started.

    Math::Matrix<float, 4, 4> xTx_;
    Math::Matrix<float, 4, 1> xTy_;

    Math::Vector<float, 3> magBias_ = {0, 0, 0}; // The bias of the magnetometer in sensor frame.
    float magScale_ = 1; // The scale of the magnetometer in all axis.

    float magneticFieldStrength_ = 50e-6; // The strength of the magnetic field in Tesla. This is used to scale the magnetometer data to the correct values.


public:

    MagnetometerCalibrationTask() : Task_Periodic("Magnetometer Calibration", 0.1*Core::SECONDS)
    {
        Core::getSystemScheduler().addTask(*this);
        //setPriority(500);
        setPaused(true);
    }

    void beginCalibration() {
        magCalibrationEnabled_ = true; // Set the magnetometer calibration to true
        resetCalibration();
        setPaused(false); // Start the task
    }

    void endCalibration() {
        magCalibrationEnabled_ = false; // Set the magnetometer calibration to false
        setPaused(true); // Stop the task
    }

    Math::Vector<float, 3> getMagBias() const {
        return magBias_; // Get the bias of the magnetometer in sensor frame.
    }

    const float getMagScale() const {
        return magScale_; // Get the scale of the magnetometer in all axis.
    }

    void taskInit() override
    {
        magSubr.subscribe(qmc.getMagTopic()); // Subscribe to the magnetometer topic
        resetCalibration(); // Set the matricies to the starting values

    }

    void taskThread() override {

        for (int i = 0; i < magSubr.size(); i++) {
            updateCalib(magSubr[i].data.val); // Update the calibration with the magnetometer data
        }

        if (magSubr.size() > 0) {

            auto beta = xTx_.inverse() * xTy_; // Calculate the beta vector from the xTx and xTy matrices
            magBias_ = {beta(0), beta(1), beta(2)}; // Get the bias from the beta vector
            magBias_ = magBias_ * 0.5; // Scale the bias to the correct values

            magScale_ = sqrt(beta(3) + magBias_(0)*magBias_(0) + magBias_(1)*magBias_(1) + magBias_(2)*magBias_(2)); // Get the scale from the beta vector

            LOG_MSG("Magnetometer bias: %.3f %.3f %.3f\n", magBias_(0), magBias_(1), magBias_(2)); // Print the bias to the console
            LOG_MSG("Magnetometer scale: %.3f\n", magScale_); // Print the scale to the console

        }

        magSubr.clear();

        if (!magCalibrationEnabled_) {
            setPaused(true); // Pause the task if the calibration is not enabled
        }

    }

    void updateCalib(const Math::Vector<float, 3>& magData) {
        
        Math::Vector<float, 4> mag4 = {magData(0), magData(1), magData(2), 1}; // Convert the magnetometer data to a 4D vector
        xTx_ = xTx_ + mag4 * mag4.transpose(); // Update the xTx matrix with the magnetometer data

        auto magSq = magData(0)*magData(0) + magData(1)*magData(1) + magData(2)*magData(2); // Convert the magnetometer data to a 4D vector
        xTy_ = xTy_ + mag4 * magSq; // Update the xTy matrix with the magnetometer data

    }

    void resetCalibration() {
        xTx_ = 0;
        xTy_ = 0;
    }


};
MagnetometerCalibrationTask magCalibTask;


MissionRTH defaultMission_(attitudeTopicSwitch.getTopic(), positionTopicSwitch.getTopic(), {0, 0, 1.5}); // The default mission is the return to home mission.
MissionWaypoint missionWaypointTask(attitudeTopicSwitch.getTopic(), positionTopicSwitch.getTopic());
MissionFreefall missionFreefallTask(attitudeTopicSwitch.getTopic(), positionTopicSwitch.getTopic(), VEHICLE_MASS_KG, TVC_THRUST_LIMIT_N, 30);
MissionBellyflop missionBellyflop(attitudeTopicSwitch.getTopic(), controlAttitudeTvc, controlMappingAccToAtt, 1 * Core::SECONDS, 60*DEGREES, 90*DEGREES);

/**
 * This class takes care of enabling, disabling and setting the actuators for the rocket. It also prepares the system for mission start and signals when something is wrong.
 */
class VehicleSafetyAndControlTask : public Core::Task_Periodic
{
private:

    const int64_t DATA_TIMEOUT = 0.5*Core::SECONDS;

    const float POSITION_OUTOFBOUNDS_STATIONARY = 2.0f; // m
    const float ANGLE_OUTOFBOUNDS_STATIONARY = 20*DEG_TO_RAD; // rad

    const float POSITION_OUTOFBOUNDS_FLIGHT = 50.0f; // m
    const float ANGLE_OUTOFBOUNDS_FLIGHT = 120*DEG_TO_RAD; // rad

    Core::Simple_Subscriber<Core::Timestamped<SNSR::GNSSData>> gnssSubr;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> gyroSubr;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> magSubr;
    Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 1>>> baroSubr;

    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posEstSubr;
    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attEstSubr;

    Core::Simple_Subscriber<Telecommand> telecommandSubr;
    
    VehicleMode vehicleMode_ = VehicleMode::VehicleMode_Startup;
    SensoryState sensoryState_;
    FailureState failureState_;

    MissionMode lastMissionMode_ = MissionMode::MissionMode_Idle; // The last mission mode that was set. This is used to check if the mission mode has changed.

    bool vehicleArmed_ = false; // If the vehicle is armed or not. This is set to true when the vehicle is armed.

    bool allSystemsInitialised_ = false; // If all systems are initialised or not. This is set to true when all systems are initialised.

    bool simulationMode_ = false; // If the vehicle is in simulation mode or not. This is set to true when the vehicle is in simulation mode.

    size_t missionSelection_ = 0; // The index of the currently selected mission.
    Core::ListArray<MissionAbstract*> missionList_;

    MissionAbstract* mission_;


public:

    VehicleSafetyAndControlTask() : 
        Task_Periodic("Vehicle Safety and Control", 0.1*Core::SECONDS)
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
        failureState_.radioConnectionLoss = false;

        mission_ = &defaultMission_; // Set the mission to the default mission

        //mission_ = defaultMission_; // Set the mission to the default mission

    }

    const VehicleMode& getVehicleMode() {
        return vehicleMode_;
    }

    const SensoryState& getSensoryState() {
        return sensoryState_;
    }

    const FailureState& getFailureState() {
        return failureState_;
    }

    bool simulationModeEnabled() {
        return simulationMode_;
    }

    bool isVehicleArmed() {
        return vehicleArmed_;
    }

    MissionAbstract* getCurrentMission() {
        return mission_;
    }

    void switchMissionTo(MissionAbstract* mission, int64_t startTime = 0) {
        mission_->resetMission(); // Reset the mission
        mission_ = mission; // Set the mission to the default mission
        mission_->resetMission(); // Reset the mission
        mission_->beginMission(startTime); // Start the default mission to return to home
        //starshipFlaps.setFlapSettingTopic(mission_->getFlapSettingTopic()); // Set the flap setting topic to the default mission
        controlPositionStandard.subscribeSetpoint(mission_->getSetpointTopic()); // Subscribe to the setpoint topic of the mission
    }

    void taskInit() override
    {
        
        gnssSubr.subscribe(gnss.getGNSSTopic());
        gyroSubr.subscribe(gyroTransformTopic.getOutputTopic());
        magSubr.subscribe(magTransformTopic.getOutputTopic());
        baroSubr.subscribe(bme.getBaroTopic());
        posEstSubr.subscribe(positionTopicSwitch.getTopic());
        attEstSubr.subscribe(attitudeTopicSwitch.getTopic());

        telecommandSubr.subscribe(telecommandTopic);

        missionList_.append(&defaultMission_); // Add the mission guidance task to the list of missions
        missionList_.append(&missionWaypointTask); // Add the mission waypoint task to the list of missions

        missionWaypointTask.addWaypoint({0, 0, 0.3}, 100, 3, 1, 5, 0 * Core::SECONDS); // Add a waypoint to the mission waypoint task
        missionWaypointTask.addWaypoint({0, 0, 1}, 0.5, 3, 1, 0.5, 15 * Core::SECONDS, 10*Core::SECONDS, true); // Add a waypoint to the mission waypoint task

    }

    void taskThread() override {

        handleTelecommands();

        if (!systemsInitialised()) {
            starshipTVC.enableActuators(false); // Disable actuators
            vehicleShutdownControl(true); // Disable everything
            return; // Wait until all systems are initialised
        }

        //if (mission_ == &defaultMission_) {
        //    LOG_MSG("Default mission selected.\n"); // Log the default mission selection
        //}

        checkForFailures();

        handleVehicleMissionControl(); // Handle the vehicle mission control
        
        auto vehicleReady = vehicleIsReady();
        vehicleShutdownControl(!vehicleReady);

        vehicleArmingControl();

    }

    void handleVehicleMissionControl() {

        MissionState missionState = mission_->getMissionState();
        missionState.missionIndex = missionSelection_; // Set the mission index to the current mission index

        if (missionState.missionMode != lastMissionMode_) {
            
            if (missionState.missionMode == MissionMode::MissionMode_Idle) {
                posEstTask.enableZeroingMode(false);
                imuTask.enableZeroingMode(false); // Enable zeroing mode for the attitude estimator
                bodySimulator.enableZeroingMode(false); // Enable zeroing mode for the body simulator
                //controlRocket.enableControl(false); // Disable control for the rocket
                controlAttitudeTvc.enableControl(false); // Disable control for the attitude
                controlPositionStandard.enableControl(false); // Disable control for the position
                //starshipTVC.enableMotors(false); // Enable motors
                starshipFlaps.enableActuators(false); // Disable actuators
            } else if (missionState.missionMode == MissionMode::MissionMode_Initialisation) {
                posEstTask.enableZeroingMode(true); // Enable zeroing mode for the position estimator
                imuTask.enableZeroingMode(true); // Enable zeroing mode for the attitude estimator
                bodySimulator.enableZeroingMode(true); // Enable zeroing mode for the body simulator
                //controlRocket.enableControl(false); // Disable control for the rocket
                controlAttitudeTvc.enableControl(false); // Disable control for the attitude
                controlPositionStandard.enableControl(false); // Disable control for the position
                //starshipTVC.enableMotors(false); // Enable motors
                starshipFlaps.enableActuators(true); // Disable actuators
            } else if (missionState.missionMode == MissionMode::MissionMode_Startup) {
                posEstTask.enableZeroingMode(false); // Disable zeroing mode for the position estimator
                imuTask.enableZeroingMode(false); // Enable zeroing mode for the attitude estimator
                bodySimulator.enableZeroingMode(false); // Enable zeroing mode for the body simulator
                //controlRocket.enableControl(false); // Disable control for the rocket
                controlAttitudeTvc.enableControl(false); // Disable control for the attitude
                controlPositionStandard.enableControl(false); // Disable control for the position
                starshipTVC.beginActuatorTest(0); // Start the actuator test
                #ifdef DO_FLAP_TEST_STARTUP
                starshipFlaps.beginActuatorTest(4*Core::SECONDS); // Start the actuator test
                #endif
                //starshipTVC.enableMotors(true); // Enable motors
                starshipFlaps.enableActuators(false); // Disable actuators
            } else if (missionState.missionMode == MissionMode::MissionMode_Running) {
                posEstTask.enableZeroingMode(false); // Disable zeroing mode for the position estimator
                imuTask.enableZeroingMode(false); // Enable zeroing mode for the attitude estimator
                bodySimulator.enableZeroingMode(false); // Enable zeroing mode for the body simulator
                //controlRocket.enableControl(true); // Disable control for the rocket
                controlAttitudeTvc.enableControl(mission_->getActuatorsEnabled());
                controlPositionStandard.enableControl(true); // Disable control for the position
                //starshipTVC.enableMotors(true); // Enable motors
                starshipFlaps.enableActuators(true); // Disable actuators
            } else if (missionState.missionMode == MissionMode::MissionMode_Finished) {
                posEstTask.enableZeroingMode(false); // Disable zeroing mode for the position estimator
                imuTask.enableZeroingMode(false); // Enable zeroing mode for the attitude estimator
                bodySimulator.enableZeroingMode(false); // Enable zeroing mode for the body simulator
                //controlRocket.enableControl(false); // Disable control for the rocket
                controlAttitudeTvc.enableControl(false); // Disable control for the attitude
                controlPositionStandard.enableControl(false); // Disable control for the position
                //starshipTVC.enableMotors(false); // Enable motors
                starshipFlaps.enableActuators(false); // Disable actuators
            }

            lastMissionMode_ = missionState.missionMode; // Set the last mission mode to the current mission mode\

        }

        if (mission_ == &defaultMission_ && mission_->missionEnd()) {
            LOG_MSG("RTH mission ended.\n"); // Log the mission end
            starshipTVC.enableMotors(false); // Disable motors
            starshipFlaps.enableActuators(false); // Disable actuators
            vehicleShutdownControl(true); // Disable everything
        } else if (mission_->missionEnd() && (mission_->nextMission() == &defaultMission_ || mission_->nextMission() == nullptr)) { // Mission has ended and we are not in the default mission (RTH), then we switch to RTH mission.
            LOG_MSG("Mission ended. Switching to default mission.\n"); // Log the mission end
            missionSelection_ = 0; // Set the mission selection to the default mission
            switchMissionTo(&defaultMission_); // Switch to the default mission
        } else if (mission_->missionEnd() && mission_->nextMission() != nullptr) { // If the mission has ended and there is a next mission, then we switch to the next mission.
            LOG_MSG("Mission ended. Switching to next mission.\n"); // Log the mission end
            switchMissionTo(mission_->nextMission()); // Switch to the next mission
        }

    }

    void handleTelecommands() {

        if (!telecommandSubr.isDataNew()) return; // No new telecommand data

        auto telecommand = telecommandSubr.getItem();

        switch (telecommand.type)
        {

        case TelecommandType::Telecommand_CalibStart:
            if (telecommand.subsystem == Subsystem::Subsystem_Magneto) {

                if (telecommand.paramInt > 0) {
                    magCalibTask.beginCalibration(); // Start the mag calibration
                    starshipTVC.enableActuators(false); // Disable actuators

                    vehicleMode_ = VehicleMode::VehicleMode_SensorCalib;
                    LOG_MSG("Mag calibration started\n"); // Log the mag calibration start
                } else {
                    magCalibTask.endCalibration(); // End the mag calibration
                    vehicleMode_ = VehicleMode::VehicleMode_Startup; // Reset the vehicle mode to startup
                    LOG_MSG("Mag calibration Ended\n"); // Log the mag calibration start
                }

            } 
            break;

        case TelecommandType::Telecommand_CalibApply:
            if (telecommand.subsystem == Subsystem::Subsystem_Magneto) {

                LOG_MSG("Applying magnetometer calibration\n"); // Log the mag calibration apply

                auto magScale = magCalibTask.getMagScale(); 
                DSP::ValueCov<float, 3> magTransform;
                magTransform.val = magCalibTask.getMagBias(); // Get the mag bias from the mag calibration task
                magTransform.cov = {
                    magScale, 0, 0,
                    0, magScale, 0,
                    0, 0, magScale
                };

                LOG_MSG("Saving this magnetometer bias: %.3f %.3f %.3f\n", magTransform.val(0), magTransform.val(1), magTransform.val(2)); // Print the bias to the console

                if (memoryManager.writeItem(magTransform, MEMORY_KEY_MAGCALIB)) { // Save the mag transform matrix to memory
                    //memoryManager.writeItem(magTransformTopic.getOutputTopic().getTransformMatrix(), MEMORY_KEY_MAGTRANSFORM); // Save the mag transform matrix to memory
                    LOG_MSG("Magnetometer calib saved to memory\n"); // Log the mag transform matrix save
                } else {
                    LOG_MSG("Magnetometer calib not saved to memory\n"); // Log the mag transform matrix not saved
                }

                Math::Matrix<float, 3, 3> sensorToBody_ = {
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, 1
                };
                if (!memoryManager.readItem(sensorToBody_, MEMORY_KEY_MAGTRANSFORM)) { // Read the mag transform matrix from memory
                    LOG_MSG("Magnetometer transform matrix not found in memory. Using identity matrix.\n"); // Log the mag transform matrix not found in memory
                } else {
                    LOG_MSG("Magnetometer transform matrix found in memory. Using it.\n"); // Log the mag transform matrix found in memory
                }

                magTransformTopic.setTransform(magTransform.cov * sensorToBody_, magTransform.val); // Set the transform matrix to the mag transform matrix

                LOG_MSG("Mag calibration applied\n"); // Log the mag calibration apply

            }
            break;

        case TelecommandType::Telecommand_Save:
            if (telecommand.subsystem == Subsystem::Subsystem_Magneto) {

                LOG_MSG("Saving EEPROM Memory!\n");
                eeprom.transferFrom(internalMemory); // Transfer the internal memory to the EEPROM

            } 
            break;
        
        case TelecommandType::Telecommand_SystemReset:
            if (telecommand.paramInt == 0xC5) { //Validate. 

                vehicleMode_ = VehicleMode::VehicleMode_Startup; // Reset the vehicle mode to startup
                allSystemsInitialised_ = false; // Reset the system initialisation flag
                starshipTVC.enableActuators(false); // Disable actuators
                vehicleShutdownControl(true); // Disable everything
                mission_->resetMission();
                clearFailures();
                posEstTask.setPositionReference();
                bodySimulator.setAttitudeState({0, 0, 0, 1, 0, 0, 0}); // Reset the attitude state to the origin
                bodySimulator.setPositionState({0, 0, 0, 0, 0, 0}); // Reset the position state to the origin
                //bodySimulator.enableZeroingMode(true); // Enable zeroing mode for the body simulator

                starshipTVC.enableMotors(false); // Disable motors
                starshipFlaps.enableActuators(false); // Disable actuators
                vehicleArmed_ = false; // Disarm the vehicle

                LOG_MSG("System reset telecommand\n"); // Log the system reset

            }
            break;

        case TelecommandType::Telecommand_Arm:
            if (telecommand.paramInt == 0xA5) { //Validate. 

                vehicleArmed_ = true;

                LOG_MSG("System arm telecommand\n"); // Log the system arm

            } else {

                vehicleArmed_ = false; // Disarm the vehicle if the telecommand is not valid
                LOG_MSG("System disarm telecommand\n"); // Log the system disarm

            }
            break;

        case TelecommandType::Telecommand_MissionSelect:

            if (vehicleMode_ == VehicleMode::VehicleMode_Ready) {

                //missionSelection_ = telecommand.paramInt; // Get the mission selection from the telecommand

                //mission_->resetMission(); // Reset the mission
                //mission_ = *missionList_[missionSelection_]; // Set the mission to the selected mission
                //mission_->resetMission(); // Reset the mission
                //controlRocket.subscribeSetpoint(mission_->set); // Subscribe to the control topic of the mission

                LOG_MSG("Mission select telecommand\n"); // Log the mission select

            }
            break;

        case TelecommandType::Telecommand_MissionBegin:

            if (vehicleMode_ == VehicleMode::VehicleMode_Ready) {

                int64_t startTime = int64_t(telecommand.paramInt) * Core::SECONDS; // Get the start time from the telecommand
                
                missionSelection_ = 1;
                switchMissionTo(&missionWaypointTask, startTime); // Switch to the selected mission

                vehicleMode_ = VehicleMode::VehicleMode_Running; // Vehicle is now running and will do the mission

                LOG_MSG("Mission begin telecommand\n"); // Log the mission begin

                if (!simulationMode_) {
                    starshipTVC.enableMotors(true); // Enable actuators
                } else {
                    starshipTVC.enableMotors(false); // Disable actuators in simulation mode
                }

            }
            break;

        case TelecommandType::Telecommand_MissionAbort:

            if (vehicleMode_ == VehicleMode::VehicleMode_Running) {

                if (mission_->getMissionState().missionMode == MissionMode::MissionMode_Running) {
                    missionSelection_ = 0;
                    switchMissionTo(&defaultMission_); // Switch to the default mission
                } else {
                    starshipTVC.enableActuators(false); // Disable actuators
                    vehicleShutdownControl(true); // Disable everything
                    mission_->resetMission();
                    clearFailures();
                }



                LOG_MSG("Mission abort telecommand\n"); // Log the mission abort

            }
            break;

        case TelecommandType::Telecommand_SimulationMode:

            if (telecommand.paramInt == 0xA9) { //Validate. 

                simulationMode_ = !simulationMode_; // Toggle simulation mode

                if (simulationMode_) {

                    starshipTVC.enableMotors(false); // Disable actuators in simulation mode

                    positionTopicSwitch.subscribe(bodySimulator.getStateEstTopic()); // Subscribe to the position topic of the body simulator
                    attitudeTopicSwitch.subscribe(bodySimulator.getAttitudeEstTopic()); // Subscribe to the attitude topic of the body simulator

                    //bodySimulator.setPaused(false); // Unpause the body simulator to start the simulation
                    bodySimulator.setAttitudeState({0, 0, 0, 1, 0, 0, 0}); // Reset the attitude state to the origin
                    bodySimulator.setPositionState({0, 0, 0, 0, 0, 0}); // Reset the position state to the origin

                    bodySimulator.setPaused(false); // Unpause the body simulator to start the simulation

                    
                } else {

                    positionTopicSwitch.subscribe(posEstTask.getStateEstTopic()); // Subscribe to the position topic of the position estimator
                    attitudeTopicSwitch.subscribe(imuTask.getAttitudeEstTopic()); // Subscribe to the attitude topic of the attitude estimator

                    bodySimulator.setPaused(true);

                    // Reset the system to normal mode
                    vehicleMode_ = VehicleMode::VehicleMode_Startup; // Reset the vehicle mode to startup
                    allSystemsInitialised_ = false; // Reset the system initialisation flag
                    starshipTVC.enableActuators(false); // Disable actuators
                    vehicleShutdownControl(true); // Disable everything
                }

            }
            break;
        
        default:
            break;
        }

    }

    bool systemsInitialised() {

        if (allSystemsInitialised_) {
            return true;
        } else {
            vehicleMode_ = VehicleMode::VehicleMode_Startup;
        }

        if (!gnssSubr.isDataNew()) {
            LOG_MSG("GNSS data not new\n");
            sensoryState_.gnss = TelemetrySensor::TelemetrySensor_Init;
            return false;
        }

        if (!gyroSubr.isDataNew()) {
            LOG_MSG("Gyro data not new\n");
            sensoryState_.imu = TelemetrySensor::TelemetrySensor_Init;
            return false;
        }

        if (!magSubr.isDataNew()) {
            LOG_MSG("Mag data not new\n");
            sensoryState_.mag = TelemetrySensor::TelemetrySensor_Init;
            return false;
        }

        if (!baroSubr.isDataNew()) {
            LOG_MSG("Baro data not new\n");
            sensoryState_.baro = TelemetrySensor::TelemetrySensor_Init;
            return false;
        }

        if (!posEstSubr.isDataNew()) {
            LOG_MSG("Position data not new\n");
            sensoryState_.positionKF = TelemetrySensor::TelemetrySensor_Init;
            return false;
        }

        if (!attEstSubr.isDataNew()) {
            LOG_MSG("Attitude data not new\n");
            sensoryState_.attitudeKF = TelemetrySensor::TelemetrySensor_Init;
            return false;
        }

        LOG_MSG("All systems initialised\n");

        allSystemsInitialised_ = true; // Set all systems initialised to true
        vehicleMode_ = VehicleMode::VehicleMode_Ready; // Set the vehicle mode to safe

        return true;

    }

    void checkForFailures() {   

        //Check radio for a connection loss with the ground station
        if (!networkNode.isNodeReachable(0)) {
            vehicleMode_ = VehicleMode::VehicleMode_Failure;
            failureState_.radioConnectionLoss = true;
        } 

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

        if (Core::NOW() - magSubr.getItem().timestamp > DATA_TIMEOUT*5 && sensoryState_.mag != TelemetrySensor::TelemetrySensor_Failure) {
            //vehicleMode_ = VehicleMode::VehicleMode_Failure;
            //failureState_.sensorFailure = true;
            if (vehicleMode_ == VehicleMode::VehicleMode_Running && mission_->getMissionState().missionMode == MissionMode::MissionMode_Running) {
                switchMissionTo(&defaultMission_); // Switch to the default mission
                LOG_MSG("Magnetometer data timeout. Switching mission to default!\n");
            }
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
        auto missionMode = mission_->getMissionState().missionMode;
        if (!mission_->getDisableKinematicSafety() && missionMode != MissionMode::MissionMode_Idle) {
            
            bool stationary = false;
            stationary |= (missionMode == MissionMode::MissionMode_Initialisation);
            stationary |= (missionMode == MissionMode::MissionMode_Startup);

            auto position = posEstSubr.getItem().data;
            Math::Quat<float> attitude = attEstSubr.getItem().data.block<4, 1>(3, 0);

            auto zAxisBody = attitude.rotate(Math::Vector<float, 3>({0, 0, 1}));

            auto startDistance = position.magnitude(3, 5);
            auto tilt = zAxisBody.getAngleTo(Math::Vector<float, 3>({0, 0, 1}));

            if (stationary) {

                if (startDistance > POSITION_OUTOFBOUNDS_STATIONARY) {
                    failureState_.positionOutOfBounds = true;
                    vehicleMode_ = VehicleMode::VehicleMode_Failure;
                } 

                if (tilt > ANGLE_OUTOFBOUNDS_STATIONARY && vehicleMode_ != VehicleMode::VehicleMode_SensorCalib) {
                    failureState_.attitudeOutOfBounds = true;
                    vehicleMode_ = VehicleMode::VehicleMode_Failure;
                }

            } else {

                if (startDistance > POSITION_OUTOFBOUNDS_FLIGHT) {
                    failureState_.positionOutOfBounds = true;
                    vehicleMode_ = VehicleMode::VehicleMode_Failure;
                } 

                if (tilt > ANGLE_OUTOFBOUNDS_FLIGHT && vehicleMode_ != VehicleMode::VehicleMode_SensorCalib) {
                    failureState_.attitudeOutOfBounds = true;
                    vehicleMode_ = VehicleMode::VehicleMode_Failure;
                }

            }

        }

        //Auto disarm vehicle once landed.
        if (vehicleArmed_ && missionMode == MissionMode::MissionMode_Finished && (mission_ == &defaultMission_ || mission_->nextMission() == nullptr)) {
            vehicleArmed_ = false; // Disarm the vehicle if the default mission is finished
            LOG_MSG("Vehicle auto disarmed after RTH mission finished.\n"); // Log the vehicle auto disarm
        } 

    }



    bool vehicleIsReady() {

        if (
            allSystemsInitialised_ 
            && sensoryState_.imu == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.baro == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.gnss == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.attitudeKF == TelemetrySensor::TelemetrySensor_Ready
            && sensoryState_.positionKF == TelemetrySensor::TelemetrySensor_Ready
            && !failureState_.sensorFailure
            && !failureState_.attitudeFailure
            && !failureState_.positionFailure
            && !failureState_.positionOutOfBounds
            && !failureState_.attitudeOutOfBounds
            && vehicleMode_ != VehicleMode::VehicleMode_Failure
            && vehicleMode_ != VehicleMode::VehicleMode_SensorCalib
        ) {

            if (
                vehicleMode_ == VehicleMode::VehicleMode_Startup
                || vehicleMode_ == VehicleMode::VehicleMode_Failure
            ) {
                vehicleMode_ = VehicleMode::VehicleMode_Ready;
            }

            //vehicleMode_ = VehicleMode::VehicleMode_Ready; // Set the vehicle mode to ready

            //vehicleMode_ = VehicleMode::VehicleMode_Ready; // Set the vehicle mode to ready
            return true;
        }

        return false;
    }

    void vehicleShutdownControl(bool shutdown) {
        
        if (shutdown) {
            starshipTVC.enableActuators(false); // Enable actuators
            starshipFlaps.enableActuators(false); // Disable actuators
            bodySimulator.enableTVC(false); // Disable the TVC for the body simulator
            //starshipTVC.enableMotors(false); // Disable motors
            //posEstTask.enableZeroingMode(true); // Enable zeroing mode for the position estimator
            return;
        }

        if (mission_ == &missionFreefallTask) {
            starshipFlaps.enableActuators(true); // Disable actuators
        } else
            starshipFlaps.enableActuators(mission_->getActuatorsEnabled()); // Disable actuators

        starshipTVC.enableActuators(mission_->getActuatorsEnabled()); //Give mission guidance control over the actuators
        bodySimulator.enableTVC(mission_->getActuatorsEnabled()); // Give mission guidance control over the actuators
        bodySimulator.enableFlaps(starshipFlaps.getFlapSettings().enableFlaps); // Give simulator info if flaps are enabled or not

    }

    void vehicleArmingControl() {

        if (vehicleMode_ == VehicleMode::VehicleMode_Failure) {
            vehicleArmed_ = false; // Disarm the vehicle if the vehicle is in failure mode
            LOG_MSG("Vehicle disarmed due to failure mode.\n"); // Log the vehicle disarm
        } else if (vehicleMode_ == VehicleMode::VehicleMode_SensorCalib) {
            vehicleArmed_ = false; // Disarm the vehicle if the vehicle is in sensor calibration mode
            LOG_MSG("Vehicle disarmed due to sensor calibration.\n"); // Log the vehicle disarm
        } else if (vehicleMode_ == VehicleMode::VehicleMode_Startup) {
            vehicleArmed_ = false; // Disarm the vehicle if the vehicle is in startup mode
            LOG_MSG("Vehicle disarmed due to startup mode.\n"); // Log the vehicle disarm
        } else if (vehicleMode_ == VehicleMode::VehicleMode_SensorInit) {
            vehicleArmed_ = false; // Disarm the vehicle if the vehicle is in startup mode
            LOG_MSG("Vehicle disarmed due to sensor initialisation.\n"); // Log the vehicle disarm
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         

        //LOG_MSG("Vehicle armed: %d\n", vehicleArmed_); // Log the vehicle armed state

        if (vehicleArmed_) {
            starshipTVC.motorPowerLimit(1.0);
        } else {
            starshipTVC.motorPowerLimit(0.1);
        }

    }

    void clearFailures() {
        failureState_.sensorFailure = false;
        failureState_.attitudeFailure = false;
        failureState_.positionFailure = false;
        failureState_.positionOutOfBounds = false;
        failureState_.attitudeOutOfBounds = false;
        failureState_.radioConnectionLoss = false;
        sensoryState_.baro = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.imu = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.mag = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.gnss = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.attitudeKF = TelemetrySensor::TelemetrySensor_Init;
        sensoryState_.positionKF = TelemetrySensor::TelemetrySensor_Init;
        allSystemsInitialised_ = false; // Reset the system initialisation flag
    }


};
VehicleSafetyAndControlTask vehicleSafetyAndControlTask;


class FlapModeObserverTask : public Core::Task_Periodic
{
private:

    CTRL::ControlAttitudeBellyFlopSetting starshipFlopSetting; // Flap controller setting.

    Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posEstSubr; // Subscriber for the position estimator

public:

    FlapModeObserverTask() : Task_Periodic("Flap Mode Observer", 0.1*Core::SECONDS)
    {
        Core::getSystemScheduler().addTask(*this);
        //setPriority(500);
    }

    void taskInit() override
    {
        
        posEstSubr.subscribe(positionTopicSwitch.getTopic()); // Subscribe to the position estimator topic

    }

    void taskThread() override 
    {

        auto& vehicleMode = vehicleSafetyAndControlTask.getVehicleMode();
        auto mission = vehicleSafetyAndControlTask.getCurrentMission();

        auto posEst = posEstSubr.getItem().data.block<3, 1>(0, 3); // Get the position estimate 
        auto velEst = posEstSubr.getItem().data.block<3, 1>(0, 0); // Get the velocity estimate

        if (vehicleMode == VehicleMode::VehicleMode_Running && mission->getMissionState().missionMode == MissionMode::MissionMode_Running) {

            if (mission == &missionFreefallTask) {
                starshipFlopSetting.bellyFlopMode = CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::BellyFlopMode_Stabilize;
                starshipFlopSetting.azimuthAngle_Rad = 0; // Set the azimuth angle to 0 degrees
                starshipFlopSetting.pitchAngle_Rad = 0.0f; // Set the flap angle to 0
                if (posEst(2) > 60) {
                    starshipFlopSetting.azimuthAngle_Rad = 180 * DEGREES;
                }
            } else if (mission == &defaultMission_ && defaultMission_.stabilising()) {
                starshipFlopSetting.bellyFlopMode = CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::BellyFlopMode_Upright; // Enable the belly flop mode
            } else if (mission == &missionWaypointTask && defaultMission_.stabilising()) {
                starshipFlopSetting.bellyFlopMode = CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::BellyFlopMode_Acsent; // Disable the belly flop mode
                starshipFlopSetting.extentFlapsOnAscent = EXTEND_FLAPS_ASCENT;
            } else {
                starshipFlopSetting.bellyFlopMode = CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::BellyFlopMode_Acsent; // Disable the belly flop mode
                starshipFlopSetting.extentFlapsOnAscent = false;
            }

        }

        bellyFlopControlTopic.publish(starshipFlopSetting); // Publish the flap controller setting to the control system

    }

};
FlapModeObserverTask flapModeObserverTask;


class CommsSystemStateTask : public Core::Task_Periodic
{
private:

    Core::Simple_Subscriber<Math::Vector<float, 4>> tvcActualThrustSubr; // Subscriber for the actual TVC thrust vector

public:

    CommsSystemStateTask() : Task_Periodic("Comms System State", 0.2*Core::SECONDS)
    {
        Core::getSystemScheduler().addTask(*this);
        //setPriority(500);
    }

    void taskInit() override
    {
        
        tvcActualThrustSubr.subscribe(starshipTVC.getTVCActualThrustTopic()); // Subscribe to the TVC thrust vector topic

    }

    void taskThread() override 
    {

        auto vehicleMode = vehicleSafetyAndControlTask.getVehicleMode();
        auto sensoryState = vehicleSafetyAndControlTask.getSensoryState();
        auto failureState = vehicleSafetyAndControlTask.getFailureState();

        auto simulationMode = vehicleSafetyAndControlTask.simulationModeEnabled(); // Get the simulation mode

        VehicleState vehicleState;
        vehicleState.mode = vehicleMode;
        vehicleState.sensoryState = sensoryState;
        vehicleState.failureState = failureState;
        vehicleState.simulationModeEnabled = simulationMode; // Get the simulation mode
        vehicleState.armed = vehicleSafetyAndControlTask.isVehicleArmed(); // Get the vehicle armed state
        vehicleState.tvcThrustX = tvcActualThrustSubr.getItem()(0)/50.0f * INT16_MAX;
        vehicleState.tvcThrustY = tvcActualThrustSubr.getItem()(1)/50.0f * INT16_MAX;
        vehicleState.tvcThrustZ = tvcActualThrustSubr.getItem()(2)/50.0f * INT16_MAX;
        vehicleState.tvcTorque = tvcActualThrustSubr.getItem()(3)/50.0f * INT16_MAX;

        vehicleStateTopic.publish(vehicleState); // Publish the vehicle state to the control system

        auto& missionState = vehicleSafetyAndControlTask.getCurrentMission()->getMissionState();
        missionStateTopic.publish(missionState); // Publish the mission state to the control system


    }


};
CommsSystemStateTask commsSystemStateTask;


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

        attSub.subscribe(attitudeTopicSwitch.getTopic());
        accSub.subscribe(positionTopicSwitch.getTopic());

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

            /*if (angle > 80*3.14/180) {
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
            }*/

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



        /*for (size_t i = 0; i < 3; i++)
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
        LOG_MSG("\n");*/

    }

    void updateAccBias() {

        if (!accelSub.isDataNew()) return;

        auto accData = accelSub.getItem().data.val;

        accelVal = accelVal * 0.99 + accData * 0.01;

        if (firstAcc || Serial.available()) {
            Serial.flush();
            Serial.clear();
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
        updateAccBias();

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
    mpuDriver.ConfigDlpf(SNSR::MPU9250::DlpfBandwidth::DLPF_BANDWIDTH_41HZ);
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

    starshipTVC.setTVCFinsOffset(1*DEG_TO_RAD, -5*DEG_TO_RAD, 5*DEG_TO_RAD, 2*DEG_TO_RAD);

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

    Math::Matrix<float, 3, 3> magTransform;
    Math::Matrix<float, 3, 3> gyroTransform;
    Math::Matrix<float, 3, 3> accTransform;

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

    } else {
        LOG_MSG("Gyro calibration data found: %.3f, %.3f, %.3f\n", gyroCalibData.val(0) * RAD_TO_DEG, gyroCalibData.val(1) * RAD_TO_DEG, gyroCalibData.val(2) * RAD_TO_DEG);
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

    if (!memoryManager.readItem(magTransform, MEMORY_KEY_MAGTRANSFORM)) {

        LOG_MSG("No mag transform data found, creating new one\n");
        magTransform = Math::Matrix<float, 3, 3>::eye();
        memoryManager.allocateItem(magTransform, MEMORY_KEY_MAGTRANSFORM);
        memoryManager.writeItem(magTransform, MEMORY_KEY_MAGTRANSFORM);

    }

    if (!memoryManager.readItem(gyroTransform, MEMORY_KEY_GYROTRANSFORM)) {

        LOG_MSG("No gyro transform data found, creating new one\n");
        gyroTransform = Math::Matrix<float, 3, 3>::eye();
        memoryManager.allocateItem(gyroTransform, MEMORY_KEY_GYROTRANSFORM);
        memoryManager.writeItem(gyroTransform, MEMORY_KEY_GYROTRANSFORM);

    }

    if (!memoryManager.readItem(accTransform, MEMORY_KEY_ACCTRANSFORM)) {

        LOG_MSG("No acc transform data found, creating new one\n");
        accTransform = Math::Matrix<float, 3, 3>::eye();
        memoryManager.allocateItem(accTransform, MEMORY_KEY_ACCTRANSFORM);
        memoryManager.writeItem(accTransform, MEMORY_KEY_ACCTRANSFORM);

    }

    gyroCalibData.val(0) = 0.92 * DEG_TO_RAD;
    gyroCalibData.val(1) = 2.0 * DEG_TO_RAD;
    gyroCalibData.val(2) = 0.6 * DEG_TO_RAD;

    /*accCalibData.val = {0.4278, 0.560, 2.0725};
    accCalibData.val(0) /= 0.9984;
    accCalibData.val(1) /= 1.003;
    accCalibData.val(2) /= 0.9673;
    accCalibData.cov = {
        0.9984, 0, 0,
        0, 1.003, 0,
        0, 0, 0.9673
    };*/

    gyroTransformTopic.setTransform(gyroCalibData.cov * gyroTransform, gyroCalibData.val);
    accTransformTopic.setTransform(accCalibData.cov * accTransform, accCalibData.val);
    magTransformTopic.setTransform(magCalibData.cov * magTransform, magCalibData.val);

    //Sync the internal memory with the EEPROM. This wont write anything if we ended up only reading.
    eeprom.transferFrom(internalMemory);

    //Lets update the memory with known values for the calibration data. This is only for testing purposes.
    if (false) {

        accCalibData.val = {0.4278, 0.560, 2.0725};
        accCalibData.val(0) /= 0.9984;
        accCalibData.val(1) /= 1.003;
        accCalibData.val(2) /= 0.9673;
        accCalibData.cov = {
            0.9984, 0, 0,
            0, 1.003, 0,
            0, 0, 0.9673
        };
        //memoryManager.writeItem(accCalibData, MEMORY_KEY_ACCCALIB);

        gyroCalibData.val = Math::Vector<float, 3>({0.228 * DEG_TO_RAD, 1.314 * DEG_TO_RAD, -0.5085 * DEG_TO_RAD});
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

        accTransform = {
            -1, 0, 0,
            0, 1, 0,
            0, 0, -1
        };

        memoryManager.writeItem(accTransform, MEMORY_KEY_ACCTRANSFORM);

        gyroTransformTopic.setTransform(gyroCalibData.cov * gyroTransform, gyroCalibData.val);
        accTransformTopic.setTransform(accCalibData.cov * accTransform, accCalibData.val);
        magTransformTopic.setTransform(magCalibData.cov * magTransform, magCalibData.val);

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

    //controlRocket.subscribeAttitudeMeasurement(attitudeTopicSwitch.getTopic());
    //controlRocket.subscribePositionMeasurement(positionTopicSwitch.getTopic());
    //controlRocket.subscribeSetpoint(positionSetpointTopic);
    //controlPositionStandard.subscribeSetpoint(positionSetpointTopic);
    controlPositionStandard.subscribePositionMeasurement(positionTopicSwitch.getTopic());

    controlMappingAccToAtt.subscribeAccelSetpoint(controlPositionStandard.getAccelTopic());

    controlAttitudeTvc.subscribeAttitudeMeasurement(attitudeTopicSwitch.getTopic());
    controlAttitudeTvc.subscribeAttitudeSetpoint(controlMappingAccToAtt.getAttitudeTopic());
    controlAttitudeTvc.subscribeAccelerationSetpoint(controlPositionStandard.getAccelTopic());
    controlAttitudeTvc.setTVCLimitCompensation(true);

    controlAttitudeFlaps.subscribeControlSetting(bellyFlopControlTopic);
    controlAttitudeFlaps.subscribeAttitudeMeasurement(attitudeTopicSwitch.getTopic());
    controlAttitudeFlaps.subscribeFlapSettingOutputTopic(flapSettingTopic);

    starshipTVC.setTVCInputTopic(controlAttitudeTvc.getTvcTopic());
    starshipFlaps.setFlapSettingTopic(flapSettingTopic);

    bodySimulator.setPaused(true);
    bodySimulator.setTVCInputTopic(starshipTVC.getTVCActualThrustTopic(), Math::Vector<float, 3>({0, 0, -0.35}), TVC_ANGLE_LIMIT_RAD, TVC_THRUST_LIMIT_N); // Subscribe to the TVC input topic

    positionTopicSwitch.subscribe(posEstTask.getStateEstTopic());
    attitudeTopicSwitch.subscribe(imuTask.getAttitudeEstTopic());


    Math::Quat<float> accTiltX({1, 0, 0}, 0*DEG_TO_RAD);
    Math::Quat<float> accTiltY({0, 1, 0}, 0*DEG_TO_RAD);
    imuTask.setAccTiltCompensation((accTiltX * accTiltY).to3x3RotMat());


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