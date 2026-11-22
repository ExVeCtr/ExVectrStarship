#include <Arduino.h>

#include "ExVectrActuator/pwm_output.hpp"
#include "ExVectrActuator/servo_control.hpp"
#include "ExVectrArduinoPlatform.hpp"
#include "ExVectrArduinoPlatform/bus_i2c.hpp"
#include "ExVectrArduinoPlatform/bus_spi.hpp"
#include "ExVectrArduinoPlatform/interrupt_event.hpp"
#include "ExVectrArduinoPlatform/memory_eeprom.hpp"
#include "ExVectrControl/control_attitude_tvc.hpp"
#include "ExVectrControl/control_mapping_acctoatt.hpp"
#include "ExVectrControl/control_position_standard.hpp"
#include "ExVectrCore.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/random.h"
#include "ExVectrCore/scheduler2.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrDSP/body_simulator.hpp"
#include "ExVectrDSP/imu_attitude_cf.hpp"
#include "ExVectrDSP/imu_attitude_ekf.hpp"
#include "ExVectrDSP/imu_gps_position_kf.hpp"
#include "ExVectrDSP/topic_coord_transform.hpp"
#include "ExVectrData/memory_internal.hpp"
#include "ExVectrData/memory_manager.hpp"
#include "ExVectrMath/specialised_types.hpp"
#include "ExVectrSensor/Sensors/bme280.hpp"
#include "ExVectrSensor/Sensors/mpu9250.hpp"
#include "ExVectrSensor/Sensors/qmc5883.hpp"
#include "board_v_1_0.h"
#include "calibration/calibration_math.hpp"
#include "calibration/calibration_service.hpp"
#include "calibration/calibration_task.hpp"
#include "calibration/calibration_validity.hpp"
#include "comms/vehicle_comms.hpp"
#include "memory_keys.hpp"
#include "starship_flaps.hpp"
#include "starship_hardware.h"
#include "starship_tvc.hpp"
#include "safety/estimator_zeroing.hpp"
#include "safety/control_phase.hpp"
#include "safety/mission_abort.hpp"
#include "safety/motor_interlock.hpp"
#include "safety/sim_flap_fault.hpp"
#include "safety/mission_chain.hpp"
#include "subsystems.hpp"
#include "telecommand.hpp"
#include "telemetry.hpp"
#include "ublox_gnss.hpp"
#include "vl53l0x/vl53l0x_driver.hpp"

#include "Adafruit_SSD1306.h"
#include "mission/mission_abstract.hpp"
#include "mission/mission_bellyflop.hpp"
#include "mission/mission_freefall.hpp"
#include "mission/mission_rth.hpp"
#include "mission/mission_waypoint.hpp"

// #define DO_FLAP_TEST_STARTUP

#define EXTEND_FLAPS_ASCENT false
// USB Serial carries only ExVectrNetwork stream frames (src/comms/vehicle_comms.hpp): never print to Serial
// directly. Console text (Core print, LOG_MSG) leaves as LOG_TEXT packets on USB.

using namespace VCTR;

// auto gnssRef = Math::Vector<double,
// 3>({49.945884*DEG_TO_RAD, 6.699575*DEG_TO_RAD, 324.7});

// Hardware
Platform::PinGPIO mpuIntPin(MPU9250_INT_PIN);
Platform::Interrupt_GPIO<SNSR::MPU9250Driver> mpuIntHandler(MPU9250_INT_PIN);
Platform::PinGPIO mpuPin(MPU9250_NCS_PIN);
Platform::BusSPIDevice spiMPU(SPI, mpuPin, true);
SNSR::MPU9250Driver mpuDriver(spiMPU, true);

// Platform::PinGPIO bmePin(BME280_NCS_PIN);
// Platform::BusSPIDevice bmeBus(SPI, bmePin, true);
Platform::BusI2CDevice bmeBus(Wire, 0x76);
SNSR::BME280Driver bme(bmeBus);

SNSR::UbloxSerialGNSS gnss(Serial5);


Platform::BusI2CDevice qmcBus(Wire, 0x0D);
SNSR::QMC5883Driver qmc(qmcBus);

Platform::BusI2CDevice vl53l0xBus(Wire, 0b0101001); // 0x29
SNSR::VL53L0XDriver vl53l0xDriver(vl53l0xBus, true, true);

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

// Memory managment
Data::Memory_ArduinoEEPROM eeprom(EEPROM, EEPROM.length());
Data::Memory_Internal<512> internalMemory;
Data::Memory_Manager memoryManager(internalMemory);

// Sensor Processing
DSP::TopicCoordTransform<float> gyroTransformTopic;
DSP::TopicCoordTransform<float> accTransformTopic;
DSP::TopicCoordTransform<float> magTransformTopic;

// Copy of what the three transforms above apply (cal = matrix * (raw - offset)), kept only for the CALIBRATION
// telemetry. Every setTransform goes through applySensorCalibration so the copy cannot diverge.
Comms::SensorCalibration gyroCalApplied, accCalApplied, magCalApplied;

constexpr uint8_t CAL_SRC_CALIB_EEPROM = Packets::Calibration::kGyroSourceCalibEeprom;
constexpr uint8_t CAL_SRC_TRANSFORM_EEPROM = Packets::Calibration::kGyroSourceTransformEeprom;
constexpr uint8_t CAL_SRC_RAM_ONLY = Packets::Calibration::kGyroSourceRamOnly;
constexpr uint8_t CAL_SRC_MEMORY_CLEARED = Packets::Calibration::kGyroSourceMemoryCleared;

/// transform.setTransform(matrix, offset), and records it for telemetry.
void applySensorCalibration(DSP::TopicCoordTransform<float> &transform,
                            Comms::SensorCalibration &record,
                            const Math::Matrix<float, 3, 3> &matrix,
                            const Math::Vector<float, 3> &offset,
                            uint8_t source) {
  transform.setTransform(matrix, offset);
  for (size_t r = 0; r < 3; r++) {
    record.offset[r] = offset(r);
    for (size_t c = 0; c < 3; c++)
      record.matrix[3 * r + c] = matrix(r, c);
  }
  record.source = source;
}

// ---- Sensor calibration (docs/project/calibration.md, src/calibration/) ----
// What the legacy EEPROM keys hold for each sensor (key 10x calibration, key 20x mounting) and whether the key was found
// in memory at boot (not created as a default). The applied transform is M = calib.cov * transform, offset = calib.val.
struct LegacyCal {
  DSP::ValueCov<float, 3> calib;
  Math::Matrix<float, 3, 3> transform;
  bool keyPresent = false;
  bool pendingSave = false; ///< the RAM image holds a value that the EEPROM does not (cleared by a verified Save)
};
LegacyCal gyroLegacy, accLegacy, magLegacy;

// Session engine and its raw-sample task. The mag sample buffer (7.2 KB) and the engine live in RAM2 (DMAMEM).
DMAMEM float calibMagBuffer[Calibration::CalibrationService::kMagBufferCapacity][3];
DMAMEM Calibration::CalibrationService calibrationService(calibMagBuffer,
                                                          Calibration::CalibrationService::kMagBufferCapacity);
Calibration::CalibrationTask calibrationTask(calibrationService);

// Validity inputs that change at run time: the gyro run result of this power cycle and the last still-window sample.
Calibration::GyroRunState gyroRunState = Calibration::GyroRunState::NoRun;
Calibration::LiveStill calibLiveStill;

static Calibration::StoredCalibration storedFrom(const LegacyCal &l) {
  Calibration::StoredCalibration s;
  s.keyPresent = l.keyPresent;
  for (size_t r = 0; r < 3; r++) {
    s.offset[r] = l.calib.val(r);
    for (size_t c = 0; c < 3; c++) {
      s.C[3 * r + c] = l.calib.cov(r, c);
      s.T[3 * r + c] = l.transform(r, c);
    }
  }
  return s;
}

/// All three calibrations good to fly? (src/calibration/calibration_validity.hpp)
Calibration::Validity currentCalibrationValidity() {
  Calibration::ValidityInput in;
  in.gyro = gyroRunState;
  in.accel = storedFrom(accLegacy);
  in.mag = storedFrom(magLegacy);
  in.live = calibLiveStill;
  return Calibration::calibrationsValid(in);
}

/// Tells the service what is applied now (offsets and M = C T), for the verify mode and the stillness bias.
void publishActiveCalibration() {
  const LegacyCal *all[3] = {&gyroLegacy, &accLegacy, &magLegacy};
  const Calibration::Sensor sens[3] = {Calibration::Sensor::Gyro, Calibration::Sensor::Accel,
                                       Calibration::Sensor::Mag};
  for (int i = 0; i < 3; i++) {
    const Calibration::StoredCalibration st = storedFrom(*all[i]);
    double M[9];
    Calibration::mat3Mul(st.C, st.T, M);
    calibrationService.setActive(sens[i], st.offset, M);
  }
}

/// Applies legacy key values (offset b, matrix C) (cal = C T (raw - b)) and, if writeImage, writes them to the RAM
/// image (persisted by the next Save). The record is marked RAM_ONLY either way. Returns false if the RAM image
/// write failed (the transform is applied anyway).
static bool applyLegacyCalibration(LegacyCal &l, uint32_t key, DSP::TopicCoordTransform<float> &topic,
                                   Comms::SensorCalibration &record, const double offset[3], const double C[9],
                                   bool writeImage = true) {
  for (size_t r = 0; r < 3; r++) {
    l.calib.val(r) = static_cast<float>(offset[r]);
    for (size_t c = 0; c < 3; c++)
      l.calib.cov(r, c) = static_cast<float>(C[3 * r + c]);
  }
  const bool written = writeImage ? memoryManager.writeItem(l.calib, key) : true;
  l.keyPresent = written || l.keyPresent;
  if (writeImage && written)
    l.pendingSave = true;
  applySensorCalibration(topic, record, l.calib.cov * l.transform, l.calib.val,
                         static_cast<uint8_t>(CAL_SRC_RAM_ONLY | (record.source & (CAL_SRC_TRANSFORM_EEPROM | CAL_SRC_CALIB_EEPROM |
                                                                                    CAL_SRC_MEMORY_CLEARED))));
  return written;
}

/// Gyro: only the bias changes (key 100 .val, in the raw sensor frame); C stays. The automatic run per boot or
/// countdown applies the bias to the transform only (persist = false: a Save must not write a per-boot value); the
/// manual CalibApply of a gyro candidate persists it with the next Save.
static void applyGyroBias(const double bias[3], bool persist) {
  const double C[9] = {gyroLegacy.calib.cov(0, 0), gyroLegacy.calib.cov(0, 1), gyroLegacy.calib.cov(0, 2),
                       gyroLegacy.calib.cov(1, 0), gyroLegacy.calib.cov(1, 1), gyroLegacy.calib.cov(1, 2),
                       gyroLegacy.calib.cov(2, 0), gyroLegacy.calib.cov(2, 1), gyroLegacy.calib.cov(2, 2)};
  applyLegacyCalibration(gyroLegacy, MEMORY_KEY_GYROCALIB, gyroTransformTopic, gyroCalApplied, bias, C, persist);
  publishActiveCalibration();
}

/// Writes the RAM image to EEPROM and reads it back. True only if the EEPROM now equals the image.
static bool saveCalibrationMemory() {
  eeprom.transferFrom(internalMemory);
  uint8_t a[64], b[64];
  const size_t total = internalMemory.size();
  for (size_t off = 0; off < total; off += sizeof(a)) {
    const size_t n = (total - off) < sizeof(a) ? (total - off) : sizeof(a);
    internalMemory.readMem(a, n, off);
    eeprom.readMem(b, n, off);
    if (memcmp(a, b, n) != 0)
      return false;
  }
  return true;
}

DSP::IMUAttitudeEKFTask imuTask(1 * Core::MILLISECONDS);
DSP::IMUGPSPositionKalmanTask posEstTask(10 * Core::MILLISECONDS);

DMAMEM DSP::BodySimulator bodySimulator(VEHICLE_MASS_KG, {0.044, 0.044, 0.004},
                                 TVC_THRUST_LIMIT_N, TVC_ANGLE_LIMIT_RAD);

// Networking: VehicleComms (src/comms/), defined below the vehicle tasks it reads from.
Core::Topic<Telecommand> telecommandTopic;
extern Comms::VehicleComms vehicleComms;

// Control
Core::Topic<CTRL::ControlAttitudeFlapSetting> flapSettingTopic;
Core::Topic<CTRL::ControlAttitudeFlapSetting> trueFlapSettingTopic;
Core::Topic<CTRL::ControlAttitudeBellyFlopSetting> bellyFlopControlTopic;

CTRL::ControlPositionStandard controlPositionStandard;
CTRL::ControlMappingAccToAtt controlMappingAccToAtt;
CTRL::ControlAttitudeTvc controlAttitudeTvc(VEHICLE_MASS_KG, TVC_THRUST_LIMIT_N,
                                            TVC_ANGLE_LIMIT_RAD);
CTRL::ControlAttitudeFlaps controlAttitudeFlaps;

CTRL::StarshipTVC starshipTVC(servoTVCXPPin, servoTVCXNPin, servoTVCYPPin,
                              servoTVCYNPin, motorCWPIN, motorCCWPIN,
                              TVC_ANGLE_LIMIT_RAD, TVC_SERVO_LIMIT,
                              TVC_THRUST_LIMIT_N);
CTRL::StarshipFlaps starshipFlaps(flapServoULPin, flapServoURPin,
                                  flapServoDLPin, flapServoDRPin);

// Topics and data routing
Core::Topic_Switch<Core::Timestamped<Math::Vector<float, 7>>>
    attitudeTopicSwitch;
Core::Topic_Switch<Core::Timestamped<Math::Vector<float, 6>>>
    positionTopicSwitch;
Core::Topic_Switch<Core::Timestamped<Math::Vector<float, 2>>> groundTopicSwitch;


MissionRTH defaultMission_(
    attitudeTopicSwitch.getTopic(), positionTopicSwitch.getTopic(),
    groundTopicSwitch.getTopic(), posEstTask.getGroundDistanceCovTopic(),
    {0, 0, 3}); // The default mission is the return to home mission.
MissionWaypoint missionWaypointTask(attitudeTopicSwitch.getTopic(),
                                    positionTopicSwitch.getTopic());
MissionFreefall missionFreefallTask(attitudeTopicSwitch.getTopic(),
                                    positionTopicSwitch.getTopic(),
                                    VEHICLE_MASS_KG, TVC_THRUST_LIMIT_N, 20);
// Kick timing, triggers, target and thrust limit: src/safety/bellyflop_kick.hpp.
MissionBellyflop missionBellyflop(attitudeTopicSwitch.getTopic(),
                                  controlAttitudeTvc, controlMappingAccToAtt);

/**
 * Hard motor interlock watchdog (T9b-2, docs/t9b-design.md 3.4.2). 100 Hz, priority above every controller (the
 * controllers run at 100..110), independent of the controller publications: the TVC task is event driven and does
 * not run if the controller stops, so this task closes the motor outputs on its own. While the gate is closed it
 * calls StarshipTVC::forceMotorsOff() every run (duty 0, sticky closed flag, modelled thrust 0); while open it
 * releases the flag. The safety task latches a failsafe if this task has not run for 100 ms
 * (Safety::interlockWatchdogStale). Reads no simulation flag.
 */
class MotorInterlockWatchdog : public Core::Task_Periodic {
public:
  static constexpr uint16_t kPriority = 120;
  static constexpr int64_t kPeriodNs = 10 * Core::MILLISECONDS;

  MotorInterlockWatchdog() : Task_Periodic("Motor interlock watchdog", kPeriodNs) {
    Core::getSystemScheduler().addTask(*this);
    setPriority(kPriority);
  }

  void taskThread() override {
    const int64_t now = Core::NowNs();
    lastRunNs_ = now;
    if (starshipTVC.interlockOpen(now)) {
      starshipTVC.releaseMotorsForce();
    } else {
      starshipTVC.forceMotorsOff();
    }
  }

  /// Time of the last run, 0 if it never ran.
  int64_t lastRunNs() const { return lastRunNs_; }

private:
  int64_t lastRunNs_ = 0;
};
MotorInterlockWatchdog motorInterlockWatchdog;

/**
 * This class takes care of enabling, disabling and setting the actuators for
 * the rocket. It also prepares the system for mission start and signals when
 * something is wrong.
 */
class VehicleSafetyAndControlTask : public Core::Task_Periodic {
private:
  const int64_t DATA_TIMEOUT = 0.5 * Core::SECONDS;

  const float POSITION_OUTOFBOUNDS_STATIONARY = 2.0f;         // m
  const float ANGLE_OUTOFBOUNDS_STATIONARY = 20 * DEG_TO_RAD; // rad

  const float POSITION_OUTOFBOUNDS_FLIGHT = 100.0f;        // m
  const float ANGLE_OUTOFBOUNDS_FLIGHT = 120 * DEG_TO_RAD; // rad

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

  MissionMode lastMissionMode_ = MissionMode::MissionMode_Idle;

  bool vehicleArmed_ = false;
  bool interlockOpenLogged_ = false; ///< last logged gate state (superviseMotorInterlock)
  bool allSystemsInitialised_ = false;
  bool simulationMode_ = false;
  SNSR::GnssConfigWarning gnssConfigWarning_; ///< GNSS configuration warning line (information only)

  // Estimator zeroing (docs/project/overview.md, "Estimator zeroing"): the ZeroEstimators / boot-zero window
  // and the "valid zero since boot" rule for Ready. missionZeroing_ is the zeroing requested by the active
  // mission's Initialisation mode (unchanged behaviour). The estimators are in zeroing mode while either holds.
  Safety::EstimatorZeroing zeroing_;
  // Telemetry cache (VEHICLE_STATUS home_epoch / calib_state), written by updateTelemetryCache() every task run.
  Comms::HomeEpoch homeEpoch_;
  bool calibGyroValid_ = false, calibAccelValid_ = false, calibMagValid_ = false;
  bool calibSessionActive_ = false, readyToArm_ = false;
  Safety::LogRateLimit zeroRefusedLog_{1 * Core::SECONDS};
  Safety::LogRateLimit zeroBlockedLog_{1 * Core::SECONDS};
  bool missionZeroing_ = false;
  bool zeroingApplied_ = false;

  // Sensor calibration policy (docs/project/calibration.md): all three calibrations must be good to fly. A warning
  // line repeats every 10 s while one is not, Arm and MissionBegin are refused with the reason (rate-limited), and the
  // countdown is aborted if the gyro run that closes it leaves one bad. No simulation exemption (owner 2026-11-21):
  // the sensors and stored keys are real in simulation, and none of these decisions reads the simulation flag.
  static constexpr int64_t CALIB_LOG_PERIOD = 10 * Core::SECONDS;
  Safety::LogRateLimit calibWarnLog_{CALIB_LOG_PERIOD};
  Safety::LogRateLimit calibArmLog_{CALIB_LOG_PERIOD};
  Safety::LogRateLimit calibBeginLog_{CALIB_LOG_PERIOD};
  Safety::LogRateLimit calibCmdLog_{1 * Core::SECONDS};
  bool countdownGyroRun_ = false; ///< the active automatic gyro run closes the mission countdown
  char lastCalibAbortReason_[96] = ""; ///< reason text of the last countdown abort for bad calibration (log only)

  size_t missionSelection_ = 0; // The index of the currently selected mission.
  Core::ListArray<MissionAbstract *> missionList_;

  MissionAbstract *mission_;

  // Mission chain switching (docs/project/missions.md section 6.2): a chain is pending only from an accepted
  // MissionBegin until RTH ends or a reset. A reset mission (mode Idle) never advances the chain.
  Safety::MissionChain missionChain_;

  // Explicit controller phase table (docs/project/missions.md section 6.2, src/safety/control_phase.hpp): applied
  // once per tick after all mission switches. missionSwitchCount_ counts switchMissionTo() calls so a mission
  // switch while the TVC set stays enabled (for example waypoint -> RTH on an abort) also resets it.
  Safety::ControlPhaseTracker controlPhase_;
  uint32_t missionSwitchCount_ = 0;
  uint32_t lastSwitchSeen_ = 0;
  bool bellyflopFlaps_ = false;

  MissionPhase missionPhase_ = MissionPhase::MissionPhase_Idle;

  // Log rate limit of the auto-disarm messages (logging only, no effect on the
  // arm logic): once when the reason changes, then every DISARM_LOG_PERIOD.
  static constexpr int64_t DISARM_LOG_PERIOD = 5 * Core::SECONDS;
  int lastDisarmReason_ = 0; // 0 none, 1 failure, 2 calib, 3 startup, 4 init, 5 calibration invalid
  int64_t lastDisarmLogNs_ = 0;

  /// True if the auto-disarm message for `reason` should be logged now.
  bool disarmLogDue(int reason) {
    const int64_t now = Core::NowNs();
    if (reason != lastDisarmReason_ ||
        now - lastDisarmLogNs_ >= DISARM_LOG_PERIOD) {
      lastDisarmReason_ = reason;
      lastDisarmLogNs_ = now;
      return true;
    }
    return false;
  }

public:
  VehicleSafetyAndControlTask()
      : Task_Periodic("Vehicle Safety and Control", 0.1 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
    // setPriority(500);
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
    failureState_.controlSourceLost = false;
    failureState_.interlockWatchdog = false;

    mission_ = &defaultMission_; // Set the mission to the default mission

    // mission_ = defaultMission_; // Set the mission to the default mission
  }

  const VehicleMode &getVehicleMode() { return vehicleMode_; }

  const SensoryState &getSensoryState() { return sensoryState_; }

  const FailureState &getFailureState() { return failureState_; }

  bool simulationModeEnabled() { return simulationMode_; }
  /// The live simulation flag (the flap fault model reads it at apply time).
  const bool *simulationFlag() const { return &simulationMode_; }

  bool isVehicleArmed() { return vehicleArmed_; }

  /// The ZeroEstimators / boot-zero window is active (VEHICLE_STATUS flag ZEROING).
  bool estimatorZeroingActive() const { return zeroing_.active(); }

  /// Telemetry only: the cached calibration state bits and the home epoch (VEHICLE_STATUS calib_state, home_epoch).
  /// The validity is evaluated by the safety task (10 Hz, updateTelemetryCache()), not by the telemetry task.
  void fillTelemetryCache(Comms::TelemetrySnapshot &s) const {
    s.homeEpoch = homeEpoch_.value();
    s.calibGyroValid = calibGyroValid_;
    s.calibAccelValid = calibAccelValid_;
    s.calibMagValid = calibMagValid_;
    s.calibSessionActive = calibSessionActive_;
    s.readyToArm = readyToArm_;
  }

  MissionAbstract *getCurrentMission() { return mission_; }

  MissionPhase getCurrentMissionPhase() { return missionPhase_; }

  /// Bellyflop controller set active (flap attitude control in BellyFlopMode Stabilize): freefall and the
  /// bellyflop after its powered kick (never together with the TVC set).
  bool bellyflopFlapsActive() const { return bellyflopFlaps_; }

  /// TVC set (ControlPositionStandard + ControlAttitudeTvc) enabled by the controller phase table (telemetry only).
  bool tvcControlActive() const { return controlPhase_.tvcEnabled(); }

  void switchMissionTo(MissionAbstract *mission, int64_t startTime = 0) {
    mission_->resetMission(); // Reset the mission
    mission_ = mission;       // Set the mission to the default mission
    mission_->resetMission(); // Reset the mission
    mission_->beginMission(startTime);
    controlPositionStandard.subscribeSetpoint(mission_->getSetpointTopic());
    missionSwitchCount_++;
  }

  Safety::ActiveMission activeMissionKind() const {
    if (mission_ == &defaultMission_)
      return Safety::ActiveMission::Rth;
    if (mission_ == &missionWaypointTask)
      return Safety::ActiveMission::Waypoint;
    if (mission_ == &missionBellyflop)
      return Safety::ActiveMission::Bellyflop;
    if (mission_ == &missionFreefallTask)
      return Safety::ActiveMission::Freefall;
    return Safety::ActiveMission::Other;
  }

  static Safety::MissionModeIn modeIn(MissionMode m) {
    switch (m) {
    case MissionMode::MissionMode_Initialisation:
      return Safety::MissionModeIn::Initialisation;
    case MissionMode::MissionMode_Startup:
      return Safety::MissionModeIn::Startup;
    case MissionMode::MissionMode_Running:
      return Safety::MissionModeIn::Running;
    case MissionMode::MissionMode_Finished:
      return Safety::MissionModeIn::Finished;
    default:
      return Safety::MissionModeIn::Idle;
    }
  }

  /**
   * The one place where the mission controllers are switched (user decision 2026-11-20): the phase table of
   * Safety::controlPhaseFor, evaluated every tick from the active mission and its mode (level, not edge), so the
   * first and every later run of a mission get the same controllers. TVC set (position + TVC attitude controller):
   * on only in the powered phases (waypoint, RTH Running, and the bellyflop kick with the thrust capped, 2026-11-21);
   * off in the countdown, after the kick and in freefall, where the disabled ControlAttitudeTvc publishes the
   * neutral [0, 0, 0.1 N, 0]. It is reset (integral, delta-time, setpoint hold) on every enable and on a mission
   * switch while enabled. Bellyflop set: flap attitude control (Stabilize) in freefall and after the kick, read by
   * FlapModeObserverTask. The thrust cap (StarshipTVC::thrustCap) is applied here every tick.
   */
  void applyControlPhase() {
    const bool switched = missionSwitchCount_ != lastSwitchSeen_;
    lastSwitchSeen_ = missionSwitchCount_;
    const Safety::ControlPhaseStep st = controlPhase_.update(
        activeMissionKind(), modeIn(mission_->getMissionState().missionMode), mission_->getActuatorsEnabled(),
        switched, Core::NowNs());
    if (st.resetTvc) {
      controlPositionStandard.resetControlState();
      controlAttitudeTvc.resetControlState();
    }
    controlPositionStandard.enableControl(st.phase.tvcEnabled);
    controlAttitudeTvc.enableControl(st.phase.tvcEnabled);
    starshipTVC.thrustCap(st.phase.tvcThrustCapN); // kick thrust limit in the bellyflop, none otherwise
    bellyflopFlaps_ = st.phase.bellyflopFlaps;
    if (st.changed || st.resetTvc) {
      LOG_MSG("Control phase: TVC controllers %s%s, bellyflop flap control %s\n", st.phase.tvcEnabled ? "on" : "off",
              st.resetTvc ? " (reset)" : "", st.phase.bellyflopFlaps ? "on" : "off");
    }
  }

  /**
   * "On the ground: reset and stop" (Safety::AbortAction::ResetAndStop), shared by the MissionAbort telecommand and the
   * calibration abort of the countdown: TVC actuators and motors off, everything disabled, the mission chain reset and
   * the active mission reset to Idle, failures cleared (so the vehicle goes Startup -> Ready), nothing follows. An
   * automatic gyro run that belonged to the countdown is cancelled.
   */
  void resetAndStopOnGround() {
    starshipTVC.enableActuators(false); // Disable actuators
    starshipTVC.enableMotors(false);
    vehicleShutdownControl(true); // Disable everything
    // Stop, not continue: before 2026-11-20 the reset mission's missionEnd() started the next mission of the
    // chain on the next tick. The mission stays the active one, in Idle.
    missionChain_.reset();
    mission_->resetMission();
    clearFailures();
    cancelCountdownGyroRun();
    vehicleComms.motorInterlock().requireRearm(Safety::MotorInterlock::RearmReason::Abort); // D4-A
  }

  void cancelCountdownGyroRun() {
    if (countdownGyroRun_) {
      calibrationService.cancelGyroRun();
      countdownGyroRun_ = false;
    }
  }

  /// Starts the automatic gyro bias run for a zeroing window ending at windowEndNs (boot zero, ZeroEstimators,
  /// countdown Initialisation): only if the window holds the 5 s still window plus the reserve (planGyroRun).
  /// Returns true if the run started.
  bool startGyroRunForWindow(int64_t windowEndNs, bool countdown) {
    const int64_t now = Core::NowNs();
    const Calibration::GyroRunPlan plan = Calibration::planGyroRun(now, windowEndNs);
    if (!plan.run) {
      LOG_MSG("Gyro bias run skipped: the zeroing window is shorter than %.0f s\n",
              double(Calibration::kGyroRunMinWindowNs) / Core::SECONDS);
      return false;
    }
    // A finished run that was not taken yet is delivered first, with the flags of the run that produced it.
    Calibration::GyroRunDone pending;
    if (calibrationService.takeGyroRunDone(pending))
      onGyroRunDone(pending);
    if (!calibrationService.startGyroRun(now, plan.deadlineNs)) {
      LOG_MSG("Gyro bias run NOT started: a calibration session is busy\n");
      return false;
    }
    countdownGyroRun_ = countdown;
    Core::printM("Calibration: gyro bias run started (window %.1f s, keep the vehicle still)\n",
                 double(plan.deadlineNs - now) / Core::SECONDS);
    return true;
  }

  /// The waypoint mission's countdown phases (Initialisation, Startup) while the vehicle is Running.
  bool inMissionCountdown() {
    const MissionMode mm = mission_->getMissionState().missionMode;
    return vehicleMode_ == VehicleMode::VehicleMode_Running && mission_ == &missionWaypointTask &&
           (mm == MissionMode::MissionMode_Initialisation || mm == MissionMode::MissionMode_Startup);
  }

  /**
   * An automatic gyro run finished. A pass applies the measured bias (RAM image and transform; the EKF is still in
   * zeroing mode for the last second of the window, deadline = end - 1 s), makes the gyro valid for this power cycle
   * and records the still-window accel/mag means for the live check. A failure leaves the previous bias applied and the
   * gyro invalid. A run that closed the countdown then re-checks ALL THREE calibrations and aborts on the ground if one
   * is bad.
   */
  void onGyroRunDone(const Calibration::GyroRunDone &done) {
    const auto &r = done.result;
    const bool countdown = countdownGyroRun_;
    countdownGyroRun_ = false;
    const bool groundMode = vehicleMode_ == VehicleMode::VehicleMode_Startup ||
                            vehicleMode_ == VehicleMode::VehicleMode_Ready ||
                            vehicleMode_ == VehicleMode::VehicleMode_SensorCalib;
    if (r.result == DSP::Calib::CalResult::Pass &&
        !Calibration::gyroBiasMayApply(groundMode, countdown, inMissionCountdown())) {
      // A late result (starved task, a run overlapping a flight phase) must never step the bias in flight.
      Core::printW("Calibration: gyro bias run PASS discarded, the vehicle is no longer on the ground or in its countdown "
                   "(mode %d)\n", int(vehicleMode_));
    } else if (r.result == DSP::Calib::CalResult::Pass) {
      const float oldB[3] = {gyroLegacy.calib.val(0), gyroLegacy.calib.val(1), gyroLegacy.calib.val(2)};
      applyGyroBias(r.bias, false);
      gyroRunState = Calibration::GyroRunState::Ok;
      calibLiveStill.haveAccel = r.nAccel > 0;
      calibLiveStill.haveMag = r.nMag > 0;
      for (int k = 0; k < 3; k++) {
        calibLiveStill.accelRaw[k] = r.accelMean[k];
        calibLiveStill.magRaw[k] = r.magMean[k];
      }
      Core::printM("Calibration: gyro bias run PASS, bias %.3f %.3f %.3f deg/s (was %.3f %.3f %.3f), noise %.3f %.3f %.3f deg/s, "
                   "IMU %.1f C\n",
                   r.bias[0] * RAD_TO_DEG, r.bias[1] * RAD_TO_DEG, r.bias[2] * RAD_TO_DEG, double(oldB[0]) * RAD_TO_DEG,
                   double(oldB[1]) * RAD_TO_DEG, double(oldB[2]) * RAD_TO_DEG, r.noiseStd[0] * RAD_TO_DEG,
                   r.noiseStd[1] * RAD_TO_DEG, r.noiseStd[2] * RAD_TO_DEG, r.tempMean);
    } else {
      gyroRunState = Calibration::GyroRunState::Failed;
      char why[160];
      DSP::Calib::describeGyroRun(r, DSP::Calib::GyroBiasConfig(), why, sizeof why);
      Core::printW("Calibration: gyro bias run FAILED (%s, %lu restarts, checks 0x%x; %s): gyro calibration invalid until a "
                   "run passes (ZeroEstimators, vehicle still)\n",
                   DSP::Calib::toString(r.result), (unsigned long)r.restarts, (unsigned)r.checks, why);
      if (r.checks & DSP::Calib::kGyroCheckComplete) // window complete but bias/noise/temperature check failed
        Core::printW("Calibration: gyro bias run window values: bias %.3f %.3f %.3f deg/s (limit 5), noise %.3f %.3f %.3f "
                     "deg/s, IMU temp change %.2f K\n",
                     r.bias[0] * RAD_TO_DEG, r.bias[1] * RAD_TO_DEG, r.bias[2] * RAD_TO_DEG, r.noiseStd[0] * RAD_TO_DEG,
                     r.noiseStd[1] * RAD_TO_DEG, r.noiseStd[2] * RAD_TO_DEG, r.tempDelta);
    }
    // Margins against the motion limits (for tuning them after the first bench sessions).
    const DSP::Calib::GyroBiasConfig limits;
    Core::printM("Calibration: worst 1 s block: gyro std %.3f deg/s p2p %.3f deg/s (limits %.2f, %.2f), accel axis std %.3f "
                 "|a| std %.3f m/s^2 (limits %.2f, %.2f), %lu short blocks\n",
                 r.worst.gyroStd * RAD_TO_DEG, r.worst.gyroP2P * RAD_TO_DEG, limits.maxBlockStd * RAD_TO_DEG,
                 limits.maxBlockP2P * RAD_TO_DEG, r.worst.accelAxisStd, r.worst.accelNormStd, limits.maxAccelAxisStd,
                 limits.maxAccelNormStd, (unsigned long)r.worst.shortBlocks);
    Core::printM("Calibration: quietest of %lu blocks: gyro std %.3f deg/s p2p %.3f deg/s, accel axis std %.3f |a| std %.3f "
                 "m/s^2\n",
                 (unsigned long)r.worst.blocks, r.worst.minGyroStd * RAD_TO_DEG, r.worst.minGyroP2P * RAD_TO_DEG,
                 r.worst.minAccelAxisStd, r.worst.minAccelNormStd);

    if (countdown)
      checkCalibrationsAtCountdownEnd();
  }

  /// Gyro run of the countdown finished: all three must be good, else the mission is aborted before liftoff.
  void checkCalibrationsAtCountdownEnd() {
    const Calibration::Validity v = currentCalibrationValidity();
    // The countdown phases are "on the ground" (Safety::decideAbort): reset and stop, as for MissionAbort there.
    const bool inCountdown = inMissionCountdown();
    char reason[96];
    Calibration::describe(v, reason, sizeof(reason));
    switch (Calibration::decideCountdownEnd(v, inCountdown)) {
    case Calibration::CountdownAction::Continue:
      Core::printM("Calibration: all three calibrations checked at the end of the gyro run, countdown continues\n");
      break;
    case Calibration::CountdownAction::BadButNotInCountdown:
      Core::printW("Calibration invalid (%s) but the countdown is no longer running: no abort needed\n", reason);
      break;
    case Calibration::CountdownAction::Abort:
      snprintf(lastCalibAbortReason_, sizeof(lastCalibAbortReason_), "%s", reason);
      Core::printW("MISSION ABORTED before liftoff: calibration invalid (%s). Reset and stop.\n", reason);
      resetAndStopOnGround();
      break;
    }
  }

  /**
   * SystemReset (user decision 2026-11-20): the mission state becomes the state after boot, in simulation and in
   * real mode alike. No chain step is pending, every mission object is reset (Idle, mission time 0, setpoint
   * zero, actuators off, bellyflop paused with the TVC attitude setpoint back on the acc-to-att mapping), the
   * default mission (RTH, Idle) is active, the mission controllers and the flaps are off and the mission-mode edge
   * tracking starts again from Idle. Nothing can start again without a new MissionBegin.
   */
  void resetMissionsToBootState() {
    missionChain_.reset();
    missionWaypointTask.resetMission();
    missionBellyflop.resetMission();
    missionFreefallTask.resetMission();
    defaultMission_.resetMission();
    mission_ = &defaultMission_;
    controlPositionStandard.subscribeSetpoint(defaultMission_.getSetpointTopic());
    missionSelection_ = 0;
    missionPhase_ = MissionPhase::MissionPhase_Idle;
    // What the Idle mode edge does, applied now so that no edge is pending.
    missionZeroing_ = false; // applied by applyEstimatorZeroing()
    controlAttitudeTvc.enableControl(false);
    controlPositionStandard.enableControl(false);
    controlPhase_.reset();
    starshipTVC.thrustCap(Safety::kNoTvcThrustCapN);
    lastSwitchSeen_ = missionSwitchCount_;
    bellyflopFlaps_ = false;
    starshipFlaps.enableActuators(false);
    lastMissionMode_ = MissionMode::MissionMode_Idle;
  }

  void taskInit() override {
    gnssSubr.subscribe(gnss.getGNSSTopic());
    gyroSubr.subscribe(gyroTransformTopic.getOutputTopic());
    magSubr.subscribe(magTransformTopic.getOutputTopic());
    baroSubr.subscribe(bme.getBaroTopic());
    posEstSubr.subscribe(positionTopicSwitch.getTopic());
    attEstSubr.subscribe(attitudeTopicSwitch.getTopic());

    telecommandSubr.subscribe(telecommandTopic);

    missionList_.append(&defaultMission_);
    missionList_.append(&missionWaypointTask);

    missionWaypointTask.addWaypoint({0, 0, 0.3}, 0, 3, 1, 5, 0 * Core::SECONDS);
    missionWaypointTask.addWaypoint(
        {0, 0, 1}, 0.5, 3, 1, 0.5, 5 * Core::SECONDS, 10 * Core::SECONDS, true);
    missionWaypointTask.addWaypoint({0, 0, 120}, 7, 3, 2, 3, 5 * Core::SECONDS);
    missionWaypointTask.setNextMission(&missionBellyflop);
    missionBellyflop.setNextMission(&missionFreefallTask);
    missionFreefallTask.setNextMission(&defaultMission_);

    // missionWaypointTask.addWaypoint({0, 0, 0.3}, 0, 3, 1, 5, 0 *
    // Core::SECONDS); missionWaypointTask.addWaypoint({0, 0, 1}, 0.5, 3, 1,
    // 0.5,
    //                                 20 * Core::SECONDS, 10 * Core::SECONDS,
    //                                 true);
    // missionWaypointTask.addWaypoint({0, -5, 1.5}, 0.5, 0.5, 0.5, 1,
    //                                 20 * Core::SECONDS);
    // missionWaypointTask.setNextMission(&defaultMission_);
  }

  /// "Committed" (design 3.1.3): the motor interlock is open, or a mission chain is pending, or the mode is Running.
  /// Latches the control source (VehicleNetwork::setCommitted) and enables the control failsafe (checkForFailures).
  bool isCommitted(int64_t now) {
    return vehicleComms.motorInterlock().open(now) || missionChain_.pending() ||
           vehicleMode_ == VehicleMode::VehicleMode_Running;
  }

  /// Motor interlock supervision (T9b-2), every run, before anything can return early: the latch input of the
  /// control source (interlock open, mission chain pending or Running), the D4-A re-arm while in Failure, and a log
  /// line on every gate change. The gate itself is evaluated at the motor outputs and by the watchdog, not here.
  void superviseMotorInterlock() {
    const int64_t now = Core::NowNs();
    auto &il = vehicleComms.motorInterlock();
    vehicleComms.setCommitted(isCommitted(now));
    const bool open = il.open(now);
    if (vehicleMode_ == VehicleMode::VehicleMode_Failure)
      il.requireRearm(Safety::MotorInterlock::RearmReason::Failure); // D4-A: a failsafe needs a new toggle
    if (open != interlockOpenLogged_) {
      interlockOpenLogged_ = open;
      if (open) {
        LOG_MSG("Motor interlock OPEN (SD enabled, transition observed)\n");
      } else {
        static const char *const why[] = {"open", "no control source", "SD enabled but needs a toggle (off, then on)",
                                          "SD disabled", "no valid enabled state within 1 s"};
        LOG_MSG("Motor interlock CLOSED: %s\n", why[static_cast<uint8_t>(il.closeReason(now))]);
      }
    }
  }

  void taskThread() override {
    handleTelecommands();
    superviseMotorInterlock();

    // GNSS configuration visibility (review 2026-11-20). Information only: no mode, Ready or Arm decision uses it.
    // VehicleStatus has no free flag bit, so a warning LOG_TEXT line repeats every 10 s while the configuration is
    // not as intended (sequence failed, a step skipped, or the read-back is not GPS + Galileo).
    if (gnssConfigWarning_.due(gnss.configFinished(), gnss.configComplete() && gnss.constellationsAsIntended(),
                               Core::NowNs())) {
      VCTR::Core::printW("GNSS configuration not as intended: configured %s, GPS+Galileo %s (information only, "
                         "repeats every 10 s)\n",
                         gnss.configComplete() ? "yes" : "NO", gnss.constellationsAsIntended() ? "yes" : "NO");
    }

    if (zeroing_.update(Core::NowNs())) {
      LOG_MSG("Estimator zeroing complete, valid zero recorded\n");
    }

    calibrationHousekeeping();
    updateTelemetryCache();

    if (!systemsInitialised()) {
      starshipTVC.enableActuators(false); // Disable actuators
      vehicleShutdownControl(true);       // Disable everything
      applyControlPhase();
      applyEstimatorZeroing();
      return; // Wait until all systems are initialised
    }

    // Automatic boot zero: once all inputs have published and no valid zero exists since boot. The vehicle
    // stays in Startup until it has completed (vehicleIsReady, zeroing_.readyAllowed()).
    if (zeroing_.shouldStartAutoZero(allSystemsInitialised_,
                                     vehicleMode_ == VehicleMode::VehicleMode_Startup)) {
      zeroing_.startAuto(Core::NowNs());
      LOG_MSG("Automatic estimator zeroing started (%.0f s), keep the vehicle still\n",
              double(Safety::EstimatorZeroing::kDefaultDurationS));
      startGyroRunForWindow(zeroing_.endNs(), false);
    }

    if (mission_ == &defaultMission_) {
      if (defaultMission_.missionEnd()) {
        missionPhase_ = MissionPhase::MissionPhase_Completed;
      } else if (defaultMission_.stabilising()) {
        missionPhase_ = MissionPhase::MissionPhase_Deceleration;
      } else if (defaultMission_.translation()) {
        missionPhase_ = MissionPhase::MissionPhase_RTH;
      } else if (defaultMission_.descent()) {
        missionPhase_ = MissionPhase::MissionPhase_Landing;
      }
    } else if (mission_ == &missionWaypointTask) {
      if (missionWaypointTask.currentWaypointIndex() > 1) {
        missionPhase_ = MissionPhase::MissionPhase_Ascent;
      } else {
        missionPhase_ = MissionPhase::MissionPhase_Hover;
      }
    } else if (mission_ == &missionBellyflop ||
               mission_ == &missionFreefallTask) {
      missionPhase_ = MissionPhase::MissionPhase_Descent;
    } else {
      missionPhase_ = MissionPhase::MissionPhase_Idle;
    }

    // if (mission_ == &defaultMission_) {
    //     LOG_MSG("Default mission selected.\n"); // Log the default mission
    //     selection
    // }

    checkForFailures();

    handleVehicleMissionControl(); // Handle the vehicle mission control

    auto vehicleReady = vehicleIsReady();
    // Belt and braces: no actuation while a zeroing window is active (Arm and MissionBegin are refused then,
    // and ZeroEstimators is only accepted on the ground, so this is normally already the case).
    vehicleShutdownControl(!vehicleReady || zeroing_.active());

    vehicleArmingControl();

    applyEstimatorZeroing();
  }

  /// Puts the position KF, the attitude EKF and the body simulator into zeroing mode while the mission's
  /// Initialisation mode or the zeroing window asks for it (same three estimators in simulation and flight).
  void applyEstimatorZeroing() {
    const bool want = missionZeroing_ || zeroing_.active();
    if (want == zeroingApplied_)
      return;
    posEstTask.enableZeroingMode(want);
    imuTask.enableZeroingMode(want);
    bodySimulator.enableZeroingMode(want);
    // The position reference (GNSS reference point of the position KF) moves only in zeroing mode: when it ends the
    // reference is set (home_epoch).
    if (zeroingApplied_ && !want)
      homeEpoch_.referenceSet();
    zeroingApplied_ = want;
  }

  void handleVehicleMissionControl() {
    MissionState missionState = mission_->getMissionState();
    missionState.missionIndex = missionSelection_; // Set the mission index to
    // the current mission index

    // Mode-edge actions (zeroing, actuator test, flap enable; the flap enable is overridden every tick by
    // vehicleShutdownControl). The mission controllers are NOT switched here any more: applyControlPhase() does it
    // from the phase table every tick (2026-11-20; the edge-based enable made the first bellyflop after boot differ
    // from later ones and left RTH without controllers after an abort from freefall).
    if (missionState.missionMode != lastMissionMode_) {
      if (missionState.missionMode == MissionMode::MissionMode_Idle) {
        missionZeroing_ = false; // applied by applyEstimatorZeroing()
        starshipFlaps.enableActuators(false);
      } else if (missionState.missionMode ==
                 MissionMode::MissionMode_Initialisation) {
        missionZeroing_ = true; // estimator zeroing on (applyEstimatorZeroing())
        starshipFlaps.enableActuators(true);
      } else if (missionState.missionMode == MissionMode::MissionMode_Startup) {
        missionZeroing_ = false;
        starshipTVC.beginActuatorTest(0); // Start the actuator test
#ifdef DO_FLAP_TEST_STARTUP
        starshipFlaps.beginActuatorTest(
            4 * Core::SECONDS); // Start the actuator test
#endif
        starshipFlaps.enableActuators(false);
      } else if (missionState.missionMode == MissionMode::MissionMode_Running) {
        missionZeroing_ = false;
        starshipFlaps.enableActuators(true);
      } else if (missionState.missionMode ==
                 MissionMode::MissionMode_Finished) {
        missionZeroing_ = false;
        starshipFlaps.enableActuators(false);
      }

      lastMissionMode_ = missionState.missionMode;
    }

    // Chain switching. missionEnd() alone is not enough: every resetMission() sets it, so a reset mission would
    // start the next one (the SystemReset bug, 2026-11-20). Safety::MissionChain only advances a pending chain
    // whose active mission ended without being reset (docs/project/missions.md section 6.2).
    Safety::ChainInput chainIn;
    chainIn.activeIsDefault = (mission_ == &defaultMission_);
    chainIn.activeIdle = (missionState.missionMode == MissionMode::MissionMode_Idle);
    chainIn.activeEnded = mission_->missionEnd();
    chainIn.nextIsDefaultOrNone =
        (mission_->nextMission() == &defaultMission_ || mission_->nextMission() == nullptr);
    // RTH instead of the bellyflop after a failed hover check or below kMinKickAltitudeM (owner 2026-11-21).
    chainIn.nextIsBellyflop = (mission_->nextMission() == &missionBellyflop);
    chainIn.activeCancelled =
        (mission_ == &missionWaypointTask && missionWaypointTask.endReason() == MissionWaypoint::EndReason::Cancelled);
    chainIn.altitudeM = posEstSubr.getItem().data(5);
    const bool chainWasPending = missionChain_.pending();

    switch (missionChain_.step(chainIn)) {
    case Safety::ChainStep::RthEnded:
      if (starshipTVC.isActuatorsEnabled()) {
        LOG_MSG("RTH mission ended.\n"); // Log the mission end
      }

      starshipTVC.enableMotors(false);      // Disable motors
      starshipFlaps.enableActuators(false); // Disable actuators
      vehicleShutdownControl(true);         // Disable everything
      break;

    case Safety::ChainStep::SwitchToDefault:
      if (missionChain_.redirect() != Safety::ChainRedirect::None) {
        LOG_MSG("Mission ended: RTH instead of the next mission (%s, altitude %.1f m, minimum %.0f m)\n",
                Safety::toString(missionChain_.redirect()), double(chainIn.altitudeM),
                double(Safety::kMinKickAltitudeM));
      } else {
        LOG_MSG("Mission ended. Switching to default mission.\n");
      }
      missionSelection_ = 0; // Set the mission selection to the default mission
      switchMissionTo(&defaultMission_); // Switch to the default mission
      break;

    case Safety::ChainStep::SwitchToNext:
      LOG_MSG("Mission ended. Switching to next mission.\n");
      switchMissionTo(mission_->nextMission()); // Switch to the next mission
      break;

    case Safety::ChainStep::None:
      if (chainWasPending && !missionChain_.pending()) {
        LOG_MSG("Mission chain cancelled: active mission was reset\n");
      }
      break;
    }

    // D4-A: the end of a mission chain (RTH landed, or the pending chain was cancelled) needs a new SD off -> on.
    if (chainWasPending && !missionChain_.pending())
      vehicleComms.motorInterlock().requireRearm(Safety::MotorInterlock::RearmReason::MissionEnd);

    // After every switch of this tick (telecommands, failure checks, chain): same table on every run.
    applyControlPhase();
  }

  // ---- Sensor calibration commands (legacy telecommands, docs/project/calibration.md) ----

  /// Preconditions of every calibration command (design section 3.3): initialised, disarmed, motors off, mode
  /// Startup/Ready/SensorCalib, mission Idle or Finished (the same in simulation and real mode: calibration works on
  /// the real sensors). Logs the refusal (1/s).
  bool calibrationCommandAllowed(const char *what) {
    Calibration::CommandContext c;
    c.initialised = allSystemsInitialised_;
    c.armed = vehicleArmed_;
    c.motorsEnabled = starshipTVC.isMotorsEnabled();
    c.modeAllowed = vehicleMode_ == VehicleMode::VehicleMode_Startup || vehicleMode_ == VehicleMode::VehicleMode_Ready ||
                    vehicleMode_ == VehicleMode::VehicleMode_SensorCalib;
    const MissionMode mm = mission_->getMissionState().missionMode;
    c.missionIdleOrFinished = mm == MissionMode::MissionMode_Idle || mm == MissionMode::MissionMode_Finished;
    const Calibration::CommandResult r = Calibration::checkCommand(c);
    if (r == Calibration::CommandResult::Ok)
      return true;
    if (calibCmdLog_.due(Core::NowNs()))
      Core::printW("Calibration %s refused: %s\n", what, Calibration::toString(r));
    return false;
  }

  static bool calibrationSensor(Subsystem sub, Calibration::Sensor &out) {
    switch (sub) {
    case Subsystem::Subsystem_Gyro: out = Calibration::Sensor::Gyro; return true;
    case Subsystem::Subsystem_Accel: out = Calibration::Sensor::Accel; return true;
    case Subsystem::Subsystem_Magneto: out = Calibration::Sensor::Mag; return true;
    default: return false;
    }
  }

  /**
   * CalibStart: subsystem 0 gyro, 1 accel, 2 mag. paramInt 1 (or any value > 2) calibrate, 2 verify the active
   * calibration, -1 discard a candidate (also ends a collection), 0 or any other value <= 0 stop and fit.
   */
  void handleCalibStart(const Telecommand &tc) {
    Calibration::Sensor sensor;
    if (!calibrationSensor(tc.subsystem, sensor))
      return;
    if (!calibrationCommandAllowed("command"))
      return;
    const int64_t now = Core::NowNs();

    if (tc.paramInt == -1) {
      calibrationService.discard(sensor);
      return;
    }
    if (tc.paramInt <= 0) {
      if (!calibrationService.stop(sensor, now))
        Core::printW("Calibration stop ignored: %s is not collecting\n", Calibration::toString(sensor));
      return;
    }

    const bool verify = tc.paramInt == 2;
    if (calibrationService.anyBusy()) {
      Core::printW("Calibration start refused: another calibration is running, stop or discard it first\n");
      return;
    }
    // The vehicle will be handled or rotated: an active zeroing window is aborted, and whatever zero exists is no
    // longer trusted (the automatic zero runs again when the session ends).
    if (zeroing_.abort())
      Core::printM("Estimator zeroing aborted by calibration start\n");
    zeroing_.invalidate();
    cancelCountdownGyroRun();
    calibrationService.cancelGyroRun();
    publishActiveCalibration();
    calibrationService.setGyroBiasTrusted(gyroRunState == Calibration::GyroRunState::Ok);
    if (calibrationService.start(sensor, verify, now) == Calibration::StartResult::Started) {
      starshipTVC.enableActuators(false);
      vehicleMode_ = VehicleMode::VehicleMode_SensorCalib;
      applyEstimatorZeroing();
    }
  }

  /// CalibApply: only a candidate with verdict PASS or WARN (owner decision D6). Writes the legacy keys (RAM image),
  /// applies the transform, invalidates the estimator zero (the automatic zero runs again before Ready).
  void handleCalibApply(const Telecommand &tc) {
    Calibration::Sensor sensor;
    if (!calibrationSensor(tc.subsystem, sensor))
      return;
    if (!calibrationCommandAllowed("apply"))
      return;
    if (!calibrationService.applicable(sensor)) {
      Core::printW("Calibration apply refused: no passing candidate for %s (state %s, result %s)\n",
                   Calibration::toString(sensor), Calibration::toString(calibrationService.state(sensor)),
                   DSP::Calib::toString(calibrationService.result(sensor)));
      return;
    }

    // Estimator inputs are about to change: an active zeroing window is void.
    if (zeroing_.abort())
      Core::printM("Estimator zeroing aborted by calibration apply\n");
    calibrationService.cancelGyroRun();
    countdownGyroRun_ = false;

    double offset[3], C[9];
    bool written = true;
    switch (sensor) {
    case Calibration::Sensor::Accel: {
      const Calibration::StoredCalibration st = storedFrom(accLegacy);
      if (!Calibration::accelCandidateToLegacy(calibrationService.accelResult(), st.T, offset, C)) {
        Core::printW("Calibration apply refused: accel mounting matrix is singular\n");
        return;
      }
      written = applyLegacyCalibration(accLegacy, MEMORY_KEY_ACCCALIB, accTransformTopic, accCalApplied, offset, C);
      break;
    }
    case Calibration::Sensor::Mag: {
      const Calibration::StoredCalibration st = storedFrom(magLegacy);
      if (!Calibration::magCandidateToLegacy(calibrationService.magResult(), st.T, offset, C)) {
        Core::printW("Calibration apply refused: mag mounting matrix is singular\n");
        return;
      }
      written = applyLegacyCalibration(magLegacy, MEMORY_KEY_MAGCALIB, magTransformTopic, magCalApplied, offset, C);
      break;
    }
    case Calibration::Sensor::Gyro: {
      const auto &r = calibrationService.gyroResult();
      applyGyroBias(r.bias, true);
      gyroRunState = Calibration::GyroRunState::Ok;
      calibLiveStill.haveAccel = r.nAccel > 0;
      calibLiveStill.haveMag = r.nMag > 0;
      for (int k = 0; k < 3; k++) {
        calibLiveStill.accelRaw[k] = r.accelMean[k];
        calibLiveStill.magRaw[k] = r.magMean[k];
      }
      break;
    }
    }
    publishActiveCalibration();
    calibrationService.markApplied(sensor);
    zeroing_.invalidate();
    if (vehicleMode_ == VehicleMode::VehicleMode_Ready)
      vehicleMode_ = VehicleMode::VehicleMode_Startup; // a new zero before Ready
    Core::printM("Calibration: %s calibration applied (RAM only until Save)%s; the estimator zero is invalid, the automatic "
                 "zero runs next: keep the vehicle still\n",
                 Calibration::toString(sensor), written ? "" : ", RAM image write FAILED");
  }

  /// Save (any of gyro, accel, magneto): RAM image -> EEPROM, read back and compared.
  void handleCalibSave(const Telecommand &tc) {
    Calibration::Sensor sensor;
    if (!calibrationSensor(tc.subsystem, sensor))
      return;
    if (!calibrationCommandAllowed("save"))
      return;
    if (calibrationService.anyBusy()) {
      Core::printW("Calibration save refused: a calibration is still running\n");
      return;
    }
    if (saveCalibrationMemory()) {
      // Only records whose value went into the RAM image are now in EEPROM. The per-boot automatic gyro bias is not
      // written to the image, so its source keeps RAM_ONLY until the next boot.
      Comms::SensorCalibration *recs[3] = {&gyroCalApplied, &accCalApplied, &magCalApplied};
      LegacyCal *legs[3] = {&gyroLegacy, &accLegacy, &magLegacy};
      for (int i = 0; i < 3; i++) {
        if (legs[i]->pendingSave) {
          recs[i]->source = static_cast<uint8_t>((recs[i]->source & ~CAL_SRC_RAM_ONLY) | CAL_SRC_CALIB_EEPROM);
          legs[i]->pendingSave = false;
        }
      }
      calibrationService.markSaved();
      Core::printM("Calibration: saved to EEPROM and verified by read-back\n");
    } else {
      Core::printW("Calibration: EEPROM save FAILED (read-back differs), the values stay in RAM only\n");
    }
  }

  /// MissionBegin gate (Calibration::decideMissionBegin, same in simulation and real mode): all three calibrations
  /// good, and a countdown long enough for the gyro run. Logs the refusal every 10 s.
  bool missionBeginCalibrationOk(int16_t paramInt) {
    const Calibration::Validity v = currentCalibrationValidity();
    switch (Calibration::decideMissionBegin(v, paramInt)) {
    case Calibration::GateRefusal::None:
      return true;
    case Calibration::GateRefusal::CalibrationInvalid:
      if (calibBeginLog_.due(Core::NowNs())) {
        char why[128];
        Calibration::describe(v, why, sizeof(why));
        Core::printW("Mission begin refused: calibration invalid (%s)\n", why);
      }
      return false;
    case Calibration::GateRefusal::CountdownTooShort:
      if (calibBeginLog_.due(Core::NowNs()))
        Core::printW("Mission begin refused: countdown too short (%d s), the gyro calibration needs the Initialisation "
                     "phase of at least 8 s: use a start time of %d s or earlier\n",
                     int(paramInt), int(Calibration::kMinCountdownParamInt));
      return false;
    }
    return false;
  }

  /// MissionBegin step: the countdown's Initialisation phase (mission time < -10 s) is a zeroing window, and the gyro
  /// bias run in it closes the calibration check. decideMissionBegin guarantees a window of at least 8 s, so the run
  /// is always planned; if it cannot start anyway (a session is busy) MissionBegin is REFUSED, because the mission
  /// would otherwise fly without the countdown re-check. Same in simulation and real mode.
  bool startCountdownGyroRun(int16_t paramInt) {
    if (paramInt < -int(Calibration::kCountdownInitialisationEndS) &&
        startGyroRunForWindow(Calibration::countdownWindowEndNs(Core::NowNs(), paramInt), true))
      return true;
    if (calibBeginLog_.due(Core::NowNs()))
      Core::printW("Mission begin refused: the countdown gyro calibration run could not start (see the log line above)\n");
    return false;
  }

  /// Per-tick calibration upkeep: results of the automatic gyro run, the end of a calibration session (back to
  /// Startup), and the warning line while a calibration is invalid.
  /// Telemetry only (safety.md rule 9: no vehicle behaviour reads these). Same gate function as the Arm command.
  void updateTelemetryCache() {
    const Calibration::Validity v = currentCalibrationValidity();
    calibGyroValid_ = v.gyro == Calibration::Reason::Ok;
    calibAccelValid_ = v.accel == Calibration::Reason::Ok;
    calibMagValid_ = v.mag == Calibration::Reason::Ok;
    calibSessionActive_ = calibrationService.anyBusy() || calibrationService.gyroRunActive();
    readyToArm_ = Calibration::readyToArm(v, vehicleMode_ == VehicleMode::VehicleMode_Ready, zeroing_.active());
  }

  void calibrationHousekeeping() {
    Calibration::GyroRunDone done;
    if (calibrationService.takeGyroRunDone(done))
      onGyroRunDone(done);

    if (vehicleMode_ == VehicleMode::VehicleMode_SensorCalib && !calibrationService.anyBusy()) {
      vehicleMode_ = VehicleMode::VehicleMode_Startup; // the automatic zero runs before Ready
      Core::printM("Calibration session ended, back to Startup (automatic zero runs)\n");
    }

    if (allSystemsInitialised_ && !calibrationService.gyroRunActive() &&
        !calibrationService.anyBusy() && !zeroing_.active()) {
      const Calibration::Validity v = currentCalibrationValidity();
      if (!v.allOk() && calibWarnLog_.due(Core::NowNs())) {
        char why[128];
        Calibration::describe(v, why, sizeof(why));
        Core::printW("WARNING calibration invalid, Arm and MissionBegin are refused (repeats every 10 s): %s\n", why);
      }
    }
  }

  void handleTelecommands() {
    if (!telecommandSubr.isDataNew())
      return; // No new telecommand data

    auto telecommand = telecommandSubr.getItem();

    switch (telecommand.type) {
    case TelecommandType::Telecommand_CalibStart:
      handleCalibStart(telecommand);
      break;

    case TelecommandType::Telecommand_CalibApply:
      handleCalibApply(telecommand);
      break;

    case TelecommandType::Telecommand_Save:
      handleCalibSave(telecommand);
      break;

    case TelecommandType::Telecommand_SystemReset:
      if (telecommand.paramInt == 0xC5) { // Validate.

        vehicleMode_ = VehicleMode::VehicleMode_Startup; // Reset the vehicle
        // mode to startup
        allSystemsInitialised_ = false; // Reset the system initialisation flag
        starshipTVC.enableActuators(false); // Disable actuators
        vehicleShutdownControl(true);       // Disable everything
        // The whole mission chain is reset to the state after boot; it is never continued (user decision
        // 2026-11-20). Simulation mode is NOT changed (user decision 2026-11-20).
        resetMissionsToBootState();
        clearFailures();
        // State reset only (user decision 2026-11-20): the estimators are NOT zeroed or re-referenced;
        // ZeroEstimators does that (a vehicle that was carried needs ZeroEstimators). Without an active zeroing
        // window the valid-zero record is kept and the vehicle returns to Ready without a new zero. An ACTIVE window
        // is aborted and the zero becomes invalid (the estimators hold a partial zero, review 2026-11-20); the
        // automatic 10 s zero then runs again in Startup before Ready.
        // Simulation mode only (review 2026-11-20, a documented sim/flight difference, overview.md "Simulation
        // mode"): the simulated body is the world, not an estimator. It is put back to its start pose (upright, at
        // the origin, at rest), so a run that ended tilted or away from the origin does not carry over into the
        // next one and re-trigger the out-of-bounds failures. In real mode nothing physical can be reset.
        if (simulationMode_) {
          bodySimulator.resetToStartPose();
          LOG_MSG("System reset: simulated body back to its start pose\n");
        }
        if (zeroing_.abort()) {
          LOG_MSG("Estimator zeroing aborted by system reset: zero invalid, auto zero runs again\n");
        }
        calibrationService.cancelGyroRun(); // the run belonged to the aborted window or countdown
        countdownGyroRun_ = false;
        // Sim flap faults do not carry over into the next run (review 2026-11-21). The simulated
        // wind and gusts DO persist across SystemReset on purpose (a run series in the same wind); they are cleared
        // when simulation mode is switched.
        if (starshipFlaps.simFaults().any())
          LOG_MSG("Sim fault: all flap faults cleared (system reset)\n");
        starshipFlaps.simFaults().clear();
        applyEstimatorZeroing();

        starshipTVC.enableMotors(false);      // Disable motors
        starshipFlaps.enableActuators(false); // Disable actuators
        vehicleArmed_ = false;                // Disarm the vehicle
        vehicleComms.motorInterlock().requireRearm(Safety::MotorInterlock::RearmReason::SystemReset); // D4-A

        LOG_MSG("System reset telecommand\n"); // Log the system reset
      }
      break;

    case TelecommandType::Telecommand_Arm:
      if (telecommand.paramInt == 0xA5) { // Validate.

        if (!zeroing_.armAllowed()) {
          if (zeroBlockedLog_.due(Core::NowNs()))
            LOG_MSG("Arm refused: estimator zeroing active\n");
          break;
        }

        // All three calibrations must be good to fly (in simulation too: the sensors and keys are real there).
        {
          const Calibration::Validity v = currentCalibrationValidity();
          if (Calibration::decideArm(v) != Calibration::GateRefusal::None) {
            if (calibArmLog_.due(Core::NowNs())) {
              char why[128];
              Calibration::describe(v, why, sizeof(why));
              Core::printW("Arm refused: calibration invalid (%s)\n", why);
            }
            break;
          }
        }

        vehicleArmed_ = true;

        LOG_MSG("System arm telecommand\n"); // Log the system arm
      } else {
        vehicleArmed_ =
            false; // Disarm the vehicle if the telecommand is not valid
        LOG_MSG("System disarm telecommand\n"); // Log the system disarm
      }
      break;

    case TelecommandType::Telecommand_MissionSelect:

      if (vehicleMode_ == VehicleMode::VehicleMode_Ready) {
        // missionSelection_ = telecommand.paramInt; // Get the mission
        // selection from the telecommand

        // mission_->resetMission(); // Reset the mission
        // mission_ = *missionList_[missionSelection_]; // Set the mission to
        // the selected mission mission_->resetMission(); // Reset the mission
        // controlRocket.subscribeSetpoint(mission_->set); // Subscribe to the
        // control topic of the mission

        LOG_MSG("Mission select telecommand\n"); // Log the mission select
      }
      break;

    case TelecommandType::Telecommand_MissionBegin:

      if (vehicleMode_ == VehicleMode::VehicleMode_Ready && !zeroing_.missionBeginAllowed()) {
        if (zeroBlockedLog_.due(Core::NowNs()))
          LOG_MSG("Mission begin refused: estimator zeroing active\n");
      } else if (vehicleMode_ == VehicleMode::VehicleMode_Ready && !missionBeginCalibrationOk(telecommand.paramInt)) {
        // refused: reason logged by missionBeginCalibrationOk
      } else if (vehicleMode_ == VehicleMode::VehicleMode_Ready && !startCountdownGyroRun(telecommand.paramInt)) {
        // refused: the countdown's gyro run could not start (reason logged)
      } else if (vehicleMode_ == VehicleMode::VehicleMode_Ready) {
        int64_t startTime =
            int64_t(telecommand.paramInt) *
            Core::SECONDS; // Get the start time from the telecommand

        missionSelection_ = 1;
        switchMissionTo(&missionWaypointTask,
                        startTime); // Switch to the selected mission
        missionChain_.begin();      // the configured chain may advance from now on

        vehicleMode_ =
            VehicleMode::VehicleMode_Running; // Vehicle is now running and
        // will do the mission

        LOG_MSG("Mission begin telecommand\n"); // Log the mission begin

        // The countdown's gyro bias run was started by startCountdownGyroRun above (before the mission switch).

        if (!simulationMode_) {
          starshipTVC.enableMotors(true); // Enable actuators
        } else {
          starshipTVC.enableMotors(
              false); // Disable actuators in simulation mode
        }
      }
      break;

    case TelecommandType::Telecommand_MissionAbort: {
      // User decision 2026-11-20: an abort means "stop and return home" (a failsafe is a latched Failure and is
      // not handled here). In flight: RTH hot start from any phase. On the ground = before liftoff, by mission mode
      // only (countdown, nothing running, RTH landed; no altitude/speed check): reset and stop, nothing restarts.
      // Decision: Safety::decideAbort (src/safety/mission_abort.hpp), docs/project/missions.md section 6.2.
      const auto pos = posEstSubr.getItem().data;
      Safety::AbortInput in;
      in.vehicleRunning = vehicleMode_ == VehicleMode::VehicleMode_Running;
      in.active = activeMissionKind();
      in.mode = modeIn(mission_->getMissionState().missionMode);
      const float altitude = pos(5); // logged only
      const float speed = sqrtf(pos(0) * pos(0) + pos(1) * pos(1) + pos(2) * pos(2));
      const Safety::AbortAction action = Safety::decideAbort(in);

      switch (action) {
      case Safety::AbortAction::StartRth:
        // The chain stays pending until RTH ends (RTH-end shutdown and auto-disarm as after a normal RTH). The
        // TVC set is enabled and reset by applyControlPhase() later in this tick.
        missionSelection_ = 0;
        switchMissionTo(&defaultMission_);
        break;
      case Safety::AbortAction::ResetAndStop:
        resetAndStopOnGround(); // shared with the calibration abort of the countdown
        break;
      case Safety::AbortAction::KeepRth:
      case Safety::AbortAction::Ignore:
        break;
      }

      LOG_MSG("Mission abort telecommand: %s (alt %.2f m, speed %.2f m/s)\n", Safety::toString(action),
              double(altitude), double(speed));
    } break;

    case TelecommandType::Telecommand_ZeroEstimators: {
      // One-shot ground action (safety.md section 2 rule 5): accepted only while initialised, disarmed, motors
      // off and on the ground (mode Startup or Ready). The window ends on the vehicle's own timer.
      Safety::ZeroRequestContext ctx;
      ctx.initialised = allSystemsInitialised_;
      ctx.armed = vehicleArmed_;
      ctx.motorsEnabled = starshipTVC.isMotorsEnabled();
      // On the ground: no mode in which the vehicle can fly, and no mission active (since 2026-11-20 a SystemReset
      // always returns the missions to Idle; the mission check stays as defence in depth).
      const MissionMode mm = mission_->getMissionState().missionMode;
      ctx.onGround = (vehicleMode_ == VehicleMode::VehicleMode_Startup ||
                      vehicleMode_ == VehicleMode::VehicleMode_Ready) &&
                     (mm == MissionMode::MissionMode_Idle || mm == MissionMode::MissionMode_Finished);
      const int64_t now = Core::NowNs();
      const auto result = zeroing_.request(telecommand.paramInt, telecommand.paramFloat, ctx, now);
      if (Safety::accepted(result)) {
        LOG_MSG("Estimator zeroing %s (%.1f s), keep the vehicle still\n", Safety::toString(result),
                double(zeroing_.endNs() - now) / Core::SECONDS);
        applyEstimatorZeroing();
        startGyroRunForWindow(zeroing_.endNs(), false);
      } else if (zeroRefusedLog_.due(now)) {
        LOG_MSG("ZeroEstimators refused: %s (%lu refused since boot)\n", Safety::toString(result),
                (unsigned long)zeroing_.refusedCount());
      }
      break;
    }

    case TelecommandType::Telecommand_SimulationMode:

      if (telecommand.paramInt == 0xA9) { // Validate.

        simulationMode_ = !simulationMode_; // Toggle simulation mode
        vehicleComms.motorInterlock().requireRearm(Safety::MotorInterlock::RearmReason::ModeChange); // D4-A
        if (starshipFlaps.simFaults().any())
          LOG_MSG("Sim fault: all flap faults cleared (simulation mode switched)\n");
        starshipFlaps.simFaults().clear(); // faults never survive a simulation mode switch

        if (simulationMode_) {
          starshipTVC.enableMotors(
              false); // Disable actuators in simulation mode

          positionTopicSwitch.subscribe(
              bodySimulator
                  .getStateEstTopic()); // Subscribe to the position topic of
          // the body simulator
          attitudeTopicSwitch.subscribe(
              bodySimulator
                  .getAttitudeEstTopic()); // Subscribe to the attitude topic
          // of the body simulator
          groundTopicSwitch.subscribe(
              bodySimulator
                  .getGroundTopic()); // Subscribe to the ground distance
          // topic of the body simulator

          bodySimulator.resetToStartPose(); // upright, at the origin, at rest
          bodySimulator.clearWind(); // every simulation starts without wind and gusts (SimWind*/SimGust* set them)

          bodySimulator.setPaused(
              false); // Unpause the body simulator to start the simulation
        } else {
          positionTopicSwitch.subscribe(
              posEstTask
                  .getStateEstTopic()); // Subscribe to the position topic of
          // the position estimator
          attitudeTopicSwitch.subscribe(
              imuTask.getAttitudeEstTopic()); // Subscribe to the attitude topic
          // of the attitude estimator
          groundTopicSwitch.subscribe(
              posEstTask
                  .getGroundDistanceEstTopic()); // Subscribe to the ground
          // distance topic of the
          // position estimator

          bodySimulator.setPaused(true);
          bodySimulator.clearWind();

          // Reset the system to normal mode
          vehicleMode_ =
              VehicleMode::VehicleMode_Startup; // Reset the vehicle mode to
          // startup
          allSystemsInitialised_ =
              false; // Reset the system initialisation flag
          starshipTVC.enableActuators(false); // Disable actuators
          vehicleShutdownControl(true);       // Disable everything
        }
      }
      break;

    case TelecommandType::Telecommand_SimWindSpeed:
    case TelecommandType::Telecommand_SimWindDirection: {
      // Simulation-only world parameter (owner 2026-11-21): steady wind of the body simulator. Refused unless in
      // simulation mode; it changes nothing but the simulated air (overview.md "Simulation mode").
      constexpr int16_t kSimWindMagic = 0x57;
      constexpr float kSimWindMaxMps = 20.0f;
      const bool isSpeed = telecommand.type == TelecommandType::Telecommand_SimWindSpeed;
      const float value = telecommand.paramFloat;
      if (telecommand.paramInt != kSimWindMagic || !simulationMode_ || !isfinite(value)) {
        LOG_MSG("Sim wind %s refused (%s)\n", isSpeed ? "speed" : "direction",
                !simulationMode_ ? "not in simulation mode"
                                 : (telecommand.paramInt != kSimWindMagic ? "bad magic" : "value not finite"));
        break;
      }
      if (isSpeed)
        bodySimulator.setWind(value < 0.0f ? 0.0f : (value > kSimWindMaxMps ? kSimWindMaxMps : value), NAN);
      else
        bodySimulator.setWind(NAN, atan2f(sinf(value), cosf(value)));
      LOG_MSG("Sim wind: %.2f m/s toward %.1f deg (CCW from North)\n", double(bodySimulator.windSpeed()),
              double(bodySimulator.windDirection() * 180.0f / float(M_PI)));
    } break;

    case TelecommandType::Telecommand_SimGustStrength:
    case TelecommandType::Telecommand_SimGustPeriod:
    case TelecommandType::Telecommand_SimGustSeed: {
      // Simulation-only gust settings (owner-approved exception to safety.md rule 7, 2026-11-21). Same refusal
      // rules as the wind telecommands; the gust series is deterministic (BodySimParams "Gusts").
      constexpr int16_t kSimWindMagic = 0x57;
      const float value = telecommand.paramFloat;
      if (telecommand.paramInt != kSimWindMagic || !simulationMode_ || !isfinite(value)) {
        LOG_MSG("Sim gust setting %d refused (%s)\n", int(telecommand.type),
                !simulationMode_ ? "not in simulation mode"
                                 : (telecommand.paramInt != kSimWindMagic ? "bad magic" : "value not finite"));
        break;
      }
      float strength = bodySimulator.gustStrength(), period = bodySimulator.gustPeriod();
      uint32_t seed = bodySimulator.gustSeed();
      if (telecommand.type == TelecommandType::Telecommand_SimGustStrength)
        strength = value; // clamped 0..10 in the model
      else if (telecommand.type == TelecommandType::Telecommand_SimGustPeriod)
        period = value; // clamped 0.5..30 in the model
      else
        seed = uint32_t(value < 0.0f ? 0.0f : (value > 16777215.0f ? 16777215.0f : value + 0.5f));
      bodySimulator.setGust(strength, period, seed);
      LOG_MSG("Sim gusts: strength %.2f m/s, period %.2f s, seed %u\n", double(bodySimulator.gustStrength()),
              double(bodySimulator.gustPeriod()), unsigned(bodySimulator.gustSeed()));
    } break;

    case TelecommandType::Telecommand_SimFlapStuck:
    case TelecommandType::Telecommand_SimFlapSlow:
    case TelecommandType::Telecommand_SimFaultClear: {
      // Simulation-only flap servo faults (owner-approved exception to safety.md rules 7 and 9, 2026-11-21). The
      // decision is the pure Safety::decideSimFault; the fault only alters the modelled true flap angle inside
      // StarshipFlaps and only while simulation mode is active (checked again there at apply time).
      const Safety::SimFaultDecision d = Safety::decideSimFault(uint8_t(telecommand.type), uint8_t(telecommand.subsystem),
                                                                telecommand.paramInt, telecommand.paramFloat,
                                                                simulationMode_);
      if (d.action == Safety::SimFaultAction::Refuse) {
        LOG_MSG("Sim fault %d refused (%s)\n", int(telecommand.type), Safety::simFaultRefusalText(d.reason));
        break;
      }
      if (!starshipFlaps.simFaults().set(d, simulationMode_)) {
        LOG_MSG("Sim fault %d refused (not applied)\n", int(telecommand.type));
        break;
      }
      static const char *const kFlapNames[4] = {"TL", "TR", "BL", "BR"};
      if (d.action == Safety::SimFaultAction::Clear) {
        LOG_MSG("Sim fault: all flap faults cleared\n");
      } else if (d.action == Safety::SimFaultAction::Slow) {
        LOG_MSG("Sim fault: flap %s servo rate factor %.2f\n", kFlapNames[d.flap], double(d.value));
      } else if (d.captureCurrent) {
        LOG_MSG("Sim fault: flap %s STUCK at its current true angle (now)\n", kFlapNames[d.flap]);
      } else {
        LOG_MSG("Sim fault: flap %s STUCK at %.1f deg (now)\n", kFlapNames[d.flap],
                double(d.value * 180.0f / float(M_PI)));
      }
    } break;

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

    // Stay in Startup: vehicleIsReady() moves to Ready only with a valid estimator zero since boot (the
    // automatic boot zero starts now if there is none).
    allSystemsInitialised_ = true; // Set all systems initialised to true

    return true;
  }

  void checkForFailures() {
    // Control failsafe (T9b-3, the ONE rule, safety.md 4.1): while committed (interlock open, mission pending or
    // Running) the accepted RC_STATE of the latched control source is older than 2 s (or there is no source) =>
    // failsafe, latched in mode Failure until SystemReset. Not committed (on the ground, disarmed, nothing pending)
    // losing every source only sets ctrl_source NONE (D2-B). General link health, heartbeats, GS REQUESTs and
    // telecommands never keep it alive and never trigger it; the ground station on USB is never a failure
    // condition. No simulation dependence.
    if (vehicleComms.controlSourceLost(isCommitted(Core::NowNs()))) {
      if (!failureState_.controlSourceLost) LOG_MSG("Failsafe: control source lost (no accepted RC_STATE for 2 s)\n");
      vehicleMode_ = VehicleMode::VehicleMode_Failure;
      failureState_.controlSourceLost = true;
    }

    // Motor-interlock watchdog (T9b-2): the 100 Hz task that closes the motor outputs has not run for 100 ms.
    // Failsafe (mode Failure latches until SystemReset); the motors are already cut by enableMotors(false) there.
    if (Safety::interlockWatchdogStale(Core::NowNs(), motorInterlockWatchdog.lastRunNs())) {
      vehicleMode_ = VehicleMode::VehicleMode_Failure;
      failureState_.interlockWatchdog = true;
    }

    // Check sensor for data timeout which indicates a failure

    if (Core::NowNs() - gnssSubr.getItem().timestamp > DATA_TIMEOUT) {
      vehicleMode_ = VehicleMode::VehicleMode_Failure;
      failureState_.sensorFailure = true;
      sensoryState_.gnss = TelemetrySensor::TelemetrySensor_Failure;
    } else if (sensoryState_.gnss != TelemetrySensor::TelemetrySensor_Calib &&
               sensoryState_.gnss != TelemetrySensor::TelemetrySensor_Failure) {
      sensoryState_.gnss = TelemetrySensor::TelemetrySensor_Ready;
    }

    if (Core::NowNs() - gyroSubr.getItem().timestamp > DATA_TIMEOUT) {
      vehicleMode_ = VehicleMode::VehicleMode_Failure;
      failureState_.sensorFailure = true;
      sensoryState_.imu = TelemetrySensor::TelemetrySensor_Failure;
    } else if (sensoryState_.imu != TelemetrySensor::TelemetrySensor_Calib &&
               sensoryState_.imu != TelemetrySensor::TelemetrySensor_Failure) {
      sensoryState_.imu = TelemetrySensor::TelemetrySensor_Ready;
    }

    if (Core::NowNs() - magSubr.getItem().timestamp > DATA_TIMEOUT * 5 &&
        sensoryState_.mag != TelemetrySensor::TelemetrySensor_Failure) {
      // vehicleMode_ = VehicleMode::VehicleMode_Failure;
      // failureState_.sensorFailure = true;
      if (vehicleMode_ == VehicleMode::VehicleMode_Running &&
          mission_->getMissionState().missionMode ==
              MissionMode::MissionMode_Running) {
        switchMissionTo(&defaultMission_); // Switch to the default mission
        LOG_MSG("Magnetometer data timeout. Switching mission to default!\n");
      }
      sensoryState_.mag = TelemetrySensor::TelemetrySensor_Failure;
    } else if (sensoryState_.mag != TelemetrySensor::TelemetrySensor_Calib &&
               sensoryState_.mag != TelemetrySensor::TelemetrySensor_Failure) {
      sensoryState_.mag = TelemetrySensor::TelemetrySensor_Ready;
    }

    if (Core::NowNs() - baroSubr.getItem().timestamp > DATA_TIMEOUT) {
      vehicleMode_ = VehicleMode::VehicleMode_Failure;
      failureState_.sensorFailure = true;
      sensoryState_.baro = TelemetrySensor::TelemetrySensor_Failure;
    } else if (sensoryState_.baro != TelemetrySensor::TelemetrySensor_Calib &&
               sensoryState_.baro != TelemetrySensor::TelemetrySensor_Failure) {
      sensoryState_.baro = TelemetrySensor::TelemetrySensor_Ready;
    }

    if (Core::NowNs() - posEstSubr.getItem().timestamp > DATA_TIMEOUT) {
      vehicleMode_ = VehicleMode::VehicleMode_Failure;
      failureState_.sensorFailure = true;
      sensoryState_.positionKF = TelemetrySensor::TelemetrySensor_Failure;
    } else if (sensoryState_.positionKF !=
                   TelemetrySensor::TelemetrySensor_Calib &&
               sensoryState_.positionKF !=
                   TelemetrySensor::TelemetrySensor_Failure) {
      sensoryState_.positionKF = TelemetrySensor::TelemetrySensor_Ready;
    }

    if (Core::NowNs() - attEstSubr.getItem().timestamp > DATA_TIMEOUT) {
      vehicleMode_ = VehicleMode::VehicleMode_Failure;
      failureState_.sensorFailure = true;
      sensoryState_.attitudeKF = TelemetrySensor::TelemetrySensor_Failure;
    } else if (sensoryState_.attitudeKF !=
                   TelemetrySensor::TelemetrySensor_Calib &&
               sensoryState_.attitudeKF !=
                   TelemetrySensor::TelemetrySensor_Failure) {
      sensoryState_.attitudeKF = TelemetrySensor::TelemetrySensor_Ready;
    }

    // Check if any of the data is out of bounds which indicates a safety issue
    auto missionMode = mission_->getMissionState().missionMode;
    if (!mission_->getDisableKinematicSafety() &&
        missionMode != MissionMode::MissionMode_Idle) {
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

        if (tilt > ANGLE_OUTOFBOUNDS_STATIONARY &&
            vehicleMode_ != VehicleMode::VehicleMode_SensorCalib) {
          failureState_.attitudeOutOfBounds = true;
          vehicleMode_ = VehicleMode::VehicleMode_Failure;
        }
      } else {
        if (startDistance > POSITION_OUTOFBOUNDS_FLIGHT) {
          failureState_.positionOutOfBounds = true;
          vehicleMode_ = VehicleMode::VehicleMode_Failure;
        }

        if (tilt > ANGLE_OUTOFBOUNDS_FLIGHT &&
            vehicleMode_ != VehicleMode::VehicleMode_SensorCalib) {
          failureState_.attitudeOutOfBounds = true;
          vehicleMode_ = VehicleMode::VehicleMode_Failure;
        }
      }
    }

    // Auto disarm vehicle once landed.
    if (vehicleArmed_ && missionMode == MissionMode::MissionMode_Finished &&
        (mission_ == &defaultMission_ || mission_->nextMission() == nullptr)) {
      vehicleArmed_ =
          false; // Disarm the vehicle if the default mission is finished
      LOG_MSG("Vehicle auto disarmed after RTH mission finished.\n"); // Log the
      // vehicle
      // auto
      // disarm
    }
  }

  bool vehicleIsReady() {
    if (allSystemsInitialised_ &&
        sensoryState_.imu == TelemetrySensor::TelemetrySensor_Ready &&
        sensoryState_.baro == TelemetrySensor::TelemetrySensor_Ready &&
        sensoryState_.gnss == TelemetrySensor::TelemetrySensor_Ready &&
        sensoryState_.attitudeKF == TelemetrySensor::TelemetrySensor_Ready &&
        sensoryState_.positionKF == TelemetrySensor::TelemetrySensor_Ready &&
        !failureState_.sensorFailure && !failureState_.attitudeFailure &&
        !failureState_.positionFailure && !failureState_.positionOutOfBounds &&
        !failureState_.attitudeOutOfBounds &&
        vehicleMode_ != VehicleMode::VehicleMode_Failure &&
        vehicleMode_ != VehicleMode::VehicleMode_SensorCalib) {
      if (vehicleMode_ == VehicleMode::VehicleMode_Startup) {
        // Ready requires a valid estimator zero since boot and no active zeroing window.
        if (!zeroing_.readyAllowed())
          return false;
        vehicleMode_ = VehicleMode::VehicleMode_Ready;
      }

      // vehicleMode_ = VehicleMode::VehicleMode_Ready; // Set the vehicle mode
      // to ready

      // vehicleMode_ = VehicleMode::VehicleMode_Ready; // Set the vehicle mode
      // to ready
      return true;
    }

    return false;
  }

  void vehicleShutdownControl(bool shutdown) {
    if (shutdown) {
      starshipTVC.enableActuators(false);
      starshipFlaps.enableActuators(false);
      bodySimulator.enableTVC(false);
      return;
    }

    if (mission_ == &missionFreefallTask) {
      starshipFlaps.enableActuators(true);
    } else
      starshipFlaps.enableActuators(mission_->getActuatorsEnabled());

    starshipTVC.enableActuators(mission_->getActuatorsEnabled());
    bodySimulator.enableTVC(mission_->getActuatorsEnabled());
    bodySimulator.enableFlaps(starshipFlaps.actuatorsEnabled());
  }

  void vehicleArmingControl() {
    if (vehicleMode_ == VehicleMode::VehicleMode_Failure) {
      vehicleArmed_ =
          false; // Disarm the vehicle if the vehicle is in failure mode
      if (disarmLogDue(1))
        LOG_MSG("Vehicle disarmed due to failure mode.\n");
    } else if (vehicleMode_ == VehicleMode::VehicleMode_SensorCalib) {
      vehicleArmed_ = false; // Disarm the vehicle if the vehicle is in sensor
      // calibration mode
      if (disarmLogDue(2))
        LOG_MSG("Vehicle disarmed due to sensor calibration.\n");
    } else if (vehicleMode_ == VehicleMode::VehicleMode_Startup) {
      vehicleArmed_ =
          false; // Disarm the vehicle if the vehicle is in startup mode
      if (disarmLogDue(3))
        LOG_MSG("Vehicle disarmed due to startup mode.\n");
    } else if (vehicleMode_ == VehicleMode::VehicleMode_SensorInit) {
      vehicleArmed_ =
          false; // Disarm the vehicle if the vehicle is in startup mode
      if (disarmLogDue(4))
        LOG_MSG("Vehicle disarmed due to sensor initialisation.\n");
    } else {
      lastDisarmReason_ = 0; // next auto-disarm is logged immediately
    }

    // Invariant "armed implies calibrations good" (also in simulation), enforced on the ground only: nothing can
    // make a calibration invalid in flight (runs and sessions need a disarmed vehicle on the ground).
    if (vehicleArmed_ && vehicleMode_ == VehicleMode::VehicleMode_Ready) {
      const Calibration::Validity v = currentCalibrationValidity();
      if (Calibration::mustDisarmInReady(vehicleArmed_, true, v)) {
        vehicleArmed_ = false;
        if (disarmLogDue(5)) {
          char why[128];
          Calibration::describe(v, why, sizeof(why));
          Core::printW("Vehicle disarmed: calibration invalid (%s)\n", why);
        }
      }
    }

    // LOG_MSG("Vehicle armed: %d\n", vehicleArmed_); // Log the vehicle armed
    // state

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
    failureState_.controlSourceLost = false;
    failureState_.interlockWatchdog = false;
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

class FlapModeObserverTask : public Core::Task_Periodic {
private:
  CTRL::ControlAttitudeBellyFlopSetting
      starshipFlopSetting; // Flap controller setting.

  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>>
      posEstSubr; // Subscriber for the position estimator

public:
  FlapModeObserverTask()
      : Task_Periodic("Flap Mode Observer", 0.1 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
    // setPriority(500);
  }

  void taskInit() override {
    posEstSubr.subscribe(
        positionTopicSwitch
            .getTopic()); // Subscribe to the position estimator topic
  }

  void taskThread() override {
    auto &vehicleMode = vehicleSafetyAndControlTask.getVehicleMode();
    auto mission = vehicleSafetyAndControlTask.getCurrentMission();

    auto posEst = posEstSubr.getItem().data.block<3, 1>(
        3, 0); // Get the position estimate
    auto velEst = posEstSubr.getItem().data.block<3, 1>(
        0, 0); // Get the velocity estimate

    if (vehicleMode == VehicleMode::VehicleMode_Running &&
        mission->getMissionState().missionMode ==
            MissionMode::MissionMode_Running) {
      // Bellyflop controller set (flap attitude control) in freefall and after the bellyflop kick: the phase table
      // of VehicleSafetyAndControlTask (src/safety/control_phase.hpp). Before 2026-11-20 freefall only.
      if (vehicleSafetyAndControlTask.bellyflopFlapsActive()) {
        starshipFlopSetting.bellyFlopMode =
            CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::
                BellyFlopMode_Stabilize;
        // starshipFlopSetting.azimuthAngle_Rad = 180 * DEGREES;
        starshipFlopSetting.pitchAngle_Rad = 0 * DEGREES;
        auto azimuthTowardsZero =
            atan2(posEst(1), -posEst(0)); // Azimuth towards (0,0)
        auto distanceToZero =
            sqrt(posEst(0) * posEst(0) + posEst(1) * posEst(1));
        if (posEst(2) > 100) {
          auto angle = -(distanceToZero - 1) * 1;
          if (angle > 0)
            angle = 0;
          else if (angle < -40)
            angle = -40;
          starshipFlopSetting.azimuthAngle_Rad = -azimuthTowardsZero;
          starshipFlopSetting.pitchAngle_Rad = (angle + 20) * DEGREES;
        } else if (posEst(2) < missionFreefallTask.getStopAltitude() + 20) {
          starshipFlopSetting.pitchAngle_Rad = 30 * DEGREES;
        }
      } else if (mission == &defaultMission_ && defaultMission_.stabilising()) {
        starshipFlopSetting.bellyFlopMode =
            CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::
                BellyFlopMode_Upright; // Enable the belly flop mode
      } else if (mission == &missionWaypointTask &&
                 defaultMission_.stabilising()) {
        starshipFlopSetting.bellyFlopMode =
            CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::
                BellyFlopMode_Acsent; // Disable the belly flop mode
        starshipFlopSetting.extentFlapsOnAscent = EXTEND_FLAPS_ASCENT;
      } else {
        starshipFlopSetting.bellyFlopMode =
            CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::
                BellyFlopMode_Acsent; // Disable the belly flop mode
        starshipFlopSetting.extentFlapsOnAscent = false;
      }
    }

    bellyFlopControlTopic.publish(
        starshipFlopSetting); // Publish the flap controller setting to the
    // control system
  }
};
FlapModeObserverTask flapModeObserverTask;

/**
 * Samples the vehicle state for the telemetry task (VehicleComms, src/comms/). Runs in the telemetry task's
 * context only: it reads latest-value subscribers and getters, and never changes vehicle state.
 */
class TelemetryProvider : public Comms::TelemetryProviderI {
private:
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 7>>> attSubr_;
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 6>>> posSubr_;
  Core::Simple_Subscriber<Core::Timestamped<Math::Vector<float, 2>>>
      groundSubr_;
  Core::Simple_Subscriber<Core::Timestamped<SNSR::GNSSData>> gnssSubr_;
  Core::Simple_Subscriber<CTRL::ControlAttitudeFlapSetting> flapSubr_;
  Core::Simple_Subscriber<Math::Vector<float, 4>> tvcSubr_;
  // Read-only copies of the attitude controllers' setpoint inputs (telemetry setpoint_q). Extra subscribers on
  // the same topics; the controllers' own subscriptions are not touched.
  Core::Simple_Subscriber<Math::Vector<float, 7>> tvcAttSetpointSubr_;
  Core::Simple_Subscriber<CTRL::ControlAttitudeBellyFlopSetting> flopSettingSubr_;
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> gyroSubr_;
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> accSubr_;
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> magSubr_;
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 1>>> baroSubr_;
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 1>>>
      thermoSubr_;

public:
  void begin() override {
    attSubr_.subscribe(attitudeTopicSwitch.getTopic());
    posSubr_.subscribe(positionTopicSwitch.getTopic());
    groundSubr_.subscribe(groundTopicSwitch.getTopic());
    gnssSubr_.subscribe(gnss.getGNSSTopic());
    flapSubr_.subscribe(trueFlapSettingTopic);
    tvcSubr_.subscribe(starshipTVC.getTVCActualThrustTopic());
    tvcAttSetpointSubr_.subscribe(controlMappingAccToAtt.getAttitudeTopic());
    flopSettingSubr_.subscribe(bellyFlopControlTopic);
    gyroSubr_.subscribe(mpuDriver.getGyroTopic());
    accSubr_.subscribe(mpuDriver.getAccelTopic());
    thermoSubr_.subscribe(mpuDriver.getThermoTopic());
    magSubr_.subscribe(qmc.getMagTopic());
    baroSubr_.subscribe(bme.getBaroTopic());
  }

  void fillSnapshot(Comms::TelemetrySnapshot &s) override {
    auto &vt = vehicleSafetyAndControlTask;
    s.mode = vt.getVehicleMode();
    s.sensors = vt.getSensoryState();
    s.failures = vt.getFailureState();
    s.armed = vt.isVehicleArmed();
    s.simulation = vt.simulationModeEnabled();
    // MOTORS_ENABLED (T9b-2): the hard gate is open AND the vehicle logic enabled motors and actuators.
    s.motorsEnabled = starshipTVC.isMotorsEnabled() && starshipTVC.isActuatorsEnabled() &&
                      starshipTVC.interlockOpen(Core::NowNs());

    const MissionState &ms = vt.getCurrentMission()->getMissionState();
    s.missionMode = ms.missionMode;
    s.missionPhase = vt.getCurrentMissionPhase();
    s.missionIndex = ms.missionIndex;
    for (int i = 0; i < 6; i++)
      s.setpoint[i] = ms.positionSetpoint[i];
    s.missionTime_ns = ms.missionTime;
    s.waypointIndex = vt.getCurrentMission() == &missionWaypointTask
                          ? static_cast<uint8_t>(missionWaypointTask.currentWaypointIndex())
                          : uint8_t(0xFF);
    s.zeroing = vt.estimatorZeroingActive();
    vt.fillTelemetryCache(s);

    const auto &att = attSubr_.getItem().data;
    for (int i = 0; i < 7; i++)
      s.att[i] = att(i);
    const auto &pos = posSubr_.getItem().data;
    for (int i = 0; i < 6; i++)
      s.pos[i] = pos(i);

    // Attitude setpoint of the active attitude controller (src/comms/telemetry_mapping.hpp attitudeSetpoint).
    {
      Comms::AttitudeSetpointInputs in;
      in.tvcEnabled = vt.tvcControlActive();
      const auto &flop = flopSettingSubr_.getItem();
      in.flapsStabilize =
          vt.bellyflopFlapsActive() &&
          flop.bellyFlopMode == CTRL::ControlAttitudeBellyFlopSetting::BellyFlopMode::BellyFlopMode_Stabilize;
      in.flapAzimuth = flop.azimuthAngle_Rad;
      in.flapPitch = flop.pitchAngle_Rad;
      const auto &tq = tvcAttSetpointSubr_.getItem();
      for (int i = 0; i < 4; i++) {
        in.tvcQ[i] = tq(3 + i);
        in.qIs[i] = s.att[3 + i];
      }
      // Bellyflop kick: the mission writes the TVC setpoint directly (not via the mapping topic).
      in.tvcOverride = vt.getCurrentMission() == &missionBellyflop && missionBellyflop.kickActive();
      if (in.tvcOverride)
        missionBellyflop.kickSetpoint(in.tvcOverrideQ);
      Comms::attitudeSetpoint(in, s.setpointQ);
    }

    const auto &ground = groundSubr_.getItem();
    s.groundDistance = ground.timestamp != 0 ? ground.data(1) : NAN;

    const auto &flaps = flapSubr_.getItem();
    s.flaps[0] = flaps.flapTLAngle_Rad;
    s.flaps[1] = flaps.flapTRAngle_Rad;
    s.flaps[2] = flaps.flapBLAngle_Rad;
    s.flaps[3] = flaps.flapBRAngle_Rad;
    const auto &tvc = tvcSubr_.getItem();
    for (int i = 0; i < 4; i++)
      s.tvcThrust[i] = tvc(i);

    const auto &g = gnssSubr_.getItem().data;
    s.latitude = g.position(0);
    s.longitude = g.position(1);
    s.altitude = g.position(2);
    for (int i = 0; i < 3; i++)
      s.gnssVel[i] = g.velocity(i);
    s.satellites = g.numSats;
    s.gnssValid = g.positionValid;
    s.gnssHaveData = gnssSubr_.getItem().timestamp != 0;
    s.gnssFixType = gnss.fixType();
    s.hAcc = g.positionCov(0);
    s.vAcc = g.positionCov(2);
    s.sAcc = g.velocityCov(0);

    const auto &gyro = gyroSubr_.getItem().data.val;
    const auto &acc = accSubr_.getItem().data.val;
    const auto &mag = magSubr_.getItem().data.val;
    for (int i = 0; i < 3; i++) {
      s.gyro[i] = gyro(i);
      s.accel[i] = acc(i);
      s.mag[i] = mag(i) * 1e-4f; // QMC5883 publishes gauss
    }
    s.gyroCal = gyroCalApplied;
    s.accelCal = accCalApplied;
    s.magCal = magCalApplied;
    const auto &xa = imuTask.getState(); // [gyro bias(3), q(4)]
    for (int i = 0; i < 3; i++)
      s.gyroBiasEst[i] = xa(i);
    s.pressure = baroSubr_.getItem().data.val(0);
    s.temperature = thermoSubr_.getItem().data.val(0) + 273.15f; // degC -> K

    // Estimator standard deviations (the real estimators, also in simulation).
    // Attitude: the EKF's own q and its quaternion covariance block (state
    // [gyro bias(3), q(4)]), projected onto rotations about the world axes.
    const auto &pa = imuTask.getCovariance();
    float qEkf[4], pq[16];
    for (int r = 0; r < 4; r++) {
      qEkf[r] = xa(3 + r);
      for (int c = 0; c < 4; c++)
        pq[4 * r + c] = pa(3 + r, 3 + c);
    }
    Comms::attitudeStdWorld(qEkf, pq, s.attStd);
    const auto &pp = posEstTask.getCovariance();
    for (int i = 0; i < 3; i++) {
      float vv = pp(i, i), vp = pp(3 + i, 3 + i);
      s.velStd[i] = vv > 0 ? sqrtf(vv) : 0.0f;
      s.posStd[i] = vp > 0 ? sqrtf(vp) : 0.0f;
    }
  }
};
TelemetryProvider telemetryProvider;

// Network stack, links and telemetry task. Its buffers (about 6 KB) live in
// RAM2 (DMAMEM). It is constructed like every other global (constructors run
// for DMAMEM objects too).
DMAMEM Comms::VehicleComms vehicleComms(Serial, RX_MODULE_SERIALPORT,
                                        telemetryProvider, telecommandTopic);

class DisplayTask : public Core::Task_Periodic {
public:
  static constexpr int SCREEN_WIDTH = 128;    // OLED display width, in pixels
  static constexpr int SCREEN_HEIGHT = 32;    // OLED display height, in pixels
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

  DisplayTask() : Task_Periodic("Display task", 0.1 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
    // setPriority(500);
  }

  void taskInit() override {
    auto ret = display_.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    if (!ret) {
      Core::printM("Display init failed!\n");
      Core::getSystemScheduler().removeTask(*this);
      return;
    }
    display_.setRotation(0);
    // display_.clearDisplay();
    display_.display();

    attSub.subscribe(attitudeTopicSwitch.getTopic());
    accSub.subscribe(positionTopicSwitch.getTopic());

    accelSub.subscribe(accTransformTopic.getOutputTopic());

    gnssSub.subscribe(gnss.getGNSSTopic());

    gyroSub.subscribe(gyroTransformTopic.getOutputTopic());
    biasSub.subscribe(imuTask.getBiasEstTopic());

    // magSub.subscribe(qmc.getMagTopic());
  }

  void taskThread() override {
    /*if (magSub.isDataNew()) {

        auto mag = magSub.getItem().data.val;

        for (size_t i = 0; i < 3; i++)
        {
            if (mag[i][0] < magMin[i][0]) magMin[i][0] = mag[i][0];
            if (mag[i][0] > magMax[i][0]) magMax[i][0] = mag[i][0];
        }

        Core::printM("Time: %f, Mag: ", Core::NowS());
        mag.printTo(Core::printM);
        Core::printM("\n");

        Core::printM("Mag min: ");
        magMin.printTo(Core::printM);
        Core::printM("\n");

        Core::printM("Mag max: ");
        magMax.printTo(Core::printM);
        Core::printM("\n");

    }*/

    // magSub.getStandardDeviation().printTo(Core::printM);

    // imuTask.getState().printTo(Core::printM);
    // imuTask.getCovariance().printTo(Core::printM);

    /*if (Serial.available()) {

        Serial.clear();
        //Serial.flush();
        posEstTask.setState(DSP::ValueCov<float, 6>(0, 1));
        imuTask.setState(DSP::ValueCov<float, 7>({0, 0, 0, 1, 0, 0, 0}, 1));

    }*/

    /*if (accSub.isDataNew()) {

        auto accData = accSub.getItem();

        Core::printM("New Data: t: %.2f\n",
    double(accData.timestamp)/Core::SECONDS);
        accData.data.val.printTo(Core::printM);
        accData.data.cov.printTo(Core::printM);

    }*/

    // return;
    /*Core::Timestamped<SNSR::GNSSData> dummyGNSSData;
    dummyGNSSData.timestamp = Core::NowNs();
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

      auto downRot = attQuat.rotate(Math::Vector_F({0, 0, -1}));
      angle = Math::Vector_F({0, 0, -1}).getAngleTo(downRot);

      // calulate the compass heading
      auto northRot = attQuat.rotate(Math::Vector_F({1, 0, 0}));
      north = atan2(northRot(1), northRot(0));

      /*if (angle > 80*3.14/180) {
          //gnssRef = gnssSub.getItem().data.position;
          auto newRef = gnssSub.getItem().data.position;
          //newRef(2) = accSub.getItem().data.val(5);
          posEstTask.setPositionReference(newRef);
          //posEstTask.setState(DSP::ValueCov<float, 6>(0, 1));
          //calulate the the new attitude with the same tilt but the new heading
      as zero
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

        Core::printM("New Data: t: %.2f\n",
    double(accData.timestamp)/Core::SECONDS);
        attData.data.val.printTo(Core::printM);
        accData.data.val.printTo(Core::printM);

    }*/

    if (accSub.isDataNew() || true) {
      // A radio packet really seen within 2 s: general link health, information only (display, counter).
      bool connected = vehicleComms.isRadioPacketRecent();
      if (!lastConnectionState && connected) {
        connectionCounter++;
      }
      lastConnectionState = connected;

      auto accData = accSub.getItem();
      auto attData = attSub.getItem();
      auto biasData = biasSub.getItem();

      display_.clearDisplay();

      display_.setTextSize(1);
      display_.setTextColor(SSD1306_WHITE);
      display_.setTextWrap(false);

      display_.setCursor(0, 0);
      // display_.printf("Vel:%.2f,%.2f,%.2f\n", accData.data.val(0),
      // accData.data.val(1), accData.data.val(2)); display_.setCursor(0, 9);
      // display_.printf("Pos:%.2f,%.2f,%.2f\n", accData.data.val(3),
      // accData.data.val(4), accData.data.val(5)); display_.setCursor(0, 18);
      display_.printf("X:%.2f\t%.2f\n", accData.data(0), accData.data(3));
      display_.printf("Y:%.2f\t%.2f | %d\n", min(accData.data(1), 100),
                      min(accData.data(4), 100), connectionCounter);
      display_.printf("Z:%.2f\t%.2f | %s\n", min(accData.data(2), 100),
                      min(accData.data(5), 100), connected ? "OK" : "");
      // display_.printf("w:%.2f,x:%.2f,y:%.2f,z:%.2f\n", attData.data(3),
      // attData.data(4), attData.data(5), attData.data(6));
      display_.printf("S:%d,P:%.1f,A:%.0f,N:%.0f\n",
                      gnssSub.getItem().data.numSats,
                      min(gnssSub.getItem().data.positionCov(0), 99),
                      min(angle * 180 / 3.14, 99), north * 180 / 3.14);
      // display_.printf("S:%d,HA:%.2f,VA:%.2f\n", gnssData.data.numSats,
      // gnssData.data.positionCov(0, 0), gnssData.data.positionCov(2, 2));
      // display_.setCursor(0, 9);
      // display_.printf("Lat: %.10f, \nLon: %.10f\nAlt:%.2f, T:%.2f",
      // gnssData.data.latitude*180/3.14, gnssData.data.longitude*180/3.14,
      // gnssData.data.altitude, Core::NowS()/60);

      display_.display();

      // posEstTask.getCovariance().printTo(Core::printM);

      // LOG_MSG("Bias Est: \tX:%.2f\tY:%.2f\tZ:%.2f\t
      // GyroX:%.2f\tY:%.2f\tZ:%.2f\n", biasData.data(0) * RAD_TO_DEG,
      // biasData.data(1) * RAD_TO_DEG, biasData.data(2) * RAD_TO_DEG,
      // gyroBias(0) * RAD_TO_DEG, gyroBias(1) * RAD_TO_DEG, gyroBias(2) *
      // RAD_TO_DEG); LOG_MSG("Bias:     \tX:%.2f\tY:%.2f\tZ:%.2f\n",
      // gyroBias(0) * RAD_TO_DEG, gyroBias(1) * RAD_TO_DEG, gyroBias(2) *
      // RAD_TO_DEG); LOG_MSG("Attitude: W:%.2f, X:%.2f, Y:%.2f, Z:%.2f\n",
      // attData.data(3), attData.data(4), attData.data(5), attData.data(6));
    }

    if (Core::NowNs() > 5 * Core::SECONDS) {
    }

    //(magTransform * (magSub.getItem().data.val -
    // magBias)).printTo(Core::printM);
    // magSub.getItem().data.val.printTo(Core::printM);
  }
};
DisplayTask displayTask;

class CalibrationTesting : public Core::Task_Periodic {
public:
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> accelSub;
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> gyroSub;
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 3>>> magSub;

  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 1>>>
      imuTempSub;
  Core::Simple_Subscriber<Core::Timestamped<DSP::ValueCov<float, 1>>> baroSub;

  Math::Vector<float, 3> magMax, magMin;

  Math::Vector<float, 3> gyroBias;
  Math::Vector<float, 3> accelVal;

  Core::ListBuffer<float, 500> gyroBiasBuffer;

  bool firstAcc = true;

  CalibrationTesting()
      : Task_Periodic("Calibration task", 0.01 * Core::SECONDS) {
    Core::getSystemScheduler().addTask(*this);
    // setPriority(500);
  }

  void taskInit() override {
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

    LOG_MSG("Gyro Bias:  X:%.3f Y:%.3f Z:%.3f, Temp: %.2f\n",
            gyroBias(0) * RAD_TO_DEG, gyroBias(1) * RAD_TO_DEG,
            gyroBias(2) * RAD_TO_DEG, imuTemp);
  }

  void updateMagBias() {
    auto magData = magSub.getItem().data.val;

    /*for (size_t i = 0; i < 3; i++)
    {
        if (magData[i][0] < magMin[i][0]) magMin[i][0] = magData[i][0];
        if (magData[i][0] > magMax[i][0]) magMax[i][0] = magData[i][0];
    }

    LOG_MSG("Mag val: %.2f, %.2f, %.2f, M:%.2f\n", magData(0), magData(1),
    magData(2), magData.magnitude());

    LOG_MSG("Mag min: ");
    magMin.printTo(Core::printM);
    LOG_MSG("\n");

    LOG_MSG("Mag max: ");
    magMax.printTo(Core::printM);
    LOG_MSG("\n");*/
  }

  void updateAccBias() {
    if (!accelSub.isDataNew())
      return;

    auto accData = accelSub.getItem().data.val;

    accelVal = accelVal * 0.99 + accData * 0.01;

    if (firstAcc || Serial.available()) {
      Serial.flush();
      Serial.clear();
      accelVal = accData;
      firstAcc = false;
    }

    LOG_MSG("Acc Bias: \tX:%.3f\tY:%.3f\tZ:%.3f\n", accelVal(0), accelVal(1),
            accelVal(2));
  }

  void updateBaro() {
    auto baroData = baroSub.getItem().data.val;

    auto altitude = posEstTask.calcAltitudeFromPressure(baroData(0));

    gyroBiasBuffer.placeBack(altitude, true);

    auto average = gyroBiasBuffer.getAverage();
    auto variance = gyroBiasBuffer.getStandardDeviation();
    variance = variance * variance;

    LOG_MSG("Baro: %.2f, Altitude: %.2f, Var: %.2f\n", baroData(0), average,
            variance);
  }

  void taskThread() override {
    // updateBaro();
    // updateGyroBias();
    // updateMagBias();
    updateAccBias();

    // tvcTopic.publish(tvcSetting);
  }
};
// CalibrationTesting calibrationTask;

void initialiseHardware() {
  SPI.begin();
  Wire.begin();
  Wire.setClock(100000);

  mpuPin.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
  // bmePin.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);

  mpuPin.setPinValue(1);
  // bmePin.setPinValue(1);

  mpuIntPin.init(HAL::GPIO_IOMODE_t::IOMODE_INPUT);

  // mpuDriver.enablePinInterrupt(mpuIntPin);
  // mpuDriver.ConfigDlpf(mpuDriver.DLPF_BANDWIDTH_250HZ_4kHz);
  mpuDriver.taskInit();
  mpuDriver.ConfigDlpf(SNSR::MPU9250::DlpfBandwidth::DLPF_BANDWIDTH_41HZ);
  mpuDriver.setInterval(250 * Core::MICROSECONDS); // 32kHz rate
  mpuDriver.setInitialised(true);
  mpuIntHandler.setCallback(&SNSR::MPU9250Driver::interruptHandler, mpuDriver);
  mpuDriver.setPriority(110);
  // mpuDriver.DisableDrdyInt();

  imuTask.setInterval(250 * Core::MICROSECONDS);
  imuTask.setPriority(105);

  posEstTask.setPriority(105);
  // mpuDriver.setPriority(1000);
  // bme.setPriority(1000);

  // Core::getSystemScheduler().removeTask(gnss);

  // servoTest.setPosition(0);

  // Hard motor interlock: the TVC gate reads it, the watchdog (priority above the controllers) enforces it.
  starshipTVC.setMotorInterlock(&vehicleComms.motorInterlock());

  starshipTVC.setTVCFinsOffset(5 * DEG_TO_RAD, -2 * DEG_TO_RAD, 0 * DEG_TO_RAD,
                               5 * DEG_TO_RAD);
}

void initialiseMemory() {
  // Initialise the memory and read the EEPROM into the internal memory
  EEPROM.begin();
  internalMemory.transferFrom(eeprom);

  // CALIBRATION telemetry source bits (what was loaded and from where).
  uint8_t gyroSrc = 0, accSrc = 0, magSrc = 0;

  if (memoryManager.getManagerVersion() != memoryManager.getMemoryVersion()) {
    LOG_MSG("Memory manager version mismatch, clearing memory\n");
    memoryManager.clearMemory();
    gyroSrc = accSrc = magSrc = CAL_SRC_MEMORY_CLEARED;
  }
  // memoryManager.clearMemory();

  // Read the calibration and transformation data from the internal memory
  DSP::ValueCov<float, 3> magCalibData;
  DSP::ValueCov<float, 3> gyroCalibData;
  DSP::ValueCov<float, 3> accCalibData;

  Math::Matrix<float, 3, 3> magTransform;
  Math::Matrix<float, 3, 3> gyroTransform;
  Math::Matrix<float, 3, 3> accTransform;

  if (!memoryManager.readItem(magCalibData, MEMORY_KEY_MAGCALIB)) {
    LOG_MSG("No mag calibration data found, creating new one\n");
    magCalibData.val = Math::Vector<float, 3>({0, 0, 0});
    magCalibData.cov = Math::Matrix<float, 3, 3>({1, 0, 0, 0, 1, 0, 0, 0, 1});

    memoryManager.allocateItem(magCalibData, MEMORY_KEY_MAGCALIB);
    memoryManager.writeItem(magCalibData, MEMORY_KEY_MAGCALIB);
  } else {
    magSrc |= CAL_SRC_CALIB_EEPROM;
    magLegacy.keyPresent = true;
  }

  if (!memoryManager.readItem(gyroCalibData, MEMORY_KEY_GYROCALIB)) {
    LOG_MSG("No gyro calibration data found, creating new one\n");
    gyroCalibData.val = Math::Vector<float, 3>({0, 0, 0});
    gyroCalibData.cov = Math::Matrix<float, 3, 3>({1, 0, 0, 0, 1, 0, 0, 0, 1});

    memoryManager.allocateItem(gyroCalibData, MEMORY_KEY_GYROCALIB);
    memoryManager.writeItem(gyroCalibData, MEMORY_KEY_GYROCALIB);
  } else {
    gyroSrc |= CAL_SRC_CALIB_EEPROM;
    gyroLegacy.keyPresent = true;
    LOG_MSG("Gyro calibration data found: %.3f, %.3f, %.3f\n",
            gyroCalibData.val(0) * RAD_TO_DEG,
            gyroCalibData.val(1) * RAD_TO_DEG,
            gyroCalibData.val(2) * RAD_TO_DEG);
  }

  if (!memoryManager.readItem(accCalibData, MEMORY_KEY_ACCCALIB)) {
    LOG_MSG("No acc calibration data found, creating new one\n");
    accCalibData.val = Math::Vector<float, 3>({0, 0, 0});
    accCalibData.cov = Math::Matrix<float, 3, 3>({1, 0, 0, 0, 1, 0, 0, 0, 1});

    memoryManager.allocateItem(accCalibData, MEMORY_KEY_ACCCALIB);
    memoryManager.writeItem(accCalibData, MEMORY_KEY_ACCCALIB);
  } else {
    accSrc |= CAL_SRC_CALIB_EEPROM;
    accLegacy.keyPresent = true;
  }

  if (!memoryManager.readItem(magTransform, MEMORY_KEY_MAGTRANSFORM)) {
    LOG_MSG("No mag transform data found, creating new one\n");
    magTransform = Math::Matrix<float, 3, 3>::eye();
    memoryManager.allocateItem(magTransform, MEMORY_KEY_MAGTRANSFORM);
    memoryManager.writeItem(magTransform, MEMORY_KEY_MAGTRANSFORM);
  } else {
    magSrc |= CAL_SRC_TRANSFORM_EEPROM;
  }

  if (!memoryManager.readItem(gyroTransform, MEMORY_KEY_GYROTRANSFORM)) {
    LOG_MSG("No gyro transform data found, creating new one\n");
    gyroTransform = Math::Matrix<float, 3, 3>::eye();
    memoryManager.allocateItem(gyroTransform, MEMORY_KEY_GYROTRANSFORM);
    memoryManager.writeItem(gyroTransform, MEMORY_KEY_GYROTRANSFORM);
  } else {
    gyroSrc |= CAL_SRC_TRANSFORM_EEPROM;
  }

  if (!memoryManager.readItem(accTransform, MEMORY_KEY_ACCTRANSFORM)) {
    LOG_MSG("No acc transform data found, creating new one\n");
    accTransform = Math::Matrix<float, 3, 3>::eye();
    memoryManager.allocateItem(accTransform, MEMORY_KEY_ACCTRANSFORM);
    memoryManager.writeItem(accTransform, MEMORY_KEY_ACCTRANSFORM);
  } else {
    accSrc |= CAL_SRC_TRANSFORM_EEPROM;
  }

  // No hard-coded gyro bias any more: the boot offset is key 100 (the last bias a run stored) until the automatic gyro
  // run of the boot zeroing window applies a fresh one (CalibrationService, VehicleSafetyAndControlTask).

  /*accCalibData.val = {0.4278, 0.560, 2.0725};
  accCalibData.val(0) /= 0.9984;
  accCalibData.val(1) /= 1.003;
  accCalibData.val(2) /= 0.9673;
  accCalibData.cov = {
      0.9984, 0, 0,
      0, 1.003, 0,
      0, 0, 0.9673
  };*/

  applySensorCalibration(gyroTransformTopic, gyroCalApplied,
                         gyroCalibData.cov * gyroTransform, gyroCalibData.val,
                         gyroSrc);
  applySensorCalibration(accTransformTopic, accCalApplied,
                         accCalibData.cov * accTransform, accCalibData.val,
                         accSrc);
  applySensorCalibration(magTransformTopic, magCalApplied,
                         magCalibData.cov * magTransform, magCalibData.val,
                         magSrc);

  // What the calibration service and the validity check work from.
  gyroLegacy.calib = gyroCalibData;
  gyroLegacy.transform = gyroTransform;
  accLegacy.calib = accCalibData;
  accLegacy.transform = accTransform;
  magLegacy.calib = magCalibData;
  magLegacy.transform = magTransform;
  publishActiveCalibration();

  // Sync the internal memory with the EEPROM. This wont write anything if we
  // ended up only reading.
  eeprom.transferFrom(internalMemory);

  // Lets update the memory with known values for the calibration data. This is
  // only for testing purposes.
  if (false) {
    accCalibData.val = {0.4278, 0.560, 2.0725};
    accCalibData.val(0) /= 0.9984;
    accCalibData.val(1) /= 1.003;
    accCalibData.val(2) /= 0.9673;
    accCalibData.cov = {0.9984, 0, 0, 0, 1.003, 0, 0, 0, 0.9673};
    // memoryManager.writeItem(accCalibData, MEMORY_KEY_ACCCALIB);

    gyroCalibData.val = Math::Vector<float, 3>(
        {0.228 * DEG_TO_RAD, 1.314 * DEG_TO_RAD, -0.5085 * DEG_TO_RAD});
    gyroCalibData.cov =
        Math::Matrix<float, 3, 3>({-1, 0, 0, 0, 1, 0, 0, 0, -1});
    // memoryManager.writeItem(gyroCalibData, MEMORY_KEY_GYROCALIB);

    Math::Vector<float, 3> magMin = {-0.3972, -0.3577, -0.2332};
    Math::Vector<float, 3> magMax = {0.3662, 0.4033, 0.4663};
    // Mag bias
    // auto magBias =
    magCalibData.val = (magMax + magMin) / 2;
    magCalibData.cov = Math::Matrix<float, 3, 3>::eye();
    // Calculate the scale factor
    auto magScale = (magMax - magMin) / 2;
    magCalibData.cov(0, 0) = 1 / magScale(0);
    magCalibData.cov(1, 1) = 1 / magScale(1);
    magCalibData.cov(2, 2) = 1 / magScale(2);
    magCalibData.cov = magCalibData.cov * 0.05;
    magCalibData.cov = magCalibData.cov *
                       Math::Matrix<float, 3, 3>({0, 0, -1, 1, 0, 0, 0, -1, 0});
    // memoryManager.writeItem(magCalibData, MEMORY_KEY_MAGCALIB);

    accTransform = {-1, 0, 0, 0, 1, 0, 0, 0, -1};

    memoryManager.writeItem(accTransform, MEMORY_KEY_ACCTRANSFORM);

    applySensorCalibration(gyroTransformTopic, gyroCalApplied,
                           gyroCalibData.cov * gyroTransform,
                           gyroCalibData.val, CAL_SRC_RAM_ONLY);
    applySensorCalibration(accTransformTopic, accCalApplied,
                           accCalibData.cov * accTransform, accCalibData.val,
                           CAL_SRC_RAM_ONLY);
    applySensorCalibration(magTransformTopic, magCalApplied,
                           magCalibData.cov * magTransform, magCalibData.val,
                           CAL_SRC_RAM_ONLY);

    eeprom.transferFrom(internalMemory);
  }
}

void initialiseTopicConnections() {
  // Calibration service: raw driver topics (before the calibration transforms), only fed while a session or run needs them.
  calibrationTask.connect(mpuDriver.getGyroTopic(), mpuDriver.getAccelTopic(), qmc.getMagTopic(),
                          mpuDriver.getThermoTopic());

  gyroTransformTopic.setInputTopic(mpuDriver.getGyroTopic());
  accTransformTopic.setInputTopic(mpuDriver.getAccelTopic());
  magTransformTopic.setInputTopic(qmc.getMagTopic());

  imuTask.setGyroInput(gyroTransformTopic.getOutputTopic());
  imuTask.setAccInput(accTransformTopic.getOutputTopic());
  imuTask.setMagInput(magTransformTopic.getOutputTopic());

  posEstTask.setAccInput(accTransformTopic.getOutputTopic());
  posEstTask.setAttitudeInput(imuTask.getAttitudeEstTopic());
  posEstTask.setBaroInput(bme.getBaroTopic());
  posEstTask.setRangeInput(vl53l0xDriver.getRangeTopic(), 0.4, 1.5);
  posEstTask.setGNSSInput(gnss.getGNSSTopic());
  // posEstTask.setUseRangeForAltitude(true);

  // controlRocket.subscribeAttitudeMeasurement(attitudeTopicSwitch.getTopic());
  // controlRocket.subscribePositionMeasurement(positionTopicSwitch.getTopic());
  // controlRocket.subscribeSetpoint(positionSetpointTopic);
  // controlPositionStandard.subscribeSetpoint(positionSetpointTopic);
  controlPositionStandard.subscribePositionMeasurement(
      positionTopicSwitch.getTopic());

  controlMappingAccToAtt.subscribeAccelSetpoint(
      controlPositionStandard.getAccelTopic());

  controlAttitudeTvc.subscribeAttitudeMeasurement(
      attitudeTopicSwitch.getTopic());
  controlAttitudeTvc.subscribeAttitudeSetpoint(
      controlMappingAccToAtt.getAttitudeTopic());
  controlAttitudeTvc.subscribeAccelerationSetpoint(
      controlPositionStandard.getAccelTopic());
  controlAttitudeTvc.setTVCLimitCompensation(true);

  controlAttitudeFlaps.subscribeControlSetting(bellyFlopControlTopic);
  controlAttitudeFlaps.subscribeAttitudeMeasurement(
      attitudeTopicSwitch.getTopic());
  controlAttitudeFlaps.subscribePositionMeasurement(
      positionTopicSwitch.getTopic());
  controlAttitudeFlaps.subscribeFlapSettingOutputTopic(flapSettingTopic);

  starshipTVC.setTVCInputTopic(controlAttitudeTvc.getTvcTopic());
  starshipFlaps.setFlapSettingTopic(flapSettingTopic);
  starshipFlaps.setTrueFlapSettingTopic(trueFlapSettingTopic);
  starshipFlaps.setSimulationFlag(vehicleSafetyAndControlTask.simulationFlag()); // gate of the sim-only flap faults

  bodySimulator.setPaused(true);
  bodySimulator.setTVCInputTopic(starshipTVC.getTVCActualThrustTopic(),
                                 Math::Vector<float, 3>({0, 0, -0.35}),
                                 TVC_ANGLE_LIMIT_RAD, TVC_THRUST_LIMIT_N);
  bodySimulator.setFlapSettingInputTopic(trueFlapSettingTopic);

  positionTopicSwitch.subscribe(posEstTask.getStateEstTopic());
  attitudeTopicSwitch.subscribe(imuTask.getAttitudeEstTopic());

  Math::Quat<float> accTiltX({1, 0, 0}, 0 * DEG_TO_RAD);
  Math::Quat<float> accTiltY({0, 1, 0}, 0 * DEG_TO_RAD);
  imuTask.setAccTiltCompensation((accTiltX * accTiltY).to3x3RotMat());
}

void setup() {
  // delay(1000);

  VCTR::Core::initialise();

  initialiseHardware();

  initialiseMemory();

  initialiseTopicConnections();
}

void loop() { VCTR::Core::getSystemScheduler().tick(); }