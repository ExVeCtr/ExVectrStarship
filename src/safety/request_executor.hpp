#ifndef SAFETY_REQUEST_EXECUTOR_HPP
#define SAFETY_REQUEST_EXECUTOR_HPP

/**
 * Vehicle REQUEST executor (T9b-5, docs/t9b-design.md T9b-5, docs/gs-control-design.md Q5). Pure and deterministic: no
 * Arduino, no ExVectrCore, no heap, no packet types. Host-tested (test/test_request_executor).
 *
 * handle() decides one received REQUEST copy and returns the REQUEST_ACK content (status, detail). Rules:
 *  - Idempotent by (source node, req_id): the last kMemory (8) distinct requests are remembered with their result. A
 *    duplicate copy (same frame on the other link, a retry) is NOT executed again and is answered with the stored
 *    result; every copy is acknowledged. The 9th distinct request evicts the oldest. Every result is stored,
 *    refusals included, so a sender must use a NEW req_id after a refusal.
 *  - Check order of a new request (first failing rule wins):
 *      target must be the vehicle (1)                              -> REJECTED_INVALID detail 2
 *      action must be known (7, 8, 12..23; 1..6, 9, 10, 11 retired) -> REJECTED_INVALID detail 3
 *      session must be non-zero and equal the latched session      -> REJECTED_STATE detail 3
 *      then per action (details below).
 *  - 12 ZERO_ESTIMATORS, 13 SET_STOP_ALTITUDE: on the ground (not committed, interlock closed) else REJECTED_STATE 1;
 *    argument range -> INVALID 1; own preconditions -> REJECTED_STATE 1 (stop altitude) or 2 (zeroing refused).
 *    ZERO_ESTIMATORS arg_f = window seconds (NaN or 0 -> 10 s, clamped 1..60 by the zeroing object).
 *    SET_STOP_ALTITUDE arg_f finite and >= 1 m (Safety::decideSetStopAltitude).
 *  - 7 CALIB_APPLY (arg_i 0 gyro 1 accel 2 mag), 8 SAVE_CONFIG, 14 CALIB_CONTROL (arg_i = subsystem*10 + code,
 *    code 1 start, 2 verify, 0 stop and fit, 9 discard): bad arg_i -> INVALID 1; then every refusal is REJECTED_STATE 6
 *    (calibration gates in RequestContext::calibGatesOk = initialised, disarmed, motors off, mode, mission idle; 14
 *    additionally needs simulation OFF; the action itself may refuse too). Not tied to the "on the ground" rule.
 *  - 15..22 SIM_*: simulation mode only (else REJECTED_STATE 5), not tied to the ground; non-finite arg_f -> INVALID 1
 *    (22 SIM_FAULT_CLEAR ignores arguments); 20/21 arg_i flap must be 0..3 exactly -> INVALID 1.
 *    Clamps: 15 wind speed 0..20 m/s, 16 direction wrapped to -pi..pi, 17 gust strength 0..10, 18 gust period
 *    0.5..30 s, 19 seed rounded and clamped 0..16777215, 20 stuck angle 0..pi/2 (-1 = hold the current true angle),
 *    21 slow factor 0.05..1 (Safety::decideSimFault).
 *  - 23 GET_HOME_POSITION: DONE and RequestResult::sendHome (the caller sends the HOME_POSITION packet after the ACK);
 *    no position reference yet -> REJECTED_STATE 4 and no packet.
 *  Everything else (reset, link actions) is not implemented on the vehicle: REJECTED_INVALID detail 3.
 * The executor never touches vehicle state itself: the actions go through RequestActionsI.
 */

#include <stddef.h>
#include <stdint.h>

#include "sim_flap_fault.hpp"

namespace VCTR {
namespace Safety {

/// One received REQUEST copy (decoded fields) plus where it came from.
struct RequestMsg {
  uint8_t link = 0;   ///< CmdLink value of the ingress link (the ACK goes back on it)
  uint8_t src = 0;    ///< source node
  uint16_t reqId = 0;
  uint8_t action = 0;
  uint8_t target = 0;
  int32_t argI = 0;
  float argF = 0.0f;
  uint32_t session = 0;
};

/// The ACK content.
struct RequestResult {
  uint8_t status = 0; ///< RequestStatus: 0 DONE, 2 REJECTED_STATE, 3 REJECTED_INVALID
  uint8_t detail = 0;
  bool duplicate = false; ///< not executed again, stored result re-acknowledged
  bool executed = false;  ///< the action ran on this copy
  bool sendHome = false;  ///< GET_HOME_POSITION accepted on this copy: send HOME_POSITION after the ACK
};

/// Vehicle state needed to judge a request.
struct RequestContext {
  bool sessionLatched = false;
  uint32_t latchedSession = 0;
  bool interlockOpen = false; ///< motor interlock open (armed)
  bool committed = false;     ///< Safety::committed()
  bool simulationMode = false;
  bool calibGatesOk = false;  ///< Calibration::checkCommand == Ok (initialised, disarmed, motors off, mode, mission idle)
  bool homeAvailable = false; ///< a position reference exists (home_epoch != 0)
};

/// The vehicle actions. Implemented by the safety task (main.cpp). Values are already validated and clamped.
class RequestActionsI {
public:
  virtual ~RequestActionsI() = default;
  /// Starts the zeroing window. Returns false if the zeroing object refused (initialised, armed, motors, mode).
  virtual bool zeroEstimators(float windowS) = 0;
  /// Applies an already validated stop altitude.
  virtual void setStopAltitude(float metres) = 0;
  /// Calibration (same code as the former telecommands). False = refused (logged by the implementation).
  virtual bool calibApply(uint8_t subsystem) = 0;
  virtual bool saveConfig() = 0;
  virtual bool calibControl(uint8_t subsystem, uint8_t code) = 0;
  /// Simulation world (the executor has checked simulation mode and clamped the values).
  virtual void simWindSpeed(float mps) = 0;
  virtual void simWindDirection(float rad) = 0;
  virtual void simGustStrength(float mps) = 0;
  virtual void simGustPeriod(float s) = 0;
  virtual void simGustSeed(uint32_t seed) = 0;
  /// Stuck / slow / clear flap fault from the validated decision. False = not applied.
  virtual bool simFault(const SimFaultDecision &d) = 0;
};

class RequestExecutor {
public:
  static constexpr uint8_t kMemory = 8;
  static constexpr uint8_t kVehicleNode = 1;
  static constexpr uint8_t kActionZeroEstimators = 12;
  static constexpr uint8_t kActionSetStopAltitude = 13;
  static constexpr uint8_t kActionCalibApply = 7, kActionSaveConfig = 8, kActionCalibControl = 14;
  static constexpr uint8_t kActionSimWindSpeed = 15, kActionSimWindDirection = 16, kActionSimGustStrength = 17,
                           kActionSimGustPeriod = 18, kActionSimGustSeed = 19, kActionSimFlapStuck = 20,
                           kActionSimFlapSlow = 21, kActionSimFaultClear = 22, kActionGetHomePosition = 23;
  static constexpr uint8_t kStatusDone = 0, kStatusRejectedState = 2, kStatusRejectedInvalid = 3;
  // REJECTED_STATE details
  static constexpr uint8_t kDetailNotOnGround = 1, kDetailZeroingRefused = 2, kDetailSession = 3,
                                kDetailNoHome = 4, kDetailNotSimulation = 5, kDetailCalibRefused = 6;
  // REJECTED_INVALID details
  static constexpr uint8_t kDetailArgRange = 1, kDetailWrongTarget = 2, kDetailUnknownAction = 3;

  RequestResult handle(const RequestMsg &m, const RequestContext &ctx, RequestActionsI &actions);

  uint32_t executedCount() const { return executed_; }
  uint32_t duplicateCount() const { return duplicates_; }
  uint32_t refusedCount() const { return refused_; }
  /// Last executed (new, not duplicate) request id and its status, for VEHICLE_STATUS last_req_id (0 = none).
  uint16_t lastReqId() const { return lastReqId_; }
  uint8_t lastStatus() const { return lastStatus_; }

private:
  struct Entry {
    bool used = false;
    uint8_t src = 0;
    uint16_t reqId = 0;
    uint8_t status = 0, detail = 0;
  };
  void remember(const RequestMsg &m, const RequestResult &r);
  RequestResult decide(const RequestMsg &m, const RequestContext &ctx, RequestActionsI &actions);

  Entry mem_[kMemory];
  uint8_t next_ = 0; ///< ring slot of the next insert (the oldest entry once full)
  uint32_t executed_ = 0, duplicates_ = 0, refused_ = 0;
  uint16_t lastReqId_ = 0;
  uint8_t lastStatus_ = 0;
};

/// Small FIFO between the comms task (producer, taps) and the safety task (consumer). Single core, cooperative: no
/// locking. Full -> the new request is dropped and counted (the sender retries).
class RequestInbox {
public:
  static constexpr uint8_t kSize = 8;
  bool push(const RequestMsg &m) {
    if (count_ == kSize) {
      dropped_++;
      return false;
    }
    q_[(head_ + count_) % kSize] = m;
    count_++;
    return true;
  }
  bool pop(RequestMsg &m) {
    if (count_ == 0) return false;
    m = q_[head_];
    head_ = (head_ + 1) % kSize;
    count_--;
    return true;
  }
  uint8_t size() const { return count_; }
  uint32_t dropped() const { return dropped_; }

private:
  RequestMsg q_[kSize];
  uint8_t head_ = 0, count_ = 0;
  uint32_t dropped_ = 0;
};

} // namespace Safety
} // namespace VCTR

#endif
