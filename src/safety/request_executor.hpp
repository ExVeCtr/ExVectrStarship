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
 *      action must be known (12 ZERO_ESTIMATORS, 13 SET_STOP_ALT.) -> REJECTED_INVALID detail 3
 *      session must be non-zero and equal the latched session      -> REJECTED_STATE detail 3
 *      the vehicle must be on the ground (not committed)           -> REJECTED_STATE detail 1
 *      argument range                                              -> REJECTED_INVALID detail 1
 *      the action's own preconditions                              -> REJECTED_STATE detail 1 (stop altitude) or 2
 *                                                                     (zeroing state refusal)
 *  - ZERO_ESTIMATORS: arg_f = window seconds (NaN or 0 -> 10 s, clamped 1..60 by the zeroing object).
 *  - SET_STOP_ALTITUDE: arg_f finite and >= 1 m (Safety::decideSetStopAltitude).
 *  Everything else (calibration, reset, link actions) is not implemented on the vehicle: REJECTED_INVALID detail 3.
 * The executor never touches vehicle state itself: the two actions go through RequestActionsI.
 */

#include <stddef.h>
#include <stdint.h>

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
};

/// Vehicle state needed to judge a request.
struct RequestContext {
  bool sessionLatched = false;
  uint32_t latchedSession = 0;
  bool interlockOpen = false; ///< motor interlock open (armed)
  bool committed = false;     ///< Safety::committed()
};

/// The two actions. Implemented by the safety task (main.cpp).
class RequestActionsI {
public:
  virtual ~RequestActionsI() = default;
  /// Starts the zeroing window. Returns false if the zeroing object refused (initialised, armed, motors, mode).
  virtual bool zeroEstimators(float windowS) = 0;
  /// Applies an already validated stop altitude.
  virtual void setStopAltitude(float metres) = 0;
};

class RequestExecutor {
public:
  static constexpr uint8_t kMemory = 8;
  static constexpr uint8_t kVehicleNode = 1;
  static constexpr uint8_t kActionZeroEstimators = 12;
  static constexpr uint8_t kActionSetStopAltitude = 13;
  static constexpr uint8_t kStatusDone = 0, kStatusRejectedState = 2, kStatusRejectedInvalid = 3;
  // REJECTED_STATE details
  static constexpr uint8_t kDetailNotOnGround = 1, kDetailZeroingRefused = 2, kDetailSession = 3;
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
  static constexpr uint8_t kSize = 4;
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
