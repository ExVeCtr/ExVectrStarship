#include "gnss_config_sequencer.hpp"

namespace VCTR {
namespace SNSR {

constexpr uint32_t GnssConfigSequencer::kBauds[GnssConfigSequencer::kBaudCount];

GnssConfigSequencer::Action GnssConfigSequencer::next(int64_t nowNs) {
  if (state_ != State::Running)
    return Action::None;
  if (nowNs < notBeforeNs_)
    return Action::None;
  return step_;
}

void GnssConfigSequencer::report(bool ok, int64_t nowNs) {
  if (state_ != State::Running)
    return;

  switch (step_) {
  case Action::Begin:
    if (ok) {
      step_ = Action::SetSerialRate;
      notBeforeNs_ = nowNs;
    } else if (++baudIndex_ >= kBaudCount) {
      state_ = State::Failed;
    } else {
      notBeforeNs_ = nowNs; // next baud right away, as before
    }
    attempts_ = 0;
    return;

  case Action::SetSerialRate:
    // Result ignored (the receiver switches baud and its ACK is not reliably seen), as before.
    step_ = Action::Reopen;
    notBeforeNs_ = nowNs;
    attempts_ = 0;
    return;

  case Action::Reopen:
    if (ok) {
      advance(nowNs);
    } else if (++attempts_ >= kMaxAttempts) {
      state_ = State::Failed;
    } else {
      notBeforeNs_ = nowNs + kRetryWaitNs;
    }
    return;

  default:
    if (ok) {
      advance(nowNs);
    } else if (++attempts_ >= kMaxAttempts) {
      skipped_++;
      advance(nowNs); // continue with the next step, as the old code did (it ignored all results)
    } else {
      notBeforeNs_ = nowNs + kRetryWaitNs;
    }
    return;
  }
}

void GnssConfigSequencer::advance(int64_t nowNs) {
  attempts_ = 0;
  notBeforeNs_ = nowNs;
  switch (step_) {
  case Action::Reopen:
    step_ = Action::SetUart1Ubx;
    break;
  case Action::SetUart1Ubx:
    step_ = Action::SetNavRate10Hz;
    break;
  case Action::SetNavRate10Hz:
    step_ = Action::SetAutoPvt;
    break;
  case Action::SetAutoPvt:
    step_ = Action::SetDynModelAirborne2g;
    break;
  case Action::SetDynModelAirborne2g:
    step_ = Action::DisableGlonass;
    break;
  case Action::DisableGlonass:
    step_ = Action::EnableGalileo;
    break;
  case Action::EnableGalileo:
    step_ = Action::DisableBeidou;
    break;
  case Action::DisableBeidou:
    step_ = Action::DisableSbas;
    break;
  case Action::DisableSbas:
    step_ = Action::DisableQzss;
    break;
  case Action::DisableQzss:
    step_ = Action::DisableImes;
    break;
  case Action::DisableImes:
    step_ = Action::DisableGps;
    notBeforeNs_ = nowNs + kGnssSwitchWaitNs; // was delay(1000)
    break;
  case Action::DisableGps:
    step_ = Action::SaveConfig;
    break;
  case Action::SaveConfig:
    step_ = Action::None;
    state_ = State::Done;
    break;
  default:
    state_ = State::Failed;
    break;
  }
}

const char *GnssConfigSequencer::toString(Action a) {
  switch (a) {
  case Action::None:
    return "none";
  case Action::Begin:
    return "begin";
  case Action::SetSerialRate:
    return "set serial rate 115200";
  case Action::Reopen:
    return "reopen at 115200";
  case Action::SetUart1Ubx:
    return "UART1 output UBX";
  case Action::SetNavRate10Hz:
    return "navigation rate 10 Hz";
  case Action::SetAutoPvt:
    return "auto PVT";
  case Action::SetDynModelAirborne2g:
    return "dynamic model airborne 2g";
  case Action::DisableGlonass:
    return "GLONASS off";
  case Action::EnableGalileo:
    return "Galileo on";
  case Action::DisableBeidou:
    return "BeiDou off";
  case Action::DisableSbas:
    return "SBAS off";
  case Action::DisableQzss:
    return "QZSS off";
  case Action::DisableImes:
    return "IMES off";
  case Action::DisableGps:
    return "GPS off";
  case Action::SaveConfig:
    return "save configuration";
  }
  return "?";
}

} // namespace SNSR
} // namespace VCTR
