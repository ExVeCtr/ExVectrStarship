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
    step_ = Action::PollGnssForWrite;
    break;
  case Action::PollGnssForWrite:
    step_ = Action::SetGnssGpsGalileo;
    break;
  case Action::SetGnssGpsGalileo:
    step_ = Action::SaveConfig;
    // The receiver restarts its GNSS subsystem after a CFG-GNSS change: wait before the save, so the save is not
    // sent while it restarts (review 2026-11-20; before, the save followed at once and the wait came after it).
    notBeforeNs_ = nowNs + kGnssSwitchWaitNs;
    break;
  case Action::SaveConfig:
    step_ = Action::ReadGnssConfig;
    break;
  case Action::ReadGnssConfig:
    step_ = Action::ReadGnssInUse;
    break;
  case Action::ReadGnssInUse:
    step_ = Action::ReadNavRate;
    break;
  case Action::ReadNavRate:
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
  case Action::PollGnssForWrite:
    return "poll CFG-GNSS";
  case Action::SetGnssGpsGalileo:
    return "constellations GPS + Galileo";
  case Action::SaveConfig:
    return "save configuration";
  case Action::ReadGnssConfig:
    return "read back CFG-GNSS";
  case Action::ReadGnssInUse:
    return "read back MON-GNSS";
  case Action::ReadNavRate:
    return "read back CFG-RATE";
  }
  return "?";
}

bool GnssConfigWarning::due(bool finished, bool ok, int64_t nowNs) {
  if (!finished || ok) {
    warned_ = false; // a later failure warns at once again
    return false;
  }
  if (!warned_ || nowNs - lastNs_ >= kPeriodNs) {
    warned_ = true;
    lastNs_ = nowNs;
    return true;
  }
  return false;
}

// ---- UBX payload helpers -----------------------------------------------------------------------------------------

static bool cfgGnssLayoutOk(const uint8_t *p, uint16_t len) {
  if (p == nullptr || len < 4)
    return false;
  const uint16_t blocks = p[3];
  return len >= 4 + 8 * blocks;
}

bool decodeCfgGnss(const uint8_t *p, uint16_t len, GnssConstellations &out) {
  out = GnssConstellations();
  if (!cfgGnssLayoutOk(p, len))
    return false;
  out.numTrkChHw = p[1];
  out.numTrkChUse = p[2];
  for (uint16_t b = 0; b < p[3]; b++) {
    const uint8_t *blk = p + 4 + 8 * b;
    const bool en = (blk[4] & 0x01) != 0;
    switch (blk[0]) {
    case kGnssGps: out.gps = en; break;
    case kGnssSbas: out.sbas = en; break;
    case kGnssGalileo: out.galileo = en; break;
    case kGnssBeidou: out.beidou = en; break;
    case kGnssImes: out.imes = en; break;
    case kGnssQzss: out.qzss = en; break;
    case kGnssGlonass: out.glonass = en; break;
    default: break;
    }
  }
  out.valid = true;
  return true;
}

bool applyGpsGalileo(uint8_t *p, uint16_t len) {
  if (!cfgGnssLayoutOk(p, len))
    return false;
  bool haveGps = false, haveGal = false;
  for (uint16_t b = 0; b < p[3]; b++) {
    uint8_t *blk = p + 4 + 8 * b;
    // QZSS follows GPS: the u-blox M8 refuses (NACK) a CFG-GNSS with GPS on and QZSS off (bench 2026-11-20).
    const bool want = blk[0] == kGnssGps || blk[0] == kGnssGalileo || blk[0] == kGnssQzss;
    haveGps |= blk[0] == kGnssGps;
    haveGal |= blk[0] == kGnssGalileo;
    if (want) {
      blk[4] |= 0x01;
      if (blk[6] == 0)
        blk[6] = 0x01; // sigCfgMask: GPS L1C/A, Galileo E1OS, QZSS L1C/A
    } else {
      blk[4] &= 0xFE;
    }
  }
  return haveGps && haveGal;
}

bool constellationsAsIntended(const GnssConstellations &c) {
  // QZSS is not checked: it follows GPS (the receiver refuses GPS on with QZSS off) and has no satellites in view
  // outside the Asia-Pacific region.
  return c.valid && c.gps && c.galileo && !c.sbas && !c.beidou && !c.imes && !c.glonass;
}

bool decodeMonGnss(const uint8_t *p, uint16_t len, GnssInUse &out) {
  out = GnssInUse();
  if (p == nullptr || len < 5)
    return false;
  out.supported = p[1];
  out.enabled = p[3];
  out.valid = true;
  return true;
}

bool decodeCfgRate(const uint8_t *p, uint16_t len, GnssNavRate &out) {
  out = GnssNavRate();
  if (p == nullptr || len < 6)
    return false;
  out.measRateMs = uint16_t(p[0] | (p[1] << 8));
  out.navRateCycles = uint16_t(p[2] | (p[3] << 8));
  out.valid = true;
  return true;
}

} // namespace SNSR
} // namespace VCTR
