#include "safety/mission_params.hpp"

#ifndef FLASHMEM // Teensy: cold code runs from flash and keeps ITCM (RAM1 code) small; empty on the host
#ifdef __IMXRT1062__
#define FLASHMEM __attribute__((section(".flashmem")))
#else
#define FLASHMEM
#endif
#endif

namespace VCTR {
namespace Safety {

FLASHMEM bool missionParamValid(uint8_t id, float v) {
  if (!isfinite(v)) return false;
  switch (id) {
  case 1: return v >= 0.5f && v <= 10.0f;
  case 2: return v >= 2.0f && v <= 1000.0f;
  case 3:
  case 4:
  case 5:
  case 6: return v >= 0.0f && v <= 600.0f;
  default: return false;
  }
}

FLASHMEM bool missionParamApply(MissionParamSet &s, uint8_t id, float v) {
  if (!missionParamValid(id, v)) return false;
  switch (id) {
  case 1: s.ascentSpeed = v; break;
  case 2: s.apogeeAltitude = v; break;
  case 3: s.hoverHoldS = v; break;
  case 4: s.p1Hold1S = v; break;
  case 5: s.p1Hold2S = v; break;
  default: s.p1Hold3S = v; break;
  }
  return true;
}

} // namespace Safety
} // namespace VCTR
