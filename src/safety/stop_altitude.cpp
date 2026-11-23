#include "stop_altitude.hpp"

namespace VCTR {
namespace Safety {

StopAltResult decideSetStopAltitude(float valueM, const StopAltContext &c) {
  if (c.interlockOpen) return StopAltResult::InterlockOpen;
  if (c.committed) return StopAltResult::Committed;
  if (!isfinite(valueM)) return StopAltResult::NotFinite;
  if (valueM < kStopAltitudeMinM) return StopAltResult::OutOfRange;
  return StopAltResult::Accepted;
}

const char *toString(StopAltResult r) {
  switch (r) {
  case StopAltResult::Accepted: return "accepted";
  case StopAltResult::NotFinite: return "value not finite";
  case StopAltResult::OutOfRange: return "value below 1 m";
  case StopAltResult::InterlockOpen: return "motors armed (gate open)";
  case StopAltResult::Committed: return "mission committed (in flight or pending)";
  }
  return "?";
}

} // namespace Safety
} // namespace VCTR
