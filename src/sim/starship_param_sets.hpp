#ifndef STARSHIP_SIM_STARSHIP_PARAM_SETS_HPP
#define STARSHIP_SIM_STARSHIP_PARAM_SETS_HPP

/**
 * Parameter sets of the plant as data (simulation architecture, section 4.7). A test matrix loops over
 * `kVariants x scenarios x seeds`; every variant has its own Config and all of them can step side by side.
 *
 * "nominal" is today's model (aeroCalibration 4.64: belly-down terminal speed 6.4 m/s). "aero x0.7" and "aero x1.3"
 * scale every aero force and moment (robustness against the unexplained calibration, Known issue High in
 * ExVectrDSP.md). "geometry" uses the documented geometry and coefficients with no calibration (aeroCalibration 1.0,
 * a terminal speed of about 13.5 m/s). The ascent study's six variants (ascent-control-design.md, section 4) are
 * added when S5 ports that harness.
 */

#include "starship_plant.hpp"

namespace VCTR {
namespace ShipSim {

struct ParamVariant {
  const char *name;
  float aeroScale;    ///< Multiplies the aero calibration.
  bool geometryAero;  ///< true: start from the uncalibrated geometry (aeroCalibration 1.0) instead of the base value.
};

inline constexpr ParamVariant kVariants[] = {
    {"nominal", 1.0f, false},   // aeroCalibration 4.64
    {"aero x0.7", 0.7f, false}, {"aero x1.3", 1.3f, false},
    {"geometry", 1.0f, true},   // aeroCalibration 1.0
};
inline constexpr int kNumVariants = int(sizeof(kVariants) / sizeof(kVariants[0]));

inline StarshipPlantParams makeParams(const ParamVariant &v, StarshipPlantParams base = StarshipPlantParams()) {
  base.aeroCalibration = (v.geometryAero ? 1.0f : base.aeroCalibration) * v.aeroScale;
  return base;
}

} // namespace ShipSim
} // namespace VCTR

#endif
