#ifndef STARSHIP_SIM_PLACEMENT_HPP
#define STARSHIP_SIM_PLACEMENT_HPP

/**
 * Code placement of the vehicle's simulator (decision D5 of docs/project/simulation-architecture.md). The simulation runs
 * only in simulation mode, so it must not occupy ITCM (the ITCM padding below the 294912 B bank boundary is only
 * 1704 B). `SIM_COLD` puts a function into flash (`.flashmem`, executed through the flash cache) and `flatten` inlines
 * every callee that has a visible body into it, so inline helpers and templates do not end up as separate ITCM
 * functions. The non-inline functions of ExVectrDynamics carry `VCTR_DYN_COLD` (same section). No effect on the host.
 * Check with `arm-none-eabi-nm`: no `ShipSim`/`Dyn` symbol may be in `.itcm`/`.text`.
 */

#include "../flashmem.hpp"

#define SIM_COLD FLASHMEM __attribute__((flatten))

/// For member functions of class templates: they are COMDAT, and GCC reports a section type conflict when a template
/// and a plain function use the same section name. The linker script takes both (`*(.flashmem*)`).
#ifdef __IMXRT1062__
#define SIM_COLD_T __attribute__((section(".flashmem.t"), flatten))
#else
#define SIM_COLD_T __attribute__((flatten))
#endif

#endif
