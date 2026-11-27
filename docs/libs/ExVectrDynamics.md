# ExVectrDynamics

> Pure dynamics code for simulation and (later) model-based control: a rigid-body integrator, aerodynamic and actuator building blocks, and the generic model machinery. Depends on ExVectrMath only (no ExVectrCore, builds on the host). **Local repository only: the GitHub repo does not exist yet** (owner creates it, see "Repository state"). Design: [../project/simulation-architecture.md](../project/simulation-architecture.md). Status: S1 and S2 done (building blocks, host tests); S3 (flight-software `src/sim/`: `StarshipPlantT`, `starshipAero`, parameter sets, host golden test). Nothing is linked into the firmware yet (S4).

## Purpose & scope

- Reusable L1/L2 layers of the simulation architecture: building blocks (pure functions on POD types) and the machinery that runs a *model* (`Model` concept, `Sim<M, Integrator>`, integrators, probes).
- Ship-specific code (flap layout, plant composition, parameter sets) is **not** here; it lives in flight-software `src/sim/` (decision D2).
- Rules: float32, fixed step, SI, NWU, no heap, no virtual calls, no globals, no ExVectrCore.
- Bit-exactness: every block is a port of the legacy simulator code (`BodySimModel`, `PhysicsBodySim`) with the same operations in the same order, and each has a host test against a frozen verbatim copy under `test/frozen/` in flight-software.

## Dependencies

- ExVectrMath, `<math.h>`, `<stdint.h>`, `<type_traits>` (C++17).

## Key concepts & API (headers in `include/ExVectrDynamics/`)

| Header | Content |
|---|---|
| `types.hpp` | `Vec3`, `Wrench` (force and torque about the CoM, body axes), `NoProbe`, helpers `detail::clampf`, `detail::slew` |
| `rigid_body.hpp` | `RigidBodyParams` (+ `makeRigidBodyParams`), `RigidBodyState` (`att` `[ω,q]`, `pos` `[V,P]`), `ForceSums`, `addForce`/`addTorque` (mirror `PhysicsBodySim`), `semiImplicitEulerStep` (bit-exact `PhysicsBodySim::step`), `rigidBodyDeriv` (for RK4), `quatOf` |
| `model.hpp` | `is_model<M>` (detection idiom, checks the required types and functions, trivially copyable State/Input/Output) |
| `integrators.hpp` | `SemiImplicitEuler` (the model's own step), `Rk4` (rigid body, model force function, zero-order-hold discrete parts) |
| `sim.hpp` | `Sim<M, Integrator>`: shared read-only `Config*` plus own `State`; `step`, `advance` (same carry and bounded-catch-up rule as `BodySimModel::advance`), `snapshot`/`restore`, `clearTimeCarry` |

| `atmosphere.hpp` | `AtmosphereParams`, `airDensity` (constant 1.2 kg/m^3; no ISA yet) |
| `wind.hpp` (+`wind.cpp`) | `WindParams`, `GustTable`, `buildGustTable`, `gustAt`, `windAt`, `setWind`, `setGust`, `clearWind` (deterministic 8-sinusoid gust, xorshift32 phases, double phase arithmetic) |
| `slender_body.hpp` (+cpp) | `SlenderBodyGeometry/Coeffs/Config`, `slenderBodyAero(cfg, rho, cal, u_body, omega)`: crossflow strips, potential normal force, axial drag; `slenderBodyDiameterAt` |
| `flat_plate.hpp` (+cpp) | `FlatPlateCoeffs`, `FlatPlateGeom` (frame supplied by the caller), `flatPlateCn`, `flatPlateAeroAngle`, `flatPlateFlap` (force + centre of pressure; fold shield, candidate effects) |
| `tvc.hpp` (+cpp) | `TvcParams`, `tvcClamp` (NaN, magnitude, cone), `TvcState`, `tvcSlewStep`, `tvcForceFromAngles` |
| `servo.hpp` | `ServoParams`, `ServoState<N, Cap>` (dead-time line of capacity `Cap`), `servoInit`, `servoSetInput`, `servoStep` (header-only) |
| `motor.hpp` | `MotorParams`, `motorThrust`, `motorStep`: instantaneous by default (identity), optional first-order spool-up and limit (new, off, not used by the plant) |
| `ground_contact.hpp` (+cpp) | `groundContactStep(x, before)`: stop, no slide, resting body keeps its pose |

Model concept: see the comment block in `model.hpp` (types `Params, Config, State, Input, Output`, functions `configure, init, stepSize_s, maxAdvance_s, step`; Rk4 additionally `bodyParams, time_s, beginStep, forces, endStep`).

## Ship code that uses it (flight-software `src/sim/`, S3)

- `starship_aero.hpp/.cpp`: `AeroConfig`, `AeroInput/Output`, `starshipAero(cfg, in, out)` (pure, stateless, `valid=false` and zeros for non-finite input), `airVelocityBody`, `flapFrame`.
- `starship_plant.hpp/.cpp`: `StarshipPlantParams` (defaults identical to `BodySimParams`), `StarshipPlantT<kDelayCap>` with `StarshipPlant` (cap 1, vehicle) and `StarshipPlantHarness` (cap 128), `makeAeroConfig`. Explicitly instantiated in the .cpp (one instantiation per variant).
- `starship_param_sets.hpp`: `kVariants` (nominal, aero x0.7, x1.3, geometry) and `makeParams`.
- The plant takes its inputs inside `step` (clamped into the held state). A caller that wants the legacy "last valid value is kept between calls" behaviour for several `set...` calls between two steps must hold the last valid value itself (the vehicle adapter `StarshipSimAdapter` does since S4: last received thrust, last finite flap angle per channel, fed on every step).

## Behaviour notes

- `Sim::step(u, probe)` is a template over the probe type; `Sim::step(u)` uses a local `NoProbe` (no global instance, deviation from the sketch's `noProbe()`).
- The snapshot does not contain the config pointer on purpose: a rollout chooses its own config.
- `Rk4` is verified against an analytic solution (drag fall plus constant torque), not bit-exact (there is no legacy RK4).

## Tests

Run from flight-software (`pio` = `~/.platformio/penv/bin/pio`):
- `pio test -e native -f test_sim_task` (S4, flight-software `src/sim`): the vehicle adapter against the frozen legacy `BodySimulator` wiring, memcmp after every run.
- `pio test -e native -f test_dyn_rigid_body` (S1): `semiImplicitEulerStep` against the frozen `PhysicsBodySim` (1e5 random steps), `is_model`, `Sim`, RK4.
- `pio test -e native -f test_sim_equivalence` (S3, flight-software `src/sim`): the plant against the frozen `BodySimModel`, scenarios E1-E8 plus the vehicle variant, random parameter sets, the standalone aero function, parameter variants side by side, RK4 convergence.
- `pio test -e native -f test_dyn_blocks` (S2): every block against the frozen `BodySimModel` (`test/frozen/legacy_body_sim_model.hpp`, verbatim except namespace, `inline` and `private:` opened to `public:`): wind/gust tables and series, hull aero (legacy total with the flap effectiveness set to 0), `flatPlateFlap` with random geometry and candidate effects, `tvcClamp`, servo + TVC slew sequences (dead time, lag, rate limits, NaN inputs, capacity 128 and 1), ground contact. Hull and flat plate comparisons treat +0 and -0 as equal where the legacy value is a sum that starts from zero.

## Repository state (TODO for the main session)

- The library is a **local git repository** at `lib/ExVectrDynamics` (branches `master`, `Development`, feature branches per step). There is no remote and **no `.gitmodules` entry** (a URL to a missing remote would break `git submodule update`). When the owner has created `github.com/ExVeCtr/ExVectrDynamics`: add the remote, push `master`/`Development`, then `git submodule add` it in flight-software and pin the pushed commit.
- `placement.hpp` (S4): `VCTR_DYN_COLD` = `.flashmem` on the Teensy 4.x, empty on the host; every non-inline function declared in the headers carries it so the library never occupies ITCM. Inline functions and templates are inlined into the flatten functions of `src/sim` (check with `nm`: no `Dyn::` symbol below 0x60000000 on the target).
- `platformio.ini` `[env:native]` has `-Ilib/ExVectrDynamics/include` and `+<../lib/ExVectrDynamics/src/>`. Since S4 the teensy40 firmware links the library (through `src/sim`), in flash only.

## Known issues

| ID | Severity | Description |
|---|---|---|
| (none) | | No bug was found in the legacy code that these ports reproduce; observations are listed in simulation-architecture.md section 10. |
