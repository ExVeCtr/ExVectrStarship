# ExVectrDynamics

> Pure dynamics code for simulation and (later) model-based control: a rigid-body integrator, aerodynamic and actuator building blocks, and the generic model machinery. Depends on ExVectrMath only (no ExVectrCore, builds on the host). **Local repository only: the GitHub repo does not exist yet** (owner creates it, see "Repository state"). Design: [../project/simulation-architecture.md](../project/simulation-architecture.md). Status: S1 (feature/sim-s1-dynamics-skeleton).

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

Model concept: see the comment block in `model.hpp` (types `Params, Config, State, Input, Output`, functions `configure, init, stepSize_s, maxAdvance_s, step`; Rk4 additionally `bodyParams, time_s, beginStep, forces, endStep`).

## Behaviour notes

- `Sim::step(u, probe)` is a template over the probe type; `Sim::step(u)` uses a local `NoProbe` (no global instance, deviation from the sketch's `noProbe()`).
- The snapshot does not contain the config pointer on purpose: a rollout chooses its own config.
- `Rk4` is verified against an analytic solution (drag fall plus constant torque), not bit-exact (there is no legacy RK4).

## Tests

Run from flight-software: `~/.platformio/penv/bin/pio test -e native -f test_dyn_rigid_body` (S1).

## Repository state (TODO for the main session)

- The library is a **local git repository** at `lib/ExVectrDynamics` (branches `master`, `Development`, feature branches per step). There is no remote and **no `.gitmodules` entry** (a URL to a missing remote would break `git submodule update`). When the owner has created `github.com/ExVeCtr/ExVectrDynamics`: add the remote, push `master`/`Development`, then `git submodule add` it in flight-software and pin the pushed commit.
- `platformio.ini` `[env:native]` has `-Ilib/ExVectrDynamics/include` and `+<../lib/ExVectrDynamics/src/>`. The teensy40 env does not reference the library, so nothing is linked into the firmware.

## Known issues

| ID | Severity | Description |
|---|---|---|
| (none yet) | | |
