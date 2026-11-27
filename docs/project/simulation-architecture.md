# Simulation and dynamics architecture (design)

> **Status: design (2026-11-27, branch `feature/sim-architecture-design`); steps S1-S3 (host only), S4 (vehicle task, in the firmware, bench check by the owner pending) and S5 (legacy classes deleted, harnesses ported) are implemented on `feature/sim-dynamics` (section 10).** Sections 2 to 7 describe the design and the state BEFORE the steps; section 2 ("Current state") is historical since S5. The rest of this document is not implemented yet. It proposes how to rebuild the body simulator from reusable models, and how to bring the controllers into the same
> structure later. Read "Decisions for the owner" (section 9) before starting any step.
> Verified against: flight-software `Development` dc3f9f4, ExVectrDSP 5aba4c6, ExVectrControl 2c0c00e. Sizes come from
> the `teensy40` ELF of the 2026-11-26 build. Line numbers drift, so re-check them before relying on one.

## 1. Goal and stages

The owner wants a modular simulation system built from **models**:
- Models compute their results from **physics and aero building blocks**.
- **Many independent instances** can run side by side, for example parallel predictions.
- The API is clean enough that the same machinery can later serve **model predictive control (MPC)**.

| Stage | Content | This document |
|---|---|---|
| 1 | Rebuild the ship simulation on this architecture: rigid body, aerodynamics, TVC/flap/motor actuators, wind, and sensor outputs where sensible. Run the existing in-vehicle simulation mode through it with **unchanged behaviour**. | Detailed (sections 3 to 7) |
| 2 | Controllers become models with explicit state. A `{controller + plant}` model can be simulated forward. The vehicle then runs the new controller structure, **without** MPC. | High level (section 5) |
| 3 | Velocity-dependent control that uses the aero model: acceleration-loop and force/moment disturbance observers, aero feedforward, speed-scheduled limits ([../../../docs/ascent-control-design.md](../../../docs/ascent-control-design.md)). | Out of scope. The design must not block it (section 6) |

**Why the aero model must be callable outside the simulator.**
- Ascent: the nose-cone normal force acts 0.28 m ahead of the CoM. Its moment grows with airspeed² and passes the 0.256 N m
  TVC authority at about 9.5 m/s airspeed (ascent-control-design.md, section 1).
- Fast descent (RTH): tilting toward home creates an angle of attack. The resulting normal force can push the other way, so
  the position loop's assumption "tilt = lateral force" can flip sign.
- The fix needs a fast aero force/moment function that the controller and the observers can call directly. Today that
  function exists only inside the simulator, and only as a side effect of stepping it.

## 2. Current state (what exists, what is coupled, what blocks reuse)

> Historical (written before S1). Since S5 (2026-11-27) `BodySimModel`, `BodySimulator` and `PhysicsBodySim` no longer exist and the DSP/Control cycle (2.2 point 4) is gone; the frozen copies in `test/frozen/` remain as the oracle.

### 2.1 What exists

| Piece | Where | What it does |
|---|---|---|
| `BodySimModel` | `lib/ExVectrDSP/include/ExVectrDSP/body_sim_model.hpp` (313 lines), `src/body_sim_model.cpp` (449 lines) | Pure physics model: geometry, strip-theory body aero, slender-body potential force, flat-plate flaps with stall, fold shield, effective flap angle, servo and TVC slew models, wind and gusts, ground contact, fixed 2 ms step. No scheduler, topics, wall clock or random numbers. |
| `PhysicsBodySim` | `lib/ExVectrControl/include/ExVectrControl/physics_bodysim.hpp`, `src/physics_bodysim.cpp:109` | 6-DOF rigid body, semi-implicit Euler with `ω × Iω`. Accumulates force and torque sums. |
| `BodySimulator` | `lib/ExVectrDSP/include/ExVectrDSP/body_simulator.hpp` (138 lines), `src/body_simulator.cpp` (104 lines) | `Task_Periodic`, 10 ms. Reads the topics, integrates the wall-clock time since its last run (`body_simulator.cpp:53`, gaps above `kMaxGapNs` = 100 ms are dropped), applies zeroing, publishes `[ω,q]`, `[V,P]` and ground `[v_U, p_U]`. |
| `SimLidar` | `src/sim_lidar.hpp` | Sensor output: republishes the simulated height as the range reading, only below 1 m. |
| Vehicle wiring | `src/main.cpp:258` (object in DMAMEM), `:884-906` (simulation mode toggles the topic switches, resets the pose, clears the wind), `:1174` (zeroing), `:1562-1580` (wind/gust requests), `:1874-1876` (TVC/flap enable), `:2852-2856` (inputs) | The simulator reads `StarshipTVC::getTVCSimThrustTopic()` (modelled actual thrust, zero while the motor gate is closed, `src/starship_tvc.cpp:289-291`) and `trueFlapSettingTopic` (modelled true flap angles including the sim-only faults, `Safety::TrueFlapModel`). |
| Host harnesses | `test/test_sim_aero` (with `legacy_model.hpp`, the pre-redesign replica), `test_descent_guidance`, `test_descent_wiring`, `test_kick_sweep`, `test_sim_flap_fault`, `test_landing_detector` | All of them drive `BodySimModel` directly. The native env builds `body_sim_model.cpp` and `physics_bodysim.cpp` (`platformio.ini`, `[env:native]`). |

Resources (ELF, 2026-11-26):
- **Code:** `BodySimModel`, `BodySimulator` and `PhysicsBodySim` are about 11.4 KB, all in ITCM. The largest functions are
  `addFlap` 1184 B, `computeAero` 1144 B, `PhysicsBodySim::step` 952 B and `step` 656 B.
- **RAM:** the `bodySimulator` object is 3232 B in DMAMEM. 2048 B of it is the flap dead-time buffer, which the vehicle
  never uses.
- **ITCM margin:** the code is **1704 B** below the 9 × 32 KB = 294912 B ITCM boundary (codebase-structure.md, resource
  baseline). Crossing it costs 32 KB of RAM1.

### 2.2 What is coupled, and why that blocks reuse

1. **Evaluating forces mutates the model.**
   - `computeAero()` is public, but it reads the model's own rigid-body state and writes `loads_`
     (`body_sim_model.cpp:331-379`).
   - So there is no way to ask "what are the aero loads at *this* state and airspeed" without overwriting a model's
     state.
   - This is the main blocker for observers and feedforward (stage 3).
2. **Parameters, environment and state are mixed.**
   - `setWind`/`setGust` write into `params_` (`body_sim_model.cpp:109-157`), and the gust table is derived data stored
     next to the state.
   - Copying the model copies all three together: 3.2 KB per copy, most of it the delay buffer
     (`body_sim_model.hpp:278-279`).
   - Rollouts cannot share one parameter set, and they cannot choose a different environment than the vehicle's.
3. **Actuator models exist twice.**
   - The TVC thrust-angle slew is in `StarshipTVC` (`src/starship_tvc.cpp:246-267`) and again in
     `BodySimModel::updateActuators` (`body_sim_model.cpp:228-238`).
   - The flap rate limit is in `Safety::TrueFlapModel` (`src/safety/sim_flap_fault.hpp:111-129`) and again in
     `updateActuators` (`:204-226`).
   - On the vehicle the model's copy is switched off (`applyActuatorRateLimits = false`, `body_simulator.cpp:20`). The
     harness copy is a second implementation that can drift.
4. **The library dependency is circular.**
   - `body_sim_model.hpp` includes `ExVectrControl/physics_bodysim.hpp`.
   - `body_simulator.hpp:4` includes `ExVectrControl/starship/control_attitude_flaps.hpp`, a Starship type in a generic
     library.
   - ExVectrControl in turn includes `ExVectrDSP/value_covariance.hpp`.
   - The cycle is already listed as a known violation in codebase-structure.md ("Don't add more").
5. **The model is Starship-specific but lives in a generic library.** Flaps TL/TR/BL/BR, the top-flap cant, the fold
   direction and the parabolic nose are hard-coded in `BodySimModel`. The placement rule puts vehicle-specific code in
   `src/`.
6. **Controllers cannot be stepped off-line.**
   - Each controller is a `Task_Periodic` that reads `Core::NowNs()` for its own dt (`control_attitude_tvc.cpp:122`,
     `control_position_standard.cpp:117`, `control_attitude_flaps.cpp:69-74`) and talks only through topics.
   - So the host harnesses carry **replicas** of the controllers instead of the real code:
     - `test_kick_sweep.cpp:73`: `ControlAttitudeTvc` plus the `StarshipTVC` cap.
     - `test_sim_aero.cpp:399`: `FlapCtrlReplica`.
     - `test_descent_guidance.cpp:90`: the legacy flap guidance.
   - A replica proves nothing about the code that flies. The exceptions are the newer pure classes
     (`DescentGuidance`, `FlapDescentLaw`, `DescentFlapController`, `FlapModeMapper`), which already have the target
     shape.
7. **Smaller points.**
   - `attitudeState()`/`positionState()` are non-const accessors that return references into the integrator.
   - `kGravity` 9.807 is duplicated (`body_sim_model.cpp:9`).
   - `resetToStartPose()` resets neither the time nor the actuators, so the gust series continues across a reset. This is
     today's behaviour and the port keeps it.

What is already right, and is kept:
- the pure model with no clock or randomness;
- the fixed step with bounded catch-up;
- deterministic gusts from a seed;
- SI units and the NWU, `[ω,q]` and `[V,P]` conventions;
- one documented parameter struct;
- host tests with a frozen legacy replica (`legacy_model.hpp` set the precedent for golden comparison).

## 3. Constraints

| Constraint | Consequence for the design |
|---|---|
| Teensy 4.0, Cortex-M7 at 600 MHz, single- and double-precision FPU | Single-precision maths is fast. libm transcendental calls dominate the cost (section 6.3). |
| ITCM: the code is 1704 B below the 294912 B boundary | Every step reports its code delta. Code that only runs in simulation mode goes to flash (`FLASHMEM`, decision D5). Templates are instantiated once, in one `.cpp` file. |
| RAM1/RAM2 | The vehicle's simulator instance stays in DMAMEM (RAM2). Instance state is a few hundred bytes. Buffers are sized at compile time, per instance type. |
| No heap in the control loop | All state is fixed-size POD. No `new`, no containers that grow, no `std::function`. |
| float32 | float everywhere, **except** simulated time (an `int64_t` tick counter; the port keeps today's `double` accumulator first, see S3) and the gust phase arithmetic (`double`, as today, `body_sim_model.cpp:141-144`). Positions stay float: at 1 km the float resolution is 6e-5 m, and the flights stay below 200 m. |
| Deterministic, fixed step | Every model step takes `dt` as an argument. Wall-clock handling stays only in the vehicle adapter task. |
| No virtual calls on hot paths | Static composition, explained below. |
| Must compile for the host (`pio test -e native`) and for offline tools | The library depends only on ExVectrMath and `<math.h>`, with no ExVectrCore. Ship code in `src/sim/` is added to the native `build_src_filter`. |
| Many instances, copyable state, no globals, no static singletons | Each model is a stateless type whose functions take `(config, state, input)`. An instance is a small object that holds a pointer to a shared, read-only config, plus its own state. Snapshot means copying the state. |
| Toolchain: Teensy gcc 15.2 `-std=gnu++17 -fno-rtti -fno-exceptions`; native `-std=gnu++17` | C++17 is available on both: `if constexpr`, `static_assert` and the detection idiom work. C++20 concepts do not. |

**Templates vs interfaces (recommendation).**
- Use **plain static composition**: each model is a concrete type with `static` (or `const`) member functions.
  Composites call their sub-models by name.
- Generic machinery is a template over the model type: the instance `Sim<M, Integrator>`, the integrators, and the
  rollout.
- A `static_assert`-based trait (`is_model<M>`) checks the required member types and functions.
- **No CRTP and no virtual functions** in step paths. CRTP adds indirection in the code without adding anything over
  plain templates here.
- A virtual interface is allowed only at the boundary, for example a tool that selects a model at run time, and never
  inside a step.
- This follows the style already used by `DescentGuidance::step(params, target, pos, att, dt, mem)` and the ascent
  envelope design, so the codebase stays consistent.

## 4. Architecture

### 4.1 Layers and placement

```
 layer                        where                                          depends on
 ---------------------------  ---------------------------------------------  ------------------------
 L0 math                      lib/ExVectrMath                                -
 L1 building blocks (pure)    lib/ExVectrDynamics (new, D1) or ExVectrDSP     ExVectrMath
 L2 model machinery           same lib: Model trait, Sim<M,I>, integrators,  L1
                              Rollout, probes
 L3 ship models               src/sim/: StarshipPlant, StarshipAero,         L1, L2
                              parameter sets, (stage 2) controller models
 L4 vehicle adapter           src/sim/starship_sim_task.*: Task_Periodic,    L3, ExVectrCore topics
                              topics, wall clock, zeroing, wind requests
 tools / tests                test/, offline tools                           L1-L3 (never L4)
```

**Why a new library (decision D1, recommended):** `ExVectrDynamics` depends only on ExVectrMath.
- It **breaks the DSP ↔ Control cycle**:
  - `PhysicsBodySim` moves out of ExVectrControl.
  - The Starship flap type is no longer needed in a library.
- Estimators and observers (DSP) and controllers (Control) both need the building blocks. Neither of those libraries
  should own them.
- It has no ExVectrCore, so it builds for the host with no stubs.
- Alternative: keep everything in ExVectrDSP, which the placement table names for "simulation". That is cheaper (no new
  repo, submodule or doc) but keeps the cycle, and stage 3 makes Control depend on DSP's simulator code.

**Ship-specific code goes to `src/sim/` (decision D2, recommended).**
- This covers the flap layout, cant, fold direction, nose shape, the composition, the parameter sets and the vehicle task.
- That follows the CLAUDE.md placement rule.
- The precedent `lib/ExVectrControl/include/ExVectrControl/starship/` is a second option, but it puts vehicle code in a
  library.

### 4.2 Building blocks (L1)

All blocks are pure functions or small stateful step functions on POD types. Units are SI and frames are stated per
argument. The table maps each block to the current code it is ported from. The port is **bit-exact**: same operations,
same order.

| Block | Function | Ported from |
|---|---|---|
| Rigid body | `rigidBodyDeriv(p, x, wrench) → {a_world, α_body}`; `semiImplicitEulerStep(p, x, sums, dt)` | `PhysicsBodySim::step` (`physics_bodysim.cpp:109`) |
| Atmosphere | `Atmosphere::density(params, altitude) → ρ` (constant 1.2 by default; ISA is optional and changes ρ by 1 % at 120 m) | `airDensity_kgm3` |
| Wind field | `WindField::at(config, t_s) → w_world`: steady wind plus the deterministic gust sum; `buildGustTable(params) → table` | `windWorld`, `gustAt`, `buildGust` (`body_sim_model.cpp:116-172`) |
| Slender body of revolution | `bodyAero(cfg, u_body, ω_body) → Wrench`: crossflow strips with `ω × r`, potential normal force at the leading end, axial drag; radius profile from the config | `computeAero` body part (`:343-375`) |
| Flat-plate flap | `flatPlateFlap(cfg, plateFrame, u_body, ω_body, aeroAngle) → {Wrench, F}`: C_N blend, centre of pressure, shield, the optional candidate effects | `flapCn`, `flapAeroAngle`, `addFlap` (`:174-190`, `:274-329`). The plate frame (`flapFrame`, `:241-260`) is ship geometry and moves to L3 |
| Thrust / TVC | `tvcClamp(params, F_cmd) → F` (magnitude and cone limits, NaN to zero); `tvcSlewStep(params, state, F, dt)`; `thrustWrench(F, r, rollTorque)` | `setThrust` (`:80-100`), `updateActuators` TVC part (`:228-238`) |
| Servo | `ServoState<Cap>`; `servoStep(params, state, target, dt)`: dead time (capacity `Cap` fixed at compile time), rate limit, first-order lag | `updateActuators` flap part (`:204-226`) |
| Motor / propeller | `motorThrust(params, cmd) → T` (today instantaneous, as the owner stated; a first-order spool-up is a parameter, off by default) | (not modelled today) |
| Ground contact | `groundContactStep(params, x, xBefore)` (stop, no slide, a resting body keeps its pose) | `groundContact` (`:381-404`) |
| Wrench | `struct Wrench { Vec3 force; Vec3 torque; }` (body axes, about the CoM) | `BodySimLoads` |

```cpp
// lib/ExVectrDynamics/include/ExVectrDynamics/rigid_body.hpp  (sketch)
namespace VCTR::Dyn {
using Vec3 = Math::Vector<float, 3>;

struct Wrench { Vec3 force; Vec3 torque; };              // body axes, N and N m about the CoM

struct RigidBodyParams {                                 // set once; inertiaInv is derived
  float mass_kg;
  Math::Matrix<float, 3, 3> inertia, inertiaInv;          // kg m^2 about the CoM, body axes
};
struct RigidBodyState {                                  // the project layouts, unchanged
  Math::Vector<float, 7> att;                            // [ω(3) body rad/s, q(4) body->world [w,x,y,z]]
  Math::Vector<float, 6> pos;                            // [V(3), P(3)] world NWU, m/s and m
};
struct ForceSums {                                       // same split as PhysicsBodySim (bit-exact port)
  Vec3 forceBody, torqueBody;                            // N, N m, body axes
  Vec3 forceWorld;                                       // N, world axes (gravity enters as m·g here)
};

// Bit-exact port of PhysicsBodySim::step: v, then p with the new v; ω, then q·dq(ω dt); normalise; qw >= 0.
void semiImplicitEulerStep(const RigidBodyParams &p, RigidBodyState &x, const ForceSums &s, float dt);
// Continuous form for the general integrators (RK4): a_world, α_body (q̇ follows from ω).
void rigidBodyDeriv(const RigidBodyParams &p, const RigidBodyState &x, const ForceSums &s,
                    Vec3 &aWorld, Vec3 &alphaBody);
}  // namespace VCTR::Dyn
```

### 4.3 The Model concept (L2)

A **model** is a stateless type. Everything it needs comes in as arguments:

```cpp
// Required members of a model M (checked by Dyn::is_model<M>, a C++17 detection-idiom trait):
//   M::Params   POD. What a person sets (geometry, coefficients, limits). Not changed during a step.
//   M::Config   POD = { Params p; Derived d; } built by M::configure(params): derived constants (areas, strip
//               table, gust table). Read-only, and shareable by many instances.
//   M::State    POD, trivially copyable, fixed size: everything that evolves, including held inputs and
//               sub-model states. Snapshot = copy. static_assert(std::is_trivially_copyable<State>).
//   M::Input    POD, held constant over one step (zero-order hold).
//   M::Output   POD, written by every step (signals plus diagnostics such as the aero wrench).
//   static void init(const Config&, State&);
//   template <class Probe = NoProbe>
//   static void step(const Config&, State&, const Input&, float dt, Output&, Probe& = noProbe());
// Optional (needed for integrators other than the model's own):
//   static void forces(const Config&, const State&, const Input&, double t_s, Dyn::Wrench &body, Vec3 &worldForce);
```

**Composition is explicit.**
- A composite's `State` contains its sub-models' states as members, its `Config` contains their configs, and its `step`
  calls their functions in a fixed, written-out order.
- A sub-model sees only what the composite passes it. Nothing reaches into another model's state.
- A model may **use** another model in two ways, for example a controller that calls the aero model for feedforward:
  - as a pure function, with the other model's config passed by const reference;
  - or by owning that model's state as a member.
- Shared constants never come from a global. The composite's `Config` holds them once and passes them down.

**Hybrid dynamics.**
- The rigid body is continuous.
- Servos, TVC slew, dead time and ground contact are discrete. They update once per step, before or after the
  continuous integration, in the order used today (section 4.6). The step function documents this order, and the
  equivalence tests (section 7.2) pin it.

### 4.4 Instances, integrators, snapshots (L2)

```cpp
// lib/ExVectrDynamics/include/ExVectrDynamics/sim.hpp  (sketch)
namespace VCTR::Dyn {

template <class M, class Integrator = typename M::DefaultIntegrator>
class Sim {
 public:
  using Config = typename M::Config; using State = typename M::State;
  using Input = typename M::Input;   using Output = typename M::Output;

  struct Snapshot { State x; int64_t ticks; double carry_s; };   // all an instance needs to resume

  explicit Sim(const Config &cfg) : cfg_(&cfg) { reset(); }
  void reset() { M::init(*cfg_, x_); ticks_ = 0; carry_s_ = 0.0; }
  void setConfig(const Config &cfg) { cfg_ = &cfg; }              // between steps only

  template <class Probe = NoProbe>
  void step(const Input &u, Probe &pr = noProbe()) {              // one fixed step of cfg.dt
    Integrator::template step<M>(*cfg_, x_, u, dt(), y_, pr);
    ++ticks_;
  }
  int advance(float elapsed_s, const Input &u);                    // fixed steps with carry, at most maxAdvance_s

  const State &state() const { return x_; }   void setState(const State &x) { x_ = x; }
  Snapshot snapshot() const { return {x_, ticks_, carry_s_}; }
  void restore(const Snapshot &s) { x_ = s.x; ticks_ = s.ticks; carry_s_ = s.carry_s; }
  const Output &output() const { return y_; }
  double time_s() const;                                           // ticks * dt (S3 keeps the legacy accumulator)

 private:
  const Config *cfg_;  State x_;  Output y_;  int64_t ticks_ = 0;  double carry_s_ = 0.0;
};

struct SemiImplicitEuler { template <class M> static void step(/* cfg, x, u, dt, y, probe */); };  // default
struct Rk4               { template <class M> static void step(/* uses M::forces at 4 stages */); }; // rollouts
}  // namespace VCTR::Dyn
```

- **Many instances:**
  - Each `Sim` is independent. Its size is known at compile time: `sizeof(State)` plus about 32 B.
  - Instances can share one `Config`, or each use a different one (parameter variants).
  - `static_assert(sizeof(StarshipPlant::State) <= 256)` keeps the vehicle variant small (section 4.5).
- **Snapshot/restore:** a copy of `Snapshot`, with no pointers inside the state. The config pointer is not part of the
  snapshot, on purpose: a rollout chooses its own config, such as the controller's belief, a variant, or zero wind.
- **Integrator choice (decision D3):**
  - **Keep semi-implicit Euler at 2 ms** as the default and for the vehicle simulator:
    - It is what runs today, so the port can be bit-exact.
    - It costs one force evaluation per step.
    - It is symplectic, which keeps rotation and energy well behaved.
    - 2 ms is far below the fastest dynamics. These are rough estimates from the model's coefficients at 6.4 m/s,
      cal 4.64: the potential-force pitch stiffness is about 1 N m/rad on 0.044 kg m², so about 5 rad/s, and the strip
      damping is about 1.4 1/s. The 300°/s servos are handled by the discrete servo block.
  - **Add RK4** (and optionally Heun) for rollouts with larger steps (10-20 ms). Euler loses accuracy there, and RK4 at
    10 ms costs 4 evaluations per 10 ms instead of 5 per 10 ms.
  - Discrete parts stay zero-order-hold within an RK4 step.
  - A convergence test (RK4 at 10 ms against Euler at 0.5 ms) decides which step sizes are accepted for rollouts.
- **Reset** is `M::init`. The vehicle's `resetToStartPose()` keeps today's semantics: pose only, while time and actuators
  continue. It becomes `StarshipPlant::resetPose(state)`.

### 4.5 The Starship plant (L3, `src/sim/starship_plant.hpp/.cpp`)

```cpp
namespace VCTR::ShipSim {

struct FlapPairGeometry { float width_m, height_m, chamfer_m, bottomFromShipBottom_m, effectiveness, cant_rad; };

struct StarshipPlantParams {             // == today's BodySimParams, regrouped; defaults identical
  Dyn::RigidBodyParams body;             // 1.1 kg, I = diag(0.044, 0.044, 0.004)
  Dyn::SlenderBodyParams hull;           // R 0.065, tube 0.54, parabolic nose 0.20, CoM 0.37 from the bottom, strips 24
  Dyn::BodyAeroCoeffs hullAero;          // Cdc 1.2, eta 0.65, potential 1.0, Cd axial 0.9 / 0.35
  FlapPairGeometry flapTop, flapBottom;  // owner geometry; top cant 10 deg
  Dyn::FlatPlateCoeffs flapAero;         // C_N_alpha 3.0, C_N90 1.17, stall 30/5 deg, effective angle, shield 0.7
  bool flapFoldTowardBelly = false;
  float hingeRadius_m = 0.065f;
  float aeroCalibration = 4.64f;         // global scale on every aero force and moment (Known issue, ExVectrDSP.md)
  Dyn::AtmosphereParams air;             // rho 1.2
  Dyn::WindParams wind;                  // speed, direction, gust strength/period/seed/spread
  Dyn::TvcParams tvc;                    // 14 N, 3 deg, z = -0.35, slew 300/15 deg/s
  Dyn::ServoParams flapServo;            // 300 deg/s, lag 0, dead time 0, limits 0..90 deg
  bool applyActuatorModels = true;       // false on the vehicle: the drivers already publish actual values
  float step_s = 0.002f, maxAdvance_s = 0.05f;
};

template <int kDelayCap>                 // 1 on the vehicle (no dead time), 128 in the harness
struct StarshipPlantT {
  using Params = StarshipPlantParams;
  struct Derived { /* z positions, base area, flap areas and centroids, gust table */ };
  struct Config  { Params p; Derived d; };
  struct State {
    Dyn::RigidBodyState body;
    float flapCmd[4];                     // last valid flap input (a non-finite input keeps it, as today)
    Vec3 thrustCmd; float thrustMag, rollTorque; bool thrustEnabled;  // last clamped thrust input
    Dyn::ServoState<4, kDelayCap> flaps;  // delay line, rate-limited and lagged angles
    float tvcAngle[2];                    // slewed thrust angles
    double t_s;                           // S3: legacy accumulator; later int64 ticks
  };
  struct Input  { Vec3 thrustBody; float rollTorque; bool thrustEnabled; float flapAngle[4]; };
  struct Output { Dyn::Wrench aero; Vec3 flapForce[4]; Vec3 airVelBody; float flapAngle[4]; float tvcAngle[2]; };
  using DefaultIntegrator = Dyn::SemiImplicitEuler;

  static Config configure(const Params &p);
  static void init(const Config &c, State &x);                 // start pose, flaps folded (pi/2)
  static void resetPose(State &x);                             // == BodySimModel::resetToStartPose
  static void forces(const Config &c, const State &x, const Input &u, double t_s, Dyn::Wrench &body, Vec3 &world);
  template <class Probe = Dyn::NoProbe>
  static void step(const Config &c, State &x, const Input &u, float dt, Output &y, Probe &pr = Dyn::noProbe());
};
using StarshipPlant        = StarshipPlantT<1>;    // vehicle
using StarshipPlantHarness = StarshipPlantT<128>;  // host harness with servo dead time
}  // namespace VCTR::ShipSim
```

- **Size:** the vehicle `State` is about 160 B (rigid body 52, held inputs 36, servos 52 with a delay of 1, TVC 8,
  time 8). The `Config` is about 450 B (params about 260, derived about 60, gust table 128). The vehicle instance shrinks
  from 3232 B to about 0.8 KB together with the task, topics and config. The harness variant adds the 2 KB delay line.
- **Input sanitising stays bit-identical:**
  - `setThrust` clamps and replaces NaN with zero.
  - `setFlapInput` ignores non-finite values and clamps.
  - The port does both in `step` from `Input`, into the held fields in `State`.
  - The vehicle adapter still feeds "last received" values, as `BodySimulator::update` does (`body_simulator.cpp:52-81`).

### 4.6 Step order (must equal today's `BodySimModel::step`, `body_sim_model.cpp:406-428`)

1. Take the inputs: clamp them into the held fields.
2. Actuators: flap servos (delay line, rate limit, lag) and TVC slew. When `applyActuatorModels` is false the targets are
   passed through, as today.
3. Air velocity at the CoM: `u0 = qᵀ(w(t) − v)`, using the time **before** the increment.
4. Aero:
   1. hull strips;
   2. potential force;
   3. axial drag;
   4. the four flaps, in order TL, TR, BL, BR.

   Each force is accumulated in exactly the current order, because float addition is not associative.
5. Force sums:
   1. gravity as m·g into the world sum;
   2. thrust at z = `tvcFromCg_m`, then the roll torque (only while enabled);
   3. then the aero force and torque.
6. `semiImplicitEulerStep`, then ground contact with the state from before the step.
7. `t_s += double(dt)`.

### 4.7 Parameter sets as data (`src/sim/starship_param_sets.hpp`)

```cpp
struct ParamVariant { const char *name; float aeroScale; bool geometryAero; };
constexpr ParamVariant kVariants[] = {
  {"nominal", 1.0f, false},   // aeroCalibration 4.64
  {"aero x0.7", 0.7f, false}, {"aero x1.3", 1.3f, false},
  {"geometry", 1.0f, true},   // aeroCalibration 1.0
};
StarshipPlantParams makeParams(const ParamVariant &v, StarshipPlantParams base = StarshipPlantParams());
```

- A test matrix loops over `kVariants × scenarios × seeds`.
- Each variant has its own `Config`, and all of them can step side by side.
- The ascent study's 6 variants (ascent-control-design.md, section 4) become rows of this table. Their exact definitions
  are taken from that harness when S5 ports it.

### 4.8 Probes and logging (zero cost when off)

- `step` takes a template `Probe&`. The default `NoProbe` has empty inline member functions, so the compiler removes the
  calls.
- A probe sees intermediate values: each strip's force, each flap's α and centre of pressure, the per-component wrenches.
  The CSV writer of `test_sim_aero` (`SIM_AERO_CSV`) becomes a probe.
- Values that are cheap and always useful (total aero wrench, flap forces, air velocity) go into `Output`, as
  `BodySimLoads` does today.
- On the vehicle, only `NoProbe` is instantiated.

### 4.9 Sensors as outputs

- **Stage 1 keeps the perfect-state outputs** (`[ω,q]`, `[V,P]`, ground `[v_U, p_U]`) and `SimLidar`.
- `simLidarReading` becomes an output function of the plant composition (`src/sim/`), with unchanged behaviour.
- **Sensor models** (IMU, baro, GNSS, magnetometer, with noise from a seeded generator held in the state) are a natural
  next model type. But feeding them to the real estimators would change sim/flight difference (1) in overview.md: the
  estimators would run in simulation too.
- That is a safety-rule-7 change and the owner's call (decision D6). The architecture allows it without changes: a
  sensor model is just another model that takes the plant state.

### 4.10 Vehicle adapter (L4, safety-relevant)

`StarshipSimTask` (`src/sim/starship_sim_task.hpp/.cpp`) replaces `DSP::BodySimulator` with **identical external
behaviour**:
- **Same:**
  - 10 ms period, wall-clock elapsed, `kMaxGapNs` 100 ms gap rule, bounded catch-up;
  - same topics and timestamps;
  - `enableTVC`/`enableFlaps`/`enableZeroingMode`, `resetToStartPose`, `setWind`/`setGust`/`clearWind` and the getters
    used by `main.cpp`;
  - the same inputs: the gated sim-thrust topic and the true flap angles.
- **Not changed:** the motor gate (`Safety::gateSimulatedThrust` in `StarshipTVC`), the flap faults (`TrueFlapModel`) and
  the topic switches all stay where they are.
- **Mechanics:**
  - The task owns its `Config` and changes it only between steps; a wind request rebuilds the gust table.
  - It owns one `Dyn::Sim<StarshipPlant>`.
  - The plant code is compiled `FLASHMEM`/`flatten` (decision D5) because it runs only in simulation mode.

## 5. Control as a model (stage 2, high level)

### 5.1 Shape

- A controller becomes a model in the sense of section 4.3:
  - `Params`: gains and limits.
  - `State`: integrators, filter memories, previous outputs, mode memory.
  - `Input`: the estimate snapshot, setpoints or the command, enables, and `dt`.
  - `Output`: actuator commands plus status.
- The **core** is a pure step function.
- The **task** keeps only the adapter work:
  - subscribe;
  - measure dt with **exactly the current clamp**;
  - call the core;
  - publish.
- This is the pattern already proven by `DescentFlapController` and `FlapModeMapper`. `test_descent_unit` asserts
  bit-identical outputs against a verbatim copy of the old code.

### 5.2 Mapping of today's chain

| Today (task, rate) | State to make explicit | Core input → output |
|---|---|---|
| `ControlPositionStandard` (20 ms, `control_position_standard.cpp:117`) | `velCtrlIntegral_`, setpoint hold, `enableControl_` | `[V,P]` estimate, `[V,P]` setpoint, dt → accel setpoint (world) |
| `ControlMappingAccToAtt` (event, on accel publish) | none | accel setpoint → attitude setpoint `[ω,q]` |
| `ControlAttitudeTvc` (20 ms, `control_attitude_tvc.cpp:122`) | `integral_`, the last outputs | `[ω,q]` estimate, attitude setpoint, accel setpoint, dt → TVC `[F_body, T]` |
| `StarshipTVC` output model (20 ms, `src/starship_tvc.cpp:246-291`) | slewed thrust angles, twist | TVC command, enables, gate → actual thrust (telemetry) and sim thrust (gated) |
| `ControlAttitudeFlaps` (20 ms) = `FlapModeMapper` + `DescentFlapController` (already pure) | `DescentGuidanceMem`, mode memory | setting, `[ω,q]`, `[V,P]`, target, dt → four flap angles |
| `StarshipFlaps` / `TrueFlapModel` (20 ms) | true angles, faults | commanded angles, dt → true angles |
| Missions (10-100 ms) / later the flight executive | phase state | stay **outside** the controller model. The executive is already designed as a pure `step(ExecInputs) → ControlCommand` (flight-executive-design.md 2.3), so it can join a rollout later. |

### 5.3 `{controller + plant}` composition

```cpp
// src/sim/closed_loop.hpp (stage 2 sketch)
struct ClosedLoop {
  struct State  { ShipCtrl::Cascade::State ctrl; ShipCtrl::Actuators::State act; ShipSim::StarshipPlant::State plant; };
  struct Config { const ShipCtrl::Cascade::Config *ctrl; const ShipCtrl::Actuators::Config *act;
                  const ShipSim::StarshipPlant::Config *plant; float ctrlDt_s; /* 0.02 */ };
  struct Input  { ShipCtrl::Command cmd; };   // setpoints and modes (later: a ControlCommand subset)
  static void step(const Config &c, State &x, const Input &u, float /*= ctrlDt*/, Output &y) {
    const Estimate est = perfectEstimate(x.plant);              // or an estimator model (D6)
    const ActuatorCmd a = ShipCtrl::Cascade::step(*c.ctrl, x.ctrl, est, u.cmd, c.ctrlDt_s);
    const PlantInput pu = ShipCtrl::Actuators::step(*c.act, x.act, a, c.ctrlDt_s);     // TVC slew, true flaps
    for (int i = 0; i < substeps(c); ++i) ShipSim::StarshipPlant::step(*c.plant, x.plant, pu, c.plant->p.step_s, y.plant);
  }
};
```

- **One source of truth:** the vehicle tasks and the rollout call the **same cores**. Test replicas are deleted.
- **What a rollout leaves out:** the vehicle's inter-task latency. Each task runs on its own 20 ms phase, and topics pass
  values with up to one tick of delay. A composite runs the cascade in sequence, with no delay. An optional one-tick
  `Delay<T>` block per edge reproduces the on-vehicle timing where a test needs it. This affects how well a rollout
  matches the vehicle. It does not affect safety.

### 5.4 First refactor steps and risks

- **First steps:**
  1. Extract the `ControlAttitudeTvc` core. It is the smallest and is used by the kick (`test_kick_sweep` replica).
  2. Then position + mapping.
  3. Then the `StarshipTVC` output model and `TrueFlapModel` as actuator models.
  4. Then replace the replicas in the harnesses.
- Each extraction is asserted **bit-identical** on the host against a verbatim copy of the pre-change code, for scripted
  input sequences that include dt jitter, enable toggles, resets and NaN.
- **Risks:**
  - These are **flight controllers**. A mistake changes real flight, not only simulation.
  - The dt handling moves into the adapter: today's `lastRunTimestamp_` and `resetControlState` semantics must be kept
    exactly.
  - Callers that reach into controllers directly must keep working, for example `MissionBellyflop` holding a
    `ControlAttitudeTvc&` (`main.cpp:303`). The adapter keeps the public API, so they do.
- **Classification:** every stage 2 step that reaches the vehicle is **flight-safety**: it needs a simulation bench check
  and owner approval, even when the host test is bit-identical.

### 5.5 Real-time budget of a rollout (estimate, to be measured in S6)

- **Today's control tick:** the controllers run every 20 ms (50 Hz). The scheduler is cooperative and runs one task per
  tick (codebase-structure.md "Runtime model"). The IMU EKF and position KF tasks run at 1 ms, so **any single task call
  longer than about 1 ms delays the estimators**.
- **Rollout cost:** `C(N) = N · (C_ctrl + (T_ctrl / dt_plant) · C_plant)`.
- **Assumed costs:**
  - `C_plant` ≈ 12-14 µs per 2 ms step without gusts (section 6.3), plus 5-10 µs with gusts.
  - `C_ctrl` ≈ 5-10 µs.

| Plant in the rollout | Per 20 ms step | N = 25 (0.5 s) | N = 50 (1 s) | N = 100 (2 s) |
|---|---|---|---|---|
| Full plant, Euler 2 ms | ≈ 140 µs | 3.5 ms | 7 ms | 14 ms |
| Reduced (8 strips, RK4 10 ms) | ≈ 60 µs | 1.5 ms | 3 ms | 6 ms |

- **Consequences:**
  - On the Teensy, one or two rollouts with a horizon of about 1 s per 20 ms tick are the ceiling.
  - Sampling or gradient MPC with many rollouts is a host-only tool.
  - On the vehicle a rollout must be **time-sliced**: a resumable `Rollout` object advances a bounded number of steps per
    call (≤ 0.3-0.5 ms) and resumes on the next. A copyable state makes this trivial.
  - The total CPU load of today's firmware is not documented. It must be measured (DWT cycle counter) before any
    on-vehicle rollout is planned (risk R5).

## 6. The aero model as a standalone callable

### 6.1 Interface

```cpp
// src/sim/starship_aero.hpp (ship composition of the L1 hull and flat-plate blocks)
namespace VCTR::ShipSim {

struct AeroScales {                 // all multiplicative; a default of 1.0 is exact in IEEE, so bit-identical
  float global = 4.64f;             // == aeroCalibration (nominal 4.64, geometry 1.0)
  float hullCrossflow = 1.0f;       // multiplies Cdc*eta
  float hullPotential = 1.0f;       // == potentialFactor (the nose normal force: the ascent moment)
  float hullAxial = 1.0f;           // new; multiplies both axial Cd
  float flapTop = 1.0f, flapBottom = 1.0f;  // == FlapPairGeometry::effectiveness
};

struct AeroInput {
  Vec3 airVelBody;                  // m/s: air velocity relative to the CoM, body axes = R(q)^T (w_world - v_world)
  Vec3 omegaBody;                   // rad/s, body axes
  float flapAngle[4];               // TL, TR, BL, BR true servo angles, rad (0 extended, pi/2 folded toward +X)
  float rho;                        // kg/m^3
};
struct AeroOutput {
  Dyn::Wrench total;                // body axes, N and N m about the CoM
  Dyn::Wrench hull, flaps;          // split for observers and fits
  Vec3 flapForce[4];
  float alpha_rad;                  // hull angle of attack (0 axial, pi/2 broadside)
  bool valid;                       // false if an input was non-finite; then all outputs are 0
};

void starshipAero(const AeroConfig &cfg, const AeroInput &in, AeroOutput &out);   // pure, no state
Vec3 airVelocityBody(const Math::Quat_F &qBodyToWorld, const Vec3 &vWorld, const Vec3 &windWorld);
}  // namespace VCTR::ShipSim
```

- **Frames and units:** NWU world, q `[w,x,y,z]` body → world, body axes +Z nose, belly −X, TL/BL on +Y, origin at the
  CoM, SI.
- **One function for everyone.** The plant's step calls exactly this function, so the simulator and the controller never
  disagree about the model. They differ only in the **config they pass**: the simulated truth, the controller's belief,
  or a variant (decision D7).
- **Total function:**
  - Finite inputs give finite outputs.
  - Non-finite inputs give `valid = false` and zero outputs.
  - No loop count depends on the input (the strip count is in the config).
  - No static state.
  - This matters once controllers call it in flight (stage 3).
- **Calibration:**
  - The existing knobs (`aeroCalibration`, `potentialFactor`, `effectiveness`) are regrouped into `AeroScales`. Only
    `hullAxial` is new.
  - A stage 3 fit can scale, for example, the nose normal force alone (ascent-control-design.md A6 wants the observed
    moment fitted against `u_ax·u_cross`).
  - Changing a scale is a config change: rebuild with `configure`.
- **Fidelity knob:** `hull.strips` (4..64; 24 today). A reduced controller config (8 strips) is validated against 24
  strips across the flight envelope before anyone uses it (acceptance in S6).
- **Derivatives:** central differences (2 evaluations per input dimension) are enough for observers and linearisation.
  Analytic Jacobians are not designed here.

### 6.2 Worked example

```cpp
using namespace VCTR;
// 1) Aero query, for example from a controller or observer (stage 3), no simulator involved.
ShipSim::AeroConfig belief = ShipSim::makeAeroConfig(ShipSim::StarshipPlantParams());   // nominal, 4.64
ShipSim::AeroInput in;
in.airVelBody = ShipSim::airVelocityBody(qEst, vEst, Math::Vector<float,3>{0, 0, 0});   // still-air belief
in.omegaBody  = omegaEst;
for (int i = 0; i < 4; ++i) in.flapAngle[i] = trueFlap[i];
in.rho = 1.2f;
ShipSim::AeroOutput a;
ShipSim::starshipAero(belief, in, a);
// a.total.torque is the predicted aero moment (N m, body); with r x F_tvc it gives the moment balance.

// 2) Two parallel predictions from the vehicle's current simulated state (host tool or test).
auto pLo = ShipSim::makeParams(ShipSim::kVariants[1]);   // aero x0.7
auto pHi = ShipSim::makeParams(ShipSim::kVariants[2]);   // aero x1.3
const auto cLo = ShipSim::StarshipPlant::configure(pLo), cHi = ShipSim::StarshipPlant::configure(pHi);
Dyn::Sim<ShipSim::StarshipPlant> lo(cLo), hi(cHi);
lo.restore(vehicleSim.snapshot());  hi.restore(vehicleSim.snapshot());
ShipSim::StarshipPlant::Input u = lastInput;                              // held for the horizon
for (int k = 0; k < 500; ++k) { lo.step(u); hi.step(u); }                  // 1 s at 2 ms
const float dz = hi.state().body.pos(5) - lo.state().body.pos(5);          // spread of the altitude prediction
```

### 6.3 Cost (estimate from operation counts; measure in S6 with the DWT cycle counter)

| Part (one evaluation) | Operations | Cycles (estimate) |
|---|---|---|
| Hull, 24 strips | 24 × (1 sqrt + about 20 flops + radius branch) | ≈ 900 |
| Potential + axial | atan2f, sinf, cosf, 2 sqrt | ≈ 400 |
| 4 flaps | each: 4 sin/cos (frame), asinf, sinf, expf, 2 sin/cos (C_N), cosf (cp), 2-3 sqrt, about 150 flops | ≈ 4400 |
| Rotation of the air velocity, sums | about 60 flops | ≈ 100 |
| **Aero total** | | **≈ 5800 (≈ 10 µs)** |
| Plant step extra: rigid body (2 quaternion rotations, 3 × 3 products, axis-angle sin/cos, normalisation), actuators | | ≈ 1500 |
| Gusts (16 double `sin`, only when the strength is above 0) | | ≈ 3000-6000 |

- **Plant step:** ≈ 12-14 µs without gusts, ≈ 20-25 µs with them.
- **Today's simulation load:** 5 steps per 10 ms run, ≈ 0.6-1.3 % CPU.
- **Reduced aero** (8 strips): ≈ 9 µs. Most of the cost is the flaps' transcendental functions.
- **Further savings:** for a controller config, the flap frame sines and cosines can be cached per flap angle (the angles
  change at 50 Hz, the evaluations happen more often). This is a later optimisation with a tolerance test, never part of
  the bit-exact port.

## 7. Migration plan

### 7.1 Steps

Every step is a feature branch that can be merged on its own. The flight-software branch name is
`feature/sim-arch-s<n>`, with a branch of the same name in each touched submodule.

| Step | Content | Repo / branch | Acceptance | Teensy code impact | Vehicle sim mode? |
|---|---|---|---|---|---|
| **S1** | New library skeleton (if D1): `types`, `Wrench`, `RigidBodyState`, `semiImplicitEulerStep`, `rigidBodyDeriv`, `is_model`, `Sim<M,I>`, `NoProbe`. Native env paths added. Library doc `docs/libs/ExVectrDynamics.md`. | ExVectrDynamics (new repo, owner creates it) + flight-software (submodule, `platformio.ini`) | Native test: `semiImplicitEulerStep` is **bit-identical** to `PhysicsBodySim::step` over 10⁵ random force/torque/dt sequences; `Sim` snapshot/restore round trip; `is_model` rejects an incomplete type | 0 (not linked) | no |
| **S2** | Building blocks, bit-exact ports: wind/gust, hull aero, flat plate, TVC clamp/slew, servo `ServoState<Cap>`, ground contact, atmosphere, motor (off) | ExVectrDynamics | One native test per block against the legacy function extracted verbatim (frozen copy in the test), bit-identical over swept inputs, including NaN and edge cases (flow along the axis, speed below 1e-3, flap at 0/90°) | 0 | no |
| **S3** | `StarshipPlantT`, `configure`, `forces`, `step`, `advance`, parameter sets, `starshipAero` in `src/sim/`. New golden test `test/test_sim_equivalence` against a **frozen copy** of `BodySimModel`/`PhysicsBodySim` (namespace `legacy`, from DSP 5aba4c6 / Control 2c0c00e) | flight-software | Scenarios E1-E8 (section 7.2): **bit-identical** state, flap angles, TVC angles, loads and time after every step, on the host | 0 (not linked yet) | no |
| **S4** | `StarshipSimTask` replaces `DSP::BodySimulator` in `main.cpp`; plant code `FLASHMEM`/`flatten` (D5) | flight-software | Build without new `src/` warnings; ITCM below the boundary with the margin reported; native suite once; **simulation bench check** (section 7.3) | expected −8..−11 KB ITCM if D5 (plant in flash); ±1 KB otherwise, **which would cross the boundary at the 1704 B margin**. RAM2 −2.4 KB | **yes: safety-relevant, bench before merge** |
| **S5** | Port the harnesses (`test_sim_aero`, `test_descent_guidance`, `test_descent_wiring`, `test_kick_sweep`, `test_sim_flap_fault`, `test_landing_detector`) to `StarshipPlantHarness`/`StarshipPlant`; delete `BodySimModel`, `BodySimulator` (DSP) and `PhysicsBodySim` (Control); update codebase-structure.md (the cycle is gone), ExVectrDSP.md, ExVectrControl.md, overview.md | ExVectrDSP, ExVectrControl, flight-software | Every ported harness prints the **same** report lines (`SIMAERO`, `DESCENT`, `WIRING`, `KICK`) as before (diff of the `-v` output is empty); its assertions unchanged; native suite once | ≈ 0 (already unlinked since S4) | build only; short bench smoke (sim on, one hover profile) |
| **S6** | Aero API made public and documented: `AeroScales`, `valid` flag, reduced config. A bench-only timing build measures `starshipAero` and `StarshipPlant::step` with DWT (full, reduced, gusts) | flight-software | Host: `starshipAero` equals the plant's internal loads bit for bit; reduced (8 strips) within 5 % of the force and 10 % of the moment of 24 strips over the envelope (airspeed 0-15 m/s, α 0-180°, flaps 0-90°), else D7 picks 24. Measured cycle table replaces section 6.3 | 0 in the flight build | no |
| **S7a** | Stage 2: `ControlAttitudeTvc` core + adapter | ExVectrControl, flight-software | Bit-identical against a verbatim copy (scripted sequences with dt jitter, reset, enable, NaN); `test_kick_sweep` uses the real core; native suite once | ±0.5 KB | **flight-safety: bench (kick, hover, RTH) + owner approval** |
| **S7b** | Position + acc-to-att cores | ExVectrControl, flight-software | as S7a | ±0.5 KB | **flight-safety** |
| **S7c** | `StarshipTVC` output model and `TrueFlapModel` on the Dynamics servo/TVC blocks (D8); the plant's actuator models become the same code | flight-software | Bit-identical telemetry thrust and true angles against a verbatim copy, with faults; `test_sim_flap_fault` unchanged | ±0.5 KB | **flight-safety (actuator output models, telemetry)** |
| **S8** | `ClosedLoop` and a time-sliced `Rollout` in `src/sim/`; the closed-loop harnesses run the real cores | flight-software | `test_descent_wiring` and `test_kick_sweep` metrics reproduced with the real cores (differences explained by the removed replica deviations); rollout slice ≤ 0.5 ms measured on the bench | 0 unless used on the vehicle | no |
| S9 | (later) Vehicle control tick through the controller model, aligned with the flight executive (D9). Needs its own design | - | - | - | flight-safety |

### 7.2 Equivalence test strategy

- **Golden reference:** a frozen copy of the current model sources goes into the test folder (precedent:
  `test/test_sim_aero/legacy_model.hpp`). The legacy and new models are compiled in the same binary with the same flags
  and stepped with the same scripted inputs.
- **Scenarios (host, deterministic):**

| ID | Scenario | Exercises |
|---|---|---|
| E1 | Upright, thrust = m·g, flaps folded, 10 s | rigid body, thrust, axial aero |
| E2 | Scripted open-loop ascent, wind 3 m/s + gusts 3 m/s (seed 7), 10 s | wind frame, gusts, potential force at high α |
| E3 | Belly-down fall from 120 m at 50/40° with pitch/yaw offsets, until resting on the ground | flaps, shield, effective angle, ground contact (arrival and resting) |
| E4 | Flap sweeps 0-90°, out-of-range and NaN inputs mixed in | input hold, clamps |
| E5 | Thrust above 14 N, beyond 3°, NaN, enable toggles | TVC clamp and gating |
| E6 | `advance` with 0, negative, 1, 9.7, 49, 80 ms and NaN elapsed; `clearTimeCarry`; `resetPose` mid-run | carry, bounded catch-up, reset semantics |
| E7 | Harness mode: rate limits on, lag 20 ms, dead time 30 ms | servo and TVC slew models |
| E8 | `setWind`/`setGust`/`clearWind` mid-run | runtime environment changes |

- **Criterion:**
  - **Bit-identical on the host** (memcmp of the state after every step) for S1-S4 and S7.
  - It is not claimed on the target: gcc may contract `a*b+c` into single-precision FMA on the M7, differently for
    restructured code.
  - The target check is behavioural (section 7.3).
  - If a step must change the operation order (for example caching), it is a **separate** step with a tolerance:
    - per-step relative state error ≤ 1e-5 over 1 s;
    - over 20 s, summary metrics instead of trajectories, because the flap limit cycle amplifies small differences:
      - terminal speed within 0.5 %;
      - trim pitch within 0.2°;
      - the `SIMAERO` table values within their printed precision.
- **Optional replay:** ground-station recordings of bench simulations (`ACTUATORS` with the true flap angles and the
  thrust) replayed open loop into both models.
- **Tests that must stay green at every step:**
  - `test_sim_aero`, `test_descent_guidance`, `test_descent_wiring`, `test_kick_sweep`, `test_sim_flap_fault`,
    `test_landing_detector`, `test_descent_unit`, `test_mission_chain`;
  - the full native suite once per step, at the end (testing policy).

### 7.3 Simulation bench check for S4 (and the stage 2 steps)

Motors electrically disconnected (safety.md rule 9). Use the same firmware day, the old and the new build, and the same
procedure (development-workflow.md, simulation mission):

1. Profile 2 full chain: waypoint → bellyflop → freefall → RTH → landing, in calm air, and then in 3 m/s wind with
   3 m/s gusts.
2. Flap fault: TL stuck at 42° in the descent.
3. Reset and modes:
   1. SE reset puts the body at the start pose.
   2. Zeroing holds the body.
   3. The SC toggle clears the wind.
4. Landing by the simulated lidar.

**Accept:**
- same phase sequence;
- no failure bits and no `non-finite` warnings;
- kick duration within ±0.05 s of the old build;
- fall speed within ±0.2 m/s;
- landing reached.

Wall-clock jitter makes the runs non-identical. That is why the host golden test (S3) carries the exactness.

## 8. Risks and what is deliberately not done

| ID | Risk | Mitigation |
|---|---|---|
| R1 | ITCM margin of 1704 B: any growth crosses the bank boundary (−32 KB RAM1) | S4 moves the plant to flash in the same step (D5). Every step reports the code delta. |
| R2 | Template bloat (one instantiation per model × integrator × probe) | Instantiate in one `.cpp` per model, only `NoProbe` on the vehicle, and check sizes with `nm` |
| R3 | Behaviour drift in the simulation mode, which is safety-relevant because it is the verification tool | Bit-exact host golden (S3), bench check (S4), no physics changes in this work |
| R4 | Host/target float differences (FMA) | The host carries exactness and the target carries behaviour (section 7.2) |
| R5 | Rollouts blocking the cooperative scheduler (1 ms estimator tasks) | Time-sliced rollouts, measured budget, no on-vehicle rollout before the CPU load is measured |
| R6 | Stage 2 touches flight controllers | Bit-identical cores against verbatim copies, one controller per step, bench + owner approval for each |
| R7 | The model's aero calibration is unexplained (4.64 against 1.0, ExVectrDSP.md Known issues, High) | Any stage 3 law must hold over the variants of section 4.7. The controller uses its own config (D7) |
| R8 | New repo and pins: never pin an unpushed submodule commit on `Development` | S1 waits for the owner to create and push the repo (D1) |
| R9 | Two simulators exist during S3-S5 | The legacy code exists only as a frozen test fixture after S5. S4 and S5 follow each other closely |

**Deliberately not done here:**
- no MPC;
- no control-law design;
- no change of any coefficient, geometry or physical effect;
- no sensor noise;
- no estimators in the simulation loop;
- no flight executive work;
- no packet or ground-station change;
- no change to the motor gate, the flap fault guards or the sim/flight difference list.

## 9. Decisions for the owner

| # | Decision | Options | Recommendation |
|---|---|---|---|
| D1 | Where the building blocks and the model machinery live | (a) new `ExVectrDynamics` library (Math only); (b) ExVectrDSP | **(a)**: removes the DSP ↔ Control cycle, has no Core dependency, and serves DSP and Control equally. Needs a new GitHub repo created by the owner. |
| D2 | Where the Starship composition (plant, aero, parameter sets, task, later the closed loop) lives | (a) `src/sim/`; (b) `lib/.../starship/` in a library | **(a)**: the placement rule |
| D3 | Integrator | (a) semi-implicit Euler at 2 ms for the vehicle and default, plus RK4 for rollouts; (b) RK4 everywhere | **(a)**: bit-exact port, cheapest; RK4 where large steps are needed |
| D4 | Equivalence criterion for the port | (a) bit-identical on the host, behavioural on the bench; (b) tolerance only | **(a)**: makes "behaviour-equivalent" checkable instead of arguable |
| D5 | Plant code in flash (`FLASHMEM`/`flatten`) | (a) yes, simulation-only code leaves ITCM; (b) keep it in ITCM | **(a)**: frees about 10 KB of ITCM at a margin of 1704 B. It changes only simulation-mode CPU time (flash with cache), never the real-mode paths |
| D6 | Sensors as outputs | (a) keep the perfect-state outputs in stage 1, add sensor models later as a separate decision; (b) add IMU/baro/GNSS models feeding the real estimators now | **(a)**: (b) changes the sim/flight difference list (safety rule 7). Worth deciding later, because it would make the simulation exercise the estimators |
| D7 | Which aero config a controller or observer uses | (a) its own "belief" config (nominal by default), separate from the simulated truth; (b) always the simulator's config | **(a)**: otherwise a simulation can never reveal model mismatch. The ×0.7/×1.3/geometry variants then test robustness |
| D8 | Unify the actuator models (`StarshipTVC` slew, `TrueFlapModel`) with the Dynamics blocks | (a) in stage 2 (S7c); (b) never, keep both | **(a)**, after the controller cores, as its own flight-safety step |
| D9 | Stage 2 order relative to the paused flight executive (E3-E7) | (a) extract the controller cores first (S7), and the executive consumes them; (b) wait for the executive | **(a)**: the cores are needed by both, and they shrink the executive's adapter |
| D10 | On-vehicle rollouts (later) | (a) time-sliced, ≤ 0.5 ms per call, after a CPU-load measurement; (b) a dedicated long task | **(a)**: a long task delays the 1 ms estimators |

## 10. Implementation status

Branches: integration `feature/sim-dynamics`; step branches `feature/sim-s<n>-...` cut from it and merged back with `--no-ff`. The library branches carry the same step names. Legacy code is untouched; bugs found in it are recorded, not fixed, on these branches (the new code reproduces them bit for bit).

| Step | State | Branch | Commits |
|---|---|---|---|
| S1 | done (host only) | `feature/sim-s1-dynamics-skeleton` (flight-software and lib/ExVectrDynamics) | see "Hashes" below |
| S2 | done (host only) | `feature/sim-s2-building-blocks` (flight-software and lib/ExVectrDynamics) | see "Hashes" below |
| S3 | done (host only) | `feature/sim-s3-starship-plant` (flight-software and lib/ExVectrDynamics) | see "Hashes" below |
| S4 | done on the host and in the build; **owner bench check pending (section 7.3)** | `feature/sim-s4-vehicle-task` (flight-software and lib/ExVectrDynamics) | see "Hashes" below |
| S5 | done on the host and in the build (no flash, no bench use); short bench smoke (sim on, one hover profile) still wanted | `feature/sim-s5-remove-legacy-sim` (flight-software, lib/ExVectrDSP, lib/ExVectrControl) | see "Hashes" below |

**S1 (library skeleton).**
- `lib/ExVectrDynamics` created as a local git repository (no remote, no `.gitmodules` entry yet, see [libs/ExVectrDynamics.md](../libs/ExVectrDynamics.md)).
- Contents: `types`, `rigid_body` (`semiImplicitEulerStep`, `rigidBodyDeriv`, `addForce`/`addTorque`), `model` (`is_model`), `integrators` (`SemiImplicitEuler`, `Rk4`), `sim` (`Sim<M,I>`).
- Test `test/test_dyn_rigid_body`: 1e5 random steps (random sums, inertia with products, dt including 0/negative/NaN/inf) bit-identical (memcmp after every step) against the frozen `PhysicsBodySim` in `test/frozen/legacy_physics_bodysim.hpp`; `is_model` positive and negative (static_assert); snapshot/restore; `advance` rule; RK4 against an analytic solution.
- Deviations from the sketch: `ForceSums` also has `torqueWorld` (needed for a bit-exact port); no global `noProbe()` (a local `NoProbe` is used); the model concept needs `stepSize_s`/`maxAdvance_s`, and `forces(...)` takes the `Output&` (so diagnostics are filled without a second evaluation); the frozen copy omits `simulateForTime` (needs ExVectrCore).

**S2 (building blocks).**
- Added to `lib/ExVectrDynamics`: `atmosphere`, `wind`, `slender_body`, `flat_plate`, `tvc`, `servo`, `motor`, `ground_contact`, all bit-exact ports (same operations, same order).
- Test `test/test_dyn_blocks` against the frozen `test/frozen/legacy_body_sim_model.hpp` (a whole-class frozen copy so each legacy member function can be called individually; the copy is header-only and has its `private:` opened for the tests).
- Deviations from the design table: no ISA atmosphere (constant density, as today); `thrustWrench` is not a function (the plant adds thrust force and roll torque through `addForce`/`addTorque` so the torque sum keeps the legacy order); `flatPlateFlap` returns the force and the centre of pressure (the caller forms `cp x F` and accumulates) and the plate frame (`flapFrame`) stays vehicle geometry for S3; the hull block takes `rho` and the calibration separately and computes `k = 0.5 rho cal` itself, as the legacy code did; `motor` is new code with no legacy counterpart (tested for identity by default).

**S3 (Starship plant).**
- `src/sim/`: `starship_aero.*`, `starship_plant.*`, `starship_param_sets.hpp`. Legacy `BodySimModel`/`BodySimulator`/`PhysicsBodySim` are untouched and the vehicle task still uses them; nothing in `src/` references `src/sim/`.
- `platformio.ini`: `[env:native]` builds `src/sim/` and the library; `[env:teensy40]` has `-<sim/>` in `build_src_filter` (src is compiled with `+<*>`, so without the filter the new files were linked in: the first S3 build grew by +320 B code through static initialisers of the ExVectrMath constants header). **S4 removes the `-<sim/>` filter** when the task references the plant.
- Test `test/test_sim_equivalence`: E1-E8 plus the vehicle variant (capacity-1 servo line, pass-through actuators), the default parameters without a converter, 12 random parameter sets, the standalone aero function against the legacy loads, parameter variants side by side, snapshot/restore, RK4 against a fine Euler. Criterion: memcmp of attitude/position state, flap angles (true, rate-limited, held input), TVC angles, held thrust, aero force/torque, per-flap forces, air velocity and time after every step. A mutation check (gravity 9.807 -> 9.8071) fails at step 1.
- Teensy build (S3, `pio run -e teensy40`): FLASH code 308348 B, RAM1 variables 68768 B, code 293208 B, padding 1704 B (margin to 294912 B unchanged), 160608 B free: identical to the baseline above. `nm` shows no `ShipSim` or `Dyn` symbol.
- Sizes (host, `-v | grep SIMSIZE`): `StarshipPlant::State` 168 B, harness state 2200 B, `Config` 636 B, `Sim<StarshipPlant>` 304 B.
- Deviations from the design: (1) `Params` holds mass and inertia entries (as `BodySimParams`), the derived `RigidBodyParams` is in `Derived`; (2) `AeroScales` is not introduced yet (S6): the calibration is `aeroCalibration`/`AeroConfig::calibration`, the other knobs are the existing coefficients; (3) `starshipAero` returns zeros with `valid=false` for non-finite input, where the legacy model propagated NaN (only reachable with an already non-finite state; all equivalence scenarios use finite states and sanitised inputs); (4) `evalSums`/`beginStep`/`endStep` split the step so the same code serves `Sim` (Euler, bit-exact) and the RK4 integrator; (5) the Input is taken inside `step`; two `set...` calls between steps keep only the last, the adapter must hold the last valid value (see the library doc); (6) the harness `advance` is `Sim::advance` (same carry rule) and compares equal including the carry; (7) `tvcAngle`/`flapAngle` outputs are copied from the state after the actuators.
- Observations on the legacy code (no bug confirmed, none fixed, all reproduced): `clearWind` does not reset `gustSpread`; `resetToStartPose` leaves time and actuators (documented); the flap dead-time line is primed with the first input rather than the initial angle; the TVC angles keep their last value while the commanded thrust is below 1e-6 N (so a stale slewed angle is used when thrust returns, before the slew moves it); the gust phase uses double `sin` (16 calls per evaluation, cost on the target, not a defect).

**S4 (vehicle task, safety-relevant).**
- `StarshipSimTask` (`src/sim/starship_sim_task.*`, `Task_Periodic` shell: topics, scheduler, `Core::NowNs()`) replaces `DSP::BodySimulator` in `main.cpp` (same object name `bodySimulator`, same DMAMEM placement, same public API). All logic is in `StarshipSimAdapter` (`src/sim/starship_sim_adapter.*`, no Core dependency, so it is host tested): it owns the plant `Config` and one `Dyn::Sim<StarshipPlant>`, holds the inputs, applies the legacy timing rules (wall-clock elapsed, `kMaxGapNs` 100 ms, carry, bounded catch-up via `Sim::advance`), zeroing mode, `resetToStartPose` (pose only, clears the carry), wind/gust, and the vehicle's `configureTvc` (limits and the thrust point z).
- **Input holding (the S3 deviation 5):** the plant takes its input inside `step`. The adapter feeds, on every step, the last RECEIVED thrust vector and roll torque (a non-finite thrust is zero thrust until the next item, as in the legacy model, which held the raw value and clamped it each run), the current TVC enable, and the last FINITE flap angle per channel (NaN at start: the plant keeps its folded start value). Two flap items between two runs therefore keep the last valid value per channel, as `BodySimModel::setFlapInput` did.
- Same, unchanged and outside this task: topics (`getStateEstTopic`, `getAttitudeEstTopic`, `getGroundTopic`), the 10 ms period, timestamps, `SimLidar` (`src/sim_lidar.hpp`, untouched), the motor gate (`Safety::gateSimulatedThrust` in `StarshipTVC`), the flap faults (`TrueFlapModel`), the topic switches.
- Code placement (D5): `SIM_COLD` (`src/sim/sim_placement.hpp`) = `FLASHMEM` + `flatten` for the functions of `src/sim`; member functions of the plant class template use `SIM_COLD_T` (section `.flashmem.t`: GCC reports a section type conflict when a template and a plain function share the section name; the linker script takes `*(.flashmem*)`). The non-inline functions of ExVectrDynamics carry `VCTR_DYN_COLD` (new `placement.hpp`; `.flashmem` under `__IMXRT1062__`, empty on the host; a library cannot include `src/flashmem.hpp`). Includes inside `src/sim/` are relative (`"starship_plant.hpp"`), because the teensy40 build has no `-Isrc` for headers in sub-folders.
- `platformio.ini`: the `-<sim/>` filter is removed from `[env:teensy40]`; `[env:native]` builds `sim/` except `starship_sim_task.cpp` (needs the scheduler). The legacy `BodySimModel`, `BodySimulator` and `PhysicsBodySim` stayed in the libraries until S5 for the frozen tests and harnesses and were NOT linked into the firmware (checked with `nm`).
- Test `test/test_sim_task`: the adapter against a frozen copy of the legacy `BodySimulator` wiring (constructor, `setTVCInputTopic`, `resetToStartPose`, `update`) driving the frozen `BodySimModel`; memcmp after every run of attitude, position, ground pair, timestamp, flap angles, time, wind/gust getters, the lidar reading (and the held flap input after every run that stepped). Scenarios: gate closed/open, thrust below 1e-6 N, lift-off, thrust cut, fall and landing with the lidar valid below 1 m and absent above, flap inputs (stuck 42 deg, out of range, NaN/inf per channel, none yet received), last valid input held (two items between two runs, all-NaN items, NaN/inf thrust), wind/gust mid-run (NaN, clamps, clearWind), reset and zeroing while moving, timing (jitter, zero and negative elapsed, 99/100/101/250 ms gaps, first run after start), 24 random seeds x 1500 runs mixing everything. A mutation (gap rule 100 ms -> 90 ms) fails two tests.
- Sizes (Teensy build): see the table in the S4 report in codebase-structure.md "Resource baseline": code 293208 -> 283464 B (-9744), padding 1704 -> 11448 B, RAM1 variables +96 B, RAM2 -1856 B (the simulator object 3232 -> 1352 B), FLASH code +7868 B. The `bodySimulator` object (task shell, adapter 1040 B on the host: config 636, `Sim` 304) is 1352 B in DMAMEM on the target (`nm`), was 3232 B.
- Deviations from the design text: the task is split into adapter and shell (the design had one class) so that the logic is host-testable; `starship_sim_task.*` holds only the shell; the legacy getters `getAttitudeState`/`getPositionState` are `const`; `setAttitudeState`/`setPositionState` and `model()` are not provided (unused by the vehicle; the legacy `model()` handle is gone, parameters are set through `configureTvc` only).
- Not provable on the host: (1) the target's FMA contraction and flash-cache timing: `StarshipPlant::step` costs more time in flash than in ITCM (cache misses; the budget is 5 steps per 10 ms run, design 6.3 estimates 0.6-1.3 % CPU in ITCM) so check the loop timing and that the task is not starved in simulation mode; (2) the real scheduler/topic order (the shell is thin: `isDataNew` -> adapter -> `NowNs`), covered by the bench check below.
- Observations on the original code (not fixed, no new bug): the legacy `BodySimulator::update` calls `setThrust` and `setFlapInput` in every run even when no step follows (the held fields are unobservable then, the adapter does it in the step; the test compares the held input only after a run that stepped); the adapter keeps the legacy quirk that a non-finite thrust item is held raw (zero thrust until the next item) while flap channels hold the last finite value.

**S5 (remove the legacy simulator).**
- Harnesses ported to the plant: `test_sim_aero`, `test_descent_guidance`, `test_descent_wiring`, `test_kick_sweep`, `test_sim_flap_fault` (the only tests that used the legacy classes besides the frozen-copy tests; `test_landing_detector` and `test_ground_rest` never used them and are unchanged). They use `test/support/plant_model.hpp` (`plant_test::PlantModel`): a header-only handle on one `Dyn::Sim<StarshipPlantHarness>` with its own `Config`, the held inputs (thrust as given, a flap channel keeps its last finite value, as the vehicle adapter does) and the calls the harnesses used (`setThrust`, `setFlapInput`, `step`, `advance`, state getters, `loads()`, `flapPlate`, `computeAero`, ...). It adds no physics: every call forwards to `src/sim` or ExVectrDynamics. Parameters are `StarshipPlantParams` (nested members: `bodyStrips` -> `hull.strips`, `flapFoldShieldFactor` -> `flapAero.foldShieldFactor`, `windSpeed_mps` -> `wind.speed_mps`, `applyActuatorRateLimits` -> `applyActuatorModels`, `topFlapCantRad` -> `flapTop.cant_rad`, ...). The test bodies and all assertions are unchanged (only type and field names, includes and four header comments differ). `test_sim_aero` keeps its own pre-redesign replica `legacy_model.hpp` (it never used the deleted classes).
- Acceptance: the `-v` output of every ported harness was captured before the port and after; the report lines (`SIMAERO`, `DESCENT`, `WIRING`, `KICK`, flap-fault results) are identical, the only differences are the run times in the Unity summary.
- Deleted: ExVectrDSP `body_sim_model.*`, `body_simulator.*`; ExVectrControl `physics_bodysim.*`; the two source lines of them in `[env:native]`. The frozen copies `test/frozen/legacy_body_sim_model.hpp` and `legacy_physics_bodysim.hpp` stay and `test_sim_equivalence`, `test_sim_task`, `test_dyn_*` use only them (D4).
- The DSP/Control include cycle is gone (checked by grep over `lib/`: nothing in ExVectrDSP, ExVectrDynamics or ExVectrMath includes `ExVectrControl/...`; Control includes only `ExVectrDSP/value_covariance.hpp`). The Starship-specific `ExVectrControl/starship/` code still lives in the generic library.
- Docs updated: codebase-structure.md, libs/ExVectrDSP.md, libs/ExVectrControl.md, libs/ExVectrDynamics.md, project/overview.md, README.md, hardware-and-drivers.md, missions.md. The simulator model description in ExVectrDSP.md ("Body simulator model") and its Known issues rows are kept (they still describe `StarshipPlant`), with a note that file references point to removed code.

### Hashes

- flight-software, merge commits on `feature/sim-dynamics`: S1 `2b1d2c9`, S2 `4e7b227`, S3 `36eefe7`.
- lib/ExVectrDynamics (local repo, branch `Development`), merge commits: 7e39b34 Merge feature/sim-s2-building-blocks, 5441750 Merge feature/sim-s1-dynamics-skeleton. Library commits (no merges, newest first): cda07e0 d50ad90 5e95cdf. S3 changed no library file (no S3 library branch).
- S4: flight-software branch commit 63e6406, merge commit on `feature/sim-dynamics` a0acef0. lib/ExVectrDynamics: commit 9a5b0ae on `feature/sim-s4-vehicle-task`, merged into the library `Development` as 42e675c (not pushed when written).
- S5: flight-software branch commits 7d3c14f (code, harnesses, pins) and the docs commit on `feature/sim-s5-remove-legacy-sim`, merged into `feature/sim-dynamics` (merge commit in `git log`). lib/ExVectrDSP: 7043ab8 on `feature/sim-s5-remove-legacy-sim` (cut from the pinned 5aba4c6 on `feature/t9b-4`; NOT merged into the library `Development`, which is behind that branch). lib/ExVectrControl: 5575b74 on `feature/sim-s5-remove-legacy-sim`, merged into the library `Development` as 70e4541 (the old pin 2c0c00e was the `Development` tip). None of these library commits is pushed.
