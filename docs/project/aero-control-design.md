# Aerodynamics-aware position and velocity control (stage 3, design)

> **Status: design, 2026-11-27, branch `feature/aero-control-design` (cut from `feature/sim-dynamics` 79b3cee).
> Nothing here is implemented and nothing changes flight behaviour.** The only code on this branch is the report-only
> host study `test/test_aero_control_study` that produced the numbers in section 2.
> Reproduce: `pio test -e native -f test_aero_control_study -v | grep AEROCTL` (about 1 s).
> All numbers come from the simulation model (`src/sim/starship_aero.hpp`, the function the simulator uses; 24 strips,
> `aeroCalibration` 4.64 unless marked). That model is **not validated** for the upright vehicle (ascent-control-design.md
> section 8, R7 in simulation-architecture.md). Read section 8 (open decisions) before starting a step.

Related documents: [simulation-architecture.md](simulation-architecture.md) (stages, aero API, stage 2 steps S7a-c),
workspace `docs/ascent-control-design.md` (ascent governor R, deferred decisions), workspace
`docs/descent-control-design.md` (flap descent), [missions.md](missions.md) (RTH sub-states, controller phase table),
workspace `docs/safety.md` (rules 1, 7, 9).

## 1. Summary

- **What goes wrong.** The cascade position → velocity → acceleration → tilt assumes that a tilt θ gives a lateral
  acceleration b·θ with b = T/m (≈ 9.8-12.7 m/s² per rad) and no other lateral force. The aero model says:
  - **Ascent:** b *grows* with airspeed (×1.1 at a calm 5 m/s climb, ×1.6-2.6 at 5-10 m/s climbs in 3-5 m/s wind),
    a lateral aero force of 2-8 N appears with the wind, and the nose moment uses up the 0.256 N m TVC authority
    (0.32 N m at a 5 m/s climb in 5 m/s wind, with the vehicle upright). The outer loop gets a higher gain and an
    inner loop that cannot follow, so position holding degrades with speed while hover is fine.
  - **Fast descent (tail first):** b *shrinks* and **changes sign**. With folded flaps the flip is at a descent speed
    of 18 m/s in calm air, but already at **9.8 m/s with 3 m/s sideways airspeed** and 6.8 m/s with 6 m/s. With the
    RTH "Upright" flaps it flips at 9.5 m/s even without sideways speed. Above the flip a tilt toward home pushes the
    vehicle *away* from home: the position loop becomes positive feedback. Across the ×0.7…×1.3 variants the flip
    moves by about ±20 %. The geometry model (cal 1.0) puts it above 20 m/s.
  - **Fast descent, attitude:** with folded flaps the tail-first potential force is a destabilising moment that equals
    Kp = 1.2 N m/rad at 6.8 m/s. With sideways airspeed the trim moment alone is beyond the TVC authority (0.36 N m at
    4 m/s down and 3 m/s sideways). The extended top flaps (Upright) make the descent attitude holdable.
- **A disturbance observer alone cannot fix a sign flip.** Section 2.6 shows that an additive force observer is stable
  only while the true tilt-to-force gain has the same sign as the assumed one. It cancels the slowly varying aero force
  (wind, drag, thrust shortfall) and restores tracking for any b of the right sign, but near and beyond the flip it
  diverges with the loop it is meant to fix. So the three agreed parts have distinct jobs:
  1. **Observers** (force: world 3-axis; moment: body x/y) estimate the unmodelled aero effect online, model-free,
     from signals that exist identically in simulation and flight.
  2. **Speed-scheduled limits** keep the vehicle out of the regions where the cascade's assumptions fail: a tilt limit
     and a **lateral-authority fade** that, in fast tail-first flow, stops steering by tilt (the vehicle stays upright,
     brakes vertically, and lets drag brake the horizontal motion) until the airspeed is back below the flip region.
  3. **Aero feedforward** from the validated model (after flight validation) predicts the aero force and, more
     importantly, the true gain b̂ and its sign, so the vehicle can keep lateral authority at speed. It is gated by a
     runtime confidence check (model vs observer residual) and a validated envelope; tilt *inversion* (deliberately
     tilting away from home to go home) stays disabled until the owner approves it.
- **Placement.** One new Starship stage between the position controller's acceleration topic and its two consumers
  (`ControlMappingAccToAtt`, `ControlAttitudeTvc`), plus a moment observer and an optional torque-feedforward input in
  `ControlAttitudeTvc`. Mode 0 is a bit-identical pass-through, so the flashed behaviour does not change until the
  owner sets the mode. Same code in simulation and flight (D8): observers read the switched estimate topics and the
  gated modelled thrust, never the simulation flag.
- **Budget.** All new code in flash (`FLASHMEM`, about 6-9 KB flash in total, ITCM ≤ +0.3 KB of call sites against
  11384 B padding). Observers ≈ 2-4 µs per 20 ms tick. Feedforward: 3-7 aero calls per tick at an **assumed 10-20 µs
  per call** (the DWT measurement is pending) = 30-140 µs per 20 ms, below the 1 ms single-call limit.

## 2. Physical model: what goes wrong (host study)

### 2.1 What the cascade assumes (code as read)

| Stage | Where | Assumption that matters here |
|---|---|---|
| Position → velocity → acceleration | `ControlPositionStandard` (ExVectrControl, 50 Hz) | P 0.5 / 2.0, velocity PI 2.2/0.2 (horizontal), 2.5/0.8 (vertical); P never clamped; output `a_cmd = P + I + (0,0,g)`, world. No aero term. |
| Acceleration → attitude | `ControlMappingAccToAtt` (event callback on the accel topic) | Body +Z points along `a_cmd`, clipped at 35°: "tilt direction = lateral force direction". |
| Attitude → TVC | `ControlAttitudeTvc` (50 Hz) | τ = 1.2·e + 0.5·(ω_sp − ω) N m; thrust = m·|a_cmd|·cos∠, limit compensation on (`main.cpp`). The external moment is treated as zero. |
| TVC output | `StarshipTVC` | 14 N, 3° cone, authority A = 14·sin 3°·0.35 = 0.256 N m. Publishes the modelled actual thrust (telemetry) and a **gated** copy (zero while the motor gate is closed), today read only by the simulator. |

Linearised about the trim, the lateral channel the velocity loop sees is a = F₀/m + b·θ with
**b = (T·cos θ + ∂F_x/∂θ)/m**. The cascade designs for b = T/m and F₀ = 0.

### 2.2 Method

`test/test_aero_control_study`: the public `starshipAero` (validated config path, `makeScaledAeroConfig`), body tilted
by θ about world +Y (θ > 0: nose toward +X; NWU), zero body rate, central differences over ±1°. Variants: nominal
(cal 4.64), ×0.7, ×1.3, geometry (cal 1.0), pot0.22 (potential force at geometry level, the ascent study's variant).
Flaps: folded (90°: Ascent mode, RTH translation and descent) or Upright (top 0°, bottom 90°: RTH stabilising).
Columns: `Fx0` lateral aero force at θ = 0; `b14`/`b11` the gain b for T = 14 N / 10.8 N (hover); `M0` pitch moment at
θ = 0 (+ turns the nose toward +X); `dM` its slope (> 0 destabilising; compare Kp = 1.2 N m/rad); `hold` = A/dM, the
tilt offset the TVC can still hold.

### 2.3 Ascent (folded flaps, wind blowing toward −X, the position hold tilts into it, θ > 0)

| Climb (m/s) / wind (m/s) | Fx0 (N) | b14 (m/s²/rad) | b14 / (T/m) | M0 (N m) | dM (N m/rad) |
|---|---|---|---|---|---|
| 0 / 0 (hover) | 0 | 12.7 | 1.00 | 0 | 0 |
| 5 / 0 | 0 | 14.2 | 1.11 | 0 | 0.48 |
| 10 / 0 | 0 | 18.5 | 1.45 | 0 | 1.92 |
| 0 / 3 | −1.7 | 12.7 | 1.00 | +0.05 | −0.15 |
| 5 / 3 | −2.8 | 18.5 | 1.45 | −0.24 | +0.10 |
| 10 / 3 | −3.9 | 27.7 | 2.18 | −0.55 | +1.44 |
| 5 / 5 | −6.5 | 20.7 | 1.63 | **−0.32** (> A) | −0.39 |
| 10 / 5 | −8.4 | 33.2 | 2.61 | **−0.84** | +0.78 |

Speeds (calm): dM = Kp at **7.9 m/s** (×1.3: 6.9, ×0.7: 9.5, geometry 17.1, pot0.22 19.4); hold = 5° at 12.4 m/s.
b never flips in the ascent (the crossflow and potential forces add to the thrust's lateral force).

What this means for the position loop:
1. **Loop gain rises with speed.** The horizontal velocity loop (P 2.2 1/s) runs at about 2.2 rad/s in hover. At a 5-10
   m/s climb in 3-5 m/s wind its crossover moves to 3.2-5.7 rad/s, onto the attitude loop (ωn ≈ 5.2 rad/s) and its
   delays (TVC slew 3° in 0.15 s, one tick of phasing, 40 ms position latency). Loss of separation means overshoot and
   the slow coning seen in flight6 (ascent-control-design.md section 3) becomes more likely.
2. **A lateral force offset** grows with airspeed (crossflow drag ∝ |u|·u_cross): −1.7 N in hover, −3.9 N at a
   10 m/s climb in the same 3 m/s wind. The velocity integrator (0.2 1/s², limit 3 m/s) is too slow to follow it
   during a 20-30 s climb.
3. **The inner loop cannot follow.** The trim nose moment uses up the TVC authority (−0.32 N m at 5/5 already exceeds
   A), so the commanded tilt is not reached (the "phase 1" of the ascent study), the outer loop winds up and the static
   divergence of the ascent study follows above about 8-10 m/s airspeed.

Points 1 and 2 are what the observers and feedforward address. Point 3 is an authority limit: no outer-loop law can
create TVC moment. It is handled by the speed limits (and, deferred, by the ascent governor R of the ascent study).

### 2.4 Fast descent, tail first (calm air, v = (v_h, 0, −V_d))

θ > 0 tilts toward the horizontal velocity; a braking tilt (toward home when flying away from it) is θ < 0.

**Sign flip of the lateral gain**, speed where b14 = 0 (b11 = 0 in brackets), by sideways airspeed v_h:

| Variant | folded, v_h 0 | folded, v_h 3 | folded, v_h 6 | Upright, v_h 0 | Upright, v_h 3 |
|---|---|---|---|---|---|
| nominal | 18.1 (15.9) | ≈ 9.8 | ≈ 6.8 | ≈ 9.5 | ≈ 8.3 |
| ×0.7 | 21.6 (19.0) | ≈ 12.7 | ≈ 8.9 | - | - |
| ×1.3 | 15.9 (13.9) | ≈ 8.0 | ≈ 5.6 | - | - |
| geometry | 39.0 (34.2) | > 20 | ≈ 21 | - | - |
| pot0.22 | none < 40 | | | - | - |

(Interpolated from the 2 m/s grid in the `AEROCTL DESC` rows; Upright was computed for the nominal model only.)

Large-angle check, nominal, V_d = 16 m/s, v_h = 3 m/s (away from home), T = 14 N: lateral acceleration
a_x(θ) = (T sin θ + F_x)/m, m/s²:

| θ | −35° | −20° | −10° | 0° | +10° | +20° | +35° |
|---|---|---|---|---|---|---|---|
| nominal | +0.5 | −3.3 | −3.8 | −4.8 | −7.8 | −11.8 | −16.2 |
| geometry | −5.6 | −4.1 | −2.6 | −1.0 | +0.1 | +0.9 | +2.2 |

In the nominal model, tilting toward home (θ < 0) *reduces* the braking toward home and at −35° reverses it; tilting
away from home brakes harder. This is the owner's observation: "the controller thinks it must tilt toward the home
point, but the aero normal force then pushes the other way". In the geometry model the same command works as designed.
The truth is somewhere in this band, which is why the design must work in both worlds.

**Attitude (moment) in tail-first flow, nominal:**

| V_d / v_h (m/s) | folded: M0, dM, hold | Upright: M0, dM, hold |
|---|---|---|
| 6 / 0 | 0.00, 0.95, 15.5° | −0.06, 0.11, 128° |
| 10 / 0 | −0.01, 2.6, 5.6° | −0.16, 0.32, 46° |
| 16 / 0 | −0.02, 6.7, 2.2° | −0.40, 0.81, 18° |
| 2 / 3 | **+0.20 (77 % of A)**, −0.09 | +0.09, −0.15 |
| 4 / 3 | **+0.36 (> A)**, 0.27 | +0.18, 0.34, 43° |
| 16 / 3 | **+1.35 (5×A)**, 7.2, 2.0° | −0.17, 1.9, 7.7° |

- With folded flaps, dM = Kp at **6.8 m/s** descent (×1.3: 5.9, ×0.7: 8.1, geometry 14.6). The tail-first potential
  force acts at the base, 0.37 m behind the CoM, a longer lever than the nose's 0.28 m in the ascent.
- With sideways airspeed the trim moment alone exceeds the TVC authority from about 4 m/s down. **RTH translation
  (folded flaps, at most 2 m/s along a 3-D line) in 3 m/s wind already needs about 77 % of the authority in the
  nominal model** (geometry 17 %).
- The extended top flaps sit at the trailing end in tail-first flow and act as fins: the Upright configuration keeps
  the moment within authority up to the 16 m/s of the freefall end. This is why the bench RTH start from freefall
  (16 m/s down, missions.md section 5) recovers. The same flaps lower the lateral gain, so the flip comes earlier.

### 2.5 Where the problem shows in today's chain

- **RTH Stabilize** (after freefall, after an abort; Upright flaps; setpoint = measured position, zero velocity, so a
  pure velocity damper). The vertical demand 2.5·V_d + g saturates the thrust and keeps the tilt small
  (atan(2.2 v_h / (2.5 V_d + g)) ≈ 7-11°). The negative b then only weakens the braking, because the drag
  (−7.6 N at 10/3 Upright) dominates. In the model this phase stays stable, but the tilt it commands is useless or
  harmful above the flip.
- **Any lateral position control while descending fast through the air** (a faster RTH translation, an RTH started
  from a fast descent, a mission descent leg, a tail-first descent in wind): the position P term commands a tilt
  toward the target with a modest vertical demand, so the tilt is large and b < 0 makes the loop diverge until the
  speed drops. With folded flaps the attitude itself is at the authority limit there.
- **Ascent at 5-10 m/s, mainly in wind**: section 2.3 points 1-3.

### 2.6 What a disturbance observer can and cannot do

Lateral channel, small angles: the vehicle obeys a = F₀/m + b·θ. The controller with an additive observer commands
θ = (a_c − d̂)/b̂, and the observer filters d̂' = β·(a_meas − b̂·θ − d̂). Substituting:

```text
d̂' = β·( F₀/m + (b/b̂ − 1)·a_c − (b/b̂)·d̂ )     → stable iff b/b̂ > 0, pole at −β·b/b̂
steady state: a = a_c exactly, for any b of the same sign as b̂
```

- **Ascent** (b/b̂ = 1…2.6): tracking is restored and the force offset is cancelled, but the observer pole moves up to
  2.6β. With b̂ from the model the loop gain itself can be normalised as well (section 3.6).
- **Descent before the flip** (0 < b/b̂ < 1): works, but the observer slows down as b/b̂ → 0 and the loop has no
  lateral authority left.
- **Beyond the flip** (b/b̂ < 0): the observer is unstable together with the loop. **No observer bandwidth fixes this.**
  Either the sign is known (model feedforward, section 3.6) or the vehicle must not steer by tilt there
  (lateral-authority fade, section 3.5).
- The same argument holds for the moment observer inside the attitude loop: it cancels a slowly varying external moment
  up to the TVC authority; it cannot create authority.

### 2.7 Caveats

- The upright aerodynamics are unvalidated; flight5 and flight6 suggest cal 1-3 for the nose moment (ascent study
  section 3), i.e. between the geometry and the ×0.7 rows. The flip may not exist in the reachable speed range at all.
  The design therefore must be safe in the geometry world (do not remove authority it does not need to) and in the ×1.3
  world (never steer into a flipped gain).
- Not modelled: propeller normal force and inflow at angle of attack, prop wash over the flaps, roll/yaw coupling,
  ground effect, baro/GNSS lag at speed. The propeller is likely the largest missing term in the ascent (it adds a
  stabilising or destabilising normal force at the tail; sign unknown).

## 3. Control architecture

### 3.1 Overview

```text
                                   [V,P] est (switched)   [ω,q] est (switched)   gated thrust (StarshipTVC)   true flaps
                                          │                     │                      │                      │
 mission setpoint ─► ControlPositionStandard ─► a_cmd ─► ┌───────▼─────────────────────▼──────────────────────▼──────┐
                       (unchanged)                       │ AeroAccelStage (src/control, NEW, event-driven on a_cmd)  │
                                                         │  1 force observer   d̂ (world, 3)                          │
                                                         │  2 airspeed, u_ax, u_cross, tail/nose-first               │
                                                         │  3 aero feedforward F̂_A, b̂, confidence (mode bit FF)     │
                                                         │  4 compose a_des = a_cmd − sat(d̂) − w_ff·F̂_A/m           │
                                                         │  5 lateral gain normalisation / authority fade w_lat      │
                                                         │  6 speed-scheduled tilt limit θ_max(V, usage)             │
                                                         └───────────────┬────────────────────────────┬─────────────┘
                                                                  a_thr (world)                  a_thr (world)
                                                                         ▼                            ▼
                                                          ControlMappingAccToAtt          ControlAttitudeTvc (thrust
                                                          (unchanged, 35° backstop)       magnitude and body force)
                                                                         │ attitude sp                ▲
                                                                         └────────────────────────────┘
 [ω,q] est + gated thrust ─► MomentObserver (NEW) ─► M̂ (body x,y) ─► τ_ff ─► ControlAttitudeTvc (optional input, NEW)
                                                          └─► usage = |M̂|/A ─► θ_max schedule; (deferred) ascent governor R
```

- **Mode 0 (default): the stage forwards `a_cmd` unchanged, in the same callback** (no arithmetic on the vector, so it
  is bit-identical including the timing of the event chain), and the torque-feedforward input of `ControlAttitudeTvc`
  is not subscribed (no addition, not even of zero: `x + 0.0f` turns −0 into +0).
- The thrust magnitude stays in `ControlAttitudeTvc` (`m·|a|·cos∠`); because the stage outputs the **thrust**
  acceleration a_thr (the aero part removed), the thrust magnitude automatically includes the aero axial force.
- Nothing upstream (missions, executive, position controller) changes. The ascent study's shaping (a_z bounds, tilt
  30°) and vertical command cap naturally live in the same stage (section 3.9).

### 3.2 Placement and the same-code rule (D8)

| Part | Where | Why |
|---|---|---|
| `ForceObserver`, `MomentObserver` (generic: thrust-acceleration and moment observers with disturbance states) | `lib/ExVectrControl/include/ExVectrControl/disturbance_observer.hpp`, header-only, pure (`Params/State/Input/Output`, `step`) | Reusable, no vehicle geometry, host-testable. Library includes nothing from `src/`. |
| `AeroAccelStage` core (pure) | `src/control/aero_accel_stage.{hpp,cpp}` | Calls `ShipSim::starshipAero` (`src/sim/`), vehicle-specific: the placement rule puts it in `src/`. |
| Stage task shell (callback, topics, dt) | `src/control/aero_accel_task.{hpp,cpp}` | Same split as `StarshipSimTask`: logic host-tested, shell thin. |
| Torque feedforward input | `ControlAttitudeTvc` (ExVectrControl): `subscribeTorqueFeedforward(topic)`; added to `attCtrlOutput` x/y before the force conversion, only while subscribed and fresh | Minimal hook; after S7a it is a field of the core's `Input`. |

- **Signals used** (all identical in simulation and flight, no simulation-flag read anywhere):
  - velocity and attitude/ω from `positionTopicSwitch`/`attitudeTopicSwitch` (estimator in flight, plant in
    simulation, the existing documented sim/flight difference);
  - thrust from `StarshipTVC::getTVCSimThrustTopic()`, the modelled actual thrust that is **zero while the motor gate is
    closed**. Its name says "sim" but it is computed in both modes; step A5 adds an alias `getTVCGatedThrustTopic()` so
    the flight code does not read a "sim" name. Using the ungated telemetry topic would make the observers see a
    phantom −T/m disturbance whenever the gate is closed;
  - true flap angles from `trueFlapSettingTopic` (the modelled true angles; in flight the commanded angles after the
    rate model).
- **Not used:** the raw accelerometer. It would be the best acceleration signal in flight, but in simulation mode the
  sensors are not simulated, so an accelerometer-based observer would read the bench at rest (D8 violation). A
  simulated specific-force topic would be a new entry on the sim/flight difference list (safety rule 7, owner
  decision, open decision 6). The design uses the velocity estimate through an observer (no explicit differentiation).

### 3.3 Force observer (translational, world frame)

Model per axis (world): v̇ = a_T + a_A,ff + g⃗ + d, with a_T = R(q)·F_thr,gated/m and a_A,ff the applied aero
feedforward (0 unless mode bit FF). Observer states **ẑ (3, m/s), d̂ (3, m/s²)**; critically damped double pole at −β_F:

```text
e   = v_est − ẑ
ẑ  += dt·( a_T + w_ff·â_A + g⃗ + d̂ + 2β_F·e )
d̂  += dt·( β_F²·e )
```

- **Bandwidth:** β_F = 1/τ_F with τ_F = 1.0 s (design value; range 0.3-3 s, set by A4). Reasons: the estimator velocity
  noise of the studies (0.2 m/s, 0.3 s correlation) gives about 0.3 m/s² rms on d̂ at τ_F = 1 s (≈ 1.7° of tilt); the
  ascent gain ratio up to 2.6 moves the effective pole to ≈ 2.6 rad/s, still half the attitude ωn; the aero force
  changes over seconds in a climb or a descent.
- **Compensation and saturation:** the estimate d̂ is never clamped (it is a measurement). The compensation is
  `sat(d̂)`: horizontal norm ≤ `d_lat_max` 4.0 m/s² (≈ 22° of tilt at hover thrust), vertical within ±`d_vert_max`
  3.0 m/s². The horizontal compensation is multiplied by the lateral authority weight w_lat (section 3.5).
- **Anti-windup:** the observer uses the *actual* (capped, slewed, gated) thrust, so actuator saturation does not wind
  it up: a saturated thrust shows as a tracking error of the controller, not as a disturbance. Below `T_min_obs` 3 N
  (thrust off, kick end, gate closed) the observer **freezes** (states held, `e` not integrated): the lateral authority
  is gone anyway and the thrust model is least accurate there. The velocity loop's own integrator stays as it is
  (slow, 0.2/0.8 1/s²); A4 checks the double-integrator overshoot and may set `vel_int_scale_with_dob` < 1.
- **Reset and ramp-in:** reset (ẑ = v_est, d̂ = 0) on every `resetControlState()` of the TVC set (the controller phase
  table: every enable and mission switch), on leaving the freeze after more than 0.5 s, and on an input that was
  invalid for longer than `hold_invalid_s`. After a reset the compensation weight ramps 0 → 1 over `ramp_s` 1.0 s.
- **Failure:** a non-finite input skips the update (states held) for up to `hold_invalid_s` 0.2 s, then the
  compensation fades to zero over `fade_s` 0.5 s and the observer re-initialises when inputs return. The output a_thr
  is always finite: if anything non-finite reaches the composition, the stage outputs `a_cmd` unchanged for that tick
  (and counts it, log at most once per second).
- **Diagnostics only, never an action:** a flag "compensation saturated > 2 s" goes to the log/telemetry. No failsafe,
  abort or mission action depends on it, and nothing in this design monitors the position error (owner: no
  position-error monitoring).

### 3.4 Moment observer and torque feedforward (body x/y)

Model: I·ω̇ = r × F_thr,gated + M_ext − ω × Iω, with I = diag(0.044, 0.044, 0.004) kg m², r = (0, 0, −0.35) m. States per
axis x, y: **ω̂ (rad/s), M̂ (N m)**, double pole at −β_M, β_M = 1/0.15 s (the ascent study's filter constant):

```text
e  = ω_est − ω̂
ω̂ += dt·( (τ_tvc + M̂ − ω×Iω)/I + 2β_M·e )
M̂ += dt·( I·β_M²·e )
```

- Runs whenever inputs are valid, including thrust off (then τ_tvc = 0 and M̂ is the pure aero moment, also of the flaps
  in freefall: useful data for A9). Roll is not observed (the roll torque command is separate, Iz is small).
- **Usage** u = |M̂_xy| / A(T), A(T) = T_cap,actual · sin 3° · 0.35 m (0.256 N m at 14 N); attack/release filter 0.05 s /
  1.0 s (ascent study). Consumers: the tilt schedule (section 3.5) and, deferred, the ascent governor R.
- **Torque feedforward** τ_ff = −k_M·sat(M̂_xy, M_ff_max), added to the attitude controller's torque before the TVC force
  conversion and its 3° cone logic (so the cone limit and the limit compensation apply to the sum). k_M = 0 by default;
  design value 0.8 (leaves a margin for the lag of M̂); M_ff_max = 0.8·A. Effect: the static aero stiffness is cancelled
  up to the authority, so the commanded tilt is reached in phase 1 of the ascent and the attitude does not sag in
  tail-first flow. It cannot extend authority beyond A.
- Reset on the TVC set reset; freeze/fade on invalid inputs as in 3.3.

### 3.5 Speed-scheduled limits and the lateral-authority fade

Airspeed belief: u = ŵ − v_est in body axes (ŵ = 0 by default, see open decision 7): V_air = |u|, tail-first when
u_z,body > 0. All schedules are piecewise linear, evaluated every tick, rate-limited (1/s) so they never step.

1. **Tilt limit θ_max(V, u)** applied to the direction of a_thr (the 35° clip of `ControlMappingAccToAtt` stays as a
   backstop):
   `θ_max = θ_max0` for |u_ax| ≤ V_t1, linear to `θ_min` at V_t2, where u_ax = u_z,body is the axial airspeed in
   either direction (the destabilising moment scales with u_ax·u_cross, section 2.3/2.4, so a hover in strong wind,
   u_ax ≈ 0, keeps the full tilt it needs to hold position); then multiplied by the usage factor
   `clamp((u_hi − usage)/(u_hi − u_lo), f_min, 1)`. Design values: θ_max0 35° (today), θ_min 12°, V_t1 6 m/s,
   V_t2 12 m/s, u_lo 0.4, u_hi 0.8, f_min 0.5. Clipping keeps the vertical component and shortens the horizontal one
   (vertical priority), unlike today's mapping, which keeps |a| and rotates it.
2. **Lateral-authority fade w_lat** (tail-first flow only): driven by the **tail-first axial airspeed**
   u_tail = max(u_z,body, 0), the speed at which the vehicle descends through the air along its own axis. w_lat = 1 for
   u_tail ≤ V_lf1, linear to 0 at V_lf2. It multiplies the horizontal part of a_des (the velocity loop's demand and the
   horizontal observer compensation). With w_lat = 0 the vehicle holds vertical attitude, all thrust brakes vertically,
   and the horizontal motion is braked by drag alone. Design values (model-free, conservative): **V_lf1 3 m/s,
   V_lf2 5 m/s**. They lie below every flip of section 2.4 (lowest: ×1.3, folded, v_h = 6 m/s at 5.6 m/s descent;
   Upright nominal 8.3-9.5 m/s). Hover in strong wind (u_tail ≈ 0) and the RTH translation (≤ 2 m/s along the path)
   are not affected. Total airspeed would be the wrong variable: it would cut the authority in a hover in 5-7 m/s wind,
   where b is positive. Cost in the geometry world (no flip below 20 m/s): RTH Stabilize from 16 m/s loses tilt braking
   for about 3.8 s (16 → 5 m/s at ≈ 2.9 m/s² net vertical decel), about 11 m of extra drift at 3 m/s sideways, which
   the translation then flies back. In the nominal model the drag alone brakes 3 m/s sideways at about 4.8 m/s² at
   16 m/s (folded) and 11 m/s² (Upright).
   - With the validated feedforward (3.6) w_lat is computed from b̂ instead of the speed (keeps authority where the
     model is confident the gain is positive).
3. **Gain normalisation** (only with the feedforward, 3.6): the horizontal demand is scaled by (T/m)/b̂ so the velocity
   loop sees its design gain again (ascent: ×1/1.1…×1/2.6).
4. **Velocity limits** (inputs to the missions, not changed by this design unless decided): the descent rate during
   lateral position control `v_desc_lat_max` (design value 3 m/s = V_lf1; today the RTH translation path is ≤ 2 m/s
   so it is inactive) and the ascent ceiling (deferred, ascent decision 2; default: today's `ascent_speed` mission
   parameter, 7 m/s).
5. **Thrust:** the ascent thrust cap is deferred (ascent decision 5); default none (today).

### 3.6 Aero feedforward (only after validation) and its confidence gate

Per tick (mode bit FF and `aero_ff_validated`):
1. Belief config (D7): `makeReducedAeroConfig(params, 8, beliefScales)`; belief scales = the A9 fit (default 1.0, i.e.
   nominal). Air velocity `airVelocityBody(q_est, v_est, ŵ)`, ω_est, true flaps, ρ 1.2.
2. **F̂_A** = R(q)·total.force/m at the current attitude (1 call).
3. **b̂** along the current horizontal demand direction ĥ: central difference ±1° of tilt about the axis ⊥ ĥ
   (2 calls): b̂ = (T cos θ + ĥ·∂F̂_A/∂θ)/m. Robustness bounds: the same at belief ×0.7 and ×1.3 (4 more calls, only
   when b̂_nominal < 2·b_min).
4. **Confidence** c ∈ [0, 1] from the residual between the observer's total aero estimate and the model:
   `r = (d̂ + w_ff·â_A) − â_A` (independent of how much feedforward is applied), ratio
   ρ_c = LPF_1s(|r_h|) / max(|â_A,h|, a_floor 0.5 m/s²); c = 1 for ρ_c ≤ 0.3, 0 for ρ_c ≥ 0.8, linear between, with
   0.1 hysteresis. **Envelope:** c = 0 outside the validated envelope (V_air ≤ V_val, α ≤ α_val, |ω| ≤ ω_val, flap
   configuration in the validated set; filled in by A9).
5. **Weight** w_ff = c·enable, rate-limited +0.5/s, −2/s.
6. **Composition:** a_des = a_cmd − sat(d̂) − w_ff·â_A; horizontal part scaled by (T/m)/b̂_eff with
   b̂_eff = w_ff·b̂ + (1 − w_ff)·T/m when b̂ ≥ b_min (= 0.3·T/m) in all evaluated bounds;
   **w_lat from b̂**: w_lat = clamp((b̂_low − b_min)/b_min, 0, 1) using the lowest bound.
7. **Tilt inversion** (b̂ < −b_min confidently in all bounds, c = 1): the horizontal demand is scaled by the negative
   (T/m)/b̂, i.e. the vehicle tilts away from the target to fly toward it. **Disabled by default
   (`allow_gain_inversion` false), open decision 3.** Without it, the flipped region behaves as w_lat = 0.

Cost: 3 calls normally, 7 near the flip band. The feedforward is applied to the force only; the moment side uses the
observer (3.4), because the moment model is the least certain part (pot0.22) and the observer already covers it.

### 3.7 Modes and switches

`aero_ctrl` bitmask (default **0**), plus the parameters of section 4:

| Bit | Name | Effect | Changes flight behaviour? |
|---|---|---|---|
| (0 = none) | off | stage forwards a_cmd, no torque input | no (bit-identical, host-proven) |
| 0 | MONITOR | observers, airspeed, model prediction and confidence computed and logged; outputs unchanged | no control change (CPU only) |
| 1 | FORCE_DOB | apply sat(d̂) | **yes** |
| 2 | TORQUE_FF | apply τ_ff (k_M) | **yes** |
| 3 | SCHEDULE | tilt schedule, lateral fade | **yes** |
| 4 | FEEDFORWARD | 3.6 (needs `aero_ff_validated`) | **yes** |

Presets for the owner: 0x01 monitor, 0x0F observers + limits ("stage 3a"), 0x1F with feedforward ("stage 3b").
**How the switch is set** is open decision 1: a runtime mission parameter (id 11+, RAM only, accepted on the ground
only, read at mission begin, like `ascent_speed`) or a compile-time flag. The mission parameter is better for A/B tests
on one image but adds ids to `MISSION_PARAMS`/`SET_MISSION_PARAM` (a cross-project ExVectrPackets/ground-station
change).

### 3.8 Failure and reset behaviour (all modes)

| Case | Behaviour | Policy tie-in |
|---|---|---|
| TVC set disabled (countdown, freefall, after the kick) | observers keep their state frozen (force) / running (moment); stage forwards a_cmd (the controllers ignore it anyway) | none |
| TVC set reset (enable, mission switch: phase table) | observer reset, compensation ramp-in 1 s, schedules recomputed | unchanged phase table |
| Motor gate closed in flight | gated thrust 0 → force observer frozen below T_min; moment observer sees τ_tvc = 0 (correct) | interlock unchanged (rule 9) |
| Non-finite estimate or thrust | hold ≤ 0.2 s, fade 0.5 s, then pass-through until valid; output never non-finite | T9b/sanitiser unchanged |
| Observer compensation saturated | clamped, flag logged | **no action** (no position-error monitoring) |
| Confidence collapse (model wrong in flight) | w_ff → 0 within 0.5 s, w_lat from the speed schedule, observers unaffected | none |
| Thrust shortfall (flight6) | appears as a vertical disturbance; the vertical compensation (≤ 3 m/s²) restores tracking until the thrust saturates; the moment usage reads conservatively | (deferred) ascent thrust-margin rule |
| Wind beyond authority | schedules hold the attitude within authority; drift accepted; RTH flies back | GO wind limits (ascent decision 4, descent decision 3) |

### 3.9 Relation to the ascent governor R (deferred decisions)

The ascent study's recommendation R (observer governor, 3 m leash, a_z shaping, tilt 30°, thrust cap, 60 s limit,
5 m/s ceiling, wind limit, telemetry) is **not decided** and this design does not decide it. It provides parts R
needs and leaves R's own decisions as inputs with today's defaults:

| R item | Here | Default (keeps today) |
|---|---|---|
| Moment observer and usage | `MomentObserver` (3.4) | computed only in modes with MONITOR or higher |
| Speed governor 1-5 m/s, leash, `EnvelopeExceeded`, 60 s | not part of this design; consumes `usage` if adopted | off; waypoint speed = `ascent_speed` (7 m/s), time limit 25.5 s |
| a_z shaping [7.3, 13.8] m/s², tilt ≤ 30°, vertical command cap | can be added to the `AeroAccelStage` composition as a fifth bit instead of `ControlPositionStandard` (ascent A2) | off |
| Thrust cap 1.15·m·|a| | `StarshipTVC::thrustCap` (exists) | none |
| Telemetry (v_allow, moment, usage) | joins the fields of step A6 | not sent |

### 3.10 CPU and memory budget (assumed costs; the DWT measurement replaces them)

| Item | Per 20 ms tick | Assumption |
|---|---|---|
| Force observer, airspeed, schedules | ≈ 1-2 µs | ≈ 300 flops, no transcendentals except one sqrt/atan2 |
| Moment observer + torque hook | ≈ 1 µs | ≈ 100 flops |
| Feedforward, normal | 3 aero calls ≈ 30-60 µs | `starshipAero`, 8 strips, **10 µs (section 6.3 estimate) to 20 µs (flash cache misses)** per call |
| Feedforward near the flip band | 7 calls ≈ 70-140 µs | same |
| Worst case | ≤ 0.15 ms per 20 ms (0.75 % CPU) | below the 1 ms single-call limit of the cooperative scheduler |

- Code: all new functions `FLASHMEM` (the flight build already links `starshipAero` since S4, so the model costs no
  new flash). ITCM grows only by call sites in existing ITCM functions (`ControlAttitudeTvc::taskThread` hook ≈ 60 B,
  wiring in `main.cpp` is cold). Against the 11384 B padding this is no risk; every step still reports its delta.
- RAM: observers 2 × ≈ 40 B, stage state ≈ 100 B, belief config ≈ 400 B (8-strip `AeroConfig`), task shell ≈ 200 B;
  ≈ 0.8 KB, placed in DMAMEM like the simulator if RAM1 is tight.
- If the measured call is slower than 40 µs, the feedforward drops the bounds evaluation (4 calls) and uses forward
  differences (2 calls in total).

### 3.11 What stage 3 needs from stage 2, and what can be done first

| Stage 2 step | Needed by | Why |
|---|---|---|
| S7a `ControlAttitudeTvc` core | A5 (torque hook), A8 (enable) | The hook becomes an `Input` field of the core; host closed loops then run the real attitude code instead of the `test_kick_sweep`-style replica. |
| S7b position + acc-to-att cores | **A8 (before any behaviour-changing enable)**, A10 | The stage sits between these two cores; the host proof of modes 2-4 must run the real position and mapping code (a replica proves nothing about the code that flies, simulation-architecture.md 2.2 point 6). |
| S7c `StarshipTVC` output model, `TrueFlapModel` on the Dynamics blocks | A9, A10 | The observers' thrust prediction must equal the actuator model the plant uses; the feedforward's flap input must be the same true angles. |
| S8 `ClosedLoop` + `Rollout` | A4 (nice to have), later MPC | One composition for all harnesses. |

Can be done on the existing controllers first (no stage 2): A1-A4 (host only, with replicas, clearly labelled), A5
(wiring with mode 0 pass-through, proven bit-identical), A7 (monitor flights). The feedforward and the enable steps
wait for S7a/S7b (open decision 5).

## 4. Parameters (all new)

"Default" is the value at boot. Feature parameters only act when their mode bit is set, so their defaults are the
design values; the mode default 0 keeps today's behaviour. Values marked (A4) are set by the tuning study, (A9) by the
flight validation.

| Name | Unit | Default | Range | Meaning |
|---|---|---|---|---|
| `aero_ctrl` | bitmask | **0** | 0..0x1F | section 3.7 |
| `fobs_tau_s` | s | 1.0 (A4) | 0.3..3 | force observer time constant 1/β_F |
| `fobs_d_lat_max` | m/s² | 4.0 | 0..8 | horizontal compensation limit |
| `fobs_d_vert_max` | m/s² | 3.0 | 0..6 | vertical compensation limit |
| `fobs_t_min` | N | 3.0 | 0..14 | freeze below this gated thrust |
| `obs_ramp_s` | s | 1.0 | 0..5 | compensation ramp-in after a reset |
| `obs_hold_invalid_s` / `obs_fade_s` | s | 0.2 / 0.5 | 0..2 | invalid-input hold, then fade |
| `vel_int_scale_with_dob` | - | 1.0 (A4) | 0..1 | scale of the velocity-loop integrator while FORCE_DOB is on |
| `mobs_tau_s` | s | 0.15 | 0.05..1 | moment observer time constant 1/β_M |
| `inertia_xx`, `inertia_yy` | kg m² | 0.044, 0.044 | - | from `VEHICLE_MASS`/sim params (one source) |
| `tvc_lever_m` | m | −0.35 | - | thrust point z (one source with `StarshipTVC`) |
| `usage_attack_s` / `usage_release_s` | s | 0.05 / 1.0 | - | usage filter |
| `torque_ff_gain` k_M | - | 0.8 (A4) | 0..1 | torque feedforward gain |
| `torque_ff_max` | N m | 0.205 (0.8·A) | 0..0.256 | torque feedforward limit |
| `tilt_max0` | rad | 0.611 (35°) | 0.1..0.7 | tilt limit at low airspeed (today's value) |
| `tilt_min` | rad | 0.209 (12°) (A4) | 0.05..0.7 | tilt limit at V_t2 |
| `tilt_v1`, `tilt_v2` | m/s | 6, 12 (A4) | 0..30 | axial airspeed |u_ax| breakpoints of the tilt schedule |
| `usage_lo`, `usage_hi`, `usage_fmin` | -, -, - | 0.4, 0.8, 0.5 | 0..1 | usage reduction of the tilt limit |
| `lat_fade_v1`, `lat_fade_v2` | m/s | 3, 5 (A4) | 0..30 | tail-first axial airspeed u_tail breakpoints of w_lat |
| `v_desc_lat_max` | m/s | 3 (input to missions, inactive today) | 0.5..10 | descent-rate limit during lateral position control |
| `wind_belief_x`, `_y` | m/s | 0, 0 | ±15 | wind belief for the airspeed (open decision 7) |
| `aero_ff_validated` | bool | **false** | - | set only after A9 (owner) |
| `belief_scales` (`AeroScales`) | - | 1.0 each (A9) | 0.1..3 | controller's belief config (D7) |
| `belief_strips` | - | 8 | 4..24 | reduced config (validated: 8) |
| `ff_bound_lo`, `ff_bound_hi` | - | 0.7, 1.3 | 0.3..3 | belief bounds for the sign of b̂ |
| `ff_env_v_max`, `ff_env_alpha_max`, `ff_env_omega_max` | m/s, rad, rad/s | 0, 0, 0 (A9) | - | validated envelope (0 = no envelope = no feedforward) |
| `ff_conf_lo`, `ff_conf_hi`, `ff_conf_tau_s`, `ff_a_floor` | -, -, s, m/s² | 0.3, 0.8, 1.0, 0.5 | - | confidence ratio thresholds |
| `ff_w_up`, `ff_w_down` | 1/s | 0.5, 2.0 | - | feedforward weight rate limits |
| `b_min_frac` | - | 0.3 | 0.1..1 | minimum gain fraction of T/m for lateral authority |
| `allow_gain_inversion` | bool | **false** | - | open decision 3 |

Parameters with "one source" must be read from the existing constants (`starship_hardware.h`, `StarshipPlantParams`),
never duplicated. How the parameters are stored (compile-time struct vs mission parameters) follows open decision 1;
recommended: only `aero_ctrl`, `aero_ff_validated` and the five belief scales as runtime parameters, the rest as a
`constexpr` struct.

## 5. Verification plan

### 5.1 Host harness (new `test/test_aero_control`, style of `test_kick_sweep`/`test_descent_guidance`)

- Plant: `plant_test::PlantModel` on `StarshipPlantHarness` (servo and TVC slew models on), 2 ms.
- Controllers: until S7a/S7b, replicas of `ControlPositionStandard`, `ControlMappingAccToAtt`, `ControlAttitudeTvc`
  (limit compensation on) and the `StarshipTVC` cap/slew, each with a header comment naming the copied commit;
  afterwards the real cores. The stage, observers and schedules are always the real code from `src/control` and
  ExVectrControl.
- Estimator noise (Gauss-Markov) as in the descent and ascent studies: position 0.3/0.5 m (2 s), velocity 0.2 m/s
  (0.3 s), tilt 0.5° (5 s), heading 2° (5 s), gyro 0.01 rad/s white; position latency 40 ms; one tick of command
  phasing.
- **Truth variants** (`kVariants` plus two): nominal, ×0.7, ×1.3, geometry, pot0.22 (`AeroScales::hullPotential`
  0.22), thr11.5 (motor delivers 11.5 N, controller assumes 14). **Belief**: nominal unless the scenario says
  otherwise (D7).
- Seeds: `AEROCTL_SEEDS` (default 2, full matrix 10); report lines `AEROCTL <scenario> <variant> <mode> ...`; asserts
  only on the pass criteria below.
- Crash = tilt > 60°, or ground contact faster than 2 m/s, or ground contact above 3 m altitude climbed before.

| ID | Scenario | Modes compared | Pass criteria |
|---|---|---|---|
| V0 | Bit-identity: scripted input sequences (dt jitter, enable toggles, resets, NaN) through stage + controllers | 0 vs no stage; 1 vs no stage | outputs (accel to mapping and TVC, attitude sp, TVC vector) **memcmp-identical** every tick |
| V1 | Static maps (this branch, `test_aero_control_study`) | - | report only; re-run when the model changes |
| V2 | Hover 1 m and 60 m, wind 0/3/5 m/s + gusts 3 m/s, 30 s | 0, 0x0F | no crash; horizontal error p90 ≤ mode 0 in every cell and ≤ 1.5 m for wind ≤ 3 m/s; max tilt ≤ 35°; d̂ within 20 % of the true aero acceleration after 3·τ_F (sim truth from the plant probe) |
| V3 | Ascent 4 s hover then the waypoint at `ascent_speed` 3/5/7 m/s, offsets 0/10 m, wind 0/3/5 + gusts | 0, 0x0F (governor R off) | crashes ≤ mode 0 in every cell; total crashes −50 % for ascent speed ≤ 5 m/s and wind ≤ 3 m/s; horizontal error p90 at 5 m/s climb ≤ 2× the hover p90; no new crash class. (The authority limit stays: R's decisions are separate.) |
| V4 | RTH start from the freefall end: 60/80 m, V_d 16 m/s, v_h 0/3/6 m/s toward / across / away from home, Upright flaps, wind 0/3 | 0, 0x0F | no crash; tilt ≤ 35° after 1 s; horizontal speed at the Stabilize→Translation switch ≤ mode 0 + 0.5 m/s; landing reached; time to translation ≤ mode 0 + 3 s |
| V5 | Fast descent with lateral position control (test-only RTH translation path with descent rate 4/6/8/10 m/s, home 10/30 m off), folded flaps | 0, 0x0F, 0x1F | mode 0: report the divergence above the flip (documents the problem); 0x0F: no crash, distance to home non-increasing once w_lat = 1, tilt ≤ 35°, all variants; 0x1F (belief nominal): reaches the return point faster than 0x0F for nominal/×0.7/×1.3 truths and never more than 20 % slower in geometry truth |
| V6 | Abort to RTH during a 5 m/s climb in 5 m/s wind + gusts | 0, 0x0F | no crash for nominal and geometry; no tilt > 45° |
| V7 | Failures: NaN velocity 0.1 s and 1 s; attitude topic stall 0.3 s; motor gate closed 0.5 s in hover then reopened; reset mid-flight; thr11.5 truth | 0x0F, 0x1F | output finite every tick; no compensation step > ramp; observer re-converges in 3·τ_F; no crash attributable to the stage (mode 0 comparison) |
| V8 | Belief × truth matrix: belief 0.7/1.0/1.3 × truth nominal/×0.7/×1.3/geometry, scenarios V3 (5 m/s) and V5 | 0x1F | confidence falls below 0.5 within 2 s whenever |belief/truth − 1| ≥ 0.6 (e.g. belief ×1.3 against geometry); no inversion while `allow_gain_inversion` is false; never worse than 0x0F by more than 20 % in any metric |
| V9 | Observer unit tests (A2): step, ramp, sinusoid disturbance, dt jitter, freeze, reset, NaN, saturation | - | pole placement within 5 %; exact steady state; no NaN |

Tests that must stay green at every step: `test_sim_aero`, `test_descent_guidance`, `test_descent_wiring`,
`test_kick_sweep`, `test_sim_flap_fault`, `test_descent_unit`, `test_mission_chain`, `test_aero_api`; the full native
suite once per step at the end (testing policy).

### 5.2 Simulation bench (motors electrically disconnected, safety.md rule 9)

| Step | Procedure | Accept |
|---|---|---|
| A5 (mode 0) | Profile 2 full chain (waypoint → bellyflop → freefall → RTH → landing), calm and 3 m/s + 3 gusts, old and new build | same phase sequence, no failure bits, no non-finite warnings, kick duration ±0.05 s, fall speed ±0.2 m/s, landing reached (S4 criteria) |
| A5/A7 (mode 0x01) | Same as mode 0 plus the log lines | identical behaviour; observer values plausible (hover d̂ ≈ 0 in calm air, M̂ ≈ 0) |
| A8 (0x0F) | Profile 2 calm and wind 3 m/s; abort in the climb at 30 m; RTH from freefall | V2-V6 criteria where observable; no oscillation visible in tilt or flap telemetry |
| A10 (0x1F) | as A8, plus the belief-scale mismatch on purpose (`belief_scales.global` 0.7 and 1.3 while the plant is nominal) | confidence drops for the mismatch, behaviour never worse than 0x0F |

The belief mismatch is a normal flight parameter, not a simulation-only command, so it needs no exception to rule 9.

### 5.3 Flight (owner)

Monitor flights (0x01) first: they change no control output and give the data for A9. Then 0x0F in calm air at low
altitude (hover and a short ascent), then the full profile. 0x1F only after A9 and with the envelope set from the flown
data.

## 6. Step plan

These are stage-3 steps; to avoid confusion with the ascent study's A1-A6, refer to them as **aero-A1 … aero-A11**.
Branch per step: `feature/aero-a<n>-<name>`, with the same name in each touched submodule. Flash/ITCM are estimates;
every step reports the measured delta (padding today 11384 B below the 294912 B boundary).

| Step | Scope | Repo / files | Flash | ITCM | Flight-safety | Model |
|---|---|---|---|---|---|---|
| **aero-A1** | Closed-loop harness `test/test_aero_control` with replicas and mode 0 only: V2-V6 legacy baselines, reproduction of the descent flip divergence | flight-software `test/test_aero_control/`, `test/support/` | 0 | 0 | no (host) | Sonnet |
| **aero-A2** | `ForceObserver`, `MomentObserver` (header-only, pure) + `test/test_disturbance_observer` (V9) | ExVectrControl `include/ExVectrControl/disturbance_observer.hpp`; flight-software test | 0 (unused) | 0 | no | Sonnet |
| **aero-A3** | `AeroAccelStage` core: modes 0-3 (no feedforward), airspeed, schedules, fade, composition, failure handling; V0 bit-identity test; harness uses it | flight-software `src/control/aero_accel_stage.*`, `test/test_aero_accel_stage/`, `platformio.ini` (`[env:native]` filter; `[env:teensy40]` excludes `control/` until A5 so static initialisers do not grow RAM1) | 0 | 0 | no (not linked) | Sonnet |
| **aero-A4** | Tuning and robustness study: τ_F, k_M, tilt and fade breakpoints, `vel_int_scale_with_dob`; V2-V8 (modes 0x0F) over all variants; updates section 4 and this doc | flight-software test + this doc | 0 | 0 | no (host), but it **sets flight values** | **Opus** |
| **aero-A5** | Vehicle wiring, default mode 0: stage task between position controller and mapping/TVC; moment observer; torque-feedforward hook in `ControlAttitudeTvc` (unsubscribed by default); `getTVCGatedThrustTopic` alias; log line (1 Hz in MONITOR); mode/parameter plumbing per open decision 1 | ExVectrControl (`control_attitude_tvc.*`), flight-software `src/control/aero_accel_task.*`, `src/main.cpp`, `src/starship_tvc.hpp`, (if runtime) `src/safety/mission_params.*` | +4..6 KB | +0.1..0.3 KB | **yes** (control path wiring; mode 0 bit-identical on the host; bench mode 0) | Sonnet + careful main-session review |
| **aero-A6** | Telemetry fields (d̂_h, M̂, usage, w_lat, w_ff, c, mode) and, if decided, the `aero_ctrl` mission parameter id: cross-project | ExVectrPackets schema, flight-software `src/comms/telemetry_mapping.*`, ground station | +0.3..0.6 KB | ≈ 0 (encode in `FLASHMEM`) | no (telemetry), cross-project | Sonnet per side |
| **aero-A7** | Monitor flights (mode 0x01) and bench monitor runs; data at ≥ 50 Hz on USB or air telemetry | owner; ground station recordings | 0 | 0 | no control change; owner approval for the flight | owner |
| **aero-A8** | Enable 0x0F as a selectable mode: real-core host proof (needs S7a/S7b), bench V-runs, owner approval; first flight calm, low | flight-software (parameter only; default stays 0 unless the owner decides) | 0 | 0 | **yes** | Sonnet (runs) + main-session review |
| **aero-A9** | Model validation and fit: regress the observed M̂ and d̂ (A7/A8 data) against `starshipAero`; fit `belief_scales` (hullPotential, hullCrossflow, hullAxial, global); acceptance: residual ≤ 30 % over the flown envelope; output the envelope parameters | analysis tool (`tools/`), this doc, simulation-architecture.md | 0 | 0 | no (analysis), but it **decides `aero_ff_validated`** | **Opus** |
| **aero-A10** | Feedforward (mode bit 4): F̂_A, b̂ with bounds, confidence, weight, gain normalisation, w_lat from b̂; inversion code present but disabled; V5, V8 with real cores (needs S7b, S7c) | flight-software `src/control/aero_accel_stage.*`, tests | +2..3 KB | ≈ 0 | **yes** | Sonnet against this spec; **Opus** reviews the inversion/gating logic |
| aero-A11 (optional) | Speed-scheduled flap configuration: Upright (top flaps out) in powered tail-first flow above a descent airspeed, instead of only in RTH Stabilize (section 2.4) | flight-software `main.cpp` `FlapModeObserverTask`, `test_descent_wiring` | +0.2 KB | ≈ 0 | **yes** (flap mode, mission behaviour) | **Opus** (decision) then Sonnet |

Order: A1 → A2 → A3 → A4 (host, can start now) ; A5 (after S7a if the hook should be born in the core; else with a
bit-identity test) → A6/A7 ; S7b → A8 ; A9 → S7c → A10 ; A11 any time after A4.

## 7. Risks and what is deliberately not done

| Risk | Mitigation |
|---|---|
| The model's upright aero is wrong by a factor 1-5 | Observers are model-free; the fade uses conservative speeds; feedforward only after A9, inside the flown envelope, gated by confidence |
| Observer + velocity integrator overshoot (two integrators) | A4 tunes `vel_int_scale_with_dob`; V2/V3 metrics |
| Observer fed by a wrong thrust (ESC/battery, flight6) | Vertical compensation limited to ±3 m/s²; usage reads conservatively; thrust-stand test (ascent decision 7) |
| Replica harness proves less than real cores | Enable steps (A8, A10) wait for S7a/S7b |
| Lateral fade removes authority the vehicle has (geometry world) | Quantified (≈ 11 m extra drift in RTH Stabilize); the feedforward restores it once validated |
| Feedforward cost higher than assumed | Measured DWT table first; fall back to 2 calls; never more than 0.5 ms per call of the stage |
| Same-code rule | No simulation-flag read; observers use switched topics and the gated thrust; the belief mismatch test is a normal parameter |

Not done here: no MPC; no change to missions, the phase table, the motor gate, the flap descent law or the sim/flight
difference list; no decision on the ascent governor R items; no new telemetry fields decided; no estimator change; no
position-error monitoring of any kind.

## 8. Open decisions for the owner

| # | Decision | Options | Recommendation |
|---|---|---|---|
| 1 | How the `aero_ctrl` mode (and the belief scales, `aero_ff_validated`) are set | (a) runtime mission parameters (ground only, RAM, read at mission begin; Packets + GS change); (b) compile-time flag | **(a)**: A/B tests on one image in simulation and flight; default 0 at every boot |
| 2 | Adopt the stage-3a set (observers + torque feedforward + speed-scheduled tilt and lateral fade, mode 0x0F) as the target before the feedforward | yes / only parts | **Yes**, after A4 and the A8 checks; it needs no model validation |
| 3 | Tilt inversion beyond the flip (tilt away from home to go home) when the validated model is confident | allow / never | **Not now** (`allow_gain_inversion` false): fade instead; revisit after A9 shows the flip in real data |
| 4 | Lateral-fade breakpoints before validation | 3 / 5 m/s tail-first axial airspeed (conservative, below every modelled flip incl. ×1.3) vs no fade | **3 / 5 m/s** until A9; A4 confirms. Cost in the geometry world: ≈ 11 m extra drift in an RTH Stabilize from 16 m/s |
| 5 | Order relative to stage 2 | (a) S7a/S7b before any enable step (A8); (b) enable on replicas only | **(a)**: host proof with the real cores |
| 6 | Acceleration signal for the force observer | (a) velocity estimate (works in both modes, no list change); (b) add a simulated specific-force topic switch so the accelerometer can be used (sim/flight list change, rule 7) | **(a)** now; (b) only if A7 shows the velocity-based estimate too noisy or slow |
| 7 | Wind belief for the model airspeed | (a) zero (observer absorbs wind); (b) online wind estimate from the hover observer (later step) | **(a)** for A10; (b) as a separate design if A9 shows crosswind ascents need it |
| 8 | Speed-scheduled flap configuration (aero-A11: Upright in powered tail-first flow) | do / don't | **Do the study in A4**, decide after: it is the only lever that adds moment authority in fast descent |
| 9 | Monitor mode (0x01) as the boot default for the next flights | 0 / 0x01 | **0x01** once A5 is merged and bench-checked: no control change, gives the A9 data (CPU ≈ 5 µs per tick) |
| 10 | Telemetry fields of aero-A6 | add now / later | **With A5/A7**, otherwise the monitor flights record only the 1 Hz log |
