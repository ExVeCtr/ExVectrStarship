// Host tests of src/safety/mission_chain.cpp: pio test -e native
//
// MissionChain decides the mission switching of VehicleSafetyAndControlTask. The harness below runs a model of the
// configured chain (waypoint -> bellyflop -> freefall -> RTH) with injected time: fake missions with the same
// begin/reset/end semantics as src/mission/*.hpp (resetMission() sets missionEnd() = true, the waypoint mission ends
// in mode Running, bellyflop/freefall/RTH end in Finished, RTH hot-starts in Running), mission tasks ticking at
// 100 Hz and the safety task at 10 Hz. Vehicle::switchTo, systemReset and missionAbort mirror main.cpp
// (switchMissionTo, resetMissionsToBootState, the MissionAbort case).
//
// Main property (user decision 2026-11-20): a SystemReset in any phase leaves every mission Idle, RTH active, no
// chain pending, all mission actuators off, and nothing starts again without a new MissionBegin.
//
// Also modelled (2026-11-20, feature/bellyflop-abort-gnss): the controller phase table (Safety::ControlPhaseTracker,
// applied after the chain step like main.cpp) next to the legacy mode-edge enable, the bellyflop init order (legacy:
// taskInit on the first unpaused tick overwrites Running with Idle), and MissionAbort through Safety::decideAbort.
#include <unity.h>

#include "safety/control_phase.hpp"
#include "safety/mission_abort.hpp"
#include "safety/mission_chain.hpp"

using namespace VCTR::Safety;

static constexpr int64_t MS = 1000000LL;
static constexpr int64_t S = 1000000000LL;

enum class Mode { Idle, Init, Startup, Running, Finished };
enum class Kind { Waypoint, Bellyflop, Freefall, Rth };

struct FakeMission {
  Kind kind;
  FakeMission *next = nullptr;
  Mode mode = Mode::Idle;
  bool ended = false; // RTH and waypoint start with false until their first Idle tick, as in the real classes
  bool actuators = false;
  int64_t missionTime = 0;
  int64_t lastTick = 0;
  int64_t runningSince = 0;
  int begins = 0;
  bool taskInitDone = false; // bellyflop only: paused at construction, its taskInit runs on the first unpaused tick
  bool paused = true;        // bellyflop only: unpaused by begin, paused by reset and when finished
  bool legacyTaskInit = false; // bellyflop only: model the old taskInit that set mode Idle (fixed 2026-11-20)

  explicit FakeMission(Kind k) : kind(k) {}

  void reset() {
    paused = true;
    mode = Mode::Idle;
    ended = true;
    actuators = false;
    missionTime = 0;
  }

  void begin(int64_t t0, int64_t now) {
    begins++;
    lastTick = now;
    runningSince = now;
    ended = false;
    switch (kind) {
    case Kind::Waypoint:
      reset();
      mode = Mode::Init;
      ended = false;
      missionTime = t0;
      break;
    case Kind::Bellyflop:
      paused = false;
      mode = Mode::Running;
      actuators = true;
      break;
    case Kind::Freefall:
      mode = Mode::Running;
      actuators = false;
      break;
    case Kind::Rth:
      mode = Mode::Running; // hot start, ignores t0
      actuators = true;
      missionTime = 0;
      break;
    }
  }

  void tick(int64_t now) {
    if (kind == Kind::Bellyflop) {
      // Paused unless begun and not finished; on the first unpaused tick after boot, taskInit sets mode Idle and
      // leaves missionEnd false (MissionBellyflop::taskInit), then the transition runs on and ends in Finished.
      if (paused)
        return;
      if (!taskInitDone) {
        taskInitDone = true;
        if (legacyTaskInit)
          mode = Mode::Idle; // old MissionBellyflop::taskInit; the fixed taskInit does not touch the mode
      }
      if (now - runningSince > 1 * S) {
        mode = Mode::Finished;
        ended = true;
        paused = true;
      }
      lastTick = now;
      return;
    }
    if (mode != Mode::Idle)
      missionTime += now - lastTick;
    lastTick = now;
    switch (mode) {
    case Mode::Idle:
      actuators = false;
      missionTime = 0;
      if (kind == Kind::Waypoint || kind == Kind::Rth)
        ended = true; // missionIdle()
      break;
    case Mode::Init:
      actuators = false;
      if (missionTime > -10 * S)
        mode = Mode::Startup;
      break;
    case Mode::Startup:
      actuators = true;
      if (missionTime > 0) {
        mode = Mode::Running;
        runningSince = now;
      }
      break;
    case Mode::Running:
      if (kind == Kind::Waypoint && now - runningSince > 30 * S)
        ended = true; // waypoint ends in Running
      if (kind == Kind::Bellyflop && now - runningSince > 1 * S) {
        mode = Mode::Finished;
        ended = true;
      }
      if (kind == Kind::Freefall && now - runningSince > 8 * S) {
        mode = Mode::Finished;
        ended = true;
      }
      if (kind == Kind::Rth && now - runningSince > 15 * S) {
        mode = Mode::Finished;
        ended = true;
        actuators = false;
      }
      break;
    case Mode::Finished:
      if (kind == Kind::Rth)
        actuators = false;
      break;
    }
  }
};

struct Vehicle {
  FakeMission wp{Kind::Waypoint}, bf{Kind::Bellyflop}, ff{Kind::Freefall}, rth{Kind::Rth};
  FakeMission *active = &rth;
  MissionChain chain;
  int64_t now = 0;
  int switches = 0;
  int rthEndedTicks = 0;

  // Controller phase: table (current) or the legacy mode-edge enable (shared lastMode, Running edge enables with
  // the mission's actuator flag, every other edge disables), for the root-cause comparison.
  bool legacyEdges = false;
  ControlPhaseTracker ctl;
  int lastSwitchSeen = 0;
  Mode lastMode = Mode::Idle;
  bool tvcOn = false;
  int tvcResets = 0;

  Vehicle() {
    wp.next = &bf;
    bf.next = &ff;
    ff.next = &rth;
  }

  FakeMission *all(int i) {
    FakeMission *m[] = {&wp, &bf, &ff, &rth};
    return m[i];
  }

  void switchTo(FakeMission *m, int64_t t0 = 0) {
    active->reset();
    active = m;
    active->reset();
    active->begin(t0, now);
    switches++;
  }

  void missionBegin(int64_t t0) {
    switchTo(&wp, t0);
    chain.begin();
  }

  void systemReset() {
    chain.reset();
    for (int i = 0; i < 4; i++)
      all(i)->reset();
    active = &rth;
    ctl.reset();
    tvcOn = false;
    lastMode = Mode::Idle;
  }

  /// Only what the old SystemReset did to the missions (active->resetMission(), active pointer kept), plus
  /// chain.reset(). Shows that MissionChain alone stops the chain even if the active mission is not switched.
  void minimalReset() {
    chain.reset();
    active->reset();
  }

  static ActiveMission kindOf(const FakeMission *m) {
    switch (m->kind) {
    case Kind::Waypoint:
      return ActiveMission::Waypoint;
    case Kind::Bellyflop:
      return ActiveMission::Bellyflop;
    case Kind::Freefall:
      return ActiveMission::Freefall;
    case Kind::Rth:
      return ActiveMission::Rth;
    }
    return ActiveMission::Other;
  }

  static MissionModeIn modeOf(Mode m) { return static_cast<MissionModeIn>(static_cast<uint8_t>(m)); }

  /// The MissionAbort case of main.cpp (vehicle mode Running assumed: a mission was begun).
  AbortAction missionAbort() {
    AbortInput in;
    in.vehicleRunning = true;
    in.active = kindOf(active);
    in.mode = modeOf(active->mode);
    const AbortAction a = decideAbort(in);
    switch (a) {
    case AbortAction::StartRth:
      switchTo(&rth);
      break;
    case AbortAction::ResetAndStop:
      chain.reset();
      active->reset();
      break;
    case AbortAction::KeepRth:
    case AbortAction::Ignore:
      break;
    }
    return a;
  }

  void applyControl() {
    if (legacyEdges) {
      if (active->mode != lastMode) {
        tvcOn = active->mode == Mode::Running ? active->actuators : false;
        lastMode = active->mode;
      }
      return;
    }
    const ControlPhaseStep st =
        ctl.update(kindOf(active), modeOf(active->mode), active->actuators, switches != lastSwitchSeen);
    lastSwitchSeen = switches;
    if (st.resetTvc)
      tvcResets++;
    tvcOn = st.phase.tvcEnabled;
  }

  void safetyTick() {
    ChainInput in;
    in.activeIsDefault = active == &rth;
    in.activeIdle = active->mode == Mode::Idle;
    in.activeEnded = active->ended;
    in.nextIsDefaultOrNone = active->next == nullptr || active->next == &rth;
    switch (chain.step(in)) {
    case ChainStep::RthEnded:
      rthEndedTicks++;
      break;
    case ChainStep::SwitchToDefault:
      switchTo(&rth);
      break;
    case ChainStep::SwitchToNext:
      switchTo(active->next);
      break;
    case ChainStep::None:
      break;
    }
    applyControl();
  }

  /// Runs until `until` (absolute ns): mission tasks at 100 Hz, safety task at 10 Hz.
  void runUntil(int64_t until) {
    while (now < until) {
      now += 10 * MS;
      for (int i = 0; i < 4; i++)
        all(i)->tick(now);
      if ((now / (10 * MS)) % 10 == 0)
        safetyTick();
    }
  }

  void runFor(int64_t d) { runUntil(now + d); }

  bool anyActuator() {
    for (int i = 0; i < 4; i++)
      if (all(i)->actuators)
        return true;
    return false;
  }

  void assertBootState() {
    TEST_ASSERT_TRUE(active == &rth);
    for (int i = 0; i < 4; i++) {
      TEST_ASSERT_TRUE(all(i)->mode == Mode::Idle);
      TEST_ASSERT_FALSE(all(i)->actuators);
      TEST_ASSERT_EQUAL_INT64(0, all(i)->missionTime);
    }
    TEST_ASSERT_FALSE(chain.pending());
  }
};

void setUp() {}
void tearDown() {}

// --- step() rules -------------------------------------------------------------------------------------------

static void test_step_rules() {
  MissionChain c;
  ChainInput in;

  // Boot steady state: RTH Idle and ended -> RthEnded (switches things off), never a switch.
  in.activeIsDefault = true;
  in.activeIdle = true;
  in.activeEnded = true;
  in.nextIsDefaultOrNone = true;
  TEST_ASSERT_EQUAL(ChainStep::RthEnded, c.step(in));

  // Not pending: an ended non-default mission does not chain.
  in = ChainInput();
  in.activeEnded = true;
  TEST_ASSERT_EQUAL(ChainStep::None, c.step(in));

  // Pending, ended, not Idle -> switch.
  c.begin();
  TEST_ASSERT_EQUAL(ChainStep::SwitchToNext, c.step(in));
  in.nextIsDefaultOrNone = true;
  TEST_ASSERT_EQUAL(ChainStep::SwitchToDefault, c.step(in));
  in.nextIsDefaultOrNone = false;

  // Pending, not ended -> nothing.
  in.activeEnded = false;
  TEST_ASSERT_EQUAL(ChainStep::None, c.step(in));
  TEST_ASSERT_TRUE(c.pending());

  // Pending, Idle but not ended (bellyflop's first run after boot) -> nothing, chain stays pending.
  in.activeEnded = false;
  in.activeIdle = true;
  TEST_ASSERT_EQUAL(ChainStep::None, c.step(in));
  TEST_ASSERT_TRUE(c.pending());

  // Pending, but the active mission is Idle (was reset) and ended -> nothing, and the chain is cancelled.
  in.activeEnded = true;
  in.activeIdle = true;
  TEST_ASSERT_EQUAL(ChainStep::None, c.step(in));
  TEST_ASSERT_FALSE(c.pending());
  in.activeIdle = false;
  TEST_ASSERT_EQUAL(ChainStep::None, c.step(in)); // stays cancelled

  // reset() cancels.
  c.begin();
  c.reset();
  TEST_ASSERT_EQUAL(ChainStep::None, c.step(in));

  // RTH ended ends the chain.
  c.begin();
  in.activeIsDefault = true;
  TEST_ASSERT_EQUAL(ChainStep::RthEnded, c.step(in));
  TEST_ASSERT_FALSE(c.pending());
}

// The decision the old main.cpp made (missionEnd() alone), kept here to document the root cause.
static ChainStep legacyStep(const ChainInput &in) {
  if (in.activeIsDefault && in.activeEnded)
    return ChainStep::RthEnded;
  if (in.activeEnded)
    return in.nextIsDefaultOrNone ? ChainStep::SwitchToDefault : ChainStep::SwitchToNext;
  return ChainStep::None;
}

static void test_root_cause_legacy_chains_after_reset() {
  // A reset waypoint mission: Idle, missionEnd() true, next = bellyflop.
  ChainInput in;
  in.activeIdle = true;
  in.activeEnded = true;
  TEST_ASSERT_EQUAL(ChainStep::SwitchToNext, legacyStep(in)); // the bug: bellyflop starts
  MissionChain c;
  c.begin();
  TEST_ASSERT_EQUAL(ChainStep::None, c.step(in));
}

// --- chain model --------------------------------------------------------------------------------------------

static void test_boot_state_is_stable() {
  Vehicle v;
  v.runFor(60 * S);
  v.assertBootState();
  TEST_ASSERT_EQUAL(0, v.switches);
}

static void test_full_chain_runs() {
  Vehicle v;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  TEST_ASSERT_TRUE(v.chain.pending());
  v.runFor(200 * S);
  // waypoint (begin) -> bellyflop -> freefall -> RTH: 4 switches, RTH landed, chain over.
  TEST_ASSERT_EQUAL(4, v.switches);
  TEST_ASSERT_EQUAL(1, v.wp.begins);
  TEST_ASSERT_EQUAL(1, v.bf.begins);
  TEST_ASSERT_EQUAL(1, v.ff.begins);
  TEST_ASSERT_EQUAL(1, v.rth.begins);
  TEST_ASSERT_TRUE(v.active == &v.rth);
  TEST_ASSERT_TRUE(v.rth.mode == Mode::Finished);
  TEST_ASSERT_FALSE(v.chain.pending());
  TEST_ASSERT_FALSE(v.anyActuator());
}

/// Starts a mission, runs to `phase` (a predicate on the vehicle), sends SystemReset, then checks 60 s.
template <typename Pred> static void resetInPhaseMinimal(Pred inPhase, const char *what) {
  Vehicle v;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  int guard = 0;
  while (!inPhase(v) && guard++ < 30000)
    v.runFor(10 * MS);
  TEST_ASSERT_TRUE_MESSAGE(inPhase(v), what);
  v.runFor(300 * MS);
  FakeMission *was = v.active;
  v.minimalReset();
  const int sw = v.switches;
  v.runFor(60 * S);
  TEST_ASSERT_EQUAL_MESSAGE(sw, v.switches, what);
  TEST_ASSERT_TRUE_MESSAGE(v.active == was, what);
  TEST_ASSERT_TRUE_MESSAGE(v.active->mode == Mode::Idle, what);
  TEST_ASSERT_FALSE_MESSAGE(v.chain.pending(), what);
}

template <typename Pred> static void resetInPhase(Pred inPhase, const char *what) {
  resetInPhaseMinimal(inPhase, what);
  Vehicle v;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  int guard = 0;
  while (!inPhase(v) && guard++ < 30000)
    v.runFor(10 * MS);
  TEST_ASSERT_TRUE_MESSAGE(inPhase(v), what);
  v.runFor(300 * MS); // a few safety ticks inside the phase

  v.systemReset();
  const int switchesAtReset = v.switches;
  int begins[4];
  for (int i = 0; i < 4; i++)
    begins[i] = v.all(i)->begins;

  v.assertBootState();
  for (int t = 0; t < 600; t++) { // 60 s, checked every 100 ms
    v.runFor(100 * MS);
    TEST_ASSERT_TRUE_MESSAGE(v.active == &v.rth, what);
    TEST_ASSERT_FALSE_MESSAGE(v.anyActuator(), what);
    TEST_ASSERT_FALSE_MESSAGE(v.chain.pending(), what);
  }
  v.assertBootState();
  TEST_ASSERT_EQUAL_MESSAGE(switchesAtReset, v.switches, what);
  for (int i = 0; i < 4; i++)
    TEST_ASSERT_EQUAL_MESSAGE(begins[i], v.all(i)->begins, what);

  // A new MissionBegin afterwards runs the full chain normally.
  v.missionBegin(-15 * S);
  v.runFor(200 * S);
  TEST_ASSERT_TRUE_MESSAGE(v.active == &v.rth, what);
  TEST_ASSERT_TRUE_MESSAGE(v.rth.mode == Mode::Finished, what);
  TEST_ASSERT_EQUAL_MESSAGE(begins[1] + 1, v.bf.begins, what);
  TEST_ASSERT_EQUAL_MESSAGE(begins[2] + 1, v.ff.begins, what);
  TEST_ASSERT_FALSE_MESSAGE(v.chain.pending(), what);
}

static void test_reset_in_init() {
  resetInPhase([](Vehicle &v) { return v.active == &v.wp && v.wp.mode == Mode::Init; }, "waypoint Init");
}
static void test_reset_in_startup() {
  resetInPhase([](Vehicle &v) { return v.active == &v.wp && v.wp.mode == Mode::Startup; }, "waypoint Startup");
}
static void test_reset_in_hover() {
  resetInPhase([](Vehicle &v) { return v.active == &v.wp && v.wp.mode == Mode::Running; }, "waypoint Running");
}
static void test_reset_after_waypoint_end_same_tick_window() {
  // Waypoint has ended (missionEnd true, still Running) but the safety task has not switched yet.
  resetInPhase([](Vehicle &v) { return v.active == &v.wp && v.wp.ended && v.wp.mode == Mode::Running; },
               "waypoint ended");
}
static void test_reset_in_bellyflop() {
  resetInPhase([](Vehicle &v) { return v.active == &v.bf && !v.bf.ended; }, "bellyflop");
}
static void test_reset_in_freefall() {
  resetInPhase([](Vehicle &v) { return v.active == &v.ff && v.ff.mode == Mode::Running; }, "freefall");
}
static void test_reset_in_rth() {
  resetInPhase([](Vehicle &v) { return v.active == &v.rth && v.rth.mode == Mode::Running; }, "RTH");
}
static void test_reset_after_rth_landed() {
  resetInPhase([](Vehicle &v) { return v.active == &v.rth && v.rth.mode == Mode::Finished; }, "RTH landed");
}

static void test_bellyflop_first_run_idle_still_chains() {
  // Regression (bench 2026-11-20): with the legacy taskInit, bellyflop reported Idle (missionEnd false) during its
  // first run after boot. The chain must not treat that as a reset.
  Vehicle v;
  v.bf.legacyTaskInit = true;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  int guard = 0;
  while (v.active != &v.bf && guard++ < 10000)
    v.runFor(10 * MS);
  v.runFor(300 * MS);
  TEST_ASSERT_TRUE(v.active == &v.bf);
  TEST_ASSERT_TRUE(v.bf.mode == Mode::Idle);
  TEST_ASSERT_TRUE(v.chain.pending());
  v.runFor(200 * S);
  TEST_ASSERT_EQUAL(1, v.ff.begins);
  TEST_ASSERT_EQUAL(1, v.rth.begins);
  TEST_ASSERT_TRUE(v.rth.mode == Mode::Finished);
}

static void test_abort_not_running_stops_chain() {
  // MissionAbort during the countdown (waypoint Init / Startup): the old code reset the mission and the next tick
  // started bellyflop. Now nothing follows.
  for (int phase = 0; phase < 2; phase++) {
    Vehicle v;
    v.runFor(2 * S);
    v.missionBegin(-15 * S);
    v.runFor(phase == 0 ? 2 * S : 8 * S);
    TEST_ASSERT_TRUE(v.wp.mode == (phase == 0 ? Mode::Init : Mode::Startup));
    v.missionAbort();
    const int sw = v.switches;
    v.runFor(60 * S);
    TEST_ASSERT_EQUAL(sw, v.switches);
    TEST_ASSERT_TRUE(v.active == &v.wp);
    TEST_ASSERT_TRUE(v.wp.mode == Mode::Idle);
    TEST_ASSERT_EQUAL(0, v.bf.begins);
    TEST_ASSERT_FALSE(v.anyActuator());
  }
}

static void test_abort_running_goes_to_rth_and_ends() {
  Vehicle v;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  v.runFor(20 * S); // waypoint Running
  TEST_ASSERT_TRUE(v.wp.mode == Mode::Running);
  v.missionAbort();
  TEST_ASSERT_TRUE(v.active == &v.rth);
  v.runFor(60 * S);
  TEST_ASSERT_TRUE(v.rth.mode == Mode::Finished);
  TEST_ASSERT_EQUAL(0, v.bf.begins);
  TEST_ASSERT_FALSE(v.chain.pending());
}

// --- controller phase table, bellyflop init order, abort decision ---------------------------------------------

/// Runs (10 ms steps) until `m` is the active mission; fails after 300 s.
static void runUntilActive(Vehicle &v, FakeMission *m) {
  int guard = 0;
  while (v.active != m && guard++ < 30000)
    v.runFor(10 * MS);
  TEST_ASSERT_TRUE(v.active == m);
}

/// One mission from MissionBegin to the end of freefall; returns the TVC enable seen at every safety tick while
/// bellyflop or freefall was active (bit string, max 64 ticks) and the bellyflop mode seen at its first tick.
struct UnpoweredTrace {
  uint64_t bits = 0;
  int ticks = 0;
  Mode bellyflopModeAfter100ms = Mode::Idle;
};

static UnpoweredTrace flyToRth(Vehicle &v) {
  UnpoweredTrace t;
  v.missionBegin(-15 * S);
  runUntilActive(v, &v.bf);
  v.runFor(100 * MS);
  t.bellyflopModeAfter100ms = v.bf.mode;
  while (v.active != &v.rth && t.ticks < 64) {
    v.runFor(100 * MS);
    if (v.active == &v.bf || v.active == &v.ff) {
      if (v.tvcOn)
        t.bits |= (uint64_t(1) << t.ticks);
      t.ticks++;
    }
  }
  return t;
}

static void test_root_cause_bellyflop_first_run_differs_legacy() {
  // Legacy model (taskInit sets Idle on the first unpaused tick, controller enable on mode edges only): the first
  // bellyflop after boot runs with the TVC set disabled, a later one (after SystemReset) with it enabled.
  Vehicle v;
  v.bf.legacyTaskInit = true;
  v.legacyEdges = true;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  runUntilActive(v, &v.bf);
  v.runFor(250 * MS);
  TEST_ASSERT_TRUE(v.bf.mode == Mode::Idle);
  TEST_ASSERT_FALSE(v.tvcOn); // first run: Idle edge disabled the controllers
  v.runFor(5 * S);
  v.systemReset();
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  runUntilActive(v, &v.bf);
  v.runFor(250 * MS);
  TEST_ASSERT_TRUE(v.bf.mode == Mode::Running);
  TEST_ASSERT_TRUE(v.tvcOn); // second run: no edge, controllers stay enabled from the waypoint mission
}

static void test_bellyflop_first_and_later_runs_identical() {
  // Fixed taskInit + phase table: the first bellyflop after boot and a later one report Running and see exactly the
  // same controller phase sequence (TVC set off during bellyflop and freefall).
  Vehicle v;
  v.runFor(2 * S);
  const UnpoweredTrace a = flyToRth(v);
  v.runFor(5 * S);
  v.systemReset();
  v.runFor(2 * S);
  const UnpoweredTrace b = flyToRth(v);
  TEST_ASSERT_TRUE(a.bellyflopModeAfter100ms == Mode::Running);
  TEST_ASSERT_TRUE(b.bellyflopModeAfter100ms == Mode::Running);
  TEST_ASSERT_TRUE(a.ticks > 5);
  TEST_ASSERT_EQUAL(a.ticks, b.ticks);
  TEST_ASSERT_EQUAL_UINT64(a.bits, b.bits);
  TEST_ASSERT_EQUAL_UINT64(0, a.bits);
}

static void test_tvc_reset_on_every_enable() {
  // Full chain: the TVC set is reset when the waypoint mission goes Running and again when RTH starts after
  // freefall (enabled after being off for the whole bellyflop and freefall).
  Vehicle v;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  runUntilActive(v, &v.bf);
  TEST_ASSERT_EQUAL(1, v.tvcResets);
  runUntilActive(v, &v.rth);
  v.runFor(200 * MS);
  TEST_ASSERT_TRUE(v.tvcOn);
  TEST_ASSERT_EQUAL(2, v.tvcResets);
  v.runFor(60 * S);
  TEST_ASSERT_FALSE(v.tvcOn); // RTH finished
}

static void test_abort_in_flight_starts_rth_from_every_phase() {
  // Hover, ascent, bellyflop, freefall: MissionAbort starts RTH at once, the TVC set comes on from a clean state,
  // RTH lands and the chain ends; nothing else is begun.
  for (int phase = 0; phase < 4; phase++) {
    Vehicle v;
    v.runFor(2 * S);
    v.missionBegin(-15 * S);
    if (phase == 0) {
      v.runFor(17 * S); // waypoint Running, hover
    } else if (phase == 1) {
      v.runFor(40 * S); // still waypoint Running (ends after 30 s Running = 45 s from begin)
    } else if (phase == 2) {
      runUntilActive(v, &v.bf);
      v.runFor(300 * MS);
    } else {
      runUntilActive(v, &v.ff);
      v.runFor(2 * S);
    }
    const int resetsBefore = v.tvcResets;
    const int ffBegins = v.ff.begins;
    TEST_ASSERT_TRUE(v.missionAbort() == AbortAction::StartRth);
    TEST_ASSERT_TRUE(v.active == &v.rth);
    TEST_ASSERT_TRUE(v.rth.mode == Mode::Running);
    v.runFor(100 * MS); // next safety tick
    TEST_ASSERT_TRUE(v.tvcOn);
    TEST_ASSERT_EQUAL(resetsBefore + 1, v.tvcResets);
    v.runFor(60 * S);
    TEST_ASSERT_TRUE(v.rth.mode == Mode::Finished);
    TEST_ASSERT_FALSE(v.chain.pending());
    TEST_ASSERT_EQUAL(ffBegins, v.ff.begins);
    TEST_ASSERT_EQUAL(1, v.rth.begins);
  }
}

static void test_abort_low_and_slow_after_liftoff_starts_rth() {
  // Review 2026-11-20: waypoint Running right after liftoff (in the real case 0.3 m, 0.2 m/s). The removed ground
  // check would have reset the mission and cut the motors; now it starts RTH, which runs and lands.
  Vehicle v;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  v.runFor(15 * S + 300 * MS);
  TEST_ASSERT_TRUE(v.wp.mode == Mode::Running);
  TEST_ASSERT_TRUE(v.missionAbort() == AbortAction::StartRth);
  TEST_ASSERT_TRUE(v.active == &v.rth);
  v.runFor(100 * MS);
  TEST_ASSERT_TRUE(v.tvcOn);
  v.runFor(60 * S);
  TEST_ASSERT_TRUE(v.rth.mode == Mode::Finished);
  TEST_ASSERT_FALSE(v.chain.pending());
  TEST_ASSERT_EQUAL(0, v.bf.begins);
}

static void test_abort_during_rth_keeps_rth() {
  Vehicle v;
  v.runFor(2 * S);
  v.missionBegin(-15 * S);
  runUntilActive(v, &v.rth);
  v.runFor(3 * S);
  const int sw = v.switches;
  TEST_ASSERT_TRUE(v.missionAbort() == AbortAction::KeepRth);
  TEST_ASSERT_EQUAL(sw, v.switches);
  TEST_ASSERT_TRUE(v.rth.mode == Mode::Running);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_step_rules);
  RUN_TEST(test_root_cause_legacy_chains_after_reset);
  RUN_TEST(test_boot_state_is_stable);
  RUN_TEST(test_full_chain_runs);
  RUN_TEST(test_reset_in_init);
  RUN_TEST(test_reset_in_startup);
  RUN_TEST(test_reset_in_hover);
  RUN_TEST(test_reset_after_waypoint_end_same_tick_window);
  RUN_TEST(test_reset_in_bellyflop);
  RUN_TEST(test_reset_in_freefall);
  RUN_TEST(test_reset_in_rth);
  RUN_TEST(test_reset_after_rth_landed);
  RUN_TEST(test_bellyflop_first_run_idle_still_chains);
  RUN_TEST(test_abort_not_running_stops_chain);
  RUN_TEST(test_abort_running_goes_to_rth_and_ends);
  RUN_TEST(test_root_cause_bellyflop_first_run_differs_legacy);
  RUN_TEST(test_bellyflop_first_and_later_runs_identical);
  RUN_TEST(test_tvc_reset_on_every_enable);
  RUN_TEST(test_abort_in_flight_starts_rth_from_every_phase);
  RUN_TEST(test_abort_low_and_slow_after_liftoff_starts_rth);
  RUN_TEST(test_abort_during_rth_keeps_rth);
  return UNITY_END();
}
