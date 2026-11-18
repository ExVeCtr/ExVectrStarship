# ExVectr Starship flight computer

Autonomous model-Starship flight software (Teensy 4.0, PlatformIO, Arduino) built on the author's own ExVectr framework (`lib/ExVectr*`, one git submodule per library).

Part of the ExVectrStarship workspace (`../CLAUDE.md` describes the whole system and how sessions and subagents are organised). This file and `docs/` are self-sufficient for working on the flight software alone.

**Read [docs/README.md](docs/README.md) first.** It indexes all developer documentation. Library and project behaviour, frames, units and known bugs are documented there; don't re-derive them from scratch.

## Rules

- **Branching** ([docs/development-workflow.md](docs/development-workflow.md)):
  - `master` is flight-ready and only receives fast-forwards from `Development` during a flight release.
  - All work happens on `feature/<name>` branches cut from `Development`.
  - Before merging back: merge the latest `Development` into the feature branch and re-test.
  - Never commit to `master` directly.
- **Submodules:**
  - Library changes are committed inside the submodule, on a matching feature branch.
  - Never leave a submodule dirty, and never pin an unpushed submodule commit on `Development` or `master`.
- **Placement:** vehicle-specific code goes in `src/`; reusable code goes in the matching ExVectr lib. Libraries never include from `src/`. See [docs/codebase-structure.md](docs/codebase-structure.md).
- **Conventions:**
  - Frame is NWU (X-North, Y-West, Z-Up).
  - Attitude is `[ω(3), q(4)]` with q = `[w,x,y,z]`, rotating body → reference.
  - Position is `[V(3), P(3)]`.
  - Time is `int64_t` ns. SI units throughout.
  - Details are in [docs/libs/ExVectrMath.md](docs/libs/ExVectrMath.md).
- **Safety-critical:** changes to safety logic, actuator gating, mission transitions, estimators or controllers must be verified in simulation mode. Call out flight-safety implications explicitly.
- **Docs are part of the change:** update the affected file in `docs/` in the same branch, including the Known issues table of that doc when you find or fix bugs.
- **Build:** `~/.platformio/penv/bin/pio run`. Check for new warnings in `src/`.

## Skills

- `new-feature`: start a feature branch (parent and submodules) the right way.
- `integrate-feature`: sync a feature with Development, re-test, merge.
- `flight-release`: verification gate and promotion of Development → master.
- `post-flight`: tag the flown version and record the flight.
- `update-docs`: keep `docs/` in sync with code changes.
