# Development Workflow

> How branches, submodules, flight releases and backups work in this project. This is binding for everyone, including Claude sessions.

## Branch model

| Branch | Role | Who writes to it |
|---|---|---|
| `master` | **Flight-ready.** The firmware that is known (or intended) to fly. Must always build. | Only fast-forward/merge from `Development` during a flight release. Never commit directly. |
| `Development` | **Integration.** All finished features merged together; used for simulation and verification before the next flight. Must always build. | Merges from feature branches only (plus trivial doc/pin fixes). |
| `feature/<name>` | One feature, fix or experiment. Branched from `Development`. | Free to commit anything. |
| tag `flight/NNN` | **Immutable backup** of `master` exactly as flown in flight NNN. | Created once per flight, never moved. |

```
master      ──●───────────────────────────●──────────►   (tag flight/001, flight/002 …)
               \                         ↗ (release: ff-merge after sim + verification)
Development ────●────●─────────●────────●──────────►
                 \       ↗      \     ↗
feature/x         ●──●──●        \   /
feature/y                         ●─●
```

Old remote branches `BellyFlopMission` and `Hovertest` predate this model; treat them as archived.

## Feature lifecycle

1. **Start** from an up-to-date `Development`:
   ```sh
   git checkout Development && git pull
   git submodule update --init --recursive
   git checkout -b feature/<short-name>
   ```
2. **Implement and test on the branch** in isolation: build (`pio run`), run the simulation mode (telecommand `SimulationMode`, see [project/overview.md](project/overview.md#simulation-mode)), bench-test hardware if relevant.
3. **Integrate the newest Development into the branch** and test *again*:
   ```sh
   git fetch && git merge origin/Development      # merge, not rebase, once the branch is pushed
   git submodule update --init --recursive
   pio run   # + re-run the same sim/bench tests
   ```
4. **Merge into Development** only when the feature works on its own *and* on top of the latest Development:
   ```sh
   git checkout Development && git merge --no-ff feature/<short-name>
   pio run && git push
   ```
   Then delete the feature branch (local + remote).
5. **Update docs in the same branch** (see "Documentation rule" below). A feature is not done until `docs/` reflects it.

## Working with the ExVectr submodules

The parent repo pins each library to an exact commit. That pin, not the library's branch, defines what is built.

- **Library changes belong in the library repo.** If a feature touches a lib, create a branch of the *same name* in that submodule, branched from the lib's `Development`:
  ```sh
  cd lib/ExVectrDSP && git checkout Development && git pull && git checkout -b feature/<short-name>
  ```
  Commit there, then in the parent `git add lib/ExVectrDSP` to pin the new commit on the parent feature branch.
- **Merging a feature that touches libs:** merge the lib's feature branch into the lib's `Development` *first*, push it, then re-pin the parent to that merge commit, then merge the parent feature branch.
- **Never pin an unpushed submodule commit on `Development` or `master`.** A clone would fail to check it out. Check with:
  ```sh
  git submodule foreach 'git branch -r --contains HEAD | grep -q . || echo "UNPUSHED: $name"'
  ```
- **No uncommitted changes inside submodules** when committing the parent: `git status` must not show ` m lib/...` (lowercase m = dirty submodule work tree). The parent cannot record those changes; the flown firmware would differ from what is in git.
- Library default branches: most use `master`; `ExVectrDSP` and `ExVectrPackets` use `main`. `Development` exists in all.

## Flight release (Development → master)

Run this before a test flight. Claude: use the `flight-release` skill.

1. **Verification gate on Development**:
   - Clean build: `pio run -t clean && pio run`, with no new warnings in `src/`.
   - Simulation of the full mission chain passes (see [project/overview.md](project/overview.md#simulation-mode)).
   - Bench actuator test passes (TVC + flaps move correctly and in the right direction).
   - No dirty or unpushed submodules (commands above).
   - `docs/` is up to date; `docs/flight-log.md` has an entry for the planned flight with its goals.
2. **Promote:** `git checkout master && git merge --ff-only Development && git push`. If ff is impossible, someone committed to master; stop and resolve that first.
3. **Flash from master** and fly.

## Post-flight backup

After every flight, *before* any further development:

```sh
git checkout master
git tag -a flight/NNN -m "Flight NNN, YYYY-MM-DD: <one-line outcome>"
git submodule foreach 'git tag -a starship/flight/NNN -m "Pinned by ExVectr-Testing flight/NNN"'
git push origin flight/NNN
git submodule foreach 'git push origin starship/flight/NNN'
```

- The parent tag pins every submodule commit, so `git checkout flight/NNN && git submodule update` reproduces the exact firmware.
- The submodule tags (`starship/flight/NNN`, prefixed because the libraries are shared with other projects) protect those commits from being lost if a library branch is ever rebased or deleted.
- Record the flight in `docs/flight-log.md`: date, tag, mission flown, outcome, anomalies, links to logs, follow-up issues.
- **Flight NNN numbering:** zero-padded, sequential, never reused. A flight that is aborted on the pad still gets a number if the vehicle was armed.

## Commit conventions

- Imperative subject ≤ 72 chars (`Fix GNSS east/west sign in position KF`), body explains *why*.
- One logical change per commit. Formatting-only changes go in separate commits.
- Bumping a submodule pin: `Pin ExVectrDSP: <what changed>`.
- Never commit `.pio/`, secrets, or vendored drivers under `src/gnssdriver` and `src/sx12xxAL` (gitignored).

## Documentation rule

`docs/` is versioned with the code so every tag carries matching documentation.

- Any change to behaviour, interfaces, tuning, wiring, frames/units, safety logic or workflow updates the relevant doc **in the same branch**.
- Library docs carry `Last verified against commit:`. Update it when you re-verify a library doc.
- Bugs found but not fixed go into the "Known issues" table of the relevant library or project doc. Remove them when fixed.
