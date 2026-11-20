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

## Dependency updates and bench re-verification

A library or core-API update that is meant to change no behaviour (for example the ExVectrCore `NOW()` -> `NowNs()` update, branch `feature/exvectrcore-update`) is still a flight-software change. Keep it reviewable: one library commit per library, parent commits that separate library pins from `src/` changes, and no other edits mixed in. The build alone does not prove identical behaviour, so before merging into `Development` the user re-verifies on the bench:

1. **Startup sequence:** power on, Startup -> Ready after every input (GNSS, gyro, mag, baro, both estimators) has published; display and serial log as before; Startup actuator test (`beginActuatorTest`) traces the same circle then twist with motors held off.
2. **Simulation mission:** `SimulationMode` (0xA9), Arm, `MissionBegin`; the full waypoint mission passes through Initialisation, Startup, Running and the same phases at the same times as the previous firmware (compare phase timestamps in the log), and RTH on end. Repeat for the bellyflop/freefall/RTH missions that are used for flight. Compare mission time (`missionTime`, ns) in telemetry: negative countdown, then monotonic.
3. **Telemetry:** all packets on the same ports at the same rates; `timestamp` and `missionTime` fields advance in real time (a 1 s wall-clock interval reads 1 s in the data); attitude, position and sensor values unchanged on a stationary vehicle.
4. **Link-loss failsafe:** with the ground station silent for more than 2 s the vehicle enters Failure (`radioConnectionLoss`) and actuators are gated off exactly as before; check also the 0.5 s sensor-age failure (unplug a sensor) and the 2.5 s magnetometer timeout switching a running mission to RTH.
5. **Telecommands:** Arm/Disarm, MissionBegin/MissionAbort, SystemReset (0xC5), SimulationMode, CalibStart/CalibApply/Save each behave as before.
6. **Timing:** no new scheduler misses or serial warnings; the 10 Hz safety task and the control loop keep their rates (watch the scheduler statistics if enabled).

### Network migration (T9a) bench checklist

Branch `feature/network-migration` (built on `feature/exvectrcore-update`) replaces the on-board SX1280 link with ExVectrNetwork over USB and a UART to the ExVectrLink RX module. In addition to the checklist above (run it on this branch too), the user verifies on the bench before merging:

1. **UART wiring (hardware): done 2026-11-20.** The ship node is wired to `RX_MODULE_SERIALPORT` = Serial4 (ship node TX GPIO1 to Teensy pin 16 RX4, Teensy pin 17 TX4 to ship node RX GPIO3, 3.3 V, 460800 8N1, `src/starship_hardware.h`) and verified end to end (Router shows node 1 present, vehicle telemetry arrives over the radio). Re-check after any rewiring with `tools/uart_pin_scan.py` ([project/hardware-and-drivers.md](project/hardware-and-drivers.md#5-exvectrnetwork-links-srccomms)). The old SX1280 is physically removed.
2. **USB decode:** connect the ground station (`exvectrnet-1` profile, source `vehicle_usb`). It must show the vehicle heartbeat (node 1, schema hash 0x4FE838D2 since feature/network-migration-fixes, no incompatibility banner once the ground station pins the same ExVectrPackets) and decode, with no CRC errors or resyncs: `POSE` 100 Hz, `IMU_RAW` 100 Hz, `ACTUATORS` 50 Hz, `VEHICLE_STATUS`, `MISSION`, `ESTIMATOR` 20 Hz, `GNSS` 5 Hz, `TASK_TIMING` 5 Hz, `HANDSET_TELEM` 2 Hz, `CALIBRATION` every 2 s, and the console as `LOG_TEXT` (boot messages, "All systems initialised"). The ground station's frame statistics give the rates. Values in SI: a 90° rotation reads about 1.571 rad; `mission_time` in seconds, negative in the countdown; GNSS lat/lon in radians; ground distance correct above 2 m.
3. **Ground-station presence = radioConnectionLoss (safety, unchanged semantics):** with the ground station on USB, the vehicle reaches Ready and `GS_PRESENT` is set. Close the ground station (or unplug USB): after 2 s (not earlier) the vehicle enters Failure with failure bit 5 (`radioConnectionLoss`), actuators off, as before. Reconnecting does not clear it; `SystemReset` (0xC5) does.
4. **Telecommands (TEMPORARY port 0x7F):** the ground station has no encoder for it yet. Until one exists, send the 8-byte `Telecommand` frames from a script (layout in [project/hardware-and-drivers.md](project/hardware-and-drivers.md#10-telecommands-srctelecommandhpp-temporary-adapter-on-port-0x7f), source node 0, destination 1). Repeat step 5 of the checklist above with it: Arm/Disarm, MissionBegin/MissionAbort, SystemReset, SimulationMode (toggles once per command, also with the ground station on USB and radio at the same time), CalibStart/CalibApply/Save.
5. **Simulation mission unchanged:** `SimulationMode`, Arm, `MissionBegin -10`: the same phases at the same mission times as on `feature/exvectrcore-update` (compare `MISSION.mission_time` and `VEHICLE_STATUS.mission_phase`), RTH at the end, auto-disarm. Telemetry comes from the simulator through the same topic switches.
6. **Control-task timing independent of USB:** `TASK_TIMING` reports the average (EMA) run time of the first 8 tasks (EKF, KF, safety task among them). Compare it (a) with the ground station reading USB continuously and (b) right after re-opening the port when it was closed for a minute (the host not reading, so the vehicle's USB writes find no buffer space). Run the simulation mission once over USB and once with USB unplugged (observed over the radio path, step 7) and compare the phase times. Run times, the 10 Hz safety task and the phase times must not change.
7. **Radio path (with the RX/TX modules, T6b firmware):** the ground station via the TX module decodes `VEHICLE_STATUS` 5 Hz, `AIR_POSE` 10 Hz, `HANDSET_TELEM` 2 Hz and the vehicle heartbeat; no port >= 0x80 appears over the radio. The Pocket shows the `HANDSET_TELEM` mode text (for example `RDY`, `HOVR S` in simulation). **Expected until T9b:** over the radio alone the vehicle enters Failure (`radioConnectionLoss`) 2 s after initialisation unless the ground station sends node 0 data (telecommand or `REQUEST`) at least every 2 s, because the modules drop the ground-station heartbeat.
8. **No serial text:** `pio device monitor` on the Teensy shows only binary frames. Any readable text on USB is a bug (something printing to `Serial`).

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
