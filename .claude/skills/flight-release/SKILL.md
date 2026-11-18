---
name: flight-release
description: Run the pre-flight verification gate on Development and promote it to master (flight-ready). Use when the user wants to prepare firmware for a test flight.
---

# Flight release: Development → master

master must only ever contain firmware intended to fly. Be strict and report every check's result plainly.

## Gate (all must pass; report a table of check → result)
1. You are on Development. It is up to date with origin, the tree is clean and no submodule is dirty.
2. No unpushed submodule pins:
   ```sh
   git submodule foreach 'git branch -r --contains HEAD | grep -q . || echo "UNPUSHED: $name"'
   ```
3. A clean build passes (`~/.platformio/penv/bin/pio run -t clean && ~/.platformio/penv/bin/pio run`). List any warnings coming from `src/`. Note RAM1 free compared with the baseline in docs/codebase-structure.md.
4. Review `git log master..Development` and `git diff master..Development --stat`, including submodule logs (`git diff --submodule=log master..Development`). Summarise every behavioural change, and flag changes to safety logic, actuator gating, estimators, controllers and mission transitions.
5. Check the Known issues tables in docs/libs/ExVectrDSP.md, docs/libs/ExVectrControl.md, docs/project/missions.md and docs/project/hardware-and-drivers.md for High-severity items that affect this flight's mission profile. List them and ask the user to accept or block.
6. The user confirms the simulation run and the bench actuator test (TVC and flap directions) passed. You cannot verify hardware yourself, so ask.
7. docs/flight-log.md has a draft entry with the planned goals. Offer to write it.

## Promote (only after all checks pass and the user explicitly approves)
```sh
git checkout master && git merge --ff-only Development
```
If ff is impossible, stop and investigate. Never force. Ask before `git push origin master`. Then return to Development.

Remind the user to flash from master and to run the `post-flight` skill after the flight.
