---
name: update-docs
description: Bring docs/ in sync with code changes in the ExVectr Starship project (library docs, project docs, structure, workflow). Use after any change to behaviour, interfaces, tuning, wiring, frames/units, safety or workflow, and before integrating a feature.
---

# Update documentation

1. Find what changed: `git diff Development...HEAD --stat` and `git diff --submodule=log Development...HEAD`.
2. Map each change to its doc:
   - A library's code → `docs/libs/<Lib>.md`. Update the affected sections and the `Last verified against commit:` line, and keep the standard section structure.
   - `src/mission/*`, `src/main.cpp` mission or safety logic → `docs/project/missions.md` and `docs/project/overview.md`.
   - Pins, hardware, mixers, drivers, telemetry/telecommand structs, radio ports → `docs/project/hardware-and-drivers.md`, plus the port table in `docs/codebase-structure.md`.
   - New files or folders, or changes to the dependency direction → `docs/codebase-structure.md`.
   - Workflow changes → `docs/development-workflow.md`, `CLAUDE.md` and the affected skills.
3. Known issues: remove the entries the change fixes, and add any newly found bugs to the Known issues table of the library or project doc.
4. Verify every statement you write against the code. Cite `path:line` for non-obvious facts, and re-check line numbers you touch.
5. Commit doc changes in the same feature branch as the code (`Docs: ...`).
