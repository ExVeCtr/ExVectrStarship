---
name: new-feature
description: Start a new feature/fix branch in the ExVectr Starship project, branched from Development, including matching branches in any ExVectr submodules the feature will touch. Use when beginning any new development work.
---

# Start a feature

Follow docs/development-workflow.md. Steps:

1. Check that the work tree is clean: `git status` shows no staged or unstaged changes and no dirty submodules (` m lib/...`). If it isn't clean, stop and ask the user what to do with the changes.
2. Update Development:
   ```sh
   git checkout Development && git pull --ff-only
   git submodule update --init --recursive
   ```
3. Create `feature/<kebab-name>` from Development. Ask the user for the name if it isn't obvious.
4. Work out which libraries the feature touches; read docs/codebase-structure.md "What belongs where". For each one:
   ```sh
   cd lib/<Lib> && git fetch && git checkout Development && git pull --ff-only && git checkout -b feature/<same-name>
   ```
   Warn if the lib's local Development differs from what the parent pins. In that case, ask whether to start from the pinned commit or the latest lib Development.
5. Read the relevant docs (docs/libs/<Lib>.md, docs/project/*.md) and their Known issues before designing the change.
6. Build once with `~/.platformio/penv/bin/pio run` to confirm a working baseline, then report the branch(es) you created.
