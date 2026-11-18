---
name: integrate-feature
description: Bring a finished feature branch up to date with Development, re-test, and merge it into Development (parent repo and submodules in the correct order). Use when a feature works on its own and should be integrated.
---

# Integrate a feature into Development

Follow docs/development-workflow.md "Feature lifecycle" and "Working with the ExVectr submodules".

1. **Pre-checks:** the feature builds, its own tests and sims pass, and the docs are updated (use the `update-docs` skill). The work tree is clean, with no dirty submodules.
2. **Sync with Development:**
   - For each submodule with a `feature/<name>` branch: `git fetch && git merge origin/Development`, resolve conflicts, build.
   - In the parent: `git fetch && git merge origin/Development`. Resolve submodule pointer conflicts by pinning the merged submodule commit. Then run `git submodule update --init --recursive`.
3. **Re-test on the merged state:** clean build (`pio run -t clean && pio run`) with no new `src/` warnings, plus the same simulation and bench checks as before. Report the results honestly. If anything fails, stop.
4. **Merge the libraries first:** in each touched submodule, `git checkout Development && git merge --no-ff feature/<name>`. Push only after the user confirms pushing.
5. **Re-pin the parent feature branch** to those submodule Development merge commits and commit (`Pin <Lib>: <what>`).
6. **Merge the parent:** `git checkout Development && git merge --no-ff feature/<name>`, build again, and confirm there are no unpushed pins:
   ```sh
   git submodule foreach 'git branch -r --contains HEAD | grep -q . || echo "UNPUSHED: $name"'
   ```
7. Ask before pushing and before deleting the feature branches.
