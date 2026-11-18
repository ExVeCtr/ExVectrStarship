---
name: post-flight
description: After a test flight, back up the flown master version with flight tags (parent + submodules) and record the flight in docs/flight-log.md. Use right after every flight, before further development.
---

# Post-flight backup

Follow docs/development-workflow.md "Post-flight backup".

1. Confirm with the user which commit was flashed. It should be the current master; if it isn't, stop and clarify.
2. Determine NNN as the next number after the highest existing `flight/*` tag (`git tag -l 'flight/*'`), zero-padded to three digits.
3. Gather from the user: date, mission flown, outcome, anomalies, conditions, and where the logs/video are.
4. Tag:
   ```sh
   git tag -a flight/NNN <master-hash> -m "Flight NNN, YYYY-MM-DD: <one-line outcome>"
   git submodule foreach 'git tag -a starship/flight/NNN HEAD -m "Pinned by ExVectr-Testing flight/NNN"'
   ```
   Run the submodule tags from a checkout of the flown master, so each submodule's HEAD is the pinned commit.
5. Add the flight-log entry using the template in docs/flight-log.md, and add anomalies to the Known issues table of the affected doc. Commit these doc changes on Development, not master (master stays exactly as flown).
6. Ask before pushing: `git push origin flight/NNN` and `git submodule foreach 'git push origin starship/flight/NNN'`.
