# INT-02: Offline rehearsal and repeatability

Status: PROPOSED

| Field | Value |
|---|---|
| Objective | The demo works repeatedly with no internet, and people know what to do when something fails |
| Owner | All |
| Branch | `codex/INT-02` for evidence only |
| Depends on | INT-01. Start by Sun 06:00 |
| Budget | Sun 06:00 to 09:00 |

## Scope
1. **Disconnect internet.** Wi-Fi off or a local hotspot with no uplink. Restart the Pi from cold.
2. Run the full demo **5 times**, using reset between runs. Record each.
3. Practice failures: unplug a node, stop the camera, kill the brain. Confirm the visible status and the fallback.
4. Time the 15-minute table script. Two people can each run it.
5. Confirm cached and live labels are visible and correct, if the ML panel is in.

## Explicit exclusions
No new features. No threshold changes without a recorded human reason.

## Allowed edits
`docs/evidence/INT-02/*`. Only demo-breaking fixes after a human says so.

## Pre-approved commands and runs
Same as INT-01.

## Acceptance (evidence file `docs/evidence/INT-02/rehearsal.md`)
Per run: time and pass or fail for each step. Count of false events while idle for 10 minutes. Result of each failure drill. **Report real pass counts. 5 of 5 is not required to continue, but it must be recorded honestly.**

## Risks
Venue noise and people differ from the bench.

## Stop conditions
Same failure twice: fix or cut it from the script.

## Handoff and close
D records go or no-go for each part of the script.
