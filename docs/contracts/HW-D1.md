# HW-D1: Map alignment on the set

Status: PROPOSED. **Conditional** on the physical set and HW-A1, HW-B3.

| Field | Value |
|---|---|
| Objective | The dashboard's map matches the real set |
| Owner | A and D |
| Branch | Evidence through a human commit on `codex/HW-D1` |
| Depends on | HW-A1, HW-B3. SW-04 and SW-05 for the last two checks |
| Budget | 2 hours |

## Scope
1. Build the cardboard set where the demo will happen, or as close as possible. Check partitions are seen in the scan plane.
2. Tape-measure the node and pad, including yaw. Update `config.py`.
3. Capture the baseline on the finished, empty set.
4. Measure and set `DEMO_REGION` (after SW-05).
5. Person placed at 5 spots: compare map and tape.
6. Unplug LiDAR: confirm the warning appears (after SW-04).

## Explicit exclusions
No software change. New needs go to D as a contract.

## Allowed edits
`pi/config.py` measured values, by a human. `docs/evidence/HW-D1/*`.

## Acceptance (evidence file `docs/evidence/HW-D1/report.md`)
Measured values. Errors at each spot. Photo of the set. Note any spot where nearest-return is fooled by a partition.

## Risks
The judging table may differ from the bench. Plan a 10-minute re-baseline.

## Stop conditions
Common stop rule.

## Handoff and close
Report to D. Human commits by explicit path.
