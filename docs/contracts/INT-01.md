# INT-01: Live integration on the bench

Status: PROPOSED. **Conditional** on hardware.

| Field | Value |
|---|---|
| Objective | The full loop runs on real hardware: the demo in PLAN.md section 2 once, start to finish |
| Owner | D, with all owners |
| Branch | `codex/INT-01`. Fixes go in small new contracts, approved quickly by D |
| Depends on | SW-01 to SW-07, HW-A1, HW-B1, HW-B3, HW-D1 |
| Budget | Target by Sat 23:00. Dry-run drone only |

## Scope
Start everything with the launcher on the Pi. Walk through: person, beacon, dispatch arrow, hazard, held message, box, obstruction, reset. Check over real MQTT, which LocalBus tests cannot cover.

## Explicit exclusions
No live flight. No new features. Label anything simulated.

## Allowed edits
`docs/evidence/INT-01/*`. Measured values in `pi/config.py`, by a human.

## Pre-approved commands and runs
`python pi/preflight.py --check --live`, `./run_pi.sh` (hardware mode, dry-run), `python pi/reset.py` and the dashboard R key, the camera app command from HW-B3. All in dry-run.

## Acceptance (evidence file `docs/evidence/INT-01/report.md`)
For each step: expected, seen, delay in seconds, pass or fail. `forge/#` capture saved. Video of one full pass. Every open problem listed, with an owner.

## Risks
Timing between processes. Wi-Fi interference. Power.

## Stop conditions
A step fails: open one small fix contract. Past Sat 23:00: apply PLAN.md's cut order.

## Handoff and close
D writes the result. Physical pass is by a human; an agent cannot claim it.
