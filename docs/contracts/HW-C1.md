# HW-C1: Crazyflie inventory and link

Status: PROPOSED. **Conditional.** Human bench work; can start now. **No flight, no arming.**

| Field | Value |
|---|---|
| Objective | Know what we have and confirm a radio link without spinning motors |
| Owner | C |
| Branch | Evidence through a human commit on `codex/HW-C1` |
| Depends on | Nothing |
| Budget | 90 minutes |

## Scope
1. **Confirm presence of Flow deck v2 and Crazyradio.** Without both, the supervised flight in this plan cannot happen; tell D at once.
2. Install `cflib` in a venv on the machine with the radio. Record its version. Confirm `supervisor.send_arming_request` exists in that version (upstream master has it, `platform` version is deprecated).
3. Connect at the URI in config. Read `deck.bcFlow2` and battery voltage. Record firmware version.
4. Ask organizers: is supervised flight allowed? Who supervises? Where and when? Write their answer down.

## Explicit exclusions
No arming, take-off or motor test in this contract. No `--fly`. No `--force`.

## Allowed edits
`docs/evidence/HW-C1/*`.

## Pre-approved commands and runs
Read-only link and parameter checks only.

## Acceptance (evidence file `docs/evidence/HW-C1/report.md`)
Inventory. Versions. Deck reading. Battery. Organizer's answer, naming the person.

## Risks
Radio driver permissions. A brushless versus brushed firmware difference.

## Stop conditions
No Flow deck or no radio: stop. Record it. HW-C2 is then cut.

## Handoff and close
Report to D and the SW-08 builder. Human commits evidence by explicit path.
