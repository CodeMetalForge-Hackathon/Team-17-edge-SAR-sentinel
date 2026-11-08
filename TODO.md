# TODO

An index only. Contracts define authority; nothing here overrides them. Last updated at planning, Sat 2026-11-07.

**Status values:** `PROPOSED`, `APPROVED`, `IN PROGRESS`, `SW PASS` (software only), `CLOSED`, `BLOCKED`, `CUT`.
Nothing is approved or installed yet. Owners are roles until the team names people.
**Baseline:** teammate commit `7a9e315` is on `Rohan`; local `main` is still `69cf3a7` and must be fast-forwarded by a human first. `run_pi.sh`, `pi/reset.py` and the reset handlers exist but are teammate-reported, not independently verified.

| ID | Status | Owner | Depends on | Branch | Evidence | Blocker |
|---|---|---|---|---|---|---|
| BS-00 | PROPOSED | D + human | none | `codex/BS-00` | `docs/evidence/BS-00/` | Human must run APPLY.md |
| DEMO-1 (campaign: SW-00, SW-01, SW-03, SW-04, SW-02, SW-05, SW-06) | PROPOSED | D (coordinator) | BS-00 | `codex/DEMO-1` | `docs/evidence/DEMO-1/` | Human approves the campaign once |
| SW-00 | PROPOSED | D | BS-00 | `codex/DEMO-1` | `docs/evidence/SW-00/` | Inside DEMO-1 |
| HW-A1 | PROPOSED | A | none | `codex/HW-A1` | `docs/evidence/HW-A1/` | Can start now |
| HW-B1 | PROPOSED | B | none | `codex/HW-B1` | `docs/evidence/HW-B1/` | Part 1 can start now |
| HW-B2 | PROPOSED | B | none | `codex/HW-B2` | `docs/evidence/HW-B2/` | Can start now. AI HAT variant unknown |
| HW-C1 | PROPOSED | C | none | `codex/HW-C1` | `docs/evidence/HW-C1/` | Can start now. Flow deck and radio unconfirmed |
| SW-01 | PROPOSED | D | BS-00 | `codex/SW-01` | `docs/evidence/SW-01/` | BS-00 |
| SW-03 (extends existing reset) | PROPOSED | D | SW-01 | `codex/SW-03` | `docs/evidence/SW-03/` | SW-01 |
| SW-04 | PROPOSED | D | SW-03 | `codex/SW-04` | `docs/evidence/SW-04/` | SW-03 |
| SW-02 | PROPOSED | B | SW-04 | `codex/SW-02` | `docs/evidence/SW-02/` | SW-04 |
| SW-05 | PROPOSED | D | SW-02 | `codex/SW-05` | `docs/evidence/SW-05/` | SW-02 |
| SW-06 (extends `run_pi.sh`) | PROPOSED | D | SW-05 | `codex/SW-06` | `docs/evidence/SW-06/` | SW-05 |
| SW-07 | PROPOSED | B | BS-00 | `codex/SW-07` | `docs/evidence/SW-07/` | BS-00 |
| SW-08 | PROPOSED | C | BS-00 | `codex/SW-08` | `docs/evidence/SW-08/` | BS-00. Better after HW-C1 |
| HW-B3 | PROPOSED | B | HW-B2, SW-07, HW-A1 | `codex/HW-B3` | `docs/evidence/HW-B3/` | Dependencies |
| HW-C2 | PROPOSED | C + staff | HW-C1, SW-08 | `codex/HW-C2` | `docs/evidence/HW-C2/` | **Flight authorization not given** |
| HW-D1 | PROPOSED | A + D | HW-A1, HW-B3 | `codex/HW-D1` | `docs/evidence/HW-D1/` | Dependencies |
| ML-01 | PROPOSED | D or free teammate | Human approval | `codex/ML-01` | `docs/evidence/ML-01/` | **Budget and licence decision** |
| ML-02 | PROPOSED | D | ML-01, SW-04, Checkpoint 2 | `codex/ML-02` | `docs/evidence/ML-02/` | Dependencies |
| ML-03 | PROPOSED | D | ML-02 | `codex/ML-03` | `docs/evidence/ML-03/` | Dependencies |
| INT-01 | PROPOSED | D + all | SW-01 to SW-07, HW-A1, HW-B1, HW-B3, HW-D1 | `codex/INT-01` | `docs/evidence/INT-01/` | Dependencies |
| INT-02 | PROPOSED | All | INT-01 | `codex/INT-02` | `docs/evidence/INT-02/` | Dependencies |
| INT-03 | PROPOSED | D + human | INT-02 | `codex/INT-03` | `docs/evidence/INT-03/` | Dependencies |
