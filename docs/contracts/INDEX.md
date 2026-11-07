# Contract index

All contracts are PROPOSED. Status lives in [TODO.md](../../TODO.md). Common terms are in [TEMPLATE.md](TEMPLATE.md).
Readiness: **Ready** means fully specified and executable once its dependencies close. **Conditional** means
details depend on hardware facts we do not have yet, and they are marked rather than guessed.

| ID | Title | Owner | Depends on | Readiness | Verifier | Critical path |
|---|---|---|---|---|---|---|
| [BS-00](BS-00.md) | Guardrail bootstrap | D + human | none | Ready | Human checks | **Yes** |
| [DEMO-1](DEMO-1.md) | **Campaign**: SW-00, SW-01, SW-03, SW-04, SW-02, SW-05, SW-06 on one branch, one approval | D (coordinator) | BS-00 | Ready | Yes, per stage | **Yes** |
| [SW-00](SW-00.md) | Launcher cannot force a flight | D | BS-00 | Ready | Yes | **Yes** |
| [SW-01](SW-01.md) | Vibration meaning and LED fix | D | BS-00 | Ready | Yes | **Yes** |
| [SW-03](SW-03.md) | Reset and hazard-zone hygiene | D | SW-01 | Ready | Yes | **Yes** |
| [SW-04](SW-04.md) | Freshness, nearest beacon, honest cooldown | D | SW-03 | Ready | Yes | **Yes** |
| [SW-02](SW-02.md) | Beacon contamination mask | B | SW-04 | Ready | Yes | **Yes** |
| [SW-05](SW-05.md) | Demo region, obstruction hold-off, wording | D | SW-02 | Ready | Yes | **Yes** |
| [SW-06](SW-06.md) | Launcher and visible startup failures | D | SW-05 | Ready | Yes | **Yes** |
| [SW-07](SW-07.md) | Hailo callback app | B | BS-00 | Ready | Yes | **Yes** |
| [SW-08](SW-08.md) | Drone preflight and `--force` removal | C | BS-00 | Conditional | Draft only (protected file) | No |
| [HW-A1](HW-A1.md) | LiDAR bring-up | A | none | Conditional | Human evidence | **Yes** |
| [HW-B1](HW-B1.md) | Arduino node bring-up | B | none (bench retest after SW-01) | Conditional | Human evidence | **Yes** |
| [HW-B2](HW-B2.md) | Hailo bring-up | B | none | Conditional | Human evidence | **Yes** |
| [HW-B3](HW-B3.md) | Camera bearing calibration | B | HW-B2, SW-07, HW-A1 | Conditional | Human evidence | **Yes** |
| [HW-C1](HW-C1.md) | Crazyflie inventory and link | C | none | Conditional | Human evidence | No |
| [HW-C2](HW-C2.md) | Supervised flight recording | C + staff | HW-C1, SW-08 | Conditional, **needs flight authorization** | Human evidence | No |
| [HW-D1](HW-D1.md) | Map alignment on the set | A + D | HW-A1, HW-B3 | Conditional | Human evidence | **Yes** |
| [ML-01](ML-01.md) | ML feasibility spike | D or free teammate | Human budget approval | Ready, **needs approval** | Gate evidence | No |
| [ML-02](ML-02.md) | Cached damage context | D | ML-01 outcome, SW-04, Checkpoint 2 | Ready | Yes | No |
| [ML-03](ML-03.md) | Reconnaissance recommendations | D | ML-02 | Ready | Yes | No |
| [INT-01](INT-01.md) | Live integration on the bench | D + all | SW-01 to SW-06, SW-07, HW-A1, HW-B1, HW-B3, HW-D1 | Conditional | Human evidence | **Yes** |
| [INT-02](INT-02.md) | Offline rehearsal, repeatability | All | INT-01 | Ready | Human evidence | **Yes** |
| [INT-03](INT-03.md) | Final docs, claims check, freeze | D + human | INT-02 | Ready | Human evidence | **Yes** |

**Why this order.** SW-01 comes first because hazard behaviour on real hardware is wrong until the measurement is
right. SW-03 before SW-04 because both change `Brain` state, and reset is needed for every later demo run. SW-02
comes after SW-04 only to avoid merge conflicts in `brain.py`; if the bench shows heavy contamination before then,
disable the buzzer in firmware as a stopgap.

**Later changes that genuinely need the stronger reviewer again:** any new topic, field or field-meaning change; any
change to hazard priority or dispatch conditions; any flight code; anything that lets context affect `decide()`; any
threshold change; any test edit. Routine implementation inside a Ready contract needs only the stage reviewer.

## Integrated baseline: `7a9e315` (teammate changes, fast-forwarded locally)

Files added or changed by teammates: `run_pi.sh`, `pi/reset.py`, `pi/brain.py` (reset handler), `pi/dashboard.py` (reset handler, R key), `pi/config.py`
(`FORGE_MQTT_HOST`), `mac/dashboard.sh`, `mac/sync_to_pi.sh`, `README.md`, `context.md`. Their report that these work on the Pi is a claim; no check of it is
recorded here. Reading the source is not hardware validation.

| Item | Decision | Contract |
|---|---|---|
| `forge/control/reset` topic, `Brain.on_reset`, dashboard clear and R key | **Keep**. Payload `{ts}` stays valid | SW-03 |
| `pi/reset.py` as the reset command | **Keep and extend** (callable `send_reset`, no publish on import, ack report). `demo_ctl.py` is **replaced** by it | SW-03 |
| LiDAR change-detector reset, reset completion status, zone dedupe | **Still open** | SW-03 |
| `run_pi.sh` as the launcher | **Keep and extend**. `run_demo.py` is **replaced** (never built) | SW-06 |
| Process readiness, child-failure reporting, placeholder-model WARN, preflight | **Still open** | SW-06 |
| `run_pi.sh fly` passing `--fly --force` | **Replace** in SW-00 (a builder edit to the unprotected launcher, pinned by a contract test); SW-08 finishes the dispatcher | SW-00, SW-08 |
| `FORGE_MQTT_HOST` environment override | **Keep** as is. Add `FORGE_MQTT_PORT` later | SW-06 |
| `mac/*.sh` | **Keep**; Windows equivalents in [LAPTOP_WINDOWS.md](../LAPTOP_WINDOWS.md) | SW-06 |
| Vibration semantics and LED fix | **Still open** (firmware and `features` unchanged) | SW-01 |
| Stale-scan protection, nearest beacon, honest cooldown | **Still open** (`on_scan` still drops `ts`) | SW-04 |
| Beacon contamination mask | **Still open** | SW-02 |
| Hailo callback integration | **Still open** | SW-07 |
| Optional disaster-data ML (ML-01 to ML-03) | **Defer**. Off the live-demo critical path; never blocks a launch | ML-* |

**Prerequisite for every contract:** local `main` contains `7a9e315`. At the time of writing, `Rohan` is at `7a9e315` and `main` and `origin/main` are at `69cf3a7`.
