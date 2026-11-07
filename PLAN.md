# Operation Lost and Found: plan

Planning pass of Saturday 2026-11-07, about 13:00 EST. Original base commit `69cf3a7`; **reconciled with teammate commit `7a9e315`** (see section 1a).
The build window closes Sunday 2026-11-08 at noon.

Related documents:
[ML plan](docs/ML_FEASIBILITY.md) · [contract index](docs/contracts/INDEX.md) ·
[template and common terms](docs/contracts/TEMPLATE.md) · [status](TODO.md) ·
[guardrail install](docs/drafts/BS-00/APPLY.md) · [prompts](docs/prompts/PROMPT_PACK.md)

**Status words.** PROPOSED: written, nobody has agreed yet. DRAFTED: a file exists under `docs/drafts/`
but is not live. INSTALLED: a human put it on its live path. VERIFIED: a named command ran and the result
is recorded. HUMAN-APPLIED: a human changed a protected path. APPROVED: an independent reviewer or human
gave a recorded verdict.

**As of this pass:** everything here is PROPOSED or DRAFTED. Nothing is INSTALLED, HUMAN-APPLIED or
APPROVED. The only VERIFIED items are the observations in section 1.

## 1. Verified current state

**Repo.** Branch `Rohan` and `main` both sit at `69cf3a7`, with a clean tree before this pass. There was no
template, `CLAUDE.md`, `AGENTS.md`, `.claude/`, manifest or active git hook. The stash on `pratik` holds a
LICENSE-only change and was left alone. The first code commit (`44e35a2`, 11:27 Saturday) is inside the
build window. **Check before judging:** `THIRD_PARTY.md` says everything in `pi/`, `arduino/` and `tests/`
was written this weekend. Git can only show when the code was committed, so a human has to confirm that claim.

**Checks actually run** (on a `git archive` export of HEAD, Python 3.11.9, no installs):

| Check | Result |
|---|---|
| `python tests/test_core.py` | 6 of 6 pass |
| Parse all `pi/` and `tests/` Python | passes |
| `python simulate.py --local` | **Could not run.** `joblib` and `scikit-learn` are not installed on this laptop, and `brain.main` crashes on the missing module (N7) |
| Behaviour probes | Real `Brain`, `decide`, `ChangeDetector` and z-test. IsolationForest was stubbed because sklearn is absent |
| Hardware, firmware compile, live MQTT | **Not run.** Nothing in this plan has been checked on real hardware |

### Sonnet findings, re-checked

| # | Claim | Verdict | Evidence |
|---|---|---|---|
| 1 | Vibration counts active loop samples, not events | **Confirmed** by reading the code | `sensor_node.ino:81` increments on every loop pass while the pin is active. The loop rate has never been measured |
| 2 | A tap can trigger STRUCTURAL before SURVIVOR | **Confirmed** in a probe, with an assumed 120 samples per tap | `policy.py:48` is checked before `:51`. The tap produced `STRUCTURAL` and a hazard zone at n1, and later survivors there are held. Sim hides this: its tap has `vib_hits = 0` (`simulate.py:28-32`) |
| 3 | Fix the measurement first, keep hazards first | **Agreed**, and contract SW-01 is written this way | A hazard invariant test is DRAFTED |
| 4 | LED may stay on after a beacon | **Confirmed** by reading the code | `sensor_node.ino:86` only writes the LED while the beacon is active. The buzzer is fine (`:85`) |
| 5 | No coordinated reset | **Partly superseded by `7a9e315`**: a `forge/control/reset` topic now exists | At `69cf3a7` there was none. The detector still has no `reset()`, so a remaining box never re-fires, and nothing reports completion. SW-03 |
| 6 | Hazard zones can duplicate | **Confirmed** by probe | `brain.py:94` always appends. Four flame windows gave 4 zones once the cooldown had passed (forced to 0 to stand in for time) |
| 7 | Missing model disables learned detection, camera still works | **Confirmed** by probe | `brain.py:62`. Tap gave nothing, flame gave FIRE, person gave SURVIVOR. See N7 for the crash cases |
| 8 | Camera-only person selects no beacon | **Confirmed** by probe | `brain.py:51` passes no `node`, so no `BEACON` |
| 9 | LiDAR scan freshness unchecked | **Confirmed** by probe | `brain.py:34` drops `ts`. A one-hour-old scan placed a person and dispatched |
| 10 | n2 disabled in config | **Confirmed** | `config.py:15` is commented out, so `node_bridge.py n2` raises `KeyError`. Sim has its own `SIM_NODES` |
| 11 | Hailo callback incomplete | **Confirmed** | `hailo_hook.py` has a publisher and instructions only. It is one person per call, rate limited |
| 12 | Beacon sound can contaminate sensors | **Plausible, not measured** | The code path exists and a synthetic buzzer window produced STRUCTURAL. Real coupling needs the bench. The 15 s cooldown stops a same-kind loop, but not a different kind |
| 13 | Parser, polarity, calibration, firmware need bench checks | **Confirmed unverified** | `LINE_RE` does match the SDK's current `printf` format (`ultra_simple/main.cpp:278`). Polarity and calibration remain open |
| 14 | Obstructions show on the dashboard, navigation is separate | **Confirmed** | `dashboard.py:62-65` draws them. `policy.path_blocked` reads `avoid_zones` only, so a dispatch goes straight through an obstruction |

### New findings from this pass

| # | Finding | Where | Handled in |
|---|---|---|---|
| N1 | **The cooldown can hide a changed decision.** Dispatched, then FIRE, then the same person inside the new zone within 15 s gives no new action. A judge would see nothing | `brain.py:79-82`, probe F8 | SW-04 |
| N2 | A person entering the scan reports OBSTRUCTION before the camera fix arrives | `brain.py:54-58`, probe F9 | SW-05 |
| N3 | **No demo region.** Anyone in a crowded hall who enters the camera or LiDAR view can fire SURVIVOR or OBSTRUCTION | no filter exists | SW-05 |
| N4 | Sim shake and tap do not match real firmware counts, and the synthetic model sets its vibration threshold near 2.8 hits | `simulate.py`, probe F13 | SW-01, HW-B1 |
| N5 | A retrained model is not distinguished from a stale one after `vib` changes meaning. The check compares names only | `anomaly.py:69` | SW-01 |
| N6 | Model, baseline and CSV paths depend on the working directory, and `.gitignore` excludes them. Starting from the wrong folder silently loses the model | `config.py:29,49,50`, `brain.py:106` | SW-06 |
| N7 | `brain.main` only catches `FileNotFoundError`. A missing `joblib` crashed it here, and a feature mismatch raises `AssertionError` | `brain.py:104-108` | SW-01, SW-06 |
| N8 | Bus docs are out of date: `sources` is really `source`, and `avoid_zones` and `forge/drone/status` are undocumented | `bus.py:9-10`, `brain.py:85`, `drone_dispatch.py:82` | SW-03 |
| N9 | `--force` allows live flight while `DRONE_DRY_RUN` stays True. **`run_pi.sh fly` (added in `7a9e315`) passes `--fly --force`, so the launcher's fly mode uses it** | `drone_dispatch.py:93`, `run_pi.sh` | SW-00 (builder edit + test), SW-08 |
| N10 | The bridge ignores the firmware's `NODE_ID`, and `/dev/ttyACM*` order can swap, so two nodes can silently trade map positions | `node_bridge.py:21` | HW-B1 |
| N11 | The README's laptop dashboard needs a broker reachable over the network. Mosquitto 2 normally listens locally only (from general knowledge, check on the Pi) | `README.md:38` | SW-06 |
| N12 | `cflib`, like every other requirement, is unpinned. `supervisor.send_arming_request` matches current `cflib` master, but the earliest version with it is unknown | `requirements.txt`, `drone_dispatch.py:53` | HW-C1 |
| N13 | Reason text says "possible collapse" and the kind says SURVIVOR. Neither has been established, and the wording must not imply it | `policy.py:49` | SW-05 |
| N14 | `core.autocrlf=true` makes working files CRLF. Raw SHA hashes differ from git's, and `git archive` also exports CRLF here | verified locally | Guardrail drafts use `git hash-object --path` |

### 1a. Reconciliation with teammate commit `7a9e315`

`Rohan` is at `7a9e315` (one commit on `69cf3a7`); local `main` and `origin/main` are still at `69cf3a7`. The commit added `run_pi.sh`, `pi/reset.py`,
a brain and dashboard reset handler, `FORGE_MQTT_HOST`, `mac/*.sh`, README and `context.md` edits. Source inspection shows these exist; **nothing here
was run, and the teammates' "works on the Pi" is unverified by us.** Re-run in scratch (draft evidence, with sklearn and joblib stubbed): `test_core.py` 6 of 6 and the 11
invariants pass on the new code.

| # | Finding | Where | Handled in |
|---|---|---|---|
| N15 | `run_pi.sh` prints "started" without checking the child, never reports a later exit, and silently trains a synthetic model in hardware modes | `run_pi.sh` | SW-06 |
| N16 | `pi/reset.py` publishes at import time, has no reason or operator, and reports nothing when done | `pi/reset.py` | SW-03 |
| N17 | The brain reset does not touch the LiDAR change detector or pending state of later contracts; no service confirms a reset | `brain.py`, `lidar_change.py` | SW-03 |
| N18 | `mac/*.sh` hard-code a login and Pi address and need `rsync`; on Windows use the equivalents in [LAPTOP_WINDOWS.md](docs/LAPTOP_WINDOWS.md) | `mac/` | SW-06 |

Planned, not changed: one launcher (`run_pi.sh`) and one reset interface (`pi/reset.py`), both extended. `run_demo.py` and `demo_ctl.py` are dropped from the plan.

## 2. Demo scope

**Physical demo, in order.** The set is a mock damaged office or storefront, with cardboard partitions and boxes.

1. Show the live map with the LiDAR outline, node, pad and demo region. Camera and LiDAR are labelled LIVE.
2. A real teammate enters the region. Camera bearing plus LiDAR range gives a **possible survivor**. The
   nearest node beacons, and the dashboard draws a **dry-run dispatch arrow**.
3. Trigger flame or water at the node (IR remote for flame, not an open flame). A hazard zone appears. The
   same person is now reported as **drone held, ground assistance** (needs N1 fixed).
4. Put a box in the scan plane. **Obstruction** appears and the map updates. State plainly that the drone
   does not route around it.
5. Play the **recorded staff-supervised flight**. No live flight at the table.
6. Show the cached damage context and how the live events reorder the reconnaissance list (if ML-03 passes its gates).
7. **Explicit reset**, then repeat.

One node (n1) is enough for all of this. n2 is optional.

**What we will not claim:** structural stability from vibration or imagery, speech or scream recognition
(`loud_*` are summaries, not audio), that the drone routes around anything, or that a damage model works indoors.

## 3. ML addition

**Recommendation: Option A, a small offline pre/post damage module, with results cached and clearly
labelled.** Reconnaissance recommendations are then updated by fresh local observations. Full reasoning is in
[ML_FEASIBILITY.md](docs/ML_FEASIBILITY.md).

The model never runs on the live demo path, and it can never create a hazard zone or a dispatch. Its only
output is a labelled prior on the operator's reconnaissance list.

**Gate.** ML-01 runs for at most 3 hours on a laptop and needs human approval of its download budget and data
licence. **Fallback 1:** show xBD or CRASAR ground truth labelled as ground truth. **Fallback 2:** cut it. The
demo in section 2 stands without it.

## 4. Architecture and evidence flow

```
Arduino --serial--> node_bridge --\                        +--> forge/action --> drone_dispatch (dry run)
LiDAR --pipe--> lidar_node --------+--> MQTT --> brain ----+--> forge/event  --> dashboard
Hailo callback --> hailo_person_app/                |      \--> node cmd beacon
                                                    v
                        forge/event + forge/action  v
Cached files (no network):                       recon.py --> forge/recon/recommendation --> dashboard panel
  demo_assets/damage_context.json   --prior only-->   ^
  demo_assets/zone_association.json --STAGED------->--+

Offline laptop only: ml/ venv + ChangeOS + subset --> damage_context.json + provenance
```

**Rules that keep the evidence honest**

| Kind of data | Label on screen | Can it change hazards or dispatch? |
|---|---|---|
| Live sensor events | LIVE | Yes, through the existing `decide()` rules |
| Simulated (`simulate.py`) | SIM | Yes, but only in sim mode |
| Model prediction | CACHED PREDICTION, with sample ID | **No** |
| Dataset labels | CACHED GROUND TRUTH, with sample ID | **No** |
| Building-to-zone mapping | STAGED BY OPERATOR | **No** |

**Message changes.** Existing topics and required payload fields stay as they are, and an invariant test pins
them. Additive only:

| Addition | Contract |
|---|---|
| `forge/control/reset`, `forge/status` | SW-03 |
| `beacon_masked` on node features, only when true | SW-02 |
| `count`, `first_ts` on hazard zones | SW-03 |
| `forge/context/damage` (retained), `forge/recon/recommendation` | ML-02, ML-03 |
| **Changed meaning:** serial `vib` and feature `vib_hits` become distinct events | SW-01 |

**Geography.** There is no real transform from a dataset building's lat/lon to our LiDAR frame, and we will not
pretend otherwise. A dataset tile is linked to a demo zone by a hand-written file marked `STAGED_MANUAL`. In a
real deployment that link would need the station's surveyed position and heading, turning building footprints
into a local east/north frame and then into the LiDAR frame. That is future work.

## 5. Critical path and ownership

```
BS-00 --> SW-01 --> SW-03 --> SW-04 --> SW-02 --> SW-05 --> SW-06 --> INT-01 --> INT-02 --> INT-03
                                                                       ^
          HW-A1 (LiDAR) -------------------------------------------- --+
          HW-B1 (node) ----------------------------------------------- +
          HW-B2 (Hailo) --> SW-07 --> HW-B3 (bearing) ----------------+
```

**Off the critical path:** HW-C1, SW-08, HW-C2 (drone evidence); ML-01, ML-02, ML-03 (ML).

| Owner | Starts now | Then |
|---|---|---|
| **A** LiDAR | HW-A1 | LiDAR side of HW-D1 and INT-01 |
| **B** Nodes and Hailo | HW-B1, HW-B2 | SW-02, SW-07, HW-B3 |
| **C** Crazyflie | HW-C1 | SW-08, HW-C2 |
| **D** Brain and integration | BS-00 | SW-01, SW-03 to SW-06, ML, INT-* (D carries the most: hand ML-01 to whoever has the free laptop) |

**Hardware does not wait for guardrails.** HW contracts are human bench work with evidence files. They do not
change application code.

**Merge order.** SW-01, SW-03, SW-04, SW-02, SW-05, SW-06 and ML-03 all touch `brain.py`, `policy.py` or
`dashboard.py`, so they run **one at a time, in that order**. SW-07, HW and ML-01 can run alongside them.

### Timeline (targets, not promises)

| Time | Milestone |
|---|---|
| Sat 13:00-13:45 | BS-00 installed. Hardware bring-up already running |
| Sat 14:00-18:00 | SW-01, SW-03, SW-04. ML-01 spike (time-boxed) |
| **Sat 18:00** | **Checkpoint 1:** node and LiDAR publish real data. ML-01 go/no-go |
| Sat 18:00-23:00 | SW-02, SW-05, SW-06, SW-07, HW-B3 |
| **Sat 23:00** | **Checkpoint 2:** full loop live on the bench through the brain (INT-01). ML-02 and ML-03 may run only once this holds |
| Sun 00:00-06:00 | Supervised flight if staff are available. ML-02/03. Fixes |
| **Sun 06:00** | **Feature freeze** |
| Sun 06:00-09:00 | INT-02 rehearsal, offline, repeated |
| **Sun 09:00** | Code freeze, only demo-breaking fixes |
| Sun 11:00 | INT-03 complete. Human tags and pushes |

## 6. Cuts and fallbacks

| If | Then |
|---|---|
| Hailo not working by Sat 23:00 | Run person detection on sim bearings, **labelled SIM**. Say so to the judges |
| LiDAR dead | `simulate.py` scenario, labelled SIM. Camera cannot be placed without range, so no live person position |
| Node dead | Use another Arduino. Otherwise use the hazard rule through sim, labelled |
| n2 not ready | Use n1 only |
| Beacon contaminates sensors and SW-02 is late | Disable the buzzer in the firmware and keep the LED |
| No staff-supervised flight possible | Show the dry-run arrow, and say the flight has not been shown. Never use someone else's video as ours |
| ML-01 fails | Fallback 1, then cut. See section 3 |
| Local `main` is behind a teammate commit | A human fast-forwards it before BS-00 (APPLY step 2); otherwise scope checks count the teammates' files as out of scope |
| Guardrail setup passes 45 min | Install the verifier, manifest and deny rules. Skip the rest. Record it in `TODO.md` |
| Behind at Sat 23:00 | Cut in this order: ML-03, ML-02, SW-05's region drawing, n2, SW-08 |

**Always cut:** SLAM, routing around obstacles, speech recognition, Hailo inference of the damage model, any
training of a damage network, automatic hazard expiry.

**Feature-freeze rule.** From Sunday 06:00 no new contract starts. A contract in progress stops unless the demo
breaks without it, and only a human can make that call.

## 7. Open human decisions

1. Who is A, B, C and D? (README's owners table is blank.)
2. ML-01: approve the budget and data licence route. See [ML_FEASIBILITY.md section 6](docs/ML_FEASIBILITY.md).
3. Which AI HAT variant do we have? `hailortcli fw-control identify` answers this (HW-B2).
4. Do we have a Flow deck v2 and a Crazyradio? Are staff available to supervise, and when? (HW-C1)
5. Do hackathon rules or the sponsor allow CC BY-NC-SA material? **Not a legal opinion.** Please ask an organizer if unsure.
