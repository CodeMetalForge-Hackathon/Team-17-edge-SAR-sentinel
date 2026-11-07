# HW-B1: Arduino node bring-up

Status: PROPOSED. **Conditional.** Human bench work; part 1 can start now.

| Field | Value |
|---|---|
| Objective | Trustworthy sensor readings from n1, then proof of SW-01 and SW-02 on real parts |
| Owner | B |
| Branch | Evidence through a human commit on `codex/HW-B1`. **Any code change gets its own contract** |
| Depends on | Part 1 nothing. Part 2 needs SW-01. Part 3 needs SW-02 |
| Budget | Until Sat 18:00 for part 1. Flashing is human only |

## Part 1: now, on current firmware
1. Identify the board and FQBN. Compile and flash. Record it.
2. Serial monitor: trigger each sensor. Fix `*_ACTIVE` polarity defines by observation. Record which module each one is (Grove flame and water polarity are assumptions in the sketch).
3. **Measure the problem:** one tap, with current firmware. Record the `vib` value. Time the loop rate.
4. Bridge to MQTT. Check n1's position by tape.
5. Decide flame trigger (IR remote, no open flame at the table).
6. Use `/dev/serial/by-id/` in config if available, so nodes cannot trade ports.
7. n2 only if a second board is ready. If so, uncomment it in `config.py` with measured values.

## Part 2: after SW-01
Reflash. Ten single taps and ten sustained shakes: record `vib_hits` per window. Check LED off after a beacon. Set
`VIB_REFRACTORY_MS` from data. **If `VIB_STRUCTURAL` no longer separates tap from shake, the human retunes through
APPLY.md. Do not hide it.** Collect 5 minutes of normal data, run `python anomaly.py train`, note the window count.

## Part 3: after SW-02
Beacon with the mask off, then on. Record loudness, vibration and light values during the beacon.

## Explicit exclusions
No speech recognition: loudness summaries are not audio. No claim about structural stability.

## Allowed edits
`arduino/sensor_node/sensor_node.ino` polarity defines and `pi/config.py` measured values, by a human. `docs/evidence/HW-B1/*`.

## Pre-approved commands and runs
Flash by B only. README bridge and training commands.

## Acceptance (evidence file `docs/evidence/HW-B1/report.md`)
Everything above as tables with counts. Real numbers only.

## Risks
Normal data collected near talking teammates or with beacons going. Sensors share a table, so a tap can move all of them.

## Stop conditions
A dead sensor costs at most 30 minutes, then swap or cut it. Tell D.

## Handoff and close
Report to D. Human commits evidence by explicit path.
