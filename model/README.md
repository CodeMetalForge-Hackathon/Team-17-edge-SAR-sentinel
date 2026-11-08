# model/ — chance-of-life per square

The rover stops on a square for ~12.5 s and streams sensor samples to the Pi.
For each stop this gives **P(life)** for that square and the reasons.

Method: Bayesian evidence fusion. Start at a prior (10%), each sensor
multiplies the odds by a likelihood ratio (LR), clamp to 2-98%.
No training data needed; every number is in `CFG` at the top of `survivor.py`.

## Run (simulated data)
```
pip install numpy               # paho-mqtt only if you use --mqtt
python sim_stops.py --out run.csv --calib-out calib.csv
python survivor.py run.csv --calib calib.csv
python test_survivor.py         # scenario checks (30 seeds each)
```
Live (follows a CSV as rows are appended, prints the rising % every second):
```
python survivor.py live.csv --live --calib calib.csv [--mqtt localhost]
```
`--mqtt` publishes `forge/event` with `kind: SURVIVOR_PROB`,
`detail: {cell, p, reasons}`, `anomalous: p > 0.6`.

Custom scenario plan:
```
python sim_stops.py --plan "A1:empty,D2:unconscious_void,E3:fire,F4:conscious" --out run.csv
```
Scenarios: `empty, empty_void, conscious, conscious_void, unconscious_void, fire, false_trigger`.

## CSV format (what the rover must send, 10 Hz)
`ts, stop_id, cell, pir, loud, sound, ax, ay, az, vib, temp, co2, us_cm, flame`

- `stop_id` increments per stop; `cell` like `D2` (from the overhead marker).
- **Firmware must send the PEAK of accel |a| and raw sound over each 100 ms**,
  not an instantaneous sample. A knock lasts < 100 ms; plain 10 Hz sampling
  misses about half of them.
- `us_cm` ultrasonic range; < 15 cm = probe is inside a void.
- `flame` 1 = flame seen.

## Pipeline per stop
1. Drop the first 1.5 s (motor wobble).
2. Features: PIR fired, loudness z (0.5 s rolling), knock count (accel/sound
   peaks >= 0.3 s apart), CO2 slope over last 8 s and rise over room, temp rise,
   in-void, flame.
3. Evidence (one LR per group so correlated sensors are not double counted):

| Group | Present | Absent |
|---|---|---|
| signal | >= 3 knocks x8, else loud z>=4 x3 | x0.75 |
| motion | PIR x6 | x0.65 |
| breath | CO2 >= 5 ppm/s or +150 ppm: x15 in void, x6 open; weak (>=3 ppm/s and +30 ppm) x2 | x0.6 in void, x0.9 open |
| heat | temp +1.5 C, in void only: x2 | - |

   Gates: flame -> CO2/temp ignored. CO2 falling -> leftover from previous
   square, ignored. Live mode: absences and CO2 only count after 4 s of data.
4. Fuse in log-odds -> P(life). > 60% likely, 20-60% recheck, < 20% unlikely.

## Simulated results (demo plan)
| Scenario | P(life) |
|---|---|
| empty | ~5% |
| false_trigger (bump + temp drift) | ~16% |
| fire | ~5% |
| unconscious_void (CO2 only) | ~62% |
| conscious (knocks + PIR) | ~84% |

## Tune on site
- Calibrate with 4 empty stops (`--calib`), on the real wiring (3.3 V
  baselines differ from tutorial numbers).
- Check the CO2 noise slope on empty stops; keep `co2_slope_strong` well above it.
- Thresholds assume an NDIR-like CO2 sensor (2 s updates, ~20 s lag).
