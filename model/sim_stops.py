"""Simulate rover stops and write them as CSV, in the same format the real
rover feed will use. One row per sample at 10 Hz, 12.5 s per stop.

Columns:
  ts       seconds since start of run
  stop_id  increments at each stop
  cell     grid square name, e.g. "D2"
  pir      Mini PIR, 0/1
  loud     Grove Loudness Sensor, ADC 0-1023 (smoothed level)
  sound    Grove Sound Sensor, ADC 0-1023 (raw envelope, used for knock peaks)
  ax,ay,az 3-axis accelerometer, g (quantised like the MMA7660, ~0.047 g/step)
  vib      SW-420 vibration, 0/1
  temp     thermistor, deg C
  co2      CO2, ppm (NDIR-like: first-order lag, updates every 2 s)
  us_cm    ultrasonic range, cm (small = probe is inside a void)
  flame    flame sensor, 0/1 (1 = flame seen)

Scenarios:
  empty, empty_void, conscious, conscious_void, unconscious_void, fire, false_trigger

Usage:
  python sim_stops.py --out run.csv                       # default demo plan
  python sim_stops.py --plan "A1:empty,D2:unconscious_void,E3:fire" --out run.csv
  python sim_stops.py --calib-out calib.csv               # 4 empty stops for baselines
"""
import argparse
import csv
import math

import numpy as np

HZ = 10
STOP_S = 12.5
TRAVEL_S = 5.0        # rover driving between stops (not recorded, CO2 keeps evolving)
WOBBLE_S = 1.5        # motor wobble at the start of each stop
CO2_TAU_S = 20.0      # sensor lag (NDIR diffusion chamber)
CO2_UPDATE_S = 2.0    # sensor update period
ACC_STEP = 0.047      # MMA7660 resolution, g

COLS = ["ts", "stop_id", "cell", "pir", "loud", "sound", "ax", "ay", "az",
        "vib", "temp", "co2", "us_cm", "flame"]

# What is physically going on at each kind of square.
SCENARIOS = {
    "empty":            dict(void=False),
    "empty_void":       dict(void=True),
    "conscious":        dict(void=False, knocks=True, pir=True, voice=True),
    "conscious_void":   dict(void=True, knocks=True, pir=True, voice=True, co2_src=400, heat=2.5),
    "unconscious_void": dict(void=True, co2_src=400, heat=2.5),
    "fire":             dict(void=False, fire=True, co2_src=300, heat=5.0),
    "false_trigger":    dict(void=False, bump=True, temp_drift=2.0),
}

DEMO_PLAN = ("A1:empty,B1:empty,C1:false_trigger,D1:empty,D2:unconscious_void,"
             "E2:empty,E3:fire,F3:empty,F4:conscious,E4:empty_void")


def parse_plan(text):
    plan = []
    for item in text.split(","):
        cell, scen = item.strip().split(":")
        if scen not in SCENARIOS:
            raise SystemExit(f"unknown scenario {scen!r}; choose from {list(SCENARIOS)}")
        plan.append((cell.strip().upper(), scen))
    return plan


def simulate(plan, seed=0):
    rng = np.random.default_rng(seed)
    n = int(STOP_S * HZ)
    dt = 1.0 / HZ
    rows = []
    t = 0.0
    room = 900.0                 # busy hall
    co2_true = room              # what the sensor chamber has equilibrated to
    co2_shown = room
    next_update = 0.0

    for stop_id, (cell, scen) in enumerate(plan):
        sc = SCENARIOS[scen]
        # Travel: room drifts, the chamber relaxes towards room air.
        room += rng.normal(0, 5)
        co2_true = room + (co2_true - room) * math.exp(-TRAVEL_S / CO2_TAU_S)
        t += TRAVEL_S
        target = room + sc.get("co2_src", 0)

        # Events inside this stop (seconds since stop start).
        knocks = []
        if sc.get("knocks"):
            for start in (rng.uniform(2.5, 4.0), rng.uniform(7.0, 9.0)):
                knocks += [start + k * rng.uniform(0.45, 0.6) for k in range(3)]
        pir_win = (rng.uniform(2.0, 8.0),) if sc.get("pir") else ()
        voice = rng.uniform(3.0, 9.0) if sc.get("voice") else None
        bump = rng.uniform(3.0, 10.0) if sc.get("bump") else None
        temp0 = 24.0 + rng.normal(0, 0.1)

        for i in range(n):
            ts = i * dt
            wobble = ts < WOBBLE_S

            # Accelerometer
            sig = 0.25 if wobble else 0.025
            ax, ay, az = rng.normal(0, sig), rng.normal(0, sig), 1.0 + rng.normal(0, sig)
            # Firmware reports the PEAK over each 100 ms window (a knock lasts
            # < 100 ms, so an instantaneous sample would often miss it).
            hit = 0.0
            for k in knocks:
                if -dt < ts - k < 0.2:
                    hit = max(hit, 0.45 * math.exp(-max(0.0, ts - k) / 0.08))
            if bump is not None and -dt < ts - bump < 0.1:
                hit = max(hit, 0.6)
            az += hit
            ax, ay, az = (round(v / ACC_STEP) * ACC_STEP for v in (ax, ay, az))
            vib = int(abs(math.sqrt(ax * ax + ay * ay + az * az) - 1.0) > 0.3)

            # Sound (raw) and loudness (smoothed level)
            sound = 300 + rng.normal(0, 25) + (120 if wobble else 0)
            loud = 120 + rng.normal(0, 8) + (60 if wobble else 0)
            if hit > 0.05:
                sound += 450 * hit / 0.45
                loud += 150 * hit / 0.45
            if voice is not None and 0 <= ts - voice < 1.5:
                sound += rng.normal(60, 20)
                loud += 80
            sound = float(np.clip(sound, 0, 1023))
            loud = float(np.clip(loud, 0, 1023))

            # PIR: Grove Mini PIR holds high ~2 s after a trigger
            pir = int(any(0 <= ts - w < 2.0 for w in pir_win))

            # Temperature
            temp = temp0 + rng.normal(0, 0.08)
            if sc.get("heat"):
                temp += sc["heat"] * (1 - math.exp(-ts / 3.0))
            if sc.get("temp_drift"):
                temp += sc["temp_drift"] * ts / STOP_S

            # CO2: first-order lag, sample-and-hold every 2 s, sensor noise
            co2_true = target + (co2_true - target) * math.exp(-dt / CO2_TAU_S)
            abs_t = t + ts
            if abs_t >= next_update:
                co2_shown = co2_true + rng.normal(0, 6)
                next_update = abs_t + CO2_UPDATE_S

            us = (6 if sc["void"] else 80) + rng.normal(0, 1 if sc["void"] else 4)
            flame = int(bool(sc.get("fire")))

            rows.append([round(abs_t, 2), stop_id, cell, pir, round(loud, 1), round(sound, 1),
                         round(ax, 3), round(ay, 3), round(az, 3), vib, round(temp, 2),
                         round(co2_shown, 1), round(us, 1), flame])
        t += STOP_S
    return rows


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLS)
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", default=DEMO_PLAN, help="comma list of CELL:scenario")
    ap.add_argument("--out", default="run.csv")
    ap.add_argument("--calib-out", help="also write 4 empty stops to this file for baselines")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    plan = parse_plan(a.plan)
    write_csv(simulate(plan, a.seed), a.out)
    print(f"wrote {a.out}: {len(plan)} stops")
    if a.calib_out:
        write_csv(simulate(parse_plan("A1:empty,B1:empty,C1:empty,D1:empty"), a.seed + 1000), a.calib_out)
        print(f"wrote {a.calib_out}: 4 empty stops")


if __name__ == "__main__":
    main()
