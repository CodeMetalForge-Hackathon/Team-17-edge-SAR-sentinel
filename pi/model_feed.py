#!/usr/bin/env python3
"""UNO Q JSON log -> CSV for the chance-of-life model (model/survivor.py).

Follows ~/uno-q-readings/<node>.jsonl (written by pi/uno_q_log.py) and appends rows
in the model's format:
    ts, stop_id, cell, pir, loud, sound, ax, ay, az, vib, temp, co2, us_cm, flame

Which square the rover is on comes from a one-line stop file (default
~/uno-q-readings/stop.txt). Put a cell name in it when the rover stops
(`echo D2 > ~/uno-q-readings/stop.txt`); put `-` or nothing while it moves.
Each new cell starts a new stop_id. Rows are only written while stopped.

The Grove kit has no CO2 sensor, so co2 is written as a constant (--co2, default 400).
With a constant, survivor.py sees "no CO2 rise" and applies its weak no-breath factor.

Usage (on the Pi):
    python3 pi/model_feed.py --out ~/live.csv              # follow, cell from stop.txt
    python3 pi/model_feed.py --out ~/calib.csv --cell A1   # fixed cell, for quick tests
Then, in model/:
    python survivor.py ~/live.csv --live --calib ~/calib.csv
"""
import argparse
import csv
import json
import os
import pathlib
import time

COLS = ["ts", "stop_id", "cell", "pir", "loud", "sound", "ax", "ay", "az",
        "vib", "temp", "co2", "us_cm", "flame"]


def read_cell(path, fixed):
    if fixed:
        return fixed
    try:
        c = path.read_text().strip().upper()
    except FileNotFoundError:
        return None
    return c if c and c != "-" else None


def follow(path, from_start):
    while not path.exists():
        time.sleep(0.5)
    f = open(path, "r")
    if not from_start:
        f.seek(0, os.SEEK_END)
    buf = ""
    while True:
        chunk = f.readline()
        if not chunk:
            time.sleep(0.05)
            continue
        buf += chunk
        if not buf.endswith("\n"):
            continue
        line, buf = buf, ""
        try:
            yield json.loads(line)
        except ValueError:
            continue


def main():
    home = pathlib.Path.home() / "uno-q-readings"
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--node", default="n1")
    ap.add_argument("--log", help="JSONL input, default ~/uno-q-readings/<node>.jsonl")
    ap.add_argument("--stop-file", default=str(home / "stop.txt"))
    ap.add_argument("--cell", help="fixed cell (ignore the stop file)")
    ap.add_argument("--out", required=True, help="model CSV to append to")
    ap.add_argument("--co2", type=float, default=400.0)
    ap.add_argument("--from-start", action="store_true", help="also convert rows already in the log")
    a = ap.parse_args()

    log = pathlib.Path(a.log) if a.log else home / (a.node + ".jsonl")
    stop_file = pathlib.Path(a.stop_file)
    out = pathlib.Path(a.out).expanduser()
    new = not out.exists() or out.stat().st_size == 0
    stop_id, cur_cell, last_temp = -1, None, 22.0

    with open(out, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(COLS)
        print(f"following {log} -> {out}  (cell from {'--cell ' + a.cell if a.cell else stop_file})")
        for r in follow(log, a.from_start):
            if r.get("node") != a.node or "ax" not in r:
                continue                      # skip rows from the old sketch
            if "temp" in r:
                last_temp = r["temp"]
            cell = read_cell(stop_file, a.cell)
            if cell is None:
                cur_cell = None
                continue
            if cell != cur_cell:
                stop_id += 1
                cur_cell = cell
                print(f"stop {stop_id} at {cell}")
            w.writerow([f"{r['ms'] / 1000:.2f}", stop_id, cell, r.get("pir", 0), r.get("loud", 0),
                        r.get("sound", 0), r.get("ax", 0), r.get("ay", 0), r.get("az", 1),
                        r.get("vib", 0), last_temp, a.co2, r.get("us_cm", 400), r.get("flame", 0)])
            f.flush()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
