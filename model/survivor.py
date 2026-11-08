"""Chance-of-life model for rover stops.

The rover stops on a square for ~12.5 s and streams sensor samples to the Pi
(CSV, see sim_stops.py for the columns). For each stop we compute features,
turn each into a likelihood ratio (LR), fuse them with Bayes in log-odds, and
output P(life) for that square with the reasons.

Pipeline per stop:
  1. split rows by stop_id, drop the first 1.5 s (motor wobble)
  2. features: PIR, loudness z, knock count, CO2 slope/rise, temp rise, in-void, flame
  3. evidence: feature -> LR (table in CFG), one LR per group so correlated
     sensors are not double counted; flame gates CO2/temp; void sharpens CO2
  4. fusion: logit(prior) + sum(log LR) -> sigmoid, clamped to [2%, 98%]

Usage:
  python survivor.py run.csv                       # score every stop
  python survivor.py run.csv --calib calib.csv     # baselines from an empty run
  python survivor.py run.csv --live                # follow the file as rows arrive
  python survivor.py run.csv --mqtt localhost      # also publish forge/event
"""
import argparse
import csv
import json
import math
import os
import time
from collections import OrderedDict

import numpy as np

# ---------------------------------------------------------------- config
# Everything tunable lives here. Tune on site against real readings.
CFG = dict(
    hz=10,
    settle_s=1.5,            # drop motor wobble at the start of a stop
    min_window_s=4.0,        # live mode: absence/CO2 evidence only after this much data
    idle_finalize_s=2.0,     # live mode: finalize a stop if no rows arrive for this long
    prior=0.10,              # P(life) before any evidence
    p_min=0.02, p_max=0.98,
    void_cm=15,              # ultrasonic closer than this = probe is in a void
    # knocks
    knock_acc_g=0.2,         # accel magnitude jump that counts as a hit
    knock_sound_z=6.0,       # or a raw sound spike this many robust-sigmas up
    knock_loud_z=3.0,        # or a loudness sample this many sigmas up (floor 8 on the UNO Q: quiet 2 +- 2, knocks 11-24)
    knock_loud_min=8.0,
    knock_axis_g=0.15,       # or a jump on ANY single accel axis (a knock tilts/jolts one axis; |a| barely moves)
    knock_gap_s=0.3,         # hits closer than this are one knock
    knock_min=3,             # this many knocks = deliberate knocking
    # loudness
    loud_win_s=0.5,          # rolling mean window (ignores single spikes)
    loud_z=4.0,
    # CO2
    co2_fit_s=8.0,           # fit slope over the last N seconds of the stop
    # Sensor noise alone (2 s updates, +-6 ppm) gives slopes up to ~3 ppm/s and
    # rises up to ~15 ppm over 8 s; a breathing person in a void gives ~12 ppm/s.
    co2_slope_strong=5.0,    # ppm/s
    co2_slope_med=3.0,       # weak evidence needs slope >= this AND rise >= co2_rise_med
    co2_rise_strong=150.0,   # ppm above room baseline
    co2_rise_med=30.0,
    co2_falling=-0.5,        # slope below this = leftover from previous square, ignore
    # temperature (body heat only matters in a confined void)
    temp_rise=1.5,           # deg C above baseline
    # likelihood ratios
    lr=dict(
        knock=8.0, loud=3.0, signal_absent=0.75,
        pir=6.0, pir_absent=0.65,
        co2_strong_void=15.0, co2_strong_open=6.0, co2_med=2.0,
        co2_absent_void=0.6, co2_absent_open=0.9,
        temp_void=2.0,
    ),
)

COLS_NUM = ["ts", "pir", "loud", "sound", "ax", "ay", "az", "vib", "temp", "co2", "us_cm", "flame"]
CELL_M = 0.25


# ---------------------------------------------------------------- loading
def load_stops(path):
    """CSV -> OrderedDict stop_id -> {'cell': str, col: np.array, ...}."""
    stops = OrderedDict()
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            sid = int(r["stop_id"])
            s = stops.setdefault(sid, {"cell": r["cell"], **{c: [] for c in COLS_NUM}})
            for c in COLS_NUM:
                s[c].append(float(r[c]))
    for s in stops.values():
        for c in COLS_NUM:
            s[c] = np.asarray(s[c])
    return stops


def trim(stop, cfg=CFG):
    """Drop the settle period. Returns a new dict."""
    t = stop["ts"] - stop["ts"][0]
    keep = t >= cfg["settle_s"]
    out = {"cell": stop["cell"]}
    for c in COLS_NUM:
        out[c] = stop[c][keep]
    out["t"] = t[keep]
    return out


def mad(x):
    return float(np.median(np.abs(x - np.median(x)))) * 1.4826


# ---------------------------------------------------------------- calibration
def calibrate(stops, cfg=CFG):
    """Baselines from stops that are (mostly) empty. Uses medians across stops,
    so it is still OK if a few stops in the run contain something."""
    per = [trim(s, cfg) for s in stops.values()]
    per = [s for s in per if len(s["ts"]) > 5]

    def med_of(key, fn):
        return float(np.median([fn(s[key]) for s in per]))

    mag = lambda s: np.sqrt(s["ax"] ** 2 + s["ay"] ** 2 + s["az"] ** 2)
    return dict(
        loud_med=med_of("loud", np.median), loud_sig=max(med_of("loud", mad), 2.0),
        sound_med=med_of("sound", np.median), sound_sig=max(med_of("sound", mad), 5.0),
        acc_sig=max(float(np.median([mad(mag(s)) for s in per])), 0.02),
        temp_base=med_of("temp", np.median),
        room_co2=med_of("co2", np.median),
    )


# ---------------------------------------------------------------- features
def count_events(mask, t, gap):
    """Rising edges of a boolean series, merging edges closer than `gap` s."""
    n, last = 0, -1e9
    prev = False
    for m, ti in zip(mask, t):
        if m and not prev and ti - last >= gap:
            n += 1
            last = ti
        elif m:
            last = ti
        prev = m
    return n


def features(stop, base, cfg=CFG):
    s = stop
    t = s["t"]
    f = {"cell": s["cell"], "dur_s": float(t[-1] - t[0]) if len(t) else 0.0}
    if len(t) < 5:
        return f

    # A PIR that is high for the WHOLE stop is stuck/saturated (seen on the real UNO Q: 79/79 samples high in an empty
    # room). Only a PIR that changes state during the stop is evidence; a stuck one counts as neither motion nor absence.
    f["pir_stuck"] = bool(s["pir"].min() > 0.5)
    f["pir"] = bool(s["pir"].max() > 0.5) and not f["pir_stuck"]

    win = max(1, int(cfg["loud_win_s"] * cfg["hz"]))
    roll = np.convolve(s["loud"], np.ones(win) / win, mode="valid")
    f["loud_z"] = float((roll.max() - base["loud_med"]) / base["loud_sig"])

    mag = np.sqrt(s["ax"] ** 2 + s["ay"] ** 2 + s["az"] ** 2)
    acc_dev = np.abs(mag - np.median(mag))
    sound_z = (s["sound"] - base["sound_med"]) / base["sound_sig"]
    axis_dev = np.max([np.abs(s[k] - np.median(s[k])) for k in ("ax", "ay", "az")], axis=0)
    loud_hit = s["loud"] > max(cfg["knock_loud_min"], base["loud_med"] + cfg["knock_loud_z"] * base["loud_sig"])
    hit = ((acc_dev > max(cfg["knock_acc_g"], 6 * base["acc_sig"])) | (sound_z > cfg["knock_sound_z"])
           | loud_hit | (axis_dev > max(cfg["knock_axis_g"], 4 * base["acc_sig"])))
    f["knocks"] = count_events(hit, t, cfg["knock_gap_s"])

    # CO2: slope over the last co2_fit_s seconds, and rise above room
    sel = t >= t[-1] - cfg["co2_fit_s"]
    if sel.sum() >= 3 and np.ptp(t[sel]) > 1.0:
        f["co2_slope"] = float(np.polyfit(t[sel], s["co2"][sel], 1)[0])
    else:
        f["co2_slope"] = 0.0
    tail = t >= t[-1] - 2.0
    f["co2_rise"] = float(s["co2"][tail].mean() - base["room_co2"])

    f["temp_rise"] = float(s["temp"][tail].mean() - base["temp_base"])
    f["in_void"] = bool(np.median(s["us_cm"]) < cfg["void_cm"])
    f["flame"] = bool(s["flame"].max() > 0.5)
    return f


# ---------------------------------------------------------------- evidence + fusion
def evidence(f, cfg=CFG):
    """Features -> list of (group, LR, reason). One LR per group."""
    L = cfg["lr"]
    ev = []
    if "pir" not in f:
        return ev
    # Early in a stop, "nothing seen yet" is not evidence, and the CO2 slope
    # is too noisy. Only positive evidence counts until min_window_s of data.
    early = f["dur_s"] < cfg["min_window_s"]
    if early:
        L = dict(L, signal_absent=1.0, pir_absent=1.0, co2_absent_void=1.0, co2_absent_open=1.0)

    # Signalling: knocking beats loudness; both come from the same act.
    if f["knocks"] >= cfg["knock_min"]:
        ev.append(("signal", L["knock"], f"{f['knocks']} knocks"))
    elif f["loud_z"] >= cfg["loud_z"]:
        ev.append(("signal", L["loud"], f"sound z={f['loud_z']:.1f}"))
    else:
        ev.append(("signal", L["signal_absent"], "no knocks/sound"))

    # Motion
    if f.get("pir_stuck"):
        ev.append(("motion", 1.0, "PIR high all stop (stuck?) ignored"))
    elif f["pir"]:
        ev.append(("motion", L["pir"], "PIR motion"))
    else:
        ev.append(("motion", L["pir_absent"], "no motion"))

    # Breath (CO2). Fire makes CO2 too, so flame gates it.
    void = f["in_void"]
    if f["flame"]:
        ev.append(("breath", 1.0, "flame: CO2/temp ignored"))
    elif early:
        ev.append(("breath", 1.0, "CO2: collecting"))
    elif f["co2_slope"] < cfg["co2_falling"]:
        ev.append(("breath", 1.0, f"CO2 falling ({f['co2_slope']:+.1f} ppm/s), leftover, ignored"))
    elif f["co2_slope"] >= cfg["co2_slope_strong"] or f["co2_rise"] >= cfg["co2_rise_strong"]:
        lr = L["co2_strong_void"] if void else L["co2_strong_open"]
        ev.append(("breath", lr, f"CO2 {f['co2_slope']:+.1f} ppm/s, {f['co2_rise']:+.0f} ppm"))
    elif f["co2_slope"] >= cfg["co2_slope_med"] and f["co2_rise"] >= cfg["co2_rise_med"]:
        ev.append(("breath", L["co2_med"], f"CO2 weak rise {f['co2_slope']:+.1f} ppm/s"))
    else:
        lr = L["co2_absent_void"] if void else L["co2_absent_open"]
        ev.append(("breath", lr, "no CO2 rise" + (" in void" if void else "")))

    # Body heat: only meaningful in a confined void, and not near fire.
    if void and not f["flame"] and f["temp_rise"] >= cfg["temp_rise"]:
        ev.append(("heat", L["temp_void"], f"temp {f['temp_rise']:+.1f} C in void"))
    return ev


def fuse(ev, cfg=CFG):
    p0 = cfg["prior"]
    lo = math.log(p0 / (1 - p0)) + sum(math.log(lr) for _, lr, _ in ev)
    p = 1 / (1 + math.exp(-lo))
    return min(max(p, cfg["p_min"]), cfg["p_max"])


def score_stop(stop, base, cfg=CFG):
    f = features(trim(stop, cfg), base, cfg)
    ev = evidence(f, cfg)
    p = fuse(ev, cfg) if ev else cfg["prior"]
    reasons = [f"{r} x{lr:g}" for _, lr, r in ev]
    return p, reasons, f


def update_room(base, f, cfg=CFG):
    """Track the room CO2 baseline from open, quiet, flame-free squares."""
    if f.get("in_void") is False and not f.get("flame") and abs(f.get("co2_slope", 9)) < 1.0:
        base["room_co2"] = 0.7 * base["room_co2"] + 0.3 * (base["room_co2"] + f["co2_rise"])


# ---------------------------------------------------------------- output
# Stop names written by the rover navigator (S1, E2, ...) -> their position, from tools/navigator/sensor_tag.py.
# Checked first: "E1" is both an event label and a grid square name.
TAGS_PATH = os.path.expanduser("~/uno-q-readings/tags.json")
RESULTS_PATH = os.path.expanduser("~/uno-q-readings/results.json")


def _tags():
    try:
        with open(TAGS_PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def cell_xy(cell):
    t = _tags().get(cell)
    if t is not None:
        return t.get("x_m", t.get("x")), t.get("y_m", t.get("y"))
    try:
        col = ord(cell[0].upper()) - ord("A")
        row = int(cell[1:]) - 1
        return (col + 0.5) * CELL_M, (row + 0.5) * CELL_M
    except (ValueError, IndexError):
        return None, None


def make_event(sid, cell, p, reasons, final=False):
    x, y = cell_xy(cell)
    return dict(id=f"surv-{sid}", ts=time.time(), kind="SURVIVOR_PROB", x=x, y=y,
                score=round(p, 3), anomalous=p > 0.6, source="survivor_model",
                detail=dict(cell=cell, tag=cell, p=round(p, 3), reasons=reasons, final=final, verdict=label(p)))


def save_result(ev, path=None):
    """Latest P(life) per stop name -> results.json (atomic), read by the navigator page."""
    path = path or RESULTS_PATH
    d = ev["detail"]
    try:
        try:
            with open(path) as f:
                res = json.load(f)
        except (OSError, ValueError):
            res = {}
        res[d["tag"]] = {"p": d["p"], "verdict": d["verdict"], "reasons": d["reasons"], "final": d["final"],
                         "ts": ev["ts"], "stop_id": ev["id"]}
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(res, f, indent=1)
        os.replace(tmp, path)
    except OSError as e:
        print(f"could not write {path}: {e}")


def label(p):
    return "LIKELY - send responders" if p > 0.6 else ("UNCERTAIN - recheck" if p > 0.2 else "unlikely")


class Publisher:
    def __init__(self, host):
        self.c = None
        if host:
            try:
                import paho.mqtt.client as mqtt
                try:
                    self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)     # paho-mqtt 2.x (as in pi/bus.py)
                except AttributeError:
                    self.c = mqtt.Client()                                   # paho-mqtt 1.x
                self.c.connect(host, 1883)
                self.c.loop_start()
            except Exception as e:                                           # scoring + results.json still work
                print(f"MQTT off ({e}); results.json is still written")
                self.c = None

    def send(self, ev):
        if RESULTS_PATH:
            save_result(ev)
        if self.c:
            self.c.publish("forge/event", json.dumps(ev))


# ---------------------------------------------------------------- modes
def run_batch(path, base, pub, cfg=CFG):
    stops = load_stops(path)
    if base is None:
        base = calibrate(stops, cfg)
    results = []
    for sid, s in stops.items():
        p, reasons, f = score_stop(s, base, cfg)
        update_room(base, f, cfg)
        results.append((sid, s["cell"], p, reasons))
        pub.send(make_event(sid, s["cell"], p, reasons, final=True))
        print(f"{s['cell']:>3}  {p*100:5.1f}%  {label(p):<24} {'; '.join(reasons)}")
    return results


def run_live(path, base, pub, cfg=CFG, poll_s=0.2):
    """Follow a CSV being appended to. Prints a running score every second
    while the rover is stopped, final score when the next stop starts."""
    if base is None:
        raise SystemExit("--live needs --calib (baselines from an empty run)")
    cur, cur_id, last_print = None, None, 0.0
    last_row = time.time()
    while not (os.path.exists(path) and os.path.getsize(path) > 0):
        time.sleep(poll_s)                                # wait for model_feed.py to create the file and its header
    with open(path, newline="") as f:
        header = f.readline()
        while not header.endswith("\n"):
            time.sleep(poll_s)
            header += f.readline()
        header = header.strip().split(",")
        buf = ""
        while True:
            chunk = f.readline()
            if not chunk:
                if cur is not None and time.time() - last_row > cfg["idle_finalize_s"]:
                    finish(cur_id, cur, base, pub, cfg)   # rover left / feed stopped
                    cur = None
                time.sleep(poll_s)
                continue
            last_row = time.time()
            buf += chunk
            if not buf.endswith("\n"):
                continue
            r = dict(zip(header, buf.strip().split(",")))
            buf = ""
            sid = int(r["stop_id"])
            if sid != cur_id or cur is None:
                if cur is not None:
                    finish(cur_id, cur, base, pub, cfg)
                cur, cur_id, last_print = {"cell": r["cell"], "best": None, **{c: [] for c in COLS_NUM}}, sid, 0.0
            for c in COLS_NUM:
                cur[c].append(float(r[c]))
            el = cur["ts"][-1] - cur["ts"][0]
            # publish (and latch) only once the full evidence window is in: before settle + min_window the absence and CO2
            # evidence is not counted yet, so an early score would just be the prior (or inflated) and would stick as the peak
            if el - last_print >= 1.0 and el >= cfg["settle_s"] + cfg["min_window_s"]:
                last_print = el
                p, reasons, _ = score_stop(_arr(cur), base, cfg)
                if cur["best"] is None or p > cur["best"][0]:
                    cur["best"] = (p, reasons)           # the stop's score only goes UP: a knock early in the dwell is
                bp, br = cur["best"]                     # not undone by quiet seconds afterwards
                print(f"  {cur['cell']} t={el:4.1f}s  {p*100:5.1f}%  (stop peak {bp*100:5.1f}%)")
                pub.send(make_event(cur_id, cur["cell"], bp, br))


def _arr(s):
    return {"cell": s["cell"], **{c: np.asarray(s[c]) for c in COLS_NUM}}


def finish(sid, s, base, pub, cfg):
    p, reasons, f = score_stop(_arr(s), base, cfg)
    update_room(base, f, cfg)
    best = s.get("best")
    if best is not None and best[0] > p:                 # final = highest score reached during the stop
        p, reasons = best[0], list(best[1]) + ["peak during stop"]
    print(f"{s['cell']:>3}  FINAL {p*100:5.1f}%  {label(p)}  {'; '.join(reasons)}")
    pub.send(make_event(sid, s["cell"], p, reasons, final=True))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--calib", help="CSV of empty stops for baselines (else: medians of this run)")
    ap.add_argument("--live", action="store_true", help="follow the file as rows arrive")
    ap.add_argument("--mqtt", help="broker host; publish forge/event")
    ap.add_argument("--out", help="write results CSV")
    ap.add_argument("--tags", default=TAGS_PATH, help="stop name -> position, written by the rover navigator")
    ap.add_argument("--results", default=RESULTS_PATH, help="latest P(life) per stop name (read by the navigator page); '' = off")
    a = ap.parse_args()
    globals()["TAGS_PATH"], globals()["RESULTS_PATH"] = os.path.expanduser(a.tags), (os.path.expanduser(a.results) if a.results else "")

    base = calibrate(load_stops(a.calib)) if a.calib else None
    pub = Publisher(a.mqtt)
    if a.live:
        run_live(a.csv, base, pub)
        return
    res = run_batch(a.csv, base, pub)
    if a.out:
        with open(a.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stop_id", "cell", "p", "reasons"])
            for sid, cell, p, reasons in res:
                w.writerow([sid, cell, round(p, 3), "; ".join(reasons)])


if __name__ == "__main__":
    main()
