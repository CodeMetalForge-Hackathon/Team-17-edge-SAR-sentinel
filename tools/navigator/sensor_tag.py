"""Mission <-> UNO Q sensor pipeline link (Chaitanya's model, see model/README.md and pi/model_feed.py).

The UNO Q streams readings to the Pi all the time (~/uno-q-readings/n1.jsonl). It does not know where the rover is.
HQ tags the readings by time window instead:

  dwell at S1 starts -> tags.json gets S1's position, then stop.txt = "S1"   (model_feed.py tags every row "S1")
  dwell ends         -> stop.txt = "-"                                       (rows are no longer written)
  ~2 s later         -> survivor.py finalises the stop: P(life) for S1 -> forge/event + results.json
  the navigator page -> shows S1's P(life) from results.json

Every file write is atomic (tmp + os.replace). A sensor problem never stops the rover: errors are logged, not raised.
"""
import json
import os
import threading
import time


class SensorTagger:
    def __init__(self, folder="~/uno-q-readings", log=print):
        self.dir = os.path.expanduser(folder)
        self.stop_file = os.path.join(self.dir, "stop.txt")
        self.tags_file = os.path.join(self.dir, "tags.json")
        self.results_file = os.path.join(self.dir, "results.json")
        self.log = log
        self.lock = threading.Lock()
        self.current = None
        self._res, self._res_mtime = {}, None
        try:
            os.makedirs(self.dir, exist_ok=True)
            self._write(self.stop_file, "-\n")              # never start with a stale tag from a previous run
        except OSError as e:
            self.log("sensors: cannot use %s (%s)" % (self.dir, e))

    def _write(self, path, text):
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write(text)
        os.replace(tmp, path)

    def begin(self, label, kind, x, y, aspect=None, height_mm=None):
        """Start tagging readings with `label` (S1, E2, ...). x, y: normalized sandbox coordinates of the structure."""
        with self.lock:
            try:
                try:
                    tags = json.load(open(self.tags_file))
                except (OSError, ValueError):
                    tags = {}
                t = {"kind": kind, "x": round(x, 4), "y": round(y, 4), "t0": time.time()}
                if aspect and height_mm:                              # metres in the sandbox frame (origin bottom-left)
                    t["x_m"] = round(x * aspect * height_mm / 1000.0, 3)
                    t["y_m"] = round(y * height_mm / 1000.0, 3)
                tags[label] = t
                self._write(self.tags_file, json.dumps(tags, indent=1))   # position first, so the model can place the stop
                self._write(self.stop_file, label + "\n")
                self.current = label
            except OSError as e:
                self.log("sensors: could not tag %s (%s)" % (label, e))

    def end(self):
        with self.lock:
            if self.current is None:
                return
            try:
                self._write(self.stop_file, "-\n")
            except OSError as e:
                self.log("sensors: could not clear the tag (%s)" % e)
            self.current = None

    def archive(self):
        """Fresh start (page RESTART): move the old results/tags aside (kept, timestamped) and clear the tag."""
        stamp = time.strftime("%H%M%S")
        with self.lock:
            for path in (self.results_file, self.tags_file):
                try:
                    if os.path.exists(path):
                        os.replace(path, path.replace(".json", ".%s.json" % stamp))
                except OSError as e:
                    self.log("sensors: could not archive %s (%s)" % (path, e))
            try:
                self._write(self.stop_file, "-\n")
            except OSError:
                pass
            self.current, self._res, self._res_mtime = None, {}, None

    def results(self):
        """{label: {p, verdict, reasons, final, ts}} as written by model/survivor.py (cached on mtime)."""
        try:
            m = os.path.getmtime(self.results_file)
            if m != self._res_mtime:
                self._res, self._res_mtime = json.load(open(self.results_file)), m
        except (OSError, ValueError):
            pass
        return self._res
