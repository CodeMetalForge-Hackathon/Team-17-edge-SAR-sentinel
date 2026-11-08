"""Transport for the Waveshare UGV01 controller (ESP32 AP "UGV", http://192.168.4.1, JSON over GET /js?json=...).

Same interface as rover_control.RoverLink (move/stop/close) so the navigator drives either rover. Differential drive:
  F/B  -> {"T":1,"L":s,"R":s} / negative       Q/E -> in-place turns {"T":1,"L":-t,"R":t} / {"T":1,"L":t,"R":-t}
Every burst is short, re-sent at 10 Hz (the controller stops on its own after ~3 s of silence), then STOP is sent
three times. Feedback {"T":130} gives the IMU yaw (degrees) used as the heading source when no marker is readable.
"""
import http.client
import json
import time
import urllib.parse

MAX_BURST_S = 0.5


class UgvCommError(Exception):
    pass


class UgvLink:
    def __init__(self, host="192.168.4.1", local_ip="192.168.4.2", speed=0.35, turn=0.5, dry_run=False):
        self.host, self.local_ip, self.speed, self.turn, self.dry_run = host, local_ip, speed, turn, dry_run
        self.sent = []
        self.last_fb = None

    def _get(self, msg, timeout=0.6):
        if self.dry_run:
            self.sent.append(msg)
            return {"T": 1001, "y": 0.0, "L": 0, "R": 0, "v": 11.5}
        q = urllib.parse.quote(json.dumps(msg, separators=(",", ":")))
        try:
            c = http.client.HTTPConnection(self.host, 80, timeout=timeout, source_address=(self.local_ip, 0))
            c.request("GET", "/js?json=" + q)
            body = c.getresponse().read().decode(errors="ignore")
            c.close()
        except OSError as e:
            raise UgvCommError("http %s: %s" % (msg, e))
        try:
            return json.loads(body.strip().splitlines()[0]) if body.strip() else {}
        except ValueError:
            return {}

    def stop(self, repeats=3):
        ok = False
        for _ in range(max(1, repeats)):
            try:
                self._get({"T": 1, "L": 0, "R": 0}, timeout=0.4)
                ok = True
            except UgvCommError:
                time.sleep(0.05)
        return ok

    def move(self, cmd, dur):
        dur = min(float(dur), MAX_BURST_S)
        lr = {"F": (self.speed, self.speed), "B": (-self.speed, -self.speed),
              "Q": (-self.turn, self.turn), "E": (self.turn, -self.turn)}.get(cmd)
        if lr is None:
            raise UgvCommError("command %r not supported by a differential-drive rover" % cmd)
        t_end = time.time() + dur
        try:
            while True:
                self._get({"T": 1, "L": lr[0], "R": lr[1]}, timeout=0.3)
                if time.time() + 0.1 >= t_end:
                    break
                time.sleep(0.08)
            left = t_end - time.time()
            if left > 0:
                time.sleep(left)
        except UgvCommError:
            for _ in range(5):                       # could not keep the burst going: make certain it stops
                if self.stop(1):
                    break
            raise
        if not self.stop():
            raise UgvCommError("STOP could not be confirmed after the burst")

    def feedback(self):
        fb = self._get({"T": 130}, timeout=0.6)
        self.last_fb = fb
        return fb

    def yaw_deg(self):
        fb = self.feedback()
        return float(fb["y"]) if "y" in fb else None

    def close(self):
        self.stop()
