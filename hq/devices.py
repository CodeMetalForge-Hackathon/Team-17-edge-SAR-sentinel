"""Where photos come from and where motion commands go. Swappable so the same autopilot runs on hardware or in sim."""
import os
import subprocess
import sys
import threading

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rover"))


class ZeroCamera:
    """Takes a still on the aerial Pi Zero over SSH (passwordless key), copies it here, returns it as an image.
    Also keeps the last photo on disk (path) so other tools can use it."""

    def __init__(self, target, path, width=1280, height=720):
        self.target, self.path, self.w, self.h = target, path, width, height
        self.lock = threading.Lock()

    def capture(self):
        opts = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=accept-new"]
        shot = ("rpicam-still -n -t 1200 --autofocus-on-capture --width %d --height %d -o /tmp/shot.jpg"
                % (self.w, self.h))
        tmp = self.path + ".part"
        with self.lock:
            for cmd in (["ssh"] + opts + [self.target, shot], ["scp", "-q"] + opts + [self.target + ":/tmp/shot.jpg", tmp]):
                try:
                    r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
                except subprocess.TimeoutExpired:
                    raise RuntimeError("camera: timed out talking to %s (Zero on? on Tailscale?)" % self.target)
                if r.returncode != 0:
                    msg = [l for l in (r.stderr or r.stdout).splitlines() if "setlocale" not in l][-1:] or ["?"]
                    raise RuntimeError("camera: %s failed: %s" % (cmd[0], msg[0]))
            os.replace(tmp, self.path)
            img = cv2.imread(self.path)
        if img is None:
            raise RuntimeError("camera: photo could not be read")
        return img


class WatchedFileCamera:
    """Uses the photos someone else puts at path (e.g. the Zero pipeline writing /home/khoj/overhead_latest.jpg).
    capture() returns the NEXT photo: newer than the last one used. If capture_cmd is given it is run first to
    ask for a photo (e.g. the ssh/rpicam command), otherwise it just waits for the file to change."""

    def __init__(self, path, capture_cmd=None, timeout_s=60):
        self.path, self.cmd, self.timeout = path, capture_cmd, timeout_s
        self.last = 0.0

    def _mtime(self):
        try:
            return os.stat(self.path).st_mtime
        except OSError:
            return 0.0

    def capture(self, fresh=True):
        import time
        if self.cmd:
            r = subprocess.run(self.cmd, shell=True, capture_output=True, text=True, timeout=self.timeout)
            if r.returncode != 0:
                tail = [l for l in (r.stderr or r.stdout).splitlines() if "setlocale" not in l][-1:] or ["?"]
                raise RuntimeError("photo command failed: " + tail[0])
        t0 = time.time()
        while fresh and self._mtime() <= self.last:
            if time.time() - t0 > self.timeout:
                raise RuntimeError("no new photo at %s within %d s" % (self.path, self.timeout))
            time.sleep(0.2)
        time.sleep(0.05)
        self.last = self._mtime()
        img = cv2.imread(self.path)
        if img is None:
            raise RuntimeError("cannot read photo %s" % self.path)
        return img


class FileCamera:
    """Reads whatever image is at path (for testing with saved photos)."""

    def __init__(self, path):
        self.path = path

    def capture(self):
        img = cv2.imread(self.path)
        if img is None:
            raise RuntimeError("camera: cannot read %s" % self.path)
        return img


class Rover:
    """RoverC-Pro over UDP via rover/rover_control.py. Each move is a short burst then STOP (firmware watchdog too)."""

    def __init__(self, dry_run=False):
        import rover_control as rc
        self.rc = rc
        self.link = rc.RoverLink(dry_run=dry_run)
        self.max_burst = rc.MAX_BURST_S

    def move(self, cmd, seconds):
        self.link.move(cmd, max(0.05, min(seconds, self.max_burst)))

    def stop(self):
        return self.link.stop()
