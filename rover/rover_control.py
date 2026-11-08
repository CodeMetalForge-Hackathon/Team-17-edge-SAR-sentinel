"""RoverC-Pro UDP controller (manual).

Importing this module does NOT open a socket or send anything. Use RoverLink, or run it as a script
for the original interactive prompt:  python rover_control.py

Safety model (unchanged from the original script):
- The firmware stops by itself 400 ms after the last command (watchdog), so motion is sent as short
  bursts of repeated packets followed by an explicit STOP ("S").
- STOP is always allowed and is retried; it is also sent on exit/error.
- Burst duration is capped (MAX_BURST_S). Duration is NOT a known distance.
"""
import os
import socket
import time

ROVER_IP = os.environ.get("ROVER_IP", "192.168.4.1")
ROVER_PORT = int(os.environ.get("ROVER_PORT", "4210"))
LOCAL_IP = os.environ.get("ROVER_LOCAL_IP", "192.168.4.2")  # must be the address the firmware accepts

# key -> firmware command byte. Direction conventions are UNVERIFIED until tested on hardware.
COMMANDS = {
    "w": "F",
    "s": "B",
    "a": "L",
    "d": "R",
    "q": "Q",
    "e": "E",
    "x": "S",
}
MOVE_CMDS = {"F", "B", "L", "R", "Q", "E"}
PACKET_INTERVAL_S = 0.05
DEFAULT_BURST_S = 0.25
MAX_BURST_S = 1.0


class RoverCommError(Exception):
    """Raised when a command could not be sent; caller must treat the rover as needing STOP + new observation."""


class RoverLink:
    def __init__(self, rover_ip=ROVER_IP, port=ROVER_PORT, local_ip=LOCAL_IP, dry_run=False):
        self.addr = (rover_ip, port)
        self.local_ip = local_ip
        self.dry_run = dry_run
        self.sock = None
        self.sent = []  # dry-run log

    def open(self):
        if self.dry_run or self.sock:
            return self
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind((self.local_ip, 0))  # force packets out through the rover network
        except OSError as e:
            s.close()
            raise RoverCommError("cannot bind %s: %s" % (self.local_ip, e))
        self.sock = s
        return self

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def _send(self, cmd):
        if self.dry_run:
            self.sent.append(cmd)
            return
        if not self.sock:
            self.open()
        try:
            self.sock.sendto(cmd.encode(), self.addr)
        except OSError as e:
            raise RoverCommError("send failed: %s" % e)

    def stop(self, repeats=3):
        """Always attempt STOP; never raises. Returns True if at least one STOP was sent."""
        ok = False
        for _ in range(max(1, repeats)):
            try:
                self._send("S")
                ok = True
            except RoverCommError:
                pass
            time.sleep(0.02)
        return ok

    def move(self, cmd, duration_s=DEFAULT_BURST_S):
        """Send one command repeatedly for duration_s (capped), then STOP. cmd is a firmware letter (F B L R Q E)."""
        if cmd not in MOVE_CMDS:
            raise ValueError("not a movement command: %r" % (cmd,))
        if not (0 < duration_s <= MAX_BURST_S):
            raise ValueError("duration must be in (0, %.1f] s" % MAX_BURST_S)
        n = max(1, round(duration_s / PACKET_INTERVAL_S))
        try:
            for _ in range(n):
                self._send(cmd)
                time.sleep(PACKET_INTERVAL_S)
        except RoverCommError:
            self.stop()
            raise
        finally:
            self.stop(repeats=1)  # explicit STOP after every burst
        return n


def main():
    print("RoverC-Pro Wireless Controller")
    print("W: Forward | S: Backward")
    print("A: Left    | D: Right")
    print("Q: Rotate Left | E: Rotate Right")
    print("X: Stop | quit: Exit")
    print()
    link = RoverLink().open()
    try:
        while True:
            key = input("Command: ").strip().lower()
            if key == "quit":
                break
            if key not in COMMANDS:
                print("Invalid command")
                continue
            if key == "x":
                link.stop(repeats=1)
                print("STOP")
            else:
                link.move(COMMANDS[key], DEFAULT_BURST_S)
                print("Command completed")
    finally:
        link.stop()
        link.close()


if __name__ == "__main__":
    main()
