"""Listens for DISPATCH_DRONE actions and flies the Crazyflie (Flow deck v2 required).

The Flow deck only knows motion RELATIVE to takeoff, so we convert map (x, y) into
"meters forward/left of the pad" using DRONE_PAD from config. Flight = take off,
fly straight to the target, hover, fly back, land.

    python drone_dispatch.py           # dry run (prints the plan), the default
    python drone_dispatch.py --fly     # real flight, ONLY with organizers present

Before wiring it to the brain, check basic flight with bitcraze's own example:
    crazyflie-lib-python/examples/.../motion_commander_demo.py
"""
import math
import sys
import time
import threading

import config
from bus import make_bus


def map_to_drone(x, y, pad=None):
    """Map-frame target -> (forward, left) meters in the drone's takeoff frame."""
    pad = pad or config.DRONE_PAD
    mx, my = x - pad["x"], y - pad["y"]
    yaw = math.radians(pad["yaw_deg"])
    fwd = math.cos(yaw) * mx + math.sin(yaw) * my
    left = -math.sin(yaw) * mx + math.cos(yaw) * my
    return fwd, left


def fly_to(fwd, left, dry_run=True):
    leg = math.hypot(fwd, left)
    if leg > config.DRONE_MAX_LEG_M:
        print(f"[drone] refused: target {leg:.2f} m away > max {config.DRONE_MAX_LEG_M} m")
        return False
    plan = f"takeoff {config.DRONE_HEIGHT_M} m -> move fwd {fwd:+.2f} left {left:+.2f} -> hover {config.DRONE_HOVER_S}s -> return -> land"
    if dry_run:
        print(f"[drone] DRY RUN: {plan}")
        return True

    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander

    cflib.crtp.init_drivers()
    print(f"[drone] FLYING: {plan}")
    with SyncCrazyflie(config.CRAZYFLIE_URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        if int(scf.cf.param.get_value("deck.bcFlow2")) != 1:
            print("[drone] no Flow deck detected, aborting")
            return False
        scf.cf.supervisor.send_arming_request(True)  # newer firmware needs arming
        time.sleep(1.0)
        with MotionCommander(scf, default_height=config.DRONE_HEIGHT_M) as mc:  # takes off on enter, lands on exit
            time.sleep(1.0)
            mc.move_distance(fwd, left, 0, velocity=0.3)
            time.sleep(config.DRONE_HOVER_S)
            mc.move_distance(-fwd, -left, 0, velocity=0.3)
            time.sleep(0.5)
    return True


class Dispatcher:
    def __init__(self, bus, dry_run=True):
        self.bus = bus
        self.dry_run = dry_run
        self.busy = threading.Lock()
        bus.subscribe("forge/action", self.on_action)

    def on_action(self, topic, a):
        if a["action"] != "DISPATCH_DRONE":
            return
        if not self.busy.acquire(blocking=False):
            print("[drone] busy, skipping new dispatch")
            return

        def run():
            try:
                fwd, left = map_to_drone(a["x"], a["y"])
                ok = fly_to(fwd, left, self.dry_run)
                self.bus.publish("forge/drone/status", {"event_id": a["event_id"], "ok": ok, "ts": time.time()})
            finally:
                self.busy.release()
        if self.dry_run:
            run()
        else:
            threading.Thread(target=run, daemon=True).start()


if __name__ == "__main__":
    wants_fly = "--fly" in sys.argv
    allowed = (not config.DRONE_DRY_RUN) or ("--force" in sys.argv)
    dry = not (wants_fly and allowed)
    if wants_fly and not allowed:
        print("config.DRONE_DRY_RUN is True; set it False (or pass --force) to really fly")
    bus = make_bus(client_id="drone")
    Dispatcher(bus, dry_run=dry)
    print(f"[drone] waiting for dispatches ({'dry run' if dry else 'LIVE'})")
    bus.loop_forever()
