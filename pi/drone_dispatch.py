"""Listens for DISPATCH_DRONE actions and flies the Crazyflie (Flow deck v2 required).

The Flow deck only knows motion RELATIVE to takeoff, so we convert map (x, y) into
"meters forward/left of the pad" using DRONE_PAD from config. Flight = preflight checks,
take off, fly straight to the target, hover, fly back, land.

    python drone_dispatch.py                   # dry run (prints the plan), the default
    python drone_dispatch.py --fly             # real flight, ONLY with organizers present
    python drone_dispatch.py --leg 0.8 0.3     # one leg (fwd, left meters), no MQTT; add --fly to fly it

Check the link and hover first with drone_check.py.
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


# --- Crazyflie helpers (cflib imported lazily so dry runs need no radio) ----------

def connect():
    """Return an un-opened SyncCrazyflie for config.CRAZYFLIE_URI."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    cflib.crtp.init_drivers()
    return SyncCrazyflie(config.CRAZYFLIE_URI, cf=Crazyflie(rw_cache="./cache"))


def read_once(scf, var, var_type="float"):
    """Read one sample of a log variable (e.g. pm.vbat)."""
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncLogger import SyncLogger
    lc = LogConfig(name="once", period_in_ms=100)
    lc.add_variable(var, var_type)
    with SyncLogger(scf, lc) as logger:
        for _, data, _ in logger:
            return data[var]


def reset_estimator(scf, timeout_s=None):
    """Reset the Kalman filter and wait until its position variance stops moving.
    Taking off before this settles is the usual cause of a jump or drift at takeoff."""
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncLogger import SyncLogger
    timeout_s = timeout_s or config.DRONE_ESTIMATOR_TIMEOUT_S
    cf = scf.cf
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")

    names = ("kalman.varPX", "kalman.varPY", "kalman.varPZ")
    lc = LogConfig(name="kalman_var", period_in_ms=100)
    for n in names:
        lc.add_variable(n, "float")
    hist = {n: [] for n in names}
    t0 = time.time()
    with SyncLogger(scf, lc) as logger:
        for _, data, _ in logger:
            for n in names:
                hist[n] = (hist[n] + [data[n]])[-10:]
            if all(len(h) == 10 and max(h) - min(h) < 0.001 for h in hist.values()):
                return True
            if time.time() - t0 > timeout_s:
                return False


def preflight(scf):
    """Flow deck present, battery OK, estimator settled. Returns (ok, message)."""
    if int(scf.cf.param.get_value("deck.bcFlow2")) != 1:
        return False, "no Flow deck v2 detected"
    vbat = read_once(scf, "pm.vbat")
    if vbat < config.DRONE_MIN_VBAT:
        return False, f"battery {vbat:.2f} V < {config.DRONE_MIN_VBAT} V, swap it"
    if not reset_estimator(scf):
        return False, "position estimate did not settle (floor texture? lighting? deck dirty?)"
    return True, f"ok (battery {vbat:.2f} V)"


def arm(scf):
    """Firmware 2025.02+ needs an arming request before the motors will spin."""
    scf.cf.supervisor.send_arming_request(True)
    time.sleep(1.0)


# --- Flight ---------------------------------------------------------------------------

def fly_to(fwd, left, dry_run=True):
    leg = math.hypot(fwd, left)
    if leg > config.DRONE_MAX_LEG_M:
        print(f"[drone] refused: target {leg:.2f} m away > max {config.DRONE_MAX_LEG_M} m")
        return False
    v = config.DRONE_SPEED_MPS
    plan = (f"takeoff {config.DRONE_HEIGHT_M} m -> move fwd {fwd:+.2f} left {left:+.2f} at {v} m/s "
            f"-> hover {config.DRONE_HOVER_S}s -> return -> land")
    if dry_run:
        print(f"[drone] DRY RUN: {plan}")
        return True

    from cflib.positioning.motion_commander import MotionCommander
    t0 = time.time()
    try:
        with connect() as scf:
            ok, msg = preflight(scf)
            print(f"[drone] preflight: {msg}")
            if not ok:
                return False
            arm(scf)
            print(f"[drone] FLYING: {plan}")
            # MotionCommander takes off on enter and lands on exit, including on an exception or Ctrl-C
            with MotionCommander(scf, default_height=config.DRONE_HEIGHT_M) as mc:
                time.sleep(1.0)
                mc.move_distance(fwd, left, 0, velocity=v)
                time.sleep(config.DRONE_HOVER_S)
                mc.move_distance(-fwd, -left, 0, velocity=v)
                time.sleep(0.5)
    except Exception as e:
        print(f"[drone] flight aborted: {e!r}")
        return False
    print(f"[drone] done in {time.time() - t0:.1f} s")
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
            ok = False
            try:
                fwd, left = map_to_drone(a["x"], a["y"])
                ok = fly_to(fwd, left, self.dry_run)
            except Exception as e:
                print(f"[drone] dispatch error: {e!r}")
            finally:
                self.bus.publish("forge/drone/status", {"event_id": a["event_id"], "ok": ok, "ts": time.time()})
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

    if "--leg" in sys.argv:
        i = sys.argv.index("--leg")
        fwd, left = float(sys.argv[i + 1]), float(sys.argv[i + 2])
        sys.exit(0 if fly_to(fwd, left, dry_run=dry) else 1)

    bus = make_bus(client_id="drone")
    Dispatcher(bus, dry_run=dry)
    print(f"[drone] waiting for dispatches ({'dry run' if dry else 'LIVE'})")
    bus.loop_forever()
