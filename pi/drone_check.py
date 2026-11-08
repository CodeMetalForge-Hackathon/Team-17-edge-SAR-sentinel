"""Crazyflie bring-up, one step at a time. Run these in order on the machine with the Crazyradio.

    python drone_check.py --scan     # 1. list Crazyflies the radio can see (copy the URI into config.CRAZYFLIE_URI)
    python drone_check.py            # 2. connect, battery, Flow deck, estimator. Motors stay off.
    python drone_check.py --flow     # 3. stream Flow-deck position for 10 s; slide the drone by hand and watch x/y
    python drone_check.py --hover    # 4. take off to 0.3 m, hover 5 s, land. ONLY with organizers present.

Then fly one dispatch leg: python drone_dispatch.py --leg 0.5 0 --fly --force
"""
import sys
import time

import config
from drone_dispatch import connect, preflight, arm


def scan():
    import cflib.crtp
    cflib.crtp.init_drivers()
    found = cflib.crtp.scan_interfaces()
    if not found:
        print("no Crazyflie found. Is it on? Is the Crazyradio plugged in (Windows: Zadig libusb driver; Linux: udev rules)?")
    for uri, _ in found:
        print(uri)


def stream_flow(seconds=10):
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncLogger import SyncLogger
    with connect() as scf:
        ok, msg = preflight(scf)
        print(f"preflight: {msg}")
        if not ok:
            return
        lc = LogConfig(name="pos", period_in_ms=200)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "range.zrange"):
            lc.add_variable(v, "float" if v != "range.zrange" else "uint16_t")
        t0 = time.time()
        with SyncLogger(scf, lc) as logger:
            for _, d, _ in logger:
                print(f"x {d['stateEstimate.x']:+.2f}  y {d['stateEstimate.y']:+.2f}  "
                      f"z {d['stateEstimate.z']:+.2f}  zrange {d['range.zrange']} mm")
                if time.time() - t0 > seconds:
                    break


def hover(height=0.3, seconds=5.0):
    from cflib.positioning.motion_commander import MotionCommander
    with connect() as scf:
        ok, msg = preflight(scf)
        print(f"preflight: {msg}")
        if not ok:
            return
        arm(scf)
        print(f"taking off to {height} m")
        with MotionCommander(scf, default_height=height):
            time.sleep(seconds)
        print("landed")


def check():
    with connect() as scf:
        ok, msg = preflight(scf)
        print(f"{config.CRAZYFLIE_URI}: {msg}")


if __name__ == "__main__":
    if "--scan" in sys.argv:
        scan()
    elif "--flow" in sys.argv:
        stream_flow()
    elif "--hover" in sys.argv:
        hover()
    else:
        check()
