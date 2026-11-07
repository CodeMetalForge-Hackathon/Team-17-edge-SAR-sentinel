"""Reset the brain and dashboard between judges: clears hazard zones, events, and cooldowns.

    python reset.py          (or press R in the dashboard window)
"""
import time

from bus import make_bus

bus = make_bus(client_id="reset")
time.sleep(0.5)  # let the connection come up
bus.publish("forge/control/reset", {"ts": time.time()})
time.sleep(0.5)  # let the message go out before exiting
print("reset sent")
