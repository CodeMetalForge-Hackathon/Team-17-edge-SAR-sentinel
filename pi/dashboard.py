"""Live map for the table demo: LiDAR scan, nodes, events, hazard zones, drone dispatches.

    python dashboard.py                    # live, from the MQTT broker (run on the Pi or a laptop on the same network;
                                           #   set MQTT_HOST in config.py to the Pi's IP on the laptop)
    python dashboard.py --sim-snapshot     # run the simulated scenario and save dashboard_sim.png

Press R in the dashboard window to reset the brain and the map between judges.
"""
import math
import sys
import time

import numpy as np

import config

COLORS = {"SURVIVOR": "#2ca02c", "FIRE": "#d62728", "FLOOD": "#1f77b4", "STRUCTURAL": "#ff7f0e",
          "OBSTRUCTION": "#7f7f7f", "UNKNOWN": "#9467bd"}


class Dashboard:
    def __init__(self, bus, nodes=None):
        self.scan = None
        self.nodes = nodes or {k: (v["x"], v["y"]) for k, v in config.NODES.items()}
        self.events, self.avoid, self.dispatches, self.log = [], [], [], []
        bus.subscribe("forge/lidar/scan", lambda t, p: setattr(self, "scan", p["ranges_m"]))
        bus.subscribe("forge/event", lambda t, p: self.events.append(p))
        bus.subscribe("forge/action", self.on_action)
        bus.subscribe("forge/control/reset", lambda t, p: self.clear())

    def clear(self):
        self.events, self.avoid, self.dispatches, self.log = [], [], [], ["(reset)"]

    def on_action(self, topic, a):
        self.avoid = a.get("avoid_zones", self.avoid)
        if a["action"] == "DISPATCH_DRONE":
            self.dispatches.append((a["x"], a["y"]))
        self.log.append(f"#{a['event_id']} {a['action']}: {a['reason']}")

    def draw(self, ax_map, ax_log):
        from matplotlib.patches import Circle
        ax_map.clear()
        ax_map.set_aspect("equal")
        ax_map.set_title("Operation Lost and Found // edge map (no GPS)")
        ax_map.set_xlabel("x (m, LiDAR forward)")
        ax_map.set_ylabel("y (m)")
        ax_map.grid(alpha=0.2)
        if self.scan:
            pts = [(r * math.cos(math.radians(b)), r * math.sin(math.radians(b)))
                   for b, r in enumerate(self.scan) if r is not None]
            if pts:
                xs, ys = zip(*pts)
                ax_map.scatter(xs, ys, s=3, c="#444", label="LiDAR")
        ax_map.scatter([0], [0], marker="^", s=120, c="k")
        ax_map.annotate("Pi + LiDAR + cam", (0, 0), textcoords="offset points", xytext=(6, -12), fontsize=8)
        for z in self.avoid:
            ax_map.add_patch(Circle((z["x"], z["y"]), z["r"], color=COLORS.get(z["kind"], "r"), alpha=0.18))
        for nid, (x, y) in self.nodes.items():
            ax_map.scatter([x], [y], marker="s", s=90, facecolors="none", edgecolors="k")
            ax_map.annotate(nid, (x, y), textcoords="offset points", xytext=(6, 6), fontsize=8)
        pad = config.DRONE_PAD
        ax_map.scatter([pad["x"]], [pad["y"]], marker="X", s=110, c="#17becf")
        ax_map.annotate("drone pad", (pad["x"], pad["y"]), textcoords="offset points", xytext=(6, -12), fontsize=8)
        for x, y in self.dispatches[-3:]:
            ax_map.annotate("", (x, y), (pad["x"], pad["y"]),
                            arrowprops=dict(arrowstyle="->", color="#17becf", lw=2, ls="--"))
        seen = set()
        for e in self.events[-25:]:
            lab = e["kind"] if e["kind"] not in seen else None
            seen.add(e["kind"])
            ax_map.scatter([e["x"]], [e["y"]], s=140, c=COLORS.get(e["kind"], "k"), alpha=0.85, label=lab, zorder=5)
        ax_map.legend(loc="upper left", fontsize=7)

        ax_log.clear()
        ax_log.axis("off")
        ax_log.set_title("Responses", loc="left", fontsize=10)
        for i, line in enumerate(self.log[-14:]):
            ax_log.text(0, 1 - i * 0.07, line, fontsize=8, va="top", family="monospace", wrap=True)


def make_figure():
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(13, 6.5))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.4, 1])
    return fig, fig.add_subplot(gs[0]), fig.add_subplot(gs[1])


if __name__ == "__main__":
    if "--sim-snapshot" in sys.argv:
        import matplotlib
        matplotlib.use("Agg")
        from bus import LocalBus
        import brain
        from anomaly import NodeAnomalyModel
        from drone_dispatch import Dispatcher
        from simulate import run_scenario, synthetic_normal_windows, SIM_NODES
        config.EVENT_COOLDOWN_S = 0
        bus = LocalBus()
        b = brain.main(local_bus=bus)
        b.model = b.model or NodeAnomalyModel.train(synthetic_normal_windows(1500))
        Dispatcher(bus, dry_run=True)
        dash = Dashboard(bus, nodes=SIM_NODES)
        run_scenario(bus, step_s=0.0)
        fig, a1, a2 = make_figure()
        dash.draw(a1, a2)
        fig.tight_layout()
        fig.savefig("dashboard_sim.png", dpi=110)
        print("saved dashboard_sim.png")
    else:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from bus import make_bus
        bus = make_bus(client_id=f"dash-{int(time.time())}")
        dash = Dashboard(bus)
        fig, a1, a2 = make_figure()
        fig.suptitle("press R to reset between judges", fontsize=8, x=0.99, ha="right")
        fig.canvas.mpl_connect("key_press_event",
                               lambda e: bus.publish("forge/control/reset", {"ts": time.time()}) if e.key in ("r", "R") else None)
        anim = FuncAnimation(fig, lambda _: dash.draw(a1, a2), interval=500, cache_frame_data=False)
        plt.show()
