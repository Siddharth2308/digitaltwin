"""End-to-end demo: synthesize a trajectory -> simulate -> log -> overlay plot.

Runs with placeholder params (no bench data needed) so you can see the whole pipeline
work today. As you fill docs/params_template.yaml, pass --params to use real numbers.

    python -m kerala_bot_sim.run_demo                 # all placeholders
    python -m kerala_bot_sim.run_demo --params ../../../docs/params_template.yaml
    python -m kerala_bot_sim.run_demo --animate       # progressive drawing (GUI or GIF)
"""
from __future__ import annotations

import argparse
import os

from .params import load_params
from .trajectory import synth_square, save_csv
from .simulate import simulate, save_log
from .overlay_plot import plot_static, plot_animated

HERE = os.path.dirname(__file__)
OUT = os.path.normpath(os.path.join(HERE, "..", "output"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default=None, help="path to params yaml")
    ap.add_argument("--feed", type=float, default=0.3, help="cruise speed m/s")
    ap.add_argument("--accel", type=float, default=3.0, help="accel limit m/s^2")
    ap.add_argument("--animate", action="store_true", help="progressive animation")
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    axes = load_params(args.params, verbose=True)

    print(f"[demo] generating square trajectory (feed={args.feed} m/s, accel={args.accel} m/s^2)")
    traj = synth_square(feed=args.feed, accel=args.accel)
    save_csv(traj, os.path.join(OUT, "trajectory_cmd.csv"))

    print("[demo] simulating both axes...")
    res = simulate(traj, axes["x"], axes["y"])
    print(res.summary())

    save_log(res, os.path.join(OUT, "sim_log.csv"))
    plot_static(res, os.path.join(OUT, "overlay.png"))
    if args.animate:
        plot_animated(res, out_gif=os.path.join(OUT, "overlay.gif"))

    print(f"[demo] outputs in {OUT}")


if __name__ == "__main__":
    main()
