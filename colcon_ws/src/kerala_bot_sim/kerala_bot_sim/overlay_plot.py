"""Overlay the commanded path and the simulated (predicted-actual) path on one XY canvas.

Two modes:
  - static  : final overlay written to PNG (commanded in one colour, simulated in another;
              their divergence at corners/high-accel segments is the visible discontinuity).
  - animate : both paths are drawn progressively as the simulation time advances, so you
              watch the actual path peel away from the commanded one. Shows live if a GUI
              backend is available, otherwise saves a GIF.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .simulate import SimResult

CMD_COLOR = "#1f77b4"     # commanded (intended)
ACT_COLOR = "#d62728"     # simulated actual (what the model predicts happens)


def _setup_ax(ax, res: SimResult):
    allx = np.concatenate([res.x_cmd, res.x_act])
    ally = np.concatenate([res.y_cmd, res.y_act])
    pad = 0.05
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("machine X (m)")
    ax.set_ylabel("machine Y (m)")
    ax.set_facecolor("white")
    ax.grid(True, alpha=0.3)
    ax.set_title("Commanded (blue) vs predicted actual (red)")


def plot_static(res: SimResult, out_png: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 7))
    _setup_ax(ax, res)
    ax.plot(res.x_cmd, res.y_cmd, "-", color=CMD_COLOR, lw=2.0, label="commanded")
    ax.plot(res.x_act, res.y_act, "-", color=ACT_COLOR, lw=1.4, label="predicted actual")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f"[overlay] wrote static overlay -> {out_png}")


def plot_animated(res: SimResult, out_gif: Optional[str] = None,
                  stride: int = 10, interval_ms: int = 20):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(figsize=(8, 7))
    _setup_ax(res=res, ax=ax)
    (cmd_line,) = ax.plot([], [], "-", color=CMD_COLOR, lw=2.0, label="commanded")
    (act_line,) = ax.plot([], [], "-", color=ACT_COLOR, lw=1.4, label="predicted actual")
    (head,) = ax.plot([], [], "o", color=ACT_COLOR, ms=6)
    ax.legend(loc="upper right")

    frames = range(1, len(res.t), stride)

    def update(i):
        cmd_line.set_data(res.x_cmd[:i], res.y_cmd[:i])
        act_line.set_data(res.x_act[:i], res.y_act[:i])
        head.set_data([res.x_act[i - 1]], [res.y_act[i - 1]])
        return cmd_line, act_line, head

    anim = FuncAnimation(fig, update, frames=frames, interval=interval_ms, blit=True)

    if out_gif:
        try:
            from matplotlib.animation import PillowWriter
            anim.save(out_gif, writer=PillowWriter(fps=1000 // interval_ms))
            print(f"[overlay] wrote animation -> {out_gif}")
        except Exception as e:  # pragma: no cover
            print(f"[overlay] could not save gif ({e}); showing instead")
            plt.show()
    else:
        plt.show()
    plt.close(fig)
