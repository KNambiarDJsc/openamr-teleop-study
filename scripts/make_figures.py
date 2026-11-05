"""Figures for the README, from the recorded results only. Usage: make_figures.py RESULTS_DIR FIGURES_DIR"""

import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

res, fig_dir = sys.argv[1], sys.argv[2]
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"  # categorical slots 1 and 2, validated (light)
plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
    "xtick.color": INK2, "ytick.color": INK2, "text.color": INK, "font.size": 10, "axes.spines.top": False,
    "axes.spines.right": False, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
})

# 1. Vertical reach band of the right hand by lift height
reach = json.load(open(os.path.join(res, "reach.json")))
m = reach["meta"]
shape = (len(m["xs"]), len(m["ys"]), len(m["zs"]))
table = reach["reach"]["elbow_90_tip"]
fig, ax = plt.subplots(figsize=(7, 3.6))
for i, lift in enumerate(m["lifts"]):
    R = np.array(table[str(lift)]).reshape(shape)
    zr = [m["zs"][k] for k in range(len(m["zs"])) if R[:, :, k].any()]
    ax.barh(i, max(zr) - min(zr), left=min(zr), height=0.5, color=BLUE)
    ax.text(max(zr) + 0.015, i, f"{min(zr):.2f}–{max(zr):.2f} m", va="center", color=INK2, fontsize=9)
ax.axvline(0.78, color=INK2, lw=1, ls="--")
ax.text(0.79, -0.62, "table pose, 0.78 m", color=INK2, fontsize=9)
ax.set_ylim(-0.8, len(m["lifts"]) - 0.5)
ax.set_yticks(range(len(m["lifts"])), [f"{l * 1000:.0f} mm" for l in m["lifts"]])
ax.set_ylabel("lift position")
ax.set_xlabel("right-hand tip height above the floor (m), forward-pointing hand")
ax.set_xlim(0.5, 1.85)
ax.grid(axis="y", visible=False)
ax.set_title("The lift slides the hand's vertical reach band; it does not widen it", loc="left", fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, "reach_band.png"), dpi=160)

# 2. A released hand during a full lift stroke: held relative to the lift vs fixed in the world
d = json.load(open(os.path.join(res, "lift_table.json")))
fig, ax = plt.subplots(figsize=(7, 3.8))
for run, color, name in ((d["runs"][0], BLUE, "held relative to the lift"), (d["runs"][1], ORANGE, "held fixed in the world")):
    s = np.array(run["samples"])
    ax.plot(s[:, 1] * 1000, s[:, 4], color=color, lw=2)
    ax.text(s[-1, 1] * 1000 + 6, s[-1, 4], name, color=INK, va="center", fontsize=9)
s = np.array(d["runs"][1]["samples"])
z0 = d["runs"][1]["tip_start"][2]
brk = next(r for r in s if abs(r[4] - z0) > 0.005)
ax.plot([brk[1] * 1000], [brk[4]], "o", ms=8, color=ORANGE, mec=SURFACE, mew=2)
ax.annotate(f"hold breaks at ~{brk[1] * 1000:.0f} mm: wrist joint 6\nat -0.781 of its -0.785 rad limit.\nNothing reports it; the lift\ncarries the arm from here.",
            xy=(brk[1] * 1000, brk[4]), xytext=(225, 0.79), color=INK2, fontsize=9, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
ax.set_xlabel("lift position (mm), moving at its 18 mm/s limit")
ax.set_ylabel("tip height (m)")
ax.set_xlim(0, 470)
ax.legend(handles=[plt.Line2D([], [], color=BLUE, lw=2, label="relative to the lift"),
                   plt.Line2D([], [], color=ORANGE, lw=2, label="fixed in the world")], frameon=False, loc="upper left")
ax.set_title("A released right hand during a full 350 mm lift stroke (table pose)", loc="left", fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, "lift_stroke.png"), dpi=160)
print("figures written")
