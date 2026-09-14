"""Figure 1: feasibility-coupled task-time co-design, from planner/scenario geometry only.

Input: figure_data.json exported by the planner (no simulator ground truth).
Output: figure_overview.pdf/.png (single IEEE column).
"""
import json
import sys
from math import cos, sin
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon, Rectangle

HERE = Path(__file__).resolve().parent
data = json.loads((Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "figure_data.json").read_text())
plt.rcParams.update({"font.size": 7, "font.family": "serif", "axes.linewidth": 0.6,
                     "hatch.linewidth": 0.5})
CORE, POD, POST = "#3d5a80", "#ee964b", "#f4a261"
MORPH = data["morphologies"]
XLIM, YLIM = (0.35, 4.25), (1.12, 2.42)


def rotated_rect(cx, cy, w, h, yaw):
    c, s = cos(yaw), sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((w / 2, h / 2), (-w / 2, h / 2), (-w / 2, -h / 2), (w / 2, -h / 2))]


def draw_robot(ax, name, x, y, yaw=0.0, dashed=False, alpha=1.0, z=5):
    m = MORPH[name]
    style = {"ls": (0, (2, 1)) if dashed else "-", "lw": 0.7 if dashed else 0.4, "alpha": alpha, "zorder": z}
    ax.add_patch(Polygon(rotated_rect(x, y, *m["core_size"][:2], yaw),
                         fc="none" if dashed else CORE, ec=CORE, **style))
    c, s = cos(yaw), sin(yaw)
    for px, py, pyaw in m["pods"].values():
        ax.add_patch(Polygon(rotated_rect(x + c * px - s * py, y + s * px + c * py, *m["pod_size"][:2], yaw + pyaw),
                             fc="none" if dashed else POD, ec=CORE if dashed else "#333333", **style))


def overlaps(box, post):
    (bx, by, _), (bw, bh, _) = box["center"], box["size"]
    (px, py, _), (pw, ph, _) = post["center"], post["size"]
    return abs(bx - px) < (bw + pw) / 2 and abs(by - py) < (bh + ph) / 2


def plan_panel(ax, variant, method, title, feasible):
    res = variant["resolution"]
    for x, y in variant["walls"]:
        ax.add_patch(Rectangle((x - res / 2, y - res / 2), res, res, fc="#2b2d42", ec="none", zorder=2))
    for post in variant["posts"]:
        (x, y, _), (w, h, _) = post["center"], post["size"]
        ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, fc=POST, ec="black", lw=0.8, hatch="xxxx", zorder=7))
    entry = variant["methods"][method]
    path = entry["path"]
    for i in range(len(path) - 1):
        (x0, y0, m0, _), (x1, y1) = path[i], path[i + 1][:2]
        compact = m0 == "compact_diff"
        ax.plot([x0, x1], [y0, y1], color="black", lw=0.9 if compact else 1.5,
                ls="-" if compact else (0, (3, 1.2)), zorder=6, solid_capstyle="butt")
    posts = variant["posts"]
    # Pod parts whose relocation sweep intersects the post are the physical reason for rejection.
    colliding_pods = {box["name"].split("/")[0] for box in entry.get("c2n_sweep", [])
                      if any(overlaps(box, post) for post in posts)}
    for box in entry.get("c2n_sweep", []):
        (x, y, _), (w, h, _) = box["center"], box["size"]
        hit = box["name"].split("/")[0] in colliding_pods
        ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, fc="#c1121f" if hit else "#6c757d",
                               ec="none", alpha=0.22 if hit else 0.06, zorder=3))
    site_x, site_y, _, site_yaw = next(s for s in entry["sites"] if s[2] == "compact_to_narrow")
    draw_robot(ax, "compact_diff", site_x, site_y, site_yaw, alpha=0.95)
    draw_robot(ax, "narrow_tandem", site_x, site_y, site_yaw, dashed=True, z=6)
    ax.plot(*variant["start"], marker="^", color="black", ms=4.5, zorder=8)
    ax.text(variant["start"][0], variant["start"][1] + 0.1, "start", ha="center", fontsize=6)
    ax.annotate("", (4.22, variant["goal"][1]), (4.0, variant["goal"][1]),
                arrowprops={"arrowstyle": "-|>", "lw": 0.8})
    ax.text(4.2, variant["goal"][1] + 0.1, "to goal", ha="right", fontsize=5.8)
    ax.text(3.26, 2.38, "doorway", fontsize=5.8, va="top")
    label = (r"$\checkmark$ executable here" if feasible else
             r"$\times$ fits, but cannot transform here")
    ax.text(site_x, 1.18, label, ha="center", va="bottom", fontsize=6.2, fontweight="bold",
            color="#1b4332" if feasible else "#9d0208",
            bbox={"boxstyle": "round,pad=0.15", "fc": "white", "ec": "#1b4332" if feasible else "#9d0208", "lw": 0.6},
            zorder=9)
    if colliding_pods:
        post = posts[0]
        ax.annotate(f"{', '.join(sorted(colliding_pods)).replace('_', ' ')} sweep hits post",
                    (post["center"][0] - 0.07, post["center"][1] + 0.02), xytext=(1.05, 2.3),
                    fontsize=5.8, color="#9d0208", va="center", ha="left",
                    arrowprops={"arrowstyle": "-", "lw": 0.5, "color": "#9d0208"}, zorder=9)
    ax.set_title(title, fontsize=6.8, loc="left", pad=1.5)
    ax.set_xlim(*XLIM); ax.set_ylim(*YLIM); ax.set_aspect("equal")
    ax.tick_params(length=2, pad=1, labelsize=6)
    ax.set_yticks([1.5, 2.0])


fig = plt.figure(figsize=(3.5, 3.0))
outer = fig.add_gridspec(3, 2, height_ratios=[0.52, 1.0, 1.0], hspace=0.28, wspace=0.05)
for col, name in enumerate(("compact_diff", "narrow_tandem")):
    ax = fig.add_subplot(outer[0, col])
    draw_robot(ax, name, 0.0, 0.0)
    ax.add_patch(Polygon(MORPH[name]["footprint"], closed=True, fill=False, ls=":", lw=0.6, ec="#777777"))
    for pod, (px, py, _) in MORPH[name]["pods"].items():
        ax.text(px, py, pod[-1], ha="center", va="center", fontsize=4.8, zorder=9)
    ax.set_xlim(-0.95, 0.95); ax.set_ylim(-0.42, 0.42); ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title(("(a) " if col == 0 else "") + name, fontsize=6.8, pad=1.5, family="monospace")

blocked = data["variants"]["workshop_blocked_a"]
ax_b = fig.add_subplot(outer[1, :])
plan_panel(ax_b, blocked, "geometry_coupled",
           "(b) Blocked-A, geometry-coupled: site x = 2.15 m", feasible=False)
ax_b.set_xticklabels([])
ax_c = fig.add_subplot(outer[2, :])
plan_panel(ax_c, blocked, "feasibility_coupled",
           "(c) Blocked-A, feasibility-coupled: site x = 1.85 m", feasible=True)
ax_c.set_xlabel("x [m]", labelpad=1)
for ax in (ax_b, ax_c):
    ax.set_ylabel("y [m]", labelpad=1)
handles = [
    Line2D([], [], color="black", lw=0.9, label="compact route"),
    Line2D([], [], color="black", lw=1.5, ls=(0, (3, 1.2)), label="narrow route"),
    Rectangle((0, 0), 1, 1, fc="none", ec=CORE, ls=(0, (2, 1)), lw=0.7, label="target narrow pose"),
    Rectangle((0, 0), 1, 1, fc="#c1121f", alpha=0.35, ec="none", label="colliding pod sweep"),
    Rectangle((0, 0), 1, 1, fc="#6c757d", alpha=0.25, ec="none", label="other pod sweeps"),
    Rectangle((0, 0), 1, 1, fc=POST, ec="black", hatch="xxxx", lw=0.6, label="post, z 0-0.12 m"),
]
fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=5.5, frameon=False,
           bbox_to_anchor=(0.52, -0.045), handlelength=1.6, columnspacing=0.8)
fig.savefig(HERE / "figure_overview.pdf", bbox_inches="tight", pad_inches=0.02)
fig.savefig(HERE / "figure_overview.png", dpi=240, bbox_inches="tight", pad_inches=0.02)
print("wrote", HERE / "figure_overview.pdf")
