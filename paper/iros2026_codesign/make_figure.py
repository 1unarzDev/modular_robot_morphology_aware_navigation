"""Build the workshop overview figure from planner/scenario-only geometry.

Input: figure_data.json exported by the planner (no simulator ground truth).
Output: figure_overview.pdf (single IEEE column).
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

HERE = Path(__file__).resolve().parent
data = json.loads((Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "figure_data.json").read_text())
METHOD_STYLE = {
    "route_first_adaptation": ("#7b8794", "Route-first", "s"),
    "geometry_coupled": ("#d1495b", "Geometry-coupled", "X"),
    "feasibility_coupled": ("#00798c", "Feasibility-coupled", "o"),
}
plt.rcParams.update({"font.size": 7, "font.family": "serif", "axes.linewidth": 0.6})
fig = plt.figure(figsize=(3.5, 2.2))
grid = fig.add_gridspec(2, 2, height_ratios=[0.42, 1.0], hspace=0.02, wspace=0.08)


def morphology_panel(ax, name, title):
    m = data["morphologies"][name]
    ax.add_patch(Polygon(m["footprint"], closed=True, fill=False, ls="--", lw=0.6, ec="#999999"))
    cx, cy = m["core_size"][0] / 2, m["core_size"][1] / 2
    ax.add_patch(Rectangle((-cx, -cy), 2 * cx, 2 * cy, fc="#3d5a80", ec="none"))
    px, py = m["pod_size"][0] / 2, m["pod_size"][1] / 2
    for pod, (x, y, _) in m["pods"].items():
        ax.add_patch(Rectangle((x - px, y - py), 2 * px, 2 * py, fc="#ee964b", ec="#333333", lw=0.3))
        ax.text(x, y, pod[-1], ha="center", va="center", fontsize=5)
    ax.set_xlim(-0.95, 0.95); ax.set_ylim(-0.45, 0.45); ax.set_aspect("equal")
    ax.set_title(title, fontsize=7, pad=2); ax.set_xticks([]); ax.set_yticks([])


ax_a = fig.add_subplot(grid[0, 0])
morphology_panel(ax_a, "compact_diff", "(a) compact_diff")
ax_a2 = fig.add_subplot(grid[0, 1])
morphology_panel(ax_a2, "narrow_tandem", "(a) narrow_tandem")

ax = fig.add_subplot(grid[1, :])
variant = data["variants"]["workshop_blocked_a"]
res = variant["resolution"]
for x, y in variant["walls"]:
    ax.add_patch(Rectangle((x - res / 2, y - res / 2), res, res, fc="#2b2d42", ec="none"))
sx, sy = variant["site"]
for box in variant["attractive_site_sweep"]:
    (x, y, _), (w, h, _) = box["center"], box["size"]
    moving = box["name"].split("/")[0]
    color = "#d1495b" if moving == "pod_4" else "#bbbbbb"
    ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, fc=color, ec="none", alpha=0.10 if moving != "pod_4" else 0.18))
for post in variant["posts"]:
    (x, y, _), (w, h, _) = post["center"], post["size"]
    ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, fc="#f4a261", ec="black", lw=0.6, zorder=5))
    ax.annotate("post, z 0-0.12 m", (x + w / 2, y), xytext=(x + 1.0, y + 0.05), fontsize=6,
                va="center", arrowprops={"arrowstyle": "-", "lw": 0.5})
ax.plot(*variant["start"], marker="^", color="black", ms=4); ax.text(variant["start"][0], variant["start"][1] + 0.1, "start", ha="center", fontsize=6)
ax.plot(*variant["goal"], marker="*", color="black", ms=6); ax.text(variant["goal"][0], variant["goal"][1] + 0.1, "goal", ha="center", fontsize=6)
for method, (color, label, marker) in METHOD_STYLE.items():
    entry = variant["methods"].get(method, {})
    if "path" not in entry:
        continue
    xs = [p[0] for p in entry["path"]]; ys = [p[1] for p in entry["path"]]
    offset = {"route_first_adaptation": -0.04, "geometry_coupled": 0.0, "feasibility_coupled": 0.04}[method]
    ax.plot(xs, [y + offset for y in ys], color=color, lw=0.9, alpha=0.9, label=label)
    for x, y, tid, _ in entry["sites"]:
        if tid == "compact_to_narrow":
            ax.plot(x, y + offset, marker=marker, color=color, ms=4.5, mec="black", mew=0.4, zorder=6)
ax.text(0.45, 2.47, "red: pod-4 sweep at geometry site (X); gray: other pods",
        ha="left", va="top", fontsize=5.5, color="#9d0208")
ax.set_xlim(0.35, 5.45); ax.set_ylim(1.05, 2.55)
ax.set_aspect("equal"); ax.set_xlabel("x [m]", labelpad=1); ax.set_ylabel("y [m]", labelpad=1)
ax.tick_params(length=2, pad=1)
ax.set_title("(b) Blocked-A: post, relocation sweep, and planned compact-to-narrow sites", fontsize=6.5, pad=2)
ax.legend(loc="lower center", bbox_to_anchor=(0.6, -0.01), ncol=3, fontsize=5.5,
          frameon=False, handlelength=1.4, columnspacing=0.9)
fig.savefig(HERE / "figure_overview.pdf", bbox_inches="tight", pad_inches=0.01)
fig.savefig(HERE / "figure_overview.png", dpi=220, bbox_inches="tight", pad_inches=0.01)
print("wrote", HERE / "figure_overview.pdf")
