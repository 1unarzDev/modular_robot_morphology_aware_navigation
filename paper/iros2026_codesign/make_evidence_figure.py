"""Figure 2: per-mission evidence from immutable workshop records (n = 1 per cell).

Input: results_summary.json from summarize_results.py.
Output: figure_evidence.pdf/.png. Deliberately shows individual missions, not rates.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = Path(__file__).resolve().parent
summary = json.loads((Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "results_summary.json").read_text())
plt.rcParams.update({"font.size": 7, "font.family": "serif", "axes.linewidth": 0.6})
VARIANTS = [("workshop_blocked_a", "Blocked-A"), ("workshop_blocked_b", "Blocked-B"), ("workshop_neutral", "Neutral")]
METHODS = [("route_first_adaptation", "Route-first", "s", -0.22),
           ("geometry_coupled", "Geometry", "D", 0.0),
           ("feasibility_coupled", "Feasibility", "o", 0.22)]
BLOCKED_SITE_X = 2.15

fig, ax = plt.subplots(figsize=(3.5, 1.72))
for row, (variant, label) in enumerate(VARIANTS):
    y0 = len(VARIANTS) - 1 - row
    if variant != "workshop_neutral":
        ax.axvspan(BLOCKED_SITE_X - 0.05, BLOCKED_SITE_X + 0.05, ymin=(y0 - 0.45 + 0.6) / 3.2,
                   ymax=(y0 + 0.45 + 0.6) / 3.2, color="#f4a261", alpha=0.35, lw=0, hatch="////")
    for method, mlabel, marker, dy in METHODS:
        cell = summary["cells"].get(f"{variant}/{method}")
        if not cell or cell["n"] == 0:
            ax.text(1.62, y0 + dy, "pending", fontsize=5, va="center")
            continue
        sites = cell["c2n_site_x_m"]
        if not sites:
            ax.plot(1.62, y0 + dy, marker=marker, mfc="white", mec="black", ms=5)
            ax.text(1.67, y0 + dy, "no transition planned", fontsize=5.3, va="center")
            continue
        success = cell["transition_successes"] > 0 and cell["transition_successes"] == cell["transition_attempts"]
        x = sites[0]
        ax.plot(x, y0 + dy, marker=marker, ms=5.5, mec="black", mew=0.7,
                mfc="#2d6a4f" if success else "white", zorder=5)
        if not success:
            ax.plot(x, y0 + dy, marker="x", ms=4.2, color="#9d0208", mew=1.0, zorder=6)
        if cell["completed"]:
            detail = f"completed, {cell['mean_completed_time_s']:.0f} s"
        elif cell.get("transformation_contact"):
            detail = "contact during transform"
        elif cell.get("post_transition_check_contact"):
            detail = "transformed; post-check contact"
        else:
            detail = cell["terminal_statuses"][0].replace("_", " ")
        ax.text(2.3, y0 + dy, f"{mlabel}: {detail}", fontsize=5.4, va="center")
ax.set_yticks([2, 1, 0]); ax.set_yticklabels([label for _, label in VARIANTS])
ax.set_ylim(-0.6, 2.6); ax.set_xlim(1.65, 3.3)
ax.set_xticks([1.85, 2.15])
ax.set_xlabel("planned compact-to-narrow site x [m]", labelpad=1)
ax.tick_params(length=2, pad=1, labelsize=6)
ax.spines[["top", "right"]].set_visible(False)
handles = [Line2D([], [], marker=m, ls="", mec="black", mfc="#bbbbbb", label=l) for _, l, m, _ in METHODS]
handles += [Line2D([], [], marker="o", ls="", mec="black", mfc="#2d6a4f", label="transition executed"),
            Line2D([], [], marker="x", ls="", color="#9d0208", label="transition failed")]
ax.legend(handles=handles, loc="upper center", ncol=5, fontsize=5, frameon=False,
          bbox_to_anchor=(0.45, 1.2), handletextpad=0.2, columnspacing=0.7)
fig.savefig(HERE / "figure_evidence.pdf", bbox_inches="tight", pad_inches=0.02)
fig.savefig(HERE / "figure_evidence.png", dpi=240, bbox_inches="tight", pad_inches=0.02)
print("wrote", HERE / "figure_evidence.pdf")
