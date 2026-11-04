"""Figure: confirmatory completion by method and the two primary contrasts.

Reads only `analysis.json` from `morphology_study analyze`, so the figure is a
deterministic derivative of the accepted terminal records. Panel (a) is each
method's completion rate with its layout-cluster 95% interval; panel (b) the
two primary contrasts with their layout-bootstrap intervals and Holm p values,
the per-family means drawn beside them as heterogeneity diagnostics (no
interval, no test), against zero and the smallest effect of interest. Every
row is labelled by name, so no identity rests on colour alone.

    python paper/conference/make_results_figure.py \
        <analysis-output>/analysis.json paper/conference/figure_results.svg
"""
from __future__ import annotations

import json
import sys
from html import escape
from pathlib import Path

METHODS = ("route_first_adaptation", "geometry_coupled",
           "feasibility_coupled", "sensing_feasibility_coupled")
LABELS = {"route_first_adaptation": "route first", "geometry_coupled": "geometry",
          "feasibility_coupled": "feasibility",
          "sensing_feasibility_coupled": "sensing + feasibility"}
COLORS = {"route_first_adaptation": "#64748b", "geometry_coupled": "#f59e0b",
          "feasibility_coupled": "#0ea5e9", "sensing_feasibility_coupled": "#16a34a"}
FAMILY_LABELS = {"reconfiguration_workspace": "reconf. workspace",
                 "docking_observability": "docking observ.",
                 "combined_constraints": "combined"}
SMALLEST_EFFECT = 0.18
INK, MUTED, GRID = "#172033", "#64748b", "#e2e8f0"
WIDTH, LEFT, RIGHT = 520, 170, 24
PLOT_W = WIDTH - LEFT - RIGHT
ROW = 26


def _axis(top: int, bottom: int, low: float, high: float, ticks, fmt) -> tuple[list[str], callable]:
    def x(value: float) -> float:
        return LEFT + (value - low) / (high - low) * PLOT_W

    body = []
    for tick in ticks:
        body.append(f'<line x1="{x(tick):.1f}" y1="{top}" x2="{x(tick):.1f}" y2="{bottom}" '
                    f'stroke="{GRID}" stroke-width="1"/>')
        body.append(f'<text x="{x(tick):.1f}" y="{bottom + 16}" text-anchor="middle" '
                    f'class="tick">{fmt(tick)}</text>')
    return body, x


def _interval(x, y: float, value: float, low: float, high: float, color: str,
              hollow: bool = False) -> list[str]:
    body = [f'<line x1="{x(low):.1f}" y1="{y:.1f}" x2="{x(high):.1f}" y2="{y:.1f}" '
            f'stroke="{color}" stroke-width="2" stroke-linecap="round"/>'] if low is not None else []
    fill = "white" if hollow else color
    body.append(f'<circle cx="{x(value):.1f}" cy="{y:.1f}" r="{4 if hollow else 5}" '
                f'fill="{fill}" stroke="{color}" stroke-width="2"/>')
    return body


def _p(value: float) -> str:
    return "p < 0.001" if value < 0.001 else f"p = {value:.3f}"


def render(result: dict) -> str:
    body: list[str] = []
    rows = {row["method"]: row for row in result["methods"]}

    # (a) completion by method.
    top = 34
    body.append(f'<text x="12" y="20" class="title">(a) Mission completion, '
                f'{result["trial_count"]} missions on {result["layout_count"]} layouts</text>')
    bottom = top + ROW * len(METHODS)
    axis, x = _axis(top - 8, bottom, 0.0, 1.0, (0, 0.25, 0.5, 0.75, 1.0),
                    lambda v: f"{v:.2f}".rstrip("0").rstrip(".") if v else "0")
    body += axis
    for index, method in enumerate(METHODS):
        row, y = rows[method], top + ROW * index + 8
        low, high = row["completion_rate_ci95"]
        body.append(f'<text x="{LEFT - 10}" y="{y + 4}" text-anchor="end" class="label">'
                    f'{LABELS[method]}</text>')
        body += _interval(x, y, row["completion_rate"], low, high, COLORS[method])
    body.append(f'<text x="{LEFT + PLOT_W / 2:.0f}" y="{bottom + 32}" text-anchor="middle" '
                f'class="tick">completion probability (layout-cluster 95% CI)</text>')

    # (b) primary contrasts with family diagnostics.
    top = bottom + 80
    body.append(f'<text x="12" y="{top - 14}" class="title">(b) Treatment minus control, '
                f'completion (layout-bootstrap 95% CI)</text>')
    entries = []
    for contrast in result["primary_contrasts"]:
        entries.append(("contrast", contrast))
        for family, value in contrast["family_effects"].items():
            entries.append(("family", (family, value)))
    values = [c["ci95_layout_bootstrap"][0] for c in result["primary_contrasts"]]
    values += [c["ci95_layout_bootstrap"][1] for c in result["primary_contrasts"]]
    values += [v for c in result["primary_contrasts"] for v in c["family_effects"].values()]
    low_limit = min(-0.25, min(values) - 0.05)
    high_limit = max(1.0, max(values) + 0.05)
    bottom = top + ROW * len(entries)
    ticks = [t / 4 for t in range(int(low_limit * 4) - 1, int(high_limit * 4) + 2)
             if low_limit <= t / 4 <= high_limit]
    axis, x = _axis(top - 8, bottom, low_limit, high_limit, ticks, lambda v: f"{v:+.2f}" if v else "0")
    body += axis
    body.append(f'<line x1="{x(0):.1f}" y1="{top - 8}" x2="{x(0):.1f}" y2="{bottom}" '
                f'stroke="{MUTED}" stroke-width="1.5"/>')
    body.append(f'<line x1="{x(SMALLEST_EFFECT):.1f}" y1="{top - 8}" x2="{x(SMALLEST_EFFECT):.1f}" '
                f'y2="{bottom}" stroke="{MUTED}" stroke-width="1.5" stroke-dasharray="4 3"/>')
    for index, (kind, entry) in enumerate(entries):
        y = top + ROW * index + 8
        if kind == "contrast":
            control = entry["control"]
            low, high = entry["ci95_layout_bootstrap"]
            body.append(f'<text x="{LEFT - 10}" y="{y + 4}" text-anchor="end" class="label">'
                        f'vs. {LABELS[control]}</text>')
            body += _interval(x, y, entry["estimate_treatment_minus_control"], low, high,
                              COLORS["sensing_feasibility_coupled"])
            # A white halo keeps the note legible over gridlines and intervals.
            body.append(f'<text x="{LEFT + PLOT_W:.0f}" y="{y - 8}" text-anchor="end" '
                        f'class="tick" paint-order="stroke" stroke="white" stroke-width="4">'
                        f'{entry["layout_count"]} layouts, Holm '
                        f'{_p(entry["holm_adjusted_p"])}</text>')
        else:
            family, value = entry
            body.append(f'<text x="{LEFT - 10}" y="{y + 4}" text-anchor="end" class="small">'
                        f'{FAMILY_LABELS.get(family, family)}</text>')
            body += _interval(x, y, value, None, None, MUTED, hollow=True)
    body.append(f'<text x="{LEFT + PLOT_W / 2:.0f}" y="{bottom + 32}" text-anchor="middle" '
                f'class="tick">dashed: smallest effect of interest ({SMALLEST_EFFECT}); hollow: '
                f'family mean, untested</text>')
    height = bottom + 44
    return "\n".join([
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{height}" '
        f'viewBox="0 0 {WIDTH} {height}">',
        "<title>Confirmatory completion and contrasts</title>",
        '<rect width="100%" height="100%" fill="white"/>',
        f"<style>text{{font-family:system-ui,sans-serif;fill:{INK}}}"
        f".title{{font-size:15px;font-weight:600}}.label{{font-size:14px}}"
        f".small{{font-size:13px;fill:{MUTED}}}.tick{{font-size:12px;fill:{MUTED}}}</style>",
        *body, "</svg>", ""])


def main() -> None:
    result = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    Path(sys.argv[2]).write_text(render(result), encoding="utf-8")


if __name__ == "__main__":
    main()
