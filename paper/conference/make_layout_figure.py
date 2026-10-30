"""Figure: sampled generator-2 layouts and the transition site each method chose.

Reads only the committed planner manipulation check and regenerates each
layout's geometry from its frozen seed, so the figure carries no outcome data.

    python paper/conference/make_layout_figure.py \
        studies/pilot/manipulation_check.json paper/conference/figure_layouts.svg
"""
from __future__ import annotations

import json
import sys
from html import escape
from pathlib import Path

from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from morphology_planner.grid import OCCUPIED

METHODS = ("route_first_adaptation", "geometry_coupled",
           "feasibility_coupled", "sensing_feasibility_coupled")
LABELS = ("route first", "geometry", "feasibility", "sensing + feasibility")
COLORS = ("#64748b", "#f59e0b", "#0ea5e9", "#16a34a")
# Marker radii shrink with the ladder so coincident sites stay visible.
RADII = (9.0, 7.5, 5.5, 3.5)
FAMILIES = ("reconfiguration_workspace", "docking_observability", "combined_constraints")
SCALE = 36.0          # px per metre
PANEL_W, PANEL_H = 6.8 * SCALE, 4.2 * SCALE


def _panel(layout: dict, generator: int, ox: float, oy: float) -> list[str]:
    scenario = make_confirmatory_scenario(
        layout["family"], layout["layout_index"], layout["world_seed"], generator=generator)
    grid = scenario.grid
    res = grid.resolution
    height_px = grid.height * res * SCALE

    def px(x: float, y: float) -> tuple[float, float]:
        return ox + x * SCALE, oy + height_px - y * SCALE

    body = []
    for y in range(grid.height):
        for x in range(grid.width):
            if grid.value(x, y) == OCCUPIED:
                left, top = px(x * res, (y + 1) * res)
                body.append(f'<rect x="{left:.1f}" y="{top:.1f}" width="{res*SCALE:.1f}" '
                            f'height="{res*SCALE:.1f}" fill="#334155"/>')
    for box in scenario.transition_obstacles:
        (cx, cy, cz), (sx, sy, sz) = box.center, box.size
        left, top = px(cx - sx / 2, cy + sy / 2)
        elevated = cz - sz / 2 > 0.3      # the ADR 0006 screen hangs above the robot
        style = ('fill="none" stroke="#7c3aed" stroke-dasharray="4 3" stroke-width="1.5"'
                 if elevated else 'fill="#d97706" opacity="0.8"')
        body.append(f'<rect x="{left:.1f}" y="{top:.1f}" width="{sx*SCALE:.1f}" '
                    f'height="{sy*SCALE:.1f}" {style}/>')
    for station in scenario.fiducial_stations:
        sx_, sy_ = px(station.position[0], station.position[1])
        body.append(f'<path d="M{sx_-6:.1f},{sy_+5:.1f} L{sx_+6:.1f},{sy_+5:.1f} '
                    f'L{sx_:.1f},{sy_-6:.1f} Z" fill="#7c3aed"/>')
    for (cell, label) in ((scenario.start, "S"), (scenario.goal, "G")):
        x, y = px(*grid.cell_center(*cell))
        body.append(f'<text x="{x:.1f}" y="{y+4:.1f}" text-anchor="middle" '
                    f'class="mark">{label}</text>')
    for method, color, radius in zip(METHODS, COLORS, RADII):
        site = layout["decisions"][method].get("transition_site")
        if not site:
            continue
        x, y = px(*grid.cell_center(*site))
        body.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{radius}" fill="none" '
                    f'stroke="{color}" stroke-width="2.2"/>')
    kind = "neutral control" if scenario.neutral_control else "manipulated"
    body.append(f'<text x="{ox:.1f}" y="{oy-6:.1f}" class="label">'
                f'{escape(layout["family"].replace("_", " "))} {layout["layout_index"]} '
                f'({kind})</text>')
    return body


def main() -> None:
    report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    generator = int(report.get("layout_generator", 1))
    chosen = []
    for family in FAMILIES:
        layouts = [entry for entry in report["layouts"] if entry["family"] == family]
        neutral = next(entry for entry in layouts if entry["layout_index"] % 4 == 0)
        # Illustrative: the first manipulated layout where every coupled method
        # planned. Layouts where one could not are counted in the separation
        # table, which the caption must point to.
        manipulated = next(entry for entry in layouts if entry["layout_index"] % 4 != 0
                           and all(entry["decisions"][method]["planned"]
                                   for method in METHODS[1:]))
        chosen.append((neutral, manipulated))
    width = 3 * (PANEL_W + 24) + 24
    height = 2 * (PANEL_H + 34) + 104
    body = []
    for column, pair in enumerate(chosen):
        for row, layout in enumerate(pair):
            body.extend(_panel(layout, generator, 24 + column * (PANEL_W + 24),
                               40 + row * (PANEL_H + 34)))
    legend_y = height - 44
    for index, (label, color, radius) in enumerate(zip(LABELS, COLORS, RADII)):
        x = 30 + index * 190
        body.append(f'<circle cx="{x}" cy="{legend_y}" r="{radius}" fill="none" '
                    f'stroke="{color}" stroke-width="2.2"/>')
        body.append(f'<text x="{x+14}" y="{legend_y+4}" class="label">{label} site</text>')
    shown = [layout for pair in chosen for layout in pair]
    route_first = sum(layout["decisions"]["route_first_adaptation"]["planned"]
                      for layout in shown)
    body.append(f'<text x="30" y="{legend_y+26}" class="label">'
                'S start, G goal; filled bar: raised shelf; dashed: elevated sight '
                f'screen; triangle: fiducial station. Route first planned in '
                f'{route_first} of these {len(shown)} layouts.</text>')
    svg = "\n".join([
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
        f'viewBox="0 0 {width:.0f} {height:.0f}">',
        "<title>Sampled layouts and chosen transition sites</title>",
        '<rect width="100%" height="100%" fill="white"/>',
        "<style>text{font-family:system-ui,sans-serif;fill:#172033}"
        ".label{font-size:12px}.mark{font-size:13px;font-weight:700}</style>",
        *body, "</svg>", ""])
    Path(sys.argv[2]).write_text(svg, encoding="utf-8")


if __name__ == "__main__":
    main()
