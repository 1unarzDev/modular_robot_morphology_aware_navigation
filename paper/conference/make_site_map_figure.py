"""Figure: where the robot can change shape, by the stage of the transition test.

Every free cell of one layout's start room is put through the treatment's full
transition test (compact to narrow) at the heading the treatment planned, and
shaded by the first stage it fails: the target body does not fit, a relocating
pod collides, the post-transition verification maneuver collides, or a moved
pod cannot be observed well enough to dock. Each coupled method's chosen site
is drawn on top, so the figure shows why the ladder's sites differ. It runs the
planner on priors only and carries no outcome data.

    python paper/conference/make_site_map_figure.py \
        studies/pilot_v3b/manipulation_check.json combined_constraints 2 \
        paper/conference/figure_site_map.svg
"""
from __future__ import annotations

import json
import sys
from html import escape
from pathlib import Path

from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from modular_robot_benchmarks.manipulation_check import DEFAULT_CATALOG, QUALIFYING_TRANSITION
from morphology_planner import HybridState, load_catalog, make_method_planner
from morphology_planner.grid import OCCUPIED
from morphology_planner.transition_policy import station_sensing_provider

METHODS = ("geometry_coupled", "feasibility_coupled", "sensing_feasibility_coupled")
LABELS = ("geometry", "feasibility", "sensing + feasibility")
COLORS = ("#f59e0b", "#0ea5e9", "#16a34a")
# First failing stage, in the order the ladder adds them.
STAGES = (
    ("fit", "no 1.0 m planar clearance", "#e2e8f0"),
    ("relocation", "pod relocation collides", "#fdba74"),
    ("verification", "verification maneuver collides", "#fca5a5"),
    ("observability", "dock not observable", "#c4b5fd"),
    ("feasible", "transition feasible", "#bbf7d0"),
)
SCALE = 120.0         # px per metre


def stage(reasons: tuple[str, ...]) -> str:
    if not reasons:
        return "feasible"
    if {"planar_clearance", "target_footprint"} & set(reasons):
        return "fit"
    sensing = {r for r in reasons if r.endswith((":connector_not_visible", ":covariance_too_large"))}
    verification = {r for r in reasons if r.endswith(":post_transition_collision")}
    if set(reasons) - sensing - verification:
        return "relocation"
    return "verification" if verification else "observability"


def _planners(scenario, catalog):
    return {method: make_method_planner(
        method, catalog, scenario.grid, heading_bins=8,
        environment=scenario.transition_obstacles,
        sensing_provider=station_sensing_provider(
            catalog, scenario.fiducial_stations, scenario.transition_obstacles,
            scenario.grid, 8))
        for method in METHODS}


def main() -> None:
    report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    family, index, destination = sys.argv[2], int(sys.argv[3]), Path(sys.argv[4])
    layout = next(entry for entry in report["layouts"]
                  if entry["family"] == family and entry["layout_index"] == index)
    generator = int(report.get("layout_generator", 1))
    scenario = make_confirmatory_scenario(family, index, layout["world_seed"], generator=generator)
    catalog = load_catalog(str(DEFAULT_CATALOG)).supported_experiment_subset()
    transition = next(t for t in catalog.transitions if t.id == QUALIFYING_TRANSITION)
    grid, res = scenario.grid, scenario.grid.resolution

    plans = {}
    for method, planner in _planners(scenario, catalog).items():
        plan = planner.plan(HybridState(*scenario.start, 0, "compact_diff"), scenario.goal,
                            float(report.get("epsilon", 2.5)))
        site = next(s.source for s in plan.segments if s.transition_id == QUALIFYING_TRANSITION)
        recorded = layout["decisions"][method]["transition_site"]
        if [site.x, site.y] != recorded:
            raise SystemExit(f"{method} site {site} differs from the committed check {recorded}")
        plans[method] = (plan, site)

    # Shade the start room (west of the interior wall) at the treatment's heading.
    heading = plans["sensing_feasibility_coupled"][1].heading
    policy = _planners(scenario, catalog)["sensing_feasibility_coupled"].transition_policy
    wall_x = min(x for x in range(grid.width) for y in range(1, grid.height - 1)
                 if x > 2 and all(grid.value(x, yy) == OCCUPIED
                                  for yy in range(1, min(4, grid.height - 1))))
    shown_w = wall_x + 2
    cells, counts = {}, {key: 0 for key, _, _ in STAGES}
    for x in range(shown_w):
        for y in range(grid.height):
            if grid.value(x, y) == OCCUPIED:
                continue
            policy(transition, HybridState(x, y, heading, "compact_diff"))
            cells[(x, y)] = stage(policy.decisions[-1].reasons)
            counts[cells[(x, y)]] += 1

    ox, oy = 20.0, 40.0
    height_px = grid.height * res * SCALE

    def px(x: float, y: float) -> tuple[float, float]:
        return ox + x * SCALE, oy + height_px - y * SCALE

    fill = {key: color for key, _, color in STAGES}
    body = []
    for (x, y), key in cells.items():
        left, top = px(x * res, (y + 1) * res)
        body.append(f'<rect x="{left:.1f}" y="{top:.1f}" width="{res*SCALE:.1f}" '
                    f'height="{res*SCALE:.1f}" fill="{fill[key]}" shape-rendering="crispEdges"/>')
    for x in range(shown_w):
        for y in range(grid.height):
            if grid.value(x, y) == OCCUPIED:
                left, top = px(x * res, (y + 1) * res)
                body.append(f'<rect x="{left:.1f}" y="{top:.1f}" width="{res*SCALE:.1f}" '
                            f'height="{res*SCALE:.1f}" fill="#334155"/>')
    for box in scenario.transition_obstacles:
        (cx, cy, cz), (sx, sy, sz) = box.center, box.size
        if cx - sx / 2 > shown_w * res:
            continue
        left, top = px(cx - sx / 2, cy + sy / 2)
        elevated = cz - sz / 2 > 0.3
        style = ('fill="none" stroke="#7c3aed" stroke-dasharray="5 3" stroke-width="2"'
                 if elevated else 'fill="#b45309" fill-opacity="0.85"')
        body.append(f'<rect x="{left:.1f}" y="{top:.1f}" width="{sx*SCALE:.1f}" '
                    f'height="{sy*SCALE:.1f}" {style}/>')
    for station in scenario.fiducial_stations:
        sx_, sy_ = px(station.position[0], station.position[1])
        body.append(f'<path d="M{sx_-8:.1f},{sy_+7:.1f} L{sx_+8:.1f},{sy_+7:.1f} '
                    f'L{sx_:.1f},{sy_-8:.1f} Z" fill="#7c3aed"/>')
    x, y = px(*grid.cell_center(*scenario.start))
    body.append(f'<text x="{x:.1f}" y="{y+5:.1f}" text-anchor="middle" class="mark">S</text>')
    for offset, (method, color) in enumerate(zip(METHODS, COLORS)):
        plan, site = plans[method]
        points = []
        for state in plan.states:
            if state.x > shown_w:
                break
            points.append("{:.1f},{:.1f}".format(*px(*grid.cell_center(state.x, state.y))))
        body.append(f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" '
                    f'stroke-width="2" stroke-opacity="0.8" transform="translate(0,{(offset-1)*2.5})"/>')
        cx_, cy_ = px(*grid.cell_center(site.x, site.y))
        body.append(f'<circle cx="{cx_:.1f}" cy="{cy_:.1f}" r="9" fill="white" '
                    f'stroke="{color}" stroke-width="3"/>')
        # Sites sit about 0.5 m apart: geometry labelled up-right, feasibility below,
        # the treatment above, so no two labels overlap.
        anchor, label_y = (("start", cy_ - 18), ("middle", cy_ + 34), ("middle", cy_ - 18))[offset]
        body.append(f'<text x="{cx_:.1f}" y="{label_y:.1f}" text-anchor="{anchor}" '
                    f'class="small" style="fill:{color};font-weight:700">'
                    f'{escape(LABELS[offset])}</text>')

    # Legend under the map, one column, so the figure keeps column width.
    map_w = shown_w * res * SCALE
    width = ox * 2 + map_w
    body.append(f'<text x="{ox}" y="26" class="label">{escape(family.replace("_", " "))} '
                f'{index}: compact &#8594; narrow, heading {heading * 45}&#176;'
                f'</text>')
    swatches = [(f'fill="{color}" stroke="#94a3b8"', f"{label} ({counts[key]})")
                for key, label, color in STAGES]
    swatches += [('fill="#b45309"', "raised shelf"),
                 ('fill="none" stroke="#7c3aed" stroke-dasharray="5 3" stroke-width="2"',
                  "elevated sight screen"),
                 (None, "fiducial station")]
    top = oy + height_px + 20
    for item, (style, label) in enumerate(swatches):
        x = ox
        y = top + item * 28
        if style is None:
            body.append(f'<path d="M{x},{y+20} L{x+20},{y+20} L{x+10},{y} Z" fill="#7c3aed"/>')
        else:
            body.append(f'<rect x="{x}" y="{y}" width="20" height="20" {style}/>')
        body.append(f'<text x="{x+28}" y="{y+16}" class="small">{label}</text>')
    height = top + len(swatches) * 28 + 6
    svg = "\n".join([
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
        f'viewBox="0 0 {width:.0f} {height:.0f}">',
        "<title>Where the robot can change shape</title>",
        '<rect width="100%" height="100%" fill="white"/>',
        "<style>text{font-family:system-ui,sans-serif;fill:#172033}"
        ".label{font-size:20px}.small{font-size:19px}.mark{font-size:20px;font-weight:700}</style>",
        *body, "</svg>", ""])
    destination.write_text(svg, encoding="utf-8")
    print(json.dumps({"heading_bin": heading, "cells": counts,
                      "site_stages": {m: cells[(s.x, s.y)] for m, (_, s) in plans.items()},
                      "sites": {m: [s.x, s.y, s.heading] for m, (_, s) in plans.items()}}))


if __name__ == "__main__":
    main()
