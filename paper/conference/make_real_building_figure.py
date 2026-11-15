"""Figure: one mission in a scanned real building, and why the sites differ.

Illustrative, not evidence. The example is chosen by a rule fixed before the
real-buildings sample was evaluated (2026-11-15): the first building in
sample order whose first mission has a `geometry_coupled` site the full test
rejects and a site the treatment plans. The map is the mission's planning
window as the planner sees it: grid-mapped cells, raised geometry the scan
plane misses, the station, and each method's site drawn as the narrow body's
footprint, with the station's sight lines to the docked pods at the planned
pose (solid: clear, dashed: blocked). Sites come from the committed report;
the scan is re-read only to draw the map and test the sight lines, so the
script needs HM3D locally (Matterport academic licence; see
studies/real_buildings/DATA_TERMS.md).

    python paper/conference/make_real_building_figure.py \
        studies/real_buildings/real_buildings_20261115.json \
        studies/real_buildings/sample.txt C:/Users/risha/datasets/hm3d \
        paper/conference/figure_real_building.svg
"""
from __future__ import annotations

import json
import sys
from html import escape
from math import cos, pi, sin
from pathlib import Path

from modular_robot_benchmarks.building_mesh import (
    BOX_BAND_M, VOXEL_Z, build_levels, crop_level, read_glb_triangles,
)
from modular_robot_benchmarks.real_buildings import DEFAULT_CATALOG, HEADING_BINS, STATION_HEIGHT_M
from morphology_planner import load_catalog
from morphology_planner.observability import FiducialStation, station_observes
from morphology_planner.transition_policy import Pose3, _world_pose

METHODS = ("geometry_coupled", "feasibility_coupled", "sensing_feasibility_coupled")
LABELS = ("geometry", "feasibility", "sensing + feasibility")
COLORS = ("#f59e0b", "#0ea5e9", "#16a34a")
SCALE = 60.0          # px per metre
RAISED_TOP_M = 0.30   # the tallest body: raised geometry above this never meets it


def choose(report: dict):
    for building in report["results"]:
        if not building["missions"]:
            continue
        mission = building["missions"][0]
        methods = mission["result"]["methods"]
        geometry = methods["geometry_coupled"]
        treatment = methods.get("sensing_feasibility_coupled", {})
        if (geometry.get("planned") and treatment.get("planned") and treatment["sites"]
                and any(not s["full_test_feasible"] for s in geometry["sites"])):
            return building, mission
    raise SystemExit("no building meets the selection rule")


def main() -> None:
    report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    sample = [line.strip() for line in Path(sys.argv[2]).read_text(encoding="utf-8").splitlines()
              if line.strip()]
    root, destination = Path(sys.argv[3]), Path(sys.argv[4])
    building, mission = choose(report)
    path = root / next(p for p in sample if Path(p).stem == building["building"])
    level = {l.index: l for l in build_levels(read_glb_triangles(path))}[mission["level"]]
    grid, sight_boxes, unmapped = crop_level(level, *mission["window"])
    catalog = load_catalog(str(DEFAULT_CATALOG)).supported_experiment_subset()
    transition = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    narrow = catalog.morphologies["narrow_tandem"]
    res = grid.resolution
    station = FiducialStation("station", (*grid.cell_center(*mission["station_cell"]),
                                          STATION_HEIGHT_M))

    ox, oy = 12.0, 34.0
    height_px = grid.height * res * SCALE

    def px(x: float, y: float) -> tuple[float, float]:
        return ox + x * SCALE, oy + height_px - y * SCALE

    body = []
    x0, y0, x1, y1 = mission["window"]
    occupancy = level.occupancy[y0:y1, x0:x1]
    low = int((0.14 - BOX_BAND_M[0]) / VOXEL_Z)
    high = int((RAISED_TOP_M - BOX_BAND_M[0]) / VOXEL_Z)
    raised = level.voxels[y0:y1, x0:x1, low:high].any(axis=2) & ~occupancy
    for y in range(grid.height):
        for x in range(grid.width):
            colour = "#334155" if occupancy[y, x] else "#93c5fd" if raised[y, x] else None
            if colour:
                left, top = px(x * res, (y + 1) * res)
                body.append(f'<rect x="{left:.1f}" y="{top:.1f}" width="{res*SCALE:.2f}" '
                            f'height="{res*SCALE:.2f}" fill="{colour}" shape-rendering="crispEdges"/>')
    sx, sy = px(*station.position[:2])
    sites = {}
    for method in METHODS:
        entry = mission["result"]["methods"].get(method, {})
        if entry.get("planned") and entry["sites"]:
            sites[method] = entry["sites"][0]
    clear_counts = {}
    for offset, method in enumerate(METHODS):
        if method not in sites:
            continue
        colour = COLORS[offset]
        x, y, heading = sites[method]["site"]
        wx, wy = grid.cell_center(x, y)
        yaw = heading * 2 * pi / HEADING_BINS
        clear = 0
        for pod in transition.moved_pods:
            docked = _world_pose(narrow.pod_poses[pod], Pose3(wx, wy, 0.0, yaw),
                                 catalog.module_sizes[pod][2])
            point = (docked.x, docked.y, docked.z)
            seen = station_observes(station, point, sight_boxes)
            clear += seen
            ex, ey = px(docked.x, docked.y)
            dash = "" if seen else ' stroke-dasharray="4 3"'
            body.append(f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" '
                        f'stroke="{colour}" stroke-width="1.4" stroke-opacity="0.8"{dash}/>')
        clear_counts[method] = clear
        c, s = cos(yaw), sin(yaw)
        corners = " ".join("{:.1f},{:.1f}".format(*px(wx + c * fx - s * fy, wy + s * fx + c * fy))
                           for fx, fy in narrow.footprint)
        # Earlier methods get wider outlines, so a site two methods share
        # shows both colours.
        body.append(f'<polygon points="{corners}" fill="{colour}" fill-opacity="0.18" '
                    f'stroke="{colour}" stroke-width="{2.5 + 3 * (len(METHODS) - 1 - offset)}"/>')
    body.append(f'<path d="M{sx-8:.1f},{sy+7:.1f} L{sx+8:.1f},{sy+7:.1f} L{sx:.1f},{sy-8:.1f} Z" '
                f'fill="#7c3aed"/>')
    for label, cell in (("S", mission["start"]), ("G", mission["goal"])):
        x, y = px(*grid.cell_center(*cell[:2]))
        body.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="9" fill="white" stroke="#172033"/>'
                    f'<text x="{x:.1f}" y="{y+5:.1f}" text-anchor="middle" class="mark">{label}</text>')
    width = ox * 2 + grid.width * res * SCALE
    # Scale bar, 1 m.
    bx, by = ox, oy + height_px + 16
    body.append(f'<line x1="{bx}" y1="{by}" x2="{bx+SCALE}" y2="{by}" stroke="#172033" stroke-width="3"/>'
                f'<text x="{bx+SCALE+6}" y="{by+5}" class="small">1 m</text>')
    legend = [(COLORS[i], f"{LABELS[i]} site ({clear_counts[m]}/{len(transition.moved_pods)} pods seen)")
              for i, m in enumerate(METHODS) if m in clear_counts]
    legend += [("#334155", "mapped by the 0.12 m scan plane"), ("#93c5fd", "raised, unseen by it")]
    top = by + 18
    for item, (colour, label) in enumerate(legend):
        body.append(f'<rect x="{ox}" y="{top + item*22}" width="16" height="16" fill="{colour}"/>'
                    f'<text x="{ox+24}" y="{top + item*22 + 13}" class="small">{escape(label)}</text>')
    height = top + len(legend) * 22 + 4
    body.insert(0, f'<text x="{ox}" y="22" class="label">HM3D {escape(building["building"])}, '
                   f'floor {mission["level"]}: compact &#8594; narrow</text>')
    svg = "\n".join([
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
        f'viewBox="0 0 {width:.0f} {height:.0f}">',
        "<title>A mission in a scanned building</title>",
        '<rect width="100%" height="100%" fill="white"/>',
        "<style>text{font-family:system-ui,sans-serif;fill:#172033}"
        ".label{font-size:17px}.small{font-size:15px}.mark{font-size:13px;font-weight:700}</style>",
        *body, "</svg>", ""])
    destination.write_text(svg, encoding="utf-8")
    print(json.dumps({"building": building["building"], "level": mission["level"],
                      "sites": {m: s["site"] for m, s in sites.items()},
                      "pods_seen": clear_counts}))


if __name__ == "__main__":
    main()
