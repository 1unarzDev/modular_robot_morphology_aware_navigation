"""Export Figure 1 geometry from an accepted workshop record set.

Usage: python3 make_figure_data.py <results/workshop_diagnostic> [out.json]

Routes and transition sites come from the immutable terminal records, so the
overview figure always depicts the plans that were actually executed and cannot
drift from the evidence figure. Scenario geometry and the module sweeps are
recomputed from the same generator and catalog the planner used; no simulator
ground truth is read.

Terminal records store route waypoints as poses without a morphology label. In
this corridor the robot is compact until it transforms and narrow until it
transforms back, so morphology is recovered from each waypoint's position
relative to the recorded transition sites.
"""
import glob
import json
import sys
from pathlib import Path

sys.path[:0] = [str(Path(__file__).resolve().parents[2] / "src" / p)
                for p in ("morphology_planner", "modular_robot_benchmarks")]

from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from morphology_planner import load_catalog
from morphology_planner.grid import OCCUPIED
from morphology_planner.transition_policy import (
    build_transition_trajectories, build_verification_trajectories,
)
from morphology_planner.transition_validation import (
    Pose3, TransitionTrajectoryValidator,
)

ROOT = Path(__file__).resolve().parents[2]
study = Path(sys.argv[1])
out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).resolve().parent / "figure_data.json"

catalog = load_catalog(
    ROOT / "src/modular_robot_description/config/morphologies.yaml"
).supported_experiment_subset()
validator = TransitionTrajectoryValidator()
c2n = next(t for t in catalog.transitions if t.id == "compact_to_narrow")

# The site geometry-coupled search prefers, and which the post intercepts.
ATTRACTIVE_SITE_X = 2.15
# The site the relocation-only model preferred, rejected once the declared
# post-transition maneuver is swept as well.
RELOCATION_ONLY_SITE_X = 1.85


def boxes(swept):
    return [{"name": b.name, "center": list(b.center), "size": list(b.size)} for b in swept]


def relocation_sweep(x, y):
    return boxes(validator.swept_boxes(
        build_transition_trajectories(catalog, c2n, Pose3(x, y, 0.0, 0.0))))


def verification_sweep(x, y):
    return boxes(validator.swept_boxes(build_verification_trajectories(
        catalog, "narrow_tandem", Pose3(x, y, 0.0, 0.0),
        catalog.post_transition_verification)))


def morphology_at(x, sites):
    """compact before the first transition and after the return, narrow between."""
    forward = next((s for s in sites if s["transition_id"] == "compact_to_narrow"), None)
    back = next((s for s in sites if s["transition_id"] == "narrow_to_compact"), None)
    if forward is None or x < forward["x"]:
        return "compact_diff"
    if back is not None and x >= back["x"]:
        return "compact_diff"
    return "narrow_tandem"


records = {}
for path in sorted(glob.glob(str(study / "raw" / "*.json"))):
    r = json.loads(Path(path).read_text())
    records.setdefault(r["spec"]["family"], {})[r["spec"]["method"]] = r

morphologies = {
    name: {
        "footprint": [list(p) for p in catalog.morphologies[name].footprint],
        "pods": {pod: list(pose)
                 for pod, pose in catalog.morphologies[name].pod_poses.items()},
        "core_size": list(catalog.module_sizes["core"]),
        "pod_size": list(catalog.module_sizes["pod_0"]),
    }
    for name in ("compact_diff", "narrow_tandem")
}

variants = {}
for variant, by_method in sorted(records.items()):
    any_record = next(iter(by_method.values()))
    scenario = make_confirmatory_scenario(variant, 0, any_record["spec"]["world_seed"])
    grid = scenario.grid
    walls = [list(grid.cell_center(cx, cy))
             for cy in range(grid.height) for cx in range(grid.width)
             if grid.value(cx, cy) >= OCCUPIED]
    site_y = scenario.parameters["attractive_site_xy"][1]
    methods = {}
    for method, record in sorted(by_method.items()):
        sites = record["planned_transition_sites"]
        seen, unique = set(), []
        for s in sites:
            key = (s["transition_id"], round(s["x"], 3))
            if key not in seen:
                seen.add(key)
                unique.append([round(s["x"], 3), round(s["y"], 3), s["transition_id"], 0.0])
        forward = next((s for s in unique if s[2] == "compact_to_narrow"), None)
        methods[method] = {
            "path": [[w["x"], w["y"], morphology_at(w["x"], sites), 0.0]
                     for w in record["planned_route"]],
            "sites": unique,
            "c2n_sweep": relocation_sweep(forward[0], forward[1]) if forward else [],
        }
    variants[variant] = {
        "resolution": grid.resolution,
        "width": grid.width,
        "height": grid.height,
        "walls": walls,
        "start": list(grid.cell_center(*scenario.start)),
        "goal": list(grid.cell_center(*scenario.goal)),
        "site": [ATTRACTIVE_SITE_X, site_y],
        "posts": [{"center": list(b.center), "size": list(b.size)}
                  for b in scenario.transition_obstacles],
        "methods": methods,
        "attractive_site_sweep": relocation_sweep(ATTRACTIVE_SITE_X, site_y),
        # Why the relocation-only model's preferred site is now refused.
        "relocation_only_site": [RELOCATION_ONLY_SITE_X, site_y],
        "relocation_only_verification_sweep": verification_sweep(
            RELOCATION_ONLY_SITE_X, site_y),
    }

out_path.write_text(
    json.dumps({"morphologies": morphologies, "variants": variants}, indent=1) + "\n",
    encoding="utf-8")
for variant, value in variants.items():
    for method, entry in value["methods"].items():
        print(f"{variant:20s} {method:24s} sites={[s[:3] for s in entry['sites']]}")
print("wrote", out_path)
