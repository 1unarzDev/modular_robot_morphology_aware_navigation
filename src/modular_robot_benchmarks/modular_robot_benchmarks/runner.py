from __future__ import annotations

import argparse
import csv
import time
from dataclasses import replace
from pathlib import Path

from morphology_planner import HybridState, MorphologyAStar, load_catalog
from morphology_planner.planner import NoPathError

from .scenarios import FAMILIES, make_scenario


def run(catalog_path: Path, layouts: int, output: Path) -> None:
    """Run a planner diagnostic; these rows are never mission trial records."""
    catalog = load_catalog(catalog_path)
    rows = []
    for family in FAMILIES:
        for seed in range(layouts):
            scenario = make_scenario(family, seed)
            methods = {"coupled_analytic": catalog}
            for morphology in catalog.morphologies:
                methods[f"fixed_{morphology}"] = replace(catalog, transitions=())
            for method, selected_catalog in methods.items():
                start_morphology = method.removeprefix("fixed_") if method.startswith("fixed_") else "compact_diff"
                planner = MorphologyAStar(selected_catalog, scenario.grid, heading_bins=8)
                started = time.perf_counter()
                try:
                    plan = planner.plan(HybridState(*scenario.start, 0, start_morphology), scenario.goal, epsilon=2.5)
                    success, cost, expanded = True, plan.total_cost, plan.expanded
                    transitions = sum(s.kind == "reconfigure" for s in plan.segments)
                except NoPathError:
                    success, cost, expanded, transitions = False, float("nan"), 0, 0
                rows.append({
                    "family": family, "layout_seed": seed, "method": method,
                    "plan_found": int(success), "objective_cost": cost,
                    "planning_seconds": time.perf_counter() - started,
                    "expanded_states": expanded, "reconfigurations": transitions,
                })
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--catalog", type=Path, default=Path("src/modular_robot_description/config/morphologies.yaml"))
    parser.add_argument("--layouts", type=int, default=3)
    parser.add_argument("--output", type=Path, default=Path("results/diagnostics/planner_benchmark.csv"))
    args = parser.parse_args()
    run(args.catalog, args.layouts, args.output)


if __name__ == "__main__":
    main()
