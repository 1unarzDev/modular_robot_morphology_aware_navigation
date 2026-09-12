from pathlib import Path

from .catalog import load_catalog
from .grid import OCCUPIED, OccupancyGrid
from .planner import HybridState, MorphologyAStar


def _catalog_path() -> Path:
    return Path(__file__).parents[2] / "modular_robot_description" / "config" / "morphologies.yaml"


def main() -> None:
    grid = OccupancyGrid(40, 25, 0.1)
    for y in range(grid.height):
        if not 11 <= y <= 14:
            grid.set_value(20, y, OCCUPIED)
    planner = MorphologyAStar(load_catalog(_catalog_path()), grid, heading_bins=8)
    plans = planner.plan_anytime(HybridState(7, 13, 0, "compact_diff"), (32, 13))
    plan = plans[-1]
    morphologies = [plan.states[0].morphology]
    for segment in plan.segments:
        if segment.kind == "reconfigure":
            morphologies.append(segment.target.morphology)
    print(f"cost={plan.total_cost:.2f} expanded={plan.expanded} morphologies={morphologies}")


if __name__ == "__main__":
    main()
