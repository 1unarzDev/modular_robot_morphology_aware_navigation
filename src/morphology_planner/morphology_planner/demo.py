from pathlib import Path

from .catalog import load_catalog
from .grid import OCCUPIED, OccupancyGrid
from .planner import HybridState, MorphologyAStar


def _catalog_path() -> Path:
    return Path(__file__).parents[2] / "modular_robot_description" / "config" / "morphologies.yaml"


def main() -> None:
    # Leave room for the 1.76 m narrow body and its transition sweep on both
    # sides of the wall.  The opening admits narrow_tandem but not compact_diff.
    grid = OccupancyGrid(55, 25, 0.1)
    for y in range(grid.height):
        if not 10 <= y <= 16:
            grid.set_value(20, y, OCCUPIED)
    catalog = load_catalog(_catalog_path()).supported_experiment_subset()
    planner = MorphologyAStar(catalog, grid, heading_bins=4)
    plans = planner.plan_anytime(HybridState(7, 13, 0, "compact_diff"), (42, 13))
    plan = plans[-1]
    morphologies = [plan.states[0].morphology]
    for segment in plan.segments:
        if segment.kind == "reconfigure":
            morphologies.append(segment.target.morphology)
    print(f"cost={plan.total_cost:.2f} expanded={plan.expanded} morphologies={morphologies}")


if __name__ == "__main__":
    main()
