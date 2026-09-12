from __future__ import annotations

from dataclasses import asdict, dataclass
from random import Random

from morphology_planner import HybridState, PodSensingState
from morphology_planner.catalog import Transition
from morphology_planner.grid import OCCUPIED, OccupancyGrid
from morphology_planner.transition_validation import Box3

from .design import CONFIRMATORY_FAMILIES


@dataclass(frozen=True)
class ObservabilityRegion:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    connector_visible: bool
    covariance_trace: float

    def contains(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


@dataclass(frozen=True)
class ConfirmatoryScenario:
    family: str
    layout_index: int
    world_seed: int
    neutral_control: bool
    grid: OccupancyGrid
    start: tuple[int, int]
    goal: tuple[int, int]
    transition_obstacles: tuple[Box3, ...]
    observability_regions: tuple[ObservabilityRegion, ...]
    parameters: dict

    @property
    def layout_id(self) -> str:
        return f"{self.family}-{self.layout_index:02d}"

    def sensing_for(self, transition: Transition, state: HybridState):
        wx, wy = self.grid.cell_center(state.x, state.y)
        visible, covariance = True, 0.003
        for region in self.observability_regions:
            if region.contains(wx, wy):
                visible = region.connector_visible
                covariance = region.covariance_trace
                break
        return {
            pod: PodSensingState(visible, covariance)
            for pod in transition.moved_pods
        }

    def manifest(self) -> dict:
        return {
            "schema_version": 1,
            "family": self.family,
            "layout_id": self.layout_id,
            "world_seed": self.world_seed,
            "neutral_control": self.neutral_control,
            "start": list(self.start),
            "goal": list(self.goal),
            "parameters": self.parameters,
            "transition_obstacles": [asdict(box) for box in self.transition_obstacles],
            "observability_regions": [asdict(region) for region in self.observability_regions],
        }


def make_confirmatory_scenario(
    family: str,
    layout_index: int,
    world_seed: int,
    resolution: float = 0.1,
) -> ConfirmatoryScenario:
    if family not in CONFIRMATORY_FAMILIES:
        raise ValueError(f"unknown confirmatory family: {family}")
    if not 0 <= layout_index < 12:
        raise ValueError("confirmatory layout index must be in [0, 12)")
    rng = Random(world_seed)
    grid = OccupancyGrid(round(6.0 / resolution), round(3.5 / resolution), resolution)
    _border(grid)
    wall_x = round(3.0 / resolution) + rng.randrange(-1, 2)
    door_center = round(1.75 / resolution) + rng.randrange(-1, 2)
    # Use an odd cell count so the opening is centered on the route cell. The
    # compact 0.70 m safety footprint is rejected at wall contact, while the
    # narrow 0.52 m safety footprint retains 0.09 m clearance per side.
    door_cells = max(7, round(0.7 / resolution))
    door_low = door_center - door_cells // 2
    for y in range(1, grid.height - 1):
        if not door_low <= y < door_low + door_cells:
            grid.set_value(wall_x, y, OCCUPIED)
    start = (round(0.6 / resolution), door_center)
    goal = (round(5.0 / resolution), door_center)
    neutral = layout_index % 4 == 0

    # The direct staging band is broad enough to admit the 2D clearance disk.
    band_x0 = 0.0
    band_x1 = wall_x * resolution - 0.8 + 0.05 * rng.randrange(3)
    center_y = (door_center + 0.5) * resolution
    band_half_height = 0.45 + 0.05 * rng.randrange(3)
    obstacle = Box3(
        "raised_transition_shelf",
        ((band_x0 + band_x1) / 2.0, center_y + 0.20, 0.14),
        (band_x1 - band_x0, 0.16, 0.08),
    )
    poor_region = ObservabilityRegion(
        band_x0, band_x1,
        center_y - band_half_height, center_y + band_half_height,
        False, 0.04,
    )

    obstacles: tuple[Box3, ...] = ()
    regions: tuple[ObservabilityRegion, ...] = ()
    if not neutral:
        if family == "reconfiguration_workspace":
            obstacles = (obstacle,)
        elif family == "docking_observability":
            regions = (poor_region,)
        else:
            obstacles = (obstacle,)
            regions = (ObservabilityRegion(
                band_x0, band_x1, 0.0, center_y + band_half_height,
                False, 0.04,
            ),)

    parameters = {
        "resolution": resolution,
        "width": grid.width,
        "height": grid.height,
        "wall_x_cell": wall_x,
        "door_center_cell": door_center,
        "door_cells": door_cells,
        "direct_band_x": [band_x0, band_x1],
        "direct_band_half_height": band_half_height,
    }
    return ConfirmatoryScenario(
        family, layout_index, world_seed, neutral, grid, start, goal,
        obstacles, regions, parameters,
    )


def _border(grid: OccupancyGrid) -> None:
    for x in range(grid.width):
        grid.set_value(x, 0, OCCUPIED)
        grid.set_value(x, grid.height - 1, OCCUPIED)
    for y in range(grid.height):
        grid.set_value(0, y, OCCUPIED)
        grid.set_value(grid.width - 1, y, OCCUPIED)
