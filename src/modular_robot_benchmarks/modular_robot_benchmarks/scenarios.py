from __future__ import annotations

from dataclasses import dataclass
from random import Random

from morphology_planner.grid import OCCUPIED, UNKNOWN, OccupancyGrid


FAMILIES = (
    "route_choice", "open_narrow_open", "compound_turn",
    "corridor_maneuver", "occluded_blockage",
)


@dataclass(frozen=True)
class Scenario:
    family: str
    layout_seed: int
    grid: OccupancyGrid
    start: tuple[int, int]
    goal: tuple[int, int]


def make_scenario(family: str, seed: int, resolution: float = 0.1) -> Scenario:
    if family not in FAMILIES:
        raise ValueError(f"unknown scenario family: {family}")
    rng = Random(seed)
    grid = OccupancyGrid(60, 40, resolution)
    _border(grid)
    start, goal = (8, 20), (51, 20)

    if family == "route_choice":
        gap_center = rng.randint(17, 22)
        for y in range(1, 34):
            if not gap_center - 1 <= y <= gap_center + 2:
                grid.set_value(30, y, OCCUPIED)
    elif family == "open_narrow_open":
        gap_center = rng.randint(18, 21)
        for x in (24, 36):
            for y in range(1, 39):
                if not gap_center - 1 <= y <= gap_center + 2:
                    grid.set_value(x, y, OCCUPIED)
    elif family == "compound_turn":
        offset = rng.randint(-2, 2)
        for x in range(18, 43):
            for y in range(1, 40):
                in_horizontal = (18 <= x <= 31 and 17 + offset <= y <= 22 + offset)
                in_vertical = (28 <= x <= 34 and 17 + offset <= y <= 34)
                in_exit = (31 <= x <= 43 and 29 <= y <= 34)
                if not (in_horizontal or in_vertical or in_exit):
                    grid.set_value(x, y, OCCUPIED)
        goal = (40, 31)
    elif family == "corridor_maneuver":
        width = rng.choice((5, 6))
        for x in range(13, 44):
            for y in range(1, 39):
                if not 20 - width // 2 <= y <= 20 + width // 2:
                    grid.set_value(x, y, OCCUPIED)
        for x, y in ((47, 16), (47, 24), (51, 16), (51, 24)):
            _block(grid, x, y, 2)
    else:
        for x in range(25, 48):
            for y in range(12, 29):
                grid.set_value(x, y, UNKNOWN)
        blockage_x = rng.randint(34, 40)
        for y in range(15, 26):
            grid.set_value(blockage_x, y, OCCUPIED)

    return Scenario(family, seed, grid, start, goal)


def _border(grid: OccupancyGrid) -> None:
    for x in range(grid.width):
        grid.set_value(x, 0, OCCUPIED)
        grid.set_value(x, grid.height - 1, OCCUPIED)
    for y in range(grid.height):
        grid.set_value(0, y, OCCUPIED)
        grid.set_value(grid.width - 1, y, OCCUPIED)


def _block(grid: OccupancyGrid, cx: int, cy: int, radius: int) -> None:
    for x in range(cx - radius, cx + radius + 1):
        for y in range(cy - radius, cy + radius + 1):
            grid.set_value(x, y, OCCUPIED)
