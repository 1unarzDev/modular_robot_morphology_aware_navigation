"""Which planner states a robot can reach, computed as masks.

The real-buildings sampler needs to know, before any method plans, which
cells a mission can only reach by changing shape. Approximating that with
"the footprint fits at some heading" drew starts the planner could not leave:
the planner drives forward only and turns in place with clearance, so a cell
the compact body fits in may still be a dead end. This module reproduces the
planner's own lattice -- `MorphologyAStar._motion_neighbors` and
`_traversal_is_free` -- as boolean masks, by rasterizing each motion
primitive's swept footprint into a kernel of cell offsets once, so reachability
over a whole floor is a breadth-first sweep of array shifts.

It reproduces the planner's 2D grid checks (the same sample positions,
footprints and polygon-cell test). Offsets are computed relative to the
source cell rather than in absolute coordinates, so where a footprint edge
exactly touches a cell boundary floating-point rounding can differ; checked
against `_traversal_is_free` on a scanned floor, 8 of 987 moves differed, all
with the mask the stricter. Raised obstacles are rasterized to whole
cells, a slight over-approximation; the sampler only proposes missions and the
planner decides them.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from math import ceil, cos, hypot, pi, sin

import numpy as np

from morphology_planner.grid import _polygon_intersects_cell, polygon_intersects_rectangle
from morphology_planner.planner import (
    DRIVE_OBSTACLE_CLEARANCE_M, TURN_CLEARANCE_M, inflated_footprint,
)


def _polygon(footprint, x, y, yaw):
    c, s = cos(yaw), sin(yaw)
    return tuple((x + c * px - s * py, y + s * px + c * py) for px, py in footprint)


def _cells_hit(polygon, resolution):
    xs, ys = [p[0] for p in polygon], [p[1] for p in polygon]
    low_x, high_x = int(np.floor(min(xs) / resolution)) - 1, int(np.ceil(max(xs) / resolution)) + 1
    low_y, high_y = int(np.floor(min(ys) / resolution)) - 1, int(np.ceil(max(ys) / resolution)) + 1
    return {(i, j) for i in range(low_x, high_x + 1) for j in range(low_y, high_y + 1)
            if _polygon_intersects_cell(polygon, i * resolution, j * resolution, resolution)}


def direction(heading: int, bins: int) -> tuple[int, int]:
    yaw = heading * 2 * pi / bins
    return int(round(cos(yaw))), int(round(sin(yaw)))


@lru_cache(maxsize=None)
def _footprint_kernel(footprint, heading: int, bins: int, resolution: float):
    return frozenset(_cells_hit(_polygon(footprint, 0.0, 0.0, heading * 2 * pi / bins),
                                resolution))


def move_kernels(morphology, heading: int, move: tuple[int, int, int], bins: int,
                 resolution: float) -> tuple[frozenset, frozenset]:
    return _move_kernels(morphology.footprint, morphology.radius, heading, move, bins,
                         resolution)


@lru_cache(maxsize=None)
def _move_kernels(footprint_points, radius, heading, move, bins, resolution):
    """Grid and drive-obstacle cell offsets swept by one primitive from (0, 0).

    Mirrors `_traversal_is_free`: the same sample count, footprints and
    interpolation; offsets are relative to the source cell.
    """
    dx, dy, dheading = move
    delta_yaw = dheading * 2.0 * pi / bins
    swept = max(hypot(dx * resolution, dy * resolution), abs(delta_yaw) * radius)
    samples = max(1, ceil(swept / (0.25 * resolution)))
    footprint = (inflated_footprint(footprint_points, TURN_CLEARANCE_M)
                 if dheading else footprint_points)
    drive_footprint = inflated_footprint(footprint, DRIVE_OBSTACLE_CLEARANCE_M)
    yaw0 = heading * 2.0 * pi / bins
    grid_cells, drive_cells = set(), set()
    for index in range(samples + 1):
        fraction = index / samples
        x, y = fraction * dx * resolution, fraction * dy * resolution
        yaw = yaw0 + fraction * delta_yaw
        grid_cells |= _cells_hit(_polygon(footprint, x, y, yaw), resolution)
        drive_cells |= _cells_hit(_polygon(drive_footprint, x, y, yaw), resolution)
    return frozenset(grid_cells), frozenset(drive_cells)


def _dilate_offsets(mask: np.ndarray, offsets, pad: int) -> np.ndarray:
    """out[y, x] = any(mask[y + j, x + i] for (i, j) in offsets); outside is True."""
    height, width = mask.shape
    padded = np.ones((height + 2 * pad, width + 2 * pad), bool)
    padded[pad:pad + height, pad:pad + width] = mask
    out = np.zeros_like(mask)
    for i, j in offsets:
        out |= padded[pad + j:pad + j + height, pad + i:pad + i + width]
    return out


@dataclass
class LatticeModel:
    """Per-level blocked masks for every primitive of the planner's lattice."""

    bins: int
    resolution: float
    state_free: dict      # (morph, heading) -> bool[h, w]
    move_free: dict       # (morph, heading, move) -> bool[h, w], move from the cell
    transition_free: dict  # (transition, heading) -> bool[h, w], geometry-only test
    moves: dict           # (morph, heading) -> list of (dx, dy, dheading)


def build_model(occupancy: np.ndarray, drive_cells: dict, catalog, bins: int,
                resolution: float) -> LatticeModel:
    """`drive_cells[morph]`: cells a raised obstacle below that body's height covers."""
    pad = 20
    state_free, move_free, transition_free, moves = {}, {}, {}, {}
    for morph in catalog.morphologies.values():
        for heading in range(bins):
            footprint = _footprint_kernel(morph.footprint, heading, bins, resolution)
            state_free[morph.id, heading] = ~_dilate_offsets(occupancy, footprint, pad)
            forward = direction(heading, bins)
            primitive = [(forward[0], forward[1], 0), (0, 0, 1), (0, 0, -1)]
            moves[morph.id, heading] = primitive
            for move in primitive:
                grid_k, drive_k = move_kernels(morph, heading, move, bins, resolution)
                blocked = _dilate_offsets(occupancy, grid_k, pad)
                blocked |= _dilate_offsets(drive_cells[morph.id], drive_k, pad)
                move_free[morph.id, heading, move] = ~blocked
    for transition in catalog.transitions:
        radius_cells = int(transition.swept_radius / resolution) + 1
        disk = [(i, j) for i in range(-radius_cells, radius_cells + 1)
                for j in range(-radius_cells, radius_cells + 1)
                # Cells exactly on the radius tie under rounding in
                # `disk_is_free`; including them keeps the mask the stricter.
                if (i * resolution) ** 2 + (j * resolution) ** 2
                <= transition.swept_radius ** 2 + 1e-9]
        disk_free = ~_dilate_offsets(occupancy, disk, pad)
        for heading in range(bins):
            transition_free[transition.id, heading] = (
                disk_free & state_free[transition.target, heading])
    return LatticeModel(bins, resolution, state_free, move_free, transition_free, moves)


def drive_cell_masks(shape, drive_boxes, catalog, resolution) -> dict:
    """Cells each body's drive check would meet, rounded out to whole cells."""
    masks = {}
    for morph in catalog.morphologies.values():
        mask = np.zeros(shape, bool)
        for box in drive_boxes:
            if box.center[2] - box.size[2] / 2.0 >= morph.height:
                continue
            x0 = box.center[0] - box.size[0] / 2.0
            x1 = box.center[0] + box.size[0] / 2.0
            y0 = box.center[1] - box.size[1] / 2.0
            y1 = box.center[1] + box.size[1] / 2.0
            cx0, cx1 = max(0, int(np.floor(x0 / resolution))), min(shape[1] - 1, int(np.floor(x1 / resolution)))
            cy0, cy1 = max(0, int(np.floor(y0 / resolution))), min(shape[0] - 1, int(np.floor(y1 / resolution)))
            mask[cy0:cy1 + 1, cx0:cx1 + 1] = True
        masks[morph.id] = mask
    return masks


def _shift(mask: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """out[y + dy, x + dx] = mask[y, x]."""
    out = np.zeros_like(mask)
    height, width = mask.shape
    ys = slice(max(0, dy), min(height, height + dy))
    xs = slice(max(0, dx), min(width, width + dx))
    yo = slice(max(0, -dy), min(height, height - dy))
    xo = slice(max(0, -dx), min(width, width - dx))
    out[ys, xs] = mask[yo, xo]
    return out


def reach(model: LatticeModel, start: tuple[str, int, int, int], transitions,
          max_depth: int) -> dict:
    """Breadth-first depth of every reachable (morph, heading) cell mask.

    `start` is (morph, heading, y, x). Transitions in `transitions` (catalog
    objects) are edges where the geometry-only test passes. Returns
    {(morph, heading): int16 depth array, -1 unreached}.
    """
    keys = list(model.state_free)
    shape = next(iter(model.state_free.values())).shape
    depth = {key: np.full(shape, -1, np.int16) for key in keys}
    morph, heading, sy, sx = start
    depth[morph, heading][sy, sx] = 0
    frontier = {key: np.zeros(shape, bool) for key in keys}
    frontier[morph, heading][sy, sx] = True
    for level in range(1, max_depth + 1):
        new = {key: np.zeros(shape, bool) for key in keys}
        any_new = False
        for (m, h), current in frontier.items():
            if not current.any():
                continue
            for move in model.moves[m, h]:
                movable = current & model.move_free[m, h, move]
                if not movable.any():
                    continue
                dx, dy, dh = move
                new[m, (h + dh) % model.bins] |= _shift(movable, dx, dy)
            for transition in transitions:
                if transition.source == m:
                    new[transition.target, h] |= current & model.transition_free[transition.id, h]
        for key in keys:
            fresh = new[key] & (depth[key] < 0) & model.state_free[key]
            depth[key][fresh] = level
            frontier[key] = fresh
            any_new |= bool(fresh.any())
        if not any_new:
            break
    return depth


def backtrack(model: LatticeModel, depth: dict, goal: tuple[str, int, int, int],
              transitions) -> list[tuple[str, int, int, int]]:
    """One shortest lattice path from the start to `goal`, start first."""
    path = [goal]
    m, h, y, x = goal
    while depth[m, h][y, x] > 0:
        d = depth[m, h][y, x]
        found = None
        for (pm, ph), moves in model.moves.items():
            for move in moves:
                dx, dy, dh = move
                if pm != m or (ph + dh) % model.bins != h:
                    continue
                py, px = y - dy, x - dx
                if (0 <= py < depth[pm, ph].shape[0] and 0 <= px < depth[pm, ph].shape[1]
                        and depth[pm, ph][py, px] == d - 1
                        and model.move_free[pm, ph, move][py, px]):
                    found = (pm, ph, py, px)
                    break
            if found:
                break
        if found is None:
            for transition in transitions:
                if (transition.target == m and depth[transition.source, h][y, x] == d - 1
                        and model.transition_free[transition.id, h][y, x]):
                    found = (transition.source, h, y, x)
                    break
        if found is None:
            raise RuntimeError("broken breadth-first depth map")
        path.append(found)
        m, h, y, x = found
    return path[::-1]
