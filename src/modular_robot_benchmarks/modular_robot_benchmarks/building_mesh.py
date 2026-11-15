"""Turn a scanned building mesh into the planner's two world models.

The planner holds two priors about a world (ADR 0012): a 2D occupancy grid,
which is what the robot's lidar maps at its 0.12 m scan plane, and a set of 3D
boxes for everything the scan plane cannot see but a module or a sight line can
meet. A scanned building supplies both from one surface: the grid from the
geometry crossing the scan plane, the boxes from everything up to the height
the fiducial station's sight lines need.

Pure numpy and no ROS, so it runs on the Windows host. Only geometry is read:
HM3D carries no object labels in its GLB files beyond ceilings, and none is
needed. Every threshold below is a declared constant, fixed before the
real-buildings sample is read (`studies/real_buildings/declaration.md`).
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import struct
from pathlib import Path

import numpy as np

from morphology_planner.grid import FREE, OCCUPIED, OccupancyGrid
from morphology_planner.transition_validation import Box3

RESOLUTION = 0.1           # grid cell, m; as in every study layout
VOXEL_Z = 0.05             # vertical voxel size for the 3D boxes, m
SAMPLE_DENSITY = 2500.0    # surface samples per m^2 (about 2 cm spacing)
SAMPLE_SEED = 0            # surface sampling is deterministic per building
UPWARD_NZ = 0.85           # a triangle faces up if its normal's z exceeds this
LEVEL_MIN_AREA_M2 = 8.0    # upward area in one 5 cm band that makes a floor level
LEVEL_MIN_SEPARATION_M = 1.8
FLOOR_TOLERANCE_M = 0.05   # floor samples within this of the level height
# The lidar scan plane is 0.12 m above the floor (modular_robot.urdf.xacro).
# A surface anywhere in [0.05, 0.14] m is mapped into the grid; one wholly above
# it is not, which keeps the study's raised-obstacle regime (underside
# 0.13-0.26 m) as the only unmapped class. The generated worlds had nothing
# below the scan plane; leaving low geometry unmapped would turn scan noise at
# wall bases into invisible obstacles for every method alike. Below 0.05 m is
# floor.
SCAN_BAND_M = (0.05, 0.14)
# Geometry from the top of the floor band up to above the 1.8 m station:
# everything a module sweep or a station sight line can meet.
BOX_BAND_M = (0.05, 2.0)


@dataclass
class Level:
    """One floor of a building, in the building's own frame shifted to its grid."""

    index: int
    floor_z: float
    grid: OccupancyGrid
    floor_cells: np.ndarray     # bool [height, width]: observed floor
    occupancy: np.ndarray       # bool [height, width]: occupied in the grid
    voxels: np.ndarray          # bool [height, width, nz]: surface in the box band
    origin: tuple[float, float]


def read_glb_triangles(path: Path) -> np.ndarray:
    """All triangles of a GLB as an [n, 3, 3] float array (z up, metres).

    HM3D GLBs have no node transforms; a file that has any is refused rather
    than misread.
    """
    data = Path(path).read_bytes()
    magic, _version, _length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF":
        raise ValueError(f"{path}: not a GLB file")
    offset, document, binary = 12, None, None
    while offset < len(data):
        chunk_length, chunk_type = struct.unpack_from("<II", data, offset)
        body = data[offset + 8: offset + 8 + chunk_length]
        if chunk_type == 0x4E4F534A:
            document = json.loads(body)
        elif chunk_type == 0x004E4942:
            binary = body
        offset += 8 + chunk_length
    if document is None or binary is None:
        raise ValueError(f"{path}: missing JSON or BIN chunk")
    for node in document.get("nodes", ()):
        if any(key in node for key in ("matrix", "rotation", "scale", "translation")):
            raise ValueError(f"{path}: node transforms are not supported")
    component = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}

    def accessor(index):
        spec = document["accessors"][index]
        view = document["bufferViews"][spec["bufferView"]]
        width = {"SCALAR": 1, "VEC3": 3}[spec["type"]]
        start = view.get("byteOffset", 0) + spec.get("byteOffset", 0)
        dtype = np.dtype(component[spec["componentType"]])
        stride = view.get("byteStride", 0)
        if stride and stride != dtype.itemsize * width:
            raise ValueError(f"{path}: interleaved buffers are not supported")
        values = np.frombuffer(binary, dtype, spec["count"] * width, start)
        return values.reshape(-1, width) if width > 1 else values

    triangles = []
    for node in document.get("nodes", ()):
        if "mesh" not in node or "ceiling" in node.get("name", ""):
            continue
        for primitive in document["meshes"][node["mesh"]]["primitives"]:
            if primitive.get("mode", 4) != 4:
                continue
            positions = accessor(primitive["attributes"]["POSITION"]).astype(np.float64)
            indices = (accessor(primitive["indices"]).astype(np.int64)
                       if "indices" in primitive else np.arange(len(positions)))
            triangles.append(positions[indices.reshape(-1, 3)])
    if not triangles:
        raise ValueError(f"{path}: no triangles")
    return np.concatenate(triangles)


def sample_surface(triangles: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Area-weighted surface samples and whether each lies on an upward face.

    Every vertex is kept as well, so no triangle smaller than the sample
    spacing vanishes.
    """
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    cross = np.cross(b - a, c - a)
    doubled = np.linalg.norm(cross, axis=1)
    area = doubled / 2.0
    keep = doubled > 1e-12
    a, b, c, cross, doubled, area = (v[keep] for v in (a, b, c, cross, doubled, area))
    upward = cross[:, 2] / doubled > UPWARD_NZ
    rng = np.random.default_rng(SAMPLE_SEED)
    counts = rng.poisson(area * SAMPLE_DENSITY)
    owner = np.repeat(np.arange(len(area)), counts)
    r1, r2 = rng.random(len(owner)), rng.random(len(owner))
    flip = r1 + r2 > 1.0
    r1[flip], r2[flip] = 1.0 - r1[flip], 1.0 - r2[flip]
    points = a[owner] + r1[:, None] * (b - a)[owner] + r2[:, None] * (c - a)[owner]
    vertices = np.concatenate([a, b, c])
    return (np.concatenate([points, vertices]),
            np.concatenate([upward[owner], np.tile(upward, 3)]))


def floor_levels(triangles: np.ndarray) -> list[float]:
    """Heights of the building's floors, from where upward-facing area piles up."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    cross = np.cross(b - a, c - a)
    doubled = np.linalg.norm(cross, axis=1)
    keep = doubled > 1e-12
    upward = np.zeros(len(triangles), bool)
    upward[keep] = cross[keep, 2] / doubled[keep] > UPWARD_NZ
    z = triangles[upward, :, 2].mean(axis=1)
    weight = doubled[upward] / 2.0
    if not len(z):
        return []
    bins = np.floor(z / 0.05).astype(np.int64)
    low = bins.min()
    histogram = np.bincount(bins - low, weights=weight)
    candidates = sorted(
        ((area, (index + low + 0.5) * 0.05) for index, area in enumerate(histogram)
         if area >= LEVEL_MIN_AREA_M2), reverse=True)
    levels: list[float] = []
    for _area, height in candidates:
        if all(abs(height - other) >= LEVEL_MIN_SEPARATION_M for other in levels):
            levels.append(height)
    # Refine each level to the area-weighted mean height of its band.
    refined = []
    for height in sorted(levels):
        near = np.abs(z - height) <= 0.05
        refined.append(float(np.average(z[near], weights=weight[near])))
    return refined


def build_levels(triangles: np.ndarray) -> list[Level]:
    """Grid, observed floor and box-band voxels for every floor of a building."""
    points, upward = sample_surface(triangles)
    margin = 2 * RESOLUTION
    origin_x = float(np.floor((points[:, 0].min() - margin) / RESOLUTION) * RESOLUTION)
    origin_y = float(np.floor((points[:, 1].min() - margin) / RESOLUTION) * RESOLUTION)
    width = int(np.ceil((points[:, 0].max() + margin - origin_x) / RESOLUTION))
    height = int(np.ceil((points[:, 1].max() + margin - origin_y) / RESOLUTION))
    cx = np.clip(((points[:, 0] - origin_x) / RESOLUTION).astype(np.int64), 0, width - 1)
    cy = np.clip(((points[:, 1] - origin_y) / RESOLUTION).astype(np.int64), 0, height - 1)
    nz = int(round((BOX_BAND_M[1] - BOX_BAND_M[0]) / VOXEL_Z))
    levels = []
    for index, floor_z in enumerate(floor_levels(triangles)):
        relative = points[:, 2] - floor_z
        floor = np.zeros((height, width), bool)
        on_floor = upward & (np.abs(relative) <= FLOOR_TOLERANCE_M)
        floor[cy[on_floor], cx[on_floor]] = True
        scan = np.zeros((height, width), bool)
        in_scan = (relative >= SCAN_BAND_M[0]) & (relative <= SCAN_BAND_M[1])
        scan[cy[in_scan], cx[in_scan]] = True
        occupancy = scan | ~floor
        voxels = np.zeros((height, width, nz), bool)
        in_box = (relative >= BOX_BAND_M[0]) & (relative < BOX_BAND_M[1])
        cz = ((relative[in_box] - BOX_BAND_M[0]) / VOXEL_Z).astype(np.int64)
        voxels[cy[in_box], cx[in_box], np.clip(cz, 0, nz - 1)] = True
        grid = OccupancyGrid(width, height, RESOLUTION)
        grid.data = [OCCUPIED if value else FREE for value in occupancy.ravel()]
        levels.append(Level(index, floor_z, grid, floor, occupancy, voxels,
                            (origin_x, origin_y)))
    return levels


def crop_level(level: Level, x0: int, y0: int, x1: int, y1: int
               ) -> tuple[OccupancyGrid, tuple[Box3, ...], tuple[Box3, ...]]:
    """Grid, all boxes, and unmapped boxes of one cell window [x0, x1) x [y0, y1).

    The window becomes the planning world: its own frame starts at the
    window's corner, so planner cells are window cells. Boxes are greedy
    merges of the voxels, z runs first, then along x, then along y, so a wall
    is a handful of boxes rather than thousands. Voxels over grid-occupied
    cells and over free cells are merged separately: all of them block a
    sight line, but only the second set is what the scan plane misses (raised
    furniture), so only it enters module sweeps and driving -- as in every
    generated layout, where walls were grid-only and raised obstacles
    box-only.
    """
    occupancy = level.occupancy[y0:y1, x0:x1]
    height, width = occupancy.shape
    grid = OccupancyGrid(width, height, RESOLUTION)
    grid.data = [OCCUPIED if value else FREE for value in occupancy.ravel()]
    voxels = level.voxels[y0:y1, x0:x1]
    mapped = _merge_voxels(voxels & occupancy[:, :, None], "mapped")
    unmapped = _merge_voxels(voxels & ~occupancy[:, :, None], "unmapped")
    return grid, mapped + unmapped, unmapped


def _merge_voxels(voxels: np.ndarray, prefix: str) -> tuple[Box3, ...]:
    runs: dict[tuple[int, int, int], list[int]] = {}
    ys, xs = np.nonzero(voxels.any(axis=2))
    for y, x in zip(ys.tolist(), xs.tolist()):
        column = voxels[y, x]
        z = 0
        while z < len(column):
            if column[z]:
                start = z
                while z < len(column) and column[z]:
                    z += 1
                runs.setdefault((y, start, z), []).append(x)
            else:
                z += 1
    strips: dict[tuple[int, int, int, int], list[int]] = {}
    for (y, z0, z1), row in runs.items():
        row.sort()
        start = previous = row[0]
        for x in row[1:] + [None]:
            if x is not None and x == previous + 1:
                previous = x
                continue
            strips.setdefault((start, previous + 1, z0, z1), []).append(y)
            if x is not None:
                start = previous = x
    boxes = []
    for (bx0, bx1, z0, z1), column in strips.items():
        column.sort()
        start = previous = column[0]
        for y in column[1:] + [None]:
            if y is not None and y == previous + 1:
                previous = y
                continue
            zl = BOX_BAND_M[0] + z0 * VOXEL_Z
            zh = BOX_BAND_M[0] + z1 * VOXEL_Z
            boxes.append(Box3(
                f"{prefix}_{len(boxes)}",
                ((bx0 + bx1) / 2.0 * RESOLUTION, (start + previous + 1) / 2.0 * RESOLUTION,
                 (zl + zh) / 2.0),
                ((bx1 - bx0) * RESOLUTION, (previous + 1 - start) * RESOLUTION, zh - zl)))
            if y is not None:
                start = previous = y
    return tuple(boxes)
