"""Offset-doorway courses from the ExperimentChoice track (EXPLORATORY).

One course is a rectangular approach bay, a door wall with one opening, and an
exit room. It is defined by the integer centimetres an ExperimentChoice truth
manifest hashes (`crane_ml/Tools/ExperimentChoice/course.py`, schema
`expchoice.course/v0`), and everything a trial needs is derived from those
integers here: the wall boxes of the Gazebo world, the occupancy map, the
evaluator's obstacle boxes, the start and the goal. Nothing in this module is
used by any other study.

Frame: x along the bay depth (back wall inner face at x = 0, door wall in
[d, d + t]); y lateral, door opening of width w centred at y = e; heading from
+x. The map frame, the Gazebo world frame and the manifest frame coincide.

Sub-cell geometry. The other families live on the planner's 0.1 m grid: a wall
is a set of cells, and the world, the map and the evaluator all use those
cells. A doorway course is given to the centimetre, so here

* the world and the evaluator use the exact wall rectangles (millimetre
  integers, one box per wall; the evaluator's copies are cut into tiles no
  larger than `OBSTACLE_TILE_MM` because `clearance_metrics` only looks at
  obstacles whose centre is within 1 m of a module);
* the occupancy map is rasterised at `MAP_CELL_MM` = 10 mm, with its origin on
  the 50 mm lattice through the manifest origin and its size a multiple of
  five cells, so a 5x downsample tiles it exactly. A cell is occupied when it
  overlaps a wall with positive area (conservative). Every wall edge of a
  course with even `w_cm` and even `B_cm` is a whole centimetre, and its map is
  then exact; an odd `w_cm` or `B_cm` puts edges on the half centimetre and the
  map closes the opening by one cell in total. `map_exactness` says which.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from math import ceil, floor
from pathlib import Path
from typing import Any, Mapping

FAMILY = "offset_doorway"
MANIFEST_SCHEMA = "expchoice.course/v0"
MAP_CELL_MM = 10
MAP_RESOLUTION_M = MAP_CELL_MM / 1000.0
# The map origin and extent sit on this lattice (anchored at the manifest
# origin), so a planner that downsamples the costmap by ALIGNMENT/MAP_CELL = 5
# gets blocks that never straddle a wall edge of a 5 cm-aligned course.
MAP_ALIGNMENT_MM = 50
MAP_PAD_MM = 250
WALL_HEIGHT_M = 1.0
OBSTACLE_TILE_MM = 200
COURSE_FIELDS = (
    "w_cm", "e_cm", "d_cm", "B_cm", "t_cm", "wall_cm", "exit_depth_cm",
    "start_x_cm", "start_y_cm", "start_yaw_mrad", "goal_x_cm", "goal_y_cm",
)


@dataclass(frozen=True)
class WallBox:
    """An axis-aligned wall, integer millimetres."""

    name: str
    xmin: int
    xmax: int
    ymin: int
    ymax: int


@dataclass(frozen=True)
class DoorwayCourse:
    w_cm: int
    e_cm: int
    d_cm: int
    B_cm: int
    t_cm: int
    wall_cm: int
    exit_depth_cm: int
    start_x_cm: int
    start_y_cm: int
    start_yaw_mrad: int
    goal_x_cm: int
    goal_y_cm: int

    def __post_init__(self) -> None:
        for name in COURSE_FIELDS:
            if not isinstance(getattr(self, name), int) or isinstance(getattr(self, name), bool):
                raise ValueError(f"course field {name} must be an integer")
        if min(self.w_cm, self.d_cm, self.B_cm, self.t_cm, self.wall_cm,
               self.exit_depth_cm) <= 0:
            raise ValueError("w, d, B, t, wall and exit depth must be positive")
        if 2 * abs(self.e_cm) + self.w_cm > self.B_cm:
            raise ValueError("door opening must lie inside the bay width")

    # ------------------------------------------------------------ identity
    def params(self) -> dict[str, Any]:
        """The canonical content ExperimentChoice hashes, key for key."""
        return {
            "schema": MANIFEST_SCHEMA, "family": FAMILY,
            "w_cm": self.w_cm, "e_cm": self.e_cm, "d_cm": self.d_cm,
            "B_cm": self.B_cm, "t_cm": self.t_cm,
            "start_cm_mrad": [self.start_x_cm, self.start_y_cm, self.start_yaw_mrad],
            "goal_cm": [self.goal_x_cm, self.goal_y_cm],
            "wall_cm": self.wall_cm, "exit_depth_cm": self.exit_depth_cm,
        }

    @property
    def course_id(self) -> str:
        canon = json.dumps(self.params(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canon.encode()).hexdigest()[:16]

    def fields(self) -> dict[str, int]:
        return {name: getattr(self, name) for name in COURSE_FIELDS}

    # ------------------------------------------------------------ geometry
    @property
    def start(self) -> tuple[float, float, float]:
        return (self.start_x_cm / 100.0, self.start_y_cm / 100.0,
                self.start_yaw_mrad / 1000.0)

    @property
    def goal(self) -> tuple[float, float]:
        return (self.goal_x_cm / 100.0, self.goal_y_cm / 100.0)

    def walls(self) -> tuple[WallBox, ...]:
        """The wall rectangles, as `course.py` lays them out, in millimetres."""
        d, t, wall = 10 * self.d_cm, 10 * self.t_cm, 10 * self.wall_cm
        half_b = 5 * self.B_cm                      # B/2 in mm
        low = 10 * self.e_cm - 5 * self.w_cm        # opening edges in mm
        high = 10 * self.e_cm + 5 * self.w_cm
        x_end = d + t + 10 * self.exit_depth_cm
        boxes = [
            WallBox("back_wall", -wall, 0, -half_b - wall, half_b + wall),
            WallBox("side_wall_neg", -wall, x_end + wall, -half_b - wall, -half_b),
            WallBox("side_wall_pos", -wall, x_end + wall, half_b, half_b + wall),
            WallBox("end_wall", x_end, x_end + wall, -half_b - wall, half_b + wall),
        ]
        if low > -half_b:
            boxes.append(WallBox("door_wall_neg", d, d + t, -half_b, low))
        if high < half_b:
            boxes.append(WallBox("door_wall_pos", d, d + t, high, half_b))
        return tuple(boxes)

    def map_exactness(self) -> dict[str, Any]:
        """Whether the 10 mm map reproduces every wall edge, and by how much not."""
        off = sorted({(box.name, edge) for box in self.walls()
                      for edge in (box.xmin, box.xmax, box.ymin, box.ymax)
                      if edge % MAP_CELL_MM})
        aligned = all(edge % MAP_ALIGNMENT_MM == 0 for box in self.walls()
                      for edge in (box.xmin, box.xmax, box.ymin, box.ymax))
        return {
            "exact_at_map_resolution": not off,
            "off_lattice_edges": [f"{name}:{edge / 1000.0:.4f}" for name, edge in off],
            # True when a 5x downsample of the map (0.05 m) is exact as well.
            "exact_at_0p05_m": aligned,
        }


def course_from_manifest(manifest: Mapping[str, Any]) -> DoorwayCourse:
    """A course from an ExperimentChoice truth manifest, checked against itself.

    The manifest states its geometry three times: the hashed integers, the
    metre parameters and the wall list. All three and the course id have to
    agree with what this module derives, or the manifest is refused.
    """
    if manifest.get("schema") != MANIFEST_SCHEMA or manifest.get("family") != FAMILY:
        raise ValueError("not an expchoice.course/v0 offset_doorway manifest")
    params = manifest["parameters"]
    start, goal = params["start_cm_mrad"], params["goal_cm"]
    course = DoorwayCourse(
        w_cm=params["w_cm"], e_cm=params["e_cm"], d_cm=params["d_cm"],
        B_cm=params["B_cm"], t_cm=params["t_cm"], wall_cm=params["wall_cm"],
        exit_depth_cm=params["exit_depth_cm"],
        start_x_cm=start[0], start_y_cm=start[1], start_yaw_mrad=start[2],
        goal_x_cm=goal[0], goal_y_cm=goal[1])
    if course.params() != dict(params):
        raise ValueError("manifest parameters carry fields this family does not model")
    if course.course_id != manifest.get("course_id"):
        raise ValueError(
            f"manifest course_id {manifest.get('course_id')} does not match its "
            f"parameters ({course.course_id})")
    declared = {wall["name"]: tuple(round(float(wall[key]) * 1000.0)
                                    for key in ("xmin", "xmax", "ymin", "ymax"))
                for wall in manifest["walls"]}
    derived = {box.name: (box.xmin, box.xmax, box.ymin, box.ymax) for box in course.walls()}
    if declared != derived:
        raise ValueError("manifest walls disagree with the walls its parameters imply")
    metres = manifest.get("parameters_m", {})
    for key in ("w", "e", "d", "B", "t"):
        if key in metres and round(float(metres[key]) * 100.0) != getattr(course, f"{key}_cm"):
            raise ValueError(f"manifest parameters_m.{key} disagrees with {key}_cm")
    if "start" in manifest:
        stated = manifest["start"]
        if (round(stated["x"] * 100), round(stated["y"] * 100),
                round(stated["theta"] * 1000)) != tuple(start):
            raise ValueError("manifest start disagrees with start_cm_mrad")
    if "goal" in manifest:
        stated = manifest["goal"]
        if (round(stated["x"] * 100), round(stated["y"] * 100)) != tuple(goal):
            raise ValueError("manifest goal disagrees with goal_cm")
    return course


def course_from_spec(spec) -> DoorwayCourse:
    """The course a doorway trial names; its id must be the one the spec states."""
    course = DoorwayCourse(**{name: getattr(spec, name) for name in COURSE_FIELDS})
    if course.course_id != spec.course_id:
        raise ValueError(
            f"trial {spec.trial_id}: course fields hash to {course.course_id}, "
            f"the spec says {spec.course_id}")
    return course


# --------------------------------------------------------------------- map

@dataclass(frozen=True)
class DoorwayMap:
    """Occupancy raster: `rows[iy][ix]` is 1 where the cell overlaps a wall.

    Cell (ix, iy) covers origin + [ix, ix + 1] x [iy, iy + 1] cells; row 0 is
    the lowest y. Origin in millimetres.
    """

    width: int
    height: int
    origin_x_mm: int
    origin_y_mm: int
    rows: tuple[bytes, ...]

    @property
    def origin(self) -> tuple[float, float]:
        return (self.origin_x_mm / 1000.0, self.origin_y_mm / 1000.0)

    def occupied(self, ix: int, iy: int) -> bool:
        return bool(self.rows[iy][ix])

    def cell_of(self, x_m: float, y_m: float) -> tuple[int, int]:
        return (floor((x_m * 1000.0 - self.origin_x_mm) / MAP_CELL_MM),
                floor((y_m * 1000.0 - self.origin_y_mm) / MAP_CELL_MM))


def _floor_to(value: int, step: int) -> int:
    return (value // step) * step


def _ceil_to(value: int, step: int) -> int:
    return -((-value) // step) * step


def rasterize(course: DoorwayCourse) -> DoorwayMap:
    walls = course.walls()
    x0 = _floor_to(min(box.xmin for box in walls) - MAP_PAD_MM, MAP_ALIGNMENT_MM)
    y0 = _floor_to(min(box.ymin for box in walls) - MAP_PAD_MM, MAP_ALIGNMENT_MM)
    x1 = _ceil_to(max(box.xmax for box in walls) + MAP_PAD_MM, MAP_ALIGNMENT_MM)
    y1 = _ceil_to(max(box.ymax for box in walls) + MAP_PAD_MM, MAP_ALIGNMENT_MM)
    width, height = (x1 - x0) // MAP_CELL_MM, (y1 - y0) // MAP_CELL_MM
    rows = [bytearray(width) for _ in range(height)]
    for box in walls:
        ix0 = max(0, floor((box.xmin - x0) / MAP_CELL_MM))
        ix1 = min(width, ceil((box.xmax - x0) / MAP_CELL_MM))
        iy0 = max(0, floor((box.ymin - y0) / MAP_CELL_MM))
        iy1 = min(height, ceil((box.ymax - y0) / MAP_CELL_MM))
        if ix1 <= ix0:
            continue
        run = b"\x01" * (ix1 - ix0)
        for iy in range(iy0, iy1):
            rows[iy][ix0:ix1] = run
    return DoorwayMap(width, height, x0, y0, tuple(bytes(row) for row in rows))


def pgm_bytes(raster: DoorwayMap) -> bytes:
    """Binary PGM in map_server order: first row is the highest y; 0 occupied, 254 free."""
    table = bytes([254, 0])
    body = b"".join(row.translate(table + bytes(254)) for row in reversed(raster.rows))
    return f"P5\n{raster.width} {raster.height}\n255\n".encode("ascii") + body


def map_yaml(image_name: str, raster: DoorwayMap) -> str:
    # Same thresholds and mode as `sdf_export.export_occupancy_map`.
    return (f"image: {image_name}\nmode: trinary\nresolution: {MAP_RESOLUTION_M}\n"
            f"origin: [{raster.origin[0]}, {raster.origin[1]}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")


def export_doorway_map(course: DoorwayCourse, output_yaml: Path) -> tuple[Path, Path]:
    output_yaml = Path(output_yaml)
    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    image_path = output_yaml.with_suffix(".pgm")
    raster = rasterize(course)
    image_path.write_bytes(pgm_bytes(raster))
    output_yaml.write_text(map_yaml(image_path.name, raster), encoding="utf-8")
    return output_yaml, image_path


def measure_map(course: DoorwayCourse, raster: DoorwayMap) -> dict[str, float]:
    """Door width, door centre, bay depth and wall thickness read back off the raster.

    Free runs are counted in cells along one column through the door wall and
    one row through the bay, so this measures the map and not the course.
    """
    def free_run(cells, start):
        low = high = start
        while low - 1 >= 0 and not cells[low - 1]:
            low -= 1
        while high + 1 < len(cells) and not cells[high + 1]:
            high += 1
        return low, high

    door_ix, door_iy = raster.cell_of((course.d_cm + course.t_cm / 2.0) / 100.0,
                                      course.e_cm / 100.0)
    column = [raster.rows[iy][door_ix] for iy in range(raster.height)]
    if column[door_iy]:
        raise ValueError("the door centre is occupied in the map")
    low, high = free_run(column, door_iy)
    width_m = (high - low + 1) * MAP_RESOLUTION_M
    centre_m = raster.origin[1] + (low + high + 1) / 2.0 * MAP_RESOLUTION_M
    # A row through the bay that meets the door wall, not the opening.
    solid_y_mm = (5 * course.B_cm + 10 * course.e_cm + 5 * course.w_cm) // 2
    if 10 * course.e_cm + 5 * course.w_cm >= 5 * course.B_cm:
        solid_y_mm = (-5 * course.B_cm + 10 * course.e_cm - 5 * course.w_cm) // 2
    bay_ix, bay_iy = raster.cell_of(course.d_cm / 200.0, solid_y_mm / 1000.0)
    row = raster.rows[bay_iy]
    if row[bay_ix]:
        raise ValueError("the bay is occupied in the map")
    low, high = free_run(row, bay_ix)
    depth_m = (high - low + 1) * MAP_RESOLUTION_M
    back_face_m = raster.origin[0] + low * MAP_RESOLUTION_M
    thickness = 0
    index = high + 1
    while index < raster.width and row[index]:
        thickness += 1
        index += 1
    return {"door_width_m": round(width_m, 6), "door_centre_m": round(centre_m, 6),
            "bay_depth_m": round(depth_m, 6), "back_wall_face_m": round(back_face_m, 6),
            "door_wall_thickness_m": round(thickness * MAP_RESOLUTION_M, 6)}


# --------------------------------------------------------------- evaluator

def wall_boxes_m(course: DoorwayCourse) -> list[tuple[str, tuple, tuple]]:
    """(name, centre, size) of every wall as the world file places it, metres."""
    return [(box.name,
             ((box.xmin + box.xmax) / 2000.0, (box.ymin + box.ymax) / 2000.0,
              WALL_HEIGHT_M / 2.0),
             ((box.xmax - box.xmin) / 1000.0, (box.ymax - box.ymin) / 1000.0,
              WALL_HEIGHT_M))
            for box in course.walls()]


def obstacle_boxes(course: DoorwayCourse) -> list[tuple[tuple, tuple]]:
    """The walls as `clearance_metrics` needs them: exact, in tiles <= 0.2 m.

    `clearance_metrics` skips an obstacle whose centre is more than 1 m from a
    module, which is harmless for the per-cell boxes of the other families and
    would hide a 5 m wall. The tiles' union is exactly the wall.
    """
    def cuts(low: int, high: int) -> list[tuple[int, int]]:
        count = max(1, ceil((high - low) / OBSTACLE_TILE_MM))
        edges = [low + (high - low) * index // count for index in range(count + 1)]
        return list(zip(edges[:-1], edges[1:]))

    tiles = []
    for box in course.walls():
        for xa, xb in cuts(box.xmin, box.xmax):
            for ya, yb in cuts(box.ymin, box.ymax):
                tiles.append((((xa + xb) / 2000.0, (ya + yb) / 2000.0, WALL_HEIGHT_M / 2.0),
                              ((xb - xa) / 1000.0, (yb - ya) / 1000.0, WALL_HEIGHT_M)))
    return tiles


def scenario_manifest(course: DoorwayCourse) -> dict[str, Any]:
    """What `world.manifest.json` states, in the shape the stack's readers expect.

    The planner and the station node read `transition_obstacles` and
    `fiducial_stations` from this file; a doorway course declares neither.
    """
    return {
        "schema_version": 1,
        "family": FAMILY,
        "layout_id": f"{FAMILY}-{course.course_id}",
        "course_id": course.course_id,
        "neutral_control": True,
        "start": list(course.start),
        "goal": list(course.goal),
        "parameters": course.params(),
        "walls": [{"name": box.name, "xmin": box.xmin / 1000.0, "xmax": box.xmax / 1000.0,
                   "ymin": box.ymin / 1000.0, "ymax": box.ymax / 1000.0}
                  for box in course.walls()],
        "map": {"resolution": MAP_RESOLUTION_M, **course.map_exactness()},
        "transition_obstacles": [],
        "observability_regions": [],
        "fiducial_stations": [],
    }
