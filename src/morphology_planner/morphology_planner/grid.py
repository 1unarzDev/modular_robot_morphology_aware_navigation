from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, cos, floor, sin
from typing import Iterable


FREE = 0
OCCUPIED = 100
UNKNOWN = -1


@dataclass
class OccupancyGrid:
    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    data: list[int] = field(default_factory=list)
    revision: int = 0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0:
            raise ValueError("grid dimensions and resolution must be positive")
        if not self.data:
            self.data = [FREE] * (self.width * self.height)
        if len(self.data) != self.width * self.height:
            raise ValueError("grid data length does not match dimensions")

    def index(self, x: int, y: int) -> int:
        return y * self.width + x

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, x: int, y: int) -> int:
        return self.data[self.index(x, y)] if self.in_bounds(x, y) else OCCUPIED

    def set_value(self, x: int, y: int, value: int) -> None:
        if not self.in_bounds(x, y):
            raise IndexError((x, y))
        self.data[self.index(x, y)] = value
        self.revision += 1

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (floor((x - self.origin_x) / self.resolution),
                floor((y - self.origin_y) / self.resolution))

    def cell_center(self, x: int, y: int) -> tuple[float, float]:
        return (self.origin_x + (x + 0.5) * self.resolution,
                self.origin_y + (y + 0.5) * self.resolution)

    def coarsen(self, target_resolution: float) -> "OccupancyGrid":
        """Conservatively aggregate a SLAM grid for global hybrid search."""
        if target_resolution <= self.resolution:
            return self
        scale = max(1, ceil(target_resolution / self.resolution))
        output = OccupancyGrid(
            width=ceil(self.width / scale),
            height=ceil(self.height / scale),
            resolution=self.resolution * scale,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            revision=self.revision,
        )
        for oy in range(output.height):
            for ox in range(output.width):
                values = [
                    self.value(ix, iy)
                    for iy in range(oy * scale, min((oy + 1) * scale, self.height))
                    for ix in range(ox * scale, min((ox + 1) * scale, self.width))
                ]
                if any(value >= OCCUPIED for value in values):
                    value = OCCUPIED
                elif all(value == UNKNOWN for value in values):
                    value = UNKNOWN
                else:
                    value = max((value for value in values if value != UNKNOWN), default=FREE)
                output.data[output.index(ox, oy)] = value
        return output

    def padded_to_include(
        self,
        points: Iterable[tuple[float, float]],
        margin: float = 0.0,
    ) -> "OccupancyGrid":
        """Extend map bounds with UNKNOWN cells to contain world-frame points."""
        points = tuple(points)
        if not points:
            return self
        margin_cells = ceil(max(0.0, margin) / self.resolution)
        cells = [self.world_to_cell(x, y) for x, y in points]
        min_x = min(x for x, _ in cells) - margin_cells
        max_x = max(x for x, _ in cells) + margin_cells
        min_y = min(y for _, y in cells) - margin_cells
        max_y = max(y for _, y in cells) + margin_cells
        left, bottom = max(0, -min_x), max(0, -min_y)
        right = max(0, max_x - self.width + 1)
        top = max(0, max_y - self.height + 1)
        if not any((left, right, bottom, top)):
            return self
        output = OccupancyGrid(
            self.width + left + right,
            self.height + bottom + top,
            self.resolution,
            self.origin_x - left * self.resolution,
            self.origin_y - bottom * self.resolution,
            [UNKNOWN] * ((self.width + left + right) * (self.height + bottom + top)),
            self.revision,
        )
        for y in range(self.height):
            destination = output.index(left, y + bottom)
            source = self.index(0, y)
            output.data[destination:destination + self.width] = self.data[source:source + self.width]
        return output

    def disk_is_free(
        self, wx: float, wy: float, radius: float, unknown_is_occupied: bool = False
    ) -> bool:
        cx, cy = self.world_to_cell(wx, wy)
        cells = int(radius / self.resolution) + 1
        for y in range(cy - cells, cy + cells + 1):
            for x in range(cx - cells, cx + cells + 1):
                px, py = self.cell_center(x, y)
                if (px - wx) ** 2 + (py - wy) ** 2 <= radius ** 2:
                    value = self.value(x, y)
                    if value >= OCCUPIED or (unknown_is_occupied and value == UNKNOWN):
                        return False
        return True

    def footprint_is_free(
        self, wx: float, wy: float, yaw: float,
        footprint: Iterable[tuple[float, float]],
        unknown_is_occupied: bool = False,
    ) -> bool:
        c, s = cos(yaw), sin(yaw)
        polygon = tuple((wx + c * x - s * y, wy + s * x + c * y) for x, y in footprint)
        min_x = min(x for x, _ in polygon)
        max_x = max(x for x, _ in polygon)
        min_y = min(y for _, y in polygon)
        max_y = max(y for _, y in polygon)
        cmin_x, cmin_y = self.world_to_cell(min_x, min_y)
        cmax_x, cmax_y = self.world_to_cell(max_x, max_y)
        for gy in range(cmin_y - 1, cmax_y + 2):
            for gx in range(cmin_x - 1, cmax_x + 2):
                value = self.value(gx, gy)
                if value >= OCCUPIED or (unknown_is_occupied and value == UNKNOWN):
                    px, py = self.cell_center(gx, gy)
                    if _polygon_intersects_cell(polygon, px, py, self.resolution):
                        return False
        return True

    def footprint_unknown_fraction(
        self, wx: float, wy: float, yaw: float,
        footprint: Iterable[tuple[float, float]],
    ) -> float:
        c, s = cos(yaw), sin(yaw)
        polygon = tuple((wx + c * x - s * y, wy + s * x + c * y) for x, y in footprint)
        min_x = min(x for x, _ in polygon)
        max_x = max(x for x, _ in polygon)
        min_y = min(y for _, y in polygon)
        max_y = max(y for _, y in polygon)
        cmin_x, cmin_y = self.world_to_cell(min_x, min_y)
        cmax_x, cmax_y = self.world_to_cell(max_x, max_y)
        sampled = unknown = 0
        for gy in range(cmin_y - 1, cmax_y + 2):
            for gx in range(cmin_x - 1, cmax_x + 2):
                px, py = self.cell_center(gx, gy)
                if _point_in_polygon(px, py, polygon):
                    sampled += 1
                    unknown += self.value(gx, gy) == UNKNOWN
        return unknown / sampled if sampled else 0.0


def _point_in_polygon(x: float, y: float, polygon: tuple[tuple[float, float], ...]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        intersects = ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1
        )
        if intersects:
            inside = not inside
        previous = current
    return inside


def _polygon_intersects_cell(
    polygon: tuple[tuple[float, float], ...],
    center_x: float,
    center_y: float,
    resolution: float,
) -> bool:
    half = resolution / 2.0
    return polygon_intersects_rectangle(
        polygon, center_x - half, center_y - half, center_x + half, center_y + half)


def polygon_intersects_rectangle(
    polygon: tuple[tuple[float, float], ...],
    left: float, bottom: float, right: float, top: float,
) -> bool:
    corners = ((left, bottom), (right, bottom), (right, top), (left, top))
    if any(_point_in_polygon(x, y, polygon) for x, y in corners):
        return True
    if any(left <= x <= right and bottom <= y <= top for x, y in polygon):
        return True
    cell_edges = tuple(zip(corners, corners[1:] + corners[:1]))
    polygon_edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
    return any(_segments_intersect(*first, *second)
               for first in polygon_edges for second in cell_edges)


def _segments_intersect(a, b, c, d) -> bool:
    def orientation(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_segment(p, q, r):
        return (min(p[0], r[0]) - 1e-12 <= q[0] <= max(p[0], r[0]) + 1e-12
                and min(p[1], r[1]) - 1e-12 <= q[1] <= max(p[1], r[1]) + 1e-12)

    o1, o2 = orientation(a, b, c), orientation(a, b, d)
    o3, o4 = orientation(c, d, a), orientation(c, d, b)
    if ((o1 > 0) != (o2 > 0)) and ((o3 > 0) != (o4 > 0)):
        return True
    return ((abs(o1) <= 1e-12 and on_segment(a, c, b))
            or (abs(o2) <= 1e-12 and on_segment(a, d, b))
            or (abs(o3) <= 1e-12 and on_segment(c, a, d))
            or (abs(o4) <= 1e-12 and on_segment(c, b, d)))
