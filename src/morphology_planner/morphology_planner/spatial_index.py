"""A bucket grid over boxes' ground-plane extents, for exact pre-filtering.

Sight-line and swept-volume tests scan every obstacle box. A generated layout
has a dozen boxes; a scanned building has thousands, and the scans dominated
planning time (2026-11-15). The index returns a superset of the boxes whose
ground-plane extent meets a query rectangle, so a test that can only succeed
on such boxes gives the same answer run on the candidates alone.
"""

from __future__ import annotations

from math import floor
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # transition_validation imports this module
    from .transition_validation import Box3

# Below this many boxes a plain scan is as fast and needs no index.
INDEX_MIN_BOXES = 64
BUCKET_M = 0.5
# Queries and boxes are widened by this much, so a box that only touches a
# query rectangle under rounding is still a candidate.
MARGIN_M = 1e-6


class BoxIndex:
    def __init__(self, boxes: tuple[Box3, ...], bucket: float = BUCKET_M) -> None:
        self.boxes = boxes
        self.bucket = bucket
        self.buckets: dict[tuple[int, int], list[int]] = {}
        for index, box in enumerate(boxes):
            for key in self._keys(box.center[0] - box.size[0] / 2.0,
                                  box.center[1] - box.size[1] / 2.0,
                                  box.center[0] + box.size[0] / 2.0,
                                  box.center[1] + box.size[1] / 2.0):
                self.buckets.setdefault(key, []).append(index)

    def _keys(self, low_x, low_y, high_x, high_y):
        x0 = floor((low_x - MARGIN_M) / self.bucket)
        x1 = floor((high_x + MARGIN_M) / self.bucket)
        y0 = floor((low_y - MARGIN_M) / self.bucket)
        y1 = floor((high_y + MARGIN_M) / self.bucket)
        return ((i, j) for i in range(x0, x1 + 1) for j in range(y0, y1 + 1))

    def candidates(self, low_x: float, low_y: float, high_x: float, high_y: float
                   ) -> list[Box3]:
        """Boxes that may meet the rectangle, in their original order."""
        found: set[int] = set()
        for key in self._keys(low_x, low_y, high_x, high_y):
            found.update(self.buckets.get(key, ()))
        return [self.boxes[index] for index in sorted(found)]


_CACHE: dict[int, tuple[tuple[Box3, ...], BoxIndex]] = {}
_CACHE_SIZE = 16


def cached_index(boxes: tuple[Box3, ...]) -> BoxIndex | None:
    """An index for a long-lived obstacle tuple, or None for a short one.

    Keyed by identity; the entry holds the tuple itself so its id cannot be
    reused while cached.
    """
    if len(boxes) < INDEX_MIN_BOXES:
        return None
    entry = _CACHE.get(id(boxes))
    if entry is not None and entry[0] is boxes:
        return entry[1]
    if len(_CACHE) >= _CACHE_SIZE:
        _CACHE.pop(next(iter(_CACHE)))
    index = BoxIndex(boxes)
    _CACHE[id(boxes)] = (boxes, index)
    return index
