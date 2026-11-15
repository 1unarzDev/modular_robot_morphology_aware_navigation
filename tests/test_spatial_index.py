"""The spatial index is a pure speed-up: every answer matches a full scan."""

from random import Random

import morphology_planner.observability as observability
import morphology_planner.transition_validation as validation
from morphology_planner.observability import FiducialStation, station_observes
from morphology_planner.spatial_index import BoxIndex
from morphology_planner.transition_validation import Box3, TransitionTrajectoryValidator


def _boxes(rng, count, snap=None):
    boxes = []
    for index in range(count):
        def value(low, high):
            v = rng.uniform(low, high)
            return round(v / snap) * snap if snap else v
        size = (value(0.05, 1.5) or 0.1, value(0.05, 1.5) or 0.1, value(0.05, 1.0) or 0.1)
        boxes.append(Box3(f"b{index}", (value(0, 10), value(0, 10), value(0, 2)), size))
    return tuple(boxes)


def test_candidates_cover_every_box_meeting_the_query():
    rng = Random(3)
    boxes = _boxes(rng, 300)
    index = BoxIndex(boxes)
    for _ in range(200):
        x0, y0 = rng.uniform(-1, 11), rng.uniform(-1, 11)
        x1, y1 = x0 + rng.uniform(0, 3), y0 + rng.uniform(0, 3)
        found = set(b.name for b in index.candidates(x0, y0, x1, y1))
        for box in boxes:
            meets = not (box.center[0] + box.size[0] / 2 < x0 or box.center[0] - box.size[0] / 2 > x1
                         or box.center[1] + box.size[1] / 2 < y0 or box.center[1] - box.size[1] / 2 > y1)
            if meets:
                assert box.name in found


def test_sight_lines_match_a_full_scan(monkeypatch):
    # Snapped coordinates put segment ends and box faces on shared values, so
    # ties are exercised as well as the general case.
    for snap in (None, 0.1):
        rng = Random(5)
        boxes = _boxes(rng, 200, snap)
        stations = [FiducialStation("s", (rng.uniform(0, 10), rng.uniform(0, 10), 1.8))
                    for _ in range(10)]
        points = [(rng.uniform(0, 10), rng.uniform(0, 10), rng.uniform(0, 0.3))
                  for _ in range(150)]
        if snap:
            points = [tuple(round(v / snap) * snap for v in p) for p in points]
        indexed = [station_observes(s, p, boxes) for s in stations for p in points]
        monkeypatch.setattr(observability, "cached_index", lambda boxes: None)
        scanned = [station_observes(s, p, boxes) for s in stations for p in points]
        monkeypatch.undo()
        assert indexed == scanned
        assert 0 < sum(indexed) < len(indexed)


def test_swept_environment_matches_a_full_scan(monkeypatch):
    for snap in (None, 0.1):
        rng = Random(7)
        environment = _boxes(rng, 150, snap)
        swept = tuple(Box3(f"pod_{i}/part", (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.1),
                           (0.2, 0.2, 0.12)) for i in range(40))
        offsets = [(rng.uniform(0, 10), rng.uniform(0, 10)) for _ in range(60)]
        if snap:
            offsets = [(round(x / snap) * snap, round(y / snap) * snap) for x, y in offsets]
        check = TransitionTrajectoryValidator.validate_swept_environment
        indexed = [check(swept, environment, x, y).reasons for x, y in offsets]
        monkeypatch.setattr(validation, "SWEPT_INDEX_MIN_BOXES", 10 ** 9)
        scanned = [check(swept, environment, x, y).reasons for x, y in offsets]
        monkeypatch.undo()
        assert indexed == scanned
        assert any(indexed) and not all(indexed)
