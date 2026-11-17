"""Scan conversion and lattice reachability for the real-buildings study."""

import json
import struct
from pathlib import Path
from random import Random

import numpy as np
import pytest

from morphology_planner import HybridState, load_catalog, make_method_planner
from morphology_planner.grid import FREE, OCCUPIED, OccupancyGrid
from modular_robot_benchmarks import real_buildings
from modular_robot_benchmarks.building_mesh import (
    BOX_BAND_M, RESOLUTION, VOXEL_Z, Level, _merge_voxels, read_glb_triangles,
)
from modular_robot_benchmarks.lattice_reach import build_model
from modular_robot_benchmarks.real_buildings import draw_missions

ROOT = Path(__file__).parents[1]
CATALOG = ROOT / "src/modular_robot_description/config/morphologies.yaml"


def test_merged_boxes_cover_exactly_the_voxels():
    rng = np.random.default_rng(1)
    voxels = rng.random((12, 15, 10)) < 0.35
    voxels[2:9, 3:4, :] = True  # a wall
    boxes = _merge_voxels(voxels, "t")
    covered = np.zeros(voxels.shape, int)
    for box in boxes:
        x0 = round((box.center[0] - box.size[0] / 2) / RESOLUTION)
        x1 = round((box.center[0] + box.size[0] / 2) / RESOLUTION)
        y0 = round((box.center[1] - box.size[1] / 2) / RESOLUTION)
        y1 = round((box.center[1] + box.size[1] / 2) / RESOLUTION)
        z0 = round((box.center[2] - box.size[2] / 2 - BOX_BAND_M[0]) / VOXEL_Z)
        z1 = round((box.center[2] + box.size[2] / 2 - BOX_BAND_M[0]) / VOXEL_Z)
        covered[y0:y1, x0:x1, z0:z1] += 1
    assert (covered == voxels).all()
    wall = np.zeros_like(voxels)
    wall[2:9, 3:4, :] = True
    assert len(_merge_voxels(wall, "w")) == 1


def test_lattice_masks_agree_with_the_planner_and_never_admit_more():
    catalog = load_catalog(str(CATALOG)).supported_experiment_subset()
    rng = Random(2)
    width, height = 40, 30
    occupancy = np.zeros((height, width), bool)
    occupancy[0, :] = occupancy[-1, :] = occupancy[:, 0] = occupancy[:, -1] = True
    occupancy[:, 20] = True
    occupancy[12:19, 20] = False  # a 0.7 m door
    for _ in range(25):
        occupancy[rng.randrange(height), rng.randrange(width)] = True
    grid = OccupancyGrid(width, height, RESOLUTION)
    grid.data = [OCCUPIED if v else FREE for v in occupancy.ravel()]
    model = build_model(occupancy, {m: np.zeros_like(occupancy) for m in catalog.morphologies},
                        catalog, 8, RESOLUTION)
    planner = make_method_planner("geometry_coupled", catalog, grid, heading_bins=8).planner
    checked = differ = 0
    for m in catalog.morphologies:
        for h in range(8):
            for y in range(height):
                for x in range(width):
                    state = HybridState(x, y, h, m)
                    free = planner._state_is_free(state)
                    assert free == model.state_free[m, h][y, x]
                    if not free:
                        continue
                    for move in model.moves[m, h]:
                        dx, dy, dh = move
                        target = HybridState(x + dx, y + dy, (h + dh) % 8, m)
                        exact = planner._traversal_is_free(state, target)
                        mask = bool(model.move_free[m, h, move][y, x])
                        checked += 1
                        if exact != mask:
                            differ += 1
                            assert exact and not mask  # the mask is only ever stricter
    assert checked > 1000 and differ <= checked // 100


def test_glb_reader_returns_indexed_triangles(tmp_path):
    positions = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float32)
    indices = np.array([0, 1, 2, 0, 2, 3], np.uint16)
    binary = positions.tobytes() + indices.tobytes()
    document = {
        "asset": {"version": "2.0"},
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 48},
                        {"buffer": 0, "byteOffset": 48, "byteLength": 12}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3"},
                      {"bufferView": 1, "componentType": 5123, "count": 6, "type": "SCALAR"}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]},
                   {"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
        "nodes": [{"mesh": 0, "name": "chunk0_objectunknown"},
                  {"mesh": 1, "name": "chunk1_objectceiling"}],
    }
    text = json.dumps(document).encode()
    text += b" " * (-len(text) % 4)
    binary += b"\0" * (-len(binary) % 4)
    body = (struct.pack("<II", len(text), 0x4E4F534A) + text
            + struct.pack("<II", len(binary), 0x004E4942) + binary)
    path = tmp_path / "t.glb"
    path.write_bytes(b"glTF" + struct.pack("<II", 2, 12 + len(body)) + body)
    triangles = read_glb_triangles(path)
    assert triangles.shape == (2, 3, 3)  # the ceiling node is dropped
    assert triangles[1].tolist() == [[0, 0, 0], [0, 1, 0], [0, 0, 1]]


# --- A building interrupted mid-way resumes to what an uninterrupted run returns ---
# The slowest sample buildings take hours and the host is shut down nightly, so
# the draw loop checkpoints before every draw. These tests are what licenses a
# building finished across several sessions to stand beside the others.

class _Killed(BaseException):
    """Stands in for the process being killed: nothing in the study catches it."""


class _KilledAfter(Random):
    """A generator that returns `calls` draws and is killed on the next."""

    calls = 0

    def choice(self, sequence):
        self.calls -= 1
        if self.calls < 0:
            raise _Killed
        return super().choice(sequence)


def _killed_after(seed, calls):
    rng = _KilledAfter(seed)
    rng.calls = calls
    return rng


def _two_room_level():
    """Two rooms joined by a 0.70 m door, which only the narrow body passes."""
    width, height = 78, 36
    occupancy = np.zeros((height, width), bool)
    occupancy[0, :] = occupancy[-1, :] = occupancy[:, 0] = occupancy[:, -1] = True
    occupancy[:, 40] = True
    occupancy[15:22, 40] = False
    levels = round((BOX_BAND_M[1] - BOX_BAND_M[0]) / VOXEL_Z)
    grid = OccupancyGrid(width, height, RESOLUTION)
    grid.data = [OCCUPIED if value else FREE for value in occupancy.ravel()]
    return Level(0, 0.0, grid, ~occupancy, occupancy,
                 np.repeat(occupancy[:, :, None], levels, axis=2), (0.0, 0.0))


def _accept(level, mission):
    """Deterministic stand-in for planning: rejects some draws, attaches a result."""
    mission["result"] = {"route": mission["route_m"] / 3.0, "cells": (mission["start"],),
                         "methods": {"geometry_coupled": {"planned": True, "sites": []}}}
    return (mission["start"][0] + 2 * mission["goal"][1] + mission["station_cell"][0]) % 3 != 0


def _same(first, second):
    return json.dumps(first, sort_keys=True) == json.dumps(second, sort_keys=True)


def test_an_interrupted_draw_resumes_to_the_uninterrupted_result(tmp_path):
    catalog = load_catalog(str(CATALOG)).supported_experiment_subset()
    level, seed = _two_room_level(), 11
    counter = _killed_after(seed, 10 ** 9)
    reference = draw_missions([level], catalog, counter, _accept)
    draws = 10 ** 9 - counter.calls
    missions, stats = reference
    # The world must exercise every branch a real building does.
    assert len(missions) == 3 and stats["geometry_unplanned"] >= 1 and draws >= 12
    assert _same(reference, draw_missions([level], catalog, Random(seed), _accept,
                                          tmp_path / "never_killed.json", "t"))
    for kill in range(draws):           # killed at every draw of the generator in turn
        path = tmp_path / f"kill_{kill}.json"
        with pytest.raises(_Killed):
            draw_missions([level], catalog, _killed_after(seed, kill), _accept, path, "t")
        assert path.exists()
        resumed = draw_missions([level], catalog, Random(seed), _accept, path, "t")
        assert _same(resumed, reference), kill
    # Killed again and again: each session gets through at most one more draw.
    path, sessions = tmp_path / "many.json", 0
    while True:
        sessions += 1
        try:
            resumed = draw_missions([level], catalog, _killed_after(seed, 5), _accept, path, "t")
            break
        except _Killed:
            assert sessions < 100
    assert sessions > 3 and _same(resumed, reference)


def test_a_kill_while_a_mission_is_being_planned_loses_only_that_draw(tmp_path):
    catalog = load_catalog(str(CATALOG)).supported_experiment_subset()
    level, seed = _two_room_level(), 11
    reference = draw_missions([level], catalog, Random(seed), _accept)
    planned = reference[1]["attempts"] - reference[1]["no_goal"] - reference[1]["no_station"]
    for kill in range(planned):
        calls = []

        def killed_while_planning(level, mission):
            calls.append(1)
            if len(calls) > kill:
                raise _Killed
            return _accept(level, mission)

        path = tmp_path / f"plan_{kill}.json"
        with pytest.raises(_Killed):
            draw_missions([level], catalog, Random(seed), killed_while_planning, path, "t")
        assert _same(draw_missions([level], catalog, Random(seed), _accept, path, "t"),
                     reference), kill


def test_a_draw_state_saved_for_another_building_or_seed_is_ignored(tmp_path):
    catalog = load_catalog(str(CATALOG)).supported_experiment_subset()
    level, path = _two_room_level(), tmp_path / "state.json"
    with pytest.raises(_Killed):
        draw_missions([level], catalog, _killed_after(11, 7), _accept, path, "11:a")
    other = draw_missions([level], catalog, Random(12), _accept)
    assert not _same(other, draw_missions([level], catalog, Random(11), _accept))
    assert _same(other, draw_missions([level], catalog, Random(12), _accept, path, "12:a"))
    path.write_text("{ truncated", encoding="utf-8")
    assert _same(other, draw_missions([level], catalog, Random(12), _accept, path, "12:a"))


def _write_two_room_glb(path):
    """The two-room building as a scan: a floor and zero-thickness walls 2.2 m high."""
    def wall(x0, y0, x1, y1):
        a, b, c, d = (x0, y0, 0.0), (x1, y1, 0.0), (x1, y1, 2.2), (x0, y0, 2.2)
        return [a, b, c, a, c, d]
    floor = [(0, 0, 0), (7.8, 0, 0), (7.8, 3.6, 0), (0, 0, 0), (7.8, 3.6, 0), (0, 3.6, 0)]
    # Walls run through cell centres and the door's jambs sit mid-cell, so no
    # sample lands on a cell boundary: seven free cells, 0.70 m, in the door.
    vertices = floor + wall(0.05, 0, 0.05, 3.6) + wall(7.75, 0, 7.75, 3.6) \
        + wall(0, 0.05, 7.8, 0.05) + wall(0, 3.55, 7.8, 3.55) \
        + wall(4.05, 0, 4.05, 1.45) + wall(4.05, 2.25, 4.05, 3.6)
    binary = np.array(vertices, np.float32).tobytes()
    document = {
        "asset": {"version": "2.0"},
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(binary)}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": len(vertices),
                       "type": "VEC3"}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}],
        "nodes": [{"mesh": 0, "name": "chunk0_objectunknown"}],
    }
    text = json.dumps(document).encode()
    text += b" " * (-len(text) % 4)
    body = (struct.pack("<II", len(text), 0x4E4F534A) + text
            + struct.pack("<II", len(binary), 0x004E4942) + binary)
    path.write_bytes(b"glTF" + struct.pack("<II", 2, 12 + len(body)) + body)


def _planned_like_the_study(level, mission, catalog):
    """A result in the shape `evaluate_mission` returns, decided by the draw alone.

    The planner takes about 20 s a mission in this building; what `main` must
    get right is the bookkeeping around it. The planner itself is checked across
    real kills by `tools/check_real_buildings_resume.py`.
    """
    planned = (mission["start"][0] + 2 * mission["goal"][1]) % 3 != 0
    site = {"transition": "compact_to_narrow", "site": list(mission["start"]),
            "full_test_feasible": mission["route_steps"] % 2 == 0,
            "failed_stages": ["observability"] * (mission["route_steps"] % 2)}
    methods = {"geometry_coupled": {"planned": planned, "sites": [site]}}
    if planned:
        methods["feasibility_coupled"] = {"planned": True, "sites": [site]}
        methods["sensing_feasibility_coupled"] = {"planned": False, "error": "NoPathError"}
    return {"sight_boxes": 6, "unmapped_boxes": 0, "methods": methods}


def test_the_study_killed_inside_a_building_writes_the_report_an_unbroken_run_writes(
        tmp_path, monkeypatch):
    """End to end through `main`: the scan, the draws, both checkpoints, the report."""
    glb = tmp_path / "scan" / "TwoRooms.glb"
    glb.parent.mkdir()
    _write_two_room_glb(glb)
    listing = tmp_path / "sample.txt"
    listing.write_text(str(glb), encoding="utf-8")

    def run(output, evaluate):
        monkeypatch.setattr(real_buildings, "evaluate_mission", evaluate)
        monkeypatch.setattr("sys.argv", ["real_buildings", "--buildings", str(listing),
                                         "--master-seed", "3", "--output", str(output),
                                         "--catalog", str(CATALOG)])
        real_buildings.main()

    unbroken = tmp_path / "unbroken" / "report.json"
    run(unbroken, _planned_like_the_study)
    entry = json.loads(unbroken.read_text(encoding="utf-8"))["results"][0]
    assert "error" not in entry and len(entry["missions"]) == 3, entry
    assert entry["draw"]["geometry_unplanned"] >= 1
    assert not (unbroken.parent / "report.json.partial.draws").exists()

    draw = entry["draw"]
    for kill in range(1, draw["attempts"] - draw["no_goal"] - draw["no_station"] + 1):
        calls = []

        def killed(level, mission, catalog):
            calls.append(1)
            if len(calls) == kill:
                raise KeyboardInterrupt     # not an Exception: no error entry is written
            return _planned_like_the_study(level, mission, catalog)

        broken = tmp_path / f"broken_{kill}" / "report.json"
        with pytest.raises(KeyboardInterrupt):
            run(broken, killed)
        assert not broken.exists()
        state = broken.parent / "report.json.partial.draws" / "TwoRooms.json"
        # The state on disk is the one before the draw that was being planned.
        assert json.loads(state.read_text(encoding="utf-8"))["stats"]["attempts"] >= kill - 1
        run(broken, _planned_like_the_study)
        assert broken.read_bytes() == unbroken.read_bytes(), kill
        assert not state.parent.exists()
