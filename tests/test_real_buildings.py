"""Scan conversion and lattice reachability for the real-buildings study."""

import json
import struct
from pathlib import Path
from random import Random

import numpy as np

from morphology_planner import HybridState, load_catalog, make_method_planner
from morphology_planner.grid import FREE, OCCUPIED, OccupancyGrid
from modular_robot_benchmarks.building_mesh import (
    BOX_BAND_M, RESOLUTION, VOXEL_Z, _merge_voxels, read_glb_triangles,
)
from modular_robot_benchmarks.lattice_reach import build_model

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
