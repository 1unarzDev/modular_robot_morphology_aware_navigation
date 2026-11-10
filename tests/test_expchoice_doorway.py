"""ExperimentChoice offset-doorway family: geometry, design, configuration, outcome.

Host tests only; nothing here starts a simulator or imports rclpy. The
manifests under `studies/expchoice_doorway_v0/courses` and
`tests/data/expchoice` were written by `crane_ml/Tools/ExperimentChoice/
course.py` and are read here as the other repository left them.
"""
from dataclasses import asdict, fields, replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

import pytest
import yaml

from modular_robot_benchmarks.design import (
    DOORWAY_METHODS, EXPCHOICE_DOORWAY_DESIGN_KIND, DoorwayTrialSpec, StudyDesign,
    TrialSpec, generate_design, trial_spec_from_dict,
)
from modular_robot_benchmarks.doorway_batch import prepare_trial
from modular_robot_benchmarks.doorway_design import (
    DEFAULT_METHODS, NAV2_CONFIG_DIR, NAV2_CONFIGS, build_design, load_courses,
    nav2_config_for,
)
from modular_robot_benchmarks.doorway_outcome import (
    STATUS_ABORTED, STATUS_CANCELED, STATUS_SUCCEEDED, doorway_outcome, footprint_edges,
    goal_truth, motion_summary, nav2_terminal,
)
from modular_robot_benchmarks.doorway_scenarios import (
    MAP_RESOLUTION_M, OBSTACLE_TILE_MM, course_from_manifest, course_from_spec,
    export_doorway_map, measure_map, obstacle_boxes, rasterize, wall_boxes_m,
)
from modular_robot_benchmarks.evaluator_metrics import clearance_metrics
from modular_robot_benchmarks.mission_batch import _parser, pending_trials
from modular_robot_benchmarks.records import TrialManifest, TrialRecord, TrialStore
from modular_robot_benchmarks.sdf_export import (
    export_confirmatory_sdf, export_doorway_sdf, physical_disturbance,
)

ROOT = Path(__file__).parents[1]
CATALOG = ROOT / "src/modular_robot_description/config/morphologies.yaml"
STUDY = ROOT / "studies/expchoice_doorway_v0"
BATCH_MANIFESTS = sorted((STUDY / "courses").glob("course_*.json"))
EXTRA_MANIFESTS = sorted((ROOT / "tests/data/expchoice").glob("course_*.json"))
ALL_MANIFESTS = BATCH_MANIFESTS + EXTRA_MANIFESTS
CANARY = "8d876a63964f9337"


def _manifest(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _ids(paths):
    return [path.stem.removeprefix("course_") for path in paths]


# ------------------------------------------------------------------ course

def test_fixture_sets_are_present():
    assert len(BATCH_MANIFESTS) == 10 and len(EXTRA_MANIFESTS) == 6


@pytest.mark.parametrize("path", ALL_MANIFESTS, ids=_ids(ALL_MANIFESTS))
def test_course_reproduces_the_manifest_identity_and_walls(path):
    manifest = _manifest(path)
    course = course_from_manifest(manifest)
    assert course.course_id == manifest["course_id"] == path.stem.removeprefix("course_")
    assert course.params() == manifest["parameters"]
    derived = {box.name: (box.xmin, box.xmax, box.ymin, box.ymax) for box in course.walls()}
    for wall in manifest["walls"]:
        assert derived[wall["name"]] == tuple(
            round(wall[key] * 1000) for key in ("xmin", "xmax", "ymin", "ymax"))
    assert course.start == (manifest["start"]["x"], manifest["start"]["y"],
                            manifest["start"]["theta"])
    assert course.goal == (manifest["goal"]["x"], manifest["goal"]["y"])


def test_a_manifest_that_disagrees_with_itself_is_refused():
    manifest = _manifest(BATCH_MANIFESTS[0])
    moved = json.loads(json.dumps(manifest))
    moved["parameters"]["d_cm"] += 1                     # id no longer matches
    with pytest.raises(ValueError, match="course_id"):
        course_from_manifest(moved)
    walls = json.loads(json.dumps(manifest))
    walls["walls"][-1]["ymin"] += 0.01                   # one jamb moved a centimetre
    with pytest.raises(ValueError, match="walls disagree"):
        course_from_manifest(walls)
    other = json.loads(json.dumps(manifest))
    other["family"] = "something_else"
    with pytest.raises(ValueError, match="offset_doorway"):
        course_from_manifest(other)


# --------------------------------------------------------------------- SDF

def _sdf_walls(world_path):
    walls = {}
    for model in ElementTree.parse(world_path).getroot().find("world").findall("model"):
        if model.get("name") == "ground":
            continue
        x, y, z = (float(v) for v in model.find("pose").text.split()[:3])
        sx, sy, sz = (float(v) for v in model.find("link/collision/geometry/box/size").text.split())
        assert model.find("link/visual/geometry/box/size").text == \
            model.find("link/collision/geometry/box/size").text
        assert model.find("static").text == "true" and z == sz / 2.0
        walls[model.get("name")] = (x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2)
    return walls


@pytest.mark.parametrize("path", ALL_MANIFESTS, ids=_ids(ALL_MANIFESTS))
def test_world_file_agrees_with_the_manifest_to_the_millimetre(path, tmp_path):
    manifest = _manifest(path)
    course = course_from_manifest(manifest)
    world, scenario = export_doorway_sdf(course, tmp_path / "world.sdf", CATALOG, 11)
    walls = _sdf_walls(world)
    assert set(walls) == {wall["name"] for wall in manifest["walls"]}
    for wall in manifest["walls"]:
        assert walls[wall["name"]] == pytest.approx(
            (wall["xmin"], wall["xmax"], wall["ymin"], wall["ymax"]), abs=1e-9)
    truth = manifest["parameters_m"]
    # Door width, offset, bay depth and wall thickness read off the world's boxes.
    if "door_wall_neg" in walls and "door_wall_pos" in walls:
        low, high = walls["door_wall_neg"][3], walls["door_wall_pos"][2]
        assert high - low == pytest.approx(truth["w"], abs=1e-9)
        assert (high + low) / 2 == pytest.approx(truth["e"], abs=1e-9)
    door = walls.get("door_wall_pos") or walls["door_wall_neg"]
    assert door[0] - walls["back_wall"][1] == pytest.approx(truth["d"], abs=1e-9)
    assert door[1] - door[0] == pytest.approx(truth["t"], abs=1e-9)
    assert walls["side_wall_pos"][2] - walls["side_wall_neg"][3] == pytest.approx(
        truth["B"], abs=1e-9)
    # The robot spawns at the manifest's start plus the seeded disturbance.
    include = ElementTree.parse(world).getroot().find("world/include")
    x, y, z, _, _, yaw = (float(v) for v in include.find("pose").text.split())
    disturbance = physical_disturbance(11)
    assert (x, y, yaw) == pytest.approx((
        manifest["start"]["x"] + disturbance["initial_dx_m"],
        manifest["start"]["y"] + disturbance["initial_dy_m"],
        manifest["start"]["theta"] + disturbance["initial_yaw_rad"]))
    stated = json.loads(scenario.read_text())
    assert stated["course_id"] == manifest["course_id"]
    assert stated["transition_obstacles"] == [] and stated["fiducial_stations"] == []
    assert stated["physical_disturbance"] == disturbance


def test_doorway_world_shares_the_confirmatory_world_skeleton(tmp_path):
    """Same physics, systems, topology plugin, light and ground; only walls and spawn differ."""
    course = course_from_manifest(_manifest(BATCH_MANIFESTS[0]))
    ours, _ = export_doorway_sdf(course, tmp_path / "a/world.sdf", CATALOG, 5)
    theirs, _ = export_confirmatory_sdf(
        "reconfiguration_workspace", 0, 3, tmp_path / "b/world.sdf", CATALOG, 5, generator=3)

    def skeleton(path):
        world = ElementTree.parse(path).getroot().find("world")
        kept = [child for child in world
                if child.tag in ("physics", "plugin", "light")
                or (child.tag == "model" and child.get("name") == "ground")]
        return [ElementTree.tostring(child) for child in kept]

    assert skeleton(ours) == skeleton(theirs)
    assert ElementTree.parse(ours).getroot().find("world/include/uri").text == \
        "model://modular_robot"


# --------------------------------------------------------------------- map

def _read_map(yaml_path):
    """The map as map_server would load it: (rows from the lowest y, origin, resolution)."""
    meta = yaml.safe_load(yaml_path.read_text())
    raw = (yaml_path.parent / meta["image"]).read_bytes()
    magic, size, maxval, body = raw.split(b"\n", 3)
    width, height = (int(v) for v in size.split())
    assert magic == b"P5" and maxval == b"255" and len(body) == width * height
    assert meta["mode"] == "trinary" and meta["negate"] == 0
    assert set(body) <= {0, 254}
    rows = [body[index * width:(index + 1) * width] for index in range(height)]
    occupied = [[pixel == 0 for pixel in row] for row in reversed(rows)]
    return occupied, meta["origin"], meta["resolution"]


@pytest.mark.parametrize("path", ALL_MANIFESTS, ids=_ids(ALL_MANIFESTS))
def test_map_agrees_with_the_manifest_to_the_centimetre(path, tmp_path):
    manifest = _manifest(path)
    course = course_from_manifest(manifest)
    map_yaml, _ = export_doorway_map(course, tmp_path / "map.yaml")
    occupied, origin, resolution = _read_map(map_yaml)
    raster = rasterize(course)
    assert resolution == MAP_RESOLUTION_M == 0.01
    assert origin == [raster.origin[0], raster.origin[1], 0.0]
    # Written rows are the raster, bottom row first once map_server flips them.
    assert occupied == [[bool(cell) for cell in row] for row in raster.rows]
    # Origin on the 5 cm lattice through the manifest origin, size in 5-cell blocks.
    assert raster.origin_x_mm % 50 == 0 and raster.origin_y_mm % 50 == 0
    assert raster.width % 5 == 0 and raster.height % 5 == 0
    truth, measured = manifest["parameters_m"], measure_map(course, raster)
    exact = course.map_exactness()
    assert exact["exact_at_map_resolution"] == (course.w_cm % 2 == 0 and course.B_cm % 2 == 0)
    assert measured["bay_depth_m"] == pytest.approx(truth["d"], abs=1e-9)
    assert measured["back_wall_face_m"] == pytest.approx(0.0, abs=1e-9)
    assert measured["door_wall_thickness_m"] == pytest.approx(truth["t"], abs=1e-9)
    assert measured["door_centre_m"] == pytest.approx(truth["e"], abs=1e-9)
    if exact["exact_at_map_resolution"]:
        assert measured["door_width_m"] == pytest.approx(truth["w"], abs=1e-9)
    else:
        # Odd w: both jambs sit on the half centimetre and each closes half a cell.
        assert measured["door_width_m"] == pytest.approx(truth["w"] - 0.01, abs=1e-9)
        assert exact["off_lattice_edges"]


EVEN_OFF_LATTICE = [path for path in EXTRA_MANIFESTS if "abc7e3fff672273c" in path.name]


@pytest.mark.parametrize("path", [BATCH_MANIFESTS[0], *EVEN_OFF_LATTICE],
                         ids=_ids([BATCH_MANIFESTS[0], *EVEN_OFF_LATTICE]))
def test_exact_map_is_the_world_cell_for_cell(path):
    """For an exact course a cell is occupied iff its centre lies inside a world wall."""
    course = course_from_manifest(_manifest(path))
    assert course.map_exactness()["exact_at_map_resolution"]
    raster = rasterize(course)
    walls = [(box.xmin, box.xmax, box.ymin, box.ymax) for box in course.walls()]
    for iy in range(raster.height):
        y = raster.origin_y_mm + iy * 10 + 5
        spans = [(xmin, xmax) for xmin, xmax, ymin, ymax in walls if ymin < y < ymax]
        expected = bytearray(raster.width)
        for xmin, xmax in spans:
            first = (xmin - raster.origin_x_mm) // 10
            last = (xmax - raster.origin_x_mm) // 10
            expected[first:last] = b"\x01" * (last - first)
        assert raster.rows[iy] == bytes(expected), f"row {iy}"


@pytest.mark.parametrize("path", BATCH_MANIFESTS, ids=_ids(BATCH_MANIFESTS))
def test_first_batch_courses_survive_a_five_fold_downsample(path):
    """Arm C's planner max-pools the costmap 5x (0.05 m); for these courses that is exact."""
    manifest = _manifest(path)
    course = course_from_manifest(manifest)
    assert course.map_exactness() == {
        "exact_at_map_resolution": True, "off_lattice_edges": [], "exact_at_0p05_m": True}
    raster = rasterize(course)
    for iy in range(0, raster.height, 5):
        for ix in range(0, raster.width, 5):
            block = {raster.rows[iy + dy][ix + dx] for dy in range(5) for dx in range(5)}
            assert len(block) == 1, "a 5x5 block straddles a wall edge"


# --------------------------------------------------------------- evaluator

@pytest.mark.parametrize("path", ALL_MANIFESTS[::3], ids=_ids(ALL_MANIFESTS[::3]))
def test_evaluator_tiles_are_exactly_the_walls(path):
    course = course_from_manifest(_manifest(path))
    tiles = obstacle_boxes(course)
    limit = OBSTACLE_TILE_MM / 1000.0 + 1e-9
    assert all(size[0] <= limit and size[1] <= limit and size[2] == 1.0 for _, size in tiles)
    wall_area = sum((box.xmax - box.xmin) * (box.ymax - box.ymin) for box in course.walls())
    assert sum(round(size[0] * 1000) * round(size[1] * 1000) for _, size in tiles) == wall_area
    for centre, size in tiles:
        assert any(box.xmin <= round((centre[0] - size[0] / 2) * 1000)
                   and round((centre[0] + size[0] / 2) * 1000) <= box.xmax
                   and box.ymin <= round((centre[1] - size[1] / 2) * 1000)
                   and round((centre[1] + size[1] / 2) * 1000) <= box.ymax
                   for box in course.walls())


def test_contact_with_a_long_wall_needs_the_tiles():
    """Whole walls would be skipped by the evaluator's 1 m centre-distance filter."""
    course = course_from_manifest(_manifest(BATCH_MANIFESTS[0]))
    side = -course.B_cm / 200.0                      # inner face of the negative side wall
    # Core box (0.26 x 0.20) in the bay, 2 cm into the side wall, then 10 cm clear of
    # it; the 5.9 m side wall's centre is about 1.4 m away along x.
    touching = [{"time_s": 1.0, "module": "core", "x": 0.40, "y": side + 0.08,
                 "z": 0.18, "yaw": 0.0}]
    clear = [{"time_s": 1.0, "module": "core", "x": 0.40, "y": side + 0.20,
              "z": 0.18, "yaw": 0.0}]
    tiled = clearance_metrics(touching, obstacle_boxes(course))
    assert tiled["collision_count"] == 1 and tiled["minimum_clearance_m"] == 0.0
    assert clearance_metrics(clear, obstacle_boxes(course))["minimum_clearance_m"] == \
        pytest.approx(0.10, abs=1e-9)
    whole = [(centre, size) for _, centre, size in wall_boxes_m(course)]
    assert clearance_metrics(touching, whole)["collision_count"] == 0


# ------------------------------------------------------------------ design

def _design():
    return build_design(load_courses(BATCH_MANIFESTS), canary_course_id=CANARY)


def test_frozen_design_is_what_the_builder_produces():
    frozen = StudyDesign.read_frozen(STUDY / "design.json")
    assert frozen == _design()
    assert frozen.design_kind == EXPCHOICE_DOORWAY_DESIGN_KIND
    assert frozen.methods == DEFAULT_METHODS and len(frozen.trials) == 40


def test_design_blocks_are_paired_adjacent_and_canary_first():
    design = _design()
    trials = design.trials
    assert [trial.run_order for trial in trials] == list(range(40))
    assert len({trial.trial_id for trial in trials}) == 40
    for first, second in zip(trials[0::2], trials[1::2]):
        assert (first.course_id, first.replicate) == (second.course_id, second.replicate)
        assert {first.arm, second.arm} == {"A", "B"}
        for seed in ("world_seed", "sensing_seed", "friction_seed", "fault_seed"):
            assert getattr(first, seed) == getattr(second, seed)
    assert {trial.course_id for trial in trials[:2]} == {CANARY}
    # Replicate 0 is complete before replicate 1 starts; arm order is not constant.
    assert [trial.replicate for trial in trials] == [0] * 20 + [1] * 20
    assert {trial.arm for trial in trials[0::2]} == {"A", "B"}
    for trial in trials:
        assert course_from_spec(trial).course_id == trial.course_id
        assert trial.morphology == "compact_diff" and trial.deadline_s == 120.0
        assert (trial.nav2_config, trial.nav2_config_sha256) == nav2_config_for(
            "compact_diff", trial.method, ROOT)
        TrialStore(Path("/nonexistent")).path_for(trial.trial_id)   # a safe record name


def test_design_matches_the_job_queue_adapter_contract():
    """`mrmn_trial.sh` picks the trial whose run_order is the seed and checks the hash."""
    envelope = json.loads((STUDY / "design.json").read_text())
    for seed in (0, 1, 39):
        matches = [trial["trial_id"] for trial in envelope["design"]["trials"]
                   if trial.get("run_order") == seed]
        assert len(matches) == 1
    assert envelope["design_hash"] == StudyDesign.read_frozen(STUDY / "design.json").design_hash
    assert json.loads((STUDY / "canary_trial_ids.json").read_text()) == [
        trial["trial_id"] for trial in envelope["design"]["trials"][:2]]


def test_builder_refuses_what_cannot_be_run_or_mapped_exactly():
    courses = load_courses(BATCH_MANIFESTS)
    with pytest.raises(ValueError, match="no world spawns it assembled"):
        build_design(courses, morphology="narrow_tandem")
    odd = load_courses([path for path in EXTRA_MANIFESTS if "c077e984a4cfaf72" in path.name])
    with pytest.raises(ValueError, match="half centimetre"):
        build_design(odd)
    assert len(build_design(odd, allow_inexact_map=True).trials) == 4
    with pytest.raises(ValueError, match="methods"):
        build_design(courses, methods=("route_first_adaptation",))


def test_existing_designs_and_specs_are_untouched(tmp_path):
    # Plain specs keep exactly their fields, so every existing hash stands.
    assert [field.name for field in fields(TrialSpec)] == [
        "trial_id", "family", "layout_id", "replicate", "method", "method_order",
        "world_seed", "sensing_seed", "friction_seed", "fault_seed"]
    plain = generate_design(2, 1)
    assert all(type(trial) is TrialSpec for trial in plain.trials)
    path = plain.write_frozen(tmp_path / "plain.json")
    assert StudyDesign.read_frozen(path) == plain
    for frozen in sorted((ROOT / "config").glob("*.json")):
        design = StudyDesign.read_frozen(frozen)
        assert all(type(trial) is TrialSpec for trial in design.trials)
    assert type(trial_spec_from_dict(asdict(plain.trials[0]))) is TrialSpec
    # A doorway trial cannot ride in another design, nor a doorway design carry others.
    doorway = _design()
    mixed = replace(plain, trials=(*plain.trials, doorway.trials[0]))
    mixed_path = tmp_path / "mixed.json"
    mixed_path.write_text(json.dumps(
        {"design_hash": mixed.design_hash, "design": asdict(mixed)}))
    with pytest.raises(ValueError, match="doorway trials belong only"):
        StudyDesign.read_frozen(mixed_path)
    wrong = replace(doorway, methods=("geometry_coupled",))
    wrong_path = tmp_path / "wrong.json"
    wrong_path.write_text(json.dumps(
        {"design_hash": wrong.design_hash, "design": asdict(wrong)}))
    with pytest.raises(ValueError, match="unsupported method set"):
        StudyDesign.read_frozen(wrong_path)


def test_pending_order_and_only_filter(tmp_path):
    design = _design()
    store = TrialStore(tmp_path / "raw")
    assert [trial.run_order for trial in pending_trials(design, store)] == list(range(40))
    chosen = [design.trials[7].trial_id, design.trials[3].trial_id]
    assert [trial.run_order for trial in pending_trials(design, store, chosen)] == [3, 7]
    with pytest.raises(ValueError, match="not in the design"):
        pending_trials(design, store, ["ecd-nope-r0-A"])
    # Specs without a run order keep the order the runner always used.
    plain = generate_design(3, 2)
    legacy = sorted(plain.trials, key=lambda spec: (
        spec.family, spec.layout_id, spec.replicate, spec.method_order))
    assert pending_trials(plain, store) == legacy
    assert _parser().parse_args(
        ["--design", "d.json", "--raw", "r", "--artifacts", "a", "--only", "o.json"]
    ).only == Path("o.json")
    assert _parser().parse_args(["--design", "d", "--raw", "r", "--artifacts", "a"]).only is None


# ----------------------------------------------------------- configuration

def _load(name):
    return yaml.safe_load((ROOT / NAV2_CONFIG_DIR / name).read_text())


def _flat(value, prefix=""):
    if isinstance(value, dict):
        out = {}
        for key, inner in value.items():
            out.update(_flat(inner, f"{prefix}{key}."))
        return out
    return {prefix[:-1]: value}


# Parameter names declared by Nav2's jazzy branch (1.3.x), read from
# nav2_regulated_pure_pursuit_controller/src/parameter_handler.cpp,
# nav2_smac_planner/src/smac_planner_hybrid.cpp + include/.../types.hpp,
# nav2_navfn_planner/src/navfn_planner.cpp and nav2_bringup/params/nav2_params.yaml.
RPP_PARAMETERS = {
    "plugin", "allow_reversing", "approach_velocity_scaling_dist", "cancel_deceleration",
    "cost_scaling_dist", "cost_scaling_gain", "curvature_lookahead_dist",
    "desired_linear_vel", "inflation_cost_scaling_factor",
    "interpolate_curvature_after_goal", "lookahead_dist", "lookahead_time",
    "max_allowed_time_to_collision_up_to_carrot", "max_angular_accel",
    "max_lookahead_dist", "max_robot_pose_search_dist", "min_approach_linear_velocity",
    "min_lookahead_dist", "regulated_linear_scaling_min_radius",
    "regulated_linear_scaling_min_speed", "rotate_to_heading_angular_vel",
    "rotate_to_heading_min_angle", "stateful", "transform_tolerance",
    "use_cancel_deceleration", "use_collision_detection",
    "use_cost_regulated_linear_velocity_scaling", "use_fixed_curvature_lookahead",
    "use_regulated_linear_velocity_scaling", "use_rotate_to_heading",
    "use_velocity_scaled_lookahead_dist"}
SMAC_PARAMETERS = {
    "plugin", "allow_primitive_interpolation", "allow_unknown",
    "analytic_expansion_max_cost", "analytic_expansion_max_cost_override",
    "analytic_expansion_max_length", "analytic_expansion_ratio",
    "angle_quantization_bins", "cache_obstacle_heuristic", "change_penalty",
    "cost_penalty", "debug_visualizations", "downsample_costmap",
    "downsample_obstacle_heuristic", "downsampling_factor", "lookup_table_size",
    "max_iterations", "max_on_approach_iterations", "max_planning_time",
    "minimum_turning_radius", "motion_model_for_search", "non_straight_penalty",
    "retrospective_penalty", "reverse_penalty", "smooth_path",
    "terminal_checking_interval", "tolerance", "use_quadratic_cost_penalty", "smoother"}
SMAC_SMOOTHER_PARAMETERS = {
    "tolerance", "max_iterations", "w_data", "w_smooth", "do_refinement", "refinement_num"}
NAVFN_PARAMETERS = {"plugin", "tolerance", "use_astar", "allow_unknown",
                    "use_final_approach_orientation"}
MPPI_PARAMETERS = {
    "plugin", "time_steps", "model_dt", "batch_size", "ax_max", "ax_min", "ay_max",
    "ay_min", "az_max", "vx_std", "vy_std", "wz_std", "vx_max", "vx_min", "vy_max",
    "wz_max", "iteration_count", "prune_distance", "transform_tolerance", "temperature",
    "gamma", "motion_model", "visualize", "regenerate_noises", "critics",
    "ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic",
    "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"}


@pytest.mark.parametrize("key", sorted(NAV2_CONFIGS), ids=lambda key: "-".join(key))
def test_parameter_files_are_frozen_and_use_known_names(key):
    name, digest = NAV2_CONFIGS[key]
    assert hashlib.sha256((ROOT / NAV2_CONFIG_DIR / name).read_bytes()).hexdigest() == digest
    assert nav2_config_for(*key, ROOT) == (f"{NAV2_CONFIG_DIR}/{name}", digest)
    config = _load(name)
    controller = config["controller_server"]["ros__parameters"]
    planner = config["planner_server"]["ros__parameters"]
    # The ids stock Nav2's default behaviour tree selects.
    assert controller["controller_plugins"] == ["FollowPath"]
    assert planner["planner_plugins"] == ["GridBased"]
    follow, grid = controller["FollowPath"], planner["GridBased"]
    if follow["plugin"].startswith("nav2_mppi_controller"):
        assert set(follow) == MPPI_PARAMETERS
        assert follow["motion_model"] == "DiffDrive" and follow["vx_min"] < 0
        assert set(follow["critics"]) <= set(follow)
    else:
        assert follow["plugin"] == (
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
        assert set(follow) <= RPP_PARAMETERS
        # Nav2 refuses to combine these two.
        assert not (follow["use_rotate_to_heading"] and follow["allow_reversing"])
    if grid["plugin"] == "nav2_smac_planner::SmacPlannerHybrid":
        assert set(grid) == SMAC_PARAMETERS
        assert set(grid["smoother"]) == SMAC_SMOOTHER_PARAMETERS
        assert grid["motion_model_for_search"] == "REEDS_SHEPP"
        # 0.01 m map -> 0.05 m search grid.
        assert grid["downsample_costmap"] and grid["downsampling_factor"] * MAP_RESOLUTION_M == 0.05
    else:
        assert grid["plugin"] == "nav2_navfn_planner::NavfnPlanner"
        assert set(grid) <= NAVFN_PARAMETERS
    for costmap in ("local_costmap", "global_costmap"):
        assert "filters" not in config[costmap][costmap]["ros__parameters"]


@pytest.mark.parametrize("morphology,block", [("compact_diff", "FollowPathDiff"),
                                              ("narrow_tandem", "FollowPathNarrow")])
def test_arm_a_is_the_repository_setup_and_arms_differ_only_where_declared(morphology, block):
    base = yaml.safe_load((ROOT / "src/modular_robot_bringup/config/nav2.yaml").read_text())
    arms = {DOORWAY_METHODS[method]: _load(NAV2_CONFIGS[(morphology, method)][0])
            for method in DOORWAY_METHODS}
    a = arms["A"]
    controller = a["controller_server"]["ros__parameters"]
    assert controller["FollowPath"] == {
        **base["controller_server"]["ros__parameters"][block],
        "use_rotate_to_heading": True, "allow_reversing": False}
    # Everything outside the controller plugin list is the repository's, minus keepout.
    expected = json.loads(json.dumps(base))
    reference = expected["controller_server"]["ros__parameters"]
    for name in reference.pop("controller_plugins"):
        del reference[name]
    for name in ("local_costmap", "global_costmap"):
        del expected[name][name]["ros__parameters"]["filters"]
        del expected[name][name]["ros__parameters"]["keepout_filter"]
    stripped = json.loads(json.dumps(a))
    del stripped["controller_server"]["ros__parameters"]["FollowPath"]
    del stripped["controller_server"]["ros__parameters"]["controller_plugins"]
    assert stripped == expected

    flat = {arm: _flat(config) for arm, config in arms.items()}

    def differing(first, second):
        keys = set(flat[first]) | set(flat[second])
        return {key for key in keys if flat[first].get(key) != flat[second].get(key)}

    follow = "controller_server.ros__parameters.FollowPath."
    grid = "planner_server.ros__parameters.GridBased."
    # A vs B: the controller plugin block and nothing else (same planner, costmaps,
    # goal and progress checkers, smoother limits, collision monitor, behaviours).
    assert all(key.startswith(follow) for key in differing("A", "B"))
    # A vs C: two controller flags and the planner block.
    assert differing("A", "C") - {follow + "use_rotate_to_heading", follow + "allow_reversing"} \
        == {key for key in differing("A", "C") if key.startswith(grid)}
    assert {follow + "use_rotate_to_heading", follow + "allow_reversing"} <= differing("A", "C")
    # Same speed and acceleration limits across the arms.
    rpp, mppi = arms["A"]["controller_server"]["ros__parameters"]["FollowPath"], \
        arms["B"]["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = a["velocity_smoother"]["ros__parameters"]
    assert mppi["vx_max"] == rpp["desired_linear_vel"] == -mppi["vx_min"]
    assert mppi["wz_max"] == rpp["rotate_to_heading_angular_vel"]
    assert -mppi["vx_min"] <= -smoother["min_velocity"][0]
    assert (mppi["ax_max"], -mppi["ax_min"], mppi["az_max"]) == (
        smoother["max_accel"][0], -smoother["max_decel"][0], smoother["max_accel"][2])
    assert mppi["transform_tolerance"] == rpp["transform_tolerance"]
    assert arms["C"]["planner_server"]["ros__parameters"]["GridBased"]["tolerance"] == \
        a["planner_server"]["ros__parameters"]["GridBased"]["tolerance"]


def test_launch_file_takes_the_parameter_file_and_defaults_to_the_existing_one():
    text = (ROOT / "src/modular_robot_bringup/launch/demo.launch.py").read_text()
    compile(text, "demo.launch.py", "exec")
    assert '"nav2_params", default_value=os.path.join(bringup, "config", "nav2.yaml")' in text
    assert 'launch_arguments={"params_file": nav2_params, "use_sim_time": "true"}' in text
    assert text.count('"config", "nav2.yaml"') == 1


# ------------------------------------------------------------ trial set-up

def test_prepare_trial_writes_a_consistent_trial_and_checks_the_frozen_file(tmp_path):
    design = _design()
    spec = design.trials[1]
    prepared = prepare_trial(ROOT, spec, tmp_path / spec.trial_id, CATALOG, 600.0, 2)
    arguments = dict(argument.split(":=", 1) for argument in prepared["launch_arguments"])
    assert set(arguments) == {"world", "map", "initial_x", "initial_y", "initial_yaw",
                              "transition_environment", "station_gated_visibility",
                              "nav2_params"}
    assert arguments["nav2_params"] == str((ROOT / spec.nav2_config).resolve())
    assert arguments["station_gated_visibility"] == "false"
    for key in ("world", "map", "transition_environment", "nav2_params"):
        assert Path(arguments[key]).is_absolute() and Path(arguments[key]).is_file()
    # AMCL's prior is the pose the world spawns the robot at.
    include = ElementTree.parse(arguments["world"]).getroot().find("world/include/pose")
    x, y, _, _, _, yaw = (float(v) for v in include.text.split())
    assert (float(arguments["initial_x"]), float(arguments["initial_y"]),
            float(arguments["initial_yaw"])) == pytest.approx((x, y, yaw))
    parameters = prepared["parameters"]
    assert parameters["nav2_config_sha256"] == spec.nav2_config_sha256
    assert parameters["course_id"] == spec.course_id and parameters["deadline_s"] == 120.0
    assert parameters["map_exactness"]["exact_at_map_resolution"] is True
    assert prepared["course"].goal == (spec.goal_x_cm / 100, spec.goal_y_cm / 100)
    # The two arms of a block get the same world and map but a different stack hash.
    other = prepare_trial(ROOT, design.trials[0], tmp_path / "other", CATALOG, 600.0, 2)
    assert (tmp_path / "other/world.sdf").read_bytes() == \
        (tmp_path / spec.trial_id / "world.sdf").read_bytes()
    assert (tmp_path / "other/map.pgm").read_bytes() == \
        (tmp_path / spec.trial_id / "map.pgm").read_bytes()
    assert other["parameters"]["stack_configuration_hash"] != \
        parameters["stack_configuration_hash"]
    with pytest.raises(ValueError, match="the design froze"):
        prepare_trial(ROOT, replace(spec, nav2_config_sha256="0" * 64),
                      tmp_path / "bad", CATALOG, 600.0, 2)
    with pytest.raises(ValueError, match="course fields hash"):
        prepare_trial(ROOT, replace(spec, d_cm=spec.d_cm + 1), tmp_path / "bad2",
                      CATALOG, 600.0, 2)


def test_a_doorway_record_round_trips_through_the_store(tmp_path):
    spec = _design().trials[0]
    record = TrialRecord(
        schema_version=1, spec=spec,
        manifest=TrialManifest("c", "h", "d", {}, {}), terminal_status="timeout",
        completed=False, simulated_duration_s=120.0, wall_duration_s=200.0,
        deadline_s=spec.deadline_s,
        infrastructure_attempts=[{"attempt": 1, "terminal_status": "timeout"}])
    store = TrialStore(tmp_path)
    store.write_terminal(record)
    loaded = store.load_all()[0]
    assert type(loaded.spec) is DoorwayTrialSpec and loaded.spec == spec
    assert loaded.deadline_penalized_time_s == 120.0


# ----------------------------------------------------------------- outcome

def test_nav2_result_maps_onto_the_shared_terminal_statuses():
    assert nav2_terminal(STATUS_SUCCEEDED, 0, False) == ("completed", True)
    assert nav2_terminal(STATUS_SUCCEEDED, 0, True) == ("timeout", False)
    assert nav2_terminal(STATUS_CANCELED, 0, False) == ("cancelled", False)
    assert nav2_terminal(STATUS_ABORTED, 209, False) == ("planning_failure", False)
    assert nav2_terminal(STATUS_ABORTED, 105, False) == ("controller_failure", False)
    assert nav2_terminal(STATUS_ABORTED, 0, False) == ("controller_failure", False)


def test_contact_before_a_planning_failure_is_a_collision():
    contact = [{"time_s": 12.0, "module": "pod_0", "clearance_m": 0.0}]
    assert doorway_outcome("planning_failure", False, [], []) == ("planning_failure", False, "")
    status, completed, note = doorway_outcome("planning_failure", False, contact, [])
    assert (status, completed) == ("collision", False) and "planning_failure" in note
    assert doorway_outcome("completed", True, contact, [])[0] == "collision"
    assert doorway_outcome("infrastructure_failure", False, contact, []) == (
        "infrastructure_failure", False, "")


def test_motion_summary_counts_stretches_and_reverse_travel():
    commands = [{"body_linear_x": v} for v in (0.0, 0.3, 0.3, 0.01, -0.2, -0.2, 0.0, 0.3)]
    odometry = [
        {"time_s": 0.0, "x": 0.0, "y": 0.0, "linear_x": 0.0, "angular_z": 0.0},
        {"time_s": 0.5, "x": 0.1, "y": 0.0, "linear_x": 0.3, "angular_z": 0.0},
        {"time_s": 1.0, "x": 0.06, "y": 0.0, "linear_x": -0.2, "angular_z": 0.4},
    ]
    summary = motion_summary(commands, odometry)
    assert (summary["stretches"], summary["reversals"], summary["reverse_stretches"]) == (3, 2, 1)
    assert summary["forward_travel_m"] == pytest.approx(0.1)
    assert summary["reverse_travel_m"] == pytest.approx(0.04)
    assert summary["rotation_rad_approx"] == pytest.approx(0.2)
    assert motion_summary([], [])["reversals"] == 0


def test_goal_truth_uses_the_last_evaluator_pose():
    samples = [
        {"time_s": 1.0, "core_world_x": 0.5, "core_world_y": 0.0},
        {"time_s": 9.0, "core_world_x": 2.2, "core_world_y": 0.6, "core_world_yaw": 0.1},
        {"time_s": 3.0, "core_world_x": 1.0, "core_world_y": 0.2},
    ]
    truth = goal_truth(samples, (2.25, 0.70))
    assert truth["time_s"] == 9.0 and truth["within_goal_tolerance"]
    assert truth["goal_distance_m"] == pytest.approx((0.05 ** 2 + 0.1 ** 2) ** 0.5)
    assert goal_truth([], (0.0, 0.0)) == {"available": False}


# ------------------------------------------------------------- batch loop

ROS_MODULES = (
    "rclpy", "rclpy.action", "rclpy.duration", "rclpy.executors", "rclpy.node",
    "rclpy.qos", "rclpy.time", "geometry_msgs", "geometry_msgs.msg", "lifecycle_msgs",
    "lifecycle_msgs.msg", "lifecycle_msgs.srv", "modular_robot_msgs",
    "modular_robot_msgs.action", "modular_robot_msgs.msg", "modular_robot_msgs.srv",
    "nav_msgs", "nav_msgs.msg", "nav2_msgs", "nav2_msgs.action", "sensor_msgs",
    "sensor_msgs.msg", "rosgraph_msgs", "rosgraph_msgs.msg", "std_msgs", "std_msgs.msg",
    "std_srvs", "std_srvs.srv", "tf2_ros",
)


@pytest.fixture
def ros_or_stubs(monkeypatch):
    """The real ROS modules where they exist (the container), stand-ins on a bare host.

    Only the imports are satisfied; no node is created. With real ROS this also
    proves `doorway_trial` imports against the installed nav2_msgs.
    """
    import sys
    from unittest import mock

    before = set(sys.modules)
    try:
        import rclpy  # noqa: F401
        import nav2_msgs.action  # noqa: F401
    except ImportError:
        for name in ROS_MODULES:
            monkeypatch.setitem(sys.modules, name, mock.MagicMock(name=name))
    yield
    # Modules first imported here may hold stand-ins; drop them completely.
    package = sys.modules["modular_robot_benchmarks"]
    for name in set(sys.modules) - before:
        if name.startswith("modular_robot_benchmarks."):
            del sys.modules[name]
            if hasattr(package, name.rsplit(".", 1)[1]):
                delattr(package, name.rsplit(".", 1)[1])


class _FakeLaunch:
    commands = []

    def __init__(self, command, **kwargs):
        type(self).commands.append(command)
        kwargs["stdout"].write("[INFO] [launch]: fake stack\n")
        self.pid = 0

    def poll(self):
        return None


def _run_fake_batch(monkeypatch, tmp_path, observation_for, only):
    from importlib import import_module

    doorway_batch = import_module("modular_robot_benchmarks.doorway_batch")
    doorway_trial = import_module("modular_robot_benchmarks.doorway_trial")
    mission_batch = import_module("modular_robot_benchmarks.mission_batch")

    _FakeLaunch.commands = []
    # Only the runner's own handle: `platform.platform()` also spawns a process.
    monkeypatch.setattr(doorway_batch, "subprocess", SimpleNamespace(
        Popen=_FakeLaunch, STDOUT=doorway_batch.subprocess.STDOUT))
    monkeypatch.setattr(mission_batch, "_stop_process", lambda process: None)
    monkeypatch.setattr(mission_batch, "_git_revision", lambda root, dirty: "f" * 40)
    monkeypatch.setattr(mission_batch, "clear_unused_dds_segments", lambda: [])
    monkeypatch.setattr(doorway_trial, "execute_doorway_mission_isolated", observation_for)
    only_path = tmp_path / "only.json"
    only_path.write_text(json.dumps(only))
    result = mission_batch.run_batch(
        ROOT, STUDY / "design.json", tmp_path / "raw", tmp_path / "artifacts", CATALOG,
        None, 600.0, False, 2, json.loads(only_path.read_text()))
    return result, _FakeLaunch.commands


def test_batch_loop_writes_a_valid_record_without_a_simulator(ros_or_stubs, monkeypatch, tmp_path):
    from modular_robot_benchmarks.mission_trial import MissionObservation

    design = StudyDesign.read_frozen(STUDY / "design.json")
    spec = design.trials[0]
    goal = (spec.goal_x_cm / 100.0, spec.goal_y_cm / 100.0)
    calls = []

    def at_goal(goal_xy, goal_yaw, deadline_s, wall_watchdog_s):
        calls.append((goal_xy, goal_yaw, deadline_s))
        truth = [{"time_s": t, "pod": "pod_0", "core_world_x": x, "core_world_y": goal[1],
                  "core_world_z": 0.18, "core_world_yaw": 0.0}
                 for t, x in ((1.0, 0.6), (20.0, goal[0] - 0.05))]
        return MissionObservation(
            "completed", True, "nav2 navigate_to_pose goal status 4, error code 0",
            21.0, 60.0, final_execution_state="READY", final_morphology="compact_diff",
            phase_timing={"readiness_wall_s": 40.0, "mission_wall_s": 20.0,
                          "real_time_factor": 1.05},
            command_history=[{"time_s": 1.0, "body_linear_x": 0.3},
                             {"time_s": 1.5, "body_linear_x": -0.2}],
            controller_diagnostics={"pod_drive_diagnostic_samples": truth,
                                    "pod_pose_samples": [], "clock_discontinuities": [],
                                    "nav2_result": {"goal_status": 4, "error_code": 0}})

    result, commands = _run_fake_batch(monkeypatch, tmp_path, at_goal, [spec.trial_id])
    assert result == {"design_hash": design.design_hash, "written": [spec.trial_id],
                      "written_count": 1, "remaining_count": 39}
    assert calls == [(goal, 0.0, 120.0)]
    assert commands[0][:4] == ["ros2", "launch", "modular_robot_bringup", "demo.launch.py"]
    assert f"nav2_params:={(ROOT / spec.nav2_config).resolve()}" in commands[0]
    assert not any(argument.startswith("keepout_mask") for argument in commands[0])
    raw = (tmp_path / "raw" / f"{spec.trial_id}.json").read_text()
    record = TrialRecord.from_dict(json.loads(raw))
    assert record.spec == spec and record.completed and record.deadline_s == 120.0
    assert record.manifest.design_hash == design.design_hash
    assert record.manifest.parameters["nav2_config_sha256"] == spec.nav2_config_sha256
    extra = record.controller_diagnostics["expchoice"]
    assert extra["goal_truth"]["within_goal_tolerance"]
    assert extra["motion"]["reversals"] == 1
    # What the job-queue adapter reads back from the record.
    stored = json.loads(raw)
    assert stored["manifest"]["design_hash"] == design.design_hash
    assert stored["terminal_status"] == "completed"
    assert stored["phase_timing"]["real_time_factor"] == 1.05
    assert (tmp_path / "artifacts" / spec.trial_id / "launch.log").is_file()
    # A finished trial is skipped, so a retried seed is idempotent.
    again, _ = _run_fake_batch(monkeypatch, tmp_path, at_goal, [spec.trial_id])
    assert again["written"] == []


def test_batch_loop_scores_wall_contact_as_collision(ros_or_stubs, monkeypatch, tmp_path):
    from modular_robot_benchmarks.mission_trial import MissionObservation

    spec = StudyDesign.read_frozen(STUDY / "design.json").trials[3]
    door_x = spec.d_cm / 100.0

    def into_the_door_wall(goal_xy, goal_yaw, deadline_s, wall_watchdog_s):
        # Core nose 3 cm inside the door wall, well away from the opening.
        truth = [{"time_s": 8.0, "pod": "pod_0", "core_world_x": door_x - 0.10,
                  "core_world_y": -1.5, "core_world_z": 0.18, "core_world_yaw": 0.0}]
        return MissionObservation(
            "controller_failure", False, "nav2 navigate_to_pose goal status 6, error code 105",
            30.0, 70.0, final_execution_state="READY",
            controller_diagnostics={"pod_drive_diagnostic_samples": truth,
                                    "pod_pose_samples": [], "clock_discontinuities": []})

    _run_fake_batch(monkeypatch, tmp_path, into_the_door_wall, [spec.trial_id])
    record = TrialStore(tmp_path / "raw").load_all()[0]
    assert record.terminal_status == "collision" and record.collision_count == 1
    assert record.minimum_clearance_m == 0.0
    assert not record.controller_diagnostics["expchoice"]["goal_truth"]["within_goal_tolerance"]


def test_published_footprint_is_reduced_to_its_edge_lengths():
    # navigation_footprint 0.66 x 0.56 padded by 0.01 a side, rotated a quarter turn.
    corners = [(0.29, -0.34), (0.29, 0.34), (-0.29, 0.34), (-0.29, -0.34)]
    assert footprint_edges(corners) == [0.58, 0.58, 0.68, 0.68]
    assert footprint_edges([]) == []
