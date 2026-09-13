import json
from pathlib import Path
from xml.etree.ElementTree import parse

from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from modular_robot_benchmarks.sdf_export import (
    export_confirmatory_sdf, export_occupancy_map, physical_disturbance,
)


CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"


def test_confirmatory_export_contains_robot_physics_constraints_and_manifest(tmp_path):
    world_path, manifest_path = export_confirmatory_sdf(
        "combined_constraints", 1, 8, tmp_path / "world.sdf", CATALOG)
    world = parse(world_path).getroot().find("world")
    assert world is not None
    plugins = {plugin.get("filename") for plugin in world.findall("plugin")}
    assert "libtopology_joint_system.so" in plugins
    assert "gz-sim-logical-camera-system" in plugins
    includes = {item.findtext("name"): item.findtext("uri")
                for item in world.findall("include")}
    assert includes["core"] == "model://modular_robot"
    assert "pod_5" not in includes
    topology = next(plugin for plugin in world.findall("plugin")
                    if plugin.get("filename") == "libtopology_joint_system.so")
    children = [item.text for item in topology.findall("initial_child")]
    assert len(children) == 6
    assert all(value.startswith("core::pod_") for value in children)
    assert world.find("model[@name='raised_transition_shelf']") is not None
    manifest = json.loads(manifest_path.read_text())
    assert manifest["layout_id"] == "combined_constraints-01"
    assert manifest["world_seed"] == 8
    assert manifest["physical_disturbance"] == physical_disturbance(None)


def test_friction_seed_changes_recorded_plant_and_initial_pose(tmp_path):
    first_world, first_manifest = export_confirmatory_sdf(
        "combined_constraints", 1, 8, tmp_path / "first.sdf", CATALOG, 10)
    second_world, second_manifest = export_confirmatory_sdf(
        "combined_constraints", 1, 8, tmp_path / "second.sdf", CATALOG, 11)
    first = json.loads(first_manifest.read_text())["physical_disturbance"]
    second = json.loads(second_manifest.read_text())["physical_disturbance"]
    assert first == physical_disturbance(10)
    assert first != second
    assert 0.70 <= first["ground_friction"] <= 1.10
    pose = parse(first_world).getroot().find(
        "world/include[name='core']/pose").text.split()
    assert float(pose[5]) == first["initial_yaw_rad"]
    ground = parse(first_world).getroot().find("world/model[@name='ground']")
    assert float(ground.findtext("link/collision/surface/friction/ode/mu")) == first["ground_friction"]


def test_occupancy_map_matches_scenario_grid_and_y_axis(tmp_path):
    scenario = make_confirmatory_scenario("combined_constraints", 1, 42)
    yaml_path, image_path = export_occupancy_map(scenario.grid, tmp_path / "map.yaml")
    header, dimensions, maximum, pixels = image_path.read_bytes().split(b"\n", 3)
    assert header == b"P5"
    assert dimensions == f"{scenario.grid.width} {scenario.grid.height}".encode()
    assert maximum == b"255"
    assert set(pixels[:scenario.grid.width]) == {0}
    assert set(pixels[-scenario.grid.width:]) == {0}
    assert "image: map.pgm" in yaml_path.read_text()
