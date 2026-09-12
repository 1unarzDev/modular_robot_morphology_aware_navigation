import json
from pathlib import Path
from xml.etree.ElementTree import parse

from modular_robot_benchmarks.sdf_export import export_confirmatory_sdf


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
    assert includes["core"] == "model://sensor_core"
    assert includes["pod_5"] == "model://drive_pod_5"
    assert world.find("model[@name='raised_transition_shelf']") is not None
    manifest = json.loads(manifest_path.read_text())
    assert manifest["layout_id"] == "combined_constraints-01"
    assert manifest["world_seed"] == 8
