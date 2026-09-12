from pathlib import Path
from xml.etree.ElementTree import parse


ROOT = Path(__file__).parents[1]


def test_detached_pod_rig_uses_shared_single_owner_drive():
    model = parse(ROOT / "src/modular_robot_sim/models/self_mobile_pod/model.sdf").getroot().find("model")
    assert model is not None
    assert model.find("include/name").text == "pod_0"
    plugin = model.find("plugin")
    assert plugin.get("filename") == "libmulti_pod_drive_system.so"
    assert [value.text for value in plugin.findall("pod")] == ["pod_0"]
    assert plugin.findtext("actuation_mode") == "effort"
    assert float(plugin.findtext("max_wheel_effort")) > 0.0
    assert float(plugin.findtext("velocity_gain")) > 0.0


def test_detached_pod_world_contains_no_topology_latch_plugin():
    world = parse(ROOT / "src/modular_robot_sim/worlds/self_mobile_pod_qualification.sdf").getroot().find("world")
    assert world is not None
    assert "TopologyJointSystem" not in " ".join(
        plugin.get("name", "") for plugin in world.findall("plugin"))
