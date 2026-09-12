from pathlib import Path
from xml.etree.ElementTree import parse


ROOT = Path(__file__).parents[1]


def test_articulated_model_has_one_drive_owner_and_six_nested_pods():
    model = parse(
        ROOT / "src/modular_robot_sim/models/modular_robot/model.sdf"
    ).getroot().find("model")
    assert model is not None
    includes = model.findall("include")
    pod_names = {
        include.findtext("name") for include in includes
        if (include.findtext("name") or "").startswith("pod_")
    }
    assert pod_names == {f"pod_{index}" for index in range(6)}
    plugins = model.findall("plugin")
    assert [plugin.get("filename") for plugin in plugins] == [
        "libmulti_pod_drive_system.so"
    ]
    configured = {item.text for item in plugins[0].findall("pod")}
    assert configured == pod_names
    assert plugins[0].findtext("actuation_mode") == "effort"


def test_articulated_pod_keeps_physical_wheels_without_competing_controller():
    text = (
        ROOT / "src/modular_robot_sim/models/articulated_pod/model.sdf.in"
    ).read_text()
    assert "@POD_ID@_left_wheel_joint" in text
    assert "@POD_ID@_right_wheel_joint" in text
    assert "DiffDrive" not in text
