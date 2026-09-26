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
    assert text.count("<lower>-0.015</lower><upper>0.015</upper>") == 2
    assert text.count("_caster_collision") == 2
    assert text.count("<mu>0.01</mu><mu2>0.01</mu2>") == 2
    assert text.count("<mu>1.2</mu><mu2>0.08</mu2>") == 2


def test_wheel_friction_is_still_anisotropic_without_fdir1():
    """Pin the state every recorded motion figure was collected in.

    The wheels declare mu 1.2 against mu2 0.08 and no fdir1, so the friction
    principal axes are world-fixed rather than attached to the wheel and
    traction depends on the robot's heading in the world. Measured: rotation
    authority is 74% of command at every heading once fdir1 is set, and exactly
    0% at +/-90 degrees without it, with straight travel 21.2% short there
    (studies/engineering/wheel_friction_intervention.json).

    Repairing it is an author decision because it moves every signed-motion,
    travel and reconfiguration figure on record (ADR 0007, proposed). This test
    fails when that decision is taken, which is the point: the re-collection it
    forces must not start as a side effect of a tuning change.
    """
    sim = ROOT / "src/modular_robot_sim"
    declaring = [
        path.relative_to(ROOT).as_posix()
        for pattern in ("models/**/*.sdf", "models/**/*.sdf.in", "worlds/*.sdf")
        for path in sim.glob(pattern)
        if "fdir1" in path.read_text()
    ]
    assert declaring == [], (
        f"{declaring} now declare fdir1; if ADR 0007 was adopted, invert this "
        "test and re-collect Gate 0 item 1, the round-trip gate and the fault "
        "matrix"
    )


def test_drive_diagnostics_measure_both_suspension_carriers():
    text = (
        ROOT / "src/modular_robot_gz_plugins/src/multi_pod_drive_system.cc"
    ).read_text()
    for field in (
        "left_suspension_m", "right_suspension_m",
        "left_suspension_mps", "right_suspension_mps",
    ):
        assert field in text
    assert "_left_suspension" in text
    assert "_right_suspension" in text
