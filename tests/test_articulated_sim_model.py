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


def test_every_anisotropic_collision_attaches_its_friction_axes():
    """No collision may declare ``mu != mu2`` without ``fdir1`` (ADR 0007).

    Without ``fdir1`` the friction principal axes are world-fixed rather than
    attached to the wheel, so traction depends on the robot's heading in the
    world. Measured on the unrepaired model: rotation authority ran 0.733 of
    command near 0 and 180 degrees and 0.042 within 10 degrees of 90, and
    straight travel lost 21% at 90 -- entirely in the acceleration ramp
    (``studies/engineering/heading_traction_curve.json``). The repair is
    ``fdir1 1 0 0``: the wheel collision pose rotates +1.5708 rad about x, so
    collision-frame x is the rolling direction, putting ``mu`` along rolling and
    ``mu2`` along lateral scrub.

    This guards the class of defect rather than one file, because the figures it
    silently scales are every signed-motion, travel and reconfiguration number
    the platform records.
    """
    sim = ROOT / "src/modular_robot_sim"
    offenders = []
    for pattern in ("models/**/*.sdf", "models/**/*.sdf.in", "worlds/*.sdf"):
        for path in sorted(sim.glob(pattern)):
            for surface in re.findall(r"<friction>.*?</friction>",
                                      path.read_text(), re.S):
                mu = re.search(r"<mu>([^<]+)</mu>", surface)
                mu2 = re.search(r"<mu2>([^<]+)</mu2>", surface)
                if mu is None or mu2 is None:
                    continue
                if float(mu.group(1)) != float(mu2.group(1))                         and "<fdir1>" not in surface:
                    offenders.append(
                        (path.relative_to(ROOT).as_posix(), surface[:80]))
    assert offenders == [], (
        "anisotropic friction without fdir1 makes traction depend on world "
        f"heading (ADR 0007): {offenders}"
    )


def test_repaired_wheels_put_the_primary_axis_along_rolling():
    text = (
        ROOT / "src/modular_robot_sim/models/articulated_pod/model.sdf.in"
    ).read_text()
    assert text.count(
        "<mu>1.2</mu><mu2>0.08</mu2><fdir1>1 0 0</fdir1>") == 2


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
