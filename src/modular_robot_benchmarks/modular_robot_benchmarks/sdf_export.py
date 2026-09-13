from __future__ import annotations

import argparse
import json
from pathlib import Path
from random import Random
from xml.etree.ElementTree import Element, ElementTree, SubElement

from morphology_planner.grid import OCCUPIED
from morphology_planner import load_catalog

from .confirmatory_scenarios import make_confirmatory_scenario
from .design import CONFIRMATORY_FAMILIES
from .scenarios import FAMILIES, make_scenario


def physical_disturbance(seed: int | None) -> dict[str, float]:
    """Realized plant nuisance values shared by every method in a paired block."""
    if seed is None:
        return {"ground_friction": 1.0, "initial_dx_m": 0.0,
                "initial_dy_m": 0.0, "initial_yaw_rad": 0.0}
    rng = Random(seed)
    return {
        "ground_friction": rng.uniform(0.70, 1.10),
        "initial_dx_m": rng.uniform(-0.015, 0.015),
        "initial_dy_m": rng.uniform(-0.015, 0.015),
        "initial_yaw_rad": rng.uniform(-0.035, 0.035),
    }


def export_occupancy_map(grid, output_yaml: Path) -> tuple[Path, Path]:
    """Export a deterministic Nav2 map; image rows use map-server y ordering."""
    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    image_path = output_yaml.with_suffix(".pgm")
    pixels = bytearray()
    for y in reversed(range(grid.height)):
        for x in range(grid.width):
            value = grid.value(x, y)
            pixels.append(0 if value >= OCCUPIED else (205 if value < 0 else 254))
    image_path.write_bytes(
        f"P5\n{grid.width} {grid.height}\n255\n".encode("ascii") + pixels)
    output_yaml.write_text(
        f"image: {image_path.name}\n"
        f"mode: trinary\nresolution: {grid.resolution}\n"
        f"origin: [{grid.origin_x}, {grid.origin_y}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    return output_yaml, image_path


def export_sdf(family: str, seed: int, output: Path) -> None:
    scenario = make_scenario(family, seed)
    sdf = Element("sdf", version="1.10")
    world = SubElement(sdf, "world", name=f"{family}_{seed}")
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        SubElement(world, "plugin", filename=filename, name=name)
    for y in range(scenario.grid.height):
        for x in range(scenario.grid.width):
            if scenario.grid.value(x, y) != OCCUPIED:
                continue
            model = SubElement(world, "model", name=f"obstacle_{x}_{y}")
            SubElement(model, "static").text = "true"
            wx, wy = scenario.grid.cell_center(x, y)
            SubElement(model, "pose").text = f"{wx} {wy} 0.5 0 0 0"
            link = SubElement(model, "link", name="link")
            for tag in ("collision", "visual"):
                item = SubElement(link, tag, name=tag)
                geometry = SubElement(item, "geometry")
                box = SubElement(geometry, "box")
                SubElement(box, "size").text = f"{scenario.grid.resolution} {scenario.grid.resolution} 1"
    output.parent.mkdir(parents=True, exist_ok=True)
    ElementTree(sdf).write(output, encoding="unicode", xml_declaration=True)


def export_confirmatory_sdf(
    family: str, layout_index: int, world_seed: int,
    output: Path, catalog_path: Path, friction_seed: int | None = None,
) -> tuple[Path, Path]:
    """Write a complete Gazebo world plus its immutable scenario manifest."""
    scenario = make_confirmatory_scenario(family, layout_index, world_seed)
    disturbance = physical_disturbance(friction_seed)
    catalog = load_catalog(catalog_path).supported_experiment_subset()
    sdf = Element("sdf", version="1.10")
    world = SubElement(sdf, "world", name=scenario.layout_id.replace("-", "_"))
    physics = SubElement(world, "physics", name="physics", type="ignored")
    SubElement(physics, "max_step_size").text = "0.002"
    SubElement(physics, "real_time_factor").text = "1"
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
        ("gz-sim-logical-camera-system", "gz::sim::systems::LogicalCamera"),
    ):
        plugin = SubElement(world, "plugin", filename=filename, name=name)
        if filename == "gz-sim-sensors-system":
            SubElement(plugin, "render_engine").text = "ogre2"
    topology = SubElement(
        world, "plugin", filename="libtopology_joint_system.so",
        name="modular_robot_gz_plugins::TopologyJointSystem")
    SubElement(topology, "parent_link").text = "core::base_link"
    SubElement(topology, "command_topic").text = "/topology_joint/command"
    for pod in sorted(catalog.morphologies["compact_diff"].pod_poses):
        SubElement(topology, "initial_child").text = f"core::{pod}::{pod}_base_link"

    light = SubElement(world, "light", name="sun", type="directional")
    SubElement(light, "pose").text = "0 0 10 0 0 0"
    SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    SubElement(light, "direction").text = "-0.5 0.2 -1"
    ground = SubElement(world, "model", name="ground")
    SubElement(ground, "static").text = "true"
    ground_link = SubElement(ground, "link", name="link")
    ground_collision = SubElement(ground_link, "collision", name="collision")
    ground_geometry = SubElement(ground_collision, "geometry")
    plane = SubElement(ground_geometry, "plane")
    SubElement(plane, "normal").text = "0 0 1"
    SubElement(plane, "size").text = "20 12"
    surface = SubElement(ground_collision, "surface")
    friction = SubElement(surface, "friction")
    ode = SubElement(friction, "ode")
    SubElement(ode, "mu").text = str(disturbance["ground_friction"])
    SubElement(ode, "mu2").text = str(disturbance["ground_friction"])

    for y in range(scenario.grid.height):
        for x in range(scenario.grid.width):
            if scenario.grid.value(x, y) == OCCUPIED:
                wx, wy = scenario.grid.cell_center(x, y)
                _box_model(
                    world, f"wall_{x}_{y}", (wx, wy, 0.5),
                    (scenario.grid.resolution, scenario.grid.resolution, 1.0))
    for box in scenario.transition_obstacles:
        _box_model(world, box.name, box.center, box.size)

    start_x, start_y = scenario.grid.cell_center(*scenario.start)
    _include(world, "model://modular_robot", "core", (
        start_x + disturbance["initial_dx_m"],
        start_y + disturbance["initial_dy_m"], 0.18,
        disturbance["initial_yaw_rad"],
    ))

    output.parent.mkdir(parents=True, exist_ok=True)
    ElementTree(sdf).write(output, encoding="unicode", xml_declaration=True)
    manifest_path = output.with_suffix(".manifest.json")
    manifest_path.write_text(
        json.dumps({**scenario.manifest(), "friction_seed": friction_seed,
                    "physical_disturbance": disturbance},
                   indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return output, manifest_path


def _box_model(world, name, center, size) -> None:
    model = SubElement(world, "model", name=name)
    SubElement(model, "static").text = "true"
    SubElement(model, "pose").text = " ".join(str(value) for value in (*center, 0, 0, 0))
    link = SubElement(model, "link", name="link")
    for tag in ("collision", "visual"):
        item = SubElement(link, tag, name=tag)
        geometry = SubElement(item, "geometry")
        box = SubElement(geometry, "box")
        SubElement(box, "size").text = " ".join(str(value) for value in size)


def _include(world, uri: str, name: str, pose) -> None:
    include = SubElement(world, "include")
    SubElement(include, "uri").text = uri
    SubElement(include, "name").text = name
    x, y, z, yaw = pose
    SubElement(include, "pose").text = f"{x} {y} {z} 0 0 {yaw}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("family", choices=(*FAMILIES, *CONFIRMATORY_FAMILIES))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--layout-index", type=int)
    parser.add_argument(
        "--catalog", type=Path,
        default=Path("src/modular_robot_description/config/morphologies.yaml"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.family in CONFIRMATORY_FAMILIES:
        if args.layout_index is None:
            parser.error("--layout-index is required for a confirmatory family")
        export_confirmatory_sdf(
            args.family, args.layout_index, args.seed, args.output, args.catalog)
    else:
        export_sdf(args.family, args.seed, args.output)


if __name__ == "__main__":
    main()
