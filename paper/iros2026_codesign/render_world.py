"""Visualization only: add overview cameras and materials to a generated world.

Usage: render_world.py <world_in> <world_out> [robot_x robot_y robot_yaw]
Also importable: inject(tree_root, site_xy) for the mission recorder.
Cameras publish /overview/<name>; no autonomy node subscribes to them.
"""
import sys
from math import atan2, hypot
from pathlib import Path
from xml.etree.ElementTree import ElementTree, SubElement, parse

WALL_RGBA = "0.55 0.58 0.63 1"
POST_RGBA = "0.90 0.22 0.18 1"


def _material(model, rgba):
    for visual in model.iter("visual"):
        material = visual.find("material")
        if material is None:
            material = SubElement(visual, "material")
        for tag in ("ambient", "diffuse"):
            element = material.find(tag)
            if element is None:
                element = SubElement(material, tag)
            element.text = rgba


def cameras_for(site_x, site_y):
    return {
        # name: (eye xyz, target xyz, horizontal fov)
        "oblique": ((site_x - 0.55, site_y - 1.25, 1.05), (site_x + 0.2, site_y + 0.15, 0.05), 1.05),
        "close": ((site_x + 0.25, site_y - 0.55, 0.62), (site_x + 0.3, site_y + 0.38, 0.04), 1.2),
        "top": ((site_x + 0.2, site_y, 2.4), (site_x + 0.2, site_y + 0.001, 0.0), 1.2),
    }


def inject(root, site_xy, robot_pose=None):
    world = root.find("world")
    if robot_pose is not None:
        for include in world.findall("include"):
            if include.findtext("uri") == "model://modular_robot":
                include.find("pose").text = f"{robot_pose[0]} {robot_pose[1]} 0.18 0 0 {robot_pose[2]}"
    for model in world.findall("model"):
        name = model.get("name", "")
        if name.startswith("wall_"):
            _material(model, WALL_RGBA)
        elif name == "transition_post":
            _material(model, POST_RGBA)
    for light in world.findall("light"):
        light.find("diffuse").text = "0.9 0.9 0.9 1"
    scene = world.find("scene") or SubElement(world, "scene")
    SubElement(scene, "ambient").text = "0.6 0.6 0.62 1"
    SubElement(scene, "background").text = "0.95 0.96 0.97 1"
    SubElement(scene, "shadows").text = "true"
    for name, (eye, target, fov) in cameras_for(*site_xy).items():
        dx, dy, dz = (t - e for t, e in zip(target, eye))
        model = SubElement(world, "model", name=f"overview_camera_{name}")
        SubElement(model, "static").text = "true"
        SubElement(model, "pose").text = (
            f"{eye[0]} {eye[1]} {eye[2]} 0 {atan2(-dz, hypot(dx, dy))} {atan2(dy, dx)}")
        link = SubElement(model, "link", name="link")
        sensor = SubElement(link, "sensor", name=f"overview_{name}", type="camera")
        SubElement(sensor, "always_on").text = "true"
        SubElement(sensor, "update_rate").text = "2"
        SubElement(sensor, "topic").text = f"/overview/{name}"
        camera = SubElement(sensor, "camera")
        SubElement(camera, "horizontal_fov").text = str(fov)
        image = SubElement(camera, "image")
        SubElement(image, "width").text = "1600"
        SubElement(image, "height").text = "900"
        SubElement(image, "format").text = "R8G8B8"
        clip = SubElement(camera, "clip")
        SubElement(clip, "near").text = "0.03"
        SubElement(clip, "far").text = "30"
    return root


if __name__ == "__main__":
    world_in, world_out = sys.argv[1:3]
    pose = tuple(float(v) for v in sys.argv[3:6]) if len(sys.argv) >= 6 else None
    tree = parse(world_in)
    site = (pose[0], pose[1]) if pose else (2.15, 1.75)
    inject(tree.getroot(), site, pose)
    Path(world_out).parent.mkdir(parents=True, exist_ok=True)
    ElementTree(tree.getroot()).write(world_out, encoding="unicode", xml_declaration=True)
    print("wrote", world_out)
