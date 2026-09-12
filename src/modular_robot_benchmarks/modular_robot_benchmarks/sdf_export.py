from __future__ import annotations

import argparse
from pathlib import Path
from xml.etree.ElementTree import Element, ElementTree, SubElement

from morphology_planner.grid import OCCUPIED

from .scenarios import FAMILIES, make_scenario


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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("family", choices=FAMILIES)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export_sdf(args.family, args.seed, args.output)


if __name__ == "__main__":
    main()

