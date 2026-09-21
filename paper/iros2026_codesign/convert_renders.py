"""Copy Gazebo overview frames from the WSL workspace and convert them to PNG.

Usage: python convert_renders.py            (static + mission frames)
Renders are visualization only and never enter study records.
"""
import shutil
from pathlib import Path

from PIL import Image

WSL = Path(r"\\wsl.localhost\Ubuntu\home\checker\morphology_ws\results\debug")
OUT = Path(__file__).resolve().parent / "renders"
OUT.mkdir(exist_ok=True)
sources = [(WSL / "render_static" / "capture", "static_"), (WSL / "render_mission" / "frames", "mission_")]
count = 0
for directory, prefix in sources:
    if not directory.exists():
        print("missing", directory)
        continue
    for ppm in sorted(directory.glob("*.ppm")):
        target = OUT / f"{prefix}{ppm.stem}.png"
        if not target.exists():
            Image.open(ppm).save(target)
            count += 1
print("converted", count, "frames into", OUT)
