"""Run the real-buildings study on the Windows host, with the repo packages on sys.path.

Resumable (it keeps a per-building checkpoint beside the output). The declared
run is given in studies/real_buildings/declaration.md.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for pkg in ("morphology_planner", "modular_robot_bringup", "reconfiguration_executor",
            "modular_robot_benchmarks", "morphology_manager"):
    sys.path.insert(0, str(ROOT / "src" / pkg))

from modular_robot_benchmarks.real_buildings import main  # noqa: E402

if __name__ == "__main__":
    main()
