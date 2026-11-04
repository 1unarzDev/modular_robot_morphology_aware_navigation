"""Run the prevalence study on the Windows host, with the repo packages on sys.path.

Resumable (it keeps a per-layout checkpoint beside the output). The declared run:

    .venv-host\\Scripts\\python tools\\run_prevalence_host.py --master-seed 20261104 \\
        --layouts 300 --workers 3 --output studies\\prevalence\\prevalence_20261104.json
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for pkg in ("morphology_planner", "modular_robot_bringup", "reconfiguration_executor",
            "modular_robot_benchmarks", "morphology_manager"):
    sys.path.insert(0, str(ROOT / "src" / pkg))

from modular_robot_benchmarks.prevalence import main  # noqa: E402

if __name__ == "__main__":
    main()
