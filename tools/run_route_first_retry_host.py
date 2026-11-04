"""Run the route-first retry check on the Windows host, with the repo packages on sys.path.

    .venv-host\\Scripts\\python tools\\run_route_first_retry_host.py --sample tuning \\
        --output studies\\route_first_retry\\tuning.json

The declared run (`studies/route_first_retry/declaration.md`) uses `--sample design`.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for pkg in ("morphology_planner", "modular_robot_bringup", "reconfiguration_executor",
            "modular_robot_benchmarks", "morphology_manager"):
    sys.path.insert(0, str(ROOT / "src" / pkg))

from modular_robot_benchmarks.route_first_retry import main  # noqa: E402

if __name__ == "__main__":
    main()
