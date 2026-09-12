import sys
from pathlib import Path

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "src" / "morphology_planner"))
sys.path.insert(0, str(ROOT / "src" / "modular_robot_bringup"))
sys.path.insert(0, str(ROOT / "src" / "reconfiguration_executor"))
sys.path.insert(0, str(ROOT / "src" / "modular_robot_benchmarks"))
