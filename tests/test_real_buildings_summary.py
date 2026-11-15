"""The real-buildings report renders into the paper's macros end to end.

Synthetic entries in the shape `real_buildings._worker` writes, so a schema
drift between the study and `make_exploratory_macros.py` fails here rather
than when the report first exists.
"""
import importlib.util
import re
from pathlib import Path

import pytest

from modular_robot_benchmarks.real_buildings import summarize


def _site(feasible, stages=()):
    return {"transition": "compact_to_narrow", "site": [1, 1, 0],
            "full_test_feasible": feasible, "failed_stages": list(stages)}


def _mission(geometry, feasibility, treatment=True):
    methods = {"geometry_coupled": {"planned": True, "sites": geometry},
               "feasibility_coupled": ({"planned": True, "sites": feasibility}
                                       if feasibility is not None
                                       else {"planned": False, "error": "NoPathError"}),
               "sensing_feasibility_coupled": ({"planned": True, "sites": [_site(True)]}
                                               if treatment
                                               else {"planned": False,
                                                     "error": "NoPathError"})}
    return {"result": {"methods": methods}}


BUILDINGS = [
    {"building": "a", "missions": [
        _mission([_site(False, ["observability"])], [_site(False, ["observability"])]),
        _mission([_site(True)], [_site(True)])]},
    {"building": "b", "missions": [
        _mission([_site(False, ["module_trajectory", "verification"])], [_site(True)],
                 treatment=False)]},
    {"building": "c", "missions": [_mission([_site(True)], None)]},
    {"building": "d", "missions": []},
    {"building": "e", "error": "ValueError: bad mesh", "draw": None, "missions": []},
]


def test_real_buildings_summary_counts_first_and_all_missions():
    summary = summarize(BUILDINGS, bootstrap=200)
    assert (summary["buildings"], summary["buildings_with_mission"], summary["missions"]) \
        == (5, 3, 4)
    geometry = summary["geometry_coupled"]
    assert geometry["first_mission"]["with_transition"] == 3
    assert geometry["first_mission"]["rejected"] == 2
    assert geometry["all_missions"]["with_transition"] == 4
    assert geometry["all_missions"]["rejected"] == 2
    assert geometry["rejected_site_stages"] == {
        "observability": 1, "module_trajectory": 1, "verification": 1}
    assert geometry["rejected_and_treatment_plans"] == 1
    assert geometry["unplanned"] == 0
    low, high = geometry["all_missions"]["building_bootstrap95"]
    assert 0 <= low <= 0.5 <= high <= 1
    feasibility = summary["feasibility_coupled"]
    # building c's feasibility_coupled did not plan: no site, so no denominator.
    assert feasibility["first_mission"]["with_transition"] == 2
    assert feasibility["first_mission"]["rejected"] == 1
    assert feasibility["all_missions"]["with_transition"] == 3
    assert feasibility["unplanned"] == 1
    assert summary["sensing_feasibility_coupled"]["unplanned"] == 1


def test_real_buildings_report_renders_every_macro_the_paper_uses():
    root = Path(__file__).resolve().parents[1]
    path = root / "paper" / "conference" / "make_exploratory_macros.py"
    spec = importlib.util.spec_from_file_location("make_exploratory_macros", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    report = {"summary": summarize(BUILDINGS, bootstrap=200), "results": BUILDINGS}
    tex = module.real_buildings_macros(report)
    defined = set(re.findall(r"\\newcommand\{\\(\w+)\}", tex))
    used = set(re.findall(r"\\(real[A-Z]\w*)",
                          (root / "paper" / "conference" / "main.tex").read_text(encoding="utf-8")))
    assert used and used <= defined, sorted(used - defined)
    assert r"\newcommand{\realErrors}{1}" in tex
    assert r"\newcommand{\realGeomFirstPct}{66.7\%}" in tex
    assert r"\newcommand{\realGeomStages}{module trajectory 1, verification maneuver 1, " \
           r"observability 1}" in tex
    assert "None" not in tex and "nan" not in tex


@pytest.mark.parametrize("missing", ["geometry_coupled", "feasibility_coupled"])
def test_real_buildings_summary_survives_a_control_with_no_transition_anywhere(missing):
    buildings = [{"building": "x", "missions": [_mission([_site(True)], [_site(True)])]}]
    buildings[0]["missions"][0]["result"]["methods"][missing] = {"planned": True, "sites": []}
    entry = summarize(buildings, bootstrap=50)[missing]
    assert entry["first_mission"]["rejected_fraction"] is None
    assert entry["all_missions"]["building_bootstrap95"] is None
