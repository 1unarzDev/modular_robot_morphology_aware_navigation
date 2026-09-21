import json
from pathlib import Path

from modular_robot_benchmarks import manipulation_check
from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from modular_robot_benchmarks.design import generate_design
from modular_robot_benchmarks.manipulation_check import (
    DECLARED_CONTRASTS, METHOD_ORDER, layout_seeds, method_decision,
    separation_report,
)
from morphology_planner import load_catalog


ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "src/modular_robot_description/config/morphologies.yaml"


def test_layout_seeds_collapse_replicates_to_one_environment_per_layout():
    design = generate_design(layouts_per_family=3, replicates=3)
    seeds = layout_seeds(design, "combined_constraints")
    assert [index for index, _ in seeds] == [0, 1, 2]
    assert len({seed for _, seed in seeds}) == 3


def test_declared_contrasts_match_the_statistical_analysis_plan():
    """The check must test the contrasts the plan declares, not others."""
    plan = (ROOT / "docs/research/statistical_analysis_plan.md").read_text(
        encoding="utf-8")
    assert "`sensing_feasibility_coupled` minus `geometry_coupled` over all families" in plan
    assert "`sensing_feasibility_coupled` minus `feasibility_coupled` in docking" in plan
    assert DECLARED_CONTRASTS[0][:2] == ("sensing_feasibility_coupled", "geometry_coupled")
    assert DECLARED_CONTRASTS[0][2] is None
    assert DECLARED_CONTRASTS[1][:2] == ("sensing_feasibility_coupled", "feasibility_coupled")
    assert set(DECLARED_CONTRASTS[1][2]) == {"docking_observability", "combined_constraints"}


def _stub_decisions(monkeypatch, sites):
    """Replace search with declared per-method decisions. Search itself is
    covered by the golden fixture test; this exercises the scoping and
    counting this module adds on top of it."""
    def fake(method, scenario, catalog, heading_bins, epsilon):
        site = sites[method]
        return {"method": method, "planned": site is not None,
                "route_signature": f"route-{site}", "transition_site": site,
                "rejection_reasons": []}
    monkeypatch.setattr(manipulation_check, "method_decision", fake)
    monkeypatch.setattr(manipulation_check, "load_catalog",
                        lambda path: type("C", (), {"supported_experiment_subset": lambda s: None})())
    monkeypatch.setattr(manipulation_check, "make_confirmatory_scenario",
                        lambda family, index, seed: None)


def test_report_covers_every_method_and_counts_layouts_that_separate(monkeypatch):
    _stub_decisions(monkeypatch, {
        "route_first_adaptation": [1, 1],
        "geometry_coupled": [2, 2],
        "feasibility_coupled": [3, 3],
        "sensing_feasibility_coupled": [3, 3],
    })
    design = generate_design(layouts_per_family=4, replicates=2)
    report = separation_report(design, ("combined_constraints",), CATALOG)
    assert report["design_hash"] == design.design_hash
    entry = report["layouts"][0]
    assert set(entry["decisions"]) == set(METHOD_ORDER)
    assert len(report["layouts"]) == 4

    geometry = report["contrast_summary"][
        "sensing_feasibility_coupled_vs_geometry_coupled"]["combined_constraints"]
    feasibility = report["contrast_summary"][
        "sensing_feasibility_coupled_vs_feasibility_coupled"]["combined_constraints"]
    # Sensing differs from geometry everywhere, and from feasibility nowhere:
    # the second contrast is unidentifiable in this layout set.
    assert geometry == {"layouts": 4, "site_separating": 4,
                        "route_separating": 4, "unplanned": 0}
    assert feasibility == {"layouts": 4, "site_separating": 0,
                           "route_separating": 0, "unplanned": 0}
    json.dumps(report)


def test_feasibility_contrast_is_not_reported_outside_its_declared_scope(monkeypatch):
    _stub_decisions(monkeypatch, {method: [1, 1] for method in METHOD_ORDER})
    design = generate_design(layouts_per_family=1, replicates=1)
    report = separation_report(design, ("reconfiguration_workspace",), CATALOG)
    assert set(report["layouts"][0]["separates"]) == {
        "sensing_feasibility_coupled_vs_geometry_coupled"}
    assert "sensing_feasibility_coupled_vs_feasibility_coupled" not in report["contrast_summary"]


def test_unplanned_layouts_are_counted_rather_than_silently_separating(monkeypatch):
    _stub_decisions(monkeypatch, {
        "route_first_adaptation": [1, 1], "geometry_coupled": [2, 2],
        "feasibility_coupled": [3, 3], "sensing_feasibility_coupled": None,
    })
    design = generate_design(layouts_per_family=2, replicates=1)
    report = separation_report(design, ("combined_constraints",), CATALOG)
    for summary in report["contrast_summary"].values():
        assert summary["combined_constraints"]["unplanned"] == 2
        assert summary["combined_constraints"]["site_separating"] == 0


def test_method_decision_runs_the_real_planner_and_reports_a_site():
    """One real call, so the stubs above cannot drift from the planner API."""
    catalog = load_catalog(str(CATALOG)).supported_experiment_subset()
    scenario = make_confirmatory_scenario("docking_observability", 1, 8)
    decision = method_decision("feasibility_coupled", scenario, catalog,
                               heading_bins=4, epsilon=3.0)
    assert decision["planned"] and decision["method"] == "feasibility_coupled"
    assert len(decision["transition_site"]) == 2
    assert decision["route_signature"]
