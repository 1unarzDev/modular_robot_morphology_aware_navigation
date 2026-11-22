"""Identity tests for predetach_check_v2 (design, seeds, re-timed model)."""
import json
from pathlib import Path

from modular_robot_benchmarks import predetach_check_v2 as v2
from modular_robot_benchmarks.design import StudyDesign

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "studies" / "predetach_check_v2"


def test_design_reproduces_frozen_file():
    frozen = StudyDesign.read_frozen(STUDY / "design.json")
    assert v2.build_design().design_hash == frozen.design_hash
    assert len(frozen.trials) == 132


def test_seeds_disjoint_from_v1_and_confirmatory_v3():
    others = [StudyDesign.read_frozen(ROOT / "studies/predetach_check_v1/design.json"),
              StudyDesign.read_frozen(ROOT / "studies/confirmatory_v3/design.json")]
    assert v2.seeds_disjoint(v2.build_design(), others)


def test_smoke_first_and_arms():
    trials = v2.build_design().trials
    assert [t.trial_id for t in trials[:4]] == list(v2.SMOKE_TRIAL_IDS)
    assert {t.arm for t in trials} == {"fresh", "probe"}
    assert all(t.trial_id.startswith("pdc2-") for t in trials)


def test_timed_outcome_rules():
    timing = json.loads((STUDY / "timing_calibration.json").read_text())
    cell = {"method": "feasibility_checked", "outcome": "completed", "rejections": 0}
    assert v2.timed_outcome(cell, timing)[0] == "completed"          # 50.3 + 129.1 < 300
    cell = {"method": "feasibility_checked", "outcome": "completed", "rejections": 2}
    assert v2.timed_outcome(cell, timing)[0] == "timeout"            # 50.3 + 161 + 129.1 > 300
    cell = {"method": "feasibility_coupled", "outcome": "docking_refused", "rejections": 0}
    assert v2.timed_outcome(cell, timing) == ("docking_refused", None)


def test_predictions_match_design_and_model():
    preds = json.loads((STUDY / "predictions.json").read_text())
    timing = json.loads((STUDY / "timing_calibration.json").read_text())
    assert preds["design_hash"] == v2.build_design().design_hash
    for cell in preds["cells"].values():
        base = dict(cell, outcome=cell["v1_rule_outcome"])
        assert v2.timed_outcome(base, timing)[0] == cell["outcome"]


def test_planner_cells_recompute_from_scratch():
    """Non-circular check: re-run the planner-level model for two cells and compare with the stored cells."""
    from morphology_planner import load_catalog
    from modular_robot_benchmarks import predetach_check as v1
    preds = json.loads((STUDY / "predictions.json").read_text())
    catalog = load_catalog(str(ROOT / "src/modular_robot_description/config/morphologies.yaml")
                           ).supported_experiment_subset()
    wanted = ("pdc2-docking_observability-21-fresh-feasibility_checked-ray_wedge",
              "pdc2-docking_observability-00-probe-feasibility_probed-radius")
    rows = [t for t in v2.build_design().trials if t.trial_id in wanted]
    cells = v1.predict(rows, catalog)
    for tid in wanted:
        got, stored = cells[tid], preds["cells"][tid]
        for key in ("outcome", "rejections", "planning_calls", "planned_drive_m"):
            want = stored["v1_rule_outcome"] if key == "outcome" else stored[key]
            assert got[key] == want, (tid, key, got[key], want)


def test_job_lists_exclude_canary_from_main():
    import importlib.util
    spec = importlib.util.spec_from_file_location("pdc2_make_jobs", STUDY / "make_jobs.py")
    jobs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(jobs)
    main, canary = jobs.job_lists("0" * 40)
    main_seeds = [s for j in main for s in j["seeds"]]
    canary_seeds = sorted(s for j in canary for s in j["seeds"])
    assert 2 not in main_seeds and 3 not in main_seeds
    assert canary_seeds == [2, 3]
    assert sorted(main_seeds) == [s for s in range(132) if s not in (2, 3)]
    assert main_seeds == sorted(main_seeds)                      # run order kept
    assert all(j["needs"] == {"host": "innovation"} for j in main + canary)
    assert {j["campaign"] for j in canary} == {"predetach_check_v2_canary"}
    assert {j["campaign"] for j in main} == {"predetach_check_v2"}
