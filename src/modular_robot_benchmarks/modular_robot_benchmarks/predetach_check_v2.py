"""predetach_check_v2: fresh seeds and a re-timed planner-level model, tested.

v1 (studies/predetach_check_v1) stopped at its tier-1 interim look: 15 of the
60 refused-layout `feasibility_checked` cells ended in a different outcome
class than predicted (declared limit 6), and the ray_wedge falsifier fired
(7 of 30, limit 3). Most mismatches were 300 s timeouts: in execution a
rejection cycle (check, rejection, replan, drive to the next site) took far
longer than the frozen v1 time model, which ignored start-up and planner
latency. v2 keeps v1's planner-level model (`simulate_mission`: which sites
are checked, which are rejected, which dockings are refused) unchanged and
replaces only the mission-time model by one fitted on v1's executed tier-0/1
`feasibility_checked` records (the calibration data); a checked cell whose
fitted time to its terminal event exceeds the 300 s deadline is predicted
`timeout`. v2 is the test of that model, on fresh seeds:

* fresh arm (confirmatory, H3): the v1 fresh layout ids, 6 cells each, with
  world/plant seeds from a new master seed (new worlds, no selection);
* probe arm (exploratory): `feasibility_probed` (both rules) on the v1
  same-layout and control layout ids, also with seeds from the new master
  seed, so these are new worlds of those families and indices, not the
  confirmatory_v3 worlds.

The v1 design, predictions and records are never modified. See
studies/predetach_check_v2/declaration.md.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import statistics
import sys

from morphology_planner import load_catalog

from .design import PREDETACH_CHECK_DESIGN_KIND, PredetachTrialSpec, StudyDesign, _seed
from . import predetach_check as v1

STUDY_DIR = Path("studies/predetach_check_v2")
MASTER_SEED_V2 = 20261118
DEADLINE_S = v1.DEADLINE_S
CHECKED = ("feasibility_checked", "feasibility_probed")
PROBE_CELLS = (("feasibility_probed", "ray_wedge"), ("feasibility_probed", "radius"))
PROBE_LAYOUTS = v1.REFUSED_LAYOUTS + v1.CONTROL_LAYOUTS
FRESH_LAYOUTS = v1.FRESH_LAYOUTS
# Smoke = run_order 0..3, fixed by rule: one checked and one probed fresh cell,
# one probed cell of each probe-arm family kind (a former refused id, a control).
SMOKE_TRIAL_IDS = (
    "pdc2-docking_observability-21-fresh-feasibility_checked-ray_wedge",
    "pdc2-docking_observability-21-fresh-feasibility_probed-ray_wedge",
    "pdc2-docking_observability-01-probe-feasibility_probed-ray_wedge",
    "pdc2-docking_observability-00-probe-feasibility_probed-radius",
)
TIERS = {0: "smoke", 1: "fresh arm, every cell (confirmatory)",
         2: "probe arm, feasibility_probed on new worlds of the v1 layout ids"}


def trial_id(layout_id: str, arm: str, method: str, rule: str) -> str:
    """v2 ids carry the arm name itself ("fresh" or "probe")."""
    return f"pdc2-{layout_id}-{arm}-{method}" + (f"-{rule}" if rule else "")


# ---------------------------------------------------------------------------
# Timing model, fitted on v1 calibration records


def fit_timing(raw: Path, v1_predictions: dict) -> dict:
    """Medians of the executed event intervals in v1's tier-0/1 checked records."""
    tiers = v1_predictions["tiers"]
    calibration = set(tiers["1"]) | {t for t in tiers["0"] if "feasibility_checked" in t}
    first_check, first_plan, cycle, complete, refuse, noplan = [], [], [], [], [], []
    used = []
    for path in sorted(raw.glob("*.json")):
        payload = path.read_bytes()
        record = json.loads(payload)
        if record["spec"]["trial_id"] not in calibration:
            continue
        used.append([path.name, hashlib.sha256(payload).hexdigest()])
        events = record["controller_diagnostics"]["predetach_checks"]
        checks = [e for e in events if e["event"] == "check"]
        plans = [e for e in events if e["event"] == "plan"]
        final = [e for e in events if e["event"] == "final"]
        if plans:
            first_plan.append(plans[0]["time_s"])
        if checks:
            first_check.append(checks[0]["time_s"])
        for a, b in zip(checks, checks[1:]):
            if not a["passed"]:
                cycle.append(b["time_s"] - a["time_s"])
        if final and checks:
            outcome, last = final[0]["outcome"], checks[-1]["time_s"]
            if outcome == "completed":
                start = (checks[0]["time_s"] if all(c["passed"] for c in checks) else last)
                complete.append(final[0]["time_s"] - start)
            elif outcome == "docking_refused":
                refuse.append(final[0]["time_s"] - last)
            elif outcome == "no_plan":
                noplan.append(final[0]["time_s"] - last)
    med = lambda xs: round(statistics.median(xs), 2)  # noqa: E731
    return {"source": "predetach_check_v1 tier-0/1 feasibility_checked records",
            "records": used, "n_records": len(used),
            "first_check_s": med(first_check), "first_plan_s": med(first_plan),
            "rejection_cycle_s": med(cycle), "completion_tail_s": med(complete),
            "refusal_tail_s": med(refuse), "no_plan_tail_s": med(noplan),
            "n": {"first_check": len(first_check), "rejection_cycle": len(cycle),
                  "completion_tail": len(complete), "refusal_tail": len(refuse),
                  "no_plan_tail": len(noplan)}}


def timed_outcome(cell: dict, timing: dict) -> tuple[str, float | None]:
    """v2 class: v1's class, or `timeout` if the fitted time exceeds the deadline."""
    if cell["method"] not in CHECKED:
        return cell["outcome"], None   # references have no check cycle: class as v1
    k, outcome = cell["rejections"], cell["outcome"]
    first, cyc = timing["first_check_s"], timing["rejection_cycle_s"]
    if outcome == "completed":
        t = first + k * cyc + timing["completion_tail_s"]
    elif outcome == "docking_refused":
        t = first + k * cyc + timing["refusal_tail_s"]
    elif outcome == "attempt_cap":
        t = first + (k - 1) * cyc
    elif outcome == "no_plan":
        t = timing["first_plan_s"] if k == 0 else first + (k - 1) * cyc + timing["no_plan_tail_s"]
    else:
        raise ValueError(outcome)
    return ("timeout" if t > DEADLINE_S else outcome), round(t, 1)


# ---------------------------------------------------------------------------
# Design (fresh seeds for every layout)


def layout_seeds(arm: str, layout_id: str) -> tuple[int, dict]:
    family, index = layout_id.rsplit("-", 1)
    index = int(index)
    parts = (MASTER_SEED_V2, "predetach_check_v2", arm, family, index)
    return _seed(*parts, "world"), {name: _seed(*parts, name.split("_")[0])
                                     for name in ("sensing_seed", "friction_seed", "fault_seed")}


def build_design() -> StudyDesign:
    rows = []
    for arm, layouts, cells in (("fresh", FRESH_LAYOUTS, v1.CELLS),
                                ("probe", PROBE_LAYOUTS, PROBE_CELLS)):
        for layout_id in layouts:
            world_seed, seeds = layout_seeds(arm, layout_id)
            family = layout_id.rsplit("-", 1)[0]
            for method, rule in cells:
                order = v1.CELLS.index((method, rule))
                rows.append(PredetachTrialSpec(
                    trial_id=trial_id(layout_id, arm, method, rule), family=family,
                    layout_id=layout_id, replicate=0, method=method, method_order=order,
                    world_seed=world_seed, exclusion_rule=rule, arm=arm,
                    source_trial_id="", run_order=0, **seeds))
    smoke = {t: i for i, t in enumerate(SMOKE_TRIAL_IDS)}

    def key(spec):
        tier = 0 if spec.trial_id in smoke else (1 if spec.arm == "fresh" else 2)
        return (tier, smoke.get(spec.trial_id, 0), spec.layout_id, spec.method_order)
    ordered = sorted(rows, key=key)
    trials = tuple(replace(s, run_order=i) for i, s in enumerate(ordered))
    return StudyDesign(1, PREDETACH_CHECK_DESIGN_KIND,
                       len(FRESH_LAYOUTS) + len(PROBE_LAYOUTS), 1, MASTER_SEED_V2,
                       tuple(dict.fromkeys(m for m, _ in v1.CELLS)),
                       ("docking_observability", "combined_constraints",
                        "reconfiguration_workspace"), trials)


def tier_of(spec) -> int:
    if spec.trial_id in SMOKE_TRIAL_IDS:
        return 0
    return 1 if spec.arm == "fresh" else 2


def seeds_disjoint(design: StudyDesign, others: list[StudyDesign]) -> bool:
    mine = {t.world_seed for t in design.trials}
    return not any(mine & {t.world_seed for t in o.trials} for o in others)


def build_predictions(design: StudyDesign, catalog, timing: dict, cells=None) -> dict:
    if cells is None:
        cells = v1.predict(design.trials, catalog)
    for spec in design.trials:
        cell = cells[spec.trial_id]
        cell["v1_rule_outcome"] = cell["outcome"]
        cell["outcome"], cell["predicted_time_v2_s"] = timed_outcome(cell, timing)
    by_tier: dict[int, list[str]] = {}
    for spec in design.trials:
        by_tier.setdefault(tier_of(spec), []).append(spec.trial_id)
    summary: dict = {}
    for cell in cells.values():
        k = f'{cell["arm"]}|{cell["method"]}|{cell["rule"]}'
        summary.setdefault(k, {}).setdefault(cell["outcome"], 0)
        summary[k][cell["outcome"]] += 1
    return {"schema_version": 1,
            "purpose": "predetach_check_v2 predictions (re-timed model), generated before any v2 trial",
            "design_hash": design.design_hash, "deadline_s": DEADLINE_S,
            "timing_model": {k: v for k, v in timing.items() if k != "records"},
            "smoke_trial_ids": list(SMOKE_TRIAL_IDS),
            "tiers": {str(k): v for k, v in sorted(by_tier.items())},
            "tier_meaning": {str(k): v for k, v in TIERS.items()},
            "summary": summary, "cells": cells}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--v1-raw", type=Path, help="v1 raw records (calibration data)")
    p.add_argument("--v1-dir", type=Path, default=v1.STUDY_DIR)
    p.add_argument("--catalog", type=Path, default=v1.DEFAULT_CATALOG)
    p.add_argument("--confirmatory-v3", type=Path,
                   default=Path("studies/confirmatory_v3/design.json"))
    p.add_argument("--out", type=Path, default=STUDY_DIR)
    p.add_argument("--cells", type=Path, help="precomputed planner cells (json)")
    p.add_argument("--fit-only", action="store_true")
    p.add_argument("--design-only", action="store_true")
    p.add_argument("--layouts", help="comma list: predict only these layout ids (to --cells)")
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    design = build_design()
    if a.design_only:
        missing = set(SMOKE_TRIAL_IDS) - {t.trial_id for t in design.trials}
        if missing:
            raise ValueError(f"smoke ids not in the design: {sorted(missing)}")
        others = [StudyDesign.read_frozen(a.v1_dir / "design.json")]
        if a.confirmatory_v3.exists():
            others.append(StudyDesign.read_frozen(a.confirmatory_v3))
        if not seeds_disjoint(design, others):
            raise ValueError("v2 world seeds collide with a frozen design")
        design.write_frozen(a.out / "design.json")
        (a.out / "smoke_trial_ids.json").write_text(json.dumps(list(SMOKE_TRIAL_IDS), indent=1) + "\n")
        print(json.dumps({"design_hash": design.design_hash, "trials": len(design.trials)}))
        return
    if a.fit_only:
        v1p = json.loads((a.v1_dir / "predictions.json").read_text())
        timing = fit_timing(a.v1_raw, v1p)
        (a.out / "timing_calibration.json").write_text(json.dumps(timing, indent=1, sort_keys=True) + "\n")
        print(json.dumps({k: v for k, v in timing.items() if k != "records"}, sort_keys=True))
        return
    catalog = load_catalog(str(a.catalog)).supported_experiment_subset()
    if a.layouts:
        rows = [t for t in design.trials if t.layout_id in a.layouts.split(",")]
        cells = v1.predict(rows, catalog)
        a.cells.write_text(json.dumps(cells, sort_keys=True))
        return
    timing = json.loads((a.out / "timing_calibration.json").read_text())
    cells = json.loads(a.cells.read_text()) if a.cells else None
    preds = build_predictions(design, catalog, timing, cells)
    (a.out / "predictions.json").write_text(json.dumps(preds, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"design_hash": design.design_hash, "summary": preds["summary"]}, indent=1, sort_keys=True))


if __name__ == "__main__":
    main()
