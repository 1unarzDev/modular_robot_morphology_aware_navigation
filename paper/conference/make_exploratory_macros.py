"""Macros: the exploratory planner-only studies, rendered from their reports.

None enters the confirmatory analysis. Each reports exactly what its
declaration fixes (`studies/prevalence/declaration.md`,
`studies/route_first_retry/declaration.md`,
`studies/real_buildings/declaration.md`), and the paper's sentences about
them were written before each result existed, so no number is transcribed
by hand. A report that does not exist yet leaves its macros undefined and the
paper prints its pending marker.

    python paper/conference/make_exploratory_macros.py \
        studies/prevalence/prevalence_20261104.json \
        studies/route_first_retry/design.json \
        paper/conference/exploratory_macros.tex \
        studies/real_buildings/real_buildings_20261115.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path
from statistics import median

# A rejected site can fail several stages, so these counts may sum past it.
STAGES = (("module_trajectory", "module trajectory"),
          ("verification", "verification maneuver"),
          ("observability", "observability"),
          ("planar", "planar"))
CONTROLS = (("geometry_coupled", "Geom"), ("feasibility_coupled", "Feas"))


def _macro(name: str, value) -> str:
    return f"\\newcommand{{\\{name}}}{{{value}}}\n"


def _percent(value: float) -> str:
    return f"{100 * value:.1f}\\%"


def _stages(counts: dict) -> str:
    parts = [f"{label} {counts[key]}" for key, label in STAGES if counts.get(key)]
    return ", ".join(parts) if parts else "none"


def prevalence_macros(report: dict) -> str:
    summary = report["summary"]
    out = _macro("prevLayouts", summary["layouts"])
    out += _macro("prevTreatUnplanned", summary["sensing_feasibility_coupled"]["unplanned"])
    for control, tag in CONTROLS:
        entry = summary[control]
        sited = entry["planned_with_transition"]
        out += _macro(f"prev{tag}Sited", sited)
        out += _macro(f"prev{tag}Rejected", entry["site_rejected_by_full_test"])
        out += _macro(f"prev{tag}Unplanned", entry["unplanned"])
        out += _macro(f"prev{tag}TreatPlans", entry["rejected_and_treatment_plans"])
        out += _macro(f"prev{tag}Stages", _stages(entry["rejected_by_stage"]))
        if sited:
            low, high = entry["rejected_fraction_wilson95"]
            out += _macro(f"prev{tag}Pct", _percent(entry["rejected_fraction"]))
            out += _macro(f"prev{tag}CI", f"[{_percent(low)}, {_percent(high)}]")
        else:
            out += _macro(f"prev{tag}Pct", "n/a")
            out += _macro(f"prev{tag}CI", "n/a")
    return out


def retry_macros(report: dict) -> str:
    summary, parameters = report["summary"], report["parameters"]
    within = {int(k): v for k, v in summary["planned_within_k_routes"].items()}
    ratios = summary["cost_ratio_to_treatment"]
    out = _macro("retryLayouts", summary["layouts"])
    out += _macro("retryK", parameters["max_routes"])
    out += _macro("retryRadius", parameters["penalty_radius"])
    out += _macro("retryWeight", f"{parameters['penalty_weight']:g}")
    out += _macro("retryPlannedOne", within[1])
    out += _macro("retryPlannedK", within[parameters["max_routes"]])
    out += _macro("retryTreatPlanned", summary["treatment_planned"])
    out += _macro("retryBoth", summary["both_planned"])
    out += _macro("retrySameSite", summary["same_site_as_treatment"])
    out += _macro("retryCostMedian", f"{median(ratios):.2f}" if ratios else "n/a")
    return out


def real_buildings_macros(report: dict) -> str:
    summary = report["summary"]
    errors = sum("error" in building for building in report["results"])
    out = _macro("realBuildings", summary["buildings"])
    out += _macro("realWithMission", summary["buildings_with_mission"])
    out += _macro("realMissions", summary["missions"])
    out += _macro("realErrors", errors)
    out += _macro("realTreatUnplanned", summary["sensing_feasibility_coupled"]["unplanned"])
    for control, tag in CONTROLS:
        entry = summary[control]
        first, every = entry["first_mission"], entry["all_missions"]
        out += _macro(f"real{tag}FirstSited", first["with_transition"])
        out += _macro(f"real{tag}FirstRejected", first["rejected"])
        out += _macro(f"real{tag}AllSited", every["with_transition"])
        out += _macro(f"real{tag}AllRejected", every["rejected"])
        out += _macro(f"real{tag}TreatPlans", entry["rejected_and_treatment_plans"])
        out += _macro(f"real{tag}Unplanned", entry["unplanned"])
        out += _macro(f"real{tag}Stages", _stages(entry["rejected_site_stages"]))
        for name, value, interval in (
                ("First", first["rejected_fraction"], first["wilson95"]),
                ("All", every["rejected_fraction"], every["building_bootstrap95"])):
            out += _macro(f"real{tag}{name}Pct", _percent(value) if value is not None else "n/a")
            out += _macro(f"real{tag}{name}CI",
                          f"[{_percent(interval[0])}, {_percent(interval[1])}]"
                          if interval else "n/a")
    return out


def main() -> None:
    prevalence, retry, destination = (Path(arg) for arg in sys.argv[1:4])
    real = Path(sys.argv[4]) if len(sys.argv) > 4 else None
    text = "% Generated by make_exploratory_macros.py; do not edit.\n"
    if prevalence.exists():
        text += prevalence_macros(json.loads(prevalence.read_text(encoding="utf-8")))
    if retry.exists():
        text += retry_macros(json.loads(retry.read_text(encoding="utf-8")))
    if real is not None and real.exists():
        text += real_buildings_macros(json.loads(real.read_text(encoding="utf-8")))
    destination.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
