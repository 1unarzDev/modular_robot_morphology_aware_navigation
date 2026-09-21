"""Gate 2 manipulation check over the layouts the frozen design actually draws.

An ablation that changes no planner decision cannot identify the contrast it
exists to support. The roadmap records that `combined_constraints` separated on
only 4 of 12 sampled seeds once the post-transition verification maneuver
entered the transition model, so whether the family still manipulates has to be
measured on the layouts the frozen design draws rather than on one golden
fixture.

Scope, which bounds what a passing report means. This exercises the planner
against each scenario's declared location-dependent observability, because that
is the manipulation the design assumes. Missions do not yet supply it:
`morphology_planner.ros_node` feeds the planner live `RelativePoseEstimate`
sensing instead, and location-dependent perceived observability is an open
roadmap item. A family separating here is therefore a necessary condition for
its contrast, not evidence that a mission would exhibit it.

This is a manipulation check and never an outcome. It reports planner decisions
only, receives no significance test, and must not be used to choose families,
layouts, or seeds after outcome data exists.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from morphology_planner import (
    HybridState, load_catalog, make_method_planner, plan_signature,
)

from .confirmatory_scenarios import make_confirmatory_scenario
from .design import StudyDesign


DEFAULT_CATALOG = Path("src/modular_robot_description/config/morphologies.yaml")
QUALIFYING_TRANSITION = "compact_to_narrow"

# The two contrasts the statistical analysis plan declares, as
# (treatment, control, families). A contrast is only identifiable in a layout
# where the two methods decide differently.
DECLARED_CONTRASTS = (
    ("sensing_feasibility_coupled", "geometry_coupled", None),
    ("sensing_feasibility_coupled", "feasibility_coupled",
     ("docking_observability", "combined_constraints")),
)

METHOD_ORDER = ("route_first_adaptation", "geometry_coupled",
                "feasibility_coupled", "sensing_feasibility_coupled")


def layout_seeds(design: StudyDesign, family: str) -> list[tuple[int, int]]:
    """Distinct (layout index, world seed) pairs the design draws for a family.

    Replicates share a layout's world seed, so the environment is one sampling
    unit regardless of how many trials reference it.
    """
    seeds: dict[int, int] = {}
    for spec in design.trials:
        if spec.family != family:
            continue
        index = int(spec.layout_id.rsplit("-", 1)[1])
        seed = int(spec.world_seed)
        if seeds.setdefault(index, seed) != seed:
            raise ValueError(
                f"{family} layout {index} draws more than one world seed")
    return sorted(seeds.items())


def method_decision(method: str, scenario, catalog, heading_bins: int,
                    epsilon: float) -> dict:
    """One method's route and transition-site decision for one layout."""
    planner = make_method_planner(
        method, catalog, scenario.grid, heading_bins=heading_bins,
        environment=scenario.transition_obstacles,
        sensing_provider=scenario.sensing_for,
    )
    try:
        plan = planner.plan(
            HybridState(*scenario.start, 0, "compact_diff"), scenario.goal, epsilon)
    except Exception as error:  # planner raises NoPathError and ValueError
        return {"method": method, "planned": False,
                "error": f"{type(error).__name__}: {error}"}
    segment = next((value for value in plan.segments
                    if value.transition_id == QUALIFYING_TRANSITION), None)
    return {
        "method": method,
        "planned": True,
        "route_signature": plan_signature(plan),
        "transition_site": ([segment.source.x, segment.source.y]
                            if segment is not None else None),
        "rejection_reasons": sorted({
            reason
            for decision in planner.transition_policy.decisions
            if not decision.feasible
            for reason in decision.reasons
        }),
    }


def _contrast_applies(control: str, families: tuple[str, ...] | None,
                      family: str) -> bool:
    return families is None or family in families


def separation_report(design: StudyDesign, families: tuple[str, ...],
                      catalog_path: Path = DEFAULT_CATALOG,
                      heading_bins: int = 8, epsilon: float = 2.5) -> dict:
    """Per-layout decisions for every method, and which contrasts separate.

    `heading_bins` and `epsilon` default to what `morphology_planner.ros_node`
    uses for missions, so the report predicts the decision a mission would
    reach from the same inputs.
    """
    catalog = load_catalog(str(catalog_path)).supported_experiment_subset()
    layouts = []
    for family in families:
        for index, world_seed in layout_seeds(design, family):
            scenario = make_confirmatory_scenario(family, index, world_seed)
            decisions = {
                method: method_decision(method, scenario, catalog, heading_bins, epsilon)
                for method in METHOD_ORDER
            }
            separates = {}
            for treatment, control, scoped in DECLARED_CONTRASTS:
                if not _contrast_applies(control, scoped, family):
                    continue
                left, right = decisions[treatment], decisions[control]
                if not (left["planned"] and right["planned"]):
                    separates[f"{treatment}_vs_{control}"] = None
                    continue
                separates[f"{treatment}_vs_{control}"] = {
                    "site_differs": left["transition_site"] != right["transition_site"],
                    "route_differs": left["route_signature"] != right["route_signature"],
                }
            layouts.append({
                "family": family, "layout_index": index, "world_seed": world_seed,
                "decisions": decisions, "separates": separates,
            })

    summary = {}
    for treatment, control, scoped in DECLARED_CONTRASTS:
        key = f"{treatment}_vs_{control}"
        scope = [entry for entry in layouts
                 if _contrast_applies(control, scoped, entry["family"])]
        if not scope:
            continue
        per_family: dict[str, dict] = {}
        for entry in scope:
            counts = per_family.setdefault(
                entry["family"], {"layouts": 0, "site_separating": 0,
                                  "route_separating": 0, "unplanned": 0})
            counts["layouts"] += 1
            verdict = entry["separates"].get(key)
            if verdict is None:
                counts["unplanned"] += 1
                continue
            counts["site_separating"] += int(verdict["site_differs"])
            counts["route_separating"] += int(verdict["route_differs"])
        summary[key] = per_family
    return {
        "schema_version": 1,
        "purpose": "gate2_planner_manipulation_check",
        "design_hash": design.design_hash,
        "heading_bins": heading_bins,
        "epsilon": epsilon,
        "sensing": "scenario_declared_location_dependent_observability",
        "sensing_caveat": (
            "Missions feed the planner live RelativePoseEstimate sensing, not "
            "this provider; separation here is necessary but not sufficient."),
        "contrast_summary": summary,
        "layouts": layouts,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Gate 2 planner manipulation check over a frozen design")
    parser.add_argument("--design", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--family", action="append", dest="families",
                        default=None, help="repeatable; defaults to the two "
                                           "sensing-contrast families")
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--heading-bins", type=int, default=8)
    parser.add_argument("--epsilon", type=float, default=2.5)
    return parser


def main() -> None:
    args = _parser().parse_args()
    design = StudyDesign.read_frozen(args.design)
    families = tuple(args.families
                     or ("docking_observability", "combined_constraints"))
    report = separation_report(design, families, args.catalog,
                               args.heading_bins, args.epsilon)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(json.dumps(report["contrast_summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
