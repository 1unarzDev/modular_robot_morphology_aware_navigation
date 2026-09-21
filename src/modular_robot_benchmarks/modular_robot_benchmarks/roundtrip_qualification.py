from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path
from statistics import median

from .design import METHODS, StudyDesign, generate_design
from .records import TrialRecord, TrialStore


DESIGN_KIND = "engineering_roundtrip_qualification"
QUALIFICATION_METHOD = "feasibility_coupled"


def generate_roundtrip_design(runs: int = 20, master_seed: int = 20260913) -> StudyDesign:
    """Create repeated paired-seed plant realizations for the Gate 0 round trip."""
    generated = generate_design(
        layouts_per_family=1, replicates=runs, master_seed=master_seed,
        families=("combined_constraints",), design_kind=DESIGN_KIND)
    trials = tuple(spec for spec in generated.trials
                   if spec.method == QUALIFICATION_METHOD)
    return replace(generated, trials=trials)


# Gate 0 signed-motion bounds, mirrored from
# `modular_robot_bringup.qualification.qualification_pass` as
# (label, stage, field, direction, bound). `tests/test_roundtrip_qualification.py`
# binds the two by flipping the real gate either side of every bound, so a
# margin can never be reported against a limit the gate does not enforce.
#
# `min` means the value must stay at or above the bound, `max` at or below it,
# and `abs_max` bounds its magnitude. `translation_per_yaw` is derived, not
# recorded: it is the yaw-normalized translation coupling.
MOTION_BOUNDS = (
    ("positive_yaw_rad", "positive_yaw", "yaw_rad", "min", 0.10),
    ("negative_yaw_rad", "negative_yaw", "yaw_rad", "max", -0.10),
    ("forward_travel_m", "forward", "forward_m", "min", 0.08),
    ("reverse_travel_m", "reverse", "forward_m", "max", -0.08),
    ("positive_yaw_translation_m", "positive_yaw", "translation_m", "max", 0.12),
    ("negative_yaw_translation_m", "negative_yaw", "translation_m", "max", 0.12),
    ("positive_yaw_coupling", "positive_yaw", "translation_per_yaw", "max", 0.25),
    ("negative_yaw_coupling", "negative_yaw", "translation_per_yaw", "max", 0.25),
    ("forward_lateral_m", "forward", "lateral_m", "abs_max", 0.08),
    ("reverse_lateral_m", "reverse", "lateral_m", "abs_max", 0.08),
    ("forward_yaw_rad", "forward", "yaw_rad", "abs_max", 0.15),
    ("reverse_yaw_rad", "reverse", "yaw_rad", "abs_max", 0.15),
)


def _margin(observed: float, direction: str, bound: float) -> float:
    """Room remaining before the gate rejects. Negative means it rejected."""
    if direction == "min":
        return observed - bound
    if direction == "max":
        return bound - observed
    return bound - abs(observed)


def _stage_values(stages: dict, stage: str) -> dict[str, float]:
    values = dict(stages.get(stage) or {})
    yaw = values.get("yaw_rad")
    if "translation_m" in values and yaw:
        values["translation_per_yaw"] = values["translation_m"] / abs(float(yaw))
    return values


def motion_margins(records: list[TrialRecord]) -> dict:
    """Each signed-motion quantity against the bound it has to meet.

    Gate 0 owes a machine-readable summary of signed yaw, travel, and cross
    coupling, not just a verdict. The worst margin over a campaign is the
    number that says how close the platform actually came to failing, and it
    is what an operational threshold has to be set from.

    This is only meaningful for records whose command windows ran their full
    simulated duration. `TrialRecord.validate` rejects truncated ones at load,
    so a margin cannot be computed from load-dependent travel.
    """
    observed: dict[str, list[float]] = {label: [] for label, *_ in MOTION_BOUNDS}
    for record in records:
        for qualification in record.motion_qualifications:
            stages = qualification.get("stages") or {}
            for label, stage, field, _direction, _bound in MOTION_BOUNDS:
                values = _stage_values(stages, stage)
                if field in values:
                    observed[label].append(float(values[field]))
    summary = {}
    for label, _stage, _field, direction, bound in MOTION_BOUNDS:
        values = observed[label]
        if not values:
            continue
        margins = [_margin(value, direction, bound) for value in values]
        summary[label] = {
            "bound": bound,
            "direction": direction,
            "samples": len(values),
            "observed_min": min(values),
            "observed_median": median(values),
            "observed_max": max(values),
            "worst_margin": min(margins),
            "median_margin": median(margins),
        }
    return summary


def real_time_factors(records: list[TrialRecord]) -> dict:
    """Load spread the campaign actually met.

    Commanded maneuver windows advance on the simulated clock, so travel must
    not track this. Reporting it beside the margins is what makes that
    checkable rather than asserted.
    """
    factors = [float(record.phase_timing["real_time_factor"])
               for record in records
               if isinstance(record.phase_timing, dict)
               and record.phase_timing.get("real_time_factor") is not None]
    if not factors:
        return {}
    return {"samples": len(factors), "minimum": min(factors),
            "median": median(factors), "maximum": max(factors)}


def _morphology_sequence(record: TrialRecord) -> list[str]:
    sequence = []
    for event in record.topology_history:
        morphology = str(event.get("morphology", ""))
        if morphology and (not sequence or sequence[-1] != morphology):
            sequence.append(morphology)
    return sequence


def audit_roundtrip_records(records: list[TrialRecord], design: StudyDesign) -> dict:
    expected_ids = {spec.trial_id for spec in design.trials}
    observed_ids = {record.spec.trial_id for record in records}
    failures = []
    if design.design_kind != DESIGN_KIND:
        failures.append(f"unexpected design kind {design.design_kind}")
    if len(design.trials) != 20:
        failures.append(f"expected 20 frozen runs, found {len(design.trials)}")
    if observed_ids != expected_ids or len(records) != len(expected_ids):
        failures.append("terminal record set does not exactly match frozen design")
    disturbances = []
    for record in records:
        reasons = []
        sequence = _morphology_sequence(record)
        qualifications = record.motion_qualifications
        if not record.completed or record.terminal_status != "completed":
            reasons.append(f"terminal={record.terminal_status}")
        if record.collision_count:
            reasons.append(f"collisions={record.collision_count}")
        if record.reconfiguration_attempts != 2 or record.reconfiguration_failures:
            reasons.append(
                f"transitions={record.reconfiguration_attempts}/"
                f"failures={record.reconfiguration_failures}")
        if "narrow_tandem" not in sequence or sequence[-1:] != ["compact_diff"]:
            reasons.append(f"morphology_sequence={sequence}")
        if (record.final_morphology != "compact_diff"
                or record.final_execution_state != "READY" or record.unrecovered_fault):
            reasons.append(
                f"final={record.final_morphology}/{record.final_execution_state}/"
                f"fault={record.unrecovered_fault}")
        if len(qualifications) != 2 or any(not value.get("passed", False)
                                           for value in qualifications):
            reasons.append(f"motion_qualifications={len(qualifications)}")
        if any(not value.get("pod_rigidity", {}).get("passed", False)
               for value in qualifications):
            reasons.append("pod_rigidity_failed_or_missing")
        disturbance = record.manifest.parameters.get("physical_disturbance")
        if not isinstance(disturbance, dict):
            reasons.append("physical_disturbance_missing")
        else:
            disturbances.append(json.dumps(disturbance, sort_keys=True))
        if reasons:
            failures.append(f"{record.spec.trial_id}: " + "; ".join(reasons))
    if len(disturbances) != len(set(disturbances)):
        failures.append("realized physical disturbances are not unique across runs")
    return {
        "schema_version": 2,
        "purpose": DESIGN_KIND,
        "design_hash": design.design_hash,
        "expected_runs": 20,
        "observed_terminal_records": len(records),
        "unique_realized_disturbances": len(set(disturbances)),
        "motion_margins": motion_margins(records),
        "real_time_factors": real_time_factors(records),
        "passed": not failures,
        "failures": failures,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Freeze or audit Gate 0 round trips")
    commands = parser.add_subparsers(dest="command", required=True)
    generate = commands.add_parser("generate")
    generate.add_argument("--output", type=Path, required=True)
    generate.add_argument("--runs", type=int, default=20)
    generate.add_argument("--master-seed", type=int, default=20260913)
    audit = commands.add_parser("audit")
    audit.add_argument("--design", type=Path, required=True)
    audit.add_argument("--raw", type=Path, required=True)
    audit.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "generate":
        design = generate_roundtrip_design(args.runs, args.master_seed)
        design.write_frozen(args.output)
        print(json.dumps({"design_hash": design.design_hash,
                          "runs": len(design.trials)}, indent=2, sort_keys=True))
        return
    design = StudyDesign.read_frozen(args.design)
    store = TrialStore(args.raw)
    result = audit_roundtrip_records(store.load_all(), design)
    # Committing this summary preserves the record set's identity even though
    # the records themselves are far too large for version control. A later
    # re-collection can then be shown to differ from the set a claim was made
    # on, and a restored archive can be shown to be the original.
    result["record_digests"] = store.digests()
    result["campaign_digest"] = store.campaign_digest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
