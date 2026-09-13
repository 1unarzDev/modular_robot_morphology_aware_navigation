from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

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
        "schema_version": 1,
        "purpose": DESIGN_KIND,
        "design_hash": design.design_hash,
        "expected_runs": 20,
        "observed_terminal_records": len(records),
        "unique_realized_disturbances": len(set(disturbances)),
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
    result = audit_roundtrip_records(TrialStore(args.raw).load_all(), design)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
