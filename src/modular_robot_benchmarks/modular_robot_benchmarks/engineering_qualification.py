"""Gate 0 engineering qualification: declared executor fault matrix.

Round-trip reliability and all-pod rigidity are audited by
`roundtrip_qualification` and `rigidity`. Evaluator-only Gazebo poses are
consumed here after a trial has ended; nothing in this module is reachable from
localization, planning, docking, or recovery. Engineering records are never
pilot or confirmatory evidence.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from .design import (
    FAULT_MATRIX, FAULT_MATRIX_DESIGN_KIND, FaultCase, StudyDesign,
    fault_case_for_run,
    generate_fault_matrix_campaign,
)
from .records import TrialRecord, TrialStore


# Assembled drive is inhibited when no pod receives motion and the core stays put.
INHIBITION_MAX_POD_COMMAND = 1e-6
INHIBITION_MAX_CORE_TRANSLATION_M = 0.01
INHIBITION_MAX_CORE_YAW_RAD = 0.02


def injection_fired(record: TrialRecord, case: FaultCase) -> bool:
    """Whether the executor reported injecting this case's declared fault."""
    stage, _, pod = case.injection.partition(":")
    return any(event.get("stage") == stage and (not pod or event.get("pod") == pod)
               for event in record.fired_injections)


def evaluate_fault_trial(record: TrialRecord, case: FaultCase) -> dict[str, Any]:
    probe = record.recovery_probe
    reconcile = probe.get("reconcile") or {}
    if case.expected_reconciled_morphology is None:
        reconciliation = (reconcile.get("success") is False
                          and reconcile.get("execution_state") == "RECOVERY_REQUIRED")
    else:
        reconciliation = (reconcile.get("success") is True
                          and reconcile.get("execution_state") == "READY"
                          and reconcile.get("morphology") == case.expected_reconciled_morphology)
    checks = {
        "declared_injection": (
            record.manifest.parameters.get("failure_injection") == case.injection),
        # Declaring a fault is not exercising it: a mission that never reaches
        # the injection point, or fails earlier on its own, tests nothing.
        "injection_fired": injection_fired(record, case),
        "fail_closed_terminal": (not record.completed
                                 and record.terminal_status == "unsafe_topology"),
        "recovery_required": (probe.get("state_before") == "RECOVERY_REQUIRED"
                              and record.final_execution_state == "RECOVERY_REQUIRED"),
        "drive_inhibited": (
            probe.get("performed") is True
            and probe.get("max_pod_command", float("inf")) <= INHIBITION_MAX_POD_COMMAND
            and probe.get("core_translation_m", float("inf"))
            <= INHIBITION_MAX_CORE_TRANSLATION_M
            and probe.get("core_yaw_change_rad", float("inf")) <= INHIBITION_MAX_CORE_YAW_RAD),
        "reconciliation_matches_observed_topology": reconciliation,
    }
    return {
        "case": case.name, "injection": case.injection,
        "expected_reconciled_morphology": case.expected_reconciled_morphology,
        "trial_id": record.spec.trial_id, "terminal_status": record.terminal_status,
        "message": record.notes[0] if record.notes else "",
        "exercised": checks["injection_fired"],
        "passed": all(checks.values()), "checks": checks,
        "failed_checks": sorted(name for name, value in checks.items() if not value),
        "recovery_probe": probe,
    }


def summarize_fault_matrix(design: StudyDesign, records: list[TrialRecord]) -> dict[str, Any]:
    if design.design_kind != FAULT_MATRIX_DESIGN_KIND:
        raise ValueError(f"design kind {design.design_kind} is not {FAULT_MATRIX_DESIGN_KIND}")
    wrong_hash = sorted(record.spec.trial_id for record in records
                        if record.manifest.design_hash != design.design_hash)
    if wrong_hash:
        raise ValueError(f"records have a different design hash: {wrong_hash}")
    expected = {spec.trial_id for spec in design.trials}
    by_id = {record.spec.trial_id: record for record in records}
    cases = []
    for spec in sorted(design.trials, key=lambda value: (value.layout_id, value.replicate)):
        case = fault_case_for_run(spec.replicate)
        record = by_id.get(spec.trial_id)
        evaluated = evaluate_fault_trial(record, case) if record is not None else {
            "case": case.name, "injection": case.injection, "trial_id": spec.trial_id,
            "passed": False, "failed_checks": ["pending"]}
        evaluated["layout_id"] = spec.layout_id
        evaluated["realization"] = spec.replicate // len(FAULT_MATRIX)
        cases.append(evaluated)
    layouts = sorted({spec.layout_id for spec in design.trials})
    by_layout = {
        layout: {
            "passed": all(case["passed"] for case in cases
                          if case["layout_id"] == layout),
            "cases_passed": sum(1 for case in cases
                                if case["layout_id"] == layout and case["passed"]),
            "cases_total": sum(1 for case in cases if case["layout_id"] == layout),
            "cases_not_exercised": sum(
                1 for case in cases
                if case["layout_id"] == layout and case.get("exercised") is False),
        }
        for layout in layouts
    }
    # Repeating the matrix is only evidence if each cell is an independent
    # realization; a duplicated fault seed would re-run one injection.
    fault_seeds = [spec.fault_seed for spec in design.trials]
    distinct_fault_seeds = len(set(fault_seeds)) == len(fault_seeds)
    # Every declared case must appear in every layout, or the matrix was
    # narrowed rather than repeated.
    declared = {case.name for case in FAULT_MATRIX}
    complete_layouts = all(
        {case["case"] for case in cases if case["layout_id"] == layout} == declared
        for layout in layouts
    )
    return {
        "purpose": "engineering_qualification_not_study_evidence",
        "design_hash": design.design_hash,
        "unexpected_records": sorted(set(by_id) - expected),
        "layouts": layouts,
        "realizations_per_case": design.replicates // len(FAULT_MATRIX),
        "by_layout": by_layout,
        "distinct_fault_seeds": distinct_fault_seeds,
        "every_case_in_every_layout": complete_layouts,
        "gate_passed": (
            all(case["passed"] for case in cases)
            and distinct_fault_seeds
            and complete_layouts
        ),
        "cases": cases,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Freeze and evaluate the Gate 0 executor fault matrix")
    commands = parser.add_subparsers(dest="command", required=True)
    freeze = commands.add_parser(
        "freeze-fault-matrix", help=f"write an immutable {FAULT_MATRIX_DESIGN_KIND} design")
    freeze.add_argument("--output", type=Path, required=True)
    freeze.add_argument("--family", required=True)
    freeze.add_argument(
        "--layout-index", type=int, action="append", required=True,
        help="repeat to span layouts; Gate 0 requires more than one")
    freeze.add_argument("--method", required=True)
    freeze.add_argument(
        "--realizations", type=int, default=1,
        help="independent fault_seed realizations of every declared case")
    freeze.add_argument("--master-seed", type=int, default=20260912)
    summary = commands.add_parser("fault-matrix", help="summarize fault-matrix records")
    summary.add_argument("--design", type=Path, required=True)
    summary.add_argument("--raw", type=Path, required=True)
    summary.add_argument("--output", type=Path, required=True)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.command == "freeze-fault-matrix":
        design = generate_fault_matrix_campaign(
            args.family, tuple(args.layout_index), args.method,
            args.realizations, args.master_seed)
        design.write_frozen(args.output)
        result = {"path": str(args.output), "design_hash": design.design_hash,
                  "expected_trials": design.expected_trials,
                  "layouts": sorted({spec.layout_id for spec in design.trials})}
    else:
        design = StudyDesign.read_frozen(args.design)
        store = TrialStore(args.raw)
        summary = summarize_fault_matrix(design, store.load_all())
        # As for the round-trip audit: the committed summary must identify the
        # record set, because the records themselves are not version-controlled.
        summary["record_digests"] = store.digests()
        summary["campaign_digest"] = store.campaign_digest()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                               encoding="utf-8")
        result = {key: summary[key] for key in ("design_hash", "gate_passed")}
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
