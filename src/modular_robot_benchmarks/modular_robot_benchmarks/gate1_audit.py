"""Gate 1: every measured outcome is present, synchronized and attributable.

The roadmap's Gate 1 asks for synchronized evaluator-only collision and 3D
clearance, localization error, predicted transition probability and outcome,
mechanical work, docking attempts, recovery actions, failure cause, process
health, and source/container/configuration hashes, with an audit that rejects
missing or misaligned telemetry. ``telemetry_audit`` already rejects the
localization and truth streams; this module applies it together with every other
Gate 1 field, record by record, and reports which terminal outcome classes a
record set covers.

A problem here means the record cannot support an outcome metric, not that the
robot failed. Records that ended before a mission began carry no mission to
measure and are checked for provenance only.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Any, Iterable

from .evaluator_metrics import MINIMUM_AUDITED_DURATION_S, telemetry_audit
from .records import TrialRecord, TrialStore


# The outcome classes the Gate 1 artifact must show controlled records of. A
# fail-closed docking failure records as ``unsafe_topology``.
REQUIRED_OUTCOME_CLASSES = {
    "success": ("completed",),
    "collision": ("collision",),
    "localization_loss": ("localization_lost",),
    "docking_failure": ("docking_failure", "unsafe_topology"),
    "timeout": ("timeout",),
}

KNOWN_TERMINAL_STATUSES = {
    "completed", "collision", "localization_lost", "docking_failure",
    "unsafe_topology", "timeout", "controller_failure", "planning_failure",
    "motion_qualification_failure", "cancelled", "process_crash",
    "infrastructure_failure",
}


def provenance_problems(record: TrialRecord) -> list[str]:
    manifest = record.manifest
    problems = []
    if not manifest.commit or manifest.commit.endswith("-dirty"):
        problems.append(f"source revision is not a clean commit: {manifest.commit!r}")
    if not manifest.configuration_hash:
        problems.append("configuration hash missing")
    if not manifest.design_hash:
        problems.append("design hash missing")
    if manifest.software_versions.get("container_image", "unknown") == "unknown":
        problems.append("container image not recorded")
    return problems


def mission_problems(record: TrialRecord) -> list[str]:
    """Gate 1 fields a record with a mission must carry, beyond telemetry_audit."""
    problems = list(telemetry_audit(record))
    if record.terminal_status not in KNOWN_TERMINAL_STATUSES:
        problems.append(f"unknown terminal status {record.terminal_status!r}")
    # A mission that never came within range of an obstacle legitimately has no
    # minimum clearance; a record whose clearance was never computed does not.
    if "evaluator_clearance" not in record.controller_diagnostics:
        problems.append("evaluator clearance was not computed")
    if not record.covariance_trace:
        problems.append("no estimate covariance trace")
    if record.mechanical_work_j <= 0.0:
        problems.append("mechanical work not measured")
    if len(record.predicted_transition_probabilities) != len(
            record.observed_transition_outcomes):
        problems.append(
            f"{len(record.predicted_transition_probabilities)} transition predictions "
            f"against {len(record.observed_transition_outcomes)} observed outcomes")
    final = record.infrastructure_attempts[-1] if record.infrastructure_attempts else {}
    if "stack_process_deaths" not in final:
        problems.append("process health not recorded")
    elif final["stack_process_deaths"]:
        names = ", ".join(death["process"] for death in final["stack_process_deaths"])
        problems.append(f"stack processes died during the trial: {names}")
    return problems


def audit_record(record: TrialRecord) -> dict[str, Any]:
    has_mission = float(record.simulated_duration_s) >= MINIMUM_AUDITED_DURATION_S
    problems = provenance_problems(record) + (mission_problems(record) if has_mission else [])
    row = {"trial_id": record.spec.trial_id, "terminal_status": record.terminal_status,
           "has_mission": has_mission, "problems": problems}
    # A controlled record declares the class its stimulus was meant to produce.
    # A mismatch is reported, never a problem: the record is scored by the same
    # rules as any trial, and a stimulus that missed is a fact about it.
    case = record.manifest.parameters.get("gate1_outcome_case")
    if case:
        row["outcome_case"] = case["name"]
        row["expected_status"] = case["expected_status"]
        row["stimulus_produced_expected_class"] = (
            record.terminal_status == case["expected_status"])
    return row


def audit_records(records: Iterable[TrialRecord]) -> dict[str, Any]:
    rows = [audit_record(record) for record in records]
    statuses = Counter(row["terminal_status"] for row in rows)
    problem_kinds = Counter(
        problem.split(":")[0] for row in rows for problem in row["problems"])
    clean = [row for row in rows if not row["problems"]]
    coverage = {
        name: sum(1 for row in clean if row["terminal_status"] in members)
        for name, members in REQUIRED_OUTCOME_CLASSES.items()}
    return {
        "schema_version": 1,
        "purpose": "gate1_measured_outcome_coverage",
        "records": len(rows),
        "records_without_problems": len(clean),
        "terminal_statuses": dict(sorted(statuses.items())),
        "problem_counts": dict(sorted(problem_kinds.items())),
        "clean_records_per_required_class": coverage,
        "required_classes_missing": sorted(
            name for name, count in coverage.items() if count == 0),
        "controlled_cases": {
            row["trial_id"]: {"case": row["outcome_case"],
                              "expected": row["expected_status"],
                              "observed": row["terminal_status"]}
            for row in rows if "outcome_case" in row},
        "gate_passed": bool(rows) and len(clean) == len(rows) and all(coverage.values()),
        "per_record": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--raw", type=Path, action="append", required=True,
                        help="raw record directory; repeat to pool campaigns")
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    records, digests, record_digests = [], {}, {}
    for raw in arguments.raw:
        store = TrialStore(raw)
        records.extend(store.load_all())
        digests[str(raw)] = store.campaign_digest()
        # Per-record digests let `retain_campaign_records` prove a restored
        # archive is the set this audit was computed on (ADR 0005).
        record_digests.update(store.digests())
    summary = audit_records(records)
    summary["campaign_digests"] = digests
    summary["record_digests"] = dict(sorted(record_digests.items()))
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                                encoding="utf-8")
    print(json.dumps({key: summary[key] for key in (
        "records", "records_without_problems", "problem_counts",
        "clean_records_per_required_class", "gate_passed")}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
