from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

from .analysis import run_analysis, validate_records
from .design import CONFIRMATORY_FAMILIES, StudyDesign, generate_design
from .power import (
    PowerAssumptions, estimate_power, estimate_power_grid, write_power,
)
from .records import TrialStore


def study_status(design: StudyDesign, raw: str | Path) -> dict:
    records = TrialStore(raw).load_all()
    expected = {trial.trial_id for trial in design.trials}
    observed = {record.spec.trial_id for record in records}
    invalid_hashes = sorted(record.spec.trial_id for record in records
                            if record.manifest.design_hash != design.design_hash)
    terminal = Counter(record.terminal_status for record in records)
    methods = Counter(record.spec.method for record in records)
    integrity_errors = []
    if observed == expected and not invalid_hashes:
        try:
            validate_records(records, design)
        except ValueError as exc:
            integrity_errors.append(str(exc))
    return {
        "design_hash": design.design_hash,
        "expected_records": design.expected_trials,
        "observed_records": len(records),
        "pending_records": len(expected - observed),
        "unexpected_records": sorted(observed - expected),
        "records_with_wrong_design_hash": invalid_hashes,
        "integrity_errors": integrity_errors,
        "terminal_status_counts": dict(sorted(terminal.items())),
        "method_counts": {method: methods[method] for method in design.methods},
        "analysis_ready": (observed == expected and not invalid_hashes and not integrity_errors),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Freeze, audit, and analyze morphology-navigation studies")
    commands = parser.add_subparsers(dest="command", required=True)

    freeze = commands.add_parser("freeze-design", help="write an immutable randomized trial design")
    freeze.add_argument("--output", type=Path, default=Path("studies/confirmatory/design.json"))
    freeze.add_argument("--layouts-per-family", type=int, default=12)
    freeze.add_argument("--replicates", type=int, default=3)
    freeze.add_argument("--master-seed", type=int, default=20260911)
    freeze.add_argument("--kind", default="confirmatory")
    freeze.add_argument("--families", nargs="+", default=list(CONFIRMATORY_FAMILIES))

    status = commands.add_parser("status", help="audit progress against a frozen design")
    status.add_argument("--design", type=Path, required=True)
    status.add_argument("--raw", type=Path, required=True)

    analysis = commands.add_parser("analyze", help="validate a complete study and generate artifacts")
    analysis.add_argument("--design", type=Path, required=True)
    analysis.add_argument("--raw", type=Path, required=True)
    analysis.add_argument("--output", type=Path, required=True)
    analysis.add_argument("--bootstrap-draws", type=int, default=10000)
    analysis.add_argument("--permutation-draws", type=int, default=100000)
    analysis.add_argument("--hierarchical-bootstrap-draws", type=int, default=400)

    power = commands.add_parser("power", help="run assumption-based prospective power simulation")
    power.add_argument("--layouts", type=int, required=True)
    power.add_argument("--replicates", type=int, default=3)
    power.add_argument("--control-rate", type=float, required=True)
    power.add_argument("--treatment-rate", type=float, required=True)
    power.add_argument("--layout-logit-sd", type=float, required=True)
    power.add_argument("--paired-noise-fraction", type=float, required=True)
    power.add_argument("--alpha", type=float, default=0.025)
    power.add_argument("--simulations", type=int, default=2000)
    power.add_argument("--randomization-draws", type=int, default=1999)
    power.add_argument("--seed", type=int, default=20260911)
    power.add_argument("--output", type=Path, required=True)

    grid = commands.add_parser(
        "power-grid", help="run a conservative prospective nuisance grid")
    grid.add_argument("--layouts", type=int, required=True)
    grid.add_argument("--replicates", type=int, default=3)
    grid.add_argument("--control-rates", type=float, nargs="+", required=True)
    grid.add_argument("--absolute-effect", type=float, required=True)
    grid.add_argument("--layout-logit-sds", type=float, nargs="+", required=True)
    grid.add_argument("--paired-noise-fractions", type=float, nargs="+", required=True)
    grid.add_argument("--alpha", type=float, default=0.025)
    grid.add_argument("--target-power", type=float, default=0.8)
    grid.add_argument("--simulations", type=int, default=2000)
    grid.add_argument("--randomization-draws", type=int, default=1999)
    grid.add_argument("--seed", type=int, default=20260911)
    grid.add_argument("--output", type=Path, required=True)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.command == "freeze-design":
        design = generate_design(args.layouts_per_family, args.replicates,
                                 args.master_seed, args.families, args.kind)
        design.write_frozen(args.output)
        result = {"path": str(args.output), "design_hash": design.design_hash,
                  "expected_trials": design.expected_trials}
    elif args.command == "status":
        result = study_status(StudyDesign.read_frozen(args.design), args.raw)
    elif args.command == "analyze":
        result = run_analysis(args.raw, StudyDesign.read_frozen(args.design), args.output,
                              args.bootstrap_draws, args.permutation_draws,
                              args.hierarchical_bootstrap_draws)
    elif args.command == "power":
        assumptions = PowerAssumptions(
            args.layouts, args.replicates, args.control_rate, args.treatment_rate,
            args.layout_logit_sd, args.paired_noise_fraction, args.alpha)
        result = estimate_power(assumptions, args.simulations, args.randomization_draws, args.seed)
        write_power(result, args.output)
    else:
        result = estimate_power_grid(
            args.layouts, args.replicates, args.control_rates,
            args.absolute_effect, args.layout_logit_sds,
            args.paired_noise_fractions, args.alpha, args.target_power,
            args.simulations, args.randomization_draws, args.seed)
        write_power(result, args.output)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
