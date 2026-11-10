"""Frozen designs for ExperimentChoice offset-doorway courses (EXPLORATORY).

A design is courses x Nav2 configurations x replicates for one morphology.
Both configurations of a (course, replicate) block share every plant seed, and
the two trials of a block run back to back, so a comparison within a block is
paired on spawn disturbance, ground friction and, as nearly as a shared host
allows, on load. Lidar noise and AMCL's particle draws are not seeded by this
stack, so they are not common random numbers.

    python3 -m modular_robot_benchmarks.doorway_design \
        --manifests studies/expchoice_doorway_v0/courses/*.json \
        --canary-course <course_id> --replicates 2 \
        --out studies/expchoice_doorway_v0/design.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from random import Random
from typing import Sequence

from .design import (
    DOORWAY_FAMILY, DOORWAY_METHODS, EXPCHOICE_DOORWAY_DESIGN_KIND,
    DoorwayTrialSpec, StudyDesign, _seed,
)
from .doorway_scenarios import DoorwayCourse, course_from_manifest

NAV2_CONFIG_DIR = "src/modular_robot_bringup/config/expchoice"
# (morphology, method) -> (file, sha256 of its bytes). The files are frozen: a
# changed file fails `nav2_config_for` and the test that reads this table.
NAV2_CONFIGS = {
    ("compact_diff", "rpp_rotate_navfn"): (
        "compact_diff_A_rpp_rotate_navfn.yaml",
        "50d697d5a0159da1b3e44419134e2bb2068f5e21a9032ee84f3e7b1d377908d1"),
    ("compact_diff", "mppi_reverse_navfn"): (
        "compact_diff_B_mppi_reverse_navfn.yaml",
        "9ebe0ef54e798be43adfae4a771bf62df4d5d8752ed870d5192e5cda6e1de512"),
    ("compact_diff", "rpp_reverse_smac_rs"): (
        "compact_diff_C_rpp_reverse_smac_rs.yaml",
        "ebd94d2d681964a8387242a837a87e8364890dc3518b9abe87c902498ae94617"),
    ("narrow_tandem", "rpp_rotate_navfn"): (
        "narrow_tandem_A_rpp_rotate_navfn.yaml",
        "8eabb941e04acaf0596a9afd927bbaefb1fffde7f717cacd632116e46432f80c"),
    ("narrow_tandem", "mppi_reverse_navfn"): (
        "narrow_tandem_B_mppi_reverse_navfn.yaml",
        "4dcb4a6e09c3b8178ccc8323a90fcda20028dbfe47fd67399e98080a5804da55"),
    ("narrow_tandem", "rpp_reverse_smac_rs"): (
        "narrow_tandem_C_rpp_reverse_smac_rs.yaml",
        "3749c8150b8a0dedd3b52321a2c0084a847aa5a4b8399318cef2a99c41ab0966"),
}
# Every world spawns `model://modular_robot`, whose pods sit at the compact
# poses, and the manager and drive adapter start as compact_diff. Nothing spawns
# an assembled narrow_tandem, so only compact_diff trials can run today; the
# narrow_tandem parameter files are frozen ahead of that work.
RUNNABLE_MORPHOLOGIES = ("compact_diff",)
DEFAULT_DEADLINE_S = 120.0
# The first study's pair. Arm C is frozen as an alternative reverse-capable
# configuration and is scheduled only when a design names it.
DEFAULT_METHODS = ("rpp_rotate_navfn", "mppi_reverse_navfn")
DEFAULT_MASTER_SEED = 20261110


def nav2_config_for(morphology: str, method: str, root: Path | None = None) -> tuple[str, str]:
    """Repository-relative parameter file and its frozen sha256.

    With `root`, the file on disk is hashed and must match.
    """
    try:
        name, digest = NAV2_CONFIGS[(morphology, method)]
    except KeyError:
        raise ValueError(f"no frozen Nav2 configuration for {morphology} / {method}") from None
    relative = f"{NAV2_CONFIG_DIR}/{name}"
    if root is not None:
        actual = hashlib.sha256((Path(root) / relative).read_bytes()).hexdigest()
        if actual != digest:
            raise ValueError(
                f"{relative} has sha256 {actual}; the frozen configuration is {digest}")
    return relative, digest


def trial_id_for(course_id: str, replicate: int, arm: str) -> str:
    return f"ecd-{course_id}-r{replicate}-{arm}"


def build_design(
    courses: Sequence[DoorwayCourse],
    morphology: str = "compact_diff",
    replicates: int = 2,
    master_seed: int = DEFAULT_MASTER_SEED,
    methods: Sequence[str] = DEFAULT_METHODS,
    canary_course_id: str | None = None,
    deadline_s: float = DEFAULT_DEADLINE_S,
    allow_inexact_map: bool = False,
    allow_unrunnable_morphology: bool = False,
) -> StudyDesign:
    """Courses x methods x replicates, in execution order.

    Order: the canary course's replicate-0 block first (its two trials are
    `run_order` 0 and 1), then the remaining replicate-0 blocks in a seeded
    shuffle, then each later replicate the same way, so stopping after any
    whole replicate leaves a balanced set. Within a block the arm order is a
    seeded coin flip.
    """
    courses = list(courses)
    methods = tuple(methods)
    if not courses or replicates <= 0:
        raise ValueError("a design needs at least one course and one replicate")
    if not methods or not set(methods) <= set(DOORWAY_METHODS) or len(set(methods)) != len(methods):
        raise ValueError(f"methods must be distinct and among {sorted(DOORWAY_METHODS)}")
    if morphology not in RUNNABLE_MORPHOLOGIES and not allow_unrunnable_morphology:
        raise ValueError(
            f"{morphology} cannot be run: no world spawns it assembled "
            f"(runnable: {', '.join(RUNNABLE_MORPHOLOGIES)})")
    by_id = {course.course_id: course for course in courses}
    if len(by_id) != len(courses):
        raise ValueError("courses must be distinct")
    if canary_course_id is None:
        canary_course_id = courses[0].course_id
    if canary_course_id not in by_id:
        raise ValueError(f"canary course {canary_course_id} is not in the design")
    if not allow_inexact_map:
        inexact = [course.course_id for course in courses
                   if not course.map_exactness()["exact_at_map_resolution"]]
        if inexact:
            raise ValueError(
                "the 0.01 m map cannot represent these courses exactly (odd w_cm "
                f"or B_cm puts an edge on the half centimetre): {inexact}")
    configs = {method: nav2_config_for(morphology, method) for method in methods}

    trials: list[DoorwayTrialSpec] = []
    for replicate in range(replicates):
        order = sorted(by_id)
        Random(_seed(master_seed, EXPCHOICE_DOORWAY_DESIGN_KIND, replicate, "blocks")).shuffle(order)
        if replicate == 0:
            order.remove(canary_course_id)
            order.insert(0, canary_course_id)
        for course_id in order:
            course = by_id[course_id]
            arms = list(methods)
            Random(_seed(master_seed, EXPCHOICE_DOORWAY_DESIGN_KIND, course_id,
                         replicate, "order")).shuffle(arms)
            seeds = {
                name: _seed(master_seed, EXPCHOICE_DOORWAY_DESIGN_KIND, course_id,
                            replicate, name)
                for name in ("sensing", "friction", "fault")}
            for position, method in enumerate(arms):
                arm = DOORWAY_METHODS[method]
                trials.append(DoorwayTrialSpec(
                    trial_id=trial_id_for(course_id, replicate, arm),
                    family=DOORWAY_FAMILY,
                    layout_id=f"{DOORWAY_FAMILY}-{course_id}",
                    replicate=replicate, method=method, method_order=position,
                    # The course is not drawn from a seed; this only names it.
                    world_seed=int(course_id, 16),
                    sensing_seed=seeds["sensing"], friction_seed=seeds["friction"],
                    fault_seed=seeds["fault"],
                    run_order=len(trials), arm=arm, morphology=morphology,
                    nav2_config=configs[method][0],
                    nav2_config_sha256=configs[method][1],
                    deadline_s=float(deadline_s), course_id=course_id,
                    **course.fields()))
    return StudyDesign(1, EXPCHOICE_DOORWAY_DESIGN_KIND, len(courses), replicates,
                       master_seed, methods, (DOORWAY_FAMILY,), tuple(trials))


def load_courses(paths: Sequence[Path]) -> list[DoorwayCourse]:
    """Courses from ExperimentChoice manifest files (one manifest or a list each)."""
    courses = []
    for path in paths:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        for manifest in (value if isinstance(value, list) else [value]):
            courses.append(course_from_manifest(manifest))
    return courses


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifests", type=Path, nargs="+", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--morphology", default="compact_diff")
    parser.add_argument("--replicates", type=int, default=2)
    parser.add_argument("--master-seed", type=int, default=DEFAULT_MASTER_SEED)
    parser.add_argument("--canary-course")
    parser.add_argument("--deadline", type=float, default=DEFAULT_DEADLINE_S)
    parser.add_argument("--methods", nargs="+", default=list(DEFAULT_METHODS),
                        choices=sorted(DOORWAY_METHODS))
    parser.add_argument("--root", type=Path, default=Path.cwd(),
                        help="repository root, to check the parameter files' hashes")
    args = parser.parse_args()
    design = build_design(
        load_courses(args.manifests), args.morphology, args.replicates,
        args.master_seed, methods=args.methods, canary_course_id=args.canary_course,
        deadline_s=args.deadline)
    for method in design.methods:
        nav2_config_for(args.morphology, method, args.root)
    design.write_frozen(args.out)
    print(json.dumps({"design_hash": design.design_hash, "trials": len(design.trials),
                      "canary": [trial.trial_id for trial in design.trials[:2]]}, indent=2))


if __name__ == "__main__":
    main()
