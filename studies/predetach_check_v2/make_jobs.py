#!/usr/bin/env python3
"""Write the predetach_check_v2 job files (main campaign and canary) pinned to `git rev-parse HEAD`.

Usage (from the repository root, on a clean tree):
    python3 studies/predetach_check_v2/make_jobs.py <out_dir>
Writes <out_dir>/pdc2_jobs_main.json and <out_dir>/pdc2_jobs_canary.json. Refuses a dirty tree.
Rules (declaration.md): canary = run_order 2, 3 in campaign predetach_check_v2_canary (excluded from analysis);
main = every other run_order in campaign predetach_check_v2, in run order, chunks of 10 seeds.
"""
import json
import subprocess
import sys
from pathlib import Path

N_TRIALS = 132
CANARY_SEEDS = (2, 3)
EXCLUDED_FROM_MAIN = frozenset(CANARY_SEEDS)
CHUNK = 10
EST_SECONDS = 2400   # est_seconds per seed (the guard's hard timeout is 5 x est x seeds + 60 min)
URL = "/home/lunarz/modular_robot_morphology_aware_navigation"
DESIGN = "studies/predetach_check_v2/design.json"


def _job(job_id, seeds, campaign, priority, commit, purpose):
    return {"schema": "rc-job/1", "job_id": job_id, "kind": "mrmn_trial", "seeds": list(seeds),
            "priority": priority, "est_seconds": EST_SECONDS, "submitter": "fleet-p1-writer",
            "campaign": campaign, "purpose": purpose, "needs": {"host": "innovation"},
            "repos": [{"name": "mrmn", "url": URL, "ref": commit}],
            "spec": {"design": DESIGN, "campaign": campaign}}


def job_lists(commit: str):
    if len(commit) != 40 or any(c not in "0123456789abcdef" for c in commit):
        raise ValueError("commit must be 40 lowercase hex characters")
    canary = [_job("pdc2-canary-002-003", CANARY_SEEDS, "predetach_check_v2_canary", 50, commit,
                   "Pre-detach check baseline, version 2: diagnostic canary (two exploratory probe-arm trials, "
                   "excluded from analysis) confirming the build and code hash on the pinned host.")]
    seeds = [s for s in range(N_TRIALS) if s not in EXCLUDED_FROM_MAIN]
    main = []
    for i in range(0, len(seeds), CHUNK):
        chunk = seeds[i:i + CHUNK]
        main.append(_job(f"pdc2-main-{chunk[0]:03d}-{chunk[-1]:03d}", chunk, "predetach_check_v2",
                         60 + i // CHUNK, commit,
                         "Pre-detach check baseline, version 2: fresh-seed Gazebo trials testing a re-timed "
                         "planner-level model, in run order (fresh arm first)."))
    assert not any(s in EXCLUDED_FROM_MAIN for j in main for s in j["seeds"])
    assert sorted(s for j in canary for s in j["seeds"]) == list(CANARY_SEEDS)
    return main, canary


def main():
    out = Path(sys.argv[1])
    dirty = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, check=True).stdout
    if dirty.strip():
        sys.exit("refusing: the working tree is dirty; commit first")
    commit = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
    mains, canary = job_lists(commit)
    out.mkdir(parents=True, exist_ok=True)
    (out / "pdc2_jobs_main.json").write_text(json.dumps({"jobs": mains}, indent=1) + "\n")
    (out / "pdc2_jobs_canary.json").write_text(json.dumps({"jobs": canary}, indent=1) + "\n")
    print(json.dumps({"commit": commit, "main_jobs": len(mains),
                      "main_seeds": sum(len(j["seeds"]) for j in mains), "canary_seeds": list(CANARY_SEEDS)}))


if __name__ == "__main__":
    main()
