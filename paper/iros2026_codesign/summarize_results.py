"""Summarize immutable workshop-study records into the paper table.

Usage: python3 summarize_results.py <results/workshop_diagnostic> <out_dir>
Rejects records from another design or source commit.
"""
import glob
import json
import sys
from pathlib import Path
from statistics import mean

study, out = Path(sys.argv[1]), Path(sys.argv[2])
design = json.loads((study / "design.json").read_text())
design_hash = design["design_hash"]
records = [json.loads(Path(p).read_text()) for p in sorted(glob.glob(str(study / "raw" / "*.json")))]
commits = {r["manifest"]["commit"] for r in records}
if any(r["manifest"]["design_hash"] != design_hash for r in records):
    raise SystemExit("record from a different design")
if len(commits) > 1:
    raise SystemExit(f"records span multiple commits: {commits}")

contact_path = study / "contact_phases.json"
contact_phases = json.loads(contact_path.read_text()) if contact_path.exists() else {}
VARIANTS = [("workshop_blocked_a", "Blocked-A"), ("workshop_blocked_b", "Blocked-B"),
            ("workshop_neutral", "Neutral")]
METHODS = [("route_first_adaptation", "Route-first"), ("geometry_coupled", "Geometry"),
           ("feasibility_coupled", "Feasibility")]
rows, summary = [], {"design_hash": design_hash, "commits": sorted(commits),
                     "records": len(records), "expected": len(design["design"]["trials"]), "cells": {}}
for variant, vlabel in VARIANTS:
    for method, mlabel in METHODS:
        cell = [r for r in records if r["spec"]["family"] == variant and r["spec"]["method"] == method]
        n = len(cell)
        completed = sum(r["completed"] for r in cell)
        attempts = sum(len(r["observed_transition_outcomes"]) for r in cell)
        successes = sum(sum(r["observed_transition_outcomes"]) for r in cell)
        env_rejections = [sum(d.get("count", 1) for d in r["transition_edge_decisions"]
                              if not d["feasible"] and any("environment_collision" in x for x in d["reasons"]))
                          for r in cell]
        sites = sorted({round(s["x"], 2) for r in cell for s in r["planned_transition_sites"]
                        if s["transition_id"] == "compact_to_narrow"})
        clearances = [r["minimum_clearance_m"] for r in cell if r["minimum_clearance_m"] is not None]
        times = [r["simulated_duration_s"] for r in cell if r["completed"]]
        latency = [r["planning_latency_s"] for r in cell]
        statuses = sorted({r["terminal_status"] for r in cell})
        summary["cells"][f"{variant}/{method}"] = {
            "n": n, "completed": completed, "transition_successes": successes,
            "transition_attempts": attempts, "environment_rejections": env_rejections,
            "c2n_site_x_m": sites, "min_clearance_m": min(clearances) if clearances else None,
            "collisions": sum(r["collision_count"] for r in cell),
            "mean_completed_time_s": mean(times) if times else None,
            "mean_planning_latency_s": mean(latency) if latency else None,
            "terminal_statuses": statuses,
            "contact_phases": sorted({p for r in cell for p in contact_phases.get(r["spec"]["trial_id"], [])}),
            "transformation_contact": any(p.startswith("transformation:")
                                          for r in cell for p in contact_phases.get(r["spec"]["trial_id"], [])),
            "post_transition_check_contact": any(p.startswith("post_transition_check:")
                                                 for r in cell for p in contact_phases.get(r["spec"]["trial_id"], [])),
        }
        site = "/".join(f"{x:.2f}" for x in sites) if sites else "--"
        clear = f"{100 * min(clearances):.1f}" if clearances else "--"
        time_s = f"{mean(times):.0f}" if times else "--"
        lat = f"{mean(latency):.1f}" if latency else "--"
        rej = f"{min(env_rejections)}" if env_rejections and min(env_rejections) == max(env_rejections) else \
            ("--" if not env_rejections else f"{min(env_rejections)}--{max(env_rejections)}")
        rows.append(f"{vlabel if method == METHODS[0][0] else ''} & {mlabel} & {completed}/{n} & "
                    f"{site} & {rej} & {successes}/{attempts} & {clear} & {time_s} & {lat} \\\\")
    rows.append("\\midrule" if variant != VARIANTS[-1][0] else "")
table = "\n".join([
    "\\setlength{\\tabcolsep}{2.2pt}",
    "\\begin{tabular}{llccccccc}",
    "\\toprule",
    "Env. & Planner & Done & Site $x$ [m] & Env.\\ rej. & Trans. & Clear.\\ [cm] & Time [s] & Plan [s] \\\\",
    "\\midrule",
    *[row for row in rows if row],
    "\\bottomrule",
    "\\end{tabular}",
])
out.mkdir(parents=True, exist_ok=True)
(out / "results_table.tex").write_text(table + "\n", encoding="utf-8")
(out / "results_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
print(table)
print(json.dumps({k: v for k, v in summary.items() if k != "cells"}))
