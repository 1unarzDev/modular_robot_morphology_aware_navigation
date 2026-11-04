"""Blind integrity check of confirmatory_v3 records: no outcome is tabulated by method.

Checks every record parses, belongs to the frozen design with identical spec,
carries one commit / design hash / configuration hash, has sane durations,
infrastructure provenance, and evaluator telemetry when the mission ran.
Prints only harness-health counts.
"""
import json, sys
from collections import Counter
from pathlib import Path

WS = Path("/home/checker/morphology_ws")
raw = WS / "results/confirmatory/confirmatory_v3_raw"
design = json.loads((WS / "studies/confirmatory_v3/design.json").read_text())
specs = {t["trial_id"]: t for t in design["design"]["trials"]}
HARNESS = {"infrastructure_failure", "process_crash", "stale_topic", "cancelled"}
problems, commits, designs, configs, images = [], Counter(), Counter(), Counter(), Counter()
harness, retried, extra_attempts, ran, no_truth, no_loc = Counter(), 0, 0, 0, 0, 0
max_sim, max_wall = 0.0, 0.0
files = sorted(raw.glob("*.json"))
for path in files:
    try:
        r = json.loads(path.read_text())
    except Exception as error:
        problems.append(f"{path.name}: unparseable ({error})"); continue
    s = r["spec"]; tid = s["trial_id"]
    if path.stem != tid: problems.append(f"{path.name}: file/trial id mismatch")
    if tid not in specs: problems.append(f"{tid}: not in frozen design")
    elif specs[tid] != s: problems.append(f"{tid}: spec differs from frozen design")
    m = r["manifest"]
    commits[m["commit"]] += 1; designs[m["design_hash"]] += 1
    # The resume (02:47 UTC 2026-11-04) must not change the shared stack.
    era = "after_resume" if path.stat().st_mtime > 1793760420 else "before_resume"
    configs[(era, m.get("parameters", {}).get("stack_configuration_hash"))] += 1
    images[str(m.get("parameters", {}).get("container_image", m.get("software_versions", {}).get("container_image")))] += 1
    if r["schema_version"] != 1: problems.append(f"{tid}: schema")
    if r["completed"] != (r["terminal_status"] == "completed"): problems.append(f"{tid}: completed flag")
    sim, wall = r["simulated_duration_s"], r["wall_duration_s"]
    if not (0 <= sim <= r["deadline_s"] + 5): problems.append(f"{tid}: simulated duration {sim}")
    max_sim, max_wall = max(max_sim, sim), max(max_wall, wall)
    attempts = r.get("infrastructure_attempts") or []
    if not attempts: problems.append(f"{tid}: no attempt provenance")
    if len(attempts) > 1: retried += 1; extra_attempts += len(attempts) - 1
    if r["terminal_status"] in HARNESS: harness[r["terminal_status"]] += 1
    if sim > 0:
        ran += 1
        if not r.get("localization_history"): no_loc += 1
        if not r.get("localization_error_m"): no_truth += 1
print(f"records {len(files)}/720  unique ids {len({p.stem for p in files})}")
print(f"commits {dict(commits)}")
print(f"design hashes {len(designs)} ({next(iter(designs))[:12]}; frozen {design.get('design_hash','?')[:12]})")
print("stack configuration hash by era: " + ", ".join(
    f"{era} {str(h)[:12]} x{n}" for (era, h), n in sorted(configs.items(), key=str)))
print(f"image fields {dict(images)}")
print(f"trials retried {retried} (extra attempts {extra_attempts}); harness-class terminal statuses {dict(harness) or 0}")
print(f"missions with simulated time {ran}: missing localization history {no_loc}, missing truth error {no_truth}")
print(f"max simulated {max_sim:.1f}s  max wall {max_wall:.0f}s")
print(f"problems {len(problems)}"); [print("  " + p) for p in problems[:20]]
