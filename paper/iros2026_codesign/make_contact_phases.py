"""Attribute recorded evaluator collisions to the mission phase they occurred in.

Usage: python3 make_contact_phases.py <results/workshop_diagnostic>

Terminal records carry ``collision_count`` as a scalar, with no per-contact
timestamp, so a contact cannot be attributed to a pod. The phase is still
recoverable from execution outcome, which is measured and not inferred:

* the first transition attempt failing means the robot never reached the
  post-transition check, so any recorded contact happened during the
  transformation;
* transitions succeeding while a motion qualification fails places any recorded
  contact in the post-transition check;
* otherwise contact happened under ordinary navigation.

Phases are emitted as ``<phase>:unattributed`` to match the consumer's
``<phase>:`` prefix convention while stating that the pod is not recoverable.

A mission with zero recorded collisions is emitted with no phase, which is a
claim that the evaluator saw no contact, not that contact was unattributable.
"""
import glob
import json
import sys
from pathlib import Path

study = Path(sys.argv[1])
phases: dict[str, list[str]] = {}

for path in sorted(glob.glob(str(study / "raw" / "*.json"))):
    record = json.loads(Path(path).read_text())
    trial = record["spec"]["trial_id"]
    if not record.get("collision_count"):
        phases[trial] = []
        continue
    outcomes = record.get("observed_transition_outcomes", [])
    qualifications = record.get("motion_qualifications", [])
    if outcomes and not outcomes[0]:
        phases[trial] = ["transformation:unattributed"]
    elif outcomes and all(outcomes) and any(
            not q.get("passed", True) for q in qualifications):
        phases[trial] = ["post_transition_check:unattributed"]
    else:
        phases[trial] = ["navigation:unattributed"]

out = study / "contact_phases.json"
out.write_text(json.dumps(phases, indent=2, sort_keys=True) + "\n", encoding="utf-8")
for trial, value in sorted(phases.items()):
    print(f"{trial:52s} {value}")
print("wrote", out)
