#!/bin/bash
# Assemble predetach_check_v1 predictions once the three planner-level shards
# have finished, fill the declaration, run the tests, and commit locally.
# Never pushes. Usage (from the worktree root):
#   bash tools/finish_predetach_predictions.sh <cells-dir> [handoff-note]
# The shards were started as
#   python3 -m modular_robot_benchmarks.predetach_check --shard i/3 --cells-dir <cells-dir>
set -euo pipefail
CELLS=${1:?cells dir}
NOTE=${2:-}
export PYTHONPATH=src/morphology_planner:src/modular_robot_bringup:src/modular_robot_benchmarks:src/reconfiguration_executor
export TMPDIR=${TMPDIR:-$HOME/.cache/mrmn-predetach/tmp}
mkdir -p "$TMPDIR"
until [ -s "$CELLS/shard_0.json" ] && [ -s "$CELLS/shard_1.json" ] && [ -s "$CELLS/shard_2.json" ]; do
  sleep 60
done
nice -n 15 python3 -m modular_robot_benchmarks.predetach_check --cells-dir "$CELLS" \
  > "$CELLS/assemble.log" 2>&1
nice -n 15 python3 tools/fill_predetach_declaration.py studies/predetach_check_v1 \
  > "$CELLS/prediction_summary.md"
nice -n 15 python3 -m pytest -q -p no:cacheprovider > "$CELLS/tests_after.log" 2>&1 || true
git add studies/predetach_check_v1/predictions.json studies/predetach_check_v1/declaration.md \
        studies/predetach_check_v1/design.json studies/predetach_check_v1/smoke_trial_ids.json
git commit -q -m "Freeze predetach_check_v1 predictions and declaration (before any trial)

Planner-level predictions for all cells from predetach_check.simulate_mission;
declaration filled from them. Generated before any trial of this design."
git rev-parse HEAD > "$CELLS/FINAL_COMMIT"
if [ -n "$NOTE" ] && [ -f "$NOTE" ]; then
  python3 - "$NOTE" "$CELLS" <<'EOF'
import sys, pathlib
note, cells = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
text = note.read_text()
summary = (cells / "prediction_summary.md").read_text().strip()
tail = (cells / "tests_after.log").read_text().strip().splitlines()[-1:]
commit = (cells / "FINAL_COMMIT").read_text().strip()
text = text.replace("@@FINAL_COMMIT@@", commit)
text = text.replace("@@PRED_SUMMARY@@", summary)
text = text.replace("@@TESTS_AFTER_PRED@@", " ".join(tail))
note.write_text(text)
EOF
fi
echo "done $(cat "$CELLS/FINAL_COMMIT")"
