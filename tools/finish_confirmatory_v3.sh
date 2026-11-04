#!/bin/bash
# After `confirmatory_v3` ends: retain its records, run the frozen analysis, copy
# the committable evidence into the Windows checkout and render the paper's
# results artifacts. Run inside WSL, in one call (the checkout is CRLF):
#
#   wsl -e bash -lc "bash <(sed 's/\r$//' /mnt/c/Users/risha/gitclones/modular_robot_morphology_aware_navigation/tools/finish_confirmatory_v3.sh)"
#
# It refuses to run before the campaign's "done" marker and changes nothing in
# the design, the analysis or the records; it only runs what is frozen.
set -euo pipefail
C=morphology_navigation_dev
WS=/home/checker/morphology_ws
REPO=/mnt/c/Users/risha/gitclones/modular_robot_morphology_aware_navigation
DEST=$REPO/studies/confirmatory_v3
R=$WS/results/confirmatory
grep -q "start_campaign run_confirmatory_v3 done" $WS/results/run_confirmatory_v3.launch.log \
  || { echo "ABORT: campaign not finished"; exit 1; }
[ "$(git -C $WS rev-parse --short HEAD)" = "8a53344" ] || { echo "ABORT: clone moved off 8a53344"; exit 1; }
docker start $C >/dev/null
if [ "$(docker exec $C bash -c 'ps -eo cmd | grep -e "gz sim" -e "ros2 launch" -e parameter_bridge | grep -v grep | wc -l')" -ne 0 ]; then
  echo "ABORT: simulation processes still running"; exit 1
fi
echo "records: $(ls $R/confirmatory_v3_raw | wc -l) of 720"
cat $R/confirmatory_v3_status.json | head -40
# Retain (archive + verify against the gate1 audit), then the frozen analysis
# (~20 min, almost all of it the hierarchical refits).
docker exec $C bash /home/roboboat/morphology_ws/results/qualification/post_confirmatory_v3.sh
cat $R/confirmatory_v3_retain.json
mkdir -p $DEST/records $DEST/analysis
cp $R/confirmatory_v3_gate1_audit.json $R/confirmatory_v3_status.json \
   $R/confirmatory_v3_retain.json $DEST/
cp $R/retained/confirmatory_v3_raw.tar.xz* $DEST/records/
cp -r $R/analysis/. $DEST/analysis/
# The paper generators are stdlib-only and newer than the pinned clone, so they
# run from the Windows checkout on the WSL host Python.
for g in tables:results_tables.tex figure:figure_results.svg; do
  tr -d '\r' < $REPO/paper/conference/make_results_${g%%:*}.py > /tmp/make_results_${g%%:*}.py
  python3 /tmp/make_results_${g%%:*}.py $DEST/analysis/analysis.json $REPO/paper/conference/${g#*:}
done
tr -d '\r' < $REPO/paper/conference/make_failure_table.py > /tmp/make_failure_table.py
python3 /tmp/make_failure_table.py $R/confirmatory_v3_raw \
  $REPO/paper/conference/failure_table.tex $DEST/failure_table.json
# The exploratory studies are independent of this campaign; regenerating their
# macros here only keeps the paper's inputs in one place.
tr -d '\r' < $REPO/paper/conference/make_exploratory_macros.py > /tmp/make_exploratory_macros.py
python3 /tmp/make_exploratory_macros.py $REPO/studies/prevalence/prevalence_20261104.json \
  $REPO/studies/route_first_retry/design.json $REPO/paper/conference/exploratory_macros.tex
ls -l $DEST $DEST/records $DEST/analysis
echo "=== finish_confirmatory_v3 done; build the paper with paper/conference/build.sh on Windows"
