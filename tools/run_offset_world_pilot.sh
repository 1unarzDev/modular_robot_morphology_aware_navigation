#!/bin/bash
# Launch `offset_world_pilot_v1` (smoke subset first, then the full pilot) on the
# WSL host, inside the existing container. Run inside WSL:
#
#   bash tools/run_offset_world_pilot.sh smoke <commit>
#   bash tools/run_offset_world_pilot.sh full  <commit>
#
# ASSUMPTIONS. The campaign scripts the earlier studies used (`start_campaign`,
# `resume_campaign.sh`, `results/run_<name>.sh`) live in the WSL clone under
# results/, which is not versioned, so this is the closest equivalent written
# from what the repository documents (AGENTS.md; tools/finish_confirmatory_v3.sh):
#   * container `morphology_navigation_dev`; WSL clone at /home/checker/morphology_ws,
#     the same tree as /home/roboboat/morphology_ws in the container;
#   * the clone is checked out at <commit> (the proposal branch tip) and clean,
#     because a campaign runs at one commit and mission_batch refuses a dirty tree;
#   * the ROS packages are installed from that tree by `colcon build`;
#   * records go to results/offset_world_pilot/raw, never beside confirmatory data.
# `smoke` runs the 12 trials in smoke_trial_ids.json; `full` runs the whole frozen
# design and skips every record that already exists, so it resumes after `smoke`
# or after an interruption (rerun the same command).
set -euo pipefail
MODE=${1:?smoke|full}
COMMIT=${2:?commit}
C=morphology_navigation_dev
WS=${WS:-/home/checker/morphology_ws}   # host path of the repo mounted into the container
CW=${CW:-/home/roboboat/morphology_ws}
STUDY=studies/offset_world_pilot_v1
R=results/offset_world_pilot
case "$MODE" in smoke) ONLY="--only $STUDY/smoke_trial_ids.json";; full) ONLY="";; *) echo "mode?"; exit 2;; esac
[ "$(git -C $WS rev-parse --short HEAD)" = "$COMMIT" ] || { echo "ABORT: clone is not at $COMMIT"; exit 1; }
[ -z "$(git -C $WS status --porcelain --untracked-files=no)" ] || { echo "ABORT: clone is dirty"; exit 1; }
docker start $C >/dev/null
if [ "$(docker exec $C bash -c 'ps -eo cmd | grep -e "gz sim" -e "ros2 launch" -e parameter_bridge | grep -v grep | wc -l')" -ne 0 ]; then
  echo "ABORT: simulation processes still running"; exit 1
fi
LOG=$WS/results/run_offset_world_pilot_$MODE.launch.log
mkdir -p $WS/$R
# Build, run the suite first (the earlier campaigns did), then the batch, detached.
docker exec -d -e GZ_IP=127.0.0.1 \
  -e MORPHOLOGY_CONTAINER_IMAGE="$(docker inspect -f '{{.Image}}' $C)" $C bash -lc "
  set -e
  cd $CW
  exec >> results/run_offset_world_pilot_$MODE.launch.log 2>&1
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select morphology_planner modular_robot_bringup modular_robot_sim modular_robot_benchmarks
  source install/setup.bash
  python3 -m pytest -q tests
  nice -n 5 run_morphology_missions --design $STUDY/design.json \
    --raw $R/raw --artifacts $R/artifacts $ONLY
  echo 'offset_world_pilot $MODE done'
"
echo "launched; follow with: tail -f $LOG   (ends with 'offset_world_pilot $MODE done')"
