# offset_world_pilot_v1: runbook (any host)

Read `declaration.md` first: it fixes the predictions, the interim stop and the
falsification rules. This file only says how to run it. Design hash
`3a02f5769a2609c53983f00fca02ec7fc17e73c6da43690ac6d20c829f5f64d8`, 260 trials,
smoke 12. Nothing here pushes to any shared remote.

## 1. Prerequisites

- Docker with the compose plugin; `git`; `python3` 3.10+ and `jq` on the host.
- The repo's devcontainer: `.devcontainer/docker-compose.yml` plus **one**
  override file, chosen by `.devcontainer/prebuild.sh` (it copies it to
  `docker-compose.override.yml`):
  - **native Linux without an NVIDIA GPU:** `docker-compose.override.linux.yml`
    (X11 socket and `/dev`);
  - **native Linux with `nvidia-smi`:** `docker-compose.override.nvidia.yml`
    (chosen automatically);
  - **WSL2:** the existing campaigns ran in container `morphology_navigation_dev`
    started from the same compose file; use the linux override (no GPU needed;
    the sim is GPU-less). If the container already exists, just `docker start` it.
- The compose file mounts the repo (`${PWD}`) at `/home/roboboat/morphology_ws`
  inside the container and uses host networking; image `lunarzdev/astro:core`
  (override with `ASTRO_IMAGE`). Records and artifacts written in the container
  appear in the host checkout under `results/`.
- Host timing: sleeping the machine stops the run (the WSL campaigns were
  interrupted by host sleep). Block sleep for the duration.

## 2. Get the branch onto the machine without pushing

On the machine that has it (`/home/lunarz/worktrees/mrmn-offset-pilot`):
```
cd /home/lunarz/worktrees/mrmn-offset-pilot
git rev-parse --short HEAD                       # note <commit>
git bundle create ~/.cache/mrmn-offset-pilot/offset_pilot.bundle 00ac590..proposal/offset-world-pilot
git bundle verify ~/.cache/mrmn-offset-pilot/offset_pilot.bundle
sha256sum ~/.cache/mrmn-offset-pilot/offset_pilot.bundle
scp ~/.cache/mrmn-offset-pilot/offset_pilot.bundle <user>@<host>:~/
```
On the target (an existing clone of the project that has commit 00ac590, i.e.
`origin/main`; pull from the remote is fine, never push):
```
cd <clone>
git fetch origin                                  # read-only; skip if 00ac590 is present
sha256sum ~/offset_pilot.bundle                   # compare with the source
git bundle verify ~/offset_pilot.bundle
git fetch ~/offset_pilot.bundle proposal/offset-world-pilot:proposal/offset-world-pilot
git checkout proposal/offset-world-pilot
test "$(git rev-parse --short HEAD)" = "<commit>" && git status --porcelain --untracked-files=no   # must print nothing
```
Do not `git push`; do not add the bundle to the repo.

## 3. Build and the suite first

```
.devcontainer/prebuild.sh                         # picks the override file
docker compose -f .devcontainer/docker-compose.yml -f .devcontainer/docker-compose.override.yml up -d
# (or, for an existing container:  docker start morphology_navigation_dev)
docker exec morphology_navigation_dev bash -lc '
  cd /home/roboboat/morphology_ws && source /opt/ros/jazzy/setup.bash &&
  colcon build --symlink-install --packages-select morphology_planner modular_robot_bringup modular_robot_sim modular_robot_benchmarks &&
  source install/setup.bash && python3 -m pytest -q tests'
```
Expect 365 passed (pure-Python count on the branch; the container suite is
larger and was 337+ at earlier campaigns). Any failure: stop.

Check nothing is already running: `docker exec morphology_navigation_dev bash -c 'ps -eo cmd | grep -e "gz sim" -e "ros2 launch" -e parameter_bridge | grep -v grep'` must print nothing.

## 4. Smoke (12 trials, about 0.9 h)

```
WS=$PWD bash tools/run_offset_world_pilot.sh smoke <commit>
tail -f results/run_offset_world_pilot_smoke.launch.log      # ends: offset_world_pilot smoke done
```
(`WS` is the host path of the checkout; `CW` defaults to the in-container path.
The script refuses a dirty or moved tree and a busy simulator, and passes
`GZ_IP=127.0.0.1` and the image digest.) Equivalent manual command inside the
container, from the workspace root:
```
GZ_IP=127.0.0.1 MORPHOLOGY_CONTAINER_IMAGE=<docker inspect -f '{{.Image}}' morphology_navigation_dev> \
run_morphology_missions --design studies/offset_world_pilot_v1/design.json \
  --raw results/offset_world_pilot/raw --artifacts results/offset_world_pilot/artifacts \
  --only studies/offset_world_pilot_v1/smoke_trial_ids.json
```

## 5. Verify the offset took effect (from the first record)

The first smoke trial finished is enough. `R=results/offset_world_pilot/raw`,
`A=results/offset_world_pilot/artifacts`, `T=<trial id>` of an offset trial
(for example the `-m00-t06` one):
```
jq '{spec_offset: .spec.world_offset, cond: .spec.world_condition, margin: .spec.prior_margin_m,
     params: .manifest.parameters | {world_offset, prior_margin_m, world_condition},
     status: .terminal_status, failures: .reconfiguration_failures, site: .planned_transition_sites}' $R/$T.json
grep -A5 '"world_offset"' $A/$T/world.manifest.json
diff <(jq 'del(.role)' $A/$T/world.prior.manifest.json) <(jq 'del(.world_offset)' $A/$T/world.manifest.json)
```
Expect: `spec_offset` and `params.world_offset` equal (nonzero for `t0x`, all zero
for `ctl`, matching `predictions.json`); the `diff` shows **only** the
`elevated_sight_screen` centre differing (by the declared offset); the prior
manifest is what the planner loaded (check the launch log:
`grep transition_environment $A/$T/launch.log`; a `world_manifest:=` argument
appears for offset trials and `prior_margin_m:=0.04` for margin trials). Then:
```
python3 -m modular_robot_benchmarks.offset_world_pilot --out studies/offset_world_pilot_v1 --score $R
```
Smoke gates (declaration, stop rules): every `m=0 ctl` trial completed; `site_mismatch`
empty; the most confident predicted refusal (`combined_constraints-03`, m=0, 6 cm)
shows `terminal_status` in {docking_failure, unsafe_topology} or
`reconfiguration_failures >= 1`, with its `m=0.04` and no-offset partners
completing. If the 6 cm cell completes and the control/partners behave, the seam
may not be taking effect: diagnose before the full run (look at
`pod_alignment_history` for the moved pod's `connector_visible`).

## 6. Full run, with the interim look

```
WS=$PWD bash tools/run_offset_world_pilot.sh full <commit>   # skips the 12 done; ends: offset_world_pilot full done
```
Interrupted (sleep, crash, container stop)? Start the container and re-run the
same command: finished records are kept, an unfinished trial reruns from its
seed. Check the newest record parses (`jq . $R/<newest>.json >/dev/null`); if
truncated, delete it.

**Interim look.** Trials run in `run_order`; the first 52 are the 26 `m=0 t06`
cells and their 26 `m=0 ctl` controls. When `ls $R | wc -l` reaches at least 52
and all tier-1 trials exist (`--score` shows `t06_margin_0.n == 26` and
`control_cells.n == 26`), the full script keeps running; read `--score` then and, if
`toward_6cm_shipped_shows_loss` is false (fewer than 14 refusals) or
`controls_all_complete` is false, **stop the run** (`docker exec ... pkill -INT -f run_morphology_missions`),
and diagnose; do not spend the remaining ~15 h.

## 7. Archive the records with sha256

After "done" (or after a stop, labelled as partial):
```
D=studies/offset_world_pilot_v1/records
mkdir -p $D
( cd results/offset_world_pilot/raw && sha256sum *.json ) > $D/record_sha256.txt
tar -C results/offset_world_pilot -cJf $D/offset_world_pilot_v1_raw.tar.xz raw
sha256sum $D/offset_world_pilot_v1_raw.tar.xz > $D/offset_world_pilot_v1_raw.tar.xz.sha256
python3 -m modular_robot_benchmarks.offset_world_pilot --out studies/offset_world_pilot_v1 \
    --score results/offset_world_pilot/raw > $D/score.json
# archive size: split under 50 MB if committing, as confirmatory_v3 did (split -b 45M)
ls -l $D; wc -l $D/record_sha256.txt           # 260 lines when complete
```
Commit the three small files and the archive on the local branch only. Records
are never pooled with confirmatory data.

## 8. Expected wall time

Per trial 259 s mean for a completed treatment trial on these layouts (from the
confirmatory_v3 records), about 74 s for a docking refusal, 12 s for a planning
failure. Smoke about 0.9 h. Interim look (52 trials) about 3 h. Full 260 trials:
about 17.5 h expected, 18.7 h upper bound, 19 to 20 h with retries; about 16.5 h
after smoke. One container, serial; do not run two at once (shared Gazebo
discovery and DDS segments).
