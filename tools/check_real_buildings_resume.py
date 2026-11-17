"""Check across real process kills that a resumed building is the unbroken one.

The real-buildings study checkpoints a building's draw loop before every draw
(`real_buildings.draw_missions`), because its slowest buildings take longer
than the host stays up. `tests/test_real_buildings.py` proves the bookkeeping
with a stand-in planner; this runs the study's own worker, planner included, on
one scanned building twice -- once in a single process, and once in processes
killed every `--kill-after` seconds and restarted from the checkpoint until one
finishes -- and compares the two entries byte for byte.

Give it development buildings only (`minival`, master seed 1): it prints
nothing of a building's outcomes, but a sample building must not be evaluated
outside the declared run.

    .venv-host/Scripts/python tools/check_real_buildings_resume.py \
        --building C:/Users/risha/datasets/hm3d/minival/00801-HaxA7YrQdEC/HaxA7YrQdEC.glb \
        --master-seed 1 --kill-after 45 \
        --record studies/real_buildings/resume_check_HaxA7YrQdEC.json
"""
import argparse
import hashlib
import json
import multiprocessing
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for pkg in ("morphology_planner", "modular_robot_bringup", "reconfiguration_executor",
            "modular_robot_benchmarks", "morphology_manager"):
    sys.path.insert(0, str(ROOT / "src" / pkg))

CATALOG = ROOT / "src" / "modular_robot_description" / "config" / "morphologies.yaml"


def _evaluate(building: str, master_seed: int, draws, destination: str) -> None:
    from modular_robot_benchmarks.real_buildings import _worker
    entry = _worker((building, master_seed, CATALOG, draws))
    Path(destination).write_text(json.dumps(entry, sort_keys=True), encoding="utf-8")


def _attempts(state: Path):
    try:
        return json.loads(state.read_text(encoding="utf-8"))["stats"]["attempts"]
    except (OSError, ValueError, KeyError):
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--building", type=Path, required=True)
    parser.add_argument("--master-seed", type=int, required=True)
    parser.add_argument("--kill-after", type=float, default=45.0,
                        help="seconds each broken session lives; doubled whenever a "
                             "session ends with no draw gained")
    parser.add_argument("--record", type=Path, default=None)
    args = parser.parse_args()
    building = str(args.building)
    with tempfile.TemporaryDirectory() as scratch:
        scratch = Path(scratch)
        started = time.monotonic()
        unbroken = scratch / "unbroken.json"
        process = multiprocessing.Process(
            target=_evaluate, args=(building, args.master_seed, None, str(unbroken)))
        process.start()
        process.join()
        unbroken_seconds = time.monotonic() - started
        if not unbroken.exists():
            raise SystemExit("the unbroken run wrote no entry")

        draws, resumed = scratch / "draws", scratch / "resumed.json"
        draws.mkdir()
        state = draws / f"{args.building.stem}.json"
        kill_after, attempts_at_kill = args.kill_after, []
        while True:
            process = multiprocessing.Process(
                target=_evaluate, args=(building, args.master_seed, draws, str(resumed)))
            process.start()
            process.join(kill_after)
            if not process.is_alive():
                break
            process.terminate()      # TerminateProcess: no handler runs, as in a shutdown
            process.join()
            attempts_at_kill.append(_attempts(state))
            if len(attempts_at_kill) > 1 and attempts_at_kill[-1] == attempts_at_kill[-2]:
                kill_after *= 2
            print(f"killed {len(attempts_at_kill)} times; draws on record "
                  f"{attempts_at_kill[-1]}", flush=True)
        if not resumed.exists():
            raise SystemExit("the resumed run wrote no entry")
        first, second = unbroken.read_bytes(), resumed.read_bytes()
        entry = json.loads(first)
        record = {
            "purpose": "real_buildings_resume_equivalence_check",
            "building": args.building.stem,
            "master_seed": args.master_seed,
            "errored": "error" in entry,
            "draws": entry["draw"]["attempts"] if entry.get("draw") else None,
            "missions": len(entry["missions"]),
            "unbroken_seconds": round(unbroken_seconds, 1),
            "kills": len(attempts_at_kill),
            "draws_on_record_at_each_kill": attempts_at_kill,
            "unbroken_entry_sha256": hashlib.sha256(first).hexdigest(),
            "resumed_entry_sha256": hashlib.sha256(second).hexdigest(),
            "identical": first == second,
        }
    print(json.dumps(record, indent=2, sort_keys=True))
    if args.record is not None:
        args.record.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n",
                               encoding="utf-8")
    if not record["identical"] or not record["kills"]:
        raise SystemExit("resume check failed: the entries differ, or nothing was killed")


if __name__ == "__main__":
    main()
