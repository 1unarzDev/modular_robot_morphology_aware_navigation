"""Narrow door approach: planned straight run-in vs arrival error vs passage.
Usage (in the container): door_runin.py OUT RAW_DIR...  Generator-3 records only."""
import glob, json, math, sys
from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from morphology_planner.grid import OCCUPIED

def door(family, index, seed):
    sc = make_confirmatory_scenario(family, index, seed, generator=3); g = sc.grid
    for x in range(5, g.width - 5):
        occ = {y for y in range(g.height) if g.value(x, y) >= OCCUPIED}
        if len(occ) > g.height * 0.6:
            free = [y for y in range(1, g.height - 1) if y not in occ]
            wx = g.origin_x + x * g.resolution
            return wx, wx + g.resolution, g.origin_y + free[0] * g.resolution, g.origin_y + (free[-1] + 1) * g.resolution
    return None

def yaw(p): return 2 * math.atan2(p["orientation_z"], p["orientation_w"])

rows = []
for d in sys.argv[2:]:
    for f in sorted(glob.glob(d + "/*.json")):
        r = json.load(open(f)); s = r["spec"]
        idx = int(s["layout_id"].rsplit("-", 1)[1])
        wall_lo, wall_hi, door_lo, door_hi = door(s["family"], idx, s["world_seed"])
        door_c = (door_lo + door_hi) / 2
        th = sorted(r["topology_history"], key=lambda e: e["time_s"])
        def morph(t):
            m = "compact_diff"
            for e in th:
                if e["time_s"] <= t: m = e["morphology"]
            return m
        pd = sorted([x for x in r["controller_diagnostics"]["pod_drive_diagnostic_samples"] if x["pod"] == "pod_0"], key=lambda x: x["time_s"])
        narrow = [x for x in pd if morph(x["time_s"]) == "narrow_tandem"]
        if not narrow: continue
        crossed = any(x["core_world_x"] > wall_hi + 0.3 for x in narrow)
        # arrival: first narrow sample with nose (core + 0.81 m) at or past the wall face
        arr = next((x for x in narrow if x["core_world_x"] + 0.81 * math.cos(x["core_world_yaw"]) >= wall_lo - 0.02), None)
        # planned run-in on the first plan: walk back from the first route point past the wall
        route = r["planned_route"]; runin = None
        for i, p in enumerate(route):
            if i and math.hypot(p["x"] - route[i-1]["x"], p["y"] - route[i-1]["y"]) > 0.2: break   # next plan
            if p["x"] >= wall_lo:
                j = i
                while j > 0 and abs(math.remainder(yaw(route[j-1]) - yaw(p), 2 * math.pi)) < 1e-3 \
                        and math.hypot(route[j]["x"] - route[j-1]["x"], route[j]["y"] - route[j-1]["y"]) < 0.2:
                    j -= 1
                runin = wall_lo - route[j]["x"]; break
        row = {"trial": s["trial_id"], "status": r["terminal_status"], "crossed": crossed,
               "runin_m": None if runin is None else round(runin, 3)}
        if arr:
            lat = arr["core_world_y"] - door_c; hd = math.remainder(arr["core_world_yaw"], 2 * math.pi)
            row.update(arrival_lateral_m=round(lat, 3), arrival_heading_rad=round(hd, 3),
                       nose_offset_m=round(abs(lat + 0.81 * math.sin(hd)), 3))
        rows.append(row)
json.dump(rows, open(sys.argv[1], "w"), indent=1)
for x in rows:
    print(f'{x["trial"][-62:]:62s} {x["status"]:19s} crossed={x["crossed"]!s:5s} runin={x["runin_m"]} nose_off={x.get("nose_offset_m")} lat={x.get("arrival_lateral_m")} hd={x.get("arrival_heading_rad")}')
