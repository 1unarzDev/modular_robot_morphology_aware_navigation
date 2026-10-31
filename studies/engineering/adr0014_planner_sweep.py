"""Planner-only: forbid reverse traversal primitives (none / narrow only / all) and
record each method's site per layout. Scratch monkeypatch; repository unchanged."""
import sys, os, json, math
ROOT = r"C:/Users/risha/gitclones/modular_robot_morphology_aware_navigation"
for p in ("morphology_planner","modular_robot_bringup","reconfiguration_executor","modular_robot_benchmarks","morphology_manager"):
    sys.path.insert(0, os.path.join(ROOT,"src",p))
os.chdir(ROOT)
from morphology_planner import planner as P, load_catalog
from modular_robot_benchmarks.manipulation_check import method_decision, DEFAULT_CATALOG, METHOD_ORDER
from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
orig = P.MorphologyAStar._motion_neighbors
MODE = ["none"]
def patched(self, state, morphology):
    for target, segment in orig(self, state, morphology):
        forbid = MODE[0] == "all" or (MODE[0] == "narrow" and morphology.id == "narrow_tandem")
        dx, dy = target.x - state.x, target.y - state.y
        if forbid and (dx or dy):
            yaw = self._yaw(state.heading)
            if dx * math.cos(yaw) + dy * math.sin(yaw) < 0:
                continue
        yield target, segment
P.MorphologyAStar._motion_neighbors = patched
orig_hit = P.MorphologyAStar._footprint_hits_drive_obstacle
MARGIN = [0.0]
def patched_hit(self, wx, wy, yaw, morphology, footprint=None):
    return orig_hit(self, wx, wy, yaw, morphology, P.inflated_footprint(footprint or morphology.footprint, MARGIN[0]))
P.MorphologyAStar._footprint_hits_drive_obstacle = patched_hit
cat = load_catalog(str(DEFAULT_CATALOG)).supported_experiment_subset()
designs = sys.argv[2:]
layouts = []
path_of = {}
for path in designs:
    d = json.load(open(path))["design"]
    found = sorted({(t["family"], t["layout_id"], t["world_seed"]) for t in d["trials"]})
    layouts += found
    path_of.update({(l, w): path for _, l, w in found})
out = []
for fam, lid, ws in layouts:
    idx = int(lid.rsplit("-",1)[1])
    sc = make_confirmatory_scenario(fam, idx, ws, generator=3)
    for mode, margin in (("none", 0.0), ("all", 0.0), ("all", 0.03)):
        MODE[0] = mode; MARGIN[0] = margin
        row = {"design": path_of[lid, ws], "layout": lid, "world_seed": ws, "forbid_reverse": mode, "drive_margin_m": margin}
        for meth in METHOD_ORDER:
            r = method_decision(meth, sc, cat, 8, 2.5)
            row[meth] = r.get("transition_site") if r["planned"] else None
        out.append(row); print(json.dumps(row), flush=True)
json.dump(out, open(sys.argv[1], "w"), indent=1)
print("DONE", flush=True)
