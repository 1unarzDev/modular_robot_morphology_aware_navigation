"""Why does the single-route rung fail? Traced on the 60 tuning layouts, outside the design.

Engineering evidence for the paper's description of `route_first_adaptation`; not part
of the declared retry check. Usage: python tools/trace_route_first_failures.py <out.json>
"""
import json, sys
from collections import Counter
from pathlib import Path
ROOT = str(Path(__file__).resolve().parents[1])
for pkg in ("morphology_planner", "modular_robot_bringup", "reconfiguration_executor",
            "modular_robot_benchmarks", "morphology_manager"):
    sys.path.insert(0, f"{ROOT}/src/{pkg}")
from modular_robot_benchmarks import route_first_retry as m
from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from morphology_planner import (HybridState, load_catalog, CoupledTransitionPolicy,
                                station_sensing_provider)
from morphology_planner.route_first import RouteFirstAdaptationPlanner, SpatialState

cat = load_catalog(str(m.DEFAULT_CATALOG)).supported_experiment_subset()
c2n = next(t for t in cat.transitions if t.id == "compact_to_narrow")
rows, kinds = [], Counter()
for fam, i, s in m.tuning_seeds():
    sc = make_confirmatory_scenario(fam, i, s, generator=3)
    pol = CoupledTransitionPolicy(
        m.TREATMENT, cat, sc.grid, 8, sc.transition_obstacles,
        sensing_provider=station_sensing_provider(cat, sc.fiducial_stations,
                                                  sc.transition_obstacles, sc.grid, 8))
    rf = RouteFirstAdaptationPlanner(cat, sc.grid, heading_bins=8, transition_validator=pol,
                                     drive_obstacles=sc.transition_obstacles)
    st = rf.hybrid.repaired_start(HybridState(*sc.start, 0, "compact_diff"))
    route, _ = rf._spatial_route(SpatialState(st.x, st.y, st.heading), sc.goal, 2.5)
    n = len(route)

    def step_ok(idx, mo):
        p, q = route[idx], route[idx + 1]
        src = HybridState(p.x, p.y, p.heading, mo)
        exp = HybridState(q.x, q.y, q.heading, mo)
        return any(t == exp for t, _ in rf.hybrid._motion_neighbors(src, cat.morphologies[mo]))

    compact_ok = [step_ok(k, "compact_diff") for k in range(n - 1)]
    narrow_ok = [step_ok(k, "narrow_tandem") for k in range(n - 1)]
    compact_reach = next((k for k, ok in enumerate(compact_ok) if not ok), n - 1)
    sites = [k for k in range(compact_reach + 1)
             if pol(c2n, HybridState(route[k].x, route[k].y, route[k].heading, "compact_diff"))]
    # From a feasible site, can the narrow body follow the rest (ignoring the return)?
    def narrow_from(k):
        j = k
        while j < n - 1 and narrow_ok[j]:
            j += 1
        return j
    best = max((narrow_from(k) for k in sites), default=None)
    if not sites:
        kind = "no_site_before_compact_blocked"
    else:
        last = max(sites)
        blocked = narrow_from(last)
        p, q = route[blocked], route[min(blocked + 1, n - 1)]
        move = ("turn_in_place" if (p.x, p.y) == (q.x, q.y)
                else "diagonal" if p.x != q.x and p.y != q.y else "straight")
        kind = f"narrow_blocked_after_last_site:{move}" if blocked < n - 1 else "narrow_reaches_goal"
    kinds[kind] += 1
    rows.append({"family": fam, "layout_index": i, "route_len": n, "compact_reach": compact_reach,
                 "sites": len(sites), "last_site": max(sites) if sites else None,
                 "narrow_best": best, "kind": kind})
    print(fam, i, rows[-1]["compact_reach"], n, len(sites), rows[-1]["last_site"], best, kind, flush=True)
print(dict(kinds))
json.dump(rows, open(sys.argv[1], "w"), indent=1)
