"""Fill studies/predetach_check_v1/declaration.md placeholders from predictions.json.

Usage: python3 tools/fill_predetach_declaration.py studies/predetach_check_v1
Prints the compact summary table and the hour estimate.
"""
import json, sys
from pathlib import Path
study = Path(sys.argv[1])
pred = json.loads((study/'predictions.json').read_text())
design = json.loads((study/'design.json').read_text())
cells = pred['cells']
CONTROL = {"docking_observability-00","combined_constraints-00","docking_observability-04",
           "reconfiguration_workspace-01","reconfiguration_workspace-02","reconfiguration_workspace-03"}
def group(c):
    if c['layout_id'] in CONTROL: return 'controls (6)'
    return 'same-layout refused (30)' if c['arm']=='same_layout' else 'fresh (10)'
order=[("feasibility_coupled",""),("feasibility_checked","ray_wedge"),("feasibility_checked","radius"),
       ("feasibility_probed","ray_wedge"),("feasibility_probed","radius"),("sensing_feasibility_coupled","")]
lines=["| group | cell | completed | docking_refused | no_plan | attempt_cap | rejections (total / max) | fragile checks | deadline risk |","|---|---|---|---|---|---|---|---|---|"]
for g in ['same-layout refused (30)','controls (6)','fresh (10)']:
    for m,r in order:
        cs=[c for c in cells.values() if group(c)==g and c['method']==m and c['rule']==r]
        cnt=lambda o: sum(c['outcome']==o for c in cs)
        rej=[c['rejections'] for c in cs]
        fr=sum(1 for c in cs for k in c['checks'] if k.get('fragile'))
        dr=sum(1 for c in cs if c.get('deadline_risk'))
        lines.append(f"| {g} | {m}{'/'+r if r else ''} | {cnt('completed')} | {cnt('docking_refused')} | {cnt('no_plan')} | {cnt('attempt_cap')} | {sum(rej)} / {max(rej)} | {fr} | {dr} |")
def lst(m,r,arm_filter):
    out={}
    for c in cells.values():
        if c['method']==m and c['rule']==r and arm_filter(c):
            out.setdefault(c['outcome'],[]).append(c['layout_id'].replace('docking_observability','dock').replace('combined_constraints','comb')+(f"({c['rejections']})" if c['rejections'] else ''))
    return '; '.join(f"{k}: {', '.join(sorted(v))}" for k,v in sorted(out.items()))
same=lambda c: c['arm']=='same_layout' and c['layout_id'] not in CONTROL
fresh=lambda c: c['arm']=='fresh'
text="\n".join(lines)+"\n\nPer layout (rejections in parentheses):\n\n"
for m,r in order:
    text+=f"- `{m}{'/'+r if r else ''}`, same layouts: {lst(m,r,same)}\n"
for m,r in order:
    text+=f"- `{m}{'/'+r if r else ''}`, fresh: {lst(m,r,fresh)}\n"
probe=[k.get('probe_geometry',{}).get('all_free') for c in cells.values() for k in c['checks'] if 'probe_geometry' in k]
text+=f"\nProbe geometry: at {sum(1 for p in probe if p)} of {len(probe)} probed checks every probe pose is free for the compact footprint (occupancy and raised drive obstacles).\n"
t1=pred['tiers']['1']; 
runorder={t['trial_id']:t['run_order'] for t in design['design']['trials']}
r1=sorted(runorder[t] for t in t1)
d=(study/'declaration.md').read_text()
smoke=len(pred['smoke_trial_ids'])
d=d.replace('@@HASH@@',pred['design_hash']).replace('@@TRIALS@@',str(len(design['design']['trials']))).replace('@@SMOKE@@',str(smoke)).replace('@@SMOKE_LAST@@',str(smoke-1)).replace('@@HOURS@@',str(pred['estimated_hours']['all'])).replace('@@T1@@',f"{r1[0]}..{r1[-1]}").replace('@@PREDICTIONS@@',text)
(study/'declaration.md').write_text(d)
compact = []
for g in ['same-layout refused (30)', 'controls (6)', 'fresh (10)']:
    parts = []
    for m, r in order:
        cs = [c for c in cells.values() if group(c) == g and c['method'] == m and c['rule'] == r]
        short = {'feasibility_coupled': 'feas', 'feasibility_checked': 'chk',
                 'feasibility_probed': 'prb', 'sensing_feasibility_coupled': 'sens'}[m]
        parts.append(f"{short}{'/'+r if r else ''} {sum(c['outcome']=='completed' for c in cs)}"
                     f" ({sum(c['rejections'] for c in cs)} rej)")
    compact.append(f"- {g}, completed: " + "; ".join(parts))
compact.append(f"- Estimated hours: {pred['estimated_hours']['all']} total; per tier "
               + ", ".join(f"{k} {v}" for k, v in pred['estimated_hours'].items() if k != 'all'))
print("\n".join(compact))
