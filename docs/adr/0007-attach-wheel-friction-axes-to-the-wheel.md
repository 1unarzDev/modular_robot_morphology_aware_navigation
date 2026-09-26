# ADR 0007: Attach the wheel friction axes to the wheel

- Status: **proposed** (awaiting author decision)
- Date: 2026-09-26

## Context

`articulated_pod/model.sdf.in:19-20` declares anisotropic Coulomb friction on
every wheel collision, `mu 1.2` and `mu2 0.08`, and `fdir1` is not specified
there or anywhere else in the repository. Without `fdir1` the friction principal
axes are not attached to the wheel, so which coefficient resists rolling and
which resists lateral scrub depends on the robot's orientation in the world.

Every wheel that has touched ground in this study is one of these: the assembled
robot includes `articulated_pod_0..5`, and the detached-pod world reaches the
same template through `self_mobile_pod`. (`drive_pod/model.sdf`, which an earlier
write-up blamed, is a legacy model that `CMakeLists.txt` still configures and
nothing loads.)

This surfaced as the ADR 0006 blocker: `sensing_feasibility_coupled` planned a
site 0.20 m off the route, and `_align_to_site` commanded 0.271 rad/s of in-place
rotation for 59 continuous seconds while the body turned 0.0000 rad, at a world
heading of -1.577 rad. `studies/engineering/rotation_authority_sweep.json` then
eliminated the commanded rate, the world, position, teleporting, attitude,
suspension and actuator stall, and measured authority full at 0 and 180 degrees
and exactly zero at both +90 and -90 -- a 180-degree period, which is the
signature of world-fixed friction axes.

## Measurement

`studies/engineering/wheel_friction_intervention.json` confirms the cause by
intervention. The protocol was declared at `e2bc940` before any of it ran. The
patch was applied to the built and installed model tree inside the development
container only, never to the source template; the tree was restored from
byte-for-byte backups and all six sha256 sums matched before the final control.

At 0.271 rad/s in the retained smoke world, at the position the collapse was
measured at:

| Tree | Imposed yaw | Rotation, fraction of command | Straight (m) | Gate |
|---|---|---|---|---|
| unmodified | +1.5708 | 0.000 / 0.000 | 0.5830 | fails |
| unmodified | 0 (from the earlier sweep) | 0.650 / 0.644 | 0.7397 | passes |
| isotropic, `mu2` 1.2 | +1.5708 | 0.458 / 0.472 | 0.7407 | passes |
| isotropic, `mu2` 1.2 | -1.5708 | 0.458 / 0.472 | 0.7407 | passes |
| isotropic, `mu2` 1.2 | 0.0000 | 0.458 / 0.472 | 0.7407 | passes |
| `fdir1 1 0 0` | +1.5708 | 0.741 / 0.742 | 0.7336 | passes |
| `fdir1 1 0 0` | -1.5708 | 0.742 / 0.742 | 0.7336 | passes |
| `fdir1 1 0 0` | 0.0000 | 0.740 / 0.746 | 0.7374 | passes |
| restored | +1.5708 | 0.000 / 0.000 | 0.5830 | fails |

Four things follow, and the third and fourth are why this is an ADR rather than
a bug fix.

1. **The cause is confirmed.** Under either variant the three imposed headings
   return the same rotation and the same straight travel to four decimals. The
   imposed yaws are recorded per run from the records' own first sample, so the
   invariance is not a failed teleport, and the record digests differ.
2. **The two repairs are not equivalent.** `fdir1 1 0 0` keeps the declared
   anisotropy and rotates at 74% of command at every heading, better than the
   most favourable heading has ever produced. Isotropy removes the heading
   dependence too but yields 46%, because skid-steer rotation is resisted by
   lateral scrub and isotropy raises that resistance from 0.08 to 1.2. The
   engine reads `fdir1`: that was the stated risk and it did not materialize.
3. **Straight travel is heading-dependent as well**, by 21.2% at +/-90, against
   committed figures collected at the spawn heading near zero. The confirmatory
   layouts route in all directions and no layout has ever been run at a heading
   near +/-90.
4. **Adopting any repair moves every recorded figure.** Gate 0 item 1, the
   20-run round-trip audit and the 42/42 fault matrix were all collected on the
   unmodified model, and the fault-matrix and round-trip gates already need
   re-qualification for the ADR 0006 work. The repair makes those figures
   better, not worse, but it makes them different.

## The full curve, and what it costs

`studies/engineering/heading_traction_curve.json` measures the dependence across
13 headings on the unmodified model, in an obstruction-free world so the straight
stage measures traction rather than geometry. Two results bear on the options.

**Rotation authority falls off across most of the circle, not in a notch.**
Stage totals cannot answer this -- a 4 s stage turns the body up to 1 rad, so a
total integrates authority over the arc traversed -- but the records sample true
pose at 0.05 s, and local rate against instantaneous heading over 2200 windows
is symmetric about 90 degrees with a 180-degree period:

| Heading, mod 180 | 0--10 | 20--30 | 40--50 | 60--70 | 80--90 | 90--100 | 130--140 | 170--180 |
|---|---|---|---|---|---|---|---|---|
| Fraction of command | 0.733 | 0.656 | 0.453 | 0.261 | 0.042 | 0.090 | 0.441 | 0.735 |

Authority is down to 78% of its best value by 30 degrees, 62% by 45 and 20% by
75. The gate's yaw stages need a fraction of 0.267 (0.25 rad/s for 1.5 s against
a 0.10 rad floor), which the medians hold only up to about 60 degrees and again
from about 120, so **the signed-motion yaw floor projects to fail for roughly a
third of all headings**.

**The translation deficit is a startup penalty.** Instantaneous speed after the
ramp is 0.927--0.928 of command at every heading; the time to reach 85% of
command rises from 0.05--0.10 s near 0 and 180 to 1.44--1.54 s near 90. The
friction limit caps acceleration, not constant-velocity rolling. So the gate's
forward floor cannot be assessed by scaling: its stage is 0.12 m/s for 1.0 s,
comparable to the ramp, and a short stage is hurt proportionally more than a
long one.

**The detached pod is affected in travel and not in yaw**, 17--21% short at
+/-90 with rotation unchanged, because a two-wheel pod turning in place needs
little tractive force. Both halves of Gate 0 item 1 are therefore heading-
dependent, which was previously inference.

## Options

- **A. Set `fdir1 1 0 0` on the wheel collisions (recommended).** The wheel
  collision pose rotates +1.5708 rad about x, so collision-frame x is the link's
  forward axis: `fdir1 1 0 0` puts `mu` 1.2 along rolling and `mu2` 0.08 along
  lateral scrub, which is what the declaration reads as intending. Measured at
  74% of commanded yaw rate at every heading. Cost: every recorded motion figure
  must be re-collected, and the gates' margins change.
- **B. Make the wheel friction isotropic.** One number, no dependence on how the
  engine interprets `fdir1`, and it also removes the heading dependence. Costs
  the low lateral scrub the casters and the `mu2` value were tuned for (see the
  pod-pitch and scrub work in `docs/current_status.md`), and rotates at 46%
  rather than 74%. Same re-collection cost.
- **C. Leave the model and constrain the study to favourable headings.** No
  re-collection, but the curve prices it: the band that keeps the gate's own yaw
  floor is roughly +/-60 degrees of the favourable axis, so a third of all
  headings are excluded, and authority is already down 38% at 45 degrees inside
  the retained band. Every published figure would carry a heading term that is
  now measured rather than unknown, and it leaves the ADR 0006 sensing contrast
  unexecutable, since its sites are off-route by construction.
- **D. Repair the model and separately give the alignment law a floor.** A is
  necessary but does not by itself make `_align_to_site` robust: the law refuses
  to translate while `|bearing| > 0.15 rad` and its bearing branch has no angular
  floor, so any future rotation failure still becomes a hard stop rather than
  degraded progress. `reconfiguration_executor` already applies
  `min_pod_angular = 0.30` to pod in-place turns (`node.py:408-417`).

## Decision (proposed)

Adopt A, and treat D's second half as a separate repair rather than a
consequence of it. Then re-collect, in this order, because each stage's evidence
is a precondition for the next: Gate 0 item 1, the 20-run round-trip gate, the
fault matrix, and only then the ADR 0006 station smoke runs that exposed this.

Two things should be added with the repair, both cheap:

- A test that fails any collision declaring `mu != mu2` without `fdir1`, so the
  class of defect cannot return silently.
- A signed-motion measurement at the worst heading in the re-collected Gate 0
  item 1, so heading invariance is part of the gate rather than an engineering
  note. The gate currently measures one heading, and the projection above says a
  third of headings would fail its yaw floor on the unmodified model. Measure it
  at the gate's own stage rather than scaling the 4 s figures, because the
  translation penalty is a startup cost.

`site_position_tolerance` remains a separate open decision: at 0.030 m it sits
below the localization error the controller must servo against, and no friction
repair changes that.
