# Statistical analysis plan

## Confirmatory question

Does joint route and morphology selection using transition feasibility and
sensing uncertainty improve completed missions under a common 300 s simulated
deadline? Completion requires reaching the goal with no collision, a valid
observed topology in `READY`, and no unrecovered fault.

The four methods share mechanics, localization, maps, controllers, retry limits,
and disturbance realizations:

1. `route_first_adaptation`: choose a route first and adapt morphology locally.
2. `geometry_coupled`: coupled pose–morphology planning using footprint and 2D
   reconfiguration clearance.
3. `feasibility_coupled`: add connector approach and full transition-trajectory
   feasibility.
4. `sensing_feasibility_coupled`: additionally model visibility, estimate
   covariance, and transition failure probability.

Only `compact_diff` and `narrow_tandem` enter confirmatory experiments.

## Design and outcomes

The frozen target has 12 held-out layouts in each of three families, three
stochastic replicates, and all four methods: 432 terminal records. Methods are
randomized within each layout/replicate block. World, sensing, friction, and
fault seeds are deterministically derived as independent streams; the same four
seeds are paired across methods. Pilot layouts and seeds are disjoint.

The primary outcome is mission completion. The secondary mission-level outcome
is deadline-penalized time: failed trials receive 300 s. Successful-run time is
descriptive. Mechanical work remains separate from time so the paper does not
hide a completion/work tradeoff in one scalar objective.

The two confirmatory contrasts are:

- `sensing_feasibility_coupled` minus `geometry_coupled` over all families.
- `sensing_feasibility_coupled` minus `feasibility_coupled` in docking
  observability and combined-constraint families.

Report absolute paired completion-rate differences with 95% confidence
intervals. Bootstrap whole layouts within each family, retaining every method
and replicate in a sampled cluster. Obtain two-sided paired randomization p
values by sign-flipping layout-level mean differences. Apply Holm correction to
the two p values. Report effect sizes and intervals regardless of significance.

## Diagnostics and exclusions

Before analysis, require exactly one terminal record for every frozen trial ID,
all four methods in every pair block, identical disturbance seeds within a block,
and one commit/configuration/design hash. Keep every robot failure. Only a
predeclared infrastructure failure may be rerun, and both the superseded record
and reason must remain in an audit log.

Also report planning latency, expanded states, mechanical work, collision and
clearance metrics, docking attempts and recovery, localization RMSE, failure
taxonomy, transition Brier score, and reliability bins. Use pilot variance to
plot prospective precision or power; do not report retrospective observed power.

Raw terminal records are append-only JSON. Generated CSV, Markdown, and figures
live in a separate derived directory. The synthetic `edge_observations.csv` is
excluded from this pipeline.
