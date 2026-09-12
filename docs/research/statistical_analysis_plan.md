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

The target design has 12 held-out layouts in each of three families, three
stochastic replicates, and all four methods: 432 terminal records. Freeze the
generated JSON design before the confirmatory run and report its SHA-256 hash.
Methods are randomized within each layout/replicate block. World, sensing,
friction, and fault seeds are deterministically derived as independent streams;
the same four seeds are paired across methods. Pilot layouts and seeds are
disjoint. Layout is the independent sampling unit; replicates estimate
within-layout disturbance sensitivity and do not inflate the layout count.

The primary outcome is mission completion. The secondary mission-level outcome
is deadline-penalized time: failed trials receive 300 s. Successful-run time is
descriptive. Mechanical work remains separate from time so the paper does not
hide a completion/work tradeoff in one scalar objective.

The primary estimand is the layout-balanced average effect over the generated
layout population in each declared family scope. Each layout receives equal
weight after averaging its three paired disturbance replicates. This prevents
replicates from being treated as independent environments. The all-family
contrast weights layouts equally; because the frozen design has the same number
of layouts per family, it also weights families equally.

The two confirmatory contrasts are:

- `sensing_feasibility_coupled` minus `geometry_coupled` over all families.
- `sensing_feasibility_coupled` minus `feasibility_coupled` in docking
  observability and combined-constraint families.

Report absolute paired completion-rate differences with 95% confidence
intervals. Bootstrap whole layouts within each family, retaining every method
and replicate in a sampled cluster. Repeatedly sampled layouts retain their
multiplicity. Obtain two-sided paired randomization p
values by sign-flipping layout-level mean differences. Enumerate every sign
assignment when a contrast has at most 20 layouts (including small pilots);
otherwise use the predeclared seeded Monte Carlo draw count and report that
count with the result. Apply Holm correction to
the two p values. Report effect sizes and intervals regardless of significance.
The randomization test targets a sharp symmetry/null assumption at the layout
level; the bootstrap interval is the main uncertainty summary for the average
effect. Report the Monte Carlo standard error of every simulated p value, and
state when exact-test discreteness limits attainable p values.

For the secondary outcome, report treatment-minus-control differences in
deadline-penalized seconds with the same stratified layout bootstrap. Negative
values favor the treatment. Do not assign confirmatory p values to secondary or
family-level results. Report family-level effects as heterogeneity diagnostics,
not independent confirmatory tests. Estimate transition Brier scores and
reliability bins separately by method so a pooled score cannot hide method
differences.

Before unblinding confirmatory outcomes, freeze two sensitivity analyses: (1) a
paired hierarchical logistic model with method fixed effects and layout random
intercepts, reported as marginal probability differences; and (2) a deadline-
time analysis with a 300 s point mass for failures. These support the
predeclared nonparametric analysis and do not replace it. Do not choose among
models based on which produces a smaller p value.

## Sample-size gate

The current 12-layout-per-family target is a resource-based starting point, not
yet a justified sample size. After the disjoint pilot, estimate the control
completion rate, plausible treatment effect, layout-level logit variance, and
paired disturbance correlation. Run the prospective hierarchical simulation
implemented by `morphology_study power` for both primary contrast scopes: 36
layouts for the all-family contrast and 24 layouts for the two sensing families.
Use alpha 0.025 as a conservative Holm planning threshold. Archive assumptions,
seed, Monte Carlo standard error, and output JSON. Increase layouts if either
contrast has inadequate power for the predeclared smallest effect of interest.
Do not tune the effect threshold to the pilot result, and do not report
retrospective observed power.

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

Treat planner-decision diagnostics as manipulation checks rather than outcomes.
For every expanded transition edge, retain the method, transition, planning
state, accepted/rejected flag, checked trajectory samples, and structured
rejection reasons. Retain a deterministic signature of the selected route and
transition sites. Report rejection-reason counts by method and the fraction of
paired blocks in which each baseline selects a different route or transition
site from `sensing_feasibility_coupled`. These checks establish that the
ablations changed the intended decision mechanism; they do not replace the
mission-completion estimand and receive no significance tests.

For mechanism evidence, report transition prediction coverage as well as
calibration: the number of attempted transitions with a logged probability,
the Brier score by method, reliability bins with counts, and every structured
rejection reason. Calibration is uninterpretable unless the executor logs both
a pre-action prediction and a terminal outcome.

Treat collision, clearance, localization error, and transition failures as
secondary outcomes with explicit denominators. Define minimum clearance over
the full robot geometry at synchronized evaluator timestamps. Localization RMSE
compares the autonomy pose with evaluator-only truth after time alignment. No
ground-truth stream may feed planning, control, recovery, or readiness.

The smallest effect of interest is intentionally not filled in before the
disjoint pilot. The pilot decision record must set it from operational value
(for example, the minimum absolute completion gain that justifies added
planning and reconfiguration complexity), record the rationale, and then run
power across a conservative range of nuisance assumptions. It must not define
the threshold by rounding or otherwise adapting to the observed treatment
effect.

Raw terminal records are append-only JSON. Generated CSV, Markdown, and figures
live in a separate derived directory. The synthetic `edge_observations.csv` is
excluded from this pipeline.

The analysis command must reject incomplete designs, changed trial
specifications, mixed commit/configuration/design hashes, and unpaired
disturbance seeds. Figures and tables are deterministic derivatives of accepted
terminal records. Any rerun for a declared infrastructure failure receives a
new audit entry; the original terminal record remains immutable.
