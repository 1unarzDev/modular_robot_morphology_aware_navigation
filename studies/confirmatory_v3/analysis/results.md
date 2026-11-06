# Confirmatory analysis

Design hash: `c939f12699e1eca0e7f74bfe9f5dc0ef69343c92fe82c1ce95e5c6af1dad5734`

Terminal mission records: 720

## Completion and cost

| Method | Completion (cluster 95% CI) | Deadline-penalized time (s) | Work (J) |
|---|---:|---:|---:|
| route_first_adaptation | 0.083 [0.017, 0.150] | 286.10 | 42.54 |
| geometry_coupled | 0.250 [0.150, 0.367] | 262.15 | 238.52 |
| feasibility_coupled | 0.500 [0.417, 0.600] | 227.10 | 312.52 |
| sensing_feasibility_coupled | 0.928 [0.861, 0.983] | 183.44 | 495.19 |

## Primary completion contrasts

| Treatment − control | Difference (95% cluster CI) | Holm p |
|---|---:|---:|
| sensing_feasibility_coupled − geometry_coupled | 0.678 [0.561, 0.794] | 2e-05 |
| sensing_feasibility_coupled − feasibility_coupled | 0.642 [0.492, 0.792] | 2e-05 |

## Secondary deadline-penalized-time contrasts

Negative values favor the treatment. These intervals are descriptive and receive no confirmatory p value.

| Treatment − control | Difference in seconds (95% cluster CI) |
|---|---:|
| sensing_feasibility_coupled − geometry_coupled | -78.71 [-92.31, -64.41] |
| sensing_feasibility_coupled − feasibility_coupled | -66.01 [-80.97, -49.97] |

## Planner manipulation checks

These are descriptive checks of the planned ablations.

| Baseline compared with full method | Paired blocks | Different decision fraction |
|---|---:|---:|
| route_first_adaptation | 15 | 1.000 |
| geometry_coupled | 168 | 0.744 |
| feasibility_coupled | 168 | 0.482 |

## Hierarchical logistic sensitivity

Marginal completion-probability differences from a binomial-logit model with a Gaussian layout random intercept. Intervals resample whole layouts within family.

| Treatment − control | Marginal difference (95% cluster CI) | Layout SD |
|---|---:|---:|
| sensing_feasibility_coupled − geometry_coupled | 0.538 [0.296, 0.715] | 7.660 |
| sensing_feasibility_coupled − feasibility_coupled | 0.536 [0.303, 0.747] | 7.730 |

## Secondary measures

Counts carry their denominators; no p value is attached. Infrastructure attempts are launch attempts that failed before the mission and are not mission outcomes.

| Method | Infrastructure attempts | Contact | Localization loss | Transition failures | Site placement median / p95 / max (m) |
|---|---:|---:|---:|---:|---:|
| route_first_adaptation | 4/184 | 0/180 | 0/180 | 0/30 | 0.011 / 0.030 / 0.035 (n=30, 2 beyond 0.03 m) |
| geometry_coupled | 6/186 | 45/180 | 14/180 | 90/225 | 0.021 / 0.039 / 0.053 (n=225, 41 beyond 0.03 m) |
| feasibility_coupled | 1/181 | 0/180 | 0/180 | 90/270 | 0.018 / 0.034 / 0.052 (n=269, 45 beyond 0.03 m) |
| sensing_feasibility_coupled | 1/181 | 0/180 | 0/180 | 0/336 | 0.018 / 0.035 / 0.055 (n=336, 34 beyond 0.03 m) |

Structured transition rejection counts are retained in `analysis.json`.
