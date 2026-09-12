# ADR 0002: Separate hybrid search from docking execution

- Status: accepted
- Date: 2026-09-11

## Context

Contact-rich docking simulation inside every global-search expansion is too
expensive and too sensitive to physics-engine details. A scalar transition cost
alone cannot reject unsafe module trajectories or unobservable docking sites.

## Decision

The planner admits a reconfiguration edge only after conservative trajectory,
collision, support, latch, and sensing checks. It estimates time, energy, and
failure risk from configured or learned costs. The executor separately performs
staged detach, self-mobile relocation, alignment, latch, topology observation,
and recovery through Gazebo physics and ROS interfaces.

## Consequences

The planner evaluates transition feasibility without simulating contact at each
expansion, while execution retains a fail-closed physical check. Planner
certificates remain approximations: simulator attachment is not proof of
physical docking, and certificate calibration must be evaluated against
observed transition outcomes.
