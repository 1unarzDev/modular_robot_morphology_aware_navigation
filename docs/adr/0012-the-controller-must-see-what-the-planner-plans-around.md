# ADR 0012: The controller must see what the planner plans around

- Status: **accepted** (2026-10-30, a pilot finding; implementation and
  re-qualification are owed before the confirmatory freeze)
- Date: 2026-10-30

## Context

ADR 0004 made raised obstacles -- the `raised_transition_shelf`, 0.10--0.18 m
high -- constrain driving in the hybrid planner, so every method plans its
route around them. They are below the lidar plane, so they never enter `/scan`,
and the exported occupancy map holds only walls. Nav2's global and local
costmaps therefore do not contain them, and the MPPI controller that executes a
leg is free to cut a corner through one.

Under layout generator 1 this never mattered: every route ran straight along
the door line, and nothing a controller does on a straight leg reaches the
shelf. ADR 0011's generator 2 puts the start and goal off the door line, so the
narrow body now turns toward the doorway, and the pilot (2026-10-30, at
`6409b15`) recorded the consequence. In its first 30 trials, all three
`collision` outcomes were `pod_0` touching the `raised_transition_shelf` at a
heading of 0.57--0.63 rad on the diagonal approach to an off-line doorway, after
a successful reconfiguration; the planner had routed around the shelf and the
controller had not.

This is a defect in the autonomy stack, not a property of the methods: one
layer plans around an obstacle that the layer executing the plan cannot see.
It is the kind of finding a pilot exists to surface.

## Decision

Give Nav2 the same prior the planner has. The declared raised obstacles are
rasterized into a **keepout filter mask** served beside the map, and both
costmaps apply Nav2's `KeepoutFilter` with it. The occupancy map AMCL localizes
against is unchanged, because cells the lidar can never see would only degrade
localization. Nothing here changes a scenario, a method or a threshold, and
every method gets the same costmaps.

Not adopted: inflating the shelf in the planner instead. It would move the
transition sites the manipulation check fixes, which is a manipulation change,
and it would leave the controller exactly as blind.

## Consequences

- The pilot runs to completion unchanged: it measures the stack as frozen, its
  records stay as collected, and its nuisance estimates are read by the rules
  fixed at `76ffe6e`. The collision outcomes it records describe the old
  stack and are reported as a pilot finding, not hidden.
- After the fix, the round-trip gate is re-run on the generator-2 layouts
  before the confirmatory freeze, since the costmaps change for every method.
- The pilot also exposed a labelling bug, fixed alongside: route-first's
  planning failure, whose message names "feasible transitions", was recorded as
  `docking_failure`. Completion was never affected; the failure taxonomy was.
