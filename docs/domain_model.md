# Domain model

This project uses the following terms consistently in code, experiment records,
and publications.

- **Module:** physical robot building block with stable identity and typed ports.
- **Port:** mechanical and electrical connection site owned by one module.
- **Topology:** port-to-port graph observed for the current assembly.
- **Morphology:** validated topology plus geometry, mass properties, and
  articulation state.
- **Locomotion mode:** kinematic constraints and control policy of a morphology.
- **Traversal primitive:** short dynamically admissible motion for one
  locomotion mode.
- **Reconfiguration transition:** validated physical procedure that changes one
  morphology into another.
- **Transition site:** world pose at which a reconfiguration transition is
  considered or executed.
- **Hybrid state:** planar pose, heading, and morphology.
- **Hybrid plan:** ordered traversal and reconfiguration segments from a start
  state to a goal state.
- **Detached pod:** self-mobile drive module temporarily outside the assembly.
- **Observed topology:** topology reconstructed from connector observations.
  It is authoritative for safety even when it differs from the requested
  topology.
- **Evaluator truth:** simulator state recorded only for scoring and diagnosis;
  autonomy must never consume it.

Use *configuration* only for software settings. Use *shape* only for geometry,
and *route* only for the spatial projection of a hybrid plan.
