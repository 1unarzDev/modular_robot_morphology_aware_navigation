# Morphology-Aware Navigation

This context describes a modular mobile robot that changes its physical assembly
while navigating.

## Language

**Module**:
A physical robot building block with a stable identity and typed docking ports.
_Avoid_: Component, part

**Port**:
A typed mechanical and electrical connection site belonging to one module.
_Avoid_: Socket, attachment point

**Topology**:
The port-to-port connection graph of the currently assembled modules.
_Avoid_: Shape, layout

**Morphology**:
A validated topology together with its physical geometry, mass properties, and
articulation state.
_Avoid_: Configuration, shape

**Locomotion mode**:
The kinematic constraints and control policy used by one morphology.
_Avoid_: Morphology, controller

**Traversal primitive**:
A short, dynamically admissible motion available to one locomotion mode.
_Avoid_: Step, move

**Reconfiguration transition**:
A certified physical procedure that changes one morphology into another.
_Avoid_: Morph, switch

**Hybrid plan**:
An ordered sequence of traversal and reconfiguration segments from a start state
to a goal state.
_Avoid_: Path, route

**Detached pod**:
A self-mobile drive module temporarily outside the core assembly during a
reconfiguration transition.
_Avoid_: Free module

