# ADR 0001: Plan over a finite morphology catalog

- Status: accepted
- Date: 2026-09-11

## Context

Searching arbitrary module arrangements together with world pose makes the
state space unbounded and permits geometries that have no validated locomotion
or reconfiguration procedure.

## Decision

Search the hybrid state `(x, y, heading, morphology_id)`. Each morphology ID
resolves through the canonical catalog to an explicit topology, footprint,
locomotion mode, limits, support assumptions, and cost parameters. Every
reconfiguration edge names a staged transition between two catalog entries.
New entries remain `future_concept` until their mechanics and transitions pass
the declared evidence gates.

## Consequences

Search remains finite and every plan decision is auditable against one catalog
revision. The method cannot synthesize novel assemblies online; expanding that
scope would require a separate configuration-generation and certification
module.
