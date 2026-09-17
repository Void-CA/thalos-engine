# ADR-017: `thalos-api` as the curated public boundary

## Status

Accepted

## Context

Before this decision, external consumers reached engine types through a
re-export shim (`thalos_runtime::engine::core::…`) that mirrored the internal
crate graph. A naive replacement — `pub use thalos_core as core;` and similar —
would have published the *entire* internal surface as stable API, including
modules we already know we want to move or remove (legacy ADR-014 inventory
types, the analysis subsystem, the kinematic catalog).

## Decision

`thalos-api` is the **single public boundary** of the engine. It is a thin,
**concept-curated** facade:

- Namespaces reflect **concepts**, not the internal filesystem:
  `core`, `analysis`, `models`, `catalog`, `math`, `planning`, `lang`,
  `semantic`, `importer`, `ports`, `document`, `visual`.
- It is **curated**: internal crates are re-exported module-by-module (or
  item-by-item), never aliased wholesale.
- **Being in the `thalos-engine` workspace does not make a crate public.** The
  workspace may contain internal implementation crates.

Canonical concept splits:

- `models` = structural / URDF data (`thalos-models`).
- `catalog` = built-in kinematic model catalog (`core::models`: `RobotModel`,
  `RobotRegistry`).
- `core::station::Station` (engine, resource-binding) is a different concept
  from `thalos_runtime::station::Station` (application, operational).

## Guard

`thalos-api/tests/public_surface.rs` compiles using **only** `thalos-api` and
enumerates the public concepts. If a concept becomes reachable only through an
internal crate, the probe stops compiling.

## Consequences

- Internal crates can be reorganized without breaking the boundary, as long as
  the `thalos-api` path is preserved (the namespace is decoupled from the
  physical crate).
- Test doubles are exposed opt-in via the `thalos-api/test-support` feature.

## Related

- ADR-016 (boundary), ADR-018 (closure).
