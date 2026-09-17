# ADR-016: Engine / Industrial Boundary

## Status

Accepted

## Context

`thalos-engine` is the reusable robotics platform. It was forked from Thalos
Industrial together with an all-in-one `thalos-engine` facade crate. That facade
was later deleted, but a compatibility shim (`thalos_runtime::engine`) remained
with 226 internal uses, and `thalos-runtime` itself — the application/composition
layer (station, execution sessions, scene, telemetry, persistence ports) — still
lived inside the engine repository.

Consequences of the old layout:

- `thalos-runtime` reached back into `thalos-industrial` for `thalos-persistence`
  in dev-dependencies, so the engine could not be tested or built standalone.
- CI could not resolve the sibling/inter-repo paths.
- `thalos-runtime` acted as an accidental second facade of the engine.

## Decision

Keep the two repositories as separate layers:

- **Thalos Engine** — the reusable platform: domain crates (`core`, `models`,
  `math`, `planning`, `lang`, `language-service`, `importer`, `ports`,
  `analysis`, `document`, `visual`, `transport`) plus the public boundary
  (`thalos-api`).
- **Thalos Industrial** — the application: `thalos-runtime` (application
  runtime), `thalos-persistence`, `thalos-dtos`, `thalos-tauri`.

`thalos-runtime` moved from `thalos-engine/thalos-runtime` to
`thalos-industrial/backend/crates/thalos-runtime`.

## Consequences

- The engine repository is self-contained: `cargo test --workspace` needs no
  sibling repositories.
- Industrial depends on the engine only through `thalos-api` (see ADR-018).
- Application-specific concepts (`Station` as an operational entity, sessions,
  persistence) live in Industrial, not in the engine.

## Related

- ADR-017 (public boundary), ADR-018 (API closure).
