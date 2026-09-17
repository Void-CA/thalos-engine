# ADR-018: API Closure — applications build on `thalos-api`

## Status

Accepted

## Context

A public boundary is only real if we can build a serious consumer on top of it.
`thalos-runtime` is the reference consumer of Thalos as a platform, so it is the
strongest available test of the boundary.

If `thalos-runtime` keeps depending on the internal engine crates directly, we
never discover the gaps in `thalos-api`.

## Decision

`thalos-runtime` consumes the engine through **`thalos-api` as its only
functional engine dependency**. The API gap list is produced by the compiler:
every unresolved path is either promoted in `thalos-api`, abstracted, or
declared out of contract.

Rules:

- `thalos-runtime` must not re-export engine concepts. Consumers import engine
  concepts from `thalos-api` and application concepts from `thalos-runtime`.
- **Temporary exception:** `thalos-transport` (concrete serial/TCP/ESP32 IO)
  remains a direct dependency while its public-boundary status is undecided.
  This is documented, not permanent.
- Test doubles are exposed through the `thalos-api/test-support` feature.

## Guards (verifiable invariants)

- `thalos-runtime/tests/engine_dependency_boundary.rs` — parses `Cargo.toml` and
  allows only `{thalos-api, thalos_transport}` as engine dependencies.
- `thalos-runtime/tests/no_engine_reexports.rs` — fails on any
  `pub use thalos_*` in `thalos-runtime/src`.

## Consequences

- Adding an engine dependency to the application now fails CI and forces the
  conversation: is this concept part of the public contract?
- API gaps surfaced by this exercise were resolved in `thalos-api`: `resource`,
  `capability`, `execution`, `station`, the `catalog` namespace, `analysis`
  services (`thalos-analysis`), `visual`, and the `test-support` feature.

## Related

- ADR-016 (boundary), ADR-017 (public boundary).
