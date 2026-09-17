# Architecture Decision Records

This directory holds the engine's Architecture Decision Records.

## Index

| ADR | Title | Status |
|-----|-------|--------|
| [002](002-robot-capability-model.md) | Robot Capability Model & Skill Resolution | Accepted |
| [015](015-physical-io-concurrency-boundary.md) | Physical I/O Concurrency Boundary | Proposed |
| [016](016-engine-industrial-boundary.md) | Engine / Industrial Boundary | Accepted |
| [017](017-thalos-api-public-boundary.md) | `thalos-api` as the curated public boundary | Accepted |
| [018](018-api-closure-runtime-over-api.md) | API Closure — applications build on `thalos-api` | Accepted |

## Known gaps

The numbering is sparse: `001` and `003`–`014` are missing. They are referenced
by source comments (e.g. `ADR-014` for the resource/station inventory model,
`ADR-019` for physical command semantics) but were not recovered into this
directory. Reconstructing or retiring them is pending.
