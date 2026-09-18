# Thalos Engine

Reusable robotics platform — domain kernel and runtime extracted from Thalos Industrial.

The former `thalos-engine` facade crate was removed. The single public boundary
is `thalos-api` (see ADR-017/018): applications depend on `thalos-api` only and
never on the internal engine crates.

Per ADR-020, the engine owns domain + supervision semantics — not its execution
mechanism. Namespaces for supervision (`execution::supervision`), evidence
(`telemetry`), and the robot controller contract (`robot::controller`) are added
incrementally, each in the same change that brings their concepts across the
boundary.
