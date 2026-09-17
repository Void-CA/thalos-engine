# Thalos Engine

Reusable robotics platform — domain kernel and runtime extracted from Thalos Industrial.

The former `thalos-engine` facade crate was removed. Domain crates are consumed
directly; applications that need the old facade path use `thalos_runtime::engine`
(`core`, `math`, `models`, `importer`, `lang`, `planning`) with the language
front-end exposed as `thalos_runtime::engine::semantic`
(`thalos-language-service`).
