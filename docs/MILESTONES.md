# Milestones — Thalos Engine

> **Qué es esto:** el registro vivo de lo que se decidió y se cerró. Cada fila
> reemplaza al antiguo ADR individual. La columna **Estado actual** dice si la
> decisión sigue siendo cierta **hoy**.
>
> **Estados:** ✅ Vigente · 🔄 Superado · ❌ Nunca implementado · 🟠 Parcial

## Decisiones

| Decisión | Estado actual | Evidencia en código |
|---|---|---|
| **Convención Z-up** — el eje Z es la vertical canónica en todos los modelos | ✅ vigente | tests de FK en `thalos-core/src/models/*/tests/fk.rs` |
| **Modelo de capacidad y resolución de skill** — `RobotDefinition` declara capacidades; un `SkillRegistry` resuelve la implementación por `(RobotId, SkillId)`; el AST de programa es puro e independiente del robot | ✅ vigente | `thalos-core/src/capability.rs`, `thalos-core/src/skill`, `thalos-core/src/program` |
| **Modelo operacional de station** — Station, recursos y su identidad como entidades de dominio | ✅ vigente | `thalos-core/src/station.rs`, `thalos-core/src/resource.rs`, `thalos-core/src/ids.rs` |
| **Semántica de comando físico** — mando, setpoint y trigger se distinguen; `DirectActuation` queda fuera del vocabulario por defecto; el lazo no lo cierra siempre el engine | ✅ vigente | `thalos-core/src/command` |
| **Modelo de supervisión: semántica, no mecanismo** — el engine posee la máquina de estados de sesión, las reglas de transición y el contrato de tick; el loop, el reloj y el event bus viven en la aplicación | ✅ vigente | `thalos-core/src/execution/session.rs`; tests de pureza en `thalos-api/tests` |
| **Frontera Engine / Industrial** — dos repositorios en capas separadas; el engine se construye y testea standalone | ✅ vigente | `docs/ARCHITECTURE.md` |
| **`thalos-api` como frontera pública curada** — fachada por concepto, nunca alias de crates enteros; la superficie pública es exactamente la declarada | ✅ vigente | `thalos-api/tests/{public_surface,dependency_purity}.rs` |
| **Cierre de API** — la aplicación construye solo sobre `thalos-api`; cada hueco de la API lo detectó el compilador, no una discusión | ✅ vigente | `thalos-api/tests/dependency_purity.rs` |
| **Modelo temporal del movimiento** — Requested / Planned / Actual: el engine posee `Requested → Planned`, el ejecutor reporta `Actual` | ✅ vigente | `thalos-planning`, `thalos-core/src/execution` |
| **Frontera de I/O físico** — el engine posee solo los puertos abstractos (`thalos-ports`); el I/O concreto (serial/TCP/ESP32) se movió a Thalos Industrial (F0, 2026-09-21) para no contaminar el workspace del engine | ✅ vigente | `thalos-ports/src/robot/transport.rs`; `thalos-industrial/backend/crates/thalos-transport` |
| **Namespace numérico de ADRs** | 🔄 retirado | Los ADRs se eliminaron el 2026-09-20; las referencias en código se sustituyeron por nombres de contrato |

## Fases cerradas

| Fase | Resultado | Estado |
|---|---|---|
| **1 — Extracción del engine** | El engine se separó de la aplicación; `thalos-runtime` dejó de estar en este repositorio | ✅ |
| **2 — Cierre de frontera** | `thalos-runtime` consume el engine solo por `thalos-api`; superficie pública verificada por tests | ✅ |
