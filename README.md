# Thalos Engine

Núcleo reutilizable de robótica: dominio, matemática, cinemática, planificación,
lenguaje (`.thls`) y semántica de supervisión. Es una **librería pura y
autónoma**: no conoce aplicaciones, transporte, persistencia ni UI.

Lo consume **Thalos Industrial** (capa de aplicación), que vive en un repositorio
hermano. El engine nunca depende de ella.

## Cómo se usa

```bash
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo doc --workspace --no-deps
```

Los consumidores externos importan **solo** desde `thalos-api`. Los crates
internos no forman parte del contrato público.

### Cinemática, movimiento y ejecución por intención

El uso normal no necesita conocer el solver ni el pipeline: se pide una intención
y el engine decide cómo conseguirla.

```rust
use thalos_api::intent::{CartesianTarget, Manipulator};

let robot = Manipulator::from_urdf(urdf_source)?;   // engine is pure: URDF text, not a path

let tcp = robot.forward(&[0.3, 0.4])?;
let solution = robot.inverse(CartesianTarget::position(0.4, 0.2, 0.3))?;
```

Lo mismo para mover o preparar ejecución, sin ensamblar el mecanismo:

```rust
use thalos_api::intent::{JointMove, MotionRequest, prepare_execution};

let plan = robot.plan_motion(
    &[0.0, 0.0],
    MotionRequest::Joint(JointMove::to(vec![0.3, 0.4])),
)?;
let prepared = prepare_execution(&plan);
```

Para control fino existe **Tier 2** (`IkRequest`, `MotionProfile`); el mecanismo
(`SerialChain`, `DampedLeastSquaresSolver`, `MoveJPlanner`, …) sigue público como
**Tier 3** para quien construye encima del engine. Ver `docs/API-INTENT-SPEC.md`.

## Frontera pública

```rust
// PERMITIDO en un consumidor
use thalos_api::core::execution::plan::ExecutionPlan;
use thalos_api::planning::PlanAdvisor;
use thalos_api::lang::parse_source;

// PROHIBIDO
use thalos_core::...;
use thalos_math::...;
use thalos_planning::...;
```

`thalos-api` es una fachada **curada por concepto**, no un alias del grafo interno:
re-exporta módulo a módulo. Su pureza la verifican
`thalos-api/tests/dependency_purity.rs` y `public_surface.rs`.

## Crates

| Crate | Responsabilidad |
|---|---|
| `thalos-api` | Única frontera pública. Fachada curada por concepto |
| `thalos-math` | Álgebra, cuaterniones, dual quaternions, transformaciones |
| `thalos-models` | Tipos canónicos de modelo robótico |
| `thalos-core` | Primitivas de dominio: cinemática (FK/IK), colisión, operación, skill, program, station, recursos, ejecución (IR de sesión) |
| `thalos-planning` | Trayectorias, interpolación, resolución, `ExecutionPlan` |
| `thalos-lang` | AST, parser y sistema de unidades del DSL `.thls` |
| `thalos-language-service` | Front-end del lenguaje: compiler → resolver → ir → validation → lowering, y `knowledge` |
| `thalos-importer` | Importación de modelos (URDF, assets, diagnóstico, normalización) |
| `thalos-analysis` | Servicios de análisis (workspace, manipulabilidad, singularidad, comparación) |
| `thalos-telemetry` | Semántica de evidencia: trazas, muestras, vocabulario de eventos. No adquiere ni persiste |
| `thalos-ports` | Puertos de dominio (device, robot). El I/O concreto vive en Thalos Industrial |
| `thalos-document` | Modelo de documento de escena y programa |
| `thalos-visual` | Construcción de escena y visualización (mallas, trayectoria, spatial) |

## Documentación

| Documento | Para qué |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Qué posee el engine y qué está prohibido |
| [`docs/STATUS.md`](docs/STATUS.md) | Qué funciona hoy y qué no |
| [`docs/MILESTONES.md`](docs/MILESTONES.md) | Decisiones cerradas y su vigencia |
| [`../thalos-industrial/docs/`](../thalos-industrial/docs/README.md) | Documentación de la capa de aplicación |

No hay ADRs: una decisión se registra como una fila en `MILESTONES.md`.
