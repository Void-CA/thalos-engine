# API — Thalos Engine

> **Carácter:** normativo. Describe el contrato público **implementado** por el
> engine. No es una propuesta ni un registro de fases: lo que aquí aparece existe
> en el código y lo verifica la compilación.
>
> **Cómo se mantiene:** se actualiza al cambiar la superficie de `thalos-api`.
> Los ejemplos de esta página son los mismos programas ejecutables de
> [`../thalos-api/examples/`](../thalos-api/examples/); compilan con
> `cargo check -p thalos-api --examples`.

## 1. La frontera pública

Los consumidores importan **solo** desde `thalos-api`. Es una fachada **curada
por concepto**, no un alias del grafo interno: re-exporta módulo a módulo. Los
crates internos (`thalos-core`, `thalos-math`, `thalos-planning`, …) no forman
parte del contrato.

La pureza de la frontera la verifican dos tests:

- `thalos-api/tests/public_surface.rs` — todo camino público se alcanza sin
  nombrar un crate interno.
- `thalos-api/tests/dependency_purity.rs` — `thalos-api` no depende de la capa de
  aplicación.

## 2. Modelo de tiers

| Tier | Qué es | Puede aparecer en el consumidor |
|---|---|---|
| **1 — Intención** | El verbo del dominio; estable, no expone algoritmo | `Manipulator`, `MotionRequest`, `MotionPlan`, `CartesianTarget`, `IkSolution`, `PreparedPlan` |
| **2 — Configuración** | Perilla opcional sobre una intención | `MotionProfile`, `IkRequest` |
| **3 — Mecanismo** | Cómo lo hace el engine; público para quien construye encima | `SerialChain`, `DampedLeastSquaresSolver`, `MoveJPlanner`, `Trajectory`, `ExecutionPlan`, … |

**Tier 3 sigue siendo público.** No está oculto; simplemente deja de ser la
puerta de entrada. El consumidor normal expresa intenciones (Tier 1) y, cuando lo
necesita, baja a Tier 3 de forma deliberada.

## 3. La entrada: `Manipulator`

`Manipulator` es el receptor de Tier 1: una **capacidad sin estado de pipeline**.
Guarda el modelo cinemático y nada más — ni plan activo, ni sesión, ni joints
actuales, ni solver cacheado. El estado entra como parámetro en cada petición,
porque el estado vivo del robot pertenece a la aplicación (ver
[`ARCHITECTURE.md`](ARCHITECTURE.md)).

```rust
use thalos_api::intent::{CartesianTarget, Manipulator};

let robot = Manipulator::from_urdf(urdf_source)?;   // el engine no hace I/O: texto, no ruta

let tcp = robot.forward(&[0.0, 0.0])?;              // FK → TcpPose
let solution = robot.inverse(CartesianTarget::position(0.4, 0.2, 0.3))?;
```

| Método | Firma | Notas |
|---|---|---|
| `from_urdf` | `fn from_urdf(source: &str) -> Result<Self, RobotLoadError>` | Punto de entrada; el end-effector se elige automáticamente |
| `from_model` | `fn from_model(model: &models::Robot) -> Result<Self, RobotLoadError>` | Desde un modelo estructural (`thalos_api::models::Robot`) |
| `from_chain` | `fn from_chain(chain: SerialChain) -> Self` | Escape a Tier 3 |
| `forward` | `fn forward(&self, joints: &[f64]) -> Result<TcpPose, MotionError>` | FM; solo falla por `DofMismatch` |
| `inverse` | `fn inverse(&self, target: impl Into<CartesianTarget>) -> Result<IkSolution, MotionError>` | Seed por defecto (ceros) |
| `inverse_with` | `fn inverse_with(&self, request: IkRequest) -> Result<IkSolution, MotionError>` | Tier 2: seed y tolerancias |
| `plan_motion` | `fn plan_motion(&self, from: &[f64], request: MotionRequest) -> Result<MotionPlan, MotionError>` | `from` es explícito, no estado |
| `dof` | `fn dof(&self) -> usize` | Grados de libertad actuados |
| `end_effector_name` | `fn end_effector_name(&self) -> Option<&str>` | Responde "¿cuál es el EE?" sin exponer `FrameId` |
| `chain` | `fn chain(&self) -> &SerialChain` | Único escape a Tier 3 |

## 4. Tipos de intención (Tier 1 y 2)

### Objetivos cartesianos

```rust
use thalos_api::intent::CartesianTarget;
use thalos_api::math::UnitQuaternion;

let position = CartesianTarget::position(0.4, 0.2, 0.3);              // world
let pose = CartesianTarget::pose(0.4, 0.2, 0.3, UnitQuaternion::identity());
```

`CartesianTarget` es Tier 1 **propio** (no un re-export de `IKGoal`), así el
mecanismo de debajo puede evolucionar sin romper la frontera. `From<Vector3>` y
`From<Pose>` permiten pasarlos directamente a `inverse`/`LinearMove::to`.

### Movimiento

```rust
pub enum MotionRequest {          // #[non_exhaustive]
    Joint(JointMove),
    Linear(LinearMove),
}
```

- `JointMove::to(target: Vec<f64>)` — espacio articular; **no** menciona IK.
- `LinearMove::to(target: impl Into<CartesianTarget>)` — objetivo cartesiano; el
  engine resuelve internamente `cartesiano → IK → planificación articular`.
- Ambos aceptan `.with_profile(MotionProfile)`.

`MotionProfile` (Tier 2), `Default = { max_velocity: 1.0, max_acceleration: 0.5, time_step: 0.01 }`.

### Resultados

- `IkSolution` — `joints()`, `into_joints()`, `converged()`, `iterations()`,
  `final_error()`. Una solución no convergente **se devuelve** (`converged() ==
  false`); solo un error del solver es `Err`.
- `MotionPlan` — valor Tier 1: `kind()`, `waypoint_count()`, `duration()`, y
  `trajectory()` (escape a Tier 3). Se produce una vez y cruza capas sin
  re-planificar.
- `PreparedPlan` — IR de ejecución listo: `waypoint_count()`, `duration()`,
  `execution_plan()` (escape a Tier 3).

```rust
use thalos_api::intent::{JointMove, Manipulator, MotionRequest, prepare_execution};

let plan = robot.plan_motion(&[0.0, 0.0], MotionRequest::Joint(JointMove::to(vec![0.3, 0.4])))?;
let prepared = prepare_execution(&plan);   // usa la MISMA trayectoria; no replanifica
```

`IkRequest` (Tier 2) construye una petición de IK con control fino:

```rust
use thalos_api::intent::IkRequest;

let request = IkRequest::new(CartesianTarget::position(1.0, 0.5, 0.0))
    .with_seed(vec![0.2, 0.1])
    .with_tolerance(1e-8);
```

## 5. Ejemplos ejecutables

Cada ejemplo de [`../thalos-api/examples/`](../thalos-api/examples/) es un flujo
real escrito como consumidor externo (solo importa `thalos-api`). Se ejecutan con:

```bash
cargo run -p thalos-api --example <escenario>
```

| Escenario | Intención | Tier 1 que usa |
|---|---|---|
| `load_robot` | Cargar un robot y hacer IK | `Manipulator`, `CartesianTarget` |
| `forward_kinematics` | FK desde joints | `Manipulator`, `TcpPose` |
| `inverse_kinematics` | Resolver un objetivo | `Manipulator`, `CartesianTarget` |
| `move_j` | Mover en espacio articular | `Manipulator`, `MotionRequest`, `JointMove` |
| `move_l` | Mover a un punto cartesiano | `Manipulator`, `MotionRequest`, `LinearMove`, `CartesianTarget` |
| `execute` | Preparar un plan | `prepare_execution` |
| `frames` | Poses y frames explícitos | `Pose`, `FrameId`, `FrameRegistry` (Tier 3) |
| `plan` | Planificar un programa `.thls` | *(sin API de intención — ver §8)* |

## 6. Semántica de errores

> Una operación de **análisis** puede devolver un resultado que describe un estado
> fallido; una operación que **produce un artefacto ejecutable** debe fallar si no
> puede producirlo.

| Operación | Falla de IK | DOF mismatch | Carga inválida |
|---|---|---|---|
| `inverse` / `inverse_with` | `Ok(IkSolution { converged: false })` | — | — |
| `plan_motion(Linear)` | `Err(Unreachable)` | — | — |
| `plan_motion(Joint)` | — | `Err(DofMismatch)` | — |
| `forward` | — | `Err(DofMismatch)` | — |
| `from_urdf` / `from_model` | — | — | `Err(RobotLoadError)` |

`MotionError` es Tier 1: envuelve `IkError`/`PlanningError` sin exponerlos.

## 7. Escapes a Tier 3

El consumidor baja a Tier 3 solo de forma explícita:

- `Manipulator::chain()` → `&SerialChain` (selección de tip, frames no-default).
- `MotionPlan::trajectory()` → `&Trajectory`.
- `PreparedPlan::execution_plan()` → `&ExecutionPlan`.
- `Manipulator::from_chain(chain)` → construir sobre una cadena ya existente.

## 8. Limitaciones vigentes

1. **Planificación de programas `.thls` sin API de intención.** `Manipulator`
   cubre carga, FK, IK, MoveJ, MoveL y preparación de ejecución de un
   **movimiento**. Compilar un **programa** (`.thls`) a plan sigue siendo Tier 3:
   el consumidor orquesta el pipeline por etapas (ver `examples/plan.rs`). Un
   candidato (`Manipulator::plan_source(source) -> Plan`) está **diferido** hasta
   que un programa real demuestre la necesidad.
2. **Frames.** El caso común (base por defecto, end-effector por defecto) no
   necesita `FrameId`. La selección explícita de tip o la resolución contra
   frames no-default es Tier 3 vía `chain()`.
3. **MoveL cartesiano.** `LinearMove` resuelve el objetivo mediante IK y
   planificación articular; un planificador de trayectoria cartesiana continua
   (pose a lo largo del camino) no está implementado.
4. **El engine no ejecuta.** `prepare_execution` prepara el plan; el loop, el
   reloj, el scheduler y el hardware viven en la aplicación (Thalos Industrial).

## 9. Cómo se verifica

```bash
cargo test  -p thalos-api               # superficie pública y pureza de frontera
cargo check -p thalos-api --examples    # los ejemplos de §5 compilan
cargo doc   --workspace --no-deps       # rustdoc del contrato por crate
```
