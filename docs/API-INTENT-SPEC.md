# Especificación de la API de intención (Fase C)

> **Carácter:** especificación. Define **cómo debería sentirse** la API pública del
> engine, no la implementa. Se deriva de los escenarios de Fase B
> (`thalos-api/examples/`) y de la auditoría de Fase A (`docs/API-ERGONOMICS.md`).
>
> **No-objetivo:** implementar. Fase D es la migración; aquí solo se fija la
> forma y el contrato.

## Regla única de C

Cada abstracción propuesta debe responder:

> **¿Qué conocimiento elimina del consumidor?**

Si una abstracción solo renombra una llamada pero el consumidor sigue conociendo
los mismos conceptos internos, **se descarta**.

## Modelo de tiers (frontera fijada en B)

| Tier | Qué es | Ejemplos |
|---|---|---|
| **1 — Intención** | El verbo del dominio. Estable, no expone algoritmo | `Manipulator`, `MotionRequest`, `Plan` |
| **2 — Configuración** | Perilla opcional sobre una intención | `MotionProfile`, `IkRequest`, `ExecutionOptions`, `Frame` |
| **3 — Mecanismo** | Cómo lo hace el engine. Público para quien construye encima | `SerialChain`, `IKSolver`, `DampedLeastSquaresSolver`, `MoveJPlanner`, `ForwardKinematics`, `PlanCompiler` |

**Tier 3 sigue siendo público.** No se oculta; se deja de ser la puerta de
entrada. B7 (`execute`) demuestra que Tier 3-heavy es correcto cuando es el
contrato del dominio.

## Mapa de familias

```text
Robot / Kinematics        Motion              Planning / Execution
──────────────────        ──────              ────────────────────
load                      joint move           plan
forward                   linear move          execute
inverse
jacobian
frames
```

El flujo vertical es `Robot → Kinematics → Motion → Planning → Execution`, y los
valores Tier 1 viajan entre familias: `Trajectory`/`MotionPlan` (Motion → Planning)
y `Plan` (Planning → Execution).

---

## Decisión de forma: un receiver, no un god-object, no sub-objetos

**Decisión:** una única entidad de intención `Manipulator` es el receiver, con
métodos agrupados por familia. **No** se adopta `robot.kinematics().forward()`
(sube un nivel de indirección sin eliminar ningún conocimiento) ni una constelación
de objetos independientes (el consumidor tendría que saber *cuál* objeto buscar).

`Manipulator` no es un god-object porque **no guarda estado de pipeline**: cada
método recibe una petición y devuelve un valor; los mecanismos (`PlanCompiler`,
`MoveJPlanner`, …) se construyen on-demand y nunca se expone su ciclo de vida.

> **Ubicación (decidida en D1).** La capa Tier 1 se implementa **dentro de
> `thalos-api`** (`pub mod intent`), no en un crate aparte. Razón: `thalos-api`
> *es* la superficie del usuario, así que la separación Tier 1/2/3 se expresa de
> forma natural ahí. La frontera sigue siendo que Tier 3 no sea la puerta de
> entrada; no que la lógica viva en otro crate.

**Alternativas descartadas:**
- `robot.motion().linear(pose)` → añade un hop; no elimina conocimiento (regla de C).
- Objetos sueltos (`MotionPlanner`, `ProgramPlanner` sin receiver) → obligan al
  consumidor a resolver la navegación.

---

## 1. Robot / Kinematics

### 1.1 Cargar un robot — `load`

**Intención (B1):** `Robot::load(urdf)`.

```rust
// Especificación (Tier 1)
let robot = Manipulator::from_urdf(urdf_source)?;      // URDF
let robot = Manipulator::from_chain(chain);            // cadena ya construida
```

| Aspecto | Definición |
|---|---|
| API propuesta | `Manipulator::from_urdf(&str) -> Result<Manipulator, RobotLoadError>`, `Manipulator::from_chain(SerialChain)` |
| Tipos públicos Tier 1 | `Manipulator`, `RobotLoadError` |
| Mecanismo oculto | `import_urdf` + `adapter::auto/from_robot` + elección de end-effector |
| Tier 2/3 deliberado | `from_chain` acepta un `SerialChain` (Tier 3) para avanzados; `adapter` sigue público |
| Conocimiento eliminado | `import_urdf`, `adapter`, `SerialChain` como pasos obligatorios |

### 1.2 Cinemática directa — `forward`

**Intención (B2):** `robot.forward_kinematics(&joints)`.

```rust
let tcp: TcpPose = robot.forward(&joints)?;            // ya existe: KinematicContext
```

| Aspecto | Definición |
|---|---|
| API propuesta | `Manipulator::forward(&self, joints: &[f64]) -> Result<TcpPose, KinematicError>` |
| Mecanismo oculto | `ForwardKinematics`, `KinematicContext`, resolución de `ToolFrame` |
| Precedente | `KinematicContext` (`context.rs:58`) **ya es la forma correcta**; se conserva y se expone como método del robot. No se reemplaza |
| Conocimiento eliminado | construcción del contexto FK; el `Option`/DOF-mismatch se convierte en error tipado |

### 1.3 Cinemática inversa — `inverse`

**Intención (B3):** `robot.inverse_kinematics(target)`.

```rust
let solution: IkSolution = robot.inverse(target)?;
// Con control (Tier 2):
let solution = robot.inverse_with(IkRequest::new(target).with_seed(q0).with_tolerance(1e-6))?;
```

| Aspecto | Definición |
|---|---|
| API propuesta | `Manipulator::inverse<T: Into<CartesianTarget>>(&self, target: T) -> Result<IkSolution, KinematicError>` + `inverse_with(IkRequest)` |
| Tipos Tier 1 | `CartesianTarget` (enum `Position(Vector3)`/`Pose(Pose)`), `IkSolution { joints, converged, iterations, error }` |
| Tipos Tier 2 | `IkRequest` (seed, tolerance, iterations, lambda) |
| Mecanismo oculto | `DampedLeastSquaresSolver`, `IKSolver`, `IKConfig`, `IKGoal`, elección de frame, seed por defecto |
| Tier 3 deliberado | `DampedLeastSquaresSolver`, `MultiStartIKSolver`, `IKSolver` para construir encima |
| Conocimiento eliminado | solver + config + enum de goal + trait + frame |

**Nota de estabilidad:** `CartesianTarget`/`IkSolution` son Tier 1 **propios**, no
reexports de `IKGoal`/`IKResult`. Así Tier 3 puede evolucionar sin romper Tier 1
(la distinción "abstracción vs reexport" de Fase A).

### 1.4 Frames y poses

**Intención (B8):** expresar una pose sin conocer el sistema de frames.

```rust
let target = CartesianTarget::position(0.4, 0.2, 0.3);       // world, identidad
let target = CartesianTarget::pose(x, y, z, quat);           // con orientación
```

| Aspecto | Definición |
|---|---|
| API propuesta | constructores de `CartesianTarget` world-referenciados; `FrameId`/`FrameRegistry` permanecen Tier 2/3 |
| Mecanismo oculto | `FrameId`, `FrameRegistry`, `Pose::new(reference, target, transform)` |
| Tier 2/3 deliberado | `Pose`, `FrameId`, `FrameRegistry`, `Transform3D` para quien manipula frames explícitamente |
| Conocimiento eliminado | `FrameId::World` y el registry para el caso normal |

---

## 2. Motion

### 2.1 Pregunta C-1 — ¿cómo representar un movimiento sin arrastrar dependencias que no necesita?

**Decisión:** la petición es un **enum cuyos variantes llevan solo los datos que
esa intención requiere**. MoveJ no menciona IK ni en su tipo ni en su firma.

```rust
pub enum MotionRequest {
    Joint(JointMove),    // solo espacio articular
    Linear(LinearMove),  // espacio cartesiano
}

pub struct JointMove {
    pub target: Vec<f64>,
    pub profile: MotionProfile,     // Tier 2, Default
}

pub struct LinearMove {
    pub target: CartesianTarget,
    pub profile: MotionProfile,
    pub constraints: Option<MotionConstraints>, // Tier 2
}

pub struct MotionProfile {           // Tier 2
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub time_step: f64,
}
```

```rust
let plan: MotionPlan = robot.plan_motion(&from, MotionRequest::Joint(JointMove::to(q)))?;
let plan: MotionPlan = robot.plan_motion(&from, MotionRequest::Linear(LinearMove::to(pose)))?;
```

> **Refinamientos de D1 (implementación).** (a) El método se llama
> `plan_motion`, no `move`: `move` es palabra reservada de Rust. (b) El estado
> inicial (`from`) es parámetro explícito, no estado de `Manipulator` — protege
> la decisión de C de un receiver sin estado de pipeline.

Internamente, el `PlanningContext` actual exige `ik_solver: &dyn IKSolver`
obligatorio (causa de B4). La especificación **separa el contexto interno**:

```text
JointPlanningContext   { robot, current_state }
CartesianPlanningContext { robot, current_state, ik_solver, tcp }
```

así el camino articular **no construye solver**. Es un cambio Tier 3 interno;
ambas variantes quedan ocultas. (Alternativa aceptable en C: construir un solver
no usado — es un objeto barato y no resuelve nada — pero la separación es más
honesta.)

**Conocimiento eliminado:** `PlanningContext`, `MoveJConfig`, `MoveJPlanner`,
`SegmentPlanner`, `ValidatedGoal`, `GoalMetadata`, `PlanningAssessment`,
`RobotState`, y el solver de IK en un MoveJ.

### 2.2 Pregunta C-2 — ¿cómo representar MoveL sin exponer su pipeline `IK → MoveJ`?

**Decisión:** `LinearMove` es solo un objetivo cartesiano. La secuencia
"muestrear cartesiano → IK → waypoints articulares → planning articular" es
**responsabilidad del engine**, no del consumidor.

| Aspecto | Definición |
|---|---|
| API | `robot.plan_motion(&from, MotionRequest::Linear(LinearMove::to(target)))` |
| Resultado | `MotionPlan` (Tier 1) envolviendo `Trajectory` (Tier 3) |
| Mecanismo oculto | `ForwardKinematics` + `DampedLeastSquaresSolver` + `IKGoal` + `MoveJPlanner` + `MoveJConfig` |
| Tier 3 deliberado | `MoveJPlanner`, `MoveLPlanner`, `interpolate::*` para quien diseña planners |
| Conocimiento eliminado | que MoveL termina usando el planner articular; el consumidor solo da una pose |

**Salvedad de alcance:** mover a pose completa exige IK de pose (hoy el ejemplo usa
IK de posición). La spec acepta `LinearMove` con `CartesianTarget::Pose`, dejando
la política de orientación (exigir/relajar) a Tier 2 (`MotionConstraints`).

---

## 3. Planning / Execution

### 3.1 Planificar — `plan`

**Intención (B6):** `robot.plan(source)`.

```rust
let plan: Plan = robot.plan_source(source)?;     // compila .thls → Plan
let plan: Plan = robot.plan(&resolved_program)?; // o desde un programa resuelto
```

| Aspecto | Definición |
|---|---|
| API | `Manipulator::plan_source(&self, source: &str) -> Result<Plan, PlanError>`; `plan(&self, program: &ResolvedProgram)` |
| Tipo Tier 1 | `Plan` (valor opaco que conserva el artefacto compilado) |
| Mecanismo oculto | `parse_source`, `SemanticCompiler`, `SemanticResolver`, `PlanningInput`, `PlanCompiler`, `DefaultPlannerDispatcher`, `SegmentPlanningContext`, `DampedLeastSquaresSolver` |
| Tier 2 | `PlanOptions` (perfil por defecto, `GoalResolverConfig`) |
| Tier 3 deliberado | el pipeline por etapas sigue público para diagnósticos finos |
| Conocimiento eliminado | orquestar 6 etapas y el par dispatcher/compiler |

### 3.2 Pregunta C-3 — ¿cómo conservar `plan` para que `execute` no lo reconstruya?

**Decisión:** `Plan` es un **valor Tier 1 de primera clase** que cruza la frontera
Planning → Execution. El artefacto compilado **no se vuelve a derivar**.

```rust
let plan: Plan = robot.plan_source(source)?;
let session: ExecutionSession = robot.prepare_execution(&plan, ExecutionOptions::default())?;
// El loop/reloj/event bus siguen en la aplicación (ARCHITECTURE.md).
```

| Aspecto | Definición |
|---|---|
| API | `Manipulator::prepare_execution(&self, plan: &Plan, options: ExecutionOptions) -> Result<ExecutionSession, ExecutionError>` |
| Tipo Tier 1 | `Plan` (contiene o produce el `ExecutionPlan`) |
| Mecanismo oculto | `ExecutionPlanBuilder` + el `CompiledPlan` intermedio; el consumidor ya no compila dos veces |
| Tier 2 | `ExecutionOptions` (mapea a `ExecutionConfiguration`) |
| Tier 3 deliberado | `ExecutionSession`, `ExecutionRunner`, `TickContext` — el contrato de supervisión del engine; el **mecanismo de ejecución** vive en Industrial |
| Conocimiento eliminado | reconstruir el plan; el orden `initialize → start`; la creación de identidad |

**Frontera explícita:** el engine **no ejecuta** (no tiene loop ni reloj). Tier 1
"execute" significa *preparar un plan para ejecución*, no correrla. Esto es
coherente con B7 y con `ARCHITECTURE.md`.

> **Refinamiento de D3.** La implementación separó dos pasos que C-3 tenía
> juntos: (1) **movimiento** — `MotionPlan → prepare_execution → PreparedPlan`
> (IR de ejecución), implementado; (2) **programa** — `plan_source → Plan →
> sesión de supervisión`, pendiente como D3-bis. El artefacto que cruza la
> frontera en el caso de movimiento es `PreparedPlan` (envuelve `ExecutionPlan`),
> no una `ExecutionSession`.

---

## Contrato entre capas

```text
Manipulator::from_urdf ──► Manipulator
Manipulator::forward/inverse/jacobian ──► TcpPose / IkSolution / Jacobian
Manipulator::move(Joint|Linear) ──► MotionPlan ──┐
Manipulator::plan_source ──► Plan ◄──────────────┘ (acepta MotionPlan/trayectoria)
Manipulator::prepare_execution(&Plan) ──► ExecutionSession
```

Regla de conexión: **los valores Tier 1 son los que cruzan capas** (`MotionPlan`,
`Plan`), nunca los Tier 3 (`Trajectory`, `CompiledPlan`, `ExecutionPlan`).

## Qué cambia internamente (Tier 3) y qué es aditivo

| Cambio | Tipo | Riesgo |
|---|---|---|
| Nuevo módulo Tier 1 (`Manipulator` + tipos de intención) | Aditivo | Bajo |
| Separar `PlanningContext` en `Joint`/`Cartesian` | Interno Tier 3 | Medio (toca planner) |
| `Plan` como valor que une `CompiledPlan` y `ExecutionPlan` | Aditivo | Bajo |
| `inverse` de pose completa en el camino LinearMove | Interno | Medio (calidad de IK) |

Nada se elimina: Tier 3 permanece público. `public_surface.rs` se amplía (no se
recorta).

## Relación con el prototipo congelado

`KinematicRobot` / `IkRequest` (`thalos-core/src/kinematics/robot_api.rs`) fueron
un prototipo de Fase 0. Esta especificación **los superó**: el nombre pasó a
`Manipulator`, el objetivo a `CartesianTarget`, el resultado a `IkSolution`, y la
petición a `MotionRequest`/`IkRequest`. **Fueron eliminados en D4** (código y
test), una vez que `intent` ofreció el reemplazo definitivo; nunca se difundieron
a Industrial.

## Riesgos y preguntas abiertas para D

1. `Manipulator` vs `models::Robot` vs `RobotModel` (catalog): confirmar que el
   nombre no colisiona conceptualmente. Alternativas: `KinematicModel`.
2. `motion` cartesiano real (trayectoria de pose, no IK puntual + MoveJ): puede
   requerir un planificador cartesiano que hoy no existe.
3. ¿`plan` desde fuente y desde programa resuelto, o solo fuente en Tier 1?
4. ¿`prepare_execution` en el engine o como `Plan::bind(options)`?
5. ¿`MotionConstraints`/`ExecutionOptions` como structs o builders?

## Criterio de cierre de C

La especificación responde, para cada uno de los 8 escenarios de B:

- [x] la intención del consumidor;
- [x] la API propuesta (Tier 1);
- [x] los tipos públicos necesarios (Tier 1 + Tier 2);
- [x] qué mecanismo queda oculto;
- [x] qué queda deliberadamente Tier 2/3;
- [x] cómo se conecta con las otras capas.

Y resuelve las tres preguntas centrales:

- [x] **C-1** movimiento sin dependencias que no necesita → request por variante +
      contextos internos separados;
- [x] **C-2** MoveL sin exponer `IK → MoveJ` → `LinearMove` cartesiano, pipeline
      interno;
- [x] **C-3** conservar `plan` para `execute` → `Plan` Tier 1 que cruza la frontera.

## Estado de implementación (Fase D, por slices)

| Slice | Alcance | Estado |
|---|---|---|
| **D1** | `MotionRequest::Joint` + separación de `PlanningContext` | ✅ implementado |
| **D2** | `LinearMove` (pipeline cartesiano → IK → planning interno) | ✅ implementado |
| **D3** | `MotionPlan` como valor que cruza planificación → preparación de ejecución | ✅ implementado |
| D3-bis | `Plan` desde `.thls` (`plan_source`) | pendiente |
| **D4** | IK en la forma definitiva (`Manipulator::inverse` + `CartesianTarget`/`IkSolution`) | ✅ implementado |
| **D5** | `load_robot` (`from_urdf`/`from_model`) + frames | ✅ implementado |

**D1 — qué se implementó**
- `thalos-planning`: nuevo `JointPlanningContext` (sin `ik_solver`) y
  `MoveJPlanner::plan_joint`; el `SegmentPlanner::plan` existente delega en él.
  `SegmentPlanningContext`/`PlanningContext` **no cambian** (Tier 3 estable).
- `thalos-api`: `pub mod intent` con `Manipulator`, `MotionRequest`, `JointMove`,
  `MotionProfile`, `MotionPlan`, `MotionError`.
- Gate: `thalos-api/examples/move_j.rs` pasó de 7 imports (con `IKSolver`) a 2 y
  es incapaz de nombrar el solver.

**D2 — qué se implementó**
- `thalos-api::intent`: `LinearMove` (solo `target` + `MotionProfile`) y
  `CartesianTarget` (Tier 1 propio, no reexport de `IKGoal`); `MotionRequest`
  gana la variante `Linear`; `MotionError` gana `Unreachable` y `Kinematics`.
- `Manipulator::plan_motion` resuelve `Linear` internamente: IK → planificación
  articular, sin exponer nada. `LinearMove` **no** lleva config de IK/sampling/
  planner (regla anti-disfraz).
- Gate: `thalos-api/examples/move_l.rs` pasó de 7 imports (dos motores) a 2 y es
  incapaz de nombrar `IKGoal`/`ForwardKinematics`/`IKSolver`/`MoveJPlanner`.

**D3 — qué se implementó**
- El objeto que cruza la frontera es el **`MotionPlan`**: `plan_motion` lo
  produce una sola vez y `intent::prepare_execution(&plan)` lo consume.
- `thalos-planning`: `ExecutionPlanBuilder::build_from_trajectory(&Trajectory,
  PlanInstruction)` — constructor Tier 3 **puro** que no replanifica.
- `thalos-api::intent`: `PreparedPlan` (Tier 1, envuelve el IR de ejecución),
  `MotionKind` y la función libre `prepare_execution`.
- **Refinamiento vs C-3:** `prepare_execution` devuelve `PreparedPlan` (el IR de
  ejecución listo), **no** un `ExecutionSession`. Una sesión es supervisión de un
  *programa* (id, configuración, ciclo de vida) y su loop vive en Industrial; para
  un movimiento suelto, el artefacto correcto es el IR. La función es libre (no
  usa el robot) e infalible.
- Gate: `thalos-api/examples/execute.rs` pasa el mismo `MotionPlan` a
  `prepare_execution` sin reconstruirlo; el consumidor no nombra `PlanCompiler`,
  `ExecutionRunner` ni `Trajectory`.

**D4 — qué se implementó**
- `thalos-api::intent`: `kinematics` module con `CartesianTarget` (movido aquí,
  su hogar natural), `IkSolution` (valor útil: `joints()`, `converged()`, …) e
  `IkRequest` (Tier 2: seed + tolerancias, **sin** exponer `IKConfig`).
- `Manipulator::inverse(target)` y `inverse_with(IkRequest)`. La no-convergencia
  se devuelve como `IkSolution { converged: false }` (IK es la pregunta); solo un
  error del solver es `Err`.
- `CartesianTarget::pose(x, y, z, UnitQuaternion)` construye una pose **sin
  `FrameId`** (usa `Pose::from_translation_rotation`), cerrando la fuga de frames
  que D2 no podía ver.
- Retiro del prototipo congelado `KinematicRobot`/`IkRequest`
  (`thalos-core/src/kinematics/robot_api.rs`) y su test `convenience_api.rs`: D4
  es su reemplazo definitivo. Esto elimina la colisión de nombres entre el
  `IkRequest` del prototipo y el de `intent`.
- Gate: `thalos-api/examples/inverse_kinematics.rs` no nombra `IKGoal`,
  `SerialChain`, `ForwardKinematics`, `DampedLeastSquaresSolver`, `IKConfig`,
  `IKResult` ni `IKSolver`.

**D5 — qué se implementó**
- `Manipulator::from_urdf(&str)` y `from_model(&Robot)`: el pipeline
  `URDF → modelo → extracción de cadena → SerialChain` queda dentro del engine.
  `from_chain` sobrevive como escape hatch.
- `RobotLoadError` (Tier 1): envuelve `AdapterError` sin exponerlo.
- **Frames:** no se fabricó una API completa. El caso común (base por defecto,
  end-effector por defecto) se resuelve sin `FrameId`; se añadió
  `end_effector_name()` para responder "¿cuál es el EE?" sin exponer `FrameId`.
  Selección explícita de tip / resolución contra frames no-default quedan como
  Tier 3 vía `chain()`.
- **Hallazgo de modelado:** en un URDF, la longitud del último eslabón debe
  expresarse como un joint (fijo) hacia un `tool`; si no, el EE cae sobre el eje
  del último joint y su columna del jacobiano es nula. El ejemplo lo refleja con
  un joint fijo a `tool0`.
- Gate: `thalos-api/examples/load_robot.rs` carga desde URDF y hace IK sin
  nombrar `SerialChain`, `Robot`, `adapter`, `FrameId` ni `ForwardKinematics`.

**Auditoría final D1–D5:** ver `docs/API-INTENT-AUDIT.md`. Añadió
`Manipulator::forward` (FK en el receptor, delegando en `KinematicContext`),
migró los ejemplos a `from_urdf` y declaró **D3-bis — Deferred, no evidence**.
Estado: **D1–D5 cerrado**.
