# Ergonomía de la API pública — Auditoría (Fase A)

> **Carácter:** diagnóstico, no normativo. Este documento **detecta fricción y la
> clasifica**; no decide abstracciones. Las "hipótesis de abstracción" están
> marcadas como tales y no son compromisos.
>
> **Alcance:** los 12 dominios del engine, con la pregunta transversal:
> *¿una operación exige Tier 3 por necesidad del dominio o por accidente de la
> API actual?*
>
> **Estado:** Fase A. Las Fases B (consumer scenarios), C (consolidación) y D
> (migración) están **fuera de alcance** de este documento.

## Método

Para cada dominio se registra:

```text
operación | API pública actual | tipos que el consumidor debe conocer
          | mecanismo expuesto | fricción | hipótesis de abstracción | veredicto de tier
```

- **Fuente de verdad del consumo:** imports de `thalos_api` en código de
  producción de `thalos-industrial/backend/crates/*/src` (tests excluidos salvo
  indicación). Medición: `rg`.
- **"Usado" ≠ "necesario".** Que Industrial importe un tipo no prueba que deba
  conocerlo. La columna de veredicto separa ambos.
- **Evidencia:** referencias `archivo:línea` a los puntos reales de consumo.
- El criterio de cierre de A está al final.

## Mapa de dominios y uso real

Conteo de imports de `thalos_api` en producción (medido en este commit):

| Dominio | Usado por Industrial | Imports | Señal de uso (evidencia) |
|---|---|---|---|
| Execution / Supervision | ✅ fuerte | 131 | `thalos-tauri/.../execution/dto.rs:4`, `runtime/.../execution/session/*` |
| Device (canales/física) | ✅ fuerte | 92 | `runtime/.../interconnection`, `thalos-transport` |
| Robot / Models / loading | ✅ fuerte | 45 + 13 + 5 | `runtime/.../application/semantic.rs`, `scene/service.rs` |
| Kinematics (FK/IK/Jacobian) | ✅ fuerte | 23 | `runtime/.../scene/service.rs:498`, `scene/state.rs:152` |
| Planning | ✅ | 26 | `runtime/.../application/planning.rs:408`, `application/semantic.rs:251` |
| Ports (contratos I/O) | ✅ | 40 | `thalos-transport/...`, `runtime/.../infrastructure` |
| Importer | ✅ | 20 | `thalos-tauri/.../scene/commands.rs:157` |
| Analysis | ✅ | 11 | `runtime/.../execution/session/deviation.rs:8`, `thalos-tauri/.../workspace` |
| Language / Semantic | ✅ | 11 + 3 | `thalos-tauri/.../features/language/commands.rs:1`, `semantic/commands.rs:47` |
| Telemetry / Evidence | ✅ | (vía grupo) | `runtime/.../telemetry/observer.rs:5`, `telemetry/motion/recorder.rs:20` |
| Math / Spatial | ✅ | 6 (+math) | `application/planning.rs:18`, `commands/kinematics.rs:4` |
| Capability / Command | ✅ | 14 + 6 | `runtime/.../infrastructure/execution/hardware/executor.rs:6` |
| Trajectory | ✅ | 18 | `runtime/.../scene/tests/scene.rs:37` (usa `TrajectoryPoint`) |
| Document / Visual | ✅ | 2 + 2 | `runtime/.../application/semantic.rs:4`, `thalos-tauri/.../scene/mapper.rs:6` |

**Resultado:** los 12 dominios se consumen. No hay dominios "de biblioteca sin
consumidor" salvo por `catalog` (5), que Industrial usa indirectamente.

## Verdictos de tier — definiciones

| Veredicto | Significado |
|---|---|
| **Necesario** | El dominio exige conocer el mecanismo; Tier 3 es legítimo. |
| **Accidental** | El consumidor conoce Tier 3 por cómo está construida la API, no porque la operación lo requiera. |
| **Mixto** | Parte del conocimiento es de dominio; parte es mecánica de la API. |
| **Indeterminado** | Falta evidencia; se resuelve en Fase B. |

---

## 1. Math / Spatial

**Operaciones:** construir vectores, transformaciones, posiciones y poses.

| Aspecto | Observación |
|---|---|
| API actual | `math::{Vector3, Transform3D, UnitQuaternion, Quaternion}`, `core::spatial::{frame::FrameId, pose::Pose}` |
| Tipos que el consumidor conoce | `Vector3`, `Transform3D`, `UnitQuaternion` (vocabulario de dominio, sano) |
| Mecanismo expuesto | `Pose` exige `FrameId` de referencia y destino (`pose.rs:14`); `FrameId::Id(u64)` es plomería interna |
| Fricción | Para expresar un objetivo hay que conocer `FrameId`; no existen constructores `from_xyz` (hasta el prototipo de Fase 0) |
| Evidencia | `application/planning.rs:18`; `commands/kinematics.rs:4`; `thalos-tauri/.../scene/commands.rs:7` |
| Hipótesis (no decisión) | Constructores de conveniencia en `Pose`; `FrameId` permanece accesible |
| Veredicto | **Mixto** — el vocabulario es de dominio; `FrameId` en la construcción de poses es accidental |

## 2. Kinematics (FK / IK / Jacobian)

**Operaciones:** cinemática directa, inversa, jacobiano.

| Aspecto | Observación |
|---|---|
| API actual | `kinematics::{forward::ForwardKinematics, inverse::{DampedLeastSquaresSolver, IKSolver, IKGoal, IKConfig}, KinematicContext}` |
| Tipos que el consumidor conoce | `ForwardKinematics`, `DampedLeastSquaresSolver`, `IKSolver`, `IKGoal`, `IKConfig`, `FrameId`, `SerialChain` |
| Mecanismo expuesto | El consumidor **construye el solver** (`from_config`/`new`), elige el frame y pasa la seed |
| Fricción | 6+ imports por sitio; `IKSolver` (trait interno) como entrada; rutas a `inverse::result` y `forward::result` |
| Evidencia (IK) | `runtime/.../application/scene/service.rs:498-531` (`from_config` + wrapper `solve_ik_at_position`); `scene/state.rs:152-165` (`DLS::new`); `commands/kinematics.rs:19,24`; `commands/motion.rs:29-32,149-151`; `application/semantic.rs:180,251`; `application/planning.rs:409-415,541`; `thalos-tauri/.../scene/commands.rs:313-419` |
| Evidencia (FK/TCP) | `KinematicContext`: `execution/kinematics_registry.rs:53`, `execution/service.rs:213-214`, `execution/session/tick_loop.rs:1134,1282` |
| Precedente de diseño | `KinematicContext` (`context.rs:58`) **ya es intención**: `tcp_pose(joints)`, `resolve(joints)`, `at_end_effector(chain)` |
| Hipótesis (no decisión) | Capa de intención FK/IK/jacobiano; el prototipo `KinematicRobot`/`IkRequest` (Fase 0) fue el candidato validado y luego retirado en D4 |
| Veredicto | **Accidental** — la operación "solicitar IK" no requiere conocer el solver; `KinematicContext` demuestra que la intención es representable sin él |

## 3. Robot / Models / carga

**Operaciones:** cargar un robot (URDF o catálogo), obtener su cadena, su estado.

| Aspecto | Observación |
|---|---|
| API actual | `models::Robot`, `models::robot_asset::RobotAsset`, `importer::import_urdf(_resolved)`, `core::robot::adapter::{from_urdf, auto}`, `catalog::{RobotModel, RobotRegistry}`, `core::robot::active_robot::ActiveRobot`, `core::robot::serial_chain::SerialChain` |
| Tipos que el consumidor conoce | `Robot` + `import_urdf` + `adapter` + `RobotRegistry` + `SerialChain` (cadena de 3–4 pasos) |
| Mecanismo expuesto | No hay una operación "cargar robot"; el consumidor compone import + adapter + registry |
| Fricción | Dos rutas al mismo concepto (`models::Robot` y `models::robot::Robot`, esta última en `thalos-runtime/src/infrastructure/robot/adapter/importer.rs:9`); `model → chain` es manual |
| Evidencia | `runtime/.../application/scene/service.rs:465-496` (`load_urdf_robot_command` recibe `Robot` + `SerialChain` ya construidos); `thalos-tauri/.../scene/commands.rs:157,196-224` (`import_urdf_resolved` + `UrdfAssetDiscovery` + `UriResolver`); `application/workspace.rs:162` (`RobotModel::Planar2R`) |
| Hipótesis (no decisión) | Operación "load": `Robot::load(urdf) -> robot listo` que encapsule import+adapter+registry |
| Veredicto | **Accidental** — el dominio es "tengo un robot"; la secuencia import→adapter→registry es mecánica de la API |

## 4. Motion / Trajectory

**Operaciones:** MoveJ, MoveL, interpolación, generar trayectoria.

| Aspecto | Observación |
|---|---|
| API actual | `planning::motion::{program::{CompiledPlan, PlanningProgram}, planner::{PlanningContext, SegmentPlanner}, move_j::{MoveJPlanner, MoveJConfig}, compiler::{PlanCompiler, DefaultPlannerDispatcher}}`, `planning::goal::{ValidatedGoal, GoalMetadata, PlanningAssessment, GoalResolver*, PlanningPolicy}`, `core::trajectory::Trajectory`, `core::motion::segment::MotionSegment` |
| Tipos que el consumidor conoce | **10+** para un solo MoveL |
| Mecanismo expuesto | IK → `RobotState` → `PlanningContext` → `ValidatedGoal` → `MoveJPlanner`, todo ensamblado a mano |
| Fricción | El caso testigo más claro; el consumidor construye el solver, el estado, el contexto y la meta |
| Evidencia | `runtime/.../application/commands/motion.rs:29-32` (`make_ik_solver`), `:35-47` (`make_planning_ctx`), `:135-176` (`PlanAndMoveL`) |
| Precedente de diseño | No existe operación de intención hoy; `PlanAndMoveL` es un `enum` de comando que **internamente** arma el pipeline |
| Hipótesis (no decisión) | Petición de movimiento (`MotionRequest` / `robot.move_l(pose)`) que encapsule IK+contexto+planner |
| Veredicto | **Accidental** — "mover a esta pose" no justifica conocer `PlanningAssessment` ni `MoveJConfig` |

## 5. Planning

**Operaciones:** planificar un programa, compilar, editar.

| Aspecto | Observación |
|---|---|
| API actual | `planning::{input::{PlanningInput, PlanningStep}, motion::compiler::PlanCompiler, program_edit::ProgramEdit, error::{PlanningError, CompileError}, execution_plan_builder::ExecutionPlanBuilder}` |
| Tipos que el consumidor conoce | `PlanCompiler`, `DefaultPlannerDispatcher`, `PlanningInput`, `MovementResolver`, `PlanningContext` |
| Mecanismo expuesto | El consumidor instancia compilador + dispatcher + contexto y los encadena |
| Fricción | El pipeline compile→resolve→lower→plan es visible; `application/semantic.rs` lo reconstruye |
| Evidencia | `runtime/.../application/planning.rs:400-424`; `application/semantic.rs:185-196,248-258`; import `planning::goal::{…}` |
| Hipótesis (no decisión) | Facade de planificación sobre el pipeline (una llamada para los casos normales) |
| Veredicto | **Mixto** — la semántica de planificación es de dominio; el ensamblado de compilador/dispatcher/contexto es mecánica |

## 6. Capability / Command / Program

**Operaciones:** declarar capacidades, emitir comandos de robot, definir programas.

| Aspecto | Observación |
|---|---|
| API actual | `core::capability::{CapabilityRequest, CommandCapability, EvidenceLevel}`, `core::robot::{RobotCommand, capability::RobotCapability}`, `core::program`, `core::command` |
| Tipos que el consumidor conoce | `RobotCommand`, `CommandCapability`, `RobotCapability`, `EvidenceLevel` |
| Mecanismo expuesto | Poco; son vocabulario de dominio |
| Fricción | Baja; los tipos se usan como datos, no como pipeline |
| Evidencia | `runtime/.../infrastructure/execution/hardware/executor.rs:6,312`; `infrastructure/execution/dispatcher.rs:103,113`; `thalos-tauri/.../features/execution/commands.rs:453` |
| Veredicto | **Necesario** — el consumidor necesita este vocabulario para operar el robot; exposición legítima |

## 7. Execution / Supervision

**Operaciones:** ejecutar un plan, supervisar la sesión, evaluar ticks.

| Aspecto | Observación |
|---|---|
| API actual | `core::execution::{session::{ExecutionSession, ExecutionSessionState, TickContext, TickOutcome, Decision, Action, ObservationBundle, RobotSample, ExpectedState, RuntimeState}, runner::{ExecutionRunner, ObservationProvider, CommandProvider}, plan::ExecutionPlan, execution::ExecutionSource, TemporalInvariants}` |
| Tipos que el consumidor conoce | Muchos, pero son la **semántica** que el engine posee por diseño (ver `ARCHITECTURE.md`: el engine define el significado; Industrial posee el mecanismo) |
| Mecanismo expuesto | Rutas profundas `session::*`, `runner::*`; el consumidor implementa los traits del runner |
| Fricción | Volumen alto (131 imports), pero justificado por contrato de dominio |
| Evidencia | `thalos-tauri/.../features/execution/dto.rs:4-6,62-66`; `runtime/.../infrastructure/execution/dispatcher.rs:12-13`; `runtime/.../application/queries.rs:15`; `runtime/.../execution/session/runners/observed.rs:1` |
| Hipótesis (no decisión) | Ninguna en A; candidato a **reorganización de namespace** (rutas más cortas), no a ocultamiento |
| Veredicto | **Necesario** — es el dominio que el engine **debe** exponer; la fricción es de nombres/rutas, no de mecanismo |

## 8. Device / Ports (contratos de I/O)

**Operaciones:** transportar a robot/dispositivo, observar canales.

| Aspecto | Observación |
|---|---|
| API actual | `ports::{robot::{RobotTransport, RobotObservation, TransportState}, device::{DeviceTransport, ChannelObservation, SignalQuality, ChannelId}}`, `core::device::{…}` |
| Tipos que el consumidor conoce | Traits de transporte + observaciones (necesarios para implementar el I/O) |
| Mecanismo expuesto | Traits — legítimo: son el contrato que Industrial implementa |
| **Hallazgo de frontera** | `thalos-transport` importa **`thalos_ports` directo** (11 imports, 6 archivos) y **no depende de `thalos-api`**: `thalos-transport/Cargo.toml` → `thalos-ports = { path = ... }`. Evidencia: `thalos-transport/src/mqtt_bridge/transport.rs:16,19`, `esp32/device.rs:2,6,7`, `esp32/robot.rs:2,5`, `terminal_controller/transport.rs:10`, `mqtt_bridge/codec.rs:25`, `common.rs` |
| Fricción | El único consumidor que rompe la regla "importar solo `thalos-api`" |
| Evidencia (vía api) | `runtime/.../infrastructure/*`; `thalos_api::ports`×40 |
| Hipótesis (no decisión) | Evaluar si `thalos-transport` debe depender de `thalos-api::ports` o si es una excepción deliberada documentada |
| Veredicto | **Accidental** (el bypass de frontera) — los traits son de dominio, pero la ruta directa a `thalos_ports` merece justificación explícita |

## 9. Language / Semantic (.thls)

**Operaciones:** parsear, compilar, resolver, analizar un programa `.thls`.

| Aspecto | Observación |
|---|---|
| API actual | `lang::parse_source`, `semantic::{compiler::SemanticCompiler, resolver::SemanticResolver, intelligence::{analyze_intelligence, DocumentIntelligence}, analyze_document, model::{MotionKind, MotionTarget}}`, `document::program_document::ProgramDocument` |
| Tipos que el consumidor conoce | Pipeline parse→compile→resolve→lower explícito |
| Mecanismo expuesto | El consumidor encadena etapas; rutas profundas (`semantic::compiler::`, `semantic::resolver::`) |
| Fricción | El pipeline es visible; el consumidor lo rearma en cada entrada |
| Evidencia | `thalos-tauri/.../features/language/commands.rs:1-16`; `features/semantic/commands.rs:47`; `runtime/.../application/semantic.rs:233-258` |
| Hipótesis (no decisión) | Facade "compilar fuente `.thls`" que oculte las etapas (el consumidor pide el resultado, no la orquestación) |
| Veredicto | **Mixto** — las etapas tienen valor (diagnóstico), pero la orquestación es mecánica |

## 10. Analysis

**Operaciones:** workspace, singularidad, manipulabilidad, comparación de ejecución.

| Aspecto | Observación |
|---|---|
| API actual | `analysis::{WorkspaceService, SingularityService, ManipulabilityService, PlanExecutionComparison, compare_points, compare_tick, detect_deviations}`, subpaths `analysis::{workspace, singularity::, manipulability::, observation, location}` |
| Tipos que el consumidor conoce | Servicios + reportes; `WorkspaceConfig` |
| Mecanismo expuesto | Bajo: los servicios son la operación |
| Fricción | Rutas profundas (`analysis::manipulability::`), pero el patrón "servicio" ya es de intención |
| Evidencia | `runtime/.../execution/session/deviation.rs:8`; `thalos-tauri/.../features/workspace/commands.rs:31-34`; `features/workspace/mapper.rs:5`; `analysis/comparison.rs` |
| Hipótesis (no decisión) | Ninguna urgente; posible acortado de rutas |
| Veredicto | **Necesario** — `*Service` ya expone intención; es el patrón que Kinematics no tiene |

## 11. Telemetry / Evidence

**Operaciones:** producir/leer trazas y muestras de ejecución.

| Aspecto | Observación |
|---|---|
| API actual | `telemetry::{ExecutionTrace, MotionTrace, ExecutionSample, MotionSample, TraceMetadata, TraceAnalyzer, TelemetryLifecycleEvent}` |
| Tipos que el consumidor conoce | Tipos de evidencia (vocabulario); el mecanismo de captura vive en Industrial (`runtime/.../telemetry/mod.rs:4` lo declara) |
| Mecanismo expuesto | Bajo; la frontera "el engine transforma, no adquiere" ya está respetada |
| Evidencia | `runtime/.../telemetry/observer.rs:5`; `telemetry/motion/recorder.rs:20`; `telemetry/motion/adapter.rs:2`; `telemetry/recorder.rs:6` |
| Veredicto | **Necesario** — vocabulario de dominio, bien separado |

## 12. Document / Visual

**Operaciones:** representar documentos de programa y escenas visuales.

| Aspecto | Observación |
|---|---|
| API actual | `document::program_document::ProgramDocument`, `visual::{SceneBuilder, VisualScene, project_spatial, trajectory::…}` |
| Tipos que el consumidor conoce | `ProgramDocument`, tipos de escena visual |
| Mecanismo expuesto | Bajo |
| Fricción | Uso escaso (2+2 imports); `program_document` es una ruta profunda simple |
| Evidencia | `runtime/.../application/semantic.rs:4`; `thalos-tauri/.../features/semantic/commands.rs:3`; `thalos-tauri/.../features/execution/dto.rs:451`, `scene/mapper.rs:6`, `planning/mapper.rs:7` |
| Veredicto | **Necesario** — representación, no mecanismo |

---

## Hallazgos transversales

1. **Tier 3 como puerta de entrada accidental** en Kinematics, Robot-loading, Motion y (parcial) Planning. Contrasta con Analysis y Telemetry, donde el patrón "servicio/vocabulario" ya expresa intención.
2. **Rutas `::result::` y traits como entrada.** El consumidor importa `forward::result::FKResult`, `inverse::result::IKResult`, `inverse::IKSolver`. Los módulos `result` y los traits internos se filtran al contrato.
3. **Sin operación de carga unificada de robot.** `Robot` → `import_urdf` → `adapter` → `SerialChain` es manual, y hay dos rutas al mismo concepto (`models::Robot` vs `models::robot::Robot`).
4. **Fricción medida ≠ fricción real.** Execution tiene el mayor volumen (131) pero es **necesario**; Motion tiene menos imports pero la peor relación intención/mecanismo (10+ tipos por MoveL).
5. **Anomalía de frontera:** `thalos-transport` es el único consumidor que importa un crate interno del engine (`thalos_ports`) directamente y no depende de `thalos-api`.
6. **El precedente de diseño correcto ya existe en el propio engine:** `KinematicContext` (intención FK), y los `*Service` de Analysis. Son la referencia para cualquier consolidación futura.

## Prototipo congelado

`KinematicRobot` / `IkRequest` (`thalos-core/src/kinematics/robot_api.rs`) fueron
un prototipo de validación durante A y B. Los escenarios de B confirmaron el
patrón y la Fase C lo especificó como `Manipulator` + tipos de intención; el
prototipo se **retiró en D4** (código y test), reemplazado por `thalos_api::intent`.
Nunca se difundió a Industrial.

## Criterio de cierre de Fase A

Este documento permite:

- [x] recorrer los 12 dominios;
- [x] identificar qué usa realmente Industrial (mapa de uso + evidencia);
- [x] identificar qué conocimiento interno exige cada operación (columna "tipos que el consumidor debe conocer");
- [x] distinguir exposición legítima de accidental (veredicto de tier por dominio).

## Preguntas para Fase B (no se responden aquí)

1. ¿Tier 1 es **un** objeto de intención o una familia (robot/motion/planning)?
2. ¿La reorganización de Execution es de namespace o de superficie?
3. ¿`thalos_ports` directo en `thalos-transport` es excepción documentada o deuda a cerrar?
4. ¿La operación de carga de robot debe ser `Robot::load` o un servicio?
5. ¿Qué dominios (si alguno) deben permanecer Tier 3 como entrada recomendada?
