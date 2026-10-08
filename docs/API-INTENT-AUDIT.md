# Auditoría final de la API de intención (D1–D5)

> **Carácter:** pasada de coherencia y evidencia sobre los cinco gates de Fase D.
> No introduce diseño nuevo; cierra o documenta lo que ya existe.
>
> **Resultado:** API ergonomics **D1–D5 cerrado**. Una operación (planificar
> programas) queda como **D3-bis — Deferred, no evidence**.

## 1. Superficie pública por operación

| Operación | Tier 1 | Tier 2 | Tier 3 (oculto o escape) |
|---|---|---|---|
| Cargar robot | `Manipulator::from_urdf` / `from_model` | — | `from_chain` / `SerialChain` (escape) |
| FK | `Manipulator::forward` | — | `ForwardKinematics`, `KinematicContext` |
| IK | `Manipulator::inverse` | `IkRequest` | `DampedLeastSquaresSolver`, `IKGoal`, `IKConfig` |
| MoveJ | `plan_motion` + `JointMove` | `MotionProfile` | `MoveJPlanner`, `JointPlanningContext` |
| MoveL | `plan_motion` + `LinearMove` | `MotionProfile` | IK + `MoveJPlanner` |
| Preparar ejecución | `prepare_execution` → `PreparedPlan` | — | `ExecutionPlanBuilder`, `ExecutionPlan` |
| Ejecución | *(no es del engine)* | — | `ExecutionSession` / runtime (Industrial) |
| Frames | `CartesianTarget` (posición/pose) | — | `FrameId`, `FrameRegistry` (escape) |

**Pregunta de cierre:** ¿alguna operación principal obliga al consumidor a
conocer un mecanismo para expresar una intención? **No** — salvo planificar
programas (§7).

## 2. Los 8 ejemplos como un único recorrido

`load_robot → forward_kinematics → inverse_kinematics → move_j / move_l → plan → execute` (+ `frames`).

Vocabulario público, verificado coherente:

- **Objetivo:** un solo concepto, `CartesianTarget` (`.position` / `.pose`), usado
  por `inverse` y `LinearMove`. No aparece `IKGoal` ni `Pose` interna en el caso
  común.
- **Movimiento:** un solo receptor, `Manipulator::plan_motion`, con
  `MotionRequest::{Joint, Linear}`. MoveJ y MoveL se expresan igual.
- **Resultado:** `IkSolution`, `MotionPlan`, `PreparedPlan` — valores Tier 1.
- **Carga:** `Manipulator::from_urdf` para todos los escenarios.

**Inconsistencias encontradas y corregidas en esta auditoría:**

1. FK no estaba en el receptor: el ejemplo construía una cadena desde el
   catálogo (`RobotRegistry`) y usaba `KinematicContext` directamente. Se añadió
   `Manipulator::forward` (delega en la misma autoridad) y el ejemplo usa
   `from_urdf` + `forward`.
2. Cuatro ejemplos (move_j, move_l, inverse, execute) todavía construían vía
   `RobotRegistry::create_default` + `from_chain` (catálogo = Tier 3). Migrados a
   `Manipulator::from_urdf`.

## 3. Semántica de errores

Regla que explica las APIs (documentada, no universal):

> **Una operación de análisis puede devolver un resultado que describe un estado
> fallido; una operación que promete producir un artefacto ejecutable debe fallar
> si no puede producirlo.**

| Operación | Falla de IK | DOF mismatch | Carga inválida |
|---|---|---|---|
| `inverse` / `inverse_with` | `Ok(IkSolution { converged: false })` | — | — |
| `plan_motion(Linear)` | `Err(Unreachable)` | — | — |
| `plan_motion(Joint)` | — | `Err(DofMismatch)` | — |
| `forward` | — | `Err(DofMismatch)` | — |
| `from_urdf` / `from_model` | — | — | `Err(RobotLoadError)` |

Sin contradicciones: `inverse` responde *la pregunta* (IK es aproximada); `plan_motion`
*produce* un artefacto y falla si no puede.

## 4. `Manipulator` no es God Object

Métodos: `from_urdf`, `from_model`, `from_chain`, `forward`, `inverse`,
`inverse_with`, `plan_motion`, `chain`, `dof`, `end_effector_name`.

- Todas son operaciones sobre **el recurso** (un robot), no sobre otros
  subsistemas.
- **No contiene estado de pipeline**: sin plan activo, sin sesión, sin joints
  actuales, sin solver cacheado. El estado entra como parámetro (`from: &[f64]`)
  o como request.
- `chain()` es el único escape a Tier 3.

## 5. Prueba mecánica de fugas

Búsqueda de Tier 3 en el **código** (no doc-comentarios) de `thalos-api/examples`:

```text
plan.rs     → RobotRegistry, ForwardKinematics, PlanCompiler, SegmentPlanningContext   (gap D3-bis, §7)
frames.rs   → FrameId, FrameRegistry                                                    (escenario SOBRE frames, deliberado)
resto (6)   → 0
```

Búsqueda en la **implementación** de `thalos-api/src/intent`:

```text
manipulator.rs → 15 referencias a mecanismos (legítimas)
kinematics.rs  → 2
mod.rs         → 2
```

**Conclusión:** la dirección de dependencia está invertida — `intent` conoce los
mecanismos; el consumidor no.

## 6. Antes vs después

```text
ANTES (consumidor)                     DESPUÉS (consumidor)
──────────────────                     ────────────────────
Robot                                  Manipulator
SerialChain                            CartesianTarget
ForwardKinematics                      JointMove / LinearMove
IKSolver                               MotionRequest
IKConfig                               MotionProfile
IKGoal                                 IkRequest
PlanningContext                        MotionPlan
MoveJPlanner                           PreparedPlan
PlanCompiler                           IkSolution
ExecutionRunner
FrameId
```

No se ocultaron imports: cambió la **unidad conceptual** con la que el consumidor
habla con el engine.

## 7. D3-bis — Deferred, no evidence

Planificar **programas** (`.thls`) no tiene API de intención. `examples/plan.rs`
sigue en Tier 3 y se mantiene visible **a propósito**.

Esto **no es un pendiente técnico** ("sabemos que falta"), es una decisión
arquitectónica: *podría* hacer falta, pero no hay evidencia de que los programas
la necesiten. Si un programa real la demuestra, se abre D3-bis con ese caso.

Candidato (no comprometido): `Manipulator::plan_source(source) -> Plan`.

## 8. Criterio de cierre

- [x] los 8 ejemplos son consumidores externos plausibles;
- [x] ninguna operación principal filtra Tier 3 (salvo el gap D3-bis documentado);
- [x] `Manipulator` sigue siendo una frontera razonable, sin estado de pipeline;
- [x] los errores tienen semántica consistente (regla §3);
- [x] los frames no están sobre-modelados (caso común sin `FrameId`);
- [x] `MotionPlan → PreparedPlan` no replanifica;
- [x] no quedan fugas accidentales importantes.

**API ergonomics D1–D5 cerrado.** Siguiente paso (fuera de este documento):
poner la capa frente a `thalos-industrial` como consumidor real.
