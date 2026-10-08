# Consumer scenarios — Fase B

Estos ejemplos son un **harness de validación de la API pública**, no demos.
Cada uno reproduce un flujo real de `thalos-industrial` escrito **como un
consumidor externo** que solo puede importar `thalos-api`, sin mirar la
implementación interna.

Regla de la fase: **no se optimizan los ejemplos para que compilen.** Cuando una
intención sencilla obliga a importar mecanismo, ese coste se deja visible y se
documenta. Si algo no existe (p. ej. `robot.move_l(pose)`), se muestra en el
encabezado como bloque `ignore`, no se disfraza.

```bash
cargo run -p thalos-api --example <scenario>
```

## Escenarios y veredicto

| # | Escenario | Intención | Líneas `use` | Mecanismo filtrado | Veredicto |
|---|---|---|---|---|---|
| B1 | `load_robot` | `Manipulator::from_urdf(src)` | **1** | ✅ resuelto en D5 (`intent::Manipulator`) | Resuelto |
| B2 | `forward_kinematics` | `robot.forward(&q)` | **2** | ✅ alineado en la auditoría (`Manipulator::forward`) | Resuelto |
| B3 | `inverse_kinematics` | `robot.inverse(target)` | **2** | ✅ resuelto en D4 (`intent::IkSolution`) | Resuelto |
| B4 | `move_j` | `robot.plan_motion(joint)` | **2** | ✅ resuelto en D1 (`intent::Manipulator`) | Resuelto |
| B5 | `move_l` | `robot.plan_motion(linear)` | **2** | ✅ resuelto en D2 (IK→MoveJ interno) | Resuelto |
| B6 | `plan` | `robot.plan_source(source)` | 10 | orquesta 6 etapas — **gap D3-bis (deferred)** | Gap |
| B7 | `execute` | `prepare_execution(&plan)` | **2** | ✅ resuelto en D3 (mismo `MotionPlan`) | Resuelto |
| B8 | `frames` | `Pose::from_xyz(x,y,z)` | 3 | `FrameId`/`FrameRegistry` en la construcción de poses | Mixta |

> **D1 (hecho):** `move_j` ya no conoce `IKSolver`: usa
> `thalos_api::intent::{Manipulator, MotionRequest, JointMove}`. El gate es que el
> archivo sea *incapaz* de nombrar el solver. La separación interna
> (`JointPlanningContext`, sin solver) es lo que lo hace posible.
>
> **D2 (hecho):** `move_l` ya no conoce el pipeline `IK → MoveJPlanner`: usa
> `thalos_api::intent::{LinearMove, CartesianTarget}` sobre el mismo
> `plan_motion`. `LinearMove` expone **solo** el target y un `MotionProfile`
> opcional — ninguna config de IK, sampling o planner. El engine resuelve
> internamente cartesiano → IK → planificación articular.
>
> **D3 (hecho):** `execute` deja de re-derivar el plan. El **mismo `MotionPlan`**
> producido por `plan_motion` se pasa a `intent::prepare_execution(&plan)`, que
> construye el IR de ejecución desde la trayectoria ya calculada (sin volver a
> resolver IK ni planificar). El consumidor no conoce `PlanCompiler`,
> `ExecutionRunner` ni `Trajectory`; el engine no ejecuta (no hay loop/clock).

## Hallazgos de Fase B

1. **El criterio de A se sostiene.** El problema no es el número de imports:
   B7 (`execute`) tiene los más y es **necesario**; B4 (`move_j`) tiene menos y es
   el peor caso de intención-vs-mecanismo.
2. **FK ya tiene su capa de intención** (`KinematicContext`). Es la referencia a
   imitar; su patrón (`at_end_effector` + `tcp_pose`) no filtra el algoritmo.
3. **MoveJ es peor que IK.** Un movimiento puramente articular arrastra el solver
   de IK (campo obligatorio de `PlanningContext`) que nunca se usa.
4. **MoveL demuestra el salto de capa.** El consumidor encadena dos motores; el
   gap conceptual MoveJ→MoveL es nulo, el gap de API es grande.
5. **`execute` obliga a re-derivar el plan por completo** (`compile_plan` en el
   ejemplo es copia del pipeline de B6). Ejecutar y planificar no están unidos.
6. **Ningún escenario cruzó capas sin fricción**: cada salto
   (Robot→Kinematics→Motion→Planning→Execution) reexpone Tier 3.

## Frontera de la fase

- El prototipo `KinematicRobot` / `IkRequest` fue retirado en **D4**, reemplazado
  por `thalos_api::intent`.
- Slice **D4 (hecho):** `inverse_kinematics` usa `robot.inverse(CartesianTarget)`
  y recibe un `IkSolution`; no conoce solver, config ni `IKGoal`. Para control
  avanzado, `IkRequest` es Tier 2.
- Slice **D5 (hecho):** `load_robot` usa `Manipulator::from_urdf(src)`; el
  pipeline `URDF → modelo → SerialChain` queda dentro del engine. Frames: el caso
  común no expone `FrameId`; `end_effector_name()` responde cuál es el EE.
- **Auditoría final D1–D5 (hecha):** ver `docs/API-INTENT-AUDIT.md`. Corrigió dos
  incoherencias (FK no estaba en el receptor; 4 ejemplos construían vía catálogo)
  y confirmó la inversión de dependencia (los 6 ejemplos de intención no nombran
  Tier 3; `intent` sí conoce los mecanismos).
- Fase C (consolidación) cerrada; **D1–D5 cerrados**. `plan_source` (B6/D3-bis)
  queda como **Deferred — no evidence** hasta que un programa real lo justifique.
