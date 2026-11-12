# Estado real — Thalos Engine

> **Qué es esto:** lo que **funciona hoy**. Si algo dice ✅, es demostrable con el
> código y los tests en este commit.
>
> **Cómo se mantiene:** se reescribe al cerrar una fase, no en cada cambio.
>
> **Leyenda:** ✅ funciona · 🟠 parcial · ❌ no existe

Última revisión: sistema de tipos Fases 1–3, API del lenguaje curada y warnings de deprecación en el compilador.

## Pipeline del lenguaje

Cadena: `.thls` → parse → compile → resolve → ir → validation → lowering →
`ExecutionPlan`.

| Etapa | Estado | Dónde |
|---|---|---|
| Parse `.thls` | 🟠 parcial | `thalos-lang/src/parser` (Chumsky) |
| Compile / resolve / IR / lowering | ✅ | `thalos-language-service` |
| Sistema dimensional (unidades, estrictez) | ✅ Fase 1; 🟠 álgebra evaluable | `thalos-lang/src/units`, `thalos-language-service` |
| Sistema de tipos (anotaciones, `Joints<N>`, efectos, `Error` fuera del IR, registro y constructores `PascalCase`/chaining) | ✅ Fases 1–3 | `thalos-lang/src/ast/type_expr.rs`, `thalos-language-service/src/{types,effects,registry,checker,compiler,evaluator}.rs` |
| API pública del lenguaje (`lang`/`semantic`) | ✅ explícita y testeada; globs restantes en ratchet | `thalos-api/src/lib.rs`, `tests/{public_surface,public_surface_globs}.rs` |
| `movec` (arco circular) | ✅ | `thalos-planning/src/motion/move_c.rs` |
| IK (DLS) | ✅ | `thalos-core/src/kinematics/inverse` |
| `ExecutionPlan` con joints + timing | ✅ | `thalos-planning/src/execution_plan_builder.rs` |
| Colisión | 🟠 existe en `thalos-core`, no integrada al planner | `thalos-core/src/collision` |

## Dominio

| Área | Estado | Dónde |
|---|---|---|
| Cinemática FK/IK | ✅ | `thalos-core/src/kinematics` |
| Modelos canónicos (SCARA, 2R, 3R, 3DOF, RPP, RRP) | ✅ | `thalos-core/src/models` |
| Importación URDF | ✅ | `thalos-importer/src/urdf` |
| Capacidad y requisitos (Plane A/B) | ✅ | `thalos-core/src/capability.rs` |
| Semántica de comando físico | ✅ | `thalos-core/src/command` |
| IR de sesión de ejecución | ✅ | `thalos-core/src/execution/session.rs` |
| Análisis (workspace, manipulabilidad, singularidad) | ✅ | `thalos-analysis`, `thalos-core/src/analysis` |
| Current robot activo | ✅ | `thalos-core/src/robot/active_robot.rs` |

## Evidencia

| Área | Estado | Dónde |
|---|---|---|
| Trazas y muestras de ejecución | ✅ | `thalos-telemetry` |
| Analizador de trazas | ✅ | `thalos-telemetry/src/analyzer.rs` |
| Vocabulario de eventos de ciclo de vida | ✅ | `thalos-telemetry/src/event.rs` |

## Frontera pública

| Propiedad | Estado | Guarda |
|---|---|---|
| `thalos-api` sin dependencias de infraestructura | ✅ | `thalos-api/tests/dependency_purity.rs` |
| Superficie pública exactamente la declarada | ✅ | `thalos-api/tests/public_surface.rs` |
| Sin re-exports de crates internos | ✅ | `no_engine_reexports.rs` (Industrial) |

## Qué NO existe

- Mecanismo de ejecución (loop, reloj, event bus, persistencia): vive en Industrial.
- Adquisición de datos: el engine transforma evidencia, no la obtiene.
- Ejecución física: requiere hardware.
