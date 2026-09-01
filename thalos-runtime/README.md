# thalos-runtime

**Pregunta que responde:** ¿Qué hace el robot? ¿Cómo se ejecutan comandos
sobre el modelo?

Mantiene el estado mutable del robot (ángulos articulares, robot cargado,
TCP activo, plan en ejecución) y ejecuta comandos. Es la capa que orquesta
la cinemática: recibe un comando, lo resuelve contra el modelo de core, y
devuelve un snapshot del resultado.

### Comandos (`Command` enum)

| Comando | Descripción |
|---------|-------------|
| `SetJoints(Vec<f64>)` | Mutar ángulos articulares |
| `LoadRobot(RobotModel)` | Cargar robot del catálogo |
| `LoadUrdfRobot{name, chain, robot}` | Cargar robot desde URDF |
| `Kinematics(MoveToPosition/Pose)` | Resolver IK y aplicar |
| `Motion(MoveJ/PlanAndMoveJ/PlanAndMoveL)` | Planificar y ejecutar movimiento |
| `SelectToolFrame(Option<ToolFrame>)` | Seleccionar/limpiar TCP |

### Servicios

- **SceneService** — orquestador principal, `execute(Command) → RuntimeSnapshot`
- **WorkspaceService** — muestreo Monte Carlo, análisis de alcanzabilidad,
  singularidad, manipulabilidad

Depende de `thalos-core` para tipos de dominio y solvers de IK, de
`thalos-planning` para planificadores de movimiento, y de `thalos-models`
para estructura URDF.

**No debe contener:** HTTP, representaciones visuales, lógica de validación
de escenas.
