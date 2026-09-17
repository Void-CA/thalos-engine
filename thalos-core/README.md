# thalos-core

**Pregunta que responde:** ¿Qué es un robot? ¿Cómo se modela matemáticamente?

Contiene las definiciones fundamentales del dominio: tipos geométricos
(vectores, transformaciones rígidas, cuaterniones), cinemática directa e
inversa, modelos de robot, articulaciones, y el grafo espacial de frames.

### Submódulos

| Módulo | Descripción |
|--------|-------------|
| `robot` | Joints, Links, Segments, SerialChain, ToolFrame, ActiveRobot |
| `kinematics` | FK, Jacobianos (geométrico, numérico), IK (DLS, JT) |
| `spatial` | Frames, Poses, FrameRegistry, FrameGraph |
| `models` | Catálogo cinemático de robots: Planar2R/3R, SCARA, Manipulator3DOF/6DOF, CylindricalRPP, SphericalPolarRRP, SingleRevolute |
| `motion`, `trajectory`, `program`, `command`, `operation` | Dominio de movimiento, programas y comandos físicos |
| `resource`, `capability`, `station` | Vocabulario de recursos/estaciones referenciado por `command` |
| `skill`, `deviation`, `evaluation` | Skills, análisis de desviación y métricas |
| `collision` | Trait `CollisionChecker` |

Depende de `thalos-math` (tipos base), `thalos-models` (estructura URDF) y
`thalos-importer` (importación URDF, vía `robot::adapter`).

Los analizadores y servicios de análisis se extrajeron a `thalos-analysis`.

**No debe contener:** estado mutable, HTTP, escenas visuales, lógica de
ejecución.
