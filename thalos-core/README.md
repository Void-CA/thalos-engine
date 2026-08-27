# thalos-core

**Pregunta que responde:** ¿Qué es un robot? ¿Cómo se modela matemáticamente?

Contiene las definiciones fundamentales del dominio: tipos geométricos
(vectores, transformaciones rígidas, cuaterniones), cinemática directa e
inversa, modelos de robot, articulaciones, y el grafo espacial de frames.

### Submódulos

| Módulo | Descripción |
|--------|-------------|
| `math` | Álgebra, geometría, rotaciones, transformaciones |
| `robot` | Joints, Links, Segments, SerialChain, ToolFrame, ActiveRobot |
| `kinematics` | FK, Jacobianos (geométrico, numérico), IK (DLS, JT) |
| `spatial` | Frames, Poses, FrameRegistry, FrameGraph |
| `models` | Catálogo de robots: Planar2R/3R, SCARA, Manipulator3DOF/6DOF, CylindricalRPP, SphericalPolarRRP, SingleRevolute |
| `collision` | Trait `CollisionChecker` |

No depende de ningún otro crate del proyecto (excepto `thalos-math` para
tipos base). Es la base sobre la que todo lo demás se construye.

**No debe contener:** estado mutable, HTTP, escenas visuales, lógica de
ejecución.
