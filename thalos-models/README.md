# thalos-models

**Pregunta que responde:** ¿Cuál es la estructura canónica de un robot?

Define **qué es un robot**: su estructura, componentes y propiedades físicas.
Cada tipo aquí es datos puros — sin algoritmos cinemáticos, sin estado runtime,
sin sistemas de frames.

Mapea 1:1 a conceptos URDF y puede serializarse sin pérdida de significado.

### Tipos

- `Robot` — contenedor top-level (links + joints + graph)
- `Link`, `Inertial` — eslabón con propiedades físicas
- `Joint`, `JointKind`, `JointLimits` — articulación
- `Geometry`, `Visual`, `Collision` — geometrías (Sphere, Box, Cylinder, Mesh)
- `Material`, `Color` — materiales visuales
- `RobotGraph` — grafo de conectividad (LinkId, JointId, Path)
- `urdf` — parser URDF

Depende de `thalos-math` para tipos geométricos base.

**No debe contener:** estado mutable, HTTP, cinemática, visualización,
algoritmos de planning.
