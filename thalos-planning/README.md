# thalos-planning

**Pregunta que responde:** ¿Cómo se mueve el robot de A a B?

Planificadores de movimiento e interpolación de trayectorias para robots
seriales.

### Componentes

- **MoveJPlanner** — Planificación en joint space (trayectoria sincronizada)
- **MoveLPlanner** — Planificación en cartesian space (línea recta del EE)
- **GoalResolver** — Validación de metas (límites articulares, políticas)
- **Interpolación** — Rampas de velocidad/aceleración, time scaling
- **Trajectory** — Representación de trayectorias con waypoints y metadatos
- **Collision** — Chequeo de colisiones durante la planificación

Depende de `thalos-core` para tipos de dominio y cinemática.

**No debe contener:** estado mutable, HTTP, representaciones visuales.
