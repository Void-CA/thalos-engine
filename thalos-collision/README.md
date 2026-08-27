# thalos-collision

**Pregunta que responde:** ¿Chocan dos objetos en el espacio?

Implementa algoritmos concretos de detección de colisiones que cumplen el
contrato definido en `thalos_core::collision::CollisionChecker`.

### Componentes

- **NaiveCollisionChecker** — O(n²) sin optimizaciones
- **SAT** — Separating Axis Theorem para cajas orientadas (OBB)
- **Sphere-box** — Intersección esfera-caja
- **Clasificación** — Tipos de colisión semántica

Depende de `thalos-core` para tipos geométricos y el trait `CollisionChecker`.

**No debe contener:** estado mutable, HTTP, lógica de planificación.
