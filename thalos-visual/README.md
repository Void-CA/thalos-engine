# thalos-visual

**Pregunta que responde:** ¿Cómo se representa visualmente el robot? ¿Cómo se
valida y compara la escena?

Toma los resultados de cinemática directa y construye una representación
visual (`VisualScene`) con frames, links, joint axes, twists, primitivas
geométricas y `FrameStyle`. También incluye validación de escenas
(`SceneValidator` con 8 invariantes), generación de diffs entre estados
(`SceneDiff`), y builders específicos por robot (`ScaraVisualBuilder`).

### Tipos principales

- `VisualScene` — frames, links, joint_axes, twists, primitives
- `VisualPrimitive` — cilindros, esferas, cajas con color por ID
- `FrameStyle` — personalización visual por frame (colores, tamaños, etiquetas)
- `SceneDiff` — delta entre dos escenas (agregados, eliminados, modificados)
- `SceneValidator` — 8 invariantes: world, unicidad, topología, conectividad,
  finitos, quaternions, links, twists

Depende de `thalos-core` para los tipos espaciales (frames, poses).

**No debe contener:** estado mutable, HTTP, comandos de ejecución, solvers de IK.
