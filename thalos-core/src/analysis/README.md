# Thalos Analysis Module — Canonical Observation Language

El módulo `analysis` define el lenguaje canónico utilizado por Thalos para
representar conocimiento derivado del análisis de artefactos. No contiene
lógica de presentación ni algoritmos de análisis; únicamente define el modelo
compartido entre analizadores, agregadores, renderers y consumidores.

(The `analysis` module defines the canonical language Thalos uses to represent
knowledge derived from artifact analysis. It contains no presentation logic and
no analysis algorithms; it only defines the model shared between analyzers,
aggregators, renderers, and consumers.)

## Filosofía (Philosophy)

1. **Hechos separados de presentación (I1)**
   `Observation` es un hecho puro del dominio: no hay `message`, iconos ni
   directivas de UI. Toda representación humana es responsabilidad exclusiva de
   un renderer (trait `Renderer`, fases posteriores).

2. **Machine-readability (I2)**
   `kind` + `artifact` + `location` identifican un fenómeno sin parsear texto.
   `kind` denota un fenómeno (NearSingularity, TrackingError…), no una
   clasificación de display. `attributes` son datos tipados
   ([`AttributeValue`]: `Number | Text | Bool | Integer`), nunca strings de
   presentación.

3. **Trazabilidad por `causes` / `related` (I4)**
   `causes[]` define una relación dirigida y acíclica entre observaciones;
   `related[]` agrupa sin dirección causal. La validez del grafo (ciclos,
   referencias colgantes) se verifica a nivel de reporte, no en cada
   observación.

4. **Política de `attributes`**
   Las claves son strings estables y con convención (p. ej. `value`,
   `threshold`, `object_id`). No se introduce un tipo `AttributeKey`: el
   vocabulario de claves vive en cada analizador y se documenta por
   `ObservationKind`. El valor SIEMPRE es un `AttributeValue` tipado — sin
   `Box<dyn Any>` ni mapas con orden no determinista (decisión D5).

5. **Extensibilidad sin catch-all (C4)**
   `ObservationKind`, `Location` y `ArtifactRef` son `#[non_exhaustive]`:
   se agregan variantes sin romper matches externos y sin degradar la
   machine-readability con variantes `Other(String)`.

## Decisiones de nomenclatura (canónicas, fijadas en PR 1a)

| Nombre | Rol |
|--------|-----|
| `ObservationKind` | Fenómeno (NearSingularity, TrackingError, PlaceWithoutPick…) |
| `ArtifactRef` | Enum tipado `Robot \| Scene \| SemanticProgram \| MotionPlan \| ExecutionSession`, cada uno con su id |
| `Location` | Enum `Joint \| Waypoint \| Operation \| Region \| Object \| Frame \| Timestamp` |
| `AttributeValue` | Valor tipado de atributo (`Number \| Text \| Bool \| Integer`) |
| `Severity` | `Info \| Warning \| Error` |
| `ObservationId` | Newtype `u32` counter (decisión cerrada: NO UUID) |

`ObservationId` y los ids de artefacto (`MotionPlanId`, `ExecutionSessionId`,
`SemanticProgramId`, `RobotId`, `SceneId`) viven en `crate::ids` para que el
modelo nunca dependa de crates de planning/runtime (C1 — thalos-core es el
crate raíz).

## Contrato de capas (C1)

`analysis` no importa nada de planning, runtime, visual, api ni frontend.
Los demás crates dependen de `thalos-core`, nunca al revés. Cualquier análisis
nuevo debe emitir `Observation` y vivir fuera de core.

## Referencias

- Spec: `openspec/changes/analysis-model/specs/analysis-model/spec.md`
- Design: `openspec/changes/analysis-model/design.md` (decisiones D1, D5)
