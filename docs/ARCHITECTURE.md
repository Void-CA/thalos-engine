# Arquitectura — Thalos Engine

> **Carácter:** normativo. Define qué posee el engine y qué tiene prohibido. La
> pureza de la frontera está verificada por tests.

## Qué es el engine

Una librería pura y autónoma. No conoce:

- Tauri / IPC
- React / UI
- SQLite / persistencia
- TCP / serial / MQTT / ESP32
- Thalos Industrial (la aplicación)

Una única frontera pública: `thalos-api` (fachada curada por concepto). Un
consumidor importa de ahí, nunca de los crates internos.

## Qué posee

- **Dominio** — entidades, IR de operación y programa, contratos de capacidad.
- **Matemática y cinemática** — FK/IK, transformaciones, álgebra.
- **Planificación** — trayectorias, interpolación, `ExecutionPlan`.
- **Lenguaje** — AST, parser, unidades, pipeline semántico (`.thls`).
- **Semántica de supervisión** — significados de una ejecución: máquina de estados
  de sesión, reglas de transición, contrato de tick, vocabulario de eventos.
- **Semántica de evidencia** — qué representa una traza y cómo se transforma. **No**
  cómo se adquiere ni dónde se persiste.

## Qué NO posee

El **mecanismo**. El loop Tokio, el reloj concreto, el registry de sesiones, el
event bus y la persistencia viven en la aplicación (`thalos-runtime`). Que el
engine defina el significado de una sesión **no** significa que deba ejecutarla.

## Reglas (verificadas por CI)

| Regla | Guarda |
|---|---|
| Un consumidor importa solo `thalos-api` | `thalos-api/tests/dependency_purity.rs` |
| La superficie pública es exactamente la declarada | `thalos-api/tests/public_surface.rs` |
| Los crates de dominio puro no dependen de infraestructura | `dependency_purity.rs` |
| La aplicación no vuelve a exportar conceptos del engine | `thalos-runtime/tests/no_engine_reexports.rs` (repo Industrial) |

### Excepción documentada

El I/O concreto (`thalos-transport`: serial, TCP, ESP32) cruza la frontera de
`thalos-api` porque la aplicación necesita adaptadores. Sigue siendo I/O de
infraestructura, no dominio.

## Retroalimentación con la aplicación

```text
Thalos Industrial ──► thalos-api ──► crates internos
       ▲                                    │
       └── nunca (el engine no conoce la aplicación) ──┘
```

Si el engine necesita un dato de la aplicación (reloj, identidad, estado vivo),
lo recibe como **parámetro de dominio**, no como dependencia.
