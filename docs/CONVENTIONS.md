# Convenciones de código — Thalos Engine

> **Estado:** vigente · **Alcance:** crates del workspace del engine
> **Criterio rector:** hacer verificables las garantías del dominio y de la
> frontera pública, no mejorar la apariencia del repositorio.

## 1. Tamaño y ratchet

| Situación | Regla |
|---|---|
| Archivo de producción nuevo | ≤300 líneas |
| Archivo de producción existente fuera de baseline | ≤300 líneas |
| Archivo de producción en baseline | No puede crecer; debe disminuir hasta salir |
| Archivo de test (`tests/`, `examples/`, `#[cfg(test)]`) | ≤400 líneas |
| Baseline | Solo decrece; una entrada dentro del límite debe eliminarse |

- Producción = líneas anteriores a `#[cfg(test)]`. El bloque de tests se mide
  aparte y tiene su propio límite.
- Guard: `thalos-api/tests/file_size_guard.rs`.
- 300/400 señalan responsabilidades concentradas; no autorizan a repartir
  funciones sin cambiar el diseño.

## 2. Crates y directorios

- `thalos-api` es la única frontera pública. Los demás crates son internos
  (`publish = false`) y no forman parte del contrato de consumidores.
- `thalos-core` concentra hoy dominio, cinemática, modelos y análisis; cualquier
  extracción debe ser una decisión arquitectónica con su fila en
  `docs/MILESTONES.md`, no un movimiento mecánico.
- Un crate nuevo requiere una responsabilidad nombrable y una regla de
  dependencia explícita. Evitar nombres que colisionen con conceptos existentes
  (`thalos-models` vs `core::models`; `capability` vs `robot::capability`).
- Código nuevo usa `foo.rs` + `foo/`; no se añaden `mod.rs` nuevos.
- Tests unitarios junto al módulo; integración en `tests/`. Un directorio
  `src/**/tests/` no es producción y se mide como test.

## 3. Nombres

- `Dto` (no `DTO`), `Id` (no `ID`), `Uuid` (no `UUID`), `Ik*` (no `IK*`).
- Tipos en singular; módulos y archivos en `snake_case`.
- Un nombre, un significado: si dos conceptos comparten término, calificarlo.
- Comentarios y nombres de código en inglés; documentación de producto en
  español. No mezclar idiomas en un mismo archivo.

## 4. Visibilidad y frontera

- `thalos-api` usa re-exports explícitos. Prohibido `pub use crate::...::*` para
  módulos o crates completos.
- Por defecto, lo interno es `pub(crate)`. Un `pub` nuevo en un crate interno
  debe justificarse por su uso desde `thalos-api` o un consumidor declarado.
- Cambiar la superficie pública exige actualizar `docs/API.md`,
  `thalos-api/tests/public_surface.rs` y los ejemplos afectados.
- Industrial consume el engine solo por `thalos-api` y con `rev` fijado. Un
  cambio del engine necesita commit upstream y bump de `rev` en Industrial.

## 5. Dependencias

- Dirección: `thalos-api → crates internos`; nunca al revés. El engine no
  depende de Industrial.
- Evitar dependencias transitivas de capas inferiores a superiores
  (`thalos-core → thalos-importer` es deuda conocida y no se amplía).
- Una interfaz no duplica un contrato existente; si dos capas necesitan la misma
  forma, se define una vez y se proyecta en la frontera.
- Toda arista nueva entre crates debe estar justificada y reflejada en
  `docs/ARCHITECTURE.md`.

## 6. Cierre de cada fase

1. Comandos ejecutados y su resultado.
2. Tests que fijan la garantía, no solo el camino feliz.
3. Commits de un solo propósito; no mezclar renombrados masivos con refactors.
4. Fila en `docs/MILESTONES.md` si cambia una frontera, una garantía o el
   tamaño de la superficie pública.
