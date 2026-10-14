# Revisión de lanzamiento — Thalos Engine

> **Revisión congelada:** tag `v0.1.0` (apunta a la revisión exacta verificada en
> este documento).
>
> **Alcance:** preparar el consumo de Thalos Industrial como dependencia Git sin
> acoplarse a los internos del engine. **No** incluye fusionar crates, rediseñar
> `thalos-core` ni publicar en crates.io.
>
> **Resultado:** apto para consumo interno por Git, tras correcciones puntuales
> (ya aplicadas). Sin bloqueantes estructurales.

## Línea base verificada

Entorno: `rustc`/`cargo` 1.94.1 (stable), `clippy` + `rustfmt` instalados.

| Check | Resultado |
|---|---|
| `cargo check --workspace` | ✅ |
| `cargo test --workspace` (unit + integración + doc-tests) | ✅ 1384 passed / 0 failed / 4 ignored |
| `cargo build -p thalos-api --examples` | ✅ 8/8 |
| `cargo clippy --workspace --all-targets -- -D warnings` | ✅ exit 0 |
| `cargo doc --workspace --no-deps` | ✅ (17 warnings rustdoc) |
| `cargo fmt --all --check` | ~551 archivos con diferencias (deuda preexistente) |

## Prueba de frontera (decisiva)

Un crate **externo** que depende **solo** de `thalos-api` compila y ejecuta
`load → FK → IK → plan_motion → prepare_execution`. Los crates internos son
inalcanzables (`use thalos_core::…` / `use thalos_ports::…` → `E0433`).

**La frontera de crate es sólida.** El flujo de programa `.thls` sigue
orquestando el pipeline por etapas (gap D3-bis), documentado en `docs/API.md` §8.

## Correcciones aplicadas

| # | Corrección | Evidencia |
|---|---|---|
| **I1** | `cargo clippy --workspace --all-targets -- -D warnings` verde | `thalos-importer/src/normalize.rs` (`collapsible_if`); `thalos-lang/src/parser/mod.rs` (`clone_on_copy` + aliases de tipo); `thalos-language-service/src/{offset,intelligence,compiler}.rs` |
| **I3** | `test-support` deja de ser dependencia de producción | `thalos-planning/Cargo.toml` (movido a `[dev-dependencies]`); `thalos-runtime/Cargo.toml` (feature fuera del dep normal) |
| **I4** | El transporte de Industrial importa la fachada, no el crate interno | `thalos-transport` depende de `thalos-api`; `thalos_ports::` → `thalos_api::ports::` |

> **Conteo de Clippy corregido.** El diagnóstico inicial reportó 21 errores; el
> conteo real era **28 en 3 crates** (`thalos-lang`, `thalos-importer`,
> `thalos-language-service`). `cargo clippy` corta en el primer crate que falla y
> oculta los errores de los dependientes. Los 28 están resueltos.

## Deuda arquitectónica aplazada — I2 (`core → importer`)

| Campo | Valor |
|---|---|
| **Estado** | Aplazado |
| **Motivo** | Eliminar la dependencia de `thalos-core` sobre `thalos-importer` exige retirar la ruta pública `thalos_api::core::robot::adapter::from_urdf` y migrar sus consumidores (tests de `thalos-core`, tests de importación `ur5`/`scara`, y un test de `thalos-planning`) |
| **Consecuencia conocida** | `thalos-core` continúa arrastrando `quick-xml` de forma transitiva |
| **Criterio para reabrir** | Diseñar y validar una migración explícita de `from_urdf` a la capa API/importer, incluidos los consumidores y las pruebas afectadas |
| **Fuera del alcance actual** | No eliminar rutas públicas ni introducir cambios incompatibles en esta revisión |

Esto no afirma que la arquitectura actual sea ideal: afirma que el coste de
corregirla no está justificado dentro de este lanzamiento.

## Deuda documentada (no ampliada)

- **Formato:** `cargo fmt --check` no es una puerta de CI (el workspace no es
  rustfmt-clean). Los archivos tocados en esta revisión ya estaban sucios.
- **rustdoc:** 17 warnings (10 enlaces sin resolver, 6 targets redundantes, 1
  bloque no parseable).
- **H1/H3/H4:** `core::analysis` convive con el crate `thalos-analysis`;
  `core::models` (catálogo) con `thalos-models`; `core::prelude` es una superficie
  interna usada cross-crate. Sin problema de consumibilidad demostrado.
- **H7:** `unwrap`/`expect` en rutas de producción (core 233) — pendiente de
  revisión dirigida, no un conteo de defectos.
- **Docs congelados de lenguaje** (`thalos-industrial/docs/lenguaje/`) citan el
  crate histórico `thalos-semantic` (hoy `thalos-language-service`); no se
  reescriben por estar congelados por versión.

## Cómo reproducir

```bash
cd thalos-engine
cargo check --workspace
cargo test  --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo build -p thalos-api --examples
```
