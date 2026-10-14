# Desarrollo — Thalos Engine

> Cómo preparar el entorno, construir, probar y modificar el engine. Para las
> fronteras y responsabilidades de cada crate, ver [`ARCHITECTURE.md`](ARCHITECTURE.md);
> para el contrato público, [`API.md`](API.md).

## Requisitos

- **Rust** con soporte de **edition 2024** (Rust 1.85 o superior). No hay
  `rust-toolchain` fijado: se usa el toolchain estable.
- No se necesita Node, bases de datos ni hardware: el engine es una librería pura.

## Preparar y verificar

```bash
cargo build   --workspace
cargo test    --workspace
cargo clippy  --workspace --all-targets -- -D warnings
cargo doc     --workspace --no-deps
```

Estos cuatro comandos son **exactamente** lo que corre la CI
(`.github/workflows/ci.yml`). `cargo fmt --check` **no** es todavía una puerta:
el workspace no está rustfmt-clean; puedes formatear localmente, pero la CI no lo
exige.

Los ejemplos ejecutables del contrato público se comprueban aparte:

```bash
cargo check -p thalos-api --examples
cargo run   -p thalos-api --example move_j     # cualquier escenario de docs/API.md §5
```

## Estructura del workspace

13 crates en `resolver = "3"`, todos con `version = "0.1.0"` heredada del
workspace. La frontera pública es **`thalos-api`**; el resto son internos. La
tabla de responsabilidades y las reglas de dependencia están en
[`ARCHITECTURE.md`](ARCHITECTURE.md) — no se repiten aquí.

`Cargo.lock` **no** se versiona (es librería); `/target` está ignorado.

## Dónde vive cada tipo de prueba

- **Contract tests de la frontera** — `thalos-api/tests/`:
  - `public_surface.rs`: todo camino público debe resolverse sin nombrar un crate
    interno. Si un concepto solo es alcanzable por un crate interno, este test
    deja de compilar y la violación aparece en el punto del cambio.
  - `dependency_purity.rs`: `thalos-api` no depende de la capa de aplicación.
- **Tests unitarios** — junto al código, en `#[cfg(test)] mod tests` dentro de
  cada crate.
- **Fixtures** — `tests/fixtures/` (p. ej. `basic_motion.thls`) y, para modelos,
  URDF de referencia en `thalos-models/tests/fixtures/`.
- **Feature `test-support`** — dobles de test (transports falsos, fixtures
  semánticos) a través de la frontera. Está **desactivada** por defecto para que
  los consumidores de producción no arrastren código de test. Actívala con
  `--features test-support` cuando pruebes a través de la frontera.

## Añadir o modificar una intención (Tier 1)

El patrón de la API pública es: **el consumidor expresa una intención; el engine
guarda el mecanismo**. Al añadir una capacidad:

1. Verifica que responde a *"¿qué conocimiento elimina del consumidor?"*. Si solo
   renombra una llamada a Tier 3, no es Tier 1.
2. Implementa el tipo de intención dentro de `thalos-api/src/intent/` (no en un
   crate aparte): la frontera es donde vive el consumidor.
3. Mantén Tier 1 **propio** de Tier 3: los tipos de intención no son re-exports de
   los tipos de mecanismo, para que el mecanismo evolucione sin romper el
   contrato.
4. Si cambia la superficie pública, actualiza en el mismo cambio:
   `docs/API.md`, `thalos-api/tests/public_surface.rs` y, si aplica, los
   ejemplos de `thalos-api/examples/`.
5. Añade un ejemplo ejecutable si la capacidad es de las principales: el ejemplo
   es el gate de compilación del contrato.

## Modificar un crate interno (Tier 3)

Tier 3 es público para quien construye encima, así que sus cambios no son libres
por definición: si un tipo Tier 3 deja de ser alcanzable o cambia de forma, puede
romper a consumidores que bajaron a ese nivel deliberadamente. Registra el cambio
como una fila en [`MILESTONES.md`](MILESTONES.md) si mueve una frontera.

## Depurar

- `cargo run -p thalos-api --example <escenario>` reproduce un flujo de consumidor
  externo de extremo a extremo.
- `cargo test -p <crate> <filtro>` acota al crate y test que te interesa.
- `cargo clippy` con `-D warnings` es la definición de "sin avisos" del proyecto.
