# Contribuir — Thalos Engine

> Reglas para entregar cambios al engine. El entorno y los comandos están en
> [`docs/DEVELOPMENT.md`](docs/DEVELOPMENT.md); las fronteras, en
> [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Flujo

1. Crea una rama desde `main` con un nombre descriptivo (`api/…`, `fix/…`, `docs/…`).
2. Haz commits pequeños y de un solo propósito. Se usa estilo de mensaje
   convencional (`feat(api): …`, `fix(planning): …`, `docs: …`).
3. Abre un pull request contra `main`. La CI debe pasar antes de la revisión.
4. Un pull request no se fusiona con avisos de Clippy ni tests en rojo.

## Lo que la CI exige

`.github/workflows/ci.yml` corre, y debe pasar:

```bash
cargo build   --workspace
cargo test    --workspace
cargo clippy  --workspace --all-targets -- -D warnings
cargo doc     --workspace --no-deps
```

`cargo fmt --check` aún no es una puerta (el workspace no está rustfmt-clean).

## La regla de la frontera

El contrato de los consumidores es **`thalos-api`**. Al cambiar código:

- No introduzcas una dependencia nueva de crates internos hacia la aplicación
  (el engine nunca depende de Thalos Industrial).
- Un cambio que **amplía** la superficie pública debe actualizar, en el mismo
  pull request: `docs/API.md`, `thalos-api/tests/public_surface.rs` y los
  ejemplos afectados de `thalos-api/examples/`.
- Evita **romper** el Tier 1 (`Manipulator` y sus tipos de intención). Tier 1 está
  diseñado para evolucionar sin exponer el mecanismo; si un cambio lo rompe,
  justifícalo y documéntalo.
- Tier 3 (mecanismo) es público y puede cambiar, pero es una superficie que
  algunos consumidores usan a propósito: un cambio de forma o de alcance es una
  decisión, no un detalle. Regístrala como fila en `docs/MILESTONES.md`.

## Documentación como parte del cambio

No hay ADRs ni RFCs. Una decisión se registra como **una fila** en
[`docs/MILESTONES.md`](docs/MILESTONES.md); el razonamiento largo queda en el
historial de Git. Si el cambio mueve una frontera, actualiza
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md); si cambia lo que funciona hoy,
[`docs/STATUS.md`](docs/STATUS.md).

## Distribución

El engine se consume desde Thalos Industrial como **dependencia Git fijada a una
revisión** (hoy, por ruta hermana `../thalos-engine`). La publicación en crates.io
está **diferida**: los crates están marcados `publish = false` hasta que exista
una API estable y una política de versionado. Hasta entonces, consumir una
revisión concreta —no `main`— es lo que hace reproducible la integración.
