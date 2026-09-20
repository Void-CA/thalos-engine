# Documentación — Thalos Engine

Este directorio contiene **dirección**, no historial. Son pocos documentos,
estables y vivos.

## La regla

> **Un documento se mantiene si y solo si un recién llegado lo necesita para
> operar el engine hoy.**

Si un documento exige inspeccionar todo el código para mantenerse correcto, no
debe existir como documento: vive en el código y en los tests.

## Los documentos vivos

| Documento | Pregunta que responde | Se actualiza |
|---|---|---|
| [`../README.md`](../README.md) | ¿Qué es esto y cómo se usa? | rara vez |
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | ¿Qué posee el engine y qué está prohibido? | al mover una frontera |
| [`STATUS.md`](STATUS.md) | ¿Qué funciona **hoy**? | al cerrar una fase |
| [`MILESTONES.md`](MILESTONES.md) | ¿Qué se decidió y sigue vigente? | al cerrar una fase |

## Cómo se registra una decisión

Igual que en Thalos Industrial: **una fila** en [`MILESTONES.md`](MILESTONES.md),
no un archivo nuevo. El razonamiento de fondo vive en el historial de Git.

## Por qué no hay ADRs

Los cinco ADRs (002, 015, 016, 017, 018) se eliminaron el 2026-09-20 porque:

- **No eran verificables.** Decían `Accepted` sin que nada comprobara que el
  código seguía cumpliéndolos.
- **Su contenido útil cabía en una fila.** Las cuatro decisiones de frontera que
  describían están en [`ARCHITECTURE.md`](ARCHITECTURE.md) y verificadas por tests.
- **La numeración era un fantasma.** Faltaban 001 y 003–014, y el engine citaba
  números de ADR de Thalos Industrial como si fueran propios.

Las referencias `ADR-xxx` que quedaban en los comentarios del código se
sustituyeron por nombres de contrato (`provider contract`, `safety-envelope
authority`, `Z-up`, …), que es lo que el comentario quería decir.
