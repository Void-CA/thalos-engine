//! Algoritmos de detección de colisiones para Thalos.
//!
//! Este crate implementa los detectores concretos que cumplen el contrato
//! definido en [`thalos_core::collision::CollisionChecker`].
//!
//! # Organización
//!
//! - [`naive`] — `NaiveCollisionChecker`: O(n²), sin optimizaciones.
//! - [`intersect`] — Primitivas de intersección geométrica (SAT, esferas, cajas).
//! - [`classify`] — Clasificación semántica del tipo de colisión.
//! - [`distance`] — Queries de distancia mínima entre geometrías.

pub mod classify;
pub mod distance;
pub mod intersect;
pub mod naive;

pub use naive::NaiveCollisionChecker;
