//! Timeline — logical → temporal event transformation (IR-3).

pub mod scheduler;

#[cfg(test)]
mod scheduler_tests;

pub use scheduler::TimelineScheduler;
