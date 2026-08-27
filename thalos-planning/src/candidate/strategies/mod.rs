//! Bounded strategy library (PR1, Phase 2): the MVP generating strategies.
//!
//! Each strategy produces at most one candidate per invocation and records a
//! documented no-candidate reason when it cannot (spec candidate-generation
//! "Bounded Strategy Library").

pub mod alternate_elbow;
pub mod insert_waypoint;

pub use alternate_elbow::AlternateElbow;
pub use insert_waypoint::InsertWaypoint;
