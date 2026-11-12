//! # thalos-api
//!
//! Public boundary of **Thalos Engine**.
//!
//! This crate is a deliberately thin, *concept-curated* facade. It re-exports the
//! stable public surface of the engine so consumers (e.g. Thalos Industrial) do
//! not depend on the internal crate graph.
//!
//! ## Design rules
//!
//! - The namespace reflects **concepts**, not the internal filesystem layout.
//! - The boundary is **curated**, not an alias of whole crates. Internal modules
//!   that are not ready to be public (e.g. legacy/orphan types) are intentionally
//!   omitted; slimming the internal crates later must not break this surface.
//! - `thalos-api` **must not** depend on `thalos-runtime`: the application layer
//!   depends on the API, never the other way around.
//!
//! ## Target surface
//!
//! The engine owns domain + supervision **semantics** (not its
//! execution mechanism). The following public namespaces are planned:
//!
//! - `execution::supervision` — execution semantics and supervision contracts
//! - `telemetry` — execution evidence and observation semantics (**materialized**, F2.1b)
//! - `robot::controller` — robot controller contract and capabilities
//!
//! The remaining namespaces are intentionally **not materialized** until the
//! corresponding ownership move introduces their first concrete public items.
//! Each is added in the same change that brings the concepts across the boundary,
//! so every public path here has real semantic content behind it.

/// Curated core domain surface.
///
/// Only the fundamental domain modules are public here. Internal/stepping-stone
/// modules (`analysis`, `models` catalog, and the legacy inventory types)
/// are exposed through their own namespaces below or kept private.
///
/// Note: `core::station::Station` (engine, resource-binding) is a different
/// concept from `thalos_runtime::station::Station` (application, operational).
pub mod core {
    pub use thalos_core::{
        capability, collision, command, device, deviation, execution, ids, kinematics, motion,
        operation, program, resource, robot, skill, spatial, station, trajectory,
    };
}

/// Analysis layer: workspace/singularity/manipulability analyzers, the
/// canonical observation language, and the reusable analysis services.
pub mod analysis {
    pub use thalos_core::analysis::*;
    pub use thalos_analysis::{
        AlignedPair, Alignment, ComparePipeline, ComparePipelineError, ComparePipelineOutput,
        ComparisonMetrics, ConditionOutcome, Deviation, JointErrorMetrics, KinematicViolation,
        ManipulabilityService, PlanExecutionComparison, SignalComparison, SignalViolation,
        SingularityService, TickComparison, TickDeviation, TracePoint, WorkspaceService,
        compare_points, compare_tick, detect_deviations,
    };
}

/// Robot models: structural / URDF data types.
pub mod models {
    pub use thalos_models::*;
}

/// Built-in kinematic model catalog (`RobotModel`, `RobotRegistry`, …).
///
/// Distinct from [`models`]: this is the engine's preset kinematic catalog,
/// not the structural robot description.
pub mod catalog {
    pub use thalos_core::models::*;
}

/// Fundamental math types (`Vector3`, `Transform3D`, `Quaternion`, …).
pub mod math {
    pub use thalos_math::*;
}

/// Motion planning and interpolation (Tier 3 mechanism).
pub mod planning {
    pub use thalos_planning::*;
}

/// Intention-oriented API (Tier 1): the consumer expresses *what* it wants
/// without assembling the mechanism. See `docs/API.md`.
///
/// Unlike the namespaces above, this one is **implemented here**, not
/// re-exported: the public boundary is where the consumer lives, so the
/// Tier 1 / Tier 2 / Tier 3 separation is expressed inside `thalos-api` itself.
pub mod intent;

/// Raw Thalos language front-end (AST and parser).
pub mod lang {
    pub use thalos_lang::ast;
    pub use thalos_lang::ast::{
        Arg, BinaryOp, CompareOp, ConstDecl, CycleDecl, Event, ExpectDecl, Expr, FnDecl, Item,
        OnDecl, Param, ProcessDecl, ProcessItem, Program, Spanned, SpannedArg, SpannedConstDecl,
        SpannedExpr, SpannedExprKind, SpannedFnDecl, SpannedItem, SpannedProgram, SpannedStatement,
        SpannedStatementKind, SpannedTargetDecl, StateDecl, Statement, TargetDecl, TypeArg,
        TypeExpr, UnaryOp, UseDecl, UseTree, WorkDecl,
    };
    pub use thalos_lang::parser::{parse_source, parse_source_spanned};
    pub use thalos_lang::span::Span;
    pub use thalos_lang::units::{AngleRadians, DurationSeconds, LengthMeters};
}

/// Semantic language service (diagnostics, symbols, compilation pipeline).
pub mod semantic {
    pub use thalos_language_service::{
        analyze_document, char_range_to_byte_span, Diagnostic, DiagnosticSeverity,
        DocumentAnalysis, ProvenanceEntry, ProvenanceKind, SourceSpan, Symbol, SymbolKind,
    };

    pub mod compiler {
        pub use thalos_language_service::compiler::{
            CompileReport, CompilerWarning, SemanticCompiler, ValidatedProgram,
        };
    }
    pub mod evaluator {
        pub use thalos_language_service::evaluator::{
            CompileTimeValue, EvalResult, Evaluator, Pose, Position, negate_value, read_member,
        };
    }
    pub mod intelligence {
        pub use thalos_language_service::intelligence::{
            DocumentIntelligence, HintOrigin, InlayHint, SemanticTokenFact, SemanticTokenKind,
            SignatureInfo, TypedExpression, TypedSymbol, analyze_intelligence,
        };
    }
    pub mod model {
        pub use thalos_language_service::model::{
            CallSite, JointConfiguration, MotionKind, MotionTarget, Provenance, ResolvedMotion,
            ResolvedProgram, ResolvedStatement, ResolvedTarget, SemanticArg, SemanticExpr,
            SemanticFunction, SemanticMotion, SemanticProgram, SemanticStatement,
        };
    }
    pub mod reactive {
        pub use thalos_language_service::reactive::{
            ReactiveExpr, ReactiveProgram, ReactiveStatement, SignalBinding, SignalRef, SignalSchema,
            SignalType, lower_reactive,
        };
    }
    pub mod reactive_eval {
        pub use thalos_language_service::reactive_eval::{
            ReactiveEvaluator, ReactiveStep, RuntimeValue,
        };
    }
    pub mod reactive_process {
        pub use thalos_language_service::reactive_process::{
            Effect, PEvent, PExpr, PReaction, PState, PStmt, PWork, ProcessProgram, ProcessRuntime,
            RejectionReason, SignalAlias, TokenDeliveryError, TokenEvent, TokenOutcome,
            compile_process, specialize_work,
        };
    }
    pub mod resolver {
        pub use thalos_language_service::resolver::SemanticResolver;
    }
    pub mod types {
        pub use thalos_language_service::types::{
            Dimension, FunctionType, Type, TypeAnnotationError, resolve_type,
        };
    }
}

/// URDF / asset importing.
pub mod importer {
    pub use thalos_importer::*;
}

/// IO and transport contracts.
pub mod ports {
    pub use thalos_ports::*;
}

/// Document representations (programs, scenes, resources).
pub mod document {
    pub use thalos_document::*;
}

/// Visual scene representation (frames, links, primitives, trajectories).
pub mod visual {
    pub use thalos_visual::*;
}

/// Execution evidence and observation semantics (traces, samples, analyzer,
/// lifecycle events). Implemented by [`thalos_telemetry`]; the recording
/// mechanism and the live state stay in the application.
pub mod telemetry {
    pub use thalos_telemetry::*;
}
