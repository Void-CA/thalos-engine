//! Request occurrence tokens: `request ... as job` + `on job.<event>`.
//!
//! These tests exercise the engine-side contract only: compilation, token
//! validation, FIFO outcome delivery, non-terminal `Busy`, and correlation by
//! occurrence. The Industrial bench exercises the full runtime path.

use thalos_core::device::{ChannelObservation, ChannelValue, SignalQuality};
use thalos_core::execution::session::ObservationBundle;
use thalos_lang::parse_source;
use thalos_language_service::reactive::{SignalSchema, SignalType};
use thalos_language_service::reactive_eval::RuntimeValue;
use thalos_language_service::reactive_process::{
    Effect, PEvent, ProcessProgram, ProcessRuntime, RejectionReason, TokenDeliveryError,
    TokenEvent, TokenOutcome, compile_process,
};

fn schema() -> SignalSchema {
    SignalSchema::new()
        .with("box_sensor", "present", SignalType::Bool)
        .with("box_sensor", "id", SignalType::Int)
}

fn obs(present: bool, id: f64) -> ObservationBundle {
    let mut bundle = ObservationBundle::default();
    for (sequence, (channel, value)) in [
        ("present", ChannelValue::Boolean(present)),
        ("id", ChannelValue::Scalar(id)),
    ]
    .into_iter()
    .enumerate()
    {
        bundle.observations.insert(
            channel.to_string(),
            ChannelObservation {
                channel_id: channel.to_string(),
                sequence: sequence as u64,
                sampled_at_ns: 0,
                received_at_ns: 0,
                value,
                unit: None,
                quality: SignalQuality::Nominal,
            },
        );
    }
    bundle
}

fn compile(src: &str) -> ProcessProgram {
    compile_process(&parse_source(src).expect("parse"), &schema()).expect("compile")
}

fn request(effects: &[Effect]) -> Option<(&Option<String>, u64, &str)> {
    effects.iter().find_map(|effect| match effect {
        Effect::Request {
            token,
            occurrence,
            work,
            ..
        } => Some((token, *occurrence, work.as_str())),
        _ => None,
    })
}

fn busy(effects: &[Effect]) -> Option<(&str, u64)> {
    effects.iter().find_map(|effect| match effect {
        Effect::Busy { token, occurrence } => Some((token.as_str(), *occurrence)),
        _ => None,
    })
}

const TOKEN_SRC: &str = r#"
use station::{ box_sensor as box }

work pick_box(box_id: Int) { }

process palletizing {
    state phase: Int = 0
    on rising(box.present) {
        request pick_box(box_id = box.id) as job
    }
    on job.admitted { phase = 1 }
    on job.rejected { phase = 2 }
    on job.busy { phase = 3 }
    on job.completed { phase = 4 }
    on job.failed { phase = 5 }
    on job.unconfirmed { phase = 6 }
}

fn main() { start palletizing }
"#;

#[test]
fn token_program_compiles_and_resolves_events() {
    let program = compile(TOKEN_SRC);
    assert_eq!(program.tokens, vec!["job".to_string()]);
    assert!(program.reactions.iter().any(|reaction| matches!(
        &reaction.event,
        PEvent::Token { token, event } if token == "job" && *event == TokenEvent::Completed
    )));
}

#[test]
fn token_validation_rejects_collisions_and_invalid_events() {
    let collision = TOKEN_SRC.replace("state phase: Int = 0", "state job: Int = 0");
    let errors = compile_process(&parse_source(&collision).unwrap(), &schema()).unwrap_err();
    assert!(
        errors.iter().any(|e| e.contains("collides with a state")),
        "{errors:?}"
    );

    let invalid = TOKEN_SRC.replace("on job.completed", "on job.nope");
    let errors = compile_process(&parse_source(&invalid).unwrap(), &schema()).unwrap_err();
    assert!(
        errors.iter().any(|e| e.contains("has no event 'nope'")),
        "{errors:?}"
    );

    let in_work = format!("{TOKEN_SRC}\nwork other() {{ request pick_box(box_id = 1) as late }}");
    let errors = compile_process(&parse_source(&in_work).unwrap(), &schema()).unwrap_err();
    assert!(
        errors
            .iter()
            .any(|e| e.contains("only allowed in `on`/`cycle`")),
        "{errors:?}"
    );
}

#[test]
fn admitted_does_not_free_and_busy_is_non_terminal() {
    let mut runtime = ProcessRuntime::new(compile(TOKEN_SRC));

    assert!(runtime.tick(&obs(false, 7.0)).is_empty());
    let first = runtime.tick(&obs(true, 7.0));
    let (token, occurrence, work) = request(&first).expect("first request");
    assert_eq!(token.as_deref(), Some("job"));
    assert_eq!(work, "pick_box");
    assert_eq!(occurrence, 0);

    runtime
        .deliver_outcome("job", occurrence, TokenOutcome::Admitted)
        .unwrap();
    let admitted = runtime.tick(&obs(true, 7.0));
    assert_eq!(runtime.state("phase"), Some(&RuntimeValue::Int(1)));
    assert!(request(&admitted).is_none());

    // The admitted occurrence still owns the token: a new edge is rejected.
    assert!(runtime.tick(&obs(false, 7.0)).is_empty());
    let second = runtime.tick(&obs(true, 7.0));
    assert_eq!(busy(&second), Some(("job", 1)));

    // Busy is a non-terminal event and runs its reaction on the next tick.
    let busy_tick = runtime.tick(&obs(true, 7.0));
    assert!(request(&busy_tick).is_none());
    assert!(busy(&busy_tick).is_none());
    assert_eq!(runtime.state("phase"), Some(&RuntimeValue::Int(3)));

    // The original occurrence is still the current one.
    let delivered = runtime.deliver_outcome("job", 1, TokenOutcome::Completed);
    assert!(matches!(
        delivered,
        Err(TokenDeliveryError::StaleOccurrence { .. })
    ));
}

#[test]
fn completed_frees_and_allows_reuse_in_the_same_tick() {
    let src = TOKEN_SRC.replace(
        "on job.completed { phase = 4 }",
        "on job.completed { phase = 4\n        request pick_box(box_id = box.id) as job }",
    );
    let mut runtime = ProcessRuntime::new(compile(&src));

    runtime.tick(&obs(false, 7.0));
    let first = runtime.tick(&obs(true, 7.0));
    let (_, occurrence, _) = request(&first).expect("first request");

    runtime
        .deliver_outcome("job", occurrence, TokenOutcome::Completed)
        .unwrap();
    let after = runtime.tick(&obs(true, 7.0));
    assert_eq!(runtime.state("phase"), Some(&RuntimeValue::Int(4)));
    let (token, next_occurrence, _) = request(&after).expect("reused token");
    assert_eq!(token.as_deref(), Some("job"));
    assert_eq!(next_occurrence, occurrence + 1);
}

#[test]
fn terminal_outcomes_free_and_stale_outcomes_are_rejected() {
    for outcome in [
        TokenOutcome::Rejected {
            reason: RejectionReason::Capacity,
        },
        TokenOutcome::Failed {
            reason: "planning failed".to_string(),
        },
        TokenOutcome::Unconfirmed,
    ] {
        let mut runtime = ProcessRuntime::new(compile(TOKEN_SRC));
        runtime.tick(&obs(false, 7.0));
        let first = runtime.tick(&obs(true, 7.0));
        let (_, occurrence, _) = request(&first).expect("first request");

        runtime.deliver_outcome("job", occurrence, outcome).unwrap();
        runtime.tick(&obs(true, 7.0));
        assert!(
            !matches!(runtime.state("phase"), Some(&RuntimeValue::Int(1))),
            "a terminal outcome must not look admitted"
        );

        // The token is free: a new edge starts a new occurrence.
        runtime.tick(&obs(false, 7.0));
        let next = runtime.tick(&obs(true, 7.0));
        assert_eq!(request(&next).map(|(_, o, _)| o), Some(occurrence + 1));

        // A late outcome for the old occurrence is rejected, not applied.
        let stale = runtime.deliver_outcome("job", occurrence, TokenOutcome::Completed);
        assert!(matches!(
            stale,
            Err(TokenDeliveryError::StaleOccurrence { .. })
        ));
    }
}

#[test]
fn outcomes_are_applied_fifo() {
    let mut runtime = ProcessRuntime::new(compile(TOKEN_SRC));
    runtime.tick(&obs(false, 7.0));
    let first = runtime.tick(&obs(true, 7.0));
    let (_, occurrence, _) = request(&first).expect("first request");

    runtime
        .deliver_outcome("job", occurrence, TokenOutcome::Admitted)
        .unwrap();
    runtime
        .deliver_outcome("job", occurrence, TokenOutcome::Completed)
        .unwrap();
    runtime.tick(&obs(true, 7.0));

    // Admitted reaction ran first, then Completed reaction.
    assert_eq!(runtime.state("phase"), Some(&RuntimeValue::Int(4)));
}

#[test]
fn unconfirmed_is_defined_but_not_produced() {
    let program = compile(TOKEN_SRC);
    let mut runtime = ProcessRuntime::new(program);

    runtime.tick(&obs(false, 7.0));
    let first = runtime.tick(&obs(true, 7.0));
    let (_, occurrence, _) = request(&first).expect("first request");

    // No outcome delivery means no token event at all.
    let quiet = runtime.tick(&obs(true, 7.0));
    assert!(quiet.is_empty());
    assert_eq!(runtime.state("phase"), Some(&RuntimeValue::Int(0)));

    // The contract accepts it when a future evidence source produces it.
    runtime
        .deliver_outcome("job", occurrence, TokenOutcome::Unconfirmed)
        .unwrap();
    runtime.tick(&obs(true, 7.0));
    assert_eq!(runtime.state("phase"), Some(&RuntimeValue::Int(6)));
}
