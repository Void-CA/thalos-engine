use std::fs;
use std::path::Path;
use serde::Deserialize;
use thiserror::Error;

use thalos_core::ids::{RobotId, SkillId};
use thalos_core::robot::capability::RobotCapability;
use thalos_core::robot::definition::RobotDefinition;
use thalos_core::robot::profile::{RobotProfile, SkillBinding, SkillBindingSource};
use thalos_core::skill::{RobotSkill, SkillCapability, SkillImplementation, SkillRegistry};

use crate::script::parse_fragment;

/// Errors that can occur during `RobotProfile` loading and skill materialization.
#[derive(Debug, Error)]
pub enum ProfileLoadError {
    #[error("I/O error reading profile at {path}: {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("TOML parsing error in {path}: {source}")]
    Toml {
        path: String,
        #[source]
        source: toml::de::Error,
    },
    #[error("Skill script parse error in {path}: {message}")]
    ScriptParse { path: String, message: String },
}

#[derive(Debug, Deserialize)]
struct TomlRobotProfile {
    id: String,
    name: String,
    capabilities: Vec<String>,
    #[serde(default)]
    skills: Vec<TomlSkillBinding>,
}

#[derive(Debug, Deserialize)]
struct TomlSkillBinding {
    skill: String,
    source_type: String,
    path: Option<String>,
    native_id: Option<String>,
    policy: Option<String>,
}

/// Loader for materializing `RobotProfile` and `SkillRegistry` from disk.
pub struct RobotProfileLoader;

impl RobotProfileLoader {
    /// Load a `RobotProfile` from a directory containing `robot.toml`.
    pub fn load_from_dir(dir_path: impl AsRef<Path>) -> Result<RobotProfile, ProfileLoadError> {
        let dir = dir_path.as_ref();
        let config_path = dir.join("robot.toml");
        let content = fs::read_to_string(&config_path).map_err(|e| ProfileLoadError::Io {
            path: config_path.display().to_string(),
            source: e,
        })?;

        let toml_prof: TomlRobotProfile =
            toml::from_str(&content).map_err(|e| ProfileLoadError::Toml {
                path: config_path.display().to_string(),
                source: e,
            })?;

        let skill_capabilities = toml_prof
            .capabilities
            .into_iter()
            .map(|c| SkillCapability::new(SkillId(c)))
            .collect();

        let definition = RobotDefinition::new(
            RobotId(toml_prof.id.clone()),
            toml_prof.name,
            None,
            vec![],
            RobotCapability::default(),
            skill_capabilities,
        );

        let skill_bindings = toml_prof
            .skills
            .into_iter()
            .map(|s| {
                let binding_source = match s.source_type.as_str() {
                    "script" => SkillBindingSource::Script {
                        path: s.path.unwrap_or_default(),
                    },
                    "native" => SkillBindingSource::Native {
                        native_id: s.native_id.unwrap_or_default(),
                    },
                    "planner" => SkillBindingSource::Planner {
                        policy: s.policy.unwrap_or_default(),
                    },
                    _ => SkillBindingSource::Script {
                        path: s.path.unwrap_or_default(),
                    },
                };
                SkillBinding::new(SkillId(s.skill), binding_source)
            })
            .collect();

        Ok(RobotProfile::new(
            RobotId(toml_prof.id),
            definition,
            skill_bindings,
        ))
    }

    /// Materialize a `SkillRegistry` from a loaded `RobotProfile` and base directory.
    pub fn materialize_skills(
        profile: &RobotProfile,
        base_dir: impl AsRef<Path>,
    ) -> Result<SkillRegistry, ProfileLoadError> {
        let mut registry = SkillRegistry::new();
        let base = base_dir.as_ref();

        for binding in &profile.skill_bindings {
            let impl_strategy = match &binding.source {
                SkillBindingSource::Script { path } => {
                    let script_path = base.join(path);
                    let script_content =
                        fs::read_to_string(&script_path).map_err(|e| ProfileLoadError::Io {
                            path: script_path.display().to_string(),
                            source: e,
                        })?;

                    let fragment = parse_fragment(&script_content).map_err(|errs| {
                        let msg = errs
                            .iter()
                            .map(|e| e.to_string())
                            .collect::<Vec<_>>()
                            .join("; ");
                        ProfileLoadError::ScriptParse {
                            path: script_path.display().to_string(),
                            message: msg,
                        }
                    })?;

                    SkillImplementation::Program(fragment)
                }
                SkillBindingSource::Native { native_id } => {
                    SkillImplementation::Native(thalos_core::skill::NativeSkillId(native_id.clone()))
                }
                SkillBindingSource::Planner { policy } => {
                    SkillImplementation::Planner(thalos_core::skill::SkillPlanner {
                        policy: policy.clone(),
                    })
                }
            };

            let skill = RobotSkill::new(
                binding.skill.clone(),
                binding.skill.as_str().to_string(),
                vec![],
                vec![],
                vec![],
                impl_strategy,
            );

            registry.register_for_robot(profile.id.clone(), skill);
        }

        Ok(registry)
    }
}
