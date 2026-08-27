use crate::robot::error::RobotBuilderError;
use crate::robot::{segment::Segment, serial_chain::SerialChain};

use crate::spatial::frame::{Frame, FrameId, FrameRegistry};

pub struct SerialChainBuilder {
    segments: Vec<Segment>,
    frames: FrameRegistry,
    end_effector: Option<FrameId>,
}

impl SerialChainBuilder {
    pub fn new() -> Self {
        Self {
            segments: Vec::new(),
            frames: FrameRegistry::new(),
            end_effector: None,
        }
    }

    // Exponer acceso mutable al registry
    pub fn frames_mut(&mut self) -> &mut FrameRegistry {
        &mut self.frames
    }

    pub fn create_frame(&mut self, name: &str) -> FrameId {
        self.frames.create(name)
    }

    pub fn add_frame(&mut self, frame: Frame) {
        self.frames.register(frame);
    }

    pub fn add_segment(&mut self, segment: Segment) {
        self.segments.push(segment);
    }

    pub fn set_end_effector(&mut self, id: FrameId) {
        self.end_effector = Some(id);
    }

    pub fn build(self) -> Result<SerialChain, RobotBuilderError> {
        // Validar que todos los frames existan
        for segment in &self.segments {
            if segment.parent != FrameId::World {
                if !self.frames.contains(&segment.parent) {
                    return Err(RobotBuilderError::FrameNotFound(segment.parent));
                }
            }

            if !self.frames.contains(&segment.child) {
                return Err(RobotBuilderError::FrameNotFound(segment.child));
            }
        }

        let end_effector = self
            .end_effector
            .ok_or_else(|| RobotBuilderError::EndEffectorNotDefined)?;

        if !self.frames.contains(&end_effector) {
            return Err(RobotBuilderError::FrameNotFound(end_effector));
        }

        Ok(SerialChain {
            segments: self.segments,
            frames: self.frames,
            end_effector,
        })
    }
}
