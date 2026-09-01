#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionType {
    MoveJ,
    MoveL,
    /// Multi-segment motion program (compiled from several commands).
    Program,
}
