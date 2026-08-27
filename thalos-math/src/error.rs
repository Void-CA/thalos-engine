use thiserror::Error;

#[derive(Error, Debug)]
pub enum MathError {
    #[error("Cannot normalize a zero vector")]
    ZeroVectorNormalization,

    #[error("Cannot normalize a zero quaternion")]
    ZeroQuaternionNormalization,

    #[error("Cannot invert a quaternion with near-zero norm (norm² = {norm_sq})")]
    ZeroQuaternionInverse { norm_sq: f64 },

    #[error("Quaternion is not unit: norm² = {norm_sq}, expected ≈ 1")]
    QuaternionNotUnit { norm_sq: f64 },
}
