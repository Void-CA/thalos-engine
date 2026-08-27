mod dynamic_matrix;
mod dynamic_vector;

pub use dynamic_matrix::DynamicMatrix;
pub use dynamic_vector::DynamicVector;

/// Convierte un [`crate::Vector3`] en un [`DynamicVector`] de 3 elementos.
pub fn vector_to_dynamic(v: crate::Vector3) -> DynamicVector {
    DynamicVector::from_vec(vec![v.x, v.y, v.z])
}
