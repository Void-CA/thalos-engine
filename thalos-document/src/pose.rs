use serde::{Deserialize, Serialize};

/// Lightweight serializable pose — position `[f64; 3]` + orientation `[f64; 4]`.
///
/// No dependency on `thalos-math`; designed as a document-format type.
/// Orientation is stored as `[x, y, z, w]` (quaternion).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json;

    // --- Construction ---

    #[test]
    fn pose_construction() {
        let p = Pose {
            position: [1.0, 2.0, 3.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        };
        assert_eq!(p.position[0], 1.0);
        assert_eq!(p.position[1], 2.0);
        assert_eq!(p.position[2], 3.0);
        assert_eq!(p.orientation[3], 1.0);
    }

    // --- Equality ---

    #[test]
    fn pose_equality() {
        let a = Pose {
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        };
        let b = Pose {
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        };
        let c = Pose {
            position: [1.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        };
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    // --- Serde round-trip ---

    #[test]
    fn pose_serde_round_trip() {
        let original = Pose {
            position: [0.5, 0.0, 0.3],
            orientation: [0.0, 0.0, 0.707, 0.707],
        };
        let json = serde_json::to_string(&original).expect("serialize");
        assert!(json.contains("position"));
        assert!(json.contains("orientation"));
        let deserialized: Pose = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, deserialized);
    }

    // --- Clone and Debug ---

    #[test]
    fn pose_is_clone_and_debug() {
        let a = Pose {
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        };
        let b = a.clone();
        assert_eq!(a, b);
        let _ = format!("{:?}", a);
    }
}
