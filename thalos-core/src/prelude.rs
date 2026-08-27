pub use crate::robot::{
    active_robot::ActiveRobot,
    builder::SerialChainBuilder,
    error::RobotBuilderError,
    joint::{
        FixedJoint, JointId, JointInfo, JointKind, JointLimits, JointType, PrismaticJoint,
        RevoluteJoint,
    },
    link::Link,
    segment::Segment,
    serial_chain::SerialChain,
    state::RobotState,
};

pub use thalos_math::{
    DynamicMatrix, DynamicVector, Quaternion, Transform3D, UnitQuaternion, UnitVector3, Vector3,
    algebra::vector_to_dynamic,
    constants::{EPS, PI, PI_2},
};

pub use crate::spatial::{
    frame::{Frame, FrameId, FrameRegistry},
    pose::Pose,
};

pub use crate::kinematics::{
    forward::ForwardKinematics,
    inverse::{
        DampedLeastSquaresSolver, IKGoal, IKResult, IKSolver, IKStatus, JacobianTransposeSolver,
    },
    jacobian::{
        GeometricJacobian, Jacobian, JacobianSolver, ManipulabilityReport, NumericalJacobian,
        SingularityReport,
    },
};

pub use crate::analysis::manipulability::{
    ManipulabilityAnalysis, ManipulabilityAnalyzer, ManipulabilityMetrics, ManipulabilitySample,
};
pub use crate::analysis::singularity::{
    SingularityAnalysis, SingularityAnalyzer, SingularityConfig, SingularityMetrics,
    SingularitySample, SingularityState,
};
pub use crate::analysis::workspace::{
    BoundingBox, Reachability, Workspace, WorkspaceConfig, WorkspaceError, WorkspaceKey,
    WorkspaceMetrics, WorkspaceSample, WorkspaceSampler,
};

pub use crate::collision::{
    Box3D, CollisionBody, CollisionBodyBuilder, CollisionChecker, CollisionGeometry,
    CollisionMatrix, CollisionPair, CollisionResult, CollisionType, Cylinder, EntityId, Sphere,
};

pub use crate::trajectory::{Trajectory, TrajectoryPoint};
