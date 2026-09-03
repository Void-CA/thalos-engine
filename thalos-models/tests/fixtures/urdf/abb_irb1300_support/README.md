# ABB IRB 1300-10/1.15 Fixture Specification

## Metadata
- **Robot Model**: ABB IRB 1300-10/1.15 (10 kg payload, 1.15 m reach, 6-DOF industrial manipulator)
- **Source**: Formatted according to ROS-Industrial ABB package conventions (`abb_irb1300_support`)
- **Format**: Expanded URDF (`irb1300_10_115.urdf`) + Binary STL meshes
- **License**: CC-BY-4.0 / Apache-2.0 (ROS-Industrial ecosystem standard)

## Package Layout
```text
abb_irb1300_support/
├── README.md
├── urdf/
│   └── irb1300_10_115.urdf
└── meshes/
    ├── visual/
    │   ├── base_link.stl
    │   ├── link_1.stl
    │   ├── link_2.stl
    │   ├── link_3.stl
    │   ├── link_4.stl
    │   ├── link_5.stl
    │   └── link_6.stl
    └── collision/
        ├── base_link.stl
        ├── link_1.stl
        ├── link_2.stl
        ├── link_3.stl
        ├── link_4.stl
        ├── link_5.stl
        └── link_6.stl
```

## Supported URI Forms
- `package://abb_irb1300_support/meshes/visual/base_link.stl`
- `package://abb_irb1300_support/meshes/visual/link_1.stl`
- `package://abb_irb1300_support/meshes/visual/link_2.stl`
- `package://abb_irb1300_support/meshes/visual/link_3.stl`
- `package://abb_irb1300_support/meshes/visual/link_4.stl`
- `package://abb_irb1300_support/meshes/visual/link_5.stl`
- `package://abb_irb1300_support/meshes/visual/link_6.stl`
- `package://abb_irb1300_support/meshes/collision/...`

## Kinematic Chain
`base_link` -> `joint_1` (revolute Z) -> `link_1` -> `joint_2` (revolute Y) -> `link_2` -> `joint_3` (revolute Y) -> `link_3` -> `joint_4` (revolute X) -> `link_4` -> `joint_5` (revolute Y) -> `link_5` -> `joint_6` (revolute X) -> `link_6` -> `flange` (fixed) -> `tool0` (fixed)
