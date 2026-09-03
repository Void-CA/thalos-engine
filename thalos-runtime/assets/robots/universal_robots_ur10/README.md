# Universal Robots UR10 Fixture Specification

## Metadata
- **Robot Model**: Universal Robots UR10 (10 kg payload, 1.3 m reach, 6-DOF industrial arm)
- **Source**: Formatted according to ROS-Industrial `ur_description` package conventions
- **Format**: URDF (`ur10.urdf`) + Collada DAE visual meshes + Binary STL collision meshes
- **License**: BSD-3-Clause / Apache-2.0

## Package Layout
```text
ur_description/
├── README.md
├── urdf/
│   └── ur10.urdf
└── meshes/
    └── ur10/
        ├── visual/
        │   ├── base.dae
        │   ├── shoulder.dae
        │   ├── upperarm.dae
        │   ├── forearm.dae
        │   ├── wrist1.dae
        │   ├── wrist2.dae
        │   └── wrist3.dae
        └── collision/
            ├── base.stl
            ├── shoulder.stl
            ├── upperarm.stl
            ├── forearm.stl
            ├── wrist1.stl
            ├── wrist2.stl
            └── wrist3.stl
```

## Supported URI Forms
- `package://ur_description/meshes/ur10/visual/base.dae`
- `package://ur_description/meshes/ur10/collision/base.stl`
- ... (shoulder, upperarm, forearm, wrist1, wrist2, wrist3)
