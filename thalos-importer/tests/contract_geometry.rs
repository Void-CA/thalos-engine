use thalos_importer::import_urdf;
use thalos_models::geometry::Geometry;

#[test]
fn geometry_primitives_and_mesh_contract() {
    let xml = r#"
        <robot name="geom_bot">
            <link name="base_link"/>
            <link name="l_box">
                <visual><geometry><box size="1.0 2.0 3.0"/></geometry></visual>
            </link>
            <link name="l_cyl">
                <visual><geometry><cylinder radius="0.2" length="1.5"/></geometry></visual>
            </link>
            <link name="l_sph">
                <visual><geometry><sphere radius="0.5"/></geometry></visual>
            </link>
            <link name="l_mesh">
                <visual><geometry><mesh filename="package://robot/meshes/arm.stl" scale="0.001 0.001 0.001"/></geometry></visual>
            </link>

            <joint name="j1" type="fixed"><parent link="base_link"/><child link="l_box"/></joint>
            <joint name="j2" type="fixed"><parent link="base_link"/><child link="l_cyl"/></joint>
            <joint name="j3" type="fixed"><parent link="base_link"/><child link="l_sph"/></joint>
            <joint name="j4" type="fixed"><parent link="base_link"/><child link="l_mesh"/></joint>
        </robot>
    "#;

    let robot = import_urdf(xml).expect("geometry contract robot should import");

    // Box
    let l_box = robot.links.get("l_box").unwrap();
    match &l_box.visual[0].geometry {
        Geometry::Box { width, height, depth } => {
            assert_eq!(*width, 1.0);
            assert_eq!(*height, 2.0);
            assert_eq!(*depth, 3.0);
        }
        other => panic!("expected Box, got {:?}", other),
    }

    // Cylinder
    let l_cyl = robot.links.get("l_cyl").unwrap();
    match &l_cyl.visual[0].geometry {
        Geometry::Cylinder { radius, height } => {
            assert_eq!(*radius, 0.2);
            assert_eq!(*height, 1.5);
        }
        other => panic!("expected Cylinder, got {:?}", other),
    }

    // Sphere
    let l_sph = robot.links.get("l_sph").unwrap();
    match &l_sph.visual[0].geometry {
        Geometry::Sphere { radius } => {
            assert_eq!(*radius, 0.5);
        }
        other => panic!("expected Sphere, got {:?}", other),
    }

    // Mesh
    let l_mesh = robot.links.get("l_mesh").unwrap();
    match &l_mesh.visual[0].geometry {
        Geometry::Mesh { filename, scale } => {
            assert_eq!(filename, "package://robot/meshes/arm.stl");
            let s = scale.expect("mesh scale should be present");
            assert_eq!(s.x, 0.001);
            assert_eq!(s.y, 0.001);
            assert_eq!(s.z, 0.001);
        }
        other => panic!("expected Mesh, got {:?}", other),
    }
}
