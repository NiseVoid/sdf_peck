use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_prototype_sdf::SdfPlugin;
use sdf_peck::{SdfCollider, SdfCollisionPlugin};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            SdfPlugin,
            PhysicsPlugins::default(),
            SdfCollisionPlugin::<()>::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, scale_container)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    // mut sdfs: ResMut<Assets<Sdf3d>>,
    loader: Res<AssetServer>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0., 10., 15.).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(3., 5., 0.).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        RigidBody::Static,
        Mesh3d(meshes.add(Sphere::new(7.).mesh().ico(9).unwrap())),
        MeshMaterial3d(materials.add(Color::srgba(0.9, 0.9, 0.9, 0.1))),
        SdfCollider::sdf(loader.load("sphere_stage.sdf3d")),
        Transform::from_xyz(0., 1., 0.),
        Container,
    ));

    let sphere = Sphere::new(0.3);
    let sphere_mesh = meshes.add(sphere.mesh().ico(3).unwrap());
    let sphere_mats = [
        materials.add(Color::srgb(0.7, 0.9, 1.)),
        materials.add(Color::srgb(0.35, 0.75, 0.85)),
        materials.add(Color::srgb(0.4, 0.85, 1.)),
    ];
    for x in -7..=7 {
        for y in 0..2 {
            for z in -4..=5 {
                let color = (3.19 * x as f32 + 2.92 * y as f32 + 1.24 * z as f32).abs() % 2.01;
                let scale = 0.5 + (8.29 * x as f32 + 1.39 * y as f32 + 3.91 * z as f32).abs() % 0.5;
                commands.spawn((
                    RigidBody::Dynamic,
                    Transform::from_xyz(x as f32, 3. + y as f32, z as f32)
                        .with_scale(Vec3::splat(scale)),
                    Mesh3d(sphere_mesh.clone()),
                    MeshMaterial3d(sphere_mats[color as usize].clone()),
                    Friction::new(0.).with_combine_rule(CoefficientCombine::Min),
                    LinearVelocity(Vec3::new(-x as f32, 0., -z as f32) * 3.),
                    Restitution::PERFECTLY_ELASTIC.with_combine_rule(CoefficientCombine::Max),
                    SdfCollider::sphere(sphere.radius),
                ));
            }
        }
    }
}

#[derive(Component)]
struct Container;

fn scale_container(mut containers: Query<&mut Transform, With<Container>>, time: Res<Time>) {
    for mut transform in containers.iter_mut() {
        transform.scale = Vec3::splat(1. - (time.elapsed_secs() * 0.1).sin() * 0.4)
    }
}
