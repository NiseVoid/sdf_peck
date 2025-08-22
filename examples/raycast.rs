use std::f32::consts::{PI, TAU};

use avian3d::prelude::*;
use bevy::{
    prelude::*,
    render::{render_resource::ShaderType, renderer::RenderDevice},
};
use bevy_march::{
    MarcherConeTexture, MarcherMainTextures, MarcherMaterial, MarcherScale, MarcherSettings,
    RayMarcherPlugin, RenderedSdf,
};
use sdf_peck::{ColliderShape, SdfCollider, SdfCollisionPlugin};

fn main() {
    let mut app = App::new();

    app.add_plugins(DefaultPlugins);

    let march_shader = app.world().resource::<AssetServer>().load("march.wgsl");
    app.add_plugins(RayMarcherPlugin::<SdfMaterial>::new(march_shader));

    app.add_plugins((PhysicsPlugins::default(), SdfCollisionPlugin::default()))
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, cast_ray)
        .add_systems(Update, move_camera)
        .run();
}

#[derive(Asset, Reflect, Clone, Debug, ShaderType)]
struct SdfMaterial {
    color: Vec3,
}

impl MarcherMaterial for SdfMaterial {}

const CENTER: Vec3 = Vec3::new(2.8, 10.3, -0.3);

fn setup(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<SdfMaterial>>,
    loader: Res<AssetServer>,
    device: Res<RenderDevice>,
) {
    commands.spawn((
        Camera3d::default(),
        Camera {
            hdr: true,
            ..default()
        },
        CameraAngle::default(),
        MarcherSettings::default(),
        MarcherMainTextures::new(&mut images, (8, 8)),
        MarcherConeTexture::new(&mut images, &device, (8, 8)),
        MarcherScale(1),
    ));

    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(3., 5., 0.).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let sdf = loader.load("query_target.sdf3d");
    commands.spawn((
        RigidBody::Static,
        SdfCollider::sdf(sdf.clone()),
        Transform::from_translation(CENTER),
        RenderedSdf {
            sdf,
            material: materials.add(SdfMaterial { color: Vec3::ONE }),
        },
    ));
}

#[derive(Component, Deref, DerefMut, Default)]
struct CameraAngle(Vec2);

fn move_camera(
    camera: Single<(&mut CameraAngle, &mut Transform), With<Camera3d>>,
    time: Res<Time>,
    input: Res<ButtonInput<KeyCode>>,
) {
    let movement = Vec2::new(
        (input.pressed(KeyCode::KeyS) as i32 - input.pressed(KeyCode::KeyW) as i32) as f32,
        (input.pressed(KeyCode::KeyD) as i32 - input.pressed(KeyCode::KeyA) as i32) as f32,
    );

    let (mut angle, mut transform) = camera.into_inner();

    angle.x = (angle.x + movement.x * time.delta_secs() * 0.1).clamp(-PI * 0.49, PI * 0.49);
    angle.y = (angle.y + movement.y * time.delta_secs() * 0.1) % TAU;

    let rot = Quat::from_rotation_y(angle.y) * Quat::from_rotation_x(angle.x);
    transform.translation = CENTER + rot * Vec3::new(0., 0., 2.);
    transform.look_at(CENTER, Vec3::Y);
}

fn cast_ray(
    mut gizmos: Gizmos,
    camera: Single<&Transform, With<Camera3d>>,
    spatial_query: SpatialQuery<SdfCollider>,
) {
    let origin = camera.translation;
    // let direction = camera.forward();
    // let max_dist = 5.;
    // gizmos.line(origin, origin + direction * max_dist, Color::WHITE);
    let hits = spatial_query.shape_intersections(
        &ColliderShape::Sphere(Sphere::new(1.3)),
        origin,
        Quat::default(),
        &SpatialQueryFilter::DEFAULT,
    );

    if hits.is_empty() {
        return;
    }

    gizmos.sphere(origin, 1.3, Color::srgb(1., 0.5, 0.5));

    // let hit_pos = origin + direction * hit.distance;
    // gizmos.sphere(hit_pos, 0.2, Color::srgb(1., 0.5, 0.5));

    // gizmos.arrow(
    //     hit_pos + hit.point2,
    //     hit_pos + hit.point2 + hit.normal2 * 0.1,
    //     Color::srgb(1., 0.5, 0.5),
    // );
}
