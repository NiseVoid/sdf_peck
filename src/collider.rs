use bevy::{
    asset::prelude::Handle, ecs::prelude::Component, math::primitives::*, reflect::Reflect,
};
use bevy_prototype_sdf::Sdf3d;

#[derive(Component, Debug, Reflect)]
#[type_path(sdf_peck)]
pub struct SdfCollider {
    pub(crate) collider: SdfColliderKind,
    pub(crate) scale: f32,
}

impl SdfCollider {
    pub fn sphere(radius: f32) -> Self {
        Self {
            collider: SdfColliderKind::Sphere(Sphere::new(radius)),
            scale: 1.,
        }
    }

    pub fn capsule(radius: f32, length: f32) -> Self {
        Self {
            collider: SdfColliderKind::Capsule(Capsule3d::new(radius, length)),
            scale: 1.,
        }
    }

    pub fn sdf(handle: Handle<Sdf3d>) -> Self {
        Self {
            collider: SdfColliderKind::Arbitrary(handle),
            scale: 1.,
        }
    }

    pub fn collider(&self) -> &SdfColliderKind {
        &self.collider
    }
}

#[derive(Component, Debug, Reflect)]
pub enum SdfColliderKind {
    Sphere(Sphere),
    Capsule(Capsule3d),
    // TODO: Uneven capsule
    // TODO: Torus
    Arbitrary(Handle<Sdf3d>),
}

impl Default for SdfColliderKind {
    fn default() -> Self {
        Self::Sphere(Sphere::default())
    }
}
