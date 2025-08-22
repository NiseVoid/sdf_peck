use avian3d::{
    collision::collider::{PairContext, SingleContext},
    prelude::*,
};
use bevy::prelude::*;
use bevy_math::bounding::{Bounded3d, BoundingVolume};
use bevy_prototype_sdf::{dim3::Dim3, ExecutableSdfs};

use crate::{
    adder::{Contact, ManifoldAdder, Manifolds},
    collider::SdfColliderKind,
    primitives::{Collider, ScaledIsometry3d},
    SdfCollider,
};

use avian3d::prelude::{AnyCollider, ContactManifold, ScalableCollider};
use bevy::math::Vec3;

impl From<Contact> for ContactManifold {
    fn from(value: Contact) -> Self {
        Self {
            points: vec![ContactPoint::new(
                value.anchor1,
                value.anchor2,
                value.point,
                value.penetration,
            )],
            normal: value.normal,
            friction: 0.,
            restitution: 0.,
            tangent_velocity: Vec3::ZERO,
        }
    }
}

impl ComputeMassProperties3d for SdfCollider {
    fn mass(&self, density: f32) -> f32 {
        match self.collider {
            SdfColliderKind::Sphere(sphere) => sphere.mass(density),
            SdfColliderKind::Capsule(capsule) => capsule.mass(density),
            _ => density,
        }
    }

    fn unit_principal_angular_inertia(&self) -> Vec3 {
        match self.collider {
            SdfColliderKind::Sphere(sphere) => sphere.unit_principal_angular_inertia(),
            SdfColliderKind::Capsule(capsule) => capsule.unit_principal_angular_inertia(),
            _ => Sphere::new(1.).unit_principal_angular_inertia(),
        }
    }

    fn center_of_mass(&self) -> Vec3 {
        Vec3::ZERO
    }
}

impl AnyCollider for SdfCollider {
    type Context = ExecutableSdfs<'static, Dim3>;

    fn aabb_with_context(
        &self,
        position: avian3d::math::Vector,
        rotation: impl Into<Rotation>,
        context: SingleContext<Self::Context>,
    ) -> ColliderAabb {
        let iso = Isometry3d::new(position, *rotation.into());
        let aabb = match &self.collider {
            &SdfColliderKind::Sphere(mut s) => {
                s.radius *= self.scale;
                s.aabb_3d(iso)
            }
            &SdfColliderKind::Capsule(mut c) => {
                c.radius *= self.scale;
                c.half_length *= self.scale;
                c.aabb_3d(iso)
            }
            SdfColliderKind::Arbitrary(handle) => {
                let Some((_, sdf)) = context.get(handle.id()) else {
                    eprintln!("Failed to get SDF!");
                    return ColliderAabb::INVALID;
                };

                let fake_iso = Isometry3d::new(Vec3A::ZERO, iso.rotation);

                let mut aabb = sdf.aabb(fake_iso);
                aabb.min *= self.scale;
                aabb.max *= self.scale;
                aabb.translate_by(iso.translation);
                aabb
            }
        };
        ColliderAabb {
            min: aabb.min.into(),
            max: aabb.max.into(),
        }
    }

    fn contact_manifolds_with_context(
        &self,
        other: &Self,
        position1: avian3d::math::Vector,
        rotation1: impl Into<Rotation>,
        position2: avian3d::math::Vector,
        rotation2: impl Into<Rotation>,
        pred_dist: avian3d::math::Scalar,
        contacts: &mut Vec<ContactManifold>,
        context: PairContext<Self::Context>,
    ) {
        contacts.clear();
        let manifolds = Manifolds(contacts);

        let iso1 = Isometry3d::new(position1, *rotation1.into());
        let iso2 = Isometry3d::new(position2, *rotation2.into());

        let scale1 = self.scale;
        let scale2 = other.scale;
        match (&self.collider, &other.collider) {
            (SdfColliderKind::Sphere(mut s1), SdfColliderKind::Sphere(mut s2)) => {
                s1.radius *= scale1;
                s2.radius *= scale2;
                s1.get_collisions(iso1, &s2, iso2, ManifoldAdder::normal(manifolds), pred_dist);
            }
            (SdfColliderKind::Sphere(mut s1), SdfColliderKind::Capsule(mut c2)) => {
                s1.radius *= scale1;
                c2.radius *= scale2;
                c2.half_length *= scale2;
                s1.get_collisions(iso1, &c2, iso2, ManifoldAdder::normal(manifolds), pred_dist);
            }
            (SdfColliderKind::Capsule(mut c1), SdfColliderKind::Capsule(mut c2)) => {
                c1.radius *= scale1;
                c1.half_length *= scale1;
                c2.radius *= scale2;
                c2.half_length *= scale2;
                c1.get_collisions(iso1, &c2, iso2, ManifoldAdder::normal(manifolds), pred_dist);
            }
            (SdfColliderKind::Capsule(mut c1), SdfColliderKind::Sphere(mut s2)) => {
                c1.radius *= scale1;
                c1.half_length *= scale1;
                s2.radius *= scale2;
                s2.get_collisions(
                    iso2,
                    &c1,
                    iso1,
                    ManifoldAdder::flipped(manifolds),
                    pred_dist,
                );
            }

            (&SdfColliderKind::Sphere(mut s), SdfColliderKind::Arbitrary(handle)) => {
                let Some((_, sdf)) = context.get(handle.id()) else {
                    return;
                };

                s.radius *= scale1;

                s.get_collisions(
                    iso1,
                    &sdf,
                    ScaledIsometry3d {
                        iso: iso2,
                        scale: scale2,
                    },
                    ManifoldAdder::normal(manifolds),
                    pred_dist,
                );
            }
            (SdfColliderKind::Arbitrary(handle), &SdfColliderKind::Sphere(mut s)) => {
                let Some((_, sdf)) = context.get(handle.id()) else {
                    return;
                };

                s.radius *= scale2;

                s.get_collisions(
                    iso2,
                    &sdf,
                    ScaledIsometry3d {
                        iso: iso1,
                        scale: scale1,
                    },
                    ManifoldAdder::flipped(manifolds),
                    pred_dist,
                );
            }

            (&SdfColliderKind::Capsule(mut c), SdfColliderKind::Arbitrary(handle)) => {
                let Some((_, sdf)) = context.get(handle.id()) else {
                    return;
                };

                c.radius *= scale1;
                c.half_length *= scale1;

                c.get_collisions(
                    iso1,
                    &sdf,
                    ScaledIsometry3d {
                        iso: iso2,
                        scale: scale2,
                    },
                    ManifoldAdder::normal(manifolds),
                    pred_dist,
                );
            }
            (SdfColliderKind::Arbitrary(handle), &SdfColliderKind::Capsule(mut c)) => {
                let Some((_, sdf)) = context.get(handle.id()) else {
                    return;
                };

                c.radius *= scale2;
                c.half_length *= scale2;

                c.get_collisions(
                    iso2,
                    &sdf,
                    ScaledIsometry3d {
                        iso: iso1,
                        scale: scale1,
                    },
                    ManifoldAdder::flipped(manifolds),
                    pred_dist,
                );
            }

            (t1, t2) => warn!(
                "Unsupported collision: {:?} vs {:?} ({} vs {})",
                t1, t2, context.entity1, context.entity2
            ),
        }
    }
}

impl ScalableCollider for SdfCollider {
    fn scale(&self) -> Vec3 {
        Vec3::splat(self.scale)
    }
    fn set_scale(&mut self, scale: Vec3, _: u32) {
        self.scale = scale.min_element();
    }
}
