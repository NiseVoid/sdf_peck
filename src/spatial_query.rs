use avian3d::{
    collision::collider::{BoundedShape, QueryCollider, QueryShapeCastHit, SingleContext},
    prelude::{AnyCollider, ColliderAabb, Rotation},
    spatial_query::obvhs::ray::Ray,
};
use bevy::{
    asset::Handle,
    ecs::system::SystemParamItem,
    prelude::{Capsule3d, Sphere},
};
use bevy_math::{bounding::Bounded3d, Dir3, FloatPow, Isometry3d, Quat, Ray3d, Vec2, Vec3};
use bevy_prototype_sdf::{Sdf, Sdf3d};

use crate::{
    adder::{Contact, ManifoldAdder, Manifolds},
    collider::SdfColliderKind,
    primitives::{march_edge, Collider, MarchResult, ScaledIsometry3d},
    SdfCollider,
};

#[derive(Debug)]
pub enum ColliderShape {
    Sphere(Sphere),
    Capsule(Capsule3d),
    Arbitrary(Handle<Sdf3d>),
}

impl BoundedShape<<SdfCollider as AnyCollider>::Context> for ColliderShape {
    fn shape_aabb(
        &self,
        translation: Vec3,
        rotation: Quat,
        context: &SystemParamItem<<SdfCollider as AnyCollider>::Context>,
    ) -> ColliderAabb {
        let iso = Isometry3d::new(translation, rotation);
        let aabb = match self {
            &Self::Sphere(s) => s.aabb_3d(iso),
            &Self::Capsule(c) => c.aabb_3d(iso),
            Self::Arbitrary(handle) => {
                let Some(sdf) = context.get(handle.id()) else {
                    return ColliderAabb::default();
                };
                sdf.1.aabb(iso)
            }
        };
        ColliderAabb {
            min: aabb.min.into(),
            max: aabb.max.into(),
        }
    }
}

impl ColliderShape {
    pub fn sphere(radius: f32) -> Self {
        Self::Sphere(Sphere::new(radius))
    }

    pub fn capsule(radius: f32, length: f32) -> Self {
        Self::Capsule(Capsule3d::new(radius, length))
    }

    pub fn sdf(handle: Handle<Sdf3d>) -> Self {
        Self::Arbitrary(handle)
    }
}

impl QueryCollider for SdfCollider {
    type CastShape = Sphere;
    type Shape = ColliderShape;

    fn ray_hit(&self, ray: Ray, solid: bool, context: SingleContext<Self::Context>) -> f32 {
        match &self.collider {
            SdfColliderKind::Arbitrary(handle) => {
                let Some(sdf) = context.get(handle.id()) else {
                    return f32::INFINITY;
                };
                let res = march_edge(
                    &sdf.1,
                    ray.origin.into(),
                    ray.direction.into(),
                    0.001,
                    ray.tmax,
                );
                let MarchResult::Hit(toi, _) = res else {
                    return f32::INFINITY;
                };
                *toi
            }
            &SdfColliderKind::Sphere(Sphere { radius }) => {
                let bray = Ray3d::new(ray.origin.into(), Dir3::new_unchecked(ray.direction.into()));
                local_ray_distance_with_sphere(radius, bray, solid)
                    .filter(|&distance| distance <= ray.tmax)
                    .unwrap_or(f32::INFINITY)
            }
            SdfColliderKind::Capsule(capsule) => {
                let bray = Ray3d::new(ray.origin.into(), Dir3::new_unchecked(ray.direction.into()));
                local_ray_distance_with_capsule(capsule, bray, ray.tmax, solid)
                    .unwrap_or(f32::INFINITY)
            }
        }
    }

    fn ray_normal(
        &self,
        point: Vec3,
        _: Dir3,
        _: bool,
        context: SingleContext<Self::Context>,
    ) -> Vec3 {
        match &self.collider {
            SdfColliderKind::Arbitrary(handle) => {
                let Some(sdf) = context.get(handle.id()) else {
                    return Vec3::Y;
                };
                sdf.1.gradient(point)
            }
            SdfColliderKind::Sphere(s) => s.gradient(point),
            SdfColliderKind::Capsule(c) => c.gradient(point),
        }
    }

    fn shape_cast(
        &self,
        shape: &Self::CastShape,
        _: Rotation,
        local_origin: Vec3,
        local_dir: Dir3,
        range: (f32, f32),
        context: SingleContext<Self::Context>,
    ) -> Option<QueryShapeCastHit> {
        match &self.collider {
            SdfColliderKind::Arbitrary(handle) => {
                let Some(sdf) = context.get(handle.id()) else {
                    return None;
                };
                let start = local_origin + local_dir * range.0;
                let res = march_edge(
                    &sdf.1,
                    start,
                    local_dir.into(),
                    shape.radius,
                    range.1 - range.0,
                );
                let MarchResult::Hit(toi, distance) = res else {
                    return None;
                };
                let pos = start + local_dir * *toi;
                let gradient = sdf.1.gradient(pos);
                Some(QueryShapeCastHit {
                    distance: range.0 + *toi,
                    point: pos - gradient * distance,
                    normal: gradient,
                })
            }
            SdfColliderKind::Sphere(s) => {
                let sum = shape.radius + s.radius;
                let bray = Ray3d::new(local_origin.into(), Dir3::new_unchecked(local_dir.into()));
                local_ray_distance_with_sphere(sum, bray, true)
                    .filter(|&distance| distance <= range.1)
                    .map(|distance| {
                        let normal = (local_origin + local_dir * distance).normalize_or(Vec3::Y);
                        QueryShapeCastHit {
                            distance,
                            point: normal * s.radius,
                            normal,
                        }
                    })
            }
            SdfColliderKind::Capsule(c) => {
                let expanded = Capsule3d {
                    radius: c.radius + shape.radius,
                    half_length: c.half_length,
                };
                let bray = Ray3d::new(local_origin.into(), Dir3::new_unchecked(local_dir.into()));
                local_ray_distance_with_capsule(&expanded, bray, range.1, true).map(|distance| {
                    let normal = c.gradient(local_origin + local_dir * distance);
                    QueryShapeCastHit {
                        distance,
                        point: normal * c.radius,
                        normal,
                    }
                })
            }
        }
    }

    fn shape_intersection(
        &self,
        shape: &Self::Shape,
        shape_rotation: Rotation,
        local_origin: Vec3,
        context: SingleContext<Self::Context>,
    ) -> bool {
        let mut contacts = Vec::<Contact>::new();
        let manifolds = Manifolds(&mut contacts);
        let iso1 = Isometry3d::default();
        let iso2 = Isometry3d::new(local_origin, *shape_rotation);
        match &self.collider {
            SdfColliderKind::Sphere(s1) => match shape {
                ColliderShape::Sphere(s2) => {
                    s1.get_collisions(iso1, s2, iso2, ManifoldAdder::normal(manifolds), 0.)
                }
                ColliderShape::Capsule(c2) => {
                    s1.get_collisions(iso1, c2, iso2, ManifoldAdder::normal(manifolds), 0.)
                }
                ColliderShape::Arbitrary(handle2) => {
                    let Some(sdf2) = context.get(handle2.id()) else {
                        return false;
                    };
                    let scaled = ScaledIsometry3d {
                        iso: iso2,
                        scale: 1.,
                    };
                    s1.get_collisions(iso1, &sdf2.1, scaled, ManifoldAdder::normal(manifolds), 0.)
                }
            },
            SdfColliderKind::Capsule(c1) => match shape {
                ColliderShape::Sphere(s2) => {
                    s2.get_collisions(iso2, c1, iso1, ManifoldAdder::flipped(manifolds), 0.)
                }
                ColliderShape::Capsule(c2) => {
                    c1.get_collisions(iso1, c2, iso2, ManifoldAdder::normal(manifolds), 0.)
                }
                ColliderShape::Arbitrary(handle2) => {
                    let Some(sdf2) = context.get(handle2.id()) else {
                        return false;
                    };
                    let scaled = ScaledIsometry3d {
                        iso: iso2,
                        scale: 1.,
                    };
                    c1.get_collisions(iso1, &sdf2.1, scaled, ManifoldAdder::normal(manifolds), 0.)
                }
            },
            SdfColliderKind::Arbitrary(handle) => {
                let Some(sdf1) = context.get(handle.id()) else {
                    return false;
                };
                let scaled1 = ScaledIsometry3d {
                    iso: iso1,
                    scale: 1.,
                };
                match shape {
                    ColliderShape::Sphere(s2) => s2.get_collisions(
                        iso2,
                        &sdf1.1,
                        scaled1,
                        ManifoldAdder::flipped(manifolds),
                        0.,
                    ),
                    ColliderShape::Capsule(c2) => c2.get_collisions(
                        iso2,
                        &sdf1.1,
                        scaled1,
                        ManifoldAdder::flipped(manifolds),
                        0.,
                    ),
                    ColliderShape::Arbitrary(handle2) => {
                        let Some(sdf2) = context.get(handle2.id()) else {
                            return false;
                        };
                        _ = (sdf1, sdf2);
                        todo!();
                    }
                }
            }
        }

        contacts.iter().any(|c| c.penetration >= 0.)
    }

    fn closest_point(
        &self,
        point: Vec3,
        solid: bool,
        context: SingleContext<Self::Context>,
    ) -> Vec3 {
        _ = (point, solid, context);
        todo!()
    }

    fn contains_point(&self, point: Vec3, context: SingleContext<Self::Context>) -> bool {
        _ = (point, context);
        todo!()
    }
}

// Use the version from bevy if it ever lands.
// See: https://github.com/bevyengine/bevy/pull/15724
#[inline]
fn local_ray_distance_with_sphere(radius: f32, ray: Ray3d, solid: bool) -> Option<f32> {
    // See `Circle` for the math and detailed explanation of how this works.

    // The squared distance between the ray origin and the boundary of the sphere.
    let c = ray.origin.length_squared() - radius.squared();

    if c > 0.0 {
        // The ray origin is outside of the sphere.
        let b = ray.origin.dot(*ray.direction);

        if b > 0.0 {
            // The ray points away from the sphere, so there can be no hits.
            return None;
        }

        // The distance corresponding to the boundary hit is the second root.
        let d = b.squared() - c;
        let t2 = -b - d.abs().sqrt().copysign(d);

        Some(t2)
    } else if solid {
        // The ray origin is inside of the solid sphere.
        Some(0.0)
    } else {
        // The ray origin is inside of the hollow sphere.
        // The distance corresponding to the boundary hit is the first root.
        let b = ray.origin.dot(*ray.direction);
        let d = b.squared() - c;
        let t1 = -b + d.sqrt();
        Some(t1)
    }
}

// Use the version from bevy if it ever lands.
// See: https://github.com/bevyengine/bevy/pull/15724
#[inline]
fn local_ray_distance_with_capsule(
    cap: &Capsule3d,
    ray: Ray3d,
    max_distance: f32,
    solid: bool,
) -> Option<f32> {
    // Adapted from Inigo Quilez's ray-capsule intersection algorithm: https://iquilezles.org/articles/intersectors/
    let radius_squared = cap.radius * cap.radius;

    let ba = 2.0 * cap.half_length;
    let oa = Vec3::new(ray.origin.x, ray.origin.y + cap.half_length, ray.origin.z);

    let baba = ba * ba;
    let bard = ba * ray.direction.y;
    let baoa = ba * oa.y;
    let rdoa = ray.direction.dot(oa);
    let oaoa = oa.dot(oa);

    // Note: We use `f32::EPSILON` to avoid division by zero later for rays parallel to the capsule's axis.
    let a = (baba - bard * bard).max(f32::EPSILON);
    let b = baba * rdoa - baoa * bard;
    let c = baba * oaoa - baoa * baoa - radius_squared * baba;
    let d = b * b - a * c;

    if d >= 0.0 {
        let is_inside_cylinder_horizontal = c < 0.0;
        let is_inside_cylinder_vertical = ray.origin.y.abs() < cap.half_length;
        let intersects_hemisphere = is_inside_cylinder_horizontal && {
            // The ray origin intersects one of the hemispheres if the distance
            // between the ray origin and hemisphere center is negative.
            Vec2::new(ray.origin.x, cap.half_length - ray.origin.y.abs()).length_squared()
                < radius_squared
        };
        let is_origin_inside =
            intersects_hemisphere || (is_inside_cylinder_horizontal && is_inside_cylinder_vertical);

        if solid && is_origin_inside {
            return Some(0.);
        }

        let cylinder_distance = if is_origin_inside {
            (-b + d.sqrt()) / a
        } else {
            (-b - d.sqrt()) / a
        };

        let y = baoa + cylinder_distance * bard;

        // Check if the ray hit the cylindrical part.
        let hit_rectangle = y > 0.0 && y < baba;
        if hit_rectangle && cylinder_distance > 0.0 {
            if cylinder_distance > max_distance {
                return None;
            }
            return Some(cylinder_distance);
        }

        // Next, we check the hemispheres for intersections.
        // It's enough to only check one hemisphere and just take the side into account.

        // Offset between the ray origin and the center of the hit hemisphere.
        let offset_ray = Ray3d {
            origin: if y <= 0.0 {
                oa
            } else {
                Vec3::new(ray.origin.x, ray.origin.y - cap.half_length, ray.origin.z)
            },
            direction: ray.direction,
        };

        // See `Sphere` ray casting implementation.

        let b = offset_ray.origin.dot(*ray.direction);
        let c = offset_ray.origin.length_squared() - radius_squared;

        // No intersections if the ray direction points away from the ball and the ray origin is outside of the ball.
        if c > 0.0 && b > 0.0 {
            return None;
        }

        let d = b * b - c;

        if d < 0.0 {
            // No solution, no intersection.
            return None;
        }

        let d_sqrt = d.sqrt();

        let t2 = if is_origin_inside {
            -b + d_sqrt
        } else {
            -b - d_sqrt
        };

        if t2 > 0.0 && t2 <= max_distance {
            // The ray origin is outside of the hemisphere that was hit.
            return Some(t2);
        }

        // The ray hit the hemisphere that the ray origin is in.
        // The distance corresponding to the boundary hit is the first root.
        let t1 = -b + d_sqrt;

        if t1 > max_distance {
            return None;
        }

        return Some(t1);
    }
    None
}
