use std::ops::{Add, Deref, DerefMut, Sub};

use approx::ulps_eq;
use bevy::math::{primitives::*, Isometry3d, Vec3, Vec3A};
use bevy_prototype_sdf::{ExecutableSdf3d, Isometry};

#[cfg(test)]
use crate::adder::Manifolds;
#[cfg(test)]
use bevy::math::Quat;
#[cfg(test)]
use std::f32::consts::PI;

use crate::adder::{Contact, ManifoldAdder};

pub struct ScaledIsometry3d {
    pub iso: Isometry3d,
    pub scale: f32,
}

impl Deref for ScaledIsometry3d {
    type Target = Isometry3d;
    fn deref(&self) -> &Self::Target {
        &self.iso
    }
}

impl DerefMut for ScaledIsometry3d {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.iso
    }
}

pub trait Collidable {
    type Isometry;
}

impl Collidable for Sphere {
    type Isometry = Isometry3d;
}

impl Collidable for Capsule3d {
    type Isometry = Isometry3d;
}

impl Collidable for ExecutableSdf3d<'_> {
    type Isometry = ScaledIsometry3d;
}

pub trait Collider<Target: Collidable>: Collidable {
    fn get_collisions<T: From<Contact>>(
        &self,
        self_iso: Self::Isometry,
        other: &Target,
        other_iso: Target::Isometry,
        adder: ManifoldAdder<T>,
        pred_dist: f32,
    );
}

impl Collider<Sphere> for Sphere {
    fn get_collisions<T: From<Contact>>(
        &self,
        self_iso: Isometry3d,
        other: &Self,
        other_iso: Isometry3d,
        mut adder: ManifoldAdder<T>,
        pred_dist: f32,
    ) {
        let offset = self_iso.translation.distance(other_iso.translation);
        let dist = offset - self.radius - other.radius;
        if dist > pred_dist {
            return;
        }

        let mut self_to_other = (other_iso.translation - self_iso.translation) / offset;
        if self_to_other == Vec3A::NAN {
            self_to_other = Vec3A::Y;
        }

        let half_dist = dist * 0.5;
        let anchor1 = self_to_other * (self.radius + half_dist);
        let world_point = self_iso.translation + anchor1;
        let anchor2 = world_point - other_iso.translation;

        adder.push(world_point, anchor1, anchor2, self_to_other, -dist);
    }
}

#[test]
fn test_sphere_sphere() {
    let s1 = Sphere { radius: 1.2 };
    let s1_iso = Isometry3d {
        translation: Vec3A::new(-1., 0.3, -0.4),
        rotation: Quat::from_rotation_x(PI / 2.),
    };
    let s2 = Sphere { radius: 0.8 };
    let s2_iso = Isometry3d {
        translation: Vec3A::new(1., 0.5, 0.1),
        rotation: Quat::from_rotation_z(-PI / 2.),
    };
    let mut contacts = Vec::<Contact>::default();
    let manifolds = Manifolds(&mut contacts);
    let adder = ManifoldAdder::normal(manifolds);
    s1.get_collisions(s1_iso, &s2, s2_iso, adder, 0.);
    panic!("{:?}", contacts);
}

impl Collider<ExecutableSdf3d<'_>> for Sphere {
    fn get_collisions<T: From<Contact>>(
        &self,
        self_iso: Isometry3d,
        sdf: &ExecutableSdf3d,
        sdf_iso: ScaledIsometry3d,
        mut adder: ManifoldAdder<T>,
        pred_dist: f32,
    ) {
        let sdf_local_pos = sdf_iso.rotation.inverse()
            * (self_iso.translation - sdf_iso.translation)
            / sdf_iso.scale;
        let distance = sdf.distance(Vec3::from(sdf_local_pos)) * sdf_iso.scale;
        if distance < self.radius + pred_dist {
            let gradient = Vec3A::from(sdf.gradient(Vec3::from(sdf_local_pos)));
            let world_normal = sdf_iso.rotation * -gradient;

            let pen = self.radius - distance;
            let anchor1 = world_normal * (self.radius - pen * 0.5);
            let world_point = self_iso.translation + anchor1;
            let anchor2 = world_point - sdf_iso.translation;

            adder.push(world_point, anchor1, anchor2, world_normal, pen);
        }
    }
}

impl Collider<Capsule3d> for Sphere {
    fn get_collisions<T: From<Contact>>(
        &self,
        self_iso: Isometry3d,
        other: &Capsule3d,
        other_iso: Isometry3d,
        adder: ManifoldAdder<T>,
        pred_dist: f32,
    ) {
        let capsule_up = other_iso.rotation * Vec3A::Y;
        let t = (self_iso.translation - other_iso.translation)
            .dot(capsule_up)
            .clamp(-other.half_length, other.half_length);
        let capsule_sphere = Sphere {
            radius: other.radius,
        };
        let capsule_sphere_iso = other_iso.translate(Vec3::new(0., t, 0.));
        self.get_collisions(
            self_iso,
            &capsule_sphere,
            capsule_sphere_iso,
            adder,
            pred_dist,
        );
    }
}

impl Collider<Capsule3d> for Capsule3d {
    fn get_collisions<T: From<Contact>>(
        &self,
        self_iso: Isometry3d,
        other: &Self,
        other_iso: Isometry3d,
        mut adder: ManifoldAdder<T>,
        pred_dist: f32,
    ) {
        let up1 = self_iso.rotation * Vec3::Y;
        let up2 = other_iso.rotation * Vec3::Y;
        let bottom1 = Vec3::from(self_iso.translation) - up1 * self.half_length;
        let bottom2 = Vec3::from(other_iso.translation) - up2 * other.half_length;
        let to_end1 = up1 * self.half_length * 2.;
        let to_end2 = up2 * other.half_length * 2.;
        let seg1 = (bottom1, to_end1);
        let seg2 = (bottom2, to_end2);
        let (p1, p2) = closest_points_on_segments(seg1, seg2);

        let wp1 = bottom1 + to_end1 * p1;
        let wp2 = bottom2 + to_end2 * p2;
        let offset = wp1.distance(wp2);
        let dist = offset - self.radius - other.radius;
        if dist > pred_dist {
            return;
        }

        let world_normal = if offset == 0. {
            Vec3A::Y
        } else {
            Vec3A::from((wp2 - wp1) / offset)
        };

        // TODO: This point might not be on the surface of the capsules even when it should be
        let anchor1 =
            Vec3A::from(up1) * (p1 - self.half_length) + world_normal * (self.radius + dist * 0.5);
        let world_point = self_iso.translation + anchor1;
        let anchor2 = world_point - other_iso.translation;

        adder.push(world_point, anchor1, anchor2, world_normal, -dist);
    }
}

trait Point: Add<Output = Self> + Sub<Output = Self> + Copy {
    fn length_squared(self) -> f32;
    fn dot(self, rhs: Self) -> f32;
}

impl Point for Vec3 {
    fn length_squared(self) -> f32 {
        Vec3::length_squared(self)
    }

    fn dot(self, rhs: Self) -> f32 {
        Vec3::dot(self, rhs)
    }
}

fn closest_points_on_segments<P: Point>(
    (origin1, to_end1): (P, P),
    (origin2, to_end2): (P, P),
) -> (f32, f32) {
    // Based on `closest_points_segment_segment_with_locations_nD` from parry which is
    // in turn inspired by RealField-time collision detection by Christer Ericson.
    let r = origin1 - origin2;

    let a = to_end1.length_squared();
    let e = to_end2.length_squared();
    let c = to_end1.dot(r);
    let f = to_end2.dot(r);

    let mut s;
    let mut t;

    let eps = f32::EPSILON;
    if a <= eps && e <= eps {
        s = 0.0;
        t = 0.0;
    } else if a <= eps {
        s = 0.0;
        t = (f / e).clamp(0., 1.);
    } else if e <= eps {
        t = 0.0;
        s = (-c / a).clamp(0., 1.);
    } else {
        let b = to_end1.dot(to_end2);
        let ae = a * e;
        let bb = b * b;
        let denom = ae - bb;

        // Use absolute and ulps error to test collinearity.
        if denom > eps && !ulps_eq!(ae, bb) {
            s = ((b * f - c * e) / denom).clamp(0.0, 1.0);
        } else {
            s = 0.0;
        }

        t = (b * s + f) / e;

        if t < 0.0 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else if t > 1.0 {
            t = 1.0;
            s = ((b - c) / a).clamp(0.0, 1.0);
        }
    }

    (s, t)
}

#[test]
fn test_capsule_capsule() {
    let c1 = Capsule3d {
        radius: 0.2,
        half_length: 1.,
    };
    let c1_iso = Isometry3d {
        translation: Vec3A::ZERO,
        rotation: Quat::from_rotation_y(PI),
    };

    let c2 = Capsule3d {
        radius: 0.3,
        half_length: 2.,
    };
    let c2_iso = Isometry3d {
        translation: Vec3A::new(0., 0.25, 0.),
        rotation: Quat::from_rotation_z(PI / 2.),
    };

    let mut contacts = Vec::<Contact>::default();
    let manifolds = Manifolds(&mut contacts);
    let adder = ManifoldAdder::normal(manifolds);
    c1.get_collisions(c1_iso, &c2, c2_iso, adder, 0.);

    panic!("{:?}", contacts);
}

impl Collider<ExecutableSdf3d<'_>> for Capsule3d {
    fn get_collisions<T: From<Contact>>(
        &self,
        self_iso: Isometry3d,
        sdf: &ExecutableSdf3d,
        sdf_iso: ScaledIsometry3d,
        mut adder: ManifoldAdder<T>,
        pred_dist: f32,
    ) {
        let sdf_local_center = sdf_iso.rotation.inverse()
            * (self_iso.translation - sdf_iso.translation)
            / sdf_iso.scale;

        let center_dist = sdf.distance(sdf_local_center.into());
        if center_dist > self.radius + self.half_length + pred_dist {
            return;
        }

        let sdf_local_up =
            sdf_iso.rotation.inverse() * self_iso.rotation * Vec3A::Y / sdf_iso.scale;

        let mut total = self.half_length * 2.;
        let start = sdf_local_center - sdf_local_up * self.half_length;
        let res = march_edge(sdf, start.into(), sdf_local_up.into(), self.radius, total);

        let (at, dist) = match res {
            MarchResult::Hit(toi, dist) => {
                total = total - *toi;
                (toi, dist)
            }
            MarchResult::Closest(toi, dist) => {
                total = 0.;
                (toi, dist)
            }
        };

        if dist < self.radius + pred_dist {
            let sdf_local_min_point = (start + sdf_local_up * *at).into();
            let gradient = Vec3A::from(sdf.gradient(sdf_local_min_point));
            let world_normal = sdf_iso.rotation * -gradient;

            let pen = self.radius - dist;
            let anchor1 =
                sdf_local_up * (*at - self.half_length) + world_normal * (self.radius - pen * 0.5);
            let world_point = self_iso.translation + anchor1;
            let anchor2 = world_point - sdf_iso.translation;

            adder.push(world_point, anchor1, anchor2, world_normal, pen);
        }

        let start = sdf_local_center + sdf_local_up * self.half_length;
        let res = march_edge(
            sdf,
            start.into(),
            (-sdf_local_up).into(),
            self.radius,
            total,
        );
        let (at, dist) = res.either();

        if dist < self.radius + pred_dist {
            let sdf_local_min_point = (start - sdf_local_up * *at).into();
            let gradient = Vec3A::from(sdf.gradient(sdf_local_min_point));
            let world_normal = sdf_iso.rotation * -gradient;

            let pen = self.radius - dist;
            let anchor1 =
                sdf_local_up * (self.half_length - *at) + world_normal * (self.radius - pen * 0.5);
            let world_point = self_iso.translation + anchor1;
            let anchor2 = world_point - sdf_iso.translation;

            adder.push(world_point, anchor1, anchor2, world_normal, pen);
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct TimeOfImpact(f32);
impl Deref for TimeOfImpact {
    type Target = f32;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum MarchResult {
    Hit(TimeOfImpact, f32),
    Closest(TimeOfImpact, f32),
}

impl MarchResult {
    pub fn either(&self) -> (TimeOfImpact, f32) {
        match self {
            &Self::Hit(a, b) => (a, b),
            &Self::Closest(a, b) => (a, b),
        }
    }
}

const MINIMUM_STEP: f32 = 0.001;

pub(crate) fn march_edge(
    sdf: &ExecutableSdf3d,
    local_start: Vec3,
    local_direction: Vec3,
    radius: f32,
    length: f32,
) -> MarchResult {
    let mut traveled = 0.;
    let mut closest = (0., f32::INFINITY);

    // Iterate over the line until we find a very small distance or get a contact
    while traveled < length {
        let sdf_local_pos = local_start + local_direction * traveled;
        let distance = sdf.distance(sdf_local_pos);
        // TODO: Improve behavior for ghost surfaces from subtract/intersect ops by continuing
        //    until we find a negative distance, then picking the zero surface at the sign change
        if distance <= radius {
            return MarchResult::Hit(TimeOfImpact(traveled), distance);
        }
        if distance < closest.1 {
            closest = (traveled, distance);
        }

        traveled += (distance - radius).max(MINIMUM_STEP);
    }

    MarchResult::Closest(TimeOfImpact(closest.0), closest.1)
}
