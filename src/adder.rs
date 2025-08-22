use std::ops::Deref;

use bevy::math::Vec3;
use bevy_math::Vec3A;

// Rework contacts, see Jondolf's example:
// https://discord.com/channels/691052431525675048/1124043933886976171/1398707094252945408
#[derive(Debug)]
pub struct Contact {
    pub point: Vec3,
    pub anchor1: Vec3,
    pub anchor2: Vec3,
    pub normal: Vec3,
    pub penetration: f32,
}

pub struct Manifolds<'a, T: From<Contact>>(pub(crate) &'a mut Vec<T>);

impl<T: From<Contact>> Deref for Manifolds<'_, T> {
    type Target = Vec<T>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

pub(crate) struct ManifoldAdder<'a, T: From<Contact>> {
    manifolds: Manifolds<'a, T>,
    flipped: bool,
}

impl<'a, T: From<Contact>> ManifoldAdder<'a, T> {
    pub fn normal(manifolds: Manifolds<'a, T>) -> Self {
        Self {
            manifolds,
            flipped: false,
        }
    }

    pub fn flipped(manifolds: Manifolds<'a, T>) -> Self {
        Self {
            manifolds,
            flipped: true,
        }
    }

    pub fn push(
        &mut self,
        point: Vec3A,
        anchor_a: Vec3A,
        anchor_b: Vec3A,
        normal: Vec3A,
        penetration: f32,
    ) {
        self.manifolds.0.push(
            Contact {
                point: point.into(),
                anchor1: if self.flipped { anchor_b } else { anchor_a }.into(),
                anchor2: if self.flipped { anchor_a } else { anchor_b }.into(),
                normal: if self.flipped { -normal } else { normal }.into(),
                penetration,
            }
            .into(),
        );
    }
}
