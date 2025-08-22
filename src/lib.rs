mod collider;
pub use collider::{SdfCollider, SdfColliderKind};

mod primitives;

mod adder;

mod avian;

mod spatial_query;
pub use spatial_query::ColliderShape;

use avian3d::prelude::*;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};
use bevy_prototype_sdf::SdfProcessed;

pub struct SdfCollisionPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl Default for SdfCollisionPlugin {
    fn default() -> Self {
        Self {
            schedule: FixedPostUpdate.intern(),
        }
    }
}

impl Plugin for SdfCollisionPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<SdfCollider>()
            .add_plugins((
                ColliderBackendPlugin::<SdfCollider>::new(self.schedule),
                NarrowPhasePlugin::<SdfCollider>::default(),
                SpatialQueryPlugin::<SdfCollider>::default(),
            ))
            .add_observer(invalidate_changed_handle_colliders);
    }
}

fn invalidate_changed_handle_colliders(
    trigger: Trigger<SdfProcessed>,
    mut query: Query<&mut SdfCollider>,
) {
    let SdfProcessed(id) = trigger.event();
    let id = AssetId::from(*id);
    for mut col in query.iter_mut() {
        if let SdfColliderKind::Arbitrary(handle) = col.collider() {
            if handle.id() == id {
                col.set_changed();
            }
        }
    }
}
