mod collider;
use std::marker::PhantomData;

pub use collider::{SdfCollider, SdfColliderKind};

mod primitives;

mod adder;

mod avian;

mod spatial_query;
pub use spatial_query::ColliderShape;

use avian3d::prelude::*;
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel, system::SystemParamItem},
    prelude::*,
};
use bevy_prototype_sdf::SdfProcessed;

pub struct SdfCollisionPlugin<H: CollisionHooks = ()> {
    schedule: Interned<dyn ScheduleLabel>,
    phantom: PhantomData<H>,
}

impl<H: CollisionHooks> Default for SdfCollisionPlugin<H> {
    fn default() -> Self {
        Self {
            schedule: FixedPostUpdate.intern(),
            phantom: PhantomData,
        }
    }
}

impl<H: CollisionHooks + 'static> Plugin for SdfCollisionPlugin<H>
where
    for<'w, 's> SystemParamItem<'w, 's, H>: CollisionHooks,
{
    fn build(&self, app: &mut App) {
        app.register_type::<SdfCollider>()
            .add_plugins((
                ColliderBackendPlugin::<SdfCollider>::new(self.schedule),
                SpatialQueryPlugin::<SdfCollider>::default(),
                NarrowPhasePlugin::<SdfCollider, H>::default(),
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
