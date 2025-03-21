//
// Copyright (C) YuqiaoZhang(HanetakaChou)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#include "../include/brx_physics.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Core.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterMask.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>

static int const g_solver_buffer_size = 32 * 1024 * 1024;

class brx_physics_context
{
	JPH::TempAllocator *m_temp_allocator;
	JPH::JobSystemThreadPool *m_job_system;

public:
	inline brx_physics_context();
	inline ~brx_physics_context();
	inline void init();
	inline void uninit();
	inline JPH::TempAllocator *get_temp_allocator() const;
	inline JPH::JobSystemThreadPool *get_job_system() const;
};

class brx_physics_world : public JPH::BroadPhaseLayerInterface, JPH::ObjectVsBroadPhaseLayerFilter
{
	JPH::ObjectLayerPairFilterMask *m_object_vs_object_layer_filter;
	JPH::PhysicsSystem *m_physics_system;
	float m_local_time;

public:
	inline brx_physics_world();
	inline ~brx_physics_world();
	inline void init(float const gravity[3]);
	inline void uninit();
	inline JPH::BodyID physics_world_create_body(float const rotation[4], float const position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type, float const shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type, uint32_t collision_filter_group, uint32_t collision_filter_mask, float mass, float linear_damping, float angular_damping, float friction, float restitution);
	inline void physics_world_destory_body(JPH::BodyID body_id);
	inline JPH::Constraint *physics_world_create_constraint(JPH::BodyID body_id_a, JPH::BodyID body_id_b, BRX_PHYSICS_CONSTRAINT_TYPE constraint_type, float const pivot[3], float const twist_axis[3], float const plane_axis[3], float const normal_axis[3], float const twist_limit[2], float const plane_limit[2], float const normal_limit[2]);
	inline void physics_world_add_body(JPH::BodyID body_id);
	inline void physics_world_remove_body(JPH::BodyID body_id);
	inline void physics_world_add_constraint(JPH::Constraint *physics_constraint);
	inline void physics_world_remove_constraint(JPH::Constraint *physics_constraint);
	inline void physics_world_step(brx_physics_context *physics_context, float delta_time);
	inline void physics_world_apply_body_key_frame(JPH::BodyID body_id, float const rotation[4], float const position[3], float delta_time);
	inline void physics_world_get_body_transform(JPH::BodyID body_id, float out_rotation[4], float out_position[3]) const;

private:
	JPH::uint GetNumBroadPhaseLayers() const override;
	JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override;
	bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override;
};

class brx_physics_rigid_body
{
	JPH::BodyID m_body_id;

public:
	inline brx_physics_rigid_body();
	inline ~brx_physics_rigid_body();
	inline void init(brx_physics_world *physics_world, float const rotation[4], float const position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type, float const shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type, uint32_t collision_filter_group, uint32_t collision_filter_mask, float mass, float linear_damping, float angular_damping, float friction, float restitution);
	inline void uninit(brx_physics_world *physics_world);
	inline JPH::BodyID get_body_id() const;
};

class brx_physics_constraint
{
	JPH::Ref<JPH::Constraint> m_constraint;

public:
	inline brx_physics_constraint();
	inline ~brx_physics_constraint();
	inline void init(brx_physics_world *physics_world, JPH::BodyID body_id_a, JPH::BodyID body_id_b, BRX_PHYSICS_CONSTRAINT_TYPE constraint_type, float const pivot[3], float const twist_axis[3], float const plane_axis[3], float const normal_axis[3], float const twist_limit[2], float const plane_limit[2], float const normal_limit[2]);
	inline void uninit();
	inline JPH::Constraint *get_constraint() const;
};

extern "C" brx_physics_context *brx_physics_create_context()
{
	brx_physics_context *physics_context = new (JPH::Allocate(sizeof(brx_physics_context))) brx_physics_context();

	physics_context->init();

	return physics_context;
}

extern "C" void brx_physics_destory_context(brx_physics_context *physics_context)
{
	physics_context->uninit();

	physics_context->~brx_physics_context();
	JPH::Free(physics_context);
}

extern "C" brx_physics_world *brx_physics_create_world(brx_physics_context *physics_context, float const gravity[3])
{
	brx_physics_world *physics_world = new (JPH::Allocate(sizeof(brx_physics_world))) brx_physics_world();

	physics_world->init(gravity);

	return physics_world;
}

extern "C" void brx_physics_destory_world(brx_physics_context *physics_context, brx_physics_world *physics_world)
{
	physics_world->uninit();

	physics_world->~brx_physics_world();
	JPH::Free(physics_world);
}

extern "C" void brx_physics_world_add_rigid_body(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body)
{
	physics_world->physics_world_add_body(physics_rigid_body->get_body_id());
}

extern "C" void brx_physics_world_remove_rigid_body(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body)
{
	physics_world->physics_world_remove_body(physics_rigid_body->get_body_id());
}

extern "C" void brx_physics_world_add_constraint(brx_physics_context *, brx_physics_world *physics_world, brx_physics_constraint *physics_constraint)
{
	physics_world->physics_world_add_constraint(physics_constraint->get_constraint());
}

extern "C" void brx_physics_world_remove_constraint(brx_physics_context *, brx_physics_world *physics_world, brx_physics_constraint *physics_constraint)
{
	physics_world->physics_world_remove_constraint(physics_constraint->get_constraint());
}

extern "C" void brx_physics_world_step(brx_physics_context *physics_context, brx_physics_world *physics_world, float delta_time)
{
	physics_world->physics_world_step(physics_context, delta_time);
}

extern "C" brx_physics_rigid_body *brx_physics_create_rigid_body(brx_physics_context *physics_context, brx_physics_world *physics_world, float const rotation[4], float const position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type, float const shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type, uint32_t collision_filter_group, uint32_t collision_filter_mask, float mass, float linear_damping, float angular_damping, float friction, float restitution)
{
	brx_physics_rigid_body *physics_rigid_body = new (JPH::Allocate(sizeof(brx_physics_rigid_body))) brx_physics_rigid_body();

	physics_rigid_body->init(physics_world, rotation, position, shape_type, shape_size, motion_type, collision_filter_group, collision_filter_mask, mass, linear_damping, angular_damping, restitution, friction);

	return physics_rigid_body;
}

extern "C" void brx_physics_destory_rigid_body(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body)
{
	physics_rigid_body->uninit(physics_world);

	physics_rigid_body->~brx_physics_rigid_body();
	JPH::Free(physics_rigid_body);
}

extern "C" void brx_physics_rigid_body_apply_key_frame(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body, float const rotation[4], float const position[3], float delta_time)
{
	physics_world->physics_world_apply_body_key_frame(physics_rigid_body->get_body_id(), rotation, position, delta_time);
}

extern "C" void brx_physics_rigid_body_get_transform(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body, float out_rotation[4], float out_position[3])
{
	physics_world->physics_world_get_body_transform(physics_rigid_body->get_body_id(), out_rotation, out_position);
}

extern "C" brx_physics_constraint *brx_physics_create_constraint(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body_a, brx_physics_rigid_body *physics_rigid_body_b, BRX_PHYSICS_CONSTRAINT_TYPE constraint_type, float const pivot[3], float const twist_axis[3], float const plane_axis[3], float const normal_axis[3], float const twist_limit[2], float const plane_limit[2], float const normal_limit[2])
{
	brx_physics_constraint *physics_constraint = new (JPH::Allocate(sizeof(brx_physics_constraint))) brx_physics_constraint();

	physics_constraint->init(physics_world, physics_rigid_body_a->get_body_id(), physics_rigid_body_b->get_body_id(), constraint_type, pivot, twist_axis, plane_axis, normal_axis, twist_limit, plane_limit, normal_limit);

	return physics_constraint;
}

extern "C" void brx_physics_destory_constraint(brx_physics_context *, brx_physics_world *, brx_physics_constraint *physics_constraint)
{
	physics_constraint->uninit();

	physics_constraint->~brx_physics_constraint();
	JPH::Free(physics_constraint);
}

inline brx_physics_context::brx_physics_context() : m_temp_allocator(NULL), m_job_system(NULL)
{
}

inline brx_physics_context::~brx_physics_context()
{
	JPH_ASSERT(NULL == this->m_temp_allocator);
	JPH_ASSERT(NULL == this->m_job_system);
}

inline void brx_physics_context::init()
{
	JPH::RegisterTypes();

	JPH_ASSERT(NULL == this->m_temp_allocator);
	this->m_temp_allocator = new JPH::TempAllocatorImpl(g_solver_buffer_size);

	JPH_ASSERT(NULL == this->m_job_system);
	this->m_job_system = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, std::thread::hardware_concurrency() - 1);
}

inline void brx_physics_context::uninit()
{
	JPH_ASSERT(NULL != this->m_job_system);
	delete this->m_job_system;
	this->m_job_system = NULL;

	JPH_ASSERT(NULL != this->m_temp_allocator);
	delete this->m_temp_allocator;
	this->m_temp_allocator = NULL;
}

inline JPH::TempAllocator *brx_physics_context::get_temp_allocator() const
{
	return this->m_temp_allocator;
}

inline JPH::JobSystemThreadPool *brx_physics_context::get_job_system() const
{
	return this->m_job_system;
}

inline brx_physics_world::brx_physics_world() : m_object_vs_object_layer_filter(NULL), m_physics_system(NULL), m_local_time(0.0F)
{
}

inline brx_physics_world::~brx_physics_world()
{
	JPH_ASSERT(NULL == this->m_object_vs_object_layer_filter);
	JPH_ASSERT(NULL == this->m_physics_system);
}

inline void brx_physics_world::init(float const gravity[3])
{
	JPH_ASSERT(NULL == this->m_object_vs_object_layer_filter);
	this->m_object_vs_object_layer_filter = new JPH::ObjectLayerPairFilterMask();

	JPH_ASSERT(NULL == this->m_physics_system);
	this->m_physics_system = new JPH::PhysicsSystem();
	this->m_physics_system->Init(65536, 0, 65536, 10240, *this, *this, *this->m_object_vs_object_layer_filter);
	// [btManifoldResult::calculateCombinedFriction](https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/CollisionDispatch/btManifoldResult.cpp#L56)
	// [btManifoldResult::calculateCombinedRestitution](https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/CollisionDispatch/btManifoldResult.cpp#L68)
	this->m_physics_system->SetCombineFriction(
		[](const JPH::Body &inBody1, const JPH::SubShapeID &, const JPH::Body &inBody2, const JPH::SubShapeID &) -> float
		{
			float const combined_friction = JPH::min(JPH::max(0.0F, JPH::sqrt(inBody1.GetFriction() * inBody2.GetFriction())), 10.0F);
			return combined_friction;
		});
	this->m_physics_system->SetCombineRestitution(
		[](const JPH::Body &inBody1, const JPH::SubShapeID &, const JPH::Body &inBody2, const JPH::SubShapeID &)
		{
			float const combined_restitution = JPH::min(JPH::max(0.0F, JPH::sqrt(inBody1.GetRestitution() * inBody2.GetRestitution())), 1.0F);
			return combined_restitution;
		});
	JPH::PhysicsSettings physics_settings;
	// [btContactSolverInfo](https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/ConstraintSolver/btContactSolverInfo.h#L85)
	// physics_settings.mBaumgarte = 0.3F;
	// [UpdateRigidBodyWorld::__get_rigid_body_world_objects](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/operators/rigid_body.py#L506)
	// physics_settings.mNumVelocitySteps = 10U;
	// physics_settings.mNumPositionSteps = 10U;
	physics_settings.mAllowSleeping = false;

	this->m_physics_system->SetPhysicsSettings(physics_settings);
	this->m_physics_system->SetGravity(JPH::Vec3(gravity[0], gravity[1], gravity[2]));

	JPH_ASSERT(0.0F == this->m_local_time);
}

inline void brx_physics_world::uninit()
{
	JPH_ASSERT(NULL != this->m_physics_system);
	delete this->m_physics_system;
	this->m_physics_system = NULL;

	JPH_ASSERT(NULL != this->m_object_vs_object_layer_filter);
	delete this->m_object_vs_object_layer_filter;
	this->m_object_vs_object_layer_filter = NULL;
}

inline JPH::BodyID brx_physics_world::physics_world_create_body(float const wrapped_rotation[4], float const wrapped_position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE wrapped_shape_type, float const wrapped_shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE wrapped_motion_type, uint32_t wrapped_collision_filter_group, uint32_t wrapped_collision_filter_mask, float wrapped_mass, float wrapped_linear_damping, float wrapped_angular_damping, float wrapped_friction, float wrapped_restitution)
{
	JPH::Shape *shape;
	switch (wrapped_shape_type)
	{
	case BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE:
	{
		JPH_ASSERT(BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE == wrapped_shape_type);
		float const radius = wrapped_shape_size[0];
		shape = new JPH::SphereShape(radius);
	}
	break;
	case BRX_PHYSICS_RIGID_BODY_SHAPE_BOX:
	{
		JPH_ASSERT(BRX_PHYSICS_RIGID_BODY_SHAPE_BOX == wrapped_shape_type);
		JPH::Vec3 const half_extents(wrapped_shape_size[0], wrapped_shape_size[1], wrapped_shape_size[2]);

		float convex_radius;
		{
			// [CONVEX_DISTANCE_MARGIN](https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp#L20)
			constexpr float const CONVEX_DISTANCE_MARGIN = 0.04F;

			// [btConvexInternalShape::setSafeMargin](https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/CollisionShapes/btConvexInternalShape.h#L74)
			constexpr float const default_margin_multiplier = 0.1F;
			float const min_dimension = half_extents.ReduceMin();
			float const safe_margin = min_dimension * default_margin_multiplier;
			float const margin = ((CONVEX_DISTANCE_MARGIN <= safe_margin) ? CONVEX_DISTANCE_MARGIN : safe_margin);

			convex_radius = margin;
		}

		shape = new JPH::BoxShape(half_extents, convex_radius);
	}
	break;
	default:
	{
		JPH_ASSERT(BRX_PHYSICS_RIGID_BODY_SHAPE_CAPSULE == wrapped_shape_type);
		float const half_height = wrapped_shape_size[1] * 0.5F;
		float const radius = wrapped_shape_size[0];
		shape = new JPH::CapsuleShape(half_height, radius);
	}
	}

	JPH::EMotionType motion_type;
	float mass;
	switch (wrapped_motion_type)
	{
	case BRX_PHYSICS_RIGID_BODY_MOTION_FIXED:
	{
		JPH_ASSERT(BRX_PHYSICS_RIGID_BODY_MOTION_FIXED == wrapped_motion_type);
		motion_type = JPH::EMotionType::Static;
		mass = 0.0F;
	}
	break;
	case BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME:
	{

		JPH_ASSERT(BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME == wrapped_motion_type);
		motion_type = JPH::EMotionType::Kinematic;
		mass = JPH::max(wrapped_mass, 1E-6F);
	}
	break;
	default:
	{
		JPH_ASSERT(BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC == wrapped_motion_type);
		motion_type = JPH::EMotionType::Dynamic;
		mass = JPH::max(wrapped_mass, 1E-6F);
	}
	}

	JPH::Quat rotation(wrapped_rotation[0], wrapped_rotation[1], wrapped_rotation[2], wrapped_rotation[3]);

	JPH::Vec3 position(wrapped_position[0], wrapped_position[1], wrapped_position[2]);

	JPH_ASSERT(wrapped_collision_filter_group >= 0);
	JPH_ASSERT(wrapped_collision_filter_group < 16);
	JPH::ObjectLayer const collision_group_filter_layer = JPH::ObjectLayerPairFilterMask::sGetObjectLayer((1U << wrapped_collision_filter_group), wrapped_collision_filter_mask);

	JPH::BodyCreationSettings settings(shape, position, rotation, motion_type, collision_group_filter_layer);

	// CCD
	// settings.mMotionQuality = JPH::EMotionQuality::LinearCast;

	settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
	settings.mMassPropertiesOverride.mMass = mass;

	settings.mLinearDamping = wrapped_linear_damping;
	settings.mAngularDamping = wrapped_angular_damping;
	settings.mRestitution = wrapped_restitution;
	settings.mFriction = wrapped_friction;
	// TODO: how to specify Bullet Physics "addition damping"?
	// construction_info.m_additionalDamping = true;

	settings.mAllowSleeping = false;

	JPH::BodyInterface *const body_interface = &this->m_physics_system->GetBodyInterface();

	JPH::Body *body = body_interface->CreateBody(settings);

	JPH::BodyID body_id = body->GetID();
	return body_id;
}

inline void brx_physics_world::physics_world_destory_body(JPH::BodyID body_id)
{
	JPH::BodyInterface *const body_interface = &this->m_physics_system->GetBodyInterface();

	body_interface->DestroyBody(body_id);
}

inline JPH::Constraint *brx_physics_world::physics_world_create_constraint(JPH::BodyID body_id_a, JPH::BodyID body_id_b, BRX_PHYSICS_CONSTRAINT_TYPE wrapped_constraint_type, float const wrapped_pivot[3], float const wrapped_twist_axis[3], float const wrapped_plane_axis[3], float const wrapped_normal_axis[3], float const wrapped_twist_limit[2], float const wrapped_plane_limit[2], float const wrapped_normal_limit[2])
{
	constexpr float const epsilon = 1E-6F;

#if 0
	// TODO: Add system group id

	// RagdollSettings::DisableParentChildCollisions
	JPH::ObjectLayer new_object_layer_a;
	bool has_new_object_layer_a;
	{
		JPH::BodyLockInterfaceLocking const *const body_lock_interface = &this->m_physics_system->GetBodyLockInterface();

		JPH::BodyID body_ids[2] = {body_id_a, body_id_b};
		JPH::BodyLockMultiRead lock(*body_lock_interface, body_ids, 2);

		JPH::Body const *body_a = lock.GetBody(0);
		JPH::Body const *body_b = lock.GetBody(1);

		JPH::ObjectLayer const object_layer_a = body_a->GetObjectLayer();
		JPH::ObjectLayer const object_layer_b = body_b->GetObjectLayer();

		if (!m_object_vs_object_layer_filter->ShouldCollide(object_layer_a, object_layer_b))
		{
			has_new_object_layer_a = true;

			JPH::uint32 const object_layer_group_a = JPH::ObjectLayerPairFilterMask::sGetGroup(object_layer_a);
			JPH::uint32 const object_layer_mask_a = JPH::ObjectLayerPairFilterMask::sGetMask(object_layer_a);
			JPH::uint32 const object_layer_group_b = JPH::ObjectLayerPairFilterMask::sGetGroup(object_layer_b);

			JPH::uint32 const new_object_layer_mask_a = object_layer_mask_a & (~object_layer_group_b);
			new_object_layer_a = JPH::ObjectLayerPairFilterMask::sGetObjectLayer(object_layer_group_a, new_object_layer_mask_a);
		}
		else
		{
			has_new_object_layer_a = false;
		}
	}

	if (has_new_object_layer_a)
	{
		JPH::BodyInterface *const body_interface = &this->m_physics_system->GetBodyInterface();

		body_interface->SetObjectLayer(body_id_a, new_object_layer_a);
	}
#endif

	// TODO
	// RagdollSettings::Stabilize

	JPH::Constraint *constraint;
	{
		JPH::BodyLockInterfaceLocking const *const body_lock_interface = &this->m_physics_system->GetBodyLockInterface();

		JPH::BodyID body_ids[2] = {body_id_a, body_id_b};
		JPH::BodyLockMultiWrite lock(*body_lock_interface, body_ids, 2);

		JPH::Body *body_a = lock.GetBody(0);
		JPH::Body *body_b = lock.GetBody(1);

		JPH::Vec3 const pivot = JPH::Vec3(wrapped_pivot[0], wrapped_pivot[1], wrapped_pivot[2]);
		JPH::Vec3 const twist_axis(wrapped_twist_axis[0], wrapped_twist_axis[1], wrapped_twist_axis[2]);
		JPH::Vec3 const plane_axis(wrapped_plane_axis[0], wrapped_plane_axis[1], wrapped_plane_axis[2]);
		JPH::Vec3 const normal_axis(wrapped_normal_axis[0], wrapped_normal_axis[1], wrapped_normal_axis[2]);

		switch (wrapped_constraint_type)
		{
		case BRX_PHYSICS_CONSTRAINT_FIXED:
		{
			JPH_ASSERT(BRX_PHYSICS_CONSTRAINT_FIXED == wrapped_constraint_type);
			JPH::FixedConstraintSettings constraint_settings;
			constraint_settings.SetEmbedded();

			constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
			constraint_settings.mPoint1 = pivot;
			constraint_settings.mPoint2 = pivot;
			constraint_settings.mAxisX1 = twist_axis;
			constraint_settings.mAxisX2 = twist_axis;
			constraint_settings.mAxisY1 = plane_axis;
			constraint_settings.mAxisY2 = plane_axis;

			JPH_ASSERT(std::abs(wrapped_twist_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_twist_limit[1]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_plane_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_plane_limit[1]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_normal_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_normal_limit[1]) < epsilon);

			constraint = constraint_settings.Create(*body_a, *body_b);
		}
		break;
		case BRX_PHYSICS_CONSTRAINT_BALL_AND_SOCKET:
		{
			JPH_ASSERT(BRX_PHYSICS_CONSTRAINT_BALL_AND_SOCKET == wrapped_constraint_type);
			JPH::PointConstraintSettings constraint_settings;
			constraint_settings.SetEmbedded();

			constraint_settings.mPoint1 = pivot;
			constraint_settings.mPoint2 = pivot;

			JPH_ASSERT(std::abs(wrapped_twist_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_twist_limit[1]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_plane_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_plane_limit[1]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_normal_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_normal_limit[1]) < epsilon);

			constraint = constraint_settings.Create(*body_a, *body_b);
		}
		break;
		case BRX_PHYSICS_CONSTRAINT_HINGE:
		{
			JPH_ASSERT(BRX_PHYSICS_CONSTRAINT_HINGE == wrapped_constraint_type);

			JPH::HingeConstraintSettings constraint_settings;
			constraint_settings.SetEmbedded();

			constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
			constraint_settings.mPoint1 = pivot;
			constraint_settings.mPoint2 = pivot;
			constraint_settings.mHingeAxis2 = normal_axis;
			constraint_settings.mHingeAxis2 = normal_axis;
			constraint_settings.mNormalAxis1 = twist_axis;
			constraint_settings.mNormalAxis1 = twist_axis;

			JPH_ASSERT(std::abs(wrapped_twist_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_twist_limit[1]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_plane_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_plane_limit[1]) < epsilon);

			JPH_ASSERT(wrapped_normal_limit[0] < wrapped_normal_limit[1]);

			JPH_ASSERT(wrapped_normal_limit[0] >= (JPH::JPH_PI * -1.0F));
			JPH_ASSERT(wrapped_normal_limit[0] <= (JPH::JPH_PI * 1.0F));
			JPH_ASSERT(wrapped_normal_limit[1] >= (JPH::JPH_PI * -1.0F));
			JPH_ASSERT(wrapped_normal_limit[1] <= (JPH::JPH_PI * 1.0F));
			constraint_settings.mLimitsMin = JPH::min(JPH::max(JPH::JPH_PI * -1.0F, JPH::min(wrapped_normal_limit[0], wrapped_normal_limit[1])), JPH::JPH_PI * 1.0F);
			constraint_settings.mLimitsMax = JPH::min(JPH::max(JPH::JPH_PI * -1.0F, JPH::max(wrapped_normal_limit[0], wrapped_normal_limit[1])), JPH::JPH_PI * 1.0F);

			constraint = constraint_settings.Create(*body_a, *body_b);
		}
		break;
		case BRX_PHYSICS_CONSTRAINT_PRISMATIC:
		{
			JPH_ASSERT(BRX_PHYSICS_CONSTRAINT_PRISMATIC == wrapped_constraint_type);

			JPH::SliderConstraintSettings constraint_settings;
			constraint_settings.SetEmbedded();

			constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
			constraint_settings.mPoint1 = pivot;
			constraint_settings.mPoint2 = pivot;
			constraint_settings.mSliderAxis1 = normal_axis;
			constraint_settings.mSliderAxis1 = normal_axis;
			constraint_settings.mNormalAxis1 = twist_axis;
			constraint_settings.mNormalAxis1 = twist_axis;

			JPH_ASSERT(std::abs(wrapped_twist_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_twist_limit[1]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_plane_limit[0]) < epsilon);
			JPH_ASSERT(std::abs(wrapped_plane_limit[1]) < epsilon);

			JPH_ASSERT(wrapped_normal_limit[0] <= wrapped_normal_limit[1]);
			constraint_settings.mLimitsMin = JPH::min(wrapped_normal_limit[0], wrapped_normal_limit[1]);
			constraint_settings.mLimitsMax = JPH::max(wrapped_normal_limit[0], wrapped_normal_limit[1]);

			constraint = constraint_settings.Create(*body_a, *body_b);
		}
		break;
		default:
		{
			JPH_ASSERT(BRX_PHYSICS_CONSTRAINT_RAGDOLL == wrapped_constraint_type);

			JPH::SwingTwistConstraintSettings constraint_settings;
			constraint_settings.SetEmbedded();

			constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
			constraint_settings.mPosition1 = pivot;
			constraint_settings.mPosition2 = pivot;
			constraint_settings.mTwistAxis1 = twist_axis;
			constraint_settings.mTwistAxis2 = twist_axis;
			constraint_settings.mPlaneAxis1 = plane_axis;
			constraint_settings.mPlaneAxis2 = plane_axis;

			JPH_ASSERT(wrapped_twist_limit[0] <= wrapped_twist_limit[1]);
			JPH_ASSERT(wrapped_twist_limit[0] >= (JPH::JPH_PI * -1.0F));
			JPH_ASSERT(wrapped_twist_limit[0] <= (JPH::JPH_PI * 1.0F));
			JPH_ASSERT(wrapped_twist_limit[1] >= (JPH::JPH_PI * -1.0F));
			JPH_ASSERT(wrapped_twist_limit[1] <= (JPH::JPH_PI * 1.0F));
			constraint_settings.mTwistMinAngle = JPH::min(JPH::max(JPH::JPH_PI * -1.0F, JPH::min(wrapped_twist_limit[0], wrapped_twist_limit[1])), JPH::JPH_PI * 1.0F);
			constraint_settings.mTwistMaxAngle = JPH::min(JPH::max(JPH::JPH_PI * -1.0F, JPH::max(wrapped_twist_limit[0], wrapped_twist_limit[1])), JPH::JPH_PI * 1.0F);

			JPH_ASSERT(wrapped_plane_limit[0] < wrapped_plane_limit[1]);
			JPH_ASSERT(wrapped_plane_limit[0] >= (JPH::JPH_PI * -0.5F));
			JPH_ASSERT(wrapped_plane_limit[0] <= (JPH::JPH_PI * 0.5F));
			JPH_ASSERT(wrapped_plane_limit[1] >= (JPH::JPH_PI * -0.5F));
			JPH_ASSERT(wrapped_plane_limit[1] <= (JPH::JPH_PI * 0.5F));
			constraint_settings.mPlaneHalfConeAngle = JPH::min(JPH::max(JPH::abs(wrapped_plane_limit[0]), JPH::abs(wrapped_plane_limit[1])), JPH::JPH_PI * 0.5F);

			JPH_ASSERT(wrapped_normal_limit[0] < wrapped_normal_limit[1]);
			JPH_ASSERT(wrapped_normal_limit[0] >= (JPH::JPH_PI * -0.5F));
			JPH_ASSERT(wrapped_normal_limit[0] <= (JPH::JPH_PI * 0.5F));
			JPH_ASSERT(wrapped_normal_limit[1] >= (JPH::JPH_PI * -0.5F));
			JPH_ASSERT(wrapped_normal_limit[1] <= (JPH::JPH_PI * 0.5F));

			constraint_settings.mNormalHalfConeAngle = JPH::min(JPH::max(JPH::abs(wrapped_normal_limit[0]), JPH::abs(wrapped_normal_limit[1])), JPH::JPH_PI * 0.5F);

			JPH_ASSERT(JPH::max(JPH::abs(constraint_settings.mTwistMinAngle), JPH::abs(constraint_settings.mTwistMaxAngle)) <= constraint_settings.mPlaneHalfConeAngle);
			JPH_ASSERT(JPH::max(JPH::abs(constraint_settings.mTwistMinAngle), JPH::abs(constraint_settings.mTwistMaxAngle)) <= constraint_settings.mNormalHalfConeAngle);

			constraint = constraint_settings.Create(*body_a, *body_b);
		}
		}
	}

	return constraint;
}

inline void brx_physics_world::physics_world_add_body(JPH::BodyID body_id)
{
	JPH::BodyInterface *const body_interface = &this->m_physics_system->GetBodyInterface();

	body_interface->AddBody(body_id, JPH::EActivation::Activate);

	this->m_physics_system->OptimizeBroadPhase();
}

inline void brx_physics_world::physics_world_remove_body(JPH::BodyID body_id)
{
	JPH::BodyInterface *const body_interface = &this->m_physics_system->GetBodyInterface();

	body_interface->RemoveBody(body_id);

	this->m_physics_system->OptimizeBroadPhase();
}

inline void brx_physics_world::physics_world_add_constraint(JPH::Constraint *physics_constraint)
{
	this->m_physics_system->AddConstraint(physics_constraint);
}

inline void brx_physics_world::physics_world_remove_constraint(JPH::Constraint *physics_constraint)
{
	this->m_physics_system->RemoveConstraint(physics_constraint);
}

inline void brx_physics_world::physics_world_step(brx_physics_context *physics_context, float time_step)
{
	// [UpdateRigidBodyWorld::__get_rigid_body_world_objects](https://github.com/MMD-Blender/blender_mmd_tools/blob/main/mmd_tools/operators/rigid_body.py#L506)

	// [btDiscreteDynamicsWorld::stepSimulation](https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp#L382)
	constexpr int const max_sub_steps = 10;
	constexpr float const fixed_time_step = 1.0F / 120.0F;

	int clamped_simulation_steps;
	{
		JPH_ASSERT(this->m_local_time < fixed_time_step);

		this->m_local_time += time_step;

		int num_simulation_sub_steps;
		if (this->m_local_time >= fixed_time_step)
		{
			num_simulation_sub_steps = int(this->m_local_time / fixed_time_step);
			this->m_local_time -= num_simulation_sub_steps * fixed_time_step;
		}
		else
		{
			num_simulation_sub_steps = 0;
		}

		clamped_simulation_steps = (num_simulation_sub_steps < max_sub_steps) ? num_simulation_sub_steps : max_sub_steps;
	}

	JPH::EPhysicsUpdateError physics_update_error = this->m_physics_system->Update(fixed_time_step * clamped_simulation_steps, clamped_simulation_steps, physics_context->get_temp_allocator(), physics_context->get_job_system());
	JPH_ASSERT(JPH::EPhysicsUpdateError::None == physics_update_error);
}

inline void brx_physics_world::physics_world_apply_body_key_frame(JPH::BodyID body_id, float const wrapped_rotation[4], float const wrapped_position[3], float delta_time)
{
	JPH::BodyInterface *const body_interface = &this->m_physics_system->GetBodyInterface();

	JPH::Quat rotation(wrapped_rotation[0], wrapped_rotation[1], wrapped_rotation[2], wrapped_rotation[3]);

	JPH::Vec3 position(wrapped_position[0], wrapped_position[1], wrapped_position[2]);

	body_interface->MoveKinematic(body_id, position, rotation, delta_time);
}

inline void brx_physics_world::physics_world_get_body_transform(JPH::BodyID body_id, float out_rotation[4], float out_position[3]) const
{
	JPH::BodyLockInterfaceLocking const *const body_lock_interface = &this->m_physics_system->GetBodyLockInterface();

	JPH::BodyLockRead lock(*body_lock_interface, body_id);

	JPH_ASSERT(lock.Succeeded());

	JPH::Body const *body = &lock.GetBody();

	JPH::RMat44 world_transform = body->GetWorldTransform();

	JPH::Float4 world_rotation;
	world_transform.GetQuaternion().mValue.StoreFloat4(&world_rotation);

	out_rotation[0] = world_rotation.x;
	out_rotation[1] = world_rotation.y;
	out_rotation[2] = world_rotation.z;
	out_rotation[3] = world_rotation.w;

	JPH::Float3 world_translation;
	world_transform.GetTranslation().StoreFloat3(&world_translation);

	out_position[0] = world_translation.x;
	out_position[1] = world_translation.y;
	out_position[2] = world_translation.z;
}

JPH::uint brx_physics_world::GetNumBroadPhaseLayers() const
{
	return 1U;
}

JPH::BroadPhaseLayer brx_physics_world::GetBroadPhaseLayer(JPH::ObjectLayer) const
{
	return JPH::BroadPhaseLayer(0U);
}

bool brx_physics_world::ShouldCollide(JPH::ObjectLayer, JPH::BroadPhaseLayer) const
{
	return true;
}

inline brx_physics_rigid_body::brx_physics_rigid_body() : m_body_id(JPH::BodyID::cInvalidBodyID)
{
}

inline brx_physics_rigid_body::~brx_physics_rigid_body()
{
	JPH_ASSERT(this->m_body_id.IsInvalid());
}

inline void brx_physics_rigid_body::init(brx_physics_world *physics_world, float const rotation[4], float const position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type, float const shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type, uint32_t collision_filter_group, uint32_t collision_filter_mask, float mass, float linear_damping, float angular_damping, float friction, float restitution)
{
	JPH_ASSERT(this->m_body_id.IsInvalid());
	this->m_body_id = physics_world->physics_world_create_body(rotation, position, shape_type, shape_size, motion_type, collision_filter_group, collision_filter_mask, mass, linear_damping, angular_damping, friction, restitution);
}

inline void brx_physics_rigid_body::uninit(brx_physics_world *physics_world)
{
	JPH_ASSERT(!this->m_body_id.IsInvalid());
	physics_world->physics_world_destory_body(this->m_body_id);
	this->m_body_id = JPH::BodyID();
}

inline JPH::BodyID brx_physics_rigid_body::get_body_id() const
{
	return this->m_body_id;
}

inline brx_physics_constraint::brx_physics_constraint() : m_constraint(nullptr)
{
}

inline brx_physics_constraint::~brx_physics_constraint()
{
	JPH_ASSERT(nullptr == this->m_constraint.GetPtr());
}

inline void brx_physics_constraint::init(brx_physics_world *physics_world, JPH::BodyID body_id_a, JPH::BodyID body_id_b, BRX_PHYSICS_CONSTRAINT_TYPE constraint_type, float const pivot[3], float const twist_axis[3], float const plane_axis[3], float const normal_axis[3], float const twist_limit[2], float const plane_limit[2], float const normal_limit[2])
{
	this->m_constraint = physics_world->physics_world_create_constraint(body_id_a, body_id_b, constraint_type, pivot, twist_axis, plane_axis, normal_axis, twist_limit, plane_limit, normal_limit);
}

inline void brx_physics_constraint::uninit()
{
	JPH_ASSERT(nullptr != this->m_constraint.GetPtr());
	this->m_constraint = nullptr;
}

inline JPH::Constraint *brx_physics_constraint::get_constraint() const
{
	return this->m_constraint;
}

#include "../../McRT-Malloc/include/mcrt_malloc.h"

JPH_NAMESPACE_BEGIN

static void *AllocateHook(size_t inSize)
{
	return mcrt_malloc(inSize, 16U);
}

static void *ReallocateHook(void *inOldBlock, size_t inOldSize, size_t inNewSize)
{
	if ((NULL != inOldBlock) && (0U < inNewSize))
	{
		void *inNewBlock = mcrt_malloc(inNewSize, 16U);
		if (NULL != inNewBlock)
		{
			std::memcpy(inNewBlock, inOldBlock, ((inOldSize < inNewSize) ? inOldSize : inNewSize));
			mcrt_free(inOldBlock);
		}
		return inNewBlock;
	}
	else if (0U < inNewSize)
	{
		return mcrt_malloc(inNewSize, 16U);
	}
	else if (NULL != inOldBlock)
	{
		mcrt_free(inOldBlock);
		return NULL;
	}
	else
	{
		return NULL;
	}
}

static void FreeHook(void *inBlock)
{
	return mcrt_free(inBlock);
}

static void *AlignedAllocateHook(size_t inSize, size_t inAlignment)
{
	return mcrt_malloc(inSize, inAlignment);
}

static void AlignedFreeHook(void *inBlock)
{
	return mcrt_free(inBlock);
}

AllocateFunction Allocate = AllocateHook;
ReallocateFunction Reallocate = ReallocateHook;
FreeFunction Free = FreeHook;
AlignedAllocateFunction AlignedAllocate = AlignedAllocateHook;
AlignedFreeFunction AlignedFree = AlignedFreeHook;

JPH_NAMESPACE_END
