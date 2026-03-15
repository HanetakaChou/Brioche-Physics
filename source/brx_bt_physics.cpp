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

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>

static void *_internal_aligned_alloc(size_t size, int alignment);

static void _internal_aligned_free(void *ptr);

static constexpr float const INTERNAL_KINEMATIC_MASS = 1E12F;
static constexpr float const INTERNAL_KINEMATIC_MASS_THRESHOLD = 1E6F;
static constexpr float const INTERNAL_ZERO_THRESHOLD = 1E-6F;

struct brx_overlap_filter_callback : btOverlapFilterCallback
{
	bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const override
	{
		// btHashedOverlappingPairCache::needsBroadphaseCollision
		btCollisionObject const *const obj0 = static_cast<btCollisionObject *>(proxy0->m_clientObject);
		btCollisionObject const *const obj1 = static_cast<btCollisionObject *>(proxy1->m_clientObject);

		btRigidBody const *const body0 = btRigidBody::upcast(obj0);
		btRigidBody const *const body1 = btRigidBody::upcast(obj1);

		bool const kinematic0 = ((NULL != body0) && (body0->getMass() > INTERNAL_KINEMATIC_MASS_THRESHOLD) && (0 != (BT_DISABLE_WORLD_GRAVITY & body0->getFlags())));
		bool const kinematic1 = ((NULL != body1) && (body1->getMass() > INTERNAL_KINEMATIC_MASS_THRESHOLD) && (0 != (BT_DISABLE_WORLD_GRAVITY & body1->getFlags())));
		if ((!kinematic0) || (!kinematic1))
		{
			bool const collision01 = (0 != (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask)) && (0 != (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask));
			return collision01;
		}
		else
		{
			return false;
		}
	}
};

static brx_overlap_filter_callback internal_overlap_filter_callback;

class brx_physics_context : public btITaskScheduler
{
public:
	inline brx_physics_context();
	inline ~brx_physics_context();
	inline void init();
	inline void uninit();

private:
	int getNumThreads() const override;
	int getCurrentThreadIndex() const override;
	void parallelFor(int iBegin, int iEnd, int grainSize, btIParallelForBody const &body) override;
	btScalar parallelSum(int iBegin, int iEnd, int grainSize, btIParallelSumBody const &body) override;
};

class brx_physics_world
{
	btDbvtBroadphase *m_broad_phase;
	btDefaultCollisionConfiguration *m_collision_configuration;
	btCollisionDispatcherMt *m_collision_dispatcher;
	btConstraintSolverPoolMt *m_constraint_solver_pool;
	btSequentialImpulseConstraintSolverMt *m_constraint_solver;
	btDiscreteDynamicsWorldMt *m_dynamics_world;

public:
	inline brx_physics_world();
	inline ~brx_physics_world();
	inline void init(float const gravity[3]);
	inline void uninit();
	inline void physics_world_add_body(btRigidBody *rigid_body, int collision_filter_group, int collision_filter_mask);
	inline void physics_world_remove_body(btRigidBody *rigid_body);
	inline void physics_world_add_constraint(btTypedConstraint *physics_constraint);
	inline void physics_world_remove_constraint(btTypedConstraint *physics_constraint);
	inline void physics_world_step(float delta_time, uint32_t max_substep_count, float substep_delta_time);
	inline float get_local_time() const;
	inline float get_fixed_time_step() const;
};

class brx_physics_rigid_body
{
	btCollisionShape *m_collision_shape;
	btRigidBody *m_rigid_body;
	int m_collision_filter_group;
	int m_collision_filter_mask;

public:
	inline brx_physics_rigid_body();
	inline ~brx_physics_rigid_body();
	inline void init(brx_physics_world *physics_world, float const rotation[4], float const position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type, float const shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type, uint32_t collision_filter_group, uint32_t collision_filter_mask, float mass, float linear_damping, float angular_damping, float friction, float restitution);
	inline void uninit(brx_physics_world *physics_world);
	inline btRigidBody *get_rigid_body() const;
	inline int get_collision_filter_group() const;
	inline int get_collision_filter_mask() const;
	inline void apply_key_frame(float const local_time, float const rotation[4], float const position[3], float delta_time, uint32_t max_substep_count, float substep_delta_time);
	inline void get_body_transform(float out_rotation[4], float out_position[3]) const;
};

class brx_physics_constraint
{
	btTypedConstraint *m_constraint;

public:
	inline brx_physics_constraint();
	inline ~brx_physics_constraint();
	inline void init(brx_physics_world *physics_world, btRigidBody *rigid_body_a, btRigidBody *rigid_body_b, BRX_PHYSICS_CONSTRAINT_TYPE constraint_type, float const pivot[3], float const twist_axis[3], float const plane_axis[3], float const normal_axis[3], float const twist_limit[2], float const plane_limit[2], float const normal_limit[2]);
	inline void uninit();
	inline btTypedConstraint *get_constraint() const;
};

extern "C" brx_physics_context *brx_physics_create_context()
{
	btAlignedAllocSetCustomAligned(_internal_aligned_alloc, _internal_aligned_free);

	brx_physics_context *physics_context = new (btAlignedAlloc(sizeof(brx_physics_context), alignof(brx_physics_context))) brx_physics_context();

	physics_context->init();

	return physics_context;
}

extern "C" void brx_physics_destroy_context(brx_physics_context *physics_context)
{
	physics_context->uninit();

	physics_context->~brx_physics_context();
	btAlignedFree(physics_context);

#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	int num_bytes_leaked = btDumpMemoryLeaks();
	btAssert(0 == num_bytes_leaked);
#endif
}

extern "C" brx_physics_world *brx_physics_create_world(brx_physics_context *physics_context, float const gravity[3])
{
	brx_physics_world *physics_world = new (btAlignedAlloc(sizeof(brx_physics_world), alignof(brx_physics_world))) brx_physics_world();

	physics_world->init(gravity);

	return physics_world;
}

extern "C" void brx_physics_destroy_world(brx_physics_context *physics_context, brx_physics_world *physics_world)
{
	physics_world->uninit();

	physics_world->~brx_physics_world();
	btAlignedFree(physics_world);
}

extern "C" void brx_physics_world_add_rigid_body(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body)
{
	physics_world->physics_world_add_body(physics_rigid_body->get_rigid_body(), physics_rigid_body->get_collision_filter_group(), physics_rigid_body->get_collision_filter_mask());
}

extern "C" void brx_physics_world_remove_rigid_body(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body)
{
	physics_world->physics_world_remove_body(physics_rigid_body->get_rigid_body());
}

extern "C" void brx_physics_world_add_constraint(brx_physics_context *, brx_physics_world *physics_world, brx_physics_constraint *physics_constraint)
{
	physics_world->physics_world_add_constraint(physics_constraint->get_constraint());
}

extern "C" void brx_physics_world_remove_constraint(brx_physics_context *, brx_physics_world *physics_world, brx_physics_constraint *physics_constraint)
{
	physics_world->physics_world_remove_constraint(physics_constraint->get_constraint());
}

extern "C" void brx_physics_world_step(brx_physics_context *, brx_physics_world *physics_world, float delta_time, uint32_t max_substep_count, float substep_delta_time)
{
	physics_world->physics_world_step(delta_time, max_substep_count, substep_delta_time);
}

extern "C" brx_physics_rigid_body *brx_physics_create_rigid_body(brx_physics_context *physics_context, brx_physics_world *physics_world, float const rotation[4], float const position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type, float const shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type, uint32_t collision_filter_group, uint32_t collision_filter_mask, float mass, float linear_damping, float angular_damping, float friction, float restitution)
{
	brx_physics_rigid_body *physics_rigid_body = new (btAlignedAlloc(sizeof(brx_physics_rigid_body), alignof(brx_physics_rigid_body))) brx_physics_rigid_body();

	physics_rigid_body->init(physics_world, rotation, position, shape_type, shape_size, motion_type, collision_filter_group, collision_filter_mask, mass, linear_damping, angular_damping, friction, restitution);

	return physics_rigid_body;
}

extern "C" void brx_physics_destroy_rigid_body(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body)
{
	physics_rigid_body->uninit(physics_world);

	physics_rigid_body->~brx_physics_rigid_body();
	btAlignedFree(physics_rigid_body);
}

extern "C" void brx_physics_rigid_body_apply_key_frame(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body, float const rotation[4], float const position[3], float delta_time, uint32_t max_substep_count, float substep_delta_time)
{
	btAssert((physics_world->get_fixed_time_step() == 0.0F) || (physics_world->get_fixed_time_step() == substep_delta_time));
	physics_rigid_body->apply_key_frame(physics_world->get_local_time(), rotation, position, delta_time, max_substep_count, substep_delta_time);
}

extern "C" void brx_physics_rigid_body_get_transform(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body, float out_rotation[4], float out_position[3])
{
	physics_rigid_body->get_body_transform(out_rotation, out_position);
}

extern "C" brx_physics_constraint *brx_physics_create_constraint(brx_physics_context *, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body_reference, brx_physics_rigid_body *physics_rigid_body_attached, BRX_PHYSICS_CONSTRAINT_TYPE constraint_type, float const pivot[3], float const twist_axis[3], float const plane_axis[3], float const normal_axis[3], float const twist_limit[2], float const plane_limit[2], float const normal_limit[2])
{
	brx_physics_constraint *physics_constraint = new (btAlignedAlloc(sizeof(brx_physics_constraint), alignof(brx_physics_constraint))) brx_physics_constraint();

	physics_constraint->init(physics_world, physics_rigid_body_reference->get_rigid_body(), physics_rigid_body_attached->get_rigid_body(), constraint_type, pivot, twist_axis, plane_axis, normal_axis, twist_limit, plane_limit, normal_limit);

	return physics_constraint;
}

extern "C" void brx_physics_destroy_constraint(brx_physics_context *, brx_physics_world *, brx_physics_constraint *physics_constraint)
{
	physics_constraint->uninit();

	physics_constraint->~brx_physics_constraint();
	btAlignedFree(physics_constraint);
}

inline brx_physics_context::brx_physics_context() : btITaskScheduler("McRT-Malloc")
{
}

inline brx_physics_context::~brx_physics_context()
{
}

inline void brx_physics_context::init()
{
	btSetTaskScheduler(this);
}

inline void brx_physics_context::uninit()
{
}

inline brx_physics_world::brx_physics_world() : m_broad_phase(NULL), m_collision_configuration(NULL), m_collision_dispatcher(NULL), m_constraint_solver_pool(NULL), m_constraint_solver(NULL), m_dynamics_world(NULL)
{
}

inline brx_physics_world::~brx_physics_world()
{
	btAssert(NULL == this->m_broad_phase);
	btAssert(NULL == this->m_collision_configuration);
	btAssert(NULL == this->m_collision_dispatcher);
	btAssert(NULL == this->m_constraint_solver_pool);
	btAssert(NULL == this->m_constraint_solver);
	btAssert(NULL == this->m_dynamics_world);
}

inline void brx_physics_world::init(float const wrapped_gravity[3])
{
	btAssert(NULL == this->m_broad_phase);
	this->m_broad_phase = new btDbvtBroadphase();

	btAssert(NULL == this->m_collision_configuration);
	btDefaultCollisionConstructionInfo construction_info;
	construction_info.m_defaultMaxPersistentManifoldPoolSize = 65536;
	construction_info.m_defaultMaxCollisionAlgorithmPoolSize = 65536;
	this->m_collision_configuration = new btDefaultCollisionConfiguration(construction_info);

	btAssert(NULL == this->m_collision_dispatcher);
	this->m_collision_dispatcher = new btCollisionDispatcherMt(this->m_collision_configuration);

	btAssert(NULL == this->m_constraint_solver_pool);
	{
		btConstraintSolver *solvers[BT_MAX_THREAD_COUNT];
		for (uint32_t thread_index = 0U; thread_index < BT_MAX_THREAD_COUNT; ++thread_index)
		{
			// pool solvers shouldn't be parallel solvers
			// we don't allow that kind of nested parallelism because of performance issues
			solvers[thread_index] = new btSequentialImpulseConstraintSolver();
		}
		this->m_constraint_solver_pool = new btConstraintSolverPoolMt(solvers, BT_MAX_THREAD_COUNT);
	}

	btAssert(NULL == this->m_constraint_solver);
	this->m_constraint_solver = new btSequentialImpulseConstraintSolverMt();

	btAssert(NULL == this->m_dynamics_world);
	this->m_dynamics_world = new btDiscreteDynamicsWorldMt(this->m_collision_dispatcher, this->m_broad_phase, this->m_constraint_solver_pool, this->m_constraint_solver, this->m_collision_configuration);

	this->m_dynamics_world->getPairCache()->setOverlapFilterCallback(&internal_overlap_filter_callback);

	this->m_dynamics_world->setGravity(btVector3(wrapped_gravity[0], wrapped_gravity[1], wrapped_gravity[2]));

	btAssert(this->m_dynamics_world->getDispatchInfo().m_useContinuous);

	gDisableDeactivation = true;
}

inline void brx_physics_world::uninit()
{
	btAssert(NULL != this->m_dynamics_world);
	delete this->m_dynamics_world;
	this->m_dynamics_world = NULL;

	btAssert(NULL != this->m_constraint_solver);
	delete this->m_constraint_solver;
	this->m_constraint_solver = NULL;

	btAssert(NULL != this->m_constraint_solver_pool);
	delete this->m_constraint_solver_pool;
	this->m_constraint_solver_pool = NULL;

	btAssert(NULL != this->m_collision_dispatcher);
	delete this->m_collision_dispatcher;
	this->m_collision_dispatcher = NULL;

	btAssert(NULL != this->m_collision_configuration);
	delete this->m_collision_configuration;
	this->m_collision_configuration = NULL;

	btAssert(NULL != this->m_broad_phase);
	delete this->m_broad_phase;
	this->m_broad_phase = NULL;
}

inline void brx_physics_world::physics_world_add_body(btRigidBody *rigid_body, int collision_filter_group, int collision_filter_mask)
{
	rigid_body->getGravity();

	this->m_dynamics_world->addRigidBody(rigid_body, collision_filter_group, collision_filter_mask);
}

inline void brx_physics_world::physics_world_remove_body(btRigidBody *rigid_body)
{
	this->m_dynamics_world->removeRigidBody(rigid_body);
}

inline void brx_physics_world::physics_world_add_constraint(btTypedConstraint *physics_constraint)
{
	// btHashedOverlappingPairCache::needsBroadphaseCollision
	btRigidBody const &body0 = physics_constraint->getRigidBodyA();
	btRigidBody const &body1 = physics_constraint->getRigidBodyB();

	btBroadphaseProxy const *const proxy0 = body0.getBroadphaseProxy();
	btBroadphaseProxy const *const proxy1 = body1.getBroadphaseProxy();

	bool const kinematic0 = ((body0.getMass() > INTERNAL_KINEMATIC_MASS_THRESHOLD) && (0 != (BT_DISABLE_WORLD_GRAVITY & body0.getFlags())));
	bool const kinematic1 = ((body1.getMass() > INTERNAL_KINEMATIC_MASS_THRESHOLD) && (0 != (BT_DISABLE_WORLD_GRAVITY & body1.getFlags())));
	if ((!kinematic0) || (!kinematic1))
	{
		bool const collision01 = (0 != (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask)) && (0 != (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask));
		if ((!kinematic0) && (!kinematic1))
		{
			// TODO: usually we do not need?
			btAssert(!collision01);
			this->m_dynamics_world->addConstraint(physics_constraint, true);
		}
		else
		{
			this->m_dynamics_world->addConstraint(physics_constraint, (!collision01));
		}
	}
	else
	{
		// no constaint between kinematic and kinematic
		btAssert(false);
		this->m_dynamics_world->addConstraint(physics_constraint, true);
	}
}

inline void brx_physics_world::physics_world_remove_constraint(btTypedConstraint *physics_constraint)
{
	this->m_dynamics_world->removeConstraint(physics_constraint);
}

inline void brx_physics_world::physics_world_step(float delta_time, uint32_t max_substep_count, float substep_delta_time)
{
	this->m_dynamics_world->stepSimulation(delta_time, max_substep_count, substep_delta_time);
}

inline float brx_physics_world::get_local_time() const
{
	return this->m_dynamics_world->m_localTime;
}

inline float brx_physics_world::get_fixed_time_step() const
{
	return this->m_dynamics_world->m_fixedTimeStep;
}

inline brx_physics_rigid_body::brx_physics_rigid_body() : m_collision_shape(NULL), m_rigid_body(NULL), m_collision_filter_group(0), m_collision_filter_mask(0)
{
}

inline brx_physics_rigid_body::~brx_physics_rigid_body()
{
	btAssert(NULL == this->m_collision_shape);
	btAssert(NULL == this->m_rigid_body);
}

inline void brx_physics_rigid_body::init(brx_physics_world *physics_world, float const wrapped_rotation[4], float const wrapped_position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE wrapped_shape_type, float const wrapped_shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE wrapped_motion_type, uint32_t wrapped_collision_filter_group, uint32_t wrapped_collision_filter_mask, float wrapped_mass, float wrapped_linear_damping, float wrapped_angular_damping, float wrapped_friction, float wrapped_restitution)
{
	btAssert(NULL == this->m_collision_shape);

	btScalar min_dimension;
	btScalar max_dimension;
	switch (wrapped_shape_type)
	{
	case BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE:
	{
		btAssert(BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE == wrapped_shape_type);
		btScalar const radius = wrapped_shape_size[0];
		this->m_collision_shape = new btSphereShape(radius);
		min_dimension = radius;
		max_dimension = radius;
	}
	break;
	case BRX_PHYSICS_RIGID_BODY_SHAPE_BOX:
	{
		btAssert(BRX_PHYSICS_RIGID_BODY_SHAPE_BOX == wrapped_shape_type);
		btVector3 const half_extents(wrapped_shape_size[0] * 0.5F, wrapped_shape_size[1] * 0.5F, wrapped_shape_size[2] * 0.5F);
		this->m_collision_shape = new btBoxShape(half_extents);
		min_dimension = btMin(btMin(half_extents.getX(), half_extents.getY()), half_extents.getZ());
		max_dimension = btMax(btMax(half_extents.getX(), half_extents.getY()), half_extents.getZ());
	}
	break;
	default:
	{
		float const radius = wrapped_shape_size[0];
		float const height = wrapped_shape_size[1];
		this->m_collision_shape = new btCapsuleShape(radius, height);
		min_dimension = btMin(radius, radius + height * 0.5F);
		max_dimension = btMax(radius, radius + height * 0.5F);
	}
	}

	int collision_flags;
	int rigid_body_flags;
	btScalar mass;
	btScalar linear_damping;
	btScalar angular_damping;
	btScalar friction;
	btScalar restitution;
	bool additional_damping;
	btVector3 linear_factor;
	btVector3 angular_factor;
	switch (wrapped_motion_type)
	{
	case BRX_PHYSICS_RIGID_BODY_MOTION_FIXED:
	{
		btAssert(BRX_PHYSICS_RIGID_BODY_MOTION_FIXED == wrapped_motion_type);
		collision_flags = btCollisionObject::CF_STATIC_OBJECT;
		rigid_body_flags = 0;
		mass = 0.0F;
		linear_damping = 0.0F;
		angular_damping = 0.0F;
		friction = wrapped_friction;
		restitution = wrapped_restitution;
		additional_damping = false;
		linear_factor = btVector3(1.0F, 1.0F, 1.0F);
		angular_factor = btVector3(1.0F, 1.0F, 1.0F);
	}
	break;
	case BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME:
	{
		btAssert(BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME == wrapped_motion_type);
		// To support CCD, we use really large mass dynamic object to simulate kinematic object
		collision_flags = btCollisionObject::CF_DYNAMIC_OBJECT;
		rigid_body_flags = BT_DISABLE_WORLD_GRAVITY;
		mass = INTERNAL_KINEMATIC_MASS;
		linear_damping = 0.0F;
		angular_damping = 0.0F;
		friction = wrapped_friction;
		restitution = wrapped_restitution;
		additional_damping = false;
		linear_factor = btVector3(0.0F, 0.0F, 0.0F);
		angular_factor = btVector3(0.0F, 0.0F, 0.0F);
	}
	break;
	default:
	{
		btAssert(BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC == wrapped_motion_type);
		collision_flags = btCollisionObject::CF_DYNAMIC_OBJECT;
		rigid_body_flags = 0;
		mass = btMin(btMax(wrapped_mass, INTERNAL_ZERO_THRESHOLD), INTERNAL_KINEMATIC_MASS_THRESHOLD);
		linear_damping = wrapped_linear_damping;
		angular_damping = wrapped_angular_damping;
		friction = wrapped_friction;
		restitution = wrapped_restitution;
		additional_damping = true;
		linear_factor = btVector3(1.0F, 1.0F, 1.0F);
		angular_factor = btVector3(1.0F, 1.0F, 1.0F);
	}
	}

	btVector3 local_inertia;
	if (mass >= INTERNAL_ZERO_THRESHOLD)
	{
		this->m_collision_shape->calculateLocalInertia(mass, local_inertia);
	}
	else
	{
		local_inertia.setZero();
	}

	btAssert(btFabs(btQuaternion(wrapped_rotation[0], wrapped_rotation[1], wrapped_rotation[2], wrapped_rotation[3]).length() - 1.0F) < INTERNAL_ZERO_THRESHOLD);
	btTransform const transform(btQuaternion(wrapped_rotation[0], wrapped_rotation[1], wrapped_rotation[2], wrapped_rotation[3]).normalized(), btVector3(wrapped_position[0], wrapped_position[1], wrapped_position[2]));

	btAssert(NULL == this->m_rigid_body);

	btRigidBody::btRigidBodyConstructionInfo construction_info(mass, NULL, this->m_collision_shape, local_inertia);
	construction_info.m_startWorldTransform = transform;
	construction_info.m_linearDamping = linear_damping;
	construction_info.m_angularDamping = angular_damping;
	construction_info.m_friction = friction;
	construction_info.m_restitution = restitution;
	// TODO: how to specify addition damping on JoltPhysics?
	construction_info.m_additionalDamping = additional_damping;

	this->m_rigid_body = new btRigidBody(construction_info);

	this->m_rigid_body->setLinearFactor(linear_factor);
	this->m_rigid_body->setAngularFactor(angular_factor);

	this->m_rigid_body->setCollisionFlags(this->m_rigid_body->getCollisionFlags() | collision_flags);
	this->m_rigid_body->setFlags(this->m_rigid_body->getFlags() | rigid_body_flags);

	btAssert(gDisableDeactivation);
	// TODO: why we still need to set these even if the "gDisableDeactivation" is set to true?
	this->m_rigid_body->setSleepingThresholds(1E-2F, btRadians(1E-1F));
	this->m_rigid_body->setActivationState(DISABLE_DEACTIVATION);

	btScalar const ccd_swept_sphere_radius = btMax(INTERNAL_ZERO_THRESHOLD, min_dimension * 0.75F);
	btScalar const ccd_motion_threshold = btMax(INTERNAL_ZERO_THRESHOLD, min_dimension * 0.25F);
	this->m_rigid_body->setCcdSweptSphereRadius(ccd_swept_sphere_radius);
	this->m_rigid_body->setCcdMotionThreshold(ccd_motion_threshold);

	btAssert(0 == this->m_collision_filter_group);
	btAssert(wrapped_collision_filter_group >= 0);
	btAssert(wrapped_collision_filter_group < 16);
	this->m_collision_filter_group = (1U << wrapped_collision_filter_group);
	btAssert(0U == (static_cast<uint32_t>(this->m_collision_filter_group) & (~0XFFFFU)));

	btAssert(0 == this->m_collision_filter_mask);
	this->m_collision_filter_mask = wrapped_collision_filter_mask;
	btAssert(0U == (static_cast<uint32_t>(this->m_collision_filter_mask) & (~0XFFFFU)));
}

inline void brx_physics_rigid_body::uninit(brx_physics_world *physics_world)
{
	btAssert(0U == this->m_rigid_body->getNumConstraintRefs());
	btAssert(!this->m_rigid_body->isInWorld());

	btAssert(NULL != this->m_rigid_body);
	delete this->m_rigid_body;
	this->m_rigid_body = NULL;

	btAssert(NULL != this->m_collision_shape);
	delete this->m_collision_shape;
	this->m_collision_shape = NULL;
}

inline btRigidBody *brx_physics_rigid_body::get_rigid_body() const
{
	return this->m_rigid_body;
}

inline int brx_physics_rigid_body::get_collision_filter_group() const
{
	return this->m_collision_filter_group;
}

inline int brx_physics_rigid_body::get_collision_filter_mask() const
{
	return this->m_collision_filter_mask;
}

inline void brx_physics_rigid_body::apply_key_frame(float const world_local_time, float const wrapped_rotation[4], float const wrapped_position[3], float delta_time, uint32_t max_substep_count, float substep_delta_time)
{
	btAssert(this->m_rigid_body->getMass() > INTERNAL_KINEMATIC_MASS_THRESHOLD);
	btAssert(0 != (BT_DISABLE_WORLD_GRAVITY & this->m_rigid_body->getFlags()));

	// [Body::MoveKinematic](https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Physics/Body/Body.cpp#L81)
	// [MotionProperties::MoveKinematic](https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Physics/Body/MotionProperties.inl#L9)

	// btDiscreteDynamicsWorld::integrateTransformsInternal
	// btKinematicCharacterController::stepUp

	// btDiscreteDynamicsWorld::stepSimulation
	float timp_step;
	{
		float const fixed_time_step = substep_delta_time;
		int const max_sub_steps = max_substep_count;

		int num_simulation_sub_steps;
		{
			float const local_time = world_local_time + delta_time;
			if (local_time >= fixed_time_step)
			{
				num_simulation_sub_steps = int(local_time / fixed_time_step);
				btAssert(local_time >= (num_simulation_sub_steps * fixed_time_step));
			}
			else
			{
				num_simulation_sub_steps = 0;
			}
		}

		if (num_simulation_sub_steps > 0)
		{
			btAssert(num_simulation_sub_steps <= max_sub_steps);
			int clamped_simulation_steps = (num_simulation_sub_steps < max_sub_steps) ? num_simulation_sub_steps : max_sub_steps;

			timp_step = fixed_time_step * clamped_simulation_steps;
		}
		else
		{
			timp_step = -1.0F;
		}
	}

	// btRigidBody::saveKinematicState
	if (timp_step > 0.0F)
	{
		btAssert(btFabs(btQuaternion(wrapped_rotation[0], wrapped_rotation[1], wrapped_rotation[2], wrapped_rotation[3]).length() - 1.0F) < INTERNAL_ZERO_THRESHOLD);
		btQuaternion const target_rotation = btQuaternion(wrapped_rotation[0], wrapped_rotation[1], wrapped_rotation[2], wrapped_rotation[3]).normalized();
		btVector3 const target_position(wrapped_position[0], wrapped_position[1], wrapped_position[2]);
		btTransform const target_transform(target_rotation, target_position);

		btTransform current_transform = this->m_rigid_body->getWorldTransform();

		btVector3 linear_velocity;
		btVector3 angular_velocity;
		btTransformUtil::calculateVelocity(current_transform, target_transform, timp_step, linear_velocity, angular_velocity);

		this->m_rigid_body->clearForces();
		this->m_rigid_body->setAngularVelocity(angular_velocity);
		this->m_rigid_body->setLinearVelocity(linear_velocity);
		this->m_rigid_body->activate(true);
	}
}

inline void brx_physics_rigid_body::get_body_transform(float out_rotation[4], float out_position[3]) const
{
	btTransform transform = this->m_rigid_body->getWorldTransform();

	btQuaternion rotation = transform.getRotation();
	out_rotation[0] = rotation.getX();
	out_rotation[1] = rotation.getY();
	out_rotation[2] = rotation.getZ();
	out_rotation[3] = rotation.getW();

	btVector3 translation = transform.getOrigin();
	out_position[0] = translation.getX();
	out_position[1] = translation.getY();
	out_position[2] = translation.getZ();
}

inline brx_physics_constraint::brx_physics_constraint() : m_constraint(NULL)
{
}

inline brx_physics_constraint::~brx_physics_constraint()
{
	btAssert(NULL == this->m_constraint);
}

inline void brx_physics_constraint::init(brx_physics_world *physics_world, btRigidBody *rigid_body_reference, btRigidBody *rigid_body_attached, BRX_PHYSICS_CONSTRAINT_TYPE wrapped_constraint_type, float const wrapped_pivot[3], float const wrapped_twist_axis[3], float const wrapped_plane_axis[3], float const wrapped_normal_axis[3], float const wrapped_twist_limit[2], float const wrapped_plane_limit[2], float const wrapped_normal_limit[2])
{
	// https://help.autodesk.com/view/MAYAUL/2024/ENU/?guid=GUID-CDB3638D-23AF-49EF-8EF6-53081EE4D39D

	// convention
	//
	// pivot: child bone (head) position
	//
	// rigid body reference: mapped to parent bone, centered at the midpoint of parent bone (head) position and child bone (head) position
	//
	// rigid body attached: mapped to child bone, centered at the midpoint of child bone (head) position and "child of child" bone (head) position

	btAssert((btCross(btVector3(wrapped_twist_axis[0], wrapped_twist_axis[1], wrapped_twist_axis[2]), btVector3(wrapped_plane_axis[0], wrapped_plane_axis[1], wrapped_plane_axis[2])) - btVector3(wrapped_normal_axis[0], wrapped_normal_axis[1], wrapped_normal_axis[2])).length() < INTERNAL_ZERO_THRESHOLD);

	btTransform const joint_world_transform(
		btMatrix3x3(wrapped_twist_axis[0], wrapped_plane_axis[0], wrapped_normal_axis[0],
					wrapped_twist_axis[1], wrapped_plane_axis[1], wrapped_normal_axis[1],
					wrapped_twist_axis[2], wrapped_plane_axis[2], wrapped_normal_axis[2]),
		btVector3(wrapped_pivot[0], wrapped_pivot[1], wrapped_pivot[2]));

	btTransform rigid_body_reference_world_transform = rigid_body_reference->getWorldTransform();

	btTransform rigid_body_attached_world_transform = rigid_body_attached->getWorldTransform();

	btTransform const frame_reference = rigid_body_reference_world_transform.inverse() * joint_world_transform;
	btTransform const frame_attached = rigid_body_attached_world_transform.inverse() * joint_world_transform;

	btAssert(NULL == this->m_constraint);
	{
		switch (wrapped_constraint_type)
		{
		case BRX_PHYSICS_CONSTRAINT_FIXED:
		{
			btAssert(BRX_PHYSICS_CONSTRAINT_FIXED == wrapped_constraint_type);

			btFixedConstraint *constraint = new btFixedConstraint(*rigid_body_attached, *rigid_body_reference, frame_attached, frame_reference);

			btAssert(btFabs(wrapped_twist_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_twist_limit[1]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_plane_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_plane_limit[1]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_normal_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_normal_limit[1]) < INTERNAL_ZERO_THRESHOLD);

			this->m_constraint = constraint;
		}
		break;
		case BRX_PHYSICS_CONSTRAINT_BALL_AND_SOCKET:
		{
			btAssert(BRX_PHYSICS_CONSTRAINT_BALL_AND_SOCKET == wrapped_constraint_type);

			btPoint2PointConstraint *constraint = new btPoint2PointConstraint(*rigid_body_attached, *rigid_body_reference, frame_attached.getOrigin(), frame_reference.getOrigin());

			btAssert(btFabs(wrapped_twist_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_twist_limit[1]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_plane_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_plane_limit[1]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_normal_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_normal_limit[1]) < INTERNAL_ZERO_THRESHOLD);

			this->m_constraint = constraint;
		}
		break;
		case BRX_PHYSICS_CONSTRAINT_HINGE:
		{
			btAssert(BRX_PHYSICS_CONSTRAINT_HINGE == wrapped_constraint_type);

			btHingeConstraint *constraint = new btHingeConstraint(*rigid_body_attached, *rigid_body_reference, frame_attached, frame_reference, false);

			btAssert(btFabs(wrapped_twist_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_twist_limit[1]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_plane_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_plane_limit[1]) < INTERNAL_ZERO_THRESHOLD);

			btAssert(wrapped_normal_limit[0] <= wrapped_normal_limit[1]);
			btAssert(wrapped_normal_limit[0] >= (SIMD_PI * -1.0F));
			btAssert(wrapped_normal_limit[0] <= (SIMD_PI * 1.0F));
			btAssert(wrapped_normal_limit[1] >= (SIMD_PI * -1.0F));
			btAssert(wrapped_normal_limit[1] <= (SIMD_PI * 1.0F));
			constraint->setLimit(btMin(btMax((SIMD_PI * -1.0F), btMin(wrapped_normal_limit[0], wrapped_normal_limit[1])), (SIMD_PI * 1.0F)), btMin(btMax((SIMD_PI * -1.0F), btMax(wrapped_normal_limit[0], wrapped_normal_limit[1])), (SIMD_PI * 1.0F)));

			this->m_constraint = constraint;
		}
		break;
		case BRX_PHYSICS_CONSTRAINT_PRISMATIC:
		{
			btAssert(BRX_PHYSICS_CONSTRAINT_PRISMATIC == wrapped_constraint_type);

			btSliderConstraint *constraint = new btSliderConstraint(*rigid_body_attached, *rigid_body_reference, frame_attached, frame_reference, false);

			btAssert(btFabs(wrapped_plane_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_plane_limit[1]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_normal_limit[0]) < INTERNAL_ZERO_THRESHOLD);
			btAssert(btFabs(wrapped_normal_limit[1]) < INTERNAL_ZERO_THRESHOLD);

			btAssert(wrapped_twist_limit[0] <= wrapped_twist_limit[1]);
			constraint->setLowerLinLimit(btMin(wrapped_twist_limit[0], wrapped_twist_limit[1]));
			constraint->setUpperLinLimit(btMax(wrapped_twist_limit[0], wrapped_twist_limit[1]));
			constraint->setLowerAngLimit(0.0F);
			constraint->setUpperAngLimit(0.0F);

			this->m_constraint = constraint;
		}
		break;
		default:
		{
			btAssert(BRX_PHYSICS_CONSTRAINT_RAGDOLL == wrapped_constraint_type);

			btAssert(wrapped_twist_limit[0] <= wrapped_twist_limit[1]);
			btAssert(wrapped_twist_limit[0] >= (SIMD_PI * -1.0F));
			btAssert(wrapped_twist_limit[0] <= (SIMD_PI * 1.0F));
			btAssert(wrapped_twist_limit[1] >= (SIMD_PI * -1.0F));
			btAssert(wrapped_twist_limit[1] <= (SIMD_PI * 1.0F));
			btScalar const twist_span = btMin(btMax(btFabs(wrapped_twist_limit[0]), btFabs(wrapped_twist_limit[1])), (SIMD_PI * 1.0F));

			// TODO: do we really should allow equal?
			btAssert(wrapped_plane_limit[0] <= wrapped_plane_limit[1]);
			btAssert(wrapped_plane_limit[0] >= (SIMD_PI * -0.5F));
			btAssert(wrapped_plane_limit[0] <= (SIMD_PI * 0.5F));
			btAssert(wrapped_plane_limit[1] >= (SIMD_PI * -0.5F));
			btAssert(wrapped_plane_limit[1] <= (SIMD_PI * 0.5F));
			btScalar const swing_span2 = btMin(btMax(btFabs(wrapped_plane_limit[0]), btFabs(wrapped_plane_limit[1])), (SIMD_PI * 0.5F));

			// TODO: do we really should allow equal?
			btAssert(wrapped_normal_limit[0] <= wrapped_normal_limit[1]);
			btAssert(wrapped_normal_limit[0] >= (SIMD_PI * -0.5F));
			btAssert(wrapped_normal_limit[0] <= (SIMD_PI * 0.5F));
			btAssert(wrapped_normal_limit[1] >= (SIMD_PI * -0.5F));
			btAssert(wrapped_normal_limit[1] <= (SIMD_PI * 0.5F));
			btScalar const swing_span1 = btMin(btMax(btFabs(wrapped_normal_limit[0]), btFabs(wrapped_normal_limit[1])), (SIMD_PI * 0.5F));

			btAssert(twist_span <= swing_span1);
			btAssert(twist_span <= swing_span2);

			btConeTwistConstraint *constraint = new btConeTwistConstraint(*rigid_body_attached, *rigid_body_reference, frame_attached, frame_reference);
			constraint->setLimit(swing_span1, swing_span2, twist_span);

			this->m_constraint = constraint;
		}
		}
	}
}

inline void brx_physics_constraint::uninit()
{
	btAssert(NULL != this->m_constraint);
	delete this->m_constraint;
	this->m_constraint = NULL;
}

inline btTypedConstraint *brx_physics_constraint::get_constraint() const
{
	return this->m_constraint;
}

#include "../../McRT-Malloc/include/mcrt_max_concurrency.h"
#include "../../McRT-Malloc/include/mcrt_current_thread_index.h"
#include "../../McRT-Malloc/include/mcrt_parallel_map.h"
#include "../../McRT-Malloc/include/mcrt_parallel_reduce.h"
int brx_physics_context::getNumThreads() const
{
	return mcrt_max_concurrency();
}

int brx_physics_context::getCurrentThreadIndex() const
{
	return mcrt_current_thread_index();
}

void brx_physics_context::parallelFor(int begin, int end, int grain_size, btIParallelForBody const &body)
{
	if ((begin >= 0) && (end > begin) && (grain_size > 0))
	{
		mcrt_parallel_map(
			begin,
			end,
			grain_size,
			[](uint32_t begin, uint32_t end, void *user_data) -> void
			{
				btIParallelForBody const *const body = static_cast<btIParallelForBody *>(user_data);
				body->forLoop(begin, end);
			},
			const_cast<btIParallelForBody *>(&body));
	}
	else
	{
		btAssert(begin >= 0);
		btAssert(grain_size > 0);

		body.forLoop(begin, end);
	}
}

btScalar brx_physics_context::parallelSum(int begin, int end, int grain_size, btIParallelSumBody const &body)
{
	if ((begin >= 0) && (end > begin) && (grain_size > 0))
	{
		return mcrt_parallel_reduce_float(
			begin,
			end,
			grain_size,
			[](uint32_t begin, uint32_t end, void *user_data) -> float
			{
				btIParallelSumBody const *const body = static_cast<btIParallelSumBody *>(user_data);
				return body->sumLoop(begin, end);
			},
			const_cast<btIParallelSumBody *>(&body));
	}
	else
	{
		btAssert(begin >= 0);
		btAssert(grain_size > 0);

		return body.sumLoop(begin, end);
	}
}

#include "../../McRT-Malloc/include/mcrt_malloc.h"

static void *_internal_aligned_alloc(size_t size, int alignment)
{
	btAssert(alignment >= 16);
	void *ptr = mcrt_malloc(size, alignment);
	return ptr;
}

static void _internal_aligned_free(void *ptr)
{
	return mcrt_free(ptr);
}
