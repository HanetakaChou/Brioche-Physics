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

#ifndef _BRX_PHYSICS_H_
#define _BRX_PHYSICS_H_ 1

#include <cstddef>
#include <cstdint>

enum BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE : uint32_t
{
    BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE = 0,
    BRX_PHYSICS_RIGID_BODY_SHAPE_BOX = 1,
    BRX_PHYSICS_RIGID_BODY_SHAPE_CAPSULE = 2
};

enum BRX_PHYSICS_RIGID_BODY_MOTION_TYPE : uint32_t
{
    BRX_PHYSICS_RIGID_BODY_MOTION_FIXED = 0,
    BRX_PHYSICS_RIGID_BODY_MOTION_KEYFRAME = 1,
    BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC = 2
};

enum BRX_PHYSICS_CONSTRAINT_TYPE : uint32_t
{
    BRX_PHYSICS_CONSTRAINT_FIXED = 0,
    BRX_PHYSICS_CONSTRAINT_BALL_AND_SOCKET = 1,
    BRX_PHYSICS_CONSTRAINT_HINGE = 2,
    BRX_PHYSICS_CONSTRAINT_PRISMATIC = 3,
    BRX_PHYSICS_CONSTRAINT_RAGDOLL = 4
};

class brx_physics_context;
class brx_physics_world;
class brx_physics_rigid_body;
class brx_physics_constraint;

extern "C" brx_physics_context *brx_physics_create_context();

extern "C" void brx_physics_destroy_context(brx_physics_context *physics_context);

extern "C" brx_physics_world *brx_physics_create_world(brx_physics_context *physics_context, float const gravity[3]);

extern "C" void brx_physics_destroy_world(brx_physics_context *physics_context, brx_physics_world *physics_world);

extern "C" void brx_physics_world_add_rigid_body(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body);

extern "C" void brx_physics_world_remove_rigid_body(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body);

extern "C" void brx_physics_world_add_constraint(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_constraint *physics_constraint);

extern "C" void brx_physics_world_remove_constraint(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_constraint *physics_constraint);

extern "C" void brx_physics_world_step(brx_physics_context *physics_context, brx_physics_world *physics_world, float delta_time, uint32_t max_substep_count, float substep_delta_time);

// sphere
// radius: shape_size[0]
//
// box
// extent_x: shape_size[0]
// extent_y: shape_size[1]
// extent_z: shape_size[2]
//
// capsule
// radius: shape_size[0]
// height: shape_size[1]

extern "C" brx_physics_rigid_body *brx_physics_create_rigid_body(brx_physics_context *physics_context, brx_physics_world *physics_world, float const rotation[4], float const position[3], BRX_PHYSICS_RIGID_BODY_SHAPE_TYPE shape_type, float const shape_size[3], BRX_PHYSICS_RIGID_BODY_MOTION_TYPE motion_type, uint32_t collision_filter_group, uint32_t collision_filter_mask, float mass, float linear_damping, float angular_damping, float friction, float restitution);

extern "C" void brx_physics_destroy_rigid_body(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body);

extern "C" void brx_physics_rigid_body_apply_key_frame(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body, float const rotation[4], float const position[3], float delta_time);

extern "C" void brx_physics_rigid_body_set_transform(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body, float const rotation[4], float const position[3]);

extern "C" void brx_physics_rigid_body_get_transform(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body, float out_rotation[4], float out_position[3]);

// [RagdollLoader::sLoad](https://github.com/jrouwe/JoltPhysics/blob/master/Samples/Utils/RagdollLoader.cpp#L24)

// fixed
//
// ball and socket (point)
//
// hinge
//
// prismatic (slider)
//
// ragdoll (swing twist)

extern "C" brx_physics_constraint *brx_physics_create_constraint(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_rigid_body *physics_rigid_body_a, brx_physics_rigid_body *physics_rigid_body_b, BRX_PHYSICS_CONSTRAINT_TYPE constraint_type, float const pivot[3], float const twist_axis[3], float const plane_axis[3], float const normal_axis[3], float const twist_limit[2], float const plane_limit[2], float const normal_limit[2]);

extern "C" void brx_physics_destroy_constraint(brx_physics_context *physics_context, brx_physics_world *physics_world, brx_physics_constraint *physics_constraint);

#endif
