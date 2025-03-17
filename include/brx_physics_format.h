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

#ifndef _BRX_PHYSICS_FORMAT_H_
#define _BRX_PHYSICS_FORMAT_H_ 1

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

#endif