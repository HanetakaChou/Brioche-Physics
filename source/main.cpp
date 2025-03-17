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

#include "../thirdparty/Brioche-Physics/include/brx_physics.h"
#include <cstddef>
#include <cstdint>
#include <cassert>

static inline uint64_t _internal_tick_count_per_second();
static inline uint64_t _internal_tick_count_now();

static inline void _internal_pause();

int main(int argc, char **argv)
{

	brx_physics_context *physics_context = brx_physics_create_context();

	float const gravity[3] = {0.0F, -9.8F * 10.0F, 0.0F};

	brx_physics_world *physics_world = brx_physics_create_world(physics_context, gravity);

	float const ground_shape_size[3] = {50.0F, 2.0F, 50.0F};
	float const ground_rotation[4] = {0.0F, 0.0F, 0.0F, 1.0F};
	float const ground_position[3] = {0.0F, -1.0F, 0.0F};

	brx_physics_rigid_body *physics_rigid_body_ground = brx_physics_create_rigid_body(
		physics_context,
		physics_world,
		ground_rotation,
		ground_position,
		BRX_PHYSICS_RIGID_BODY_SHAPE_BOX,
		ground_shape_size,
		BRX_PHYSICS_RIGID_BODY_MOTION_FIXED,
		0U, ((1U << 1U) | (1U << 2U)),
		0.0F, 0.0F, 0.05F, 0.7F, 0.1F);

	float const ball1_shape_size[3] = {2.0F, 0.0F, 0.0F};
	float const ball1_rotation[4] = {0.0F, 0.0F, 0.0F, 1.0F};
	float const ball1_position[3] = {0.0F, 50.0F, 0.0F};

	brx_physics_rigid_body *physics_rigid_body_ball1 = brx_physics_create_rigid_body(
		physics_context,
		physics_world,
		ball1_rotation,
		ball1_position,
		BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE,
		ball1_shape_size,
		BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC,
		1U, ((1U << 0U)),
		1.0F, 0.0F, 0.05F, 0.9F, 0.1F);

	float const ball2_shape_size[3] = {2.0F, 0.0F, 0.0F};
	float const ball2_rotation[4] = {0.0F, 0.0F, 0.0F, 1.0F};
	float const ball2_position[3] = {0.0F, 25.0F, 0.0F};

	brx_physics_rigid_body *physics_rigid_body_ball2 = brx_physics_create_rigid_body(
		physics_context,
		physics_world,
		ball2_rotation,
		ball2_position,
		BRX_PHYSICS_RIGID_BODY_SHAPE_SPHERE,
		ball2_shape_size,
		BRX_PHYSICS_RIGID_BODY_MOTION_DYNAMIC,
		2U, ((1U << 0U)),
		1.0F, 0.0F, 0.05F, 1.1F, 0.1F);

	brx_physics_world_add_rigid_body(physics_context, physics_world, physics_rigid_body_ground);
	brx_physics_world_add_rigid_body(physics_context, physics_world, physics_rigid_body_ball1);
	brx_physics_world_add_rigid_body(physics_context, physics_world, physics_rigid_body_ball2);

	// 60 FPS
	constexpr int const step_count_per_second = 60;
	// simulate for 15 second
	constexpr const float time_seconds = 15.0F;

	constexpr float const time_step = 1.0F / static_cast<float>(step_count_per_second);
	constexpr const int step_count = static_cast<int>(time_seconds / time_step);

	uint64_t time_step_tick = static_cast<int64_t>(static_cast<double>(_internal_tick_count_per_second()) * static_cast<double>(time_step));
	uint64_t last_time_tick = _internal_tick_count_now();

	for (int step_index = 0; step_index < step_count; ++step_index)
	{
		float ball1_rotation[4];
		float ball1_translation[3];
		brx_physics_rigid_body_get_transform(physics_context, physics_world, physics_rigid_body_ball1, ball1_rotation, ball1_translation);

		float ball2_rotation[4] = {0.0F, 0.0F, 0.0F, 1.0F};
		float ball2_position[3] = {0.0F, 7.7F, 0.0F};
		brx_physics_rigid_body_apply_key_frame(physics_context, physics_world, physics_rigid_body_ball2, ball2_rotation, ball2_position, time_step);

		brx_physics_world_step(physics_context, physics_world, time_step);

		while (_internal_tick_count_now() < last_time_tick + time_step_tick)
		{
			_internal_pause();
		}

		float ground_rotation_hk[4];
		float ground_translation_hk[3];
		brx_physics_rigid_body_get_transform(physics_context, physics_world, physics_rigid_body_ground, ground_rotation_hk, ground_translation_hk);

		last_time_tick += time_step_tick;
	}

	brx_physics_world_remove_rigid_body(physics_context, physics_world, physics_rigid_body_ball1);
	brx_physics_world_remove_rigid_body(physics_context, physics_world, physics_rigid_body_ball2);
	brx_physics_world_remove_rigid_body(physics_context, physics_world, physics_rigid_body_ground);

	brx_physics_destory_rigid_body(physics_context, physics_world, physics_rigid_body_ball1);
	brx_physics_destory_rigid_body(physics_context, physics_world, physics_rigid_body_ball2);
	brx_physics_destory_rigid_body(physics_context, physics_world, physics_rigid_body_ground);

	brx_physics_destory_world(physics_context, physics_world);

	brx_physics_destory_context(physics_context);

	return 0;
}

#if defined(__GNUC__)

#include <time.h>

extern uint64_t _internal_tick_count_per_second()
{
	constexpr uint64_t const _internal_tick_count_per_second = 1000000000ULL;
	return _internal_tick_count_per_second;
}

extern uint64_t _internal_tick_count_now()
{
	struct timespec time_monotonic;
	int result_clock_get_time_monotonic = clock_gettime(CLOCK_MONOTONIC, &time_monotonic);
	assert(0 == result_clock_get_time_monotonic);

	uint64_t const _internal_tick_count_now = static_cast<uint64_t>(1000000000ULL) * static_cast<uint64_t>(time_monotonic.tv_sec) + static_cast<uint64_t>(time_monotonic.tv_nsec);
	return _internal_tick_count_now;
}

#elif defined(_MSC_VER)

#define NOMINMAX 1
#define WIN32_LEAN_AND_MEAN 1
#include <sdkddkver.h>
#include <Windows.h>

extern uint64_t _internal_tick_count_per_second()
{
	LARGE_INTEGER int64_frequency;
	BOOL result_query_performance_frequency = QueryPerformanceFrequency(&int64_frequency);
	assert(NULL != result_query_performance_frequency);

	uint64_t const _internal_tick_count_per_second = static_cast<uint64_t>(int64_frequency.QuadPart);
	return _internal_tick_count_per_second;
}

extern uint64_t _internal_tick_count_now()
{
	LARGE_INTEGER int64_performance_count;
	BOOL result_query_performance_counter = QueryPerformanceCounter(&int64_performance_count);
	assert(NULL != result_query_performance_counter);

	uint64_t const _internal_tick_count_now = static_cast<uint64_t>(int64_performance_count.QuadPart);
	return _internal_tick_count_now;
}

#else
#error Unknown Compiler
#endif

#if defined(__GNUC__)
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#elif defined(__aarch64__) || defined(__arm__)
#include <arm_acle.h>
#else
#error Unknown Architecture
#endif
#elif defined(_MSC_VER)
#if defined(_M_X64) || defined(_M_IX86)
#include <immintrin.h>
#elif defined(_M_ARM64) || defined(_M_ARM)
#include <intrin.h>
#else
#error Unknown Architecture
#endif
#else
#error Unknown Compiler
#endif

static inline void _internal_pause()
{
#if defined(__GNUC__)
#if defined(__x86_64__) || defined(__i386__)
	_mm_pause();
#elif defined(__aarch64__) || defined(__arm__)
	__yield();
#else
#error Unknown Architecture
#endif
#elif defined(_MSC_VER)
#if defined(_M_X64) || defined(_M_IX86)
	_mm_pause();
#elif defined(_M_ARM64) || defined(_M_ARM)
	__yield();
#else
#error Unknown Architecture
#endif
#else
#error Unknown Compiler
#endif
}
