// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
// Author: Aditya Mehra <aix.m@outlook.com>
//
// Motion smoothing: what all its passes share. One descriptor set layout and one push constant block for
// all of them (render_motion.c); each pass uses the bindings it needs.
//
// The passes, per view: a pyramid of each new frame (half, quarter, eighth size, sRGB encoded), the previous frame's
// pyramid turned to the new frame's head pose, block matching from coarse to fine (blocks of 8 at every level), and on
// refreshes without a new frame: the motion carried forward, then the newest frame moved on by it.


#define MOTION_BLOCK 8

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(push_constant) uniform MotionPush
{
	vec4 v0;
	vec4 v1;
	vec4 v2;
	vec4 v3;
	ivec4 i0;
	ivec4 i1;
} push;
