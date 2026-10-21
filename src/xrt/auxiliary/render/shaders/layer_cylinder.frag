// Copyright 2023, Collabora Ltd.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// Author: Jakob Bornecrantz <jakob@collabora.com>
// SPDX-License-Identifier: BSL-1.0

#version 460

// needed for color_scale & color_bias
layout (binding = 0, std140) uniform Config
{
	vec4 post_transform;
	mat4 mvp;
	float radius;
	float central_angle;
	float aspect_ratio;
	float _pad;
	vec4 color_scale;
	vec4 color_bias;
} ubo;

layout (binding = 1) uniform sampler2D image;

layout (location = 0)  in vec2 uv;
layout (location = 0) out vec4 out_color;


void main ()
{
	out_color = texture(image, uv);
	// Values above 1.0 (HDR) kept for a float target; a UNORM one clamps them.
	out_color = out_color * ubo.color_scale + ubo.color_bias;
	out_color = vec4(max(out_color.rgb, vec3(0.0)), clamp(out_color.a, 0.0, 1.0));
}
