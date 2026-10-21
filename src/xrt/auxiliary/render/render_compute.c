// Copyright 2019-2023, Collabora, Ltd.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The compositor compute based rendering code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_render
 */

#include "math/m_api.h"
#include "math/m_matrix_4x4_f64.h"

#include "vk/vk_mini_helpers.h"

#include "render/render_interface.h"
#include "render/render_output_settings.h"

#include "util/u_debug.h"
#include "os/os_time.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>


/*
 *
 * Helper functions.
 *
 */

/*!
 * Get the @ref vk_bundle from @ref render_compute.
 */
static inline struct vk_bundle *
vk_from_render(struct render_compute *render)
{
	return render->r->vk;
}

static uint32_t
uint_divide_and_round_up(uint32_t a, uint32_t b)
{
	return (a + (b - 1)) / b;
}

static void
calc_dispatch_dims_1_view(const struct render_viewport_data views, uint32_t *out_w, uint32_t *out_h)
{
	// Power of two divide and round up.
	uint32_t w = uint_divide_and_round_up(views.w, 8);
	uint32_t h = uint_divide_and_round_up(views.h, 8);

	*out_w = w;
	*out_h = h;
}

/*
 * For dispatching compute to the view, calculate the number of groups.
 */
static void
calc_dispatch_dims_views(const struct render_viewport_data views[XRT_MAX_VIEWS],
                         uint32_t view_count,
                         uint32_t *out_w,
                         uint32_t *out_h)
{
#define IMAX(a, b) ((a) > (b) ? (a) : (b))
	uint32_t w = 0;
	uint32_t h = 0;
	for (uint32_t i = 0; i < view_count; ++i) {
		w = IMAX(w, views[i].w);
		h = IMAX(h, views[i].h);
	}
#undef IMAX

	// Power of two divide and round up.
	w = uint_divide_and_round_up(w, 8);
	h = uint_divide_and_round_up(h, 8);

	*out_w = w;
	*out_h = h;
}


/*
 *
 * Vulkan helpers.
 *
 */

XRT_MAYBE_UNUSED static void
update_compute_layer_descriptor_set(struct vk_bundle *vk,
                                    uint32_t src_binding,
                                    VkSampler src_samplers[RENDER_MAX_IMAGES_SIZE],
                                    VkImageView src_image_views[RENDER_MAX_IMAGES_SIZE],
                                    uint32_t image_count,
                                    uint32_t target_binding,
                                    VkImageView target_image_view,
                                    uint32_t ubo_binding,
                                    VkBuffer ubo_buffer,
                                    VkDeviceSize ubo_size,
                                    uint32_t hdr_stats_binding,
                                    VkBuffer hdr_stats_buffer,
                                    VkDescriptorSet descriptor_set)
{
	VkDescriptorImageInfo src_image_info[RENDER_MAX_IMAGES_SIZE];
	for (uint32_t i = 0; i < image_count; i++) {
		src_image_info[i].sampler = src_samplers[i];
		src_image_info[i].imageView = src_image_views[i];
		src_image_info[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	}

	VkDescriptorImageInfo target_image_info = {
	    .imageView = target_image_view,
	    .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
	};

	VkDescriptorBufferInfo buffer_info = {
	    .buffer = ubo_buffer,
	    .offset = 0,
	    .range = ubo_size,
	};

	VkDescriptorBufferInfo hdr_stats_info = {
	    .buffer = hdr_stats_buffer,
	    .offset = 0,
	    .range = VK_WHOLE_SIZE,
	};

	VkWriteDescriptorSet write_descriptor_sets[4] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = hdr_stats_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
	        .pBufferInfo = &hdr_stats_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = src_binding,
	        .descriptorCount = image_count,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .pImageInfo = src_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = target_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .pImageInfo = &target_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = ubo_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .pBufferInfo = &buffer_info,
	    },
	};

	vk->vkUpdateDescriptorSets(            //
	    vk->device,                        //
	    ARRAY_SIZE(write_descriptor_sets), // descriptorWriteCount
	    write_descriptor_sets,             // pDescriptorWrites
	    0,                                 // descriptorCopyCount
	    NULL);                             // pDescriptorCopies
}

XRT_MAYBE_UNUSED static void
update_compute_shared_descriptor_set(struct vk_bundle *vk,
                                     uint32_t src_binding,
                                     VkSampler src_samplers[XRT_MAX_VIEWS],
                                     VkImageView src_image_views[XRT_MAX_VIEWS],
                                     uint32_t distortion_binding,
                                     VkSampler distortion_samplers[3 * XRT_MAX_VIEWS],
                                     VkImageView distortion_image_views[3 * XRT_MAX_VIEWS],
                                     uint32_t target_binding,
                                     VkImageView target_image_view,
                                     uint32_t ubo_binding,
                                     VkBuffer ubo_buffer,
                                     VkDeviceSize ubo_size,
                                     VkDescriptorSet descriptor_set,
                                     uint32_t view_count)
{
	VkDescriptorImageInfo src_image_info[XRT_MAX_VIEWS];
	for (uint32_t i = 0; i < view_count; ++i) {
		src_image_info[i].sampler = src_samplers[i];
		src_image_info[i].imageView = src_image_views[i];
		src_image_info[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	}

	VkDescriptorImageInfo distortion_image_info[3 * XRT_MAX_VIEWS];
	for (uint32_t i = 0; i < 3 * view_count; ++i) {
		distortion_image_info[i].sampler = distortion_samplers[i];
		distortion_image_info[i].imageView = distortion_image_views[i];
		distortion_image_info[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	}

	VkDescriptorImageInfo target_image_info = {
	    .imageView = target_image_view,
	    .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
	};

	VkDescriptorBufferInfo buffer_info = {
	    .buffer = ubo_buffer,
	    .offset = 0,
	    .range = ubo_size,
	};

	VkWriteDescriptorSet write_descriptor_sets[4] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = src_binding,
	        .descriptorCount = view_count,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .pImageInfo = src_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = distortion_binding,
	        .descriptorCount = 3 * view_count,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .pImageInfo = distortion_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = target_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .pImageInfo = &target_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = ubo_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .pBufferInfo = &buffer_info,
	    },
	};

	vk->vkUpdateDescriptorSets(            //
	    vk->device,                        //
	    ARRAY_SIZE(write_descriptor_sets), // descriptorWriteCount
	    write_descriptor_sets,             // pDescriptorWrites
	    0,                                 // descriptorCopyCount
	    NULL);                             // pDescriptorCopies
}

XRT_MAYBE_UNUSED static void
update_compute_descriptor_set_target(struct vk_bundle *vk,
                                     uint32_t target_binding,
                                     VkImageView target_image_view,
                                     uint32_t ubo_binding,
                                     VkBuffer ubo_buffer,
                                     VkDeviceSize ubo_size,
                                     VkDescriptorSet descriptor_set,
                                     uint32_t view_count)
{
	VkDescriptorImageInfo target_image_info = {
	    .imageView = target_image_view,
	    .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
	};

	VkDescriptorBufferInfo buffer_info = {
	    .buffer = ubo_buffer,
	    .offset = 0,
	    .range = ubo_size,
	};

	VkWriteDescriptorSet write_descriptor_sets[2] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = target_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .pImageInfo = &target_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = ubo_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .pBufferInfo = &buffer_info,
	    },
	};

	vk->vkUpdateDescriptorSets(            //
	    vk->device,                        //
	    ARRAY_SIZE(write_descriptor_sets), // descriptorWriteCount
	    write_descriptor_sets,             // pDescriptorWrites
	    0,                                 // descriptorCopyCount
	    NULL);                             // pDescriptorCopies
}

/*!
 * The image holding the panel's previous frame, the target's size (the union of the views); a new one on a size
 * change, the replaced one kept until fini (it may still be in use).
 */
static bool
ensure_previous_frame_image(struct render_compute *render, const struct render_viewport_data views[XRT_MAX_VIEWS])
{
	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;
	__typeof__(r->compute.distortion.previous) *prev = &r->compute.distortion.previous;

	uint32_t width = 0, height = 0;
	for (uint32_t i = 0; i < r->view_count; ++i) {
		width = MAX(width, views[i].x + views[i].w);
		height = MAX(height, views[i].y + views[i].h);
	}
	if (prev->image != VK_NULL_HANDLE && prev->width >= width && prev->height >= height) {
		return true;
	}
	if (prev->image != VK_NULL_HANDLE) {
		D(ImageView, prev->old_view);
		D(Image, prev->old_image);
		DF(Memory, prev->old_memory);
		prev->old_view = prev->view;
		prev->old_image = prev->image;
		prev->old_memory = prev->memory;
		prev->view = VK_NULL_HANDLE;
		prev->image = VK_NULL_HANDLE;
		prev->memory = VK_NULL_HANDLE;
	}

	const VkFormat format = VK_FORMAT_R16G16B16A16_SFLOAT;
	VkResult ret = vk_create_image_simple(vk, (VkExtent2D){width, height}, format,
	                                      VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
	                                      &prev->memory, &prev->image);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "Overdrive: no previous-frame image (%s)", vk_result_string(ret));
		return false;
	}
	VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	ret = vk_create_view(vk, prev->image, VK_IMAGE_VIEW_TYPE_2D, format, range, &prev->view);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "Overdrive: no previous-frame view (%s)", vk_result_string(ret));
		return false;
	}
	prev->width = width;
	prev->height = height;
	prev->fresh = true;
	return true;
}

// The layer squasher's UBO must fit the 64 KiB a uniform buffer is guaranteed on NVIDIA (RiftLinux added per-layer
// HDR data).
_Static_assert(sizeof(struct render_compute_layer_ubo_data) <= 65536, "layer UBO larger than 64 KiB");

/*
 * Output capture for tests (RiftLinux): what goes to the panel, after Auto-HDR and before the
 * dither, sRGB encoded, at float precision (the previous-frame image). XRT_COMPOSITOR_CAPTURE_OUTPUT captures, one
 * every 180 frames from frame XRT_COMPOSITOR_CAPTURE_OUTPUT_START on (default 2700, 30 s at 90 Hz: SteamVR Home is
 * up), to $XDG_CACHE_HOME/riftlinux/output-capture (else ~/.cache/...): "RLOF", width, height (uint32), then RGBA half
 * floats, rows top down. With XRT_COMPOSITOR_CAPTURE_OUTPUT_AB each capture is two consecutive frames, the first
 * without Auto-HDR (the same scene 11 ms apart): outputNN-off and outputNN-on. Off (0) by default; writing the files
 * stalls that frame.
 */
DEBUG_GET_ONCE_NUM_OPTION(capture_output, "XRT_COMPOSITOR_CAPTURE_OUTPUT", 0)
DEBUG_GET_ONCE_NUM_OPTION(capture_output_start, "XRT_COMPOSITOR_CAPTURE_OUTPUT_START", 2700)
DEBUG_GET_ONCE_BOOL_OPTION(capture_output_ab, "XRT_COMPOSITOR_CAPTURE_OUTPUT_AB", false)

enum capture_step
{
	CAPTURE_NONE,
	CAPTURE_PLAIN, //!< Copy this frame.
	CAPTURE_OFF,   //!< Copy this frame, rendered without Auto-HDR (A/B pair, first).
	CAPTURE_ON,    //!< Copy this frame (A/B pair, second).
	CAPTURE_WRITE, //!< Write the copies out: made over ten frames ago, finished (the command buffer is reused
	               //!< each frame only after its fence).
};

//! What the capture does this frame; counts the frames.
static enum capture_step
capture_step_for_frame(struct render_resources *r)
{
	__typeof__(r->compute.distortion.capture) *cap = &r->compute.distortion.capture;
	const uint64_t frame = cap->frames++;
	const int64_t count = debug_get_num_option_capture_output();
	const uint64_t start = (uint64_t)MAX(debug_get_num_option_capture_output_start(), 0);
	if (count <= 0 || cap->done >= count || frame < start) {
		return CAPTURE_NONE;
	}
	const bool ab = debug_get_bool_option_capture_output_ab();
	switch ((frame - start) % 180) {
	case 0: return ab ? CAPTURE_OFF : CAPTURE_PLAIN;
	case 1: return ab ? CAPTURE_ON : CAPTURE_NONE;
	case 12: return CAPTURE_WRITE;
	default: return CAPTURE_NONE;
	}
}

static void
capture_output_write_one(struct render_resources *r, struct render_buffer *buffer, const char *suffix)
{
	__typeof__(r->compute.distortion.capture) *cap = &r->compute.distortion.capture;
	struct vk_bundle *vk = r->vk;
	char dir[512];
	const char *cache = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	if (cache != NULL && cache[0] != '\0') {
		snprintf(dir, sizeof(dir), "%s/riftlinux", cache);
	} else {
		snprintf(dir, sizeof(dir), "%s/.cache/riftlinux", home != NULL ? home : "/tmp");
	}
	mkdir(dir, 0755);
	strncat(dir, "/output-capture", sizeof(dir) - strlen(dir) - 1);
	mkdir(dir, 0755);

	char path[600];
	snprintf(path, sizeof(path), "%s/output%02d%s.rlof", dir, cap->done, suffix);
	if (render_buffer_map(vk, buffer) != VK_SUCCESS) {
		VK_ERROR(vk, "Output capture: cannot map the buffer");
		return;
	}
	FILE *f = fopen(path, "wb");
	if (f != NULL) {
		const uint32_t header[3] = {0x464f4c52u /* "RLOF" */, cap->width, cap->height};
		fwrite(header, sizeof(header), 1, f);
		fwrite(buffer->mapped, (size_t)cap->width * cap->height * 8, 1, f);
		fclose(f);
		VK_INFO(vk, "Output capture %d: %s (%ux%u)", cap->done, path, cap->width, cap->height);
	} else {
		VK_ERROR(vk, "Output capture: cannot write %s", path);
	}
	render_buffer_unmap(vk, buffer);
}

//! Called after the distortion dispatch is recorded.
static void
capture_output(struct render_compute *render, enum capture_step step)
{
	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;
	__typeof__(r->compute.distortion.previous) *prev = &r->compute.distortion.previous;
	__typeof__(r->compute.distortion.capture) *cap = &r->compute.distortion.capture;

	if (step == CAPTURE_NONE) {
		return;
	}
	if (step == CAPTURE_WRITE) {
		const bool ab = debug_get_bool_option_capture_output_ab();
		capture_output_write_one(r, &cap->buffers[0], ab ? "-off" : "");
		if (ab) {
			capture_output_write_one(r, &cap->buffers[1], "-on");
		}
		cap->done++;
		return;
	}

	struct render_buffer *buffer = &cap->buffers[step == CAPTURE_ON ? 1 : 0];
	const VkDeviceSize size = (VkDeviceSize)prev->width * prev->height * 8;
	if (buffer->buffer == VK_NULL_HANDLE || buffer->size != size) {
		render_buffer_fini(vk, buffer);
		VkResult ret = render_buffer_init(vk, buffer, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		                                  size);
		if (ret != VK_SUCCESS) {
			VK_ERROR(vk, "Output capture: no buffer (%s)", vk_result_string(ret));
			return;
		}
	}
	const VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	vk_cmd_image_barrier_gpu_locked(vk, r->cmd, prev->image, VK_ACCESS_SHADER_WRITE_BIT,
	                                VK_ACCESS_TRANSFER_READ_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
	                                range);
	const VkBufferImageCopy region = {
	    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
	    .imageExtent = {prev->width, prev->height, 1},
	};
	vk->vkCmdCopyImageToBuffer(r->cmd, prev->image, VK_IMAGE_LAYOUT_GENERAL, buffer->buffer, 1, &region);
	vk_cmd_image_barrier_gpu_locked(vk, r->cmd, prev->image, VK_ACCESS_TRANSFER_READ_BIT,
	                                VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_GENERAL,
	                                VK_IMAGE_LAYOUT_GENERAL, range);
	cap->width = prev->width;
	cap->height = prev->height;
}

// OKLab lightness of the grey at sRGB @p level of 255.
static float
grey_level_lightness(float level)
{
	const float x = level / 255.0f;
	return cbrtf(x <= 0.04045f ? x / 12.92f : powf((x + 0.055f) / 1.055f, 2.4f));
}

/*
 * Strength of distortion.comp's near-black lift that puts sRGB level 1 at @p level + 1, the first the panel shows; at
 * most 6, beyond that the curve would flatten.
 */
static float
black_lift_for_level(float level)
{
	if (level <= 0.0f) {
		return 0.0f;
	}
	level = fminf(level, 6.0f);
	const float floor_l = 0.2f;
	const float l1 = grey_level_lightness(1.0f);
	const float x = 1.0f - l1 / floor_l;
	return (grey_level_lightness(level + 1.0f) - l1) / (x * x * l1 / (l1 + 0.01f));
}

/*!
 * Auto-HDR's lightness and colour parameters from @p s: highlights, shadows, colour (all 0 off), knee.
 */
static void
auto_hdr_params_from(const struct render_output_settings *s, bool auto_hdr_off, float out[4])
{
	const bool auto_hdr = s->auto_hdr && !auto_hdr_off;
	out[0] = auto_hdr ? fminf(fmaxf(s->auto_hdr_highlights, 0.0f), 0.95f) : 0.0f;
	out[1] = auto_hdr ? fminf(fmaxf(s->auto_hdr_shadows, 0.0f), 1.0f) : 0.0f;
	// Vibrance, if set, is the colour stage's strength, with or without Auto-HDR.
	out[2] = auto_hdr_off ? 0.0f : s->vibrance != 0.0f ? s->vibrance : auto_hdr ? s->auto_hdr_colour : 0.0f;
	out[3] = fminf(fmaxf(s->auto_hdr_knee, 0.3f), 0.8f);
}

/*!
 * The picture settings into the UBO; @p auto_hdr_off leaves out Auto-HDR (black level, lightness, colour) for this
 * frame (the output capture's A/B pair); @p source_processed: the layer squasher applied lightness and colour per
 * layer already, only the black level is left here.
 */
static void
fill_output_processing(struct render_compute_distortion_ubo_data *data,
                       bool previous_fresh,
                       bool auto_hdr_off,
                       bool source_processed)
{
	static uint32_t frame = 0;
	struct render_output_settings s;
	render_output_settings_get(&s);

	data->post0[0] = s.dither_bits >= 1.0f ? 1.0f / (exp2f(s.dither_bits) - 1.0f) : 0.0f;
	data->post0[1] = auto_hdr_off ? 0.0f : black_lift_for_level(s.black_level);
	data->post0[2] = s.tonemap_shoulder;
	data->post0[3] = (float)(frame++ % 1024u);
	// post1: unused (overdrive, parked on riftlinux/overdrive-parked).
	(void)previous_fresh;
	memset(data->post1, 0, sizeof(data->post1));

	auto_hdr_params_from(&s, auto_hdr_off || source_processed, data->post2);
}

static void
dispatch_project_pipeline(struct render_compute *render,
                          VkSampler src_samplers[XRT_MAX_VIEWS],
                          VkImageView src_image_views[XRT_MAX_VIEWS],
                          const struct xrt_normalized_rect src_norm_rects[XRT_MAX_VIEWS],
                          VkImage target_image,
                          VkImageView target_image_view,
                          VkImageLayout target_final_layout,
                          const struct render_viewport_data views[XRT_MAX_VIEWS],
                          VkPipeline pipeline)
{
	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;


	/*
	 * UBO
	 */

	struct render_compute_distortion_ubo_data *data =
	    (struct render_compute_distortion_ubo_data *)r->compute.distortion.ubo.mapped;
	for (uint32_t i = 0; i < render->r->view_count; ++i) {
		data->views[i] = views[i];
		data->post_transforms[i] = src_norm_rects[i];
	}
	// The shader always reads and writes the previous frame, so the image must exist.
	if (!ensure_previous_frame_image(render, views)) {
		return;
	}
	__typeof__(r->compute.distortion.previous) *prev = &r->compute.distortion.previous;
	const enum capture_step capture_step = (enum capture_step)r->compute.distortion.capture.step;
	fill_output_processing(data, prev->fresh, capture_step == CAPTURE_OFF, render->source_processed);
	// The fast path's HDR layer, measured into the squasher's histograms; nothing after the squasher.
	if (render->source_processed) {
		memset(data->hdr_source, 0, sizeof(data->hdr_source));
		memset(data->hdr_range, 0, sizeof(data->hdr_range));
	} else {
		memcpy(data->hdr_source, render->source_hdr, sizeof(data->hdr_source));
		memcpy(data->hdr_range, render->source_hdr_range, sizeof(data->hdr_range));
	}
	if (data->hdr_source[0] > 0.5f) {
		for (uint32_t i = 0; i < render->r->view_count; ++i) {
			r->compute.layer.hdr_stats_pixels += (uint64_t)views[i].w * views[i].h;
		}
	}


	/*
	 * Source, target and distortion images.
	 */

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	vk_cmd_image_barrier_gpu_locked( //
	    vk,                          //
	    r->cmd,                      //
	    target_image,                //
	    0,                           //
	    VK_ACCESS_SHADER_WRITE_BIT,  //
	    VK_IMAGE_LAYOUT_UNDEFINED,   //
	    VK_IMAGE_LAYOUT_GENERAL,     //
	    subresource_range);          //

	VkSampler sampler = r->samplers.clamp_to_edge;
	VkSampler distortion_samplers[3 * XRT_MAX_VIEWS];
	for (uint32_t i = 0; i < render->r->view_count; ++i) {
		distortion_samplers[3 * i + 0] = sampler;
		distortion_samplers[3 * i + 1] = sampler;
		distortion_samplers[3 * i + 2] = sampler;
	}

	update_compute_shared_descriptor_set( //
	    vk,                               //
	    r->compute.src_binding,           //
	    src_samplers,                     //
	    src_image_views,                  //
	    r->compute.distortion_binding,    //
	    distortion_samplers,              //
	    r->distortion.image_views,        //
	    r->compute.target_binding,        //
	    target_image_view,                //
	    r->compute.ubo_binding,           //
	    r->compute.distortion.ubo.buffer, //
	    VK_WHOLE_SIZE,                    //
	    render->shared_descriptor_set,    //
	    render->r->view_count);           //

	// The previous frame: written by the last dispatch, read and written by this one.
	VkDescriptorImageInfo previous_info = {
	    .imageView = prev->view,
	    .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
	};
	VkWriteDescriptorSet previous_write = {
	    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	    .dstSet = render->shared_descriptor_set,
	    .dstBinding = r->compute.previous_binding,
	    .descriptorCount = 1,
	    .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	    .pImageInfo = &previous_info,
	};
	vk->vkUpdateDescriptorSets(vk->device, 1, &previous_write, 0, NULL);

	// HDR layers' histograms (the fast path measures its layer), this frame's buffer.
	VkDescriptorBufferInfo hdr_stats_info = {
	    .buffer = r->compute.layer.hdr_stats[r->compute.layer.hdr_stats_index].buffer,
	    .offset = 0,
	    .range = VK_WHOLE_SIZE,
	};
	VkWriteDescriptorSet hdr_stats_write = {
	    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	    .dstSet = render->shared_descriptor_set,
	    .dstBinding = r->compute.hdr_stats_binding,
	    .descriptorCount = 1,
	    .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
	    .pBufferInfo = &hdr_stats_info,
	};
	vk->vkUpdateDescriptorSets(vk->device, 1, &hdr_stats_write, 0, NULL);
	vk_cmd_image_barrier_gpu_locked(                                   //
	    vk,                                                            //
	    r->cmd,                                                        //
	    prev->image,                                                   //
	    prev->fresh ? 0 : VK_ACCESS_SHADER_WRITE_BIT,                  //
	    VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,        //
	    prev->fresh ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_GENERAL, //
	    VK_IMAGE_LAYOUT_GENERAL,                                       //
	    (VkImageSubresourceRange){VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}); //
	prev->fresh = false;

	vk->vkCmdBindPipeline(              //
	    r->cmd,                         //
	    VK_PIPELINE_BIND_POINT_COMPUTE, // pipelineBindPoint
	    pipeline);                      // pipeline

	vk->vkCmdBindDescriptorSets(               //
	    r->cmd,                                //
	    VK_PIPELINE_BIND_POINT_COMPUTE,        // pipelineBindPoint
	    r->compute.distortion.pipeline_layout, // layout
	    0,                                     // firstSet
	    1,                                     // descriptorSetCount
	    &render->shared_descriptor_set,        // pDescriptorSets
	    0,                                     // dynamicOffsetCount
	    NULL);                                 // pDynamicOffsets


	uint32_t w = 0, h = 0;
	calc_dispatch_dims_views(views, render->r->view_count, &w, &h);
	assert(w != 0 && h != 0);

	vk->vkCmdDispatch(  //
	    r->cmd,         //
	    w,              // groupCountX
	    h,              // groupCountY
	    r->view_count); // groupCountZ

	capture_output(render, capture_step);

	VkImageMemoryBarrier memoryBarrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_MEMORY_READ_BIT,
	    .oldLayout = VK_IMAGE_LAYOUT_GENERAL,
	    .newLayout = target_final_layout,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = target_image,
	    .subresourceRange = subresource_range,
	};

	vk->vkCmdPipelineBarrier(                 //
	    r->cmd,                               //
	    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, //
	    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,    //
	    0,                                    //
	    0,                                    //
	    NULL,                                 //
	    0,                                    //
	    NULL,                                 //
	    1,                                    //
	    &memoryBarrier);                      //
}


/*
 *
 * 'Exported' functions.
 *
 */

/*!
 * HDR layers' measured peaks from last frame's histograms: per slot the brightness that 0.02 % of the
 * covered pixels reach (a lone sparkle does not set it), relative to reference white; smoothed, rising within a few
 * frames, falling over about half a second, and both at once after a scene cut (a change of more than double).
 */
static void
hdr_stats_frame_begin(struct render_resources *r)
{
	__typeof__(r->compute.layer) *l = &r->compute.layer;
	const uint32_t *bins = (const uint32_t *)l->hdr_stats[l->hdr_stats_index].mapped;
	const double threshold = fmax((double)l->hdr_stats_pixels * 2e-4, 16.0);
	for (uint32_t s = 0; s < RENDER_HDR_STATS_SLOTS; s++) {
		float *peak = &l->hdr_measured_peak[s];
		if ((l->hdr_slots_used & (1u << s)) == 0) {
			// Not used last frame: forgotten.
			*peak = 0.0f;
			continue;
		}
		const uint32_t *b = bins + s * RENDER_HDR_STATS_BINS;
		float measured = 1.0f; // Nothing above reference white.
		uint64_t sum = 0;
		for (int i = RENDER_HDR_STATS_BINS - 1; i >= 0; i--) {
			sum += b[i];
			if ((double)sum >= threshold) {
				measured = exp2f((float)(i + 1) / 4.0f);
				break;
			}
		}
		if (*peak <= 0.0f || measured > 2.0f * *peak || measured < 0.5f * *peak) {
			*peak = measured;
		} else if (measured > *peak) {
			*peak += 0.3f * (measured - *peak);
		} else {
			*peak += 0.02f * (measured - *peak);
		}
	}
	l->hdr_slots_used = 0;

	// The other buffer for this frame, emptied (its frame was read a frame ago).
	l->hdr_stats_index ^= 1u;
	memset(l->hdr_stats[l->hdr_stats_index].mapped, 0,
	       RENDER_HDR_STATS_SLOTS * RENDER_HDR_STATS_BINS * sizeof(uint32_t));
	l->hdr_stats_pixels = 0;
}

void
render_compute_frame_begin(struct render_compute *render)
{
	render->r->compute.distortion.capture.step = capture_step_for_frame(render->r);
	hdr_stats_frame_begin(render->r);
}

void
render_compute_auto_hdr_params(struct render_compute *render, float out_params[4])
{
	struct render_output_settings s;
	render_output_settings_get(&s);
	auto_hdr_params_from(&s, render->r->compute.distortion.capture.step == CAPTURE_OFF, out_params);
}

void
render_compute_hdr_layer(struct render_compute *render,
                         const struct xrt_layer_hdr_data *hdr,
                         int slot,
                         float out_source[4],
                         float out_range[4])
{
	memset(out_source, 0, sizeof(float) * 4);
	memset(out_range, 0, sizeof(float) * 4);
	if (hdr->transfer == XRT_LAYER_TRANSFER_SDR) {
		return;
	}
	struct render_output_settings s;
	render_output_settings_get(&s);
	const float white_nits = hdr->reference_white_nits > 0.0f ? hdr->reference_white_nits : 203.0f;

	// The slot's measured peak; the slot is kept as it is used this frame. Not measured with the
	// measured peak switched off: the slot is then forgotten, and starts afresh when it is on again.
	float measured = 0.0f;
	if (s.hdr_measured_peak && slot >= 0 && slot < RENDER_HDR_STATS_SLOTS) {
		render->r->compute.layer.hdr_slots_used |= 1u << slot;
		measured = render->r->compute.layer.hdr_measured_peak[slot];
		out_range[3] = (float)(slot + 1);
	}
	// The content's brightness metadata, unless switched off: then as if none came.
	const float static_peak = !s.hdr_metadata                  ? 0.0f
	                          : hdr->max_cll_nits > 0.0f        ? hdr->max_cll_nits
	                          : hdr->mastering_max_nits > 0.0f ? hdr->mastering_max_nits
	                                                           : 0.0f;
	const float scene_peak = s.hdr_metadata ? hdr->scene_max_nits : 0.0f;
	float peak_nits = 1000.0f;
	if (scene_peak > 0.0f) {
		peak_nits = scene_peak;
	} else if (measured > 0.0f) {
		peak_nits = measured * white_nits;
		if (static_peak > 0.0f) {
			peak_nits = fminf(peak_nits, static_peak);
		}
	} else if (static_peak > 0.0f) {
		peak_nits = static_peak;
	}
	out_source[0] = (float)hdr->transfer;
	out_source[1] = hdr->primaries == XRT_LAYER_PRIMARIES_BT2020 ? 1.0f : 0.0f;
	out_source[2] = white_nits;
	/*
	 * How much of the HDR mapping. A float layer without metadata (a SteamVR game's images) may be an
	 * SDR game drawing into a float image: as SDR while its measured peak stays within the first bins past white
	 * (1.19 x), all HDR from a stop above (2 x), in between a blend; the peak is smoothed, so the change is gradual.
	 * Without the measured peak nothing tells the two apart: all HDR.
	 */
	out_source[3] = 1.0f;
	if (hdr->transfer == XRT_LAYER_TRANSFER_LINEAR && scene_peak <= 0.0f && static_peak <= 0.0f &&
	    s.hdr_measured_peak) {
		const float first_bin = 1.189207f; // 2^(1/4), the top of the first bin past white.
		out_source[3] = fminf(fmaxf((measured - first_bin) / (2.0f - first_bin), 0.0f), 1.0f);
	}

	// The shoulder starts with the slope at white, white * (peak - 1) over the room left, 1 - white, in its own
	// units; above 1 the highlights are compressed (auto_hdr_hdr_lightness in auto_hdr.inc.glsl).
	const float white = fminf(fmaxf(s.hdr_bright ? s.hdr_white_bright : s.hdr_white, 0.5f), 0.98f);
	const float peak = cbrtf(fmaxf(peak_nits, white_nits) / white_nits);
	out_range[0] = white;
	out_range[1] = peak;
	out_range[2] = white * (peak - 1.0f) / (1.0f - white);

	// Shown as HDR (more than half): the Rift tab says so at its HDR Bright switch.
	if (out_source[3] > 0.5f) {
		render_output_note_hdr_layer(os_monotonic_get_ns());
	}
}

bool
render_compute_init(struct render_compute *render, struct render_resources *r)
{
	VkResult ret;

	assert(render->r == NULL);

	struct vk_bundle *vk = r->vk;
	render->r = r;

	for (uint32_t i = 0; i < RENDER_MAX_LAYER_RUNS_COUNT(r); i++) {
		ret = vk_create_descriptor_set(             //
		    vk,                                     // vk_bundle
		    r->compute.descriptor_pool,             // descriptor_pool
		    r->compute.layer.descriptor_set_layout, // descriptor_set_layout
		    &render->layer_descriptor_sets[i]);     // descriptor_set
		VK_CHK_WITH_RET(ret, "vk_create_descriptor_set", false);

		VK_NAME_DESCRIPTOR_SET(vk, render->layer_descriptor_sets[i], "render_compute layer descriptor set");
	}

	ret = vk_create_descriptor_set(                  //
	    vk,                                          // vk_bundle
	    r->compute.descriptor_pool,                  // descriptor_pool
	    r->compute.distortion.descriptor_set_layout, // descriptor_set_layout
	    &render->shared_descriptor_set);             // descriptor_set
	VK_CHK_WITH_RET(ret, "vk_create_descriptor_set", false);

	VK_NAME_DESCRIPTOR_SET(vk, render->shared_descriptor_set, "render_compute shared descriptor set");

	return true;
}

bool
render_compute_begin(struct render_compute *render)
{
	VkResult ret;
	struct vk_bundle *vk = vk_from_render(render);

	ret = vk->vkResetCommandPool(vk->device, render->r->cmd_pool, 0);
	VK_CHK_WITH_RET(ret, "vkResetCommandPool", false);

	VkCommandBufferBeginInfo begin_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
	    .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
	};

	ret = vk->vkBeginCommandBuffer( //
	    render->r->cmd,             //
	    &begin_info);               //
	VK_CHK_WITH_RET(ret, "vkBeginCommandBuffer", false);

	vk->vkCmdResetQueryPool(   //
	    render->r->cmd,        //
	    render->r->query_pool, //
	    0,                     // firstQuery
	    2);                    // queryCount

	vk->vkCmdWriteTimestamp(               //
	    render->r->cmd,                    //
	    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, // pipelineStage
	    render->r->query_pool,             //
	    0);                                // query

	return true;
}

bool
render_compute_end(struct render_compute *render)
{
	struct vk_bundle *vk = vk_from_render(render);
	VkResult ret;

	vk->vkCmdWriteTimestamp(                  //
	    render->r->cmd,                       //
	    VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, // pipelineStage
	    render->r->query_pool,                //
	    1);                                   // query

	ret = vk->vkEndCommandBuffer(render->r->cmd);
	VK_CHK_WITH_RET(ret, "vkEndCommandBuffer", false);

	return true;
}

void
render_compute_fini(struct render_compute *render)
{
	assert(render->r != NULL);

	struct vk_bundle *vk = vk_from_render(render);

	// Reclaimed by vkResetDescriptorPool.
	render->shared_descriptor_set = VK_NULL_HANDLE;
	for (uint32_t i = 0; i < ARRAY_SIZE(render->layer_descriptor_sets); i++) {
		render->layer_descriptor_sets[i] = VK_NULL_HANDLE;
	}

	vk->vkResetDescriptorPool(vk->device, render->r->compute.descriptor_pool, 0);

	render->r = NULL;
}

void
render_compute_layers(struct render_compute *render,
                      VkDescriptorSet descriptor_set,
                      VkBuffer ubo,
                      VkSampler src_samplers[RENDER_MAX_IMAGES_SIZE],
                      VkImageView src_image_views[RENDER_MAX_IMAGES_SIZE],
                      uint32_t num_srcs,
                      VkImageView target_image_view,
                      const struct render_viewport_data *view,
                      bool do_timewarp)
{
	assert(render->r != NULL);

	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;


	/*
	 * Source, target and distortion images.
	 */

	update_compute_layer_descriptor_set( //
	    vk,                              //
	    r->compute.src_binding,          //
	    src_samplers,                    //
	    src_image_views,                 //
	    num_srcs,                        //
	    r->compute.target_binding,       //
	    target_image_view,               //
	    r->compute.ubo_binding,          //
	    ubo,                             //
	    VK_WHOLE_SIZE,                   //
	    r->compute.hdr_stats_binding,    //
	    r->compute.layer.hdr_stats[r->compute.layer.hdr_stats_index].buffer, //
	    descriptor_set);                 //
	r->compute.layer.hdr_stats_pixels += (uint64_t)view->w * view->h;

	VkPipeline pipeline = do_timewarp ? r->compute.layer.timewarp_pipeline : r->compute.layer.non_timewarp_pipeline;
	vk->vkCmdBindPipeline(              //
	    render->r->cmd,                 //
	    VK_PIPELINE_BIND_POINT_COMPUTE, // pipelineBindPoint
	    pipeline);                      // pipeline

	vk->vkCmdBindDescriptorSets(          //
	    r->cmd,                           //
	    VK_PIPELINE_BIND_POINT_COMPUTE,   // pipelineBindPoint
	    r->compute.layer.pipeline_layout, // layout
	    0,                                // firstSet
	    1,                                // descriptorSetCount
	    &descriptor_set,                  // pDescriptorSets
	    0,                                // dynamicOffsetCount
	    NULL);                            // pDynamicOffsets


	uint32_t w = 0, h = 0;
	calc_dispatch_dims_1_view(*view, &w, &h);
	assert(w != 0 && h != 0);

	vk->vkCmdDispatch( //
	    r->cmd,        //
	    w,             // groupCountX
	    h,             // groupCountY
	    1);            // groupCountZ
}

void
render_compute_projection_timewarp(struct render_compute *render,
                                   VkSampler src_samplers[XRT_MAX_VIEWS],
                                   VkImageView src_image_views[XRT_MAX_VIEWS],
                                   const struct xrt_normalized_rect src_norm_rects[XRT_MAX_VIEWS],
                                   const struct xrt_pose src_poses[XRT_MAX_VIEWS],
                                   const struct xrt_fov src_fovs[XRT_MAX_VIEWS],
                                   const struct xrt_pose new_poses_scanout_begin[XRT_MAX_VIEWS],
                                   const struct xrt_pose new_poses_scanout_end[XRT_MAX_VIEWS],
                                   VkImage target_image,
                                   VkImageView target_image_view,
                                   VkImageLayout target_final_layout,
                                   const struct render_viewport_data views[XRT_MAX_VIEWS])
{
	assert(render->r != NULL);
	struct render_resources *r = render->r;


	/*
	 * UBO
	 */

	struct xrt_matrix_4x4 time_warp_matrix_scanout_begin[XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 time_warp_matrix_scanout_end[XRT_MAX_VIEWS];
	for (uint32_t i = 0; i < render->r->view_count; ++i) {
		render_calc_time_warp_matrix(            //
		    &src_poses[i],                       //
		    &src_fovs[i],                        //
		    &new_poses_scanout_begin[i],         //
		    &time_warp_matrix_scanout_begin[i]); //

		render_calc_time_warp_matrix(          //
		    &src_poses[i],                     //
		    &src_fovs[i],                      //
		    &new_poses_scanout_end[i],         //
		    &time_warp_matrix_scanout_end[i]); //
	}

	struct render_compute_distortion_ubo_data *data =
	    (struct render_compute_distortion_ubo_data *)r->compute.distortion.ubo.mapped;
	for (uint32_t i = 0; i < render->r->view_count; ++i) {
		data->views[i] = views[i];
		data->pre_transforms[i] = r->distortion.uv_to_tanangle[i];
		data->transform_timewarp_scanout_begin[i] = time_warp_matrix_scanout_begin[i];
		data->transform_timewarp_scanout_end[i] = time_warp_matrix_scanout_end[i];
		data->post_transforms[i] = src_norm_rects[i];
	}

	dispatch_project_pipeline(render, src_samplers, src_image_views, src_norm_rects, target_image,
	                          target_image_view, target_final_layout, views,
	                          r->compute.distortion.timewarp_pipeline);
}


/*
 * This function is intended to be used on content already timewarped to new_poses_scanout_begin.
 * It performs only the timewarp nesscary to compensate for the time delta between the start and end of
 * scanout.
 */
void
render_compute_projection_scanout_compensation(struct render_compute *render,
                                               VkSampler src_samplers[XRT_MAX_VIEWS],
                                               VkImageView src_image_views[XRT_MAX_VIEWS],
                                               const struct xrt_normalized_rect src_rects[XRT_MAX_VIEWS],
                                               const struct xrt_fov src_fovs[XRT_MAX_VIEWS],
                                               const struct xrt_pose new_poses_scanout_begin[XRT_MAX_VIEWS],
                                               const struct xrt_pose new_poses_scanout_end[XRT_MAX_VIEWS],
                                               VkImage target_image,
                                               VkImageView target_image_view,
                                               VkImageLayout target_final_layout,
                                               const struct render_viewport_data views[XRT_MAX_VIEWS])
{
	assert(render->r != NULL);
	struct render_resources *r = render->r;


	/*
	 * UBO
	 */

	struct xrt_matrix_4x4 time_warp_matrix_scanout_begin[XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 time_warp_matrix_scanout_end[XRT_MAX_VIEWS];
	for (uint32_t i = 0; i < render->r->view_count; ++i) {
		render_calc_time_warp_projection(&src_fovs[i], &time_warp_matrix_scanout_begin[i]);

		render_calc_time_warp_matrix(          //
		    &new_poses_scanout_begin[i],       //
		    &src_fovs[i],                      //
		    &new_poses_scanout_end[i],         //
		    &time_warp_matrix_scanout_end[i]); //
	}

	struct render_compute_distortion_ubo_data *data =
	    (struct render_compute_distortion_ubo_data *)r->compute.distortion.ubo.mapped;
	for (uint32_t i = 0; i < render->r->view_count; ++i) {
		data->views[i] = views[i];
		data->pre_transforms[i] = r->distortion.uv_to_tanangle[i];
		data->transform_timewarp_scanout_begin[i] = time_warp_matrix_scanout_begin[i];
		data->transform_timewarp_scanout_end[i] = time_warp_matrix_scanout_end[i];
		data->post_transforms[i] = src_rects[i];
	}

	dispatch_project_pipeline(render, src_samplers, src_image_views, src_rects, target_image, target_image_view,
	                          target_final_layout, views, r->compute.distortion.timewarp_pipeline);
}

void
render_compute_projection_no_timewarp(struct render_compute *render,
                                      VkSampler src_samplers[XRT_MAX_VIEWS],
                                      VkImageView src_image_views[XRT_MAX_VIEWS],
                                      const struct xrt_normalized_rect src_rects[XRT_MAX_VIEWS],
                                      VkImage target_image,
                                      VkImageView target_image_view,
                                      VkImageLayout target_final_layout,
                                      const struct render_viewport_data views[XRT_MAX_VIEWS])
{
	assert(render->r != NULL);
	struct render_resources *r = render->r;

	dispatch_project_pipeline(render, src_samplers, src_image_views, src_rects, target_image, target_image_view,
	                          target_final_layout, views, r->compute.distortion.pipeline);
}

void
render_compute_clear(struct render_compute *render,
                     VkImage target_image,
                     VkImageView target_image_view,
                     VkImageLayout target_final_layout,
                     const struct render_viewport_data views[XRT_MAX_VIEWS])
{
	assert(render->r != NULL);

	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;


	/*
	 * UBO
	 */

	// Calculate transforms.
	struct xrt_matrix_4x4 transforms[XRT_MAX_VIEWS];
	for (uint32_t i = 0; i < render->r->view_count; i++) {
		math_matrix_4x4_identity(&transforms[i]);
	}

	struct render_compute_distortion_ubo_data *data =
	    (struct render_compute_distortion_ubo_data *)r->compute.clear.ubo.mapped;
	for (uint32_t i = 0; i < render->r->view_count; ++i) {
		data->views[i] = views[i];
	}

	/*
	 * Source, target and distortion images.
	 */

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	vk_cmd_image_barrier_gpu_locked( //
	    vk,                          //
	    r->cmd,                      //
	    target_image,                //
	    0,                           //
	    VK_ACCESS_SHADER_WRITE_BIT,  //
	    VK_IMAGE_LAYOUT_UNDEFINED,   //
	    VK_IMAGE_LAYOUT_GENERAL,     //
	    subresource_range);          //

	VkSampler sampler = r->samplers.mock;
	VkSampler src_samplers[XRT_MAX_VIEWS];
	VkImageView src_image_views[XRT_MAX_VIEWS];
	VkSampler distortion_samplers[3 * XRT_MAX_VIEWS];
	for (uint32_t i = 0; i < render->r->view_count; ++i) {
		src_samplers[i] = sampler;
		src_image_views[i] = r->mock.color.image_view;
		distortion_samplers[3 * i + 0] = sampler;
		distortion_samplers[3 * i + 1] = sampler;
		distortion_samplers[3 * i + 2] = sampler;
	}

	update_compute_shared_descriptor_set( //
	    vk,                               //
	    r->compute.src_binding,           //
	    src_samplers,                     //
	    src_image_views,                  //
	    r->compute.distortion_binding,    //
	    distortion_samplers,              //
	    r->distortion.image_views,        //
	    r->compute.target_binding,        //
	    target_image_view,                //
	    r->compute.ubo_binding,           //
	    r->compute.clear.ubo.buffer,      //
	    VK_WHOLE_SIZE,                    // ubo_size
	    render->shared_descriptor_set,    // descriptor_set
	    render->r->view_count);           //

	vk->vkCmdBindPipeline(              //
	    r->cmd,                         //
	    VK_PIPELINE_BIND_POINT_COMPUTE, // pipelineBindPoint
	    r->compute.clear.pipeline);     // pipeline

	vk->vkCmdBindDescriptorSets(               //
	    r->cmd,                                //
	    VK_PIPELINE_BIND_POINT_COMPUTE,        // pipelineBindPoint
	    r->compute.distortion.pipeline_layout, // layout
	    0,                                     // firstSet
	    1,                                     // descriptorSetCount
	    &render->shared_descriptor_set,        // pDescriptorSets
	    0,                                     // dynamicOffsetCount
	    NULL);                                 // pDynamicOffsets


	uint32_t w = 0, h = 0;
	calc_dispatch_dims_views(views, render->r->view_count, &w, &h);
	assert(w != 0 && h != 0);

	vk->vkCmdDispatch(  //
	    r->cmd,         //
	    w,              // groupCountX
	    h,              // groupCountY
	    r->view_count); // groupCountZ

	VkImageMemoryBarrier memoryBarrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_MEMORY_READ_BIT,
	    .oldLayout = VK_IMAGE_LAYOUT_GENERAL,
	    .newLayout = target_final_layout,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = target_image,
	    .subresourceRange = subresource_range,
	};

	vk->vkCmdPipelineBarrier(                 //
	    r->cmd,                               //
	    VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, //
	    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,    //
	    0,                                    //
	    0,                                    //
	    NULL,                                 //
	    0,                                    //
	    NULL,                                 //
	    1,                                    //
	    &memoryBarrier);                      //
}
