// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Backdrop blur of layers (RiftLinux, XR_MNDX_composition_layer_backdrop_blur): frosted glass.
 * @ingroup aux_render
 *
 * Used for a view only while one of its layers asks for it; otherwise nothing here runs or exists (everything is made
 * at the first use). For each blurred layer (the lowest RENDER_BACKDROP_MAX_BLURS of a view): a pre-pass composes the
 * layers under it at a quarter of the resolution (layer.comp's second build, four samples a pixel, linear light) into a
 * small image, two Gaussian passes (backdrop_blur.comp) blur it into that layer's blurred image, which is put in the
 * layer pass's source images; the view's layer pass then runs with the second build, which mixes each blurred image
 * under its layer by the layer's alpha. The view's other layers and every other view are composed as before.
 * The setting up (its pipelines take about a second to make) runs on a thread of its own from the first layer that
 * asks; until it is done such layers are shown without the blur, so the compositor never waits for it.
 */

#include "render/render_interface.h"

#include "math/m_api.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "vk/vk_mini_helpers.h"

#include "cache/render_compute_pipeline_cache.h"
#include "cache/render_shader_specialization_helpers.h"

#include "backdrop_blur.comp.h"
#include "layer_blur.comp.h"

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>


#define DOWNSAMPLE 4
#define MAX_SIGMA_SMALL 16.0f
#define BINDING_BACKDROP_UBO 6

//! layer.comp's BackdropConfig (std140).
struct backdrop_ubo_blur
{
	int32_t index[4];
	float params[4];
	float tint[4];
};

struct backdrop_ubo
{
	int32_t info[4];
	struct backdrop_ubo_blur blurs[RENDER_BACKDROP_MAX_BLURS];
};

//! backdrop_blur.comp's push constants.
struct blur_push
{
	int32_t size[2];
	int32_t dir[2];
	float sigma;
	int32_t radius;
};

struct backdrop_image
{
	VkDeviceMemory memory;
	VkImage image;
	VkImageView view;
};

struct backdrop_view
{
	//! The pre-pass's small image, then the result of the first blur pass.
	struct backdrop_image small[2];
	//! Each blurred layer's blurred backdrop.
	struct backdrop_image blurred[RENDER_BACKDROP_MAX_BLURS];
	struct render_buffer ubo;
	VkDescriptorSet main_set;
	VkDescriptorSet prepass_sets[RENDER_BACKDROP_MAX_BLURS];
	VkDescriptorSet blur_sets[RENDER_BACKDROP_MAX_BLURS][2];
};

enum backdrop_state
{
	BACKDROP_SETTING_UP,
	BACKDROP_READY,
	BACKDROP_FAILED,
};

struct render_backdrop
{
	struct render_resources *r;
	pthread_t thread;
	bool thread_started;
	//! @ref backdrop_state, written by the setting-up thread.
	atomic_int state;
	bool images_ready;
	VkExtent2D small_extent;

	VkShaderModule layer_module;
	VkShaderModule blur_module;
	VkDescriptorSetLayout layer_set_layout;
	VkDescriptorSetLayout blur_set_layout;
	VkPipelineLayout layer_pipeline_layout;
	VkPipelineLayout blur_pipeline_layout;
	struct render_layer_pipeline_cache *layer_cache;
	VkPipeline layer_pipeline[2]; // without and with timewarp
	VkPipeline blur_pipeline;
	VkDescriptorPool pool;

	struct backdrop_view views[XRT_MAX_VIEWS];
};


/*
 *
 * Setting up, at the first use.
 *
 */

static VkResult
make_module(struct vk_bundle *vk, const uint32_t *code, size_t size, VkShaderModule *out)
{
	VkShaderModuleCreateInfo info = {
	    .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
	    .codeSize = size,
	    .pCode = code,
	};
	return vk->vkCreateShaderModule(vk->device, &info, NULL, out);
}

static VkResult
make_pipeline_layout(struct vk_bundle *vk, VkDescriptorSetLayout set_layout, uint32_t push_size, VkPipelineLayout *out)
{
	VkPushConstantRange range = {
	    .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    .offset = 0,
	    .size = push_size,
	};
	VkPipelineLayoutCreateInfo info = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
	    .setLayoutCount = 1,
	    .pSetLayouts = &set_layout,
	    .pushConstantRangeCount = 1,
	    .pPushConstantRanges = &range,
	};
	return vk->vkCreatePipelineLayout(vk->device, &info, NULL, out);
}

static VkResult
make_image(struct vk_bundle *vk, VkExtent2D extent, struct backdrop_image *img)
{
	const VkFormat format = VK_FORMAT_R16G16B16A16_SFLOAT;
	VkResult ret = vk_create_image_simple(vk, extent, format, VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
	                                      &img->memory, &img->image);
	if (ret != VK_SUCCESS) {
		return ret;
	}
	VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	return vk_create_view(vk, img->image, VK_IMAGE_VIEW_TYPE_2D, format, range, &img->view);
}

static bool
setup(struct render_resources *r, struct render_backdrop *b, VkExtent2D small_extent)
{
	struct vk_bundle *vk = r->vk;
	VkResult ret;

	ret = make_module(vk, layer_blur_comp, sizeof(layer_blur_comp), &b->layer_module);
	VK_CHK_WITH_RET(ret, "vkCreateShaderModule (layer, backdrop blur)", false);
	ret = make_module(vk, backdrop_blur_comp, sizeof(backdrop_blur_comp), &b->blur_module);
	VK_CHK_WITH_RET(ret, "vkCreateShaderModule (backdrop blur)", false);

	// The layer pass's bindings as render_resources.c makes them, and the blur settings.
	VkDescriptorSetLayoutBinding layer_bindings[5] = {
	    {r->compute.src_binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, r->compute.layer.image_array_size,
	     VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {r->compute.target_binding, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {r->compute.ubo_binding, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {r->compute.hdr_stats_binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {BINDING_BACKDROP_UBO, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	};
	VkDescriptorSetLayoutCreateInfo layer_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(layer_bindings),
	    .pBindings = layer_bindings,
	};
	ret = vk->vkCreateDescriptorSetLayout(vk->device, &layer_layout_info, NULL, &b->layer_set_layout);
	VK_CHK_WITH_RET(ret, "vkCreateDescriptorSetLayout (layer, backdrop blur)", false);

	VkDescriptorSetLayoutBinding blur_bindings[2] = {
	    {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	};
	VkDescriptorSetLayoutCreateInfo blur_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(blur_bindings),
	    .pBindings = blur_bindings,
	};
	ret = vk->vkCreateDescriptorSetLayout(vk->device, &blur_layout_info, NULL, &b->blur_set_layout);
	VK_CHK_WITH_RET(ret, "vkCreateDescriptorSetLayout (backdrop blur)", false);

	ret = make_pipeline_layout(vk, b->layer_set_layout, sizeof(int32_t), &b->layer_pipeline_layout);
	VK_CHK_WITH_RET(ret, "vkCreatePipelineLayout (layer, backdrop blur)", false);
	ret = make_pipeline_layout(vk, b->blur_set_layout, sizeof(struct blur_push), &b->blur_pipeline_layout);
	VK_CHK_WITH_RET(ret, "vkCreatePipelineLayout (backdrop blur)", false);

	// The same specialisations as the layer pass's own pipelines (render_resources.c).
	xrt_result_t xret = render_layer_pipeline_cache_create("render_backdrop layer", &b->layer_cache);
	if (xret != XRT_SUCCESS) {
		return false;
	}
	xret = render_layer_pipeline_cache_init(b->layer_cache, b->layer_module, b->layer_pipeline_layout,
	                                        VK_NULL_HANDLE);
	if (xret != XRT_SUCCESS) {
		return false;
	}
	for (int timewarp = 0; timewarp < 2; timewarp++) {
		const struct render_layer_spec spec =
		    render_make_layer_spec(timewarp ? VK_TRUE : VK_FALSE, VK_TRUE, r->compute.layer.image_array_size,
		                           RENDER_LAYER_DEFAULT_INSET_BLEND_EDGE);
		xret = render_layer_pipeline_cache_get(b->layer_cache, vk, &spec, &b->layer_pipeline[timewarp]);
		if (xret != XRT_SUCCESS) {
			return false;
		}
	}
	ret = vk_create_compute_pipeline(vk, VK_NULL_HANDLE, b->blur_module, b->blur_pipeline_layout, NULL,
	                                 &b->blur_pipeline);
	VK_CHK_WITH_RET(ret, "vk_create_compute_pipeline (backdrop blur)", false);

	// Per view: the main pass's set and a pre-pass set per blur (layer pass layout), two blur sets per blur.
	const uint32_t layer_sets = r->view_count * (1 + RENDER_BACKDROP_MAX_BLURS);
	const uint32_t blur_sets = r->view_count * RENDER_BACKDROP_MAX_BLURS * 2;
	VkDescriptorPoolSize sizes[] = {
	    {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, layer_sets * r->compute.layer.image_array_size + blur_sets},
	    {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, layer_sets + blur_sets},
	    {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, layer_sets * 2},
	    {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, layer_sets},
	};
	VkDescriptorPoolCreateInfo pool_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
	    .maxSets = layer_sets + blur_sets,
	    .poolSizeCount = ARRAY_SIZE(sizes),
	    .pPoolSizes = sizes,
	};
	ret = vk->vkCreateDescriptorPool(vk->device, &pool_info, NULL, &b->pool);
	VK_CHK_WITH_RET(ret, "vkCreateDescriptorPool (backdrop blur)", false);

	b->small_extent = small_extent;
	for (uint32_t v = 0; v < r->view_count; v++) {
		struct backdrop_view *bv = &b->views[v];
		for (int i = 0; i < 2; i++) {
			ret = make_image(vk, small_extent, &bv->small[i]);
			VK_CHK_WITH_RET(ret, "backdrop blur image", false);
		}
		for (int i = 0; i < RENDER_BACKDROP_MAX_BLURS; i++) {
			ret = make_image(vk, small_extent, &bv->blurred[i]);
			VK_CHK_WITH_RET(ret, "backdrop blur image", false);
		}
		ret = render_buffer_init(vk, &bv->ubo, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
		                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		                         sizeof(struct backdrop_ubo));
		VK_CHK_WITH_RET(ret, "render_buffer_init (backdrop blur)", false);
		ret = render_buffer_map(vk, &bv->ubo);
		VK_CHK_WITH_RET(ret, "render_buffer_map (backdrop blur)", false);

		ret = vk_create_descriptor_set(vk, b->pool, b->layer_set_layout, &bv->main_set);
		VK_CHK_WITH_RET(ret, "vk_create_descriptor_set (backdrop blur)", false);
		for (int i = 0; i < RENDER_BACKDROP_MAX_BLURS; i++) {
			ret = vk_create_descriptor_set(vk, b->pool, b->layer_set_layout, &bv->prepass_sets[i]);
			VK_CHK_WITH_RET(ret, "vk_create_descriptor_set (backdrop blur)", false);
			for (int d = 0; d < 2; d++) {
				ret = vk_create_descriptor_set(vk, b->pool, b->blur_set_layout, &bv->blur_sets[i][d]);
				VK_CHK_WITH_RET(ret, "vk_create_descriptor_set (backdrop blur)", false);
			}
		}
	}

	U_LOG_I("Backdrop blur: first used by a layer; set up for %u view(s) at %ux%u", r->view_count,
	        small_extent.width, small_extent.height);
	return true;
}

//! The setting up, on its own thread: Vulkan objects of its own only (device-level calls, no queue).
static void *
setup_thread(void *ptr)
{
	struct render_backdrop *b = ptr;
	const bool ok = setup(b->r, b, b->small_extent);
	if (!ok) {
		U_LOG_W("Backdrop blur: could not be set up; layers that ask for it are shown without it");
	}
	atomic_store(&b->state, ok ? BACKDROP_READY : BACKDROP_FAILED);
	return NULL;
}


/*
 *
 * Recording.
 *
 */

//! Every compute write and read before it done before the next compute dispatch reads or writes.
static void
compute_barrier(struct vk_bundle *vk, VkCommandBuffer cmd)
{
	VkMemoryBarrier barrier = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT,
	    .dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
	};
	vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
	                         &barrier, 0, NULL, 0, NULL);
}

//! The small images into the general layout, once.
static void
images_to_general(struct render_resources *r, struct render_backdrop *b)
{
	struct vk_bundle *vk = r->vk;
	VkImageMemoryBarrier barriers[XRT_MAX_VIEWS * (2 + RENDER_BACKDROP_MAX_BLURS)];
	uint32_t n = 0;
	for (uint32_t v = 0; v < r->view_count; v++) {
		struct backdrop_view *bv = &b->views[v];
		VkImage images[2 + RENDER_BACKDROP_MAX_BLURS] = {bv->small[0].image, bv->small[1].image};
		for (int i = 0; i < RENDER_BACKDROP_MAX_BLURS; i++) {
			images[2 + i] = bv->blurred[i].image;
		}
		for (int i = 0; i < 2 + RENDER_BACKDROP_MAX_BLURS; i++) {
			barriers[n++] = (VkImageMemoryBarrier){
			    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
			    .srcAccessMask = 0,
			    .dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
			    .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
			    .newLayout = VK_IMAGE_LAYOUT_GENERAL,
			    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
			    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
			    .image = images[i],
			    .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
			};
		}
	}
	vk->vkCmdPipelineBarrier(r->cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0,
	                         NULL, 0, NULL, n, barriers);
}

//! A layer pass set: the layer pass's own bindings (sources, @p first_general on in the general layout), the blur's.
static void
write_layer_set(struct render_resources *r,
                VkDescriptorSet set,
                const VkSampler *samplers,
                const VkImageView *views,
                uint32_t first_general,
                VkImageView target,
                VkBuffer ubo,
                VkBuffer backdrop_ubo)
{
	struct vk_bundle *vk = r->vk;
	const uint32_t count = r->compute.layer.image_array_size;
	VkDescriptorImageInfo src_infos[RENDER_MAX_IMAGES_SIZE];
	for (uint32_t i = 0; i < count; i++) {
		src_infos[i] = (VkDescriptorImageInfo){
		    .sampler = samplers[i],
		    .imageView = views[i],
		    .imageLayout = i >= first_general ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
		};
	}
	VkDescriptorImageInfo target_info = {.imageView = target, .imageLayout = VK_IMAGE_LAYOUT_GENERAL};
	VkDescriptorBufferInfo ubo_info = {ubo, 0, VK_WHOLE_SIZE};
	VkDescriptorBufferInfo stats_info = {r->compute.layer.hdr_stats[r->compute.layer.hdr_stats_index].buffer, 0,
	                                     VK_WHOLE_SIZE};
	VkDescriptorBufferInfo backdrop_info = {backdrop_ubo, 0, VK_WHOLE_SIZE};
	VkWriteDescriptorSet writes[5] = {
	    {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, NULL, set, r->compute.src_binding, 0, count,
	     VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, src_infos, NULL, NULL},
	    {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, NULL, set, r->compute.target_binding, 0, 1,
	     VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &target_info, NULL, NULL},
	    {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, NULL, set, r->compute.ubo_binding, 0, 1,
	     VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, NULL, &ubo_info, NULL},
	    {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, NULL, set, r->compute.hdr_stats_binding, 0, 1,
	     VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, NULL, &stats_info, NULL},
	    {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, NULL, set, BINDING_BACKDROP_UBO, 0, 1,
	     VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, NULL, &backdrop_info, NULL},
	};
	vk->vkUpdateDescriptorSets(vk->device, ARRAY_SIZE(writes), writes, 0, NULL);
}

static void
write_blur_set(struct render_resources *r, VkDescriptorSet set, VkImageView src, VkImageView dst)
{
	struct vk_bundle *vk = r->vk;
	VkDescriptorImageInfo src_info = {r->samplers.clamp_to_edge, src, VK_IMAGE_LAYOUT_GENERAL};
	VkDescriptorImageInfo dst_info = {VK_NULL_HANDLE, dst, VK_IMAGE_LAYOUT_GENERAL};
	VkWriteDescriptorSet writes[2] = {
	    {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, NULL, set, 0, 0, 1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	     &src_info, NULL, NULL},
	    {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, NULL, set, 1, 0, 1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &dst_info, NULL,
	     NULL},
	};
	vk->vkUpdateDescriptorSets(vk->device, ARRAY_SIZE(writes), writes, 0, NULL);
}

static void
dispatch_layer_pass(struct render_resources *r,
                    struct render_backdrop *b,
                    VkDescriptorSet set,
                    bool do_timewarp,
                    int32_t pass,
                    uint32_t w,
                    uint32_t h)
{
	struct vk_bundle *vk = r->vk;
	vk->vkCmdBindPipeline(r->cmd, VK_PIPELINE_BIND_POINT_COMPUTE, b->layer_pipeline[do_timewarp ? 1 : 0]);
	vk->vkCmdBindDescriptorSets(r->cmd, VK_PIPELINE_BIND_POINT_COMPUTE, b->layer_pipeline_layout, 0, 1, &set, 0,
	                            NULL);
	vk->vkCmdPushConstants(r->cmd, b->layer_pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pass), &pass);
	vk->vkCmdDispatch(r->cmd, (w + 7) / 8, (h + 7) / 8, 1);
}

static void
dispatch_blur(struct render_resources *r, struct render_backdrop *b, VkDescriptorSet set, const struct blur_push *push)
{
	struct vk_bundle *vk = r->vk;
	vk->vkCmdBindPipeline(r->cmd, VK_PIPELINE_BIND_POINT_COMPUTE, b->blur_pipeline);
	vk->vkCmdBindDescriptorSets(r->cmd, VK_PIPELINE_BIND_POINT_COMPUTE, b->blur_pipeline_layout, 0, 1, &set, 0,
	                            NULL);
	vk->vkCmdPushConstants(r->cmd, b->blur_pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(*push), push);
	vk->vkCmdDispatch(r->cmd, ((uint32_t)push->size[0] + 7) / 8, ((uint32_t)push->size[1] + 7) / 8, 1);
}


/*
 *
 * 'Exported' functions.
 *
 */

bool
render_backdrop_layers(struct render_compute *render,
                       uint32_t view_index,
                       VkBuffer ubo,
                       const VkSampler src_samplers[RENDER_MAX_IMAGES_SIZE],
                       const VkImageView src_image_views[RENDER_MAX_IMAGES_SIZE],
                       uint32_t used_images,
                       const struct render_backdrop_request *requests,
                       uint32_t request_count,
                       VkImageView target_image_view,
                       const struct render_viewport_data *view,
                       bool do_timewarp)
{
	struct render_resources *r = render->r;
	struct vk_bundle *vk = r->vk;
	const uint32_t count = MIN(MIN(request_count, (uint32_t)RENDER_BACKDROP_MAX_BLURS),
	                           r->compute.layer.image_array_size - MIN(used_images, r->compute.layer.image_array_size));
	if (count == 0 || view_index >= r->view_count || view->w == 0 || view->h == 0) {
		return false;
	}
	const VkExtent2D small = {(view->w + DOWNSAMPLE - 1) / DOWNSAMPLE, (view->h + DOWNSAMPLE - 1) / DOWNSAMPLE};

	if (r->backdrop == NULL) {
		// The first layer that asks: set up on a thread of its own, this frame without the blur.
		struct render_backdrop *nb = U_TYPED_CALLOC(struct render_backdrop);
		if (nb == NULL) {
			return false;
		}
		nb->r = r;
		nb->small_extent = small;
		atomic_init(&nb->state, BACKDROP_SETTING_UP);
		r->backdrop = nb;
		if (pthread_create(&nb->thread, NULL, setup_thread, nb) == 0) {
			nb->thread_started = true;
		} else {
			atomic_store(&nb->state, BACKDROP_FAILED);
		}
		return false;
	}
	struct render_backdrop *b = r->backdrop;
	if (atomic_load(&b->state) != BACKDROP_READY || small.width > b->small_extent.width ||
	    small.height > b->small_extent.height) {
		return false;
	}
	if (!b->images_ready) {
		images_to_general(r, b);
		b->images_ready = true;
	}
	struct backdrop_view *bv = &b->views[view_index];

	// The sources with each blurred backdrop after the layers' own images.
	VkSampler samplers[RENDER_MAX_IMAGES_SIZE];
	VkImageView views[RENDER_MAX_IMAGES_SIZE];
	memcpy(samplers, src_samplers, sizeof(VkSampler) * r->compute.layer.image_array_size);
	memcpy(views, src_image_views, sizeof(VkImageView) * r->compute.layer.image_array_size);
	for (uint32_t i = 0; i < count; i++) {
		samplers[used_images + i] = r->samplers.clamp_to_edge;
		views[used_images + i] = bv->blurred[i].view;
	}

	struct backdrop_ubo *u = bv->ubo.mapped;
	memset(u, 0, sizeof(*u));
	u->info[0] = (int32_t)count;
	u->info[1] = DOWNSAMPLE;
	for (uint32_t i = 0; i < count; i++) {
		u->blurs[i].index[0] = (int32_t)requests[i].layer;
		u->blurs[i].index[1] = (int32_t)(used_images + i);
		u->blurs[i].index[2] = (int32_t)requests[i].layer;
		u->blurs[i].params[0] = requests[i].mask_alpha;
		u->blurs[i].params[1] = requests[i].saturation;
		u->blurs[i].params[2] = (float)small.width / (float)b->small_extent.width;
		u->blurs[i].params[3] = (float)small.height / (float)b->small_extent.height;
		u->blurs[i].tint[0] = requests[i].tint.r;
		u->blurs[i].tint[1] = requests[i].tint.g;
		u->blurs[i].tint[2] = requests[i].tint.b;
		u->blurs[i].tint[3] = requests[i].tint.a;
	}

	for (uint32_t i = 0; i < count; i++) {
		// What is under the layer, small; then across and down into its blurred backdrop.
		write_layer_set(r, bv->prepass_sets[i], samplers, views, used_images, bv->small[0].view, ubo,
		                bv->ubo.buffer);
		write_blur_set(r, bv->blur_sets[i][0], bv->small[0].view, bv->small[1].view);
		write_blur_set(r, bv->blur_sets[i][1], bv->small[1].view, bv->blurred[i].view);

		compute_barrier(vk, r->cmd);
		dispatch_layer_pass(r, b, bv->prepass_sets[i], do_timewarp, (int32_t)i, small.width, small.height);

		const float sigma = fminf(requests[i].sigma_px / (float)DOWNSAMPLE, MAX_SIGMA_SMALL);
		struct blur_push push = {
		    .size = {(int32_t)small.width, (int32_t)small.height},
		    .dir = {1, 0},
		    .sigma = fmaxf(sigma, 0.5f),
		    .radius = (int32_t)ceilf(3.0f * fmaxf(sigma, 0.5f)),
		};
		compute_barrier(vk, r->cmd);
		dispatch_blur(r, b, bv->blur_sets[i][0], &push);
		push.dir[0] = 0;
		push.dir[1] = 1;
		compute_barrier(vk, r->cmd);
		dispatch_blur(r, b, bv->blur_sets[i][1], &push);
	}

	// The view's layers, each blurred one over its blurred backdrop.
	write_layer_set(r, bv->main_set, samplers, views, used_images, target_image_view, ubo, bv->ubo.buffer);
	compute_barrier(vk, r->cmd);
	dispatch_layer_pass(r, b, bv->main_set, do_timewarp, -1, view->w, view->h);
	r->compute.layer.hdr_stats_pixels += (uint64_t)view->w * view->h;
	return true;
}

void
render_backdrop_fini(struct render_resources *r)
{
	struct render_backdrop *b = r->backdrop;
	if (b == NULL) {
		return;
	}
	if (b->thread_started) {
		pthread_join(b->thread, NULL);
	}
	struct vk_bundle *vk = r->vk;
	for (uint32_t v = 0; v < XRT_MAX_VIEWS; v++) {
		struct backdrop_view *bv = &b->views[v];
		struct backdrop_image *images[2 + RENDER_BACKDROP_MAX_BLURS] = {&bv->small[0], &bv->small[1]};
		for (int i = 0; i < RENDER_BACKDROP_MAX_BLURS; i++) {
			images[2 + i] = &bv->blurred[i];
		}
		for (int i = 0; i < 2 + RENDER_BACKDROP_MAX_BLURS; i++) {
			D(ImageView, images[i]->view);
			D(Image, images[i]->image);
			DF(Memory, images[i]->memory);
		}
		if (bv->ubo.buffer != VK_NULL_HANDLE) {
			render_buffer_fini(vk, &bv->ubo);
		}
	}
	D(DescriptorPool, b->pool);
	D(Pipeline, b->blur_pipeline);
	if (b->layer_cache != NULL) {
		render_layer_pipeline_cache_destroy(&b->layer_cache, vk);
	}
	D(PipelineLayout, b->layer_pipeline_layout);
	D(PipelineLayout, b->blur_pipeline_layout);
	D(DescriptorSetLayout, b->layer_set_layout);
	D(DescriptorSetLayout, b->blur_set_layout);
	D(ShaderModule, b->layer_module);
	D(ShaderModule, b->blur_module);
	free(b);
	r->backdrop = NULL;
}
