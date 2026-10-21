// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Motion smoothing for the compute path: in-between frames when an application is late (RiftLinux).
 *
 * When the compositor would show an application's last frame again, it shows that frame moved on by the motion seen
 * between the application's last two frames, for the time that has passed since it arrived, as Oculus' ASW and
 * SteamVR's Motion Smoothing do. Designed and scored on recorded No Man's Sky
 * frames:
 *
 * - Each new frame is copied (in-between frames are made from the copy, never from the application's image) and
 *   reduced to half size; the previous one is turned to the new frame's head orientation (what is left is the scene's
 *   own motion and the head's translation).
 * - Motion between the two: NVIDIA's optical flow engine (VK_NV_optical_flow, RTX 30 and newer) on its own queue, one
 *   vector per 4x4 pixels of half size, started right after the frame's work (render_motion_signal_semaphore,
 *   render_motion_submitted) and ready for the next refresh. Elsewhere block matching (coarse to fine, blocks of 8),
 *   at the first refresh that needs it.
 * - Vectors under 0.75 pixels are dropped (noise of still scenes).
 * - In-between frame: each pixel shows the content that has moved there, found by a few fixed-point steps on the
 *   smooth (bilinear) field: no blocks, a gentle stretch where motion changes. The distortion pass samples it instead
 *   of the application's image, with the pose it was made for; timewarp turns it to the display's pose as before.
 *
 * Off unless render_output_settings::motion_smoothing (XRT_COMPOSITOR_MOTION_SMOOTHING) is set. Tests, capture and
 * the recorder of real frames: see the options below.
 *
 * @ingroup aux_render
 */

#include "render/render_interface.h"
#include "render/render_output_settings.h"

#include "math/m_api.h"
#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "os/os_time.h"
#include "util/u_time.h"
#include "vk/vk_mini_helpers.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <pthread.h>
#include <stdatomic.h>

/*
 * Capture for tests: XRT_COMPOSITOR_MOTION_CAPTURE captures, one every 90 new frames from the
 * XRT_COMPOSITOR_MOTION_CAPTURE_START'th new frame on (default 900), to $XDG_CACHE_HOME/riftlinux/motion-capture (else
 * ~/.cache/...), taken at an in-between frame: the inputs (half size), the flow or fields, the field used, the frame
 * itself, the in-between frame (and in the tests the held-back frame). Files "RLMC", width, height, format (1 RGBA8,
 * 2 RGBA16F, 3 R32UI, 4 RG16 signed 10.5) as uint32, then the pixels, rows top down; NN-info.txt holds the numbers.
 */
/*
 * Per-pixel motion (motion_pick.comp): each pixel of half size takes the best matching of the vectors of the field
 * cells around it, so motion does not smear across the edge of moving things (XRT_COMPOSITOR_MOTION_PICK, default on;
 * 0 = the smooth field between cell centres, as before).
 */
DEBUG_GET_ONCE_BOOL_OPTION(motion_pick, "XRT_COMPOSITOR_MOTION_PICK", true)
#define MOTION_PICK_LAMBDA 1.0f

DEBUG_GET_ONCE_NUM_OPTION(motion_capture, "XRT_COMPOSITOR_MOTION_CAPTURE", 0)
DEBUG_GET_ONCE_NUM_OPTION(motion_capture_start, "XRT_COMPOSITOR_MOTION_CAPTURE_START", 900)

/*
 * Tests. XRT_COMPOSITOR_MOTION_TEST_HALF: every other new frame is held back and an in-between frame shown instead, as
 * for an application at half rate (the capture then also keeps the held-back frame: the truth to compare with).
 * XRT_COMPOSITOR_MOTION_TEST_YAW_MDEG: the previous frame is turned by this many millidegrees about the view's up
 * axis before matching, a known motion for checks of the passes against the model (it shows as a sideways drift:
 * never while someone wears the headset). XRT_COMPOSITOR_MOTION_TEST_SHADOW: as the half-rate test, but the in-between
 * frames are only made, timed and captured; the headset shows the real frames (safe while worn).
 */
DEBUG_GET_ONCE_BOOL_OPTION(motion_test_half, "XRT_COMPOSITOR_MOTION_TEST_HALF", false)
DEBUG_GET_ONCE_NUM_OPTION(motion_test_yaw_mdeg, "XRT_COMPOSITOR_MOTION_TEST_YAW_MDEG", 0)
DEBUG_GET_ONCE_BOOL_OPTION(motion_test_shadow, "XRT_COMPOSITOR_MOTION_TEST_SHADOW", false)

/*
 * Recording real frames for developing the motion: XRT_COMPOSITOR_MOTION_RECORD bursts of
 * XRT_COMPOSITOR_MOTION_RECORD_FRAMES consecutive new frames of the left eye (half size, sRGB, as the pyramid's finest
 * level), one burst every XRT_COMPOSITOR_MOTION_RECORD_EVERY_S seconds, with each frame's pose, field of view and
 * time, and the burst's last frame at full size. Nothing is shown differently; works with motion smoothing off. Files
 * in $XDG_CACHE_HOME/riftlinux/motion-record (else ~/.cache/...), written by a thread of their own (no stall):
 * bNN-fMM-l1.rlmc, bNN-full.rlmc, bNN-info.txt.
 */
DEBUG_GET_ONCE_NUM_OPTION(motion_record, "XRT_COMPOSITOR_MOTION_RECORD", 0)
DEBUG_GET_ONCE_NUM_OPTION(motion_record_frames, "XRT_COMPOSITOR_MOTION_RECORD_FRAMES", 8)
DEBUG_GET_ONCE_NUM_OPTION(motion_record_every_s, "XRT_COMPOSITOR_MOTION_RECORD_EVERY_S", 5)

//! XRT_COMPOSITOR_MOTION_FLOW: 1 = NVIDIA's optical flow engine when the GPU has it (default), 0 = block matching.
DEBUG_GET_ONCE_NUM_OPTION(motion_flow, "XRT_COMPOSITOR_MOTION_FLOW", 1)

#define MOTION_LEVELS 3
#define MOTION_BLOCK 8
#define MOTION_MAX_SETS 40
#define MOTION_CAPTURE_ITEMS 16
#define MOTION_RECORD_FRAMES_MAX 16

//! Charge per pixel of offset in block matching (flat areas keep no motion).
#define MOTION_LAMBDA 0.002f

//! XRT_COMPOSITOR_MOTION_SUBPIXEL: block matching's finest answers to a fraction of a pixel (motion_search.comp). On
//! recorded No Man's Sky sessions (54 frame triplets): error 2.99 -> 2.97 levels, damage 0.28 -> 0.26 %,
//! better on 76 % of them; NVIDIA's optical flow on the same frames 3.02 / 0.27 %.
DEBUG_GET_ONCE_BOOL_OPTION(motion_subpixel, "XRT_COMPOSITOR_MOTION_SUBPIXEL", true)
//! Search radius at the coarsest level (eighth size: 8 pixels there are 64 of the frame).
#define MOTION_RADIUS_TOP 8
//! Search radius around each candidate at the finer levels.
#define MOTION_RADIUS_REFINE 2
//! The optical flow's output: one vector per this many pixels (of half size) each way.
#define MOTION_FLOW_GRID 4
//! Vectors shorter than this (pixels of half size) are dropped.
#define MOTION_MIN_VECTOR 0.75f
//! Fixed-point steps of the in-between frame.
#define MOTION_STEPS 4
//! Optical flow is measured for new frames while frames were shown again this recently.
#define MOTION_FLOW_KEEP_NS ((int64_t)2 * U_TIME_1S_IN_NS)

enum motion_format
{
	MOTION_RGBA8 = 1,
	MOTION_RGBA16F = 2,
	MOTION_R32UI = 3,
	MOTION_FLOW_S10_5 = 4,
	MOTION_COST_U8 = 5,
};

//! Matches MotionPush in motion.inc.glsl.
struct motion_push
{
	float v0[4];
	float v1[4];
	float v2[4];
	float v3[4];
	int32_t i0[4];
	int32_t i1[4];
};

struct motion_image
{
	VkImage image;
	VkDeviceMemory memory;
	VkImageView view;
	enum motion_format format;
	uint32_t width, height;
	VkImageLayout layout;
};

struct motion_buffer
{
	VkBuffer buffer;
	VkDeviceMemory memory;
	VkDeviceSize size;
};

struct motion_frame
{
	bool valid;
	//! The frame as handed in (image, view, rect, pose, fov).
	struct render_motion_source src;
	int64_t first_ns; //!< Display time of the refresh it was first shown at.
};

//! NVIDIA's optical flow for one view.
struct motion_flow
{
	bool made;
	//! Half size, grid, brightness row pitch in bytes.
	uint32_t w, h, gw, gh, row;
#ifdef VK_NV_optical_flow
	VkOpticalFlowSessionNV session;
#endif
	struct motion_image in, ref, flow, cost;
	struct motion_buffer luma_cur, luma_ref, flow_buf, cost_buf;
	//! The pair (motion_view::pair) whose flow is in the command buffer, and whose flow is in flow_buf.
	int64_t recorded_pair, result_pair;
};

struct motion_view
{
	//! What the images were made for.
	uint32_t src_width, src_height;
	uint32_t rect_x, rect_y, rect_w, rect_h;

	struct motion_image pyr[2][MOTION_LEVELS];
	struct motion_image warped[MOTION_LEVELS];
	struct motion_image field[MOTION_LEVELS];
	//! The motion field used, per cell of grid_cell pixels of half size.
	struct motion_image grid_flow, grid_blocks;
	//! The field per pixel of half size, chosen from the cells' (motion_pick.comp); cells: the cell field it was
	//! chosen from.
	struct motion_image dense;
	struct motion_image *cells;
	struct motion_image out;
	//! The newest frame, copied when it arrives: the application (or SteamVR) may draw into its image again while
	//! it is still the newest here.
	struct motion_image copy;
	struct motion_flow flow;

	//! The pyramid slot of the newest frame.
	uint32_t slot;
	struct motion_frame cur, prev;
	//! Counts new frames: names the pair (prev, cur).
	int64_t pair;

	//! The field for cur and prev is in grid (grid_cell pixels per cell).
	bool have_field;
	struct motion_image *grid;
	float grid_cell;
	//! Block matching: cur and prev are close enough in time; matched at the first refresh needing it.
	bool field_stale;
	int64_t gap_ns;

	//! The application's frame period, smoothed.
	int64_t period_ns;

	//! The last homography, for the capture.
	float homography[3][3];
	struct render_motion_source cur_turned;
};

enum capture_stage
{
	CAPTURE_IDLE,
	CAPTURE_ARMED, //!< Taken at the next in-between frame.
	CAPTURE_WRITE, //!< Copied in an earlier frame: write them out.
};

struct capture_item
{
	struct render_buffer buffer;
	uint32_t width, height;
	enum motion_format format;
	char name[32];
};

struct render_motion
{
	VkDescriptorPool pool;
	VkDescriptorSetLayout set_layout;
	VkPipelineLayout pipeline_layout;
	VkPipeline reduce, warp, search, luma, field, synth, pick;
	VkSampler sampler;

	VkQueryPool query_pool;
	bool query_written;

	//! Fills buffer bindings a pass does not read.
	struct render_buffer dummy;

	VkDescriptorSet sets[MOTION_MAX_SETS];
	uint32_t next_set;

	//! The last refresh took a new frame (the half-rate test).
	bool last_fresh;
	//! When a frame was last shown again (optical flow is measured while this is recent).
	int64_t last_repeat_ns;

	struct motion_view views[XRT_MAX_VIEWS];

	//! NVIDIA's optical flow: its queue, commands, the semaphore it waits on (the frame's work) and its fence.
	struct
	{
		bool ok;
		struct vk_bundle_queue *queue;
		VkCommandPool pool;
		VkCommandBuffer cmd;
		VkSemaphore semaphore;
		VkFence fence;
		bool recording, pending, in_flight;
	} of;

	struct
	{
		uint64_t fresh, synthesized, repeated, flow_late, flow_runs;
		double alpha_sum;
		double gpu_ms_sum, gpu_ms_max;
		uint64_t gpu_count;
		int64_t last_log_ns;
	} stats;

	struct
	{
		int64_t new_frames;
		int done;
		enum capture_stage stage;
		struct capture_item items[MOTION_CAPTURE_ITEMS];
		uint32_t item_count;
		char info[4096];
		size_t info_len;
	} capture;

	struct
	{
		int bursts_done;
		int frames;
		bool active;
		bool pending_write;
		int64_t next_ns;
		struct render_buffer l1[MOTION_RECORD_FRAMES_MAX];
		uint32_t l1_w, l1_h;
		char info[MOTION_RECORD_FRAMES_MAX][640];
		struct render_buffer full;
		uint32_t full_w, full_h;
		atomic_int writing;
		pthread_t thread;
		bool thread_started;
	} record;
};


/*
 *
 * Helpers.
 *
 */

static uint32_t
div_round_up(uint32_t a, uint32_t b)
{
	return (a + b - 1) / b;
}

static VkFormat
vk_format_of(enum motion_format f)
{
	switch (f) {
	case MOTION_RGBA8: return VK_FORMAT_R8G8B8A8_UNORM;
	case MOTION_RGBA16F: return VK_FORMAT_R16G16B16A16_SFLOAT;
	case MOTION_R32UI: return VK_FORMAT_R32_UINT;
	default: return VK_FORMAT_UNDEFINED;
	}
}

static uint32_t
bytes_per_pixel(enum motion_format f)
{
	return f == MOTION_RGBA16F ? 8 : 4;
}

static void
image_destroy(struct vk_bundle *vk, struct motion_image *img)
{
	D(ImageView, img->view);
	D(Image, img->image);
	DF(Memory, img->memory);
	img->width = img->height = 0;
}

static bool
image_create(struct vk_bundle *vk, struct motion_image *img, uint32_t width, uint32_t height, enum motion_format format)
{
	VkImageUsageFlags usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
	                          VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	VkFormat vk_format = vk_format_of(format);
	VkResult ret = vk_create_image_simple(vk, (VkExtent2D){width, height}, vk_format, usage, &img->memory,
	                                      &img->image);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "Motion smoothing: no %ux%u image (%s)", width, height, vk_result_string(ret));
		return false;
	}
	VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	ret = vk_create_view(vk, img->image, VK_IMAGE_VIEW_TYPE_2D, vk_format, range, &img->view);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "Motion smoothing: no image view (%s)", vk_result_string(ret));
		return false;
	}
	img->format = format;
	img->width = width;
	img->height = height;
	img->layout = VK_IMAGE_LAYOUT_UNDEFINED;
	return true;
}

static void
image_layout(struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_image *img, VkImageLayout layout)
{
	if (img->layout == layout) {
		return;
	}
	VkImageMemoryBarrier barrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_READ_BIT |
	                     VK_ACCESS_TRANSFER_WRITE_BIT,
	    .oldLayout = img->layout,
	    .newLayout = layout,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = img->image,
	    .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
	};
	vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
	                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL, 0,
	                         NULL, 1, &barrier);
	img->layout = layout;
}

//! Everything written before is visible to what comes after (compute and transfer).
static void
barrier_all(struct vk_bundle *vk, VkCommandBuffer cmd)
{
	VkMemoryBarrier barrier = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_READ_BIT |
	                     VK_ACCESS_TRANSFER_WRITE_BIT,
	};
	vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
	                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &barrier, 0,
	                         NULL, 0, NULL);
}

//! The inputs of one pass: sampled images at bindings 0 and 1, storage images at 2..6, storage buffers at 7 and 8.
struct pass_inputs
{
	VkImageView sampled[2];
	VkImageLayout sampled_layout[2];
	struct motion_image *storage[5];
	VkBuffer buffers[2];
};

static VkDescriptorSet
set_for_pass(struct render_motion *m, struct vk_bundle *vk, const struct pass_inputs *in)
{
	if (m->next_set >= MOTION_MAX_SETS) {
		VK_ERROR(vk, "Motion smoothing: out of descriptor sets");
		return VK_NULL_HANDLE;
	}
	VkDescriptorSet set = m->sets[m->next_set++];

	VkDescriptorImageInfo infos[7];
	VkDescriptorBufferInfo buffer_infos[2];
	VkWriteDescriptorSet writes[9];
	uint32_t count = 0, n_images = 0, n_buffers = 0;
	for (uint32_t i = 0; i < 2; i++) {
		if (in->sampled[i] == VK_NULL_HANDLE) {
			continue;
		}
		infos[n_images] = (VkDescriptorImageInfo){m->sampler, in->sampled[i], in->sampled_layout[i]};
		writes[count++] = (VkWriteDescriptorSet){
		    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
		    .dstSet = set,
		    .dstBinding = i,
		    .descriptorCount = 1,
		    .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
		    .pImageInfo = &infos[n_images++],
		};
	}
	for (uint32_t i = 0; i < 5; i++) {
		if (in->storage[i] == NULL) {
			continue;
		}
		infos[n_images] = (VkDescriptorImageInfo){VK_NULL_HANDLE, in->storage[i]->view, VK_IMAGE_LAYOUT_GENERAL};
		writes[count++] = (VkWriteDescriptorSet){
		    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
		    .dstSet = set,
		    .dstBinding = 2 + i,
		    .descriptorCount = 1,
		    .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
		    .pImageInfo = &infos[n_images++],
		};
	}
	for (uint32_t i = 0; i < 2; i++) {
		if (in->buffers[i] == VK_NULL_HANDLE) {
			continue;
		}
		buffer_infos[n_buffers] = (VkDescriptorBufferInfo){in->buffers[i], 0, VK_WHOLE_SIZE};
		writes[count++] = (VkWriteDescriptorSet){
		    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
		    .dstSet = set,
		    .dstBinding = 7 + i,
		    .descriptorCount = 1,
		    .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
		    .pBufferInfo = &buffer_infos[n_buffers++],
		};
	}
	vk->vkUpdateDescriptorSets(vk->device, count, writes, 0, NULL);
	return set;
}

static void
run_groups(struct render_motion *m,
           struct vk_bundle *vk,
           VkCommandBuffer cmd,
           VkPipeline pipeline,
           VkDescriptorSet set,
           const struct motion_push *push,
           uint32_t groups_x,
           uint32_t groups_y)
{
	if (set == VK_NULL_HANDLE) {
		return;
	}
	vk->vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
	vk->vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, m->pipeline_layout, 0, 1, &set, 0, NULL);
	vk->vkCmdPushConstants(cmd, m->pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(*push), push);
	vk->vkCmdDispatch(cmd, groups_x, groups_y, 1);
	barrier_all(vk, cmd);
}

//! One invocation per pixel, in groups of 8x8.
static void
run_pass(struct render_motion *m,
         struct vk_bundle *vk,
         VkCommandBuffer cmd,
         VkPipeline pipeline,
         const struct pass_inputs *in,
         const struct motion_push *push,
         uint32_t width,
         uint32_t height)
{
	VkDescriptorSet set = set_for_pass(m, vk, in);
	run_groups(m, vk, cmd, pipeline, set, push, div_round_up(width, 8), div_round_up(height, 8));
}


/*
 *
 * NVIDIA's optical flow: images, buffers, sessions.
 *
 */

#ifdef VK_NV_optical_flow

static bool
buffer_create(struct render_motion *m, struct vk_bundle *vk, struct motion_buffer *b, VkDeviceSize size, VkBufferUsageFlags usage)
{
	const uint32_t families[2] = {vk->main_queue->family_index, m->of.queue->family_index};
	const bool shared = families[0] != families[1];
	VkBufferCreateInfo ci = {
	    .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
	    .size = size,
	    .usage = usage,
	    .sharingMode = shared ? VK_SHARING_MODE_CONCURRENT : VK_SHARING_MODE_EXCLUSIVE,
	    .queueFamilyIndexCount = shared ? 2 : 0,
	    .pQueueFamilyIndices = families,
	};
	if (vk->vkCreateBuffer(vk->device, &ci, NULL, &b->buffer) != VK_SUCCESS) {
		return false;
	}
	VkMemoryRequirements req;
	vk->vkGetBufferMemoryRequirements(vk->device, b->buffer, &req);
	uint32_t type = 0;
	if (!vk_get_memory_type(vk, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &type)) {
		return false;
	}
	VkMemoryAllocateInfo ai = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
	    .allocationSize = req.size,
	    .memoryTypeIndex = type,
	};
	if (vk->vkAllocateMemory(vk->device, &ai, NULL, &b->memory) != VK_SUCCESS ||
	    vk->vkBindBufferMemory(vk->device, b->buffer, b->memory, 0) != VK_SUCCESS) {
		return false;
	}
	b->size = size;
	return true;
}

static void
buffer_destroy(struct vk_bundle *vk, struct motion_buffer *b)
{
	D(Buffer, b->buffer);
	DF(Memory, b->memory);
	b->size = 0;
}

//! An image for the optical flow engine, shared by the compositor's queue and the engine's.
static bool
flow_image_create(struct render_motion *m,
                  struct vk_bundle *vk,
                  struct motion_image *img,
                  uint32_t width,
                  uint32_t height,
                  VkFormat format,
                  VkOpticalFlowUsageFlagsNV of_usage,
                  VkImageUsageFlags usage)
{
	const uint32_t families[2] = {vk->main_queue->family_index, m->of.queue->family_index};
	const bool shared = families[0] != families[1];
	VkOpticalFlowImageFormatInfoNV ofi = {
	    .sType = VK_STRUCTURE_TYPE_OPTICAL_FLOW_IMAGE_FORMAT_INFO_NV,
	    .usage = of_usage,
	};
	VkImageCreateInfo ci = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = &ofi,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = format,
	    .extent = {width, height, 1},
	    .mipLevels = 1,
	    .arrayLayers = 1,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_OPTIMAL,
	    .usage = usage,
	    .sharingMode = shared ? VK_SHARING_MODE_CONCURRENT : VK_SHARING_MODE_EXCLUSIVE,
	    .queueFamilyIndexCount = shared ? 2 : 0,
	    .pQueueFamilyIndices = families,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};
	if (vk->vkCreateImage(vk->device, &ci, NULL, &img->image) != VK_SUCCESS) {
		return false;
	}
	VkMemoryRequirements req;
	vk->vkGetImageMemoryRequirements(vk->device, img->image, &req);
	if (vk_alloc_and_bind_image_memory(vk, img->image, &req, NULL, "motion flow image", &img->memory) !=
	    VK_SUCCESS) {
		return false;
	}
	VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	if (vk_create_view(vk, img->image, VK_IMAGE_VIEW_TYPE_2D, format, range, &img->view) != VK_SUCCESS) {
		return false;
	}
	img->width = width;
	img->height = height;
	img->layout = VK_IMAGE_LAYOUT_UNDEFINED;
	return true;
}

static void
flow_destroy(struct vk_bundle *vk, struct motion_flow *f)
{
	if (f->session != VK_NULL_HANDLE) {
		vk->vkDestroyOpticalFlowSessionNV(vk->device, f->session, NULL);
		f->session = VK_NULL_HANDLE;
	}
	image_destroy(vk, &f->in);
	image_destroy(vk, &f->ref);
	image_destroy(vk, &f->flow);
	image_destroy(vk, &f->cost);
	buffer_destroy(vk, &f->cost_buf);
	buffer_destroy(vk, &f->luma_cur);
	buffer_destroy(vk, &f->luma_ref);
	buffer_destroy(vk, &f->flow_buf);
	f->made = false;
	f->recorded_pair = f->result_pair = -1;
}

static bool
flow_create(struct render_motion *m, struct vk_bundle *vk, struct motion_flow *f, uint32_t w, uint32_t h)
{
	f->w = w;
	f->h = h;
	f->gw = div_round_up(w, MOTION_FLOW_GRID);
	f->gh = div_round_up(h, MOTION_FLOW_GRID);
	f->row = div_round_up(w, 4) * 4;
	const VkImageUsageFlags in_usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	bool ok = flow_image_create(m, vk, &f->in, w, h, VK_FORMAT_R8_UNORM, VK_OPTICAL_FLOW_USAGE_INPUT_BIT_NV,
	                            in_usage) &&
	          flow_image_create(m, vk, &f->ref, w, h, VK_FORMAT_R8_UNORM, VK_OPTICAL_FLOW_USAGE_INPUT_BIT_NV,
	                            in_usage) &&
	          flow_image_create(m, vk, &f->flow, f->gw, f->gh, VK_FORMAT_R16G16_S10_5_NV,
	                            VK_OPTICAL_FLOW_USAGE_OUTPUT_BIT_NV, VK_IMAGE_USAGE_TRANSFER_SRC_BIT) &&
	          buffer_create(m, vk, &f->luma_cur, (VkDeviceSize)f->row * h,
	                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT) &&
	          buffer_create(m, vk, &f->luma_ref, (VkDeviceSize)f->row * h,
	                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT) &&
	          flow_image_create(m, vk, &f->cost, f->gw, f->gh, VK_FORMAT_R8_UINT, VK_OPTICAL_FLOW_USAGE_COST_BIT_NV,
	                            VK_IMAGE_USAGE_TRANSFER_SRC_BIT) &&
	          buffer_create(m, vk, &f->flow_buf, (VkDeviceSize)f->gw * f->gh * 4,
	                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
	                            VK_BUFFER_USAGE_TRANSFER_SRC_BIT) &&
	          buffer_create(m, vk, &f->cost_buf, (VkDeviceSize)div_round_up(f->gw * f->gh, 4) * 4,
	                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
	                            VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
	if (!ok) {
		VK_ERROR(vk, "Motion smoothing: no optical flow images");
		return false;
	}
	VkOpticalFlowSessionCreateInfoNV sci = {
	    .sType = VK_STRUCTURE_TYPE_OPTICAL_FLOW_SESSION_CREATE_INFO_NV,
	    .width = w,
	    .height = h,
	    .imageFormat = VK_FORMAT_R8_UNORM,
	    .flowVectorFormat = VK_FORMAT_R16G16_S10_5_NV,
	    .costFormat = VK_FORMAT_R8_UINT,
	    .outputGridSize = VK_OPTICAL_FLOW_GRID_SIZE_4X4_BIT_NV,
	    .performanceLevel = VK_OPTICAL_FLOW_PERFORMANCE_LEVEL_SLOW_NV,
	    .flags = VK_OPTICAL_FLOW_SESSION_CREATE_ENABLE_COST_BIT_NV,
	};
	VkResult ret = vk->vkCreateOpticalFlowSessionNV(vk->device, &sci, NULL, &f->session);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "Motion smoothing: no optical flow session (%s)", vk_result_string(ret));
		return false;
	}
	if (vk->vkBindOpticalFlowSessionImageNV(vk->device, f->session, VK_OPTICAL_FLOW_SESSION_BINDING_POINT_INPUT_NV,
	                                        f->in.view, VK_IMAGE_LAYOUT_GENERAL) != VK_SUCCESS ||
	    vk->vkBindOpticalFlowSessionImageNV(vk->device, f->session,
	                                        VK_OPTICAL_FLOW_SESSION_BINDING_POINT_REFERENCE_NV, f->ref.view,
	                                        VK_IMAGE_LAYOUT_GENERAL) != VK_SUCCESS ||
	    vk->vkBindOpticalFlowSessionImageNV(vk->device, f->session,
	                                        VK_OPTICAL_FLOW_SESSION_BINDING_POINT_FLOW_VECTOR_NV, f->flow.view,
	                                        VK_IMAGE_LAYOUT_GENERAL) != VK_SUCCESS ||
	    vk->vkBindOpticalFlowSessionImageNV(vk->device, f->session, VK_OPTICAL_FLOW_SESSION_BINDING_POINT_COST_NV,
	                                        f->cost.view, VK_IMAGE_LAYOUT_GENERAL) != VK_SUCCESS) {
		VK_ERROR(vk, "Motion smoothing: optical flow images not bound");
		return false;
	}
	f->recorded_pair = f->result_pair = -1;
	f->made = true;
	return true;
}

#endif


/*
 *
 * Capture.
 *
 */

static struct capture_item *
capture_item_for(struct render_motion *m, struct vk_bundle *vk, VkDeviceSize size, const char *name)
{
	if (m->capture.item_count >= MOTION_CAPTURE_ITEMS) {
		return NULL;
	}
	struct capture_item *item = &m->capture.items[m->capture.item_count];
	if (item->buffer.buffer == VK_NULL_HANDLE || item->buffer.size != size) {
		render_buffer_fini(vk, &item->buffer);
		VkResult ret = render_buffer_init(vk, &item->buffer, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		                                  size);
		if (ret != VK_SUCCESS) {
			VK_ERROR(vk, "Motion capture: no buffer (%s)", vk_result_string(ret));
			return NULL;
		}
	}
	m->capture.item_count++;
	snprintf(item->name, sizeof(item->name), "%s", name);
	return item;
}

static void
capture_copy(struct render_motion *m, struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_image *img, const char *name)
{
	struct capture_item *item =
	    capture_item_for(m, vk, (VkDeviceSize)img->width * img->height * bytes_per_pixel(img->format), name);
	if (item == NULL) {
		return;
	}
	item->width = img->width;
	item->height = img->height;
	item->format = img->format;

	VkImageLayout layout = img->layout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL ? VK_IMAGE_LAYOUT_GENERAL
	                                                                               : img->layout;
	image_layout(vk, cmd, img, layout);
	const VkBufferImageCopy region = {
	    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
	    .imageExtent = {img->width, img->height, 1},
	};
	vk->vkCmdCopyImageToBuffer(cmd, img->image, img->layout, item->buffer.buffer, 1, &region);
	barrier_all(vk, cmd);
}

static void
capture_copy_buffer(struct render_motion *m,
                    struct vk_bundle *vk,
                    VkCommandBuffer cmd,
                    struct motion_buffer *buf,
                    uint32_t width,
                    uint32_t height,
                    enum motion_format format,
                    const char *name)
{
	struct capture_item *item = capture_item_for(m, vk, buf->size, name);
	if (item == NULL) {
		return;
	}
	item->width = width;
	item->height = height;
	item->format = format;
	const VkBufferCopy region = {0, 0, buf->size};
	vk->vkCmdCopyBuffer(cmd, buf->buffer, item->buffer.buffer, 1, &region);
	barrier_all(vk, cmd);
}

static void
capture_info(struct render_motion *m, const char *fmt, ...) XRT_PRINTF_FORMAT(2, 3);

static void
capture_info(struct render_motion *m, const char *fmt, ...)
{
	if (m->capture.info_len >= sizeof(m->capture.info) - 1) {
		return;
	}
	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(m->capture.info + m->capture.info_len, sizeof(m->capture.info) - m->capture.info_len, fmt,
	                  args);
	va_end(args);
	if (n > 0) {
		m->capture.info_len = MIN(m->capture.info_len + (size_t)n, sizeof(m->capture.info) - 1);
	}
}

static void
cache_dir(char *dir, size_t size, const char *sub)
{
	const char *cache = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	if (cache != NULL && cache[0] != '\0') {
		snprintf(dir, size, "%s/riftlinux", cache);
	} else {
		snprintf(dir, size, "%s/.cache/riftlinux", home != NULL ? home : "/tmp");
	}
	mkdir(dir, 0755);
	strncat(dir, "/", size - strlen(dir) - 1);
	strncat(dir, sub, size - strlen(dir) - 1);
	mkdir(dir, 0755);
}

static void
write_rlmc(const char *path, const void *data, uint32_t w, uint32_t h, enum motion_format format, size_t bytes)
{
	FILE *f = fopen(path, "wb");
	if (f == NULL) {
		return;
	}
	const uint32_t header[4] = {0x434d4c52u /* "RLMC" */, w, h, format};
	fwrite(header, sizeof(header), 1, f);
	fwrite(data, bytes, 1, f);
	fclose(f);
}

static void
capture_write(struct render_motion *m, struct vk_bundle *vk)
{
	char dir[512], path[700];
	cache_dir(dir, sizeof(dir), "motion-capture");
	for (uint32_t i = 0; i < m->capture.item_count; i++) {
		struct capture_item *item = &m->capture.items[i];
		snprintf(path, sizeof(path), "%s/%02d-%s.rlmc", dir, m->capture.done, item->name);
		if (render_buffer_map(vk, &item->buffer) != VK_SUCCESS) {
			continue;
		}
		write_rlmc(path, item->buffer.mapped, item->width, item->height, item->format, item->buffer.size);
		render_buffer_unmap(vk, &item->buffer);
	}
	snprintf(path, sizeof(path), "%s/%02d-info.txt", dir, m->capture.done);
	FILE *f = fopen(path, "w");
	if (f != NULL) {
		fwrite(m->capture.info, m->capture.info_len, 1, f);
		fclose(f);
	}
	VK_INFO(vk, "Motion capture %d written to %s (%u images)", m->capture.done, dir, m->capture.item_count);
	m->capture.done++;
	m->capture.item_count = 0;
	m->capture.info_len = 0;
	m->capture.info[0] = '\0';
	m->capture.stage = CAPTURE_IDLE;
}


/*
 *
 * Recording.
 *
 */

//! A copy of @p img into a persistently mapped host buffer (made or remade to fit).
static bool
copy_to_host(struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_image *img, struct render_buffer *buf)
{
	const VkDeviceSize size = (VkDeviceSize)img->width * img->height * bytes_per_pixel(img->format);
	if (buf->buffer == VK_NULL_HANDLE || buf->size != size) {
		render_buffer_fini(vk, buf);
		if (render_buffer_init(vk, buf, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		                       size) != VK_SUCCESS ||
		    render_buffer_map(vk, buf) != VK_SUCCESS) {
			render_buffer_fini(vk, buf);
			return false;
		}
	}
	VkImageLayout layout =
	    img->layout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL ? VK_IMAGE_LAYOUT_GENERAL : img->layout;
	image_layout(vk, cmd, img, layout);
	const VkBufferImageCopy region = {
	    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
	    .imageExtent = {img->width, img->height, 1},
	};
	vk->vkCmdCopyImageToBuffer(cmd, img->image, img->layout, buf->buffer, 1, &region);
	barrier_all(vk, cmd);
	return true;
}

//! Writes the finished burst; the buffers are not touched by the renderer until it is done.
static void *
record_writer(void *arg)
{
	struct render_motion *m = arg;
	char dir[512], path[700];
	cache_dir(dir, sizeof(dir), "motion-record");
	const int b = m->record.bursts_done;
	for (int i = 0; i < m->record.frames; i++) {
		snprintf(path, sizeof(path), "%s/b%02d-f%02d-l1.rlmc", dir, b, i);
		write_rlmc(path, m->record.l1[i].mapped, m->record.l1_w, m->record.l1_h, MOTION_RGBA8,
		           (size_t)m->record.l1_w * m->record.l1_h * 4);
	}
	if (m->record.full.mapped != NULL) {
		snprintf(path, sizeof(path), "%s/b%02d-full.rlmc", dir, b);
		write_rlmc(path, m->record.full.mapped, m->record.full_w, m->record.full_h, MOTION_RGBA16F,
		           (size_t)m->record.full_w * m->record.full_h * 8);
	}
	snprintf(path, sizeof(path), "%s/b%02d-info.txt", dir, b);
	FILE *f = fopen(path, "w");
	if (f != NULL) {
		for (int i = 0; i < m->record.frames; i++) {
			fprintf(f, "frame %d\n%s", i, m->record.info[i]);
		}
		fclose(f);
	}
	U_LOG_I("Motion record: burst %d written to %s (%d frames)", b, dir, m->record.frames);
	m->record.bursts_done++;
	m->record.frames = 0;
	atomic_store(&m->record.writing, 0);
	return NULL;
}

//! Does the recorder want this new frame (a burst runs or is due)?
static bool
record_wants_frame(struct render_motion *m)
{
	const int64_t bursts = debug_get_num_option_motion_record();
	if (bursts <= 0) {
		return false;
	}
	if (m->record.active) {
		return true;
	}
	return !m->record.pending_write && !atomic_load(&m->record.writing) && m->record.bursts_done < bursts &&
	       os_monotonic_get_ns() >= m->record.next_ns;
}

//! A new frame in view 0: into the burst if one runs or is due.
static void
record_new_frame(struct render_motion *m, struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_view *v, int64_t now_ns)
{
	const int64_t bursts = debug_get_num_option_motion_record();
	const int frames = (int)CLAMP(debug_get_num_option_motion_record_frames(), 2, MOTION_RECORD_FRAMES_MAX);
	const int64_t mono = os_monotonic_get_ns();
	if (!m->record.active) {
		if (m->record.pending_write || atomic_load(&m->record.writing) || m->record.bursts_done >= bursts ||
		    mono < m->record.next_ns) {
			return;
		}
		m->record.active = true;
		m->record.frames = 0;
	}
	struct motion_image *l1 = &v->pyr[v->slot][0];
	if (m->record.frames > 0 && (l1->width != m->record.l1_w || l1->height != m->record.l1_h)) {
		// The application changed (another size): this burst starts again with it.
		m->record.frames = 0;
	}
	const int i = m->record.frames;
	if (!copy_to_host(vk, cmd, l1, &m->record.l1[i])) {
		m->record.active = false;
		return;
	}
	m->record.l1_w = l1->width;
	m->record.l1_h = l1->height;
	const struct render_motion_source *src = &v->cur.src;
	snprintf(m->record.info[i], sizeof(m->record.info[i]),
	         "first_ns %lld\nperiod_ns %lld\npose %.9g %.9g %.9g %.9g %.9g %.9g %.9g\nfov %.9g %.9g %.9g %.9g\n"
	         "rect %u %u %u %u\nsource %u %u\nflip_y %d\n",
	         (long long)now_ns, (long long)v->period_ns, src->pose.orientation.x, src->pose.orientation.y,
	         src->pose.orientation.z, src->pose.orientation.w, src->pose.position.x, src->pose.position.y,
	         src->pose.position.z, src->fov.angle_left, src->fov.angle_right, src->fov.angle_up, src->fov.angle_down,
	         v->rect_x, v->rect_y, v->rect_w, v->rect_h, v->src_width, v->src_height, src->flip_y ? 1 : 0);
	m->record.frames++;
	if (m->record.frames >= frames) {
		// The last frame at full size too, for looking at.
		if (copy_to_host(vk, cmd, &v->copy, &m->record.full)) {
			m->record.full_w = v->copy.width;
			m->record.full_h = v->copy.height;
		}
		m->record.active = false;
		m->record.pending_write = true;
		m->record.next_ns = mono + debug_get_num_option_motion_record_every_s() * U_TIME_1S_IN_NS;
	}
}

static void
record_start_writer(struct render_motion *m)
{
	if (!m->record.pending_write || atomic_load(&m->record.writing)) {
		return;
	}
	if (m->record.thread_started) {
		pthread_join(m->record.thread, NULL);
		m->record.thread_started = false;
	}
	m->record.pending_write = false;
	atomic_store(&m->record.writing, 1);
	if (pthread_create(&m->record.thread, NULL, record_writer, m) == 0) {
		m->record.thread_started = true;
	} else {
		atomic_store(&m->record.writing, 0);
	}
}


/*
 *
 * Per view.
 *
 */

//! Waits for the optical flow engine's last work (before its images change).
static void
flow_wait_idle(struct render_motion *m, struct vk_bundle *vk)
{
	if (m->of.in_flight) {
		vk->vkWaitForFences(vk->device, 1, &m->of.fence, VK_TRUE, UINT64_MAX);
		m->of.in_flight = false;
	}
}

static void
view_destroy(struct render_motion *m, struct vk_bundle *vk, struct motion_view *v)
{
	flow_wait_idle(m, vk);
	for (uint32_t s = 0; s < 2; s++) {
		for (uint32_t l = 0; l < MOTION_LEVELS; l++) {
			image_destroy(vk, &v->pyr[s][l]);
		}
	}
	for (uint32_t l = 0; l < MOTION_LEVELS; l++) {
		image_destroy(vk, &v->warped[l]);
		image_destroy(vk, &v->field[l]);
	}
	image_destroy(vk, &v->grid_flow);
	image_destroy(vk, &v->grid_blocks);
	image_destroy(vk, &v->dense);
	image_destroy(vk, &v->out);
	image_destroy(vk, &v->copy);
#ifdef VK_NV_optical_flow
	flow_destroy(vk, &v->flow);
#endif
	v->src_width = v->src_height = 0;
	v->cur.valid = v->prev.valid = false;
	v->have_field = false;
	v->field_stale = false;
}

static void
rect_in_pixels(const struct render_motion_source *src, uint32_t *x, uint32_t *y, uint32_t *w, uint32_t *h)
{
	const float fw = (float)src->width, fh = (float)src->height;
	*x = (uint32_t)MAX(lroundf(src->rect.x * fw), 0);
	*y = (uint32_t)MAX(lroundf(src->rect.y * fh), 0);
	*w = (uint32_t)MAX(lroundf(src->rect.w * fw), 8);
	*h = (uint32_t)MAX(lroundf(src->rect.h * fh), 8);
}

//! The view's images, made again (and its history dropped) when the frame's size or rect changes.
static bool
view_ensure(struct render_motion *m, struct vk_bundle *vk, struct motion_view *v, const struct render_motion_source *src)
{
	uint32_t rx, ry, rw, rh;
	rect_in_pixels(src, &rx, &ry, &rw, &rh);
	if (v->out.image != VK_NULL_HANDLE && v->src_width == src->width && v->src_height == src->height &&
	    v->rect_x == rx && v->rect_y == ry && v->rect_w == rw && v->rect_h == rh) {
		return true;
	}
	view_destroy(m, vk, v);

	const uint32_t w1 = div_round_up(rw, 2), h1 = div_round_up(rh, 2);
	uint32_t w = w1, h = h1;
	bool ok = true;
	for (uint32_t l = 0; l < MOTION_LEVELS; l++) {
		for (uint32_t s = 0; s < 2; s++) {
			ok = ok && image_create(vk, &v->pyr[s][l], w, h, MOTION_RGBA8);
		}
		ok = ok && image_create(vk, &v->warped[l], w, h, MOTION_RGBA8);
		ok = ok && image_create(vk, &v->field[l], MAX(w / MOTION_BLOCK, 1), MAX(h / MOTION_BLOCK, 1),
		                        MOTION_RGBA16F);
		w = div_round_up(w, 2);
		h = div_round_up(h, 2);
	}
	ok = ok && image_create(vk, &v->grid_blocks, v->field[0].width, v->field[0].height, MOTION_RGBA16F);
	ok = ok && image_create(vk, &v->dense, w1, h1, MOTION_RGBA16F);
	ok = ok && image_create(vk, &v->out, src->width, src->height, MOTION_RGBA16F);
	ok = ok && image_create(vk, &v->copy, src->width, src->height, MOTION_RGBA16F);
#ifdef VK_NV_optical_flow
	if (ok && m->of.ok) {
		if (flow_create(m, vk, &v->flow, w1, h1)) {
			ok = image_create(vk, &v->grid_flow, v->flow.gw, v->flow.gh, MOTION_RGBA16F);
		} else {
			flow_destroy(vk, &v->flow);
		}
	}
#endif
	if (!ok) {
		view_destroy(m, vk, v);
		return false;
	}
	v->src_width = src->width;
	v->src_height = src->height;
	v->rect_x = rx;
	v->rect_y = ry;
	v->rect_w = rw;
	v->rect_h = rh;
	v->pair = 0;
	VK_INFO(vk, "Motion smoothing: images for a %ux%u frame (rect %u,%u %ux%u), motion from %s", src->width,
	        src->height, rx, ry, rw, rh, v->flow.made ? "NVIDIA's optical flow" : "block matching");
	return true;
}

//! Is this the frame already held as the newest (the application had nothing new for this refresh)?
static bool
same_frame(const struct motion_frame *f, const struct render_motion_source *src)
{
	return f->valid && f->src.image == src->image && memcmp(&f->src.pose, &src->pose, sizeof(src->pose)) == 0;
}

/*!
 * The homography from this frame's view (u, v, 1) to the previous frame's (u, v, 1) * w, for rotation only. Tangent
 * space of a view: x right, y up, looking down -z; v = 0 at the top of the rect (the bottom if flipped).
 */
static void
homography(const struct render_motion_source *cur, const struct render_motion_source *prev, bool flip_y, float out[3][3])
{
	const double cl = tan(cur->fov.angle_left), cr = tan(cur->fov.angle_right);
	double cu = tan(cur->fov.angle_up), cd = tan(cur->fov.angle_down);
	const double pl = tan(prev->fov.angle_left), pr = tan(prev->fov.angle_right);
	double pu = tan(prev->fov.angle_up), pd = tan(prev->fov.angle_down);
	if (flip_y) {
		double t = cu;
		cu = cd;
		cd = t;
		t = pu;
		pu = pd;
		pd = t;
	}

	// B: (u, v, 1) -> direction in this frame's view.
	const double B[3][3] = {{cr - cl, 0, cl}, {0, cd - cu, cu}, {0, 0, -1}};

	// M: this frame's view -> the previous frame's view, columns = rotated unit vectors.
	struct xrt_quat inv_prev;
	math_quat_invert(&prev->pose.orientation, &inv_prev);
	double M[3][3];
	for (int c = 0; c < 3; c++) {
		struct xrt_vec3 e = {c == 0 ? 1.f : 0.f, c == 1 ? 1.f : 0.f, c == 2 ? 1.f : 0.f};
		struct xrt_vec3 world, view;
		math_quat_rotate_vec3(&cur->pose.orientation, &e, &world);
		math_quat_rotate_vec3(&inv_prev, &world, &view);
		M[0][c] = view.x;
		M[1][c] = view.y;
		M[2][c] = view.z;
	}

	// A: direction in the previous view -> (u, v, 1) * w, w = -z.
	const double A[3][3] = {{1 / (pr - pl), 0, pl / (pr - pl)}, {0, 1 / (pd - pu), pu / (pd - pu)}, {0, 0, -1}};

	double MB[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			MB[i][j] = M[i][0] * B[0][j] + M[i][1] * B[1][j] + M[i][2] * B[2][j];
		}
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out[i][j] = (float)(A[i][0] * MB[0][j] + A[i][1] * MB[1][j] + A[i][2] * MB[2][j]);
		}
	}
}

static void
reduce(struct render_motion *m,
       struct vk_bundle *vk,
       VkCommandBuffer cmd,
       VkImageView src_view,
       VkImageLayout src_layout,
       struct xrt_normalized_rect rect,
       bool from_frame,
       struct motion_image *dst)
{
	struct pass_inputs in = {.sampled = {src_view}, .sampled_layout = {src_layout}, .storage = {dst}};
	struct motion_push push = {
	    .v0 = {rect.x, rect.y, rect.w, rect.h},
	    .i0 = {from_frame ? 1 : 0, 0, 0, 0},
	};
	run_pass(m, vk, cmd, m->reduce, &in, &push, dst->width, dst->height);
}

//! The previous frame's finest level turned to the newest frame's orientation (and the test's turn).
static void
warp_previous(struct render_motion *m, struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_view *v)
{
	v->cur_turned = v->cur.src;
	const int64_t test_mdeg = debug_get_num_option_motion_test_yaw_mdeg();
	if (test_mdeg != 0) {
		const struct xrt_vec3 up = {0, 1, 0};
		struct xrt_quat turn;
		math_quat_from_angle_vector((float)(test_mdeg * 1e-3 * M_PI / 180.0), &up, &turn);
		math_quat_rotate(&v->cur.src.pose.orientation, &turn, &v->cur_turned.pose.orientation);
	}
	homography(&v->cur_turned, &v->prev.src, v->cur.src.flip_y, v->homography);
	image_layout(vk, cmd, &v->warped[0], VK_IMAGE_LAYOUT_GENERAL);
	struct pass_inputs in = {
	    .sampled = {v->pyr[1 - v->slot][0].view},
	    .sampled_layout = {VK_IMAGE_LAYOUT_GENERAL},
	    .storage = {&v->warped[0]},
	};
	struct motion_push push = {
	    .v0 = {v->homography[0][0], v->homography[0][1], v->homography[0][2], 0},
	    .v1 = {v->homography[1][0], v->homography[1][1], v->homography[1][2], 0},
	    .v2 = {v->homography[2][0], v->homography[2][1], v->homography[2][2], 0},
	};
	run_pass(m, vk, cmd, m->warp, &in, &push, v->warped[0].width, v->warped[0].height);
}

static void
search(struct render_motion *m, struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_view *v, uint32_t level)
{
	const bool top = level == MOTION_LEVELS - 1;
	// The top level reads no coarser field; any image of that format fills the binding.
	struct motion_image *coarser = top ? &v->field[0] : &v->field[level + 1];
	struct pass_inputs in = {.storage = {&v->pyr[v->slot][level], &v->warped[level], coarser, &v->field[level]}};
	struct motion_push push = {
	    .v0 = {MOTION_LAMBDA, 0, 0, 0},
	    .i0 = {top ? MOTION_RADIUS_TOP : MOTION_RADIUS_REFINE, top ? 0 : 1,
	           level == 0 && debug_get_bool_option_motion_subpixel() ? 1 : 0, 0},
	};
	VkDescriptorSet set = set_for_pass(m, vk, &in);
	// One workgroup per block.
	run_groups(m, vk, cmd, m->search, set, &push, v->field[level].width, v->field[level].height);
}

/*!
 * The motion field used for the in-between frames: from the optical flow buffer, or from the block matcher's finest
 * field; short vectors dropped.
 */
static void
make_field(struct render_motion *m, struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_view *v, bool from_flow)
{
	struct motion_image *grid = from_flow ? &v->grid_flow : &v->grid_blocks;
	image_layout(vk, cmd, grid, VK_IMAGE_LAYOUT_GENERAL);
	image_layout(vk, cmd, &v->field[0], VK_IMAGE_LAYOUT_GENERAL);
	// The flow binding must hold a buffer either way (block matching: a small one, not read).
	struct pass_inputs in = {
	    .storage = {NULL, NULL, &v->field[0], grid},
	    .buffers = {from_flow ? v->flow.flow_buf.buffer : m->dummy.buffer,
	                from_flow ? v->flow.cost_buf.buffer : m->dummy.buffer},
	};
	struct motion_push push = {
	    .v0 = {MOTION_MIN_VECTOR, 0, 0, 0},
	    .i0 = {from_flow ? 1 : 0, from_flow ? 1 : 0, 0, 0},
	};
	run_pass(m, vk, cmd, m->field, &in, &push, grid->width, grid->height);
	v->cells = grid;
	v->grid = grid;
	v->grid_cell = from_flow ? (float)MOTION_FLOW_GRID : (float)MOTION_BLOCK;
	v->have_field = true;

	// Per pixel: each takes the best matching of the cells' vectors around it (motion_pick.comp).
	if (debug_get_bool_option_motion_pick()) {
		struct motion_image *cur = &v->pyr[v->slot][0];
		image_layout(vk, cmd, cur, VK_IMAGE_LAYOUT_GENERAL);
		image_layout(vk, cmd, &v->warped[0], VK_IMAGE_LAYOUT_GENERAL);
		image_layout(vk, cmd, &v->dense, VK_IMAGE_LAYOUT_GENERAL);
		struct pass_inputs pin = {
		    .sampled = {cur->view, v->warped[0].view},
		    .sampled_layout = {VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL},
		    .storage = {NULL, NULL, grid, &v->dense},
		};
		struct motion_push ppush = {
		    .v0 = {MOTION_PICK_LAMBDA, 0, 0, 0},
		    .i0 = {(int32_t)v->grid_cell, 0, 0, 0},
		};
		run_pass(m, vk, cmd, m->pick, &pin, &ppush, v->dense.width, v->dense.height);
		v->grid = &v->dense;
		v->grid_cell = 1.0f;
	}
}

/*!
 * The newest frame moved on by @p alpha of its motion into @p target; with alpha 0 a plain copy (texel centres).
 * Source: a view of an image laid out as the application's (size, rect).
 */
static void
synthesize(struct render_motion *m,
           struct vk_bundle *vk,
           VkCommandBuffer cmd,
           struct motion_view *v,
           VkImageView src_view,
           VkImageLayout src_layout,
           struct xrt_normalized_rect rect,
           float alpha,
           struct motion_image *target)
{
	// Written whole: what it held does not matter.
	target->layout = VK_IMAGE_LAYOUT_UNDEFINED;
	image_layout(vk, cmd, target, VK_IMAGE_LAYOUT_GENERAL);
	// The field binding always needs an image; with alpha 0 it is not read.
	struct motion_image *grid = v->grid != NULL ? v->grid : &v->grid_blocks;
	image_layout(vk, cmd, grid, VK_IMAGE_LAYOUT_GENERAL);

	struct pass_inputs in = {
	    .sampled = {src_view, grid->view},
	    .sampled_layout = {src_layout, VK_IMAGE_LAYOUT_GENERAL},
	    .storage = {NULL, NULL, NULL, target},
	};
	struct motion_push push = {
	    .v0 = {rect.x, rect.y, rect.w, rect.h},
	    .v1 = {alpha, (float)v->pyr[0][0].width / (float)v->rect_w, (float)v->pyr[0][0].height / (float)v->rect_h,
	           v->grid_cell > 0 ? v->grid_cell : (float)MOTION_BLOCK},
	    .i0 = {MOTION_STEPS, 0, 0, 0},
	};
	run_pass(m, vk, cmd, m->synth, &in, &push, target->width, target->height);
}

#ifdef VK_NV_optical_flow
/*!
 * Optical flow between the view's newest frame and the previous one (turned): brightness into the engine's images on
 * the compositor's queue, the engine's work into its own command buffer (submitted after the frame's work).
 */
static void
flow_record(struct render_motion *m, struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_view *v)
{
	struct motion_flow *f = &v->flow;

	// Brightness of both.
	struct pass_inputs in = {
	    .storage = {&v->pyr[v->slot][0], &v->warped[0]},
	    .buffers = {f->luma_cur.buffer, f->luma_ref.buffer},
	};
	struct motion_push push = {.i0 = {(int32_t)(f->row / 4), 0, 0, 0}};
	run_pass(m, vk, cmd, m->luma, &in, &push, f->row / 4, f->h);

	// Into the engine's images.
	image_layout(vk, cmd, &f->in, VK_IMAGE_LAYOUT_GENERAL);
	image_layout(vk, cmd, &f->ref, VK_IMAGE_LAYOUT_GENERAL);
	image_layout(vk, cmd, &f->flow, VK_IMAGE_LAYOUT_GENERAL);
	image_layout(vk, cmd, &f->cost, VK_IMAGE_LAYOUT_GENERAL);
	const VkBufferImageCopy region = {
	    .bufferRowLength = f->row,
	    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
	    .imageExtent = {f->w, f->h, 1},
	};
	vk->vkCmdCopyBufferToImage(cmd, f->luma_cur.buffer, f->in.image, VK_IMAGE_LAYOUT_GENERAL, 1, &region);
	vk->vkCmdCopyBufferToImage(cmd, f->luma_ref.buffer, f->ref.image, VK_IMAGE_LAYOUT_GENERAL, 1, &region);
	barrier_all(vk, cmd);

	// The engine's commands.
	if (!m->of.recording) {
		vk->vkResetCommandPool(vk->device, m->of.pool, 0);
		VkCommandBufferBeginInfo begin = {
		    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		    .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
		};
		vk->vkBeginCommandBuffer(m->of.cmd, &begin);
		m->of.recording = true;
	}
	VkOpticalFlowExecuteInfoNV exec = {
	    .sType = VK_STRUCTURE_TYPE_OPTICAL_FLOW_EXECUTE_INFO_NV,
	    .flags = VK_OPTICAL_FLOW_EXECUTE_DISABLE_TEMPORAL_HINTS_BIT_NV,
	};
	vk->vkCmdOpticalFlowExecuteNV(m->of.cmd, f->session, &exec);
	VkMemoryBarrier mb = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT,
	};
	vk->vkCmdPipelineBarrier(m->of.cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1,
	                         &mb, 0, NULL, 0, NULL);
	const VkBufferImageCopy out = {
	    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
	    .imageExtent = {f->gw, f->gh, 1},
	};
	vk->vkCmdCopyImageToBuffer(m->of.cmd, f->flow.image, VK_IMAGE_LAYOUT_GENERAL, f->flow_buf.buffer, 1, &out);
	vk->vkCmdCopyImageToBuffer(m->of.cmd, f->cost.image, VK_IMAGE_LAYOUT_GENERAL, f->cost_buf.buffer, 1, &out);
	f->recorded_pair = v->pair;
	m->stats.flow_runs++;
}
#endif

//! A new frame in this view: kept (copy), its pyramid; the motion to the previous one started or prepared.
static void
view_new_frame(struct render_motion *m,
               struct vk_bundle *vk,
               VkCommandBuffer cmd,
               struct motion_view *v,
               const struct render_motion_source *src,
               int64_t now_ns,
               int64_t interval_ns,
               bool want_motion)
{
	v->prev = v->cur;
	v->cur = (struct motion_frame){
	    .valid = true,
	    .src = *src,
	    .first_ns = now_ns,
	};
	v->slot = 1 - v->slot;
	v->pair++;
	v->have_field = false;
	v->field_stale = false;
	v->grid = NULL;

	// Our own copy first: in-between frames are made from it, never from the application's image.
	synthesize(m, vk, cmd, v, src->view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, src->rect, 0.0f, &v->copy);
	struct motion_image *pyr = v->pyr[v->slot];
	for (uint32_t l = 0; l < MOTION_LEVELS; l++) {
		image_layout(vk, cmd, &pyr[l], VK_IMAGE_LAYOUT_GENERAL);
	}
	const struct xrt_normalized_rect whole = {0, 0, 1, 1};
	reduce(m, vk, cmd, src->view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, src->rect, true, &pyr[0]);
	// Coarser levels only for block matching.
	if (!v->flow.made) {
		for (uint32_t l = 1; l < MOTION_LEVELS; l++) {
			reduce(m, vk, cmd, pyr[l - 1].view, VK_IMAGE_LAYOUT_GENERAL, whole, false, &pyr[l]);
		}
	}

	if (!v->prev.valid) {
		return;
	}
	v->gap_ns = v->cur.first_ns - v->prev.first_ns;
	if (v->gap_ns <= 0 || v->gap_ns > 6 * interval_ns) {
		// Too long ago to say anything about motion now.
		v->period_ns = 0;
		return;
	}
	v->period_ns = v->period_ns == 0 ? v->gap_ns : v->period_ns + (v->gap_ns - v->period_ns) / 4;
	v->period_ns = CLAMP(v->period_ns, interval_ns, 4 * interval_ns);
	if (!want_motion) {
		return;
	}

#ifdef VK_NV_optical_flow
	if (v->flow.made) {
		// Started now, ready for the next refresh. Not while the engine still works on the last.
		if (!m->of.in_flight) {
			warp_previous(m, vk, cmd, v);
			flow_record(m, vk, cmd, v);
		}
		return;
	}
#endif
	v->field_stale = true;
}

//! Block matching between the view's two newest frames, then the field (at the first refresh that needs it).
static void
view_blocks(struct render_motion *m, struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_view *v, bool capture)
{
	v->field_stale = false;
	for (uint32_t l = 0; l < MOTION_LEVELS; l++) {
		image_layout(vk, cmd, &v->warped[l], VK_IMAGE_LAYOUT_GENERAL);
		image_layout(vk, cmd, &v->field[l], VK_IMAGE_LAYOUT_GENERAL);
	}
	warp_previous(m, vk, cmd, v);
	const struct xrt_normalized_rect whole = {0, 0, 1, 1};
	for (uint32_t l = 1; l < MOTION_LEVELS; l++) {
		reduce(m, vk, cmd, v->warped[l - 1].view, VK_IMAGE_LAYOUT_GENERAL, whole, false, &v->warped[l]);
	}
	for (int l = MOTION_LEVELS - 1; l >= 0; l--) {
		search(m, vk, cmd, v, (uint32_t)l);
	}
	make_field(m, vk, cmd, v, false);

	if (capture) {
		struct motion_image *pyr = v->pyr[v->slot];
		capture_copy(m, vk, cmd, &pyr[1], "cur2");
		capture_copy(m, vk, cmd, &pyr[2], "cur3");
		capture_copy(m, vk, cmd, &v->warped[1], "warped2");
		capture_copy(m, vk, cmd, &v->warped[2], "warped3");
		capture_copy(m, vk, cmd, &v->field[0], "field1");
		capture_copy(m, vk, cmd, &v->field[1], "field2");
		capture_copy(m, vk, cmd, &v->field[2], "field3");
	}
}

static void
capture_common(struct render_motion *m, struct vk_bundle *vk, VkCommandBuffer cmd, struct motion_view *v)
{
	capture_copy(m, vk, cmd, &v->pyr[1 - v->slot][0], "prev1");
	capture_copy(m, vk, cmd, &v->pyr[v->slot][0], "cur1");
	capture_copy(m, vk, cmd, &v->warped[0], "warped1");
	capture_info(m, "rect %u %u %u %u\nsource %u %u\nflip_y %d\ngap_ns %lld\nperiod_ns %lld\n", v->rect_x,
	             v->rect_y, v->rect_w, v->rect_h, v->src_width, v->src_height, v->cur.src.flip_y ? 1 : 0,
	             (long long)v->gap_ns, (long long)v->period_ns);
	capture_info(m, "lambda %g\nradius_top %d\nradius_refine %d\ntest_yaw_mdeg %lld\nmin_vector %g\nsteps %d\n",
	             MOTION_LAMBDA, MOTION_RADIUS_TOP, MOTION_RADIUS_REFINE,
	             (long long)debug_get_num_option_motion_test_yaw_mdeg(), MOTION_MIN_VECTOR, MOTION_STEPS);
	for (int i = 0; i < 3; i++) {
		capture_info(m, "H%d %.9g %.9g %.9g\n", i, v->homography[i][0], v->homography[i][1], v->homography[i][2]);
	}
	const struct render_motion_source *fr[2] = {&v->cur_turned, &v->prev.src};
	const char *names[2] = {"cur", "prev"};
	for (int i = 0; i < 2; i++) {
		capture_info(m, "%s_pose %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n%s_fov %.9g %.9g %.9g %.9g\n", names[i],
		             fr[i]->pose.orientation.x, fr[i]->pose.orientation.y, fr[i]->pose.orientation.z,
		             fr[i]->pose.orientation.w, fr[i]->pose.position.x, fr[i]->pose.position.y,
		             fr[i]->pose.position.z, names[i], fr[i]->fov.angle_left, fr[i]->fov.angle_right,
		             fr[i]->fov.angle_up, fr[i]->fov.angle_down);
	}
}


/*
 *
 * Timing and stats.
 *
 */

static void
timing_read(struct render_motion *m, struct vk_bundle *vk)
{
	if (!m->query_written) {
		return;
	}
	m->query_written = false;
	uint64_t ts[2] = {0};
	VkResult ret = vk->vkGetQueryPoolResults(vk->device, m->query_pool, 0, 2, sizeof(ts), ts, sizeof(uint64_t),
	                                         VK_QUERY_RESULT_64_BIT);
	if (ret != VK_SUCCESS || ts[1] < ts[0]) {
		return;
	}
	const double ms = (double)(ts[1] - ts[0]) * vk->features.timestamp_period / 1e6;
	m->stats.gpu_ms_sum += ms;
	m->stats.gpu_ms_max = MAX(m->stats.gpu_ms_max, ms);
	m->stats.gpu_count++;
}

static void
stats_log(struct render_motion *m, struct vk_bundle *vk)
{
	const int64_t now = os_monotonic_get_ns();
	if (m->stats.last_log_ns == 0) {
		m->stats.last_log_ns = now;
		return;
	}
	if (now - m->stats.last_log_ns < (int64_t)10 * U_TIME_1S_IN_NS) {
		return;
	}
	const double alpha = m->stats.synthesized > 0 ? m->stats.alpha_sum / (double)m->stats.synthesized : 0.0;
	const double gpu = m->stats.gpu_count > 0 ? m->stats.gpu_ms_sum / (double)m->stats.gpu_count : 0.0;
	VK_INFO(vk,
	        "Motion smoothing: %llu new frames, %llu in-between (mean fraction %.2f)%s, %llu repeated "
	        "as they were (%llu without motion yet); optical flow runs %llu; GPU %.2f ms mean, %.2f max per refresh",
	        (unsigned long long)m->stats.fresh, (unsigned long long)m->stats.synthesized, alpha,
	        debug_get_bool_option_motion_test_shadow() ? " made, not shown (shadow test)" : "",
	        (unsigned long long)m->stats.repeated, (unsigned long long)m->stats.flow_late,
	        (unsigned long long)m->stats.flow_runs, gpu, m->stats.gpu_ms_max);
	memset(&m->stats, 0, sizeof(m->stats));
	m->stats.last_log_ns = now;
}


/*
 *
 * 'Exported' functions.
 *
 */

#ifdef VK_NV_optical_flow
static void
flow_init(struct render_motion *m, struct vk_bundle *vk)
{
	if (vk->optical_flow_queue == NULL || debug_get_num_option_motion_flow() == 0) {
		VK_INFO(vk, "Motion smoothing: block matching (no NVIDIA optical flow%s)",
		        vk->optical_flow_queue == NULL ? " on this GPU" : ": XRT_COMPOSITOR_MOTION_FLOW=0");
		return;
	}
	m->of.queue = vk->optical_flow_queue;
	VkCommandPoolCreateInfo pci = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
	    .flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT,
	    .queueFamilyIndex = m->of.queue->family_index,
	};
	VkSemaphoreCreateInfo sci = {.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
	VkFenceCreateInfo fci = {.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
	if (vk->vkCreateCommandPool(vk->device, &pci, NULL, &m->of.pool) != VK_SUCCESS ||
	    vk->vkCreateSemaphore(vk->device, &sci, NULL, &m->of.semaphore) != VK_SUCCESS ||
	    vk->vkCreateFence(vk->device, &fci, NULL, &m->of.fence) != VK_SUCCESS) {
		VK_ERROR(vk, "Motion smoothing: no optical flow command pool, semaphore or fence");
		return;
	}
	VkCommandBufferAllocateInfo cai = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
	    .commandPool = m->of.pool,
	    .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
	    .commandBufferCount = 1,
	};
	if (vk->vkAllocateCommandBuffers(vk->device, &cai, &m->of.cmd) != VK_SUCCESS) {
		return;
	}
	m->of.ok = true;
	VK_INFO(vk, "Motion smoothing: NVIDIA's optical flow on queue family %u", m->of.queue->family_index);
}
#endif

bool
render_motion_init(struct render_resources *r)
{
	struct vk_bundle *vk = r->vk;
	struct render_motion *m = U_TYPED_CALLOC(struct render_motion);
	VkResult ret;

	const VkDescriptorSetLayoutBinding bindings[] = {
	    {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {3, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {4, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {5, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {6, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {7, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	    {8, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL},
	};
	const VkDescriptorSetLayoutCreateInfo set_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(bindings),
	    .pBindings = bindings,
	};
	ret = vk->vkCreateDescriptorSetLayout(vk->device, &set_info, NULL, &m->set_layout);
	if (ret != VK_SUCCESS) {
		goto err;
	}

	const VkPushConstantRange range = {VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(struct motion_push)};
	const VkPipelineLayoutCreateInfo layout_info = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
	    .setLayoutCount = 1,
	    .pSetLayouts = &m->set_layout,
	    .pushConstantRangeCount = 1,
	    .pPushConstantRanges = &range,
	};
	ret = vk->vkCreatePipelineLayout(vk->device, &layout_info, NULL, &m->pipeline_layout);
	if (ret != VK_SUCCESS) {
		goto err;
	}

	struct
	{
		VkShaderModule shader;
		VkPipeline *pipeline;
	} pipelines[] = {
	    {r->shaders->motion_reduce_comp, &m->reduce}, {r->shaders->motion_warp_comp, &m->warp},
	    {r->shaders->motion_search_comp, &m->search}, {r->shaders->motion_luma_comp, &m->luma},
	    {r->shaders->motion_field_comp, &m->field},   {r->shaders->motion_synth_comp, &m->synth},
	    {r->shaders->motion_pick_comp, &m->pick},
	};
	for (uint32_t i = 0; i < ARRAY_SIZE(pipelines); i++) {
		ret = vk_create_compute_pipeline(vk, r->pipeline_cache, pipelines[i].shader, m->pipeline_layout, NULL,
		                                 pipelines[i].pipeline);
		if (ret != VK_SUCCESS) {
			goto err;
		}
	}

	const struct vk_descriptor_pool_info pool_info = {
	    .sampler_per_descriptor_count = 2,
	    .storage_image_per_descriptor_count = 5,
	    .storage_buffer_per_descriptor_count = 2,
	    .descriptor_count = MOTION_MAX_SETS,
	    .freeable = false,
	};
	ret = vk_create_descriptor_pool(vk, &pool_info, &m->pool);
	if (ret != VK_SUCCESS) {
		goto err;
	}
	for (uint32_t i = 0; i < MOTION_MAX_SETS; i++) {
		ret = vk_create_descriptor_set(vk, m->pool, m->set_layout, &m->sets[i]);
		if (ret != VK_SUCCESS) {
			goto err;
		}
	}

	ret = vk_create_sampler(vk, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, &m->sampler);
	if (ret != VK_SUCCESS) {
		goto err;
	}

	const VkQueryPoolCreateInfo query_info = {
	    .sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO,
	    .queryType = VK_QUERY_TYPE_TIMESTAMP,
	    .queryCount = 2,
	};
	ret = vk->vkCreateQueryPool(vk->device, &query_info, NULL, &m->query_pool);
	if (ret != VK_SUCCESS) {
		goto err;
	}

	ret = render_buffer_init(vk, &m->dummy, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
	                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 256);
	if (ret != VK_SUCCESS) {
		goto err;
	}

#ifdef VK_NV_optical_flow
	flow_init(m, vk);
#endif

	r->motion = m;
	return true;

err:
	VK_ERROR(vk, "Motion smoothing unavailable: %s", vk_result_string(ret));
	r->motion = m;
	render_motion_fini(r);
	return false;
}

void
render_motion_fini(struct render_resources *r)
{
	struct render_motion *m = r->motion;
	if (m == NULL) {
		return;
	}
	struct vk_bundle *vk = r->vk;
	flow_wait_idle(m, vk);
	for (uint32_t i = 0; i < XRT_MAX_VIEWS; i++) {
		view_destroy(m, vk, &m->views[i]);
	}
	for (uint32_t i = 0; i < MOTION_CAPTURE_ITEMS; i++) {
		render_buffer_fini(vk, &m->capture.items[i].buffer);
	}
	if (m->record.thread_started) {
		pthread_join(m->record.thread, NULL);
	}
	for (uint32_t i = 0; i < MOTION_RECORD_FRAMES_MAX; i++) {
		render_buffer_fini(vk, &m->record.l1[i]);
	}
	render_buffer_fini(vk, &m->record.full);
	render_buffer_fini(vk, &m->dummy);
	D(Fence, m->of.fence);
	D(Semaphore, m->of.semaphore);
	D(CommandPool, m->of.pool);
	D(QueryPool, m->query_pool);
	D(Sampler, m->sampler);
	D(DescriptorPool, m->pool);
	D(Pipeline, m->reduce);
	D(Pipeline, m->warp);
	D(Pipeline, m->search);
	D(Pipeline, m->luma);
	D(Pipeline, m->field);
	D(Pipeline, m->synth);
	D(Pipeline, m->pick);
	D(PipelineLayout, m->pipeline_layout);
	D(DescriptorSetLayout, m->set_layout);
	free(m);
	r->motion = NULL;
}

VkSemaphore
render_motion_signal_semaphore(struct render_resources *r)
{
	struct render_motion *m = r->motion;
	return m != NULL && m->of.pending ? m->of.semaphore : VK_NULL_HANDLE;
}

void
render_motion_submitted(struct render_resources *r, bool ok)
{
	struct render_motion *m = r->motion;
	if (m == NULL || !m->of.pending) {
		return;
	}
	struct vk_bundle *vk = r->vk;
	m->of.pending = false;
	VkResult ret = VK_ERROR_UNKNOWN;
	if (ok) {
		vk->vkResetFences(vk->device, 1, &m->of.fence);
		const VkPipelineStageFlags stage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
		const VkSubmitInfo submit = {
		    .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
		    .waitSemaphoreCount = 1,
		    .pWaitSemaphores = &m->of.semaphore,
		    .pWaitDstStageMask = &stage,
		    .commandBufferCount = 1,
		    .pCommandBuffers = &m->of.cmd,
		};
		vk_queue_lock(m->of.queue);
		ret = vk->vkQueueSubmit(m->of.queue->queue, 1, &submit, m->of.fence);
		vk_queue_unlock(m->of.queue);
	}
	if (ret == VK_SUCCESS) {
		m->of.in_flight = true;
		return;
	}
	if (ok) {
		VK_ERROR(vk, "Motion smoothing: optical flow not submitted (%s)", vk_result_string(ret));
	}
	for (uint32_t i = 0; i < XRT_MAX_VIEWS; i++) {
		m->views[i].flow.recorded_pair = -1;
	}
}

void
render_motion_dispatch(struct render_compute *render,
                       uint32_t view_count,
                       const struct render_motion_source sources[XRT_MAX_VIEWS],
                       VkImageView out_views[XRT_MAX_VIEWS],
                       struct xrt_pose out_poses[XRT_MAX_VIEWS])
{
	for (uint32_t i = 0; i < XRT_MAX_VIEWS; i++) {
		out_views[i] = VK_NULL_HANDLE;
	}
	struct render_motion *m = render->r->motion;
	if (m == NULL) {
		return;
	}
	struct vk_bundle *vk = render->r->vk;
	VkCommandBuffer cmd = render->r->cmd;
	m->next_set = 0;

	// The previous frame's command buffer has finished (the renderer waits for its queue).
	timing_read(m, vk);
	if (m->capture.stage == CAPTURE_WRITE) {
		capture_write(m, vk);
	}
	// The optical flow engine's last work: done means its flow is there for the pair it was given.
	if (m->of.in_flight && vk->vkGetFenceStatus(vk->device, m->of.fence) == VK_SUCCESS) {
		m->of.in_flight = false;
		for (uint32_t i = 0; i < XRT_MAX_VIEWS; i++) {
			struct motion_flow *f = &m->views[i].flow;
			if (f->recorded_pair >= 0) {
				f->result_pair = f->recorded_pair;
				f->recorded_pair = -1;
			}
		}
	}

	struct render_output_settings settings;
	render_output_settings_get(&settings);
	view_count = MIN(view_count, XRT_MAX_VIEWS);
	// The previous frame's copies have finished: the recorder's thread may write them.
	record_start_writer(m);
	const bool recording = debug_get_num_option_motion_record() > 0;
	if ((!settings.motion_smoothing && !recording) || sources == NULL || view_count == 0) {
		for (uint32_t i = 0; i < XRT_MAX_VIEWS; i++) {
			m->views[i].cur.valid = m->views[i].prev.valid = false;
			m->views[i].have_field = false;
		}
		return;
	}

	const int64_t now_ns = render->motion_display_time_ns;
	const int64_t interval_ns = render->motion_frame_interval_ns > 0 ? render->motion_frame_interval_ns
	                                                                  : U_TIME_1S_IN_NS / 90;

	// New frame or a repeat: all views alike (an application submits its views together).
	bool fresh = false;
	for (uint32_t i = 0; i < view_count; i++) {
		fresh = fresh || !same_frame(&m->views[i].cur, &sources[i]);
	}
	// Tests: hold back every other new frame, as if the application ran at half rate; in the shadow test the
	// in-between frames are made (and captured, timed) but the real frames are shown.
	// Recording alone (smoothing off): new frames are kept and recorded, nothing is made or shown.
	const bool make = settings.motion_smoothing;
	const bool test_shadow = debug_get_bool_option_motion_test_shadow();
	const bool test_half = debug_get_bool_option_motion_test_half();
	const bool shadow = test_shadow || !make;
	bool held = false;
	if (make && fresh && (test_half || test_shadow) && m->last_fresh) {
		fresh = false;
		held = true;
	}
	m->last_fresh = fresh;
	if (!fresh) {
		m->last_repeat_ns = now_ns;
	}
	// Motion is measured while frames are being shown again (and always in the tests).
	const bool want_motion =
	    make && (test_half || test_shadow || (m->last_repeat_ns != 0 && now_ns - m->last_repeat_ns < MOTION_FLOW_KEEP_NS));

	vk->vkCmdResetQueryPool(cmd, m->query_pool, 0, 2);
	vk->vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, m->query_pool, 0);

	if (fresh) {
		m->stats.fresh++;
		m->capture.new_frames++;
		const int64_t count = debug_get_num_option_motion_capture();
		const int64_t start = debug_get_num_option_motion_capture_start();
		if (count > 0 && m->capture.done < count && m->capture.stage == CAPTURE_IDLE &&
		    m->capture.new_frames >= start && (m->capture.new_frames - start) % 90 == 0) {
			// Taken at the next in-between frame.
			m->capture.stage = CAPTURE_ARMED;
		}
	}

	for (uint32_t i = 0; i < view_count; i++) {
		struct motion_view *v = &m->views[i];
		if (fresh) {
			// Recording alone: frames are kept only while a burst wants them (no work otherwise).
			if (!make && !(i == 0 && record_wants_frame(m))) {
				continue;
			}
			if (view_ensure(m, vk, v, &sources[i])) {
				view_new_frame(m, vk, cmd, v, &sources[i], now_ns, interval_ns, want_motion);
				if (recording && i == 0) {
					record_new_frame(m, vk, cmd, v, now_ns);
				}
			}
			continue;
		}

		// Shown again (or held back in the tests): the newest frame as kept, moved on.
		const int64_t since_ns = now_ns - v->cur.first_ns;
		if (!make || !v->cur.valid || v->period_ns <= 0 || since_ns <= 0 || since_ns > 3 * v->period_ns) {
			if (i == 0) {
				m->stats.repeated++;
			}
			continue;
		}
		const bool capture = i == 0 && m->capture.stage == CAPTURE_ARMED;
		if (!v->have_field) {
			if (v->flow.made) {
				if (v->flow.result_pair != v->pair) {
					// The engine's flow is not there (yet) for this pair: shown as it was.
					if (i == 0) {
						m->stats.repeated++;
						m->stats.flow_late++;
					}
					continue;
				}
				make_field(m, vk, cmd, v, true);
				if (capture) {
					capture_copy_buffer(m, vk, cmd, &v->flow.flow_buf, v->flow.gw, v->flow.gh,
					                    MOTION_FLOW_S10_5, "flow");
					capture_copy_buffer(m, vk, cmd, &v->flow.cost_buf, v->flow.gw, v->flow.gh,
					                    MOTION_COST_U8, "cost");
				}
			} else if (v->field_stale) {
				view_blocks(m, vk, cmd, v, capture);
			} else {
				if (i == 0) {
					m->stats.repeated++;
				}
				continue;
			}
		}
		const float alpha = (float)MIN((double)since_ns / (double)v->period_ns, 1.0);

		if (capture) {
			capture_common(m, vk, cmd, v);
			// The cells' field as "grid", and the per-pixel one chosen from it.
			capture_copy(m, vk, cmd, v->cells != NULL ? v->cells : v->grid, "grid");
			if (v->grid == &v->dense) {
				capture_copy(m, vk, cmd, &v->dense, "dense");
			}
			// The frame itself (our copy), and in the tests the held-back frame.
			capture_copy(m, vk, cmd, &v->copy, "frame");
			if (held && sources[i].width == v->src_width && sources[i].height == v->src_height) {
				synthesize(m, vk, cmd, v, sources[i].view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
				           sources[i].rect, 0.0f, &v->out);
				capture_copy(m, vk, cmd, &v->out, "truth");
				// Its head pose: rendered later than the frame, turned to it before comparing.
				const struct xrt_pose *tp = &sources[i].pose;
				capture_info(m, "truth_pose %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", tp->orientation.x,
				             tp->orientation.y, tp->orientation.z, tp->orientation.w, tp->position.x,
				             tp->position.y, tp->position.z);
			}
		}
		synthesize(m, vk, cmd, v, v->copy.view, VK_IMAGE_LAYOUT_GENERAL, v->cur.src.rect, alpha, &v->out);
		if (capture) {
			capture_copy(m, vk, cmd, &v->out, "between");
			capture_info(m, "alpha %.9g\nsince_ns %lld\nheld %d\nshadow %d\ngrid_cell %g\nflow %d\ndense %d\n",
			             alpha, (long long)since_ns, held ? 1 : 0, shadow ? 1 : 0,
			             v->cells == &v->grid_flow ? (float)MOTION_FLOW_GRID : (float)MOTION_BLOCK,
			             v->flow.made ? 1 : 0, v->grid == &v->dense ? 1 : 0);
			m->capture.stage = CAPTURE_WRITE;
		}
		image_layout(vk, cmd, &v->out, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
		if (!shadow) {
			out_views[i] = v->out.view;
			out_poses[i] = v->cur.src.pose;
		}
		if (i == 0) {
			m->stats.synthesized++;
			m->stats.alpha_sum += alpha;
		}
	}

#ifdef VK_NV_optical_flow
	if (m->of.recording) {
		vk->vkEndCommandBuffer(m->of.cmd);
		m->of.recording = false;
		m->of.pending = true;
	}
#endif

	vk->vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, m->query_pool, 1);
	m->query_written = true;
	stats_log(m, vk);
}
