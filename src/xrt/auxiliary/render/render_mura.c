// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Panel calibration (RiftLinux): the headset's prepared panel calibration as images for the distortion pass.
 *
 * The file ("RLSPUDC1", built by riftlinux-setup from the headset's factory measurement, or imported) holds per eye
 * three response maps (one per colour, 54 x 60, each texel the panel's response at four drive levels, the file's
 * drive_shades: 12, 25, 35 and 90 on the CV1's usual layout, as four bytes with a scale and offset per level and colour)
 * and one per-pixel map (1080 x 1200 RGBA bytes with a scale and offset per channel). The response knots are decoded
 * here into a float image array (layer = eye * 3 + colour); the per-pixel
 * maps go up as stored (layer = eye) with their constants kept for the shader. distortion.comp (mura_correct) reads
 * both with texelFetch, so no filtering mixes eyes or colours.
 *
 * @ingroup aux_render
 */

#include "render/render_interface.h"
#include "render/render_output_settings.h"

#include "os/os_time.h"

#include "util/u_json.h"
#include "util/u_logging.h"
#include "vk/vk_mini_helpers.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>


#define CAL_MAGIC "RLSPUDC1"
#define CAL_FORMAT "riftlinux-spud-calibration"
#define CAL_FORMAT_VERSION 1     // maps packed as bytes with a scale and offset (imported calibrations)
#define CAL_FORMAT_VERSION_OWN 2 // RiftLinux's own calibration: maps as floats and half floats
#define CAL_MAX_FILE_SIZE (64u * 1024u * 1024u)
#define RESPONSE_W 54
#define RESPONSE_H 60
#define WALL_W 1080
#define WALL_H 1200

XRT_CHECK_RESULT static VkResult
create_image(struct vk_bundle *vk,
             VkFormat format,
             VkExtent2D extent,
             uint32_t layers,
             const char *name,
             VkImage *out_image,
             VkDeviceMemory *out_memory,
             VkImageView *out_view)
{
	VkImageCreateInfo info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = format,
	    .extent = {extent.width, extent.height, 1},
	    .mipLevels = 1,
	    .arrayLayers = layers,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_OPTIMAL,
	    .usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};
	VkImage image = VK_NULL_HANDLE;
	VkDeviceMemory memory = VK_NULL_HANDLE;
	VkImageView view = VK_NULL_HANDLE;

	VkResult ret = vk->vkCreateImage(vk->device, &info, NULL, &image);
	VK_CHK_AND_RET(ret, "vkCreateImage");

	VkMemoryRequirements requirements;
	vk->vkGetImageMemoryRequirements(vk->device, image, &requirements);
	ret = vk_alloc_and_bind_image_memory(vk, image, &requirements, NULL, name, &memory);
	VK_CHK_WITH_GOTO(ret, "vk_alloc_and_bind_image_memory", err_image);

	VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, layers};
	ret = vk_create_view(vk, image, VK_IMAGE_VIEW_TYPE_2D_ARRAY, format, range, &view);
	VK_CHK_WITH_GOTO(ret, "vk_create_view", err_memory);

	VK_NAME_IMAGE(vk, image, name);
	*out_image = image;
	*out_memory = memory;
	*out_view = view;
	return VK_SUCCESS;

err_memory:
	DF(Memory, memory);
err_image:
	D(Image, image);
	return ret;
}

//! Copies @p size bytes (all layers, tightly packed, layer after layer) into @p image, ready for the shaders.
XRT_CHECK_RESULT static VkResult
upload(struct render_resources *r, VkImage image, VkExtent2D extent, uint32_t layers, const void *data, VkDeviceSize size)
{
	struct vk_bundle *vk = r->vk;
	struct render_buffer staging = {0};

	VkResult ret = render_buffer_init(vk, &staging, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
	                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, size);
	VK_CHK_AND_RET(ret, "render_buffer_init");
	ret = render_buffer_map_and_write(vk, &staging, (void *)data, size);
	VK_CHK_WITH_GOTO(ret, "render_buffer_map_and_write", err_buffer);

	struct vk_cmd_pool *pool = &r->distortion_pool;
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	vk_cmd_pool_lock(pool);
	ret = vk_cmd_pool_create_and_begin_cmd_buffer_locked(vk, pool, 0, &cmd);
	VK_CHK_WITH_GOTO(ret, "vk_cmd_pool_create_and_begin_cmd_buffer_locked", err_unlock);

	VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, layers};
	vk_cmd_image_barrier_gpu_locked(vk, cmd, image, 0, VK_ACCESS_TRANSFER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED,
	                                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, range);
	VkBufferImageCopy region = {
	    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, layers},
	    .imageExtent = {extent.width, extent.height, 1},
	};
	vk->vkCmdCopyBufferToImage(cmd, staging.buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
	vk_cmd_image_barrier_gpu_locked(vk, cmd, image, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
	                                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, range);

	ret = vk_cmd_pool_end_submit_wait_and_free_cmd_buffer_locked(vk, pool, cmd);
	VK_CHK_WITH_GOTO(ret, "vk_cmd_pool_end_submit_wait_and_free_cmd_buffer_locked", err_unlock);

	vk_cmd_pool_unlock(pool);
	render_buffer_fini(vk, &staging);
	return VK_SUCCESS;

err_unlock:
	vk_cmd_pool_unlock(pool);
err_buffer:
	render_buffer_fini(vk, &staging);
	return ret;
}

struct images
{
	VkImage response_image, wall_image;
	VkDeviceMemory response_memory, wall_memory;
	VkImageView response_view, wall_view;
};

static void
destroy(struct vk_bundle *vk, struct images *im)
{
	D(ImageView, im->response_view);
	D(Image, im->response_image);
	DF(Memory, im->response_memory);
	D(ImageView, im->wall_view);
	D(Image, im->wall_image);
	DF(Memory, im->wall_memory);
}

//! Both image arrays made and filled (response: RGBA float knots, 6 layers; wall: RGBA bytes, or RGBA half floats when
//! @p wall_half, 2 layers), or nothing.
static bool
make_images(struct render_resources *r,
            VkExtent2D response_extent,
            const float *response,
            VkExtent2D wall_extent,
            const void *wall,
            bool wall_half,
            struct images *out)
{
	struct vk_bundle *vk = r->vk;
	struct images im = {0};
	const VkDeviceSize response_size = (VkDeviceSize)response_extent.width * response_extent.height * 6 * 4 * sizeof(float);
	const VkDeviceSize wall_size = (VkDeviceSize)wall_extent.width * wall_extent.height * 2 * 4 * (wall_half ? 2 : 1);

	VkResult ret = create_image(vk, VK_FORMAT_R32G32B32A32_SFLOAT, response_extent, 6, "panel calibration response",
	                            &im.response_image, &im.response_memory, &im.response_view);
	VK_CHK_WITH_GOTO(ret, "create_image (response)", err);
	ret = upload(r, im.response_image, response_extent, 6, response, response_size);
	VK_CHK_WITH_GOTO(ret, "upload (response)", err);
	ret = create_image(vk, wall_half ? VK_FORMAT_R16G16B16A16_SFLOAT : VK_FORMAT_R8G8B8A8_UNORM, wall_extent, 2,
	                   "panel calibration wall", &im.wall_image, &im.wall_memory, &im.wall_view);
	VK_CHK_WITH_GOTO(ret, "create_image (wall)", err);
	ret = upload(r, im.wall_image, wall_extent, 2, wall, wall_size);
	VK_CHK_WITH_GOTO(ret, "upload (wall)", err);
	*out = im;
	return true;

err:
	destroy(vk, &im);
	return false;
}

static void
take(struct render_resources *r, const struct images *im)
{
	__typeof__(r->compute.distortion.mura) *m = &r->compute.distortion.mura;
	struct images old = {m->response_image, m->wall_image, m->response_memory,
	                     m->wall_memory,    m->response_view, m->wall_view};
	destroy(r->vk, &old);
	m->response_image = im->response_image;
	m->response_memory = im->response_memory;
	m->response_view = im->response_view;
	m->wall_image = im->wall_image;
	m->wall_memory = im->wall_memory;
	m->wall_view = im->wall_view;
}

//! The calibration file's path: XRT_COMPOSITOR_MURA_FILE, else $XDG_CONFIG_HOME/riftlinux/spud/HMD_<serial>.rlspudc.
static bool
calibration_path(const char *serial, char *path, size_t size)
{
	const char *file = getenv("XRT_COMPOSITOR_MURA_FILE");
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	if (file != NULL && file[0] != '\0') {
		snprintf(path, size, "%s", file);
	} else if (xdg != NULL && xdg[0] != '\0') {
		snprintf(path, size, "%s/riftlinux/spud/HMD_%s.rlspudc", xdg, serial);
	} else if (home != NULL && home[0] != '\0') {
		snprintf(path, size, "%s/.config/riftlinux/spud/HMD_%s.rlspudc", home, serial);
	} else {
		return false;
	}
	return true;
}

//! Whether @p path exists, with its modification time and size.
static void
file_state(const char *path, bool *out_seen, int64_t *out_mtime_ns, int64_t *out_size)
{
	struct stat st;
	*out_seen = stat(path, &st) == 0;
	*out_mtime_ns = *out_seen ? (int64_t)st.st_mtim.tv_sec * 1000000000 + st.st_mtim.tv_nsec : 0;
	*out_size = *out_seen ? (int64_t)st.st_size : 0;
}

//! @p count finite numbers of a JSON array into @p out; false otherwise.
static bool
numbers(const cJSON *array, float *out, int count)
{
	if (!cJSON_IsArray(array) || cJSON_GetArraySize(array) != count) {
		return false;
	}
	for (int i = 0; i < count; i++) {
		const cJSON *v = cJSON_GetArrayItem(array, i);
		if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble)) {
			return false;
		}
		out[i] = (float)v->valuedouble;
	}
	return true;
}

static int
number_of(const cJSON *object, const char *key)
{
	const cJSON *v = cJSON_GetObjectItemCaseSensitive(object, key);
	return cJSON_IsNumber(v) ? v->valueint : -1;
}

//! The blob @p name's place in the body; false when missing or out of the body.
static bool
blob(const cJSON *blobs, const char *name, size_t body_size, size_t length, size_t *out_offset)
{
	const cJSON *b = NULL;
	cJSON_ArrayForEach(b, blobs)
	{
		const cJSON *n = cJSON_GetObjectItemCaseSensitive(b, "name");
		if (cJSON_IsString(n) && strcmp(n->valuestring, name) == 0) {
			const cJSON *o = cJSON_GetObjectItemCaseSensitive(b, "offset");
			const cJSON *l = cJSON_GetObjectItemCaseSensitive(b, "length");
			if (!cJSON_IsNumber(o) || !cJSON_IsNumber(l) || o->valuedouble < 0 || l->valuedouble != (double)length ||
			    o->valuedouble + l->valuedouble > (double)body_size) {
				return false;
			}
			*out_offset = (size_t)o->valuedouble;
			return true;
		}
	}
	return false;
}



//! Calibration maps read from a file, ready for the GPU (render_mura.c's own; freed by prepared_free).
struct prepared
{
	float *response;
	uint8_t *wall;
	float wall_scale[2][4];
	float wall_offset[2][4];
	float shades[4]; //!< The response knots' drive levels, 0..1.
	bool wall_half;  //!< wall holds RGBA half floats (format 2), else RGBA bytes with scale and offset (format 1).
};

static void
prepared_free(struct prepared *p)
{
	free(p->response);
	free(p->wall);
	p->response = NULL;
	p->wall = NULL;
}

/*!
 * Reads and checks the calibration file @p path of headset @p serial and decodes it (CPU only, any thread): NULL and
 * @p out filled, else why it is not used (@p out empty); @p out_missing when there is no file.
 */
static const char *
read_calibration(const char *serial, const char *path, struct prepared *out, bool *out_missing)
{
	memset(out, 0, sizeof(*out));
	*out_missing = false;
	FILE *f = fopen(path, "rb");
	if (f == NULL) {
		*out_missing = true;
		return "none";
	}
	uint8_t *data = NULL;
	long size = -1;
	if (fseek(f, 0, SEEK_END) == 0) {
		size = ftell(f);
	}
	if (size > 12 && (unsigned long)size <= CAL_MAX_FILE_SIZE && fseek(f, 0, SEEK_SET) == 0) {
		data = malloc((size_t)size + 1);
		if (data != NULL && fread(data, 1, (size_t)size, f) != (size_t)size) {
			free(data);
			data = NULL;
		}
	}
	fclose(f);
	if (data == NULL) {
		return "it cannot be read";
	}

	const char *wrong = NULL;
	cJSON *manifest = NULL;
	float *response = NULL;
	size_t wall_at[2] = {0, 0};
	uint8_t *wall = NULL;
	uint32_t json_len = 0;
	if (memcmp(data, CAL_MAGIC, 8) != 0) {
		wrong = "not a RiftLinux panel calibration file";
	} else {
		memcpy(&json_len, data + 8, sizeof(json_len));
		if ((size_t)json_len > (size_t)size - 12) {
			wrong = "its manifest runs past the end";
		}
	}
	if (wrong == NULL) {
		char saved = (char)data[12 + json_len];
		data[12 + json_len] = '\0';
		manifest = cJSON_Parse((const char *)data + 12);
		data[12 + json_len] = (uint8_t)saved;
		if (manifest == NULL) {
			wrong = "its manifest is not JSON";
		}
	}
	const uint8_t *body = data + 12 + json_len;
	const size_t body_size = (size_t)size - 12 - json_len;
	int version = 0;
	if (wrong == NULL) {
		const cJSON *fmt = cJSON_GetObjectItemCaseSensitive(manifest, "format");
		const cJSON *ser = cJSON_GetObjectItemCaseSensitive(manifest, "headset_serial");
		const cJSON *shades = cJSON_GetObjectItemCaseSensitive(manifest, "drive_shades");
		float s[5] = {0};
		version = number_of(manifest, "format_version");
		if (!cJSON_IsString(fmt) || strcmp(fmt->valuestring, CAL_FORMAT) != 0 ||
		    (version != CAL_FORMAT_VERSION && version != CAL_FORMAT_VERSION_OWN)) {
			wrong = "another format or version";
		} else if (!cJSON_IsString(ser) || strcmp(ser->valuestring, serial) != 0) {
			wrong = "it is for another headset";
		} else if (!numbers(shades, s, 5) || !(s[0] > 0 && s[0] < s[1] && s[1] < s[2] && s[2] < s[3] && s[3] < 255) ||
		           s[4] != 255) {
			wrong = "its drive levels are not four rising levels below 255, then 255";
		} else {
			for (int i = 0; i < 4; i++) {
				out->shades[i] = s[i] / 255.0f;
			}
		}
	}
	const bool own = version == CAL_FORMAT_VERSION_OWN;
	const cJSON *eyes = wrong == NULL ? cJSON_GetObjectItemCaseSensitive(manifest, "eyes") : NULL;
	const cJSON *blobs = wrong == NULL ? cJSON_GetObjectItemCaseSensitive(manifest, "blobs") : NULL;
	if (wrong == NULL && !own && (!cJSON_IsArray(eyes) || cJSON_GetArraySize(eyes) != 2 || !cJSON_IsArray(blobs))) {
		wrong = "it does not hold two eyes";
	}
	if (wrong == NULL && own) {
		const cJSON *resp = cJSON_GetObjectItemCaseSensitive(manifest, "response");
		const cJSON *pix = cJSON_GetObjectItemCaseSensitive(manifest, "pixel");
		const cJSON *re = cJSON_GetObjectItemCaseSensitive(resp, "encoding");
		const cJSON *pe = cJSON_GetObjectItemCaseSensitive(pix, "encoding");
		if (!cJSON_IsArray(blobs) || number_of(resp, "width") != RESPONSE_W || number_of(resp, "height") != RESPONSE_H ||
		    number_of(pix, "width") != WALL_W || number_of(pix, "height") != WALL_H || !cJSON_IsString(re) ||
		    strcmp(re->valuestring, "float32") != 0 || !cJSON_IsString(pe) || strcmp(pe->valuestring, "float16") != 0) {
			wrong = "its maps are not 54 x 60 floats and 1080 x 1200 half floats";
		}
	}
	if (wrong == NULL) {
		response = calloc((size_t)6 * RESPONSE_H * RESPONSE_W * 4, sizeof(float));
		wall = calloc((size_t)2 * WALL_H * WALL_W * 4, own ? 2 : 1);
		if (response == NULL || wall == NULL) {
			wrong = "out of memory";
		}
	}
	// RiftLinux's own calibration (format 2): per eye the knots as floats (colour, level, row, column) and the
	// per-pixel corrections as half floats (channel, row, column), used as they are (scale 1, offset 0).
	for (int e = 0; e < 2 && wrong == NULL && own; e++) {
		const size_t n = (size_t)RESPONSE_W * RESPONSE_H;
		const size_t np = (size_t)WALL_W * WALL_H;
		char name[32];
		size_t at = 0;
		snprintf(name, sizeof(name), "eye%d.response", e);
		if (!blob(blobs, name, body_size, 3 * 4 * n * sizeof(float), &at)) {
			wrong = "a response map is missing or cut short";
			break;
		}
		for (int c = 0; c < 3; c++) {
			float *dst = response + (size_t)(e * 3 + c) * n * 4;
			for (int k = 0; k < 4; k++) {
				for (size_t i = 0; i < n; i++) {
					float v;
					memcpy(&v, body + at + (((size_t)c * 4 + k) * n + i) * sizeof(float), sizeof(v));
					if (!isfinite(v)) {
						wrong = "its response maps are not finite";
					}
					dst[i * 4 + k] = v;
				}
			}
		}
		snprintf(name, sizeof(name), "eye%d.pixel", e);
		if (wrong == NULL && !blob(blobs, name, body_size, 3 * np * sizeof(uint16_t), &at)) {
			wrong = "a per-pixel map is missing or cut short";
		}
		if (wrong == NULL) {
			uint16_t *dst = (uint16_t *)wall + (size_t)e * np * 4;
			for (int c = 0; c < 3; c++) {
				for (size_t i = 0; i < np; i++) {
					memcpy(&dst[i * 4 + c], body + at + ((size_t)c * np + i) * sizeof(uint16_t), sizeof(uint16_t));
				}
			}
			for (int c = 0; c < 4; c++) {
				out->wall_scale[e][c] = c < 3 ? 1.0f : 0.0f;
				out->wall_offset[e][c] = 0.0f;
			}
		}
	}
	out->wall_half = own;
	for (int e = 0; e < 2 && wrong == NULL && !own; e++) {
		const cJSON *eye = cJSON_GetArrayItem(eyes, e);
		const cJSON *resp = cJSON_GetObjectItemCaseSensitive(eye, "response");
		const cJSON *wl = cJSON_GetObjectItemCaseSensitive(eye, "wall");
		float scales[12], offsets[12];
		if (number_of(resp, "width") != RESPONSE_W || number_of(resp, "height") != RESPONSE_H ||
		    number_of(wl, "width") != WALL_W || number_of(wl, "height") != WALL_H) {
			wrong = "its maps are not 54 x 60 and 1080 x 1200";
		} else if (!numbers(cJSON_GetObjectItemCaseSensitive(resp, "scales_level_major"), scales, 12) ||
		           !numbers(cJSON_GetObjectItemCaseSensitive(resp, "offsets_level_major"), offsets, 12) ||
		           !numbers(cJSON_GetObjectItemCaseSensitive(wl, "scales"), out->wall_scale[e], 4) ||
		           !numbers(cJSON_GetObjectItemCaseSensitive(wl, "offsets"), out->wall_offset[e], 4)) {
			wrong = "its constants are missing or not finite";
		}
		static const char *const names[3] = {"R", "G", "B"};
		for (int c = 0; c < 3 && wrong == NULL; c++) {
			char name[32];
			snprintf(name, sizeof(name), "eye%d.response.%s", e, names[c]);
			size_t at = 0;
			if (!blob(blobs, name, body_size, (size_t)RESPONSE_W * RESPONSE_H * 4, &at)) {
				wrong = "a response map is missing or cut short";
				break;
			}
			// The four knots: byte / 255 * scale[level][colour] + offset[level][colour] (riftlinux-setup's reader).
			float *dst = response + (size_t)(e * 3 + c) * RESPONSE_H * RESPONSE_W * 4;
			for (size_t i = 0; i < (size_t)RESPONSE_W * RESPONSE_H; i++) {
				for (int k = 0; k < 4; k++) {
					dst[i * 4 + k] =
					    (float)body[at + i * 4 + k] / 255.0f * scales[k * 3 + c] + offsets[k * 3 + c];
				}
			}
		}
		char name[32];
		snprintf(name, sizeof(name), "eye%d.wall", e);
		if (wrong == NULL && !blob(blobs, name, body_size, (size_t)WALL_W * WALL_H * 4, &wall_at[e])) {
			wrong = "a per-pixel map is missing or cut short";
		}
		if (wrong == NULL) {
			memcpy(wall + (size_t)e * WALL_H * WALL_W * 4, body + wall_at[e], (size_t)WALL_W * WALL_H * 4);
		}
	}
	cJSON_Delete(manifest);
	free(data);
	if (wrong != NULL) {
		free(response);
		free(wall);
		memset(out, 0, sizeof(*out));
		return wrong;
	}
	out->response = response;
	out->wall = wall;
	return NULL;
}

//! The images of @p p made and filled (any thread: the pool and the queue are locked for the upload).
static bool
prepared_to_images(struct render_resources *r, const struct prepared *p, struct images *out)
{
	return make_images(r, (VkExtent2D){RESPONSE_W, RESPONSE_H}, p->response, (VkExtent2D){WALL_W, WALL_H}, p->wall,
	                   p->wall_half, out);
}

//! Puts @p im (and @p p's constants) in use; the images it replaces are freed: only with the queue idle.
static void
use_images(struct render_resources *r, const struct images *im, const struct prepared *p)
{
	__typeof__(r->compute.distortion.mura) *m = &r->compute.distortion.mura;
	take(r, im);
	memcpy(m->wall_scale, p->wall_scale, sizeof(m->wall_scale));
	memcpy(m->wall_offset, p->wall_offset, sizeof(m->wall_offset));
	memcpy(m->shades, p->shades, sizeof(m->shades));
	m->loaded = true;
}


/*
 *
 * 'Exported' functions.
 *
 */

bool
render_mura_init(struct render_resources *r)
{
	struct vk_bundle *vk = r->vk;
	__typeof__(r->compute.distortion.mura) *m = &r->compute.distortion.mura;
	memset(m, 0, sizeof(*m));

	VkResult ret = vk_create_sampler(vk, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, &m->sampler);
	VK_CHK_WITH_RET(ret, "vk_create_sampler (panel calibration)", false);

	// Neutral images: bound while the correction is off (the shader does not read them then).
	const float response[1 * 1 * 6 * 4] = {0};
	const uint8_t wall[1 * 1 * 2 * 4] = {0};
	struct images im = {0};
	if (!make_images(r, (VkExtent2D){1, 1}, response, (VkExtent2D){1, 1}, wall, false, &im)) {
		return false;
	}
	take(r, &im);
	return true;
}

bool
render_mura_load(struct render_resources *r, const char *serial)
{
	__typeof__(r->compute.distortion.mura) *m = &r->compute.distortion.mura;
	if (serial == NULL || serial[0] == '\0') {
		U_LOG_W("Panel calibration: the headset reports no serial, so none is loaded");
		return false;
	}

	char path[1024];
	if (!calibration_path(serial, path, sizeof(path))) {
		return false;
	}
	file_state(path, &m->file_seen, &m->file_mtime_ns, &m->file_size);

	struct prepared p;
	bool missing;
	const char *wrong = read_calibration(serial, path, &p, &missing);
	if (missing) {
		U_LOG_W("Panel calibration: none for headset %s (%s)", serial, path);
		return false;
	}
	if (wrong != NULL) {
		U_LOG_W("Panel calibration: %s not used: %s", path, wrong);
		return false;
	}
	struct images im = {0};
	const bool made = prepared_to_images(r, &p, &im);
	if (!made) {
		prepared_free(&p);
		U_LOG_W("Panel calibration for headset %s could not be put on the GPU", serial);
		return false;
	}
	use_images(r, &im, &p);
	prepared_free(&p);
	// Warning level: the default log level shows it (what is in use for the picture belongs in the log).
	U_LOG_W("Panel calibration: loaded for headset %s (%s; response maps %dx%d, per-pixel maps %dx%d)", serial, path,
	        RESPONSE_W, RESPONSE_H, WALL_W, WALL_H);
	return true;
}

/*!
 * A calibration loaded while running (render_mura_reload_if_changed): read, decoded and put on the GPU on its own
 * thread (the command pool and the queue are locked for the upload), so no frame waits for it; the render thread only
 * puts the finished images in use, with the queue idle.
 */
struct render_mura_loader
{
	struct render_resources *r;
	pthread_t thread;
	char serial[64];
	char path[1024];
	int64_t start_ns;

	pthread_mutex_t lock;
	//! Set last by the thread, with the outcome below.
	bool done;
	const char *wrong;
	bool missing;
	bool gpu_failed;
	struct prepared p; //!< The constants (its maps are freed once uploaded).
	struct images im;
	int64_t ready_ns;
};

static void *
loader_run(void *ptr)
{
	struct render_mura_loader *l = ptr;
	struct prepared p;
	bool missing;
	const char *wrong = read_calibration(l->serial, l->path, &p, &missing);
	struct images im = {0};
	bool gpu_failed = false;
	if (wrong == NULL) {
		gpu_failed = !prepared_to_images(l->r, &p, &im);
		prepared_free(&p);
	}
	pthread_mutex_lock(&l->lock);
	l->wrong = wrong;
	l->missing = missing;
	l->gpu_failed = gpu_failed;
	l->p = p;
	l->im = im;
	l->ready_ns = os_monotonic_get_ns();
	l->done = true;
	pthread_mutex_unlock(&l->lock);
	return NULL;
}

//! Waits for the loader's thread and frees it with what it made (not put in use).
static void
loader_free(struct render_resources *r, struct render_mura_loader *l)
{
	pthread_join(l->thread, NULL);
	if (l->wrong == NULL && !l->gpu_failed) {
		destroy(r->vk, &l->im);
	}
	pthread_mutex_destroy(&l->lock);
	free(l);
}

void
render_mura_reload_if_changed(struct render_resources *r, const char *serial, int64_t now_ns)
{
	__typeof__(r->compute.distortion.mura) *m = &r->compute.distortion.mura;
	struct render_mura_loader *l = m->loader;
	if (l != NULL) {
		pthread_mutex_lock(&l->lock);
		const bool done = l->done;
		pthread_mutex_unlock(&l->lock);
		if (!done) {
			return;
		}
		pthread_join(l->thread, NULL);
		if (l->missing) {
			U_LOG_W("Panel calibration: %s is gone again, the one in use stays", l->path);
		} else if (l->wrong != NULL) {
			U_LOG_W("Panel calibration: %s not used: %s", l->path, l->wrong);
		} else if (l->gpu_failed) {
			U_LOG_W("Panel calibration for headset %s could not be put on the GPU", l->serial);
		} else {
			const int64_t swap_ns = os_monotonic_get_ns();
			use_images(r, &l->im, &l->p);
			U_LOG_W("Panel calibration: loaded for headset %s (%s; response maps %dx%d, per-pixel maps %dx%d)",
			        l->serial, l->path, RESPONSE_W, RESPONSE_H, WALL_W, WALL_H);
			U_LOG_W("Panel calibration: read and put on the GPU in %.1f ms beside the frames, in use after %.2f ms",
			        (double)(l->ready_ns - l->start_ns) / 1e6, (double)(os_monotonic_get_ns() - swap_ns) / 1e6);
		}
		pthread_mutex_destroy(&l->lock);
		free(l);
		m->loader = NULL;
		return;
	}

	if (now_ns < m->next_check_ns || serial == NULL || serial[0] == '\0') {
		return;
	}
	m->next_check_ns = now_ns + 1000000000;

	struct render_output_settings s;
	render_output_settings_get(&s);
	char path[1024];
	if (!s.mura_correction || !calibration_path(serial, path, sizeof(path))) {
		return;
	}
	bool seen;
	int64_t mtime_ns, size;
	file_state(path, &seen, &mtime_ns, &size);
	if (!seen || (m->file_seen && mtime_ns == m->file_mtime_ns && size == m->file_size)) {
		return; // unchanged, or removed (the one loaded stays until the next start)
	}
	U_LOG_W("Panel calibration: %s %s, loading it", path, m->file_seen ? "changed" : "appeared");
	m->file_seen = seen; // as it is now: loaded once, again only when it changes again
	m->file_mtime_ns = mtime_ns;
	m->file_size = size;

	l = calloc(1, sizeof(*l));
	if (l == NULL) {
		return;
	}
	l->r = r;
	snprintf(l->serial, sizeof(l->serial), "%s", serial);
	snprintf(l->path, sizeof(l->path), "%s", path);
	l->start_ns = os_monotonic_get_ns();
	pthread_mutex_init(&l->lock, NULL);
	if (pthread_create(&l->thread, NULL, loader_run, l) != 0) {
		U_LOG_W("Panel calibration: could not start loading %s", path);
		pthread_mutex_destroy(&l->lock);
		free(l);
		return;
	}
	m->loader = l;
}

void
render_mura_fini(struct render_resources *r)
{
	struct vk_bundle *vk = r->vk;
	__typeof__(r->compute.distortion.mura) *m = &r->compute.distortion.mura;
	if (m->loader != NULL) {
		loader_free(r, m->loader);
		m->loader = NULL;
	}
	struct images old = {m->response_image, m->wall_image, m->response_memory,
	                     m->wall_memory,    m->response_view, m->wall_view};
	destroy(vk, &old);
	D(Sampler, m->sampler);
	m->loaded = false;
}
