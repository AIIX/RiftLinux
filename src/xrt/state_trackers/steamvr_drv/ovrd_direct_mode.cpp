// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SteamVR driver direct mode into Monado's compositor, see ovrd_direct_mode.hpp.
 */

#include "ovrd_direct_mode.hpp"
#include "ovrd_log.hpp"

#include "math/m_api.h"
#include "math/m_vec3.h"
#include "os/os_time.h"
#include "util/u_time.h"

#include <unistd.h>
#include <dlfcn.h>
#include <string>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>


#ifdef XRT_HAVE_VULKAN
#define RLN_CAPTURE 1
#include "vk/vk_helpers.h"
#include "vk/vk_cmd_pool.h"
#include "util/comp_swapchain.h"
#include <sys/stat.h>
#endif

namespace {

/*!
 * Test (settings.json display.motion_pacing_trace N, _after_s): the first N pacing events after that many seconds,
 * each with its time, into ~/.cache/riftlinux/pacing-trace.txt: when SteamVR calls the driver against the vsyncs we
 * declare, for the frame lock's hold. Written once, by a thread of its own.
 */
struct PacingTrace
{
	std::mutex lock;
	std::vector<std::array<int64_t, 3>> events;
	size_t want = 0;
	int64_t from_ns = 0;
	bool done = false;

	PacingTrace()
	{
		const char *n = getenv("RIFTLINUX_PACING_TRACE");
		const char *after = getenv("RIFTLINUX_PACING_TRACE_AFTER_S");
		want = n != nullptr ? (size_t)std::clamp(atoi(n), 0, 200000) : 0;
		from_ns = os_monotonic_get_ns() + (after != nullptr ? atoi(after) : 30) * (int64_t)U_TIME_1S_IN_NS;
		events.reserve(want);
	}

	void
	add(char type, int64_t a = 0, int64_t b = 0)
	{
		if (want == 0) {
			return;
		}
		const int64_t now = os_monotonic_get_ns();
		std::lock_guard<std::mutex> guard(lock);
		if (done || now < from_ns) {
			return;
		}
		events.push_back({now, (int64_t)type | (a << 8), b});
		if (events.size() < want) {
			return;
		}
		done = true;
		std::thread([list = std::move(events)]() {
			const char *home = getenv("HOME");
			std::string path = std::string(home != nullptr ? home : "/tmp") + "/.cache/riftlinux/pacing-trace.txt";
			FILE *f = fopen(path.c_str(), "w");
			if (f == nullptr) {
				return;
			}
			for (const auto &e : list) {
				fprintf(f, "%lld %c %lld %lld\n", (long long)e[0], (char)(e[1] & 0xff), (long long)(e[1] >> 8),
				        (long long)e[2]);
			}
			fclose(f);
			ovrd_log("Direct mode: pacing trace written (%zu events)\n", list.size());
		}).detach();
	}
};

PacingTrace &
pacing_trace()
{
	static PacingTrace trace;
	return trace;
}

#ifdef RLN_CAPTURE
/*!
 * Copies SteamVR's shared images out, for finding what they hold and when they are finished: from Monado's own
 * import of them (a second import into a device of the driver's was refused), on Monado's Vulkan device. Diagnostics
 * only (display.direct_mode_capture), it stalls the frame it runs in.
 */
struct Capture
{
	struct vk_bundle *vk = nullptr;
	struct vk_cmd_pool pool = {};

	//! Image @p index of @p xsc (Monado's swapchain of the set), every 4th pixel each way, bytes as stored.
	bool
	grab(struct xrt_swapchain *xsc, uint32_t index, uint32_t w, uint32_t h, std::vector<uint8_t> &out, float mean[3])
	{
		// Imported through the multi compositor into the main one: a comp_swapchain.
		struct comp_swapchain *sc = (struct comp_swapchain *)xsc;
		if (vk == nullptr) {
			if (vk_cmd_pool_init(sc->vk, &pool, VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT) != VK_SUCCESS) {
				ovrd_log("Direct mode: capture: no command pool\n");
				return false;
			}
			vk = sc->vk;
		}
		const VkImage image = sc->vkic.images[index].handle;
		const VkDeviceSize size = (VkDeviceSize)w * h * 4;
		VkBuffer buf = VK_NULL_HANDLE;
		VkDeviceMemory mem = VK_NULL_HANDLE;
		if (!vk_buffer_init(vk, size, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		                    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &buf, &mem)) {
			ovrd_log("Direct mode: capture: no buffer\n");
			return false;
		}
		vk_cmd_pool_lock(&pool);
		VkCommandBuffer cmd = VK_NULL_HANDLE;
		VkResult r = vk_cmd_pool_create_and_begin_cmd_buffer_locked(vk, &pool, 0, &cmd);
		if (r == VK_SUCCESS) {
			// Monado keeps the images it samples in shader-read layout.
			VkImageMemoryBarrier bar = {};
			bar.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
			bar.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
			bar.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			bar.oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			bar.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			bar.image = image;
			bar.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
			vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
			                         0, nullptr, 0, nullptr, 1, &bar);
			VkBufferImageCopy copy = {};
			copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			copy.imageExtent = {w, h, 1};
			vk->vkCmdCopyImageToBuffer(cmd, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buf, 1, &copy);
			bar.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			bar.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
			bar.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			bar.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
			                         nullptr, 0, nullptr, 1, &bar);
			r = vk_cmd_pool_end_submit_wait_and_free_cmd_buffer_locked(vk, &pool, cmd);
		}
		vk_cmd_pool_unlock(&pool);
		bool ok = r == VK_SUCCESS;
		if (!ok) {
			ovrd_log("Direct mode: capture: copy %d\n", (int)r);
		}
		void *p = nullptr;
		if (ok && vk->vkMapMemory(vk->device, mem, 0, size, 0, &p) == VK_SUCCESS) {
			const uint8_t *px = (const uint8_t *)p;
			const uint32_t ow = w / 4, oh = h / 4;
			out.resize((size_t)ow * oh * 4);
			double sum[3] = {0, 0, 0};
			for (uint32_t y = 0; y < oh; y++) {
				for (uint32_t x = 0; x < ow; x++) {
					const uint8_t *src = px + ((size_t)(y * 4) * w + x * 4) * 4;
					memcpy(&out[((size_t)y * ow + x) * 4], src, 4);
					for (int c = 0; c < 3; c++) {
						sum[c] += src[c];
					}
				}
			}
			for (int c = 0; c < 3; c++) {
				mean[c] = (float)(sum[c] / ((double)ow * oh));
			}
			vk->vkUnmapMemory(vk->device, mem);
		} else {
			ok = false;
		}
		vk->vkDestroyBuffer(vk->device, buf, nullptr);
		vk->vkFreeMemory(vk->device, mem, nullptr);
		return ok;
	}
};

void
write_capture(const std::vector<uint8_t> &px, uint32_t w, uint32_t h, const char *name)
{
	const char *home = getenv("HOME");
	const std::string dir = std::string(home != nullptr ? home : "/tmp") + "/.cache/riftlinux/dm-capture";
	mkdir((std::string(home != nullptr ? home : "/tmp") + "/.cache/riftlinux").c_str(), 0700);
	mkdir(dir.c_str(), 0700);
	FILE *f = fopen((dir + "/" + name).c_str(), "wb");
	if (f == nullptr) {
		return;
	}
	const uint32_t hdr[2] = {w / 4, h / 4};
	fwrite("RLRG", 1, 4, f);
	fwrite(hdr, 4, 2, f);
	fwrite(px.data(), 1, px.size(), f);
	fclose(f);
}

Capture g_capture;
#endif

// Vulkan values, as SteamVR's resource manager takes them (no Vulkan headers needed here).
constexpr uint32_t kVkImageCreateMutableFormat = 0x00000008;
constexpr uint32_t kVkImageUsageTransferSrc = 0x00000001;
constexpr uint32_t kVkImageUsageTransferDst = 0x00000002;
constexpr uint32_t kVkImageUsageSampled = 0x00000004;
constexpr uint32_t kVkImageUsageInputAttachment = 0x00000080;

//! A pose from SteamVR's 3x4 row-major matrix.
struct xrt_pose
pose_from_matrix(const vr::HmdMatrix34_t &m)
{
	struct xrt_matrix_3x3 rot = {{
	    m.m[0][0], m.m[0][1], m.m[0][2], //
	    m.m[1][0], m.m[1][1], m.m[1][2], //
	    m.m[2][0], m.m[2][1], m.m[2][2], //
	}};
	struct xrt_pose pose = XRT_POSE_IDENTITY;
	math_quat_from_matrix_3x3(&rot, &pose.orientation);
	math_quat_normalize(&pose.orientation);
	pose.position = {m.m[0][3], m.m[1][3], m.m[2][3]};
	return pose;
}

#ifdef RLN_CAPTURE
//! SMPTE ST 2084 (PQ) signal for @p nits.
float
nits_to_pq(float nits)
{
	const float m1 = 2610.0f / 16384.0f, m2 = 2523.0f / 4096.0f * 128.0f;
	const float c1 = 3424.0f / 4096.0f, c2 = 2413.0f / 4096.0f * 32.0f, c3 = 2392.0f / 4096.0f * 32.0f;
	const float y = powf(std::max(nits, 0.0f) / 10000.0f, m1);
	return powf((c1 + c2 * y) / (1.0f + c3 * y), m2);
}

float
srgb_to_linear(float v)
{
	return v <= 0.04045f ? v / 12.92f : powf((v + 0.055f) / 1.055f, 2.4f);
}

/*!
 * The HDR test chart, 1024 x 512, as HDR10 video is: PQ signal, BT.2020 primaries, 16 bits a channel.
 * Top row: greys at 0.5, 5, 50, 100, 203 (reference white), 400, 1000, 4000 nits. Middle row, at reference white
 * unless said: BT.709 red, green, blue; BT.2020 red, green (outside BT.709); skin; sky; an orange highlight at three
 * times reference white. Bottom row: the PQ signal from 0 to 1 (0 to 10 000 nits), left to right.
 */
void
hdr_test_chart_pixels(std::vector<uint16_t> &px, uint32_t w, uint32_t h)
{
	const float white = 203.0f;
	const float greys[8] = {0.5f, 5.0f, 50.0f, 100.0f, 203.0f, 400.0f, 1000.0f, 4000.0f};
	// Linear, reference white 1.0; BT.709 unless the fourth value is 1 (BT.2020).
	const float colours[8][4] = {
	    {1.0f, 0.0f, 0.0f, 0.0f},
	    {0.0f, 1.0f, 0.0f, 0.0f},
	    {0.0f, 0.0f, 1.0f, 0.0f},
	    {1.0f, 0.0f, 0.0f, 1.0f},
	    {0.0f, 1.0f, 0.0f, 1.0f},
	    {srgb_to_linear(224 / 255.0f), srgb_to_linear(172 / 255.0f), srgb_to_linear(105 / 255.0f), 0.0f},
	    {srgb_to_linear(135 / 255.0f), srgb_to_linear(206 / 255.0f), srgb_to_linear(235 / 255.0f), 0.0f},
	    {3.0f, 1.5f, 0.3f, 0.0f},
	};
	px.assign((size_t)w * h * 4, 0);
	for (uint32_t y = 0; y < h; y++) {
		for (uint32_t x = 0; x < w; x++) {
			const uint32_t col = x * 8 / w;
			float rgb[3];
			if (y < h / 3) {
				rgb[0] = rgb[1] = rgb[2] = greys[col] / white;
			} else if (y < 2 * h / 3) {
				const float *c = colours[col];
				if (c[3] > 0.5f) {
					rgb[0] = c[0], rgb[1] = c[1], rgb[2] = c[2];
				} else {
					// BT.709 to BT.2020, linear.
					rgb[0] = 0.6274f * c[0] + 0.3293f * c[1] + 0.0433f * c[2];
					rgb[1] = 0.0691f * c[0] + 0.9195f * c[1] + 0.0114f * c[2];
					rgb[2] = 0.0164f * c[0] + 0.0880f * c[1] + 0.8956f * c[2];
				}
			} else {
				const float signal = (float)x / (float)(w - 1);
				uint16_t *o = &px[((size_t)y * w + x) * 4];
				o[0] = o[1] = o[2] = (uint16_t)lroundf(signal * 65535.0f);
				o[3] = 65535;
				continue;
			}
			uint16_t *o = &px[((size_t)y * w + x) * 4];
			for (int i = 0; i < 3; i++) {
				o[i] = (uint16_t)lroundf(nits_to_pq(rgb[i] * white) * 65535.0f);
			}
			o[3] = 65535;
		}
	}
}

/*!
 * The labelled HDR test chart, resources/notices/hdr-chart.rlhd (scripts/riftlinux/make_notice_images.py): "RLHD",
 * width, height, then RGBA as 16-bit PQ (BT.2020). False if missing; then the unlabelled chart above is used.
 */
bool
load_hdr_test_chart(std::vector<uint16_t> &px, uint32_t &w, uint32_t &h)
{
	Dl_info info = {};
	if (dladdr(reinterpret_cast<void *>(&load_hdr_test_chart), &info) == 0 || info.dli_fname == nullptr) {
		return false;
	}
	std::string dir(info.dli_fname);
	dir = dir.substr(0, dir.rfind('/'));
	const std::string path = dir + "/../../resources/notices/hdr-chart.rlhd";
	FILE *f = fopen(path.c_str(), "rb");
	if (f == nullptr) {
		return false;
	}
	char magic[4] = {};
	uint32_t size[2] = {};
	bool ok = fread(magic, 1, 4, f) == 4 && memcmp(magic, "RLHD", 4) == 0 && fread(size, 4, 2, f) == 2 &&
	          size[0] > 0 && size[1] > 0 && size[0] <= 4096 && size[1] <= 4096;
	if (ok) {
		w = size[0];
		h = size[1];
		px.resize((size_t)w * h * 4);
		ok = fread(px.data(), sizeof(uint16_t), px.size(), f) == px.size();
	}
	fclose(f);
	return ok;
}

//! The chart's swapchain: one static image, filled once through Monado's Vulkan device.
struct xrt_swapchain *
make_hdr_test_chart(struct xrt_compositor_native *xcn, uint32_t &out_w, uint32_t &out_h)
{
	std::vector<uint16_t> px;
	uint32_t w = 1024, h = 512;
	const bool labelled = load_hdr_test_chart(px, w, h);
	if (!labelled) {
		w = 1024;
		h = 512;
		hdr_test_chart_pixels(px, w, h);
	}
	struct xrt_swapchain_create_info info = {};
	info.create = XRT_SWAPCHAIN_CREATE_STATIC_IMAGE;
	info.bits = (enum xrt_swapchain_usage_bits)(XRT_SWAPCHAIN_USAGE_COLOR | XRT_SWAPCHAIN_USAGE_SAMPLED |
	                                            XRT_SWAPCHAIN_USAGE_TRANSFER_DST);
	info.format = VK_FORMAT_R16G16B16A16_UNORM;
	info.sample_count = 1;
	info.width = w;
	info.height = h;
	info.face_count = 1;
	info.array_size = 1;
	info.mip_count = 1;
	struct xrt_swapchain *xsc = nullptr;
	if (xrt_comp_create_swapchain(&xcn->base, &info, &xsc) != XRT_SUCCESS || xsc == nullptr) {
		ovrd_log("Direct mode: HDR test chart: no swapchain\n");
		return nullptr;
	}
	uint32_t index = 0;
	if (xrt_swapchain_acquire_image(xsc, &index) != XRT_SUCCESS ||
	    xrt_swapchain_wait_image(xsc, 100LL * U_TIME_1MS_IN_NS, index) != XRT_SUCCESS) {
		ovrd_log("Direct mode: HDR test chart: no image\n");
		xrt_swapchain_reference(&xsc, nullptr);
		return nullptr;
	}

	struct comp_swapchain *sc = (struct comp_swapchain *)xsc;
	struct vk_bundle *vk = sc->vk;
	const VkDeviceSize size = (VkDeviceSize)px.size() * sizeof(uint16_t);
	VkBuffer buf = VK_NULL_HANDLE;
	VkDeviceMemory mem = VK_NULL_HANDLE;
	bool ok = vk_buffer_init(vk, size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
	                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &buf, &mem);
	void *mapped = nullptr;
	if (ok && vk->vkMapMemory(vk->device, mem, 0, size, 0, &mapped) == VK_SUCCESS) {
		memcpy(mapped, px.data(), (size_t)size);
		vk->vkUnmapMemory(vk->device, mem);
	} else {
		ok = false;
	}
	struct vk_cmd_pool pool = {};
	if (ok && vk_cmd_pool_init(vk, &pool, VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT) == VK_SUCCESS) {
		vk_cmd_pool_lock(&pool);
		VkCommandBuffer cmd = VK_NULL_HANDLE;
		VkResult r = vk_cmd_pool_create_and_begin_cmd_buffer_locked(vk, &pool, 0, &cmd);
		if (r == VK_SUCCESS) {
			const VkImage image = sc->vkic.images[index].handle;
			VkImageMemoryBarrier bar = {};
			bar.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
			bar.srcAccessMask = 0;
			bar.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			bar.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			bar.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			bar.image = image;
			bar.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
			vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
			                         0, nullptr, 0, nullptr, 1, &bar);
			VkBufferImageCopy copy = {};
			copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			copy.imageExtent = {w, h, 1};
			vk->vkCmdCopyBufferToImage(cmd, buf, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
			bar.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			bar.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
			bar.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			bar.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
			                         nullptr, 0, nullptr, 1, &bar);
			r = vk_cmd_pool_end_submit_wait_and_free_cmd_buffer_locked(vk, &pool, cmd);
		}
		vk_cmd_pool_unlock(&pool);
		vk_cmd_pool_destroy(vk, &pool);
		ok = r == VK_SUCCESS;
	} else {
		ok = false;
	}
	if (buf != VK_NULL_HANDLE) {
		vk->vkDestroyBuffer(vk->device, buf, nullptr);
		vk->vkFreeMemory(vk->device, mem, nullptr);
	}
	xrt_swapchain_release_image(xsc, index);
	if (!ok) {
		ovrd_log("Direct mode: HDR test chart: upload failed\n");
		xrt_swapchain_reference(&xsc, nullptr);
		return nullptr;
	}
	ovrd_log("Direct mode: HDR test chart shown (PQ, BT.2020, %ux%u, %s)\n", w, h,
	         labelled ? "labelled" : "unlabelled, resources/notices/hdr-chart.rlhd missing");
	out_w = w;
	out_h = h;
	return xsc;
}
#endif

} // namespace

OvrdDirectMode::OvrdDirectMode(struct xrt_compositor_native *xcn,
                               struct xrt_device *head,
                               std::function<void(struct xrt_pose *)> driver_from_world,
                               float guard_ms,
                               bool compositor_unorm,
                               bool app_unorm)
    : m_xcn(xcn), m_head(head), m_driver_from_world(std::move(driver_from_world)), m_guard_ms(guard_ms),
      m_compositor_unorm(compositor_unorm), m_app_unorm(app_unorm)
{
	// The log is per source file (ovrd_log.hpp): this one's too.
	ovrd_log_init(vr::VRDriverLog());
	ovrd_log("Direct mode: SteamVR's images go to Monado's compositor; its compositor's read as %s, the "
	         "application's as %s\n",
	         compositor_unorm ? "UNORM" : "sRGB", app_unorm ? "UNORM" : "sRGB");
}

OvrdDirectMode::~OvrdDirectMode()
{
	m_vsync_exit.store(true);
	if (m_vsync_thread.joinable()) {
		m_vsync_thread.join();
	}
	std::lock_guard<std::mutex> lock(m_lock);
	ovrd_log("Direct mode: driver cleanup, %zu set(s) still held\n", m_sets.size());
	while (!m_sets.empty()) {
		destroy_set_locked(m_sets.back().get());
	}
	xrt_swapchain_reference(&m_hdr_test_xsc, nullptr);
}

void
OvrdDirectMode::CreateSwapTextureSet(uint32_t unPid,
                                     const SwapTextureSetDesc_t *pSwapTextureSetDesc,
                                     SwapTextureSet_t *pOutSwapTextureSet)
{
	const SwapTextureSetDesc_t &desc = *pSwapTextureSetDesc;
	ovrd_log("Direct mode: swap texture set for pid %u: format %u, %ux%u, %u samples\n", unPid, desc.nFormat,
	         desc.nWidth, desc.nHeight, desc.nSampleCount);
	memset(pOutSwapTextureSet, 0, sizeof(*pOutSwapTextureSet));

	auto set = std::make_unique<Set>();
	set->pid = unPid;
	set->desc = desc;

	/*
	 * SteamVR's shared images, described exactly as Monado then imports them: the GPU lays out (and compresses)
	 * an image by its description, and read through a different one the frames came out dark, some flashing right
	 * (by eye; captures black). Renderable (colour attachment), no compute (storage), and these on both sides.
	 */
	const uint32_t usage =
	    kVkImageUsageTransferSrc | kVkImageUsageTransferDst | kVkImageUsageSampled | kVkImageUsageInputAttachment;
	struct xrt_image_native natives[3] = {};
	for (int i = 0; i < 3; i++) {
		vr::SharedTextureHandle_t handle = 0;
		uint64_t ipc = 0;
		int fd = -1;
		if (!vr::VRIPCResourceManager()->NewSharedVulkanImage(desc.nFormat, desc.nWidth, desc.nHeight, true, false,
		                                                      false, 1, 1, kVkImageCreateMutableFormat, usage,
		                                                      &handle) ||
		    !vr::VRIPCResourceManager()->RefResource(handle, &ipc) ||
		    !vr::VRIPCResourceManager()->ReceiveSharedFd(ipc, &fd) || fd < 0) {
			ovrd_log("Direct mode: SteamVR made no shared image %d\n", i);
			if (handle != 0) {
				vr::VRIPCResourceManager()->UnrefResource(handle);
			}
			std::lock_guard<std::mutex> lock(m_lock);
			destroy_set_locked(set.get());
			return;
		}
		set->handles[i] = handle;
		set->fds[i] = fd;
		natives[i].handle = fd;
		// Monado checks the memory is as large as its image needs; a dma-buf tells its size. Else the image's
		// own requirement goes unchecked (Monado allocates by that).
		const off_t size = lseek(fd, 0, SEEK_END);
		lseek(fd, 0, SEEK_SET);
		natives[i].size = size > 0 ? (uint64_t)size : (1ull << 40);
		natives[i].use_dedicated_allocation = true;
		if (i == 0) {
			ovrd_log("Direct mode: shared image %d bytes (%s)\n", (int)size,
			         size > 0 ? "dma-buf size" : "unknown, not checked");
		}
	}

	// The same images as a Monado swapchain; Monado duplicates the descriptors.
	struct xrt_swapchain_create_info info = {};
	info.bits = (enum xrt_swapchain_usage_bits)(XRT_SWAPCHAIN_USAGE_COLOR | XRT_SWAPCHAIN_USAGE_SAMPLED |
	                                            XRT_SWAPCHAIN_USAGE_TRANSFER_SRC | XRT_SWAPCHAIN_USAGE_TRANSFER_DST |
	                                            XRT_SWAPCHAIN_USAGE_INPUT_ATTACHMENT |
	                                            XRT_SWAPCHAIN_USAGE_MUTABLE_FORMAT);
	/*
	 * SteamVR asks for sRGB images (format 43) for its compositor and for the application, whose images it can
	 * hand over directly, but the two store different values in them: read alike, most frames were too dark and
	 * some right, or most right and some too bright (by eye). Each set is read by whose it is (its process):
	 * display.direct_mode_format_compositor / _app, "unorm" (as stored) or "srgb".
	 */
	char comm[64] = "";
	{
		char path[64];
		snprintf(path, sizeof(path), "/proc/%u/comm", unPid);
		FILE *f = fopen(path, "r");
		if (f != nullptr) {
			if (fgets(comm, sizeof(comm), f) == nullptr) {
				comm[0] = '\0';
			}
			comm[strcspn(comm, "\n")] = '\0';
			fclose(f);
		}
	}
	set->is_compositor = strcmp(comm, "vrcompositor") == 0;
	set->number = m_set_count++;
	/*
	 * SteamVR hands an application's float images over as they are (format 97 for a game submitting
	 * R16G16B16A16_SFLOAT in linear colour, in a desk test): linear light, white at 1.0, highlights above; the
	 * compositor shows them as HDR layers (comp_layer_accum.c, display.hdr_float_layers).
	 */
	switch (desc.nFormat) {
	case 90:  // VK_FORMAT_R16G16B16_SFLOAT
	case 97:  // VK_FORMAT_R16G16B16A16_SFLOAT
	case 106: // VK_FORMAT_R32G32B32_SFLOAT
	case 109: // VK_FORMAT_R32G32B32A32_SFLOAT
	case 122: // VK_FORMAT_B10G11R11_UFLOAT_PACK32
	case 123: // VK_FORMAT_E5B9G9R9_UFLOAT_PACK32
		set->is_float = true;
		break;
	default: break;
	}
	const bool read_unorm = set->is_compositor ? m_compositor_unorm : m_app_unorm;
	ovrd_log("Direct mode: set %d of %s (%s), read as %s\n", set->number, comm[0] ? comm : "?",
	         set->is_compositor ? "SteamVR's compositor" : "the application",
	         set->is_float ? "linear float (HDR)" : (read_unorm ? "UNORM" : "sRGB"));
	info.format = desc.nFormat;
	if (read_unorm) {
		if (desc.nFormat == 43) { // VK_FORMAT_R8G8B8A8_SRGB
			info.format = 37;  // VK_FORMAT_R8G8B8A8_UNORM
		} else if (desc.nFormat == 50) { // VK_FORMAT_B8G8R8A8_SRGB
			info.format = 44;  // VK_FORMAT_B8G8R8A8_UNORM
		}
		info.format_count = 2;
		info.formats[0] = info.format;
		info.formats[1] = desc.nFormat;
	}
	info.sample_count = std::max(1u, desc.nSampleCount);
	info.width = desc.nWidth;
	info.height = desc.nHeight;
	info.face_count = 1;
	info.array_size = 1;
	info.mip_count = 1;
	const xrt_result_t xret = xrt_comp_import_swapchain(&m_xcn->base, &info, natives, 3, &set->xsc);
	if (xret != XRT_SUCCESS) {
		ovrd_log("Direct mode: Monado could not import the images (%d)\n", (int)xret);
		std::lock_guard<std::mutex> lock(m_lock);
		destroy_set_locked(set.get());
		return;
	}

	ovrd_log("Direct mode: imported into Monado as a swapchain; set %d shared images 0x%llx 0x%llx 0x%llx\n",
	         set->number, (unsigned long long)set->handles[0], (unsigned long long)set->handles[1],
	         (unsigned long long)set->handles[2]);
	std::lock_guard<std::mutex> lock(m_lock);
	// SteamVR draws into the first image before it asks for an index.
	acquire_locked(set.get());
	for (uint32_t i = 0; i < 3; i++) {
		pOutSwapTextureSet->rSharedTextureHandles[i] = set->handles[i];
		m_handles[set->handles[i]] = {set.get(), i};
	}
	m_sets.push_back(std::move(set));
}

void
OvrdDirectMode::acquire_locked(Set *set)
{
	uint32_t index = 0;
	if (xrt_swapchain_acquire_image(set->xsc, &index) != XRT_SUCCESS ||
	    xrt_swapchain_wait_image(set->xsc, U_TIME_1S_IN_NS, index) != XRT_SUCCESS) {
		ovrd_log("Direct mode: no image to acquire\n");
		set->acquired = -1;
		return;
	}
	set->acquired = (int32_t)index;
}

void
OvrdDirectMode::destroy_set_locked(Set *set)
{
	for (int i = 0; i < 3; i++) {
		if (set->handles[i] != 0) {
			m_handles.erase(set->handles[i]);
			vr::VRIPCResourceManager()->UnrefResource(set->handles[i]);
			set->handles[i] = 0;
		}
		if (set->fds[i] >= 0) {
			close(set->fds[i]);
			set->fds[i] = -1;
		}
	}
	xrt_swapchain_reference(&set->xsc, nullptr);
	m_sets.erase(std::remove_if(m_sets.begin(), m_sets.end(),
	                            [set](const std::unique_ptr<Set> &s) { return s.get() == set; }),
	             m_sets.end());
}

void
OvrdDirectMode::DestroySwapTextureSet(vr::SharedTextureHandle_t sharedTextureHandle)
{
	std::lock_guard<std::mutex> lock(m_lock);
	auto it = m_handles.find(sharedTextureHandle);
	ovrd_log("Direct mode: SteamVR destroys the set of 0x%llx (set %d)\n", (unsigned long long)sharedTextureHandle,
	         it != m_handles.end() ? it->second.first->number : -1);
	if (it != m_handles.end()) {
		destroy_set_locked(it->second.first);
	}
}

void
OvrdDirectMode::DestroyAllSwapTextureSets(uint32_t unPid)
{
	std::lock_guard<std::mutex> lock(m_lock);
	std::vector<Set *> gone;
	for (const auto &set : m_sets) {
		if (set->pid == unPid) {
			gone.push_back(set.get());
		}
	}
	ovrd_log("Direct mode: SteamVR destroys all sets of pid %u (%zu)\n", unPid, gone.size());
	for (Set *set : gone) {
		destroy_set_locked(set);
	}
}

void
OvrdDirectMode::GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t sharedTextureHandles[2],
                                           uint32_t (*pIndices)[2])
{
	std::lock_guard<std::mutex> lock(m_lock);
	m_next_calls++;
	pacing_trace().add('G', 0, (int64_t)sharedTextureHandles[0]);
	if (m_compositor_start_ns == 0) {
		m_compositor_start_ns = os_monotonic_get_ns();
	}
	Set *done[2] = {nullptr, nullptr};
	for (int eye = 0; eye < 2; eye++) {
		auto it = m_handles.find(sharedTextureHandles[eye]);
		if (it == m_handles.end()) {
			continue;
		}
		Set *set = it->second.first;
		// Both eyes in one set (side by side): one image for both.
		if (eye == 1 && set == done[0]) {
			(*pIndices)[1] = (*pIndices)[0];
			continue;
		}
		if (set->acquired < 0) {
			acquire_locked(set);
		}
		(*pIndices)[eye] = set->acquired >= 0 ? (uint32_t)set->acquired : 0;
		done[eye] = set;
	}
}

void
OvrdDirectMode::SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2])
{
	std::lock_guard<std::mutex> lock(m_lock);
	pacing_trace().add('S', m_layers.empty() ? 1 : 0, (int64_t)perEye[0].hTexture);
	if (m_layers.empty()) {
		// The first layer is the application's (or SteamVR's own); does it come with depth (motion smoothing)?
		m_first_layers++;
		m_first_with_depth += perEye[0].hDepthTexture != 0;
		if (perEye[0].hTexture != m_last_new_texture) {
			m_new_app_frames++;
			m_last_new_texture = perEye[0].hTexture;
		}
	}
	if (m_layers.size() < 16) {
		m_layers.push_back({perEye[0], perEye[1]});
	}
}

void
OvrdDirectMode::Present(vr::SharedTextureHandle_t syncTexture)
{
	pacing_trace().add('P');
	{
		// How long SteamVR's compositor waited between starting this frame and presenting it: about nothing when
		// the game's frame was there already (it renders a frame ahead), the game's own time when it waits for the
		// frame it let the game start with (a game slower than a refresh, or after a hitch). For the hold.
		std::lock_guard<std::mutex> lock(m_lock);
		const int64_t start = m_compositor_start_ns;
		m_compositor_start_ns = 0;
		if (start > 0) {
			m_compositor_waits[m_compositor_wait_at] = os_monotonic_get_ns() - start;
			m_compositor_wait_at = (m_compositor_wait_at + 1) % kCompositorWaits;
			m_compositor_wait_max = std::max(m_compositor_wait_max, os_monotonic_get_ns() - start);
		}
	}
	// Test (settings.json display.motion_test_present_delay_ms): a game this much slower, for the frame lock.
	static const int test_delay_ms = getenv("RIFTLINUX_TEST_PRESENT_DELAY_MS") != nullptr
	                                     ? std::clamp(atoi(getenv("RIFTLINUX_TEST_PRESENT_DELAY_MS")), 0, 40)
	                                     : 0;
	if (test_delay_ms > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(test_delay_ms));
	}
	std::vector<std::array<SubmitLayerPerEye_t, 2>> layers;
	int64_t shown = 0;
	const int64_t present_ns = os_monotonic_get_ns();
	if (m_last_present_ns != 0) {
		const int64_t gap_ns = present_ns - m_last_present_ns;
		m_gap_max_ns = std::max(m_gap_max_ns, gap_ns);
		m_late += gap_ns > 16LL * U_TIME_1MS_IN_NS; // 1.5 frames at 90 Hz
	}
	m_last_present_ns = present_ns;
	{
		// The images SteamVR drew go back to Monado before the frame that shows them, as an OpenXR application
		// releases before it ends a frame; GetNextSwapTextureSetIndex acquires the next.
		std::lock_guard<std::mutex> lock(m_lock);
		layers.swap(m_layers);
		if (!layers.empty()) {
			const vr::SharedTextureHandle_t first = layers[0][0].hTexture;
			m_repeat_run = first == m_last_first_texture ? m_repeat_run + 1 : 0;
			m_repeat_max = std::max(m_repeat_max, m_repeat_run);
			m_last_first_texture = first;
		}
		for (const auto &set : m_sets) {
			if (set->acquired >= 0) {
				xrt_swapchain_release_image(set->xsc, (uint32_t)set->acquired);
				set->acquired = -1;
			}
		}
	}

	/*
	 * SteamVR gives no sign when its GPU has finished drawing the images (no sync texture on Linux, and they are
	 * opaque memory, no dma-buf fences): Monado read some half drawn, seen as bright glitches. A short
	 * wait bounds that, as ALVR's Linux driver does; display.direct_mode_guard_ms in settings.json.
	 */
#ifdef RLN_CAPTURE
	/*
	 * display.direct_mode_capture: that many frames, one every half second once the application's images come,
	 * each copied as SteamVR hands it over and again 10 ms later (are they finished?), into
	 * ~/.cache/riftlinux/dm-capture, with their means in the log.
	 */
	if (m_capture_left > 0 && !layers.empty()) {
		const int64_t t = os_monotonic_get_ns();
		std::lock_guard<std::mutex> lock(m_lock);
		auto it = m_handles.find(layers[0][0].hTexture);
		if (it != m_handles.end() && m_capture_next_ns != 0 && t >= m_capture_next_ns) {
			Set *set = it->second.first;
			const uint32_t index = it->second.second;
			std::vector<uint8_t> a, b;
			float ma[3] = {}, mb[3] = {};
			const bool got_a = g_capture.grab(set->xsc, index, set->desc.nWidth, set->desc.nHeight, a, ma);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			const bool got_b = g_capture.grab(set->xsc, index, set->desc.nWidth, set->desc.nHeight, b, mb);
			const int n = m_capture_count - m_capture_left;
			char name[64];
			if (got_a) {
				snprintf(name, sizeof(name), "frame%02d-%s-i%u-at-handover.rgba", n,
				         set->is_compositor ? "compositor" : "app", index);
				write_capture(a, set->desc.nWidth, set->desc.nHeight, name);
			}
			if (got_b) {
				snprintf(name, sizeof(name), "frame%02d-%s-i%u-10ms-later.rgba", n,
				         set->is_compositor ? "compositor" : "app", index);
				write_capture(b, set->desc.nWidth, set->desc.nHeight, name);
			}
			size_t differ = 0;
			for (size_t i = 0; got_a && got_b && i < a.size(); i++) {
				differ += a[i] != b[i];
			}
			ovrd_log("Direct mode: capture %d (%s image %u): handed over mean %.1f %.1f %.1f, 10 ms later %.1f %.1f "
			         "%.1f, %zu of %zu bytes changed%s\n",
			         n, set->is_compositor ? "compositor" : "app", index, ma[0], ma[1], ma[2], mb[0], mb[1], mb[2], differ, a.size(),
			         got_a && got_b ? "" : " (a copy failed)");
			m_capture_left--;
			m_capture_next_ns = t + 500 * U_TIME_1MS_IN_NS;
		} else if (it != m_handles.end() && m_capture_next_ns == 0) {
			m_capture_next_ns = t + (int64_t)(m_capture_delay_s * 1e9); // Once the application has run a while.
		}
	}
#endif

	if (m_guard_ms > 0) {
		std::this_thread::sleep_for(std::chrono::microseconds((int64_t)(m_guard_ms * 1000.0f)));
	}

	// Monado's frame pacing: this waits until it is time to hand over the next frame, which paces SteamVR's
	// compositor to the headset's refresh.
	int64_t frame_id = 0;
	int64_t display_time_ns = 0;
	int64_t display_period_ns = 0;
	struct xrt_compositor *xc = &m_xcn->base;
	// Throttled (SteamVR's frame lock): never early for the refresh Monado should show this frame at, 1 + that many
	// after the last one's. Monado gives a frame the first refresh it wakes for later than the call, so a frame
	// handed over before Monado woke for the refresh before that one would be shown a refresh early: wait past it.
	if (m_last_throttle > 0 && m_present_display_ns > 0 && m_wake_lead_ns > 0) {
		const int64_t period_ns = m_vsync_period_ns.load();
		const int64_t not_before = m_present_display_ns + (int64_t)m_last_throttle * period_ns - m_wake_lead_ns +
		                           300 * U_TIME_1US_IN_NS;
		const int64_t early = not_before - os_monotonic_get_ns();
		if (period_ns > 0 && early > 0 && early < 3 * period_ns) {
			std::this_thread::sleep_for(std::chrono::nanoseconds(early));
			m_guarded++;
		}
	}
	pacing_trace().add('W');
	const int64_t wait_from_ns = os_monotonic_get_ns();
	const xrt_result_t waited = xrt_comp_wait_frame(xc, &frame_id, &display_time_ns, &display_period_ns);
	const int64_t woken_ns = os_monotonic_get_ns();
	pacing_trace().add('w', frame_id, display_time_ns);
	// When Monado wakes a present before the refresh it shows it at (a wait that really waited), for the hold.
	if (waited == XRT_SUCCESS) {
		if (woken_ns - wait_from_ns > 300 * U_TIME_1US_IN_NS) {
			m_wake_lead_ns = display_time_ns - woken_ns;
		}
		// Shown later or earlier than the refresh its throttling asked for.
		if (m_present_display_ns > 0 && display_period_ns > 0) {
			const int64_t want = m_present_display_ns + (int64_t)(1 + m_last_throttle) * display_period_ns;
			m_shown_late += display_time_ns > want + display_period_ns / 2;
			m_shown_early += display_time_ns < want - display_period_ns / 2;
		}
		m_present_display_ns = display_time_ns;
	}
	if (waited != XRT_SUCCESS || xrt_comp_begin_frame(xc, frame_id) != XRT_SUCCESS) {
		ovrd_log("Direct mode: Monado's compositor refused a frame\n");
		return;
	}
	struct xrt_layer_frame_data frame = {};
	frame.frame_id = frame_id;
	frame.display_time_ns = display_time_ns;
	frame.env_blend_mode = XRT_BLEND_MODE_OPAQUE;
	xrt_comp_layer_begin(xc, &frame);

	// The eyes relative to the head, and their fields of view, as the headset has them (SteamVR renders with
	// them, from this driver's display component).
	const struct xrt_vec3 default_eye_relation = {0.063f, 0.0f, 0.0f};
	struct xrt_space_relation head_now = {};
	struct xrt_fov fovs[2] = {};
	struct xrt_pose eye_in_head[2] = {};
	xrt_device_get_view_poses(m_head, &default_eye_relation, display_time_ns, XRT_VIEW_TYPE_STEREO, 2, &head_now,
	                          fovs, eye_in_head);

	{
		std::lock_guard<std::mutex> lock(m_lock);
		// The layers above the first: from which set, and how often (flashes were seen).
		for (size_t l = 1; l < layers.size(); l++) {
			auto it = m_handles.find(layers[l][0].hTexture);
			if (it != m_handles.end()) {
				(it->second.first->is_compositor ? m_upper_from_compositor : m_upper_from_app)++;
			}
		}
		const size_t count = std::min(layers.size(), (size_t)std::max(1, m_max_layers));
		for (size_t l = 0; l < count; l++) {
			struct xrt_swapchain *xscs[2] = {nullptr, nullptr};
			struct xrt_layer_data data = {};
			data.type = XRT_LAYER_PROJECTION;
			data.name = XRT_INPUT_GENERIC_HEAD_POSE;
			data.timestamp = display_time_ns;
			// Layers above the first (SteamVR's overlays) blend over it.
			data.flags = l == 0 ? (enum xrt_layer_composition_flags)0
			                    : XRT_LAYER_COMPOSITION_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
			data.flip_y = false;
			data.color_scale = {1.0f, 1.0f, 1.0f, 1.0f};
			data.view_count = 2;
			bool ok = true;
			for (int eye = 0; eye < 2 && ok; eye++) {
				const SubmitLayerPerEye_t &in = layers[l][eye];
				auto it = m_handles.find(in.hTexture);
				if (it == m_handles.end() || it->second.first->xsc == nullptr) {
					ok = false;
					break;
				}
				Set *set = it->second.first;
				xscs[eye] = set->xsc;
				// The player's own HDR description of its images, if it gave one (set_hdr) and they are used.
				if (eye == 0 && m_players_hdr.load()) {
					auto described = m_pid_hdr.find(set->pid);
					if (described != m_pid_hdr.end()) {
						data.hdr = described->second;
						m_layers_described++;
					}
				}
				if (l == 0 && eye == 0) {
					(set->is_compositor ? m_from_compositor : m_from_app)++;
					m_first_layer_set[set->number]++;
				}
				struct xrt_layer_projection_view_data &v = data.proj.v[eye];
				const float w = (float)set->desc.nWidth;
				const float h = (float)set->desc.nHeight;
				v.sub.image_index = it->second.second;
				v.sub.array_index = 0;
				v.sub.norm_rect = {in.bounds.uMin, in.bounds.vMin, in.bounds.uMax - in.bounds.uMin,
				                   in.bounds.vMax - in.bounds.vMin};
				v.sub.rect.offset = {(int)(in.bounds.uMin * w), (int)(in.bounds.vMin * h)};
				v.sub.rect.extent = {(int)((in.bounds.uMax - in.bounds.uMin) * w),
				                     (int)((in.bounds.vMax - in.bounds.vMin) * h)};
				v.fov = fovs[eye];
				// Where the head was when SteamVR rendered this, in Monado's space, then the eye.
				struct xrt_pose head = pose_from_matrix(in.mHmdPose);
				m_driver_from_world(&head);
				math_pose_transform(&head, &eye_in_head[eye], &v.pose);
				// How far the pose SteamVR rendered with is from where Monado shows the frame: a timewarp
				// of more than a few degrees or centimetres means the spaces disagree.
				if (l == 0 && eye == 0 && (head_now.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) {
					const float dot = math_quat_dot(&head.orientation, &head_now.pose.orientation);
					const float angle = 2.0f * acosf(std::min(1.0f, fabsf(dot))) * 57.2958f;
					struct xrt_vec3 dp = m_vec3_sub(head.position, head_now.pose.position);
					const float dist = m_vec3_len(dp) * 100.0f;
					m_pose_deg_max = std::max(m_pose_deg_max, angle);
					m_pose_cm_max = std::max(m_pose_cm_max, dist);
					m_pose_deg_sum += angle;
					m_pose_n++;
				}
			}
			if (ok && xrt_comp_layer_projection(xc, m_head, xscs, &data) == XRT_SUCCESS) {
				shown++;
			}
		}
	}

#ifdef RLN_CAPTURE
	// The HDR test chart (display.hdr_test): a quad 1.2 m ahead, fixed to the view, described as HDR10 video.
	if (m_hdr_test > 0 && !m_hdr_test_tried) {
		m_hdr_test_tried = true;
		m_hdr_test_xsc = make_hdr_test_chart(m_xcn, m_hdr_test_w, m_hdr_test_h);
	}
	if (m_hdr_test_xsc != nullptr) {
		struct xrt_layer_data q = {};
		q.type = XRT_LAYER_QUAD;
		q.name = XRT_INPUT_GENERIC_HEAD_POSE;
		q.timestamp = display_time_ns;
		q.flags = XRT_LAYER_COMPOSITION_VIEW_SPACE_BIT;
		q.color_scale = {1.0f, 1.0f, 1.0f, 1.0f};
		q.quad.visibility = XRT_LAYER_EYE_VISIBILITY_BOTH;
		q.quad.sub.image_index = 0;
		q.quad.sub.rect.extent = {(int)m_hdr_test_w, (int)m_hdr_test_h};
		q.quad.sub.norm_rect = {0.0f, 0.0f, 1.0f, 1.0f};
		q.quad.pose.orientation.w = 1.0f;
		q.quad.pose.position = {0.0f, 0.0f, -1.3f};
		q.quad.size = {1.2f, 1.2f * (float)m_hdr_test_h / (float)m_hdr_test_w};
		q.hdr.transfer = XRT_LAYER_TRANSFER_PQ;
		q.hdr.primaries = XRT_LAYER_PRIMARIES_BT2020;
		q.hdr.reference_white_nits = 203.0f;
		if (m_hdr_test == 1 || m_hdr_test == 3) {
			q.hdr.mastering_max_nits = 4000.0f;
			q.hdr.mastering_min_nits = 0.005f;
			q.hdr.max_cll_nits = 4000.0f;
			q.hdr.max_fall_nits = 400.0f;
		}
		if (m_hdr_test == 3) {
			q.hdr.scene_max_nits = 1000.0f;
			q.hdr.scene_avg_nits = 100.0f;
		}
		xrt_comp_layer_quad(xc, m_head, m_hdr_test_xsc, &q);
	}
#endif

	xrt_comp_layer_commit(xc, XRT_GRAPHICS_SYNC_HANDLE_INVALID);
	pacing_trace().add('p', (int64_t)layers.size(), display_time_ns);

	// Vsync events follow Monado's display: this frame's refresh is on their grid (11 ms of vsync to photons).
	m_vsync_period_ns.store(display_period_ns);
	m_vsync_anchor_ns.store(display_time_ns - 11 * U_TIME_1MS_IN_NS);

	m_frames++;
	m_layers_total += (int64_t)layers.size();
	m_layers_shown += shown;
	const int64_t now_ns = os_monotonic_get_ns();
	if (now_ns - m_last_log_ns > 10LL * U_TIME_1S_IN_NS) {
		ovrd_log("Direct mode: %lld frames (first layer from the compositor %lld, the application %lld; layers "
		         "above it from the compositor %lld, the application %lld; at most %d shown), %lld "
		         "layers given, %lld shown, %lld index requests; sync texture %s; rendered vs shown head: mean "
		         "%.2f deg, max %.2f deg, max %.1f cm; guard %.1f ms\n",
		         (long long)m_frames, (long long)m_from_compositor, (long long)m_from_app,
		         (long long)m_upper_from_compositor, (long long)m_upper_from_app, m_max_layers,
		         (long long)m_layers_total, (long long)m_layers_shown, (long long)m_next_calls,
		         syncTexture != 0 ? "given" : "none", m_pose_n > 0 ? m_pose_deg_sum / m_pose_n : 0.0f,
		         m_pose_deg_max, m_pose_cm_max, m_guard_ms);
		std::string sets;
		for (const auto &kv : m_first_layer_set) {
			sets += " set" + std::to_string(kv.first) + "x" + std::to_string(kv.second);
		}
		ovrd_log("Direct mode: first layers by set:%s; late presents %lld (longest gap %.1f ms), longest run of one "
		         "image shown again %d frames\n",
		         sets.c_str(), (long long)m_late, (double)m_gap_max_ns / 1e6, m_repeat_max);
		{
			std::lock_guard<std::mutex> lock(m_lock);
			if (m_layers_described > 0 || !m_pid_hdr.empty()) {
				ovrd_log("Direct mode: %lld layers with a player's HDR description (%zu process(es) described)\n",
				         (long long)m_layers_described, m_pid_hdr.size());
			}
			m_layers_described = 0;
		}
		ovrd_log("Direct mode: vsync events %lld; frames SteamVR throttled by 0/1/2/3: %lld/%lld/%lld/%lld; held "
		         "%.1f ms on average, %lld not held; SteamVR's compositor waited up to %.1f ms for the game; frames "
		         "shown later than asked %lld, earlier %lld (%lld kept from early); Monado wakes a present %.1f ms "
		         "before its refresh\n",
		         (long long)m_vsync_events.exchange(0), (long long)m_throttle_frames[0], (long long)m_throttle_frames[1],
		         (long long)m_throttle_frames[2], (long long)m_throttle_frames[3],
		         m_lock_frames > 0 ? (double)m_lock_held_sum / (double)m_lock_frames / 10.0 : 0.0,
		         (long long)m_lock_missed, (double)m_compositor_wait_max / 1e6, (long long)m_shown_late,
		         (long long)m_shown_early, (long long)m_guarded, (double)m_wake_lead_ns / 1e6);
		m_lock_held_sum = m_lock_frames = m_lock_missed = 0;
		m_shown_late = m_shown_early = m_guarded = 0;
		m_compositor_wait_max = 0;
		m_throttle_frames[0] = m_throttle_frames[1] = m_throttle_frames[2] = m_throttle_frames[3] = 0;
		{
			std::lock_guard<std::mutex> lock(m_lock);
			ovrd_log("Direct mode: motion: new first-layer images %lld of %lld layers, with depth "
			         "%lld; reprojection flags 0x%x over %lld timing calls; throttled %lld of %lld presents (max %u, "
			         "predict max %u)\n",
			         (long long)m_new_app_frames, (long long)m_first_layers, (long long)m_first_with_depth,
			         m_reprojection_flags, (long long)m_timing_calls, (long long)m_throttled,
			         (long long)m_post_presents, m_throttle_max, m_predict_max);
			m_new_app_frames = m_first_layers = m_first_with_depth = m_timing_calls = m_throttled =
			    m_post_presents = 0;
			m_reprojection_flags = 0;
			m_throttle_max = m_predict_max = 0;
		}
		m_late = 0;
		m_gap_max_ns = 0;
		m_repeat_max = 0;
		m_first_layer_set.clear();
		m_pose_deg_max = m_pose_cm_max = m_pose_deg_sum = 0.0f;
		m_pose_n = 0;
		m_last_log_ns = now_ns;
	}
}


void
OvrdDirectMode::PostPresent(const Throttling_t *pThrottling)
{
	// Motion smoothing: how SteamVR throttles the application in direct mode.
	uint32_t throttle = 0;
	if (pThrottling != nullptr) {
		std::lock_guard<std::mutex> lock(m_lock);
		m_post_presents++;
		m_throttled += pThrottling->nFramesToThrottle > 0;
		m_throttle_max = std::max(m_throttle_max, pThrottling->nFramesToThrottle);
		m_predict_max = std::max(m_predict_max, pThrottling->nAdditionalFramesToPredict);
		throttle = std::min(pThrottling->nFramesToThrottle, 3u);
	}
	pacing_trace().add('Q', throttle, pThrottling != nullptr ? pThrottling->nAdditionalFramesToPredict : 0);
	/*
	 * SteamVR's throttling (half rate when the game cannot keep up and motion smoothing is on, or the
	 * per-application setting): Monado shows each frame for that many more refreshes, moved on by motion smoothing.
	 * What SteamVR does when this returns (pacing traces): it lets the game start its next frame, and
	 * its compositor starts the next frame too, presenting either the game frame it already has (the game renders a
	 * frame ahead: about a millisecond later) or, when it has none, waiting for the one it just let start. A game
	 * slower than a refresh ends up in the second way after one late frame and stays there; a hold timed for the
	 * first then made its frames miss every other refresh: 22.5 a second instead of 45 (No Man's Sky; the
	 * pace test at 15 ms).
	 * So held until the target refresh's wake-up (Monado's, 1 + that many refreshes after this frame's) less the
	 * longest of the compositor's last waits and a margin: a frame ahead the hold ends just before the wake-up,
	 * waiting for the game it ends early enough for the game's frame. Counted from Monado's frame, not from when
	 * SteamVR calls. Present keeps a frame from being early (if the game got faster).
	 */
	// Only a wait: a discarded frame would make Monado drop the frame it shows (comp_multi), not show it again.
	const int64_t period = m_vsync_period_ns.load();
	const int64_t display = m_present_display_ns;
	const int64_t wake_lead = m_wake_lead_ns;
	const int64_t now = os_monotonic_get_ns();
	int64_t until = 0;
	bool missed = false;
	m_last_throttle = throttle;
	if (throttle > 0 && period > 0 && display > 0 && wake_lead > 0) {
		int64_t wait_est = 0;
		{
			std::lock_guard<std::mutex> lock(m_lock);
			for (int64_t w : m_compositor_waits) {
				wait_est = std::max(wait_est, w);
			}
		}
		const int64_t span = (int64_t)(1 + throttle) * period;
		wait_est = std::clamp(wait_est, (int64_t)(600 * U_TIME_1US_IN_NS), span);
		until = display + span - wake_lead - wait_est - 2500 * U_TIME_1US_IN_NS;
		const int64_t wait = until - now;
		if (wait > 0 && wait < span + period) {
			std::this_thread::sleep_for(std::chrono::nanoseconds(wait));
			m_lock_held_sum += wait / (100 * U_TIME_1US_IN_NS);
			m_lock_frames++;
		} else {
			// The game needs all the time there is: let it start now.
			missed = true;
			m_lock_missed++;
		}
	}
	pacing_trace().add('q', missed ? 1 : 0, until);
	m_throttle_frames[throttle]++;
}

void
OvrdDirectMode::set_hdr(uint32_t pid, const struct xrt_layer_hdr_data *hdr)
{
	std::lock_guard<std::mutex> lock(m_lock);
	if (hdr == nullptr) {
		m_pid_hdr.erase(pid);
	} else {
		m_pid_hdr[pid] = *hdr;
	}
}

void
OvrdDirectMode::start_vsync_events()
{
	if (m_vsync_thread.joinable()) {
		return;
	}
	m_vsync_exit.store(false);
	m_vsync_thread = std::thread(&OvrdDirectMode::vsync_loop, this);
	ovrd_log("Direct mode: vsync events to SteamVR from Monado's display timing\n");
}

void
OvrdDirectMode::vsync_loop()
{
	// Declared this long before each refresh (running start), as Valve suggests for direct mode drivers.
	const int64_t lead_ns = 2 * U_TIME_1MS_IN_NS;
	int64_t last_tick_ns = 0;
	while (!m_vsync_exit.load()) {
		const int64_t period = m_vsync_period_ns.load();
		const int64_t anchor = m_vsync_anchor_ns.load();
		if (period <= 0 || anchor == 0) {
			// No frame yet: Monado's grid is not known.
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}
		const int64_t now = os_monotonic_get_ns();
		// The next tick on the grid whose announce point is still ahead, and never one already declared.
		int64_t k = (now + lead_ns - anchor) / period + 1;
		int64_t tick = anchor + k * period;
		while (tick <= last_tick_ns + period / 2) {
			tick += period;
		}
		const int64_t announce = tick - lead_ns;
		const int64_t wait = announce - os_monotonic_get_ns();
		if (wait > 0) {
			std::this_thread::sleep_for(std::chrono::nanoseconds(wait));
		}
		if (m_vsync_exit.load()) {
			break;
		}
		const double offset_s = (double)(announce - (int64_t)os_monotonic_get_ns()) / 1e9;
		vr::VRServerDriverHost()->VsyncEvent(offset_s);
		pacing_trace().add('V', 0, tick);
		last_tick_ns = tick;
		m_vsync_last_tick_ns.store(tick);
		m_vsync_events++;
	}
}

void
OvrdDirectMode::GetFrameTiming(vr::DriverDirectMode_FrameTiming *pFrameTiming)
{
	/*
	 * SteamVR sets m_nReprojectionFlags for the driver to respond to: Motion Smoothing on for the application in its
	 * settings (VRCompositor_ReprojectionMotion_Enabled, _ForcedOn), the application asking to be throttled
	 * (_AppThrottled). Recorded for motion smoothing; Monado reprojects every frame to the newest head
	 * pose itself.
	 */
	std::lock_guard<std::mutex> lock(m_lock);
	m_timing_calls++;
	pacing_trace().add('T', pFrameTiming->m_nReprojectionFlags, pFrameTiming->m_nNumFramePresents);
	m_reprojection_flags |= pFrameTiming->m_nReprojectionFlags;
	// Cleared again: these bits share the field with VRCompositor_ThrottleMask, what is left in it SteamVR reads as
	// frames to throttle the application by (openvr_driver.h); 0x100 would hold it back one frame.
	pFrameTiming->m_nReprojectionFlags = 0;
}
