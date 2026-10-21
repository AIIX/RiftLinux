// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  riftlinux-pace-test: a SteamVR scene application as slow as asked, for testing the frame lock and motion
 *         smoothing without a game: each frame it waits for its poses, works the CPU for that many
 *         milliseconds (as No Man's Sky, which needs 14-17 ms of CPU a frame), and shows a textured pattern moving
 *         sideways at a fixed speed over the top two thirds of each eye (the bottom third stands still), so made
 *         frames can be checked against where the pattern really was.
 *
 * Usage: riftlinux-pace-test <SteamVR's vrclient.so> [seconds, 60] [CPU ms a frame, 15] [pixels a second, 300]
 * The CPU time can swing like a game's between places: "16-30/20" goes from 16 to 30 ms and back every 20 s.
 * [drawn part, 1]: draws only that part of each eye (from the top left) and tells SteamVR those bounds, as SteamVR
 * Home does with its changing resolution; the rest flashes red and green each frame, so anything reading past the
 * drawn area shows it.
 * [yaw degrees, 0]: each frame handed over with the head pose turned by that much (as rendered for a head that has
 * turned since), so the compositor's reprojection has to reach past the drawn area on one side, as in a fast turn.
 * Logs once a second; each submitted frame's time and pattern offset go to ~/.cache/riftlinux/pace-test.txt.
 */

#include "openvr.h"
#include "IVRClientCore_003.h"

#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>

#include <dlfcn.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

double
now_s()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::vector<std::string>
split(const std::string &text)
{
	std::vector<std::string> out;
	size_t at = 0;
	while (at < text.size()) {
		const size_t end = std::min(text.find(' ', at), text.size());
		if (end > at) {
			out.push_back(text.substr(at, end - at));
		}
		at = end + 1;
	}
	return out;
}

//! RIFTLINUX_PACE_HDR=1: 16-bit float images in linear light, a quarter of the pattern's cells 4x brighter than white,
//! submitted as linear colour (as an HDR game would): shows what SteamVR hands a direct-mode driver for HDR content.
//! RIFTLINUX_PACE_HDR=2: the same float images with nothing past white (an SDR game drawing into float images).
static const int g_hdr_mode = getenv("RIFTLINUX_PACE_HDR") != nullptr ? atoi(getenv("RIFTLINUX_PACE_HDR")) : 0;
static const bool g_hdr = g_hdr_mode == 1 || g_hdr_mode == 2;

static uint16_t
to_half(float f)
{
	uint32_t x;
	memcpy(&x, &f, 4);
	const uint32_t sign = (x >> 16) & 0x8000u;
	int32_t e = (int32_t)((x >> 23) & 0xff) - 127 + 15;
	uint32_t m = x & 0x7fffffu;
	if (e <= 0) {
		return (uint16_t)sign;
	}
	if (e >= 31) {
		return (uint16_t)(sign | 0x7c00u);
	}
	return (uint16_t)(sign | ((uint32_t)e << 10) | (m >> 13));
}

static float
srgb_to_linear(float v)
{
	return v <= 0.04045f ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f);
}

//! The pattern's period across, in pixels: the source image is the eye's width plus this, and repeats every period.
constexpr int kPeriod = 256;

struct Vk
{
	PFN_vkGetInstanceProcAddr gipa = nullptr;
#define PT_VK_FUNCS(X)                                                                                                 \
	X(vkCreateInstance)                                                                                            \
	X(vkEnumeratePhysicalDevices)                                                                                  \
	X(vkGetPhysicalDeviceQueueFamilyProperties)                                                                    \
	X(vkGetPhysicalDeviceMemoryProperties)                                                                         \
	X(vkCreateDevice)                                                                                              \
	X(vkGetDeviceQueue)                                                                                            \
	X(vkCreateCommandPool)                                                                                         \
	X(vkAllocateCommandBuffers)                                                                                    \
	X(vkCreateFence)                                                                                               \
	X(vkWaitForFences)                                                                                             \
	X(vkResetFences)                                                                                               \
	X(vkCreateImage)                                                                                               \
	X(vkGetImageMemoryRequirements)                                                                                \
	X(vkAllocateMemory)                                                                                            \
	X(vkBindImageMemory)                                                                                           \
	X(vkCreateBuffer)                                                                                              \
	X(vkGetBufferMemoryRequirements)                                                                               \
	X(vkBindBufferMemory)                                                                                          \
	X(vkMapMemory)                                                                                                 \
	X(vkBeginCommandBuffer)                                                                                        \
	X(vkEndCommandBuffer)                                                                                          \
	X(vkResetCommandBuffer)                                                                                        \
	X(vkCmdPipelineBarrier)                                                                                        \
	X(vkCmdCopyBufferToImage)                                                                                      \
	X(vkCmdCopyImage)                                                                                              \
	X(vkCmdClearColorImage)                                                                                        \
	X(vkQueueSubmit)
#define PT_VK_DECL(name) PFN_##name name = nullptr;
	PT_VK_FUNCS(PT_VK_DECL)
#undef PT_VK_DECL

	VkInstance instance = VK_NULL_HANDLE;
	VkPhysicalDevice phys = VK_NULL_HANDLE;
	VkDevice device = VK_NULL_HANDLE;
	VkQueue queue = VK_NULL_HANDLE;
	uint32_t family = 0;
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	VkFence fence = VK_NULL_HANDLE;
	//! The pattern (eye width + one period across), and three images per eye in turn.
	VkImage pattern = VK_NULL_HANDLE;
	VkImage eyes[3][2] = {};
	bool used[3] = {};
	uint32_t w = 0, h = 0;

	int32_t
	memory_type(uint32_t bits, VkMemoryPropertyFlags want)
	{
		VkPhysicalDeviceMemoryProperties props;
		vkGetPhysicalDeviceMemoryProperties(phys, &props);
		for (uint32_t i = 0; i < props.memoryTypeCount; i++) {
			if ((bits & (1u << i)) != 0 && (props.memoryTypes[i].propertyFlags & want) == want) {
				return static_cast<int32_t>(i);
			}
		}
		return -1;
	}

	bool
	allocate(VkMemoryRequirements req, VkMemoryPropertyFlags want, VkDeviceMemory *out)
	{
		const int32_t type = memory_type(req.memoryTypeBits, want);
		if (type < 0) {
			return false;
		}
		VkMemoryAllocateInfo info = {};
		info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		info.allocationSize = req.size;
		info.memoryTypeIndex = static_cast<uint32_t>(type);
		return vkAllocateMemory(device, &info, nullptr, out) == VK_SUCCESS;
	}

	bool
	image(uint32_t iw, uint32_t ih, VkImage *out)
	{
		VkImageCreateInfo ii = {};
		ii.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		ii.imageType = VK_IMAGE_TYPE_2D;
		ii.format = g_hdr ? VK_FORMAT_R16G16B16A16_SFLOAT : VK_FORMAT_R8G8B8A8_SRGB;
		ii.extent = {iw, ih, 1};
		ii.mipLevels = 1;
		ii.arrayLayers = 1;
		ii.samples = VK_SAMPLE_COUNT_1_BIT;
		ii.tiling = VK_IMAGE_TILING_OPTIMAL;
		ii.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
		ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		VkMemoryRequirements req;
		VkDeviceMemory mem = VK_NULL_HANDLE;
		return vkCreateImage(device, &ii, nullptr, out) == VK_SUCCESS &&
		       (vkGetImageMemoryRequirements(device, *out, &req), allocate(req, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &mem)) &&
		       vkBindImageMemory(device, *out, mem, 0) == VK_SUCCESS;
	}

	void
	barrier(VkImage img, VkImageLayout from, VkImageLayout to, VkAccessFlags src, VkAccessFlags dst)
	{
		VkImageMemoryBarrier b = {};
		b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		b.srcAccessMask = src;
		b.dstAccessMask = dst;
		b.oldLayout = from;
		b.newLayout = to;
		b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		b.image = img;
		b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr,
		                     0, nullptr, 1, &b);
	}

	bool
	run_and_wait()
	{
		vkEndCommandBuffer(cmd);
		VkSubmitInfo submit = {};
		submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		submit.commandBufferCount = 1;
		submit.pCommandBuffers = &cmd;
		vkResetFences(device, 1, &fence);
		return vkQueueSubmit(queue, 1, &submit, fence) == VK_SUCCESS &&
		       vkWaitForFences(device, 1, &fence, VK_TRUE, 1000000000ull) == VK_SUCCESS;
	}

	void
	begin()
	{
		vkResetCommandBuffer(cmd, 0);
		VkCommandBufferBeginInfo bi = {};
		bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		vkBeginCommandBuffer(cmd, &bi);
	}

	bool
	init(vr::IVRSystem *system, vr::IVRCompositor *compositor, uint32_t width, uint32_t height)
	{
		w = width;
		h = height;
		void *lib = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
		gipa = lib != nullptr ? reinterpret_cast<PFN_vkGetInstanceProcAddr>(dlsym(lib, "vkGetInstanceProcAddr"))
		                      : nullptr;
		if (gipa == nullptr) {
			return fail("no libvulkan.so.1");
		}
		vkCreateInstance = reinterpret_cast<PFN_vkCreateInstance>(gipa(VK_NULL_HANDLE, "vkCreateInstance"));
		std::string text(compositor->GetVulkanInstanceExtensionsRequired(nullptr, 0) + 1, '\0');
		compositor->GetVulkanInstanceExtensionsRequired(text.data(), static_cast<uint32_t>(text.size()));
		std::vector<std::string> names = split(text.c_str());
		std::vector<const char *> exts;
		for (const std::string &n : names) {
			exts.push_back(n.c_str());
		}
		VkApplicationInfo app = {};
		app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		app.pApplicationName = "riftlinux-pace-test";
		app.apiVersion = VK_API_VERSION_1_1;
		VkInstanceCreateInfo ici = {};
		ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		ici.pApplicationInfo = &app;
		ici.enabledExtensionCount = static_cast<uint32_t>(exts.size());
		ici.ppEnabledExtensionNames = exts.data();
		if (vkCreateInstance == nullptr || vkCreateInstance(&ici, nullptr, &instance) != VK_SUCCESS) {
			return fail("vkCreateInstance");
		}
#define PT_VK_LOAD(name) name = reinterpret_cast<PFN_##name>(gipa(instance, #name));
		PT_VK_FUNCS(PT_VK_LOAD)
#undef PT_VK_LOAD
		uint64_t out_device = 0;
		system->GetOutputDevice(&out_device, vr::TextureType_Vulkan, instance);
		phys = reinterpret_cast<VkPhysicalDevice>(out_device);
		if (phys == VK_NULL_HANDLE) {
			uint32_t count = 1;
			vkEnumeratePhysicalDevices(instance, &count, &phys);
		}
		if (phys == VK_NULL_HANDLE) {
			return fail("no GPU");
		}
		uint32_t nfam = 0;
		vkGetPhysicalDeviceQueueFamilyProperties(phys, &nfam, nullptr);
		std::vector<VkQueueFamilyProperties> fams(nfam);
		vkGetPhysicalDeviceQueueFamilyProperties(phys, &nfam, fams.data());
		family = UINT32_MAX;
		for (uint32_t i = 0; i < nfam && family == UINT32_MAX; i++) {
			if ((fams[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
				family = i;
			}
		}
		if (family == UINT32_MAX) {
			return fail("no graphics queue");
		}
		std::string dtext(compositor->GetVulkanDeviceExtensionsRequired(phys, nullptr, 0) + 1, '\0');
		compositor->GetVulkanDeviceExtensionsRequired(phys, dtext.data(), static_cast<uint32_t>(dtext.size()));
		std::vector<std::string> dnames = split(dtext.c_str());
		std::vector<const char *> dexts;
		for (const std::string &n : dnames) {
			dexts.push_back(n.c_str());
		}
		const float priority = 1.0f;
		VkDeviceQueueCreateInfo qci = {};
		qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		qci.queueFamilyIndex = family;
		qci.queueCount = 1;
		qci.pQueuePriorities = &priority;
		VkDeviceCreateInfo dci = {};
		dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
		dci.queueCreateInfoCount = 1;
		dci.pQueueCreateInfos = &qci;
		dci.enabledExtensionCount = static_cast<uint32_t>(dexts.size());
		dci.ppEnabledExtensionNames = dexts.data();
		if (vkCreateDevice(phys, &dci, nullptr, &device) != VK_SUCCESS) {
			return fail("vkCreateDevice");
		}
		vkGetDeviceQueue(device, family, 0, &queue);
		VkCommandPoolCreateInfo pci = {};
		pci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
		pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
		pci.queueFamilyIndex = family;
		VkCommandPool pool = VK_NULL_HANDLE;
		VkCommandBufferAllocateInfo cai = {};
		cai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		cai.commandBufferCount = 1;
		VkFenceCreateInfo fci = {};
		fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		if (vkCreateCommandPool(device, &pci, nullptr, &pool) != VK_SUCCESS ||
		    (cai.commandPool = pool, vkAllocateCommandBuffers(device, &cai, &cmd) != VK_SUCCESS) ||
		    vkCreateFence(device, &fci, nullptr, &fence) != VK_SUCCESS) {
			return fail("command buffer");
		}
		if (!image(w + kPeriod, h, &pattern)) {
			return fail("pattern image");
		}
		for (auto &set : eyes) {
			for (VkImage &img : set) {
				if (!image(w, h, &img)) {
					return fail("eye image");
				}
			}
		}
		return upload_pattern();
	}

	/*!
	 * Cells of 32 x 32 pixels with a grey from a hash of the cell (repeating across every period), thin dark lines
	 * between them: texture for the optical flow in both directions, edges to see judder on.
	 */
	bool
	upload_pattern()
	{
		const uint32_t pw = w + kPeriod;
		VkBufferCreateInfo bi = {};
		bi.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bi.size = static_cast<VkDeviceSize>(pw) * h * (g_hdr ? 8 : 4);
		bi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		VkBuffer staging = VK_NULL_HANDLE;
		VkMemoryRequirements req;
		VkDeviceMemory mem = VK_NULL_HANDLE;
		void *mapped = nullptr;
		if (vkCreateBuffer(device, &bi, nullptr, &staging) != VK_SUCCESS ||
		    (vkGetBufferMemoryRequirements(device, staging, &req),
		     !allocate(req, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &mem)) ||
		    vkBindBufferMemory(device, staging, mem, 0) != VK_SUCCESS ||
		    vkMapMemory(device, mem, 0, bi.size, 0, &mapped) != VK_SUCCESS) {
			return fail("staging buffer");
		}
		auto *px = static_cast<uint8_t *>(mapped);
		for (uint32_t y = 0; y < h; y++) {
			for (uint32_t x = 0; x < pw; x++) {
				const uint32_t cx = (x % kPeriod) / 32, cy = y / 32;
				uint32_t hsh = (cx * 73856093u) ^ (cy * 19349663u);
				hsh ^= hsh >> 13;
				hsh *= 0x5bd1e995u;
				hsh ^= hsh >> 15;
				const bool line = (x % 32) < 2 || (y % 32) < 2;
				const uint8_t v = line ? 20 : static_cast<uint8_t>(60 + hsh % 180);
				const uint8_t g = static_cast<uint8_t>(line ? 20 : 60 + (hsh >> 8) % 180);
				const uint8_t b = static_cast<uint8_t>(line ? 20 : 60 + (hsh >> 16) % 180);
				if (g_hdr) {
					const float gain = (g_hdr_mode == 1 && !line && (hsh >> 24) % 4 == 0) ? 4.0f : 1.0f;
					uint16_t *q = reinterpret_cast<uint16_t *>(px) + (static_cast<size_t>(y) * pw + x) * 4;
					q[0] = to_half(srgb_to_linear(v / 255.0f) * gain);
					q[1] = to_half(srgb_to_linear(g / 255.0f) * gain);
					q[2] = to_half(srgb_to_linear(b / 255.0f) * gain);
					q[3] = to_half(1.0f);
					continue;
				}
				uint8_t *p = px + (static_cast<size_t>(y) * pw + x) * 4;
				p[0] = v;
				p[1] = g;
				p[2] = b;
				p[3] = 255;
			}
		}
		begin();
		barrier(pattern, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0,
		        VK_ACCESS_TRANSFER_WRITE_BIT);
		VkBufferImageCopy copy = {};
		copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		copy.imageExtent = {pw, h, 1};
		vkCmdCopyBufferToImage(cmd, staging, pattern, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
		barrier(pattern, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
		        VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
		return run_and_wait() || fail("pattern upload");
	}

	//! Set @p set's eye images: the top two thirds from the pattern @p offset pixels on, the bottom third still; only
	//! @p dw x @p dh of each drawn, the rest a flashing colour (@p flash).
	bool
	draw(int set, int offset, uint32_t dw, uint32_t dh, bool flash)
	{
		begin();
		for (VkImage img : eyes[set]) {
			barrier(img, used[set] ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED,
			        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);
			if (dw < w || dh < h) {
				VkClearColorValue cc = {};
				cc.float32[0] = flash ? 1.0f : 0.0f;
				cc.float32[1] = flash ? 0.0f : 1.0f;
				cc.float32[3] = 1.0f;
				VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
				vkCmdClearColorImage(cmd, img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &cc, 1, &range);
				barrier(img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
				        VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);
			}
			VkImageCopy regions[2] = {};
			const uint32_t split_y = dh * 2 / 3;
			for (VkImageCopy &r : regions) {
				r.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
				r.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			}
			regions[0].srcOffset = {offset % kPeriod, 0, 0};
			regions[0].extent = {dw, split_y, 1};
			regions[1].srcOffset = {0, static_cast<int32_t>(split_y), 0};
			regions[1].dstOffset = {0, static_cast<int32_t>(split_y), 0};
			regions[1].extent = {dw, dh - split_y, 1};
			vkCmdCopyImage(cmd, pattern, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, img,
			               VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 2, regions);
			barrier(img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
			        VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
		}
		used[set] = true;
		return run_and_wait();
	}

	bool
	fail(const char *what)
	{
		fprintf(stderr, "riftlinux-pace-test: %s\n", what);
		return false;
	}
};

} // namespace

int
main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "Usage: riftlinux-pace-test <vrclient.so> [seconds] [CPU ms a frame] [pixels a second]\n");
		return 2;
	}
	const double run_s = argc > 2 ? atof(argv[2]) : 60.0;
	double cpu_lo = argc > 3 ? atof(argv[3]) : 15.0, cpu_hi = cpu_lo, cpu_period_s = 0.0;
	if (argc > 3 && strchr(argv[3], '-') != nullptr) {
		sscanf(argv[3], "%lf-%lf/%lf", &cpu_lo, &cpu_hi, &cpu_period_s);
	}
	const double cpu_ms = cpu_lo;
	const double speed = argc > 4 ? atof(argv[4]) : 300.0;
	const double drawn = argc > 5 ? std::clamp(atof(argv[5]), 0.2, 1.0) : 1.0;
	const double yaw_deg = argc > 6 ? std::clamp(atof(argv[6]), -30.0, 30.0) : 0.0;

	void *lib = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
	if (lib == nullptr) {
		fprintf(stderr, "riftlinux-pace-test: %s\n", dlerror());
		return 1;
	}
	using Factory = void *(*)(const char *, int *);
	auto factory = reinterpret_cast<Factory>(dlsym(lib, "VRClientCoreFactory"));
	int rc = 0;
	auto *core = factory != nullptr
	                 ? static_cast<vr::IVRClientCore_003 *>(factory(vr::IVRClientCore_003_Version, &rc))
	                 : nullptr;
	if (core == nullptr) {
		fprintf(stderr, "riftlinux-pace-test: no IVRClientCore_003 in %s\n", argv[1]);
		return 1;
	}
	vr::EVRInitError err = core->Init(vr::VRApplication_Scene, nullptr);
	if (err != vr::VRInitError_None) {
		fprintf(stderr, "riftlinux-pace-test: SteamVR said %s\n", core->GetIDForVRInitError(err));
		return 1;
	}
	auto *system = static_cast<vr::IVRSystem *>(core->GetGenericInterface(vr::IVRSystem_Version, &err));
	auto *compositor = static_cast<vr::IVRCompositor *>(core->GetGenericInterface(vr::IVRCompositor_Version, &err));
	if (system == nullptr || compositor == nullptr) {
		fprintf(stderr, "riftlinux-pace-test: no system or compositor interface\n");
		core->Cleanup();
		return 1;
	}
	uint32_t w = 0, h = 0;
	system->GetRecommendedRenderTargetSize(&w, &h);
	Vk vk;
	if (w == 0 || h == 0 || !vk.init(system, compositor, w, h)) {
		core->Cleanup();
		return 1;
	}
	fprintf(stderr, "riftlinux-pace-test: %ux%u an eye, %.1f ms of CPU a frame, pattern %.0f pixels a second, %.0f s, %s\n",
	        w, h, cpu_ms, speed, run_s, g_hdr_mode == 1 ? "16-bit float linear (HDR)" : g_hdr ? "16-bit float linear, nothing past white" : "8-bit sRGB");

	const char *home = getenv("HOME");
	const std::string dir = std::string(home != nullptr ? home : "/tmp") + "/.cache/riftlinux";
	mkdir(dir.c_str(), 0755);
	FILE *log = fopen((dir + "/pace-test.txt").c_str(), "w");
	if (log != nullptr) {
		fprintf(log, "# frame steady_s offset_px (riftlinux-pace-test %.1f ms CPU, %.0f px/s, %ux%u, %s)\n", cpu_ms,
		        speed, w, h, g_hdr_mode == 1 ? "hdr" : g_hdr ? "float-sdr" : "sdr");
	}

	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
	const double t0 = now_s();
	double logged = t0;
	int frames = 0, frames_logged = 0, set = 0;
	long frame = 0;
	while (now_s() - t0 < run_s) {
		vr::VREvent_t event;
		bool quit = false;
		while (system->PollNextEvent(&event, sizeof(event))) {
			quit |= event.eventType == vr::VREvent_Quit;
		}
		if (quit) {
			break;
		}
		if (compositor->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0) != vr::VRCompositorError_None) {
			usleep(10000);
			continue;
		}
		const double posed = now_s();
		// The game's CPU work for this frame (swinging between the two values if asked).
		double work_ms = cpu_lo;
		if (cpu_period_s > 0.0) {
			const double phase = std::fmod(posed - t0, cpu_period_s) / cpu_period_s;
			work_ms = cpu_lo + (cpu_hi - cpu_lo) * 0.5 * (1.0 - std::cos(2.0 * M_PI * phase));
		}
		while ((now_s() - posed) * 1000.0 < work_ms) {
		}
		// Where the pattern is at this frame's poses: a steady speed in time.
		const int offset = static_cast<int>(std::lround((posed - t0) * speed));
		const uint32_t dw = (uint32_t)(w * drawn), dh = (uint32_t)(h * drawn);
		if (!vk.draw(set, offset, dw, dh, (frame & 1) != 0)) {
			break;
		}
		for (int eye = 0; eye < 2; eye++) {
			vr::VRVulkanTextureData_t data = {};
			data.m_nImage = reinterpret_cast<uint64_t>(vk.eyes[set][eye]);
			data.m_pDevice = vk.device;
			data.m_pPhysicalDevice = vk.phys;
			data.m_pInstance = vk.instance;
			data.m_pQueue = vk.queue;
			data.m_nQueueFamilyIndex = vk.family;
			data.m_nWidth = w;
			data.m_nHeight = h;
			data.m_nFormat = g_hdr ? VK_FORMAT_R16G16B16A16_SFLOAT : VK_FORMAT_R8G8B8A8_SRGB;
			data.m_nSampleCount = 1;
			vr::VRTextureBounds_t bounds = {0.0f, 0.0f, (float)drawn, (float)drawn};
			if (yaw_deg != 0.0) {
				// The head's pose turned about its own up axis: m = pose * R_y(yaw).
				vr::VRVulkanTextureData_t *d = &data;
				vr::VRTextureWithPose_t tp = {};
				tp.handle = d;
				tp.eType = vr::TextureType_Vulkan;
				tp.eColorSpace = g_hdr ? vr::ColorSpace_Linear : vr::ColorSpace_Auto;
				const vr::HmdMatrix34_t &m = poses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
				const double a = yaw_deg * M_PI / 180.0, c = std::cos(a), sn = std::sin(a);
				for (int r = 0; r < 3; r++) {
					tp.mDeviceToAbsoluteTracking.m[r][0] = (float)(m.m[r][0] * c - m.m[r][2] * sn);
					tp.mDeviceToAbsoluteTracking.m[r][1] = m.m[r][1];
					tp.mDeviceToAbsoluteTracking.m[r][2] = (float)(m.m[r][0] * sn + m.m[r][2] * c);
					tp.mDeviceToAbsoluteTracking.m[r][3] = m.m[r][3];
				}
				compositor->Submit(eye == 0 ? vr::Eye_Left : vr::Eye_Right, &tp, drawn < 1.0 ? &bounds : nullptr,
				                   vr::Submit_TextureWithPose);
			} else {
				vr::Texture_t texture = {&data, vr::TextureType_Vulkan, g_hdr ? vr::ColorSpace_Linear : vr::ColorSpace_Auto};
				compositor->Submit(eye == 0 ? vr::Eye_Left : vr::Eye_Right, &texture, drawn < 1.0 ? &bounds : nullptr);
			}
		}
		if (log != nullptr) {
			fprintf(log, "%ld %.6f %d\n", frame, posed, offset);
		}
		frame++;
		set = (set + 1) % 3;
		frames++;
		const double t = now_s();
		if (t - logged >= 1.0) {
			fprintf(stderr, "riftlinux-pace-test: %.1f frames a second\n", (frames - frames_logged) / (t - logged));
			frames_logged = frames;
			logged = t;
		}
	}
	if (log != nullptr) {
		fclose(log);
	}
	core->Cleanup();
	return 0;
}
