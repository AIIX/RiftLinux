// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  riftlinux-openxr-hdr-test: a native OpenXR application showing an HDR pattern on a quad layer, described
 *         or not with XR_MNDX_composition_layer_hdr (RiftLinux), to test the extension and the HDR
 *         mapping with our own runtime: a 16-bit float image of coloured cells in linear light, every fourth cell
 *         four times brighter than white, 1.2 m ahead, fixed to the view (as the driver's HDR test chart).
 *
 * Usage: riftlinux-openxr-hdr-test <libopenxr_monado.so> [seconds, 60] [mode, "none"]
 * The runtime library is loaded directly (no OpenXR loader needed); it connects to a running monado-service.
 * Modes:
 *   none       float image, no description: the compositor takes it as linear HDR and measures its peak
 *   linear     described as linear light, BT.709, no metadata (the same, through the extension)
 *   scene:N    linear, with a scene peak of N nits each frame (HDR10+ / Dolby Vision L1 style)
 *   cll:N      linear, with MaxCLL N nits (HDR10 static)
 *   pq:N       the same cells encoded as PQ (SMPTE ST 2084), BT.709, MaxCLL N nits
 *   sdr        described as SDR: the float image clipped at white, as before HDR
 *   probe      an instance and its system only, no session, held for the seconds given: as Proton's OpenXR check
 *              does (run as d3ddriverquery64.exe to stand in for Steam's graphics driver check)
 *   glass:N    the pattern (as none) with a translucent panel in front of it, rounded corners, whose backdrop is
 *              blurred by N milliradians (XR_MNDX_composition_layer_backdrop_blur); glass:0 the same two layers
 *              without the blur, for comparing
 *   mask       a session only (XR_KHR_visibility_mask): prints the runtime's hidden, visible and line-loop masks of
 *              both views, then quits without drawing
 *   swapchains:N  a session that makes N swapchains as a desktop shell does for its windows (12 alive, random window
 *              sizes, one replaced at random each step, each used once), printing its memory (VmRSS) every 50, then
 *              quits without drawing: for the client library's memory per swapchain made
 *   churn:N    the pattern shown while N swapchains come and go as a desktop shell's windows do (a new one every 4
 *              frames between xrBeginFrame and xrEndFrame, RGBA8 sRGB and RGBA16F alternately, window sizes, 12 alive,
 *              one destroyed at random after xrBeginFrame, every live image acquired, waited and released each frame),
 *              printing VmRSS and the heap in use (mallinfo2) every 50, then quits
 *   shell:N    as a VR desktop shell uses swapchains, until N are made: windows of 10 swapchains (quads,
 *              some cylinders, every layer with XR_MNDX_composition_layer_hdr, glass ones with a backdrop blur, a fading
 *              one with a colour scale), opened and closed inside frames, a closed window's swapchains destroyed in a
 *              burst while they were in the previous xrEndFrame's layers, a size change replacing one in the same
 *              frame, most images used once, video layers every frame, a room layer, and a layer per eye drawn every
 *              frame with a second fenced submit; the application's own clears go through one fenced submit and a wait
 *              a frame on the one queue, as a shell's copies do. VmRSS and the heap in use every 50 made.
 *              shell:N:nolayers keeps the windows out of the layers, shell:N:nocopy leaves the clears out,
 *              shell:N:hold makes N and then goes on without making more until the seconds given (heap every 10 s)
 *   rlof:X:Y:F a 1024x768 crop at X, Y of an output-capture style image F ("RLOF", width, height, RGBA half floats),
 *              shown as PQ BT.2020 video with MaxCLL 1000, MaxFALL 400, mastering 0.005-1000 nits (as a VR desktop
 *              describes Firefox's HDR video): the same frame through the runtime, for comparing builds
 * Logs to stdout once a second.
 */

#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>

#define XR_NO_PROTOTYPES
#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_loader_negotiation.h>
#include <openxr/XR_MNDX_composition_layer_hdr.h>
#include <openxr/XR_MNDX_composition_layer_backdrop_blur.h>

#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kWidth = 1024;
constexpr uint32_t kHeight = 768;
constexpr uint32_t kCell = 64;
constexpr float kWhiteNits = 203.0f;

double
now_s()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

//! An OpenXR structure of type @p type, all else zero.
template <typename T>
T
xr_struct(XrStructureType type)
{
	T v = {};
	v.type = type;
	return v;
}

bool
fail(const char *what)
{
	fprintf(stderr, "riftlinux-openxr-hdr-test: %s failed\n", what);
	return false;
}

//! IEEE half from float (round to nearest, no NaN handling needed for this pattern).
uint16_t
half(float f)
{
	uint32_t x;
	memcpy(&x, &f, 4);
	const uint32_t sign = (x >> 16) & 0x8000u;
	int32_t exp = (int32_t)((x >> 23) & 0xff) - 127 + 15;
	uint32_t mant = x & 0x7fffffu;
	if (exp <= 0) {
		return (uint16_t)sign;
	}
	if (exp >= 31) {
		return (uint16_t)(sign | 0x7bffu);
	}
	mant += 0x1000u; // round
	if (mant & 0x800000u) {
		mant = 0;
		exp++;
		if (exp >= 31) {
			return (uint16_t)(sign | 0x7bffu);
		}
	}
	return (uint16_t)(sign | ((uint32_t)exp << 10) | (mant >> 13));
}

//! SMPTE ST 2084 encoding of nits.
float
pq_encode(float nits)
{
	const float m1 = 2610.0f / 16384.0f, m2 = 2523.0f / 4096.0f * 128.0f;
	const float c1 = 3424.0f / 4096.0f, c2 = 2413.0f / 4096.0f * 32.0f, c3 = 2392.0f / 4096.0f * 32.0f;
	const float y = powf(fmaxf(nits, 0.0f) / 10000.0f, m1);
	return powf((c1 + c2 * y) / (1.0f + c3 * y), m2);
}

struct Mode
{
	std::string name = "none";
	bool describe = false;
	XrHdrTransferMNDX transfer = XR_HDR_TRANSFER_LINEAR_MNDX;
	float scene_nits = 0.0f;
	float cll_nits = 0.0f;
	//! glass:N: the panel's blur in milliradians; below 0: no panel.
	float glass_mrad = -1.0f;
	//! probe: an instance only.
	bool probe = false;
	//! mask: the visibility masks printed.
	bool mask = false;
	//! swapchains:N: swapchains made and destroyed, memory printed.
	int swapchains = 0;
	//! churn:N: swapchains coming and going while frames run.
	int churn = 0;
	//! shell:N: swapchains used as a VR desktop shell uses them; its windows in the layers, its own clears.
	int shell = 0;
	bool shell_layers = true;
	bool shell_copy = true;
	//! shell:N:hold: after N made, frames go on as they are until the seconds given (memory over time).
	bool shell_hold = false;
	//! rlof:X:Y:F: the image shown instead of the pattern, its crop, and BT.2020 video's description.
	std::string image;
	uint32_t crop_x = 0, crop_y = 0;
	bool video = false;
};

bool
parse_mode(const char *text, Mode &m)
{
	const std::string t = text;
	const size_t colon = t.find(':');
	const std::string kind = t.substr(0, colon);
	const float value = colon == std::string::npos ? 0.0f : (float)atof(t.c_str() + colon + 1);
	m.name = t;
	if (kind == "none") {
		return true;
	}
	m.describe = true;
	if (kind == "linear") {
		return true;
	}
	if (kind == "scene" && value > 0.0f) {
		m.scene_nits = value;
		return true;
	}
	if (kind == "cll" && value > 0.0f) {
		m.cll_nits = value;
		return true;
	}
	if (kind == "pq" && value > 0.0f) {
		m.transfer = XR_HDR_TRANSFER_PQ_MNDX;
		m.cll_nits = value;
		return true;
	}
	if (kind == "sdr") {
		m.transfer = XR_HDR_TRANSFER_SDR_MNDX;
		return true;
	}
	if (kind == "probe") {
		m.describe = false;
		m.probe = true;
		return true;
	}
	if (kind == "rlof") {
		const size_t c2 = t.find(':', colon + 1);
		const size_t c3 = c2 == std::string::npos ? std::string::npos : t.find(':', c2 + 1);
		if (c3 == std::string::npos) {
			return false;
		}
		m.crop_x = (uint32_t)atoi(t.c_str() + colon + 1);
		m.crop_y = (uint32_t)atoi(t.c_str() + c2 + 1);
		m.image = t.substr(c3 + 1);
		m.transfer = XR_HDR_TRANSFER_PQ_MNDX;
		m.cll_nits = 1000.0f;
		m.video = true;
		return true;
	}
	if (kind == "shell" && value >= 1.0f) {
		m.describe = false;
		m.shell = (int)value;
		m.shell_layers = t.find(":nolayers") == std::string::npos;
		m.shell_copy = t.find(":nocopy") == std::string::npos;
		m.shell_hold = t.find(":hold") != std::string::npos;
		return true;
	}
	if (kind == "churn" && value >= 1.0f) {
		m.describe = false;
		m.churn = (int)value;
		return true;
	}
	if (kind == "swapchains" && value >= 1.0f) {
		m.describe = false;
		m.swapchains = (int)value;
		return true;
	}
	if (kind == "mask") {
		m.describe = false;
		m.mask = true;
		return true;
	}
	if (kind == "glass" && value >= 0.0f) {
		m.describe = false;
		m.glass_mrad = value;
		return true;
	}
	return false;
}

//! glass:N's panel: a light grey-blue at alpha 0.35 (unpremultiplied), rounded corners, alpha 0 around them.
std::vector<uint16_t>
make_panel()
{
	std::vector<uint16_t> px((size_t)kWidth * kHeight * 4);
	const float r = 96.0f;
	for (uint32_t y = 0; y < kHeight; y++) {
		for (uint32_t x = 0; x < kWidth; x++) {
			// Distance outside the rounded rectangle (negative inside), for a two-pixel soft edge.
			const float cx = std::fmax(std::fabs((float)x + 0.5f - kWidth / 2.0f) - (kWidth / 2.0f - r), 0.0f);
			const float cy = std::fmax(std::fabs((float)y + 0.5f - kHeight / 2.0f) - (kHeight / 2.0f - r), 0.0f);
			const float d = std::sqrt(cx * cx + cy * cy) - r;
			const float a = 0.35f * std::fmin(std::fmax(0.5f - d / 2.0f, 0.0f), 1.0f);
			uint16_t *o = &px[((size_t)y * kWidth + x) * 4];
			o[0] = half(0.85f);
			o[1] = half(0.9f);
			o[2] = half(1.0f);
			o[3] = half(a);
		}
	}
	return px;
}

//! The pattern as RGBA half floats: cells in pseudo-random colours (linear light, white 1.0), every fourth four
//! times as bright; PQ-encoded for PQ mode.
std::vector<uint16_t>
make_pattern(const Mode &mode)
{
	std::vector<uint16_t> px((size_t)kWidth * kHeight * 4);
	if (!mode.image.empty()) {
		// rlof:X:Y:F: the crop, its half floats as they are (PQ values).
		FILE *f = fopen(mode.image.c_str(), "rb");
		char magic[4] = {};
		uint32_t wh[2] = {};
		const bool ok = f != nullptr && fread(magic, 1, 4, f) == 4 && memcmp(magic, "RLOF", 4) == 0 &&
		                fread(wh, 4, 2, f) == 2 && mode.crop_x + kWidth <= wh[0] && mode.crop_y + kHeight <= wh[1];
		for (uint32_t y = 0; ok && y < kHeight; y++) {
			fseek(f, 12 + (((long)(mode.crop_y + y) * wh[0] + mode.crop_x) * 8), SEEK_SET);
			if (fread(&px[(size_t)y * kWidth * 4], 8, kWidth, f) != kWidth) {
				break;
			}
		}
		if (f != nullptr) {
			fclose(f);
		}
		printf("riftlinux-openxr-hdr-test: image %s (%ux%u), crop at %u %u: %s\n", mode.image.c_str(), wh[0], wh[1],
		       mode.crop_x, mode.crop_y, ok ? "loaded" : "NOT loaded (pattern shown)");
		if (ok) {
			return px;
		}
	}
	uint32_t seed = 12345u;
	auto rnd = [&]() {
		seed = seed * 1664525u + 1013904223u;
		return (float)(seed >> 8) / 16777216.0f;
	};
	const uint32_t cols = kWidth / kCell, rows = kHeight / kCell;
	std::vector<float> cell((size_t)cols * rows * 3);
	for (uint32_t i = 0; i < cols * rows; i++) {
		const float gain = (i % 4 == 0) ? 4.0f : 1.0f;
		for (int c = 0; c < 3; c++) {
			cell[i * 3 + c] = (0.05f + 0.95f * rnd()) * gain;
		}
	}
	for (uint32_t y = 0; y < kHeight; y++) {
		for (uint32_t x = 0; x < kWidth; x++) {
			const uint32_t i = (y / kCell) * cols + (x / kCell);
			for (int c = 0; c < 3; c++) {
				float v = cell[i * 3 + c];
				if (mode.transfer == XR_HDR_TRANSFER_PQ_MNDX && mode.describe) {
					v = pq_encode(v * kWhiteNits);
				}
				px[((size_t)y * kWidth + x) * 4 + c] = half(v);
			}
			px[((size_t)y * kWidth + x) * 4 + 3] = half(1.0f);
		}
	}
	return px;
}

#define HT_XR_FUNCS(_)                                                                                                 \
	_(xrEnumerateInstanceExtensionProperties)                                                                      \
	_(xrCreateInstance)                                                                                            \
	_(xrGetSystem)                                                                                                 \
	_(xrGetVulkanGraphicsRequirements2KHR)                                                                         \
	_(xrCreateVulkanInstanceKHR)                                                                                   \
	_(xrGetVulkanGraphicsDevice2KHR)                                                                               \
	_(xrCreateVulkanDeviceKHR)                                                                                     \
	_(xrCreateSession)                                                                                             \
	_(xrCreateReferenceSpace)                                                                                      \
	_(xrEnumerateSwapchainFormats)                                                                                 \
	_(xrCreateSwapchain)                                                                                           \
	_(xrEnumerateSwapchainImages)                                                                                  \
	_(xrPollEvent)                                                                                                 \
	_(xrBeginSession)                                                                                              \
	_(xrEndSession)                                                                                                \
	_(xrRequestExitSession)                                                                                        \
	_(xrWaitFrame)                                                                                                 \
	_(xrBeginFrame)                                                                                                \
	_(xrEndFrame)                                                                                                  \
	_(xrAcquireSwapchainImage)                                                                                     \
	_(xrWaitSwapchainImage)                                                                                        \
	_(xrReleaseSwapchainImage)                                                                                     \
	_(xrDestroySwapchain)                                                                                          \
	_(xrDestroySpace)                                                                                              \
	_(xrDestroySession)                                                                                            \
	_(xrDestroyInstance)

#define HT_VK_FUNCS(_)                                                                                                 \
	_(vkGetPhysicalDeviceQueueFamilyProperties)                                                                    \
	_(vkGetPhysicalDeviceMemoryProperties)                                                                         \
	_(vkGetDeviceQueue)                                                                                            \
	_(vkCreateCommandPool)                                                                                         \
	_(vkAllocateCommandBuffers)                                                                                    \
	_(vkCreateFence)                                                                                               \
	_(vkCreateBuffer)                                                                                              \
	_(vkGetBufferMemoryRequirements)                                                                               \
	_(vkAllocateMemory)                                                                                            \
	_(vkBindBufferMemory)                                                                                          \
	_(vkMapMemory)                                                                                                 \
	_(vkBeginCommandBuffer)                                                                                        \
	_(vkEndCommandBuffer)                                                                                          \
	_(vkResetCommandBuffer)                                                                                        \
	_(vkCmdPipelineBarrier)                                                                                        \
	_(vkCmdCopyBufferToImage)                                                                                      \
	_(vkCmdClearColorImage)                                                                                        \
	_(vkQueueSubmit)                                                                                               \
	_(vkWaitForFences)                                                                                             \
	_(vkResetFences)                                                                                               \
	_(vkDeviceWaitIdle)

struct App
{
	Mode mode;
	PFN_xrGetInstanceProcAddr gipa = nullptr;
#define HT_DECL(name) PFN_##name name = nullptr;
	HT_XR_FUNCS(HT_DECL)
	HT_VK_FUNCS(HT_DECL)
#undef HT_DECL
	PFN_vkGetInstanceProcAddr vk_gipa = nullptr;

	XrInstance instance = XR_NULL_HANDLE;
	XrSystemId system = XR_NULL_SYSTEM_ID;
	XrSession session = XR_NULL_HANDLE;
	XrSpace view_space = XR_NULL_HANDLE;
	XrSwapchain swapchain = XR_NULL_HANDLE;
	std::vector<XrSwapchainImageVulkan2KHR> images;
	bool has_hdr_ext = false;
	bool has_blur_ext = false;
	bool has_cyl_ext = false;
	bool has_csb_ext = false;
	//! glass:N's panel.
	XrSwapchain panel_swapchain = XR_NULL_HANDLE;
	std::vector<XrSwapchainImageVulkan2KHR> panel_images;
	VkBuffer panel_staging = VK_NULL_HANDLE;

	VkInstance vk_instance = VK_NULL_HANDLE;
	VkPhysicalDevice phys = VK_NULL_HANDLE;
	VkDevice device = VK_NULL_HANDLE;
	uint32_t family = 0;
	VkQueue queue = VK_NULL_HANDLE;
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	VkFence fence = VK_NULL_HANDLE;
	//! shell:N's second submit (the layers per eye).
	VkCommandBuffer cmd2 = VK_NULL_HANDLE;
	VkFence fence2 = VK_NULL_HANDLE;
	VkBuffer staging = VK_NULL_HANDLE;

	bool
	xr_ok(XrResult r, const char *what)
	{
		if (XR_FAILED(r)) {
			fprintf(stderr, "riftlinux-openxr-hdr-test: %s: XrResult %d\n", what, (int)r);
			return false;
		}
		return true;
	}

	bool
	load_runtime(const char *path)
	{
		void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
		if (lib == nullptr) {
			fprintf(stderr, "riftlinux-openxr-hdr-test: %s\n", dlerror());
			return false;
		}
		auto negotiate = reinterpret_cast<PFN_xrNegotiateLoaderRuntimeInterface>(
		    dlsym(lib, "xrNegotiateLoaderRuntimeInterface"));
		if (negotiate == nullptr) {
			return fail("xrNegotiateLoaderRuntimeInterface lookup");
		}
		XrNegotiateLoaderInfo li = {};
		li.structType = XR_LOADER_INTERFACE_STRUCT_LOADER_INFO;
		li.structVersion = XR_LOADER_INFO_STRUCT_VERSION;
		li.structSize = sizeof(li);
		li.minInterfaceVersion = 1;
		li.maxInterfaceVersion = XR_CURRENT_LOADER_RUNTIME_VERSION;
		li.minApiVersion = XR_MAKE_VERSION(1, 0, 0);
		li.maxApiVersion = XR_MAKE_VERSION(1, 1, 0xffff);
		XrNegotiateRuntimeRequest rr = {};
		rr.structType = XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST;
		rr.structVersion = XR_RUNTIME_INFO_STRUCT_VERSION;
		rr.structSize = sizeof(rr);
		if (XR_FAILED(negotiate(&li, &rr)) || rr.getInstanceProcAddr == nullptr) {
			return fail("runtime negotiation");
		}
		gipa = rr.getInstanceProcAddr;
		gipa(XR_NULL_HANDLE, "xrEnumerateInstanceExtensionProperties",
		     reinterpret_cast<PFN_xrVoidFunction *>(&xrEnumerateInstanceExtensionProperties));
		gipa(XR_NULL_HANDLE, "xrCreateInstance", reinterpret_cast<PFN_xrVoidFunction *>(&xrCreateInstance));
		return xrEnumerateInstanceExtensionProperties != nullptr && xrCreateInstance != nullptr;
	}

	bool
	create_instance()
	{
		uint32_t n = 0;
		xrEnumerateInstanceExtensionProperties(nullptr, 0, &n, nullptr);
		std::vector<XrExtensionProperties> props(n, xr_struct<XrExtensionProperties>(XR_TYPE_EXTENSION_PROPERTIES));
		xrEnumerateInstanceExtensionProperties(nullptr, n, &n, props.data());
		bool has_vk2 = false;
		bool has_mask_ext = false;
		for (const auto &p : props) {
			has_mask_ext = has_mask_ext || strcmp(p.extensionName, XR_KHR_VISIBILITY_MASK_EXTENSION_NAME) == 0;
			has_vk2 = has_vk2 || strcmp(p.extensionName, XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME) == 0;
			has_hdr_ext = has_hdr_ext || strcmp(p.extensionName, XR_MNDX_COMPOSITION_LAYER_HDR_EXTENSION_NAME) == 0;
			has_blur_ext = has_blur_ext ||
			               strcmp(p.extensionName, XR_MNDX_COMPOSITION_LAYER_BACKDROP_BLUR_EXTENSION_NAME) == 0;
			has_cyl_ext = has_cyl_ext || strcmp(p.extensionName, XR_KHR_COMPOSITION_LAYER_CYLINDER_EXTENSION_NAME) == 0;
			has_csb_ext = has_csb_ext ||
			              strcmp(p.extensionName, XR_KHR_COMPOSITION_LAYER_COLOR_SCALE_BIAS_EXTENSION_NAME) == 0;
		}
		if (!has_vk2) {
			return fail("XR_KHR_vulkan_enable2 (runtime has none)");
		}
		printf("riftlinux-openxr-hdr-test: runtime %s XR_MNDX_composition_layer_hdr\n",
		       has_hdr_ext ? "offers" : "does not offer");
		if (mode.glass_mrad > 0.0f && !has_blur_ext) {
			return fail("mode needs XR_MNDX_composition_layer_backdrop_blur");
		}
		if (mode.describe && !has_hdr_ext) {
			return fail("mode needs XR_MNDX_composition_layer_hdr");
		}
		std::vector<const char *> exts = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME};
		if (mode.describe) {
			exts.push_back(XR_MNDX_COMPOSITION_LAYER_HDR_EXTENSION_NAME);
		}
		if (mode.glass_mrad > 0.0f) {
			exts.push_back(XR_MNDX_COMPOSITION_LAYER_BACKDROP_BLUR_EXTENSION_NAME);
		}
		if (mode.shell > 0) {
			// Whatever of a shell's layer extensions the runtime offers.
			if (has_hdr_ext) {
				exts.push_back(XR_MNDX_COMPOSITION_LAYER_HDR_EXTENSION_NAME);
			}
			if (has_blur_ext) {
				exts.push_back(XR_MNDX_COMPOSITION_LAYER_BACKDROP_BLUR_EXTENSION_NAME);
			}
			if (has_cyl_ext) {
				exts.push_back(XR_KHR_COMPOSITION_LAYER_CYLINDER_EXTENSION_NAME);
			}
			if (has_csb_ext) {
				exts.push_back(XR_KHR_COMPOSITION_LAYER_COLOR_SCALE_BIAS_EXTENSION_NAME);
			}
			printf("riftlinux-openxr-hdr-test: shell layers: hdr %d, blur %d, cylinder %d, colour scale %d\n",
			       has_hdr_ext, has_blur_ext, has_cyl_ext, has_csb_ext);
		}
		if (mode.mask) {
			if (!has_mask_ext) {
				return fail("mode needs XR_KHR_visibility_mask");
			}
			exts.push_back(XR_KHR_VISIBILITY_MASK_EXTENSION_NAME);
		}
		XrInstanceCreateInfo ici = {};
		ici.type = XR_TYPE_INSTANCE_CREATE_INFO;
		snprintf(ici.applicationInfo.applicationName, XR_MAX_APPLICATION_NAME_SIZE, "riftlinux-openxr-hdr-test");
		ici.applicationInfo.applicationVersion = 1;
		ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
		ici.enabledExtensionCount = (uint32_t)exts.size();
		ici.enabledExtensionNames = exts.data();
		if (!xr_ok(xrCreateInstance(&ici, &instance), "xrCreateInstance")) {
			return false;
		}
#define HT_LOAD(name) gipa(instance, #name, reinterpret_cast<PFN_xrVoidFunction *>(&name));
		HT_XR_FUNCS(HT_LOAD)
#undef HT_LOAD
		XrSystemGetInfo sgi = {};
		sgi.type = XR_TYPE_SYSTEM_GET_INFO;
		sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
		return xr_ok(xrGetSystem(instance, &sgi, &system), "xrGetSystem");
	}

	bool
	create_vulkan()
	{
		void *lib = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
		vk_gipa = lib != nullptr ? reinterpret_cast<PFN_vkGetInstanceProcAddr>(dlsym(lib, "vkGetInstanceProcAddr"))
		                         : nullptr;
		if (vk_gipa == nullptr) {
			return fail("libvulkan.so.1");
		}
		XrGraphicsRequirementsVulkan2KHR req = {};
		req.type = XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR;
		if (!xr_ok(xrGetVulkanGraphicsRequirements2KHR(instance, system, &req), "graphics requirements")) {
			return false;
		}
		VkApplicationInfo app = {};
		app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		app.pApplicationName = "riftlinux-openxr-hdr-test";
		app.apiVersion = VK_API_VERSION_1_1;
		VkInstanceCreateInfo vici = {};
		vici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		vici.pApplicationInfo = &app;
		XrVulkanInstanceCreateInfoKHR xvici = {};
		xvici.type = XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR;
		xvici.systemId = system;
		xvici.pfnGetInstanceProcAddr = vk_gipa;
		xvici.vulkanCreateInfo = &vici;
		VkResult vr = VK_SUCCESS;
		if (!xr_ok(xrCreateVulkanInstanceKHR(instance, &xvici, &vk_instance, &vr), "xrCreateVulkanInstanceKHR") ||
		    vr != VK_SUCCESS) {
			return fail("Vulkan instance");
		}
#define HT_VK_LOAD(name) name = reinterpret_cast<PFN_##name>(vk_gipa(vk_instance, #name));
		HT_VK_FUNCS(HT_VK_LOAD)
#undef HT_VK_LOAD
		XrVulkanGraphicsDeviceGetInfoKHR gdi = {};
		gdi.type = XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR;
		gdi.systemId = system;
		gdi.vulkanInstance = vk_instance;
		if (!xr_ok(xrGetVulkanGraphicsDevice2KHR(instance, &gdi, &phys), "graphics device")) {
			return false;
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
			return fail("graphics queue");
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
		XrVulkanDeviceCreateInfoKHR xdci = {};
		xdci.type = XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR;
		xdci.systemId = system;
		xdci.pfnGetInstanceProcAddr = vk_gipa;
		xdci.vulkanPhysicalDevice = phys;
		xdci.vulkanCreateInfo = &dci;
		if (!xr_ok(xrCreateVulkanDeviceKHR(instance, &xdci, &device, &vr), "xrCreateVulkanDeviceKHR") ||
		    vr != VK_SUCCESS) {
			return fail("Vulkan device");
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
		    vkAllocateCommandBuffers(device, &cai, &cmd2) != VK_SUCCESS ||
		    vkCreateFence(device, &fci, nullptr, &fence) != VK_SUCCESS ||
		    vkCreateFence(device, &fci, nullptr, &fence2) != VK_SUCCESS) {
			return fail("command buffer");
		}

		// The pattern in a host-visible buffer, copied into each swapchain image as it is used.
		if (!make_staging(make_pattern(mode), staging)) {
			return false;
		}
		return mode.glass_mrad < 0.0f || make_staging(make_panel(), panel_staging);
	}

	//! @p px in a new host-visible buffer.
	bool
	make_staging(const std::vector<uint16_t> &px, VkBuffer &out)
	{
		VkBufferCreateInfo bci = {};
		bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bci.size = px.size() * sizeof(uint16_t);
		bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		if (vkCreateBuffer(device, &bci, nullptr, &out) != VK_SUCCESS) {
			return fail("staging buffer");
		}
		VkMemoryRequirements mr;
		vkGetBufferMemoryRequirements(device, out, &mr);
		VkPhysicalDeviceMemoryProperties mp;
		vkGetPhysicalDeviceMemoryProperties(phys, &mp);
		uint32_t type = UINT32_MAX;
		const VkMemoryPropertyFlags want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
		for (uint32_t i = 0; i < mp.memoryTypeCount && type == UINT32_MAX; i++) {
			if ((mr.memoryTypeBits & (1u << i)) != 0 && (mp.memoryTypes[i].propertyFlags & want) == want) {
				type = i;
			}
		}
		VkMemoryAllocateInfo mai = {};
		mai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		mai.allocationSize = mr.size;
		mai.memoryTypeIndex = type;
		VkDeviceMemory mem = VK_NULL_HANDLE;
		void *mapped = nullptr;
		if (type == UINT32_MAX || vkAllocateMemory(device, &mai, nullptr, &mem) != VK_SUCCESS ||
		    vkBindBufferMemory(device, out, mem, 0) != VK_SUCCESS ||
		    vkMapMemory(device, mem, 0, VK_WHOLE_SIZE, 0, &mapped) != VK_SUCCESS) {
			return fail("staging memory");
		}
		memcpy(mapped, px.data(), bci.size);
		return true;
	}

	bool
	create_session()
	{
		XrGraphicsBindingVulkan2KHR binding = {};
		binding.type = XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR;
		binding.instance = vk_instance;
		binding.physicalDevice = phys;
		binding.device = device;
		binding.queueFamilyIndex = family;
		binding.queueIndex = 0;
		XrSessionCreateInfo sci = {};
		sci.type = XR_TYPE_SESSION_CREATE_INFO;
		sci.next = &binding;
		sci.systemId = system;
		if (!xr_ok(xrCreateSession(instance, &sci, &session), "xrCreateSession")) {
			return false;
		}
		XrReferenceSpaceCreateInfo rsci = {};
		rsci.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO;
		rsci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
		rsci.poseInReferenceSpace.orientation.w = 1.0f;
		if (!xr_ok(xrCreateReferenceSpace(session, &rsci, &view_space), "view space")) {
			return false;
		}
		uint32_t n = 0;
		xrEnumerateSwapchainFormats(session, 0, &n, nullptr);
		std::vector<int64_t> formats(n);
		xrEnumerateSwapchainFormats(session, n, &n, formats.data());
		bool has_float = false;
		for (int64_t f : formats) {
			has_float = has_float || f == VK_FORMAT_R16G16B16A16_SFLOAT;
		}
		if (!has_float) {
			return fail("R16G16B16A16_SFLOAT swapchain (runtime offers none)");
		}
		return make_swapchain(swapchain, images) &&
		       (mode.glass_mrad < 0.0f || make_swapchain(panel_swapchain, panel_images));
	}

	//! A float swapchain of the pattern's size.
	bool
	make_swapchain(XrSwapchain &out,
	               std::vector<XrSwapchainImageVulkan2KHR> &out_images,
	               uint32_t width = kWidth,
	               uint32_t height = kHeight,
	               int64_t format = VK_FORMAT_R16G16B16A16_SFLOAT)
	{
		uint32_t n = 0;
		XrSwapchainCreateInfo swci = {};
		swci.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
		swci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
		                  XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
		swci.format = format;
		swci.sampleCount = 1;
		swci.width = width;
		swci.height = height;
		swci.faceCount = 1;
		swci.arraySize = 1;
		swci.mipCount = 1;
		if (!xr_ok(xrCreateSwapchain(session, &swci, &out), "xrCreateSwapchain")) {
			return false;
		}
		xrEnumerateSwapchainImages(out, 0, &n, nullptr);
		out_images.assign(n, xr_struct<XrSwapchainImageVulkan2KHR>(XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR));
		return xr_ok(xrEnumerateSwapchainImages(out, n, &n,
		                                        reinterpret_cast<XrSwapchainImageBaseHeader *>(out_images.data())),
		             "swapchain images");
	}

	//! @p buffer into swapchain image @p image, left as a colour attachment as the runtime expects.
	void
	fill_image(VkImage image, VkBuffer buffer)
	{
		vkResetCommandBuffer(cmd, 0);
		VkCommandBufferBeginInfo bi = {};
		bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		vkBeginCommandBuffer(cmd, &bi);
		VkImageMemoryBarrier b = {};
		b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		b.srcAccessMask = 0;
		b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		b.image = image;
		b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr,
		                     0, nullptr, 1, &b);
		VkBufferImageCopy copy = {};
		copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		copy.imageExtent = {kWidth, kHeight, 1};
		vkCmdCopyBufferToImage(cmd, buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
		b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		b.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		b.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
		                     0, nullptr, 0, nullptr, 1, &b);
		vkEndCommandBuffer(cmd);
		VkSubmitInfo si = {};
		si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		si.commandBufferCount = 1;
		si.pCommandBuffers = &cmd;
		vkResetFences(device, 1, &fence);
		vkQueueSubmit(queue, 1, &si, fence);
		vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX);
	}

	//! @p imgs cleared to a grey, one fenced submit and a wait, left as colour attachments (shell:N's copies).
	void
	clear_images(VkCommandBuffer cb, VkFence f, const std::vector<VkImage> &imgs, float shade)
	{
		if (imgs.empty()) {
			return;
		}
		vkResetCommandBuffer(cb, 0);
		VkCommandBufferBeginInfo bi = {};
		bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		vkBeginCommandBuffer(cb, &bi);
		const VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		const VkClearColorValue colour = {{shade, shade, shade, 1.0f}};
		for (VkImage image : imgs) {
			VkImageMemoryBarrier b = {};
			b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
			b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			b.image = image;
			b.subresourceRange = range;
			vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0,
			                     nullptr, 0, nullptr, 1, &b);
			vkCmdClearColorImage(cb, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &colour, 1, &range);
			b.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			b.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
			b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			b.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
			vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
			                     0, nullptr, 0, nullptr, 1, &b);
		}
		vkEndCommandBuffer(cb);
		VkSubmitInfo si = {};
		si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		si.commandBufferCount = 1;
		si.pCommandBuffers = &cb;
		vkResetFences(device, 1, &f);
		vkQueueSubmit(queue, 1, &si, f);
		vkWaitForFences(device, 1, &f, VK_TRUE, UINT64_MAX);
	}

	//! shell:N: a window's layer, the room, the layers per eye.
	struct ShellLayer
	{
		XrSwapchain sc = XR_NULL_HANDLE;
		std::vector<XrSwapchainImageVulkan2KHR> imgs;
		uint32_t w = 0, h = 0;
		int64_t format = VK_FORMAT_R16G16B16A16_SFLOAT;
		bool video = false, cylinder = false, glass = false;
	};
	std::vector<std::vector<ShellLayer>> shell_windows;
	ShellLayer shell_room, shell_eye[2];
	int shell_made = 0, shell_destroyed = 0, shell_window_count = 0;
	uint32_t shell_seed = 4242u;

	//! shell:N: a new swapchain for @p l, its size and format as a shell's window layers have them.
	bool
	shell_make(ShellLayer &l, bool &exit_requested)
	{
		auto rnd = [&](uint32_t n) {
			shell_seed = shell_seed * 1664525u + 1013904223u;
			return (shell_seed >> 8) % n;
		};
		if (l.video) {
			l.w = 1920;
			l.h = 1080;
			l.format = VK_FORMAT_R16G16B16A16_SFLOAT;
		} else {
			// Window sizes rounded up by an eighth, in steps of 16.
			auto round = [](uint32_t v) { return ((v + v / 8 + 15) / 16) * 16; };
			l.w = round(64 + rnd(1536));
			l.h = round(48 + rnd(864));
			l.format = rnd(3) == 0 ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R16G16B16A16_SFLOAT;
		}
		if (!make_swapchain(l.sc, l.imgs, l.w, l.h, l.format)) {
			return false;
		}
		shell_made++;
		if (shell_made % 50 == 0 || shell_made == mode.shell) {
			long rss, heap;
			memory_kb(rss, heap);
			printf("riftlinux-openxr-hdr-test: shell %d swapchains made, %d destroyed, VmRSS %ld kB, heap in use %ld "
			       "kB\n",
			       shell_made, shell_destroyed, rss, heap);
			fflush(stdout);
		}
		if (shell_made >= mode.shell && !mode.shell_hold && !exit_requested) {
			xrRequestExitSession(session);
			exit_requested = true;
		}
		return true;
	}

	void
	shell_destroy(ShellLayer &l)
	{
		if (l.sc != XR_NULL_HANDLE) {
			xrDestroySwapchain(l.sc);
			l.sc = XR_NULL_HANDLE;
			shell_destroyed++;
		}
	}

	//! One frame of shell:N, between xrBeginFrame and xrEndFrame, then xrEndFrame.
	bool
	shell_frame(const XrFrameState &fs, uint64_t frames, bool &exit_requested)
	{
		auto rnd = [&](uint32_t n) {
			shell_seed = shell_seed * 1664525u + 1013904223u;
			return (shell_seed >> 8) % n;
		};
		XrSwapchainImageAcquireInfo ai = {};
		ai.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO;
		XrSwapchainImageWaitInfo wi = {};
		wi.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO;
		wi.timeout = XR_INFINITE_DURATION;
		XrSwapchainImageReleaseInfo ri = {};
		ri.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
		std::vector<XrSwapchain> to_release;
		std::vector<VkImage> to_clear;
		auto use = [&](ShellLayer &l, std::vector<XrSwapchain> &rel, std::vector<VkImage> &clr) {
			uint32_t index = 0;
			if (xrAcquireSwapchainImage(l.sc, &ai, &index) == XR_SUCCESS &&
			    xrWaitSwapchainImage(l.sc, &wi) == XR_SUCCESS) {
				clr.push_back(l.imgs[index].image);
				rel.push_back(l.sc);
			}
		};
		const bool making = shell_made < mode.shell;

		// The room and the layers per eye, once.
		if (shell_room.sc == XR_NULL_HANDLE) {
			shell_room.w = 2048;
			shell_room.h = 1024;
			if (!make_swapchain(shell_room.sc, shell_room.imgs, shell_room.w, shell_room.h)) {
				return false;
			}
			use(shell_room, to_release, to_clear);
			for (ShellLayer &e : shell_eye) {
				e.w = 1344;
				e.h = 1600;
				e.format = VK_FORMAT_R8G8B8A8_SRGB;
				if (!make_swapchain(e.sc, e.imgs, e.w, e.h, e.format)) {
					return false;
				}
			}
		}
		// A window closed: its layers left out from this frame on, its swapchains destroyed now, in a burst, while
		// they were in the last xrEndFrame's layers.
		const uint64_t phase = frames % 40;
		if (phase == 0 && shell_windows.size() >= 4) {
			for (ShellLayer &l : shell_windows.front()) {
				shell_destroy(l);
			}
			shell_windows.erase(shell_windows.begin());
		}
		// A window opened: its 10 swapchains made mid-frame, each image used once now and shown in this frame.
		if (phase == 20 && making) {
			std::vector<ShellLayer> win(10);
			const int n = shell_window_count++;
			for (size_t i = 0; i < win.size(); i++) {
				win[i].video = i == 0 && n % 3 == 0;
				win[i].cylinder = has_cyl_ext && n % 3 == 1;
				win[i].glass = i >= 8;
				if (!shell_make(win[i], exit_requested)) {
					return false;
				}
				use(win[i], to_release, to_clear);
			}
			shell_windows.push_back(std::move(win));
		}
		// A size change: the new swapchain made and used, the old one (in the last xrEndFrame) destroyed, same frame.
		if (phase == 10 && making && !shell_windows.empty()) {
			std::vector<ShellLayer> &win = shell_windows[rnd((uint32_t)shell_windows.size())];
			ShellLayer &old = win[1 + rnd((uint32_t)win.size() - 1)];
			ShellLayer fresh;
			fresh.cylinder = old.cylinder;
			fresh.glass = old.glass;
			if (!shell_make(fresh, exit_requested)) {
				return false;
			}
			use(fresh, to_release, to_clear);
			shell_destroy(old);
			old = std::move(fresh);
		}
		// Video layers every frame.
		for (auto &win : shell_windows) {
			for (ShellLayer &l : win) {
				if (l.video) {
					use(l, to_release, to_clear);
				}
			}
		}
		// The application's own copies: one fenced submit and a wait, then everything acquired released.
		if (mode.shell_copy) {
			clear_images(cmd, fence, to_clear, 0.2f + 0.1f * (float)(frames % 5));
		}
		for (XrSwapchain sc : to_release) {
			xrReleaseSwapchainImage(sc, &ri);
		}
		// The layers per eye, drawn every frame with a second fenced submit.
		std::vector<XrSwapchain> eye_release;
		std::vector<VkImage> eye_clear;
		for (ShellLayer &e : shell_eye) {
			use(e, eye_release, eye_clear);
		}
		if (mode.shell_copy) {
			clear_images(cmd2, fence2, eye_clear, 0.05f);
		}
		for (XrSwapchain sc : eye_release) {
			xrReleaseSwapchainImage(sc, &ri);
		}

		// The layers: the room, the windows (unless :nolayers), one layer per eye on top.
		std::vector<XrCompositionLayerQuad> quads;
		std::vector<XrCompositionLayerCylinderKHR> cyls;
		std::vector<XrCompositionLayerHdrMNDX> hdrs;
		std::vector<XrCompositionLayerBackdropBlurMNDX> blurs;
		std::vector<XrCompositionLayerColorScaleBiasKHR> csbs;
		quads.reserve(128);
		cyls.reserve(128);
		hdrs.reserve(128);
		blurs.reserve(128);
		csbs.reserve(128);
		std::vector<const XrCompositionLayerBaseHeader *> layers;
		auto describe = [&](const ShellLayer &l) -> void * {
			if (!has_hdr_ext) {
				return nullptr;
			}
			XrCompositionLayerHdrMNDX h = {};
			h.type = XR_TYPE_COMPOSITION_LAYER_HDR_MNDX;
			h.referenceWhiteNits = kWhiteNits;
			if (l.video) {
				h.transfer = XR_HDR_TRANSFER_PQ_MNDX;
				h.primaries = XR_HDR_PRIMARIES_BT2020_MNDX;
				h.maxContentLightLevelNits = 1000.0f;
				h.maxFrameAverageLightLevelNits = 400.0f;
				h.masteringMinNits = 0.005f;
				h.masteringMaxNits = 1000.0f;
			} else {
				h.transfer = l.format == VK_FORMAT_R8G8B8A8_SRGB ? XR_HDR_TRANSFER_SDR_MNDX : XR_HDR_TRANSFER_LINEAR_MNDX;
				h.primaries = XR_HDR_PRIMARIES_BT709_MNDX;
			}
			hdrs.push_back(h);
			return &hdrs.back();
		};
		auto add = [&](const ShellLayer &l, float x, float y, float z, float width, XrEyeVisibility eye, bool fading,
		               int &glass_count) {
			void *next = describe(l);
			if (l.glass && has_blur_ext && glass_count < 4) {
				XrCompositionLayerBackdropBlurMNDX b = {};
				b.type = XR_TYPE_COMPOSITION_LAYER_BACKDROP_BLUR_MNDX;
				b.next = next;
				b.blurRadians = 0.01f;
				b.saturation = 1.0f;
				b.tint = {1.0f, 1.0f, 1.0f, 0.0f};
				b.maskAlpha = 0.3f;
				blurs.push_back(b);
				next = &blurs.back();
				glass_count++;
			}
			if (fading && has_csb_ext) {
				XrCompositionLayerColorScaleBiasKHR c = {};
				c.type = XR_TYPE_COMPOSITION_LAYER_COLOR_SCALE_BIAS_KHR;
				c.next = next;
				c.colorScale = {1.0f, 1.0f, 1.0f, 0.5f};
				csbs.push_back(c);
				next = &csbs.back();
			}
			const XrSwapchainSubImage sub = {l.sc, {{0, 0}, {(int32_t)l.w, (int32_t)l.h}}, 0};
			const XrCompositionLayerFlags flags =
			    l.video ? 0 : XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
			XrPosef pose = {};
			pose.orientation.w = 1.0f;
			pose.position = {x, y, z};
			if (l.cylinder) {
				XrCompositionLayerCylinderKHR c = {};
				c.type = XR_TYPE_COMPOSITION_LAYER_CYLINDER_KHR;
				c.next = next;
				c.layerFlags = flags;
				c.space = view_space;
				c.eyeVisibility = eye;
				c.subImage = sub;
				c.pose = pose;
				c.radius = -z;
				c.centralAngle = width / -z;
				c.aspectRatio = (float)l.w / (float)l.h;
				cyls.push_back(c);
				layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader *>(&cyls.back()));
			} else {
				XrCompositionLayerQuad q = {};
				q.type = XR_TYPE_COMPOSITION_LAYER_QUAD;
				q.next = next;
				q.layerFlags = flags;
				q.space = view_space;
				q.eyeVisibility = eye;
				q.subImage = sub;
				q.pose = pose;
				q.size = {width, width * (float)l.h / (float)l.w};
				quads.push_back(q);
				layers.push_back(reinterpret_cast<const XrCompositionLayerBaseHeader *>(&quads.back()));
			}
		};
		if (fs.shouldRender) {
			int glass_count = 0;
			ShellLayer room = shell_room;
			room.cylinder = has_cyl_ext;
			add(room, 0.0f, 0.0f, -4.0f, 6.0f, XR_EYE_VISIBILITY_BOTH, has_csb_ext, glass_count);
			if (mode.shell_layers) {
				for (size_t w = 0; w < shell_windows.size(); w++) {
					const bool fading = shell_windows.size() >= 4 && w == 0;
					for (size_t i = 0; i < shell_windows[w].size(); i++) {
						const ShellLayer &l = shell_windows[w][i];
						add(l, -0.9f + 0.6f * (float)w + 0.05f * (float)(i % 3), -0.3f + 0.06f * (float)i,
						    -2.0f + 0.01f * (float)i, 0.5f, XR_EYE_VISIBILITY_BOTH, fading, glass_count);
					}
				}
			}
			add(shell_eye[0], 0.0f, -0.4f, -1.0f, 0.3f, XR_EYE_VISIBILITY_LEFT, false, glass_count);
			add(shell_eye[1], 0.0f, -0.4f, -1.0f, 0.3f, XR_EYE_VISIBILITY_RIGHT, false, glass_count);
		}
		XrFrameEndInfo fei = {};
		fei.type = XR_TYPE_FRAME_END_INFO;
		fei.displayTime = fs.predictedDisplayTime;
		fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
		fei.layerCount = (uint32_t)layers.size();
		fei.layers = layers.data();
		return xr_ok(xrEndFrame(session, &fei), "xrEndFrame");
	}

	//! churn:N's swapchains alive, how many made, its random sizes.
	std::vector<XrSwapchain> churn_alive;
	int churn_made = 0;
	uint32_t churn_seed = 4242u;

	//! The process's resident memory and its heap in use (glibc's mallinfo2, looked up as old headers lack it), kB.
	static void
	memory_kb(long &rss, long &heap)
	{
		rss = -1;
		heap = -1;
		FILE *f = fopen("/proc/self/status", "r");
		char line[256];
		while (f != nullptr && fgets(line, sizeof(line), f) != nullptr) {
			if (strncmp(line, "VmRSS:", 6) == 0) {
				rss = atol(line + 6);
			}
		}
		if (f != nullptr) {
			fclose(f);
		}
		struct mi2
		{
			size_t arena, ordblks, smblks, hblks, hblkhd, usmblks, fsmblks, uordblks, fordblks, keepcost;
		};
		auto fn = reinterpret_cast<mi2 (*)()>(dlsym(RTLD_DEFAULT, "mallinfo2"));
		if (fn != nullptr) {
			const mi2 m = fn();
			heap = (long)((m.uordblks + m.hblkhd) / 1024);
		}
	}

	int
	run(double seconds)
	{
		const double start = now_s();
		double last_log = start;
		bool running = false, exit_requested = false;
		uint64_t frames = 0, shown = 0;
		for (;;) {
			XrEventDataBuffer ev = {};
			ev.type = XR_TYPE_EVENT_DATA_BUFFER;
			while (xrPollEvent(instance, &ev) == XR_SUCCESS) {
				if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
					const auto *sc = reinterpret_cast<const XrEventDataSessionStateChanged *>(&ev);
					printf("riftlinux-openxr-hdr-test: session state %d\n", (int)sc->state);
					if (sc->state == XR_SESSION_STATE_READY) {
						XrSessionBeginInfo sbi = {};
						sbi.type = XR_TYPE_SESSION_BEGIN_INFO;
						sbi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
						running = xr_ok(xrBeginSession(session, &sbi), "xrBeginSession");
					} else if (sc->state == XR_SESSION_STATE_STOPPING) {
						xrEndSession(session);
						running = false;
					} else if (sc->state == XR_SESSION_STATE_EXITING ||
					           sc->state == XR_SESSION_STATE_LOSS_PENDING) {
						printf("riftlinux-openxr-hdr-test: %llu frames, %llu with the layer\n",
						       (unsigned long long)frames, (unsigned long long)shown);
						return 0;
					}
				} else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
					return 1;
				}
				ev = xr_struct<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
			}
			const double now = now_s();
			if (now - start > seconds && !exit_requested && running) {
				xrRequestExitSession(session);
				exit_requested = true;
			}
			if (!running) {
				struct timespec ts = {0, 10000000};
				nanosleep(&ts, nullptr);
				continue;
			}
			XrFrameState fs = {};
			fs.type = XR_TYPE_FRAME_STATE;
			if (!xr_ok(xrWaitFrame(session, nullptr, &fs), "xrWaitFrame") ||
			    !xr_ok(xrBeginFrame(session, nullptr), "xrBeginFrame")) {
				return 1;
			}
			frames++;
			if (mode.churn > 0) {
				// As a desktop shell's windows: one closed at frame start, one opened mid-frame, all used each frame.
				auto rnd = [&](uint32_t n) {
					churn_seed = churn_seed * 1664525u + 1013904223u;
					return (churn_seed >> 8) % n;
				};
				const bool step = churn_made < mode.churn && frames % 4 == 0;
				if (step && churn_alive.size() >= 12) {
					const uint32_t k = rnd((uint32_t)churn_alive.size());
					xrDestroySwapchain(churn_alive[k]);
					churn_alive.erase(churn_alive.begin() + k);
				}
				if (step) {
					XrSwapchain sc = XR_NULL_HANDLE;
					std::vector<XrSwapchainImageVulkan2KHR> imgs;
					const int64_t format = (churn_made % 2) ? VK_FORMAT_R16G16B16A16_SFLOAT : VK_FORMAT_R8G8B8A8_SRGB;
					if (make_swapchain(sc, imgs, 128 + rnd(1792), 128 + rnd(952), format)) {
						churn_alive.push_back(sc);
						churn_made++;
						if (churn_made % 50 == 0 || churn_made == mode.churn) {
							long rss, heap;
							memory_kb(rss, heap);
							printf("riftlinux-openxr-hdr-test: churn %d swapchains made, VmRSS %ld kB, heap in use %ld kB\n",
							       churn_made, rss, heap);
							fflush(stdout);
						}
						if (churn_made == mode.churn && !exit_requested) {
							xrRequestExitSession(session);
							exit_requested = true;
						}
					}
				}
				XrSwapchainImageAcquireInfo cai = {};
				cai.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO;
				XrSwapchainImageWaitInfo cwi = {};
				cwi.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO;
				cwi.timeout = XR_INFINITE_DURATION;
				XrSwapchainImageReleaseInfo cri = {};
				cri.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
				for (XrSwapchain sc : churn_alive) {
					uint32_t index = 0;
					if (xrAcquireSwapchainImage(sc, &cai, &index) == XR_SUCCESS &&
					    xrWaitSwapchainImage(sc, &cwi) == XR_SUCCESS) {
						xrReleaseSwapchainImage(sc, &cri);
					}
				}
			}
			if (mode.shell > 0) {
				if (!shell_frame(fs, frames, exit_requested)) {
					return 1;
				}
				if (now - last_log >= (mode.shell_hold ? 10.0 : 1.0)) {
					last_log = now;
					long rss, heap;
					memory_kb(rss, heap);
					printf("riftlinux-openxr-hdr-test: %.0f s, mode %s, %llu frames, %d windows, %d swapchains made, "
					       "%d destroyed, VmRSS %ld kB, heap in use %ld kB\n",
					       now - start, mode.name.c_str(), (unsigned long long)frames, (int)shell_windows.size(),
					       shell_made, shell_destroyed, rss, heap);
					fflush(stdout);
				}
				continue;
			}
			XrCompositionLayerQuad quad = {};
			quad.type = XR_TYPE_COMPOSITION_LAYER_QUAD;
			XrCompositionLayerHdrMNDX hdr = {};
			hdr.type = XR_TYPE_COMPOSITION_LAYER_HDR_MNDX;
			XrCompositionLayerQuad panel = {};
			panel.type = XR_TYPE_COMPOSITION_LAYER_QUAD;
			XrCompositionLayerBackdropBlurMNDX blur = {};
			blur.type = XR_TYPE_COMPOSITION_LAYER_BACKDROP_BLUR_MNDX;
			const XrCompositionLayerBaseHeader *layers[2] = {
			    reinterpret_cast<const XrCompositionLayerBaseHeader *>(&quad),
			    reinterpret_cast<const XrCompositionLayerBaseHeader *>(&panel)};
			uint32_t layer_count = 0;
			if (fs.shouldRender) {
				uint32_t index = 0;
				XrSwapchainImageAcquireInfo ai = {};
				ai.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO;
				XrSwapchainImageWaitInfo wi = {};
				wi.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO;
				wi.timeout = XR_INFINITE_DURATION;
				XrSwapchainImageReleaseInfo ri = {};
				ri.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
				if (xr_ok(xrAcquireSwapchainImage(swapchain, &ai, &index), "acquire") &&
				    xr_ok(xrWaitSwapchainImage(swapchain, &wi), "wait image")) {
					fill_image(images[index].image, staging);
					xrReleaseSwapchainImage(swapchain, &ri);
					quad.space = view_space;
					quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
					quad.subImage.swapchain = swapchain;
					quad.subImage.imageRect = {{0, 0}, {(int32_t)kWidth, (int32_t)kHeight}};
					quad.pose.orientation.w = 1.0f;
					quad.pose.position.z = -1.2f;
					quad.size = {1.2f, 1.2f * kHeight / kWidth};
					if (mode.describe) {
						hdr.transfer = mode.transfer;
						hdr.primaries = mode.video ? XR_HDR_PRIMARIES_BT2020_MNDX : XR_HDR_PRIMARIES_BT709_MNDX;
						hdr.referenceWhiteNits = kWhiteNits;
						hdr.maxContentLightLevelNits = mode.cll_nits;
						if (mode.video) {
							hdr.masteringMinNits = 0.005f;
							hdr.masteringMaxNits = 1000.0f;
							hdr.maxFrameAverageLightLevelNits = 400.0f;
						}
						hdr.sceneMaxNits = mode.scene_nits;
						hdr.sceneAverageNits = mode.scene_nits > 0.0f ? kWhiteNits * 0.7f : 0.0f;
						quad.next = &hdr;
					}
					layer_count = 1;
					uint32_t panel_index = 0;
					if (mode.glass_mrad >= 0.0f &&
					    xr_ok(xrAcquireSwapchainImage(panel_swapchain, &ai, &panel_index), "acquire panel") &&
					    xr_ok(xrWaitSwapchainImage(panel_swapchain, &wi), "wait panel")) {
						fill_image(panel_images[panel_index].image, panel_staging);
						xrReleaseSwapchainImage(panel_swapchain, &ri);
						panel.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
						                   XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
						panel.space = view_space;
						panel.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
						panel.subImage.swapchain = panel_swapchain;
						panel.subImage.imageRect = {{0, 0}, {(int32_t)kWidth, (int32_t)kHeight}};
						panel.pose.orientation.w = 1.0f;
						panel.pose.position.z = -1.0f;
						panel.size = {0.6f, 0.6f * kHeight / kWidth};
						if (mode.glass_mrad > 0.0f) {
							blur.blurRadians = mode.glass_mrad / 1000.0f;
							blur.saturation = 1.0f;
							blur.tint = {1.0f, 1.0f, 1.0f, 0.0f};
							blur.maskAlpha = 0.3f;
							panel.next = &blur;
						}
						layer_count = 2;
					}
					shown++;
				}
			}
			XrFrameEndInfo fei = {};
			fei.type = XR_TYPE_FRAME_END_INFO;
			fei.displayTime = fs.predictedDisplayTime;
			fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
			fei.layerCount = layer_count;
			fei.layers = layers;
			if (!xr_ok(xrEndFrame(session, &fei), "xrEndFrame")) {
				return 1;
			}
			if (now - last_log >= 1.0) {
				last_log = now;
				printf("riftlinux-openxr-hdr-test: %.0f s, mode %s, %llu frames, %llu with the layer\n", now - start,
				       mode.name.c_str(), (unsigned long long)frames, (unsigned long long)shown);
				fflush(stdout);
			}
		}
	}
};

} // namespace

int
main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s <libopenxr_monado.so> [seconds, 60] [none|linear|scene:N|cll:N|pq:N|sdr|probe|glass:N|mask|swapchains:N|churn:N|shell:N[:nolayers][:nocopy]|rlof:X:Y:F]\n",
		        argv[0]);
		return 2;
	}
	App app;
	const double seconds = argc > 2 ? atof(argv[2]) : 60.0;
	if (argc > 3 && !parse_mode(argv[3], app.mode)) {
		fprintf(stderr, "riftlinux-openxr-hdr-test: unknown mode %s\n", argv[3]);
		return 2;
	}
	setvbuf(stdout, nullptr, _IOLBF, 0);
	printf("riftlinux-openxr-hdr-test: mode %s, %.0f s\n", app.mode.name.c_str(), seconds);
	if (app.mode.probe) {
		// As Proton's OpenXR check: an instance and its system, held, then gone.
		if (!app.load_runtime(argv[1]) || !app.create_instance()) {
			return 1;
		}
		printf("riftlinux-openxr-hdr-test: probe: instance made, holding it %.0f s\n", seconds);
		struct timespec ts = {(time_t)seconds, 0};
		nanosleep(&ts, nullptr);
		app.xrDestroyInstance(app.instance);
		return 0;
	}
	if (!app.load_runtime(argv[1]) || !app.create_instance() || !app.create_vulkan() || !app.create_session()) {
		return 1;
	}
	if (app.mode.swapchains > 0) {
		// The client library's memory per swapchain made: made and destroyed again and again, VmRSS every 50.
		auto rss_kb = []() {
			long kb = -1;
			FILE *f = fopen("/proc/self/status", "r");
			char line[256];
			while (f != nullptr && fgets(line, sizeof(line), f) != nullptr) {
				if (strncmp(line, "VmRSS:", 6) == 0) {
					kb = atol(line + 6);
				}
			}
			if (f != nullptr) {
				fclose(f);
			}
			return kb;
		};
		int ret = 0;
		long first = -1;
		// As a desktop shell's windows: 12 alive at once, sizes as windows have them, one closed and another opened
		// at random each step (the pool's freed memory then lies between live command buffers).
		uint32_t seed = 4242u;
		auto rnd = [&](uint32_t n) {
			seed = seed * 1664525u + 1013904223u;
			return (seed >> 8) % n;
		};
		std::vector<XrSwapchain> alive;
		for (int i = 1; i <= app.mode.swapchains; i++) {
			if (alive.size() >= 12) {
				const uint32_t k = rnd((uint32_t)alive.size());
				app.xrDestroySwapchain(alive[k]);
				alive.erase(alive.begin() + k);
			}
			XrSwapchain sc = XR_NULL_HANDLE;
			std::vector<XrSwapchainImageVulkan2KHR> imgs;
			if (!app.make_swapchain(sc, imgs, 200 + rnd(1720), 120 + rnd(960))) {
				ret = 1;
				break;
			}
			// Used as an application uses one: every image acquired, waited for and released (the client library
			// runs its command buffers then).
			for (size_t k = 0; k < imgs.size(); k++) {
				uint32_t index = 0;
				XrSwapchainImageAcquireInfo ai = {};
				ai.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO;
				XrSwapchainImageWaitInfo wi = {};
				wi.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO;
				wi.timeout = XR_INFINITE_DURATION;
				XrSwapchainImageReleaseInfo ri = {};
				ri.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
				if (app.xrAcquireSwapchainImage(sc, &ai, &index) == XR_SUCCESS &&
				    app.xrWaitSwapchainImage(sc, &wi) == XR_SUCCESS) {
					app.xrReleaseSwapchainImage(sc, &ri);
				}
			}
			alive.push_back(sc);
			if (i == 10) {
				first = rss_kb();
			}
			if (i % 50 == 0 || i == app.mode.swapchains) {
				printf("riftlinux-openxr-hdr-test: %d swapchains made and destroyed, VmRSS %ld kB\n", i, rss_kb());
			}
		}
		for (XrSwapchain sc : alive) {
			app.xrDestroySwapchain(sc);
		}
		if (first >= 0 && app.mode.swapchains > 10) {
			printf("riftlinux-openxr-hdr-test: growth after the first 10: %.1f kB per swapchain\n",
			       (double)(rss_kb() - first) / (app.mode.swapchains - 10));
		}
		app.xrDestroySwapchain(app.swapchain);
		app.xrDestroySpace(app.view_space);
		app.xrDestroySession(app.session);
		app.xrDestroyInstance(app.instance);
		return ret;
	}
	if (app.mode.mask) {
		// The runtime's visibility masks of both views, as a game asks for them; nothing drawn.
		PFN_xrGetVisibilityMaskKHR get_mask = nullptr;
		app.gipa(app.instance, "xrGetVisibilityMaskKHR", reinterpret_cast<PFN_xrVoidFunction *>(&get_mask));
		int ret = get_mask != nullptr ? 0 : 1;
		const char *names[] = {"hidden", "visible", "line loop"};
		const XrVisibilityMaskTypeKHR types[] = {XR_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH_KHR,
		                                         XR_VISIBILITY_MASK_TYPE_VISIBLE_TRIANGLE_MESH_KHR,
		                                         XR_VISIBILITY_MASK_TYPE_LINE_LOOP_KHR};
		for (uint32_t view = 0; get_mask != nullptr && view < 2; view++) {
			for (int t = 0; t < 3; t++) {
				XrVisibilityMaskKHR m = xr_struct<XrVisibilityMaskKHR>(XR_TYPE_VISIBILITY_MASK_KHR);
				XrResult r = get_mask(app.session, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, view, types[t], &m);
				std::vector<XrVector2f> v(m.vertexCountOutput);
				std::vector<uint32_t> idx(m.indexCountOutput);
				if (r == XR_SUCCESS && m.vertexCountOutput > 0 && m.indexCountOutput > 0) {
					m.vertexCapacityInput = m.vertexCountOutput;
					m.indexCapacityInput = m.indexCountOutput;
					m.vertices = v.data();
					m.indices = idx.data();
					r = get_mask(app.session, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, view, types[t], &m);
				}
				printf("riftlinux-openxr-hdr-test: mask view %u %s: XrResult %d, %u vertices, %u indices", view,
				       names[t], (int)r, m.vertexCountOutput, m.indexCountOutput);
				for (uint32_t i = 0; i < v.size() && i < 12; i++) {
					printf(" (%.3f %.3f)", v[i].x, v[i].y);
				}
				printf("\n");
				ret = r == XR_SUCCESS ? ret : 1;
			}
		}
		app.xrDestroySpace(app.view_space);
		app.xrDestroySession(app.session);
		app.xrDestroyInstance(app.instance);
		return ret;
	}
	const int ret = app.run(seconds);
	app.vkDeviceWaitIdle(app.device);
	app.xrDestroySwapchain(app.swapchain);
	if (app.panel_swapchain != XR_NULL_HANDLE) {
		app.xrDestroySwapchain(app.panel_swapchain);
	}
	app.xrDestroySpace(app.view_space);
	app.xrDestroySession(app.session);
	app.xrDestroyInstance(app.instance);
	return ret;
}
