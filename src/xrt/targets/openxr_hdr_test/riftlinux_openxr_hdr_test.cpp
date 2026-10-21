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
	return false;
}

//! The pattern as RGBA half floats: cells in pseudo-random colours (linear light, white 1.0), every fourth four
//! times as bright; PQ-encoded for PQ mode.
std::vector<uint16_t>
make_pattern(const Mode &mode)
{
	std::vector<uint16_t> px((size_t)kWidth * kHeight * 4);
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

	VkInstance vk_instance = VK_NULL_HANDLE;
	VkPhysicalDevice phys = VK_NULL_HANDLE;
	VkDevice device = VK_NULL_HANDLE;
	uint32_t family = 0;
	VkQueue queue = VK_NULL_HANDLE;
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	VkFence fence = VK_NULL_HANDLE;
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
		for (const auto &p : props) {
			has_vk2 = has_vk2 || strcmp(p.extensionName, XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME) == 0;
			has_hdr_ext = has_hdr_ext || strcmp(p.extensionName, XR_MNDX_COMPOSITION_LAYER_HDR_EXTENSION_NAME) == 0;
		}
		if (!has_vk2) {
			return fail("XR_KHR_vulkan_enable2 (runtime has none)");
		}
		printf("riftlinux-openxr-hdr-test: runtime %s XR_MNDX_composition_layer_hdr\n",
		       has_hdr_ext ? "offers" : "does not offer");
		if (mode.describe && !has_hdr_ext) {
			return fail("mode needs XR_MNDX_composition_layer_hdr");
		}
		std::vector<const char *> exts = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME};
		if (mode.describe) {
			exts.push_back(XR_MNDX_COMPOSITION_LAYER_HDR_EXTENSION_NAME);
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
		    vkCreateFence(device, &fci, nullptr, &fence) != VK_SUCCESS) {
			return fail("command buffer");
		}

		// The pattern in a host-visible buffer, copied into each swapchain image as it is used.
		const std::vector<uint16_t> px = make_pattern(mode);
		VkBufferCreateInfo bci = {};
		bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bci.size = px.size() * sizeof(uint16_t);
		bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		if (vkCreateBuffer(device, &bci, nullptr, &staging) != VK_SUCCESS) {
			return fail("staging buffer");
		}
		VkMemoryRequirements mr;
		vkGetBufferMemoryRequirements(device, staging, &mr);
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
		    vkBindBufferMemory(device, staging, mem, 0) != VK_SUCCESS ||
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
		XrSwapchainCreateInfo swci = {};
		swci.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
		swci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
		                  XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
		swci.format = VK_FORMAT_R16G16B16A16_SFLOAT;
		swci.sampleCount = 1;
		swci.width = kWidth;
		swci.height = kHeight;
		swci.faceCount = 1;
		swci.arraySize = 1;
		swci.mipCount = 1;
		if (!xr_ok(xrCreateSwapchain(session, &swci, &swapchain), "xrCreateSwapchain")) {
			return false;
		}
		xrEnumerateSwapchainImages(swapchain, 0, &n, nullptr);
		images.assign(n, xr_struct<XrSwapchainImageVulkan2KHR>(XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR));
		return xr_ok(xrEnumerateSwapchainImages(swapchain, n, &n,
		                                        reinterpret_cast<XrSwapchainImageBaseHeader *>(images.data())),
		             "swapchain images");
	}

	//! The pattern into swapchain image @p index, left as a colour attachment as the runtime expects.
	void
	fill_image(uint32_t index)
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
		b.image = images[index].image;
		b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr,
		                     0, nullptr, 1, &b);
		VkBufferImageCopy copy = {};
		copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		copy.imageExtent = {kWidth, kHeight, 1};
		vkCmdCopyBufferToImage(cmd, staging, images[index].image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
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
			XrCompositionLayerQuad quad = {};
			quad.type = XR_TYPE_COMPOSITION_LAYER_QUAD;
			XrCompositionLayerHdrMNDX hdr = {};
			hdr.type = XR_TYPE_COMPOSITION_LAYER_HDR_MNDX;
			const XrCompositionLayerBaseHeader *layers[1] = {
			    reinterpret_cast<const XrCompositionLayerBaseHeader *>(&quad)};
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
					fill_image(index);
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
						hdr.primaries = XR_HDR_PRIMARIES_BT709_MNDX;
						hdr.referenceWhiteNits = kWhiteNits;
						hdr.maxContentLightLevelNits = mode.cll_nits;
						hdr.sceneMaxNits = mode.scene_nits;
						hdr.sceneAverageNits = mode.scene_nits > 0.0f ? kWhiteNits * 0.7f : 0.0f;
						quad.next = &hdr;
					}
					layer_count = 1;
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
		fprintf(stderr, "usage: %s <libopenxr_monado.so> [seconds, 60] [none|linear|scene:N|cll:N|pq:N|sdr]\n",
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
	if (!app.load_runtime(argv[1]) || !app.create_instance() || !app.create_vulkan() || !app.create_session()) {
		return 1;
	}
	const int ret = app.run(seconds);
	app.vkDeviceWaitIdle(app.device);
	app.xrDestroySwapchain(app.swapchain);
	app.xrDestroySpace(app.view_space);
	app.xrDestroySession(app.session);
	app.xrDestroyInstance(app.instance);
	return ret;
}
