// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  riftlinux-notices: shows the Rift driver's notices in the headset (a sensor moved, placed again), each as a
 *         panel in front of the view for a few seconds, and gives SteamVR's dashboard a "Rift" tab with the
 *         headphones' volume and mute, as the Oculus button's panel did on Windows (recentring is SteamVR's own
 *         button, or the Oculus button held) (SteamVR on Linux has no volume
 *         control). A SteamVR overlay application, started by the driver (ovrd_driver.cpp), which has no way of its
 *         own to show anything in the headset.
 *
 * Usage: riftlinux-notices <SteamVR's vrclient.so> <directory of the panels>
 * Reads one notice per line on stdin, the name of its panel without ".png" ("sensor-moved-2"), or a command from the
 * driver: "cmd-open-quick" (the right Touch's Oculus button: the dashboard on the Rift tab), "cmd-recentre" (the
 * button held: SteamVR's own seated recentre), "cmd-height <mm>" (the height above the floor now), "cmd-picture
 * <direct 0|1> <Auto-HDR 0|1> <strength %> <colour %> <smoothing 0|1> <HDR Bright 0|1>" (the picture's settings),
 * "cmd-hdr-content 0|1" (HDR content on screen now, shown at HDR Bright).
 * Answers on the same
 * socket "dashboard 1" / "dashboard 0" as SteamVR's dashboard opens and closes (the Oculus button then closes it),
 * "height <mm>" and "floor" from the tab's view card, "autohdr 0|1", "autohdr-strength <%>", "autohdr-colour <%>",
 * "hdr-bright 0|1" and "smoothing 0|1" from its picture and motion cards, "direct-mode 0|1", "render-scale <%>"
 * (both for SteamVR's next start) and "fast-path 0|1" from its display card. The driver also sends "cmd-status
 * key=value ..." (worn, controllers' batteries, sensors, the display path) and "cmd-display key=value ..." (direct
 * mode and render scale saved and running, the fast path) for the second page.
 * Ends when stdin closes (the driver is
 * gone) or SteamVR quits. The panels and the tab's pieces are rendered by
 * scripts/riftlinux/make_notice_images.py.
 */

#include "openvr.h"
#include "IVRClientCore_003.h"

#if __has_include(<vulkan/vulkan.h>)
#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#define RLN_HAVE_VULKAN 1
#endif

#if __has_include(<pulse/pulseaudio.h>)
#include <pulse/pulseaudio.h>
#define RLN_HAVE_PULSE 1
#endif

#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "riftlinux/rl_pulse.hpp"
#include "riftlinux/rl_tab.hpp"

#include <dlfcn.h>
#include <poll.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <string>
#include <vector>

namespace {

using namespace rl::tab;

//! How long a panel stays, and how long it takes to fade in and out.
constexpr double kShowS = 6.0;
//! The sensors' first measuring (two lines, shown again every 20 s while it lasts): longer, so it is read.
constexpr double kMeasuringShowS = 10.0;
constexpr double kFadeS = 0.3;
//! Where it sits: in front of the view, a little below it, metres; and how wide.
constexpr float kAheadM = 1.0f;
/*!
 * While SteamVR's dashboard is open: nearer, at the same size to the eye. SteamVR draws overlays by depth, and the
 * dashboard stays where it opened (low, tilted) while a notice follows the head: at 1 m a notice crossed the
 * dashboard's plane and showed half behind the Rift tab.
 */
constexpr float kAheadDashboardM = 0.6f;
constexpr float kBelowM = 0.18f;
constexpr float kWidthM = 0.6f;
//! Test patterns ("test-..." panels): straight ahead, filling most of the view, for longer.
constexpr double kTestShowS = 30.0;
constexpr float kTestWidthM = 1.6f;
//! The welcome panel (shown once after installing): larger, a little below the eyes, long enough to read.
constexpr double kWelcomeShowS = 20.0;
constexpr float kWelcomeWidthM = 1.1f;
constexpr float kWelcomeBelowM = 0.08f;
//! The volume shown in front of the view when it changes from elsewhere (the desktop, the headset's keys).
constexpr double kVolumeShowS = 2.0;
//! Notices waiting beyond this many are dropped, the oldest first.
constexpr size_t kMaxQueued = 6;
//! SteamVR may not take overlay apps yet when the driver starts this: tries for this long.
constexpr double kConnectForS = 60.0;

double
now_s()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

//! Only the names make_notice_images.py writes: letters, digits and dashes.
bool
valid_name(const std::string &name)
{
	return !name.empty() && name.size() < 64 && std::all_of(name.begin(), name.end(), [](char c) {
		       return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-';
	       });
}


//! Steam's main page in SteamVR's dashboard, where its menu button normally opens it.
constexpr const char *kSteamMainTab = "valve.steam.gamepadui.main";
//! In SteamVR's dashboard (text about as large as before at 1200 px over 2 m); and the volume shown in front of the
//! view when it changes from elsewhere.
constexpr float kQuickWidthM = 2.6f;
constexpr float kVolumeWidthM = 0.6f;
//! Hands @p img to SteamVR as pixels; a file it reloads blanks the overlay meanwhile (the tab flickered). A file if
//! SteamVR refuses the pixels (it bounds their size).
bool
set_overlay_image(vr::IVROverlay *overlay, vr::VROverlayHandle_t handle, Image &img, bool &raw_ok);

//! Writes @p img for SetOverlayFromFile, under a new name each time of a few (SteamVR may keep a file it loaded), in
//! ~/.cache/riftlinux: home is the same inside and outside Valve's runtime container, its private directories are not.
std::string
write_png(const Image &img)
{
	static unsigned seq = 0;
	const char *cache = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	const std::string dir = (cache != nullptr && cache[0] == '/' ? std::string(cache)
	                                                              : std::string(home != nullptr ? home : "/tmp") + "/.cache") +
	                        "/riftlinux";
	mkdir(dir.c_str(), 0700);
	const std::string path = dir + "/quick-" + std::to_string(seq++ % 4) + ".png";
	if (stbi_write_png(path.c_str(), img.w, img.h, 4, img.px.data(), img.w * 4) == 0) {
		fprintf(stderr, "riftlinux-notices: could not write %s\n", path.c_str());
		return "";
	}
	return path;
}


bool
set_overlay_image(vr::IVROverlay *overlay, vr::VROverlayHandle_t handle, Image &img, bool &raw_ok)
{
	if (raw_ok) {
		const vr::EVROverlayError err = overlay->SetOverlayRaw(handle, img.px.data(), static_cast<uint32_t>(img.w),
		                                                       static_cast<uint32_t>(img.h), 4);
		if (err == vr::VROverlayError_None) {
			return true;
		}
		fprintf(stderr, "riftlinux-notices: SteamVR took no pixels (%d), images from files\n", static_cast<int>(err));
		raw_ok = false;
	}
	const std::string png = write_png(img);
	return !png.empty() && overlay->SetOverlayFromFile(handle, png.c_str()) == vr::VROverlayError_None;
}


#ifdef RLN_HAVE_VULKAN
/*!
 * The tab's image as a Vulkan image of our own, handed over with SetOverlayTexture: SteamVR swaps it in whole, where
 * pixels (SetOverlayRaw) or a file make it rebuild the texture and the tab flickered while the slider moved. Two
 * images in turn, so the one SteamVR may still be reading is never written. libvulkan loaded at run time.
 */
struct VkTab
{
	PFN_vkGetInstanceProcAddr gipa = nullptr;
#define RLN_VK_FUNCS(X)                                                                                                \
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
	X(vkQueueSubmit)
#define RLN_VK_DECL(name) PFN_##name name = nullptr;
	RLN_VK_FUNCS(RLN_VK_DECL)
#undef RLN_VK_DECL

	VkInstance instance = VK_NULL_HANDLE;
	VkPhysicalDevice phys = VK_NULL_HANDLE;
	VkDevice device = VK_NULL_HANDLE;
	VkQueue queue = VK_NULL_HANDLE;
	uint32_t family = 0;
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	VkFence fence = VK_NULL_HANDLE;
	VkImage images[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
	bool used[2] = {false, false};
	int next = 0;
	VkBuffer staging = VK_NULL_HANDLE;
	void *mapped = nullptr;
	int w = 0;
	int h = 0;
	bool ok = false;

	static std::vector<std::string>
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
	init(vr::IVRSystem *system, vr::IVRCompositor *compositor, int width, int height)
	{
		w = width;
		h = height;
		void *lib = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
		gipa = lib != nullptr ? reinterpret_cast<PFN_vkGetInstanceProcAddr>(dlsym(lib, "vkGetInstanceProcAddr"))
		                      : nullptr;
		if (gipa == nullptr || compositor == nullptr) {
			return fail("no libvulkan.so.1 or no compositor interface");
		}
		vkCreateInstance = reinterpret_cast<PFN_vkCreateInstance>(gipa(VK_NULL_HANDLE, "vkCreateInstance"));

		// The extensions SteamVR needs to share the image.
		std::string text(compositor->GetVulkanInstanceExtensionsRequired(nullptr, 0) + 1, '\0');
		compositor->GetVulkanInstanceExtensionsRequired(text.data(), static_cast<uint32_t>(text.size()));
		std::vector<std::string> names = split(text.c_str());
		std::vector<const char *> exts;
		for (const std::string &n : names) {
			exts.push_back(n.c_str());
		}
		VkApplicationInfo app = {};
		app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
		app.pApplicationName = "riftlinux-notices";
		app.apiVersion = VK_API_VERSION_1_1;
		VkInstanceCreateInfo ici = {};
		ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
		ici.pApplicationInfo = &app;
		ici.enabledExtensionCount = static_cast<uint32_t>(exts.size());
		ici.ppEnabledExtensionNames = exts.data();
		if (vkCreateInstance == nullptr || vkCreateInstance(&ici, nullptr, &instance) != VK_SUCCESS) {
			return fail("vkCreateInstance");
		}
#define RLN_VK_LOAD(name) name = reinterpret_cast<PFN_##name>(gipa(instance, #name));
		RLN_VK_FUNCS(RLN_VK_LOAD)
#undef RLN_VK_LOAD

		// The GPU SteamVR renders on.
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
		const float priority = 0.0f;
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

		// The two images, sRGB as the pieces are drawn.
		for (VkImage &image : images) {
			VkImageCreateInfo ii = {};
			ii.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
			ii.imageType = VK_IMAGE_TYPE_2D;
			ii.format = VK_FORMAT_R8G8B8A8_SRGB;
			ii.extent = {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1};
			ii.mipLevels = 1;
			ii.arrayLayers = 1;
			ii.samples = VK_SAMPLE_COUNT_1_BIT;
			ii.tiling = VK_IMAGE_TILING_OPTIMAL;
			ii.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
			ii.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			VkMemoryRequirements req;
			VkDeviceMemory mem = VK_NULL_HANDLE;
			if (vkCreateImage(device, &ii, nullptr, &image) != VK_SUCCESS ||
			    (vkGetImageMemoryRequirements(device, image, &req),
			     !allocate(req, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &mem)) ||
			    vkBindImageMemory(device, image, mem, 0) != VK_SUCCESS) {
				return fail("image");
			}
		}
		VkBufferCreateInfo bi = {};
		bi.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bi.size = static_cast<VkDeviceSize>(w) * h * 4;
		bi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
		VkMemoryRequirements req;
		VkDeviceMemory mem = VK_NULL_HANDLE;
		if (vkCreateBuffer(device, &bi, nullptr, &staging) != VK_SUCCESS ||
		    (vkGetBufferMemoryRequirements(device, staging, &req),
		     !allocate(req, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &mem)) ||
		    vkBindBufferMemory(device, staging, mem, 0) != VK_SUCCESS ||
		    vkMapMemory(device, mem, 0, bi.size, 0, &mapped) != VK_SUCCESS) {
			return fail("staging buffer");
		}
		ok = true;
		return true;
	}

	bool
	fail(const char *what)
	{
		fprintf(stderr, "riftlinux-notices: no Vulkan image for the tab (%s), pixels instead\n", what);
		ok = false;
		return false;
	}

	//! Puts @p img (the tab's size) into the next image and hands it to SteamVR.
	bool
	show(vr::IVROverlay *overlay, vr::VROverlayHandle_t handle, const Image &img)
	{
		if (!ok || img.w != w || img.h != h) {
			return false;
		}
		memcpy(mapped, img.px.data(), img.px.size());
		VkImage image = images[next];
		const bool was_used = used[next];
		used[next] = true;
		next ^= 1;

		vkResetCommandBuffer(cmd, 0);
		VkCommandBufferBeginInfo begin = {};
		begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
		vkBeginCommandBuffer(cmd, &begin);
		VkImageMemoryBarrier barrier = {};
		barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		barrier.srcAccessMask = 0;
		barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		barrier.oldLayout = was_used ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED;
		barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		barrier.image = image;
		barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
		vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr,
		                     0, nullptr, 1, &barrier);
		VkBufferImageCopy copy = {};
		copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		copy.imageExtent = {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1};
		vkCmdCopyBufferToImage(cmd, staging, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
		// SteamVR takes images in the transfer source layout.
		barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
		barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
		                     nullptr, 1, &barrier);
		vkEndCommandBuffer(cmd);
		VkSubmitInfo submit = {};
		submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		submit.commandBufferCount = 1;
		submit.pCommandBuffers = &cmd;
		vkResetFences(device, 1, &fence);
		if (vkQueueSubmit(queue, 1, &submit, fence) != VK_SUCCESS ||
		    vkWaitForFences(device, 1, &fence, VK_TRUE, 1000000000ull) != VK_SUCCESS) {
			return fail("upload");
		}

		vr::VRVulkanTextureData_t data = {};
		data.m_nImage = reinterpret_cast<uint64_t>(image);
		data.m_pDevice = device;
		data.m_pPhysicalDevice = phys;
		data.m_pInstance = instance;
		data.m_pQueue = queue;
		data.m_nQueueFamilyIndex = family;
		data.m_nWidth = static_cast<uint32_t>(w);
		data.m_nHeight = static_cast<uint32_t>(h);
		data.m_nFormat = VK_FORMAT_R8G8B8A8_SRGB;
		data.m_nSampleCount = 1;
		vr::Texture_t texture = {&data, vr::TextureType_Vulkan, vr::ColorSpace_Auto};
		const vr::EVROverlayError err = overlay->SetOverlayTexture(handle, &texture);
		if (err != vr::VROverlayError_None) {
			fprintf(stderr, "riftlinux-notices: SteamVR took no Vulkan image (%d), pixels instead\n",
			        static_cast<int>(err));
			ok = false;
			return false;
		}
		return true;
	}
};
#endif


/*
 *
 * The volume: the desktop's sound server through libpulse, loaded at run time as the driver does (SteamVR runs in
 * Valve's runtime container, which has the library and the server's socket). The default output, which the driver
 * set to the Rift's headphones.
 *
 */

#ifdef RLN_HAVE_PULSE
using rl::Pulse;
#endif

} // namespace

/*!
 * Motion smoothing's frame lock: with the Rift tab's Smoothing on, the game in front is held to 90,
 * 45 or 30 frames a second, as Oculus' ASW holds a game that cannot keep up at 45: an even rhythm, every refresh
 * between two of its frames made by motion smoothing. Chosen from SteamVR's timings of the game (the GPU and CPU time
 * of its frames, the 80th percentile of the last second, and the intervals between its frames): see tick(). Set through SteamVR's own
 * per-application throttling (framesToThrottle, and additionalFramesToPredict so the game renders for the time its
 * frame is shown), which the driver honours as a direct mode driver with vsync events; the user's own values come back
 * when Smoothing is turned off, the game changes, or SteamVR closes.
 */
/*!
 * The Rift tab's Performance card: the game's frames a second now and over the last 10 s, its GPU time a frame, and
 * its frames shown later than SteamVR planned (judder), from SteamVR's own timings of the game's frames. One read a
 * second, and only while the card's switch is on.
 */
class Perf
{
public:
	explicit Perf(vr::IVRCompositor *compositor) : m_compositor(compositor) {}

	//! Every loop; reads once a second while @p on, else forgets (the next start counts afresh).
	void
	tick(double now, bool on, TabInfo &info)
	{
		info.perf_on = on;
		if (!on || m_compositor == nullptr) {
			m_last_index = 0;
			m_window.clear();
			info.perf_fps = info.perf_avg = info.perf_gpu_x10 = info.perf_dropped = -1;
			return;
		}
		if (now < m_next_s) {
			return;
		}
		m_next_s = now + 1.0;
		vr::Compositor_FrameTiming timings[128];
		for (auto &t : timings) {
			t.m_nSize = sizeof(t);
		}
		const uint32_t n = m_compositor->GetFrameTimings(timings, 128);
		int frames = 0, late = 0, gpu_n = 0;
		float gpu = 0.0f;
		uint32_t newest = m_last_index;
		for (uint32_t i = 0; i < n; i++) {
			const auto &t = timings[i];
			if (t.m_nFrameIndex <= m_last_index) {
				continue;
			}
			frames++;
			late += (int)t.m_nNumMisPresented;
			const float g = t.m_flPreSubmitGpuMs + t.m_flPostSubmitGpuMs;
			if (g > 0.0f) {
				gpu += g;
				gpu_n++;
			}
			newest = std::max(newest, t.m_nFrameIndex);
		}
		const bool first = m_last_index == 0;
		const double dt = now - m_last_s;
		m_last_index = newest;
		m_last_s = now;
		// The first read marks where to count from.
		if (first || dt <= 0.0) {
			return;
		}
		m_window.push_back({frames, dt, late});
		while (m_window.size() > 10) {
			m_window.pop_front();
		}
		int sum_frames = 0, sum_late = 0;
		double sum_dt = 0.0;
		for (const Second &w : m_window) {
			sum_frames += w.frames;
			sum_dt += w.dt;
			sum_late += w.late;
		}
		info.perf_fps = (int)std::lround(frames / dt);
		info.perf_avg = (int)std::lround(sum_frames / sum_dt);
		info.perf_dropped = sum_late;
		info.perf_gpu_x10 = gpu_n > 0 ? (int)std::lround(10.0f * gpu / (float)gpu_n) : -1;
	}

private:
	struct Second
	{
		int frames;
		double dt;
		int late;
	};
	vr::IVRCompositor *m_compositor = nullptr;
	double m_next_s = 0.0, m_last_s = 0.0;
	uint32_t m_last_index = 0;
	std::deque<Second> m_window;
};

class FrameLock
{
public:
	FrameLock(vr::IVRCompositor *compositor, vr::IVRSettings *settings, vr::IVRApplications *applications)
	    : m_compositor(compositor), m_settings(settings), m_applications(applications)
	{
		// A lock left behind by a SteamVR that did not close cleanly: the game's own values back first.
		FILE *f = fopen(state_path().c_str(), "r");
		if (f != nullptr) {
			char key[vr::k_unMaxApplicationKeyLength] = {};
			int had_throttle = 0, old_throttle = 0, had_predict = 0, old_predict = 0;
			if (fscanf(f, "%127s %d %d %d %d", key, &had_throttle, &old_throttle, &had_predict, &old_predict) == 5 &&
			    m_settings != nullptr) {
				m_key = key;
				m_had_throttle = had_throttle != 0;
				m_old_throttle = old_throttle;
				m_had_predict = had_predict != 0;
				m_old_predict = old_predict;
				fprintf(stderr, "riftlinux-notices: frame lock: left from last time, ");
				restore();
			}
			fclose(f);
		}
	}

	~FrameLock()
	{
		restore();
	}


	void
	tick(double now, bool smoothing)
	{
		trace();
		if (m_compositor == nullptr || m_settings == nullptr || m_applications == nullptr || now < m_next_s) {
			return;
		}
		m_next_s = now + 1.0;

		std::string key = scene_key();
		if (!smoothing || key.empty() || key != m_key) {
			restore();
		}
		if (!smoothing || key.empty()) {
			// Its own times are logged without the lock too (what a game needs, Smoothing on or off).
			if (!key.empty()) {
				log_timings(now, key);
			}
			return;
		}

		// The game's need per frame, from SteamVR's timings of its last frames.
		vr::Compositor_FrameTiming timings[90];
		for (auto &t : timings) {
			t.m_nSize = sizeof(t);
		}
		const uint32_t n = m_compositor->GetFrameTimings(timings, 90);
		std::vector<float> need;
		std::vector<float> interval;
		std::vector<float> gpus, cpus;
		for (uint32_t i = 0; i < n; i++) {
			const auto &t = timings[i];
			const float gpu = t.m_flPreSubmitGpuMs + t.m_flPostSubmitGpuMs;
			const float cpu = t.m_flNewFrameReadyMs > t.m_flNewPosesReadyMs
			                      ? t.m_flNewFrameReadyMs - t.m_flNewPosesReadyMs
			                      : 0.0f;
			if (gpu > 0.0f || cpu > 0.0f) {
				need.push_back(std::max(gpu, cpu));
				gpus.push_back(gpu);
				cpus.push_back(cpu);
			}
			if (t.m_flClientFrameIntervalMs > 0.0f) {
				interval.push_back(t.m_flClientFrameIntervalMs);
			}
		}
		const float period_ms = 1000.0f / 90.0f;
		int target = m_level;
		float need_ms = 0.0f;
		if (need.size() >= 10) {
			std::sort(need.begin(), need.end());
			need_ms = need[need.size() * 8 / 10];
			/*
			 * At 90 the game's times are its own: slower once it clearly misses (over 102 % of a frame). Held to a
			 * lower rate its GPU and CPU clock down and its times grow (Home 6.5 ms at 90, 10 at 45): there it
			 * goes slower only when it really misses its slots (frame intervals past half a refresh late), and
			 * faster only with 5 % of room in those grown times (so near a limit it stays).
			 */
			int misses = 0;
			for (float iv : interval) {
				misses += iv > ((float)m_level + 0.5f) * period_ms;
			}
			const bool missing = interval.size() >= 10 && misses * 5 > (int)interval.size();
			if (m_level == 1) {
				target = need_ms > 2.04f * period_ms ? 3 : need_ms > 1.02f * period_ms ? 2 : 1;
			} else if (missing) {
				// Slower only when slots are missed in two checks in a row (2 s), not after one hitch: each
				// change shifts what the game predicts, felt as the picture settling.
				m_missing_count++;
				target = m_missing_count >= 2 ? std::min(m_level + 1, 3) : m_level;
			} else if (need_ms < 0.9f * (float)(m_level - 1) * period_ms) {
				m_missing_count = 0;
				target = m_level - 1;
			} else {
				m_missing_count = 0;
				target = m_level;
			}
		} else if (interval.size() >= 10 && m_level == 1) {
			// No GPU or CPU times: from the game's frame interval, only towards a slower rate.
			std::sort(interval.begin(), interval.end());
			const float mid = interval[interval.size() / 2];
			target = mid > 1.15f * period_ms ? (mid > 2.15f * period_ms ? 3 : 2) : 1;
		}

		// Test: a fixed rate (settings.json display.motion_lock_test, 1-3).
		static const int forced = getenv("RIFTLINUX_FRAME_LOCK_TEST") != nullptr
		                              ? std::clamp(atoi(getenv("RIFTLINUX_FRAME_LOCK_TEST")), 0, 3)
		                              : 0;
		if (forced > 0) {
			target = forced;
		}
		if (target > m_level) {
			m_faster_since = -1.0;
			m_missing_count = 0;
			m_changed_s = now;
			set(key, target, need_ms);
		} else if (target < m_level) {
			// Faster only with 10 % of room for 10 s, and not within 15 s of the last change (NMS at 18-31 ms a
			// frame switched 45 <-> 30 every few seconds with 3 s and 5 %).
			if (m_faster_since < 0.0) {
				m_faster_since = now;
			} else if (now - m_faster_since >= 10.0 && (forced > 0 || now - m_changed_s >= 15.0)) {
				m_faster_since = -1.0;
				m_changed_s = now;
				set(key, target, need_ms);
			}
		} else {
			m_faster_since = -1.0;
		}

		if (now - m_logged_s >= 10.0 && !gpus.empty()) {
			m_logged_s = now;
			std::sort(gpus.begin(), gpus.end());
			std::sort(cpus.begin(), cpus.end());
			fprintf(stderr,
			        "riftlinux-notices: frame lock: %s at %d frames a second (game needs %.1f ms a frame: GPU %.1f, "
			        "CPU %.1f; %zu timings)\n",
			        key.c_str(), 90 / m_level, need_ms, gpus[gpus.size() * 8 / 10], cpus[cpus.size() * 8 / 10],
			        need.size());
		}
	}

	//! The user's own values back for the game that was held.
	void
	restore()
	{
		if (m_key.empty() || m_settings == nullptr) {
			m_key.clear();
			m_level = 1;
			return;
		}
		put(m_key, vr::k_pch_SteamVR_FramesToThrottle_Int32, m_had_throttle, m_old_throttle);
		put(m_key, vr::k_pch_SteamVR_AdditionalFramesToPredict_Int32, m_had_predict, m_old_predict);
		fprintf(stderr, "riftlinux-notices: frame lock: %s back to its own settings\n", m_key.c_str());
		write_state(false);
		m_key.clear();
		m_level = 1;
		m_faster_since = -1.0;
		m_missing_count = 0;
	}

private:
	//! Every 10 s, the game's GPU and CPU time a frame (80th percentile) and its frame interval (median), unlocked.
	void
	log_timings(double now, const std::string &key)
	{
		if (now - m_logged_s < 10.0) {
			return;
		}
		vr::Compositor_FrameTiming timings[90];
		for (auto &t : timings) {
			t.m_nSize = sizeof(t);
		}
		const uint32_t n = m_compositor->GetFrameTimings(timings, 90);
		std::vector<float> gpus, cpus, interval;
		for (uint32_t i = 0; i < n; i++) {
			const auto &t = timings[i];
			const float gpu = t.m_flPreSubmitGpuMs + t.m_flPostSubmitGpuMs;
			const float cpu = t.m_flNewFrameReadyMs > t.m_flNewPosesReadyMs
			                      ? t.m_flNewFrameReadyMs - t.m_flNewPosesReadyMs
			                      : 0.0f;
			if (gpu > 0.0f || cpu > 0.0f) {
				gpus.push_back(gpu);
				cpus.push_back(cpu);
			}
			if (t.m_flClientFrameIntervalMs > 0.0f) {
				interval.push_back(t.m_flClientFrameIntervalMs);
			}
		}
		if (gpus.empty()) {
			return;
		}
		m_logged_s = now;
		std::sort(gpus.begin(), gpus.end());
		std::sort(cpus.begin(), cpus.end());
		std::sort(interval.begin(), interval.end());
		fprintf(stderr,
		        "riftlinux-notices: game timing: %s unlocked: GPU %.1f, CPU %.1f ms a frame (80 %%), frame interval "
		        "%.1f ms (median); %zu timings\n",
		        key.c_str(), gpus[gpus.size() * 8 / 10], cpus[cpus.size() * 8 / 10],
		        interval.empty() ? 0.0f : interval[interval.size() / 2], gpus.size());
	}

	/*!
	 * Test (settings.json display.motion_pacing_trace, as the driver's pacing trace): SteamVR's timings of the
	 * game's frames, one line each, into ~/.cache/riftlinux/pacing-app.txt, with this clock (CLOCK_MONOTONIC) next to
	 * SteamVR's system time of the newest frame, to line them up with the driver's trace.
	 */
	void
	trace()
	{
		static const int want = getenv("RIFTLINUX_PACING_TRACE") != nullptr
		                            ? std::clamp(atoi(getenv("RIFTLINUX_PACING_TRACE")), 0, 200000) / 8
		                            : 0;
		static const double after_s = getenv("RIFTLINUX_PACING_TRACE_AFTER_S") != nullptr
		                                  ? atof(getenv("RIFTLINUX_PACING_TRACE_AFTER_S"))
		                                  : 30.0;
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		const double mono = (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
		if (want == 0 || m_compositor == nullptr || m_trace_done) {
			return;
		}
		if (m_trace_from < 0.0) {
			m_trace_from = mono + after_s;
		}
		if (mono < m_trace_from || mono < m_trace_next) {
			return;
		}
		m_trace_next = mono + 0.25;
		vr::Compositor_FrameTiming timings[64];
		for (auto &t : timings) {
			t.m_nSize = sizeof(t);
		}
		const uint32_t n = m_compositor->GetFrameTimings(timings, 64);
		char line[512];
		for (uint32_t i = 0; i < n; i++) {
			const auto &t = timings[i];
			if (t.m_nFrameIndex <= m_trace_last) {
				continue;
			}
			m_trace_last = t.m_nFrameIndex;
			snprintf(line, sizeof(line),
			         "%u %.6f %u %u %u 0x%x %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %u %u\n",
			         t.m_nFrameIndex, t.m_flSystemTimeInSeconds, t.m_nNumFramePresents, t.m_nNumMisPresented,
			         t.m_nNumDroppedFrames, t.m_nReprojectionFlags, t.m_flClientFrameIntervalMs,
			         t.m_flWaitGetPosesCalledMs, t.m_flNewPosesReadyMs, t.m_flNewFrameReadyMs, t.m_flSubmitFrameMs,
			         t.m_flCompositorUpdateStartMs, t.m_flCompositorUpdateEndMs, t.m_flCompositorRenderStartMs,
			         t.m_flPresentCallCpuMs, t.m_flWaitForPresentCpuMs, t.m_flPreSubmitGpuMs, t.m_flPostSubmitGpuMs,
			         t.m_flTotalRenderGpuMs, t.m_nNumVSyncsReadyForUse, t.m_nNumVSyncsToFirstView);
			m_trace_lines += line;
			m_trace_count++;
		}
		if (n > 0 && m_trace_count > 0 && m_trace_count >= want) {
			m_trace_done = true;
			snprintf(line, sizeof(line), "# monotonic %.6f newest system time %.6f\n", mono,
			         timings[n - 1].m_flSystemTimeInSeconds);
			FILE *f = fopen((dir_path() + "/pacing-app.txt").c_str(), "w");
			if (f != nullptr) {
				fputs("# frame system_s presents mispresented dropped flags interval_ms waitgetposes_ms "
				      "posesready_ms frameready_ms submit_ms compupdate_start_ms compupdate_end_ms "
				      "comprender_start_ms presentcall_ms waitforpresent_ms presubmit_gpu_ms postsubmit_gpu_ms "
				      "total_gpu_ms vsyncs_ready vsyncs_to_first_view\n",
				      f);
				fputs(line, f);
				fputs(m_trace_lines.c_str(), f);
				fclose(f);
			}
			fprintf(stderr, "riftlinux-notices: pacing trace: %d frames written\n", m_trace_count);
		}
	}

	std::string
	scene_key()
	{
		const uint32_t pid = m_applications->GetCurrentSceneProcessId();
		if (pid == 0) {
			return {};
		}
		char key[vr::k_unMaxApplicationKeyLength] = {};
		if (m_applications->GetApplicationKeyByProcessId(pid, key, sizeof(key)) != vr::VRApplicationError_None) {
			return {};
		}
		return key;
	}

	void
	set(const std::string &key, int level, float need_ms)
	{
		if (m_key != key) {
			// First change for this game: its own values, to put back.
			vr::EVRSettingsError e = vr::VRSettingsError_None;
			m_old_throttle = m_settings->GetInt32(key.c_str(), vr::k_pch_SteamVR_FramesToThrottle_Int32, &e);
			m_had_throttle = e == vr::VRSettingsError_None;
			m_old_predict = m_settings->GetInt32(key.c_str(), vr::k_pch_SteamVR_AdditionalFramesToPredict_Int32, &e);
			m_had_predict = e == vr::VRSettingsError_None;
			m_key = key;
		}
		write_state(true);
		m_settings->SetInt32(key.c_str(), vr::k_pch_SteamVR_FramesToThrottle_Int32, level - 1, nullptr);
		m_settings->SetInt32(key.c_str(), vr::k_pch_SteamVR_AdditionalFramesToPredict_Int32, level - 1, nullptr);
		fprintf(stderr, "riftlinux-notices: frame lock: %s %d -> %d frames a second (game needs %.1f ms a frame)\n",
		        key.c_str(), 90 / m_level, 90 / level, need_ms);
		m_level = level;
	}

	void
	put(const std::string &key, const char *name, bool had, int32_t old)
	{
		if (had) {
			m_settings->SetInt32(key.c_str(), name, old, nullptr);
		} else {
			m_settings->RemoveKeyInSection(key.c_str(), name, nullptr);
		}
	}

	static std::string
	dir_path()
	{
		const char *cache = getenv("XDG_CACHE_HOME");
		const char *home = getenv("HOME");
		std::string dir = cache != nullptr && cache[0] != '\0' ? std::string(cache) + "/riftlinux"
		                                                        : std::string(home != nullptr ? home : "/tmp") + "/.cache/riftlinux";
		mkdir(dir.c_str(), 0755);
		return dir;
	}

	//! What to put back, kept on disk while a game is held (empty when none).
	static std::string
	state_path()
	{
		return dir_path() + "/frame-lock.txt";
	}

	void
	write_state(bool held)
	{
		FILE *f = fopen(state_path().c_str(), "w");
		if (f == nullptr) {
			return;
		}
		if (held) {
			fprintf(f, "%s %d %d %d %d\n", m_key.c_str(), m_had_throttle ? 1 : 0, m_old_throttle,
			        m_had_predict ? 1 : 0, m_old_predict);
		}
		fclose(f);
	}

	vr::IVRCompositor *m_compositor;
	vr::IVRSettings *m_settings;
	vr::IVRApplications *m_applications;
	double m_next_s = 0.0;
	double m_logged_s = 0.0;
	double m_faster_since = -1.0;
	double m_changed_s = -1e9;
	int m_missing_count = 0;
	int m_level = 1;
	std::string m_key;
	bool m_had_throttle = false, m_had_predict = false;
	int32_t m_old_throttle = 0, m_old_predict = 0;
	double m_trace_from = -1.0, m_trace_next = 0.0;
	uint32_t m_trace_last = 0;
	int m_trace_count = 0;
	bool m_trace_done = false;
	std::string m_trace_lines;
};

int
main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "Usage: riftlinux-notices <vrclient.so> <panel directory>\n");
		return 2;
	}
	const std::string panels = argv[2];

	// SteamVR's client, loaded afresh for each try as OpenVR's own VR_Init does (it unloads it after a failed one).
	void *lib = nullptr;
	vr::IVRClientCore_003 *core = nullptr;
	auto load = [&]() {
		lib = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
		if (lib == nullptr) {
			fprintf(stderr, "riftlinux-notices: %s\n", dlerror());
			return false;
		}
		using Factory = void *(*)(const char *, int *);
		auto factory = reinterpret_cast<Factory>(dlsym(lib, "VRClientCoreFactory"));
		int rc = 0;
		core = factory != nullptr ? static_cast<vr::IVRClientCore_003 *>(factory(vr::IVRClientCore_003_Version, &rc))
		                          : nullptr;
		if (core == nullptr) {
			fprintf(stderr, "riftlinux-notices: no IVRClientCore_003 in %s\n", argv[1]);
			return false;
		}
		return true;
	};
	if (!load()) {
		return 1;
	}

	/*
	 * Started with SteamVR: it may not take overlay apps for a while yet (its compositor not up). A try every 2 s,
	 * each with the client loaded afresh: before, after one failed try every later one failed too, and the wait on
	 * the driver's socket returned at once (the driver had sent the tab's settings), so it tried in a tight loop,
	 * 266 connections in one slow start, then gave up: no Rift tab, notices or frame lock. Only the
	 * driver going away ends the wait early.
	 */
	vr::EVRInitError err = vr::VRInitError_None;
	int failed_tries = 0;
	for (const double until = now_s() + kConnectForS;;) {
		err = core->Init(vr::VRApplication_Overlay, nullptr);
		if (err == vr::VRInitError_None || now_s() > until) {
			break;
		}
		if (failed_tries++ == 0) {
			fprintf(stderr, "riftlinux-notices: SteamVR said %s, trying again every 2 s\n",
			        core->GetIDForVRInitError(err));
		}
		dlclose(lib);
		lib = nullptr;
		core = nullptr;
		struct pollfd pfd = {STDIN_FILENO, 0, 0};
		if (poll(&pfd, 1, 2000) > 0 && (pfd.revents & (POLLHUP | POLLERR)) != 0) {
			return 1; // The driver is gone already.
		}
		if (!load()) {
			return 1;
		}
	}
	if (err != vr::VRInitError_None) {
		fprintf(stderr, "riftlinux-notices: SteamVR said %s\n", core->GetIDForVRInitError(err));
		return 1;
	}
	auto *overlay = static_cast<vr::IVROverlay *>(core->GetGenericInterface(vr::IVROverlay_Version, &err));
	auto *system = static_cast<vr::IVRSystem *>(core->GetGenericInterface(vr::IVRSystem_Version, &err));
	FrameLock frame_lock(
	    static_cast<vr::IVRCompositor *>(core->GetGenericInterface(vr::IVRCompositor_Version, &err)),
	    static_cast<vr::IVRSettings *>(core->GetGenericInterface(vr::IVRSettings_Version, &err)),
	    static_cast<vr::IVRApplications *>(core->GetGenericInterface(vr::IVRApplications_Version, &err)));
	auto *chaperone = static_cast<vr::IVRChaperone *>(core->GetGenericInterface(vr::IVRChaperone_Version, &err));
	[[maybe_unused]] Perf perf(static_cast<vr::IVRCompositor *>(core->GetGenericInterface(vr::IVRCompositor_Version, &err)));
	// To the driver, on the socket it gave as stdin; never blocks, never raises SIGPIPE.
	[[maybe_unused]] auto tell_driver = [](const char *line) {
		const std::string text = std::string(line) + "\n";
		if (send(STDIN_FILENO, text.data(), text.size(), MSG_NOSIGNAL | MSG_DONTWAIT) < 0) {
			fprintf(stderr, "riftlinux-notices: could not tell the driver \"%s\"\n", line);
		}
	};
	vr::VROverlayHandle_t handle = vr::k_ulOverlayHandleInvalid;
	if (overlay == nullptr || system == nullptr ||
	    overlay->CreateOverlay("riftlinux.notices", "Rift CV1 notices", &handle) != vr::VROverlayError_None) {
		fprintf(stderr, "riftlinux-notices: no overlay\n");
		core->Cleanup();
		return 1;
	}
	// The last panel's size and drop at 1 m, kept to place it again nearer or back as the dashboard opens and closes.
	float placed_width = kWidthM, placed_below = kBelowM;
	bool near_dashboard = false;
	auto place = [&](float width, float below) {
		placed_width = width;
		placed_below = below;
		const float ahead = near_dashboard ? kAheadDashboardM : kAheadM;
		const float k = ahead / kAheadM;
		overlay->SetOverlayWidthInMeters(handle, width * k);
		vr::HmdMatrix34_t in_front = {
		    {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, -below * k}, {0.0f, 0.0f, 1.0f, -ahead}}};
		overlay->SetOverlayTransformTrackedDeviceRelative(handle, vr::k_unTrackedDeviceIndex_Hmd, &in_front);
	};
	place(kWidthM, kBelowM);
	fprintf(stderr, "riftlinux-notices: ready, panels in %s\n", panels.c_str());

	// The "Rift" tab in SteamVR's dashboard: sound, view, picture, motion.
	Pieces pieces;
	Layout layout;
	[[maybe_unused]] vr::VROverlayHandle_t quick = vr::k_ulOverlayHandleInvalid;
	[[maybe_unused]] vr::VROverlayHandle_t quick_icon = vr::k_ulOverlayHandleInvalid;
#ifdef RLN_HAVE_PULSE
	Pulse pulse;
#ifdef RLN_HAVE_VULKAN
	VkTab vk_tab;
#endif
	const bool have_pieces = pieces.load(panels);
	if (have_pieces) {
		layout = tab_layout(pieces);
	}
	if (!have_pieces) {
		fprintf(stderr, "riftlinux-notices: no tab pieces in %s, no Rift tab in the dashboard\n", panels.c_str());
	} else if (pulse.load() &&
	           overlay->CreateDashboardOverlay("riftlinux.quick", "Rift", &quick, &quick_icon) ==
	               vr::VROverlayError_None) {
		overlay->SetOverlayWidthInMeters(quick, kQuickWidthM);
		overlay->SetOverlayInputMethod(quick, vr::VROverlayInputMethod_Mouse);
		vr::HmdVector2_t scale = {static_cast<float>(kTabW), static_cast<float>(layout.h)};
		overlay->SetOverlayMouseScale(quick, &scale);
		overlay->SetOverlayFlag(quick, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);
		overlay->SetOverlayFromFile(quick_icon, (panels + "/quick-tab.png").c_str());
		fprintf(stderr, "riftlinux-notices: Rift tab in the dashboard\n");
#ifdef RLN_HAVE_VULKAN
		auto *compositor =
		    static_cast<vr::IVRCompositor *>(core->GetGenericInterface(vr::IVRCompositor_Version, &err));
		vk_tab.init(system, compositor, kTabW, layout.h);
#endif
	}
	bool quick_visible = false;
	bool quick_drawn = false;
	bool quick_ever_drawn = false;
	const double quick_since = now_s();
	bool raw_ok = true;
	bool dashboard_was = false;
	double opened_by_us_at = -10.0;
	double dashboard_opened_at = -10.0;
	//! What the tab shows now: drawn again only when it changes.
	TabState drawn;
	drawn.volume = -2;
	//! The floor's height setting as the driver has it, centimetres.
	int height_cm = 0;
	//! The picture's settings as the driver has them; their sliders while held.
	PictureRow picture;
	//! The headset's state and the display settings as the driver has them; the page shown; the render scale's
	//! slider while held.
	TabInfo info;
	int page = 0;
	bool dragging_scale = false;
	bool dragging_strength = false;
	double strength_sent_at = 0.0;
	bool dragging_colour = false;
	double colour_sent_at = 0.0;
	double floor_lit_until = 0.0;
	bool dragging_height = false;
	int drag_height_cm = 0;
	double height_sent_at = 0.0;
	bool dragging = false;
	//! The trigger's press on the tab: what it is on, and a grabbed knob (rl::tab::Press).
	Press press;
	//! The microphone's slider held, and where.
	bool dragging_mic = false;
	int drag_mic_percent = -1;
	double drag_mic_sent_at = 0.0;
	double drawn_at = 0.0;
	double drag_sent_at = 0.0;
	int drag_percent = -1;
#endif

	std::deque<std::string> queue;
	std::string partial;
	bool input_open = true;
	double shown_at = -1.0;
	double show_s = kShowS;
	[[maybe_unused]] bool showing_volume = false;
	//! The notice up says what is happening now (measuring the sensors, a sensor moved): it stays while that lasts.
	bool showing_lasting = false;
	std::string showing_name;

	while (true) {
		// New notices from the driver.
		struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
		if (input_open && poll(&pfd, 1, 30) > 0) {
			char buf[512];
			const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
			if (n <= 0) {
				input_open = false;
			} else {
				partial.append(buf, static_cast<size_t>(n));
				size_t eol;
				while ((eol = partial.find('\n')) != std::string::npos) {
					std::string name = partial.substr(0, eol);
					partial.erase(0, eol + 1);
#ifdef RLN_HAVE_PULSE
					// The Rift tab's state (the tab needs PulseAudio for its volume; built without it, no tab).
					if (name.rfind("cmd-height ", 0) == 0) {
						height_cm = (int)std::lround(atof(name.c_str() + 11) / 10.0);
						continue;
					}
					if (name.rfind("cmd-hdr-content ", 0) == 0) {
						picture.hdr_content = atoi(name.c_str() + 16) != 0;
						continue;
					}
					// "cmd-status key=value ..." and "cmd-display key=value ...": the second page's cards.
					if (name.rfind("cmd-status ", 0) == 0 || name.rfind("cmd-display ", 0) == 0) {
						size_t at = name.find(' ');
						while (at != std::string::npos) {
							const size_t next = name.find(' ', at + 1);
							const std::string kv = name.substr(at + 1, next == std::string::npos ? std::string::npos : next - at - 1);
							at = next;
							const size_t eq = kv.find('=');
							if (eq == std::string::npos) {
								continue;
							}
							const std::string key = kv.substr(0, eq);
							const int v = atoi(kv.c_str() + eq + 1);
							if (key == "worn") info.worn = v;
							else if (key == "left") info.left = v;
							else if (key == "right") info.right = v;
							else if (key == "sensors-known") info.sensors_known = v != 0;
							else if (key == "sensors-found") info.sensors_found = v;
							else if (key == "sensors-placed") info.sensors_placed = v;
							else if (key == "sensors-measuring") info.sensors_measuring = v != 0;
							else if (key == "display") info.display = v;
							else if (key == "direct-saved") info.direct_saved = v != 0;
							else if (key == "direct-running") info.direct_running = v != 0;
							else if (key == "scale-saved" && !dragging_scale) info.scale_saved = v;
							else if (key == "scale-running") info.scale_running = v;
							else if (key == "fast-path") info.fast_path = v != 0;
							else if (key == "perf") info.perf_on = v != 0;
						}
						continue;
					}
					if (name.rfind("cmd-picture ", 0) == 0) {
						int direct = 0, on = 0, strength = 50, colour = 50, smoothing = 0, bright = 0;
						if (sscanf(name.c_str() + 12, "%d %d %d %d %d %d", &direct, &on, &strength, &colour,
						           &smoothing, &bright) >= 3) {
							picture.direct = direct != 0;
							picture.on = on != 0;
							picture.strength = std::clamp(strength, 0, 100);
							picture.colour = std::clamp(colour, 0, 100);
							picture.smoothing = smoothing != 0;
							picture.bright = bright != 0;
						}
						continue;
					}
#endif
					if (name == "cmd-recentre") {
						// As SteamVR's Recenter: the seated origin to the head now.
						if (chaperone != nullptr) {
							chaperone->ResetZeroPose(vr::TrackingUniverseSeated);
							fprintf(stderr, "riftlinux-notices: recentred (seated origin)\n");
						}
						continue;
					}
					if (name == "cmd-open-quick") {
#ifdef RLN_HAVE_PULSE
						if (quick != vr::k_ulOverlayHandleInvalid &&
						    !(overlay->IsDashboardVisible() && overlay->IsActiveDashboardOverlay(quick))) {
							overlay->ShowDashboard("riftlinux.quick");
							opened_by_us_at = now_s();
							fprintf(stderr, "riftlinux-notices: opening the Rift tab\n");
						}
#endif
					} else if (valid_name(name)) {
						fprintf(stderr, "riftlinux-notices: got %s\n", name.c_str());
						// The driver repeats "measuring" while it lasts: the one up stays, no flicker.
						if (showing_lasting && shown_at >= 0.0 && name == showing_name) {
							continue;
						}
						queue.push_back(name);
						while (queue.size() > kMaxQueued) {
							queue.pop_front();
						}
					}
				}
			}
		} else if (!input_open) {
			usleep(30 * 1000);
		}

		vr::VREvent_t event;
		while (system->PollNextEvent(&event, sizeof(event))) {
			if (event.eventType == vr::VREvent_Quit) {
				frame_lock.restore();
				core->Cleanup();
				return 0;
			}
		}

		// The driver is gone: SteamVR is closing down.
		if (!input_open && shown_at < 0.0) {
			frame_lock.restore();
			break;
		}

		const double t = now_s();
#ifdef RLN_HAVE_PULSE
		frame_lock.tick(t, picture.smoothing);
		// The Performance card, while its switch is on; what motion smoothing makes: our compositor shows every one
		// of the CV1's 90 refreshes, the game's frames are the rest.
		perf.tick(t, info.perf_on, info);
		info.perf_made = !info.perf_on || !picture.smoothing || !picture.direct || info.perf_fps < 0
		                     ? -1
		                     : std::max(0, 90 - info.perf_fps);
#endif
#ifdef RLN_HAVE_PULSE
		// When measuring last ended: the tab takes no click for a moment after (see VREvent_MouseButtonDown).
		static bool was_measuring = false;
		static double measuring_ended_at = -10.0;
		if (was_measuring && !info.sensors_measuring) {
			measuring_ended_at = t;
		}
		was_measuring = info.sensors_measuring;
#endif

		// Whether SteamVR shows the room's walls, logged when it changes: the driver's own room keeps them hidden.
		static double walls_checked_at = 0.0;
		if (chaperone != nullptr && t - walls_checked_at >= 0.5) {
			walls_checked_at = t;
			static int walls_shown = -1;
			const int shown = chaperone->AreBoundsVisible() ? 1 : 0;
			if (shown != walls_shown) {
				walls_shown = shown;
				fprintf(stderr, "riftlinux-notices: SteamVR's room walls %s\n", shown ? "shown" : "hidden");
			}
		}

#ifdef RLN_HAVE_PULSE
		if (quick != vr::k_ulOverlayHandleInvalid) {
			pulse.tick(t);
			while (overlay->PollNextOverlayEvent(quick, &event, sizeof(event))) {
				// The laser's point in the panel's pixels from the top left (SteamVR's are from the bottom).
				const float x = event.data.mouse.x;
				const float y = layout.h - event.data.mouse.y;
				// A switch, button or page let go on where it was pressed (rl::tab::Press::release).
				const auto act = [&](const Hit &hit) {
					if (hit.part == Hit::Page) {
						page = hit.row;
						return;
					}
					// The picture's and motion's settings are Monado's compositor's: direct mode only.
					if (kCardNeedsDirect[kRows[hit.row].card] && !picture.direct) {
						return;
					}
					const bool small = hit.part == Hit::Small;
					switch (kRows[hit.row].ctl) {
					case Ctl::Volume:
						if (small) {
							pulse.set_muted(!pulse.muted, t);
						}
						break;
					case Ctl::Mic:
						if (small) {
							pulse.set_mic_muted(!pulse.mic_muted);
						}
						break;
					case Ctl::Floor:
						tell_driver("floor");
						floor_lit_until = t + 0.6;
						fprintf(stderr, "riftlinux-notices: set floor asked\n");
						break;
					case Ctl::Recalibrate:
						// Greyed (and no press) while measuring is under way.
						if (!info.sensors_measuring) {
							tell_driver("recalibrate");
							fprintf(stderr, "riftlinux-notices: measuring the sensors again asked\n");
						}
						break;
					case Ctl::Height:
						if (small) {
							height_cm = 0;
							tell_driver("height 0");
							fprintf(stderr, "riftlinux-notices: height reset\n");
						}
						break;
					case Ctl::AutoHdr:
						picture.on = !picture.on;
						tell_driver(picture.on ? "autohdr 1" : "autohdr 0");
						fprintf(stderr, "riftlinux-notices: Auto-HDR %s\n", picture.on ? "on" : "off");
						break;
					case Ctl::Strength:
						if (small) {
							picture.strength = 50;
							tell_driver("autohdr-strength 50");
							fprintf(stderr, "riftlinux-notices: Auto-HDR strength reset\n");
						}
						break;
					case Ctl::Colour:
						if (small) {
							picture.colour = 50;
							tell_driver("autohdr-colour 50");
							fprintf(stderr, "riftlinux-notices: colour reset\n");
						}
						break;
					case Ctl::Bright:
						picture.bright = !picture.bright;
						tell_driver(picture.bright ? "hdr-bright 1" : "hdr-bright 0");
						fprintf(stderr, "riftlinux-notices: HDR Bright %s\n", picture.bright ? "on" : "off");
						break;
					case Ctl::Smoothing:
						picture.smoothing = !picture.smoothing;
						tell_driver(picture.smoothing ? "smoothing 1" : "smoothing 0");
						fprintf(stderr, "riftlinux-notices: motion smoothing %s\n", picture.smoothing ? "on" : "off");
						break;
					case Ctl::DirectMode:
						// Saved for SteamVR's next start; the tab says so.
						info.direct_saved = !info.direct_saved;
						tell_driver(info.direct_saved ? "direct-mode 1" : "direct-mode 0");
						fprintf(stderr, "riftlinux-notices: direct mode %s from the next start\n",
						        info.direct_saved ? "on" : "off");
						break;
					case Ctl::RenderScale:
						if (small) {
							info.scale_saved = 100;
							tell_driver("render-scale 100");
						}
						break;
					case Ctl::FastPath:
						if (picture.direct) {
							info.fast_path = !info.fast_path;
							tell_driver(info.fast_path ? "fast-path 1" : "fast-path 0");
							fprintf(stderr, "riftlinux-notices: fast path %s\n", info.fast_path ? "on" : "off");
						}
						break;
					case Ctl::Recentre:
					case Ctl::StHeadset:
					case Ctl::StLeft:
					case Ctl::StRight:
					case Ctl::StSensors:
					case Ctl::StDisplay: break;
					case Ctl::PerfOn:
						info.perf_on = !info.perf_on;
						tell_driver(info.perf_on ? "perf 1" : "perf 0");
						fprintf(stderr, "riftlinux-notices: performance numbers %s\n", info.perf_on ? "on" : "off");
						break;
					case Ctl::PfGame:
					case Ctl::PfAvg:
					case Ctl::PfSmooth:
					case Ctl::PfGpu:
					case Ctl::PfDropped:
					case Ctl::CloseApp: break;
					}
				};
				switch (event.eventType) {
				case vr::VREvent_OverlayShown:
					quick_visible = true;
					quick_drawn = false;
					fprintf(stderr, "riftlinux-notices: Rift tab shown\n");
					/*
					 * SteamVR opens its dashboard on the tab shown last: after the Rift tab, the menu
					 * button opened the Rift tab instead of Steam's (seen twice: checked as the dashboard
					 * opened, the tab was not active yet). Shown as the dashboard opens, other than by the
					 * Oculus button, it goes to Steam's main page; the tab's icon still opens it.
					 */
					if (t - opened_by_us_at > 1.5 && t - dashboard_opened_at < 1.0) {
						overlay->ShowDashboard(kSteamMainTab);
						fprintf(stderr, "riftlinux-notices: dashboard opened on the Rift tab, Steam's instead\n");
					}
					break;
				case vr::VREvent_OverlayHidden:
					quick_visible = false;
					if (dragging_scale) {
						dragging_scale = false;
						tell_driver(("render-scale " + std::to_string(info.scale_saved)).c_str());
					}
					dragging = false;
					dragging_height = false;
					if (dragging_strength) {
						dragging_strength = false;
						strength_sent_at = -1.0;
					}
					if (dragging_colour) {
						dragging_colour = false;
						colour_sent_at = -1.0;
					}
					fprintf(stderr, "riftlinux-notices: Rift tab hidden\n");
					break;
				case vr::VREvent_ImageFailed:
					fprintf(stderr, "riftlinux-notices: SteamVR could not load the Rift tab's image\n");
					break;
				case vr::VREvent_MouseButtonDown: {
					/*
					 * Not right after measuring: the controllers come back then, and a trigger pressed while
					 * they were hidden landed on the Direct mode row and switched it off.
					 */
					if (t - measuring_ended_at < 1.5) {
						fprintf(stderr, "riftlinux-notices: click right after measuring ignored\n");
						break;
					}
					// Acts when let go on the same control; a slider's knob is grabbed (rl::tab::Press).
					const Hit hit = press.begin(layout, page, x, y, drawn);
					if (hit.part == Hit::None) {
						fprintf(stderr, "riftlinux-notices: click at %.0f, %.0f\n", x, y);
						break;
					}
					if (hit.part != Hit::Track) {
						break;
					}
					if (kCardNeedsDirect[kRows[hit.row].card] && !picture.direct) {
						press.hit = {};
						break;
					}
					// The value follows once the laser moves (MouseMove), never jumps to the point.
					switch (kRows[hit.row].ctl) {
					case Ctl::Volume:
						dragging = true;
						drag_percent = -1;
						break;
					case Ctl::Mic:
						dragging_mic = pulse.mic_known;
						drag_mic_percent = -1;
						break;
					case Ctl::Height:
						dragging_height = true;
						drag_height_cm = height_cm;
						break;
					case Ctl::Strength: dragging_strength = true; break;
					case Ctl::Colour: dragging_colour = true; break;
					case Ctl::RenderScale: dragging_scale = true; break;
					default: break;
					}
					break;
				}
				case vr::VREvent_MouseMove: {
					// A grabbed knob follows the laser from where it was grabbed, past a few pixels.
					const std::optional<float> f = press.drag(layout, x);
					if (!f) {
						break;
					}
					const int percent = (int)std::lround(100.0f * *f);
					if (dragging_scale) {
						info.scale_saved = 5 * (int)std::lround((kScaleMin + *f * (kScaleMax - kScaleMin)) / 5.0f);
					}
					if (dragging_strength) {
						picture.strength = percent;
					}
					if (dragging_colour) {
						picture.colour = percent;
					}
					if (dragging_height) {
						drag_height_cm = std::clamp(
						    (int)std::lround(kHeightMinCm + *f * (kHeightMaxCm - kHeightMinCm)), kHeightMinCm,
						    kHeightMaxCm);
					}
					if (dragging) {
						drag_percent = percent;
					}
					if (dragging_mic) {
						drag_mic_percent = percent;
					}
					break;
				}
				case vr::VREvent_MouseButtonUp:
					if (dragging_scale && press.moved) {
						// Saved for the next start: sent once, when let go.
						dragging_scale = false;
						tell_driver(("render-scale " + std::to_string(info.scale_saved)).c_str());
						fprintf(stderr, "riftlinux-notices: render scale %d %% from the next start\n", info.scale_saved);
					}
					if (dragging_strength) {
						dragging_strength = false;
						strength_sent_at = -1.0; // Sent below, now.
					}
					if (dragging_colour) {
						dragging_colour = false;
						colour_sent_at = -1.0;
					}
					if (dragging_height) {
						height_sent_at = -1.0; // Sent below, now.
						dragging_height = false;
						if (drag_height_cm != height_cm) {
							height_cm = drag_height_cm;
							tell_driver(("height " + std::to_string(height_cm * 10)).c_str());
							fprintf(stderr, "riftlinux-notices: height %+d cm\n", height_cm);
						}
					}
					if (dragging && drag_percent >= 0) {
						pulse.set_percent(drag_percent, t);
					}
					dragging = false;
					if (dragging_mic && drag_mic_percent >= 0) {
						pulse.set_mic_percent(drag_mic_percent);
					}
					dragging_mic = false;
					dragging_scale = false;
					{
						const Hit let_go = press.release(layout, page, x, y, drawn);
						if (let_go.part != Hit::None) {
							act(let_go);
						}
					}
					break;
				// The thumbstick scrolls an overlay: it changed the volume wherever the laser was. Not any more.
				case vr::VREvent_ScrollDiscrete: break;
				default: break;
				}
			}
			// The strength, live while its slider is held (the picture follows), at most 10 changes a second.
			static int strength_sent = -1;
			if (picture.strength != strength_sent && (strength_sent_at < 0.0 || t - strength_sent_at >= 0.1) &&
			    picture.direct && (dragging_strength || strength_sent_at < 0.0)) {
				tell_driver(("autohdr-strength " + std::to_string(picture.strength)).c_str());
				strength_sent = picture.strength;
				strength_sent_at = t;
			}
			// The colour the same way.
			static int colour_sent = -1;
			if (picture.colour != colour_sent && (colour_sent_at < 0.0 || t - colour_sent_at >= 0.1) &&
			    picture.direct && (dragging_colour || colour_sent_at < 0.0)) {
				tell_driver(("autohdr-colour " + std::to_string(picture.colour)).c_str());
				colour_sent = picture.colour;
				colour_sent_at = t;
			}
			// While the height slider is held, at most 10 changes a second to the driver.
			if (dragging_height && drag_height_cm != height_cm && t - height_sent_at >= 0.1) {
				height_cm = drag_height_cm;
				tell_driver(("height " + std::to_string(height_cm * 10)).c_str());
				height_sent_at = t;
			}
			// While the slider is held, at most 20 changes a second.
			if (dragging && drag_percent >= 0 && drag_percent != pulse.percent && t - drag_sent_at >= 0.05) {
				pulse.set_percent(drag_percent, t);
				drag_sent_at = t;
			}
			if (dragging_mic && drag_mic_percent >= 0 && drag_mic_percent != pulse.mic_percent &&
			    t - drag_mic_sent_at >= 0.05) {
				pulse.set_mic_percent(drag_mic_percent);
				drag_mic_sent_at = t;
			}

			// Kept current whether shown or not: SteamVR shows no dashboard overlay that has no image yet (the
			// first version drew only once shown, and the tab never came up). Before the volume is known, after
			// a moment, without it. Drawn again only when what it shows changes.
			TabState now;
			now.volume = pulse.known ? pulse.percent : -1;
			now.muted = pulse.muted;
			now.mic = pulse.mic_known ? pulse.mic_percent : -1;
			now.mic_muted = pulse.mic_muted;
			now.height_cm = height_cm;
			now.floor_lit = t < floor_lit_until;
			now.picture = picture;
			now.info = info;
			now.page = page;
			if (quick_drawn && !(now == drawn)) {
				pulse.changed = true;
			}
			if (quick_drawn && now == drawn) {
				pulse.changed = false;
			}
			if ((!quick_drawn || pulse.changed) && (pulse.known || t - quick_since >= 2.0) &&
			    t - drawn_at >= 0.05) {
				Image img = render_tab(pieces, layout, now);
				bool vulkan = false;
#ifdef RLN_HAVE_VULKAN
				vulkan = vk_tab.show(overlay, quick, img);
#endif
				const bool ok = vulkan || set_overlay_image(overlay, quick, img, raw_ok);
				if (!ok || !quick_ever_drawn) {
					fprintf(stderr, "riftlinux-notices: Rift tab image set: %s (%s)\n", ok ? "yes" : "NO",
					        vulkan ? "Vulkan image" : raw_ok ? "pixels" : "file");
				}
				quick_ever_drawn = quick_ever_drawn || ok;
				quick_drawn = true;
				drawn_at = t;
				drawn = now;
				if (quick_visible) {
					pulse.changed_elsewhere = false;
				}
				pulse.changed = false;
			}

			// Changed from elsewhere with the dashboard closed: shown in front of the view for a moment,
			// unless a notice is up.
			const bool dashboard = overlay->IsDashboardVisible();
			if (dashboard != dashboard_was) {
				tell_driver(dashboard ? "dashboard 1" : "dashboard 0");
				near_dashboard = dashboard;
				place(placed_width, placed_below);
				if (dashboard) {
					dashboard_opened_at = t;
				}
				dashboard_was = dashboard;
			}
			if (pulse.changed_elsewhere && dashboard) {
				pulse.changed_elsewhere = false;
			}
			if (pulse.changed_elsewhere && !quick_visible && (shown_at < 0.0 || showing_volume) &&
			    t - drawn_at >= 0.05) {
				TabState volume;
				volume.volume = pulse.percent;
				volume.muted = pulse.muted;
				Image img = render_volume_panel(pieces, layout, volume);
				if (set_overlay_image(overlay, handle, img, raw_ok)) {
					if (shown_at < 0.0) {
						place(kVolumeWidthM, kBelowM);
						overlay->SetOverlayAlpha(handle, 0.0f);
						overlay->ShowOverlay(handle);
						shown_at = t;
					} else {
						shown_at = std::min(shown_at, t - kFadeS); // Stays at full while it changes.
					}
					show_s = (t - shown_at) + kVolumeShowS;
					showing_volume = true;
				}
				drawn_at = t;
				pulse.changed = false;
				pulse.changed_elsewhere = false;
			} else if (pulse.changed_elsewhere && !quick_visible && shown_at >= 0.0 && !showing_volume) {
				pulse.changed_elsewhere = false;
			}
		}
#endif

#ifdef RLN_HAVE_PULSE
		// A lasting notice stays up while the driver's status says the sensors are being measured or placed, at
		// least a few seconds; it fades as soon as that ends, or when the next notice (it is done) comes.
		if (shown_at >= 0.0 && showing_lasting) {
			const double age = t - shown_at;
			if (age >= 3.0 && (!info.sensors_measuring || !queue.empty())) {
				showing_lasting = false;
				show_s = age + kFadeS;
			} else {
				show_s = std::max(show_s, age + 60.0);
			}
		}
#endif
		if (shown_at >= 0.0) {
			const double age = t - shown_at;
			if (age >= show_s) {
				overlay->HideOverlay(handle);
				shown_at = -1.0;
				showing_volume = false;
			} else {
				const double fade = std::min(age, show_s - age) / kFadeS;
				overlay->SetOverlayAlpha(handle, static_cast<float>(std::min(1.0, fade)));
			}
			continue;
		}

		if (!queue.empty()) {
			const bool test = queue.front().rfind("test-", 0) == 0;
			const bool welcome = queue.front() == "welcome";
			const std::string name = queue.front();
			const std::string path = panels + "/" + queue.front() + ".png";
			queue.pop_front();
			struct stat st;
			if (stat(path.c_str(), &st) != 0) {
				fprintf(stderr, "riftlinux-notices: no panel %s\n", path.c_str());
				continue;
			}
			const vr::EVROverlayError err = overlay->SetOverlayFromFile(handle, path.c_str());
			if (err != vr::VROverlayError_None) {
				fprintf(stderr, "riftlinux-notices: SteamVR did not take %s (overlay error %d)\n", path.c_str(),
				        (int)err);
			} else {
				place(test ? kTestWidthM : welcome ? kWelcomeWidthM : kWidthM,
				      test ? 0.0f : welcome ? kWelcomeBelowM : kBelowM);
				show_s = test ? kTestShowS : welcome ? kWelcomeShowS : name == "calibration-start" ? kMeasuringShowS : kShowS;
				overlay->SetOverlayAlpha(handle, 0.0f);
				overlay->ShowOverlay(handle);
				shown_at = t;
				showing_name = name;
				showing_lasting = name == "calibration-start" || name.rfind("sensor-moved-", 0) == 0;
				fprintf(stderr, "riftlinux-notices: showing %s%s\n", path.c_str(),
				        showing_lasting ? " (while it lasts)" : "");
			}
		}
	}

	core->Cleanup();
	return 0;
}
