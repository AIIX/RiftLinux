// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The Rift panel in RiftLinux's own runtime (see ipc_server_rift_panel.h).
 *
 * An in-process overlay client of the multi-client compositor, as the SteamVR driver's direct mode is one: three quad
 * layers, the panel, a cursor where the laser meets it and the laser beam. The panel is drawn by the same code as the
 * SteamVR tab (auxiliary/riftlinux/rl_tab.hpp) and uploaded only when it changes. While it is open the application
 * keeps drawing but loses input focus, so the trigger does not also act in it.
 * @ingroup ipc_server
 */

#include "ipc_server_rift_panel.h"
#include "server/ipc_server.h"

#include "xrt/xrt_compositor.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_system.h"

#include "b_space_overseer.h"
#include "math/m_api.h"
#include "math/m_vec3.h"
#include "os/os_time.h"
#include "render/render_output_settings.h"
#include "util/comp_swapchain.h"
#include "util/u_logging.h"
#include "util/u_notice.h"
#include "vk/vk_cmd_pool.h"
#include "vk/vk_helpers.h"

#include "riftlinux/rl_audio_switch.hpp"
#include "riftlinux/rl_floor.hpp"
#include "riftlinux/rl_room.hpp"
#include "riftlinux/rl_pulse.hpp"
#include "riftlinux/rl_settings.hpp"
#include "riftlinux/rl_tab.hpp"

#include <signal.h>
#include <unistd.h>

// The notices' panels are PNG files (made for SteamVR's overlays): read here, PNG only.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

using namespace rl::tab;

namespace {

//! The panel's width in the room, and where it is put when opened: ahead of the eyes and a little below them.
constexpr float kPanelWidthM = 1.25f;
constexpr float kPanelAheadM = 1.3f;
constexpr float kPanelBelowM = 0.12f;
//! Above every application and overlay: an application's place is its OpenXR overlay placement (32 bits), the
//! multi-client compositor sorts its clients by a 64-bit one (an overlay such as WayVR was above 1000000).
constexpr int64_t kZOrder = INT64_MAX;
//! The trigger counts as pressed above this, released below the other.
constexpr float kTriggerPress = 0.6f;
constexpr float kTriggerRelease = 0.35f;
//! The cursor's size, the beam's width and how far it reaches when it meets nothing, metres.
constexpr float kCursorM = 0.028f;
constexpr float kBeamWidthM = 0.004f;
constexpr float kBeamMissM = 2.0f;
//! "Set floor" lit this long after a press, seconds.
constexpr double kFloorLitS = 1.0;
//! Notices, as the SteamVR helper shows them: in front of the view, following it, for a few seconds.
constexpr float kNoticeWidthM = 0.6f;
constexpr float kNoticeAheadM = 1.0f;
constexpr float kNoticeBelowM = 0.18f;
constexpr double kNoticeShowS = 6.0;
constexpr double kNoticeFadeS = 0.3;

double
now_s()
{
	return (double)os_monotonic_get_ns() / U_TIME_1S_IN_NS;
}

xrt_result_t
ignore_event(struct xrt_session_event_sink *, const union xrt_session_event *)
{
	return XRT_SUCCESS;
}

struct xrt_session_event_sink g_events = {ignore_event};

//! The first folder with the tab's pieces: the SteamVR driver has them. The package's own copy beside this program
//! comes first: the user's copy of the driver is synced from it at login or plug-in, so after an upgrade it can
//! still hold the older pieces for a while.
std::string
pieces_dir()
{
	std::vector<std::string> dirs;
	if (const char *env = getenv("RIFTLINUX_NOTICES_DIR"); env != nullptr && env[0] != '\0') {
		dirs.push_back(env);
	}
	char exe[4096];
	const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
	std::string exe_dir;
	if (n > 0) {
		exe[n] = '\0';
		exe_dir = std::string(exe).substr(0, std::string(exe).rfind('/'));
		// The package: <lib>/riftlinux/native/monado-service and <lib>/riftlinux/steamvr/rift-cv1.
		dirs.push_back(exe_dir + "/../steamvr/rift-cv1/resources/notices");
	}
	const char *xdg = getenv("XDG_DATA_HOME");
	const char *home = getenv("HOME");
	const std::string data =
	    xdg != nullptr && xdg[0] == '/' ? xdg : std::string(home != nullptr ? home : "") + "/.local/share";
	dirs.push_back(data + "/riftlinux/steamvr/rift-cv1/resources/notices");
	dirs.push_back("/usr/lib/riftlinux/steamvr/rift-cv1/resources/notices");
	if (!exe_dir.empty()) {
		dirs.push_back(exe_dir + "/../share/riftlinux/notices");
	}
	for (const std::string &d : dirs) {
		if (access((d + "/tab-title.rgba").c_str(), R_OK) == 0) {
			return d;
		}
	}
	return "";
}

//! A soft white dot with a dark ring, for the cursor.
Image
make_dot(int size)
{
	Image img;
	img.w = img.h = size;
	img.px.assign((size_t)size * size * 4, 0);
	fill_circle(img, size / 2.0f, size / 2.0f, size / 2.0f - 1, Rgba{20, 22, 28, 220});
	fill_circle(img, size / 2.0f, size / 2.0f, size / 2.0f - 6, Rgba{235, 238, 243, 255});
	return img;
}

//! The beam: brighter at its middle across, fading toward its far end.
Image
make_beam(int w, int h)
{
	Image img;
	img.w = w;
	img.h = h;
	img.px.resize((size_t)w * h * 4);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			const float across = 1.0f - std::fabs((x + 0.5f) / w * 2.0f - 1.0f);
			// Image row 0 is the far end (+Y up in the quad).
			const float along = 0.35f + 0.65f * (float)y / (float)(h - 1);
			uint8_t *p = &img.px[((size_t)y * w + x) * 4];
			p[0] = 150;
			p[1] = 190;
			p[2] = 255;
			p[3] = (uint8_t)std::lround(255.0f * along * (0.4f + 0.6f * across));
		}
	}
	return img;
}

xrt_vec3
rotate(const xrt_quat &q, xrt_vec3 v)
{
	xrt_vec3 out;
	math_quat_rotate_vec3(&q, &v, &out);
	return out;
}

} // namespace

struct rl_panel
{
	struct ipc_server *s = nullptr;
	struct xrt_device *head = nullptr;
	struct xrt_device *hands[2] = {nullptr, nullptr};

	std::thread thread;
	std::atomic<bool> running{true};
	std::atomic<bool> want_open{false};
	std::atomic<bool> want_close{false}; // A test key's "Close the application" press.
	bool open = false;
	//! Opened by the test key: stays open whether or not the headset is worn (a desk check).
	bool test_mode = false;

	// The compositor client and its images.
	struct xrt_compositor_native *xcn = nullptr;
	struct xrt_swapchain *xsc_panel = nullptr, *xsc_dot = nullptr, *xsc_beam = nullptr;
	uint32_t panel_index = 0;
	bool panel_ready = false;
	struct vk_cmd_pool pool = {};
	struct vk_bundle *vk = nullptr;

	// The tab.
	Pieces pieces;
	Layout layout;
	TabState state, drawn;
	bool drawn_valid = false;
	double floor_lit_until = 0.0;
	double next_status_s = 0.0;

	// Where the panel is, and the laser.
	struct xrt_pose pose = XRT_POSE_IDENTITY;
	float height_m = 1.0f;
	int hand = 1;
	bool trigger_down[2] = {false, false};
	int drag_row = -1;
	//! The trigger's press on the panel: what it is on, and a grabbed knob (rl::tab::Press).
	Press press;
	//! When measuring last ended: no press for a moment after (the controllers come back then).
	bool was_measuring = false;
	double measuring_ended_s = -10.0;

	/*!
	 * "Close the application": the application asked to quit (OpenXR's EXITING), ended (SIGTERM, then SIGKILL) if it
	 * is still connected 3 s and 6 s later. Its client slot and process.
	 */
	struct
	{
		int index = -1;
		pid_t pid = 0;
		double since = 0.0;
		int stage = 0;
	} closing;

	//! The Performance card: the game's committed frames (the busiest application's), counted once a second.
	struct
	{
		double next_s = 0.0, last_s = 0.0;
		int64_t last_frames[IPC_MAX_CLIENTS] = {};
		std::deque<std::pair<int64_t, double>> window;
	} perf;
	bool hit_valid = false;
	float hit_px = 0, hit_py = 0;
	struct xrt_vec3 hit_world = {}, beam_from = {}, beam_to = {};
	bool beam_valid = false;

	// The picture's settings as the environment set them at start: the strength and colour sliders scale these.
	struct render_output_settings base = {};
	int scale_running = 100;

#ifdef RL_HAVE_PULSE
	rl::Pulse pulse;
	bool pulse_ok = false;
#endif
#ifdef RL_HAVE_AUDIO_SWITCH
	rl::audio::Switch audio;
#endif

	/*
	 * Notices (a sensor moved, measuring the sensors): queued by the tracker's posts (notice_sink), shown in front of
	 * the view for a few seconds on top of the application without taking its input.
	 */
	std::mutex notice_lock;
	std::deque<std::string> notices;
	struct xrt_swapchain *xsc_notice = nullptr;
	int notice_w = 0, notice_h = 0;
	double notice_since = 0.0;
	//! Up while what it says lasts (measuring), not for a fixed time.
	bool notice_lasting = false;

	/*
	 * Measuring the sensors again (the tab's button): the tracker puts the new measurement back where the room was
	 * when sensors kept their place (rl_room.hpp), and the room and the floor stay. Else, after "calibration-done",
	 * this thread recentres once the head has settled and carries the floor over (rl_floor.hpp). The floor as the
	 * settings have it.
	 */
	std::atomic<bool> recal_measuring{false};
	std::atomic<bool> recal_done{false};
	//! The last measuring result taken (u_notice_recalibration::sequence): the tab's, or a first measure from nothing.
	std::atomic<uint32_t> recal_seen{0};
	rl::floor::Keeper keeper;
	rl::room::Settle resettle;
	bool floor_known = false;
	float floor_m = 0.0f;
	double next_head_s = 0.0;
};

namespace {

void
log_line(const std::string &line)
{
	U_LOG_I("Rift panel: %s", line.c_str());
}

/*
 *
 * Settings.
 *
 */

void
apply_picture(struct rl_panel *p)
{
	struct render_output_settings rs;
	render_output_settings_get(&rs);
	const PictureRow &pic = p->state.picture;
	const float k = std::clamp(pic.strength, 0, 100) / 50.0f;
	const float c = std::clamp(pic.colour, 0, 100) / 50.0f;
	rs.auto_hdr = pic.on;
	rs.auto_hdr_highlights = p->base.auto_hdr_highlights * k;
	rs.auto_hdr_shadows = p->base.auto_hdr_shadows * k;
	rs.auto_hdr_colour = p->base.auto_hdr_colour * c;
	rs.motion_smoothing = pic.smoothing;
	rs.hdr_bright = pic.bright;
	rs.fast_path = p->state.info.fast_path;
	render_output_settings_set(&rs);
}

void
apply_floor(struct rl_panel *p, const cJSON *root)
{
	// Within reach of a room, as the SteamVR driver takes them (a file edited by hand may hold any number).
	const cJSON *floor = rl::settings::item(root, "tracking", "floor_m");
	const float raise =
	    (float)std::clamp(rl::settings::number(root, "tracking", "height_cm", 0.0), -100.0, 100.0) / 100.0f;
	p->floor_known = cJSON_IsNumber(floor);
	p->floor_m = p->floor_known ? (float)std::clamp(floor->valuedouble, -3.0, 3.0) : 0.0f;
	const cJSON *eye = rl::settings::item(root, "tracking", "eye_height_m");
	if (cJSON_IsNumber(eye)) {
		p->keeper.set_saved_eye((float)std::clamp(eye->valuedouble, 0.3, 2.5));
	}
	// The SteamVR driver's floor_m moves the world up by that much: the floor is at -floor_m in the tracker's space.
	b_space_overseer_set_floor(p->s->xso, p->floor_known, -p->floor_m, raise);
}

//! The settings file into the compositor and the spaces, at start.
void
load_settings(struct rl_panel *p)
{
	render_output_settings_get(&p->base);
	cJSON *root = rl::settings::load();
	struct render_output_settings rs = p->base;
	// The HDR parts' own switches and values, as the SteamVR driver reads them.
	rs.black_level = (float)rl::settings::number(root, "display", "black_level", rs.black_level);
	rs.dither_bits = (float)rl::settings::number(root, "display", "dither_bits", rs.dither_bits);
	rs.hdr_white = (float)rl::settings::number(root, "display", "hdr_white", rs.hdr_white);
	rs.hdr_white_bright = (float)rl::settings::number(root, "display", "hdr_white_bright", rs.hdr_white_bright);
	rs.hdr_float_layers = rl::settings::boolean(root, "display", "hdr_float_layers", rs.hdr_float_layers);
	rs.hdr_metadata = rl::settings::boolean(root, "display", "hdr_metadata", rs.hdr_metadata);
	rs.hdr_measured_peak = rl::settings::boolean(root, "display", "hdr_measured_peak", rs.hdr_measured_peak);
	render_output_settings_set(&rs);
	p->base = rs;

	p->state.info.perf_on = rl::settings::boolean(root, "tab", "performance", false);
	PictureRow &pic = p->state.picture;
	pic.direct = true; // Our compositor always draws here.
	pic.on = rl::settings::boolean(root, "display", "auto_hdr", rs.auto_hdr);
	pic.strength = (int)std::lround(rl::settings::number(root, "display", "auto_hdr_strength", 50));
	pic.colour = (int)std::lround(rl::settings::number(root, "display", "auto_hdr_colour_strength", 50));
	pic.bright = rl::settings::boolean(root, "display", "hdr_bright", rs.hdr_bright);
	pic.smoothing = rl::settings::boolean(root, "display", "motion_smoothing", rs.motion_smoothing);
	TabInfo &info = p->state.info;
	info.native = true;
	info.display = 3;
	info.fast_path = rl::settings::boolean(root, "display", "direct_mode_fast_path", rs.fast_path);
	info.scale_saved = (int)std::lround(rl::settings::number(root, "display", "render_scale", p->scale_running));
	info.scale_running = p->scale_running;
	p->state.height_cm = (int)std::lround(rl::settings::number(root, "tracking", "height_cm", 0));
	apply_picture(p);
	apply_floor(p, root);
	cJSON_Delete(root);
	U_LOG_I("Rift panel: settings applied: Auto-HDR %s %d/%d %%, HDR Bright %s, smoothing %s, fast path %s",
	        pic.on ? "on" : "off", pic.strength, pic.colour, pic.bright ? "on" : "off", pic.smoothing ? "on" : "off",
	        info.fast_path ? "on" : "off");
}

/*
 *
 * Images.
 *
 */

//! Copies @p img into image @p index of @p xsc and leaves it ready for sampling (as the HDR test chart is uploaded).
bool
upload(struct rl_panel *p, struct xrt_swapchain *xsc, uint32_t index, const Image &img)
{
	struct comp_swapchain *sc = (struct comp_swapchain *)xsc;
	struct vk_bundle *vk = sc->vk;
	const VkDeviceSize size = (VkDeviceSize)img.px.size();
	VkBuffer buf = VK_NULL_HANDLE;
	VkDeviceMemory mem = VK_NULL_HANDLE;
	bool ok = vk_buffer_init(vk, size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
	                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &buf, &mem);
	void *mapped = nullptr;
	if (ok && vk->vkMapMemory(vk->device, mem, 0, size, 0, &mapped) == VK_SUCCESS) {
		memcpy(mapped, img.px.data(), (size_t)size);
		vk->vkUnmapMemory(vk->device, mem);
	} else {
		ok = false;
	}
	VkResult r = VK_ERROR_UNKNOWN;
	if (ok) {
		vk_cmd_pool_lock(&p->pool);
		VkCommandBuffer cmd = VK_NULL_HANDLE;
		r = vk_cmd_pool_create_and_begin_cmd_buffer_locked(vk, &p->pool, 0, &cmd);
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
			vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0,
			                         nullptr, 0, nullptr, 1, &bar);
			VkBufferImageCopy copy = {};
			copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
			copy.imageExtent = {(uint32_t)img.w, (uint32_t)img.h, 1};
			vk->vkCmdCopyBufferToImage(cmd, buf, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
			bar.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			bar.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
			bar.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			bar.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0,
			                         nullptr, 0, nullptr, 1, &bar);
			r = vk_cmd_pool_end_submit_wait_and_free_cmd_buffer_locked(vk, &p->pool, cmd);
		}
		vk_cmd_pool_unlock(&p->pool);
	}
	if (buf != VK_NULL_HANDLE) {
		vk->vkDestroyBuffer(vk->device, buf, nullptr);
		vk->vkFreeMemory(vk->device, mem, nullptr);
	}
	return ok && r == VK_SUCCESS;
}

struct xrt_swapchain *
make_swapchain(struct rl_panel *p, int w, int h, bool static_image)
{
	struct xrt_swapchain_create_info info = {};
	info.create = static_image ? XRT_SWAPCHAIN_CREATE_STATIC_IMAGE : (enum xrt_swapchain_create_flags)0;
	info.bits = (enum xrt_swapchain_usage_bits)(XRT_SWAPCHAIN_USAGE_COLOR | XRT_SWAPCHAIN_USAGE_SAMPLED |
	                                            XRT_SWAPCHAIN_USAGE_TRANSFER_DST);
	info.format = VK_FORMAT_R8G8B8A8_SRGB;
	info.sample_count = 1;
	info.width = (uint32_t)w;
	info.height = (uint32_t)h;
	info.face_count = 1;
	info.array_size = 1;
	info.mip_count = 1;
	struct xrt_swapchain *xsc = nullptr;
	if (xrt_comp_create_swapchain(&p->xcn->base, &info, &xsc) != XRT_SUCCESS) {
		return nullptr;
	}
	return xsc;
}

//! A static image made once: the cursor's dot, the beam.
struct xrt_swapchain *
make_static(struct rl_panel *p, const Image &img)
{
	struct xrt_swapchain *xsc = make_swapchain(p, img.w, img.h, true);
	uint32_t index = 0;
	if (xsc == nullptr || xrt_swapchain_acquire_image(xsc, &index) != XRT_SUCCESS ||
	    xrt_swapchain_wait_image(xsc, 100LL * U_TIME_1MS_IN_NS, index) != XRT_SUCCESS) {
		xrt_swapchain_reference(&xsc, nullptr);
		return nullptr;
	}
	const bool ok = upload(p, xsc, index, img);
	xrt_swapchain_release_image(xsc, index);
	if (!ok) {
		xrt_swapchain_reference(&xsc, nullptr);
	}
	return xsc;
}

//! The tab drawn again and into the next image, when what it shows changed.
void
redraw(struct rl_panel *p)
{
	p->state.floor_lit = now_s() < p->floor_lit_until;
	if (p->drawn_valid && p->drawn == p->state) {
		return;
	}
	const Image img = render_tab(p->pieces, p->layout, p->state);
	uint32_t index = 0;
	if (xrt_swapchain_acquire_image(p->xsc_panel, &index) != XRT_SUCCESS ||
	    xrt_swapchain_wait_image(p->xsc_panel, 100LL * U_TIME_1MS_IN_NS, index) != XRT_SUCCESS) {
		return;
	}
	const bool ok = upload(p, p->xsc_panel, index, img);
	xrt_swapchain_release_image(p->xsc_panel, index);
	if (ok) {
		p->panel_index = index;
		p->panel_ready = true;
		p->drawn = p->state;
		p->drawn_valid = true;
	}
}

/*
 *
 * Status.
 *
 */

bool
read_input(struct xrt_device *xdev, enum xrt_input_name name, union xrt_input_value *out)
{
	for (uint32_t i = 0; i < xdev->input_count; i++) {
		if (xdev->inputs[i].name == name) {
			*out = xdev->inputs[i].value;
			return true;
		}
	}
	return false;
}

int
battery(struct xrt_device *xdev)
{
	if (xdev == nullptr) {
		return -2;
	}
	bool present = false, charging = false;
	float charge = 0.0f;
	if (!xdev->supported.battery_status ||
	    xrt_device_get_battery_status(xdev, &present, &charging, &charge) != XRT_SUCCESS || !present) {
		return -1;
	}
	return (int)std::lround(charge * 100.0f);
}

//! The Performance card's numbers, once a second while its switch is on: the busiest application's frames.
void
perf_tick(struct rl_panel *p, double t)
{
	TabInfo &info = p->state.info;
	if (!info.perf_on) {
		p->perf.window.clear();
		p->perf.last_s = 0.0;
		info.perf_fps = info.perf_avg = info.perf_made = -1;
		return;
	}
	if (t < p->perf.next_s) {
		return;
	}
	p->perf.next_s = t + 1.0;
	int64_t most = -1;
	for (int i = 0; i < IPC_MAX_CLIENTS; i++) {
		const int64_t frames = xrt_atomic_s64_load(&((struct ipc_client_state *)&p->s->threads[i].ics)->frames_committed);
		const int64_t delta = frames - p->perf.last_frames[i];
		p->perf.last_frames[i] = frames;
		most = std::max(most, delta);
	}
	const double dt = t - p->perf.last_s;
	const bool first = p->perf.last_s == 0.0;
	p->perf.last_s = t;
	if (first || dt <= 0.0 || most < 0) {
		return;
	}
	p->perf.window.push_back({most, dt});
	while (p->perf.window.size() > 10) {
		p->perf.window.pop_front();
	}
	int64_t sum = 0;
	double sum_dt = 0.0;
	for (const auto &[frames, d] : p->perf.window) {
		sum += frames;
		sum_dt += d;
	}
	info.perf_fps = (int)std::lround((double)most / dt);
	info.perf_avg = (int)std::lround((double)sum / sum_dt);
	// What motion smoothing makes: the compositor shows every refresh, the game's frames are the rest.
	info.perf_made = p->state.picture.smoothing ? std::max(0, 90 - info.perf_fps) : -1;
}

int
app_to_close_locked(struct rl_panel *p);

void
update_status(struct rl_panel *p, double t)
{
	if (t < p->next_status_s) {
		return;
	}
	p->next_status_s = t + 1.0;
	TabInfo &info = p->state.info;
	union xrt_input_value v = {};
	info.worn = -1;
	if (p->head->supported.presence && xrt_device_update_inputs(p->head) == XRT_SUCCESS &&
	    read_input(p->head, XRT_INPUT_GENERIC_HEAD_DETECT, &v)) {
		info.worn = v.boolean ? 1 : 0;
	}
	info.left = battery(p->hands[0]);
	info.right = battery(p->hands[1]);
	struct u_notice_sensors sensors = {};
	info.sensors_known = u_notice_get_sensors(&sensors);
	info.sensors_found = sensors.found;
	info.sensors_placed = sensors.placed;
	info.sensors_measuring = sensors.measuring;
	os_mutex_lock(&p->s->global_state.lock);
	info.app_running = app_to_close_locked(p) >= 0;
	os_mutex_unlock(&p->s->global_state.lock);
	info.app_closing = p->closing.stage > 0;
	if (p->was_measuring && !info.sensors_measuring) {
		p->measuring_ended_s = t;
	}
	p->was_measuring = info.sensors_measuring;
}

/*
 *
 * Input.
 *
 */

//! What the controller of @p hand points at, as panel pixels; false when not at the panel.
bool
point(struct rl_panel *p, int hand, int64_t at_ns, float *out_px, float *out_py, struct xrt_vec3 *out_from,
      struct xrt_vec3 *out_to)
{
	struct xrt_device *xdev = p->hands[hand];
	if (xdev == nullptr) {
		return false;
	}
	struct xrt_space_relation rel = {};
	if (xrt_device_get_tracked_pose(xdev, XRT_INPUT_TOUCH_AIM_POSE, at_ns, &rel) != XRT_SUCCESS ||
	    (rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) == 0 ||
	    (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return false;
	}
	const xrt_vec3 o = rel.pose.position;
	const xrt_vec3 d = rotate(rel.pose.orientation, {0.0f, 0.0f, -1.0f});
	const xrt_vec3 n = rotate(p->pose.orientation, {0.0f, 0.0f, 1.0f});
	*out_from = o;
	*out_to = o + d * kBeamMissM;
	const float dn = m_vec3_dot(d, n);
	if (dn > -1e-4f) {
		return false; // Along the panel or away from its face.
	}
	const float t = m_vec3_dot(p->pose.position - o, n) / dn;
	if (t <= 0.0f || t > 10.0f) {
		return false;
	}
	const xrt_vec3 hit = o + d * t;
	struct xrt_quat inv;
	math_quat_invert(&p->pose.orientation, &inv);
	const xrt_vec3 local = rotate(inv, hit - p->pose.position);
	const float px = (local.x / kPanelWidthM + 0.5f) * kTabW;
	const float py = (0.5f - local.y / p->height_m) * p->layout.h;
	*out_to = hit;
	if (px < 0 || py < 0 || px >= kTabW || py >= p->layout.h) {
		return false;
	}
	*out_px = px;
	*out_py = py;
	return true;
}

float
trigger(struct xrt_device *xdev)
{
	union xrt_input_value v = {};
	if (xdev == nullptr || xrt_device_update_inputs(xdev) != XRT_SUCCESS ||
	    !read_input(xdev, XRT_INPUT_TOUCH_TRIGGER_VALUE, &v)) {
		return 0.0f;
	}
	return v.vec1.x;
}

//! "Set floor": the lower controller lies on the floor, its origin about 4 cm above it, as in SteamVR.
void
set_floor(struct rl_panel *p, int64_t at_ns)
{
	float lowest = 1e9f;
	for (struct xrt_device *xdev : p->hands) {
		struct xrt_space_relation rel = {};
		if (xdev != nullptr && xrt_device_get_tracked_pose(xdev, XRT_INPUT_TOUCH_GRIP_POSE, at_ns, &rel) == XRT_SUCCESS &&
		    (rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) {
			lowest = std::min(lowest, rel.pose.position.y);
		}
	}
	if (lowest > 1e8f) {
		U_LOG_W("Rift panel: no tracked controller to set the floor from");
		return;
	}
	const float floor_y = lowest - 0.04f;
	p->floor_known = true;
	p->floor_m = -floor_y;
	rl::settings::set_number("tracking", "floor_m", -floor_y);
	b_space_overseer_set_floor(p->s->xso, true, floor_y, p->state.height_cm / 100.0f);
	p->floor_lit_until = now_s() + kFloorLitS;
	U_LOG_I("Rift panel: floor set, the lower controller at %.3f m, floor at %.3f m", lowest, floor_y);
}

void
set_height(struct rl_panel *p, int cm, bool save)
{
	p->state.height_cm = std::clamp(cm, kHeightMinCm, kHeightMaxCm);
	cJSON *root = rl::settings::load();
	const cJSON *floor = rl::settings::item(root, "tracking", "floor_m");
	b_space_overseer_set_floor(p->s->xso, cJSON_IsNumber(floor), cJSON_IsNumber(floor) ? -(float)floor->valuedouble : 0.0f,
	                           p->state.height_cm / 100.0f);
	cJSON_Delete(root);
	if (save) {
		rl::settings::set_number("tracking", "height_cm", p->state.height_cm);
	}
}

void
set_volume(struct rl_panel *p, int percent)
{
#ifdef RL_HAVE_PULSE
	if (p->pulse_ok) {
		p->pulse.set_percent(percent, rl::pulse_now_s());
	}
#endif
}

void
set_mic(struct rl_panel *p, int percent)
{
#ifdef RL_HAVE_PULSE
	if (p->pulse_ok) {
		p->pulse.set_mic_percent(percent);
	}
#endif
}

//! A slider moved to @p frac (0..1); @p done: the trigger was let go, so the value is saved.
void
slide(struct rl_panel *p, int row, float frac, bool done)
{
	PictureRow &pic = p->state.picture;
	switch (kRows[row].ctl) {
	case Ctl::Volume: set_volume(p, (int)std::lround(frac * 100.0f)); break;
	case Ctl::Mic: set_mic(p, (int)std::lround(frac * 100.0f)); break;
	case Ctl::Height: set_height(p, (int)std::lround(kHeightMinCm + frac * (kHeightMaxCm - kHeightMinCm)), done); break;
	case Ctl::Strength:
		pic.strength = (int)std::lround(frac * 100.0f);
		apply_picture(p);
		if (done) {
			rl::settings::set_number("display", "auto_hdr_strength", pic.strength);
		}
		break;
	case Ctl::Colour:
		pic.colour = (int)std::lround(frac * 100.0f);
		apply_picture(p);
		if (done) {
			rl::settings::set_number("display", "auto_hdr_colour_strength", pic.colour);
		}
		break;
	case Ctl::RenderScale:
		p->state.info.scale_saved = (int)std::lround((kScaleMin + frac * (kScaleMax - kScaleMin)) / 5.0f) * 5;
		if (done) {
			rl::settings::set_number("display", "render_scale", p->state.info.scale_saved);
		}
		break;
	default: break;
	}
}

//! The tab's "Measure", below.
void
recalibrate(struct rl_panel *p);

void
show(struct rl_panel *p, bool visible);

/*!
 * The application to close: the active one, else (an overlay such as WayVR is never the active one) the last
 * connected with a session. Call with global_state.lock held; -1 for none.
 */
int
app_to_close_locked(struct rl_panel *p)
{
	if (p->s->global_state.active_client_index >= 0) {
		return p->s->global_state.active_client_index;
	}
	for (int i = IPC_MAX_CLIENTS - 1; i >= 0; i--) {
		if (p->s->threads[i].state == IPC_THREAD_RUNNING && p->s->threads[i].ics.xs != nullptr) {
			return i;
		}
	}
	return -1;
}

//! "Close the application": asked to quit as OpenXR has it (its session goes to EXITING).
void
close_app(struct rl_panel *p, double t)
{
	if (p->closing.stage > 0) {
		return;
	}
	os_mutex_lock(&p->s->global_state.lock);
	const int index = app_to_close_locked(p);
	struct xrt_session *xs = nullptr;
	pid_t pid = 0;
	if (index >= 0 && p->s->threads[index].state == IPC_THREAD_RUNNING) {
		volatile struct ipc_client_state *ics = &p->s->threads[index].ics;
		xs = ics->xs;
		pid = ics->client_state.pid;
		if (xs != nullptr && xrt_session_request_exit(xs) != XRT_SUCCESS) {
			xs = nullptr; // Not asked: ended next.
		}
	}
	os_mutex_unlock(&p->s->global_state.lock);
	if (index < 0) {
		U_LOG_I("Rift panel: no application to close");
		return;
	}
	p->closing.index = index;
	p->closing.pid = pid;
	p->closing.since = t;
	p->closing.stage = 1;
	U_LOG_W("Rift panel: closing the application (pid %d): asked to quit%s", (int)pid,
	        xs != nullptr ? "" : " (no session: ended next)");
	// Out of the way, so the application is seen quitting (the loop closes it: a request left set would open it again).
	p->want_open.store(false);
}

//! An application asked to quit: gone, or ended after a while (SIGTERM at 3 s, SIGKILL at 6 s).
void
close_tick(struct rl_panel *p, double t)
{
	if (p->closing.stage == 0) {
		return;
	}
	os_mutex_lock(&p->s->global_state.lock);
	const struct ipc_thread &th = p->s->threads[p->closing.index];
	const bool still = th.state == IPC_THREAD_RUNNING && th.ics.client_state.pid == p->closing.pid;
	os_mutex_unlock(&p->s->global_state.lock);
	const double since = t - p->closing.since;
	const pid_t pid = p->closing.pid;
	if (!still) {
		U_LOG_W("Rift panel: the application quit (%.1f s)", since);
		p->closing.stage = 0;
		return;
	}
	// Never this process, nor one it can't be.
	const bool killable = pid > 1 && pid != getpid();
	if (p->closing.stage == 1 && since >= 3.0) {
		p->closing.stage = 2;
		if (killable && kill(pid, SIGTERM) == 0) {
			U_LOG_W("Rift panel: the application (pid %d) did not quit: ended (SIGTERM)", (int)pid);
		}
	} else if (p->closing.stage == 2 && since >= 6.0) {
		p->closing.stage = 3;
		if (killable && kill(pid, SIGKILL) == 0) {
			U_LOG_W("Rift panel: the application (pid %d) still running: killed (SIGKILL)", (int)pid);
		}
	} else if (since >= 10.0) {
		U_LOG_W("Rift panel: the application (pid %d) could not be closed", (int)pid);
		p->closing.stage = 0;
	}
}

//! A switch, button or page pressed and let go on the same one (rl::tab::Press::release).
void
act(struct rl_panel *p, const Hit &hit, int64_t at_ns)
{
	PictureRow &pic = p->state.picture;
	if (hit.part == Hit::Page) {
		p->state.page = hit.row;
		return;
	}
	if (hit.row < 0) {
		return;
	}
	const Ctl ctl = kRows[hit.row].ctl;
	if (hit.part == Hit::Small) {
		switch (ctl) {
		case Ctl::Volume:
#ifdef RL_HAVE_PULSE
			if (p->pulse_ok) {
				p->pulse.set_muted(!p->state.muted, rl::pulse_now_s());
			}
#endif
			break;
		case Ctl::Mic:
#ifdef RL_HAVE_PULSE
			if (p->pulse_ok) {
				p->pulse.set_mic_muted(!p->state.mic_muted);
			}
#endif
			break;
		case Ctl::Height: set_height(p, 0, true); break;
		case Ctl::Strength: slide(p, hit.row, 0.5f, true); break;
		case Ctl::Colour: slide(p, hit.row, 0.5f, true); break;
		case Ctl::RenderScale: slide(p, hit.row, (100.0f - kScaleMin) / (kScaleMax - kScaleMin), true); break;
		default: break;
		}
		return;
	}
	switch (ctl) {
	case Ctl::Floor: set_floor(p, at_ns); break;
	case Ctl::Recalibrate:
		// Greyed (and no press) while measuring is under way.
		if (!p->state.info.sensors_measuring) {
			recalibrate(p);
		}
		break;
	case Ctl::AutoHdr:
		pic.on = !pic.on;
		rl::settings::set_bool("display", "auto_hdr", pic.on);
		break;
	case Ctl::Bright:
		pic.bright = !pic.bright;
		rl::settings::set_bool("display", "hdr_bright", pic.bright);
		break;
	case Ctl::Smoothing:
		pic.smoothing = !pic.smoothing;
		rl::settings::set_bool("display", "motion_smoothing", pic.smoothing);
		break;
	case Ctl::FastPath:
		p->state.info.fast_path = !p->state.info.fast_path;
		rl::settings::set_bool("display", "direct_mode_fast_path", p->state.info.fast_path);
		break;
	case Ctl::PerfOn:
		p->state.info.perf_on = !p->state.info.perf_on;
		rl::settings::set_bool("tab", "performance", p->state.info.perf_on);
		break;
	case Ctl::CloseApp:
		// Greyed (and no press) without an application, or while one is being closed.
		if (p->state.info.app_running && p->closing.stage == 0) {
			close_app(p, now_s());
		}
		break;
	default: break;
	}
	apply_picture(p);
}

void
handle_input(struct rl_panel *p, int64_t at_ns)
{
	bool pressed[2], released[2];
	for (int h = 0; h < 2; h++) {
		const float v = trigger(p->hands[h]);
		pressed[h] = !p->trigger_down[h] && v > kTriggerPress;
		released[h] = p->trigger_down[h] && v < kTriggerRelease;
		if (pressed[h]) {
			p->trigger_down[h] = true;
		} else if (released[h]) {
			p->trigger_down[h] = false;
		}
	}
	// The laser is the hand whose trigger was pressed last.
	for (int h = 0; h < 2; h++) {
		if (pressed[h] && p->hands[h] != nullptr) {
			p->hand = h;
		}
	}
	float x = 0, y = 0;
	p->hit_valid = point(p, p->hand, at_ns, &x, &y, &p->beam_from, &p->beam_to);
	p->beam_valid = p->hands[p->hand] != nullptr;
	p->hit_px = x;
	p->hit_py = y;
	p->hit_world = p->beam_to;
	// A grabbed knob follows the laser from where it was grabbed, past a few pixels; saved when let go.
	if (p->drag_row >= 0) {
		const std::optional<float> f = p->hit_valid ? p->press.drag(p->layout, x) : std::nullopt;
		if (f) {
			slide(p, p->drag_row, *f, false);
		}
		if (released[p->hand]) {
			if (p->press.moved) {
				slide(p, p->drag_row, f ? *f : slider_frac(p->state, kRows[p->drag_row].ctl), true);
			}
			p->drag_row = -1;
			p->press.hit = {};
		}
		return;
	}
	// Not right after measuring: the controllers come back then, with the trigger maybe still pressed.
	if (pressed[p->hand] && p->hit_valid && now_s() - p->measuring_ended_s >= 1.5) {
		const Hit hit = p->press.begin(p->layout, p->state.page, x, y, p->state);
		if (hit.part == Hit::Track) {
			p->drag_row = hit.row;
		}
	}
	// Switches, buttons and pages act when let go on the one pressed.
	if (released[p->hand]) {
		const Hit let_go = p->press.release(p->layout, p->state.page, x, y, p->state);
		if (let_go.row >= 0 && p->hit_valid) {
			act(p, let_go, at_ns);
		}
	}
}

/*
 *
 * Showing it.
 *
 */

//! In front of the head, level, facing it.
bool
place(struct rl_panel *p, int64_t at_ns)
{
	struct xrt_space_relation rel = {};
	if (xrt_device_get_tracked_pose(p->head, XRT_INPUT_GENERIC_HEAD_POSE, at_ns, &rel) != XRT_SUCCESS ||
	    (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return false;
	}
	const xrt_vec3 fwd = rotate(rel.pose.orientation, {0.0f, 0.0f, -1.0f});
	const float yaw = std::atan2(-fwd.x, -fwd.z);
	const xrt_vec3 up = {0.0f, 1.0f, 0.0f};
	math_quat_from_angle_vector(yaw, &up, &p->pose.orientation);
	p->pose.position = rel.pose.position + rotate(p->pose.orientation, {0.0f, -kPanelBelowM, -kPanelAheadM});
	return true;
}

void
quad(struct xrt_layer_data *q, int64_t at_ns, const struct xrt_pose &pose, float w, float h, int img_w, int img_h,
     uint32_t index)
{
	*q = {};
	q->type = XRT_LAYER_QUAD;
	q->name = XRT_INPUT_GENERIC_HEAD_POSE;
	q->timestamp = at_ns;
	q->flags = (enum xrt_layer_composition_flags)(XRT_LAYER_COMPOSITION_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
	                                              XRT_LAYER_COMPOSITION_UNPREMULTIPLIED_ALPHA_BIT);
	q->color_scale = {1.0f, 1.0f, 1.0f, 1.0f};
	q->quad.visibility = XRT_LAYER_EYE_VISIBILITY_BOTH;
	q->quad.sub.image_index = index;
	q->quad.sub.rect.extent = {img_w, img_h};
	q->quad.sub.norm_rect = {0.0f, 0.0f, 1.0f, 1.0f};
	q->quad.pose = pose;
	q->quad.size = {w, h};
}

void
show(struct rl_panel *p, bool visible)
{
	// Closed with a notice up: still drawn, without the input.
	xrt_syscomp_set_state(p->s->xsysc, &p->xcn->base, visible || p->xsc_notice != nullptr, visible,
	                      os_monotonic_get_ns());
	ipc_server_set_panel_focus(p->s, visible);
	p->open = visible;
	p->drag_row = -1;
	U_LOG_I("Rift panel: %s", visible ? "opened" : "closed");
}

/*
 *
 * Notices and the sensors measured again.
 *
 */

//! The tracker's notices (any thread): queued for this thread, as the SteamVR driver's switches let them through.
void
notice_sink(void *ptr, enum u_notice_kind kind, const char *key, int number)
{
	auto *p = static_cast<struct rl_panel *>(ptr);
	if (key == nullptr) {
		return;
	}
	if (strcmp(key, "calibration-done") == 0) {
		p->recal_measuring.store(false);
		struct u_notice_recalibration change = {};
		if (u_notice_get_recalibration(&change) && p->recal_seen.exchange(change.sequence) != change.sequence) {
			p->recal_done.store(true);
		}
	}
	static const char *const kKinds[] = {"sensors", "calibration", "tracking", "display", "system"};
	cJSON *root = rl::settings::load();
	const int k = static_cast<int>(kind);
	const bool on = rl::settings::boolean(root, "notices", "enabled", true) &&
	                (k < 0 || k > U_NOTICE_SYSTEM || rl::settings::boolean(root, "notices", kKinds[k], true));
	cJSON_Delete(root);
	if (!on) {
		return;
	}
	const std::string name = number > 0 ? std::string(key) + "-" + std::to_string(number) : std::string(key);
	std::lock_guard<std::mutex> lock(p->notice_lock);
	if (p->notices.size() < 6) {
		p->notices.push_back(name);
	}
}

//! The tab's "Measure": every sensor measured again, as on a first start.
void
recalibrate(struct rl_panel *p)
{
	const bool started = u_notice_request_recalibration();
	if (started) {
		p->recal_measuring.store(true);
	}
	U_LOG_I("Rift panel: measuring every sensor again: %s",
	        started ? "started" : "refused (no tracker, or the sensors are being solved right now)");
}

void
keep_floor(struct rl_panel *p, const rl::floor::Result &r)
{
	U_LOG_I("Rift panel: sensors measured again: floor %.3f m (was %.3f), %s", r.floor_m, p->floor_m, r.how.c_str());
	p->floor_m = r.floor_m;
	rl::settings::set_number("tracking", "floor_m", r.floor_m);
	b_space_overseer_set_floor(p->s->xso, true, -r.floor_m, p->state.height_cm / 100.0f);
}

//! The sensors measured again (recentred, the floor carried over), and the head's heights for the eye height.
void
floor_tick(struct rl_panel *p)
{
	const double t = now_s();
	if (p->recal_done.exchange(false)) {
		struct u_notice_recalibration change = {};
		(void)u_notice_get_recalibration(&change);
		if (change.kept) {
			U_LOG_I("Rift panel: sensors measured again: the room is where it was (%d sensors kept their place)",
			        change.anchors);
		} else {
			// The world placed on the user anew once the head has settled in the new measurement, as the SteamVR
			// driver does; at once it took the head's pose from before it.
			p->resettle.arm(t);
			U_LOG_I("Rift panel: sensors measured again: no sensor kept its place, recentring once the head settles");
		}
		if (p->floor_known && !change.kept) {
			const std::optional<rl::floor::Result> r = p->keeper.done(p->floor_m, change, t);
			if (r) {
				keep_floor(p, *r);
			} else {
				U_LOG_I("Rift panel: a sensor moved up or down; the floor follows from your head's height in %.0f s",
				        rl::floor::Keeper::kWaitS);
			}
		}
	}
	if (const std::optional<rl::floor::Result> r = p->keeper.tick(t)) {
		keep_floor(p, *r);
	}
	if (const std::optional<float> e = p->keeper.eye_to_save()) {
		rl::settings::set_number("tracking", "eye_height_m", *e);
		U_LOG_I("Rift panel: standing eye height learned: %.2f m", *e);
	}
	if (t < p->next_head_s || p->recal_measuring.load()) {
		return;
	}
	p->next_head_s = t + 0.1;
	union xrt_input_value worn = {};
	if (read_input(p->head, XRT_INPUT_GENERIC_HEAD_DETECT, &worn) && !worn.boolean) {
		p->resettle.lost();
		return;
	}
	struct xrt_space_relation rel = {};
	if (xrt_device_get_tracked_pose(p->head, XRT_INPUT_GENERIC_HEAD_POSE, os_monotonic_get_ns(), &rel) != XRT_SUCCESS ||
	    (rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) == 0) {
		p->resettle.lost();
		return;
	}
	if (p->resettle.armed()) {
		const struct xrt_vec3 minus_z = {0.0f, 0.0f, -1.0f};
		struct xrt_vec3 forward;
		math_quat_rotate_vec3(&rel.pose.orientation, &minus_z, &forward);
		if (p->resettle.head(t, atan2f(-forward.x, -forward.z), rel.pose.position)) {
			xrt_space_overseer_recenter_local_spaces(p->s->xso);
			U_LOG_I("Rift panel: recentred after the sensors were measured again");
		}
	}
	p->keeper.head(rel.pose.position.y, t, p->floor_known ? std::optional<float>(p->floor_m) : std::nullopt);
}

//! The next notice up when the last one is done; the client drawn for it while the panel is closed.
void
notice_tick(struct rl_panel *p)
{
	const double t = now_s();
	// What is happening now ("measuring", "a sensor moved") stays up while it lasts (the tracker's state), at least
	// a few seconds; the notice that it is done follows it at once.
	if (p->xsc_notice != nullptr && p->notice_lasting) {
		struct u_notice_sensors sensors = {};
		const bool still = u_notice_get_sensors(&sensors) && sensors.measuring;
		bool next = false;
		{
			std::lock_guard<std::mutex> lock(p->notice_lock);
			next = !p->notices.empty();
		}
		if (t - p->notice_since >= 3.0 && (!still || next)) {
			p->notice_lasting = false;
			p->notice_since = t - (kNoticeShowS - kNoticeFadeS);
		}
	}
	if (p->xsc_notice != nullptr && !p->notice_lasting && t - p->notice_since >= kNoticeShowS) {
		xrt_swapchain_reference(&p->xsc_notice, nullptr);
		if (!p->open) {
			xrt_syscomp_set_state(p->s->xsysc, &p->xcn->base, false, false, os_monotonic_get_ns());
		}
	}
	if (p->xsc_notice != nullptr) {
		return;
	}
	std::string name;
	{
		std::lock_guard<std::mutex> lock(p->notice_lock);
		if (p->notices.empty()) {
			return;
		}
		name = p->notices.front();
		p->notices.pop_front();
	}
	const std::string path = pieces_dir() + "/" + name + ".png";
	int w = 0, h = 0, n = 0;
	stbi_uc *px = stbi_load(path.c_str(), &w, &h, &n, 4);
	if (px == nullptr) {
		U_LOG_W("Rift panel: no notice panel %s", path.c_str());
		return;
	}
	Image img;
	img.w = w;
	img.h = h;
	img.px.assign(px, px + (size_t)w * (size_t)h * 4);
	stbi_image_free(px);
	p->xsc_notice = make_static(p, img);
	if (p->xsc_notice == nullptr) {
		U_LOG_W("Rift panel: notice %s not shown (no image)", name.c_str());
		return;
	}
	p->notice_w = w;
	p->notice_h = h;
	p->notice_since = t;
	p->notice_lasting = name == "calibration-start" || name.rfind("sensor-moved-", 0) == 0;
	if (!p->open) {
		xrt_syscomp_set_state(p->s->xsysc, &p->xcn->base, true, false, os_monotonic_get_ns());
	}
	U_LOG_I("Rift panel: showing notice %s", name.c_str());
}

void
frame(struct rl_panel *p)
{
	struct xrt_compositor *xc = &p->xcn->base;
	int64_t frame_id = -1, display_ns = 0, period_ns = 0;
	if (xrt_comp_wait_frame(xc, &frame_id, &display_ns, &period_ns) != XRT_SUCCESS ||
	    xrt_comp_begin_frame(xc, frame_id) != XRT_SUCCESS) {
		os_nanosleep(U_TIME_1MS_IN_NS * 10);
		return;
	}
	const double t = now_s();
	update_status(p, t);
	perf_tick(p, t);
#ifdef RL_HAVE_PULSE
	if (p->pulse_ok) {
		p->state.volume = p->pulse.known ? p->pulse.percent : -1;
		p->state.muted = p->pulse.muted;
		p->state.mic = p->pulse.mic_known ? p->pulse.mic_percent : -1;
		p->state.mic_muted = p->pulse.mic_muted;
	}
#endif
	if (p->open) {
		handle_input(p, display_ns);
		redraw(p);
	}

	struct xrt_layer_frame_data fd = {};
	fd.frame_id = frame_id;
	fd.display_time_ns = display_ns;
	fd.env_blend_mode = XRT_BLEND_MODE_OPAQUE;
	xrt_comp_layer_begin(xc, &fd);
	struct xrt_layer_data q;
	if (p->open && p->panel_ready) {
		quad(&q, display_ns, p->pose, kPanelWidthM, p->height_m, kTabW, p->layout.h, p->panel_index);
		xrt_comp_layer_quad(xc, p->head, p->xsc_panel, &q);
	}
	if (p->open && p->beam_valid && p->xsc_beam != nullptr) {
		// A thin quad along the beam, turned about it to face the head.
		struct xrt_space_relation head = {};
		xrt_device_get_tracked_pose(p->head, XRT_INPUT_GENERIC_HEAD_POSE, display_ns, &head);
		const xrt_vec3 along = p->beam_to - p->beam_from;
		const float length = m_vec3_len(along);
		if (length > 0.01f) {
			const xrt_vec3 y = along * (1.0f / length);
			const xrt_vec3 mid = p->beam_from + along * 0.5f;
			xrt_vec3 z = head.pose.position - mid;
			z = z - y * m_vec3_dot(z, y);
			if (m_vec3_len(z) > 1e-4f) {
				z = m_vec3_normalize(z);
				xrt_vec3 x;
				math_vec3_cross(&y, &z, &x);
				struct xrt_pose beam_pose = {};
				math_quat_from_plus_x_z(&x, &z, &beam_pose.orientation);
				beam_pose.position = mid;
				quad(&q, display_ns, beam_pose, kBeamWidthM, length, 8, 128, 0);
				xrt_comp_layer_quad(xc, p->head, p->xsc_beam, &q);
			}
		}
	}
	if (p->open && p->hit_valid && p->xsc_dot != nullptr) {
		struct xrt_pose dot_pose = p->pose;
		dot_pose.position = p->hit_world + rotate(p->pose.orientation, {0.0f, 0.0f, 0.004f});
		quad(&q, display_ns, dot_pose, kCursorM, kCursorM, 64, 64, 0);
		xrt_comp_layer_quad(xc, p->head, p->xsc_dot, &q);
	}
	if (p->xsc_notice != nullptr) {
		// Following the view (view space), faded in and out; last, so over the panel, the beam and the cursor.
		const double age = now_s() - p->notice_since;
		const double left = p->notice_lasting ? kNoticeShowS : kNoticeShowS - age;
		const float alpha = (float)std::clamp(std::min(age, left) / kNoticeFadeS, 0.0, 1.0);
		struct xrt_pose at = XRT_POSE_IDENTITY;
		at.position = {0.0f, -kNoticeBelowM, -kNoticeAheadM};
		quad(&q, display_ns, at, kNoticeWidthM, kNoticeWidthM * p->notice_h / p->notice_w, p->notice_w, p->notice_h, 0);
		q.flags = (enum xrt_layer_composition_flags)(q.flags | XRT_LAYER_COMPOSITION_VIEW_SPACE_BIT);
		q.color_scale = {1.0f, 1.0f, 1.0f, alpha};
		xrt_comp_layer_quad(xc, p->head, p->xsc_notice, &q);
	}
	xrt_comp_layer_commit(xc, XRT_GRAPHICS_SYNC_HANDLE_INVALID);
}

void
run(struct rl_panel *p)
{
#ifdef RL_HAVE_AUDIO_SWITCH
	if (strncmp(p->head->serial, "WMHD", 4) == 0) {
		p->audio.to_headset(rl::audio::rift_sink(p->head->serial), rl::audio::rift_source_prefix(p->head->serial),
		                    log_line);
	}
#endif
	while (p->running.load()) {
#ifdef RL_HAVE_PULSE
		if (p->pulse_ok) {
			p->pulse.tick(rl::pulse_now_s());
		}
#endif
		floor_tick(p);
		notice_tick(p);
		// Closing an application goes on with the panel closed (it closes itself on the press).
		if (p->want_close.exchange(false)) {
			close_app(p, now_s());
		}
		close_tick(p, now_s());
		const bool want = p->want_open.load();
		if (want && !p->open) {
			if (!place(p, os_monotonic_get_ns())) {
				p->want_open.store(false);
				U_LOG_W("Rift panel: the head is not tracked, not opened");
				continue;
			}
			show(p, true);
		} else if (!want && p->open) {
			show(p, false);
		}
		if (p->open) {
			// Taken off: closed.
			if (p->state.info.worn == 0 && !p->test_mode) {
				p->want_open.store(false);
			}
			frame(p);
		} else if (p->xsc_notice != nullptr) {
			frame(p);
		} else {
			os_nanosleep(U_TIME_1MS_IN_NS * 50);
		}
	}
	if (p->open) {
		show(p, false);
	}
#ifdef RL_HAVE_AUDIO_SWITCH
	p->audio.restore(log_line);
#endif
}

} // namespace

extern "C" void
rl_panel_settings_to_env(void)
{
	cJSON *root = rl::settings::load();
	const std::string headset = rl::settings::headset_to_env(root);
	if (!headset.empty()) {
		U_LOG_I("Headset settings: %s", headset.c_str());
	}
	const double scale = rl::settings::number(root, "display", "render_scale", 0.0);
	if (scale >= 25.0 && scale <= 400.0 && getenv("XRT_COMPOSITOR_RENDER_SCALE") == nullptr) {
		char text[32];
		snprintf(text, sizeof(text), "%d", (int)std::lround(scale));
		setenv("XRT_COMPOSITOR_RENDER_SCALE", text, 0);
		// The layer renderer's images grow with it, as the SteamVR driver sets them.
		if (getenv("XRT_COMPOSITOR_SCRATCH_HEADROOM") == nullptr) {
			snprintf(text, sizeof(text), "%d", (int)std::lround(125.0 * std::sqrt(scale / 100.0)));
			setenv("XRT_COMPOSITOR_SCRATCH_HEADROOM", text, 0);
		}
	}
	cJSON_Delete(root);
}

extern "C" struct rl_panel *
rl_panel_create(struct ipc_server *s)
{
	if (s->xsysc == nullptr || s->xsysd == nullptr || s->xsysd->static_roles.head == nullptr || s->xso == nullptr) {
		return nullptr;
	}
	auto *p = new rl_panel();
	p->s = s;
	p->head = s->xsysd->static_roles.head;
	for (size_t d = 0; d < s->xsysd->static_xdev_count; d++) {
		struct xrt_device *xdev = s->xsysd->static_xdevs[d];
		if (xdev == nullptr) {
			continue;
		}
		if (xdev->device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
			p->hands[0] = xdev;
		} else if (xdev->device_type == XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER) {
			p->hands[1] = xdev;
		}
	}
	const char *scale = getenv("XRT_COMPOSITOR_RENDER_SCALE");
	p->scale_running = scale != nullptr ? std::clamp(atoi(scale), kScaleMin, kScaleMax) : 100;
	load_settings(p);

	const std::string dir = pieces_dir();
	if (dir.empty() || !p->pieces.load(dir)) {
		U_LOG_W("Rift panel: no tab pieces found (the SteamVR driver package has them); no panel");
		delete p;
		return nullptr;
	}
	p->layout = tab_layout(p->pieces, true);
	p->height_m = kPanelWidthM * p->layout.h / kTabW;

	struct xrt_session_info xsi = {};
	xsi.is_overlay = true;
	xsi.z_order = UINT32_MAX; // The session info's own field; the place is set below.
	struct xrt_begin_session_info bsi = {};
	bsi.view_type = XRT_VIEW_TYPE_STEREO;
	if (xrt_syscomp_create_native_compositor(s->xsysc, &xsi, &g_events, &p->xcn) != XRT_SUCCESS ||
	    xrt_comp_begin_session(&p->xcn->base, &bsi) != XRT_SUCCESS) {
		U_LOG_W("Rift panel: the compositor took no client; no panel");
		xrt_comp_native_destroy(&p->xcn);
		delete p;
		return nullptr;
	}
	xrt_syscomp_set_state(s->xsysc, &p->xcn->base, false, false, os_monotonic_get_ns());
	// Above the application (the multi-client compositor sorts its clients by this, not by the session info).
	xrt_syscomp_set_z_order(s->xsysc, &p->xcn->base, kZOrder);
	p->xsc_panel = make_swapchain(p, kTabW, p->layout.h, false);
	if (p->xsc_panel != nullptr) {
		p->vk = ((struct comp_swapchain *)p->xsc_panel)->vk;
	}
	if (p->vk == nullptr || vk_cmd_pool_init(p->vk, &p->pool, VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT) != VK_SUCCESS) {
		U_LOG_W("Rift panel: no images; no panel");
		xrt_swapchain_reference(&p->xsc_panel, nullptr);
		xrt_comp_end_session(&p->xcn->base);
		xrt_comp_native_destroy(&p->xcn);
		delete p;
		return nullptr;
	}
	p->xsc_dot = make_static(p, make_dot(64));
	p->xsc_beam = make_static(p, make_beam(8, 128));
#ifdef RL_HAVE_PULSE
	p->pulse_ok = p->pulse.load();
#endif
	U_LOG_I("Rift panel: ready (pieces in %s), the right Oculus button opens it", dir.c_str());

	// Test keys for desk checks without anyone pressing anything: open the panel, or press "Close the application",
	// this many seconds after start.
	const char *test = getenv("RIFTLINUX_PANEL_TEST_OPEN_S");
	const double test_s = test != nullptr ? atof(test) : -1.0;
	const char *test_close = getenv("RIFTLINUX_PANEL_TEST_CLOSE_S");
	const double test_close_s = test_close != nullptr ? atof(test_close) : -1.0;
	p->test_mode = test_s >= 0.0;
	p->thread = std::thread([p, test_s, test_close_s] {
		if (test_s >= 0.0) {
			std::thread([p, test_s] {
				std::this_thread::sleep_for(std::chrono::duration<double>(test_s));
				p->want_open.store(true);
			}).detach();
		}
		if (test_close_s >= 0.0) {
			std::thread([p, test_close_s] {
				std::this_thread::sleep_for(std::chrono::duration<double>(test_close_s));
				p->want_close.store(true);
			}).detach();
		}
		run(p);
	});
	// The tracker's notices: shown by this panel (monado-service has nothing else to show them).
	u_notice_set_sink(notice_sink, p);
	return p;
}

extern "C" void
rl_panel_destroy(struct rl_panel **panel_ptr)
{
	struct rl_panel *p = *panel_ptr;
	if (p == nullptr) {
		return;
	}
	*panel_ptr = nullptr;
	// Waits for a notice being posted, so none reaches a panel going away.
	u_notice_set_sink(nullptr, nullptr);
	p->running.store(false);
	if (p->thread.joinable()) {
		p->thread.join();
	}
	xrt_swapchain_reference(&p->xsc_notice, nullptr);
	xrt_swapchain_reference(&p->xsc_panel, nullptr);
	xrt_swapchain_reference(&p->xsc_dot, nullptr);
	xrt_swapchain_reference(&p->xsc_beam, nullptr);
	if (p->vk != nullptr) {
		vk_cmd_pool_destroy(p->vk, &p->pool);
	}
	xrt_comp_end_session(&p->xcn->base);
	xrt_comp_native_destroy(&p->xcn);
	delete p;
}

extern "C" void
rl_panel_toggle(struct rl_panel *panel)
{
	if (panel != nullptr) {
		panel->want_open.store(!panel->want_open.load());
	}
}
