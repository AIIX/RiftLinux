// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Picture settings of the distortion pass, changeable while running (RiftLinux).
 * @ingroup aux_render
 */

#include "render/render_output_settings.h"

#include "util/u_debug.h"

#include <pthread.h>

/*
 * Output processing in the distortion shader (RiftLinux): dither to the link's bit depth (the panel
 * otherwise shows steps in dark gradients), vibrance (colour only, Auto-HDR's colour stage), a tone-map shoulder for
 * content brighter than the panel, and the panel's black level: the CV1 shows sRGB levels 1-N as black, they are
 * lifted to N+1 so dark detail stays visible (0 off, at most 6; read off the test pattern).
 */
DEBUG_GET_ONCE_FLOAT_OPTION(dither_bits, "XRT_COMPOSITOR_DITHER_BITS", 8.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(vibrance, "XRT_COMPOSITOR_VIBRANCE", 0.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(tonemap_shoulder, "XRT_COMPOSITOR_TONEMAP_SHOULDER", 0.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(black_level, "XRT_COMPOSITOR_BLACK_LEVEL", 0.0f)

/*
 * Auto-HDR (inverse tone mapping): an HDR look on the SDR OLED, which has true blacks
 * but a fixed peak, over the whole picture, in OKLab (distortion.comp): highlights lifted most toward white (white
 * kept, detail below it kept), dark mid-tones deepened, mid-grey kept, chroma raised (skin and greys spared),
 * gamut-mapped. Shadows 0.10 keeps the average brightness (seen on a CV1: at 0.25 the picture was ~3 % darker on
 * average, noticeable); with the defaults sRGB 128 -> 128, 200 -> 208, 220 -> 229, 250 -> 253.
 */
DEBUG_GET_ONCE_BOOL_OPTION(auto_hdr, "XRT_COMPOSITOR_AUTO_HDR", false)
DEBUG_GET_ONCE_FLOAT_OPTION(auto_hdr_highlights, "XRT_COMPOSITOR_AUTO_HDR_HIGHLIGHTS", 0.6f)
DEBUG_GET_ONCE_FLOAT_OPTION(auto_hdr_shadows, "XRT_COMPOSITOR_AUTO_HDR_SHADOWS", 0.10f)
DEBUG_GET_ONCE_FLOAT_OPTION(auto_hdr_colour, "XRT_COMPOSITOR_AUTO_HDR_COLOUR", 0.20f)
DEBUG_GET_ONCE_FLOAT_OPTION(auto_hdr_knee, "XRT_COMPOSITOR_AUTO_HDR_KNEE", 0.55f)

/*
 * HDR layers: where their reference white (203 nits) lands on the panel, as OKLab lightness; above it
 * their highlights are fitted to the scene's peak. Higher: brighter, richer everyday colours, highlights closer
 * together; 0.88 puts white at sRGB 215 and keeps 400, 1000 and 4000 nits apart (242, 251, 255); at 0.85 normal
 * colours looked muted on a CV1.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(hdr_white, "XRT_COMPOSITOR_HDR_WHITE", 0.88f)

/*
 * HDR Bright (the Rift tab; as a TV's bright picture mode): HDR layers' reference white placed
 * higher, everything below it brighter with it, the highlights above it given less room. Off by default.
 */
DEBUG_GET_ONCE_BOOL_OPTION(hdr_bright, "XRT_COMPOSITOR_HDR_BRIGHT", false)
DEBUG_GET_ONCE_FLOAT_OPTION(hdr_white_bright, "XRT_COMPOSITOR_HDR_WHITE_BRIGHT", 0.94f)

/*
 * The HDR parts' own switches, each on by default: float images taken as HDR (games through SteamVR), the content's
 * brightness metadata (from a player's description or an OpenXR application), the measured peak.
 */
DEBUG_GET_ONCE_BOOL_OPTION(hdr_float_layers, "XRT_COMPOSITOR_HDR_FLOAT_LAYERS", true)
DEBUG_GET_ONCE_BOOL_OPTION(hdr_metadata, "XRT_COMPOSITOR_HDR_METADATA", true)
DEBUG_GET_ONCE_BOOL_OPTION(hdr_measured_peak, "XRT_COMPOSITOR_HDR_MEASURED_PEAK", true)

//! Motion smoothing: in-between frames when an application is late (render_motion.c). Off by default.
DEBUG_GET_ONCE_BOOL_OPTION(motion_smoothing, "XRT_COMPOSITOR_MOTION_SMOOTHING", false)

//! The fast path, live (XRT_COMPOSITOR_DISABLE_FAST_PATH still turns it off for good). On by default.
DEBUG_GET_ONCE_BOOL_OPTION(fast_path, "XRT_COMPOSITOR_FAST_PATH", true)

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static bool g_set = false;
static struct render_output_settings g_settings;

static void
from_environment(struct render_output_settings *s)
{
	s->dither_bits = debug_get_float_option_dither_bits();
	s->black_level = debug_get_float_option_black_level();
	s->tonemap_shoulder = debug_get_float_option_tonemap_shoulder();
	s->vibrance = debug_get_float_option_vibrance();
	s->auto_hdr = debug_get_bool_option_auto_hdr();
	s->auto_hdr_highlights = debug_get_float_option_auto_hdr_highlights();
	s->auto_hdr_shadows = debug_get_float_option_auto_hdr_shadows();
	s->auto_hdr_colour = debug_get_float_option_auto_hdr_colour();
	s->auto_hdr_knee = debug_get_float_option_auto_hdr_knee();
	s->hdr_white = debug_get_float_option_hdr_white();
	s->hdr_bright = debug_get_bool_option_hdr_bright();
	s->hdr_white_bright = debug_get_float_option_hdr_white_bright();
	s->hdr_float_layers = debug_get_bool_option_hdr_float_layers();
	s->hdr_metadata = debug_get_bool_option_hdr_metadata();
	s->hdr_measured_peak = debug_get_bool_option_hdr_measured_peak();
	s->motion_smoothing = debug_get_bool_option_motion_smoothing();
	s->fast_path = debug_get_bool_option_fast_path();
}

void
render_output_settings_get(struct render_output_settings *out_settings)
{
	pthread_mutex_lock(&g_lock);
	if (!g_set) {
		from_environment(&g_settings);
		g_set = true;
	}
	*out_settings = g_settings;
	pthread_mutex_unlock(&g_lock);
}

void
render_output_settings_set(const struct render_output_settings *settings)
{
	pthread_mutex_lock(&g_lock);
	g_settings = *settings;
	g_set = true;
	pthread_mutex_unlock(&g_lock);
}

static int64_t g_last_hdr_layer_ns = 0;

void
render_output_note_hdr_layer(int64_t now_ns)
{
	pthread_mutex_lock(&g_lock);
	g_last_hdr_layer_ns = now_ns;
	pthread_mutex_unlock(&g_lock);
}

int64_t
render_output_last_hdr_layer_ns(void)
{
	pthread_mutex_lock(&g_lock);
	const int64_t ns = g_last_hdr_layer_ns;
	pthread_mutex_unlock(&g_lock);
	return ns;
}
