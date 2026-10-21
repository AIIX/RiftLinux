// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Picture settings of the distortion pass, changeable while running (RiftLinux).
 *
 * Start from the environment (XRT_COMPOSITOR_*); a host such as the SteamVR driver can replace them at any time
 * (the Rift tab's switch), the distortion pass reads them every frame.
 *
 * @ingroup aux_render
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct render_output_settings
{
	//! Dither to this many bits per channel, the link's depth (0 off).
	float dither_bits;
	//! sRGB levels 1..N the panel shows as black, lifted to N + 1 (0 off, at most 6).
	float black_level;
	//! Tone-map shoulder for content brighter than white (0 off).
	float tonemap_shoulder;
	//! Strength of the colour stage on its own; 0 uses Auto-HDR's colour.
	float vibrance;

	//! Auto-HDR (inverse tone mapping) and its strengths.
	bool auto_hdr;
	float auto_hdr_highlights;
	float auto_hdr_shadows;
	float auto_hdr_colour;
	float auto_hdr_knee;

	//! HDR layers: the panel's OKLab lightness for their reference white (0.88: about sRGB 215 of 255), the room above
	//! it is for their highlights.
	float hdr_white;
	//! HDR Bright: @ref hdr_white_bright instead, for a brighter HDR picture with less room for highlights.
	bool hdr_bright;
	float hdr_white_bright;
	//! Float images without an HDR description (HDR games through SteamVR) taken as linear HDR;
	//! off: as SDR, clipped at white.
	bool hdr_float_layers;
	//! HDR layers' brightness metadata (mastering display, MaxCLL, per-scene peak) used for their mapping; off: as if
	//! none came (the peak measured). Transfer and primaries are always used: they say how to decode the layer.
	bool hdr_metadata;
	//! HDR layers' peak measured while compositing; off: from metadata, else 1000 nits, and a float
	//! layer without metadata is taken as HDR whatever it holds.
	bool hdr_measured_peak;

	//! Motion smoothing: in-between frames when an application is late (render_motion.c).
	bool motion_smoothing;

	//! A single projection layer read by the lens pass directly (comp_compositor.c); off: the layer renderer first.
	bool fast_path;
};

/*!
 * The current settings: the environment's until set. Thread safe.
 */
void
render_output_settings_get(struct render_output_settings *out_settings);

/*!
 * Replace the settings, used from the next frame on. Thread safe.
 */
void
render_output_settings_set(const struct render_output_settings *settings);

/*!
 * An HDR layer was composited at @p now_ns (one mapped as HDR more than half: HDR Bright acts on it). For the Rift
 * tab's "HDR content on screen" (HDR Bright does nothing to other content). Thread safe.
 */
void
render_output_note_hdr_layer(int64_t now_ns);

/*!
 * When an HDR layer was last composited, 0 never. Thread safe.
 */
int64_t
render_output_last_hdr_layer_ns(void);

#ifdef __cplusplus
}
#endif
