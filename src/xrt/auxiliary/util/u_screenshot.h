// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Screenshots of what the headset shows: asked for by any thread (an application through libmonado), served by
 *         the compositor with its next frame's views, written as one picture of the whole field of view.
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_results.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! How a view's pixels are stored.
enum u_screenshot_format
{
	//! Four half floats a pixel, linear light.
	U_SCREENSHOT_RGBA16F = 0,
	//! Four bytes a pixel, sRGB encoded.
	U_SCREENSHOT_RGBA8_SRGB = 1,
};

//! One eye's composited view, as the compositor hands it over.
struct u_screenshot_view
{
	uint32_t width, height;
	enum u_screenshot_format format;
	//! Rows top down, in the compositor's read-back memory (valid until the screenshot is written); NULL: black.
	const void *pixels;
	//! The field of view it was drawn with.
	struct xrt_fov fov;
};

/*!
 * What the compositor offers screenshots: @p prepare runs on the asking thread before a frame is asked for, so the
 * memory the views are read back into is made there (and kept), not on the compositor's thread; false: none now.
 */
struct u_screenshot_provider
{
	bool (*prepare)(void *ctx);
	void *ctx;
};

/*!
 * For the compositor: offer screenshots (@p provider), or stop (NULL: waits for a screenshot being taken to finish).
 */
void
u_screenshot_set_provider(const struct u_screenshot_provider *provider);

/*!
 * Asks the compositor for its next frame's views and writes them as one PNG to @p png_path (written beside it as
 * "<path>.part", then renamed). Blocks until the file is written or @p timeout_ns passed; never call it from the
 * compositor's thread.
 */
xrt_result_t
u_screenshot_take(const char *png_path, uint64_t timeout_ns);

//! For the compositor, once a frame: whether a screenshot waits for this frame's views.
bool
u_screenshot_wanted(void);

/*!
 * For the compositor: the views of the frame a screenshot waited for (@p count of them, left eye first), and the
 * output's highlight shoulder (0: none). The pixels stay the compositor's; none (@p count 0, a failed copy): the
 * waiting screenshot fails.
 */
void
u_screenshot_deliver(struct u_screenshot_view *views, uint32_t count, float tonemap_shoulder);

#ifdef __cplusplus
}
#endif
