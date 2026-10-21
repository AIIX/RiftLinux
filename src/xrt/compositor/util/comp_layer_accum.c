// Copyright 2019-2024, Collabora, Ltd.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Re-assemble a collection of composition layers submitted for a frame.
 *
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @ingroup comp_util
 */

#include "comp_layer_accum.h"
#include "comp_swapchain.h"
#include "render/render_output_settings.h"
#include "util/u_misc.h"
#include "xrt/xrt_compositor.h"
#include "xrt/xrt_limits.h"

/*!
 * A layer drawn into float images without an HDR description (RiftLinux): linear light with white at
 * 1.0 and highlights above it, as games render for HDR (SteamVR hands such images over as they are). Described as a
 * linear HDR layer, so the compositor measures its peak and maps what goes past white instead of clipping it; while
 * nothing much does (an SDR game in a float image) it looks as in an sRGB image. Off (XRT_COMPOSITOR_HDR_FLOAT_LAYERS=0,
 * or live through render_output_settings): as SDR, as before.
 */
static void
describe_float_layer(struct xrt_swapchain *xsc, struct comp_layer *layer)
{
	// Described by its producer (even as SDR): as said.
	if (xsc == NULL || layer->data.hdr.transfer != XRT_LAYER_TRANSFER_SDR ||
	    (layer->data.hdr.flags & XRT_LAYER_HDR_DESCRIBED) != 0) {
		return;
	}
	struct render_output_settings settings;
	render_output_settings_get(&settings);
	if (!settings.hdr_float_layers) {
		return;
	}
	switch (comp_swapchain(xsc)->vkic.info.format) {
	case 90:  // VK_FORMAT_R16G16B16_SFLOAT
	case 97:  // VK_FORMAT_R16G16B16A16_SFLOAT
	case 106: // VK_FORMAT_R32G32B32_SFLOAT
	case 109: // VK_FORMAT_R32G32B32A32_SFLOAT
	case 122: // VK_FORMAT_B10G11R11_UFLOAT_PACK32
	case 123: // VK_FORMAT_E5B9G9R9_UFLOAT_PACK32
		layer->data.hdr.transfer = XRT_LAYER_TRANSFER_LINEAR;
		layer->data.hdr.primaries = XRT_LAYER_PRIMARIES_BT709;
		break;
	default: break;
	}
}

// Shared implementation of accumulating a layer with only a single swapchain image.
static xrt_result_t
push_single_swapchain_layer(struct comp_layer_accum *cla, struct xrt_swapchain *xsc, const struct xrt_layer_data *data)
{
	uint32_t layer_id = cla->layer_count;

	struct comp_layer *layer = &cla->layers[layer_id];
	U_ZERO_ARRAY(layer->sc_array);
	layer->sc_array[0] = xsc;
	layer->data = *data;
	describe_float_layer(xsc, layer);

	cla->layer_count++;

	return XRT_SUCCESS;
}

struct xrt_swapchain *
comp_layer_get_swapchain(const struct comp_layer *cl, uint32_t swapchain_index)
{

	return cl->sc_array[swapchain_index];
}

struct xrt_swapchain *
comp_layer_get_depth_swapchain(const struct comp_layer *cl, uint32_t swapchain_index)
{

	assert(cl->data.type == XRT_LAYER_PROJECTION_DEPTH);
	return cl->sc_array[XRT_MAX_VIEWS + swapchain_index];
}

xrt_result_t
comp_layer_accum_begin(struct comp_layer_accum *cla, const struct xrt_layer_frame_data *data)
{
	cla->data = *data;
	cla->layer_count = 0;

	return XRT_SUCCESS;
}

xrt_result_t
comp_layer_accum_projection(struct comp_layer_accum *cla,
                            struct xrt_swapchain *xsc[XRT_MAX_VIEWS],
                            const struct xrt_layer_data *data)
{

	uint32_t layer_id = cla->layer_count;

	struct comp_layer *layer = &cla->layers[layer_id];
	assert(ARRAY_SIZE(layer->sc_array) >= data->view_count);
	U_ZERO_ARRAY(layer->sc_array);
	for (uint32_t i = 0; i < data->view_count; ++i) {
		layer->sc_array[i] = xsc[i];
	}
	layer->data = *data;
	describe_float_layer(xsc[0], layer);

	cla->layer_count++;

	return XRT_SUCCESS;
}

xrt_result_t
comp_layer_accum_projection_depth(struct comp_layer_accum *cla,
                                  struct xrt_swapchain *xsc[XRT_MAX_VIEWS],
                                  struct xrt_swapchain *d_xsc[XRT_MAX_VIEWS],
                                  const struct xrt_layer_data *data)
{

	uint32_t layer_id = cla->layer_count;

	struct comp_layer *layer = &cla->layers[layer_id];
	U_ZERO_ARRAY(layer->sc_array);
	for (uint32_t i = 0; i < data->view_count; ++i) {
		layer->sc_array[i] = xsc[i];
		layer->sc_array[XRT_MAX_VIEWS + i] = d_xsc[i];
	}
	layer->data = *data;
	describe_float_layer(xsc[0], layer);

	cla->layer_count++;

	return XRT_SUCCESS;
}

xrt_result_t
comp_layer_accum_quad(struct comp_layer_accum *cla, struct xrt_swapchain *xsc, const struct xrt_layer_data *data)
{
	return push_single_swapchain_layer(cla, xsc, data);
}

xrt_result_t
comp_layer_accum_cube(struct comp_layer_accum *cla, struct xrt_swapchain *xsc, const struct xrt_layer_data *data)
{
	return push_single_swapchain_layer(cla, xsc, data);
}

xrt_result_t
comp_layer_accum_cylinder(struct comp_layer_accum *cla, struct xrt_swapchain *xsc, const struct xrt_layer_data *data)
{
	return push_single_swapchain_layer(cla, xsc, data);
}

xrt_result_t
comp_layer_accum_equirect1(struct comp_layer_accum *cla, struct xrt_swapchain *xsc, const struct xrt_layer_data *data)
{
	return push_single_swapchain_layer(cla, xsc, data);
}

xrt_result_t
comp_layer_accum_equirect2(struct comp_layer_accum *cla, struct xrt_swapchain *xsc, const struct xrt_layer_data *data)
{
	return push_single_swapchain_layer(cla, xsc, data);
}
