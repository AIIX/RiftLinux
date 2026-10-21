// Copyright 2018-2024, Collabora, Ltd.
// Copyright 2024-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Holds system related entrypoints.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup oxr_main
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "xrt/xrt_device.h"
#include "util/u_debug.h"
#include "util/u_verify.h"

#include "oxr_api_verify.h"
#include "oxr_chain.h"
#include "oxr_conversions.h"
#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_two_call.h"
#include "oxr_roles.h"


/*
 *
 * General helpers
 *
 */

DEBUG_GET_ONCE_NUM_OPTION(scale_percentage, "OXR_VIEWPORT_SCALE_PERCENTAGE", 100)



static bool
allowed_to_expose_quad_views(const struct oxr_system *sys)
{
	const struct oxr_instance *inst = sys->inst;

	// App has been quirked to not be given quad views.
	if (inst->quirks.disable_quad_views) {
		return false;
	}

	// Valid in OpenXR 1.1 and forward.
	if (inst->openxr_version.major_minor >= XR_MAKE_VERSION(1, 1, 0)) {
		return true;
	}

#if defined(OXR_HAVE_VARJO_quad_views)
	// The extension that introduced it.
	if (inst->extensions.VARJO_quad_views) {
		return true;
	}
#endif

	// The app hasn't enabled an extensions that introduces the enum.
	return false;
}

static struct oxr_view_config_properties *
get_view_config_properties(struct oxr_system *sys, XrViewConfigurationType view_config_type)
{
	for (uint32_t i = 0; i < sys->view_config_count; i++) {
		if (sys->view_configs[i].view_config_type == view_config_type) {
			return &sys->view_configs[i];
		}
	}

	return NULL;
}

static void
fill_in_view_config_properties_blend_modes(struct oxr_view_config_properties *props,
                                           const struct xrt_system_compositor_info *info)
{
	// Headless path.
	if (info == NULL) {
		props->blend_modes[0] = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
		props->blend_mode_count = 1;
		return;
	}

	assert(info->supported_blend_mode_count <= ARRAY_SIZE(props->blend_modes));
	assert(info->supported_blend_mode_count != 0);

	for (uint8_t i = 0; i < info->supported_blend_mode_count; i++) {
		assert(u_verify_blend_mode_valid(info->supported_blend_modes[i]));
		props->blend_modes[i] = (XrEnvironmentBlendMode)info->supported_blend_modes[i];
	}
	props->blend_mode_count = (uint32_t)info->supported_blend_mode_count;
}

static void
fill_in_view_config_properties_view_config_type(struct oxr_view_config_properties *props, enum xrt_view_type view_type)
{
	props->view_config_type = xrt_view_type_to_xr(view_type);
	U_LOG_D("props->view_config_type = %d", props->view_config_type);
}

static void
fill_in_view_config_properties_views(struct oxr_logger *log,
                                     XrViewConfigurationView *xr_views,
                                     const struct xrt_view_config *view_config)
{
	assert(view_config->view_count <= XRT_MAX_COMPOSITOR_VIEW_CONFIGS_VIEW_COUNT);

	double scale = debug_get_num_option_scale_percentage() / 100.0;
	if (scale > 2.0) {
		scale = 2.0;
		oxr_log(log, "Clamped scale to 200%%\n");
	}

#define imin(a, b) (a < b ? a : b)
	for (uint32_t i = 0; i < view_config->view_count; ++i) {
		uint32_t w = (uint32_t)(view_config->views[i].recommended.width_pixels * scale);
		uint32_t h = (uint32_t)(view_config->views[i].recommended.height_pixels * scale);
		uint32_t w_2 = view_config->views[i].max.width_pixels;
		uint32_t h_2 = view_config->views[i].max.height_pixels;

		w = imin(w, w_2);
		h = imin(h, h_2);

		xr_views[i].type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
		xr_views[i].recommendedImageRectWidth = w;
		xr_views[i].maxImageRectWidth = w_2;
		xr_views[i].recommendedImageRectHeight = h;
		xr_views[i].maxImageRectHeight = h_2;
		xr_views[i].recommendedSwapchainSampleCount = view_config->views[i].recommended.sample_count;
		xr_views[i].maxSwapchainSampleCount = view_config->views[i].max.sample_count;
	}
#undef imin
}

static void
fill_in_view_config_properties(struct oxr_logger *log,
                               struct oxr_view_config_properties *props,
                               const struct xrt_system_compositor_info *info,
                               const struct xrt_view_config *view_config)
{
	fill_in_view_config_properties_blend_modes(props, info);
	fill_in_view_config_properties_view_config_type(props, view_config->view_type);
	fill_in_view_config_properties_views(log, props->views, view_config);
	props->view_count = view_config->view_count;
}

static void
fill_in_view_emulated_quadview_config_properties(struct oxr_logger *log,
                                                 struct oxr_view_config_properties *props,
                                                 const struct xrt_system_compositor_info *info,
                                                 const struct xrt_view_config *stereo_view_config)
{
	static_assert(ARRAY_SIZE(props->views) >= 4, "Not enough space for quad views");
	assert(stereo_view_config->view_count == 2);

	fill_in_view_config_properties_blend_modes(props, info);
	fill_in_view_config_properties_view_config_type(props, XRT_VIEW_TYPE_QUAD);

	// First two views are context views copied from stereo config.
	fill_in_view_config_properties_views(log, props->views, stereo_view_config);

	// Last two views are inset views copied from stereo config.
	fill_in_view_config_properties_views(log, props->views + 2, stereo_view_config);

	props->view_count = 4;
}

static bool
oxr_system_matches(struct oxr_logger *log, struct oxr_system *sys, XrFormFactor form_factor)
{
	return xr_form_factor_to_xrt(form_factor) == sys->xsys->properties.form_factor;
}

static bool
oxr_system_get_body_tracking_support(struct oxr_logger *log,
                                     struct oxr_instance *inst,
                                     const enum xrt_input_name body_tracking_name)
{
	struct oxr_system *sys = &inst->system;
	const struct xrt_device *body = GET_STATIC_XDEV_BY_ROLE(sys, body);
	if (body == NULL || !body->supported.body_tracking || body->inputs == NULL) {
		return false;
	}

	for (size_t input_idx = 0; input_idx < body->input_count; ++input_idx) {
		const struct xrt_input *input = &body->inputs[input_idx];
		if (input->name == body_tracking_name) {
			return true;
		}
	}
	return false;
}


/*
 *
 * Two-call helpers.
 *
 */

static void
view_configuration_type_fill_in(XrViewConfigurationType *target, struct oxr_view_config_properties *source)
{
	*target = source->view_config_type;
}

static void
view_configuration_view_fill_in(XrViewConfigurationView *target_view, XrViewConfigurationView *source_view)
{
	// clang-format off
	target_view->recommendedImageRectWidth       = source_view->recommendedImageRectWidth;
	target_view->maxImageRectWidth               = source_view->maxImageRectWidth;
	target_view->recommendedImageRectHeight      = source_view->recommendedImageRectHeight;
	target_view->maxImageRectHeight              = source_view->maxImageRectHeight;
	target_view->recommendedSwapchainSampleCount = source_view->recommendedSwapchainSampleCount;
	target_view->maxSwapchainSampleCount         = source_view->maxSwapchainSampleCount;
	// clang-format on
}


/*
 *
 * 'Exported' functions.
 *
 */

XrResult
oxr_system_select(struct oxr_logger *log,
                  struct oxr_system **systems,
                  uint32_t system_count,
                  XrFormFactor form_factor,
                  struct oxr_system **out_selected)
{
	if (system_count == 0) {
		return oxr_error(log, XR_ERROR_FORM_FACTOR_UNSUPPORTED,
		                 "(getInfo->formFactor) no system available (given: %i)", form_factor);
	}

	struct oxr_system *selected = NULL;
	for (uint32_t i = 0; i < system_count; i++) {
		if (oxr_system_matches(log, systems[i], form_factor)) {
			selected = systems[i];
			break;
		}
	}

	if (selected == NULL) {
		return oxr_error(log, XR_ERROR_FORM_FACTOR_UNSUPPORTED,
		                 "(getInfo->formFactor) no matching system "
		                 "(given: %i, first: %i)",
		                 form_factor, xrt_form_factor_to_xr(systems[0]->xsys->properties.form_factor));
	}

	struct xrt_device *xdev = GET_STATIC_XDEV_BY_ROLE(selected, head);
	if (xdev->supported.form_factor_check) {
		bool available = false;
		xrt_result_t xret =
		    xrt_device_is_form_factor_available(xdev, xr_form_factor_to_xrt(form_factor), &available);
		if (xret != XRT_SUCCESS) {
			return oxr_error(log, XR_ERROR_RUNTIME_FAILURE,
			                 "Call to xrt_device_is_form_factor_available failed: %s",
			                 u_str_xrt_result(xret));
		}
		if (!available) {
			return oxr_error(log, XR_ERROR_FORM_FACTOR_UNAVAILABLE,
			                 "request form factor %i is unavailable now", form_factor);
		}
	}

	*out_selected = selected;

	return XR_SUCCESS;
}

XrResult
oxr_system_verify_id(struct oxr_logger *log, const struct oxr_instance *inst, XrSystemId systemId)
{
	if (systemId != XRT_SYSTEM_ID) {
		return oxr_error(log, XR_ERROR_SYSTEM_INVALID, "Invalid system %" PRIu64, systemId);
	}
	return XR_SUCCESS;
}

XrResult
oxr_system_get_by_id(struct oxr_logger *log, struct oxr_instance *inst, XrSystemId systemId, struct oxr_system **system)
{
	XrResult result = oxr_system_verify_id(log, inst, systemId);
	if (result != XR_SUCCESS) {
		return result;
	}

	/* right now only have one system. */
	*system = &inst->system;

	return XR_SUCCESS;
}

XrResult
oxr_system_fill_in(
    struct oxr_logger *log, struct oxr_instance *inst, XrSystemId systemId, uint32_t view_count, struct oxr_system *sys)
{
	xrt_result_t xret = XRT_SUCCESS;

	//! @todo handle other subaction paths?

	sys->inst = inst;
	sys->systemId = systemId;

#ifdef XR_USE_GRAPHICS_API_VULKAN
	sys->vulkan_enable2_instance = VK_NULL_HANDLE;
	sys->suggested_vulkan_physical_device = VK_NULL_HANDLE;
	sys->vk_get_instance_proc_addr = VK_NULL_HANDLE;
#endif
#if defined(XR_USE_GRAPHICS_API_D3D11) || defined(XR_USE_GRAPHICS_API_D3D12)
	U_ZERO(&(sys->suggested_d3d_luid));
	sys->suggested_d3d_luid_valid = false;
#endif

	if (sys->xsysc != NULL) {
		const struct xrt_system_compositor_info *info = &sys->xsysc->info;

		if (info->view_type_count == 0) {
			return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Unexpected condition having no view configs");
		}

		// Cache these.
		struct xrt_view_config view_configs[XRT_MAX_COMPOSITOR_VIEW_CONFIGS_COUNT];
		for (uint32_t i = 0; i < info->view_type_count; i++) {
			const enum xrt_view_type view_type = info->view_types[i];

			xret = xrt_syscomp_get_view_config(sys->xsysc, view_type, &view_configs[i]);
			if (xret != XRT_SUCCESS) {
				return oxr_error(log, XR_ERROR_RUNTIME_FAILURE,
				                 "Getting view configuration failed for an expected reason.");
			}
		}

		bool has_quad_config = false;
		const struct xrt_view_config *stereo_view_config = NULL;

		for (uint32_t i = 0; i < info->view_type_count; i++) {
			const struct xrt_view_config *view_config = &view_configs[i];
			if (view_config->view_type == XRT_VIEW_TYPE_QUAD) {
				if (!allowed_to_expose_quad_views(sys)) {
					oxr_log(log, "System has quad views but it is not allowed to expose them.");
					continue;
				}
				has_quad_config = true;
			}
			if (view_config->view_type == XRT_VIEW_TYPE_STEREO) {
				stereo_view_config = view_config;
			}

			assert(sys->view_config_count < XRT_MAX_COMPOSITOR_VIEW_CONFIGS_COUNT);
			fill_in_view_config_properties(                 //
			    log,                                        //
			    &sys->view_configs[sys->view_config_count], //
			    info,                                       //
			    view_config);                               //
			sys->view_config_count++;
		}

		// If system does not natively expose quad views then emulate quad view config from stereo config.
		if (!has_quad_config &&                                         //
		    stereo_view_config != NULL &&                               //
		    allowed_to_expose_quad_views(sys) &&                        //
		    sys->xsysc->info.supports_emulated_quad_views_with_inset) { //

			assert(sys->view_config_count < XRT_MAX_COMPOSITOR_VIEW_CONFIGS_COUNT);
			fill_in_view_emulated_quadview_config_properties( //
			    log,                                          //
			    &sys->view_configs[sys->view_config_count],   //
			    info,                                         //
			    stereo_view_config);                          //
			sys->view_config_count++;
		}
	} else {
		// Headless path, view configs contain no views but still need blend modes and view types.
		assert(view_count == 1 || view_count == 2);

		enum xrt_view_type view_type = view_count == 1 ? XRT_VIEW_TYPE_MONO : XRT_VIEW_TYPE_STEREO;

		// First headless config: regular mono or stereo
		fill_in_view_config_properties_blend_modes(&sys->view_configs[0], NULL);
		fill_in_view_config_properties_view_config_type(&sys->view_configs[0], view_type);
		sys->view_config_count++;

		// Second headless config: simulated foveated stereo when allowed
		if (view_count == 2 && allowed_to_expose_quad_views(sys)) {
			fill_in_view_config_properties_blend_modes(&sys->view_configs[1], NULL);
			fill_in_view_config_properties_view_config_type(&sys->view_configs[1], XRT_VIEW_TYPE_QUAD);
			sys->view_config_count++;
		}
	}


	/*
	 * Reference space support.
	 */

	static_assert(5 <= ARRAY_SIZE(sys->reference_spaces), "Not enough space in array");

	if (sys->xso->semantic.view != NULL) {
		sys->reference_spaces[sys->reference_space_count++] = XR_REFERENCE_SPACE_TYPE_VIEW;
	}

	if (sys->xso->semantic.local != NULL) {
		sys->reference_spaces[sys->reference_space_count++] = XR_REFERENCE_SPACE_TYPE_LOCAL;
	}

	if (OXR_API_VERSION_AT_LEAST(sys->inst, 1, 1)) {
		if (sys->xso->semantic.local_floor != NULL) {
			sys->reference_spaces[sys->reference_space_count++] = XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR;
		} else {
			oxr_warn(log,
			         "OpenXR 1.1 used but system doesn't support local_floor,"
			         " breaking spec by not exposing the reference space.");
		}
	}

#ifdef OXR_HAVE_EXT_local_floor
	// If OpenXR 1.1 and the extension is enabled, don't add a second reference.
	// Note that XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR is aliased to XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR_EXT.
	if (!OXR_API_VERSION_AT_LEAST(sys->inst, 1, 1)) {
		if (sys->inst->extensions.EXT_local_floor) {
			if (sys->xso->semantic.local_floor != NULL) {
				sys->reference_spaces[sys->reference_space_count++] =
				    XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR_EXT;
			} else {
				oxr_warn(log,
				         "XR_EXT_local_floor enabled but system doesn't support local_floor,"
				         " breaking spec by not exposing the reference space.");
			}
		}
	}
#endif

	if (sys->xso->semantic.stage != NULL) {
		sys->reference_spaces[sys->reference_space_count++] = XR_REFERENCE_SPACE_TYPE_STAGE;
	}

#ifdef OXR_HAVE_MSFT_unbounded_reference_space
	if (sys->inst->extensions.MSFT_unbounded_reference_space && sys->xso->semantic.unbounded != NULL) {
		sys->reference_spaces[sys->reference_space_count++] = XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT;
	}
#endif


	/*
	 * Misc
	 */

#ifdef OXR_HAVE_MNDX_xdev_space
	// By default xdev_space is implemented in the state tracker and does not need explicit system support.
	sys->supports_xdev_space = true;
#endif

	/*
	 * Done.
	 */

	return XR_SUCCESS;
}

bool
oxr_system_get_hand_tracking_support(struct oxr_logger *log, struct oxr_instance *inst)
{
	struct oxr_system *sys = &inst->system;
#define OXR_CHECK_RET_IS_HT_SUPPORTED(HT_ROLE)                                                                         \
	{                                                                                                              \
		const struct xrt_device *ht = GET_STATIC_XDEV_BY_ROLE(sys, hand_tracking_##HT_ROLE);                   \
		if (ht && ht->supported.hand_tracking) {                                                               \
			return true;                                                                                   \
		}                                                                                                      \
	}
	OXR_CHECK_RET_IS_HT_SUPPORTED(unobstructed_left)
	OXR_CHECK_RET_IS_HT_SUPPORTED(unobstructed_right)
	OXR_CHECK_RET_IS_HT_SUPPORTED(conforming_left)
	OXR_CHECK_RET_IS_HT_SUPPORTED(conforming_right)
#undef OXR_CHECK_RET_IS_HT_SUPPORTED
	return false;
}

bool
oxr_system_get_eye_gaze_support(struct oxr_logger *log, struct oxr_instance *inst)
{
	struct oxr_system *sys = &inst->system;
	struct xrt_device *eyes = GET_STATIC_XDEV_BY_ROLE(sys, eyes);

	return eyes && eyes->supported.eye_gaze;
}

bool
oxr_system_get_force_feedback_support(struct oxr_logger *log, struct oxr_instance *inst)
{
	struct oxr_system *sys = &inst->system;
#define OXR_CHECK_RET_IS_FFB_SUPPORTED(HT_ROLE)                                                                        \
	{                                                                                                              \
		const struct xrt_device *ffb = GET_STATIC_XDEV_BY_ROLE(sys, hand_tracking_##HT_ROLE);                  \
		if (ffb && ffb->supported.force_feedback) {                                                            \
			return true;                                                                                   \
		}                                                                                                      \
	}
	OXR_CHECK_RET_IS_FFB_SUPPORTED(unobstructed_left)
	OXR_CHECK_RET_IS_FFB_SUPPORTED(unobstructed_right)
	OXR_CHECK_RET_IS_FFB_SUPPORTED(conforming_left)
	OXR_CHECK_RET_IS_FFB_SUPPORTED(conforming_right)
#undef OXR_CHECK_RET_IS_FFB_SUPPORTED
	return false;
}

void
oxr_system_get_face_tracking_android_support(struct oxr_logger *log, struct oxr_instance *inst, bool *supported)
{
	assert(supported);

	*supported = false;
	struct oxr_system *sys = &inst->system;
	const struct xrt_device *face_xdev = GET_STATIC_XDEV_BY_ROLE(sys, face);

	if (face_xdev == NULL || !face_xdev->supported.face_tracking || face_xdev->inputs == NULL) {
		return;
	}

	for (size_t input_idx = 0; input_idx < face_xdev->input_count; ++input_idx) {
		const struct xrt_input *input = &face_xdev->inputs[input_idx];
		if (input->name == XRT_INPUT_ANDROID_FACE_TRACKING) {
			*supported = true;
			return;
		}
	}
}

void
oxr_system_get_face_tracking_htc_support(struct oxr_logger *log,
                                         struct oxr_instance *inst,
                                         bool *supports_eye,
                                         bool *supports_lip)
{
	struct oxr_system *sys = &inst->system;
	struct xrt_device *face_xdev = GET_STATIC_XDEV_BY_ROLE(sys, face);

	if (supports_eye)
		*supports_eye = false;
	if (supports_lip)
		*supports_lip = false;

	if (face_xdev == NULL || !face_xdev->supported.face_tracking || face_xdev->inputs == NULL) {
		return;
	}

	for (size_t input_idx = 0; input_idx < face_xdev->input_count; ++input_idx) {
		const struct xrt_input *input = &face_xdev->inputs[input_idx];
		if (supports_eye != NULL && input->name == XRT_INPUT_HTC_EYE_FACE_TRACKING) {
			*supports_eye = true;
		}
		if (supports_lip != NULL && input->name == XRT_INPUT_HTC_LIP_FACE_TRACKING) {
			*supports_lip = true;
		}
	}
}

void
oxr_system_get_face_tracking2_fb_support(struct oxr_logger *log,
                                         struct oxr_instance *inst,
                                         bool *supports_audio,
                                         bool *supports_visual)
{
	if (supports_audio != NULL)
		*supports_audio = false;

	if (supports_visual != NULL)
		*supports_visual = false;

	struct oxr_system *sys = &inst->system;
	struct xrt_device *face_xdev = GET_STATIC_XDEV_BY_ROLE(sys, face);

	if (face_xdev == NULL || !face_xdev->supported.face_tracking || face_xdev->inputs == NULL) {
		return;
	}

	for (size_t input_idx = 0; input_idx < face_xdev->input_count; ++input_idx) {
		const struct xrt_input *input = &face_xdev->inputs[input_idx];
		if (input->name == XRT_INPUT_FB_FACE_TRACKING2_AUDIO && supports_audio != NULL) {
			*supports_audio = true;
		} else if (input->name == XRT_INPUT_FB_FACE_TRACKING2_VISUAL && supports_visual != NULL) {
			*supports_visual = true;
		}
	}
	return;
}

bool
oxr_system_get_body_tracking_fb_support(struct oxr_logger *log, struct oxr_instance *inst)
{
	return oxr_system_get_body_tracking_support(log, inst, XRT_INPUT_FB_BODY_TRACKING);
}

bool
oxr_system_get_full_body_tracking_meta_support(struct oxr_logger *log, struct oxr_instance *inst)
{
	return oxr_system_get_body_tracking_support(log, inst, XRT_INPUT_META_FULL_BODY_TRACKING);
}

bool
oxr_system_get_body_tracking_calibration_meta_support(struct oxr_logger *log, struct oxr_instance *inst)
{
	if (!oxr_system_get_body_tracking_fb_support(log, inst) &&
	    !oxr_system_get_full_body_tracking_meta_support(log, inst)) {
		return false;
	}
	struct oxr_system *sys = &inst->system;
	const struct xrt_device *body = GET_STATIC_XDEV_BY_ROLE(sys, body);
	return body->supported.body_tracking_calibration;
}

bool
oxr_system_get_body_tracking_fidelity_meta_support(struct oxr_logger *log, struct oxr_instance *inst)
{
	if (!oxr_system_get_body_tracking_fb_support(log, inst) &&
	    !oxr_system_get_full_body_tracking_meta_support(log, inst)) {
		return false;
	}
	struct oxr_system *sys = &inst->system;
	const struct xrt_device *body = GET_STATIC_XDEV_BY_ROLE(sys, body);
	return body->supported.body_tracking_fidelity;
}

XrResult
oxr_system_get_properties(struct oxr_logger *log, struct oxr_system *sys, XrSystemProperties *properties)
{
	properties->systemId = sys->systemId;
	properties->vendorId = sys->xsys->properties.vendor_id;
	memcpy(properties->systemName, sys->xsys->properties.name, sizeof(properties->systemName));

	struct xrt_device *xdev = GET_STATIC_XDEV_BY_ROLE(sys, head);

	// Get from compositor.
	struct xrt_system_compositor_info *info = sys->xsysc ? &sys->xsysc->info : NULL;

	if (info) {
		properties->graphicsProperties.maxLayerCount = info->max_layers;
	} else {
		// probably using the headless extension, but the extension does not modify the 16 layer minimum.
		properties->graphicsProperties.maxLayerCount = XRT_MAX_LAYERS;
	}
	properties->graphicsProperties.maxSwapchainImageWidth = 1024 * 16;
	properties->graphicsProperties.maxSwapchainImageHeight = 1024 * 16;
	properties->trackingProperties.orientationTracking = xdev->supported.orientation_tracking;
	properties->trackingProperties.positionTracking = xdev->supported.position_tracking;

#ifdef OXR_HAVE_EXT_hand_tracking
	XrSystemHandTrackingPropertiesEXT *hand_tracking_props = NULL;
	// We should only be looking for extension structs if the extension has been enabled.
	if (sys->inst->extensions.EXT_hand_tracking) {
		hand_tracking_props = OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT,
		                                                XrSystemHandTrackingPropertiesEXT);
	}

	if (hand_tracking_props) {
		hand_tracking_props->supportsHandTracking = oxr_system_get_hand_tracking_support(log, sys->inst);
	}
#endif

#ifdef OXR_HAVE_EXT_eye_gaze_interaction
	XrSystemEyeGazeInteractionPropertiesEXT *eye_gaze_props = NULL;
	if (sys->inst->extensions.EXT_eye_gaze_interaction) {
		eye_gaze_props =
		    OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_EYE_GAZE_INTERACTION_PROPERTIES_EXT,
		                              XrSystemEyeGazeInteractionPropertiesEXT);
	}

	if (eye_gaze_props) {
		eye_gaze_props->supportsEyeGazeInteraction = oxr_system_get_eye_gaze_support(log, sys->inst);
	}
#endif

#ifdef OXR_HAVE_MNDX_force_feedback_curl
	XrSystemForceFeedbackCurlPropertiesMNDX *force_feedback_props = NULL;
	if (sys->inst->extensions.MNDX_force_feedback_curl) {
		force_feedback_props =
		    OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_FORCE_FEEDBACK_CURL_PROPERTIES_MNDX,
		                              XrSystemForceFeedbackCurlPropertiesMNDX);
	}

	if (force_feedback_props) {
		force_feedback_props->supportsForceFeedbackCurl = oxr_system_get_force_feedback_support(log, sys->inst);
	}
#endif

#ifdef OXR_HAVE_FB_passthrough
	XrSystemPassthroughPropertiesFB *passthrough_props = NULL;
	XrSystemPassthroughProperties2FB *passthrough_props2 = NULL;
	if (sys->inst->extensions.FB_passthrough) {
		passthrough_props = OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_PASSTHROUGH_PROPERTIES_FB,
		                                              XrSystemPassthroughPropertiesFB);
		if (passthrough_props) {
			passthrough_props->supportsPassthrough = true;
		}

		passthrough_props2 = OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_PASSTHROUGH_PROPERTIES2_FB,
		                                               XrSystemPassthroughProperties2FB);
		if (passthrough_props2) {
			passthrough_props2->capabilities = XR_PASSTHROUGH_CAPABILITY_BIT_FB;
		}
	}
#endif

#ifdef OXR_HAVE_ANDROID_face_tracking
	XrSystemFaceTrackingPropertiesANDROID *android_face_tracking_props = NULL;
	if (sys->inst->extensions.ANDROID_face_tracking) {
		android_face_tracking_props = OXR_GET_OUTPUT_FROM_CHAIN(
		    properties, XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES_ANDROID, XrSystemFaceTrackingPropertiesANDROID);
	}

	if (android_face_tracking_props) {
		bool supported = false;
		oxr_system_get_face_tracking_android_support(log, sys->inst, &supported);
		android_face_tracking_props->supportsFaceTracking = supported;
	}
#endif // OXR_HAVE_HTC_facial_tracking

#ifdef OXR_HAVE_HTC_facial_tracking
	XrSystemFacialTrackingPropertiesHTC *htc_facial_tracking_props = NULL;
	if (sys->inst->extensions.HTC_facial_tracking) {
		htc_facial_tracking_props = OXR_GET_OUTPUT_FROM_CHAIN(
		    properties, XR_TYPE_SYSTEM_FACIAL_TRACKING_PROPERTIES_HTC, XrSystemFacialTrackingPropertiesHTC);
	}

	if (htc_facial_tracking_props) {
		bool supports_eye = false;
		bool supports_lip = false;
		oxr_system_get_face_tracking_htc_support(log, sys->inst, &supports_eye, &supports_lip);
		htc_facial_tracking_props->supportEyeFacialTracking = supports_eye;
		htc_facial_tracking_props->supportLipFacialTracking = supports_lip;
	}
#endif // OXR_HAVE_HTC_facial_tracking

#ifdef OXR_HAVE_FB_body_tracking
	XrSystemBodyTrackingPropertiesFB *body_tracking_fb_props = NULL;
	if (sys->inst->extensions.FB_body_tracking) {
		body_tracking_fb_props = OXR_GET_OUTPUT_FROM_CHAIN(
		    properties, XR_TYPE_SYSTEM_BODY_TRACKING_PROPERTIES_FB, XrSystemBodyTrackingPropertiesFB);
	}

	if (body_tracking_fb_props) {
		body_tracking_fb_props->supportsBodyTracking = oxr_system_get_body_tracking_fb_support(log, sys->inst);
	}
#endif // OXR_HAVE_FB_body_tracking

#ifdef OXR_HAVE_BD_body_tracking
	XrSystemBodyTrackingPropertiesBD *body_tracking_bd_props = NULL;
	if (sys->inst->extensions.BD_body_tracking) {
		body_tracking_bd_props = OXR_GET_OUTPUT_FROM_CHAIN(
		    properties, XR_TYPE_SYSTEM_BODY_TRACKING_PROPERTIES_BD, XrSystemBodyTrackingPropertiesBD);
	}

	if (body_tracking_bd_props) {
		body_tracking_bd_props->supportsBodyTracking = oxr_system_get_body_tracking_bd_support(log, sys->inst);
	}
#endif // OXR_HAVE_BD_body_tracking

#ifdef OXR_HAVE_FB_face_tracking2
	XrSystemFaceTrackingProperties2FB *face_tracking2_fb_props = NULL;
	if (sys->inst->extensions.FB_face_tracking2) {
		face_tracking2_fb_props = OXR_GET_OUTPUT_FROM_CHAIN(
		    properties, XR_TYPE_SYSTEM_FACE_TRACKING_PROPERTIES2_FB, XrSystemFaceTrackingProperties2FB);
	}

	if (face_tracking2_fb_props) {
		bool supports_audio, supports_visual;
		oxr_system_get_face_tracking2_fb_support(log, sys->inst, &supports_audio, &supports_visual);
		face_tracking2_fb_props->supportsAudioFaceTracking = supports_audio;
		face_tracking2_fb_props->supportsVisualFaceTracking = supports_visual;
	}
#endif // OXR_HAVE_FB_face_tracking2


#ifdef OXR_HAVE_MNDX_xdev_space
	XrSystemXDevSpacePropertiesMNDX *xdev_space_props = NULL;
	if (sys->inst->extensions.MNDX_xdev_space) {
		xdev_space_props = OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_XDEV_SPACE_PROPERTIES_MNDX,
		                                             XrSystemXDevSpacePropertiesMNDX);
	}

	if (xdev_space_props) {
		xdev_space_props->supportsXDevSpace = sys->supports_xdev_space;
	}
#endif // OXR_HAVE_MNDX_xdev_space

#ifdef OXR_HAVE_EXT_plane_detection
	XrSystemPlaneDetectionPropertiesEXT *plane_detection_props = NULL;
	if (sys->inst->extensions.EXT_plane_detection) {
		plane_detection_props = OXR_GET_OUTPUT_FROM_CHAIN(
		    properties, XR_TYPE_SYSTEM_PLANE_DETECTION_PROPERTIES_EXT, XrSystemPlaneDetectionPropertiesEXT);
	}

	if (plane_detection_props) {
		// for now these are mapped 1:1
		plane_detection_props->supportedFeatures =
		    (XrPlaneDetectionCapabilityFlagsEXT)xdev->supported.plane_capability_flags;
	}
#endif // OXR_HAVE_EXT_plane_detection

#ifdef OXR_HAVE_EXT_user_presence
	XrSystemUserPresencePropertiesEXT *user_presence_props = NULL;
	if (sys->inst->extensions.EXT_user_presence) {
		user_presence_props = OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_USER_PRESENCE_PROPERTIES_EXT,
		                                                XrSystemUserPresencePropertiesEXT);
	}

	if (user_presence_props) {
		user_presence_props->supportsUserPresence = xdev->supported.presence;
	}
#endif // OXR_HAVE_EXT_user_presence

#ifdef OXR_HAVE_META_body_tracking_full_body
	XrSystemPropertiesBodyTrackingFullBodyMETA *full_body_tracking_meta_props = NULL;
	if (sys->inst->extensions.META_body_tracking_full_body) {
		full_body_tracking_meta_props =
		    OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_PROPERTIES_BODY_TRACKING_FULL_BODY_META,
		                              XrSystemPropertiesBodyTrackingFullBodyMETA);
	}

	if (full_body_tracking_meta_props) {
		full_body_tracking_meta_props->supportsFullBodyTracking =
		    oxr_system_get_full_body_tracking_meta_support(log, sys->inst);
	}
#endif // OXR_HAVE_META_body_tracking_full_body

#ifdef OXR_HAVE_META_body_tracking_calibration
	XrSystemPropertiesBodyTrackingCalibrationMETA *body_tracking_calibration_meta_props = NULL;
	if (sys->inst->extensions.META_body_tracking_calibration) {
		body_tracking_calibration_meta_props =
		    OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_PROPERTIES_BODY_TRACKING_CALIBRATION_META,
		                              XrSystemPropertiesBodyTrackingCalibrationMETA);
	}

	if (body_tracking_calibration_meta_props) {
		body_tracking_calibration_meta_props->supportsHeightOverride =
		    oxr_system_get_body_tracking_calibration_meta_support(log, sys->inst);
	}
#endif // OXR_HAVE_META_body_tracking_calibration

#ifdef OXR_HAVE_META_body_tracking_fidelity
	XrSystemPropertiesBodyTrackingFidelityMETA *body_tracking_fidelity_meta_props = NULL;
	if (sys->inst->extensions.META_body_tracking_fidelity) {
		body_tracking_fidelity_meta_props =
		    OXR_GET_OUTPUT_FROM_CHAIN(properties, XR_TYPE_SYSTEM_PROPERTIES_BODY_TRACKING_FIDELITY_META,
		                              XrSystemPropertiesBodyTrackingFidelityMETA);
	}

	if (body_tracking_fidelity_meta_props) {
		body_tracking_fidelity_meta_props->supportsBodyTrackingFidelity =
		    oxr_system_get_body_tracking_fidelity_meta_support(log, sys->inst);
	}
#endif // OXR_HAVE_META_body_tracking_fidelity

	return XR_SUCCESS;
}

XrResult
oxr_system_enumerate_view_confs(struct oxr_logger *log,
                                struct oxr_system *sys,
                                uint32_t viewConfigurationTypeCapacityInput,
                                uint32_t *viewConfigurationTypeCountOutput,
                                XrViewConfigurationType *viewConfigurationTypes)
{
	OXR_TWO_CALL_FILL_IN_HELPER(log, viewConfigurationTypeCapacityInput, viewConfigurationTypeCountOutput,
	                            viewConfigurationTypes, sys->view_config_count, view_configuration_type_fill_in,
	                            sys->view_configs, XR_SUCCESS);
}

XrResult
oxr_system_enumerate_blend_modes(struct oxr_logger *log,
                                 struct oxr_system *sys,
                                 XrViewConfigurationType viewConfigurationType,
                                 uint32_t environmentBlendModeCapacityInput,
                                 uint32_t *environmentBlendModeCountOutput,
                                 XrEnvironmentBlendMode *environmentBlendModes)
{
	struct oxr_view_config_properties *props = get_view_config_properties(sys, viewConfigurationType);
	if (props == NULL) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Didn't find view configs");
	}

	OXR_TWO_CALL_HELPER(log, environmentBlendModeCapacityInput, environmentBlendModeCountOutput,
	                    environmentBlendModes, props->blend_mode_count, props->blend_modes, XR_SUCCESS);
}

XrResult
oxr_system_get_view_conf_properties(struct oxr_logger *log,
                                    struct oxr_system *sys,
                                    XrViewConfigurationType viewConfigurationType,
                                    XrViewConfigurationProperties *configurationProperties)
{
	struct oxr_view_config_properties *props = get_view_config_properties(sys, viewConfigurationType);
	if (props == NULL) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Didn't find view configs");
	}

	configurationProperties->viewConfigurationType = props->view_config_type;
	configurationProperties->fovMutable = sys->xsysc->info.supports_fov_mutable;

	return XR_SUCCESS;
}

XrResult
oxr_system_enumerate_view_conf_views(struct oxr_logger *log,
                                     struct oxr_system *sys,
                                     XrViewConfigurationType viewConfigurationType,
                                     uint32_t viewCapacityInput,
                                     uint32_t *viewCountOutput,
                                     XrViewConfigurationView *views)
{
	struct oxr_view_config_properties *props = get_view_config_properties(sys, viewConfigurationType);
	if (props == NULL) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Didn't find view configs");
	}

	OXR_TWO_CALL_FILL_IN_HELPER(log, viewCapacityInput, viewCountOutput, views, props->view_count,
	                            view_configuration_view_fill_in, props->views, XR_SUCCESS);
}
