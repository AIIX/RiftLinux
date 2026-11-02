// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Preview header for the XR_MNDX_composition_layer_backdrop_blur extension (RiftLinux).
 * Frosted glass: what the compositor shows behind a quad or cylinder layer (the layers under it, other applications'
 * included) is blurred where the layer covers it, then the layer is blended over it as usual. Only that layer's
 * backdrop is touched, only where its alpha is above zero; layers without the structure are composed as before.
 * Chained on an XrCompositionLayerQuad or XrCompositionLayerCylinderKHR given to xrEndFrame; ignored on other layers.
 * Provisional: the extension number (2002) is not registered with Khronos; it is far above the registry's numbers so
 * its structure type cannot collide with a registered one for years.
 * @ingroup external_openxr
 */
#ifndef XR_MNDX_COMPOSITION_LAYER_BACKDROP_BLUR_H
#define XR_MNDX_COMPOSITION_LAYER_BACKDROP_BLUR_H 1

#include "openxr_extension_helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

// Extension number 2002 (2000 prefix), provisional.
#define XR_MNDX_composition_layer_backdrop_blur 1
#define XR_MNDX_composition_layer_backdrop_blur_SPEC_VERSION 1
#define XR_MNDX_COMPOSITION_LAYER_BACKDROP_BLUR_EXTENSION_NAME "XR_MNDX_composition_layer_backdrop_blur"

XR_STRUCT_ENUM(XR_TYPE_COMPOSITION_LAYER_BACKDROP_BLUR_MNDX, 1002001001);
/*!
 * XrCompositionLayerBackdropBlurMNDX extends XrCompositionLayerQuad and XrCompositionLayerCylinderKHR.
 *
 * - blurRadians: the blur's size as an angle seen from the eye, in radians: the Gaussian's standard deviation (as a
 *   CSS blur's length). 0 is no blur (the structure is then ignored); the compositor caps it at 0.06 (3.4 degrees).
 * - saturation: of the blurred backdrop: 1 keeps its colours, 0 makes it grey, above 1 more vivid (up to 2).
 * - tint: mixed over the blurred backdrop by tint.a (0 none, 1 the tint alone); rgb in linear light, 1.0 being white.
 * - maskAlpha: the layer's alpha from which the backdrop is blurred fully; below it the blur fades out to none at
 *   alpha 0, so the layer's transparent parts (rounded corners) keep a sharp backdrop. Greater than 0, at most 1.
 *
 * The compositor blurs at most 4 such layers a view (the lowest ones); more are shown without the blur.
 */
typedef struct XrCompositionLayerBackdropBlurMNDX
{
	XrStructureType type;
	const void *XR_MAY_ALIAS next;
	float blurRadians;
	float saturation;
	XrColor4f tint;
	float maskAlpha;
} XrCompositionLayerBackdropBlurMNDX;

#ifdef __cplusplus
}
#endif

#endif
