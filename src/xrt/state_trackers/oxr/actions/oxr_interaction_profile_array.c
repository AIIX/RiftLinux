// Copyright 2018-2020,2023 Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Holds binding related functions.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup oxr_main
 */

#include "util/u_misc.h"

#include "oxr_interaction_profile_array.h"
#include "oxr_generated_bindings.h"
#include "oxr_binding.h"

#include "../oxr_objects.h"

#include <assert.h>


void
oxr_interaction_profile_array_clear(struct oxr_interaction_profile_array *array)
{
	for (size_t i = 0; i < array->count; i++) {
		struct oxr_interaction_profile *p = array->profiles[i];
		oxr_interaction_profile_destroy(p);
	}

	free(array->profiles);
	array->profiles = NULL;
	array->count = 0;
}

void
oxr_interaction_profile_array_add(struct oxr_interaction_profile_array *array, struct oxr_interaction_profile *profile)
{
	U_ARRAY_REALLOC_OR_FREE(array->profiles, struct oxr_interaction_profile *, (array->count + 1));
	array->profiles[array->count++] = profile;
}

void
oxr_interaction_profile_array_clone(const struct oxr_interaction_profile_array *src,
                                    struct oxr_interaction_profile_array *dst)
{
	oxr_interaction_profile_array_clear(dst);

	if (src->profiles == NULL || src->count == 0) {
		return;
	}

	dst->count = src->count;
	dst->profiles = U_TYPED_ARRAY_CALLOC(struct oxr_interaction_profile *, src->count);

	for (size_t i = 0; i < src->count; i++) {
		dst->profiles[i] = oxr_interaction_profile_clone(src->profiles[i]);
	}
}

bool
oxr_interaction_profile_array_find_by_path(const struct oxr_interaction_profile_array *array,
                                           XrPath path,
                                           struct oxr_interaction_profile **out_p)
{
	for (size_t x = 0; x < array->count; x++) {
		struct oxr_interaction_profile *p = array->profiles[x];
		if (p->path != path) {
			continue;
		}

		*out_p = p;
		return true;
	}

	*out_p = NULL;

	return false;
}

bool
oxr_interaction_profile_array_find_by_device_name(const struct oxr_interaction_profile_array *array,
                                                  const struct oxr_instance_path_cache *cache,
                                                  enum xrt_device_name name,
                                                  struct oxr_interaction_profile **out_p)
{
	if (name == XRT_DEVICE_INVALID) {
		*out_p = NULL;
		return false;
	}

	/*
	 * Map xrt_device_name to an interaction profile XrPath.
	 *
	 * There might be multiple OpenXR interaction profiles that maps to a
	 * a single @ref xrt_device_name, so we can't just grab the first one
	 * that we find and assume that wasn't bound then there isn't an OpenXR
	 * interaction profile bound for that device name. So we will need to
	 * keep looping until we find an OpenXR interaction profile, or we run
	 * out of interaction profiles that the app has suggested.
	 *
	 * For XRT_DEVICE_HAND_INTERACTION both the OpenXR hand-interaction
	 * profiles maps to it, but the app might only provide binding for one.
	 *
	 * Set *out_p to an oxr_interaction_profile if bindings for that
	 * interaction profile XrPath have been suggested.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(oxr_profile_templates); i++) {
		// If it's for a different device name, skip.
		if (name != oxr_profile_templates[i].name) {
			continue;
		}

		bool found = oxr_interaction_profile_array_find_by_path( //
		    array,                                               //
		    cache->template_paths[i],                            //
		    out_p);                                              //

		/*
		 * Keep looping even if the current matching OpenXR
		 * interaction profile wasn't suggested by the app.
		 * See comment above.
		 */
		if (found) {
			return true;
		}
	}

	*out_p = NULL;
	return false;
}

bool
oxr_interaction_profile_array_find_by_device(const struct oxr_interaction_profile_array *array,
                                             const struct oxr_instance_path_cache *cache,
                                             struct xrt_device *xdev,
                                             struct oxr_interaction_profile **out_p)
{
	bool found = false;

	if (xdev == NULL) {
		*out_p = NULL;
		return false;
	}

	// Have bindings for this device's interaction profile been suggested?
	found = oxr_interaction_profile_array_find_by_device_name( //
	    array,                                                 //
	    cache,                                                 //
	    xdev->name,                                            //
	    out_p);                                                //
	if (found) {
		return true;
	}

	// Check if bindings for any of this device's alternative interaction profiles have been suggested.
	for (size_t i = 0; i < xdev->binding_profile_count; i++) {
		struct xrt_binding_profile *xbp = &xdev->binding_profiles[i];

		found = oxr_interaction_profile_array_find_by_device_name( //
		    array,                                                 //
		    cache,                                                 //
		    xbp->name,                                             //
		    out_p);                                                //
		if (found) {
			return true;
		}
	}

	*out_p = NULL;
	return false;
}
