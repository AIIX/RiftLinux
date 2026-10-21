// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Holds interaction profile array related functions.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup oxr_main
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_device.h"

#include "oxr_extension_support.h"
#include "oxr_forward_declarations.h"


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Manages an array of interaction profiles, does not have a init function
 * but must be zero initialized where it is declared.
 */
struct oxr_interaction_profile_array
{
	struct oxr_interaction_profile **profiles;
	size_t count;
};

/*!
 * Frees all profiles on the interaction profile array.
 *
 * Frees all interaction profiles in the array and then frees the internal array
 * does not free the struct passed in. After this call, the array structure is
 * reset to an empty state.
 *
 * @param array Interaction profile array to destroy, must not be NULL
 * @public @memberof oxr_interaction_profile_array
 */
void
oxr_interaction_profile_array_clear(struct oxr_interaction_profile_array *array) XRT_NONNULL_ALL;

/*!
 * Add an interaction profile to the array.
 *
 * The profile is added to the end of the array. The array takes ownership of
 * the profile pointer.
 *
 * @param array Interaction profile array to add to, must not be NULL
 * @param profile Interaction profile to add, must not be NULL
 * @public @memberof oxr_interaction_profile_array
 */
void
oxr_interaction_profile_array_add(struct oxr_interaction_profile_array *array,
                                  struct oxr_interaction_profile *profile) XRT_NONNULL_ALL;

/*!
 * Clone an interaction profile array.
 *
 * Creates a deep copy of the source array, including cloning all interaction
 * profiles it contains. The destination array is first cleared (destroying any
 * existing profiles) before cloning.
 *
 * @param src Source interaction profile array to clone, must not be NULL
 * @param dst Destination interaction profile array, must not be NULL
 * @public @memberof oxr_interaction_profile_array
 */
void
oxr_interaction_profile_array_clone(const struct oxr_interaction_profile_array *src,
                                    struct oxr_interaction_profile_array *dst) XRT_NONNULL_ALL;

/*!
 * Find an interaction profile in the array by its path.
 *
 * Searches the array for an interaction profile with the matching XrPath.
 *
 * @param array Interaction profile array to search, must not be NULL
 * @param path XrPath to search for
 * @param[out] out_p Pointer to store the found interaction profile, must not be NULL. Set to NULL if not found.
 * @return true if a matching profile was found, false otherwise
 * @public @memberof oxr_interaction_profile_array
 */
bool
oxr_interaction_profile_array_find_by_path(const struct oxr_interaction_profile_array *array,
                                           XrPath path,
                                           struct oxr_interaction_profile **out_p) XRT_NONNULL_ALL;

/*!
 * Find an interaction profile in the array by device name.
 *
 * Maps a @ref xrt_device_name to an interaction profile by searching through
 * profile templates and then finding the matching profile in the array. There
 * might be multiple OpenXR interaction profiles that map to a single device
 * name, so the function continues searching until it finds a profile that
 * exists in the array.
 *
 * @param array Interaction profile array to search, must not be NULL
 * @param name Device name to search for, can be XRT_DEVICE_INVALID.
 * @param[out] out_p Pointer to store the found interaction profile, must not be NULL. Set to NULL if not found.
 * @return true if a matching profile was found, false otherwise
 * @public @memberof oxr_interaction_profile_array
 */
bool
oxr_interaction_profile_array_find_by_device_name(const struct oxr_interaction_profile_array *array,
                                                  const struct oxr_instance_path_cache *cache,
                                                  enum xrt_device_name name,
                                                  struct oxr_interaction_profile **out_p) XRT_NONNULL_ALL;

/*!
 * Find an interaction profile in the array by device. This function uses
 * @ref oxr_interaction_profile_array_find_by_device_name but also loops through
 * the bindings xrt_device::binding_profiles mappings.
 *
 * @param array Interaction profile array to search, must not be NULL.
 * @param[out] out_p Pointer to store the found interaction profile, must not be NULL. Set to NULL if not found.
 * @return true if a matching profile was found, false otherwise.
 * @public @memberof oxr_interaction_profile_array
 */
bool
oxr_interaction_profile_array_find_by_device(const struct oxr_interaction_profile_array *array,
                                             const struct oxr_instance_path_cache *cache,
                                             struct xrt_device *xdev,
                                             struct oxr_interaction_profile **out_p) XRT_NONNULL_FIRST;


#ifdef __cplusplus
}
#endif
