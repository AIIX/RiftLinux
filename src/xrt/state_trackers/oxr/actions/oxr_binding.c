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

#include "oxr_binding.h"
#include "oxr_subaction.h"
#include "oxr_generated_bindings.h"
#include "oxr_dpad_state.h"
#include "oxr_session_action_context.h"
#include "oxr_interaction_profile_array.h"

#include "../oxr_objects.h"
#include "../oxr_logger.h"
#include "../oxr_two_call.h"

#include <stdio.h>


static bool
get_subaction_path_from_path(const struct oxr_path_store *store,
                             XrPath path,
                             enum oxr_subaction_path *out_subaction_path)
{
	const char *str = NULL;
	size_t length = 0;
	XrResult ret;

	ret = oxr_path_store_get_string(store, path, &str, &length);
	if (ret != XR_SUCCESS) {
		return false;
	}

	if (length >= 10 && strncmp("/user/head", str, 10) == 0) {
		*out_subaction_path = OXR_SUB_ACTION_PATH_HEAD;
		return true;
	}
	if (length >= 15 && strncmp("/user/hand/left", str, 15) == 0) {
		*out_subaction_path = OXR_SUB_ACTION_PATH_LEFT;
		return true;
	}
	if (length >= 16 && strncmp("/user/hand/right", str, 16) == 0) {
		*out_subaction_path = OXR_SUB_ACTION_PATH_RIGHT;
		return true;
	}
	if (length >= 13 && strncmp("/user/gamepad", str, 13) == 0) {
		*out_subaction_path = OXR_SUB_ACTION_PATH_GAMEPAD;
		return true;
	}
	if (length >= 14 && strncmp("/user/eyes_ext", str, 14) == 0) {
		*out_subaction_path = OXR_SUB_ACTION_PATH_EYES;
		return true;
	}

	return false;
}

static void
setup_paths(struct oxr_path_store *store, const char **src_paths, XrPath **dest_paths, uint32_t *dest_path_count)
{
	uint32_t count = 0;
	while (src_paths[count] != NULL) {
		assert(count != UINT32_MAX);
		count++;
	}

	*dest_path_count = count;
	*dest_paths = U_TYPED_ARRAY_CALLOC(XrPath, count);

	for (size_t x = 0; x < *dest_path_count; x++) {
		const char *str = src_paths[x];
		size_t len = strlen(str);
		oxr_path_store_get_or_create(store, str, len, &(*dest_paths)[x]);
	}
}

static bool
get_profile_template_from_path(const struct oxr_instance_path_cache *cache,
                               XrPath path,
                               struct oxr_profile_template **out_templ)
{
	static_assert(OXR_BINDINGS_PROFILE_TEMPLATE_COUNT == ARRAY_SIZE(oxr_profile_templates), "Must match");
	static_assert(OXR_BINDINGS_PROFILE_TEMPLATE_COUNT == ARRAY_SIZE(cache->template_paths), "Must match");

	for (size_t x = 0; x < OXR_BINDINGS_PROFILE_TEMPLATE_COUNT; x++) {
		if (cache->template_paths[x] != path) {
			continue;
		}

		*out_templ = &oxr_profile_templates[x];
		return true;
	}

	*out_templ = NULL;

	return false;
}

static bool
interaction_profile_find_or_create_in_instance(struct oxr_logger *log,
                                               struct oxr_path_store *store,
                                               const struct oxr_instance_path_cache *cache,
                                               struct oxr_instance_action_context *context,
                                               XrPath path,
                                               struct oxr_interaction_profile **out_p)
{
	if (oxr_interaction_profile_array_find_by_path(&context->suggested_profiles, path, out_p)) {
		return true;
	}

	struct oxr_profile_template *templ = NULL;
	if (!get_profile_template_from_path(cache, path, &templ)) {
		*out_p = NULL;
		return false;
	}

	struct oxr_interaction_profile *p = U_TYPED_CALLOC(struct oxr_interaction_profile);

	p->xname = templ->name;
	p->binding_count = templ->binding_count;
	p->bindings = U_TYPED_ARRAY_CALLOC(struct oxr_binding, p->binding_count);
	p->dpad_count = templ->dpad_count;
	p->dpads = U_TYPED_ARRAY_CALLOC(struct oxr_dpad_emulation, p->dpad_count);
	p->path = path;
	p->localized_name = templ->localized_name;

	for (size_t x = 0; x < templ->binding_count; x++) {
		struct binding_template *t = &templ->bindings[x];
		struct oxr_binding *b = &p->bindings[x];

		XrPath subaction_path;
		XrResult r =
		    oxr_path_store_get_or_create(store, t->subaction_path, strlen(t->subaction_path), &subaction_path);
		if (r != XR_SUCCESS) {
			oxr_log(log, "Couldn't get subaction path %s\n", t->subaction_path);
		}

		if (!get_subaction_path_from_path(store, subaction_path, &b->subaction_path)) {
			oxr_log(log, "Invalid subaction path %s\n", t->subaction_path);
		}

		b->localized_name = t->localized_name;
		setup_paths(store, t->paths, &b->paths, &b->path_count);
		b->input = t->input;
		b->dpad_activate = t->dpad_activate;
		b->output = t->output;
	}

	for (size_t x = 0; x < templ->dpad_count; x++) {
		struct dpad_emulation *t = &templ->dpads[x];
		struct oxr_dpad_emulation *d = &p->dpads[x];

		XrPath subaction_path;
		XrResult r =
		    oxr_path_store_get_or_create(store, t->subaction_path, strlen(t->subaction_path), &subaction_path);
		if (r != XR_SUCCESS) {
			oxr_log(log, "Couldn't get subaction path %s\n", t->subaction_path);
		}

		if (!get_subaction_path_from_path(store, subaction_path, &d->subaction_path)) {
			oxr_log(log, "Invalid subaction path %s\n", t->subaction_path);
		}

		setup_paths(store, t->paths, &d->paths, &d->path_count);
		d->position = t->position;
		d->activate = t->activate;
	}

	// Add to the list of currently created interaction profiles.
	oxr_interaction_profile_array_add(&context->suggested_profiles, p);

	*out_p = p;

	return true;
}

static void
reset_binding_keys(struct oxr_binding *binding)
{
	free(binding->act_keys);
	free(binding->preferred_binding_path_index);
	binding->act_keys = NULL;
	binding->preferred_binding_path_index = NULL;
	binding->act_key_count = 0;
}

static void
reset_all_keys(struct oxr_binding *bindings, size_t binding_count)
{
	for (size_t x = 0; x < binding_count; x++) {
		reset_binding_keys(&bindings[x]);
	}
}

static bool
ends_with(const char *str, const char *suffix)
{
	size_t len = strlen(str);
	size_t suffix_len = strlen(suffix);

	return (len >= suffix_len) && (0 == strcmp(str + (len - suffix_len), suffix));
}

static bool
try_add_by_component(struct oxr_path_store *store,
                     struct oxr_binding *bindings,
                     size_t binding_count,
                     XrPath path,
                     struct oxr_action *act,
                     const char **components,
                     size_t component_count)
{
	for (uint32_t component_index = 0; component_index < component_count; component_index++) {
		// once we found everything for a component like click we don't want to keep going to add to a component
		// like /value
		// component is the outer loop so that we finish everything for a component in one go.
		bool found_all_for_component = false;

		for (size_t i = 0; i < binding_count; i++) {
			struct oxr_binding *b = &bindings[i];

			bool path_found = false;
			// search for path and component together and only add to the first found binding that has both
			bool component_found = false;

			uint32_t preferred_path_index;
			for (uint32_t y = 0; y < b->path_count; y++) {
				if (b->paths[y] == path) {
					path_found = true;
					// we preserve the info which path the app selected instead of pretending it
					// selected /click, /value, etc. if it did not
					preferred_path_index = y;
				}

				const char *str;
				size_t len;
				oxr_path_store_get_string(store, b->paths[y], &str, &len);
				if (ends_with(str, components[component_index])) {
					component_found = true;
				}
			}


			if (!(path_found && component_found)) {
				continue;
			}

			U_ARRAY_REALLOC_OR_FREE(b->act_keys, uint32_t, (b->act_key_count + 1));
			U_ARRAY_REALLOC_OR_FREE(b->preferred_binding_path_index, uint32_t, (b->act_key_count + 1));
			b->preferred_binding_path_index[b->act_key_count] = preferred_path_index;
			b->act_keys[b->act_key_count++] = act->act_key;
			found_all_for_component = true;
		}

		if (found_all_for_component) {
			return true;
		}
	}
	return false;
}

static bool
add_direct(struct oxr_binding *bindings, size_t binding_count, XrPath path, struct oxr_action *act)
{
	for (size_t i = 0; i < binding_count; i++) {
		struct oxr_binding *b = &bindings[i];

		bool found = false;
		uint32_t preferred_path_index;
		for (uint32_t y = 0; y < b->path_count; y++) {
			if (b->paths[y] == path) {
				found = true;
				preferred_path_index = y;
				break;
			}
		}

		if (!found) {
			continue;
		}

		U_ARRAY_REALLOC_OR_FREE(b->act_keys, uint32_t, (b->act_key_count + 1));
		U_ARRAY_REALLOC_OR_FREE(b->preferred_binding_path_index, uint32_t, (b->act_key_count + 1));
		b->preferred_binding_path_index[b->act_key_count] = preferred_path_index;
		b->act_keys[b->act_key_count++] = act->act_key;
	}

	return true;
}

static void
add_act_key_to_matching_bindings(struct oxr_path_store *store,
                                 struct oxr_binding *bindings,
                                 size_t binding_count,
                                 XrPath path,
                                 struct oxr_action *act)
{
	XrActionType xr_act_type = act->data->action_type;

	const char *str;
	size_t len;
	oxr_path_store_get_string(store, path, &str, &len);

	bool added = false;

	// check if we need to select a child, e.g. suggested str is */trigger for a bool action, or */trigger for a
	// float action
	if (xr_act_type == XR_ACTION_TYPE_BOOLEAN_INPUT && !ends_with(str, "/click") && !ends_with(str, "/touch")) {
		const char *components[2] = {"click", "value"};
		added = try_add_by_component(store, bindings, binding_count, path, act, components, 2);
	} else if (xr_act_type == XR_ACTION_TYPE_FLOAT_INPUT && !ends_with(str, "/value") &&
	           !ends_with(str, "/click")) {
		const char *components[2] = {"value", "click"};
		added = try_add_by_component(store, bindings, binding_count, path, act, components, 2);
	}

	// if the suggested str was not one of the ones that require us to select a child, fall back to the default case
	if (!added) {
		add_direct(bindings, binding_count, path, act);
	}
}

static void
add_string(char *temp, size_t max, ssize_t *current, const char *str)
{
	if (*current > 0) {
		temp[(*current)++] = ' ';
	}

	ssize_t len = snprintf(temp + *current, max - *current, "%s", str);
	if (len > 0) {
		*current += len;
	}
}

static const char *
get_subaction_path_str(enum oxr_subaction_path subaction_path)
{
	switch (subaction_path) {
	case OXR_SUB_ACTION_PATH_HEAD: return "Head";
	case OXR_SUB_ACTION_PATH_LEFT: return "Left";
	case OXR_SUB_ACTION_PATH_RIGHT: return "Right";
	case OXR_SUB_ACTION_PATH_GAMEPAD: return "Gameped";
	default: return NULL;
	}
}

static XrPath
get_interaction_bound_to_sub_path(const struct oxr_session_action_context *action_context,
                                  enum oxr_subaction_path subaction_path)
{
	switch (subaction_path) {
#define OXR_PATH_MEMBER(lower, CAP, PATH, LOCALIZED_NAME)                                                              \
	case OXR_SUB_ACTION_PATH_##CAP: return action_context->lower;

		OXR_FOR_EACH_VALID_SUBACTION_PATH_DETAILED(OXR_PATH_MEMBER)
#undef OXR_PATH_MEMBER
	default: return XR_NULL_PATH;
	}
}

static const char *
get_identifier_localized_name(XrPath path, const struct oxr_interaction_profile *profile)
{
	for (size_t i = 0; i < profile->binding_count; i++) {
		const struct oxr_binding *binding = &profile->bindings[i];

		for (size_t k = 0; k < binding->path_count; k++) {
			if (binding->paths[k] == path) {
				return binding->localized_name;
			}
		}
	}

	return NULL;
}


/*
 *
 * 'Exported' functions.
 *
 */

void
oxr_interaction_profile_destroy(struct oxr_interaction_profile *profile)
{
	if (profile == NULL) {
		return;
	}

	for (size_t y = 0; y < profile->binding_count; y++) {
		struct oxr_binding *b = &profile->bindings[y];

		reset_binding_keys(b);
		free(b->paths);
		b->paths = NULL;
		b->path_count = 0;
		b->input = 0;
		b->output = 0;
	}

	for (size_t y = 0; y < profile->dpad_count; ++y) {
		struct oxr_dpad_emulation *d = &profile->dpads[y];
		free(d->paths);
	}

	free(profile->bindings);
	profile->bindings = NULL;
	profile->binding_count = 0;

	free(profile->dpads);

	oxr_dpad_state_deinit(&profile->dpad_state);

	free(profile);
}

struct oxr_interaction_profile *
oxr_interaction_profile_clone(const struct oxr_interaction_profile *src_profile)
{
	if (src_profile == NULL)
		return NULL;

	struct oxr_interaction_profile *dst_profile = U_TYPED_CALLOC(struct oxr_interaction_profile);

	*dst_profile = *src_profile;

	dst_profile->binding_count = 0;
	dst_profile->bindings = NULL;
	if (src_profile->bindings && src_profile->binding_count > 0) {

		dst_profile->binding_count = src_profile->binding_count;
		dst_profile->bindings = U_TYPED_ARRAY_CALLOC(struct oxr_binding, src_profile->binding_count);

		for (size_t binding_idx = 0; binding_idx < src_profile->binding_count; ++binding_idx) {
			struct oxr_binding *dst_binding = dst_profile->bindings + binding_idx;
			const struct oxr_binding *src_binding = src_profile->bindings + binding_idx;

			*dst_binding = *src_binding;

			dst_binding->path_count = 0;
			dst_binding->paths = NULL;
			if (src_binding->paths && src_binding->path_count > 0) {
				dst_binding->path_count = src_binding->path_count;
				dst_binding->paths = U_TYPED_ARRAY_CALLOC(XrPath, src_binding->path_count);
				memcpy(dst_binding->paths, src_binding->paths,
				       sizeof(XrPath) * src_binding->path_count);
			}

			dst_binding->act_key_count = 0;
			dst_binding->act_keys = NULL;
			dst_binding->preferred_binding_path_index = NULL;
			if (src_binding->act_keys && src_binding->act_key_count > 0) {
				dst_binding->act_key_count = src_binding->act_key_count;
				dst_binding->act_keys = U_TYPED_ARRAY_CALLOC(uint32_t, src_binding->act_key_count);
				memcpy(dst_binding->act_keys, src_binding->act_keys,
				       sizeof(uint32_t) * src_binding->act_key_count);
			}
			if (src_binding->preferred_binding_path_index && src_binding->act_key_count > 0) {
				assert(dst_binding->act_key_count == src_binding->act_key_count);
				dst_binding->preferred_binding_path_index =
				    U_TYPED_ARRAY_CALLOC(uint32_t, src_binding->act_key_count);
				memcpy(dst_binding->preferred_binding_path_index,
				       src_binding->preferred_binding_path_index,
				       sizeof(uint32_t) * src_binding->act_key_count);
			}
		}
	}

	dst_profile->dpad_count = 0;
	dst_profile->dpads = NULL;
	if (src_profile->dpads && src_profile->dpad_count > 0) {

		dst_profile->dpad_count = src_profile->dpad_count;
		dst_profile->dpads = U_TYPED_ARRAY_CALLOC(struct oxr_dpad_emulation, src_profile->dpad_count);

		for (size_t dpad_index = 0; dpad_index < src_profile->dpad_count; ++dpad_index) {
			struct oxr_dpad_emulation *dst_dpad = dst_profile->dpads + dpad_index;
			const struct oxr_dpad_emulation *src_dpad = src_profile->dpads + dpad_index;

			*dst_dpad = *src_dpad;

			dst_dpad->path_count = 0;
			dst_dpad->paths = NULL;
			if (src_dpad->paths && src_dpad->path_count > 0) {
				dst_dpad->path_count = src_dpad->path_count;
				dst_dpad->paths = U_TYPED_ARRAY_CALLOC(XrPath, src_dpad->path_count);
				memcpy(dst_dpad->paths, src_dpad->paths, sizeof(XrPath) * src_dpad->path_count);
			}
		}
	}

	const struct oxr_dpad_state empty_dpad_state = {.uhi = NULL};
	dst_profile->dpad_state = empty_dpad_state;
	oxr_dpad_state_clone(&dst_profile->dpad_state, &src_profile->dpad_state);

	return dst_profile;
}

void
oxr_binding_find_bindings_from_act_key(struct oxr_logger *log,
                                       struct oxr_interaction_profile *profile,
                                       uint32_t key,
                                       size_t max_binding_count,
                                       struct oxr_binding **out_bindings,
                                       size_t *out_binding_count)
{
	if (profile == NULL) {
		*out_binding_count = 0;
		return;
	}

	// How many bindings are we returning?
	size_t binding_count = 0;

	/*
	 * Loop over all app provided bindings for this profile
	 * and return those matching the action.
	 */
	for (size_t binding_index = 0; binding_index < profile->binding_count; binding_index++) {
		struct oxr_binding *profile_binding = &profile->bindings[binding_index];

		for (size_t key_index = 0; key_index < profile_binding->act_key_count; key_index++) {
			if (profile_binding->act_keys[key_index] == key) {
				out_bindings[binding_count++] = profile_binding;
				break;
			}
		}

		//! @todo Should return total count instead of fixed max.
		if (binding_count >= max_binding_count) {
			oxr_warn(log, "Internal limit reached, action has too many bindings!");
			break;
		}
	}

	assert(binding_count <= max_binding_count);

	*out_binding_count = binding_count;
}


/*
 *
 * Client functions.
 *
 */

XrResult
oxr_action_suggest_interaction_profile_bindings(struct oxr_logger *log,
                                                struct oxr_path_store *store,
                                                const struct oxr_instance_path_cache *cache,
                                                struct oxr_instance_action_context *inst_context,
                                                const XrInteractionProfileSuggestedBinding *suggestedBindings,
                                                struct oxr_dpad_state *dpad_state)
{
	struct oxr_interaction_profile *p = NULL;

	// Path already validated.
	XrPath path = suggestedBindings->interactionProfile;
	interaction_profile_find_or_create_in_instance( //
	    log,                                        //
	    store,                                      //
	    cache,                                      //
	    inst_context,                               //
	    path,                                       //
	    &p);                                        //

	// Valid path, but not used.
	if (p == NULL) {
		goto out;
	}

	struct oxr_binding *bindings = p->bindings;
	size_t binding_count = p->binding_count;

	// Everything is now valid, reset the keys.
	reset_all_keys(bindings, binding_count);
	// Transfer ownership of dpad state to profile
	oxr_dpad_state_deinit(&p->dpad_state);
	p->dpad_state = *dpad_state;
	U_ZERO(dpad_state);

	for (size_t i = 0; i < suggestedBindings->countSuggestedBindings; i++) {
		const XrActionSuggestedBinding *s = &suggestedBindings->suggestedBindings[i];
		struct oxr_action *act = XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_action *, s->action);

		add_act_key_to_matching_bindings( //
		    store,                        //
		    bindings,                     //
		    binding_count,                //
		    s->binding,                   //
		    act);                         //
	}

out:
	oxr_dpad_state_deinit(dpad_state); // if it hasn't been moved

	return XR_SUCCESS;
}

XrResult
oxr_action_get_current_interaction_profile(struct oxr_logger *log,
                                           const struct oxr_instance_path_cache *cache,
                                           const struct oxr_session_action_context *action_context,
                                           XrPath topLevelUserPath,
                                           XrInteractionProfileState *interactionProfile)
{
	// Action sets attachment is checked in the API function.

#define IDENTIFY_TOP_LEVEL_PATH(X)                                                                                     \
	if (topLevelUserPath == cache->X) {                                                                            \
		interactionProfile->interactionProfile = action_context->X;                                            \
	} else

	OXR_FOR_EACH_VALID_SUBACTION_PATH(IDENTIFY_TOP_LEVEL_PATH)
	{
		// else clause
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Top level path not handled?!");
	}
#undef IDENTIFY_TOP_LEVEL_PATH
	return XR_SUCCESS;
}

XrResult
oxr_action_get_input_source_localized_name(struct oxr_logger *log,
                                           const struct oxr_path_store *store,
                                           const struct oxr_session_action_context *action_context,
                                           const XrInputSourceLocalizedNameGetInfo *getInfo,
                                           uint32_t bufferCapacityInput,
                                           uint32_t *bufferCountOutput,
                                           char *buffer)
{
	char temp[1024] = {0};
	ssize_t current = 0;
	enum oxr_subaction_path subaction_path = 0;

	if (!get_subaction_path_from_path(store, getInfo->sourcePath, &subaction_path)) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "(getInfo->sourcePath) doesn't start with a "
		                 "valid subaction_path");
	}

	// Get the interaction profile bound to this subaction_path.
	XrPath path = get_interaction_bound_to_sub_path(action_context, subaction_path);
	if (path == XR_NULL_PATH) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "(getInfo->sourcePath) no interaction profile "
		                 "bound to subaction path");
	}

	// Find the interaction profile.
	struct oxr_interaction_profile *oip = NULL;
	//! @todo: If we ever rebind a profile that has not been suggested by the client, it will not be found.
	oxr_interaction_profile_array_find_by_path(&action_context->profiles_on_attachment, path, &oip);
	if (oip == NULL) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "no interaction profile found");
	}

	// Add which hand to use.
	if (getInfo->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_USER_PATH_BIT) {
		add_string(temp, sizeof(temp), &current, get_subaction_path_str(subaction_path));
	}

	// Add a human readable and localized name of the device.
	if ((getInfo->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_INTERACTION_PROFILE_BIT) != 0) {
		add_string(temp, sizeof(temp), &current, oip->localized_name);
	}

	//! @todo This implementation is very very very inelegant.
	if ((getInfo->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_COMPONENT_BIT) != 0) {
		const char *localized_name = get_identifier_localized_name(getInfo->sourcePath, oip);
		if (localized_name == NULL) {
			return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "couldn't get identifier localized name");
		}

		/*
		 * The preceding enum is misnamed: it should be called identifier
		 * instead of component. But, this is a spec bug.
		 */
		add_string(temp, sizeof(temp), &current, localized_name);
	}

	// Include the null character.
	current += 1;

	OXR_TWO_CALL_HELPER(log, bufferCapacityInput, bufferCountOutput, buffer, (size_t)current, temp, XR_SUCCESS);
}
