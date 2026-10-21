// Copyright 2018-2024, Collabora, Ltd.
// Copyright 2023-2026, NVIDIA CORPORATION.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Holds input related functions.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup oxr_main
 */

#include "xrt/xrt_compiler.h"

#include "util/u_debug.h"
#include "util/u_time.h"
#include "util/u_misc.h"

#include "math/m_vec2.h"

#include "oxr_input.h"
#include "oxr_binding.h"
#include "oxr_pair_hashset.h"
#include "oxr_subaction.h"
#include "oxr_dpad_state.h"
#include "oxr_input_transform.h"
#include "oxr_generated_bindings.h"
#include "oxr_handle_base.h"

#include "../oxr_objects.h"
#include "../oxr_logger.h"
#include "../oxr_two_call.h"
#include "../oxr_conversions.h"
#include "../oxr_xret.h"
#include "../oxr_roles.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>


/*
 *
 * Structs.
 *
 */

struct oxr_profiles_per_subaction
{
#define PROFILE_MEMBER(X) struct oxr_interaction_profile *X;
	OXR_FOR_EACH_VALID_SUBACTION_PATH(PROFILE_MEMBER)
#undef PROFILE_MEMBER
};


/*
 *
 * Helpers
 *
 */

static void
print_profile_to_slog(struct oxr_sink_logger *slog,
                      const struct oxr_path_store *store,
                      const struct oxr_interaction_profile *profile,
                      const char *point)
{
	if (!profile) {
		oxr_slog(slog, "\n\t%s: (null)", point);
		return;
	}

	const char *str = NULL;
	size_t length = 0;

	oxr_path_store_get_string(store, profile->path, &str, &length);
	oxr_slog(slog, "\n\t%s: %s '%s'", point, str, profile->localized_name);
}

static void
print_profiles(struct oxr_logger *log,
               const struct oxr_path_store *store,
               const struct oxr_profiles_per_subaction *profiles)
{
	struct oxr_sink_logger slog = {0};

	oxr_slog(&slog, "Profiles:");

#define PRINT_PROFILE(X) print_profile_to_slog(&slog, store, profiles->X, #X);
	OXR_FOR_EACH_VALID_SUBACTION_PATH(PRINT_PROFILE)
#undef PRINT_PROFILE

	oxr_log_slog(log, &slog);
}

/*!
 * Given an Action Set handle, return the @ref oxr_action_set and the
 * associated @ref oxr_action_set_attachment in the given Session.
 */
static void
get_action_set_attachment(struct oxr_session_action_context *sess_context,
                          XrActionSet actionSet,
                          struct oxr_action_set_attachment **out_act_set_attached,
                          struct oxr_action_set **out_act_set)
{
	struct oxr_action_set *act_set = XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_action_set *, actionSet);

	// Make sure all are set.
	*out_act_set = act_set;
	*out_act_set_attached = NULL;

	// Safe to call even if nothing has been attached.
	oxr_session_action_context_find_set(sess_context, act_set->act_set_key, out_act_set_attached);
}


/*
 *
 * Pre declare functions.
 *
 */

static void
oxr_action_cache_update(struct oxr_logger *log,
                        struct oxr_session *sess,
                        uint32_t countActionSets,
                        const XrActiveActionSet *actionSets,
                        struct oxr_action_attachment *act_attached,
                        struct oxr_action_cache *cache,
                        int64_t time,
                        struct oxr_subaction_paths *subaction_path,
                        bool select,
                        const XrActiveActionSetPrioritiesEXT *active_priorities);

static void
oxr_action_attachment_update(struct oxr_logger *log,
                             struct oxr_session *sess,
                             uint32_t countActionSets,
                             const XrActiveActionSet *actionSets,
                             struct oxr_action_attachment *act_attached,
                             int64_t time,
                             struct oxr_subaction_paths subaction_paths,
                             const XrActiveActionSetPrioritiesEXT *active_priorities);

static void
oxr_action_bind_io(struct oxr_logger *log,
                   struct oxr_sink_logger *slog,
                   const struct oxr_path_store *store,
                   const struct oxr_roles *roles,
                   const struct oxr_action_ref *act_ref,
                   const uint32_t act_set_key,
                   struct oxr_action_cache *cache,
                   struct oxr_interaction_profile *profile,
                   enum oxr_subaction_path subaction_path);

/*!
 * Helper function to combine @ref oxr_subaction_paths structs, but skipping
 * @ref oxr_subaction_paths::any.
 *
 * If a real (non-"any") subaction path in @p new_subaction_paths is true, it will be
 * true in @p subaction_paths.
 *
 * @private @memberof oxr_subaction_paths
 */
static inline void
oxr_subaction_paths_accumulate_except_any(struct oxr_subaction_paths *subaction_paths,
                                          const struct oxr_subaction_paths *new_subaction_paths)
{
#define ACCUMULATE_SUBACTION_PATHS(X) subaction_paths->X |= new_subaction_paths->X;
	OXR_FOR_EACH_SUBACTION_PATH(ACCUMULATE_SUBACTION_PATHS)
#undef ACCUMULATE_SUBACTION_PATHS
}

/*!
 * Helper function to combine @ref oxr_subaction_paths structs.
 *
 * If a subaction path in @p new_subaction_paths is true, it will be true in @p subaction_paths.
 *
 * @private @memberof oxr_subaction_paths
 */
static inline void
oxr_subaction_paths_accumulate(struct oxr_subaction_paths *subaction_paths,
                               const struct oxr_subaction_paths *new_subaction_paths)
{
	subaction_paths->any |= new_subaction_paths->any;
	oxr_subaction_paths_accumulate_except_any(subaction_paths, new_subaction_paths);
}

/*
 *
 * Action attachment functions
 *
 */

/*!
 * De-initialize/de-allocate all dynamic members of @ref oxr_action_cache and
 * reset all fields. This function is used when destroying an action that has
 * been attached to the session (@ref oxr_action_attachment), or when bindings
 * are (re)made for the action. Bindings can be (re)made multiple times during
 * the runtime of the session, such as when a device is dynamically moved
 * between roles.
 *
 * @private @memberof oxr_action_cache
 */
static void
oxr_action_cache_teardown(struct oxr_action_cache *cache)
{
	// Clean up input transforms
	for (uint32_t i = 0; i < cache->input_count; i++) {
		struct oxr_action_input *action_input = &cache->inputs[i];
		oxr_input_transform_destroy(&(action_input->transforms));
		action_input->transform_count = 0;
	}

	free(cache->inputs);
	cache->inputs = NULL;

	free(cache->outputs);
	cache->outputs = NULL;

	U_ZERO(cache);
}

/*!
 * Tear down an action attachment struct.
 *
 * Does not deallocate the struct itself.
 *
 * @public @memberof oxr_action_attachment
 */
static void
oxr_action_attachment_teardown(struct oxr_action_attachment *act_attached)
{
	oxr_session_attached_actions_remove_action_attachment(act_attached->attached_actions, act_attached);

#define CACHE_TEARDOWN(X) oxr_action_cache_teardown(&(act_attached->X));
	OXR_FOR_EACH_SUBACTION_PATH(CACHE_TEARDOWN)
#undef CACHE_TEARDOWN

	// Unref this action's refcounted data
	oxr_refcounted_unref(&act_attached->act_ref->base);
}

/*!
 * Set up an action attachment struct.
 *
 * @public @memberof oxr_action_attachment
 */
static XrResult
oxr_action_attachment_init(struct oxr_logger *log,
                           struct oxr_action_attachment *act_attached,
                           struct oxr_session_attached_actions *attached_actions,
                           struct oxr_action_set_attachment *act_set_attached,
                           struct oxr_action_ref *act_ref)
{
	// Reference this action's refcounted data
	oxr_refcounted_ref(&act_ref->base);

	act_attached->attached_actions = attached_actions;
	act_attached->act_set_attached = act_set_attached;
	act_attached->act_key = act_ref->act_key; // Copy this for efficiency.
	act_attached->act_ref = act_ref;

	XrResult ret = oxr_session_attached_actions_add_action_attachment(attached_actions, act_attached);
	if (ret != XR_SUCCESS) {
		oxr_refcounted_unref(&act_attached->act_ref->base);
		U_ZERO(act_attached);
		return oxr_error(log, ret, "Failed to add action attachment to session attached actions");
	}

	return XR_SUCCESS;
}


/*
 *
 * Action set attachment functions
 *
 */

/*!
 * @public @memberof oxr_action_set_attachment
 */
static XrResult
oxr_action_set_attachment_init(struct oxr_logger *log,
                               struct oxr_action_set_attachment *act_set_attached,
                               struct oxr_session_action_context *action_context,
                               struct oxr_action_set_ref *act_set_ref)
{
	// Reference this action set's refcounted data
	oxr_refcounted_ref(&act_set_ref->base);

	act_set_attached->sess_context = action_context;
	act_set_attached->act_set_key = act_set_ref->act_set_key; // Copy this for efficiency.
	act_set_attached->act_set_ref = act_set_ref;

	int h_ret = u_hashmap_int_insert(                //
	    action_context->act_sets_attachments_by_key, //
	    act_set_ref->act_set_key,                    //
	    act_set_attached);                           //
	if (h_ret < 0) {
		oxr_refcounted_unref(&act_set_attached->act_set_ref->base);
		U_ZERO(act_set_attached);
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE,
		                 "Failed to add action set attachment to session action context");
	}

	return XR_SUCCESS;
}

void
oxr_action_set_attachment_teardown(struct oxr_action_set_attachment *act_set_attached)
{
	for (size_t i = 0; i < act_set_attached->action_attachment_count; ++i) {
		oxr_action_attachment_teardown(&(act_set_attached->act_attachments[i]));
	}

	free(act_set_attached->act_attachments);
	act_set_attached->act_attachments = NULL;
	act_set_attached->action_attachment_count = 0;

	struct oxr_session_action_context *sess_context = act_set_attached->sess_context;
	u_hashmap_int_erase(sess_context->act_sets_attachments_by_key, act_set_attached->act_set_key);

	// Unref this action set's refcounted data
	oxr_refcounted_unref(&act_set_attached->act_set_ref->base);
}


/*
 *
 * Action set functions
 *
 */
static void
oxr_action_set_ref_destroy_cb(struct oxr_refcounted *orc)
{
	struct oxr_action_set_ref *act_set_ref = (struct oxr_action_set_ref *)orc;

	oxr_pair_hashset_fini(&act_set_ref->actions);

	free(act_set_ref);
}

static XrResult
oxr_action_set_destroy_cb(struct oxr_logger *log, struct oxr_handle_base *hb)
{
	struct oxr_action_set *act_set = (struct oxr_action_set *)hb;
	struct oxr_instance_action_context *inst_context = act_set->inst_context;

	oxr_refcounted_unref(&act_set->data->base);
	act_set->data = NULL;

	if (inst_context != NULL) {
		oxr_pair_hashset_erase_and_free(&inst_context->action_sets, &act_set->name_item, &act_set->loc_item);
		act_set->inst_context = NULL;
		oxr_refcounted_unref(&inst_context->base);
	}

	free(act_set);

	return XR_SUCCESS;
}

XrResult
oxr_action_set_create(struct oxr_logger *log,
                      struct oxr_instance *inst,
                      struct oxr_instance_action_context *inst_context,
                      const XrActionSetCreateInfo *createInfo,
                      struct oxr_action_set **out_act_set)
{
	// Mod music for all!
	static uint32_t key_gen = 1;
	XrResult ret;
	int h_ret;

	struct oxr_action_set *act_set = NULL;
	OXR_ALLOCATE_HANDLE_PARENT_OR_RETURN(log, act_set, OXR_XR_DEBUG_ACTIONSET, oxr_action_set_destroy_cb,
	                                     &inst->handle);

	act_set->inst_context = NULL;

	struct oxr_action_set_ref *act_set_ref = U_TYPED_CALLOC(struct oxr_action_set_ref);
	act_set_ref->permitted_subaction_paths.any = true;
	act_set_ref->base.destroy = oxr_action_set_ref_destroy_cb;
	oxr_refcounted_ref(&act_set_ref->base);
	act_set->data = act_set_ref;

	act_set_ref->act_set_key = key_gen++;
	act_set->act_set_key = act_set_ref->act_set_key;

	act_set->inst = inst;

	ret = oxr_pair_hashset_init(log, &act_set_ref->actions);
	if (ret != XR_SUCCESS) {
		oxr_handle_parent_destroy(log, &act_set->handle);
		return ret;
	}

	snprintf(act_set_ref->name, sizeof(act_set_ref->name), "%s", createInfo->actionSetName);

	h_ret = oxr_pair_hashset_insert_str_c(  //
	    &inst_context->action_sets,         //
	    createInfo->actionSetName,          //
	    createInfo->localizedActionSetName, //
	    &act_set->name_item,                //
	    &act_set->loc_item);                //
	if (h_ret != 0) {
		oxr_handle_parent_destroy(log, &act_set->handle);
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Failed to insert action set name pair");
	}

	act_set_ref->priority = createInfo->priority;

	oxr_refcounted_ref(&inst_context->base);
	act_set->inst_context = inst_context;

	*out_act_set = act_set;

	return XR_SUCCESS;
}


/*
 *
 * Action functions
 *
 */

static void
oxr_action_ref_destroy_cb(struct oxr_refcounted *orc)
{
	struct oxr_action_ref *act_ref = (struct oxr_action_ref *)orc;
	free(act_ref);
}

static XrResult
oxr_action_destroy_cb(struct oxr_logger *log, struct oxr_handle_base *hb)
{
	struct oxr_action *act = (struct oxr_action *)hb;

	oxr_refcounted_unref(&act->data->base);
	act->data = NULL;

	oxr_pair_hashset_erase_and_free(&act->act_set->data->actions, &act->name_item, &act->loc_item);

	free(act);

	return XR_SUCCESS;
}

XrResult
oxr_action_create(struct oxr_logger *log,
                  struct oxr_action_set *act_set,
                  const XrActionCreateInfo *createInfo,
                  struct oxr_action **out_act)
{
	struct oxr_instance *inst = act_set->inst;
	struct oxr_subaction_paths subaction_paths = {0};

	// Mod music for all!
	static uint32_t key_gen = 1;
	int h_ret;

	bool bret = oxr_classify_subaction_paths( //
	    &inst->path_cache,                    //
	    createInfo->countSubactionPaths,      //
	    createInfo->subactionPaths,           //
	    &subaction_paths);                    //
	if (!bret) {
		//! @todo This is probably an internal runtime error since it should be validated before?
		return oxr_error(log, XR_ERROR_PATH_UNSUPPORTED, "Bad internal codepath taken (bug?)");
	}

	struct oxr_action *act = NULL;
	OXR_ALLOCATE_HANDLE_OR_RETURN(log, act, OXR_XR_DEBUG_ACTION, oxr_action_destroy_cb, &act_set->handle);


	struct oxr_action_ref *act_ref = U_TYPED_CALLOC(struct oxr_action_ref);
	act_ref->base.destroy = oxr_action_ref_destroy_cb;
	oxr_refcounted_ref(&act_ref->base);
	act->data = act_ref;

	act_ref->act_key = key_gen++;
	act->act_key = act_ref->act_key;

	act->act_set = act_set;
	act_ref->subaction_paths = subaction_paths;
	act_ref->action_type = createInfo->actionType;
	act_ref->subaction_paths = subaction_paths;

	// Any subaction paths allowed for this action are allowed for this
	// action set. But, do not accumulate "any" - it just means none were
	// specified for this action.
	oxr_subaction_paths_accumulate_except_any(&(act_set->data->permitted_subaction_paths), &subaction_paths);

	snprintf(act_ref->name, sizeof(act_ref->name), "%s", createInfo->actionName);

	h_ret = oxr_pair_hashset_insert_str_c( //
	    &act_set->data->actions,           //
	    createInfo->actionName,            //
	    createInfo->localizedActionName,   //
	    &act->name_item,                   //
	    &act->loc_item);                   //
	if (h_ret != 0) {
		oxr_handle_destroy(log, &act->handle);
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Failed to insert action name pair");
	}

	*out_act = act;

	return XR_SUCCESS;
}


/*
 *
 * "Exported" helper functions.
 *
 */

XrResult
oxr_action_get_pose_input(struct oxr_session *sess,
                          uint32_t act_key,
                          const struct oxr_subaction_paths *subaction_paths_ptr,
                          struct oxr_action_input **out_input)
{
	struct oxr_action_attachment *act_attached = NULL;

	oxr_session_attached_actions_find(&sess->attached_actions, act_key, &act_attached);
	if (act_attached == NULL) {
		return XR_SUCCESS;
	}

	struct oxr_subaction_paths subaction_paths = *subaction_paths_ptr;
	if (subaction_paths.any) {
		subaction_paths = act_attached->any_pose_subaction_path;
	}

	// Priority of inputs.
#define GET_POSE_INPUT(X)                                                                                              \
	if (act_attached->X.current.active && subaction_paths.X) {                                                     \
		*out_input = act_attached->X.inputs;                                                                   \
		return XR_SUCCESS;                                                                                     \
	}
	OXR_FOR_EACH_VALID_SUBACTION_PATH(GET_POSE_INPUT)

	// plus a fallback invocation for user
	GET_POSE_INPUT(user)
#undef GET_POSE_INPUT

	return XR_SUCCESS;
}


/*
 *
 * Not so hack functions.
 *
 */

static bool
find_xdev_name_from_pairs(const struct xrt_device *xdev,
                          const struct xrt_binding_profile *xbp,
                          enum xrt_input_name from_name,
                          enum xrt_input_name *out_name)
{
	if (from_name == 0) {
		*out_name = 0;
		return true; // Not asking for anything, just keep going.
	}

	/*
	 * For asymmetrical devices like PS Sense being re-bound to a symmetrical
	 * "device" like simple controller can be problemantic as depending on
	 * which hand it is menu is bound to different inputs. Instead of making
	 * the driver have two completely unique binding mappings per hand, we
	 * instead loop over all pairs finding the first match. In other words
	 * this means there can be multiple of the same 'from' value in the
	 * array of input pairs.
	 */
	for (size_t i = 0; i < xbp->input_count; i++) {
		if (from_name != xbp->inputs[i].from) {
			continue;
		}

		// What is the name on the xdev?
		enum xrt_input_name xdev_name = xbp->inputs[i].device;

		// See if we can't find it.
		for (uint32_t k = 0; k < xdev->input_count; k++) {
			if (xdev->inputs[k].name == xdev_name) {
				*out_name = xdev_name;
				return true;
			}
		}
	}

	return false;
}

static bool
do_inputs(struct oxr_binding *binding_point,
          struct xrt_device *xdev,
          struct xrt_binding_profile *xbp,
          XrPath matched_path,
          struct oxr_action_input inputs[OXR_MAX_BINDINGS_PER_ACTION],
          uint32_t *input_count)
{
	enum xrt_input_name name = 0;
	enum xrt_input_name dpad_activate_name = 0;

	if (xbp != NULL) {
		bool t1 = find_xdev_name_from_pairs(xdev, xbp, binding_point->input, &name);
		bool t2 = find_xdev_name_from_pairs(xdev, xbp, binding_point->dpad_activate, &dpad_activate_name);

		// We couldn't find the needed inputs on the device.
		if (!t1 || !t2) {
			return false;
		}
	} else {
		name = binding_point->input;
		dpad_activate_name = binding_point->dpad_activate;
	}

	struct xrt_input *input = NULL;
	struct xrt_input *dpad_activate_input = NULL;

	// Early out if there is no such input.
	if (!oxr_xdev_find_input(xdev, name, &input)) {
		return false;
	}

	// Check this before allocating an input.
	if (dpad_activate_name != 0) {
		if (!oxr_xdev_find_input(xdev, dpad_activate_name, &dpad_activate_input)) {
			return false;
		}
	}

	uint32_t index = (*input_count)++;
	inputs[index].input = input;
	inputs[index].xdev = xdev;
	inputs[index].bound_path = matched_path;
	if (dpad_activate_input != NULL) {
		inputs[index].dpad_activate_name = dpad_activate_name;
		inputs[index].dpad_activate = dpad_activate_input;
	}

	return true;
}

static bool
do_outputs(struct oxr_binding *binding_point,
           struct xrt_device *xdev,
           struct xrt_binding_profile *xbp,
           XrPath matched_path,
           struct oxr_action_output outputs[OXR_MAX_BINDINGS_PER_ACTION],
           uint32_t *output_count)
{
	enum xrt_output_name name = 0;
	if (xbp == NULL) {
		name = binding_point->output;
	} else {
		for (size_t i = 0; i < xbp->output_count; i++) {
			if (binding_point->output != xbp->outputs[i].from) {
				continue;
			}

			// We have found a device mapping.
			name = xbp->outputs[i].device;
			break;
		}

		// Didn't find a mapping.
		if (name == 0) {
			return false;
		}
	}

	struct xrt_output *output = NULL;
	if (oxr_xdev_find_output(xdev, name, &output)) {
		uint32_t index = (*output_count)++;
		outputs[index].name = name;
		outputs[index].xdev = xdev;
		outputs[index].bound_path = matched_path;
		return true;
	}

	return false;
}

/*!
 * Delegate to @ref do_outputs or @ref do_inputs depending on whether the action
 * is output or input.
 */
static bool
do_io_bindings(struct oxr_binding *binding_point,
               const struct oxr_action_ref *act_ref,
               struct xrt_device *xdev,
               struct xrt_binding_profile *xbp,
               XrPath matched_path,
               struct oxr_action_input inputs[OXR_MAX_BINDINGS_PER_ACTION],
               uint32_t *input_count,
               struct oxr_action_output outputs[OXR_MAX_BINDINGS_PER_ACTION],
               uint32_t *output_count)
{
	if (act_ref->action_type == XR_ACTION_TYPE_VIBRATION_OUTPUT) {
		return do_outputs( //
		    binding_point, //
		    xdev,          //
		    xbp,           //
		    matched_path,  //
		    outputs,       //
		    output_count); //
	}
	return do_inputs(  //
	    binding_point, //
	    xdev,          //
	    xbp,           //
	    matched_path,  //
	    inputs,        //
	    input_count);  //
}

static struct xrt_binding_profile *
get_matching_binding_profile(struct oxr_interaction_profile *profile, struct xrt_device *xdev)
{
	for (size_t i = 0; i < xdev->binding_profile_count; i++) {
		if (xdev->binding_profiles[i].name == profile->xname) {
			return &xdev->binding_profiles[i];
		}
	}

	return NULL;
}

static XrPath
get_matched_xrpath(struct oxr_binding *b, const struct oxr_action_ref *act)
{
	XrPath preferred_path = XR_NULL_PATH;
	for (uint32_t i = 0; i < b->act_key_count; i++) {
		if (b->act_keys[i] == act->act_key) {
			uint32_t preferred_path_index = XR_NULL_PATH;
			preferred_path_index = b->preferred_binding_path_index[i];
			preferred_path = b->paths[preferred_path_index];
			break;
		}
	}
	return preferred_path;
}

static void
get_binding(struct oxr_logger *log,
            struct oxr_sink_logger *slog,
            const struct oxr_path_store *store,
            const struct oxr_roles *roles,
            const struct oxr_action_ref *act_ref,
            struct oxr_interaction_profile *profile,
            enum oxr_subaction_path subaction_path,
            struct oxr_action_input inputs[OXR_MAX_BINDINGS_PER_ACTION],
            uint32_t *input_count,
            struct oxr_action_output outputs[OXR_MAX_BINDINGS_PER_ACTION],
            uint32_t *output_count)
{
	struct xrt_device *xdev = NULL;
	struct oxr_binding *binding_points[OXR_MAX_BINDINGS_PER_ACTION];
	const char *profile_str;
	const char *user_path_str;
	size_t length;

	//! @todo This probably falls on its head if the application doesn't use
	//! sub action paths.
	switch (subaction_path) {
#define PATH_CASE(NAME, NAMECAPS, PATH, LOCALIZED_NAME)                                                                \
	case OXR_SUB_ACTION_PATH_##NAMECAPS:                                                                           \
		user_path_str = PATH;                                                                                  \
		xdev = GET_XDEV_BY_ROLE(roles, NAME);                                                                  \
		break;

		OXR_FOR_EACH_VALID_SUBACTION_PATH_DETAILED(PATH_CASE)
#undef PATH_CASE

		// Manually-coded fallback for not-really-valid /user
	case OXR_SUB_ACTION_PATH_USER:
		user_path_str = "/user";
		xdev = NULL;
		break;
	default: break;
	}

	oxr_slog(slog, "\tFor: %s\n", user_path_str);

	if (xdev == NULL) {
		oxr_slog(slog, "\t\tNo xdev!\n");
		return;
	}

	if (profile == NULL) {
		oxr_slog(slog, "\t\tNo profile!\n");
		return;
	}

	oxr_path_store_get_string(store, profile->path, &profile_str, &length);

	oxr_slog(slog, "\t\tProfile: %s\n", profile_str);

	/*!
	 * Lookup device binding that matches the interactive profile, this
	 * is used as a fallback should the device not match the interactive
	 * profile. This allows the device to provide a mapping from one device
	 * to itself.
	 */
	struct xrt_binding_profile *xbp = get_matching_binding_profile(profile, xdev);

	// No point in proceeding here without either.
	if (profile->xname != xdev->name && xbp == NULL) {
		oxr_slog(slog, "\t\t\tProfile not for device and no xbp fallback!\n");
		return;
	}

	size_t binding_count = 0;
	oxr_binding_find_bindings_from_act_key( //
	    log,                                // log
	    profile,                            // p
	    act_ref->act_key,                   // key
	    ARRAY_SIZE(binding_points),         // max_binding_count
	    binding_points,                     // out_bindings
	    &binding_count);                    // out_binding_count
	if (binding_count == 0) {
		oxr_slog(slog, "\t\t\tNo bindings!\n");
		return;
	}

	for (size_t i = 0; i < binding_count; i++) {
		const char *str = NULL;
		struct oxr_binding *binding_point = binding_points[i];

		XrPath matched_path = get_matched_xrpath(binding_point, act_ref);

		oxr_path_store_get_string(store, matched_path, &str, &length);
		oxr_slog(slog, "\t\t\tBinding: %s\n", str);

		if (binding_point->subaction_path != subaction_path) {
			oxr_slog(slog, "\t\t\t\tRejected! (SUB PATH)\n");
			continue;
		}

		bool found = do_io_bindings( //
		    binding_point,           //
		    act_ref,                 //
		    xdev,                    //
		    xbp,                     //
		    matched_path,            //
		    inputs,                  //
		    input_count,             //
		    outputs,                 //
		    output_count);           //

		if (found) {
			if (xbp == NULL) {
				oxr_slog(slog, "\t\t\t\tBound (xdev '%s'): %s!\n", xdev->str,
				         u_str_xrt_input_name(binding_points[i]->input));
			} else {
				oxr_slog(slog, "\t\t\t\tBound (xbp)!\n");
			}
			continue;
		}

		if (xbp == NULL) {
			oxr_slog(slog, "\t\t\t\tRejected! (NO XDEV NAME)\n");
		} else {
			oxr_slog(slog, "\t\t\t\tRejected! (NO XBINDING)\n");
		}
	}
}

static bool
get_by_name(const struct oxr_interaction_profile_array *array,
            const struct oxr_instance_path_cache *cache,
            enum xrt_device_name name,
            struct oxr_interaction_profile **out_p)
{
	return oxr_interaction_profile_array_find_by_device_name(array, cache, name, out_p);
}

static bool
get_by_device(const struct oxr_interaction_profile_array *array,
              const struct oxr_instance_path_cache *cache,
              struct xrt_device *xdev,
              struct oxr_interaction_profile **out_p)
{
	return oxr_interaction_profile_array_find_by_device(array, cache, xdev, out_p);
}

static void
find_profiles_from_roles(const struct oxr_instance_path_cache *cache,
                         const struct oxr_interaction_profile_array *array,
                         const struct oxr_roles *roles,
                         struct oxr_profiles_per_subaction *out_profiles)
{
#define FIND_PROFILE(X)                                                                                                \
	if (!get_by_name(array, cache, GET_PROFILE_NAME_BY_ROLE(roles, X), &out_profiles->X)) {                        \
		struct xrt_device *xdev = GET_XDEV_BY_ROLE(roles, X);                                                  \
		if (xdev != NULL) {                                                                                    \
			get_by_device(array, cache, xdev, &out_profiles->X);                                           \
		}                                                                                                      \
	}

	OXR_FOR_EACH_VALID_SUBACTION_PATH(FIND_PROFILE)
#undef FIND_PROFILE
}

/*!
 * @public @memberof oxr_action_attachment
 */
static XrResult
oxr_action_attachment_bind(struct oxr_logger *log,
                           struct oxr_instance *inst,
                           struct oxr_action_attachment *act_attached,
                           const struct oxr_roles *roles,
                           const struct oxr_profiles_per_subaction *profiles)
{
	struct oxr_sink_logger slog = {0};
	const struct oxr_action_ref *act_ref = act_attached->act_ref;
	const uint32_t act_set_key = act_attached->act_set_attached->act_set_key;

	// Start logging into a single buffer.
	oxr_slog(&slog, ": Binding %s/%s\n", act_attached->act_set_attached->act_set_ref->name, act_ref->name);

	if (act_ref->subaction_paths.user || act_ref->subaction_paths.any) {
#if 0
		oxr_action_bind_io(log, &slog, sess, roles, act_ref, &act_attached->user,
		                   user, OXR_SUB_ACTION_PATH_USER);
#endif
	}

#define BIND_SUBACTION(NAME, NAME_CAPS, PATH, LOCALIZED_NAME)                                                          \
	if (act_ref->subaction_paths.NAME || act_ref->subaction_paths.any) {                                           \
		oxr_action_bind_io(log, &slog, &inst->path_store, roles, act_ref, act_set_key, &act_attached->NAME,    \
		                   profiles->NAME, OXR_SUB_ACTION_PATH_##NAME_CAPS);                                   \
	}
	OXR_FOR_EACH_VALID_SUBACTION_PATH_DETAILED(BIND_SUBACTION)
#undef BIND_SUBACTION


	/*!
	 * The any sub path is special cased for poses, it binds to one sub path
	 * and sticks with it.
	 */
	if (act_ref->action_type == XR_ACTION_TYPE_POSE_INPUT) {

#define RESET_ANY(NAME) act_attached->any_pose_subaction_path.NAME = false;
		OXR_FOR_EACH_VALID_SUBACTION_PATH(RESET_ANY)
#undef RESET_ANY

#define POSE_ANY(NAME)                                                                                                 \
	if ((act_ref->subaction_paths.NAME || act_ref->subaction_paths.any) && act_attached->NAME.input_count > 0) {   \
		act_attached->any_pose_subaction_path.NAME = true;                                                     \
		oxr_slog(&slog, "\tFor: <any>\n\t\tBinding any pose to " #NAME ".\n");                                 \
	} else
		OXR_FOR_EACH_VALID_SUBACTION_PATH(POSE_ANY)
#undef POSE_ANY

		{
			oxr_slog(&slog,
			         "\tFor: <any>\n\t\tNo active sub paths for "
			         "the any pose!\n");
		}
	}

	oxr_slog(&slog, "\tDone");

	// Also frees all data.
	if (inst->debug_bindings) {
		oxr_log_slog(log, &slog);
	} else {
		oxr_slog_cancel(&slog);
	}

	return XR_SUCCESS;
}

void
oxr_action_cache_stop_output(struct oxr_logger *log, struct oxr_session *sess, struct oxr_action_cache *cache)
{
	// Set this as stopped.
	cache->stop_output_time = 0;

	struct xrt_output_value value = {.type = XRT_OUTPUT_VALUE_TYPE_VIBRATION, .vibration = {0}};

	for (uint32_t i = 0; i < cache->output_count; i++) {
		struct oxr_action_output *output = &cache->outputs[i];
		struct xrt_device *xdev = output->xdev;

		xrt_result_t xret = xrt_device_set_output(xdev, output->name, &value);
		if (xret != XRT_SUCCESS) {
			struct oxr_sink_logger slog = {0};
			oxr_slog(&slog, "Failed to stop output ");
			u_pp_xrt_output_name(oxr_slog_dg(&slog), output->name);
			oxr_log_slog(log, &slog);
		}
	}
}

static bool
oxr_input_is_input_for_cache(struct oxr_action_input *action_input, struct oxr_action_cache *cache)
{
	for (size_t i = 0; i < cache->input_count; i++) {
		if (action_input->bound_path == cache->inputs[i].bound_path) {
			return true;
		}
	}
	return false;
}

static bool
oxr_input_is_bound_in_act_set(struct oxr_action_input *action_input, struct oxr_action_set_attachment *act_set_attached)
{
	for (size_t i = 0; i < act_set_attached->action_attachment_count; i++) {
		struct oxr_action_attachment *act_attached = &act_set_attached->act_attachments[i];

#define ACCUMULATE_PATHS(X)                                                                                            \
	if (oxr_input_is_input_for_cache(action_input, &act_attached->X)) {                                            \
		return true;                                                                                           \
	}
		OXR_FOR_EACH_SUBACTION_PATH(ACCUMULATE_PATHS)
#undef ACCUMULATE_PATHS
	}
	return false;
}

static uint32_t
oxr_get_action_set_priority(const struct oxr_action_set_ref *act_set_ref,
                            const XrActiveActionSetPrioritiesEXT *active_priorities)
{
	if (active_priorities) {
		for (uint32_t i = 0; i < active_priorities->actionSetPriorityCount; i++) {
			XrActiveActionSetPriorityEXT act_set_priority = active_priorities->actionSetPriorities[i];

			struct oxr_action_set *action_set =
			    XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_action_set *, act_set_priority.actionSet);

			if (action_set->data == act_set_ref) {
				return act_set_priority.priorityOverride;
			}
		}
	}

	return act_set_ref->priority;
}

static bool
oxr_input_supressed(struct oxr_session *sess,
                    uint32_t countActionSets,
                    const XrActiveActionSet *actionSets,
                    struct oxr_subaction_paths *subaction_path,
                    struct oxr_action_attachment *act_attached,
                    struct oxr_action_input *action_input,
                    const XrActiveActionSetPrioritiesEXT *active_priorities)
{
	struct oxr_action_set_ref *act_set_ref = act_attached->act_set_attached->act_set_ref;
	uint32_t priority = oxr_get_action_set_priority(act_set_ref, active_priorities);

	// find sources that are bound to an action in a set with higher prio
	for (uint32_t i = 0; i < countActionSets; i++) {
		XrActionSet set = actionSets[i].actionSet;

		struct oxr_action_set *other_act_set = NULL;
		struct oxr_action_set_attachment *other_act_set_attached = NULL;
		get_action_set_attachment(&sess->action_context, set, &other_act_set_attached, &other_act_set);

		if (other_act_set_attached == NULL) {
			continue;
		}

		/* skip the action set that the current action is in */
		if (other_act_set_attached->act_set_ref == act_set_ref) {
			continue;
		}

		/* input may be suppressed by action set with higher prio */
		if (oxr_get_action_set_priority(other_act_set_attached->act_set_ref, active_priorities) <= priority) {
			continue;
		}

		/* Currently updated input source with subactionpath X can be
		 * suppressed, if input source also occurs in action set with
		 * higher priority if
		 * - high prio set syncs w/ ANY subactionpath or
		 * - high prio set syncs w/ subactionpath matching this input
		 *   subactionpath
		 */
		bool relevant_subactionpath = other_act_set_attached->requested_subaction_paths.any;

#define ACCUMULATE_PATHS(X)                                                                                            \
	relevant_subactionpath |= (other_act_set_attached->requested_subaction_paths.X && subaction_path->X);
		OXR_FOR_EACH_SUBACTION_PATH(ACCUMULATE_PATHS)
#undef ACCUMULATE_PATHS

		if (!relevant_subactionpath) {
			continue;
		}

		if (oxr_input_is_bound_in_act_set(action_input, other_act_set_attached)) {
			return true;
		}
	}

	return false;
}

static bool
oxr_input_combine_input(struct oxr_session *sess,
                        uint32_t countActionSets,
                        const XrActiveActionSet *actionSets,
                        struct oxr_action_attachment *act_attached,
                        struct oxr_subaction_paths *subaction_path,
                        struct oxr_action_cache *cache,
                        struct oxr_input_value_tagged *out_input,
                        int64_t *out_timestamp,
                        bool *out_is_active,
                        const XrActiveActionSetPrioritiesEXT *active_priorities)
{
	struct oxr_action_input *inputs = cache->inputs;
	size_t input_count = cache->input_count;

	if (input_count == 0) {
		*out_is_active = false;
		return true;
	}

	bool any_active = false;
	struct oxr_input_value_tagged res = {0};
	int64_t res_timestamp = inputs[0].input->timestamp;

	for (size_t i = 0; i < input_count; i++) {
		struct oxr_action_input *action_input = &(inputs[i]);
		struct xrt_input *input = action_input->input;

		// suppress input if it is also bound to action in set with
		// higher priority
		if (oxr_input_supressed(sess, countActionSets, actionSets, subaction_path, act_attached, action_input,
		                        active_priorities)) {
			continue;
		}

		if (input->active) {
			any_active = true;
		} else {
			continue;
		}

		struct oxr_input_value_tagged raw_input = {
		    .type = XRT_GET_INPUT_TYPE(input->name),
		    .value = input->value,
		};

		struct oxr_input_value_tagged transformed = {0};
		if (!oxr_input_transform_process(action_input->transforms, action_input->transform_count, &raw_input,
		                                 &transformed)) {
			// We couldn't transform, how strange. Reset all state.
			// At this level we don't know what action this is, etc.
			// so a warning message isn't very helpful.
			return false;
		}

		// at this stage type should be "compatible" to action
		res.type = transformed.type;

		switch (transformed.type) {
		case XRT_INPUT_TYPE_BOOLEAN:
			res.value.boolean |= transformed.value.boolean;

			/* Special case bool: all bool inputs are combined with
			 * OR. The action only changes to true on the earliest
			 * input that sets it to true, and to false on the
			 * latest input that is false. */
			if (res.value.boolean && transformed.value.boolean && input->timestamp < res_timestamp) {
				res_timestamp = input->timestamp;
			} else if (!res.value.boolean && !transformed.value.boolean &&
			           input->timestamp > res_timestamp) {
				res_timestamp = input->timestamp;
			}
			break;
		case XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE:
		case XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE:
			if (fabsf(transformed.value.vec1.x) > fabsf(res.value.vec1.x)) {
				res.value.vec1.x = transformed.value.vec1.x;
				res_timestamp = input->timestamp;
			}
			break;
		case XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE: {
			float res_sq = res.value.vec2.x * res.value.vec2.x + res.value.vec2.y * res.value.vec2.y;
			float trans_sq = transformed.value.vec2.x * transformed.value.vec2.x +
			                 transformed.value.vec2.y * transformed.value.vec2.y;
			if (trans_sq > res_sq) {
				res.value.vec2 = transformed.value.vec2;
				res_timestamp = input->timestamp;
			}
		} break;
		case XRT_INPUT_TYPE_VEC3_MINUS_ONE_TO_ONE:
			// OpenXR has no vec3 right now.
			break;
		case XRT_INPUT_TYPE_POSE:
			// shouldn't be possible to get here
			break;
		case XRT_INPUT_TYPE_HAND_TRACKING:
			// shouldn't be possible to get here
			break;
		case XRT_INPUT_TYPE_FACE_TRACKING:
			// shouldn't be possible to get here
			break;
		case XRT_INPUT_TYPE_BODY_TRACKING:
			// shouldn't be possible to get here
			break;
		}
	}

	*out_is_active = any_active;
	*out_input = res;
	*out_timestamp = res_timestamp;

	return true;
}

/*!
 * Called during xrSyncActions.
 *
 * @private @memberof oxr_action_cache
 */
static void
oxr_action_cache_update(struct oxr_logger *log,
                        struct oxr_session *sess,
                        uint32_t countActionSets,
                        const XrActiveActionSet *actionSets,
                        struct oxr_action_attachment *act_attached,
                        struct oxr_action_cache *cache,
                        int64_t time,
                        struct oxr_subaction_paths *subaction_path,
                        bool selected,
                        const XrActiveActionSetPrioritiesEXT *active_priorities)
{
	struct oxr_action_state last = cache->current;

	if (!selected) {
		if (cache->stop_output_time > 0) {
			oxr_action_cache_stop_output(log, sess, cache);
		}
		U_ZERO(&cache->current);
		return;
	}

	struct oxr_input_value_tagged combined;
	int64_t timestamp = time;

	/* a cache can only have outputs or inputs, not both */
	if (cache->output_count > 0) {
		cache->current.active = true;
		if (cache->stop_output_time > 0 && cache->stop_output_time < time) {
			oxr_action_cache_stop_output(log, sess, cache);
		}
	} else if (cache->input_count > 0) {

		bool is_active = false;
		bool bret = oxr_input_combine_input( //
		    sess,                            // sess
		    countActionSets,                 // countActionSets
		    actionSets,                      // actionSets
		    act_attached,                    // act_attached
		    subaction_path,                  // subaction_path
		    cache,                           // cache
		    &combined,                       // out_input
		    &timestamp,                      // out_timestamp
		    &is_active,                      // out_is_active
		    active_priorities);              // active_priorities
		if (!bret) {
			oxr_log(log, "Failed to get/combine input values '%s'", act_attached->act_ref->name);
			return;
		}

		bool is_focused = sess->state == XR_SESSION_STATE_FOCUSED;

		// If the input is not active signal or the session state is not in focused that.
		if (!is_focused || !is_active) {
			// Reset all state.
			U_ZERO(&cache->current);
			return;
		}

		// Signal that the input is active, always set just to be sure.
		cache->current.active = true;

		bool changed = false;
		switch (combined.type) {
		case XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE:
		case XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE: {
			changed = (combined.value.vec1.x != last.value.vec1.x);
			cache->current.value.vec1.x = combined.value.vec1.x;
			break;
		}
		case XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE: {
			changed = (combined.value.vec2.x != last.value.vec2.x) ||
			          (combined.value.vec2.y != last.value.vec2.y);
			cache->current.value.vec2.x = combined.value.vec2.x;
			cache->current.value.vec2.y = combined.value.vec2.y;
			break;
		}
#if 0
		// Untested, we have no VEC3 input.
		case XRT_INPUT_TYPE_VEC3_MINUS_ONE_TO_ONE: {
			changed = (combined.value.vec3.x != last.vec3.x) ||
			          (combined.value.vec3.y != last.vec3.y) ||
			          (combined.value.vec3.z != last.vec3.z);
			cache->current.vec3.x = combined.value.vec3.x;
			cache->current.vec3.y = combined.value.vec3.y;
			cache->current.vec3.z = combined.value.vec3.z;
			break;
		}
#endif
		case XRT_INPUT_TYPE_BOOLEAN: {
			changed = (combined.value.boolean != last.value.boolean);
			cache->current.value.boolean = combined.value.boolean;
			break;
		}
		case XRT_INPUT_TYPE_POSE: return;
		default:
			// Should not end up here.
			assert(false);
		}

		if (last.active && changed) {
			// We were active last sync, and we've changed since
			// then
			cache->current.timestamp = timestamp;
			cache->current.changed = true;
		} else if (last.active) {
			// We were active last sync, but we haven't changed
			// since then.
			cache->current.timestamp = last.timestamp;
			cache->current.changed = false;
		} else {
			// We are active now but weren't active last time.
			cache->current.timestamp = timestamp;
			cache->current.changed = false;
		}
	}
}

static inline bool
oxr_state_equal_bool(const struct oxr_action_state *a, const struct oxr_action_state *b)
{
	return a->value.boolean == b->value.boolean;
}

static inline bool
oxr_state_equal_vec1(const struct oxr_action_state *a, const struct oxr_action_state *b)
{
	return a->value.vec1.x == b->value.vec1.x;
}

static inline bool
oxr_state_equal_vec2(const struct oxr_action_state *a, const struct oxr_action_state *b)
{
	return (a->value.vec2.x == b->value.vec2.x) && (a->value.vec2.y == b->value.vec2.y);
}

static inline void
oxr_state_update_bool(bool *active, bool *value, XrTime *timestamp, const struct oxr_action_state *new_state)
{
	if (new_state->active) {
		*active |= true;
		*value |= new_state->value.boolean;
		*timestamp = new_state->timestamp;
	}
}
#define BOOL_CHECK(NAME) oxr_state_update_bool(&active, &value, &timestamp, &act_attached->NAME.current);

static inline void
oxr_state_update_vec1(bool *active, float *value, XrTime *timestamp, const struct oxr_action_state *new_state)
{
	if (new_state->active) {
		*active |= true;
		if (*value < new_state->value.vec1.x) {
			*value = new_state->value.vec1.x;
			*timestamp = new_state->timestamp;
		}
	}
}
#define VEC1_CHECK(NAME) oxr_state_update_vec1(&active, &value, &timestamp, &act_attached->NAME.current);

static inline void
oxr_state_update_vec2(
    bool *active, float *x, float *y, float *distance, XrTime *timestamp, const struct oxr_action_state *new_state)
{
	if (new_state->active) {
		*active |= true;
		float curr_x = new_state->value.vec2.x;
		float curr_y = new_state->value.vec2.y;
		float curr_d = curr_x * curr_x + curr_y * curr_y;
		if (*distance < curr_d) {
			*x = curr_x;
			*y = curr_y;
			*distance = curr_d;
			*timestamp = new_state->timestamp;
		}
	}
}
#define VEC2_CHECK(NAME) oxr_state_update_vec2(&active, &x, &y, &distance, &timestamp, &act_attached->NAME.current);

/*!
 * Called during each xrSyncActions.
 *
 * @private @memberof oxr_action_attachment
 */
static void
oxr_action_attachment_update(struct oxr_logger *log,
                             struct oxr_session *sess,
                             uint32_t countActionSets,
                             const XrActiveActionSet *actionSets,
                             struct oxr_action_attachment *act_attached,
                             int64_t time,
                             struct oxr_subaction_paths subaction_paths,
                             const XrActiveActionSetPrioritiesEXT *active_priorities)
{
	// This really shouldn't be happening.
	if (act_attached == NULL) {
		return;
	}

#define UPDATE_SELECT(X)                                                                                               \
	struct oxr_subaction_paths subaction_paths_##X = {0};                                                          \
	subaction_paths_##X.X = true;                                                                                  \
	bool select_##X = subaction_paths.X || subaction_paths.any;                                                    \
	oxr_action_cache_update(log, sess, countActionSets, actionSets, act_attached, &act_attached->X, time,          \
	                        &subaction_paths_##X, select_##X, active_priorities);

	OXR_FOR_EACH_VALID_SUBACTION_PATH(UPDATE_SELECT)
#undef UPDATE_SELECT

	/*
	 * Any state.
	 */
	struct oxr_action_state last = act_attached->any_state;
	bool active = false;
	bool changed = false;
	XrTime timestamp = time_state_monotonic_to_ts_ns(sess->sys->inst->timekeeping, time);
	U_ZERO(&act_attached->any_state);

	switch (act_attached->act_ref->action_type) {
	case XR_ACTION_TYPE_BOOLEAN_INPUT: {
		bool value = false;
		OXR_FOR_EACH_VALID_SUBACTION_PATH(BOOL_CHECK)

		act_attached->any_state.value.boolean = value;
		changed = active && !oxr_state_equal_bool(&last, &act_attached->any_state);
		break;
	}
	case XR_ACTION_TYPE_FLOAT_INPUT: {
		// Smaller than any possible real value
		float value = -2.0f; // NOLINT
		OXR_FOR_EACH_VALID_SUBACTION_PATH(VEC1_CHECK)

		act_attached->any_state.value.vec1.x = value;
		changed = active && !oxr_state_equal_vec1(&last, &act_attached->any_state);
		break;
	}
	case XR_ACTION_TYPE_VECTOR2F_INPUT: {
		float x = 0.0f;
		float y = 0.0f;
		float distance = -1.0f;
		OXR_FOR_EACH_VALID_SUBACTION_PATH(VEC2_CHECK)

		act_attached->any_state.value.vec2.x = x;
		act_attached->any_state.value.vec2.y = y;
		changed = active && !oxr_state_equal_vec2(&last, &act_attached->any_state);
		break;
	}
	default:
	case XR_ACTION_TYPE_POSE_INPUT:
	case XR_ACTION_TYPE_VIBRATION_OUTPUT:
		// Nothing to do
		//! @todo You sure?
		return;
	}

	act_attached->any_state.active = active;
	// We're only changed if the value differs and we're not newly
	// active.
	act_attached->any_state.changed = last.active && changed;
	if (active) {
		act_attached->any_state.timestamp = timestamp;
		if (!act_attached->any_state.changed && last.active) {
			// Use old timestamp if we're unchanged, but were active
			// last sync
			act_attached->any_state.timestamp = last.timestamp;
		}
	}
}

/*!
 * Try to produce a transform chain to convert the available input into
 * the desired input type.
 *
 * Populates @p action_input->transforms and @p action_input->transform_count on
 * success.
 *
 * @returns false if it could not, true if it could
 */
static bool
oxr_action_populate_input_transform(struct oxr_logger *log,
                                    struct oxr_sink_logger *slog,
                                    const struct oxr_path_store *store,
                                    const struct oxr_action_ref *act_ref,
                                    struct oxr_action_input *action_input)
{
	assert(action_input->transforms == NULL);
	assert(action_input->transform_count == 0);
	const char *str;
	size_t length;
	oxr_path_store_get_string(store, action_input->bound_path, &str, &length);

	enum xrt_input_type t = XRT_GET_INPUT_TYPE(action_input->input->name);

	return oxr_input_transform_create_chain( //
	    log,                                 //
	    slog,                                //
	    t,                                   //
	    act_ref->action_type,                //
	    act_ref->name,                       //
	    str,                                 //
	    &action_input->transforms,           //
	    &action_input->transform_count);     //
}

/*!
 * Find dpad settings in @p dpad_entry whose binding path
 * is a prefix of @p bound_path_string.
 *
 * @returns true if settings were found and written to @p out_dpad_settings
 */
static bool
find_matching_dpad(const struct oxr_path_store *store,
                   const struct oxr_dpad_entry *dpad_entry,
                   const char *bound_path_string,
                   const struct oxr_dpad_binding_modification **out_dpad_binding)
{
	if (dpad_entry != NULL) {
		for (uint32_t i = 0; i < dpad_entry->dpad_count; i++) {
			const char *dpad_path_string;
			size_t dpad_path_length;
			oxr_path_store_get_string(store, dpad_entry->dpads[i].binding, &dpad_path_string,
			                          &dpad_path_length);
			if (strncmp(bound_path_string, dpad_path_string, dpad_path_length) == 0) {
				*out_dpad_binding = &dpad_entry->dpads[i];
				return true;
			}
		}
	}
	return false;
}

/*!
 * Try to produce a transform chain to create a dpad button from the selected
 * input (potentially using other inputs like `/force` in the process).
 *
 * Populates @p action_input->transforms and @p action_input->transform_count on
 * success.
 *
 * @returns false if it could not, true if it could
 */
static bool
oxr_action_populate_input_transform_dpad(struct oxr_logger *log,
                                         struct oxr_sink_logger *slog,
                                         const struct oxr_path_store *store,
                                         const struct oxr_action_ref *act_ref,
                                         const struct oxr_dpad_entry *dpad_entry,
                                         enum oxr_dpad_region dpad_region,
                                         struct oxr_interaction_profile *profile,
                                         struct oxr_action_input *action_inputs,
                                         uint32_t action_input_count,
                                         uint32_t selected_input)
{
	struct oxr_action_input *action_input = &(action_inputs[selected_input]);
	assert(action_input->transforms == NULL);
	assert(action_input->transform_count == 0);

	const char *bound_path_string;
	size_t bound_path_length;
	oxr_path_store_get_string(store, action_input->bound_path, &bound_path_string, &bound_path_length);

	// find correct dpad entry
	const struct oxr_dpad_binding_modification *dpad_binding_modification = NULL;
	find_matching_dpad(store, dpad_entry, bound_path_string, &dpad_binding_modification);

	enum xrt_input_type t = XRT_GET_INPUT_TYPE(action_input->input->name);
	enum xrt_input_type activate_t = XRT_GET_INPUT_TYPE(action_input->dpad_activate_name);

	return oxr_input_transform_create_chain_dpad( //
	    log,                                      //
	    slog,                                     //
	    t,                                        //
	    act_ref->action_type,                     //
	    bound_path_string,                        //
	    dpad_binding_modification,                //
	    dpad_region,                              //
	    activate_t,                               //
	    action_input->dpad_activate,              //
	    &action_input->transforms,                //
	    &action_input->transform_count);          //
}

static bool
is_dpad_region_for_emulation(const char *start, const char *end)
{
	// go before the first slash
	end--;

	while (end > start) {
		char curr = *end;

		// once we find a slash,
		if (curr == '/') {
			const char *to_check[] = {"/thumbstick_", "/thumbstick/", "/trackpad_", "/trackpad/"};

			// check if the passed path is a sub-path of "thumbstick[_|/]" or "trackpad[_|/]"
			for (size_t i = 0; i < ARRAY_SIZE(to_check); i++) {
				if (strncmp(end, to_check[i], strlen(to_check[i])) == 0) {
					// this is for emulation
					return true;
				}
			}

			// it's not for emulation and is an actual dpad region
			return false;
		}

		end--;
	}

	return false;
}

// based on get_subaction_path_from_path
static bool
get_dpad_region_from_path(const struct oxr_path_store *store, XrPath path, enum oxr_dpad_region *out_dpad_region)
{
	const char *str = NULL;
	size_t length = 0;
	XrResult ret;

	ret = oxr_path_store_get_string(store, path, &str, &length);
	if (ret != XR_SUCCESS) {
		return false;
	}

	// TODO: surely there's a better way to do this?
	if (length >= 10 && strncmp("/dpad_left", str + (length - 10), 10) == 0 &&
	    is_dpad_region_for_emulation(str, str + (length - 10))) {
		*out_dpad_region = OXR_DPAD_REGION_LEFT;
		return true;
	}
	if (length >= 11 && strncmp("/dpad_right", str + (length - 11), 11) == 0 &&
	    is_dpad_region_for_emulation(str, str + (length - 11))) {
		*out_dpad_region = OXR_DPAD_REGION_RIGHT;
		return true;
	}
	if (length >= 8 && strncmp("/dpad_up", str + (length - 8), 8) == 0 &&
	    is_dpad_region_for_emulation(str, str + (length - 8))) {
		*out_dpad_region = OXR_DPAD_REGION_UP;
		return true;
	}
	if (length >= 10 && strncmp("/dpad_down", str + (length - 10), 10) == 0 &&
	    is_dpad_region_for_emulation(str, str + (length - 10))) {
		*out_dpad_region = OXR_DPAD_REGION_DOWN;
		return true;
	}
	if (length >= 12 && strncmp("/dpad_center", str + (length - 12), 12) == 0 &&
	    is_dpad_region_for_emulation(str, str + (length - 12))) {
		*out_dpad_region = OXR_DPAD_REGION_CENTER;
		return true;
	}

	return false;
}

static void
oxr_action_bind_io(struct oxr_logger *log,
                   struct oxr_sink_logger *slog,
                   const struct oxr_path_store *store,
                   const struct oxr_roles *roles,
                   const struct oxr_action_ref *act_ref,
                   const uint32_t act_set_key,
                   struct oxr_action_cache *cache,
                   struct oxr_interaction_profile *profile,
                   enum oxr_subaction_path subaction_path)
{
	struct oxr_action_input inputs[OXR_MAX_BINDINGS_PER_ACTION] = {0};
	uint32_t input_count = 0;
	struct oxr_action_output outputs[OXR_MAX_BINDINGS_PER_ACTION] = {0};
	uint32_t output_count = 0;

	// If we are binding again, reset the cache fully.
	oxr_action_cache_teardown(cache);

	// Fill out the arrays with the bindings we can find.
	get_binding(        //
	    log,            // log
	    slog,           // slog
	    store,          // store
	    roles,          // roles
	    act_ref,        // act_ref
	    profile,        // profile
	    subaction_path, // subaction_path
	    inputs,         // inputs
	    &input_count,   // input_count
	    outputs,        // outputs
	    &output_count); // output_count

	// Mutually exclusive to outputs.
	if (input_count > 0) {
		uint32_t count = 0;
		cache->current.active = true;
		cache->inputs = U_TYPED_ARRAY_CALLOC(struct oxr_action_input, input_count);
		for (uint32_t i = 0; i < input_count; i++) {
			// Only add the input if we can find a transform.

			oxr_slog(slog, "\t\tFinding transforms for '%s' to action '%s' of type '%s'\n",
			         u_str_xrt_input_name(inputs[i].input->name), act_ref->name,
			         xr_action_type_to_str(act_ref->action_type));

			enum oxr_dpad_region dpad_region;
			if (get_dpad_region_from_path(store, inputs[i].bound_path, &dpad_region)) {
				const struct oxr_dpad_entry *entry =
				    oxr_dpad_state_get(&profile->dpad_state, act_set_key);

				bool bret = oxr_action_populate_input_transform_dpad( //
				    log,                                              //
				    slog,                                             //
				    store,                                            //
				    act_ref,                                          //
				    entry,                                            //
				    dpad_region,                                      //
				    profile,                                          //
				    inputs,                                           //
				    input_count,                                      //
				    i);                                               //
				if (bret) {
					cache->inputs[count++] = inputs[i];
					continue;
				}
			} else if (oxr_action_populate_input_transform(log, slog, store, act_ref, &(inputs[i]))) {
				cache->inputs[count++] = inputs[i];
				continue;
			}

			oxr_slog(slog, "\t\t\t\tRejected! (NO TRANSFORM)\n");
		}


		// No inputs found, prented we never bound it.
		if (count == 0) {
			free(cache->inputs);
			cache->inputs = NULL;
		} else {
			oxr_slog(slog, "\t\tBound to:\n");
			for (uint32_t i = 0; i < count; i++) {
				struct xrt_input *input = cache->inputs[i].input;
				enum xrt_input_type t = XRT_GET_INPUT_TYPE(input->name);
				bool active = input->active;
				oxr_slog(slog, "\t\t\t'%s' ('%s') on '%s' (%s)\n", u_str_xrt_input_name(input->name),
				         xrt_input_type_to_str(t), cache->inputs[i].xdev->str,
				         active ? "active" : "inactive");
			}
		}

		cache->input_count = count;
	}

	// Mutually exclusive to inputs.
	if (output_count > 0) {
		cache->current.active = true;
		cache->outputs = U_TYPED_ARRAY_CALLOC(struct oxr_action_output, output_count);
		for (uint32_t i = 0; i < output_count; i++) {
			cache->outputs[i] = outputs[i];
		}
		cache->output_count = output_count;
	}
}


/*
 *
 * Session functions.
 *
 */

XrResult
oxr_session_attach_action_sets(struct oxr_logger *log,
                               const struct oxr_interaction_profile_array *suggested_profiles,
                               struct oxr_session_attached_actions *attached_actions,
                               struct oxr_session_action_context *sess_context,
                               const XrSessionActionSetsAttachInfo *bindInfo)
{
	XrResult ret;

	size_t count = bindInfo->countActionSets;

	sess_context->action_set_attachment_count = count;
	sess_context->act_set_attachments = U_TYPED_ARRAY_CALLOC(struct oxr_action_set_attachment, count);
	if (sess_context->act_set_attachments == NULL) {
		// Actual clean-up.
		sess_context->action_set_attachment_count = 0;
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Failed to allocate action set attachments");
	}

	for (size_t i = 0; i < count; i++) {
		struct oxr_action_set *act_set =
		    XRT_CAST_OXR_HANDLE_TO_PTR(struct oxr_action_set *, bindInfo->actionSets[i]);
		assert(act_set != NULL);

		struct oxr_action_set_ref *act_set_ref = act_set->data;
		act_set_ref->ever_attached = true;

		struct oxr_action_set_attachment *act_set_attached = &sess_context->act_set_attachments[i];
		ret = oxr_action_set_attachment_init( //
		    log,                              //
		    act_set_attached,                 //
		    sess_context,                     //
		    act_set->data);                   //
		if (ret != XR_SUCCESS) {
			// @todo: unwind the loop and free the attachments.
			assert(false);
			return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "oxr_action_set_attachment_init failed");
		}

		// Allocate the action attachments for this set.
		act_set_attached->action_attachment_count = act_set->handle.children.count;
		act_set_attached->act_attachments =
		    U_TYPED_ARRAY_CALLOC(struct oxr_action_attachment, act_set_attached->action_attachment_count);
		if (act_set_attached->act_attachments == NULL) {
			// @todo: unwind the loop and free the attachments.
			assert(false);
			return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Failed to allocate action attachments");
		}

		// Set up the per-session data for the actions.
		uint32_t child_index = 0;
		for (uint32_t k = 0; k < act_set->handle.children.count; k++) {
			struct oxr_action *act = (struct oxr_action *)act_set->handle.children.handles[k];
			if (act == NULL) {
				continue;
			}

			struct oxr_action_attachment *act_attached = &act_set_attached->act_attachments[child_index];
			ret = oxr_action_attachment_init( //
			    log,                          //
			    act_attached,                 //
			    attached_actions,             //
			    act_set_attached,             //
			    act->data);                   //
			if (ret != XR_SUCCESS) {
				// @todo: unwind the loop and free the attachments.
				assert(false);
				return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "oxr_action_attachment_init failed");
			}

			++child_index;
		}
	}

	oxr_interaction_profile_array_clone(suggested_profiles, &sess_context->profiles_on_attachment);

	/*
	 * We used to send XrEventDataInteractionProfileChanged here, but that's
	 * wrong. The OpenXR spec says we should only send them after a
	 * successful call to xrSyncActionData.
	 */

	return XR_SUCCESS;
}

static XrResult
session_update_action_bindings(struct oxr_logger *log,
                               struct oxr_instance *inst,
                               struct oxr_session_action_context *action_context,
                               const struct oxr_roles *roles,
                               bool *out_interaction_profile_changed)
{
	// Convenience.
	const struct oxr_path_store *store = &inst->path_store;
	const struct oxr_instance_path_cache *cache = &inst->path_cache;
	const struct oxr_interaction_profile_array *array = &action_context->profiles_on_attachment;

	struct oxr_profiles_per_subaction profiles = {0};
	find_profiles_from_roles(cache, array, roles, &profiles);

	// Dump selected profiles if debugging.
	if (inst->debug_bindings) {
		print_profiles(log, store, &profiles);
	}

	for (size_t i = 0; i < action_context->action_set_attachment_count; i++) {
		struct oxr_action_set_attachment *act_set_attached = &action_context->act_set_attachments[i];
		for (size_t k = 0; k < act_set_attached->action_attachment_count; k++) {
			struct oxr_action_attachment *act_attached = &act_set_attached->act_attachments[k];
			oxr_action_attachment_bind(log, inst, act_attached, roles, &profiles);
		}
	}

	bool interaction_profile_changed = false;

	/*
	 * This code will only send events (and update the bindings) if there
	 * is a profile mapped to the subaction path. Meaning it won't update
	 * the cache or generate events when a controller goes away, basically
	 * latching the interaction profile to the last active device.
	 */
#define POPULATE_PROFILE(X)                                                                                            \
	do {                                                                                                           \
		XrPath path = profiles.X != NULL ? profiles.X->path : XR_NULL_PATH;                                    \
		if (path == XR_NULL_PATH) {                                                                            \
			break; /* Only update on "active" interaction profiles per sub-action path. */                 \
		}                                                                                                      \
		if (action_context->X != path) {                                                                       \
			action_context->X = path;                                                                      \
			interaction_profile_changed = true;                                                            \
		}                                                                                                      \
	} while (false);
	OXR_FOR_EACH_VALID_SUBACTION_PATH(POPULATE_PROFILE)
#undef POPULATE_PROFILE

	*out_interaction_profile_changed = interaction_profile_changed;

	return XR_SUCCESS;
}

XrResult
oxr_action_sync_data(struct oxr_logger *log,
                     struct oxr_session *sess,
                     uint32_t countActionSets,
                     const XrActiveActionSet *actionSets,
                     const XrActiveActionSetPrioritiesEXT *activePriorities)
{
	bool interaction_profile_changed = false;
	XrResult ret = oxr_action_sync_data_with_context( //
	    log,                                          //
	    sess,                                         //
	    &sess->action_context,                        //
	    countActionSets,                              //
	    actionSets,                                   //
	    activePriorities,                             //
	    &interaction_profile_changed);                //
	if (ret != XR_SUCCESS) {
		return ret;
	}

	if (interaction_profile_changed) {
		oxr_event_push_XrEventDataInteractionProfileChanged(log, sess);
	}

	return oxr_session_success_focused_result(sess);
}

XrResult
oxr_action_sync_data_with_context(struct oxr_logger *log,
                                  struct oxr_session *sess,
                                  struct oxr_session_action_context *sess_context,
                                  uint32_t countActionSets,
                                  const XrActiveActionSet *actionSets,
                                  const XrActiveActionSetPrioritiesEXT *activePriorities,
                                  bool *out_interaction_profile_changed)
{
	struct oxr_instance *inst = sess->sys->inst;

	struct oxr_action_set *act_set = NULL;
	struct oxr_action_set_attachment *act_set_attached = NULL;

	// Reset.
	*out_interaction_profile_changed = false;

	/*
	 * No side-effects allowed in this section as we are still
	 * validating and checking for errors at this point.
	 */

	// Check that all action sets has been attached.
	for (uint32_t i = 0; i < countActionSets; i++) {
		get_action_set_attachment(sess_context, actionSets[i].actionSet, &act_set_attached, &act_set);
		if (act_set_attached == NULL) {
			return oxr_error(log, XR_ERROR_ACTIONSET_NOT_ATTACHED,
			                 "(actionSets[%i].actionSet) action set '%s' has "
			                 "not been attached to this session",
			                 i, act_set != NULL ? act_set->data->name : "NULL");
		}
	}

	/*
	 * Can only call this function if the session state is focused. This is
	 * not an error and has to be checked after all validation, but before
	 * any side-effects happens.
	 */
	if (sess->state != XR_SESSION_STATE_FOCUSED) {
		return XR_SESSION_NOT_FOCUSED;
	}

	/*
	 * Side-effects allowed below, but not validation.
	 */

	struct oxr_roles roles = XRT_STRUCT_INIT;
	XrResult result = oxr_roles_init_on_stack(log, &roles, sess->sys);
	if (result != XR_SUCCESS) {
		// Only returns XR_ERROR_RUNTIME_FAILURE
		return result;
	}

	// Should we redo the bindings?
	{
		os_mutex_lock(&sess_context->sync_actions_mutex);
		if (sess_context->dynamic_roles_generation_id < roles.roles.generation_id) {
			sess_context->dynamic_roles_generation_id = roles.roles.generation_id;
			session_update_action_bindings(       //
			    log,                              //
			    inst,                             //
			    sess_context,                     //
			    &roles,                           //
			    out_interaction_profile_changed); //
		}
		os_mutex_unlock(&sess_context->sync_actions_mutex);
	}

	if (countActionSets == 0) {
		// Nothing to do.
		return XR_SUCCESS;
	}

	// Synchronize outputs to this time.
	int64_t now = time_state_get_now(sess->sys->inst->timekeeping);

	// Loop over all xdev devices.
	for (size_t i = 0; i < sess->sys->xsysd->static_xdev_count; i++) {
		if (sess->sys->xsysd->static_xdevs[i]) {
			xrt_result_t xret = xrt_device_update_inputs(sess->sys->xsysd->static_xdevs[i]);
			OXR_CHECK_XRET(log, sess, xret, xrt_device_update_inputs);
		}
	}

	// Reset all action set attachments.
	for (size_t i = 0; i < sess_context->action_set_attachment_count; ++i) {
		act_set_attached = &sess_context->act_set_attachments[i];
		U_ZERO(&act_set_attached->requested_subaction_paths);
	}

	// Go over all requested action sets and update their
	// attachment.
	//! @todo can be listed more than once with different paths!
	for (uint32_t i = 0; i < countActionSets; i++) {
		struct oxr_subaction_paths subaction_paths;
		get_action_set_attachment(sess_context, actionSets[i].actionSet, &act_set_attached, &act_set);
		assert(act_set_attached != NULL);

		if (!oxr_classify_subaction_path(&inst->path_cache, actionSets[i].subactionPath, &subaction_paths)) {
			//! @todo Probably an internal runtime error since it should be validated before?
			return oxr_error(log, XR_ERROR_PATH_UNSUPPORTED, "Bad internal codepath taken (bug?)");
		}

		/* never error when requesting any subactionpath */
		bool any_action_with_subactionpath = subaction_paths.any;

		oxr_subaction_paths_accumulate(&(act_set_attached->requested_subaction_paths), &subaction_paths);

		/* check if we have at least one action for requested subactionpath */
		for (uint32_t k = 0; k < act_set_attached->action_attachment_count; k++) {
			struct oxr_action_attachment *act_attached = &act_set_attached->act_attachments[k];

			if (act_attached == NULL) {
				continue;
			}

#define ACCUMULATE_REQUESTED(X)                                                                                        \
	any_action_with_subactionpath |= subaction_paths.X && act_attached->act_ref->subaction_paths.X;
			OXR_FOR_EACH_SUBACTION_PATH(ACCUMULATE_REQUESTED)
#undef ACCUMULATE_REQUESTED
		}

		//! @TODO This validation is not allowed here, must done above.
		if (!any_action_with_subactionpath) {
			return oxr_error(log, XR_ERROR_PATH_UNSUPPORTED,
			                 "No action with specified subactionpath in actionset");
		}
	}

	// Now, update all action attachments
	for (size_t i = 0; i < sess_context->action_set_attachment_count; ++i) {
		act_set_attached = &sess_context->act_set_attachments[i];
		struct oxr_subaction_paths subaction_paths = act_set_attached->requested_subaction_paths;

		for (uint32_t k = 0; k < act_set_attached->action_attachment_count; k++) {
			struct oxr_action_attachment *act_attached = &act_set_attached->act_attachments[k];

			if (act_attached == NULL) {
				continue;
			}

			oxr_action_attachment_update(log, sess, countActionSets, actionSets, act_attached, now,
			                             subaction_paths, activePriorities);
		}
	}

	return XR_SUCCESS;
}

static void
add_path_to_set(XrPath path_set[OXR_MAX_BINDINGS_PER_ACTION], XrPath new_path, uint32_t *inout_path_count)
{
	const uint32_t n = *inout_path_count;

	// Shouldn't be full
	assert(n < OXR_MAX_BINDINGS_PER_ACTION);

	for (uint32_t i = 0; i < n; ++i) {
		if (new_path == path_set[i]) {
			return;
		}
		// Should have no gaps
		assert(path_set[i] != 0);
	}
	path_set[n] = new_path;
	(*inout_path_count)++;
}

XrResult
oxr_action_enumerate_bound_sources(struct oxr_logger *log,
                                   struct oxr_session *sess,
                                   uint32_t act_key,
                                   uint32_t sourceCapacityInput,
                                   uint32_t *sourceCountOutput,
                                   XrPath *sources)
{
	struct oxr_action_attachment *act_attached = NULL;
	uint32_t path_count = 0;
	XrPath temp[OXR_MAX_BINDINGS_PER_ACTION] = {0};

	oxr_session_attached_actions_find(&sess->attached_actions, act_key, &act_attached);
	if (act_attached == NULL) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "act_key did not find any action");
	}

#define ACCUMULATE_PATHS(X)                                                                                            \
	if (act_attached->X.input_count > 0) {                                                                         \
		for (uint32_t i = 0; i < act_attached->X.input_count; i++) {                                           \
			add_path_to_set(temp, act_attached->X.inputs[i].bound_path, &path_count);                      \
		}                                                                                                      \
	}                                                                                                              \
	if (act_attached->X.output_count > 0) {                                                                        \
		for (uint32_t i = 0; i < act_attached->X.output_count; i++) {                                          \
			add_path_to_set(temp, act_attached->X.outputs[i].bound_path, &path_count);                     \
		}                                                                                                      \
	}

	OXR_FOR_EACH_SUBACTION_PATH(ACCUMULATE_PATHS)
#undef ACCUMULATE_PATHS

	OXR_TWO_CALL_HELPER(log, sourceCapacityInput, sourceCountOutput, sources, path_count, temp,
	                    oxr_session_success_result(sess));
}
