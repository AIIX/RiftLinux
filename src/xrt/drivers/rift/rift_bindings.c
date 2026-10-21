// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Bindings for the Rift Touch Controllers.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#include "rift_bindings.h"


static struct xrt_binding_input_pair simple_inputs_touch[] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_TOUCH_MENU_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
};

static struct xrt_binding_output_pair simple_outputs_touch[] = {
    {XRT_OUTPUT_NAME_SIMPLE_VIBRATION, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

static struct xrt_binding_input_pair touch_inputs_touch[] = {
    {XRT_INPUT_TOUCH_X_CLICK, XRT_INPUT_TOUCH_X_CLICK},
    {XRT_INPUT_TOUCH_X_TOUCH, XRT_INPUT_TOUCH_X_TOUCH},
    {XRT_INPUT_TOUCH_Y_CLICK, XRT_INPUT_TOUCH_Y_CLICK},
    {XRT_INPUT_TOUCH_Y_TOUCH, XRT_INPUT_TOUCH_Y_TOUCH},
    {XRT_INPUT_TOUCH_MENU_CLICK, XRT_INPUT_TOUCH_MENU_CLICK},
    {XRT_INPUT_TOUCH_A_CLICK, XRT_INPUT_TOUCH_A_CLICK},
    {XRT_INPUT_TOUCH_A_TOUCH, XRT_INPUT_TOUCH_A_TOUCH},
    {XRT_INPUT_TOUCH_B_CLICK, XRT_INPUT_TOUCH_B_CLICK},
    {XRT_INPUT_TOUCH_B_TOUCH, XRT_INPUT_TOUCH_B_TOUCH},
    {XRT_INPUT_TOUCH_SYSTEM_CLICK, XRT_INPUT_TOUCH_SYSTEM_CLICK},
    {XRT_INPUT_TOUCH_SQUEEZE_VALUE, XRT_INPUT_TOUCH_SQUEEZE_VALUE},
    {XRT_INPUT_TOUCH_TRIGGER_TOUCH, XRT_INPUT_TOUCH_TRIGGER_TOUCH},
    {XRT_INPUT_TOUCH_TRIGGER_VALUE, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_TOUCH_THUMBSTICK_CLICK, XRT_INPUT_TOUCH_THUMBSTICK_CLICK},
    {XRT_INPUT_TOUCH_THUMBSTICK_TOUCH, XRT_INPUT_TOUCH_THUMBSTICK_TOUCH},
    {XRT_INPUT_TOUCH_THUMBSTICK, XRT_INPUT_TOUCH_THUMBSTICK},
    {XRT_INPUT_TOUCH_THUMBREST_TOUCH, XRT_INPUT_TOUCH_THUMBREST_TOUCH},
    {XRT_INPUT_TOUCH_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
    {XRT_INPUT_TOUCH_TRIGGER_PROXIMITY, XRT_INPUT_TOUCH_TRIGGER_PROXIMITY},
    {XRT_INPUT_TOUCH_THUMB_PROXIMITY, XRT_INPUT_TOUCH_THUMB_PROXIMITY},
};

static struct xrt_binding_output_pair touch_outputs_touch[] = {
    {XRT_OUTPUT_NAME_TOUCH_HAPTIC, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

static struct xrt_binding_input_pair vive_tracker_inputs_touch[] = {
    {XRT_INPUT_VIVE_TRACKER_MENU_CLICK, XRT_INPUT_TOUCH_MENU_CLICK},
    {XRT_INPUT_VIVE_TRACKER_MENU_CLICK, XRT_INPUT_TOUCH_SYSTEM_CLICK},
    {XRT_INPUT_VIVE_TRACKER_TRIGGER_CLICK, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_VIVE_TRACKER_SQUEEZE_CLICK, XRT_INPUT_TOUCH_SQUEEZE_VALUE},
    {XRT_INPUT_VIVE_TRACKER_TRIGGER_VALUE, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_VIVE_TRACKER_TRACKPAD, XRT_INPUT_TOUCH_THUMBSTICK},
    {XRT_INPUT_VIVE_TRACKER_TRACKPAD_CLICK, XRT_INPUT_TOUCH_THUMBSTICK_CLICK},
    {XRT_INPUT_VIVE_TRACKER_TRACKPAD_TOUCH, XRT_INPUT_TOUCH_THUMBSTICK_TOUCH},
    {XRT_INPUT_VIVE_TRACKER_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_GENERIC_TRACKER_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
};

static struct xrt_binding_output_pair vive_tracker_outputs_touch[] = {
    {XRT_OUTPUT_NAME_VIVE_TRACKER_HAPTIC, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

struct xrt_binding_profile touch_profile_bindings[] = {
    // "CV1" touch controller -> normal touch profile
    {
        .name = XRT_DEVICE_TOUCH_CONTROLLER,
        .inputs = touch_inputs_touch,
        .input_count = ARRAY_SIZE(touch_inputs_touch),
        .outputs = touch_outputs_touch,
        .output_count = ARRAY_SIZE(touch_outputs_touch),
    },
    {
        .name = XRT_DEVICE_SIMPLE_CONTROLLER,
        .inputs = simple_inputs_touch,
        .input_count = ARRAY_SIZE(simple_inputs_touch),
        .outputs = simple_outputs_touch,
        .output_count = ARRAY_SIZE(simple_outputs_touch),
    },
    // NOTE: THIS SHOULD ALWAYS BE LAST! see rift_radio.c
    {
        .name = XRT_DEVICE_VIVE_TRACKER,
        .inputs = vive_tracker_inputs_touch,
        .input_count = ARRAY_SIZE(vive_tracker_inputs_touch),
        .outputs = vive_tracker_outputs_touch,
        .output_count = ARRAY_SIZE(vive_tracker_outputs_touch),
    },
};
uint32_t touch_profile_bindings_count = ARRAY_SIZE(touch_profile_bindings);

static struct xrt_binding_input_pair xbox_inputs_remote[] = {
    {XRT_INPUT_XBOX_A_CLICK, XRT_INPUT_RIFT_REMOTE_SELECT_CLICK},
    {XRT_INPUT_XBOX_B_CLICK, XRT_INPUT_RIFT_REMOTE_VOLUME_DOWN_CLICK},
    {XRT_INPUT_XBOX_X_CLICK, XRT_INPUT_RIFT_REMOTE_VOLUME_UP_CLICK},
    {XRT_INPUT_XBOX_DPAD_UP_CLICK, XRT_INPUT_RIFT_REMOTE_DPAD_UP_CLICK},
    {XRT_INPUT_XBOX_DPAD_DOWN_CLICK, XRT_INPUT_RIFT_REMOTE_DPAD_DOWN_CLICK},
    {XRT_INPUT_XBOX_DPAD_LEFT_CLICK, XRT_INPUT_RIFT_REMOTE_DPAD_RIGHT_CLICK},
    {XRT_INPUT_XBOX_DPAD_RIGHT_CLICK, XRT_INPUT_RIFT_REMOTE_DPAD_RIGHT_CLICK},
    {XRT_INPUT_XBOX_VIEW_CLICK, XRT_INPUT_RIFT_REMOTE_BACK_CLICK},
    {XRT_INPUT_XBOX_MENU_CLICK, XRT_INPUT_RIFT_REMOTE_OCULUS_CLICK},
};

struct xrt_binding_profile remote_profile_bindings[] = {
    {
        .name = XRT_DEVICE_XBOX_CONTROLLER,
        .inputs = xbox_inputs_remote,
        .input_count = ARRAY_SIZE(xbox_inputs_remote),
    },
};
uint32_t remote_profile_bindings_count = ARRAY_SIZE(remote_profile_bindings);
