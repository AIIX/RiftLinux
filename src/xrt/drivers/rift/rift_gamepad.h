// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Both Touch controllers as one standard Linux gamepad (uinput), for desktop programs while RiftLinux's own
 *         runtime runs with it asked for (riftlinux-runtime --gamepad): Plasma Bigscreen and anything else that takes
 *         game controllers through SDL.
 * @ingroup drv_rift
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct rift_touch_controller_input_state;
struct rift_gamepad;

/*!
 * The gamepad, when RIFT_TOUCH_GAMEPAD is set (riftlinux-runtime --gamepad sets it; nothing else does, so that no game
 * and no SteamVR session sees an extra gamepad) and /dev/uinput can be written; NULL otherwise, said in the log.
 */
struct rift_gamepad *
rift_gamepad_create(void);

//! Hands one controller's newest buttons, trigger, grip and stick to the gamepad; only changes are sent.
void
rift_gamepad_update(struct rift_gamepad *gamepad, bool left, const struct rift_touch_controller_input_state *state);

//! Lets go of whatever one controller still holds pressed (it disconnected).
void
rift_gamepad_release(struct rift_gamepad *gamepad, bool left);

void
rift_gamepad_destroy(struct rift_gamepad **gamepad_ptr);

#ifdef __cplusplus
}
#endif
