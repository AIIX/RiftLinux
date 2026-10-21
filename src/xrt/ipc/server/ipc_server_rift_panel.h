// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The Rift panel in RiftLinux's own runtime: the same tab as in SteamVR's dashboard (sound, view, picture,
 *         motion, status, display), shown in front of the user on the right controller's Oculus button, for native
 *         OpenXR applications under monado-service.
 * @ingroup ipc_server
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ipc_server;
struct rl_panel;

/*!
 * Settings that must be in the environment before the compositor is made (the render scale for applications), from
 * the settings file, unless already set. Call before the server starts.
 */
void
rl_panel_settings_to_env(void);

/*!
 * Creates the panel and its thread: applies the settings file (picture, floor and height), switches the default audio
 * output to the headset's headphones. NULL without a compositor or a head device.
 */
struct rl_panel *
rl_panel_create(struct ipc_server *s);

//! Stops the thread, puts the audio output back, frees everything.
void
rl_panel_destroy(struct rl_panel **panel_ptr);

//! Opens the panel in front of the user, or closes it (the Oculus button pressed).
void
rl_panel_toggle(struct rl_panel *panel);

#ifdef __cplusplus
}
#endif
