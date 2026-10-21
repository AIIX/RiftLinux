// Copyright 2025, Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Radio state machine functions for the Oculus Rift.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#include "rift_internal.h"

// Handles reading from the radio HID device and processing any incoming messages
int
rift_radio_handle_read(struct rift_hmd *hmd);

// Handles updating the haptic state of the controllers
int
rift_radio_handle_haptics(struct rift_hmd *hmd);

// Checks for completed async commands and processes them
int
rift_radio_handle_command(struct rift_hmd *hmd);

/*!
 * Gives the radio and the Touch controllers the setup the Oculus runtime gives them, see the implementation.
 */
int
rift_radio_handle_config(struct rift_hmd *hmd);

/*!
 * Creates the left and right Touch controllers and adds them to the HMD's devices, before any of them has said
 * anything over the radio. A controller that connects later, after being woken up, then fills in the device that
 * applications already have, rather than not existing for them.
 *
 * @return 0 on success, negative on failure.
 */
int
rift_radio_create_touch_controllers(struct rift_hmd *hmd);
