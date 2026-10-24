// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Both Touch controllers as one standard Linux gamepad, see rift_gamepad.h.
 * @ingroup drv_rift
 *
 * The Touch controllers reach the computer only through the headset's radio, so no desktop program sees them. When
 * RiftLinux's own runtime is started with --gamepad they also make one gamepad through uinput while it runs, with the
 * standard Linux gamepad codes (Documentation/input/gamepad.rst: the face buttons by position): A south, B east,
 * X west, Y north, the sticks, the triggers as analog axes, the grips as shoulder buttons, the stick clicks, and the
 * left controller's menu button as the home button. The right controller's Oculus button is not part of it: it stays
 * RiftLinux's own (recentring).
 */

#include "util/u_debug.h"
#include "util/u_logging.h"

#include "os/os_threading.h"

#include "rift_gamepad.h"
#include "rift_internal.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/ioctl.h>

DEBUG_GET_ONCE_BOOL_OPTION(rift_touch_gamepad, "RIFT_TOUCH_GAMEPAD", false)

//! A grip counts as pressed past this far in, and as let go below the second (no chatter at the edge).
#define RIFT_GAMEPAD_GRIP_PRESS 0.55f
#define RIFT_GAMEPAD_GRIP_RELEASE 0.45f

#define RIFT_GAMEPAD_STICK_MAX 32767
#define RIFT_GAMEPAD_TRIGGER_MAX 1023

//! What one controller last sent, so that only changes are written.
struct rift_gamepad_hand
{
	bool sent;
	uint8_t buttons;
	bool grip;
	int trigger;
	int stick_x;
	int stick_y;
};

struct rift_gamepad
{
	int fd;
	struct os_mutex mutex;
	struct rift_gamepad_hand hands[2];
};

//! Per hand: the codes of its two face buttons, menu, stick click and grip, its stick axes and trigger axis.
struct rift_gamepad_layout
{
	int button_lower, button_upper, button_menu, button_stick, grip;
	int axis_x, axis_y, axis_trigger;
};

static const struct rift_gamepad_layout layouts[2] = {
    // Right: A, B, (the Oculus button is not sent), stick, grip.
    {BTN_SOUTH, BTN_EAST, 0, BTN_THUMBR, BTN_TR, ABS_RX, ABS_RY, ABS_RZ},
    // Left: X, Y, menu as home, stick, grip.
    {BTN_WEST, BTN_NORTH, BTN_MODE, BTN_THUMBL, BTN_TL, ABS_X, ABS_Y, ABS_Z},
};

static bool
emit(struct rift_gamepad *gamepad, int type, int code, int value)
{
	struct input_event ev;
	memset(&ev, 0, sizeof(ev));
	ev.type = (unsigned short)type;
	ev.code = (unsigned short)code;
	ev.value = value;
	return write(gamepad->fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev);
}

static bool
setup_abs(int fd, int code, int min, int max, int fuzz, int flat)
{
	struct uinput_abs_setup abs;
	memset(&abs, 0, sizeof(abs));
	abs.code = (unsigned short)code;
	abs.absinfo.minimum = min;
	abs.absinfo.maximum = max;
	abs.absinfo.fuzz = fuzz;
	abs.absinfo.flat = flat;
	return ioctl(fd, UI_SET_ABSBIT, code) >= 0 && ioctl(fd, UI_ABS_SETUP, &abs) >= 0;
}

struct rift_gamepad *
rift_gamepad_create(void)
{
	if (!debug_get_bool_option_rift_touch_gamepad()) {
		return NULL;
	}

	const int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) {
		U_LOG_W("The Touch controllers are not offered as a gamepad: /dev/uinput can't be opened (%s).",
		        strerror(errno));
		return NULL;
	}

	bool ok = ioctl(fd, UI_SET_EVBIT, EV_KEY) >= 0 && ioctl(fd, UI_SET_EVBIT, EV_ABS) >= 0 &&
	          ioctl(fd, UI_SET_EVBIT, EV_SYN) >= 0;
	for (int h = 0; h < 2 && ok; h++) {
		const struct rift_gamepad_layout *l = &layouts[h];
		const int keys[] = {l->button_lower, l->button_upper, l->button_menu, l->button_stick, l->grip};
		for (size_t k = 0; k < sizeof(keys) / sizeof(keys[0]) && ok; k++) {
			ok = keys[k] == 0 || ioctl(fd, UI_SET_KEYBIT, keys[k]) >= 0;
		}
		ok = ok && setup_abs(fd, l->axis_x, -RIFT_GAMEPAD_STICK_MAX, RIFT_GAMEPAD_STICK_MAX, 16, 128) &&
		     setup_abs(fd, l->axis_y, -RIFT_GAMEPAD_STICK_MAX, RIFT_GAMEPAD_STICK_MAX, 16, 128) &&
		     setup_abs(fd, l->axis_trigger, 0, RIFT_GAMEPAD_TRIGGER_MAX, 0, 0);
	}

	struct uinput_setup setup;
	memset(&setup, 0, sizeof(setup));
	setup.id.bustype = BUS_VIRTUAL;
	// Oculus VR's USB vendor and the Rift CV1's product: the headset whose radio carries the controllers.
	setup.id.vendor = 0x2833;
	setup.id.product = 0x0031;
	setup.id.version = 1;
	snprintf(setup.name, sizeof(setup.name), "RiftLinux Touch controllers");
	ok = ok && ioctl(fd, UI_DEV_SETUP, &setup) >= 0 && ioctl(fd, UI_DEV_CREATE) >= 0;
	if (!ok) {
		U_LOG_W("The Touch controllers are not offered as a gamepad: uinput refused it (%s).", strerror(errno));
		close(fd);
		return NULL;
	}

	struct rift_gamepad *gamepad = calloc(1, sizeof(*gamepad));
	gamepad->fd = fd;
	os_mutex_init(&gamepad->mutex);
	U_LOG_I("The Touch controllers are offered as a gamepad, \"%s\".", setup.name);
	return gamepad;
}

static int
stick_value(float v)
{
	const float c = v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
	return (int)(c * (float)RIFT_GAMEPAD_STICK_MAX);
}

static void
send_hand_locked(struct rift_gamepad *gamepad, int h, const struct rift_gamepad_hand *now)
{
	const struct rift_gamepad_layout *l = &layouts[h];
	struct rift_gamepad_hand *was = &gamepad->hands[h];
	bool any = false;

	const struct
	{
		uint8_t mask;
		int code;
	} buttons[] = {
	    {RIFT_TOUCH_CONTROLLER_BUTTON_A, l->button_lower},
	    {RIFT_TOUCH_CONTROLLER_BUTTON_B, l->button_upper},
	    {RIFT_TOUCH_CONTROLLER_BUTTON_MENU, l->button_menu},
	    {RIFT_TOUCH_CONTROLLER_BUTTON_STICK, l->button_stick},
	};
	for (size_t b = 0; b < sizeof(buttons) / sizeof(buttons[0]); b++) {
		const bool down = (now->buttons & buttons[b].mask) != 0;
		if (buttons[b].code != 0 && (!was->sent || down != ((was->buttons & buttons[b].mask) != 0))) {
			any |= emit(gamepad, EV_KEY, buttons[b].code, down ? 1 : 0);
		}
	}
	if (!was->sent || now->grip != was->grip) {
		any |= emit(gamepad, EV_KEY, l->grip, now->grip ? 1 : 0);
	}
	if (!was->sent || now->trigger != was->trigger) {
		any |= emit(gamepad, EV_ABS, l->axis_trigger, now->trigger);
	}
	if (!was->sent || now->stick_x != was->stick_x) {
		any |= emit(gamepad, EV_ABS, l->axis_x, now->stick_x);
	}
	if (!was->sent || now->stick_y != was->stick_y) {
		any |= emit(gamepad, EV_ABS, l->axis_y, now->stick_y);
	}
	if (any) {
		emit(gamepad, EV_SYN, SYN_REPORT, 0);
	}
	*was = *now;
	was->sent = true;
}

void
rift_gamepad_update(struct rift_gamepad *gamepad, bool left, const struct rift_touch_controller_input_state *state)
{
	if (gamepad == NULL) {
		return;
	}
	const int h = left ? 1 : 0;

	os_mutex_lock(&gamepad->mutex);
	struct rift_gamepad_hand now = gamepad->hands[h];
	now.buttons = state->buttons;
	now.grip = gamepad->hands[h].grip ? state->grip > RIFT_GAMEPAD_GRIP_RELEASE : state->grip > RIFT_GAMEPAD_GRIP_PRESS;
	const float trigger = state->trigger < 0.0f ? 0.0f : (state->trigger > 1.0f ? 1.0f : state->trigger);
	now.trigger = (int)(trigger * (float)RIFT_GAMEPAD_TRIGGER_MAX);
	now.stick_x = stick_value(state->stick.x);
	// Pushed forward a stick reads +1; a gamepad's Y axis is negative up.
	now.stick_y = -stick_value(state->stick.y);
	send_hand_locked(gamepad, h, &now);
	os_mutex_unlock(&gamepad->mutex);
}

void
rift_gamepad_release(struct rift_gamepad *gamepad, bool left)
{
	if (gamepad == NULL) {
		return;
	}
	const int h = left ? 1 : 0;
	os_mutex_lock(&gamepad->mutex);
	if (gamepad->hands[h].sent) {
		const struct rift_gamepad_hand idle = {0};
		send_hand_locked(gamepad, h, &idle);
	}
	os_mutex_unlock(&gamepad->mutex);
}

void
rift_gamepad_destroy(struct rift_gamepad **gamepad_ptr)
{
	struct rift_gamepad *gamepad = *gamepad_ptr;
	if (gamepad == NULL) {
		return;
	}
	ioctl(gamepad->fd, UI_DEV_DESTROY);
	close(gamepad->fd);
	os_mutex_destroy(&gamepad->mutex);
	free(gamepad);
	*gamepad_ptr = NULL;
}
