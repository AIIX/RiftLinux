// Copyright 2019-2023, Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface of libmonado
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 */

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif


/*
 *
 * Enums, defines and objects.
 *
 */

//! Major version of the API.
#define MND_API_VERSION_MAJOR 1
//! Minor version of the API.
#define MND_API_VERSION_MINOR 12
//! Patch version of the API.
#define MND_API_VERSION_PATCH 0

/*!
 * Result codes for operations, negative are errors, zero or positives are
 * success.
 */
typedef enum mnd_result
{
	MND_SUCCESS = 0,
	MND_ERROR_INVALID_VERSION = -1,
	MND_ERROR_INVALID_VALUE = -2,
	MND_ERROR_CONNECTING_FAILED = -3,
	MND_ERROR_OPERATION_FAILED = -4,
	//! Supported in version 1.1 and above.
	MND_ERROR_RECENTERING_NOT_SUPPORTED = -5,
	//! Supported in version 1.2 and above.
	MND_ERROR_INVALID_PROPERTY = -6,
	//! Supported in version 1.3 and above.
	MND_ERROR_INVALID_OPERATION = -7,
	//! Supported in version 1.5 and above.
	MND_ERROR_UNSUPPORTED_OPERATION = -8,
} mnd_result_t;

/*!
 * Bitflags for client application state.
 */
typedef enum mnd_client_flags
{
	MND_CLIENT_PRIMARY_APP = (1u << 0u),
	MND_CLIENT_SESSION_ACTIVE = (1u << 1u),
	MND_CLIENT_SESSION_VISIBLE = (1u << 2u),
	MND_CLIENT_SESSION_FOCUSED = (1u << 3u),
	MND_CLIENT_SESSION_OVERLAY = (1u << 4u),
	//! @deprecated Deprecated in version 1.6.
	MND_CLIENT_IO_ACTIVE = (1u << 5u),
	//! Supported in version 1.6 and above.
	MND_CLIENT_POSES_BLOCKED = (1u << 6u),
	//! Supported in version 1.6 and above.
	MND_CLIENT_HT_BLOCKED = (1u << 7u),
	//! Supported in version 1.6 and above.
	MND_CLIENT_INPUTS_BLOCKED = (1u << 8u),
	//! Supported in version 1.6 and above.
	MND_CLIENT_OUTPUTS_BLOCKED = (1u << 9u),
} mnd_client_flags_t;

/*!
 * A property to get from an object.
 *
 * Supported in version 1.2 and above.
 */
typedef enum mnd_property
{
	// Devices

	//! Supported in version 1.2 and above. Supported by devices.
	MND_PROPERTY_NAME_STRING = 0,

	//! Supported in version 1.2 and above. Supported by devices.
	MND_PROPERTY_SERIAL_STRING = 1,

	//! Supported in version 1.4.0 and above. Supported by devices.
	MND_PROPERTY_TRACKING_ORIGIN_U32 = 2,

	//! Supported in version 1.4.0 and above. Supported by devices.
	MND_PROPERTY_SUPPORTS_POSITION_BOOL = 3,

	//! Supported in version 1.4.0 and above. Supported by devices.
	MND_PROPERTY_SUPPORTS_ORIENTATION_BOOL = 4,

	//! Supported in version 1.5.0 and above. Supported by devices.
	MND_PROPERTY_SUPPORTS_BRIGHTNESS_BOOL = 5,

	//! Supported in version 1.8.0 and above. Supported by clients.
	MND_PROPERTY_SUPPORTS_VIEW_CONFIGURATION_CHANGE_BOOL = 100,
} mnd_property_t;

/*!
 * Opaque type for libmonado state
 */
typedef struct mnd_root mnd_root_t;

/*!
 * A pose composed of a position and orientation.
 */
/*!
 * RiftLinux, retired (@ref mnd_root_get_controller_calibration answers that there is none).
 *
 * An orientation is the direction that is up, in the frame of the controller's aim pose, in OpenXR axes (+X right, +Y
 * its top, -Z where it points): 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z, then the diagonals, with k = 0.577: 6 (k,k,k),
 * 7 (k,k,-k), 8 (k,-k,k), 9 (k,-k,-k), 10 (-k,k,k), 11 (-k,k,-k), 12 (-k,-k,k), 13 (-k,-k,-k). So: 5 pointing
 * straight up (buttons facing you), 4 pointing straight down (likewise), 0 its buttons turned to the left, 1 to the
 * right, 2 pointing at the sensors as usually held, 3 upside down (buttons facing the floor).
 *
 * First "come closer": @ref next_orientation -2, @ref progress how well the sensors see the controller (60 and up:
 * close enough); the person holds the trigger to go on: @ref progress 100 (go on as the trigger is let go), or
 * @ref reason 8 when pressed too soon (until let go). Nothing goes on by itself.
 *
 * The person takes each pose by holding the controller's trigger: it asks for 5, 4, 0, 1 in turn
 * (@ref next_orientation). While the trigger is held, @ref progress fills (the circle); at 100 the pose is done when
 * @ref reason is 0 (its @ref orientation_fill becomes 1), or turned away with reason 6 (not the pose shown: more than
 * 100 degrees off, the opposite of it) or 7 (under 30 degrees from a pose done before). Letting go of the trigger goes
 * on to the next pose, or asks for the same again. Four done make a round; a healthy controller needs one, a turned
 * one two or three.
 */
typedef struct mnd_controller_calibration
{
	//! 0 none since the runtime started, 1 measuring, 2 done and kept, 3 failed or cancelled.
	int32_t state;
	//! The pose asked for now, as its trigger is held: how much of what it needs it has, 0-100.
	int32_t progress;
	//! Over all rounds, 0-100; does not start over.
	int32_t overall_progress;
	//! The round, 1 on.
	int32_t round;
	//! For state 3: 1 cancelled, 2 nothing new measured for a long time, 3 the solves disagree among themselves, 4 the
	//! correction came out too large, 5 the accelerometer's bias came out too large; 0 otherwise. For state 1: 6 or 7
	//! while a pose just held is shown as turned away, 8 while "come closer" was pressed too soon (until let go). For
	//! state 3 also 9: the poses held, three sets over, gave readings too unsettled for a result.
	int32_t reason;
	//! How far the motion sensor is turned against the LEDs, degrees, and the accelerometer's bias, m/s², found so far.
	float rotation_deg;
	float bias_m_s2;
	//! What the last fit left, degrees.
	float residual_deg;
	//! The four poses asked for: 1 when done this round; the one asked for now as far as it is; the others 0.
	float orientation_fill[14];
	//! The orientation it rests in now, from its own accelerometer, not its tracking; -1 when it does not rest.
	int32_t orientation_now;
	//! The pose asked for now, -2 for "come closer" first, -1 for none.
	int32_t next_orientation;
	//! The LEDs the sensors matched just now, 0 when not seen lately. A reading needs 8.
	int32_t leds;
	//! Still enough (for a hand, with the trigger held) for readings now.
	bool resting;
	//! Its tracking keeps disagreeing with its motion sensor: a calibration would help. Until one is done.
	bool suggested;
} mnd_controller_calibration_t;

typedef struct mnd_pose
{
	struct
	{
		float x, y, z, w;
	} orientation;
	struct
	{
		float x, y, z;
	} position;
} mnd_pose_t;


/*!
 * A 3 element colour with floating point channels.
 */
struct mnd_colour
{
	float r;
	float g;
	float b;
};

/*!
 * A 3 element HSV colour with floating point channels.
 * All values are in [0, 1] range, with hue wrapping at 1.
 */
struct mnd_colour_hsv
{
	float h;
	float s;
	float v;
};

/*!
 * Types of reference space.
 */
typedef enum mnd_reference_space_type
{
	MND_SPACE_REFERENCE_TYPE_VIEW,
	MND_SPACE_REFERENCE_TYPE_LOCAL,
	MND_SPACE_REFERENCE_TYPE_LOCAL_FLOOR,
	MND_SPACE_REFERENCE_TYPE_STAGE,
	MND_SPACE_REFERENCE_TYPE_UNBOUNDED,
} mnd_reference_space_type_t;

/*!
 * Bitflags for IO blocking.
 *
 * Bitflags are only to be used here, this should not be used as a template
 * for future libmonado interfaces.
 *
 * Supported in version 1.6.0 and above.
 */
typedef enum mnd_io_block_flags
{
	MND_IO_BLOCK_POSES = (1u << 0u),
	MND_IO_BLOCK_HT = (1u << 1u),
	MND_IO_BLOCK_INPUTS = (1u << 2u),
	MND_IO_BLOCK_OUTPUTS = (1u << 3u),
} mnd_io_block_flags_t;

/*!
 * Types of view configurations.
 *
 * Supported in version 1.8.0 and above.
 */
typedef enum mnd_view_type
{
	//! Invalid view type, no data is here.
	MND_VIEW_TYPE_INVALID = 0,
	//! Mono view type, with a single view.
	MND_VIEW_TYPE_MONO = 1,
	//! Stereo view type, with two views.
	MND_VIEW_TYPE_STEREO = 2,
	//! Quad view type, with two outer views, and two inset views.
	MND_VIEW_TYPE_QUAD = 3,
} mnd_view_type_t;

#define MND_MAX_VIEWS 4

/*!
 * The state of a client's session.
 *
 * Supported in version 1.8.0 and above.
 */
typedef struct mnd_session_state
{
	//! Whether the session is currently running/has been begun.
	bool running;
	//! The active view type of the running session.
	mnd_view_type_t active_view_type;
} mnd_session_state_t;

/*!
 * A view configuration, describing the resolution and sample count of a single view.
 *
 * Supported in version 1.8.0 and above.
 */
typedef struct mnd_view_config_view
{
	uint32_t width_pixels;
	uint32_t height_pixels;
	uint32_t sample_count;
} mnd_view_config_view_t;

/*!
 * A view configuration, describing the resolution and sample count of multiple views.
 *
 * Supported in version 1.8.0 and above.
 */
typedef struct mnd_recommended_view_config
{
	bool valid;
	mnd_view_config_view_t view_configs[MND_MAX_VIEWS];
} mnd_recommended_view_config_t;

/*
 *
 * Functions
 *
 */

/*!
 * Returns the version of the API (not Monado itself), follows the versioning
 * semantics of https://semver.org/ standard. In short if the major version
 * mismatch then the interface is incompatible.
 *
 * @param[out] out_major Major version number, must be valid pointer.
 * @param[out] out_minor Minor version number, must be valid pointer.
 * @param[out] out_patch Patch version number, must be valid pointer.
 *
 * Always succeeds, or crashes if any pointer isn't valid.
 */
void
mnd_api_get_version(uint32_t *out_major, uint32_t *out_minor, uint32_t *out_patch);

/*!
 * Create libmonado state and connect to service
 *
 * @param[out] out_root Address to populate with the opaque state type.
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_create(mnd_root_t **out_root);

/*!
 * Destroy libmonado state, disconnecting from the service, and zeroing the
 * pointer.
 *
 * @param root_ptr Pointer to your libmonado state. Null-checked, will be set to null.
 */
void
mnd_root_destroy(mnd_root_t **root_ptr);

/*!
 * Update our local cached copy of the client list
 *
 * @param root The libmonado state.
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_update_client_list(mnd_root_t *root);

/*!
 * Get the number of active clients
 *
 * This value only changes on calls to @ref mnd_root_update_client_list
 *
 * @param root         The libmonado state.
 * @param[out] out_num Pointer to value to populate with the number of clients.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_number_clients(mnd_root_t *root, uint32_t *out_num);

/*!
 * Get the id from the current client list.
 *
 * @param root               The libmonado state.
 * @param index              Index to retrieve id for.
 * @param[out] out_client_id Pointer to value to populate with the id at the given index.
 */
mnd_result_t
mnd_root_get_client_id_at_index(mnd_root_t *root, uint32_t index, uint32_t *out_client_id);

/*!
 * Get the name of the client at the given index.
 *
 * The string returned is only valid until the next call into libmonado.
 *
 * @param root          The libmonado state.
 * @param client_id     ID of client to retrieve name from.
 * @param[out] out_name Pointer to populate with the client name.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_client_name(mnd_root_t *root, uint32_t client_id, const char **out_name);

/*!
 * Get the state flags of the client at the given index.
 *
 * This result only changes on calls to @ref mnd_root_update_client_list
 *
 * @param root           The libmonado state.
 * @param client_id      ID of client to retrieve flags from.
 * @param[out] out_flags Pointer to populate with the flags, a bitwise combination of @ref mnd_client_flags.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_client_state(mnd_root_t *root, uint32_t client_id, uint32_t *out_flags);

/*!
 * Set the client at the given index as "primary".
 *
 * @param root      The libmonado state.
 * @param client_id ID of the client set as primary.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_set_client_primary(mnd_root_t *root, uint32_t client_id);

/*!
 * RiftLinux: a screenshot of what the headset shows, as one picture of the whole field of view (both eyes' views
 * joined), with every layer composited the way the headset shows them (backdrop blur, HDR mapping, colour) but
 * without the lens's correction. Written as an 8-bit sRGB PNG to @p png_path (written beside it as "<path>.part",
 * then renamed). Blocks until the file is written, normally one frame and the encoding, at most about 3 s: call it
 * from a thread of its own, not the one that renders frames. Black while the headset shows black.
 *
 * @param root     The libmonado state.
 * @param png_path Absolute path of the PNG; its folder must exist.
 * @param flags    0 (reserved).
 *
 * @return MND_SUCCESS when the file is written, MND_ERROR_INVALID_VALUE for a path that is not absolute or too long or
 *         flags that are not 0, MND_ERROR_OPERATION_FAILED when no frame came in time or the file could not be written.
 */
mnd_result_t
mnd_root_take_screenshot(mnd_root_t *root, const char *png_path, uint32_t flags);

/*!
 * RiftLinux: the lens test's pattern, drawn by the runtime in panel pixels around each eye's lens centre after the
 * lens correction, so that its one-pixel lines land on panel pixels (a picture drawn by the application is resampled
 * twice on its way to the panel): a green cross of four lines a bar, one pixel wide with one-pixel black gaps, 81 by 81
 * pixels; everything else shows what the application draws. Off again when the client that turned it on disconnects.
 * Rift CV1 only.
 *
 * @param root The libmonado state.
 * @param on   Whether to draw it.
 *
 * @return MND_SUCCESS, or MND_ERROR_OPERATION_FAILED when the headset gives no lens centres (not a Rift CV1).
 */
mnd_result_t
mnd_root_set_lens_pattern(mnd_root_t *root, bool on);

/*
 * RiftLinux: retired. There is no controller calibration by hand any more: a Touch controller whose motion sensor is
 * turned or biased against its LEDs is learned and corrected while it is used (settings.json controllers.imu_align).
 * These four stay for programs built against them.
 */

/*!
 * RiftLinux, retired: no calibration starts.
 *
 * @return MND_ERROR_INVALID_VALUE for a @p hand other than 0, 1 or 2, else MND_ERROR_OPERATION_FAILED.
 */
mnd_result_t
mnd_root_calibrate_controller(mnd_root_t *root, uint32_t hand, bool start);

/*!
 * RiftLinux, retired: there is no calibration to report.
 *
 * @return MND_ERROR_INVALID_VALUE for a @p hand other than 0 or 1, else MND_ERROR_OPERATION_FAILED.
 */
mnd_result_t
mnd_root_get_controller_calibration(mnd_root_t *root, uint32_t hand, mnd_controller_calibration_t *out_status);

/*!
 * RiftLinux, retired: nothing is kept to forget.
 *
 * @return MND_ERROR_INVALID_VALUE for a @p hand other than 0 or 1, else MND_ERROR_OPERATION_FAILED.
 */
mnd_result_t
mnd_root_forget_controller_calibration(mnd_root_t *root, uint32_t hand);

/*!
 * RiftLinux, retired: does nothing.
 *
 * @return MND_SUCCESS.
 */
mnd_result_t
mnd_root_set_controller_calibration_quiet(mnd_root_t *root, bool quiet);

/*!
 * Set the client at the given index as "focused".
 *
 * @param root      The libmonado state.
 * @param client_id ID of the client set as focused.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_set_client_focused(mnd_root_t *root, uint32_t client_id);

/*!
 * Toggle io activity for the client at the given index.
 *
 * @deprecated Deprecated in version 1.6.
 *
 * @param root      The libmonado state.
 * @param client_id ID of the client to toggle IO for.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_toggle_client_io_active(mnd_root_t *root, uint32_t client_id);

/*!
 * Block certain types of IO for the client at the given index.
 *
 * Supported in version 1.6 and above.
 *
 * @param root        The libmonado state.
 * @param client_id   ID of the client to block IO for.
 * @param block_flags Which types of IO to block.
 */
mnd_result_t
mnd_root_set_client_io_blocks(mnd_root_t *root, uint32_t client_id, mnd_io_block_flags_t block_flags);

/*!
 * Get the session state for the client at the given index.
 *
 * Supported in version 1.8 and above.
 *
 * @param root                   The libmonado state.
 * @param client_id              ID of client to retrieve active view type from.
 * @param[out] out_session_state Pointer to populate with the session state.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_client_session_running_state(mnd_root_t *root, uint32_t client_id, mnd_session_state_t *out_session_state);

/*!
 * Get the view configurations of the client's first system. Returns the default and the recommended view configuration,
 * if available.
 *
 * Supported in version 1.8 and above.
 *
 * @param root                             The libmonado state.
 * @param client_id                        ID of the client to get the view configuration for.
 * @param view_type                        The type of view configuration to retrieve.
 * @param[out] out_default_view_config     Pointer to populate with the default view configuration. Must point to at
 *                                         least the number of views as is contained within the passed view type.
 * @param[out] recommendation_present      Pointer to populate with whether a recommended view configuration is present.
 * @param[out] out_recommended_view_config Pointer to populate with the recommended view configuration.
 *                                         Must point to at least the number of views as is contained within the
 *                                         passed view type.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_client_system_view_config(mnd_root_t *root,
                                       uint32_t client_id,
                                       mnd_view_type_t view_type,
                                       mnd_view_config_view_t *out_default_view_config,
                                       mnd_recommended_view_config_t *out_recommended_view_config);

/*!
 * Set the recommended view configuration for a client's first system.
 *
 * Supported in version 1.8 and above.
 *
 * @param root                    The libmonado state.
 * @param client_id               ID of the client to set the recommended view configuration for.
 * @param view_type               The type of view configuration to set.
 * @param recommended_view_config The view configuration to set as recommended. Must point to at least the number of
 *                                views as is contained within the passed view type.
 *
 * @pre Called @ref mnd_root_update_client_list at least once
 *
 * @return MND_SUCCESS on success, MND_ERROR_UNSUPPORTED_OPERATION if the client doesn't support changing the
 *         recommended view configuration
 */
mnd_result_t
mnd_root_set_client_recommended_view_config(mnd_root_t *root,
                                            uint32_t client_id,
                                            mnd_view_type_t view_type,
                                            const mnd_recommended_view_config_t *recommended_view_config);

/*!
 * Get boolean property for the client at the given index.
 *
 * Supported in version 1.8.0 and above.
 *
 * @param root          The libmonado state.
 * @param client_index  Index of client to retrieve property from.
 * @param prop          A boolean property enum.
 * @param[out] out_bool Pointer to populate with the boolean.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_client_property_bool(mnd_root_t *root, uint32_t client_id, mnd_property_t prop, bool *out_bool);

/*!
 * Get the number of devices
 *
 * @param root                  The libmonado state.
 * @param[out] out_device_count Pointer to value to populate with the number of devices.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_count(mnd_root_t *root, uint32_t *out_device_count);

/*!
 * Get boolean property for the device at the given index.
 *
 * Supported in version 1.2 and above.
 *
 * @param root          The libmonado state.
 * @param device_index  Index of device to retrieve name from.
 * @param prop          A boolean property enum.
 * @param[out] out_bool Pointer to populate with the boolean.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_info_bool(mnd_root_t *root, uint32_t device_index, mnd_property_t prop, bool *out_bool);

/*!
 * Get int32_t property for the device at the given index.
 *
 * Supported in version 1.2 and above.
 *
 * @param root         The libmonado state.
 * @param device_index Index of device to retrieve name from.
 * @param prop         A int32_t property enum.
 * @param[out] out_i32 Pointer to populate with the int32_t.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_info_i32(mnd_root_t *root, uint32_t device_index, mnd_property_t prop, uint32_t *out_i32);

/*!
 * Get uint32_t property for the device at the given index.
 *
 * Supported in version 1.2 and above.
 *
 * @param root          The libmonado state.
 * @param device_index  Index of device to retrieve name from.
 * @param prop          A uint32_t property enum.
 * @param[out] out_u32 Pointer to populate with the uint32_t.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_info_u32(mnd_root_t *root, uint32_t device_index, mnd_property_t prop, uint32_t *out_u32);

/*!
 * Get float property for the device at the given index.
 *
 * Supported in version 1.2 and above.
 *
 * @param root           The libmonado state.
 * @param device_index   Index of device to retrieve name from.
 * @param prop           A float property enum.
 * @param[out] out_float Pointer to populate with the float.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_info_float(mnd_root_t *root, uint32_t device_index, mnd_property_t prop, float *out_float);

/*!
 * Get string property for the device at the given index.
 *
 * Supported in version 1.2 and above.
 *
 * @param root            The libmonado state.
 * @param device_index    Index of device to retrieve name from.
 * @param prop            A string property enum.
 * @param[out] out_string Pointer to populate with the string.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_info_string(mnd_root_t *root, uint32_t device_index, mnd_property_t prop, const char **out_string);

/*!
 * Get device info at the given index.
 *
 * @deprecated Deprecated in version 1.2, scheduled for removal in version 2.0.0 currently.
 *
 * @param root               The libmonado state.
 * @param device_index       Index of device to retrieve name from.
 * @param[out] out_device_id Pointer to value to populate with the device id at the given index.
 * @param[out] out_dev_name  Pointer to populate with the device name.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_info(mnd_root_t *root, uint32_t device_index, uint32_t *out_device_id, const char **out_dev_name);

/*!
 * Get the device index associated for a given role name.
 *
 *
 * @param root           The libmonado state.
 * @param role_name      Name of the role. Possible values are:
 *                       - "head"
 *                       - "left"
 *                       - "right"
 *                       - "gamepad"
 *                       - "eyes"
 *                       - "hand-tracking-unobstructed-[left|right]"
 *                       - "hand-tracking-conforming-[left|right]"
 *
 *                       **DEPRECATED**: The role names "hand-tracking-[left|right]"
 *                       are deprecated as of v1.5. They now map to
 *                       "hand-tracking-unobstructed-[left|right]" and are
 *                       scheduled for removal in v2.0.
 *
 * @param[out] out_index Pointer to value to populate with the device index
 *                       associated with given role name, -1 if not role is set.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_from_role(mnd_root_t *root, const char *role_name, int32_t *out_index);

/*!
 * Trigger a recenter of the local spaces.
 *
 * Supported in version 1.1 and above.
 *
 * @param root The libmonado state.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_recenter_local_spaces(mnd_root_t *root);

/*!
 * Get the current offset value of the specified reference space.
 *
 * Supported in version 1.3 and above.
 *
 * @param root The libmonado state.
 * @param type The reference space.
 * @param[out] out_offset A pointer to where the offset should be written.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_reference_space_offset(mnd_root_t *root, mnd_reference_space_type_t type, mnd_pose_t *out_offset);

/*!
 * Apply an offset to the specified reference space.
 *
 * Supported in version 1.3 and above.
 *
 * @param root The libmonado state.
 * @param type The reference space.
 * @param offset A pointer to valid xrt_pose.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_set_reference_space_offset(mnd_root_t *root, mnd_reference_space_type_t type, const mnd_pose_t *offset);

/*!
 * Read the current offset of a tracking origin.
 *
 * Supported in version 1.3 and above.
 *
 * @param root The libmonado state.
 * @param origin_index The index of the tracking origin into the internal list.
 * @param[out] out_offset A pointer to where the offset should be written.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_tracking_origin_offset(mnd_root_t *root, uint32_t origin_index, mnd_pose_t *out_offset);

/*!
 * Apply an offset to the specified tracking origin.
 *
 * Supported in version 1.3 and above.
 *
 * @param root The libmonado state.
 * @param origin_index The index of the tracking origin into the internal list.
 * @param offset A pointer to valid xrt_pose.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_set_tracking_origin_offset(mnd_root_t *root, uint32_t origin_index, const mnd_pose_t *offset);

/*!
 * Retrieve the number of tracking origins available.
 *
 * Supported in version 1.3 and above.
 *
 * @param root The libmonado state.
 * @param out_track_count Pointer to where the count should be written.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_tracking_origin_count(mnd_root_t *root, uint32_t *out_track_count);

/*!
 * Retrieve the name of the indicated tracking origin.
 *
 * Supported in version 1.3 and above.
 *
 * @param root The libmonado state.
 * @param origin_index The index of the tracking origin into the internal list.
 * @param out_string The pointer to write the name's pointer to.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_tracking_origin_name(mnd_root_t *root, uint32_t origin_index, const char **out_string);

/*!
 * Get battery status of a device.
 *
 * @param root               The libmonado state.
 * @param device_index       Index of device to retrieve battery info from.
 * @param[out] out_present   Pointer to value to populate with whether the device provides battery status info.
 * @param[out] out_charging  Pointer to value to populate with whether the device is currently being charged.
 * @param[out] out_charge    Pointer to value to populate with the battery charge as a value between 0 and 1.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_battery_status(
    mnd_root_t *root, uint32_t device_index, bool *out_present, bool *out_charging, float *out_charge);

/*!
 * Get current brightness of a display device.
 *
 * @param root                 The libmonado state.
 * @param device_index         Index of device to retrieve brightness from.
 * @param[out] out_brightness  Pointer to value to populate with the current device brightness, where 0 is 0%, and 1 is
 * 100%.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_get_device_brightness(mnd_root_t *root, uint32_t device_index, float *out_brightness);

/*!
 * @brief Set the display brightness.
 *
 * @param root                 The libmonado state.
 * @param device_index         Index of device to retrieve battery info from.
 * @param[in] brightness       Desired display brightness, usually between 0 and 1. Some devices may
 *                             allow exceeding 1 if the supported range exceeds 100%, but it will be clamped to
 *                             the supported range.
 * @param[in] relative         Whether to add \a brightness to the current brightness, instead of overwriting
 *                             the current brightness.
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_set_device_brightness(mnd_root_t *root, uint32_t device_index, float brightness, bool relative);

/*!
 * Set the chroma key parameters to be applied to the base application (if there is any).
 *
 * HSV values are in [0, 1] range. Hue wrapping is supported (min > max spans across 0).
 * Set curve to 0 to disable chroma keying.
 *
 * @param root    The libmonado state.
 * @param hsv_min Minimum HSV bounds (hue, saturation, value).
 * @param hsv_max Maximum HSV bounds (hue, saturation, value).
 * @param curve   Power curve for alpha falloff (1.0 = linear, <1 = softer, >1 = harder, 0 = disabled).
 * @param despill Despill strength (0.0 = none, 1.0 = full desaturation near key color).
 *
 * @return MND_SUCCESS on success
 */
mnd_result_t
mnd_root_set_chroma_key_params(
    mnd_root_t *root, struct mnd_colour_hsv hsv_min, struct mnd_colour_hsv hsv_max, float curve, float despill);

#ifdef __cplusplus
}
#endif
