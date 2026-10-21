// Copyright 2020 Jan Schmidt
// Copyright 2026 Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Ab-initio blob<->LED correspondence search
 * @author Jan Schmidt <jan@centricular.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "tracking/t_constellation.h"

#include "pose_metrics.h"
#include "camera_model.h"


#ifdef __cplusplus
extern "C" {
#endif

#define MAX_BLOB_SEARCH_DEPTH 5

//! Lambdatwist can sometimes have spurious failures, the anchor LED must be within 2.5mm to be accepted
#define ANCHOR_LED_MAX_DISTANCE_M_SQ (0.025 * 0.025)

enum correspondence_search_flags
{
	CS_FLAG_NONE = 0x0,
	//! Do quick search @ depth 1-2 neighbour depth
	CS_FLAG_SHALLOW_SEARCH = 0x1,
	//! do deeper searches @ up to MAX_LED_SEARCH_DEPTH/MAX_BLOB_SEARCH_DEPTH
	CS_FLAG_DEEP_SEARCH = 0x2,
	//! Stop searching if a strong match is found, otherwise search all and return best match
	CS_FLAG_STOP_FOR_STRONG_MATCH = 0x4,
	//! Allow matching against all blobs, not just unlabelled ones or for the current device
	CS_FLAG_MATCH_ALL_BLOBS = 0x8,
	//! If the input obj_cam_pose contains a valid prior
	CS_FLAG_HAVE_POSE_PRIOR = 0x10,
	//! Reject poses that tilt the device away from what its IMU says, see @ref pose_metrics_gravity_error_rad.
	CS_FLAG_MATCH_GRAVITY = 0x20,
	//! Only accept poses within the error bounds of the pose prior. Depends on CS_FLAG_HAVE_POSE_PRIOR.
	CS_FLAG_PRIOR_MUST_MATCH = 0x40,
};

struct cs_image_point
{
	struct t_blob *blob;

	double point_homog[3]; // Homogeneous version of the point
	double size[2];        // w/h of the blob, in homogeneous coordinates
	double max_dist;       // norm of (W/H) for distance checks

	/* List of the nearest blobs, filtered for the active model */
	int num_neighbours;
	struct cs_image_point *neighbours[MAX_BLOB_SEARCH_DEPTH];
};

struct cs_model_info
{
	t_constellation_device_id_t id;

	const struct t_constellation_search_model *model;

	double best_pose_found_time; /* Time (in secs) at which the best pose was found */
	int best_pose_blob_depth;    /* Blob neighbor depth the best pose is from */
	int best_pose_led_depth;     /* LED neighbour depth the best pose is from */
	struct xrt_pose best_pose;
	enum pose_match_flags match_flags;

	struct pose_metrics best_score;

	/* Search parameters */
	double search_start_time;
	//! The search gives up after this (monotonic ns), see CONSTELLATION_TRACKER_SEARCH_BUDGET_MS; 0 for never.
	int64_t deadline_ns;
	bool timed_out;
	int led_depth;
	int led_index;
	int blob_index;

	enum correspondence_search_flags search_flags;
	int min_led_depth, max_led_depth;
	int max_blob_depth;

	/* Valid when CS_FLAG_HAVE_POSE_PRIOR is set */
	struct xrt_pose pose_prior;
	const struct xrt_vec3 *pos_error_thresh;
	const struct xrt_vec3 *rot_error_thresh;

	/* Used when CS_FLAG_MATCH_GRAVITY is set: gravity in the camera frame, and in the device frame */
	struct xrt_vec3 gravity_vector;
	struct xrt_vec3 device_gravity;
	float gravity_tolerance_rad;
};

struct correspondence_search
{
	const enum u_logging_level *ct_log_level;

	int num_points;
	struct cs_image_point *points;
	struct t_blob *blobs; /* Original blobs structs [num_points] */

	unsigned int num_trials;
	unsigned int num_pose_checks;
	//! Searches that ran out of time, see CONSTELLATION_TRACKER_SEARCH_BUDGET_MS; the caller may reset it.
	unsigned int num_timeouts;

	struct camera_model *calib;

	/* List of the nearest blobs for each blob */
	struct cs_image_point
	    *blob_neighbours[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME][XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME];
};

struct correspondence_search *
correspondence_search_new(const enum u_logging_level *ct_log_level, struct camera_model *camera_calib);

void
correspondence_search_free(struct correspondence_search *cs);

void
correspondence_search_set_blobs(struct correspondence_search *cs, struct t_blob *blobs, int num_blobs);

/*!
 * Does a brute-force search for a device's pose.
 *
 * @param gravity_vector        With CS_FLAG_MATCH_GRAVITY, the direction of gravity in the camera frame.
 * @param device_gravity        With CS_FLAG_MATCH_GRAVITY, the direction of gravity in the device frame.
 * @param gravity_tolerance_rad With CS_FLAG_MATCH_GRAVITY, how far a candidate may tilt the device from that.
 *
 * @note The `pose` and `score` parameters are only written on success.
 */
bool
correspondence_search_find_one_pose(struct correspondence_search *cs,
                                    const struct t_constellation_search_model *model,
                                    enum correspondence_search_flags search_flags,
                                    struct xrt_pose *inout_pose,
                                    const struct xrt_vec3 *pos_error_thresh,
                                    const struct xrt_vec3 *rot_error_thresh,
                                    const struct xrt_vec3 *gravity_vector,
                                    const struct xrt_vec3 *device_gravity,
                                    float gravity_tolerance_rad,
                                    struct pose_metrics *out_score);

#ifdef __cplusplus
}
#endif
