// Copyright 2014-2015 Philipp Zabel
// Copyright 2019-2023 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Constellation tracker's camera model storage.
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author Jan Schmidt <jan@centricular.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "tracking/t_camera_models.h"


struct camera_model
{
	//! Frame width
	int width;
	//! Frame height
	int height;

	//! The true calibration of the camera
	struct t_camera_model_params calib_true;
	//! An identity calibration model
	struct t_camera_model_params calib_pinhole;
};
