# Copyright 2026, Beyley Cardellio
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0

add_library(
	constellation STATIC
	camera_model.h
	constellation_debug_scribble.cpp
	correspondence_search.c
	correspondence_search.h
	image_point_sort.cpp
	image_point_sort.h
	led_search_model.c
	led_search_model.h
	level_watch.cpp
	level_watch.hpp
	loose_part.cpp
	loose_part.hpp
	pose_metrics.c
	pose_metrics.h
	t_constellation_tracker_dataset.cpp
	t_constellation_tracker_dataset.hpp
	t_constellation_tracker_internal.hpp
	t_constellation_tracker.cpp
	t_constellation_tracker.h
	t_led_sync_refinement.h
	t_rift_blobwatch.c
	t_rift_blobwatch.h
	lambdatwist/cubic.h
	lambdatwist/eig3x3known0.h
	lambdatwist/lambdatwist_p3p.h
	lambdatwist/mat.h
	lambdatwist/quadratic.h
	lambdatwist/refine_lambda.h
	lambdatwist/vec.h
	led_sync/t_led_sync_refinement.c
	led_sync/pssense_gray_code.h
	led_sync/led_sync_internal.h
	optimizer/imu_preintegration.hpp
	optimizer/internal_math.cpp
	optimizer/internal_math.hpp
	optimizer/math.hpp
	optimizer/pose_optimize.cpp
	optimizer/pose_optimize.hpp
	optimizer/ransac.cpp
	optimizer/ransac.hpp
	optimizer/rig_calibration.cpp
	optimizer/rig_calibration.hpp
	optimizer/sensor_fusion.cpp
	optimizer/sensor_fusion.hpp
	)
target_link_libraries(
	constellation
	PRIVATE
		xrt-interfaces
		xrt-external-bigceres
		xrt-optimized-math
		tracking-includes
		aux_os
		aux_util
		aux_util_scribble
		aux_math
		aux_tracking
		Eigen3::Eigen
	)

if(XRT_FEATURE_RERUN)
	target_link_libraries(constellation PRIVATE rerun_sdk)

	target_sources(
		constellation
		PRIVATE
			constellation_rerun_common.hpp
			constellation_tracker_rerun.cpp
			constellation_tracker_rerun.hpp
			constellation_tracker_rerun_blobwatch.h
			optimizer/rig_calibration_rerun.cpp
			optimizer/rig_calibration_rerun.hpp
			optimizer/sensor_fusion_rerun.cpp
			optimizer/sensor_fusion_rerun.hpp
		)
endif()
