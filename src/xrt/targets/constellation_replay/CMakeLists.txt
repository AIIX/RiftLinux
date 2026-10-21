# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0

# Headless replay of constellation tracker recordings, uses the tracker's internals.
add_executable(monado-constellation-replay constellation_replay.cpp)

target_include_directories(monado-constellation-replay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../tracking)

target_link_libraries(
	monado-constellation-replay
	PRIVATE
		constellation
		xrt-external-bigceres
		xrt-interfaces
		tracking-includes
		aux_os
		aux_util
		aux_math
		aux_tracking
		Eigen3::Eigen
	)
