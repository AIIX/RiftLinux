# Copyright 2020-2021, Collabora, Ltd.
# Copyright 2026, NVIDIA CORPORATION.
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0

add_subdirectory(steamvr_bindings)

add_library(
	st_ovrd STATIC ovrd_driver.cpp ovrd_direct_mode.cpp ovrd_direct_mode.hpp ovrd_socket.cpp ovrd_socket.hpp ovrd_interface.h
	)

target_include_directories(st_ovrd INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
	st_ovrd PRIVATE xrt-interfaces xrt-external-openvr aux_math aux_util ovrd_generated_bindings ${CMAKE_DL_LIBS}
	)
# Direct mode's frame capture reads Monado's own swapchain images (ovrd_direct_mode.cpp).
if(TARGET comp_util AND TARGET aux_vk)
	target_link_libraries(st_ovrd PRIVATE comp_util aux_vk)
endif()
