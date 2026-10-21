# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0

# A native OpenXR test application for HDR layers (RiftLinux): loads the runtime library directly.
add_executable(riftlinux-openxr-hdr-test riftlinux_openxr_hdr_test.cpp)
target_link_libraries(
	riftlinux-openxr-hdr-test PRIVATE xrt-external-openxr xrt-external-vulkan-headers ${CMAKE_DL_LIBS}
	)
