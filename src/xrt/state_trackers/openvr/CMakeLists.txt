# Copyright 2026, Beyley Cardellio
# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(
	st_openvr STATIC
	openvr_devices.cpp
	openvr_devices.hpp
	openvr_events.cpp
	openvr_events.hpp
	openvr_negotiate.cpp
	common/openvr_error.cpp
	common/openvr_error.hpp
	common/openvr_logger.cpp
	common/openvr_logger.hpp
	common/openvr_math.cpp
	common/openvr_math.hpp
	compositor/openvr_compositor.cpp
	compositor/openvr_compositor.hpp
	interfaces/XRTVRChaperone.cpp
	interfaces/XRTVRChaperone.hpp
	interfaces/XRTVRChaperoneSetup.cpp
	interfaces/XRTVRChaperoneSetup.hpp
	interfaces/XRTVRClientCore.cpp
	interfaces/XRTVRClientCore.hpp
	interfaces/XRTVRCompositor.cpp
	interfaces/XRTVRCompositor.hpp
	interfaces/XRTVRRenderModels.cpp
	interfaces/XRTVRRenderModels.hpp
	interfaces/XRTVRSystem.cpp
	interfaces/XRTVRSystem.hpp
	interfaces/old/XRTVRChaperone_003.hpp
	interfaces/old/XRTVRChaperoneSetup_005.hpp
	interfaces/old/XRTVRCompositor_022.hpp
	interfaces/old/XRTVRCompositor_024.hpp
	interfaces/old/XRTVRCompositor_026.hpp
	interfaces/old/XRTVRCompositor_027.hpp
	interfaces/old/XRTVRCompositor_028.hpp
	interfaces/old/XRTVRSystem_019.hpp
	interfaces/old/XRTVRSystem_020.hpp
	interfaces/old/XRTVRSystem_021.hpp
	interfaces/old/XRTVRSystem_023.hpp
	)

if(XRT_HAVE_VULKAN)
	target_sources(st_openvr PRIVATE compositor/openvr_compositor_vulkan.cpp)

	target_link_libraries(st_openvr PRIVATE aux_vk aux_render aux_shaders aux_render_cache)
endif()

target_link_libraries(
	st_openvr
	PRIVATE xrt-interfaces xrt-external-openvr
	PUBLIC aux_os aux_glue comp_client
	)

target_include_directories(st_openvr PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/)

target_include_directories(
	st_openvr
	PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
	INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..
	)
