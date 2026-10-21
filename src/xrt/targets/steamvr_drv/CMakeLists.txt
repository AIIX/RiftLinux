# Copyright 2020-2021, Collabora, Ltd.
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0

get_property(AUX_BINDINGS_DIR GLOBAL PROPERTY AUX_BINDINGS_DIR_PROP)
set(INPUT_PROFILES_INPUT_DIR ${AUX_BINDINGS_DIR})
set(INPUT_PROFILES_OUTPUT_DIR "${PROJECT_BINARY_DIR}/steamvr-monado/resources/input/")

add_custom_command(
	OUTPUT "${INPUT_PROFILES_OUTPUT_DIR}"
	COMMAND
		${PYTHON_EXECUTABLE} ${INPUT_PROFILES_INPUT_DIR}/steamvr_profiles.py
		${INPUT_PROFILES_INPUT_DIR}/bindings.json "${INPUT_PROFILES_OUTPUT_DIR}" "-s"
	DEPENDS ${INPUT_PROFILES_INPUT_DIR}/bindings.py ${INPUT_PROFILES_INPUT_DIR}/bindings.json
	COMMENT "Generating SteamVR input profiles to ${INPUT_PROFILES_OUTPUT_DIR}"
	)
add_custom_target(
	steamvr_generated_input_profiles
	DEPENDS "${INPUT_PROFILES_OUTPUT_DIR}"
	COMMENT "Generating SteamVR input profiles"
	)

add_library(driver_monado MODULE main.c)
add_dependencies(driver_monado steamvr_generated_input_profiles)

target_link_libraries(
	driver_monado
	PRIVATE
		xrt-external-openvr
		aux_util
		st_ovrd
		st_prober
		target_lists
	)

# With Monado's compositor linked in when built, for driver direct mode (ovrd_direct_mode.hpp); it is created only
# when direct mode is switched on, so without it the driver runs as with the instance that has none.
if(TARGET target_instance)
	target_link_libraries(driver_monado PRIVATE target_instance)
else()
	target_link_libraries(driver_monado PRIVATE target_instance_no_comp)
endif()

# meta data that the steamvr plugin needs in the base directory of the steamvr plugin
file(COPY driver.vrdrivermanifest DESTINATION ${PROJECT_BINARY_DIR}/steamvr-monado)
file(COPY resources DESTINATION ${PROJECT_BINARY_DIR}/steamvr-monado)

# determine the output directory for the steamvr plugin
if(WIN32)
	# FIXME need to account for different architectures
	if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/win64")
	else()
		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/win32")
	endif()
elseif(APPLE)
	if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/osx64")
	else()
		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/osx32")
	endif()
elseif(NOT ANDROID)
	if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/linux64")
	else()
		set(PLUGINDIR "${PROJECT_BINARY_DIR}/steamvr-monado/bin/linux32")
	endif()
endif()

# message("SteamVR plugin path: ${PLUGINDIR}")
set_target_properties(driver_monado PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PLUGINDIR}")

# don't add lib prefix to driver_monado.so
set_target_properties(driver_monado PROPERTIES PREFIX "")

# The notices shown in the headset (a sensor moved, placed again): the driver hands them to this small SteamVR overlay
# app next to it, see riftlinux_notices.cpp; the panels it shows are in resources/notices.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	add_executable(riftlinux-notices riftlinux_notices.cpp)
	target_link_libraries(riftlinux-notices PRIVATE xrt-external-openvr xrt-external-stb aux_riftlinux ${CMAKE_DL_LIBS})
	set_target_properties(riftlinux-notices PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PLUGINDIR}")
	add_dependencies(driver_monado riftlinux-notices)

	# QA: what SteamVR delivers to an application from the controllers (buttons, finger curls), see
	# riftlinux_input_check.cpp; its actions are in resources/input-check.
	add_executable(riftlinux-input-check riftlinux_input_check.cpp)
	target_link_libraries(riftlinux-input-check PRIVATE xrt-external-openvr ${CMAKE_DL_LIBS})
	set_target_properties(riftlinux-input-check PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PLUGINDIR}")
	add_dependencies(driver_monado riftlinux-input-check)

	# Tests: a SteamVR scene application as slow as asked with a pattern moving at a known speed, for the frame lock
	# and motion smoothing, see riftlinux_pace_test.cpp.
	add_executable(riftlinux-pace-test riftlinux_pace_test.cpp)
	target_link_libraries(riftlinux-pace-test PRIVATE xrt-external-openvr ${CMAKE_DL_LIBS})
	set_target_properties(riftlinux-pace-test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PLUGINDIR}")
	add_dependencies(driver_monado riftlinux-pace-test)
endif()

# Started from Steam, SteamVR runs in Valve's runtime container, which lacks libraries the driver needs (OpenCV, Ceres,
# hidapi, ...): "headset not detected". Started outside it, games in the container share no frames with it: a grey
# screen. So the driver finds its own copies next to it first (a package puts them there); an old-style RPATH, as it
# also applies to the libraries those libraries load.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	set_target_properties(driver_monado PROPERTIES BUILD_RPATH "\$ORIGIN" INSTALL_RPATH "\$ORIGIN")
	target_link_options(driver_monado PRIVATE "-Wl,--disable-new-dtags")
endif()

install(
	DIRECTORY "${PROJECT_BINARY_DIR}/steamvr-monado"
	DESTINATION "${CMAKE_INSTALL_PREFIX}/share"
	)
