# Copyright 2019-2025, Collabora, Ltd.
# Copyright 2025-2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

# Binding generation: pass filename to generate
# Need to use ${CMAKE_COMMAND} to be able to set environment with -e
# Need to use $<TARGET_FILE:${PYTHON_EXECUTABLE}> to unwrap the cmake target to be able to run with ${CMAKE_COMMAND}
function(oxr_bindings_gen output)
	add_custom_command(
		OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${output}"
		COMMAND
			${CMAKE_COMMAND} -E env
			"PYTHONPATH=$<SHELL_PATH:${AUX_BINDINGS_DIR};$ENV{PYTHONPATH}>"
			$<TARGET_FILE:${PYTHON_EXECUTABLE}>
			${CMAKE_CURRENT_SOURCE_DIR}/oxr_bindings.py
			${AUX_BINDINGS_DIR}/bindings.json "${CMAKE_CURRENT_BINARY_DIR}/${output}"
		COMMAND_EXPAND_LISTS VERBATIM
		DEPENDS
			${CMAKE_CURRENT_SOURCE_DIR}/oxr_bindings.py
			${CMAKE_CURRENT_SOURCE_DIR}/oxr_generated_bindings.c.template
			${CMAKE_CURRENT_SOURCE_DIR}/oxr_generated_bindings.h.template
			${AUX_BINDINGS_DIR}/bindings.json
		COMMENT "Generating ${output}"
		)
endfunction()

oxr_bindings_gen(oxr_generated_bindings.h)
oxr_bindings_gen(oxr_generated_bindings.c)

add_custom_target(
	generate_oxr_bindings DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/oxr_generated_bindings.h"
				      "${CMAKE_CURRENT_BINARY_DIR}/oxr_generated_bindings.c"
	)
