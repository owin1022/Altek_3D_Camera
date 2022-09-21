# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2021 altek Corporation. All Rights Reserved.
target_sources(${LRS_TARGET}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/alOpenCVK.h"
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)
	find_package(Git)
		if(GIT_FOUND)
			execute_process(
			  COMMAND ${GIT_EXECUTABLE} describe --tags
			  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
			  OUTPUT_VARIABLE "AL_GIT_BUILD_VERSION"
			  ERROR_QUIET
			  OUTPUT_STRIP_TRAILING_WHITESPACE)
			message( STATUS "Git version: ${AL_GIT_BUILD_VERSION}" )
		else(GIT_FOUND)
			set(AL_GIT_BUILD_VERSION " " )
		endif(GIT_FOUND)

	add_definitions("-DAL_BUILD_VERSION=${AL_GIT_BUILD_VERSION}")
		
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/altek/alVersion.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/altek/alVersion.h @ONLY)

endif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/.git)