# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesMulticamAIResultOnly)


add_executable(rs-multicam-ai-result-only rs-multicam-ai-result-only.cpp ../example.hpp)
set_property(TARGET rs-multicam-ai-result-only PROPERTY CXX_STANDARD 11)
include_directories(../)
target_link_libraries(rs-multicam-ai-result-only ${DEPENDENCIES})
set_target_properties (rs-multicam-ai-result-only PROPERTIES FOLDER Examples)
install(TARGETS rs-multicam-ai-result-only RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

