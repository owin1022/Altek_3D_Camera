# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesMulticamAI)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-multicam-ai rs-multicam-ai.cpp ../example.hpp)
    set_property(TARGET rs-multicam-ai PROPERTY CXX_STANDARD 11)
    include_directories(../)
    target_link_libraries(rs-multicam-ai ${DEPENDENCIES})
    set_target_properties (rs-multicam-ai PROPERTIES FOLDER Examples)
    install(TARGETS rs-multicam-ai RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
