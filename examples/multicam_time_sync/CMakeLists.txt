# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesMulticamTimeSync)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-multicam-time-sync rs-multicam-time-sync.cpp ../example.hpp)
    set_property(TARGET rs-multicam-time-sync PROPERTY CXX_STANDARD 11)
    include_directories(../)
    target_link_libraries(rs-multicam-time-sync ${DEPENDENCIES})
    set_target_properties (rs-multicam-time-sync PROPERTIES FOLDER Examples)
    install(TARGETS rs-multicam-time-sync RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
