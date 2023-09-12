# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(AlTest)

find_package(OpenCV REQUIRED)
add_executable(al-test al-test.cpp ../example.hpp ../../third-party/imgui/imgui.cpp ../../third-party/imgui/imgui_draw.cpp ../../third-party/imgui/imgui_impl_glfw.cpp)
set_property(TARGET al-test PROPERTY CXX_STANDARD 11)
target_link_libraries(al-test ${DEPENDENCIES} ${OpenCV_LIBS} realsense2-gl )
include_directories(../../common ../../third-party/imgui ../../examples ${OpenCV_INCLUDE_DIRS})
set_target_properties (al-test PROPERTIES FOLDER Examples)
install(TARGETS al-test RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
