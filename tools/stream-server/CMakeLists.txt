# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseToolsStreamServer)

add_executable(rs-stream-server rs-stream-server.cpp)
set_property(TARGET rs-stream-server PROPERTY CXX_STANDARD 11)
if(WIN32 OR ANDROID)
    target_link_libraries(rs-stream-server ${DEPENDENCIES})
else()
    target_link_libraries(rs-stream-server -lpthread ${DEPENDENCIES})
endif()
include_directories(rs-terminal ../../common ../../third-party ../../third-party/tclap/include)
set_target_properties (rs-stream-server PROPERTIES
    FOLDER Tools
)

install(
    TARGETS

    rs-stream-server

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_BINDIR}
)

