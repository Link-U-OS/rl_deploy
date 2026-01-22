# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

include(FetchContent)

message(STATUS "get aimrt ...")

set(aimrt_DOWNLOAD_URL
    "https://github.com/AimRT/AimRT/archive/refs/tags/v1.2.0.tar.gz"
    CACHE STRING "")

set(aimrt_PATCH_DIR "${CMAKE_CURRENT_LIST_DIR}/patches")
set(aimrt_PATCH_SCRIPT "${aimrt_PATCH_DIR}/ApplyAimRTPatches.cmake")

if(aimrt_LOCAL_SOURCE)
  FetchContent_Declare(
    aimrt
    SOURCE_DIR ${aimrt_LOCAL_SOURCE}
    PATCH_COMMAND ${CMAKE_COMMAND} -P ${aimrt_PATCH_SCRIPT}
    OVERRIDE_FIND_PACKAGE)
else()
  FetchContent_Declare(
    aimrt
    URL ${aimrt_DOWNLOAD_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    PATCH_COMMAND ${CMAKE_COMMAND} -P ${aimrt_PATCH_SCRIPT}
    OVERRIDE_FIND_PACKAGE)
endif()

# Wrap it in a function to restrict the scope of the variables
function(get_aimrt)
  FetchContent_GetProperties(aimrt)
  if(NOT aimrt_POPULATED)
    set(AIMRT_BUILD_RUNTIME ON)
    set(AIMRT_BUILD_WITH_PROTOBUF ON)
    set(AIMRT_BUILD_WITH_ROS2 ON)
    set(AIMRT_BUILD_ROS2_PLUGIN ON)
    set(AIMRT_BUILD_ICEORYX_PLUGIN ON)

    set(CMAKE_POLICY_VERSION_MINIMUM
        3.5
        CACHE STRING "Minimum CMake policy version" FORCE)
    set(YAML_CPP_BUILD_TESTS
        OFF
        CACHE BOOL "Disable yaml-cpp tests" FORCE)
    FetchContent_MakeAvailable(aimrt)
  endif()
endfunction()

get_aimrt()
