cmake_minimum_required(VERSION 3.18)
project(aimrl_sdk LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "aimrl_sdk only supports Linux (CMAKE_SYSTEM_NAME=${CMAKE_SYSTEM_NAME}).")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

include(src/aimrl_sdk/cmake/GetAimRT.cmake)

add_subdirectory(src/aimrl_sdk/protocols/joint_msgs)

find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(_bindings src/aimrl_sdk/cpp/bindings.cpp src/aimrl_sdk/cpp/core.cpp)

find_package(sensor_msgs REQUIRED)

target_link_libraries(
    _bindings
    PUBLIC joint_msgs::joint_msgs__rosidl_generator_cpp # ROS library for message types
         joint_msgs::joint_msgs__rosidl_typesupport_cpp
         joint_msgs::joint_msgs__rosidl_typesupport_fastrtps_cpp
         joint_msgs::joint_msgs__rosidl_typesupport_introspection_cpp
         sensor_msgs::sensor_msgs__rosidl_generator_cpp
         sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
         sensor_msgs::sensor_msgs__rosidl_typesupport_fastrtps_cpp
         sensor_msgs::sensor_msgs__rosidl_typesupport_introspection_cpp
         aimrt::interface::aimrt_module_ros2_interface
         aimrt::runtime::core)

set_target_properties(_bindings PROPERTIES INSTALL_RPATH "$ORIGIN")

set(AIMRL_SDK_CONFIG_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/aimrl_sdk/config")
set(AIMRL_SDK_CONFIG_FILE "${AIMRL_SDK_CONFIG_SRC_DIR}/aimrt_ros2_backend.yaml")

add_custom_command(
    TARGET _bindings
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:_bindings>/config"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${AIMRL_SDK_CONFIG_FILE}"
            "$<TARGET_FILE_DIR:_bindings>/config/aimrt_ros2_backend.yaml")

install(DIRECTORY "${AIMRL_SDK_CONFIG_SRC_DIR}/" DESTINATION aimrl_sdk/config)

install(TARGETS _bindings LIBRARY DESTINATION aimrl_sdk)

set(JOINT_MSGS_INSTALL_LIBS
    joint_msgs__rosidl_generator_c
    joint_msgs__rosidl_typesupport_c
    joint_msgs__rosidl_typesupport_cpp
    joint_msgs__rosidl_typesupport_fastrtps_c
    joint_msgs__rosidl_typesupport_fastrtps_cpp
    joint_msgs__rosidl_typesupport_introspection_c
    joint_msgs__rosidl_typesupport_introspection_cpp)

foreach(lib ${JOINT_MSGS_INSTALL_LIBS})
  if(TARGET ${lib})
    install(TARGETS ${lib} LIBRARY DESTINATION aimrl_sdk)
  endif()
endforeach()

set(AIMRT_ROS2_PLUGIN_TARGET aimrt_plugins_ros2_plugin)
if(TARGET ${AIMRT_ROS2_PLUGIN_TARGET})
  set_target_properties(${AIMRT_ROS2_PLUGIN_TARGET} PROPERTIES INSTALL_RPATH "$ORIGIN")
  install(TARGETS ${AIMRT_ROS2_PLUGIN_TARGET} LIBRARY DESTINATION aimrl_sdk)
endif()

set(AIMRT_ICEORYX_PLUGIN_TARGET aimrt_plugins_iceoryx_plugin)
if(TARGET ${AIMRT_ICEORYX_PLUGIN_TARGET})
  set_target_properties(${AIMRT_ICEORYX_PLUGIN_TARGET} PROPERTIES INSTALL_RPATH "$ORIGIN")
  install(TARGETS ${AIMRT_ICEORYX_PLUGIN_TARGET} LIBRARY DESTINATION aimrl_sdk)
endif()

set(ROS2_PLUGIN_PROTO_INSTALL_LIBS
    ros2_plugin_proto__rosidl_generator_c
    ros2_plugin_proto__rosidl_typesupport_c
    ros2_plugin_proto__rosidl_typesupport_cpp
    ros2_plugin_proto__rosidl_typesupport_fastrtps_c
    ros2_plugin_proto__rosidl_typesupport_fastrtps_cpp
    ros2_plugin_proto__rosidl_typesupport_introspection_c
    ros2_plugin_proto__rosidl_typesupport_introspection_cpp)

foreach(lib ${ROS2_PLUGIN_PROTO_INSTALL_LIBS})
  if(TARGET ${lib})
    set_target_properties(${lib} PROPERTIES INSTALL_RPATH "$ORIGIN")
    install(TARGETS ${lib} LIBRARY DESTINATION aimrl_sdk)
  endif()
endforeach()

if(TARGET aimrt_protocols_plugins_ros2_plugin_proto_ts)
  set_target_properties(aimrt_protocols_plugins_ros2_plugin_proto_ts PROPERTIES INSTALL_RPATH "$ORIGIN")
  install(TARGETS aimrt_protocols_plugins_ros2_plugin_proto_ts LIBRARY DESTINATION aimrl_sdk)
endif()
