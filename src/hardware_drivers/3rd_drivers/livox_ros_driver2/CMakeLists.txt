cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2)

#---------------------------------------------------------------------------------------
# Standards and Flags
#---------------------------------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Print version info
include(cmake/version.cmake)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

add_definitions(-DBUILDING_ROS2)

#---------------------------------------------------------------------------------------
# Find Dependencies
#---------------------------------------------------------------------------------------
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Check for APR (Used for some internal Livox timing)
find_package(PkgConfig)
pkg_check_modules(APR apr-1)

#---------------------------------------------------------------------------------------
# Message Generation (The Jazzy Way)
#---------------------------------------------------------------------------------------
set(LIVOX_INTERFACES livox_interfaces2)

rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

#---------------------------------------------------------------------------------------
# External Libraries (Livox SDK)
#---------------------------------------------------------------------------------------
# Search for the SDK you installed in /usr/local/lib
find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)

find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES "livox_lidar_api.h" "livox_lidar_def.h"
  REQUIRED)

#---------------------------------------------------------------------------------------
# Build Library
#---------------------------------------------------------------------------------------
# We use ament_auto_add_library to handle the base dependencies
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp
  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp
  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp
  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

#---------------------------------------------------------------------------------------
# Jazzy Interface Fix
#---------------------------------------------------------------------------------------

# 1. Manually set the include directory for generated headers to prevent NOTFOUND
set(LIVOX_INTERFACES_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")

# 2. Link to the typesupport target (Correct for Humble and Jazzy)
# This automatically handles the internal include paths for your custom messages
rosidl_get_typesupport_target(cpp_typesupport_target ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")

# 3. Apply Include Directories
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}
  3rdparty
  src
)

# 4. Link Libraries
target_link_libraries(${PROJECT_NAME}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${cpp_typesupport_target}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

#---------------------------------------------------------------------------------------
# Node Registration & Install
#---------------------------------------------------------------------------------------
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch_ROS2
)