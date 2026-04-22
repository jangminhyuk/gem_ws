cmake_minimum_required(VERSION 3.5)
project(lucid_vision_driver)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_CXX_FLAGS "-g3 -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)

# --- EXACT PATHS FOR YOUR SYSTEM ---
set(arena_sdk_INCLUDES 
    "/opt/ArenaSDK/include"
    "/opt/ArenaSDK/include/Arena"
    "/opt/ArenaSDK/GenICam/library/CPP/include"
)

set(arena_sdk_LIBRARIES 
    "/opt/ArenaSDK/lib64/libarena.so"
    "/opt/ArenaSDK/lib64/libsave.so"
    "/opt/ArenaSDK/lib64/libgentl.so"
)

add_library(arena_camera_node SHARED
    src/arena_camera_node.cpp
    src/arena_camera.cpp
    src/arena_cameras_handler.cpp
)

target_include_directories(arena_camera_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${arena_sdk_INCLUDES}
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(arena_camera_node
    rclcpp
    rclcpp_components
    sensor_msgs
    cv_bridge
    OpenCV
    camera_info_manager
    image_geometry
    image_transport
)

target_link_libraries(arena_camera_node
    ${arena_sdk_LIBRARIES}
)

rclcpp_components_register_node(arena_camera_node
    PLUGIN "ArenaCameraNode"
    EXECUTABLE arena_camera_node_exe
)

install(TARGETS arena_camera_node
    EXPORT export_lucid_vision_driver
    DESTINATION lib
)

install(DIRECTORY include/ launch config param
    DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_lucid_vision_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp 
    rclcpp_components 
    sensor_msgs 
    cv_bridge 
    OpenCV 
    camera_info_manager 
    image_geometry 
    image_transport
)
ament_package()