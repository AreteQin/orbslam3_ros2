cmake_minimum_required(VERSION 3.5)
project(ORB_SLAM3_foxy)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

include_directories(
        include
        ${ORB_SLAM3_ROOT_DIR}/include
        ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
        ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

link_directories(
        include
)

add_executable(fire_localization
        src/fire_localization.cpp
)
ament_target_dependencies(fire_localization
        rclcpp
        sensor_msgs
        message_filters
        vision_msgs
        image_transport
        cv_bridge
        ORB_SLAM3
        Pangolin)

install(TARGETS fire_localization
        DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

