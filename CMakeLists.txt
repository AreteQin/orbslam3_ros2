cmake_minimum_required(VERSION 3.5)
project(orbslam3_ros2)

# check ros2 version
if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "ROS2 Distribution: $ENV{ROS_DISTRO}")
else()
    message(FATAL_ERROR "ROS_DISTRO is not defined. Be sure to source your ROS2 setup file!")
endif()
if($ENV{ROS_DISTRO} STREQUAL "humble")
    add_definitions(-DROS_DISTRO_HUMBLE)
elseif($ENV{ROS_DISTRO} STREQUAL "foxy")
    add_definitions(-DROS_DISTRO_FOXY)
else()
    message(FATAL_ERROR "Unsupported ROS distribution")
endif()

# You should set the PYTHONPATH to your own python site-packages path
IF ($ENV{ROS_DISTRO} STREQUAL "foxy")
    set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")
ENDIF ()

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(Glog REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
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

add_executable(orbslam3_ros2
        src/orbslam3_ros2.cpp
        src/tools.h
)
ament_target_dependencies(orbslam3_ros2
        rclcpp
        cv_bridge
        image_transport
        sensor_msgs
        geometry_msgs
        visualization_msgs
        tf2_ros
        tf2
        message_filters
        OpenCV
        vision_msgs
        ORB_SLAM3
        Pangolin
        tf2_geometry_msgs
)
target_link_libraries(orbslam3_ros2
        ${GLOG_LIBRARIES}
)

install(TARGETS
        orbslam3_ros2
        DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

