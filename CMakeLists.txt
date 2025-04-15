cmake_minimum_required(VERSION 3.5)
project(orbslam3_ros2)

# You should set the PYTHONPATH to your own python site-packages path
#set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

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

# enable drawing plot
add_subdirectory(third_party/matplotlib-cpp)
include_directories(${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp)
# Find Python headers
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

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
target_link_libraries(fire_localization
        ${GLOG_LIBRARIES}
)

add_executable(bounding_box_visualization
        src/bounding_box_visualization.cpp
)
ament_target_dependencies(bounding_box_visualization
        rclcpp
        sensor_msgs
        message_filters
        vision_msgs
        image_transport
        cv_bridge
        OpenCV
)

add_executable(global_localization
        src/global_localization.cpp
)

ament_target_dependencies(global_localization
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
        Pangolin
)

target_link_libraries(global_localization
        matplotlib_cpp
        ${GLOG_LIBRARIES}
)

install(TARGETS
        fire_localization
        bounding_box_visualization
        global_localization
        DESTINATION lib/${PROJECT_NAME}
)

# Install python scripts.
install(PROGRAMS
        scripts/fire_detection.py
        DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

