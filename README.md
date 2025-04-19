# ORB_SLAM3_foxy
Tested with ROS2 Foxy and Humble

## Dependencies
```Bash
sudo apt install -y ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs
pip3 install --upgrade "numpy<2"
```

ORB_SLAM3:

Modify the `FindORB_SLAM3.cmake` file in the `CMakeModules` directory to point to the correct path of the ORB_SLAM3 library.

```CMake
# To help the search ORB_SLAM3_ROOT_DIR environment variable as the path to ORB_SLAM3 root folder
#  e.g. `set( ORB_SLAM3_ROOT_DIR "Path/to/ORB_SLAM3") `
set(ORB_SLAM3_ROOT_DIR "~/ORB_SLAM3_OpenCV4")
```

## Usage

```Bash
ros2 run orbslam3_ros2 fire_detection.py
## change the path according to the username
ros2 run orbslam3_ros2 fire_localization /home/qin/ORB_SLAM3_OpenCV4/Vocabulary/ORBvoc.txt /home/qin/humble_ws/src/orbslam3_ros2/config/M300.yaml
ros2 run orbslam3_ros2 fire_localization /home/nvidia/ORB_SLAM3_OpenCV4/Vocabulary/ORBvoc.txt /home/nvidia/foxy_ws/src/orbslam3_ros2/config/M300.yaml
```

To visualize the results of ORB-SLAM3 using RViz
```bash
roslaunch '/home/qin/ORB_SLAM3_OpenCV4/Examples_old/ROS/ORB_SLAM3/play_bag_and_rviz.launch'
```