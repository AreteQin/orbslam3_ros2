# ORB_SLAM3_foxy

## Dependencies
```Bash
sudo apt install -y ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs
pip3 install --upgrade "numpy<2"
```

G2O

## ORB_SLAM3
Add the following line to the .bashrc file
```Bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ORB_SLAM3_Ubuntu_20/lib:~/Pangolin/build/:~/ORB_SLAM3_Ubuntu_20/Thirdparty/DBoW2/lib:~/ORB_SLAM3_Ubuntu_20/Thirdparty/g2o/lib
```

## Usage

```Bash
ros2 run orbslam3_ros2 fire_detection.py
## change the path according to the username
ros2 run orbslam3_ros2 fire_localization /home/qin/ORB_SLAM3_Ubuntu_20/Vocabulary/ORBvoc.txt /home/qin/humble_ws/src/orbslam3_ros2/config/M300.yaml
ros2 run orbslam3_ros2 fire_localization /home/nvidia/ORB_SLAM3_Ubuntu_20/Vocabulary/ORBvoc.txt /home/nvidia/foxy_ws/src/orbslam3_ros2/config/M300.yaml
```