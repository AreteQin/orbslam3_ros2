# ORB_SLAM3_foxy

## Dependencies
```Bash
sudo apt install ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs
```

## ORB_SLAM3
Add the following line to the .bashrc file
```Bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ORB_SLAM3_Ubuntu_20/lib:~/Pangolin/build/:~/ORB_SLAM3_Ubuntu_20/Thirdparty/DBoW2/lib:~/ORB_SLAM3_Ubuntu_20/Thirdparty/g2o/lib
```

## Usage

```Bash
ros2 run orb_slam3_foxy fire_detection.py
## change the path according to the username
ros2 run orb_slam3_foxy fire_localization /home/qin/ORB_SLAM3_Ubuntu_20/Vocabulary/ORBvoc.txt /home/qin/foxy_ws/src/ORB_SLAM3_foxy/config/M300.yaml
```