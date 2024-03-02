# ORB_SLAM3_foxy

## Dependencies

### ORB_SLAM3
Add the following line to the .bashrc file
```Bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ORB_SLAM3_Ubuntu_20/lib:~/Pangolin/build/:~/ORB_SLAM3_Ubuntu_20/Thirdparty/DBoW2/lib:~/ORB_SLAM3_Ubuntu_20/Thirdparty/g2o/lib
```

## Usage

```Bash
ros2 run orb_slam3_foxy fire_detection.py
ros2 run orb_slam3_foxy fire_localization ~/ORB_SLAM3_Ubuntu_20/Vocabulary/ORBvoc.txt ~/foxy_ws/src/orb_slam3_foxy/config/M300.yaml
```