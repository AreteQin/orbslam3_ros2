//
// Created by qin on 15/04/25.
//

#ifndef TOOLS_H
#define TOOLS_H
// Helper function to extract the x-coordinate from the bounding box center
inline double getBoxCenterX(const vision_msgs::msg::Detection2D box) {
#ifdef ROS_DISTRO_HUMBLE
    return box.bbox.center.position.x;
#elif defined(ROS_DISTRO_FOXY)
    return box.bbox.center.x;
#else
#error "Unsupported ROS distribution"
#endif
}
inline double getBoxCenterY(const vision_msgs::msg::Detection2D box) {
#ifdef ROS_DISTRO_HUMBLE
    return box.bbox.center.position.y;
#elif defined(ROS_DISTRO_FOXY)
    return box.bbox.center.y;
#else
#error "Unsupported ROS distribution"
#endif
}
#endif //TOOLS_H
