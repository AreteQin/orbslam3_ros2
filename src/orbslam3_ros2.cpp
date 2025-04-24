//
// Created by qin on 2/29/24.
//

#include <iostream>
#include <chrono>
#include <memory>
#include <fstream>
#include <limits>
#include <vector>
#include <rclcpp/rclcpp.hpp>

// OpenCV and cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// ROS 2 message types
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Image transport (only used for initialization here)
#include <image_transport/image_transport.hpp>

// TF2 (transforms)
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// For manually filling PointCloud2 data
#include <sensor_msgs/point_cloud2_iterator.hpp>

// ORB_SLAM3 and map includes (make sure these are in your include path)
#include "MapPoint.h"
#include "System.h"
#include "Map.h"
#include "tools.h"

using namespace std;

// Global drone marker variable.
visualization_msgs::msg::Marker drone_model;

/**
 * @brief Converts a vector of ORB_SLAM3 MapPoints into a sensor_msgs::msg::PointCloud2 message.
 */
sensor_msgs::msg::PointCloud2 convertMapPointsToPointCloud2(
    const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
    const std_msgs::msg::Header& header);

/**
 * @brief Callback for synchronized image and detection messages.
 */
void ImageBoxesCallback(ORB_SLAM3::System* pSLAM,
                        tf2_ros::TransformBroadcaster* tf_broadcaster,
                        const std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> point_cloud_pub,
                        const std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> marker_pub,
                        const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    // Convert image to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("fire_localization"), "cv_bridge exception: %s", e.what());
        return;
    }

    double timestamp = rclcpp::Time(cv_ptr->header.stamp).seconds();
    pSLAM->TrackMonocular(cv_ptr->image, timestamp);
    ORB_SLAM3::Map* current_map = pSLAM->GetAtlas()->GetCurrentMap();
    if (!current_map)
        return;

    // Publish the camera pose as a TF transform.
    geometry_msgs::msg::TransformStamped camera_pose_msg;
    camera_pose_msg.header.stamp = msg->header.stamp;
    camera_pose_msg.header.frame_id = "map";
    camera_pose_msg.child_frame_id = "H20T";
    Eigen::Matrix4f Twc = pSLAM->GetCurrentPose().inverse();
    camera_pose_msg.transform.translation.x = Twc(0, 3);
    camera_pose_msg.transform.translation.y = Twc(1, 3);
    camera_pose_msg.transform.translation.z = Twc(2, 3);
    Eigen::Quaternionf q(Twc.block<3, 3>(0, 0));
    camera_pose_msg.transform.rotation.x = q.x();
    camera_pose_msg.transform.rotation.y = q.y();
    camera_pose_msg.transform.rotation.z = q.z();
    camera_pose_msg.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(camera_pose_msg);

    // Broadcast a static transform from "world" to "map".
    geometry_msgs::msg::TransformStamped map_transform;
    map_transform.header.stamp = msg->header.stamp;
    map_transform.header.frame_id = "world";
    map_transform.child_frame_id = "map";
    map_transform.transform.translation.x = 0.0;
    map_transform.transform.translation.y = 0.0;
    map_transform.transform.translation.z = 0.1;
    tf2::Quaternion q_map;
    q_map.setRPY(-2.1, 0.05, 0.0);
    map_transform.transform.rotation.x = q_map.x();
    map_transform.transform.rotation.y = q_map.y();
    map_transform.transform.rotation.z = q_map.z();
    map_transform.transform.rotation.w = q_map.w();
    tf_broadcaster->sendTransform(map_transform);

    // Publish the map points as a PointCloud2.
    const vector<ORB_SLAM3::MapPoint*>& vpMPs = current_map->GetAllMapPoints();
    if (!vpMPs.empty())
    {
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        point_cloud_msg.header.stamp = msg->header.stamp;
        point_cloud_msg.header.frame_id = "map";
        point_cloud_msg = convertMapPointsToPointCloud2(vpMPs, point_cloud_msg.header);
        point_cloud_pub->publish(point_cloud_msg);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        std::cerr << "Usage: ros2 run <package_name> fire_localization path_to_vocabulary path_to_settings" <<
            std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // Create and initialize the ORB_SLAM3 system.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    auto node = rclcpp::Node::make_shared("fire_localization");
    node->declare_parameter<std::string>("image_transport", "compressed");

    auto point_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker/drone", 10);

    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // subscribe to <sensor_msgs::msg::Image> "/dji_osdk_ros/main_wide_RGB"
    auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/dji_osdk_ros/main_wide_RGB", 10,
        [&, tf_broadcaster, point_cloud_pub, marker_pub](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
        {
            ImageBoxesCallback(&SLAM, tf_broadcaster.get(), point_cloud_pub, marker_pub, msg);
        });

    rclcpp::spin(node);

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();
    return 0;
}

sensor_msgs::msg::PointCloud2 convertMapPointsToPointCloud2(
    const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
    const std_msgs::msg::Header& header)
{
    sensor_msgs::msg::PointCloud2 pointCloudMsg;
    pointCloudMsg.header = header;
    pointCloudMsg.height = 1;
    pointCloudMsg.width = mapPoints.size();
    pointCloudMsg.is_dense = false;
    pointCloudMsg.is_bigendian = false;

    // Set up the fields: x, y, z.
    pointCloudMsg.fields.resize(3);
    pointCloudMsg.fields[0].name = "x";
    pointCloudMsg.fields[0].offset = 0;
    pointCloudMsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointCloudMsg.fields[0].count = 1;

    pointCloudMsg.fields[1].name = "y";
    pointCloudMsg.fields[1].offset = 4;
    pointCloudMsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointCloudMsg.fields[1].count = 1;

    pointCloudMsg.fields[2].name = "z";
    pointCloudMsg.fields[2].offset = 8;
    pointCloudMsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointCloudMsg.fields[2].count = 1;

    pointCloudMsg.point_step = 3 * sizeof(float);
    pointCloudMsg.row_step = pointCloudMsg.point_step * pointCloudMsg.width;
    pointCloudMsg.data.resize(pointCloudMsg.row_step * pointCloudMsg.height);

    sensor_msgs::PointCloud2Iterator<float> iter_x(pointCloudMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointCloudMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointCloudMsg, "z");

    for (const auto& mapPoint : mapPoints)
    {
        if (mapPoint && !mapPoint->isBad())
        {
            const Eigen::Vector3f& pos = mapPoint->GetWorldPos();
            *iter_x = pos.x();
            *iter_y = pos.y();
            *iter_z = pos.z();
        }
        else
        {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    return pointCloudMsg;
}
