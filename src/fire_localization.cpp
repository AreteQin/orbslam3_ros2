//
// Created by qin on 2/29/24.
//
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <glog/logging.h>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "System.h"
#include "Map.h"

using namespace std;

void
ImageBoxesCallback(ORB_SLAM3::System *pSLAM,
                   const std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> fire_spots_pub,
                   const std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> camera_pose_pub,
                   const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                   const vision_msgs::msg::Detection2DArray::ConstSharedPtr &msg_fire_spot) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
    }

    // convert dectection2DArray to vector<cv::Rect>
    vector<cv::Rect> fire_spots;
//    LOG(INFO) << "fire spots number: " << msg_fire_spot->detections.size();
    for (auto &box: msg_fire_spot->detections) {
        fire_spots.emplace_back(box.bbox.center.x - box.bbox.size_x / 2,
                                box.bbox.center.y - box.bbox.size_y / 2,
                                box.bbox.size_x, box.bbox.size_y);
    }

    pSLAM->TrackMonocularAndFire(cv_ptr->image, cv_ptr->header.stamp.sec, fire_spots);
    ORB_SLAM3::Map *current_map = pSLAM->GetActiveMap();

    // Publish the fire spots
    geometry_msgs::msg::PoseArray fire_spots_msg;
    fire_spots_msg.header.stamp = msg->header.stamp;
    fire_spots_msg.header.frame_id = "map";
    for (Eigen::Vector3f &fire_spot: current_map->mvFireSpots) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = fire_spot[0];
        pose.position.y = fire_spot[1];
        pose.position.z = fire_spot[2];
        fire_spots_msg.poses.push_back(pose);
    }
    fire_spots_pub->publish(fire_spots_msg);

    // Publish the camera pose
    geometry_msgs::msg::PoseStamped camera_pose_msg;
    camera_pose_msg.header.stamp = msg->header.stamp;
    camera_pose_msg.header.frame_id = "map";
    Eigen::Matrix4f Tcw = pSLAM->GetCurrentPose();
    camera_pose_msg.pose.position.x = Tcw(0, 3);
    camera_pose_msg.pose.position.y = Tcw(1, 3);
    camera_pose_msg.pose.position.z = Tcw(2, 3);
    Eigen::Quaternionf q(Tcw.block<3, 3>(0, 0));
    camera_pose_msg.pose.orientation.x = q.x();
    camera_pose_msg.pose.orientation.y = q.y();
    camera_pose_msg.pose.orientation.z = q.z();
    camera_pose_msg.pose.orientation.w = q.w();
    camera_pose_pub->publish(camera_pose_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    auto node = rclcpp::Node::make_shared("fire_localization");
    // set image_transport parameter to "compressed"
    node->declare_parameter<std::string>("image_transport", "compressed");

    // message filter for images
    image_transport::ImageTransport it(node);
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image(node, "/dji_osdk_ros/main_wide_RGB");
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> sub_fire_spot;
    sub_fire_spot.subscribe(node, "/bounding_boxes/fire_spots");

    // Publish fire spots
    auto fire_spots_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("/position/fire_spots", 10);

    // Publish camera pose
    auto camera_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/position/camera_pose", 10);

    // Sync the subscribed data
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>
            sync(sub_image, sub_fire_spot, 100);
    sync.registerCallback(std::bind(&ImageBoxesCallback,
                                    &SLAM,
                                    fire_spots_pub,
                                    camera_pose_pub,
                                    std::placeholders::_1,
                                    std::placeholders::_2));

    rclcpp::spin(node);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}