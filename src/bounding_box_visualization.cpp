//
// Created by qin on 4/11/24.
//
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;

void ImageBoxesCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
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
    cout << "fire spots number: " << msg_fire_spot->detections.size() << endl;
    for (auto &box: msg_fire_spot->detections) {
        fire_spots.emplace_back(box.bbox.center.x - box.bbox.size_x / 2,
                                box.bbox.center.y - box.bbox.size_y / 2,
                                box.bbox.size_x,
                                box.bbox.size_y);
    }

    // Draw the bounding box
    for (auto &box: fire_spots) {
        cv::rectangle(cv_ptr->image, box, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("view", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bounding_box_visualization");
    // message filter for images
    image_transport::ImageTransport it(node);
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image(node, "/dji_osdk_ros/main_wide_RGB");
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> sub_fire_spot;
    sub_fire_spot.subscribe(node, "/bounding_boxes/fire_spots");

    // Sync the subscribed data
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>
            sync(sub_image, sub_fire_spot, 100);
    sync.registerCallback(std::bind(&ImageBoxesCallback,
                                    std::placeholders::_1,
                                    std::placeholders::_2));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
