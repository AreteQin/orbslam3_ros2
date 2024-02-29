#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray
import cv2
import torch
from ultralytics import YOLO
import pathlib
import datetime

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.subscription = self.create_subscription(
            Image,
            '/dji_osdk_ros/main_wide_RGB',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/bounding_boxes/fire_spots',
            10)
        self.bridge = CvBridge()
        self.model = YOLO("/home/qin/m300_ws/src/forest_fire_geopositioning/scripts/YoloWeights/v8l.pt")
        self.get_logger().info('Node has been initialized')

    def callback(self, image):
        try:
            frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
            results = self.model(frame, stream=True)
            ros_boxes = Detection2DArray()
            ros_boxes.header = image.header
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 1)

                    ros_box = Detection2D()
                    ros_box.header = image.header
                    ros_box.bbox.size_x = x2 - x1
                    ros_box.bbox.size_y = y2 - y1
                    ros_box.bbox.center.x = (x1 + x2) / 2
                    ros_box.bbox.center.y = (y1 + y2) / 2
                    ros_box.bbox.center.theta = 0
                    ros_boxes.detections.append(ros_box)
                self.publisher.publish(ros_boxes)
                self.get_logger().info(f"Number of boxes detected: {len(boxes)}")
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    image_listener = ImageListener()
    rclpy.spin(image_listener)
    image_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
