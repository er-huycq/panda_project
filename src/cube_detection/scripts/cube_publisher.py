#!/usr/bin/env python3

import cv2
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult, Yolov8Inference

bridge = CvBridge()

class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Yolov8Inference, '/Cube_Inference', 10)

        self.color_ranges = {
            'red': [
                (np.array([0, 80, 80]), np.array([10, 255, 255])),
                (np.array([170, 80, 80]), np.array([180, 255, 255]))
            ],
            'green': [
                (np.array([40, 80, 80]), np.array([80, 255, 255]))
            ],
            'blue': [
                (np.array([100, 80, 80]), np.array([130, 255, 255]))
            ]
        }

        self.min_area = 200

    def image_callback(self, data):
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        inference_msg = Yolov8Inference()
        inference_msg.header = Header()
        inference_msg.header.stamp = self.get_clock().now().to_msg()

        for color_name, ranges in self.color_ranges.items():
            mask = None
            for lower, upper in ranges:
                current_mask = cv2.inRange(hsv, lower, upper)
                if mask is None:
                    mask = current_mask
                else:
                    mask = cv2.bitwise_or(mask, current_mask)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue

                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int32(box)

                result = InferenceResult()
                result.class_name = color_name + '_cube'
                result.coordinates = box.flatten().tolist()
                inference_msg.yolov8_inference.append(result)

        self.publisher.publish(inference_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = node.create_rate(30)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
