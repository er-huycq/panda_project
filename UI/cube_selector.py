#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import math
import numpy as np
import cv2
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from cube_selector_window import Ui_Form

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from yolov8_msgs.msg import Yolov8Inference
from cv_bridge import CvBridge

bridge = CvBridge()

img = np.zeros([480, 640, 3])

COLOR_MAP = {
    'red_cube':   QColor(255, 0, 0, 255),
    'green_cube': QColor(0, 255, 0, 255),
    'blue_cube':  QColor(0, 0, 255, 255),
}

COLOR_MAP_FILL = {
    'red_cube':   QColor(255, 0, 0, 100),
    'green_cube': QColor(0, 255, 0, 100),
    'blue_cube':  QColor(0, 0, 255, 100),
}

class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.mouse_x = 0
        self.mouse_y = 0
        self.click_pos = None

    def mouseMoveEvent(self, event):
        self.mouse_x = event.scenePos().x()
        self.mouse_y = event.scenePos().y()

    def mousePressEvent(self, event):
        self.click_pos = (event.scenePos().x(), event.scenePos().y())


class GUI(QDialog):

    def __init__(self, parent=None):
        super(GUI, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.scene = GraphicsScene(self.ui.graphicsView)
        self.ui.graphicsView.setScene(self.scene)
        self.ui.graphicsView.setMouseTracking(True)

        rclpy.init(args=None)
        self.camera_subscriber = Node('cube_image_subscriber')
        self.sub = self.camera_subscriber.create_subscription(
            Image, '/image_raw', self.camera_callback, 10)

        self.cube_subscriber = Node('cube_inference_subscriber')
        self.sub = self.cube_subscriber.create_subscription(
            Yolov8Inference, '/Cube_Inference', self.cube_callback, 10)

        self.pub_node = Node('cube_pub_path')
        self.pub = self.pub_node.create_publisher(Float64MultiArray, '/target_point', 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

        self.brush = QBrush(QColor(255, 255, 255, 255))
        self.target_point = [0, 0, 0, 1]
        self.fx = 253.93635749816895
        self.fy = 253.93635749816895
        self.cx = 320
        self.cy = 240
        self.z = 0.7
        self.init_x = 0.2
        self.init_y = 0.6

    def camera_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, "bgr8")

    def cube_callback(self, data):
        global img

        self.scene.clear()
        rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height, 3 * width, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        pixmap_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(pixmap_item)

        handled = False
        for r in data.yolov8_inference:

            points = np.array(r.coordinates).astype(np.int32).reshape([4, 2])
            middle_point = np.sum(points, 0) / 4
            dist = math.sqrt(
                (self.scene.mouse_x - middle_point[0]) ** 2 +
                (self.scene.mouse_y - middle_point[1]) ** 2
            )

            class_name = r.class_name
            pen_color = COLOR_MAP.get(class_name, QColor(0, 0, 255, 255))
            fill_color = COLOR_MAP_FILL.get(class_name, QColor(0, 0, 255, 100))

            qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])

            if dist < 15:
                self.scene.addPolygon(qpoly, QPen(QColor(255, 0, 0, 255)),
                                      QBrush(QColor(255, 0, 0, 100)))

                if self.scene.click_pos is not None:
                    click_dist = math.sqrt(
                        (self.scene.click_pos[0] - middle_point[0]) ** 2 +
                        (self.scene.click_pos[1] - middle_point[1]) ** 2
                    )
                    if click_dist < 15:
                        self.target_point[0] = -self.z * (middle_point[1] - self.cy) / self.fy + self.init_x
                        self.target_point[1] = -self.z * (middle_point[0] - self.cx) / self.fx + self.init_y

                        dist1 = math.sqrt((points[0][0] - points[1][0]) ** 2 + (points[0][1] - points[1][1]) ** 2)
                        dist2 = math.sqrt((points[1][0] - points[2][0]) ** 2 + (points[1][1] - points[2][1]) ** 2)

                        if dist1 > dist2:
                            denominator = points[0][0] - points[1][0]
                            if denominator == 0:
                                angle = math.pi / 2
                            else:
                                angle = math.atan2(points[0][1] - points[1][1], denominator)
                        else:
                            denominator = points[1][0] - points[2][0]
                            if denominator == 0:
                                angle = math.pi / 2
                            else:
                                angle = math.atan2(points[1][1] - points[2][1], denominator)

                        self.target_point[2] = math.pi / 2 - angle
                        self.target_point[3] = 1.0

                        target_point_pub = Float64MultiArray(data=self.target_point)
                        self.pub.publish(target_point_pub)
                        self.scene.click_pos = None
                        handled = True
            else:
                self.scene.addPolygon(qpoly, QPen(pen_color), QBrush(fill_color))

            self.scene.addEllipse(middle_point[0] - 2, middle_point[1] - 2, 4, 4,
                                  QPen(Qt.green), QBrush(Qt.green))

        if not handled:
            self.scene.click_pos = None

    def update(self):
        rclpy.spin_once(self.camera_subscriber)
        rclpy.spin_once(self.cube_subscriber)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())
