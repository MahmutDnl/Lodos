#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ROS2 Parametreleri Tanımlama
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 20)

        # Parametre Değerlerini Alma
        self.camera_index = int(self.get_parameter('camera_index').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)

        # CvBridge ve Publisher Kurulumu
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(
            Image,
            '/albatros/camera/image_raw',
            qos_profile_sensor_data
        )

        # Fiziksel Kamerayı Açma
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Kamera açılamadı! (Camera Index: {self.camera_index})')
        else:
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(
                f'Kamera başarıyla açıldı. Gerçek Çözünürlük: {actual_width}x{actual_height}, FPS: {actual_fps}'
            )

        # Timer Kurulumu (Yayın Frekansı)
        timer_period = 1.0 / self.fps if self.fps > 0 else 0.05
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def publish_frame(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Kameradan tek bir frame okunamadı, işlem atlanıyor.')
            return

        # Frame'e ROS timestamp ve frame_id ekleme, yayınlama
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            self.publisher_.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Görüntü dönüştürülürken/yayınlanırken hata oluştu: {str(e)}')

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Kamera serbest bırakıldı (released).')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
