#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)

        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value

        self.publisher_ = self.create_publisher(Image, '/albatros/kamera/image_raw', 10)

        self.cap = cv2.VideoCapture(self.camera_index)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            self.get_logger().error('Kamera açılamadı.')
        else:
            self.get_logger().info('Kamera başarıyla açıldı.')
            self.get_logger().info(
                f'Kamera ayarları: {self.frame_width}x{self.frame_height} @ {self.fps} FPS'
            )

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()

        if ret:
            
            frame = cv2.flip(frame, 1)   #sağ-sol ters görüntü için ekledim.

            if len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            if frame.dtype != 'uint8':
                frame = frame.astype(np.uint8)

            image_msg = Image()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            image_msg.height = frame.shape[0]
            image_msg.width = frame.shape[1]
            image_msg.encoding = 'bgr8'
            image_msg.is_bigendian = False
            image_msg.step = frame.shape[1] * 3
            image_msg.data = frame.tobytes()

            self.publisher_.publish(image_msg)
        else:
            self.get_logger().warn('Kameradan görüntü alınamadı.')

    def destroy_node(self):
        self.cap.release()
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