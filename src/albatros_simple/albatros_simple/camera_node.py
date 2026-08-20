#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class CameraNode(Node):
    """
    camera_node (Sade İskelet)
    - Kameradan görüntü alma ve ROS2 topic üzerinden yayınlama yer tutucusudur.
    - Gerçek görüntü işleme/kamera yayını ileride eklenecektir.
    """

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('CameraNode başlatıldı (iskelet modunda çalışıyor).')
        
        # Periyodik log basımı (isteğe bağlı placeholder gösterim)
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('[CameraNode] Kamera yayını bekleniyor (iskelet)...')


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
