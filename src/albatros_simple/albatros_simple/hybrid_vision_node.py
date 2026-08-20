#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class HybridVisionNode(Node):
    """
    hybrid_vision_node (Sade İskelet)
    - Kamera görüntüsünü alma, YOLO + OpenCV ile hedef tespiti/açı hesabı yapma yer tutucusudur.
    - Elde edilen sonuçları mission_node'a aktaracaktır.
    - Pixhawk kontrolü içermez.
    """

    def __init__(self):
        super().__init__('hybrid_vision_node')
        self.get_logger().info('HybridVisionNode başlatıldı (iskelet modunda çalışıyor).')
        
        # Periyodik log basımı (isteğe bağlı placeholder gösterim)
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('[HybridVisionNode] YOLO + OpenCV hibrit görüntü işleme aktif (iskelet)...')


def main(args=None):
    rclpy.init(args=args)
    node = HybridVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
