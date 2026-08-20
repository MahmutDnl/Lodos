#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MissionNode(Node):
    """
    mission_node (Sade İskelet)
    - Pixhawk görev akışını yönetecektir (AUTO -> GUIDED geçişi vb.).
    - hybrid_vision_node'dan tespit verilerini alacaktır.
    - Pixhawk/MAVROS ile iletişim sadece bu node üzerinde yürütülecektir.
    """

    def __init__(self):
        super().__init__('mission_node')
        self.get_logger().info('MissionNode başlatıldı (iskelet modunda çalışıyor).')
        
        # Periyodik log basımı (isteğe bağlı placeholder gösterim)
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('[MissionNode] Görev durumu: BEKLEMEDE (AUTO -> GUIDED geçiş iskeleti aktif)...')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
