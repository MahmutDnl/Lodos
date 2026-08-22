#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MissionNode(Node):
    """
    mission_node (Parkur-3 Kamikaze Görev Yöneticisi)
    - /albatros/parkur3_active sinyalini dinler.
    - Sinyal gelmeden Parkur-3 kamikaze görev mantığını çalıştırmaz.
    - Sinyal geldikten sonra Parkur-3 otonom angajman kodlarını devreye sokar.
    """

    def __init__(self):
        super().__init__('mission_node')
        self.get_logger().info('MissionNode başlatıldı (Parkur-3 Kamikaze Yöneticisi).')
        
        self.parkur3_active = False
        self.parkur3_sub = self.create_subscription(
            Bool,
            '/albatros/parkur3_active',
            self.parkur3_active_callback,
            10
        )

        # Periyodik kontrol döngüsü
        self.timer = self.create_timer(2.0, self.timer_callback)

    def parkur3_active_callback(self, msg: Bool):
        if msg.data and not self.parkur3_active:
            self.get_logger().info('[MissionNode] Parkur-3 sinyali alındı! Kamikaze angajman görevi BAŞLATILDI.')
        self.parkur3_active = msg.data

    def timer_callback(self):
        if not self.parkur3_active:
            self.get_logger().info('[MissionNode] Görev durumu: BEKLEMEDE (Parkur-3 aktif sinyali bekleniyor)...')
        else:
            self.get_logger().info('[MissionNode] Görev durumu: PARKUR-3 KAMİKAZE ANGAJMANI AKTİF!')


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
