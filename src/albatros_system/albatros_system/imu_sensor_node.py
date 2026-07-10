#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros İnsansız Deniz Aracı — IMU Sensör Publisher Node
# =============================================================================
# Dosya    : imu_sensor_node.py
# Node adı : imu_sensor_node
# Paket    : albatros_system
# Görev    : IMU verisini okuyarak /albatros/imu/data topic'ine yayınlamak.
#            Karar vermez, motor/mavros komutu göndermez; yalnızca veri üretir.
# Yazan    : LODOS Yazılım Ekibi
# Tarih    : 2026
# =============================================================================
#
# Bağımlılıklar:
#   - rclpy
#   - sensor_msgs (sensor_msgs/msg/Imu)
#   - math, random, time (simülasyon modu için)
#
# Parametreler:
#   simulate_mode (bool, varsayılan: True)
#       True  → Sahte/test IMU verisi üretir. Fiziksel IMU gerekmez.
#       False → read_imu_sensor() fonksiyonu üzerinden gerçek sensör okunur.
#
#   publish_rate (float, varsayılan: 50.0 Hz)
#       IMU verisinin ROS 2 topic'e yayınlanma frekansı.
#       0 veya negatif değer girilirse otomatik olarak 50.0 Hz kullanılır.
#
# Yayınlanan Topic:
#   /albatros/imu/data  [sensor_msgs/msg/Imu]
#
# Uyumluluk:
#   Bu topic state_node.py tarafından okunmaktadır.
#   Topic adı KESİNLİKLE değiştirilmemelidir.
# =============================================================================

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


# =============================================================================
# Sabitler
# =============================================================================

# IMU yayın topic adı — state_node bu topic'i dinler, değiştirilmemeli!
IMU_TOPIC = '/albatros/imu/data'

# TF çerçeve kimliği — URDF/TF ağacında IMU montaj noktasına karşılık gelir
IMU_FRAME_ID = 'imu_link'

# Varsayılan yayın frekansı (Hz) — geçersiz değer girilirse bu kullanılır
DEFAULT_PUBLISH_RATE = 50.0

# Yer çekimi ivmesi (m/s²)
GRAVITY = 9.81

# Orientation covariance köşegen değeri (rad²)
ORIENT_COV_DIAG = 1e-4

# Angular velocity covariance köşegen değeri (rad²/s²)
ANG_VEL_COV_DIAG = 1e-5

# Linear acceleration covariance köşegen değeri (m²/s⁴)
LIN_ACC_COV_DIAG = 1e-3


# =============================================================================
# IMU Sensör Node Sınıfı
# =============================================================================

class ImuSensorNode(Node):
    """
    IMU verisini /albatros/imu/data topic'ine yayınlayan publisher node.

    simulate_mode=True  → Sahte ama geçerli IMU verisi üretir.
    simulate_mode=False → read_imu_sensor() üzerinden gerçek sensörden okur.
                          Sensör henüz uygulanmamışsa tek seferlik uyarı verir,
                          node kapanmaz.
    """

    def __init__(self):
        super().__init__('imu_sensor_node')

        # ------------------------------------------------------------------ #
        # Parametreler
        # ------------------------------------------------------------------ #
        self.declare_parameter('simulate_mode', True)
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE)

        self.simulate_mode = self.get_parameter('simulate_mode').value
        raw_rate = self.get_parameter('publish_rate').value

        # publish_rate güvenlik kontrolü — 0 veya negatif değere izin verilmez
        if raw_rate <= 0.0:
            self.get_logger().warn(
                f'publish_rate={raw_rate} geçersiz. '
                f'Varsayılan {DEFAULT_PUBLISH_RATE} Hz kullanılıyor.'
            )
            self.publish_rate = DEFAULT_PUBLISH_RATE
        else:
            self.publish_rate = raw_rate

        # ------------------------------------------------------------------ #
        # Uyarı spam önleyici — gerçek sensör henüz uygulanmadıysa
        # ilk döngüde bir kez uyarı ver, sonraki döngülerde sessiz kal
        # ------------------------------------------------------------------ #
        self._real_sensor_warning_printed = False

        # ------------------------------------------------------------------ #
        # MAVROS köprüsü — simulate_mode=False ise gerçek IMU verisi alınır
        # ------------------------------------------------------------------ #
        self._latest_imu = None

        if not self.simulate_mode:
            self._mavros_imu_sub = self.create_subscription(
                Imu,
                '/mavros/imu/data',
                self._mavros_imu_callback,
                qos_profile=qos_profile_sensor_data,
            )
            self.get_logger().info(
                'MAVROS IMU köprüsü aktif: /mavros/imu/data -> '
                f'{IMU_TOPIC}'
            )

        # ------------------------------------------------------------------ #
        # Publisher
        # ------------------------------------------------------------------ #
        self.imu_publisher = self.create_publisher(
            Imu,
            IMU_TOPIC,
            qos_profile=10
        )

        # ------------------------------------------------------------------ #
        # Timer
        # ------------------------------------------------------------------ #
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Simülasyon salınımı için zaman referansı
        self._start_time = time.time()

        # ------------------------------------------------------------------ #
        # Başlangıç logları
        # ------------------------------------------------------------------ #
        mode_str = ('SİMÜLASYON (simulate_mode=True)'
                    if self.simulate_mode
                    else 'GERÇEK SENSÖR (simulate_mode=False)')

        self.get_logger().info('=' * 60)
        self.get_logger().info('IMU Sensör Node başlatıldı.')
        self.get_logger().info(f'  Mod           : {mode_str}')
        self.get_logger().info(f'  Topic         : {IMU_TOPIC}')
        self.get_logger().info(f'  Yayın frekansı: {self.publish_rate} Hz')
        self.get_logger().info(f'  Frame ID      : {IMU_FRAME_ID}')
        self.get_logger().info('=' * 60)

    # ================================================================== #
    # Timer callback
    # ================================================================== #

    def timer_callback(self):
        """
        Belirlenen frekansta çağrılır.
        Veri geçerliyse topic'e yayınlar; None ise bu döngüyü sessizce atlar.
        """
        if self.simulate_mode:
            imu_data = self.generate_simulated_imu()
        else:
            imu_data = self.read_imu_sensor()

        if imu_data is not None:
            self.imu_publisher.publish(imu_data)

    # ================================================================== #
    # Simülasyon modu
    # ================================================================== #

    def generate_simulated_imu(self) -> Imu:
        """
        Geçerli bir sensor_msgs/Imu mesajı oluşturur.

        - Orientation : küçük roll/pitch salınımı, yaw sabit
        - Angular vel : küçük Gaussian gürültü
        - Linear accel: simülasyon için araç sabit kabul edilerek
                        z ekseninde yerçekimi ivmesi yaklaşık 9.81 m/s² verilir.
        """
        msg = Imu()

        # Header
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = IMU_FRAME_ID

        # Orientation — küçük sinüs tabanlı roll/pitch, sabit yaw
        elapsed = time.time() - self._start_time
        roll  = 0.01  * math.sin(0.5 * elapsed)   # ~0.01 rad genlik
        pitch = 0.005 * math.cos(0.3 * elapsed)   # ~0.005 rad genlik
        yaw   = 0.0                                # sabit

        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Orientation covariance (3×3, satır öncelikli)
        msg.orientation_covariance = [
            ORIENT_COV_DIAG, 0.0,             0.0,
            0.0,             ORIENT_COV_DIAG,  0.0,
            0.0,             0.0,              ORIENT_COV_DIAG,
        ]

        # Angular velocity — küçük gürültü (rad/s)
        msg.angular_velocity.x = random.gauss(0.0, 0.001)
        msg.angular_velocity.y = random.gauss(0.0, 0.001)
        msg.angular_velocity.z = random.gauss(0.0, 0.001)

        msg.angular_velocity_covariance = [
            ANG_VEL_COV_DIAG, 0.0,                0.0,
            0.0,              ANG_VEL_COV_DIAG,    0.0,
            0.0,              0.0,                 ANG_VEL_COV_DIAG,
        ]

        # Linear acceleration — z'de yerçekimi, x/y küçük gürültü (m/s²)
        msg.linear_acceleration.x = random.gauss(0.0, 0.01)
        msg.linear_acceleration.y = random.gauss(0.0, 0.01)
        msg.linear_acceleration.z = GRAVITY + random.gauss(0.0, 0.05)

        msg.linear_acceleration_covariance = [
            LIN_ACC_COV_DIAG, 0.0,               0.0,
            0.0,              LIN_ACC_COV_DIAG,   0.0,
            0.0,              0.0,                LIN_ACC_COV_DIAG,
        ]

        return msg

    # ================================================================== #
    # Gerçek sensör okuma iskelet fonksiyonu
    # ================================================================== #

    def _mavros_imu_callback(self, msg: Imu) -> None:
        """
        /mavros/imu/data topic'inden gelen IMU verisini yakalar.
        frame_id'yi Albatros standardına günceller ve ara değişkende saklar.
        """
        msg.header.frame_id = IMU_FRAME_ID
        self._latest_imu = msg

    def read_imu_sensor(self):
        """
        MAVROS köprüsü üzerinden gerçek IMU verisini döndürür.

        /mavros/imu/data topic'inden gelen en son mesaj _latest_imu'da
        tutulur. Henüz veri gelmediyse None döndürülür (timer_callback
        bu döngüyü sessizce atlar).

        Dönüş:
            sensor_msgs.msg.Imu  — MAVROS'tan gelen en son IMU mesajı
            None                 — henüz veri gelmemişse
        """
        return self._latest_imu

    # ================================================================== #
    # Yardımcı: Euler → Quaternion
    # ================================================================== #

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
        """
        Euler açılarını (rad) ZYX sırasına göre quaternion'a çevirir.

        Argümanlar:
            roll  : x ekseni dönüşü (rad)
            pitch : y ekseni dönüşü (rad)
            yaw   : z ekseni dönüşü (rad)

        Dönüş:
            tuple (qx, qy, qz, qw)
        """
        cr = math.cos(roll  * 0.5)
        sr = math.sin(roll  * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw   * 0.5)
        sy = math.sin(yaw   * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw


# =============================================================================
# Giriş noktası
# =============================================================================

def main(args=None):
    """
    Node başlangıç fonksiyonu.
    Çağrı: ros2 run albatros_system imu_sensor_node
    """
    rclpy.init(args=args)
    node = ImuSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU Sensör Node durduruldu (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()