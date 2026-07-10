#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros İnsansız Deniz Aracı — GPS Sensör Publisher Node
# =============================================================================
# Dosya    : gps_sensor_node.py
# Node adı : gps_sensor_node
# Paket    : albatros_system
# Görev    : GPS verisini okuyarak /albatros/gps/fix topic'ine yayınlamak.
#            Karar vermez, motor/mavros komutu göndermez; yalnızca veri üretir.
# Yazan    : LODOS Yazılım Ekibi
# Tarih    : 2026
# =============================================================================
#
# Bağımlılıklar:
#   - rclpy
#   - sensor_msgs (sensor_msgs/msg/NavSatFix, sensor_msgs/msg/NavSatStatus)
#   - random (simülasyon modu için)
#
# Parametreler:
#   simulate_mode   (bool,  varsayılan: True)
#       True  → Sahte/test GPS verisi üretir. Fiziksel GPS gerekmez.
#       False → read_gps_sensor() fonksiyonu üzerinden gerçek sensör okunur.
#
#   publish_rate    (float, varsayılan: 10.0 Hz)
#       GPS verisinin topic'e yayınlanma frekansı.
#       0 veya negatif değer girilirse otomatik 10.0 Hz kullanılır.
#
#   base_latitude   (float, varsayılan: 40.1885)
#       Simülasyon modunda merkez enlem değeri (derece). [-90, +90] aralığında olmalı.
#
#   base_longitude  (float, varsayılan: 29.0610)
#       Simülasyon modunda merkez boylam değeri (derece). [-180, +180] aralığında olmalı.
#
#   base_altitude   (float, varsayılan: 0.0)
#       Simülasyon modunda merkez yükseklik değeri (metre).
#
#   position_noise  (float, varsayılan: 0.000005)
#       Simülasyon modunda enlem/boylam'a eklenen Gaussian gürültü standart
#       sapması (derece). ~0.5 m'lik sapma üretir.
#       Negatif değer girilirse varsayılana döner; 0 girilirse sabit konum yayınlanır.
#
# Yayınlanan Topic:
#   /albatros/gps/fix  [sensor_msgs/msg/NavSatFix]
#
# Uyumluluk:
#   Bu topic state_node.py tarafından okunmaktadır.
#   Topic adı KESİNLİKLE değiştirilmemelidir.
# =============================================================================

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


# =============================================================================
# Sabitler
# =============================================================================

# GPS yayın topic adı — state_node bu topic'i dinler, değiştirilmemeli!
GPS_TOPIC = '/albatros/gps/fix'

# TF çerçeve kimliği
GPS_FRAME_ID = 'gps_link'

# Varsayılan yayın frekansı (Hz) — geçersiz değer girilirse bu kullanılır
DEFAULT_PUBLISH_RATE = 10.0

# Varsayılan başlangıç koordinatları (Bursa / test alanı)
DEFAULT_LATITUDE  = 40.1885
DEFAULT_LONGITUDE = 29.0610
DEFAULT_ALTITUDE  = 0.0

# Simülasyon gürültü standart sapması (derece)
DEFAULT_POSITION_NOISE = 0.000005

# Konum kovaryans köşegen değeri (m²) — yaklaşık 2.5 m² belirsizlik
POSITION_COV_DIAG = 6.25  # ~ (2.5 m)²


# =============================================================================
# GPS Sensör Node Sınıfı
# =============================================================================

class GpsSensorNode(Node):
    """
    GPS verisini /albatros/gps/fix topic'ine yayınlayan publisher node.

    simulate_mode=True  → Sahte ama geçerli NavSatFix mesajı üretir.
    simulate_mode=False → read_gps_sensor() üzerinden gerçek sensörden okur.
                          Sensör henüz uygulanmamışsa tek seferlik uyarı verir,
                          node kapanmaz.
    """

    def __init__(self):
        super().__init__('gps_sensor_node')

        # ------------------------------------------------------------------ #
        # Parametreler
        # ------------------------------------------------------------------ #
        self.declare_parameter('simulate_mode',   True)
        self.declare_parameter('publish_rate',    DEFAULT_PUBLISH_RATE)
        self.declare_parameter('base_latitude',   DEFAULT_LATITUDE)
        self.declare_parameter('base_longitude',  DEFAULT_LONGITUDE)
        self.declare_parameter('base_altitude',   DEFAULT_ALTITUDE)
        self.declare_parameter('position_noise',  DEFAULT_POSITION_NOISE)

        self.simulate_mode  = self.get_parameter('simulate_mode').value
        raw_rate            = self.get_parameter('publish_rate').value
        raw_lat             = self.get_parameter('base_latitude').value
        raw_lon             = self.get_parameter('base_longitude').value
        self.base_altitude  = self.get_parameter('base_altitude').value
        raw_noise           = self.get_parameter('position_noise').value

        # publish_rate güvenlik kontrolü — 0 veya negatif değere izin verilmez
        if raw_rate <= 0.0:
            self.get_logger().warn(
                f'publish_rate={raw_rate} geçersiz. '
                f'Varsayılan {DEFAULT_PUBLISH_RATE} Hz kullanılıyor.'
            )
            self.publish_rate = DEFAULT_PUBLISH_RATE
        else:
            self.publish_rate = raw_rate

        # base_latitude aralık kontrolü — geçerli aralık: [-90, +90]
        if not (-90.0 <= raw_lat <= 90.0):
            self.get_logger().warn(
                f'base_latitude={raw_lat} geçersiz ([-90, +90] dışında). '
                f'Varsayılan {DEFAULT_LATITUDE}° kullanılıyor.'
            )
            self.base_latitude = DEFAULT_LATITUDE
        else:
            self.base_latitude = raw_lat

        # base_longitude aralık kontrolü — geçerli aralık: [-180, +180]
        if not (-180.0 <= raw_lon <= 180.0):
            self.get_logger().warn(
                f'base_longitude={raw_lon} geçersiz ([-180, +180] dışında). '
                f'Varsayılan {DEFAULT_LONGITUDE}° kullanılıyor.'
            )
            self.base_longitude = DEFAULT_LONGITUDE
        else:
            self.base_longitude = raw_lon

        # position_noise güvenlik kontrolü — negatif değere izin verilmez
        # 0 kabul edilir: gürültüsüz sabit konum yayınlanır
        if raw_noise < 0.0:
            self.get_logger().warn(
                f'position_noise={raw_noise} negatif, geçersiz. '
                f'Varsayılan {DEFAULT_POSITION_NOISE} kullanılıyor.'
            )
            self.position_noise = DEFAULT_POSITION_NOISE
        else:
            self.position_noise = raw_noise

        # ------------------------------------------------------------------ #
        # Uyarı spam önleyici — gerçek sensör henüz uygulanmadıysa
        # ilk döngüde bir kez uyarı ver, sonraki döngülerde sessiz kal
        # ------------------------------------------------------------------ #
        self._real_sensor_warning_printed = False

        # ------------------------------------------------------------------ #
        # MAVROS köprüsü — simulate_mode=False ise gerçek GPS verisi alınır
        # ------------------------------------------------------------------ #
        self._latest_gps = None

        if not self.simulate_mode:
            self._mavros_gps_sub = self.create_subscription(
                NavSatFix,
                '/mavros/global_position/global',
                self._mavros_gps_callback,
                qos_profile=qos_profile_sensor_data,
            )
            self.get_logger().info(
                'MAVROS GPS köprüsü aktif: /mavros/global_position/global -> '
                f'{GPS_TOPIC}'
            )

        # ------------------------------------------------------------------ #
        # Publisher
        # ------------------------------------------------------------------ #
        self.gps_publisher = self.create_publisher(
            NavSatFix,
            GPS_TOPIC,
            qos_profile=10
        )

        # ------------------------------------------------------------------ #
        # Timer
        # ------------------------------------------------------------------ #
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # ------------------------------------------------------------------ #
        # Başlangıç logları
        # ------------------------------------------------------------------ #
        mode_str = ('SİMÜLASYON (simulate_mode=True)'
                    if self.simulate_mode
                    else 'GERÇEK SENSÖR (simulate_mode=False)')

        self.get_logger().info('=' * 60)
        self.get_logger().info('GPS Sensör Node başlatıldı.')
        self.get_logger().info(f'  Mod             : {mode_str}')
        self.get_logger().info(f'  Topic           : {GPS_TOPIC}')
        self.get_logger().info(f'  Yayın frekansı  : {self.publish_rate} Hz')
        self.get_logger().info(f'  Frame ID        : {GPS_FRAME_ID}')
        self.get_logger().info(f'  Başlangıç Lat   : {self.base_latitude}°')
        self.get_logger().info(f'  Başlangıç Lon   : {self.base_longitude}°')
        self.get_logger().info(f'  Başlangıç Alt   : {self.base_altitude} m')
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
            gps_data = self.generate_simulated_gps()
        else:
            gps_data = self.read_gps_sensor()

        if gps_data is not None:
            self.gps_publisher.publish(gps_data)

    # ================================================================== #
    # Simülasyon modu
    # ================================================================== #

    def generate_simulated_gps(self) -> NavSatFix:
        """
        Geçerli bir sensor_msgs/NavSatFix mesajı oluşturur.

        - Merkez konum: base_latitude / base_longitude / base_altitude
        - Konum gürültüsü: position_noise standart sapmasıyla Gaussian
        - status.status  : STATUS_FIX
        - status.service : SERVICE_GPS

        Dönüş:
            sensor_msgs.msg.NavSatFix: doldurulmuş mesaj
        """
        msg = NavSatFix()

        # Header
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = GPS_FRAME_ID

        # GPS fix durumu
        # STATUS_FIX    : GPS konum çözümü var kabul edilir.
        # SERVICE_GPS   : Veri GPS servisinden geliyor kabul edilir.
        msg.status.status  = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Konum — base değerlerine küçük Gaussian gürültü eklenir
        msg.latitude  = self.base_latitude  + random.gauss(0.0, self.position_noise)
        msg.longitude = self.base_longitude + random.gauss(0.0, self.position_noise)
        msg.altitude  = self.base_altitude  + random.gauss(0.0, 0.1)  # ±0.1 m

        # Konum kovaryans (3×3 diyagonal, yatay ve dikey belirsizlik)
        # Satır öncelikli 9 elemanlı liste: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        msg.position_covariance = [
            POSITION_COV_DIAG, 0.0,               0.0,
            0.0,               POSITION_COV_DIAG,  0.0,
            0.0,               0.0,                POSITION_COV_DIAG * 4.0,
        ]

        # COVARIANCE_TYPE_APPROXIMATED: köşegen değerler yaklaşık olarak verildi
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        return msg

    # ================================================================== #
    # Gerçek sensör okuma iskelet fonksiyonu
    # ================================================================== #

    def _mavros_gps_callback(self, msg: NavSatFix) -> None:
        """
        /mavros/global_position/global topic'inden gelen GPS verisini yakalar.
        frame_id'yi Albatros standardına günceller ve ara değişkende saklar.
        """
        msg.header.frame_id = GPS_FRAME_ID
        self._latest_gps = msg

    def read_gps_sensor(self):
        """
        MAVROS köprüsü üzerinden gerçek GPS verisini döndürür.

        /mavros/global_position/global topic'inden gelen en son mesaj
        _latest_gps'te tutulur. Henüz veri gelmediyse None döndürülür
        (timer_callback bu döngüyü sessizce atlar).

        Dönüş:
            sensor_msgs.msg.NavSatFix  — MAVROS'tan gelen en son GPS mesajı
            None                       — henüz veri gelmemişse
        """
        return self._latest_gps


# =============================================================================
# Giriş noktası
# =============================================================================

def main(args=None):
    """
    Node başlangıç fonksiyonu.
    Çağrı: ros2 run albatros_system gps_sensor_node
    """
    rclpy.init(args=args)
    node = GpsSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GPS Sensör Node durduruldu (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()