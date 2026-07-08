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

    def read_gps_sensor(self):
        """
        Fiziksel GPS sensöründen veri okuyarak sensor_msgs/NavSatFix mesajı döndürür.

        Bu fonksiyon şu anda bir iskelet olarak bırakılmıştır.
        Gerçek GPS sensörü belirlendikten sonra aşağıdaki TODO bölümleri
        doldurulmalıdır.

        Desteklenen bağlantı seçenekleri:
          SEÇENEK 1 — UART/USB Serial (örn. u-blox, NMEA modülleri)
          SEÇENEK 2 — I2C  (örn. Quectel L76-L gibi I2C GPS modülleri)
          SEÇENEK 3 — MAVROS köprüsü (aşağıya bakınız)

        MAVROS köprüsü hakkında not:
          Pixhawk GPS verisi MAVROS aracılığıyla şu topic'ten okunabilir:
            /mavros/global_position/global  (sensor_msgs/NavSatFix)
          Bu MAVROS topic'i yalnızca bir veri KAYNAĞI olarak kullanılır.
          Bu node /mavros/global_position/global topic'ine KESİNLİKLE publish etmez.
          MAVROS kullanılırsa alınan veri /albatros/gps/fix olarak yeniden
          yayınlanır (re-publish). Bu node'un dışarıya verdiği standart topic
          her koşulda /albatros/gps/fix olarak kalmalıdır.
          Yaklaşım: __init__ içinde self._latest_gps = None tanımlanır,
          subscriber callback'te güncellenir, timer_callback'te publish edilir.

        Dönüş:
            sensor_msgs.msg.NavSatFix  — başarılı okumada dolu mesaj
            None                       — okuma başarısızsa (yayın yapılmaz)
        """
        try:
            # -------------------------------------------------------------- #
            # TODO: Gerçek GPS sensörü seçildikten sonra burası düzenlenecek.
            #
            # SEÇENEK 1 — UART / USB Serial (NMEA, örn. u-blox NEO-M8):
            #   import serial
            #   import pynmea2
            #   ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1.0)
            #   line = ser.readline().decode('ascii', errors='replace')
            #   if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
            #       nmea = pynmea2.parse(line)
            #       lat = nmea.latitude
            #       lon = nmea.longitude
            #       alt = nmea.altitude
            #
            # SEÇENEK 2 — I2C GPS:
            #   import smbus2
            #   bus = smbus2.SMBus(1)
            #   raw = bus.read_i2c_block_data(GPS_I2C_ADDR, REG, 14)
            #   ...
            #
            # SEÇENEK 3 — MAVROS köprüsü:
            #   __init__ içinde subscriber aç:
            #   self._latest_gps = None
            #   self.mavros_gps_sub = self.create_subscription(
            #       NavSatFix,
            #       '/mavros/global_position/global',
            #       self._mavros_gps_cb,
            #       10
            #   )
            #
            #   def _mavros_gps_cb(self, msg):
            #       msg.header.frame_id = GPS_FRAME_ID
            #       self._latest_gps = msg
            #
            #   Bu fonksiyonda: return self._latest_gps
            #
            # Sensör başlatma (port açma, kalibrasyon vb.) __init__ içinde yapın.
            # -------------------------------------------------------------- #

            # Gerçek sensör henüz uygulanmadı — bu satırı gerçek kodla değiştir!
            raise NotImplementedError(
                'read_gps_sensor() henüz uygulanmadı. '
                'simulate_mode:=true parametresini kullanın.'
            )

            # -------------------------------------------------------------- #
            # TODO: Sensör verisini okuduktan sonra aşağıdaki yapıyı doldurun.
            # -------------------------------------------------------------- #
            msg = NavSatFix()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = GPS_FRAME_ID

            # TODO: GPS fix durumunu sensörden alın
            msg.status.status  = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            # TODO: Enlem, boylam ve yüksekliği sensörden alın
            msg.latitude  = 0.0
            msg.longitude = 0.0
            msg.altitude  = 0.0

            msg.position_covariance = [
                POSITION_COV_DIAG, 0.0,               0.0,
                0.0,               POSITION_COV_DIAG,  0.0,
                0.0,               0.0,                POSITION_COV_DIAG * 4.0,
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            return msg

        except NotImplementedError as e:
            # Uyarı spam önlemi: yalnızca ilk döngüde bir kez uyarı ver
            if not self._real_sensor_warning_printed:
                self.get_logger().warn(
                    f'GPS sensör okuma henüz uygulanmadı: {e} '
                    '| Bu uyarı bir kez gösterilir. '
                    '| simulate_mode:=true parametresini kullanabilirsiniz.'
                )
                self._real_sensor_warning_printed = True
            return None

        except Exception as e:
            # Beklenmedik sensör hatası — node kapanmaz, bu döngü atlanır
            self.get_logger().error(
                f'GPS sensör okuma hatası: {e} '
                '| Bu döngü için veri yayınlanmayacak.'
            )
            return None


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