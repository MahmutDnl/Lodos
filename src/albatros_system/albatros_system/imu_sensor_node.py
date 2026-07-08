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

    def read_imu_sensor(self):
        """
        Fiziksel IMU sensöründen veri okuyarak sensor_msgs/Imu mesajı döndürür.

        Bu fonksiyon şu anda bir iskelet olarak bırakılmıştır.
        Gerçek IMU sensörü belirlendikten sonra aşağıdaki TODO bölümleri
        doldurulmalıdır.

        Desteklenen bağlantı seçenekleri:
          SEÇENEK 1 — I2C  (örn. smbus2 + MPU-6050, BNO055, ICM-42688)
          SEÇENEK 2 — SPI  (örn. spidev + ICM-42688, BMI088)
          SEÇENEK 3 — UART (örn. pyserial + VectorNav VN-100, Xsens MTi)
          SEÇENEK 4 — MAVROS köprüsü (aşağıya bakınız)

        MAVROS köprüsü hakkında not:
          Pixhawk IMU verisi MAVROS aracılığıyla /mavros/imu/data topic'inden
          okunabilir. Bu durumda node içinde bir subscriber açılır ve
          gelen veri /albatros/imu/data olarak yeniden yayınlanır (re-publish).
          Bu yaklaşım seçilirse timer yerine subscriber callback kullanılmalı,
          self._latest_imu gibi bir ara değişkenle veri tutulmalıdır.
          Bu node'un dışarıya verdiği standart topic her durumda
          /albatros/imu/data olarak kalmalıdır.

        Dönüş:
            sensor_msgs.msg.Imu  — başarılı okumada dolu mesaj
            None                 — okuma başarısızsa (yayın yapılmaz, node durmaz)
        """
        try:
            # -------------------------------------------------------------- #
            # TODO: Gerçek IMU sensörü seçildikten sonra burası düzenlenecek.
            #
            # SEÇENEK 1 — I2C (smbus2 örneği, MPU-6050):
            #   import smbus2
            #   bus = smbus2.SMBus(1)
            #   raw = bus.read_i2c_block_data(0x68, 0x3B, 14)
            #   accel_x = (raw[0] << 8 | raw[1]) / 16384.0 * GRAVITY
            #   gyro_x  = (raw[8] << 8 | raw[9]) / 131.0 * (math.pi / 180.0)
            #   ...
            #
            # SEÇENEK 2 — SPI (spidev örneği):
            #   import spidev
            #   spi = spidev.SpiDev()
            #   spi.open(0, 0)
            #   spi.max_speed_hz = 1000000
            #   ...
            #
            # SEÇENEK 3 — UART (pyserial örneği):
            #   import serial
            #   ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.02)
            #   raw_line = ser.readline()
            #   ...
            #
            # SEÇENEK 4 — MAVROS köprüsü:
            #   __init__ içinde subscriber aç:
            #   self.mavros_imu_sub = self.create_subscription(
            #       Imu, '/mavros/imu/data', self._mavros_imu_cb, 10
            #   )
            #   self._latest_imu = None
            #
            #   def _mavros_imu_cb(self, msg):
            #       msg.header.frame_id = IMU_FRAME_ID  # frame'i güncelle
            #       self._latest_imu = msg
            #
            #   Bu fonksiyonda: return self._latest_imu
            #
            # Sensör başlatma (port açma, kalibrasyon vb.) __init__ içinde yapın.
            # -------------------------------------------------------------- #

            # Gerçek sensör henüz uygulanmadı — bu satırı gerçek kodla değiştir!
            raise NotImplementedError(
                'read_imu_sensor() henüz uygulanmadı. '
                'simulate_mode=True parametresini kullanın.'
            )

            # -------------------------------------------------------------- #
            # TODO: Sensör verisini okuduktan sonra aşağıdaki yapıyı doldurun.
            # -------------------------------------------------------------- #
            msg = Imu()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = IMU_FRAME_ID

            # TODO: Sensörden quaternion alın; yoksa Euler → quaternion dönüştürün
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

            msg.orientation_covariance = [
                ORIENT_COV_DIAG, 0.0,             0.0,
                0.0,             ORIENT_COV_DIAG,  0.0,
                0.0,             0.0,              ORIENT_COV_DIAG,
            ]

            # TODO: Gyro değerlerini (rad/s) sensörden alın
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.angular_velocity.z = 0.0

            msg.angular_velocity_covariance = [
                ANG_VEL_COV_DIAG, 0.0,                0.0,
                0.0,              ANG_VEL_COV_DIAG,   0.0,
                0.0,              0.0,                ANG_VEL_COV_DIAG,
            ]

            # TODO: İvmeölçer değerlerini (m/s²) sensörden alın
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = GRAVITY

            msg.linear_acceleration_covariance = [
                LIN_ACC_COV_DIAG, 0.0,               0.0,
                0.0,              LIN_ACC_COV_DIAG,   0.0,
                0.0,              0.0,                LIN_ACC_COV_DIAG,
            ]

            return msg

        except NotImplementedError as e:
            # Uyarı spam önlemi: aynı mesajı her döngüde basmıyoruz.
            # Gerçek sensör kodu yazılıncaya kadar yalnızca ilk kez uyarı ver.
            if not self._real_sensor_warning_printed:
                self.get_logger().warn(
                    f'IMU sensör okuma henüz uygulanmadı: {e} '
                    '| Bu uyarı bir kez gösterilir. '
                    '| simulate_mode:=true parametresini kullanabilirsiniz.'
                )
                self._real_sensor_warning_printed = True
            return None

        except Exception as e:
            # Beklenmedik sensör hatası — node kapanmaz, bu döngü atlanır
            self.get_logger().error(
                f'IMU sensör okuma hatası: {e} '
                '| Bu döngü için veri yayınlanmayacak.'
            )
            return None

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