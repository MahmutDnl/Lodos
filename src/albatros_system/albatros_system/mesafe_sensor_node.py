#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros İnsansız Deniz Aracı — Mesafe Sensörü Publisher Node
# =============================================================================
# Dosya    : mesafe_sensor_node.py
# Node adı : mesafe_sensor_node
# Paket    : albatros_system
# Görev    : Mesafe sensöründen gelen uzaklık bilgisini okuyarak
#            /albatros/mesafe/range topic'ine yayınlamak.
#            Karar vermez, motor/mavros komutu göndermez; yalnızca veri üretir.
# Yazan    : LODOS Yazılım Ekibi
# Tarih    : 2026
# =============================================================================
#
# Bağımlılıklar:
#   - rclpy
#   - sensor_msgs (sensor_msgs/msg/Range)
#   - random (simülasyon modu için)
#
# Parametreler:
#   simulate_mode   (bool,  varsayılan: True)
#       True  → Fiziksel sensör olmadan test verisi üretir.
#       False → read_range_sensor() fonksiyonu üzerinden gerçek sensör okunur.
#
#   publish_rate    (float, varsayılan: 20.0 Hz)
#       Mesafe verisinin topic'e yayınlanma frekansı.
#       0 veya negatif değer girilirse otomatik 20.0 Hz kullanılır.
#
#   min_range       (float, varsayılan: 0.20 m)
#       Sensörün algılayabileceği minimum mesafe. Negatif ise varsayılana döner.
#
#   max_range       (float, varsayılan: 4.50 m)
#       Sensörün algılayabileceği maksimum mesafe. min_range'e eşit veya
#       küçük ise varsayılana döner.
#
#   field_of_view   (float, varsayılan: 0.26 rad)
#       Sensör görüş açısı (radyan). 0 veya negatif ise varsayılana döner.
#
#   fixed_range     (float, varsayılan: 2.0 m)
#       Simülasyon modunda üretilecek ortalama mesafe değeri.
#       min_range/max_range dışına çıkarsa sınırlar içine alınır.
#
#   range_noise     (float, varsayılan: 0.05 m)
#       Simülasyon modunda mesafe değerine eklenecek Gaussian gürültünün
#       standart sapması. Negatif ise varsayılana döner.
#
# Yayınlanan Topic:
#   /albatros/mesafe/range  [sensor_msgs/msg/Range]
#
# Uyumluluk:
#   Bu topic costmap ve karar node'ları tarafından okunabilir.
#   Topic adı KESİNLİKLE değiştirilmemelidir.
# =============================================================================

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


# =============================================================================
# Sabitler
# =============================================================================

# Mesafe sensörü yayın topic adı — değiştirilmemelidir!
RANGE_TOPIC = '/albatros/mesafe/range'

# TF çerçeve kimliği — URDF/TF ağacında sensör montaj noktasına karşılık gelir
RANGE_FRAME_ID = 'range_sensor_link'

# Varsayılan yayın frekansı (Hz) — geçersiz değer girilirse bu kullanılır
DEFAULT_PUBLISH_RATE = 20.0

# Varsayılan sensör parametreleri
DEFAULT_MIN_RANGE     = 0.20   # metre — JSN-SR04T minimum algılama mesafesi
DEFAULT_MAX_RANGE     = 4.50   # metre — JSN-SR04T maksimum algılama mesafesi
DEFAULT_FIELD_OF_VIEW = 0.26   # radyan — yaklaşık 15 derece
DEFAULT_FIXED_RANGE   = 2.0    # metre — simülasyon ortalama mesafesi
DEFAULT_RANGE_NOISE   = 0.05   # metre — simülasyon Gaussian gürültü std sapması


# =============================================================================
# Mesafe Sensörü Node Sınıfı
# =============================================================================

class MesafeSensorNode(Node):
    """
    Mesafe verisini /albatros/mesafe/range topic'ine yayınlayan publisher node.

    simulate_mode=True  → fixed_range etrafında gürültülü sahte mesafe üretir.
    simulate_mode=False → read_range_sensor() üzerinden gerçek sensörden okur.
                          Sensör henüz uygulanmamışsa tek seferlik uyarı verir,
                          node kapanmaz ve o döngüde veri yayınlamaz.
    """

    def __init__(self):
        super().__init__('mesafe_sensor_node')

        # ------------------------------------------------------------------ #
        # Parametreleri tanımla ve oku
        # ------------------------------------------------------------------ #
        self.declare_parameter('simulate_mode',   True)
        self.declare_parameter('publish_rate',    DEFAULT_PUBLISH_RATE)
        self.declare_parameter('min_range',       DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range',       DEFAULT_MAX_RANGE)
        self.declare_parameter('field_of_view',   DEFAULT_FIELD_OF_VIEW)
        self.declare_parameter('fixed_range',     DEFAULT_FIXED_RANGE)
        self.declare_parameter('range_noise',     DEFAULT_RANGE_NOISE)

        self.simulate_mode = self.get_parameter('simulate_mode').value
        raw_rate           = self.get_parameter('publish_rate').value
        raw_min_range      = self.get_parameter('min_range').value
        raw_max_range      = self.get_parameter('max_range').value
        raw_fov            = self.get_parameter('field_of_view').value
        raw_fixed          = self.get_parameter('fixed_range').value
        raw_noise          = self.get_parameter('range_noise').value

        # ------------------------------------------------------------------ #
        # Parametre güvenlik kontrolleri
        # ------------------------------------------------------------------ #

        # publish_rate — 0 veya negatif değere izin verilmez
        if raw_rate <= 0.0:
            self.get_logger().warn(
                f'publish_rate={raw_rate} geçersiz. '
                f'Varsayılan {DEFAULT_PUBLISH_RATE} Hz kullanılıyor.'
            )
            self.publish_rate = DEFAULT_PUBLISH_RATE
        else:
            self.publish_rate = raw_rate

        # min_range — negatif değere izin verilmez
        if raw_min_range < 0.0:
            self.get_logger().warn(
                f'min_range={raw_min_range} negatif, geçersiz. '
                f'Varsayılan {DEFAULT_MIN_RANGE} m kullanılıyor.'
            )
            self.min_range = DEFAULT_MIN_RANGE
        else:
            self.min_range = raw_min_range

        # max_range — min_range'e eşit veya küçük olamaz
        if raw_max_range <= self.min_range:
            self.get_logger().warn(
                f'max_range={raw_max_range} min_range={self.min_range} '
                f'değerinden küçük veya eşit, geçersiz. '
                f'Varsayılan {DEFAULT_MAX_RANGE} m kullanılıyor.'
            )
            self.max_range = DEFAULT_MAX_RANGE
        else:
            self.max_range = raw_max_range

        # field_of_view — 0 veya negatif değere izin verilmez
        if raw_fov <= 0.0:
            self.get_logger().warn(
                f'field_of_view={raw_fov} geçersiz (>0 olmalı). '
                f'Varsayılan {DEFAULT_FIELD_OF_VIEW} rad kullanılıyor.'
            )
            self.field_of_view = DEFAULT_FIELD_OF_VIEW
        else:
            self.field_of_view = raw_fov

        # range_noise — negatif değere izin verilmez
        if raw_noise < 0.0:
            self.get_logger().warn(
                f'range_noise={raw_noise} negatif, geçersiz. '
                f'Varsayılan {DEFAULT_RANGE_NOISE} m kullanılıyor.'
            )
            self.range_noise = DEFAULT_RANGE_NOISE
        else:
            self.range_noise = raw_noise

        # fixed_range — min/max sınırları dışındaysa sınırlar içine al
        clamped_fixed = max(self.min_range, min(raw_fixed, self.max_range))
        if clamped_fixed != raw_fixed:
            self.get_logger().warn(
                f'fixed_range={raw_fixed} m sensör aralığı dışında '
                f'[{self.min_range}, {self.max_range}]. '
                f'{clamped_fixed} m olarak sınırlandırıldı.'
            )
        self.fixed_range = clamped_fixed

        # ------------------------------------------------------------------ #
        # Uyarı spam önleyici — gerçek sensör henüz uygulanmadıysa
        # yalnızca ilk döngüde bir kez uyarı ver, sonrakilerde sessiz kal
        # ------------------------------------------------------------------ #
        self._real_sensor_warning_printed = False

        # ------------------------------------------------------------------ #
        # Publisher
        # ------------------------------------------------------------------ #
        self.range_publisher = self.create_publisher(
            Range,
            RANGE_TOPIC,
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
        self.get_logger().info('Mesafe Sensörü Node başlatıldı.')
        self.get_logger().info(f'  Mod             : {mode_str}')
        self.get_logger().info(f'  Topic           : {RANGE_TOPIC}')
        self.get_logger().info(f'  Yayın frekansı  : {self.publish_rate} Hz')
        self.get_logger().info(f'  Frame ID        : {RANGE_FRAME_ID}')
        self.get_logger().info(f'  min_range       : {self.min_range} m')
        self.get_logger().info(f'  max_range       : {self.max_range} m')
        self.get_logger().info(f'  field_of_view   : {self.field_of_view} rad')
        if self.simulate_mode:
            self.get_logger().info(f'  fixed_range     : {self.fixed_range} m')
            self.get_logger().info(f'  range_noise     : {self.range_noise} m (std)')
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
            range_data = self.generate_simulated_range()
        else:
            range_data = self.read_range_sensor()

        if range_data is not None:
            self.range_publisher.publish(range_data)

    # ================================================================== #
    # Simülasyon modu
    # ================================================================== #

    def generate_simulated_range(self) -> Range:
        """
        Geçerli bir sensor_msgs/Range mesajı oluşturur.

        - fixed_range etrafında Gaussian gürültü eklenerek mesafe üretilir.
        - Üretilen değer [min_range, max_range] sınırları içinde tutulur.
        - radiation_type = ULTRASOUND (JSN-SR04T ultrasonik sensör varsayımı).

        Dönüş:
            sensor_msgs.msg.Range: doldurulmuş mesaj
        """
        msg = Range()

        # Header
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = RANGE_FRAME_ID

        # Sensör tipi — ultrasonik
        msg.radiation_type = Range.ULTRASOUND

        # Sensör parametreleri — parametre değerlerinden alınır
        msg.field_of_view = self.field_of_view
        msg.min_range     = self.min_range
        msg.max_range     = self.max_range

        # Mesafe değeri — fixed_range + Gaussian gürültü, sınırlar içinde kalsın
        raw_distance = self.fixed_range + random.gauss(0.0, self.range_noise)
        msg.range = float(max(self.min_range, min(raw_distance, self.max_range)))

        return msg

    # ================================================================== #
    # Gerçek sensör okuma iskelet fonksiyonu
    # ================================================================== #

    def read_range_sensor(self):
        """
        Fiziksel mesafe sensöründen veri okuyarak sensor_msgs/Range mesajı döndürür.

        Bu fonksiyon şu anda bir iskelet olarak bırakılmıştır.
        Gerçek sensör (JSN-SR04T veya benzeri) bağlandıktan sonra
        aşağıdaki TODO bölümleri doldurulmalıdır.

        Desteklenen bağlantı seçenekleri:
          SEÇENEK 1 — GPIO Trigger/Echo (RPi.GPIO veya gpiozero, JSN-SR04T)
          SEÇENEK 2 — UART (pyserial, bazı ultrasonik modüller)
          SEÇENEK 3 — I2C  (örn. VL53L0X, VL53L1X lazer mesafe sensörleri)

        Dönüş:
            sensor_msgs.msg.Range  — başarılı okumada dolu mesaj
            None                   — okuma başarısızsa (yayın yapılmaz, node durmaz)
        """
        try:
            # -------------------------------------------------------------- #
            # TODO: Gerçek sensör seçildikten sonra burası düzenlenecek.
            #
            # SEÇENEK 1 — GPIO Trigger/Echo (JSN-SR04T, HC-SR04, vs.):
            #   import RPi.GPIO as GPIO
            #   import time
            #
            #   TRIG_PIN = 23   # TODO: Kullanılan GPIO pin numarasını girin
            #   ECHO_PIN = 24   # TODO: Kullanılan GPIO pin numarasını girin
            #
            #   # GPIO başlatma __init__ içinde yapılmalı:
            #   # GPIO.setmode(GPIO.BCM)
            #   # GPIO.setup(TRIG_PIN, GPIO.OUT)
            #   # GPIO.setup(ECHO_PIN, GPIO.IN)
            #
            #   GPIO.output(TRIG_PIN, True)
            #   time.sleep(0.00001)          # 10 µs tetik sinyali
            #   GPIO.output(TRIG_PIN, False)
            #
            #   pulse_start = time.time()
            #   while GPIO.input(ECHO_PIN) == 0:
            #       pulse_start = time.time()
            #   pulse_end = time.time()
            #   while GPIO.input(ECHO_PIN) == 1:
            #       pulse_end = time.time()
            #
            #   pulse_duration = pulse_end - pulse_start
            #   distance = pulse_duration * 17150.0  # ses hızı / 2 (cm)
            #   distance = distance / 100.0           # cm → metre
            #
            # SEÇENEK 2 — UART (pyserial örneği):
            #   import serial
            #   ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
            #   # TODO: Sensörün UART protokolüne göre okuma yapın
            #   raw = ser.readline()
            #   distance = float(raw.decode().strip()) / 1000.0  # mm → m
            #
            # SEÇENEK 3 — I2C (VL53L0X lazer mesafe sensörü):
            #   import VL53L0X
            #   tof = VL53L0X.VL53L0X()   # __init__ içinde başlatılmalı
            #   tof.start_ranging()
            #   distance_mm = tof.get_distance()
            #   tof.stop_ranging()
            #   distance = distance_mm / 1000.0  # mm → m
            #
            # Sensör başlatma işlemlerini (port açma, GPIO init, vs.)
            # __init__ içinde yapın; her callback'te yeniden açmayın.
            # -------------------------------------------------------------- #

            # Gerçek sensör henüz uygulanmadı — bu satırı gerçek kodla değiştir!
            raise NotImplementedError(
                'read_range_sensor() henüz uygulanmadı. '
                'simulate_mode:=true parametresini kullanın.'
            )

            # -------------------------------------------------------------- #
            # TODO: Sensörden distance değeri okunduktan sonra mesajı doldurun.
            # -------------------------------------------------------------- #
            # distance = ...   # metre cinsinden, yukarıdaki seçeneklerden biri

            msg = Range()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = RANGE_FRAME_ID

            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view  = self.field_of_view
            msg.min_range      = self.min_range
            msg.max_range      = self.max_range

            # TODO: Okunan mesafe değerini buraya atayın
            msg.range = 0.0  # → distance ile değiştirin

            return msg

        except NotImplementedError as e:
            # Uyarı spam önlemi: yalnızca ilk döngüde bir kez uyarı ver
            if not self._real_sensor_warning_printed:
                self.get_logger().warn(
                    f'Mesafe sensörü okuma henüz uygulanmadı: {e} '
                    '| Bu uyarı bir kez gösterilir. '
                    '| simulate_mode:=true parametresini kullanabilirsiniz.'
                )
                self._real_sensor_warning_printed = True
            return None

        except Exception as e:
            # Beklenmedik sensör hatası — node kapanmaz, bu döngü atlanır
            self.get_logger().error(
                f'Mesafe sensörü okuma hatası: {e} '
                '| Bu döngü için veri yayınlanmayacak.'
            )
            return None


# =============================================================================
# Giriş noktası
# =============================================================================

def main(args=None):
    """
    Node başlangıç fonksiyonu.
    Çağrı: ros2 run albatros_system mesafe_sensor_node
    """
    rclpy.init(args=args)
    node = MesafeSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mesafe Sensörü Node durduruldu (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
