#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros Insansiz Deniz Araci — Mesafe Sensoru Publisher Node
# =============================================================================
# Dosya    : mesafe_sensor_node.py
# Node adi : mesafe_sensor_node
# Paket    : albatros_system / albatros_tahta
# Gorev    : Aracin on sol ve on sag konumlarindaki iki adet ultrasonik mesafe
#            sensorunden (Arduino'ya bagli) gelen uzaklik bilgisini USB Serial
#            uzerinden okuyarak ROS 2 sensor_msgs/msg/Range topic'lerine yayinlamak.
#
# Mimari   : 2 adet ultrasonik mesafe sensörü -> Arduino -> USB Serial -> RPi ->
#            mesafe_sensor_node.py -> ROS 2 Topicleri
#
# Format   : Arduino'dan gelen serial veriler SENSOR1_CM,SENSOR2_CM formatindadir.
#            (Ornek: "179,145"). Node bu veriyi metreye çevirerek yayinlar.
#            -1 veya <=0 olan veriler gecersiz kabul edilir ve yayinlanmaz.
#
# Yazan    : LODOS Yazilim Ekibi
# Tarih    : 2026
# =============================================================================
#
# Bagimliliklar:
#   - rclpy
#   - sensor_msgs  (sensor_msgs/msg/Range)
#   - pyserial     (import serial)
#   - random       (simulasyon modu icin)
#
# Yayinlanan Topic'ler (sensor_msgs/msg/Range):
#   /albatros/mesafe/on_sol   — on sol sensor (SENSOR 1)
#   /albatros/mesafe/on_sag   — on sag sensor (SENSOR 2)
#
# Genel Parametreler:
#   simulate_mode      (bool,  varsayilan: True)
#   publish_rate       (float, varsayilan: 20.0 Hz)
#   min_range          (float, varsayilan: 0.20 m)
#   max_range          (float, varsayilan: 4.50 m)
#   field_of_view      (float, varsayilan: 0.26 rad)
#   range_noise        (float, varsayilan: 0.05 m std)
#   log_measurements   (bool,  varsayilan: False)
#
# Gercek Sensor Serial Parametreleri:
#   serial_port        (str,   varsayilan: '/dev/ttyUSB0')
#   baud_rate          (int,   varsayilan: 115200)
#   serial_timeout     (float, varsayilan: 0.05)
#
# Simulasyon Parametreleri (her sensor icin):
#   on_sol_fixed_range (float, varsayilan: 2.0 m)
#   on_sag_fixed_range (float, varsayilan: 2.5 m)
#
# Uyumluluk:
#   ROS 2 Jazzy (Ubuntu 24.04). Python 3.
# =============================================================================

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import serial
except ImportError:
    serial = None


# =============================================================================
# Sabitler — varsayilan degerler
# =============================================================================

DEFAULT_PUBLISH_RATE   = 20.0   # Hz
DEFAULT_MIN_RANGE      = 0.20   # metre
DEFAULT_MAX_RANGE      = 4.50   # metre
DEFAULT_FIELD_OF_VIEW  = 0.26   # radyan (~15 derece)
DEFAULT_RANGE_NOISE    = 0.05   # metre (Gaussian std)

DEFAULT_SERIAL_PORT    = '/dev/ttyUSB0'
DEFAULT_BAUD_RATE      = 115200
DEFAULT_SERIAL_TIMEOUT = 0.05   # saniye

LOG_THROTTLE_SEC       = 3.0


# =============================================================================
# Sensor tanimlari — tek noktada degistirilebilir yapi
# =============================================================================

SENSOR_DEFINITIONS = {
    'on_sol': {
        'topic':               '/albatros/mesafe/on_sol',
        'frame_id':            'on_sol_sensor_link',
        'label':               '[ON SOL] ',
        'default_fixed_range': 2.0,
    },
    'on_sag': {
        'topic':               '/albatros/mesafe/on_sag',
        'frame_id':            'on_sag_sensor_link',
        'label':               '[ON SAG] ',
        'default_fixed_range': 2.5,
    },
}


# =============================================================================
# Node Sinifi
# =============================================================================

class MesafeSensorNode(Node):
    """
    Arduino USB Serial uzerinden iki adet ultrasonik mesafe sensorunden
    alinan verileri ayri ROS 2 topic'lerine yayinlayan publisher node.

    simulate_mode=True  → Her sensor icin ayri sabit mesafe + Gaussian gurultu.
    simulate_mode=False → USB Serial (/dev/ttyUSB0) uzerinden Arduino verisi okur.
    """

    def __init__(self):
        super().__init__('mesafe_sensor_node')

        # ------------------------------------------------------------------ #
        # Genel parametreler
        # ------------------------------------------------------------------ #
        self.declare_parameter('simulate_mode',    True)
        self.declare_parameter('publish_rate',     DEFAULT_PUBLISH_RATE)
        self.declare_parameter('min_range',        DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range',        DEFAULT_MAX_RANGE)
        self.declare_parameter('field_of_view',    DEFAULT_FIELD_OF_VIEW)
        self.declare_parameter('range_noise',      DEFAULT_RANGE_NOISE)
        self.declare_parameter('log_measurements', False)

        # ------------------------------------------------------------------ #
        # Serial parametreleri (Gercek sensor modu)
        # ------------------------------------------------------------------ #
        self.declare_parameter('serial_port',    DEFAULT_SERIAL_PORT)
        self.declare_parameter('baud_rate',      DEFAULT_BAUD_RATE)
        self.declare_parameter('serial_timeout', DEFAULT_SERIAL_TIMEOUT)

        self.simulate_mode    = self.get_parameter('simulate_mode').value
        self.log_measurements = self.get_parameter('log_measurements').value
        self.serial_port      = str(self.get_parameter('serial_port').value)
        self.baud_rate        = int(self.get_parameter('baud_rate').value)
        self.serial_timeout   = float(self.get_parameter('serial_timeout').value)

        raw_rate  = self.get_parameter('publish_rate').value
        raw_min   = self.get_parameter('min_range').value
        raw_max   = self.get_parameter('max_range').value
        raw_fov   = self.get_parameter('field_of_view').value
        raw_noise = self.get_parameter('range_noise').value

        # ------------------------------------------------------------------ #
        # Parametre guvenlik kontrolleri
        # ------------------------------------------------------------------ #
        self.publish_rate = self._validate_positive(
            'publish_rate', raw_rate, DEFAULT_PUBLISH_RATE
        )
        self.min_range = self._validate_non_negative(
            'min_range', raw_min, DEFAULT_MIN_RANGE
        )
        self.max_range = self._validate_max_range(
            raw_max, self.min_range, DEFAULT_MAX_RANGE
        )
        self.field_of_view = self._validate_positive(
            'field_of_view', raw_fov, DEFAULT_FIELD_OF_VIEW
        )
        self.range_noise = self._validate_non_negative(
            'range_noise', raw_noise, DEFAULT_RANGE_NOISE
        )

        # ------------------------------------------------------------------ #
        # Sensor'e ozel parametreler (simulasyon sabit mesafe)
        # ------------------------------------------------------------------ #
        self.sensors = {}

        for name, defn in SENSOR_DEFINITIONS.items():
            fixed_param = f'{name}_fixed_range'
            self.declare_parameter(fixed_param, defn['default_fixed_range'])
            raw_fixed = self.get_parameter(fixed_param).value
            clamped   = max(self.min_range, min(raw_fixed, self.max_range))
            if clamped != raw_fixed:
                self.get_logger().warn(
                    f'{fixed_param}={raw_fixed} m sensor araligi disinda '
                    f'[{self.min_range}, {self.max_range}]. '
                    f'{clamped} m olarak sinirlandirildi.'
                )

            pub = self.create_publisher(Range, defn['topic'], 10)

            self.sensors[name] = {
                'topic':       defn['topic'],
                'frame_id':    defn['frame_id'],
                'label':       defn['label'],
                'fixed_range': clamped,
                'publisher':   pub,
            }

        # ------------------------------------------------------------------ #
        # Serial port baglantisi (gercek sensor modu)
        # ------------------------------------------------------------------ #
        self.serial_connection = None
        if not self.simulate_mode:
            self._init_serial()

        # ------------------------------------------------------------------ #
        # Olcum log throttle zamanlayici
        # ------------------------------------------------------------------ #
        self._last_log_time = 0.0

        # ------------------------------------------------------------------ #
        # Timer
        # ------------------------------------------------------------------ #
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # ------------------------------------------------------------------ #
        # Baslangic loglari
        # ------------------------------------------------------------------ #
        self._log_startup()

    # ================================================================== #
    # Parametre Dogrulama Yardimci Fonksiyonlari
    # ================================================================== #

    def _validate_positive(self, name: str, value: float, default: float) -> float:
        """Degerin kesinlikle pozitif olmasini zorunlu kilar."""
        if value <= 0.0:
            self.get_logger().warn(
                f'{name}={value} gecersiz (>0 olmali). '
                f'Varsayilan {default} kullaniliyor.'
            )
            return default
        return value

    def _validate_non_negative(self, name: str, value: float, default: float) -> float:
        """Degerin negatif olmamasini zorunlu kilar."""
        if value < 0.0:
            self.get_logger().warn(
                f'{name}={value} negatif, gecersiz. '
                f'Varsayilan {default} kullaniliyor.'
            )
            return default
        return value

    def _validate_max_range(
        self, value: float, min_range: float, default: float
    ) -> float:
        """max_range degerinin min_range'den buyuk olmasini zorunlu kilar."""
        if value <= min_range:
            self.get_logger().warn(
                f'max_range={value} min_range={min_range} degerinden '
                f'kucuk veya esit, gecersiz. Varsayilan {default} m kullaniliyor.'
            )
            return default
        return value

    # ================================================================== #
    # Serial Port Baslatma
    # ================================================================== #

    def _init_serial(self):
        """
        Gercek sensor modunda Arduino USB Serial portunu bir kez acmaya calisir.
        Hata durumunda node çökmez, log basar ve serial_connection None kalir.
        """
        if serial is None:
            self.get_logger().error(
                'pyserial modulu (import serial) bulunamadi. '
                'Lutfen "sudo apt install python3-serial" komutu ile yukleyin.'
            )
            self.serial_connection = None
            return

        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout
            )
            self.get_logger().info(
                f'Arduino serial bağlantısı açıldı: {self.serial_port} @ {self.baud_rate}'
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Serial port açılamadı: {e}')
            self.serial_connection = None
        except Exception as e:
            self.get_logger().error(f'Beklenmeyen serial bağlantı hatası: {e}')
            self.serial_connection = None

    # ================================================================== #
    # Serial Veri Okuma
    # ================================================================== #

    def _read_serial_distances(self):
        """
        Arduino USB Serial portundan tek bir satir okur ve 2 sensörün mesafelerini m cinsinden dondurur.
        Gelen veri formati: SENSOR_1_CM,SENSOR_2_CM (Ornek: "179,145")

        Returns:
            (s1_m, s2_m) şeklinde tuple veya okunacak veri yoksa/hataliysa None.
            s1_m veya s2_m gecersiz (-1 veya <= 0) ise o eleman None olur.
        """
        if self.serial_connection is None or not self.serial_connection.is_open:
            return None

        try:
            if self.serial_connection.in_waiting == 0:
                return None

            raw_line = self.serial_connection.readline()
            if not raw_line:
                return None

            line_str = raw_line.decode('utf-8', errors='ignore').strip()
            if not line_str:
                return None

            parts = line_str.split(',')
            if len(parts) != 2:
                return None

            s1_cm = float(parts[0])
            s2_cm = float(parts[1])

            s1_m = (s1_cm / 100.0) if s1_cm > 0 else None
            s2_m = (s2_cm / 100.0) if s2_cm > 0 else None

            return s1_m, s2_m

        except ValueError:
            # Sayisal olmayan hatali format
            return None
        except Exception as e:
            if serial and isinstance(e, serial.SerialException):
                self.get_logger().error(f'Serial okuma hatasi: {e}')
            else:
                self.get_logger().error(f'Beklenmeyen serial okuma hatasi: {e}')
            return None

    # ================================================================== #
    # Baslangic Logu
    # ================================================================== #

    def _log_startup(self):
        """Node baslarken tum konfigurasyonu ekrana yazar."""
        mode_str = (
            'SIMULASYON (simulate_mode=True)'
            if self.simulate_mode
            else 'GERCEK SENSOR - ARDUINO USB SERIAL (simulate_mode=False)'
        )
        sep = '=' * 64

        self.get_logger().info(sep)
        self.get_logger().info('Mesafe Sensoru Node baslatildi. (2 sensor)')
        self.get_logger().info(f'  Mod            : {mode_str}')
        self.get_logger().info(f'  Yayin frekansi : {self.publish_rate} Hz')
        self.get_logger().info(f'  min_range      : {self.min_range} m')
        self.get_logger().info(f'  max_range      : {self.max_range} m')
        self.get_logger().info(f'  field_of_view  : {self.field_of_view} rad')
        if self.simulate_mode:
            self.get_logger().info(
                f'  range_noise    : {self.range_noise} m (Gaussian std)'
            )
        else:
            self.get_logger().info(f'  Serial port    : {self.serial_port}')
            self.get_logger().info(f'  Baud rate      : {self.baud_rate}')
            self.get_logger().info(f'  Serial timeout : {self.serial_timeout} s')
        self.get_logger().info('  Sensorler:')
        for name, s in self.sensors.items():
            if self.simulate_mode:
                self.get_logger().info(
                    f'    {s["label"]}  topic={s["topic"]}  '
                    f'frame={s["frame_id"]}  '
                    f'fixed={s["fixed_range"]} m'
                )
            else:
                self.get_logger().info(
                    f'    {s["label"]}  topic={s["topic"]}  '
                    f'frame={s["frame_id"]}'
                )
        self.get_logger().info(sep)

    # ================================================================== #
    # Timer Callback
    # ================================================================== #

    def timer_callback(self):
        """
        Belirlenen frekansta cagirilir.
        simulate_mode=True  → on_sol ve on_sag icin simule edilmis mesajlar uretir.
        simulate_mode=False → Serial porttan TEK SATIR okur, gecerli olan sensorler icin mesaj yayinlar.
        """
        log_lines = []

        if self.simulate_mode:
            for name in self.sensors:
                msg = self.generate_simulated_distance(name)
                if msg is not None:
                    self.sensors[name]['publisher'].publish(msg)
                    if self.log_measurements:
                        log_lines.append(
                            f'{self.sensors[name]["label"]} {msg.range:.2f} metre (Sim).'
                        )
        else:
            distances = self._read_serial_distances()
            if distances is None:
                return

            s1_m, s2_m = distances

            # Sensor 1 -> on_sol
            if s1_m is not None:
                msg1 = self.create_range_message('on_sol', s1_m)
                self.sensors['on_sol']['publisher'].publish(msg1)
                if self.log_measurements:
                    log_lines.append(
                        f'{self.sensors["on_sol"]["label"]} {msg1.range:.2f} metre.'
                    )

            # Sensor 2 -> on_sag
            if s2_m is not None:
                msg2 = self.create_range_message('on_sag', s2_m)
                self.sensors['on_sag']['publisher'].publish(msg2)
                if self.log_measurements:
                    log_lines.append(
                        f'{self.sensors["on_sag"]["label"]} {msg2.range:.2f} metre.'
                    )

        # Olcum loglarini throttle ile yazdir
        if self.log_measurements and log_lines:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_log_time >= LOG_THROTTLE_SEC:
                for line in log_lines:
                    self.get_logger().info(line)
                self._last_log_time = now

    # ================================================================== #
    # Ortak Mesaj Olusturucu
    # ================================================================== #

    def create_range_message(self, sensor_name: str, distance: float) -> Range:
        """
        Verilen sensor adi ve mesafe degerinden sensor_msgs/Range mesaji olusturur.

        Args:
            sensor_name: 'on_sol', 'on_sag'
            distance:    Metre cinsinden olculen mesafe.

        Returns:
            Doldurulmus sensor_msgs/msg/Range nesnesi.
        """
        sensor = self.sensors[sensor_name]

        msg = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = sensor['frame_id']
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = self.field_of_view
        msg.min_range       = self.min_range
        msg.max_range       = self.max_range
        msg.range           = float(
            max(self.min_range, min(distance, self.max_range))
        )
        return msg

    # ================================================================== #
    # Simulasyon Modu
    # ================================================================== #

    def generate_simulated_distance(self, sensor_name: str) -> Range:
        """
        Simulasyon modunda calisiyor.
        Sensor'e ozel fixed_range degerine Gaussian gurultu ekleyerek
        gercekci bir Range mesaji uretir.

        Args:
            sensor_name: Sensor anahtari ('on_sol', 'on_sag')

        Returns:
            sensor_msgs/msg/Range
        """
        fixed  = self.sensors[sensor_name]['fixed_range']
        noisy  = fixed + random.gauss(0.0, self.range_noise)
        return self.create_range_message(sensor_name, noisy)

    # ================================================================== #
    # Node Kapanisi
    # ================================================================== #

    def destroy_node(self):
        """
        Node kapanirken Serial baglantisini guvenli sekilde kapatir.
        """
        if hasattr(self, 'serial_connection') and self.serial_connection is not None:
            try:
                if self.serial_connection.is_open:
                    self.serial_connection.close()
                    self.get_logger().info('Arduino serial bağlantısı kapatıldı.')
            except Exception as e:
                self.get_logger().warn(f'Serial port kapatma hatasi: {e}')
        super().destroy_node()


# =============================================================================
# Giris Noktasi
# =============================================================================

def main(args=None):
    """
    Node baslangic fonksiyonu.
    Cagri: ros2 run albatros_tahta mesafe_sensor_node
    """
    rclpy.init(args=args)
    node = MesafeSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'Mesafe Sensoru Node durduruldu (KeyboardInterrupt).'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
