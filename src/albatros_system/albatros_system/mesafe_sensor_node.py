#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros Insansiz Deniz Araci — Mesafe Sensoru Publisher Node
# =============================================================================
# Dosya    : mesafe_sensor_node.py
# Node adi : mesafe_sensor_node
# Paket    : albatros_system
# Gorev    : Aracin on sol, on sag, yan sol ve yan sag konumlarindaki dort adet
#            JSN-SR04T ultrasonik mesafe sensorunden gelen uzaklik bilgisini
#            okuyarak her birine ozel ROS 2 topic'ine yayinlamak.
#            Karar vermez, motor/mavros komutu gondermez; yalnizca veri uretir.
# Yazan    : LODOS Yazilim Ekibi
# Tarih    : 2026
# =============================================================================
#
# Bagimliliklar:
#   - rclpy
#   - sensor_msgs  (sensor_msgs/msg/Range)
#   - random       (simulasyon modu icin)
#
# Yayinlanan Topic'ler (sensor_msgs/msg/Range):
#   /albatros/mesafe/on_sol   — on sol sensor
#   /albatros/mesafe/on_sag   — on sag sensor
#   /albatros/mesafe/yan_sol  — yan sol sensor
#   /albatros/mesafe/yan_sag  — yan sag sensor
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
# Simulasyon Parametreleri (her sensor icin):
#   on_sol_fixed_range   (float, varsayilan: 2.0 m)
#   on_sag_fixed_range   (float, varsayilan: 2.5 m)
#   yan_sol_fixed_range  (float, varsayilan: 3.0 m)
#   yan_sag_fixed_range  (float, varsayilan: 4.0 m)
#
# Gercek Sensor GPIO Parametreleri:
#   on_sol_trigger_pin   (int, varsayilan: 17)
#   on_sol_echo_pin      (int, varsayilan: 18)
#   on_sag_trigger_pin   (int, varsayilan: 22)
#   on_sag_echo_pin      (int, varsayilan: 23)
#   yan_sol_trigger_pin  (int, varsayilan: 24)
#   yan_sol_echo_pin     (int, varsayilan: 25)
#   yan_sag_trigger_pin  (int, varsayilan: 5)
#   yan_sag_echo_pin     (int, varsayilan: 6)
#
# Uyumluluk:
#   ROS 2 Jazzy (Ubuntu 24.04). rclpy.spin() yapisi korunmustur.
# =============================================================================

import random
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


# =============================================================================
# Sabitler — varsayilan degerler
# =============================================================================

DEFAULT_PUBLISH_RATE  = 20.0   # Hz
DEFAULT_MIN_RANGE     = 0.20   # metre
DEFAULT_MAX_RANGE     = 4.50   # metre
DEFAULT_FIELD_OF_VIEW = 0.26   # radyan (~15 derece)
DEFAULT_RANGE_NOISE   = 0.05   # metre (Gaussian std)

# GPIO okuma timeout'u — echo sinyali bu sure gelmezse okuma iptal edilir
GPIO_ECHO_TIMEOUT_SEC = 0.05   # saniye

# Olcum logu throttle suresi — her N saniyede bir basilir
LOG_THROTTLE_SEC = 3.0


# =============================================================================
# Sensor tanimlari — tek noktada degistirilebilir yapi
# =============================================================================

SENSOR_DEFINITIONS = {
    'on_sol': {
        'topic':    '/albatros/mesafe/on_sol',
        'frame_id': 'on_sol_sensor_link',
        'label':    '[ON SOL] ',
        'default_fixed_range':  2.0,
        'default_trigger_pin':  17,
        'default_echo_pin':     18,
    },
    'on_sag': {
        'topic':    '/albatros/mesafe/on_sag',
        'frame_id': 'on_sag_sensor_link',
        'label':    '[ON SAG] ',
        'default_fixed_range':  2.5,
        'default_trigger_pin':  22,
        'default_echo_pin':     23,
    },
    'yan_sol': {
        'topic':    '/albatros/mesafe/yan_sol',
        'frame_id': 'yan_sol_sensor_link',
        'label':    '[YAN SOL]',
        'default_fixed_range':  3.0,
        'default_trigger_pin':  24,
        'default_echo_pin':     25,
    },
    'yan_sag': {
        'topic':    '/albatros/mesafe/yan_sag',
        'frame_id': 'yan_sag_sensor_link',
        'label':    '[YAN SAG]',
        'default_fixed_range':  4.0,
        'default_trigger_pin':  5,
        'default_echo_pin':     6,
    },
}


# =============================================================================
# Node Sinifi
# =============================================================================

class MesafeSensorNode(Node):
    """
    Dort JSN-SR04T ultrasonik mesafe sensorunden alinan verileri
    ayri ROS 2 topic'lerine yayinlayan publisher node.

    simulate_mode=True  → Her sensor icin ayri sabit mesafe + Gaussian gurultu.
    simulate_mode=False → GPIO Trigger/Echo ile gercek sensor okumasi (iskelet).
                          Sensor henuz uygulanmamissa tek sefer uyari verir;
                          node kapanmaz, ilgili sensorun o dongusu atlanir.
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

        self.simulate_mode    = self.get_parameter('simulate_mode').value
        self.log_measurements = self.get_parameter('log_measurements').value

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
        # Sensor'e ozel parametreler (simulasyon sabit mesafe + GPIO pinleri)
        # ------------------------------------------------------------------ #
        self.sensors = {}

        for name, defn in SENSOR_DEFINITIONS.items():
            # Simulasyon sabit mesafesi
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

            # GPIO pin parametreleri
            trig_param = f'{name}_trigger_pin'
            echo_param = f'{name}_echo_pin'
            self.declare_parameter(trig_param, defn['default_trigger_pin'])
            self.declare_parameter(echo_param, defn['default_echo_pin'])
            trigger_pin = self.get_parameter(trig_param).value
            echo_pin    = self.get_parameter(echo_param).value

            # Publisher
            pub = self.create_publisher(Range, defn['topic'], 10)

            self.sensors[name] = {
                'topic':       defn['topic'],
                'frame_id':    defn['frame_id'],
                'label':       defn['label'],
                'fixed_range': clamped,
                'trigger_pin': trigger_pin,
                'echo_pin':    echo_pin,
                'publisher':   pub,
                # Uyari spam onleyici — NotImplementedError icin
                'warned':      False,
            }

        # ------------------------------------------------------------------ #
        # GPIO baslatma (gercek sensor modu)
        # Simulasyon modunda atlanir; gercek modda iskelet hazir.
        # ------------------------------------------------------------------ #
        self._gpio_initialized = False
        if not self.simulate_mode:
            self._init_gpio()

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
    # Baslangic Logu
    # ================================================================== #

    def _log_startup(self):
        """Node baslarken tum konfigurasyonu ekrana yazar."""
        mode_str = (
            'SIMULASYON (simulate_mode=True)'
            if self.simulate_mode
            else 'GERCEK SENSOR (simulate_mode=False)'
        )
        sep = '=' * 64

        self.get_logger().info(sep)
        self.get_logger().info('Mesafe Sensoru Node baslatildi. (4 sensor)')
        self.get_logger().info(f'  Mod            : {mode_str}')
        self.get_logger().info(f'  Yayin frekansi : {self.publish_rate} Hz')
        self.get_logger().info(f'  min_range      : {self.min_range} m')
        self.get_logger().info(f'  max_range      : {self.max_range} m')
        self.get_logger().info(f'  field_of_view  : {self.field_of_view} rad')
        if self.simulate_mode:
            self.get_logger().info(
                f'  range_noise    : {self.range_noise} m (Gaussian std)'
            )
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
                    f'frame={s["frame_id"]}  '
                    f'TRIG={s["trigger_pin"]}  ECHO={s["echo_pin"]}'
                )
        self.get_logger().info(sep)

    # ================================================================== #
    # Timer Callback
    # ================================================================== #

    def timer_callback(self):
        """
        Belirlenen frekansta cagirilir.
        Her sensor icin sirasyla olcum yapilir ve kendi topic'ine yayinlanir.
        Bir sensorde hata olusursa yalnizca o sensorum o dongusu atlanir;
        diger sensorler etkilenmez.
        """
        log_lines = []

        for name, sensor in self.sensors.items():
            if self.simulate_mode:
                msg = self.generate_simulated_distance(name)
            else:
                msg = self.read_real_distance(name)

            if msg is not None:
                sensor['publisher'].publish(msg)
                if self.log_measurements:
                    log_lines.append(
                        f'{sensor["label"]} {msg.range:.2f} metre mesafe olculdü.'
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
            sensor_name: 'on_sol', 'on_sag', 'yan_sol', 'yan_sag'
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
            sensor_name: Sensor anahtari ('on_sol', 'on_sag', ...)

        Returns:
            sensor_msgs/msg/Range
        """
        fixed  = self.sensors[sensor_name]['fixed_range']
        noisy  = fixed + random.gauss(0.0, self.range_noise)
        return self.create_range_message(sensor_name, noisy)

    # ================================================================== #
    # Gercek Sensor Okuma (iskelet + GPIO altyapisi)
    # ================================================================== #

    def _init_gpio(self):
        """
        Gercek sensor modunda GPIO pinlerini baslatir.
        Baslatma islemi yalnizca __init__ icerisinden cagirilir;
        timer callback icinde tekrarlanmaz.

        NOT: RPi.GPIO veya lgpio gibi bir kutuphane kurulu olmalidir.
             Kutuphane bulunamazsa node uyari verir; simulate_mode=True'ya
             gecmez ancak gercek okuma basarisiz olacaktir.
        """
        try:
            import RPi.GPIO as GPIO  # noqa: F401
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            for name, sensor in self.sensors.items():
                GPIO.setup(sensor['trigger_pin'], GPIO.OUT)
                GPIO.setup(sensor['echo_pin'],    GPIO.IN)
                # Trigger'i pasif konumda birak
                GPIO.output(sensor['trigger_pin'], False)

            self._gpio_initialized = True
            self.get_logger().info(
                'GPIO pinleri basariyla baslatildi (RPi.GPIO).'
            )

        except ImportError:
            self.get_logger().warn(
                'RPi.GPIO kutuphanesi bulunamadi. '
                'Gercek sensor okumasi calismaycak. '
                'simulate_mode:=true parametresini kullanin.'
            )
        except Exception as e:
            self.get_logger().error(
                f'GPIO baslatma hatasi: {e}. '
                'Gercek sensor okumasi calismaycak.'
            )

    def _read_gpio_distance(self, sensor_name: str) -> float | None:
        """
        JSN-SR04T icin GPIO Trigger/Echo protokolu ile mesafe okur.

        - 10 µs tetik sinyali gonderilir.
        - Echo sinyalinin yukselmesi ve dusmesi beklenir.
        - GPIO_ECHO_TIMEOUT_SEC suresi icerisinde sinyal gelmezse None doner.
        - Sonsuz while dongusu yoktur; her adimda timeout kontrolu yapilir.

        Args:
            sensor_name: 'on_sol', 'on_sag', 'yan_sol', 'yan_sag'

        Returns:
            Metre cinsinden mesafe veya None (timeout/hata durumunda).
        """
        try:
            import RPi.GPIO as GPIO

            trig = self.sensors[sensor_name]['trigger_pin']
            echo = self.sensors[sensor_name]['echo_pin']

            # 10 µs tetik sinyali
            GPIO.output(trig, True)
            time.sleep(0.00001)
            GPIO.output(trig, False)

            # Echo yukselmesini bekle
            deadline = time.time() + GPIO_ECHO_TIMEOUT_SEC
            while GPIO.input(echo) == 0:
                if time.time() > deadline:
                    return None
            pulse_start = time.time()

            # Echo dusmesini bekle
            deadline = time.time() + GPIO_ECHO_TIMEOUT_SEC
            while GPIO.input(echo) == 1:
                if time.time() > deadline:
                    return None
            pulse_end = time.time()

            # Mesafeyi hesapla: ses hizi ~343 m/s, gidis-donus bolunur 2'ye
            pulse_duration = pulse_end - pulse_start
            distance_m     = (pulse_duration * 34300.0) / 2.0 / 100.0

            return distance_m

        except ImportError:
            return None
        except Exception:
            return None

    def read_real_distance(self, sensor_name: str) -> Range | None:
        """
        Fiziksel sensor okur ve Range mesaji dondurur.

        Gercek GPIO okumasi basarisiz olursa veya henuz uygulanmamissa
        None dondurur; node kapanmaz, ilgili sensorun dongusu atlanir.

        Args:
            sensor_name: 'on_sol', 'on_sag', 'yan_sol', 'yan_sag'

        Returns:
            sensor_msgs/msg/Range veya None.
        """
        sensor = self.sensors[sensor_name]

        try:
            if not self._gpio_initialized:
                # GPIO hazir degil — yalnizca ilk seferinde uyar
                if not sensor['warned']:
                    self.get_logger().warn(
                        f'{sensor["label"]} GPIO baslatilmadigi icin '
                        'sensor okunamadi. '
                        'simulate_mode:=true parametresini kullanabilirsiniz.'
                    )
                    sensor['warned'] = True
                return None

            distance = self._read_gpio_distance(sensor_name)

            if distance is None:
                # Timeout veya okuma hatasi — sessizce atla
                return None

            return self.create_range_message(sensor_name, distance)

        except NotImplementedError as e:
            if not sensor['warned']:
                self.get_logger().warn(
                    f'{sensor["label"]} okuma henuz uygulanmadi: {e} '
                    '| Bu uyari bir kez gosterilir. '
                    '| simulate_mode:=true parametresini kullanabilirsiniz.'
                )
                sensor['warned'] = True
            return None

        except Exception as e:
            self.get_logger().error(
                f'{sensor["label"]} beklenmedik okuma hatasi: {e} '
                '| Bu dongu icin veri yayinlanmayacak.'
            )
            return None

    # ================================================================== #
    # Node kapanisi
    # ================================================================== #

    def destroy_node(self):
        """
        Node kapanirken GPIO kaynaklarini guvenli sekilde serbest birakir.
        """
        if self._gpio_initialized:
            try:
                import RPi.GPIO as GPIO
                GPIO.cleanup()
                self.get_logger().info('GPIO kaynakları temizlendi.')
            except Exception as e:
                self.get_logger().warn(f'GPIO temizleme hatasi: {e}')
        super().destroy_node()


# =============================================================================
# Giris noktasi
# =============================================================================

def main(args=None):
    """
    Node baslangic fonksiyonu.
    Cagri: ros2 run albatros_system mesafe_sensor_node
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
