#!/usr/bin/env python3
"""
imu_sensor_node.py — LODOS Albatros IMU Sensör Node'u
======================================================
Pixhawk 2.4.8 (ArduSub firmware) uçuş kontrolcüsünden MAVROS
aracılığıyla IMU verilerini alır ve /albatros/imu/data topic'ine
yayınlar.

Donanım Yapılandırması:
  - Pixhawk 2.4.8 → Raspberry Pi 5 USB3 portu (/dev/ttyACM0)
  - ArduSub frame yüklü
  - GPS modülü: GPS + I2C portlarına bağlı
  - Telemetri modülü: TELEM1 portuna bağlı
  - Güç: USB üzerinden (RPi5'ten)

Çalışma Mantığı:
  - simulate_mode=True  → Sentetik IMU verisi üretir (test amaçlı)
  - simulate_mode=False → MAVROS üzerinden Pixhawk IMU verisi alır
  - Görev modunda (AUTO/GUIDED) IMU verileri yayınlanır
  - Diğer modlarda (MANUAL, STABILIZE vb.) veri alınır ama
    yayınlanmaz (log ile bilgilendirilir)

MAVROS Topic'leri (Giriş):
  - /mavros/imu/data           → Ham IMU verisi (ivme, gyro, oryantasyon)
  - /mavros/imu/data_raw       → Kalibre edilmemiş ham IMU verisi
  - /mavros/state              → Uçuş modu ve bağlantı durumu

Albatros Topic'leri (Çıkış):
  - /albatros/imu/data         → Filtrelenmiş IMU verisi

Yazar : LODOS Takımı
Araç  : Albatros İDA
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State


# ── Topic Tanımları ──────────────────────────────────────────────────────────
IMU_TOPIC = '/albatros/imu/data'
IMU_FRAME_ID = 'imu_link'

MAVROS_IMU_TOPIC = '/mavros/imu/data'
MAVROS_IMU_RAW_TOPIC = '/mavros/imu/data_raw'
MAVROS_STATE_TOPIC = '/mavros/state'

# ── Varsayılan Parametreler ──────────────────────────────────────────────────
DEFAULT_PUBLISH_RATE = 50.0

GRAVITY = 9.81

# ── Kovaryans Sabitleri ──────────────────────────────────────────────────────
ORIENT_COV_DIAG = 1e-4
ANG_VEL_COV_DIAG = 1e-5
LIN_ACC_COV_DIAG = 1e-3

# ── Pixhawk Bağlantı Bilgileri ───────────────────────────────────────────────
# Pixhawk 2.4.8 → RPi5 USB3 bağlantısı
# MAVROS launch dosyasında fcu_url parametresi:
#   /dev/ttyACM0:921600  (veya /dev/ttyACM1)
# Bu node MAVROS'un zaten çalıştığını varsayar.

# ── Görev Modları (ArduSub) ──────────────────────────────────────────────────
# Sadece bu modlarda IMU verisi yayınlanır
ACTIVE_FLIGHT_MODES = frozenset({
    'AUTO',
    'GUIDED',
})

# Mod durumu throttle süresi (saniye)
MODE_LOG_THROTTLE_SEC = 5.0

# Bağlantı sağlığı kontrol periyodu (saniye)
CONNECTION_CHECK_TIMEOUT_SEC = 3.0


class ImuSensorNode(Node):
    """
    Pixhawk 2.4.8 (ArduSub) IMU sensör node'u.

    Simülasyon modunda sentetik veri üretir.
    Gerçek sensör modunda MAVROS üzerinden Pixhawk'tan
    IMU verilerini alır ve yalnızca görev modunda
    (AUTO/GUIDED) yayınlar.
    """

    def __init__(self):
        super().__init__('imu_sensor_node')

        # ─── Parametre Tanımları ─────────────────────────────────────────
        self.declare_parameter(
            'simulate_mode',
            True
        )

        self.declare_parameter(
            'publish_rate',
            DEFAULT_PUBLISH_RATE
        )

        self.declare_parameter(
            'use_raw_imu',
            False
        )

        self.declare_parameter(
            'mode_filter_enabled',
            True
        )

        self.declare_parameter(
            'connection_timeout_sec',
            CONNECTION_CHECK_TIMEOUT_SEC
        )

        # ─── Parametre Okuma ─────────────────────────────────────────────
        self.simulate_mode = self.get_parameter(
            'simulate_mode'
        ).value

        raw_rate = self.get_parameter(
            'publish_rate'
        ).value

        self.use_raw_imu = self.get_parameter(
            'use_raw_imu'
        ).value

        self.mode_filter_enabled = self.get_parameter(
            'mode_filter_enabled'
        ).value

        self.connection_timeout = self.get_parameter(
            'connection_timeout_sec'
        ).value

        if raw_rate <= 0.0:
            self.get_logger().warn(
                f'publish_rate={raw_rate} geçersiz. '
                f'Varsayılan {DEFAULT_PUBLISH_RATE} Hz kullanılıyor.'
            )

            self.publish_rate = DEFAULT_PUBLISH_RATE

        else:
            self.publish_rate = raw_rate

        # ─── Dahili Durum Değişkenleri ───────────────────────────────────
        self._latest_imu = None
        self._latest_imu_raw = None
        self._start_time = time.time()

        # MAVROS bağlantı durumu
        self._mavros_connected = False
        self._pixhawk_armed = False
        self._current_mode = 'UNKNOWN'
        self._last_state_time = None

        # Mod filtreleme throttle
        self._last_mode_warn_time = 0.0

        # İstatistik sayaçları
        self._total_imu_received = 0
        self._total_imu_published = 0
        self._total_imu_filtered = 0

        # Bağlantı durumu takibi
        self._connection_lost_logged = False
        self._connection_established_logged = False

        # ─── MAVROS Subscription'ları (Gerçek Sensör Modu) ───────────────
        if not self.simulate_mode:
            # Pixhawk uçuş durumu (mod, bağlantı, arm)
            self._mavros_state_sub = self.create_subscription(
                State,
                MAVROS_STATE_TOPIC,
                self._mavros_state_callback,
                10
            )

            # Kalibre edilmiş IMU verisi
            mavros_imu_source = (
                MAVROS_IMU_RAW_TOPIC
                if self.use_raw_imu
                else MAVROS_IMU_TOPIC
            )

            self._mavros_imu_sub = self.create_subscription(
                Imu,
                mavros_imu_source,
                self._mavros_imu_callback,
                qos_profile_sensor_data
            )

            self.get_logger().info(
                f'MAVROS IMU köprüsü aktif: '
                f'{mavros_imu_source} -> '
                f'{IMU_TOPIC}'
            )

            self.get_logger().info(
                'Pixhawk 2.4.8 (ArduSub) bağlantısı bekleniyor... '
                '(USB3: /dev/ttyACM0)'
            )

            # Bağlantı sağlığı kontrol timer'ı (1 Hz)
            self._health_timer = self.create_timer(
                1.0,
                self._check_connection_health
            )

        # ─── Publisher ───────────────────────────────────────────────────
        self.imu_publisher = self.create_publisher(
            Imu,
            IMU_TOPIC,
            10
        )

        # ─── Ana Yayın Timer'ı ──────────────────────────────────────────
        timer_period = 1.0 / self.publish_rate

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        # ─── Başlatma Bilgi Logları ──────────────────────────────────────
        mode_str = (
            'SİMÜLASYON (simulate_mode=True)'
            if self.simulate_mode
            else 'GERÇEK SENSÖR — Pixhawk 2.4.8 ArduSub (simulate_mode=False)'
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('IMU Sensör Node başlatıldı.')
        self.get_logger().info(f'Mod: {mode_str}')
        self.get_logger().info(f'Çıkış Topic: {IMU_TOPIC}')

        self.get_logger().info(
            f'Yayın frekansı: {self.publish_rate} Hz'
        )

        self.get_logger().info(
            f'Frame ID: {IMU_FRAME_ID}'
        )

        if not self.simulate_mode:
            imu_src = (
                MAVROS_IMU_RAW_TOPIC
                if self.use_raw_imu
                else MAVROS_IMU_TOPIC
            )

            self.get_logger().info(
                f'IMU Kaynak: {imu_src}'
            )

            self.get_logger().info(
                f'Durum Kaynağı: {MAVROS_STATE_TOPIC}'
            )

            self.get_logger().info(
                f'Mod filtresi: '
                f'{"AKTİF" if self.mode_filter_enabled else "DEVRE DIŞI"}'
                f' (AUTO, GUIDED)'
            )

            self.get_logger().info(
                'Donanım: Pixhawk 2.4.8 | ArduSub | '
                'USB3 (/dev/ttyACM0) | '
                'GPS: GPS+I2C port | '
                'Telemetri: TELEM1'
            )

        self.get_logger().info('=' * 60)

    # =====================================================================
    # MAVROS Callback'leri
    # =====================================================================

    def _mavros_state_callback(
        self,
        msg: State
    ) -> None:
        """
        /mavros/state callback'i.
        Pixhawk'ın bağlantı durumu, arm durumu ve
        mevcut uçuş modunu günceller.
        """
        prev_connected = self._mavros_connected
        prev_mode = self._current_mode

        self._mavros_connected = bool(msg.connected)
        self._pixhawk_armed = bool(msg.armed)
        self._current_mode = msg.mode
        self._last_state_time = time.monotonic()

        # Bağlantı durumu değişikliği logları
        if self._mavros_connected and not prev_connected:
            self.get_logger().info(
                '✓ Pixhawk 2.4.8 bağlantısı kuruldu! '
                f'Mod: {self._current_mode} | '
                f'Armed: {self._pixhawk_armed}'
            )
            self._connection_lost_logged = False

        elif not self._mavros_connected and prev_connected:
            self.get_logger().warn(
                '✗ Pixhawk 2.4.8 bağlantısı kesildi! '
                'USB bağlantısını kontrol edin.'
            )

        # Mod değişikliği logu
        if (
            self._current_mode != prev_mode
            and prev_mode != 'UNKNOWN'
        ):
            is_active = (
                self._current_mode in ACTIVE_FLIGHT_MODES
            )

            self.get_logger().info(
                f'Uçuş modu değişti: {prev_mode} -> '
                f'{self._current_mode} | '
                f'IMU yayını: '
                f'{"AKTİF ✓" if is_active or not self.mode_filter_enabled else "DURDURULDU ✗"}'
            )

    def _mavros_imu_callback(
        self,
        msg: Imu
    ) -> None:
        """
        MAVROS IMU verisi callback'i.
        Pixhawk 2.4.8'in dahili IMU sensöründen
        (MPU6000) gelen verileri alır.

        Pixhawk 2.4.8 IMU Sensörleri:
          - MPU6000 (Ana): 3-eksen ivmeölçer + 3-eksen jiroskop
          - L3GD20  (Yedek jiroskop, varsa)
          - LSM303D (Yedek ivmeölçer/manyetometre, varsa)
        """
        self._total_imu_received += 1

        # Frame ID'yi albatros sistemine uygun olarak güncelle
        msg.header.frame_id = IMU_FRAME_ID

        self._latest_imu = msg

    # =====================================================================
    # Ana Timer Callback
    # =====================================================================

    def timer_callback(self):
        """
        Ana yayın döngüsü.
        Simülasyon veya gerçek IMU verisini mod filtresine
        göre yayınlar.
        """
        if self.simulate_mode:
            imu_data = self.generate_simulated_imu()

        else:
            imu_data = self._get_filtered_imu()

        if imu_data is not None:
            self.imu_publisher.publish(imu_data)
            self._total_imu_published += 1

    def _get_filtered_imu(self):
        """
        Gerçek sensör modunda IMU verisini mod filtresinden
        geçirerek döndürür.

        Returns:
            Imu mesajı → Mod uygunsa ve veri varsa
            None       → Mod uygun değilse veya veri yoksa
        """
        # Bağlantı kontrolü
        if not self._mavros_connected:
            return None

        # IMU verisi henüz gelmedi
        if self._latest_imu is None:
            return None

        # Mod filtresi devre dışıysa direkt yayınla
        if not self.mode_filter_enabled:
            return self._latest_imu

        # Mod kontrolü: sadece AUTO veya GUIDED modda yayınla
        if self._current_mode in ACTIVE_FLIGHT_MODES:
            return self._latest_imu

        else:
            # Aktif görev modu dışında — veriyi yayınlama
            self._total_imu_filtered += 1

            now = time.time()

            if (
                now - self._last_mode_warn_time
                >= MODE_LOG_THROTTLE_SEC
            ):
                self.get_logger().info(
                    f'IMU verisi alınıyor ancak yayınlanmıyor. '
                    f'Mevcut mod: {self._current_mode} | '
                    f'Gerekli mod: AUTO veya GUIDED | '
                    f'Armed: {self._pixhawk_armed} | '
                    f'Toplam filtrelenen: '
                    f'{self._total_imu_filtered}'
                )

                self._last_mode_warn_time = now

            return None

    # =====================================================================
    # Bağlantı Sağlığı Kontrolü
    # =====================================================================

    def _check_connection_health(self):
        """
        Pixhawk bağlantı sağlığını periyodik olarak kontrol eder.
        MAVROS state mesajı belirli süre içinde gelmezse
        bağlantı kopmuş kabul edilir.
        """
        if self._last_state_time is None:
            if not self._connection_lost_logged:
                self.get_logger().warn(
                    'Pixhawk 2.4.8 henüz bağlanmadı. '
                    'MAVROS ve USB bağlantısını kontrol edin. '
                    '(Beklenen: /dev/ttyACM0:921600)'
                )
                self._connection_lost_logged = True

            return

        elapsed = time.monotonic() - self._last_state_time

        if elapsed > self.connection_timeout:
            self._mavros_connected = False

            if not self._connection_lost_logged:
                self.get_logger().warn(
                    f'Pixhawk bağlantısı zaman aşımına uğradı! '
                    f'Son state mesajı: {elapsed:.1f}s önce | '
                    f'Timeout: {self.connection_timeout:.1f}s | '
                    f'Olası sebepler: '
                    f'USB kablosu çıkmış, MAVROS çökmüş veya '
                    f'Pixhawk yanıt vermiyor.'
                )
                self._connection_lost_logged = True

        elif self._connection_lost_logged and self._mavros_connected:
            self.get_logger().info(
                '✓ Pixhawk bağlantısı yeniden sağlandı!'
            )
            self._connection_lost_logged = False

    # =====================================================================
    # Simülasyon IMU Verisi Üretimi
    # =====================================================================

    def generate_simulated_imu(self) -> Imu:
        """
        Test amaçlı sentetik IMU verisi üretir.
        Hafif salınım ve gürültü ekleyerek gerçekçi bir
        simülasyon sağlar.
        """
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = IMU_FRAME_ID

        elapsed = time.time() - self._start_time

        roll = 0.01 * math.sin(
            0.5 * elapsed
        )

        pitch = 0.005 * math.cos(
            0.3 * elapsed
        )

        yaw = 0.0

        qx, qy, qz, qw = self._euler_to_quaternion(
            roll,
            pitch,
            yaw
        )

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance = [
            ORIENT_COV_DIAG,
            0.0,
            0.0,
            0.0,
            ORIENT_COV_DIAG,
            0.0,
            0.0,
            0.0,
            ORIENT_COV_DIAG
        ]

        msg.angular_velocity.x = random.gauss(
            0.0,
            0.001
        )

        msg.angular_velocity.y = random.gauss(
            0.0,
            0.001
        )

        msg.angular_velocity.z = random.gauss(
            0.0,
            0.001
        )

        msg.angular_velocity_covariance = [
            ANG_VEL_COV_DIAG,
            0.0,
            0.0,
            0.0,
            ANG_VEL_COV_DIAG,
            0.0,
            0.0,
            0.0,
            ANG_VEL_COV_DIAG
        ]

        msg.linear_acceleration.x = random.gauss(
            0.0,
            0.01
        )

        msg.linear_acceleration.y = random.gauss(
            0.0,
            0.01
        )

        msg.linear_acceleration.z = (
            GRAVITY
            + random.gauss(0.0, 0.05)
        )

        msg.linear_acceleration_covariance = [
            LIN_ACC_COV_DIAG,
            0.0,
            0.0,
            0.0,
            LIN_ACC_COV_DIAG,
            0.0,
            0.0,
            0.0,
            LIN_ACC_COV_DIAG
        ]

        return msg

    # =====================================================================
    # Yardımcı Fonksiyonlar
    # =====================================================================

    @staticmethod
    def _euler_to_quaternion(
        roll: float,
        pitch: float,
        yaw: float
    ):
        """
        Euler açılarından (roll, pitch, yaw) quaternion
        (qx, qy, qz, qw) hesaplar.
        """
        cr = math.cos(
            roll * 0.5
        )

        sr = math.sin(
            roll * 0.5
        )

        cp = math.cos(
            pitch * 0.5
        )

        sp = math.sin(
            pitch * 0.5
        )

        cy = math.cos(
            yaw * 0.5
        )

        sy = math.sin(
            yaw * 0.5
        )

        qw = (
            cr * cp * cy
            + sr * sp * sy
        )

        qx = (
            sr * cp * cy
            - cr * sp * sy
        )

        qy = (
            cr * sp * cy
            + sr * cp * sy
        )

        qz = (
            cr * cp * sy
            - sr * sp * cy
        )

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)

    node = ImuSensorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'IMU Sensör Node durduruldu. '
            f'Toplam alınan: {node._total_imu_received} | '
            f'Yayınlanan: {node._total_imu_published} | '
            f'Filtrelenen: {node._total_imu_filtered}'
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
