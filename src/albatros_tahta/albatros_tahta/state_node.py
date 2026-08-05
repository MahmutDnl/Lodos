#!/usr/bin/env python3
"""
state_node.py — LODOS Albatros Birleşik Araç Durumu Node'u
===========================================================
Bu node dört kaynaktan veri toplayarak /albatros/state topic'ine
VehicleState mesajı yayınlar:

  - /albatros/mission/status  → Görev durumu bilgisi (MissionStatus)
  - /albatros/mission/target  → Anlık hedef bilgisi (MissionTarget)
  - /albatros/control/status  → Sistem sağlık bilgisi (std_msgs/String JSON)
  - /albatros/imu/data        → Yönelim bilgisi (sensor_msgs/Imu)

Bu node:
  ✓ Veri toplar, hesaplar ve state yayınlar.
  ✗ Motor komutu üretmez.
  ✗ MAVROS'a komut göndermez.
  ✗ Görev geçişi yapmaz.

Yazar : LODOS Takımı
Araç  : Albatros İDA
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu

from albatros_interfaces.msg import MissionStatus, MissionTarget, VehicleState


class StateNode(Node):
    """
    LODOS Albatros birleşik araç durumu node'u.

    Görev, hedef, IMU ve kontrol kaynaklarından gelen verileri birleştirerek
    karar node'un kullanabileceği tek bir VehicleState mesajı üretir.
    """

    def __init__(self):
        super().__init__('state_node')

        # ─── Parametreler ─────────────────────────────────────────────────────
        self.declare_parameter('publish_rate',        10.0)
        self.declare_parameter('mission_timeout_sec',  2.0)
        self.declare_parameter('target_timeout_sec',   2.0)
        self.declare_parameter('imu_timeout_sec',      2.0)
        self.declare_parameter('control_timeout_sec',  2.0)

        self._publish_rate      = self.get_parameter('publish_rate').value
        self._mission_timeout   = self.get_parameter('mission_timeout_sec').value
        self._target_timeout    = self.get_parameter('target_timeout_sec').value
        self._imu_timeout       = self.get_parameter('imu_timeout_sec').value
        self._control_timeout   = self.get_parameter('control_timeout_sec').value

        # ─── Dahili durum: Görev (Mission) ────────────────────────────────────
        self._mission_active       = False
        self._mission_completed    = False
        self._mission_error        = False
        self._mission_state        = 'IDLE'
        self._mission_error_code   = ''
        self._current_parkur       = 0
        self._current_waypoint_seq = 0

        # ─── Dahili durum: Hedef (Target) ─────────────────────────────────────
        self._target_valid         = False
        self._target_reached       = False
        self._target_latitude      = 0.0
        self._target_longitude     = 0.0
        self._distance_to_target_m = 0.0
        self._target_bearing_deg   = 0.0

        # ─── Dahili durum: IMU / Heading ──────────────────────────────────────
        self._imu_valid          = False
        self._current_yaw_deg    = 0.0
        self._heading_error_deg  = 0.0
        self._turn_direction     = 'UNKNOWN'

        # ─── Dahili durum: Sistem Sağlık ─────────────────────────────────────
        self._mavros_connected = False
        self._armed            = False
        self._mode             = 'UNKNOWN'
        self._emergency_stop   = False
        self._gps_ok           = False
        self._imu_ok           = False
        self._control_allowed  = False

        # ─── Timestamp takibi (timeout kontrolü) ─────────────────────────────
        self._last_mission_time = None
        self._last_target_time  = None
        self._last_imu_time     = None
        self._last_control_time = None

        # ─── Throttled uyarı için son yazma zamanları (sn) ───────────────────
        self._last_warn_mission  = 0.0
        self._last_warn_target   = 0.0
        self._last_warn_imu      = 0.0
        self._last_warn_control  = 0.0
        self._WARN_THROTTLE_SEC  = 3.0

        # ─── QoS profilleri ──────────────────────────────────────────────────
        default_qos = QoSProfile(depth=10)

        # ─── Subscriber'lar ───────────────────────────────────────────────────
        self._sub_mission_status = self.create_subscription(
            MissionStatus,
            '/albatros/mission/status',
            self._cb_mission_status,
            default_qos
        )

        self._sub_mission_target = self.create_subscription(
            MissionTarget,
            '/albatros/mission/target',
            self._cb_mission_target,
            default_qos
        )

        self._sub_control_status = self.create_subscription(
            String,
            '/albatros/control/status',
            self._cb_control_status,
            default_qos
        )

        # IMU için sensör verileri QoS profili kullanılır (Best Effort)
        self._sub_imu = self.create_subscription(
            Imu,
            '/albatros/imu/data',
            self._cb_imu,
            qos_profile_sensor_data
        )

        # ─── Publisher ────────────────────────────────────────────────────────
        self._pub_state = self.create_publisher(
            VehicleState,
            '/albatros/state',
            default_qos
        )

        # ─── Publish timer ────────────────────────────────────────────────────
        period_sec = 1.0 / max(self._publish_rate, 0.1)
        self._timer = self.create_timer(period_sec, self.publish_state)

        self.get_logger().info(
            f'StateNode başlatıldı. '
            f'Publish rate: {self._publish_rate:.1f} Hz | '
            f'Timeout — mission: {self._mission_timeout}s, '
            f'target: {self._target_timeout}s, '
            f'imu: {self._imu_timeout}s, '
            f'control: {self._control_timeout}s'
        )

    # =========================================================================
    # Callback'ler
    # =========================================================================

    def _cb_mission_status(self, msg: MissionStatus):
        """
        /albatros/mission/status callback'i.
        Görev durumu bilgisini günceller.
        """
        self._last_mission_time = self.get_clock().now()

        self._mission_active       = msg.mission_active
        self._mission_completed    = msg.mission_completed
        self._mission_error        = msg.mission_error
        self._mission_state        = msg.mission_state
        self._mission_error_code   = msg.error_code
        self._current_parkur       = int(msg.current_parkur)
        self._current_waypoint_seq = msg.current_waypoint_seq
        self._target_valid         = msg.target_valid
        self._target_reached       = msg.target_reached

    def _cb_mission_target(self, msg: MissionTarget):
        """
        /albatros/mission/target callback'i.
        Anlık hedef koordinatları, bearing ve görev bağlamı bilgisini günceller.
        Yeni bearing geldiğinde heading error yeniden hesaplanır.
        """
        self._last_target_time = self.get_clock().now()

        # Hedef geçerliliği ve görev bağlamı
        self._target_valid         = msg.target_valid
        self._mission_active       = msg.mission_active
        self._current_parkur       = int(msg.current_parkur)
        self._current_waypoint_seq = msg.waypoint_seq

        # Hedef konum ve bearing bilgisi
        self._target_latitude      = msg.target_latitude
        self._target_longitude     = msg.target_longitude
        self._distance_to_target_m = msg.distance_to_target_m
        self._target_bearing_deg   = msg.target_bearing_deg

        # Yeni hedef geldiğinde heading error'u anında güncelle
        self.update_heading_error()

    def _cb_imu(self, msg: Imu):
        """
        /albatros/imu/data callback'i.
        Quaternion'dan yaw açısını hesaplar.
        Yeni IMU verisiyle heading error yeniden hesaplanır.
        """
        self._last_imu_time = self.get_clock().now()

        yaw_deg = self.quaternion_to_yaw_deg(msg)

        if yaw_deg is None:
            # Geçersiz quaternion — güvenli değerlere dön
            self._imu_valid        = False
            self._current_yaw_deg  = 0.0
        else:
            self._imu_valid       = True
            self._current_yaw_deg = self.normalize_angle_360(yaw_deg)

        # Yeni yaw ile heading error'u anında güncelle
        self.update_heading_error()

    def _cb_control_status(self, msg: String):
        """
        /albatros/control/status callback'i.
        JSON string parse ederek sistem sağlık alanlarını günceller.
        """
        self._last_control_time = self.get_clock().now()
        self.parse_control_status(msg)

    # =========================================================================
    # Yardımcı Fonksiyonlar
    # =========================================================================

    def quaternion_to_yaw_deg(self, msg: Imu):
        """
        IMU mesajındaki quaternion (x, y, z, w) değerlerinden
        yaw açısını derece cinsinden hesaplayarak döndürür.

        Quaternion normu 1e-6'dan küçükse geçersiz kabul edilir
        ve None döner.

        Formül:
            siny_cosp = 2 * (w*z + x*y)
            cosy_cosp = 1 - 2 * (y*y + z*z)
            yaw_rad   = atan2(siny_cosp, cosy_cosp)
        """
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Norm kontrolü — sıfır veya çok küçük quaternion geçersizdir
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-6:
            self.get_logger().warn(
                f'IMU quaternion normu çok küçük ({norm:.2e}); '
                'yaw hesaplanamıyor. imu_valid = False.'
            )
            return None

        # Yaw hesabı (euler ZYX konvansiyonu)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad   = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg   = math.degrees(yaw_rad)

        return yaw_deg

    def normalize_angle_360(self, angle: float) -> float:
        """Açıyı [0, 360) aralığına normalize eder."""
        return (angle + 360.0) % 360.0

    def normalize_angle_180(self, angle: float) -> float:
        """Açıyı (-180, +180] aralığına normalize eder."""
        return (angle + 180.0) % 360.0 - 180.0

    def update_heading_error(self):
        """
        target_bearing_deg ile current_yaw_deg arasındaki açısal farkı hesaplar
        ve turn_direction alanını günceller.

        Kurallar:
            heading_error_deg > 5.0   → 'RIGHT'
            heading_error_deg < -5.0  → 'LEFT'
            -5.0 ≤ error ≤ 5.0        → 'ALIGNED'
            target_valid = False      → 'UNKNOWN'
            imu_valid = False         → 'UNKNOWN'
        """
        # Hedef yoksa ya da IMU geçersizse hesap yapma
        if not self._target_valid or not self._imu_valid:
            self._heading_error_deg = 0.0
            self._turn_direction    = 'UNKNOWN'
            return

        raw_error               = self._target_bearing_deg - self._current_yaw_deg
        self._heading_error_deg = self.normalize_angle_180(raw_error)

        if self._heading_error_deg > 5.0:
            self._turn_direction = 'RIGHT'
        elif self._heading_error_deg < -5.0:
            self._turn_direction = 'LEFT'
        else:
            self._turn_direction = 'ALIGNED'

    def parse_control_status(self, msg: String):
        """
        Control node'un /albatros/control/status topic'ine yayınladığı
        JSON string'ini parse eder ve sistem sağlık alanlarını günceller.

        JSON parse hatası durumunda node çökmez; güvenli varsayılan
        değerler (bağlantısız, emergency_stop=True) atanır.
        """
        try:
            data = json.loads(msg.data)

            self._mavros_connected = bool(data.get('connected',        False))
            self._armed            = bool(data.get('armed',            False))
            self._mode             = str(data.get('mode',          'UNKNOWN'))
            self._emergency_stop   = bool(data.get('emergency_stop',   True))
            self._gps_ok           = bool(data.get('gps_ok',           False))
            self._imu_ok           = bool(data.get('imu_ok',           False))
            self._control_allowed  = bool(data.get('control_allowed',  False))

        except (json.JSONDecodeError, TypeError, ValueError) as e:
            self.get_logger().warn(
                f'control_status JSON parse hatası: {e} | '
                'Güvenli değerler uygulanıyor.'
            )
            # Güvenli varsayılan değerler
            self._mavros_connected = False
            self._armed            = False
            self._mode             = 'UNKNOWN'
            self._emergency_stop   = True
            self._gps_ok           = False
            self._imu_ok           = False
            self._control_allowed  = False

    def is_fresh(self, last_time, timeout: float) -> bool:
        """
        Verilen last_time timestamp'inin timeout süresi içinde olup olmadığını kontrol eder.

        Args:
            last_time: rclpy.clock.Time nesnesi veya None.
            timeout:   Saniye cinsinden maksimum geçerlilik süresi.

        Returns:
            True  → Veri yeterince taze.
            False → Veri hiç gelmedi veya timeout doldu.
        """
        if last_time is None:
            return False
        elapsed_sec = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return elapsed_sec < timeout

    def _check_timeouts(self):
        """
        Tüm kaynak verilerinin tazeliğini kontrol eder.
        Timeout olan kaynaklar için 3 saniyede bir uyarı verir (throttle).
        IMU timeout'u durumunda imu_valid ve ilgili alanlar sıfırlanır.
        """
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # Mission status kontrolü
        if not self.is_fresh(self._last_mission_time, self._mission_timeout):
            if now_sec - self._last_warn_mission >= self._WARN_THROTTLE_SEC:
                self.get_logger().warn(
                    f'[TIMEOUT] /albatros/mission/status — '
                    f'{self._mission_timeout:.1f}s içinde veri alınamadı. '
                    'Güvenli değerlerle devam ediliyor.'
                )
                self._last_warn_mission = now_sec

        # Target kontrolü — timeout olursa hedef geçersiz sayılır
        if not self.is_fresh(self._last_target_time, self._target_timeout):
            if now_sec - self._last_warn_target >= self._WARN_THROTTLE_SEC:
                self.get_logger().warn(
                    f'[TIMEOUT] /albatros/mission/target — '
                    f'{self._target_timeout:.1f}s içinde veri alınamadı. '
                    'target_valid = False olarak işaretlendi.'
                )
                self._last_warn_target = now_sec

            # Hedef verisi bayatlamışsa hedef bilgilerini sıfırla
            self._target_valid       = False
            self._heading_error_deg  = 0.0
            self._turn_direction     = 'UNKNOWN'

        # IMU kontrolü — timeout olursa yaw geçersiz sayılır
        if not self.is_fresh(self._last_imu_time, self._imu_timeout):
            if now_sec - self._last_warn_imu >= self._WARN_THROTTLE_SEC:
                self.get_logger().warn(
                    f'[TIMEOUT] /albatros/imu/data — '
                    f'{self._imu_timeout:.1f}s içinde veri alınamadı. '
                    'imu_valid = False olarak işaretlendi.'
                )
                self._last_warn_imu = now_sec

            # IMU verisi bayatlamışsa yönelim bilgilerini sıfırla
            self._imu_valid         = False
            self._current_yaw_deg   = 0.0
            self._heading_error_deg = 0.0
            self._turn_direction    = 'UNKNOWN'

        # Control status kontrolü
        if not self.is_fresh(self._last_control_time, self._control_timeout):
            if now_sec - self._last_warn_control >= self._WARN_THROTTLE_SEC:
                self.get_logger().warn(
                    f'[TIMEOUT] /albatros/control/status — '
                    f'{self._control_timeout:.1f}s içinde veri alınamadı. '
                    'Güvenli değerlerle devam ediliyor.'
                )
                self._last_warn_control = now_sec

    def publish_state(self):
        """
        Timer callback'i: Tüm kaynaklardan gelen verileri birleştirerek
        VehicleState mesajı oluşturur ve /albatros/state topic'ine yayınlar.

        Timeout kontrolü bu fonksiyonda gerçekleştirilir.
        """
        # Timeout kontrolü — stale veri varsa uyar ve IMU'yu sıfırla
        self._check_timeouts()

        msg = VehicleState()

        # ── Header ──────────────────────────────────────────────────
        msg.header           = Header()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'base_link'

        # ── Görev Durumu ─────────────────────────────────────────────
        msg.mission_active       = self._mission_active
        msg.mission_completed    = self._mission_completed
        msg.mission_error        = self._mission_error
        msg.mission_state        = self._mission_state
        msg.error_code           = self._mission_error_code
        msg.current_parkur       = self._current_parkur
        msg.current_waypoint_seq = self._current_waypoint_seq

        # ── Hedef Bilgisi ────────────────────────────────────────────
        msg.target_valid          = self._target_valid
        msg.target_reached        = self._target_reached
        msg.target_latitude       = self._target_latitude
        msg.target_longitude      = self._target_longitude
        msg.distance_to_target_m  = self._distance_to_target_m
        msg.target_bearing_deg    = self._target_bearing_deg

        # ── IMU / Heading Bilgisi ────────────────────────────────────
        msg.imu_valid         = self._imu_valid
        msg.current_yaw_deg   = self._current_yaw_deg
        msg.heading_error_deg = self._heading_error_deg
        msg.turn_direction    = self._turn_direction

        # ── Sistem Sağlık Bilgisi ────────────────────────────────────
        msg.mavros_connected = self._mavros_connected
        msg.armed            = self._armed
        msg.mode             = self._mode
        msg.emergency_stop   = self._emergency_stop
        msg.gps_ok           = self._gps_ok
        msg.imu_ok           = self._imu_ok
        msg.control_allowed  = self._control_allowed

        self._pub_state.publish(msg)


# =============================================================================
# Entry Point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('StateNode klavye ile durduruldu (SIGINT).')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
