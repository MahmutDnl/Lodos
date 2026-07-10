#!/usr/bin/env python3
"""
mavros_node.py — LODOS Albatros MAVROS Durum İzleme Node'u
============================================================

Görevi:
    MAVROS bağlantı durumunu, topic akışını ve servis erişilebilirliğini
    izler; sonuçları /albatros/mavros/status topic'inde JSON formatında
    yayınlar.

ÖNEMLİ:
    Bu node MAVROS'u yeniden yazmaz.
    Bu node Pixhawk'a hareket komutu GÖNDERMEZ.
    Hareket komutları komut_node.py üzerinden gönderilir.

Abone olunan MAVROS topic'leri:
    /mavros/state              -> mavros_msgs/msg/State
    /mavros/global_position/global -> sensor_msgs/msg/NavSatFix
    /mavros/imu/data           -> sensor_msgs/msg/Imu
    /mavros/battery            -> sensor_msgs/msg/BatteryState

Status JSON alanları:
    connected, armed, mode,
    state_ok, gps_ok, imu_ok, battery_ok,
    arming_service_available, set_mode_service_available,
    last_state_age_sec, last_gps_age_sec, last_imu_age_sec, last_battery_age_sec

Servis client'ları (otomatik çağırılmaz, sadece hazır tutulur):
    /mavros/cmd/arming         -> mavros_msgs/srv/CommandBool
    /mavros/set_mode           -> mavros_msgs/srv/SetMode

Yayınlanan topic:
    /albatros/mavros/status    -> std_msgs/msg/String (JSON)

Parametreler:
    connection_timeout_sec  (float, default: 2.0)  — topic "ok" eşiği
    status_publish_period_sec (float, default: 1.0) — yayın periyodu
"""

import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class MavrosNode(Node):
    """MAVROS bağlantı ve sağlık durumu izleme node'u."""

    def __init__(self):
        super().__init__('mavros_node')

        # ─── Parametreler ────────────────────────────────────────────────────
        self.declare_parameter('connection_timeout_sec', 2.0)
        self.declare_parameter('status_publish_period_sec', 1.0)

        self._timeout_sec = (
            self.get_parameter('connection_timeout_sec').value
        )
        self._publish_period = (
            self.get_parameter('status_publish_period_sec').value
        )

        # ─── İç durum değişkenleri ───────────────────────────────────────────
        self._connected = False   # /mavros/state bağlantı alanı
        self._armed = False       # Pixhawk arm durumu
        self._mode = 'UNKNOWN'    # Pixhawk uçuş modu

        self._last_state_time: float | None = None    # Son State mesajı zamanı
        self._last_gps_time: float | None = None      # Son NavSatFix zamanı
        self._last_imu_time: float | None = None      # Son IMU zamanı
        self._last_battery_time: float | None = None  # Son BatteryState zamanı

        # ─── Warning throttle takibi ─────────────────────────────────────────
        # Her uyarı anahtarı için son basım zamanını tutar.
        # Aynı uyarı _WARN_INTERVAL_SEC saniyede en fazla bir kez basılır.
        self._WARN_INTERVAL_SEC: float = 5.0
        self._last_warn_time: dict[str, float] = {}

        # ─── Subscriber'lar ──────────────────────────────────────────────────
        self._state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_callback,
            qos_profile=10,
        )

        self._gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._gps_callback,
            qos_profile=10,
        )

        self._imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self._imu_callback,
            qos_profile=10,
        )

        self._battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._battery_callback,
            qos_profile=10,
        )

        # ─── Publisher ───────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String,
            '/albatros/mavros/status',
            qos_profile=10,
        )

        # ─── Servis client'ları (otomatik çağırılmaz) ────────────────────────
        # Arming ve set_mode servisleri yalnızca komut_node.py tarafından
        # kullanılmalıdır. Burada yalnızca erişilebilirlik kontrolü yapılır.
        self._arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming',
        )
        self._set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode',
        )

        # ─── Periyodik yayın timer'ı ─────────────────────────────────────────
        self._status_timer = self.create_timer(
            self._publish_period,
            self._publish_status,
        )

        self.get_logger().info(
            f'mavros_node başlatıldı. '
            f'timeout={self._timeout_sec}s, '
            f'yayın_periyodu={self._publish_period}s'
        )

    # ─── Callback'ler ────────────────────────────────────────────────────────

    def _state_callback(self, msg: State) -> None:
        """
        /mavros/state topic'inden gelen mesajı işler.
        Bağlantı, arm ve mod bilgilerini günceller.
        Ayrıca son mesaj zamanını kaydeder (state_ok hesabı için).
        """
        self._connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode
        self._last_state_time = time.monotonic()  # state_ok takibi

    def _gps_callback(self, msg: NavSatFix) -> None:
        """Son GPS mesajının zamanını kaydeder."""
        self._last_gps_time = time.monotonic()

    def _imu_callback(self, msg: Imu) -> None:
        """Son IMU mesajının zamanını kaydeder."""
        self._last_imu_time = time.monotonic()

    def _battery_callback(self, msg: BatteryState) -> None:
        """Son batarya mesajının zamanını kaydeder."""
        self._last_battery_time = time.monotonic()

    # ─── Yardımcı metotlar ───────────────────────────────────────────────────

    def _age_sec(self, last_time: float | None) -> float | None:
        """
        Son alınan mesajın yaşını saniye cinsinden döndürür.
        Hiç mesaj gelmemişse None döndürür.
        """
        if last_time is None:
            return None
        return round(time.monotonic() - last_time, 3)

    def _is_fresh(self, last_time: float | None) -> bool:
        """
        Son mesaj timeout süresi içinde geldiyse True döndürür.
        Hiç mesaj gelmemişse False döndürür.
        """
        age = self._age_sec(last_time)
        if age is None:
            return False
        return age <= self._timeout_sec

    def _warn_throttle(self, key: str, message: str) -> None:
        """
        Aynı uyarıyı _WARN_INTERVAL_SEC saniyede en fazla bir kez basar.

        Args:
            key:     Uyarıyı tanımlayan benzersiz anahtar (örn. 'gps_warn').
            message: Basılacak uyarı metni.
        """
        now = time.monotonic()
        last = self._last_warn_time.get(key, 0.0)
        if now - last >= self._WARN_INTERVAL_SEC:
            self.get_logger().warn(message)
            self._last_warn_time[key] = now

    # ─── Durum yayını ────────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        """
        Her _publish_period saniyede bir /albatros/mavros/status topic'ine
        JSON formatında durum mesajı yayınlar.
        Servis ve topic erişilebilirliğini de kontrol eder.
        Uyarı logları throttle ile en fazla _WARN_INTERVAL_SEC'te bir basılır.
        """
        state_ok = self._is_fresh(self._last_state_time)
        gps_ok = self._is_fresh(self._last_gps_time)
        imu_ok = self._is_fresh(self._last_imu_time)
        battery_ok = self._is_fresh(self._last_battery_time)

        arming_available = self._arming_client.service_is_ready()
        set_mode_available = self._set_mode_client.service_is_ready()

        state_age = self._age_sec(self._last_state_time)
        gps_age = self._age_sec(self._last_gps_time)
        imu_age = self._age_sec(self._last_imu_time)
        battery_age = self._age_sec(self._last_battery_time)

        status = {
            'connected': self._connected,
            'armed': self._armed,
            'mode': self._mode,
            'state_ok': state_ok,
            'gps_ok': gps_ok,
            'imu_ok': imu_ok,
            'battery_ok': battery_ok,
            'arming_service_available': arming_available,
            'set_mode_service_available': set_mode_available,
            'last_state_age_sec': state_age,
            'last_gps_age_sec': gps_age,
            'last_imu_age_sec': imu_age,
            'last_battery_age_sec': battery_age,
        }

        # ─── Throttle'lı uyarı logları ───────────────────────────────────────
        # Aynı uyarı _WARN_INTERVAL_SEC (5s) içinde yalnızca bir kez basılır.
        if not state_ok:
            self._warn_throttle(
                'state_warn',
                f'/mavros/state gelmiyor veya eski '
                f'(son: {state_age}s, eşik: {self._timeout_sec}s). '
                'MAVROS çalışıyor mu?'
            )
        elif not self._connected:
            # State geliyor ama connected=False — bağlantı kopuk
            self._warn_throttle(
                'connected_warn',
                'MAVROS bağlantısı yok: /mavros/state connected=False.'
            )
        if not gps_ok:
            self._warn_throttle(
                'gps_warn',
                f'GPS verisi gelmiyor veya eski '
                f'(son: {gps_age}s, eşik: {self._timeout_sec}s).'
            )
        if not imu_ok:
            self._warn_throttle(
                'imu_warn',
                f'IMU verisi gelmiyor veya eski '
                f'(son: {imu_age}s, eşik: {self._timeout_sec}s).'
            )
        if not battery_ok:
            self._warn_throttle(
                'battery_warn',
                f'Batarya verisi gelmiyor veya eski '
                f'(son: {battery_age}s, eşik: {self._timeout_sec}s).'
            )

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(msg)


def main(args=None):
    """Node giriş noktası."""
    rclpy.init(args=args)
    node = MavrosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('mavros_node kapatılıyor (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
