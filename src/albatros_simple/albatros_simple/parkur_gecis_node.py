#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros — Parkur Geçiş Düğümü (parkur_gecis_node)
=============================================================================
ROS2 Jazzy / Ubuntu 24.04

Paket: albatros_simple
ROS2 Düğüm Adı: parkur_gecis_node

Görev Akışı:
  1. Araç AUTO modda 5 navigasyon waypoint'ini Pixhawk görevi olarak tamamlar.
  2. MAVROS WaypointList'teki tüm command == 16 (NAV_WAYPOINT) elemanlarını sayarak
     gerçek 5. NAV_WAYPOINT'i doğru tespit eder (ilk elemanı körleme atlamaz).
  3. 5. waypoint MAVROS `reached` mesajı + WP5 GPS mesafe doğrulamasını BİRLİKTE (AND) kullanır.
  4. 5. waypoint'e ulaşıldığında MAVROS /mavros/set_mode servisi ile GUIDED moda geçiş ister.
  5. Geçiş başarısız olursa 1 saniye arayla en fazla 3 kez tekrar dener (mode_change_timeout_sec ile takip edilir).
  6. Servis cevabı gelse bile modun gerçekten GUIDED olduğunu /mavros/state üzerinden
     doğruladıktan sonra /albatros/parkur3_active = True yayınlar.
=============================================================================
"""

import math
import time
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

try:
    from mavros_msgs.msg import State, WaypointList, WaypointReached
    from mavros_msgs.srv import SetMode
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False
    State = None
    WaypointList = None
    WaypointReached = None
    SetMode = None


class ParkurGecisNode(Node):
    """
    MAVROS üzerinden ilk 5 waypoint takibini yürüten ve 5. waypoint'te
    AUTO -> GUIDED geçişini 3 deneme hakkı ve /mavros/state doğrulaması ile yapıp
    /albatros/parkur3_active sinyalini yayınlayan düğüm.
    """

    TARGET_WAYPOINTS = [
        (40.7236638, 29.8249893),  # Waypoint 1
        (40.7237441, 29.8248786),  # Waypoint 2
        (40.7238497, 29.8249284),  # Waypoint 3
        (40.7240384, 29.8251016),  # Waypoint 4
        (40.7235886, 29.8254987),  # Waypoint 5 (Hedef Geçiş Noktası)
    ]

    def __init__(self):
        super().__init__('parkur_gecis_node')

        # ─── ROS Parametreleri ───────────────────────────────────────────────
        self.declare_parameter('target_wp_index', 5)  # 5. Navigasyon waypoint'i
        self.declare_parameter('distance_threshold_m', 3.5)
        self.declare_parameter('mode_change_timeout_sec', 5.0)
        self.declare_parameter('max_mode_change_attempts', 3)
        self.declare_parameter('retry_interval_sec', 1.0)

        self.target_wp_index = int(self.get_parameter('target_wp_index').value)
        self.distance_threshold_m = float(self.get_parameter('distance_threshold_m').value)
        self.mode_change_timeout_sec = float(self.get_parameter('mode_change_timeout_sec').value)
        self.max_mode_change_attempts = int(self.get_parameter('max_mode_change_attempts').value)
        self.retry_interval_sec = float(self.get_parameter('retry_interval_sec').value)

        # ─── Durum Değişkenleri ──────────────────────────────────────────────
        self.current_mode = "UNKNOWN"
        self.mavros_connected = False
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.gps_valid = False

        self.nav_waypoints: List[Dict] = []
        self.target_wp_seq: Optional[int] = None
        self.reached_wp_seqs: set = set()

        # Mod Değişimi & Doğrulama Yönetimi
        self.mode_change_requested = False
        self.attempt_count = 0
        self.last_attempt_time = 0.0
        self.request_start_time = 0.0
        self.parkur3_active = False
        self.error_logged = False

        # ─── Publisher (Latched / Transient Local QoS) ────────────────────────
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.parkur3_active_pub = self.create_publisher(
            Bool,
            '/albatros/parkur3_active',
            qos_latched
        )

        # ─── MAVROS Abonelikleri ─────────────────────────────────────────────
        if MAVROS_AVAILABLE:
            self.create_subscription(
                State,
                '/mavros/state',
                self.mavros_state_callback,
                10
            )

            self.create_subscription(
                WaypointList,
                '/mavros/mission/waypoints',
                self.waypoint_list_callback,
                10
            )

            self.create_subscription(
                WaypointReached,
                '/mavros/mission/reached',
                self.waypoint_reached_callback,
                10
            )

            self.create_subscription(
                NavSatFix,
                '/mavros/global_position/global',
                self.gps_callback,
                10
            )

            self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        else:
            self.get_logger().error("mavros_msgs paketi bulunamadı! MAVROS iletişimi devre dışı.")

        # ─── Kontrol Döngüsü Timer (2 Hz) ───────────────────────────────────
        self.timer = self.create_timer(0.5, self.control_loop)

        self.get_logger().info("Parkur Geçiş Düğümü (parkur_gecis_node) Başlatıldı.")
        self.get_logger().info(f"Hedef: {self.target_wp_index}. NAV_WAYPOINT ({self.TARGET_WAYPOINTS[-1]})")

    def mavros_state_callback(self, msg: State):
        """
        MAVROS durumunu günceller.
        Modun gerçekten GUIDED olduğunu buradan doğrular.
        """
        self.mavros_connected = msg.connected
        self.current_mode = msg.mode

        # Eğer mod değişimi talep edilmişse ve mod /mavros/state üzerinde gerçekten GUIDED olduysa
        if self.mode_change_requested and not self.parkur3_active and self.current_mode.upper() == "GUIDED":
            self.parkur3_active = True
            self.get_logger().info("==================================================================")
            self.get_logger().info(f"[ParkurGecisNode] /mavros/state üzerinden modun '{self.current_mode}' olduğu DOĞRULANDI!")
            self.get_logger().info("[ParkurGecisNode] /albatros/parkur3_active = True yayınlandı.")
            self.get_logger().info("==================================================================")

    def gps_callback(self, msg: NavSatFix):
        """MAVROS GPS konumunu günceller."""
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            self.gps_valid = True

    def waypoint_list_callback(self, msg: WaypointList):
        """
        MAVROS WaypointList mesajını işler.
        İlk elemanı otomatik HOME sayıp körleme atlamaz;
        tüm command == 16 (NAV_WAYPOINT) olan elemanları sayarak gerçek 5. NAV_WAYPOINT'i bulur.
        """
        new_nav_wps = []
        for i, wp in enumerate(msg.waypoints):
            # Command 16 = NAV_WAYPOINT (ArduPilot / PX4)
            if wp.command == 16:
                new_nav_wps.append({
                    'nav_index': len(new_nav_wps) + 1,
                    'seq': i,
                    'lat': wp.x_lat,
                    'lon': wp.y_long
                })

        self.nav_waypoints = new_nav_wps

        # Gerçek 5. Navigasyon waypoint'inin MAVROS seq numarasını belirle
        if len(self.nav_waypoints) >= self.target_wp_index:
            target_wp_info = self.nav_waypoints[self.target_wp_index - 1]
            self.target_wp_seq = target_wp_info['seq']
            self.get_logger().info(
                f"[ParkurGecisNode] Gerçek {self.target_wp_index}. NAV_WAYPOINT tespit edildi -> "
                f"MAVROS Seq: {self.target_wp_seq} ({target_wp_info['lat']:.7f}, {target_wp_info['lon']:.7f})",
                throttle_duration_sec=10.0
            )

    def waypoint_reached_callback(self, msg: WaypointReached):
        """MAVROS WaypointReached mesajı alındığında çalışır."""
        seq = int(msg.wp_seq)
        self.reached_wp_seqs.add(seq)
        self.get_logger().info(f"[ParkurGecisNode] Waypoint Reached bildirimi: MAVROS Seq {seq}")

    def control_loop(self):
        """
        Ana kontrol döngüsü.
        1. /albatros/parkur3_active durumunu sürekli yayınlar.
        2. /mavros/state üzerinden GUIDED mod doğrulamasını kontrol eder.
        3. WP5 reached + GPS mesafe (AND) gerçekleştiğinde GUIDED mod değişimini yönetir.
        """
        # Her durumda mevcut parkur3_active durumunu yayınla
        active_msg = Bool()
        active_msg.data = self.parkur3_active
        self.parkur3_active_pub.publish(active_msg)

        # Parkur 3 zaten başarıyla açılmışsa ve mod GUIDED ise başka işlem yapma
        if self.parkur3_active and self.current_mode.upper() == "GUIDED":
            return

        now = time.monotonic()

        # Eğer mod geçişi istendiyse ancak henüz /mavros/state 'GUIDED' olmadıysa
        if self.mode_change_requested and not self.parkur3_active:
            # mod /mavros/state üzerinde GUIDED olmuş mu kontrol et
            if self.current_mode.upper() == "GUIDED":
                self.parkur3_active = True
                self.get_logger().info("[ParkurGecisNode] Uçuş modu GUIDED olarak doğrulandı! Parkur-3 aktif.")
                return

            # Toplam zaman aşımı kontrolü (mode_change_timeout_sec)
            elapsed_time = now - self.request_start_time
            if elapsed_time > self.mode_change_timeout_sec and self.attempt_count >= self.max_mode_change_attempts:
                if not self.error_logged:
                    self.get_logger().error(
                        f"==================================================================\n"
                        f"[ParkurGecisNode] [HATA] Max deneme hakkı ({self.max_mode_change_attempts}) doldu ve "
                        f"zaman aşımı süresi ({self.mode_change_timeout_sec}s) aşıldı!\n"
                        f"Mevcut mod hala '{self.current_mode}' (GUIDED değil).\n"
                        f"Parkur-3 kamikaze görevi BAŞLATILMAYACAK!\n"
                        f"=================================================================="
                    )
                    self.error_logged = True
                return

        # 5. Waypoint MAVROS reached + WP5 GPS mesafe doğrulamasını BİRLİKTE (AND) kontrol et
        if not self.parkur3_active and self.check_5th_waypoint_reached():
            # Eğer henüz max deneme hakkına ulaşılmadıysa ve retry_interval_sec kadar süre geçtiyse tekrar dene
            if self.attempt_count < self.max_mode_change_attempts:
                if now - self.last_attempt_time >= self.retry_interval_sec:
                    self.attempt_count += 1
                    self.last_attempt_time = now
                    if self.request_start_time == 0.0:
                        self.request_start_time = now
                    self.mode_change_requested = True
                    self.get_logger().info(
                        f"[ParkurGecisNode] 5. WP MAVROS reached + GPS mesafe DOĞRULANDI. "
                        f"AUTO -> GUIDED geçiş denemesi {self.attempt_count}/{self.max_mode_change_attempts}..."
                    )
                    self.execute_mode_transition()

    def check_5th_waypoint_reached(self) -> bool:
        """
        5. Waypoint'e ulaşıldığını iki koşulun BİRLİKTE (AND) sağlanmasıyla doğrular:
        1. MAVROS WaypointReached mesajı (target_wp_seq)
        2. WP5 GPS mesafe kontrolü (<= distance_threshold_m)
        """
        target_lat, target_lon = self.TARGET_WAYPOINTS[-1]

        # 1. Koşul: MAVROS WaypointReached mesajı
        seq_reached = (self.target_wp_seq is not None) and (self.target_wp_seq in self.reached_wp_seqs)

        # 2. Koşul: WP5 GPS Mesafe kontrolü
        dist_reached = False
        if self.gps_valid:
            dist = self.calculate_distance_m(self.current_lat, self.current_lon, target_lat, target_lon)
            if dist <= self.distance_threshold_m:
                dist_reached = True

        # Prompt gereği: 5. waypoint MAVROS reached mesajı + WP5 GPS mesafe doğrulamasını BİRLİKTE (AND) kullan
        return seq_reached and dist_reached

    def execute_mode_transition(self):
        """
        MAVROS /mavros/set_mode servisini çağırarak aracı GUIDED moda geçirmeyi talep eder.
        Servis cevabı gelse dahi parkur3_active = True yapılmaz, /mavros/state beklenir.
        """
        if not MAVROS_AVAILABLE or self.set_mode_client is None:
            self.get_logger().error("[ParkurGecisNode] [HATA] MAVROS servisi bulunamadı! Servis çağrılamıyor.")
            return

        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("[ParkurGecisNode] [UYARI] /mavros/set_mode servisi 1s içinde yanıt vermedi.")
            return

        request = SetMode.Request()
        request.custom_mode = 'GUIDED'

        self.get_logger().info(f"[ParkurGecisNode] SetMode isteği gönderiliyor (GUIDED, Deneme: {self.attempt_count}/{self.max_mode_change_attempts})...")
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self.mode_change_response_callback)

    def mode_change_response_callback(self, future):
        """SetMode servis yanıtını işler. Doğrulamayı /mavros/state'e bırakır."""
        try:
            response = future.result()
            mode_sent = getattr(response, 'mode_sent', False) or getattr(response, 'success', False)
            if mode_sent:
                self.get_logger().info(
                    f"[ParkurGecisNode] SetMode(GUIDED) servis yanıtı olumlu "
                    f"(Deneme {self.attempt_count}/{self.max_mode_change_attempts}). /mavros/state doğrulaması bekleniyor..."
                )
            else:
                self.get_logger().warn(
                    f"[ParkurGecisNode] SetMode(GUIDED) servis talebi Pixhawk tarafından reddedildi "
                    f"(Deneme {self.attempt_count}/{self.max_mode_change_attempts})."
                )
        except Exception as e:
            self.get_logger().error(f"[ParkurGecisNode] SetMode servis çağrısında istisna oluştu: {e}")

    @staticmethod
    def calculate_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """İki coğrafi koordinat arasındaki Haversine mesafesini (metre) hesaplar."""
        if math.isclose(lat1, lat2) and math.isclose(lon1, lon2):
            return 0.0
        r_earth = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)

        a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0)**2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return r_earth * c


def main(args=None):
    rclpy.init(args=args)
    node = ParkurGecisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
