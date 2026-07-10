#!/usr/bin/env python3
"""
mission_node.py — LODOS Albatros Görev Yönetim Node'u
======================================================

Görevi:
    Otonom İnsansız Su Üstü Aracı (USV) için görev akışını yönetir.
    Waypoint listesini takip eder, görev aşamalarını günceller ve
    sistemdeki diğer node'lara mevcut görev durumunu yayınlar.

ÖNEMLİ — Bu node YAPMAZ:
    - Navigasyon / yol planlama
    - Engel kaçınma algoritması
    - Motor / hız komutu gönderme
    Bu işlemler ilgili diğer node'lar tarafından yürütülür.

Abone olunan topic'ler:
    /mavros/global_position/global  -> sensor_msgs/msg/NavSatFix
    /mavros/state                   -> mavros_msgs/msg/State

Yayınlanan topic:
    /mission_state                  -> std_msgs/msg/String (JSON)

Yayınlanan JSON alanları:
    current_waypoint_id   (int)   — Aktif waypoint kimliği (0-tabanlı)
    target_latitude       (float) — Hedef waypoint enlemi
    target_longitude      (float) — Hedef waypoint boylamı
    distance_to_waypoint  (float) — Hedefe metre cinsinden mesafe
    mission_stage         (str)   — Mevcut görev aşaması
    mission_completed     (bool)  — Tüm waypoint'ler tamamlandı mı?
    vehicle_connected     (bool)  — MAVROS bağlantı durumu
    vehicle_armed         (bool)  — Araç arm durumu
    vehicle_mode          (str)   — Pixhawk uçuş modu

Parametreler:
    acceptance_radius_m   (float, default: 2.0)  — Waypoint kabul yarıçapı [m]
    publish_period_sec    (float, default: 0.5)  — Durum yayın periyodu [s]
    waypoint_log_interval (float, default: 5.0)  — Mesafe log aralığı [s]
"""

import json
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State


# ─── Görev Aşamaları Sabitleri ────────────────────────────────────────────────

class MissionStage:
    """Görev aşama sabitlerini tutan yardımcı sınıf."""
    START            = 'START'
    PARKUR_1         = 'PARKUR_1'
    PARKUR_2         = 'PARKUR_2'
    PARKUR_3         = 'PARKUR_3'
    MISSION_FINISHED = 'MISSION_FINISHED'


# ─── Waypoint Veri Yapısı ─────────────────────────────────────────────────────

class Waypoint:
    """
    Tek bir waypoint'i temsil eden veri sınıfı.

    Attributes:
        waypoint_id (int):   Waypoint sıra numarası (0-tabanlı).
        latitude    (float): Enlem [decimal degrees].
        longitude   (float): Boylam [decimal degrees].
        label       (str):   İnsan okunabilir etiket (opsiyonel, log için).
    """

    def __init__(self, waypoint_id: int, latitude: float,
                 longitude: float, label: str = '') -> None:
        self.waypoint_id = waypoint_id
        self.latitude    = latitude
        self.longitude   = longitude
        self.label       = label or f'Waypoint_{waypoint_id}'

    def __repr__(self) -> str:
        return (
            f'Waypoint(id={self.waypoint_id}, '
            f'lat={self.latitude:.7f}, '
            f'lon={self.longitude:.7f}, '
            f'label="{self.label}")'
        )


# ─── Ana Node Sınıfı ──────────────────────────────────────────────────────────

class MissionNode(Node):
    """
    LODOS Albatros görev yönetim node'u.

    Sorumlulukları:
        - Waypoint listesini yükler ve sırası ile takip eder.
        - Her GPS güncellemesinde Haversine formülü ile mesafe hesaplar.
        - Kabul yarıçapına girilince sonraki waypoint'i aktif eder.
        - Görev aşamasını (PARKUR_1 / PARKUR_2 / PARKUR_3) otomatik günceller.
        - Görev durumunu /mission_state topic'inde JSON olarak yayınlar.
    """

    # Dünya'nın ortalama yarıçapı [m] — Haversine hesabında kullanılır
    _EARTH_RADIUS_M: float = 6_371_000.0

    def __init__(self) -> None:
        super().__init__('mission_node')

        # ─── ROS2 Parametreleri ───────────────────────────────────────────────
        self.declare_parameter('acceptance_radius_m',   2.0)
        self.declare_parameter('publish_period_sec',    0.5)
        self.declare_parameter('waypoint_log_interval', 5.0)

        self._acceptance_radius: float = (
            self.get_parameter('acceptance_radius_m').value
        )
        self._publish_period: float = (
            self.get_parameter('publish_period_sec').value
        )
        self._log_interval: float = (
            self.get_parameter('waypoint_log_interval').value
        )

        # ─── Araç Durum Değişkenleri ─────────────────────────────────────────
        self._vehicle_connected: bool = False  # MAVROS bağlantı durumu
        self._vehicle_armed: bool     = False  # Pixhawk arm durumu
        self._vehicle_mode: str       = 'UNKNOWN'  # Pixhawk uçuş modu

        # ─── GPS Konum Değişkenleri ───────────────────────────────────────────
        self._current_lat: float | None = None   # Anlık enlem
        self._current_lon: float | None = None   # Anlık boylam

        # ─── Görev Durum Değişkenleri ─────────────────────────────────────────
        self._current_wp_index: int        = 0              # Aktif waypoint indeksi
        self._distance_to_wp: float        = 0.0            # Hedefe olan mesafe [m]
        self._mission_stage: str           = MissionStage.START  # Görev aşaması
        self._mission_completed: bool      = False          # Tüm WP tamamlandı mı?

        # ─── Throttle Takip Değişkenleri ─────────────────────────────────────
        self._last_warn_time:  dict[str, float] = {}  # Uyarı log throttle
        self._last_log_time:   float            = 0.0 # Mesafe log throttle
        self._WARN_INTERVAL_SEC: float          = 5.0 # Uyarı asgari aralığı

        # ─── Waypoint Listesini Yükle ─────────────────────────────────────────
        self._waypoints: list[Waypoint] = []
        self.load_waypoints()

        # ─── Subscriber'lar ───────────────────────────────────────────────────
        self._gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile=10,
        )

        self._state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile=10,
        )

        # ─── Publisher ────────────────────────────────────────────────────────
        self._mission_pub = self.create_publisher(
            String,
            '/mission_state',
            qos_profile=10,
        )

        # ─── Periyodik Yayın Timer'ı ──────────────────────────────────────────
        self._publish_timer = self.create_timer(
            self._publish_period,
            self.publish_mission_state,
        )

        # ─── Başlangıç Logu ───────────────────────────────────────────────────
        self.get_logger().info(
            f'[MissionNode] Başlatıldı. '
            f'Toplam waypoint: {len(self._waypoints)}, '
            f'Kabul yarıçapı: {self._acceptance_radius} m, '
            f'Yayın periyodu: {self._publish_period} s'
        )

        if self._waypoints:
            first = self._waypoints[0]
            self.get_logger().info(
                f'[MissionNode] İlk hedef: {first.label} '
                f'(lat={first.latitude:.7f}, lon={first.longitude:.7f})'
            )

    # ─────────────────────────────────────────────────────────────────────────
    # WAYPOINT YÖNETİMİ
    # ─────────────────────────────────────────────────────────────────────────

    def load_waypoints(self) -> None:
        """
        Önceden tanımlanmış waypoint listesini yükler.

        Waypoint'ler üç parkura ayrılmıştır:
            - Waypoint 0-1   : PARKUR_1 (örn. başlangıç bölgesi)
            - Waypoint 2-3   : PARKUR_2 (engel bölgesi)
            - Waypoint 4     : PARKUR_3 (kamikaze / bitiş)

        NOT: Gerçek koordinatlar yarışma alanına göre güncellenmelidir.
             Aşağıdaki değerler yer tutucu (placeholder) değerlerdir.
        """
        # ── Waypoint tanımları ──────────────────────────────────────────────
        # Her girdi: (waypoint_id, latitude, longitude, etiket)
        # Koordinatları yarışma alanına göre güncelleyiniz.
        raw_waypoints = [
            # PARKUR_1 — Başlangıç bölgesi
            (0, 0.000000, 0.000000, 'PARKUR_1_WP_A'),
            (1, 0.000000, 0.000000, 'PARKUR_1_WP_B'),
            # PARKUR_2 — Engel bölgesi
            (2, 0.000000, 0.000000, 'PARKUR_2_WP_A'),
            (3, 0.000000, 0.000000, 'PARKUR_2_WP_B'),
            # PARKUR_3 — Kamikaze / bitiş
            (4, 0.000000, 0.000000, 'PARKUR_3_HEDEF'),
        ]

        self._waypoints = [
            Waypoint(wp_id, lat, lon, label)
            for wp_id, lat, lon, label in raw_waypoints
        ]

        self.get_logger().info(
            f'[load_waypoints] {len(self._waypoints)} waypoint yüklendi.'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # SUBSCRIBER CALLBACK'LERİ
    # ─────────────────────────────────────────────────────────────────────────

    def gps_callback(self, msg: NavSatFix) -> None:
        """
        /mavros/global_position/global topic'inden gelen GPS verisini işler.

        Her yeni GPS mesajında:
            1. Güncel enlem ve boylam okunur.
            2. Aktif waypoint'e olan mesafe Haversine formülü ile hesaplanır.
            3. Araç waypoint kabul yarıçapına girdiyse waypoint geçişi tetiklenir.

        Args:
            msg: sensor_msgs/msg/NavSatFix mesajı.
        """
        # GPS fix kalitesi kontrolü — STATUS_NO_FIX (-1) ise veriyi atla
        if msg.status.status < 0:
            self._warn_throttle(
                'gps_no_fix',
                '[gps_callback] GPS fix yok (status < 0). Veri atlanıyor.'
            )
            return

        # Güncel konumu kaydet
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude

        # Aktif waypoint kontrolü
        self.check_waypoint()

    def state_callback(self, msg: State) -> None:
        """
        /mavros/state topic'inden gelen araç durum verisini işler.

        Args:
            msg: mavros_msgs/msg/State mesajı.
        """
        prev_connected = self._vehicle_connected
        prev_armed     = self._vehicle_armed

        self._vehicle_connected = msg.connected
        self._vehicle_armed     = msg.armed
        self._vehicle_mode      = msg.mode

        # Bağlantı veya arm durumu değişikliklerini logla
        if self._vehicle_connected != prev_connected:
            status = 'BAĞLANDI' if self._vehicle_connected else 'BAĞLANTI KESİLDİ'
            self.get_logger().info(
                f'[state_callback] MAVROS bağlantı durumu: {status}'
            )

        if self._vehicle_armed != prev_armed:
            status = 'ARM' if self._vehicle_armed else 'DISARM'
            self.get_logger().info(
                f'[state_callback] Araç durumu: {status}'
            )

    # ─────────────────────────────────────────────────────────────────────────
    # MESafe HESABI
    # ─────────────────────────────────────────────────────────────────────────

    def calculate_distance(self, lat1: float, lon1: float,
                           lat2: float, lon2: float) -> float:
        """
        İki GPS koordinatı arasındaki yüzey mesafesini Haversine formülü
        ile metre cinsinden hesaplar.

        Haversine formülü:
            a = sin²(Δlat/2) + cos(lat1) × cos(lat2) × sin²(Δlon/2)
            c = 2 × atan2(√a, √(1−a))
            d = R × c

        Args:
            lat1 (float): Başlangıç enlemi  [decimal degrees].
            lon1 (float): Başlangıç boylamı [decimal degrees].
            lat2 (float): Bitiş enlemi      [decimal degrees].
            lon2 (float): Bitiş boylamı     [decimal degrees].

        Returns:
            float: İki nokta arasındaki mesafe [metre].
        """
        # Derece → Radyan dönüşümü
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        dlat_r = math.radians(lat2 - lat1)
        dlon_r = math.radians(lon2 - lon1)

        # Haversine çarpanı
        a = (
            math.sin(dlat_r / 2) ** 2
            + math.cos(lat1_r) * math.cos(lat2_r)
            * math.sin(dlon_r / 2) ** 2
        )
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

        return self._EARTH_RADIUS_M * c

    # ─────────────────────────────────────────────────────────────────────────
    # WAYPOINT KONTROL VE GEÇİŞ
    # ─────────────────────────────────────────────────────────────────────────

    def check_waypoint(self) -> None:
        """
        Aktif waypoint ile mevcut GPS konumu arasındaki mesafeyi hesaplar.
        Araç kabul yarıçapına (acceptance_radius_m) girerse görev güncellenir.

        Bu metot her GPS callback'inde çağrılır.
        Görev tamamlandıysa veya GPS verisi yoksa erken çıkar.
        """
        # Görev zaten tamamlandıysa işlem yapma
        if self._mission_completed:
            return

        # Waypoint listesi boşsa işlem yapma
        if not self._waypoints:
            self._warn_throttle(
                'no_waypoints',
                '[check_waypoint] Waypoint listesi boş!'
            )
            return

        # GPS verisi henüz gelmemişse bekle
        if self._current_lat is None or self._current_lon is None:
            return

        # Aktif waypoint
        active_wp: Waypoint = self._waypoints[self._current_wp_index]

        # Hedef waypoint koordinatları yer tutucu (0,0) ise uyar
        if active_wp.latitude == 0.0 and active_wp.longitude == 0.0:
            self._warn_throttle(
                f'wp_{self._current_wp_index}_placeholder',
                f'[check_waypoint] {active_wp.label} koordinatları '
                f'yer tutucu (0.0, 0.0). Gerçek koordinatları giriniz!'
            )

        # Hedefe olan mesafeyi Haversine ile hesapla
        self._distance_to_wp = self.calculate_distance(
            self._current_lat, self._current_lon,
            active_wp.latitude, active_wp.longitude,
        )

        # Periyodik mesafe logu (throttle)
        now = time.monotonic()
        if now - self._last_log_time >= self._log_interval:
            self.get_logger().info(
                f'[check_waypoint] Hedef: {active_wp.label} | '
                f'Mesafe: {self._distance_to_wp:.2f} m | '
                f'Kabul yarıçapı: {self._acceptance_radius} m'
            )
            self._last_log_time = now

        # Kabul yarıçapı kontrolü — waypoint'e ulaşıldı mı?
        if self._distance_to_wp <= self._acceptance_radius:
            self.get_logger().info(
                f'[check_waypoint] ✓ Waypoint ulaşıldı: {active_wp.label} '
                f'(mesafe={self._distance_to_wp:.2f} m)'
            )
            # Görev akışını bir sonraki waypoint'e ilerlet
            self.update_mission()

    # ─────────────────────────────────────────────────────────────────────────
    # GÖREV GÜNCELLEME
    # ─────────────────────────────────────────────────────────────────────────

    def update_mission(self) -> None:
        """
        Bir waypoint tamamlandığında görev akışını günceller.

        İşlemler:
            1. Bir sonraki waypoint'i aktif eder.
            2. Waypoint indeksine göre görev aşamasını (PARKUR_X) günceller.
            3. Son waypoint tamamlandıysa mission_completed'ı True yapar.

        Görev aşaması eşleme tablosu (load_waypoints ile uyumlu olmalı):
            wp_index == 0  -> PARKUR_1  (ilk waypoint'e henüz başlandı)
            wp_index == 1  -> PARKUR_1  (parkur 1 devam)
            wp_index == 2  -> PARKUR_2
            wp_index == 3  -> PARKUR_2
            wp_index >= 4  -> PARKUR_3
        """
        next_index = self._current_wp_index + 1

        # Son waypoint tamamlandıysa görevi bitir
        if next_index >= len(self._waypoints):
            self._mission_stage     = MissionStage.MISSION_FINISHED
            self._mission_completed = True
            self.get_logger().info(
                '[update_mission] ★ TÜM GÖREVLER TAMAMLANDI. '
                f'Toplam {len(self._waypoints)} waypoint geçildi.'
            )
            return

        # Sonraki waypoint'e geç
        self._current_wp_index = next_index
        next_wp = self._waypoints[self._current_wp_index]

        # Görev aşamasını yeni waypoint indeksine göre güncelle
        self._mission_stage = self._resolve_stage(self._current_wp_index)

        self.get_logger().info(
            f'[update_mission] → Yeni hedef: {next_wp.label} | '
            f'Aşama: {self._mission_stage}'
        )

    def _resolve_stage(self, wp_index: int) -> str:
        """
        Verilen waypoint indeksine göre görev aşamasını belirler.

        Bu fonksiyon load_waypoints() içindeki waypoint sıralaması ile
        senkronize tutulmalıdır.

        Args:
            wp_index (int): Aktif waypoint sıra numarası (0-tabanlı).

        Returns:
            str: MissionStage sabiti.
        """
        # ── Parkur eşleme tablosu ─────────────────────────────────────────
        # Waypoint aralıklarını parkurlara göre tanımlayan sözlük.
        # Anahtarlar (başlangıç, bitiş_dahil) aralığını gösterir.
        stage_map = [
            (range(0, 2), MissionStage.PARKUR_1),   # WP 0-1 → PARKUR_1
            (range(2, 4), MissionStage.PARKUR_2),   # WP 2-3 → PARKUR_2
            (range(4, 99), MissionStage.PARKUR_3),  # WP 4+  → PARKUR_3
        ]

        for wp_range, stage in stage_map:
            if wp_index in wp_range:
                return stage

        # Sınır dışı durum — güvenli geri dönüş
        self.get_logger().warn(
            f'[_resolve_stage] wp_index={wp_index} için aşama bulunamadı. '
            'PARKUR_3 döndürülüyor.'
        )
        return MissionStage.PARKUR_3

    # ─────────────────────────────────────────────────────────────────────────
    # GÖREV DURUMU YAYINI
    # ─────────────────────────────────────────────────────────────────────────

    def publish_mission_state(self) -> None:
        """
        Her _publish_period saniyede bir /mission_state topic'ine
        JSON formatında görev durum mesajı yayınlar.

        Yayınlanan alanlar:
            current_waypoint_id  (int)
            target_latitude      (float)
            target_longitude     (float)
            distance_to_waypoint (float)
            mission_stage        (str)
            mission_completed    (bool)
            vehicle_connected    (bool)
            vehicle_armed        (bool)
            vehicle_mode         (str)
        """
        # Aktif waypoint bilgilerini al
        if self._waypoints and self._current_wp_index < len(self._waypoints):
            active_wp       = self._waypoints[self._current_wp_index]
            wp_id           = active_wp.waypoint_id
            target_lat      = active_wp.latitude
            target_lon      = active_wp.longitude
        else:
            # Waypoint listesi boş veya indeks dışı — güvenli varsayılanlar
            wp_id      = -1
            target_lat = 0.0
            target_lon = 0.0

        # Durum sözlüğü
        state_dict = {
            'current_waypoint_id':   wp_id,
            'target_latitude':       round(target_lat, 8),
            'target_longitude':      round(target_lon, 8),
            'distance_to_waypoint':  round(self._distance_to_wp, 3),
            'mission_stage':         self._mission_stage,
            'mission_completed':     self._mission_completed,
            'vehicle_connected':     self._vehicle_connected,
            'vehicle_armed':         self._vehicle_armed,
            'vehicle_mode':          self._vehicle_mode,
        }

        msg = String()
        msg.data = json.dumps(state_dict, ensure_ascii=False)
        self._mission_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # YARDIMCI METOTLAR
    # ─────────────────────────────────────────────────────────────────────────

    def _warn_throttle(self, key: str, message: str) -> None:
        """
        Aynı uyarının _WARN_INTERVAL_SEC saniyede en fazla bir kez
        loglanmasını sağlar. Uyarı spam'ini önler.

        Args:
            key (str):     Uyarıyı tanımlayan benzersiz anahtar.
            message (str): Loglanacak uyarı metni.
        """
        now  = time.monotonic()
        last = self._last_warn_time.get(key, 0.0)
        if now - last >= self._WARN_INTERVAL_SEC:
            self.get_logger().warn(message)
            self._last_warn_time[key] = now


# ─── Giriş Noktası ────────────────────────────────────────────────────────────

def main(args=None) -> None:
    """Node giriş noktası. ROS2 spin döngüsünü başlatır."""
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            '[MissionNode] Kapatılıyor (KeyboardInterrupt).'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
