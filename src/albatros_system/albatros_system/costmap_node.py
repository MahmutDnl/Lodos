#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros İnsansız Deniz Aracı — Costmap Node
# =============================================================================
# Dosya    : costmap_node.py
# Node adı : costmap_node
# Paket    : albatros_system
# Görev    : YOLO tespitlerini, GPS konumunu ve IMU yaw açısını birleştirerek
#            araç-merkezli 2D OccupancyGrid (costmap) üretmek.
#            Karar vermez, motor komutu göndermez; yalnızca harita üretir.
# Yazan    : LODOS Yazılım Ekibi
# Tarih    : 2026
# =============================================================================
#
# Girdi Topic'leri:
#   /albatros/gps/fix           [sensor_msgs/NavSatFix]
#       Aracın anlık GPS konumu (enlem, boylam).
#
#   /albatros/imu/data          [sensor_msgs/Imu]
#       Aracın yönelimi — quaternion'dan yaw (başlık açısı) çıkarılır.
#
#   /albatros/yolo/obstacles    [std_msgs/String]  (JSON)
#       YOLO node'undan gelen engel adayları. Format:
#       {
#         "stamp": float,
#         "obstacles": [
#           {
#             "type": "obstacle_buoy" | "border_buoy" | ...,
#             "class_name": str,
#             "confidence": float,
#             "bbox": {"x_min", "y_min", "x_max", "y_max", "width", "height"},
#             "center": {"x": int, "y": int}
#           }, ...
#         ]
#       }
#
# Çıktı Topic'leri:
#   /albatros/costmap/grid      [nav_msgs/OccupancyGrid]
#       2D engel maliyet haritası (0–100). Obstacle avoidance ve mission
#       manager node'larının temel girdisi.
#
#   /albatros/costmap/info      [std_msgs/String]  (JSON)
#       Harita meta verisi: hücre sayısı, çözünürlük, aktif engel sayısı vb.
#
#   /albatros/costmap/markers   [visualization_msgs/MarkerArray]
#       RViz2 görselleştirme için engel marker'ları (opsiyonel).
#
# Parametreler:
#   simulate_mode       (bool,  varsayılan: True)
#       True  → Sahte GPS/IMU/engel verisiyle çalışır, gerçek sensör gerekmez.
#       False → Gerçek topic'lerden veri alır.
#
#   resolution          (float, varsayılan: 0.25)
#       Her grid hücresinin gerçek dünya karşılığı (metre/hücre).
#
#   width_cells         (int,   varsayılan: 100)
#       Harita genişliği (hücre sayısı). width_cells * resolution = gerçek genişlik.
#
#   height_cells        (int,   varsayılan: 100)
#       Harita yüksekliği (hücre sayısı).
#
#   inflation_radius    (float, varsayılan: 1.5)
#       Engel etrafına eklenen güvenlik tamponu (metre).
#
#   decay_time          (float, varsayılan: 3.0)
#       Güncelleme gelmeyen engellerin haritadan silinme süresi (saniye).
#
#   publish_rate        (float, varsayılan: 5.0)
#       Costmap yayın frekansı (Hz). Çok yüksek değer CPU kullanımını artırır.
#
#   camera_fov_deg      (float, varsayılan: 70.0)
#       Kameranın yatay görüş açısı (derece). Piksel → açı dönüşümü için kullanılır.
#
#   camera_width_px     (int,   varsayılan: 640)
#       Kamera görüntüsü piksel genişliği. Görüş açısı hesabında bölen.
#
#   buoy_diameter_m     (float, varsayılan: 0.30)
#       Bilinen duba çapı (metre). Bounding box → mesafe tahmininde kullanılır.
#
# Costmap Maliyet Sözlüğü:
#   0        → Boş / güvenli alan
#   50–79    → Şişirilmiş engel tamponu (inflation zone)
#   80–99    → Yüksek maliyet (bilgi dubası, sınır dubası)
#   100      → Kesinlikle engel (sarı duba / lethal)
#   -1       → Bilinmeyen alan (henüz haritalanmamış)
#
# Uyumluluk:
#   ROS2 Jazzy / Ubuntu 24.04
#   nav_msgs, sensor_msgs, std_msgs, visualization_msgs
# =============================================================================

import json
import math
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


# =============================================================================
# Sabitler — Maliyet Değerleri
# =============================================================================

COST_FREE       = 0    # Güvenli alan
COST_UNKNOWN    = -1   # Haritalanmamış
COST_INFO       = 60   # Bilgi dubası (kırmızı/yeşil)
COST_BORDER     = 80   # Sınır dubası (turuncu)
COST_LETHAL     = 100  # Kesinlikle engel (sarı/siyah duba)

# Şişirme zonundaki maksimum maliyet (lethal'a yakın ama geçilebilir)
COST_INFLATION_MAX = 75

# Duba türü → temel maliyet eşleme tablosu
BUOY_COST_MAP = {
    "obstacle_buoy":         COST_LETHAL,   # Sarı duba → tamamen engel
    "border_buoy":           COST_BORDER,   # Turuncu duba → sınır/yüksek maliyet
    "target_or_colored_buoy": COST_INFO,    # Kırmızı/yeşil → bilgi amaçlı
    "unknown":               50,            # Bilinmeyen
}

# RViz marker renkleri: (r, g, b, a) — duba türüne göre
BUOY_MARKER_COLOR = {
    "obstacle_buoy":          (1.0, 1.0, 0.0, 0.85),   # Sarı
    "border_buoy":            (1.0, 0.5, 0.0, 0.85),   # Turuncu
    "target_or_colored_buoy": (0.2, 0.9, 0.2, 0.85),   # Yeşil
    "unknown":                (0.5, 0.5, 0.5, 0.7),    # Gri
}

# Topic adları
GPS_TOPIC        = '/albatros/gps/fix'
IMU_TOPIC        = '/albatros/imu/data'
OBSTACLES_TOPIC  = '/albatros/yolo/obstacles'
COSTMAP_TOPIC    = '/albatros/costmap/grid'
INFO_TOPIC       = '/albatros/costmap/info'
MARKERS_TOPIC    = '/albatros/costmap/markers'

# TF frame
COSTMAP_FRAME_ID = 'map'

# Simülasyon: sahte engel yenileme aralığı (saniye)
SIM_OBSTACLE_UPDATE_PERIOD = 2.0


# =============================================================================
# Yardımcı Fonksiyonlar
# =============================================================================

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Quaternion'dan yaw (z ekseni dönüş) açısını çıkarır.

    Dönüş:
        yaw açısı (radyan, [-π, +π])
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def latlon_to_local_enu(lat: float, lon: float,
                         origin_lat: float, origin_lon: float) -> tuple:
    """
    WGS84 koordinatını yerel ENU (East-North-Up) metrik koordinata çevirir.
    Küçük mesafeler için düzlemsel (planar) yaklaşım kullanılır (< 1 km hata ihmal edilebilir).

    Argümanlar:
        lat, lon         : Hedef nokta (derece)
        origin_lat, lon  : Referans orijin noktası (derece)

    Dönüş:
        (east_m, north_m) — metre cinsinden ofset
    """
    EARTH_RADIUS_M = 6_371_000.0

    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    lat_rad = math.radians(origin_lat)

    north_m = dlat * EARTH_RADIUS_M
    east_m  = dlon * EARTH_RADIUS_M * math.cos(lat_rad)

    return east_m, north_m


def pixel_to_bearing(cx_px: int, img_width_px: int, fov_deg: float, vehicle_yaw_rad: float) -> float:
    """
    Kamera görüntüsündeki piksel x koordinatını araç koordinat sisteminde
    mutlak başlık açısına (bearing) çevirir.

    Argümanlar:
        cx_px          : Nesnenin görüntüdeki yatay piksel merkezi
        img_width_px   : Görüntünün toplam piksel genişliği
        fov_deg        : Kameranın yatay görüş açısı (derece)
        vehicle_yaw_rad: Aracın IMU'dan gelen yaw açısı (radyan, kuzey=0)

    Dönüş:
        Mutlak başlık açısı (radyan, ENU sisteminde x=doğu, y=kuzey)
    """
    # Görüntü merkezine göre normalize edilmiş piksel ofseti [-0.5, +0.5]
    norm_x = (cx_px - img_width_px / 2.0) / img_width_px

    # Görüntü merkezinde açısal sapma (radyan)
    half_fov_rad = math.radians(fov_deg / 2.0)
    angle_from_center = norm_x * 2.0 * half_fov_rad

    # Araç yaw + kamera sapması = mutlak başlık
    bearing = vehicle_yaw_rad + angle_from_center
    return bearing


def estimate_distance_from_bbox(bbox_width_px: int, img_width_px: int,
                                 fov_deg: float, buoy_diameter_m: float) -> float:
    """
    Bounding box genişliği ve bilinen duba çapından perspektif mesafe tahmini yapar.

    Formül:
        d = (buoy_diameter_m * img_width_px) / (2 * bbox_width_px * tan(fov/2))

    Argümanlar:
        bbox_width_px  : Bounding box piksel genişliği
        img_width_px   : Görüntü piksel genişliği
        fov_deg        : Kameranın yatay görüş açısı (derece)
        buoy_diameter_m: Bilinen gerçek duba çapı (metre)

    Dönüş:
        Tahmini mesafe (metre). Sıfır bbox için güvenli bir maksimum döner.
    """
    if bbox_width_px <= 0:
        return 15.0  # Belirsiz → uzak kabul et

    half_fov_rad = math.radians(fov_deg / 2.0)
    tan_half_fov = math.tan(half_fov_rad)

    if tan_half_fov < 1e-9:
        return 15.0

    distance = (buoy_diameter_m * img_width_px) / (2.0 * bbox_width_px * tan_half_fov)

    # Fiziksel sınırlar: 0.5 m ile 30 m arası
    distance = max(0.5, min(distance, 30.0))
    return distance


def enu_to_grid_cell(east_m: float, north_m: float,
                      vehicle_east: float, vehicle_north: float,
                      vehicle_yaw_rad: float,
                      resolution: float, width_cells: int, height_cells: int) -> tuple:
    """
    ENU koordinatındaki bir noktayı araç-merkezli grid hücresine çevirir.

    Grid orijini haritanın sol-alt köşesidir (nav_msgs/OccupancyGrid standardı).
    Araç her zaman haritanın ortasında konumlanır.

    Dönüş:
        (col, row) — grid hücre indeksi, ya da geçersizse None
    """
    # Araca göre göreceli konum (dünya ENU)
    rel_east  = east_m  - vehicle_east
    rel_north = north_m - vehicle_north

    # Araç yönüne göre döndür (araç-merkezli koordinat)
    cos_yaw = math.cos(-vehicle_yaw_rad)
    sin_yaw = math.sin(-vehicle_yaw_rad)
    local_x =  rel_east * cos_yaw - rel_north * sin_yaw
    local_y =  rel_east * sin_yaw + rel_north * cos_yaw

    # Grid hücresi — araç haritanın tam ortasında
    center_col = width_cells  // 2
    center_row = height_cells // 2

    col = int(center_col + local_x / resolution)
    row = int(center_row + local_y / resolution)

    if 0 <= col < width_cells and 0 <= row < height_cells:
        return col, row
    return None


# =============================================================================
# Aktif Engel Kaydı
# =============================================================================

class ObstacleRecord:
    """
    Costmap üzerinde takip edilen tek bir engel kaydı.
    Engel mevcut olduğu sürece timestamp güncellenir;
    decay_time geçince haritadan silinir.
    """

    __slots__ = ('east_m', 'north_m', 'cost', 'buoy_type', 'class_name',
                 'confidence', 'last_seen', 'marker_id')

    def __init__(self, east_m: float, north_m: float, cost: int,
                 buoy_type: str, class_name: str, confidence: float,
                 marker_id: int):
        self.east_m     = east_m
        self.north_m    = north_m
        self.cost       = cost
        self.buoy_type  = buoy_type
        self.class_name = class_name
        self.confidence = confidence
        self.last_seen  = time.time()
        self.marker_id  = marker_id

    def refresh(self, east_m: float, north_m: float, confidence: float):
        """Engel tekrar görüldüğünde konum ve zamanı güncelle."""
        self.east_m     = east_m
        self.north_m    = north_m
        self.confidence = confidence
        self.last_seen  = time.time()

    def is_expired(self, decay_time: float) -> bool:
        """decay_time süresi geçtiyse True döner → haritadan sil."""
        return (time.time() - self.last_seen) > decay_time


# =============================================================================
# Costmap Node
# =============================================================================

class CostmapNode(Node):
    """
    YOLO tespitlari + GPS + IMU verilerinden araç-merkezli 2D OccupancyGrid üretir.

    simulate_mode=True  → Gerçek sensör olmadan sahte veriyle çalışır.
    simulate_mode=False → Gerçek ROS2 topic'lerinden veri alır.
    """

    def __init__(self):
        super().__init__('costmap_node')

        # ------------------------------------------------------------------ #
        # Parametreler
        # ------------------------------------------------------------------ #
        self.declare_parameter('simulate_mode',    True)
        self.declare_parameter('resolution',       0.25)
        self.declare_parameter('width_cells',      100)
        self.declare_parameter('height_cells',     100)
        self.declare_parameter('inflation_radius', 1.5)
        self.declare_parameter('decay_time',       3.0)
        self.declare_parameter('publish_rate',     5.0)
        self.declare_parameter('camera_fov_deg',   70.0)
        self.declare_parameter('camera_width_px',  640)
        self.declare_parameter('buoy_diameter_m',  0.30)

        self.simulate_mode    = bool(self.get_parameter('simulate_mode').value)
        self.resolution       = float(self.get_parameter('resolution').value)
        self.width_cells      = int(self.get_parameter('width_cells').value)
        self.height_cells     = int(self.get_parameter('height_cells').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.decay_time       = float(self.get_parameter('decay_time').value)
        self.publish_rate     = float(self.get_parameter('publish_rate').value)
        self.camera_fov_deg   = float(self.get_parameter('camera_fov_deg').value)
        self.camera_width_px  = int(self.get_parameter('camera_width_px').value)
        self.buoy_diameter_m  = float(self.get_parameter('buoy_diameter_m').value)

        # publish_rate güvenlik kontrolü
        if self.publish_rate <= 0.0:
            self.publish_rate = 5.0

        # Şişirme yarıçapı hücre cinsinden
        self.inflation_cells = int(math.ceil(self.inflation_radius / self.resolution))

        # ------------------------------------------------------------------ #
        # Araç durum değişkenleri
        # ------------------------------------------------------------------ #
        self._vehicle_lat   = 0.0
        self._vehicle_lon   = 0.0
        self._vehicle_yaw   = 0.0   # radyan, IMU'dan
        self._origin_lat    = None  # İlk GPS alındığında sabitlenir
        self._origin_lon    = None
        self._gps_ready     = False
        self._imu_ready     = False

        # ------------------------------------------------------------------ #
        # Engel kaydı sözlüğü: key → ObstacleRecord
        # Benzersiz engel kimliği için yaklaşım:
        #   Engelleri grid hücresine göre gruplandır (aynı hücreye gelen = aynı engel)
        # ------------------------------------------------------------------ #
        self._obstacles: dict = {}
        self._next_marker_id  = 0

        # Simülasyon: sahte engel ekleme zamanı
        self._sim_last_update = 0.0

        # ------------------------------------------------------------------ #
        # Subscriber'lar
        # ------------------------------------------------------------------ #
        if not self.simulate_mode:
            self._gps_sub = self.create_subscription(
                NavSatFix,
                GPS_TOPIC,
                self._gps_callback,
                qos_profile=qos_profile_sensor_data
            )
            self._imu_sub = self.create_subscription(
                Imu,
                IMU_TOPIC,
                self._imu_callback,
                qos_profile=qos_profile_sensor_data
            )
            self._obs_sub = self.create_subscription(
                String,
                OBSTACLES_TOPIC,
                self._obstacles_callback,
                qos_profile=10
            )
            self.get_logger().info('Gerçek sensör modunda çalışıyor.')
            self.get_logger().info(f'  GPS     : {GPS_TOPIC}')
            self.get_logger().info(f'  IMU     : {IMU_TOPIC}')
            self.get_logger().info(f'  Engeller: {OBSTACLES_TOPIC}')
        else:
            # Simülasyon modunda sahte araç konumu sabitle
            self._vehicle_lat = 40.1885
            self._vehicle_lon = 29.0610
            self._vehicle_yaw = 0.0
            self._origin_lat  = self._vehicle_lat
            self._origin_lon  = self._vehicle_lon
            self._gps_ready   = True
            self._imu_ready   = True
            self.get_logger().info('SİMÜLASYON modunda çalışıyor.')

        # ------------------------------------------------------------------ #
        # Publisher'lar
        # ------------------------------------------------------------------ #
        self._grid_pub = self.create_publisher(
            OccupancyGrid,
            COSTMAP_TOPIC,
            qos_profile=10
        )
        self._info_pub = self.create_publisher(
            String,
            INFO_TOPIC,
            qos_profile=10
        )
        self._markers_pub = self.create_publisher(
            MarkerArray,
            MARKERS_TOPIC,
            qos_profile=10
        )

        # ------------------------------------------------------------------ #
        # Ana zamanlayıcı
        # ------------------------------------------------------------------ #
        timer_period = 1.0 / self.publish_rate
        self._timer = self.create_timer(timer_period, self._timer_callback)

        # ------------------------------------------------------------------ #
        # Başlangıç logları
        # ------------------------------------------------------------------ #
        map_w_m = self.width_cells  * self.resolution
        map_h_m = self.height_cells * self.resolution

        self.get_logger().info('=' * 60)
        self.get_logger().info('Costmap Node başlatıldı.')
        self.get_logger().info(f'  Çözünürlük      : {self.resolution} m/hücre')
        self.get_logger().info(f'  Harita boyutu   : {self.width_cells}x{self.height_cells} hücre')
        self.get_logger().info(f'  Gerçek boyut    : {map_w_m:.1f}x{map_h_m:.1f} m')
        self.get_logger().info(f'  Inflation yarı  : {self.inflation_radius} m ({self.inflation_cells} hücre)')
        self.get_logger().info(f'  Decay süresi    : {self.decay_time} s')
        self.get_logger().info(f'  Yayın frekansı  : {self.publish_rate} Hz')
        self.get_logger().info(f'  Kamera FOV      : {self.camera_fov_deg}°')
        self.get_logger().info(f'  Duba çapı       : {self.buoy_diameter_m} m')
        self.get_logger().info(f'  Costmap topic   : {COSTMAP_TOPIC}')
        self.get_logger().info('=' * 60)

    # ====================================================================== #
    # Subscriber Callback'leri — Gerçek Sensör Modu
    # ====================================================================== #

    def _gps_callback(self, msg: NavSatFix):
        """GPS topic'inden aracın anlık konumunu al."""
        # Geçersiz fix durumlarını filtrele
        if msg.status.status < 0:
            return

        self._vehicle_lat = msg.latitude
        self._vehicle_lon = msg.longitude

        # İlk geçerli GPS mesajında harita orijinini sabitle
        if self._origin_lat is None:
            self._origin_lat = msg.latitude
            self._origin_lon = msg.longitude
            self.get_logger().info(
                f'Harita orijini sabitlendi: '
                f'lat={self._origin_lat:.6f}°, lon={self._origin_lon:.6f}°'
            )

        self._gps_ready = True

    def _imu_callback(self, msg: Imu):
        """IMU topic'inden quaternion alarak yaw açısını güncelle."""
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Sıfır quaternion kontrolü (henüz veri gelmemişse)
        if abs(qx) < 1e-9 and abs(qy) < 1e-9 and abs(qz) < 1e-9 and abs(qw) < 1e-9:
            return

        self._vehicle_yaw = quaternion_to_yaw(qx, qy, qz, qw)
        self._imu_ready   = True

    def _obstacles_callback(self, msg: String):
        """
        YOLO obstacle topic'inden engel listesini al ve costmap'e ekle.

        Her tespit için:
        1. Bounding box genişliği → perspektif mesafe tahmini
        2. Piksel merkezi → kamera açısı → araç yaw + açı = başlık
        3. Araç konumu + mesafe + başlık → ENU koordinatı
        4. ENU → grid hücresi → maliyet ata
        """
        if not (self._gps_ready and self._imu_ready):
            return

        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().warn(f'Engel JSON parse hatası: {exc}')
            return

        obstacles = payload.get('obstacles', [])
        if not obstacles:
            return

        # Araç ENU konumu (metre)
        vehicle_east, vehicle_north = latlon_to_local_enu(
            self._vehicle_lat, self._vehicle_lon,
            self._origin_lat, self._origin_lon
        )

        for obs in obstacles:
            self._process_obstacle(obs, vehicle_east, vehicle_north)

    # ====================================================================== #
    # Engel İşleme
    # ====================================================================== #

    def _process_obstacle(self, obs: dict, vehicle_east: float, vehicle_north: float):
        """
        Tek bir YOLO tespitini ENU koordinatına çevirip _obstacles sözlüğüne ekler.

        Argümanlar:
            obs          : YOLO obstacle dict (type, confidence, bbox, center)
            vehicle_east : Araç konumu, ENU doğu (metre)
            vehicle_north: Araç konumu, ENU kuzey (metre)
        """
        bbox   = obs.get('bbox', {})
        center = obs.get('center', {})

        cx_px      = center.get('x', self.camera_width_px // 2)
        bbox_w_px  = bbox.get('width', 0)
        buoy_type  = obs.get('type', 'unknown')
        class_name = obs.get('class_name', 'unknown')
        confidence = float(obs.get('confidence', 0.5))

        # Maliyet değeri
        cost = BUOY_COST_MAP.get(buoy_type, 50)

        # Perspektif mesafe tahmini (metre)
        dist_m = estimate_distance_from_bbox(
            bbox_w_px, self.camera_width_px,
            self.camera_fov_deg, self.buoy_diameter_m
        )

        # Engel başlık açısı (kuzey referanslı, radyan)
        bearing = pixel_to_bearing(
            cx_px, self.camera_width_px,
            self.camera_fov_deg, self._vehicle_yaw
        )

        # Engel ENU konumu
        obs_east  = vehicle_east  + dist_m * math.sin(bearing)
        obs_north = vehicle_north + dist_m * math.cos(bearing)

        # Engel kayıt anahtarı: 0.5 m ızgara hücresi büyüklüğünde yuvarlama
        # (aynı fiziksel engelin farklı tespitlerini birleştir)
        snap = 0.5
        key = (round(obs_east / snap) * snap, round(obs_north / snap) * snap)

        if key in self._obstacles:
            self._obstacles[key].refresh(obs_east, obs_north, confidence)
        else:
            marker_id = self._next_marker_id
            self._next_marker_id += 1
            self._obstacles[key] = ObstacleRecord(
                east_m=obs_east,
                north_m=obs_north,
                cost=cost,
                buoy_type=buoy_type,
                class_name=class_name,
                confidence=confidence,
                marker_id=marker_id
            )

    # ====================================================================== #
    # Simülasyon Modu — Sahte Engel Üretimi
    # ====================================================================== #

    def _update_simulated_obstacles(self):
        """
        Simülasyon modunda periyodik olarak aracın önüne sahte engeller ekler.
        Gerçek sensör olmadan costmap davranışını test etmek için kullanılır.
        """
        now = time.time()
        if now - self._sim_last_update < SIM_OBSTACLE_UPDATE_PERIOD:
            return
        self._sim_last_update = now

        import random

        # Sahte araç konumu sabittir, orijin referansı aynı
        sim_scenarios = [
            # (dist_m, bearing_deg, buoy_type, class_name)
            (5.0,  10.0, 'obstacle_buoy',          'sari_duba'),
            (8.0, -15.0, 'border_buoy',             'turuncu_duba'),
            (6.0,  30.0, 'target_or_colored_buoy',  'kirmizi_duba'),
            (4.0,  -5.0, 'obstacle_buoy',            'sari_duba'),
        ]

        vehicle_east  = 0.0
        vehicle_north = 0.0

        for dist_m, bearing_deg, buoy_type, class_name in sim_scenarios:
            bearing_rad = math.radians(bearing_deg) + self._vehicle_yaw
            obs_east  = vehicle_east  + dist_m * math.sin(bearing_rad)
            obs_north = vehicle_north + dist_m * math.cos(bearing_rad)

            # Hafif Gaussian gürültü ekle
            obs_east  += random.gauss(0.0, 0.1)
            obs_north += random.gauss(0.0, 0.1)

            snap = 0.5
            key = (round(obs_east / snap) * snap, round(obs_north / snap) * snap)
            cost = BUOY_COST_MAP.get(buoy_type, 50)

            if key in self._obstacles:
                self._obstacles[key].refresh(obs_east, obs_north, 0.85)
            else:
                marker_id = self._next_marker_id
                self._next_marker_id += 1
                self._obstacles[key] = ObstacleRecord(
                    east_m=obs_east,
                    north_m=obs_north,
                    cost=cost,
                    buoy_type=buoy_type,
                    class_name=class_name,
                    confidence=0.85,
                    marker_id=marker_id
                )

    # ====================================================================== #
    # Costmap Oluşturma
    # ====================================================================== #

    def _decay_obstacles(self):
        """
        Son görülme zamanı decay_time'ı aşan engelleri listeden çıkar.
        Bu sayede araç geçtikçe harita temizlenir.
        """
        expired_keys = [
            key for key, rec in self._obstacles.items()
            if rec.is_expired(self.decay_time)
        ]
        for key in expired_keys:
            del self._obstacles[key]

    def _build_grid(self) -> list:
        """
        Aktif engel listesinden 2D OccupancyGrid verisi oluşturur.

        Adımlar:
        1. Tüm hücreleri COST_FREE ile başlat.
        2. Her engel için grid hücresini bul → maliyet ata.
        3. Engel etrafında inflation uygula (Öklid mesafeye göre azalan maliyet).

        Dönüş:
            list[int] — genişlik*yükseklik boyutunda, satır öncelikli int listesi
        """
        total = self.width_cells * self.height_cells
        grid  = [COST_FREE] * total

        # Araç ENU konumu (metre)
        if self._origin_lat is not None:
            vehicle_east, vehicle_north = latlon_to_local_enu(
                self._vehicle_lat, self._vehicle_lon,
                self._origin_lat, self._origin_lon
            )
        else:
            vehicle_east, vehicle_north = 0.0, 0.0

        # Önce engel merkez hücrelerini işle, sonra inflation uygula
        lethal_cells = []

        for rec in self._obstacles.values():
            cell = enu_to_grid_cell(
                rec.east_m, rec.north_m,
                vehicle_east, vehicle_north,
                self._vehicle_yaw,
                self.resolution, self.width_cells, self.height_cells
            )
            if cell is None:
                continue

            col, row = cell
            idx = row * self.width_cells + col
            grid[idx] = rec.cost
            lethal_cells.append((col, row, rec.cost))

        # Inflation: engel etrafına azalan maliyet uygula
        r = self.inflation_cells
        for (ec, er, base_cost) in lethal_cells:
            # Inflation yalnızca lethal/yüksek maliyetli engeller için yapılır
            if base_cost < COST_BORDER:
                continue

            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    nr, nc = er + dr, ec + dc
                    if not (0 <= nc < self.width_cells and 0 <= nr < self.height_cells):
                        continue

                    dist_cells = math.sqrt(dr * dr + dc * dc)
                    if dist_cells > r:
                        continue

                    if dist_cells < 1e-9:
                        continue  # Merkez zaten atandı

                    # Mesafeye göre doğrusal azalma
                    ratio = 1.0 - (dist_cells / r)
                    inf_cost = int(COST_INFLATION_MAX * ratio)
                    inf_cost = max(1, inf_cost)

                    nidx = nr * self.width_cells + nc
                    if grid[nidx] < inf_cost:
                        grid[nidx] = inf_cost

        return grid

    def _build_occupancy_grid_msg(self, grid: list) -> OccupancyGrid:
        """
        Ham int listesinden nav_msgs/OccupancyGrid mesajı oluşturur.

        OccupancyGrid standardı: data tipi int8 (−128 ile 127 arası),
        -1 = bilinmeyen, 0 = boş, 100 = tam dolu.
        """
        msg = OccupancyGrid()

        # Header
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = COSTMAP_FRAME_ID

        # Harita meta verisi
        msg.info.resolution = self.resolution
        msg.info.width      = self.width_cells
        msg.info.height     = self.height_cells

        # Harita orijini: araç haritanın ortasında olacak şekilde sol-alt köşe
        half_w = (self.width_cells  * self.resolution) / 2.0
        half_h = (self.height_cells * self.resolution) / 2.0

        origin = Pose()
        origin.position.x    = -half_w
        origin.position.y    = -half_h
        origin.position.z    = 0.0
        origin.orientation.w = 1.0
        msg.info.origin = origin

        # Grid verisi — OccupancyGrid int8 ister, Python int listesi uyumlu
        msg.data = grid

        return msg

    def _build_info_msg(self, active_count: int) -> String:
        """Costmap meta verisini JSON olarak yayınlar."""
        now_stamp = self.get_clock().now().to_msg()
        payload = {
            'stamp':           now_stamp.sec + now_stamp.nanosec * 1e-9,
            'resolution':      self.resolution,
            'width_cells':     self.width_cells,
            'height_cells':    self.height_cells,
            'map_width_m':     self.width_cells  * self.resolution,
            'map_height_m':    self.height_cells * self.resolution,
            'inflation_radius': self.inflation_radius,
            'decay_time':      self.decay_time,
            'active_obstacles': active_count,
            'vehicle_lat':     self._vehicle_lat,
            'vehicle_lon':     self._vehicle_lon,
            'vehicle_yaw_deg': math.degrees(self._vehicle_yaw),
            'gps_ready':       self._gps_ready,
            'imu_ready':       self._imu_ready,
            'simulate_mode':   self.simulate_mode,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        return msg

    def _build_marker_array(self) -> MarkerArray:
        """
        Aktif engeller için RViz2 görselleştirme marker'ları oluşturur.
        Her duba türü için ayrı renk kullanılır.
        """
        marker_array = MarkerArray()
        now_stamp    = self.get_clock().now().to_msg()

        # Araç ENU konumu
        if self._origin_lat is not None:
            vehicle_east, vehicle_north = latlon_to_local_enu(
                self._vehicle_lat, self._vehicle_lon,
                self._origin_lat, self._origin_lon
            )
        else:
            vehicle_east, vehicle_north = 0.0, 0.0

        for rec in self._obstacles.values():
            # Engeli araç-merkezli koordinata çevir (RViz araç frame'inde)
            rel_east  = rec.east_m  - vehicle_east
            rel_north = rec.north_m - vehicle_north

            cos_yaw = math.cos(-self._vehicle_yaw)
            sin_yaw = math.sin(-self._vehicle_yaw)
            local_x =  rel_east * cos_yaw - rel_north * sin_yaw
            local_y =  rel_east * sin_yaw + rel_north * cos_yaw

            m = Marker()
            m.header.stamp    = now_stamp
            m.header.frame_id = COSTMAP_FRAME_ID
            m.ns              = 'costmap_obstacles'
            m.id              = rec.marker_id
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD

            m.pose.position.x    = local_x
            m.pose.position.y    = local_y
            m.pose.position.z    = 0.15
            m.pose.orientation.w = 1.0

            m.scale.x = self.buoy_diameter_m
            m.scale.y = self.buoy_diameter_m
            m.scale.z = 0.3

            r, g, b, a = BUOY_MARKER_COLOR.get(rec.buoy_type, (0.5, 0.5, 0.5, 0.7))
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a

            # Marker ömrü: decay_time'dan biraz daha uzun
            lifetime_sec = int(self.decay_time) + 1
            m.lifetime = Duration(sec=lifetime_sec)

            marker_array.markers.append(m)

        # Artık aktif olmayan marker'ları sil (DELETE_ALL yerine tek tek)
        # Basit yaklaşım: timer her döngüde tüm marker'ları yeniden yazar

        return marker_array

    # ====================================================================== #
    # Ana Timer Callback
    # ====================================================================== #

    def _timer_callback(self):
        """
        Ana döngü: her publish_rate periyodunda çalışır.
        1. Simülasyon modunda sahte engelleri güncelle.
        2. Süresi dolan engelleri haritadan sil (decay).
        3. Grid hesapla.
        4. OccupancyGrid, meta veri ve marker'ları yayınla.
        """
        # Simülasyon güncellemesi
        if self.simulate_mode:
            self._update_simulated_obstacles()

        # Sensörler hazır değilse yayın yapma
        if not (self._gps_ready and self._imu_ready):
            self.get_logger().warn(
                'GPS veya IMU verisi henüz gelmedi — costmap bekleniyor...',
                throttle_duration_sec=5.0
            )
            return

        # Süresi dolan engelleri temizle
        self._decay_obstacles()

        # Grid oluştur ve yayınla
        grid = self._build_grid()
        active_count = len(self._obstacles)

        self._grid_pub.publish(self._build_occupancy_grid_msg(grid))
        self._info_pub.publish(self._build_info_msg(active_count))
        self._markers_pub.publish(self._build_marker_array())

        self.get_logger().debug(
            f'Costmap yayınlandı — aktif engel: {active_count}'
        )


# =============================================================================
# Giriş Noktası
# =============================================================================

def main(args=None):
    """
    Node başlangıç fonksiyonu.
    Çağrı: ros2 run albatros_system costmap_node
    """
    rclpy.init(args=args)
    node = CostmapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Costmap Node durduruldu (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
