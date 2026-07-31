#!/usr/bin/env python3
"""
obstacle_avoidance_node.py — LODOS Albatros VFH Engelden Kaçınma Node'u
========================================================================
Costmap (OccupancyGrid) verisi ve hedef yön bilgisini kullanarak
VFH (Vector Field Histogram) algoritmasıyla engelden kaçınma
komutları üretir.

Parkur 2 Senaryosu:
  - Araç GN4'ten GN5'e ilerler.
  - Turuncu sınır dubaları koridoru belirler (costmap'te LETHAL).
  - Sarı engel dubaları koridor içinde dağınık durumdadır (costmap'te LETHAL).
  - Araç bu engellere çarpmadan hedefe ulaşmalıdır.

VFH (Vector Field Histogram) Algoritması:
  1. POLAR HİSTOGRAM: Costmap'i tarayarak araç etrafında her 5° sektördeki
     engel yoğunluğunu hesapla (72 sektör × 360°).
  2. DÜZLEŞTIRME: Komşu sektörler üzerinde kayar ortalama ile gürültü azalt.
  3. İKİLİ HİSTOGRAM: Yoğunluk > eşik → BLOCKED, aksi → FREE.
  4. VALLEY TESPİTİ: Ardışık FREE sektörleri bul; araç genişliğine uygun
     olanları filtrele.
  5. EN İYİ YÖN: Hedef yönüne, mevcut yöne ve önceki seçime en yakın
     valley'i maliyet fonksiyonuyla seç.
  6. HIZ KOMUTU: Seçilen yöne göre angular.z, engel yakınlığına göre
     linear.x hesapla ve /albatros/command/cmd_vel'e yayınla.

Veri Akışı:
  Girişler:
    - /albatros/costmap/grid   (OccupancyGrid) ← costmap_node
    - /albatros/costmap/valid  (Bool)           ← costmap_node
    - /albatros/mission/target (MissionTarget)  ← mission_node
    - /albatros/imu/data       (Imu)            ← imu_sensor_node
  Çıkışlar:
    - /albatros/command/cmd_vel     (Twist)  → control_node
    - /albatros/avoidance/status    (String) → Debug/YKİ (JSON)

Koordinat Konvansiyonu:
  Costmap (base_link):  +x = ileri (col artar), +y = sol (row artar)
  VFH açı:              0° = ileri, +90° = sol, -90° (270°) = sağ  (CCW pozitif)
  Pusula (compass):     0° = Kuzey, +90° = Doğu (CW pozitif)
  Dönüşüm:              body_angle = -(compass_heading_error)

Yazar : LODOS Takımı
Araç  : Albatros İDA
"""

import math
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from albatros_interfaces.msg import MissionTarget

# ── Topic Tanımları ──────────────────────────────────────────────────────────
COSTMAP_GRID_TOPIC   = '/albatros/costmap/grid'
COSTMAP_VALID_TOPIC  = '/albatros/costmap/valid'
MISSION_TARGET_TOPIC = '/albatros/mission/target'
IMU_TOPIC            = '/albatros/imu/data'

CMD_VEL_TOPIC        = '/albatros/command/cmd_vel'
AVOIDANCE_STATUS_TOPIC = '/albatros/avoidance/status'

# ── Varsayılan Parametreler ──────────────────────────────────────────────────
DEFAULT_SECTOR_COUNT          = 72       # 360°/72 = 5° sektör genişliği
DEFAULT_VFH_THRESHOLD         = 0.3      # Binary histogram eşiği (normalize)
DEFAULT_ACTIVE_REGION_RADIUS  = 6.0      # VFH tarama yarıçapı (m)
DEFAULT_VEHICLE_WIDTH         = 0.85     # Araç genişliği (m)
DEFAULT_SAFETY_MARGIN         = 0.5      # Ek güvenlik marjı (m)
DEFAULT_MAX_LINEAR_SPEED      = 1.0      # Maks ileri hız (m/s)
DEFAULT_MIN_LINEAR_SPEED      = 0.2      # Min ileri hız (m/s)
DEFAULT_MAX_ANGULAR_SPEED     = 0.8      # Maks dönüş hızı (rad/s)
DEFAULT_COST_GOAL_WEIGHT      = 5.0      # μ₁: Hedef yönüne yakınlık ağırlığı
DEFAULT_COST_CURRENT_WEIGHT   = 2.0      # μ₂: Mevcut heading ağırlığı
DEFAULT_COST_PREVIOUS_WEIGHT  = 2.0      # μ₃: Önceki seçim ağırlığı
DEFAULT_SLOWDOWN_DISTANCE     = 2.0      # Engel yakınlaşınca yavaşlama mesafesi (m)
DEFAULT_PUBLISH_RATE          = 10.0     # Komut yayın frekansı (Hz)
DEFAULT_COSTMAP_TIMEOUT       = 2.0      # Costmap timeout (s)
DEFAULT_IMU_TIMEOUT           = 2.0      # IMU timeout (s)
DEFAULT_TARGET_TIMEOUT        = 3.0      # Hedef timeout (s)
DEFAULT_SMOOTHING_WINDOW      = 5        # Histogram düzleştirme penceresi (sektör)
DEFAULT_STEERING_KP           = 1.5      # PID oransal kazanç (yön kontrolü)

# ── Costmap Maliyet Sabitleri ────────────────────────────────────────────────
COST_UNKNOWN  = -1
COST_FREE     = 0
COST_LETHAL   = 100


class ObstacleAvoidanceNode(Node):
    """
    VFH (Vector Field Histogram) tabanlı engelden kaçınma ROS2 node'u.

    Costmap, hedef bilgisi ve IMU verisini kullanarak araç etrafındaki
    engel yoğunluğunu polar histogram olarak hesaplar. Engelsiz yönler
    (valley) arasından hedefe en yakın olanı seçerek hız komutu üretir.
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # ─── Parametre Tanımları ─────────────────────────────────────────
        self.declare_parameter('sector_count',          DEFAULT_SECTOR_COUNT)
        self.declare_parameter('vfh_threshold',         DEFAULT_VFH_THRESHOLD)
        self.declare_parameter('active_region_radius_m', DEFAULT_ACTIVE_REGION_RADIUS)
        self.declare_parameter('vehicle_width_m',       DEFAULT_VEHICLE_WIDTH)
        self.declare_parameter('safety_margin_m',       DEFAULT_SAFETY_MARGIN)
        self.declare_parameter('max_linear_speed',      DEFAULT_MAX_LINEAR_SPEED)
        self.declare_parameter('min_linear_speed',      DEFAULT_MIN_LINEAR_SPEED)
        self.declare_parameter('max_angular_speed',     DEFAULT_MAX_ANGULAR_SPEED)
        self.declare_parameter('cost_goal_weight',      DEFAULT_COST_GOAL_WEIGHT)
        self.declare_parameter('cost_current_weight',   DEFAULT_COST_CURRENT_WEIGHT)
        self.declare_parameter('cost_previous_weight',  DEFAULT_COST_PREVIOUS_WEIGHT)
        self.declare_parameter('slowdown_distance_m',   DEFAULT_SLOWDOWN_DISTANCE)
        self.declare_parameter('publish_rate',          DEFAULT_PUBLISH_RATE)
        self.declare_parameter('costmap_timeout_sec',   DEFAULT_COSTMAP_TIMEOUT)
        self.declare_parameter('imu_timeout_sec',       DEFAULT_IMU_TIMEOUT)
        self.declare_parameter('target_timeout_sec',    DEFAULT_TARGET_TIMEOUT)
        self.declare_parameter('smoothing_window',      DEFAULT_SMOOTHING_WINDOW)
        self.declare_parameter('steering_kp',           DEFAULT_STEERING_KP)

        # ─── Parametre Okuma ─────────────────────────────────────────────
        self._sector_count    = int(self.get_parameter('sector_count').value)
        self._vfh_threshold   = float(self.get_parameter('vfh_threshold').value)
        self._active_radius   = float(self.get_parameter('active_region_radius_m').value)
        self._vehicle_width   = float(self.get_parameter('vehicle_width_m').value)
        self._safety_margin   = float(self.get_parameter('safety_margin_m').value)
        self._max_linear      = float(self.get_parameter('max_linear_speed').value)
        self._min_linear      = float(self.get_parameter('min_linear_speed').value)
        self._max_angular     = float(self.get_parameter('max_angular_speed').value)
        self._mu_goal         = float(self.get_parameter('cost_goal_weight').value)
        self._mu_current      = float(self.get_parameter('cost_current_weight').value)
        self._mu_previous     = float(self.get_parameter('cost_previous_weight').value)
        self._slowdown_dist   = float(self.get_parameter('slowdown_distance_m').value)
        self._publish_rate    = float(self.get_parameter('publish_rate').value)
        self._costmap_timeout = float(self.get_parameter('costmap_timeout_sec').value)
        self._imu_timeout     = float(self.get_parameter('imu_timeout_sec').value)
        self._target_timeout  = float(self.get_parameter('target_timeout_sec').value)
        self._smooth_window   = int(self.get_parameter('smoothing_window').value)
        self._steering_kp     = float(self.get_parameter('steering_kp').value)

        # Türetilmiş sabitler
        self._sector_width = 360.0 / self._sector_count   # Sektör genişliği (°)
        self._total_clearance = self._vehicle_width + 2.0 * self._safety_margin  # Geçiş için gerekli toplam genişlik (m)

        # ─── Dahili Durum Değişkenleri ───────────────────────────────────

        # Costmap verisi
        self._latest_costmap   = None         # OccupancyGrid
        self._costmap_valid    = False
        self._last_costmap_time = None        # rclpy.Time

        # Hedef bilgisi
        self._target_valid       = False
        self._mission_active     = False
        self._target_bearing_deg = 0.0        # Pusula derece
        self._distance_to_target = 0.0        # Metre
        self._last_target_time   = None

        # IMU verisi
        self._imu_valid        = False
        self._current_yaw_deg  = 0.0          # Pusula derece [0, 360)
        self._last_imu_time    = None

        # VFH durum
        self._previous_selected_sector = 0    # Önceki seçilen sektör (ilk = ileri)
        self._histogram         = [0.0] * self._sector_count
        self._binary_histogram  = [0] * self._sector_count
        self._min_obstacle_dist = [self._active_radius] * self._sector_count

        # İstatistikler
        self._total_commands_sent = 0
        self._total_stops         = 0
        self._last_status_time    = 0.0

        # ─── QoS Profilleri ──────────────────────────────────────────────
        default_qos = QoSProfile(depth=10)

        # ─── Subscriber'lar ──────────────────────────────────────────────
        self._sub_costmap = self.create_subscription(
            OccupancyGrid,
            COSTMAP_GRID_TOPIC,
            self._cb_costmap,
            default_qos,
        )

        self._sub_costmap_valid = self.create_subscription(
            Bool,
            COSTMAP_VALID_TOPIC,
            self._cb_costmap_valid,
            default_qos,
        )

        self._sub_target = self.create_subscription(
            MissionTarget,
            MISSION_TARGET_TOPIC,
            self._cb_mission_target,
            default_qos,
        )

        self._sub_imu = self.create_subscription(
            Imu,
            IMU_TOPIC,
            self._cb_imu,
            qos_profile_sensor_data,
        )

        # ─── Publisher'lar ───────────────────────────────────────────────
        self._pub_cmd_vel = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            default_qos,
        )

        self._pub_status = self.create_publisher(
            String,
            AVOIDANCE_STATUS_TOPIC,
            default_qos,
        )

        # ─── Ana Timer ──────────────────────────────────────────────────
        period_sec = 1.0 / max(self._publish_rate, 0.1)
        self._timer = self.create_timer(period_sec, self._timer_callback)

        # ─── Başlatma Bilgi Logları ──────────────────────────────────────
        self.get_logger().info('=' * 64)
        self.get_logger().info('VFH Engelden Kaçınma Node başlatıldı.')
        self.get_logger().info(f'  Sektör sayısı      : {self._sector_count} ({self._sector_width:.1f}°)')
        self.get_logger().info(f'  VFH eşik           : {self._vfh_threshold}')
        self.get_logger().info(f'  Tarama yarıçapı    : {self._active_radius} m')
        self.get_logger().info(f'  Araç genişliği     : {self._vehicle_width} m')
        self.get_logger().info(f'  Güvenlik marjı     : {self._safety_margin} m')
        self.get_logger().info(f'  Toplam açıklık     : {self._total_clearance:.2f} m')
        self.get_logger().info(f'  Maks hız           : {self._max_linear} m/s')
        self.get_logger().info(f'  Maks dönüş hızı    : {self._max_angular} rad/s')
        self.get_logger().info(f'  Yayın frekansı     : {self._publish_rate} Hz')
        self.get_logger().info(f'  Costmap girişi     : {COSTMAP_GRID_TOPIC}')
        self.get_logger().info(f'  Hedef girişi       : {MISSION_TARGET_TOPIC}')
        self.get_logger().info(f'  Komut çıkışı       : {CMD_VEL_TOPIC}')
        self.get_logger().info('=' * 64)

    # =====================================================================
    # Callback'ler
    # =====================================================================

    def _cb_costmap(self, msg: OccupancyGrid):
        """
        /albatros/costmap/grid callback'i.
        OccupancyGrid verisini saklar ve zaman damgasını günceller.
        """
        self._latest_costmap = msg
        self._last_costmap_time = self.get_clock().now()

    def _cb_costmap_valid(self, msg: Bool):
        """
        /albatros/costmap/valid callback'i.
        Costmap'in geçerli olup olmadığını günceller.
        """
        self._costmap_valid = msg.data

    def _cb_mission_target(self, msg: MissionTarget):
        """
        /albatros/mission/target callback'i.
        Hedef bilgisini günceller.
        """
        self._target_valid       = msg.target_valid
        self._mission_active     = msg.mission_active
        self._target_bearing_deg = msg.target_bearing_deg
        self._distance_to_target = msg.distance_to_target_m
        self._last_target_time   = self.get_clock().now()

    def _cb_imu(self, msg: Imu):
        """
        /albatros/imu/data callback'i.
        Quaternion'dan yaw açısını hesaplar.
        """
        yaw_deg = self._quaternion_to_yaw_deg(msg)

        if yaw_deg is not None:
            self._imu_valid       = True
            self._current_yaw_deg = (yaw_deg + 360.0) % 360.0
        else:
            self._imu_valid = False

        self._last_imu_time = self.get_clock().now()

    # =====================================================================
    # Ana Timer Callback — VFH Pipeline
    # =====================================================================

    def _timer_callback(self):
        """
        Ana kontrol döngüsü. VFH pipeline'ını çalıştırır:
        1. Veri tazeliği kontrolü
        2. Polar histogram oluşturma
        3. Düzleştirme + binary histogram
        4. Valley tespiti
        5. En iyi yön seçimi
        6. Hız komutu üretimi ve yayını
        """
        # ── Adım 0: Veri geçerlilik kontrolü ────────────────────────────
        data_ok, reason = self._check_data_validity()

        if not data_ok:
            self._publish_stop()
            self._publish_status(
                active=False,
                reason=reason,
                selected_angle_deg=0.0,
                linear_speed=0.0,
                angular_speed=0.0,
            )
            return

        costmap = self._latest_costmap

        # ── Adım 1: Costmap'ten araç konumunu hesapla ───────────────────
        resolution = costmap.info.resolution
        width      = costmap.info.width
        height     = costmap.info.height
        origin_x   = costmap.info.origin.position.x
        origin_y   = costmap.info.origin.position.y

        # Araç grid pozisyonu: costmap_node araç merkezli grid oluşturur
        vehicle_col = int(round(-origin_x / resolution - 0.5))
        vehicle_row = int(round(-origin_y / resolution - 0.5))

        # ── Adım 2: Hedef yönünü body frame'e dönüştür ──────────────────
        # Pusula: heading_error > 0 → hedef saat yönünde (sağ)
        # Body frame: +angle = CCW = sol
        # Dönüşüm: body_angle = -heading_error
        heading_error_deg = self._normalize_angle_180(
            self._target_bearing_deg - self._current_yaw_deg
        )
        goal_body_deg = -heading_error_deg
        goal_sector   = self._angle_to_sector(goal_body_deg)

        # ── Adım 3: Polar histogram oluştur ──────────────────────────────
        histogram, min_dist_per_sector = self._build_polar_histogram(
            costmap.data, width, height, resolution,
            vehicle_col, vehicle_row,
        )

        # ── Adım 4: Düzleştirme ─────────────────────────────────────────
        smoothed = self._smooth_histogram(histogram)

        # ── Adım 5: Binary histogram ────────────────────────────────────
        binary = self._build_binary_histogram(smoothed)

        # Dahili durumu güncelle (debug için)
        self._histogram        = smoothed
        self._binary_histogram = binary
        self._min_obstacle_dist = min_dist_per_sector

        # ── Adım 6: Valley tespiti ──────────────────────────────────────
        valleys = self._find_valleys(binary)

        # ── Adım 7: Araç genişliğine göre valley filtreleme ─────────────
        valid_valleys = self._filter_valleys_by_width(
            valleys, min_dist_per_sector,
        )

        # ── Adım 8: En iyi yön seçimi ──────────────────────────────────
        if not valid_valleys:
            # Hiçbir geçilebilir valley yok → DUR
            self._publish_stop()
            self._total_stops += 1
            self._publish_status(
                active=True,
                reason='NO_VALLEY_FOUND',
                selected_angle_deg=0.0,
                linear_speed=0.0,
                angular_speed=0.0,
                goal_sector=goal_sector,
                valleys=valleys,
                valid_valleys=valid_valleys,
            )
            self.get_logger().warn(
                'VFH: Geçilebilir valley bulunamadı — araç durduruluyor.',
                throttle_duration_sec=3.0,
            )
            return

        selected_sector = self._select_best_direction(
            valid_valleys, goal_sector,
        )

        self._previous_selected_sector = selected_sector
        selected_angle_deg = self._sector_to_angle(selected_sector)

        # ── Adım 9: Hız komutu hesapla ──────────────────────────────────
        # Engel yakınlığına göre hız ayarı
        nearest_obstacle = self._find_nearest_obstacle_in_cone(
            selected_sector, min_dist_per_sector,
        )

        linear_speed, angular_speed = self._compute_cmd_vel(
            selected_angle_deg, nearest_obstacle,
        )

        # ── Adım 10: Yayınla ────────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x  = linear_speed
        cmd.angular.z = angular_speed
        self._pub_cmd_vel.publish(cmd)
        self._total_commands_sent += 1

        # Debug status
        self._publish_status(
            active=True,
            reason='VFH_ACTIVE',
            selected_angle_deg=selected_angle_deg,
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            goal_sector=goal_sector,
            selected_sector=selected_sector,
            nearest_obstacle=nearest_obstacle,
            valleys=valleys,
            valid_valleys=valid_valleys,
        )

    # =====================================================================
    # VFH Adım 1: Polar Histogram Oluşturma
    # =====================================================================

    def _build_polar_histogram(
        self,
        grid_data,
        width: int,
        height: int,
        resolution: float,
        vehicle_col: int,
        vehicle_row: int,
    ):
        """
        Costmap verisinden polar histogram oluşturur.

        Her sektör için:
        - Engel yoğunluğu (magnitude toplamı) hesaplanır.
        - En yakın engel mesafesi takip edilir.

        Magnitude formülü (VFH standard):
            m = (cost/100)² × (a - d)
        burada:
            cost  = hücre maliyet değeri [0, 100]
            a     = active_region_radius
            d     = hücre mesafesi (m)

        Yakın ve yüksek maliyetli hücreler daha çok katkı sağlar.

        Args:
            grid_data:   OccupancyGrid.data dizisi (row-major).
            width:       Grid genişliği (hücre).
            height:      Grid yüksekliği (hücre).
            resolution:  Hücre çözünürlüğü (m).
            vehicle_col: Araç sütunu.
            vehicle_row: Araç satırı.

        Returns:
            (histogram, min_dist_per_sector) tuple'ı.
            histogram: Her sektörün toplam magnitude değeri.
            min_dist_per_sector: Her sektördeki en yakın engel mesafesi (m).
        """
        histogram     = [0.0] * self._sector_count
        min_dist      = [self._active_radius] * self._sector_count
        active_cells  = int(math.ceil(self._active_radius / resolution))

        for dr in range(-active_cells, active_cells + 1):
            for dc in range(-active_cells, active_cells + 1):
                col = vehicle_col + dc
                row = vehicle_row + dr

                # Sınır kontrolü
                if not (0 <= col < width and 0 <= row < height):
                    continue

                # Hücre maliyetini oku
                idx = row * width + col
                cost = grid_data[idx]

                # Sadece engel hücreleriyle ilgilen (FREE ve UNKNOWN atla)
                if cost <= COST_FREE:
                    continue

                # Mesafe hesabı (metre)
                dx = dc * resolution
                dy = dr * resolution
                dist = math.sqrt(dx * dx + dy * dy)

                # Tarama yarıçapı dışındakileri atla
                if dist > self._active_radius or dist < 0.01:
                    continue

                # Açı hesabı (body frame: 0°=ileri, +90°=sol)
                angle_deg = math.degrees(math.atan2(dy, dx))
                sector = self._angle_to_sector(angle_deg)

                # VFH magnitude: (cost/100)² × (active_radius - dist)
                normalized_cost = cost / 100.0
                decay = self._active_radius - dist
                magnitude = normalized_cost * normalized_cost * decay

                histogram[sector] += magnitude

                # En yakın engel takibi
                if dist < min_dist[sector]:
                    min_dist[sector] = dist

        # Histogramı normalize et: max değeri 1.0 olacak şekilde
        max_val = max(histogram) if histogram else 1.0
        if max_val > 0:
            histogram = [h / max_val for h in histogram]

        return histogram, min_dist

    # =====================================================================
    # VFH Adım 2: Histogram Düzleştirme
    # =====================================================================

    def _smooth_histogram(self, histogram):
        """
        Polar histograma kayar ortalama (moving average) uygular.

        Dairesel histogram olduğu için wrap-around (sektör 0 ↔ N-1)
        dikkate alınır.

        Pencere boyutu: self._smooth_window (varsayılan 5 sektör).

        Args:
            histogram: Ham polar histogram (list of float).

        Returns:
            Düzleştirilmiş histogram (list of float).
        """
        n = len(histogram)
        half_w = self._smooth_window // 2
        smoothed = [0.0] * n

        for k in range(n):
            total = 0.0
            count = 0
            for offset in range(-half_w, half_w + 1):
                idx = (k + offset) % n  # Dairesel wrap-around
                total += histogram[idx]
                count += 1
            smoothed[k] = total / count

        return smoothed

    # =====================================================================
    # VFH Adım 3: Binary Histogram
    # =====================================================================

    def _build_binary_histogram(self, histogram):
        """
        Düzleştirilmiş histogramı binary (0/1) histograma dönüştürür.

        Kural:
            histogram[k] > vfh_threshold → 1 (BLOCKED)
            histogram[k] ≤ vfh_threshold → 0 (FREE)

        Args:
            histogram: Düzleştirilmiş polar histogram.

        Returns:
            Binary histogram (list of int, 0 veya 1).
        """
        return [
            1 if h > self._vfh_threshold else 0
            for h in histogram
        ]

    # =====================================================================
    # VFH Adım 4: Valley Tespiti
    # =====================================================================

    def _find_valleys(self, binary):
        """
        Binary histogramda ardışık FREE (0) sektör gruplarını bulur.

        Dairesel histogram olduğu için wrap-around dikkate alınır.
        Tüm sektörler FREE ise tek bir 360° valley döner.
        Tüm sektörler BLOCKED ise boş liste döner.

        Args:
            binary: Binary histogram (list of int, 0 veya 1).

        Returns:
            Valley listesi. Her valley: (start_sector, end_sector, width_sectors).
            start ve end dahildir; genişlik sektör sayısı cinsindendir.
        """
        n = len(binary)

        # Tüm sektörler FREE mi?
        if all(b == 0 for b in binary):
            return [(0, n - 1, n)]

        # Tüm sektörler BLOCKED mı?
        if all(b == 1 for b in binary):
            return []

        # Wrap-around: diziyi iki katına çıkar ve ardışık FREE grupları bul
        # Sonra tekrarlananları birleştir
        valleys = []
        in_valley = False
        start = -1

        # İlk BLOCKED sektörü bul (başlangıç noktası olarak)
        first_blocked = -1
        for i in range(n):
            if binary[i] == 1:
                first_blocked = i
                break

        # Dairesel tarama: first_blocked'dan başla
        for offset in range(n):
            i = (first_blocked + offset) % n

            if binary[i] == 0:
                if not in_valley:
                    start = i
                    in_valley = True
            else:
                if in_valley:
                    end = (i - 1 + n) % n
                    # Genişlik hesabı (dairesel)
                    if end >= start:
                        width = end - start + 1
                    else:
                        width = (n - start) + end + 1
                    valleys.append((start, end, width))
                    in_valley = False

        # Tarama sonunda hâlâ valley içindeysek kapat
        if in_valley:
            end = (first_blocked - 1 + n) % n
            if end >= start:
                width = end - start + 1
            else:
                width = (n - start) + end + 1
            valleys.append((start, end, width))

        return valleys

    # =====================================================================
    # VFH Adım 5: Valley Genişlik Filtreleme
    # =====================================================================

    def _filter_valleys_by_width(self, valleys, min_dist_per_sector):
        """
        Valley'leri araç genişliğine göre filtreler.

        Bir valley'den geçebilmek için yeterli fiziksel açıklık gerekir.
        Gerekli minimum açısal genişlik, o yöndeki en yakın engel
        mesafesine göre hesaplanır:
            min_angle = 2 × arctan(total_clearance / (2 × obstacle_dist))

        Args:
            valleys:           Valley listesi [(start, end, width), ...].
            min_dist_per_sector: Her sektördeki en yakın engel mesafesi.

        Returns:
            Filtrelenmiş valley listesi.
        """
        valid = []

        for (start, end, width) in valleys:
            # Valley'deki en yakın engel mesafesini bul
            nearest_dist = self._active_radius
            idx = start
            for _ in range(width):
                if min_dist_per_sector[idx] < nearest_dist:
                    nearest_dist = min_dist_per_sector[idx]
                idx = (idx + 1) % self._sector_count

            # Gerekli minimum açısal genişlik (derece)
            if nearest_dist > 0.1:
                min_angle_deg = 2.0 * math.degrees(
                    math.atan2(self._total_clearance / 2.0, nearest_dist)
                )
            else:
                # Çok yakın engel — çok geniş açıklık gerekir
                min_angle_deg = 180.0

            # Valley'nin açısal genişliği
            valley_angle_deg = width * self._sector_width

            if valley_angle_deg >= min_angle_deg:
                valid.append((start, end, width))

        return valid

    # =====================================================================
    # VFH Adım 6: En İyi Yön Seçimi
    # =====================================================================

    def _select_best_direction(self, valleys, goal_sector):
        """
        Geçerli valley'ler arasından en iyi yönü maliyet fonksiyonuyla seçer.

        Her valley için aday yönler oluşturulur:
        - Geniş valley (>= 2× min genişlik): Hedef yönüne en yakın kenar
        - Dar valley: Merkez

        Maliyet fonksiyonu:
            cost = μ₁ × |Δhedef| + μ₂ × |Δmevcut| + μ₃ × |Δönceki|
        burada:
            Δhedef   = Aday ile hedef sektör arasındaki açısal fark
            Δmevcut  = Aday ile ileri yön (sektör 0) arasındaki fark
            Δönceki  = Aday ile önceki seçilen sektör arasındaki fark

        En düşük maliyetli aday seçilir.

        Args:
            valleys:     Filtrelenmiş valley listesi.
            goal_sector: Hedef yönüne karşılık gelen sektör.

        Returns:
            Seçilen sektör indeksi (int).
        """
        best_cost   = float('inf')
        best_sector = 0   # Varsayılan: ileri

        # İleri yön sektörü (body frame 0°)
        forward_sector = self._angle_to_sector(0.0)

        for (start, end, width) in valleys:
            # Aday yönleri belirle
            candidates = self._get_valley_candidates(
                start, end, width, goal_sector,
            )

            for candidate in candidates:
                # Maliyet hesabı
                d_goal    = self._sector_distance(candidate, goal_sector)
                d_current = self._sector_distance(candidate, forward_sector)
                d_prev    = self._sector_distance(candidate, self._previous_selected_sector)

                cost = (
                    self._mu_goal     * d_goal
                    + self._mu_current  * d_current
                    + self._mu_previous * d_prev
                )

                if cost < best_cost:
                    best_cost   = cost
                    best_sector = candidate

        return best_sector

    def _get_valley_candidates(self, start, end, width, goal_sector):
        """
        Bir valley için aday yön sektörlerini üretir.

        Geniş valley'ler:
            - Valley merkezi
            - Hedef yönüne en yakın kenar (start veya end)
            - Hedef yönü valley içindeyse direkt hedef yönü

        Dar valley'ler:
            - Sadece valley merkezi

        Args:
            start:       Valley başlangıç sektörü.
            end:         Valley bitiş sektörü.
            width:       Valley genişliği (sektör sayısı).
            goal_sector: Hedef sektörü.

        Returns:
            Aday sektör listesi.
        """
        candidates = []

        # Valley merkezi
        center = (start + width // 2) % self._sector_count
        candidates.append(center)

        # Geniş valley ise kenar ve hedef adayları ekle
        # "Geniş" = en az 2× sektör genişliği
        min_wide = max(4, int(30.0 / self._sector_width))  # ~30° veya 4 sektör

        if width >= min_wide:
            # Kenar sektörleri (biraz içerden, yarım araç genişliği kadar)
            margin_sectors = max(1, int(width * 0.15))
            left_candidate  = (start + margin_sectors) % self._sector_count
            right_candidate = (end - margin_sectors + self._sector_count) % self._sector_count
            candidates.append(left_candidate)
            candidates.append(right_candidate)

            # Hedef sektörü valley içinde mi?
            if self._is_sector_in_valley(goal_sector, start, width):
                candidates.append(goal_sector)

        return candidates

    def _is_sector_in_valley(self, sector, valley_start, valley_width):
        """
        Bir sektörün valley içinde olup olmadığını kontrol eder.
        Dairesel (wrap-around) yapıyı dikkate alır.
        """
        for i in range(valley_width):
            if (valley_start + i) % self._sector_count == sector:
                return True
        return False

    # =====================================================================
    # VFH Adım 7: Hız Komutu Hesaplama
    # =====================================================================

    def _compute_cmd_vel(self, selected_angle_deg, nearest_obstacle_dist):
        """
        Seçilen VFH yönüne göre linear.x ve angular.z hesaplar.

        Angular hız:
            Seçilen yön ile ileri yön (0°) arasındaki açı farkına
            oransal (P kontrol). Body frame CCW pozitif olduğundan:
            - selected_angle_deg > 0 → sola dön → angular.z > 0
            - selected_angle_deg < 0 → sağa dön → angular.z < 0

        Linear hız:
            Engel yakınlığına göre yavaşlama:
            - Engel uzakta → max_linear_speed
            - Engel yakında → min_linear_speed
            - Keskin dönüşte hız düşürme

        Args:
            selected_angle_deg:    Seçilen yön (body frame, °).
            nearest_obstacle_dist: Seçilen yöndeki en yakın engel (m).

        Returns:
            (linear_speed, angular_speed) tuple'ı.
        """
        # ── Açı farkı: seçilen yön vs ileri ──────────────────────────────
        angle_error_deg = self._normalize_angle_180(selected_angle_deg)
        angle_error_rad = math.radians(angle_error_deg)

        # ── Angular hız (P kontrol) ──────────────────────────────────────
        angular_speed = self._steering_kp * angle_error_rad
        angular_speed = self._clamp(angular_speed, -self._max_angular, self._max_angular)

        # ── Linear hız ───────────────────────────────────────────────────
        # Engel yakınlık faktörü
        if nearest_obstacle_dist <= 0.01:
            dist_factor = 0.0
        elif nearest_obstacle_dist >= self._slowdown_dist:
            dist_factor = 1.0
        else:
            dist_factor = nearest_obstacle_dist / self._slowdown_dist

        # Dönüş açısı faktörü: keskin dönüşlerde yavaşla
        abs_error = abs(angle_error_deg)
        if abs_error <= 15.0:
            turn_factor = 1.0
        elif abs_error >= 90.0:
            turn_factor = 0.3
        else:
            turn_factor = 1.0 - 0.7 * ((abs_error - 15.0) / 75.0)

        # Toplam hız
        combined_factor = min(dist_factor, turn_factor)
        linear_speed = (
            self._min_linear
            + (self._max_linear - self._min_linear) * combined_factor
        )

        return linear_speed, angular_speed

    def _find_nearest_obstacle_in_cone(self, selected_sector, min_dist_per_sector):
        """
        Seçilen sektör ve komşularındaki en yakın engel mesafesini bulur.

        ±2 sektör (±10°) genişliğindeki konide arar.

        Args:
            selected_sector:     Seçilen sektör indeksi.
            min_dist_per_sector: Her sektördeki en yakın engel mesafesi.

        Returns:
            En yakın engel mesafesi (m).
        """
        nearest = self._active_radius
        cone_half = 2  # ±2 sektör (±10°)

        for offset in range(-cone_half, cone_half + 1):
            idx = (selected_sector + offset) % self._sector_count
            if min_dist_per_sector[idx] < nearest:
                nearest = min_dist_per_sector[idx]

        return nearest

    # =====================================================================
    # Veri Geçerlilik Kontrolü
    # =====================================================================

    def _check_data_validity(self):
        """
        Tüm girdi verilerinin tazeliğini ve geçerliliğini kontrol eder.

        Kontrol edilen koşullar:
        1. Costmap verisi alınmış ve taze mi
        2. Costmap geçerli mi (costmap_valid flag)
        3. IMU verisi alınmış ve taze mi
        4. Hedef bilgisi alınmış ve taze mi
        5. Görev aktif mi

        Returns:
            (ok: bool, reason: str) tuple'ı.
        """
        now = self.get_clock().now()

        # Costmap kontrolü
        if self._latest_costmap is None:
            return False, 'COSTMAP_NOT_RECEIVED'

        if self._last_costmap_time is not None:
            costmap_age = (now - self._last_costmap_time).nanoseconds / 1e9
            if costmap_age > self._costmap_timeout:
                return False, f'COSTMAP_STALE ({costmap_age:.1f}s)'

        if not self._costmap_valid:
            return False, 'COSTMAP_INVALID'

        # IMU kontrolü
        if not self._imu_valid:
            return False, 'IMU_INVALID'

        if self._last_imu_time is not None:
            imu_age = (now - self._last_imu_time).nanoseconds / 1e9
            if imu_age > self._imu_timeout:
                return False, f'IMU_STALE ({imu_age:.1f}s)'

        # Hedef kontrolü
        if self._last_target_time is None:
            return False, 'TARGET_NOT_RECEIVED'

        target_age = (now - self._last_target_time).nanoseconds / 1e9
        if target_age > self._target_timeout:
            return False, f'TARGET_STALE ({target_age:.1f}s)'

        if not self._target_valid:
            return False, 'TARGET_INVALID'

        if not self._mission_active:
            return False, 'MISSION_NOT_ACTIVE'

        return True, 'OK'

    # =====================================================================
    # Yayın Yardımcıları
    # =====================================================================

    def _publish_stop(self):
        """Sıfır hız komutu yayınlar (araç dursun)."""
        cmd = Twist()   # Tüm alanlar varsayılan 0.0
        self._pub_cmd_vel.publish(cmd)

    def _publish_status(self, active, reason, selected_angle_deg,
                        linear_speed, angular_speed, **kwargs):
        """
        Debug/YKİ için JSON formatında durum bilgisi yayınlar.

        Yayın sıklığı: saniyede en fazla 2 kez (throttled).
        """
        now = time.time()
        if now - self._last_status_time < 0.5:
            return
        self._last_status_time = now

        status = {
            'active':              active,
            'reason':              reason,
            'selected_angle_deg':  round(selected_angle_deg, 1),
            'linear_speed':        round(linear_speed, 3),
            'angular_speed':       round(angular_speed, 3),
            'goal_bearing_deg':    round(self._target_bearing_deg, 1),
            'current_yaw_deg':     round(self._current_yaw_deg, 1),
            'distance_to_target':  round(self._distance_to_target, 2),
            'total_commands':      self._total_commands_sent,
            'total_stops':         self._total_stops,
        }

        # Ek bilgiler
        if 'goal_sector' in kwargs:
            status['goal_sector'] = kwargs['goal_sector']
        if 'selected_sector' in kwargs:
            status['selected_sector'] = kwargs['selected_sector']
        if 'nearest_obstacle' in kwargs:
            status['nearest_obstacle_m'] = round(kwargs['nearest_obstacle'], 2)
        if 'valleys' in kwargs:
            status['valley_count'] = len(kwargs['valleys'])
        if 'valid_valleys' in kwargs:
            status['valid_valley_count'] = len(kwargs['valid_valleys'])

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._pub_status.publish(msg)

    # =====================================================================
    # Yardımcı Fonksiyonlar
    # =====================================================================

    def _angle_to_sector(self, angle_deg):
        """
        Açıyı (derece) sektör indeksine dönüştürür.

        Açı [0°, 360°) aralığına normalize edilir ve
        sektör genişliğine bölünür.

        Args:
            angle_deg: Body frame açısı (derece, CCW pozitif).

        Returns:
            Sektör indeksi [0, sector_count).
        """
        normalized = angle_deg % 360.0
        sector = int(normalized / self._sector_width) % self._sector_count
        return sector

    def _sector_to_angle(self, sector):
        """
        Sektör indeksini merkez açısına dönüştürür.

        Açı [-180°, +180°) aralığında döndürülür (body frame).

        Args:
            sector: Sektör indeksi.

        Returns:
            Sektör merkez açısı (derece).
        """
        angle = (sector + 0.5) * self._sector_width
        return self._normalize_angle_180(angle)

    def _sector_distance(self, sector_a, sector_b):
        """
        İki sektör arasındaki en kısa dairesel mesafeyi hesaplar.

        Dairesel yapıyı dikkate alır (ör. sektör 0 ile sektör 71
        arasında 1 sektör fark vardır).

        Args:
            sector_a: İlk sektör indeksi.
            sector_b: İkinci sektör indeksi.

        Returns:
            Sektör farkı (pozitif tam sayı).
        """
        diff = abs(sector_a - sector_b)
        return min(diff, self._sector_count - diff)

    @staticmethod
    def _normalize_angle_180(angle_deg):
        """Açıyı [-180°, +180°) aralığına normalize eder."""
        return (angle_deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def _clamp(value, min_val, max_val):
        """Değeri [min_val, max_val] aralığına sınırlar."""
        return max(min_val, min(max_val, value))

    def _quaternion_to_yaw_deg(self, msg: Imu):
        """
        IMU quaternion'dan yaw açısını derece cinsinden hesaplar.

        Args:
            msg: sensor_msgs/Imu mesajı.

        Returns:
            Yaw açısı (derece) veya None (geçersiz quaternion).
        """
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm < 1e-6:
            return None

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(yaw_rad)


# =============================================================================
# Entry Point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'VFH Engelden Kaçınma Node durduruldu. '
            f'Toplam komut: {node._total_commands_sent} | '
            f'Toplam durma: {node._total_stops}'
        )
    finally:
        # Son olarak sıfır hız gönder
        try:
            node._publish_stop()
        except Exception:
            pass

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
