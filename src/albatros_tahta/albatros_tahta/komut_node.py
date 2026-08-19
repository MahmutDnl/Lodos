#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros — Komut Node (İlerleme Algoritması)
=====================================================
Parkur 1 ve Parkur 2 için birleşik navigasyon ve ilerleme node'u.

Bu node, aracın yarışma parkurlarını otonom olarak tamamlamasını sağlar:
  - Parkur 1: Saf PID waypoint takibi (engel yok)
  - Parkur 2: VFH engel kaçınma + PID waypoint takibi

Veri Akışı:
  QGroundControl → MAVROS → mission_node → state_node
                                              ↓ (VehicleState)
                                         ┌──────────────┐
  Costmap (P2) ─────────────────────────→│  komut_node  │ → /albatros/command/cmd_vel
                                         └──────────────┘ → /albatros/komut/status

Kullanım:
  1. Yarışma günü QGroundControl üzerinden waypointleri ArduRover'a yükleyin.
  2. Aracı manuel olarak YKİ üzerinden arm edip GUIDED moda alın.
  3. Node otomatik olarak `state_node`'dan gelen heading_error'u takip eder.

Donanım:
  - Trimaran gövde, diferansiyel itki (sol MAIN1, sağ MAIN3)
  - Pixhawk 2.4.8 ArduRover (Boat frame)
  - ESC: Little Bee 30A, PWM 1100-1900, MID 1500

Ortam  : Ubuntu 24.04 / ROS2 Jazzy
Yazar  : LODOS Takımı
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool

from albatros_interfaces.msg import VehicleState


# ═══════════════════════════════════════════════════════════════════════════════
# Sabitler
# ═══════════════════════════════════════════════════════════════════════════════

# Topic tanımları
STATE_TOPIC          = '/albatros/state'
COSTMAP_GRID_TOPIC   = '/albatros/costmap/grid'
COSTMAP_VALID_TOPIC  = '/albatros/costmap/valid'
CMD_VEL_TOPIC        = '/albatros/command/cmd_vel'
KOMUT_STATUS_TOPIC   = '/albatros/komut/status'
OBSTACLE_DETECTED_TOPIC = '/albatros/obstacle/detected'

# Varsayılan parametreler
DEFAULT_PUBLISH_RATE       = 10.0   # Hz
DEFAULT_MAX_LINEAR_SPEED   = 1.0    # m/s
DEFAULT_MIN_LINEAR_SPEED   = 0.2    # m/s
DEFAULT_MAX_ANGULAR_SPEED  = 0.8    # rad/s
DEFAULT_STEERING_KP        = 1.2    # PID P kazancı (dönüş)
DEFAULT_SLOWDOWN_DISTANCE  = 4.0    # metre — yavaşlama başlangıç mesafesi
DEFAULT_STATE_TIMEOUT      = 2.0    # saniye

# VFH varsayılan parametreleri
DEFAULT_SECTOR_COUNT          = 72
DEFAULT_VFH_THRESHOLD         = 0.35
DEFAULT_ACTIVE_REGION_RADIUS  = 6.0    # metre
DEFAULT_VEHICLE_WIDTH         = 0.60   # metre (trimaran genişlik)
DEFAULT_SAFETY_MARGIN         = 0.40   # metre
DEFAULT_COSTMAP_TIMEOUT       = 1.5    # saniye
DEFAULT_EMERGENCY_STOP_DIST   = 0.8    # metre
DEFAULT_COST_GOAL_WEIGHT      = 5.0
DEFAULT_COST_CURRENT_WEIGHT   = 2.0
DEFAULT_COST_PREVIOUS_WEIGHT  = 2.0
DEFAULT_COST_CLEARANCE_WEIGHT = 3.0


def normalize_angle_180(angle_deg):
    """Açıyı [-180°, +180°) aralığına normalize eder."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def clamp(value, min_val, max_val):
    """Değeri [min_val, max_val] aralığına sınırlar."""
    return max(min_val, min(max_val, value))


# ═══════════════════════════════════════════════════════════════════════════════
# KomutNode
# ═══════════════════════════════════════════════════════════════════════════════

class KomutNode(Node):
    """
    Albatros İDA — Birleşik ilerleme algoritması node'u.

    Parkur 1: Saf PID waypoint takibi
    Parkur 2: VFH engel kaçınma + PID waypoint takibi
    """

    def __init__(self):
        super().__init__('komut_node')

        # ─── Parametreler ────────────────────────────────────────────────
        self.declare_parameter('publish_rate',          DEFAULT_PUBLISH_RATE)
        self.declare_parameter('max_linear_speed',      DEFAULT_MAX_LINEAR_SPEED)
        self.declare_parameter('min_linear_speed',      DEFAULT_MIN_LINEAR_SPEED)
        self.declare_parameter('max_angular_speed',     DEFAULT_MAX_ANGULAR_SPEED)
        self.declare_parameter('steering_kp',           DEFAULT_STEERING_KP)
        self.declare_parameter('slowdown_distance_m',   DEFAULT_SLOWDOWN_DISTANCE)
        self.declare_parameter('state_timeout_sec',     DEFAULT_STATE_TIMEOUT)

        # Heading filtre parametreleri (osilasyon önleme)
        self.declare_parameter('heading_deadband_deg',  5.0)   # ±5° altındaki hataları yoksay
        self.declare_parameter('heading_filter_alpha',  0.3)   # EMA yumuşatma (0-1, düşük=daha pürüzsüz)

        # VFH parametreleri
        self.declare_parameter('sector_count',               DEFAULT_SECTOR_COUNT)
        self.declare_parameter('vfh_threshold',              DEFAULT_VFH_THRESHOLD)
        self.declare_parameter('active_region_radius_m',     DEFAULT_ACTIVE_REGION_RADIUS)
        self.declare_parameter('vehicle_width_m',            DEFAULT_VEHICLE_WIDTH)
        self.declare_parameter('safety_margin_m',            DEFAULT_SAFETY_MARGIN)
        self.declare_parameter('costmap_timeout_sec',        DEFAULT_COSTMAP_TIMEOUT)
        self.declare_parameter('emergency_stop_distance_m',  DEFAULT_EMERGENCY_STOP_DIST)
        self.declare_parameter('cost_goal_weight',           DEFAULT_COST_GOAL_WEIGHT)
        self.declare_parameter('cost_current_weight',        DEFAULT_COST_CURRENT_WEIGHT)
        self.declare_parameter('cost_previous_weight',       DEFAULT_COST_PREVIOUS_WEIGHT)
        self.declare_parameter('cost_clearance_weight',      DEFAULT_COST_CLEARANCE_WEIGHT)

        # Engel tespit parametreleri (hibrit AUTO/GUIDED mod geçişi için)
        self.declare_parameter('obstacle_detection_enabled',  True)
        self.declare_parameter('obstacle_forward_cone_deg',   30.0)   # ±15° ön koni

        # Parametre okuma
        self._publish_rate    = float(self.get_parameter('publish_rate').value)
        self._max_linear      = float(self.get_parameter('max_linear_speed').value)
        self._min_linear      = float(self.get_parameter('min_linear_speed').value)
        self._max_angular     = float(self.get_parameter('max_angular_speed').value)
        self._steering_kp     = float(self.get_parameter('steering_kp').value)
        self._slowdown_dist   = float(self.get_parameter('slowdown_distance_m').value)
        self._state_timeout   = float(self.get_parameter('state_timeout_sec').value)

        # Heading filtre parametreleri
        self._heading_deadband = float(self.get_parameter('heading_deadband_deg').value)
        self._heading_alpha    = float(self.get_parameter('heading_filter_alpha').value)

        self._sector_count      = int(self.get_parameter('sector_count').value)
        self._vfh_threshold     = float(self.get_parameter('vfh_threshold').value)
        self._active_radius     = float(self.get_parameter('active_region_radius_m').value)
        self._vehicle_width     = float(self.get_parameter('vehicle_width_m').value)
        self._safety_margin     = float(self.get_parameter('safety_margin_m').value)
        self._costmap_timeout   = float(self.get_parameter('costmap_timeout_sec').value)
        self._emergency_dist    = float(self.get_parameter('emergency_stop_distance_m').value)
        self._weight_goal       = float(self.get_parameter('cost_goal_weight').value)
        self._weight_current    = float(self.get_parameter('cost_current_weight').value)
        self._weight_previous   = float(self.get_parameter('cost_previous_weight').value)
        self._weight_clearance  = float(self.get_parameter('cost_clearance_weight').value)

        # Engel tespit parametreleri
        self._obstacle_detection_enabled = bool(self.get_parameter('obstacle_detection_enabled').value)
        self._obstacle_cone_deg = float(self.get_parameter('obstacle_forward_cone_deg').value)

        self._sector_width = 360.0 / self._sector_count

        # ─── VehicleState Durumu ─────────────────────────────────────────
        self._state: VehicleState = None
        self._last_state_time = None

        # ─── Costmap (Parkur 2 için) ─────────────────────────────────────
        self._costmap: OccupancyGrid = None
        self._last_costmap_time = None
        self._costmap_valid = False

        # ─── VFH Durumu ──────────────────────────────────────────────────
        self._prev_selected_sector = 0

        # ─── Heading Error Filtre Durumu ─────────────────────────────────
        # EMA (Exponential Moving Average) ile IMU gürültüsünü yumuşatır
        self._filtered_heading_error = 0.0
        self._filtered_vfh_angle = 0.0

        # ─── İstatistikler ───────────────────────────────────────────────
        self._total_commands = 0
        self._total_stops    = 0
        self._last_status_time = 0.0

        # ─── Engel Tespit Durumu ─────────────────────────────────────
        self._obstacle_detected = False

        # ─── QoS ────────────────────────────────────────────────────────
        default_qos = QoSProfile(depth=10)

        # ─── Subscriber'lar ──────────────────────────────────────────────
        self.create_subscription(
            VehicleState, STATE_TOPIC,
            self._cb_state, default_qos,
        )

        self.create_subscription(
            OccupancyGrid, COSTMAP_GRID_TOPIC,
            self._cb_costmap, default_qos,
        )

        self.create_subscription(
            Bool, COSTMAP_VALID_TOPIC,
            self._cb_costmap_valid, default_qos,
        )

        # ─── Publisher'lar ───────────────────────────────────────────────
        self._pub_cmd_vel = self.create_publisher(
            Twist, CMD_VEL_TOPIC, default_qos,
        )

        self._pub_status = self.create_publisher(
            String, KOMUT_STATUS_TOPIC, default_qos,
        )

        self._pub_obstacle = self.create_publisher(
            Bool, OBSTACLE_DETECTED_TOPIC, default_qos,
        )

        # ─── Timer ──────────────────────────────────────────────────────
        period = 1.0 / max(self._publish_rate, 0.1)
        self._timer = self.create_timer(period, self._timer_callback)

        # ─── Başlangıç Logları ───────────────────────────────────────────
        sep = '=' * 64
        self.get_logger().info(sep)
        self.get_logger().info('  LODOS Albatros — Komut Node Başlatıldı')
        self.get_logger().info(sep)
        self.get_logger().info('  QGroundControl MAVROS Entegrasyonu AKTİF')
        self.get_logger().info(f'  Maks hız           : {self._max_linear} m/s')
        self.get_logger().info(f'  Maks dönüş hızı    : {self._max_angular} rad/s')
        self.get_logger().info(f'  Steering Kp        : {self._steering_kp}')
        self.get_logger().info(f'  VFH sektör sayısı  : {self._sector_count}')
        self.get_logger().info(f'  Araç genişliği     : {self._vehicle_width} m')
        self.get_logger().info(f'  Yayın frekansı     : {self._publish_rate} Hz')
        self.get_logger().info(sep)

    # ═════════════════════════════════════════════════════════════════════
    # Callback'ler
    # ═════════════════════════════════════════════════════════════════════

    def _cb_state(self, msg: VehicleState):
        """
        VehicleState callback'i.
        Mevcut hedef uzaklığı, heading_error ve parkur numarasını içerir.
        mission_node tarafından beslenir.
        """
        self._state = msg
        self._last_state_time = self.get_clock().now()

    def _cb_costmap(self, msg: OccupancyGrid):
        """Costmap callback'i — Parkur 2 VFH için."""
        self._costmap = msg
        self._last_costmap_time = self.get_clock().now()

    def _cb_costmap_valid(self, msg: Bool):
        """Costmap geçerlilik callback'i."""
        self._costmap_valid = msg.data

    # ═════════════════════════════════════════════════════════════════════
    # Ana Timer — Kontrol Döngüsü
    # ═════════════════════════════════════════════════════════════════════

    def _timer_callback(self):
        """
        Ana kontrol döngüsü (~10 Hz).

        Akış:
          1. Güvenlik ve veri kontrolleri
          2. Engel tespit kontrolü → /albatros/obstacle/detected yayınla
          3. AUTO modda → sadece izle, cmd_vel üretme
          4. GUIDED modda → P1 veya P2 navigasyon → Hız komutu
        """

        # ── Adım 1: Güvenlik kontrolü ───────────────────────────────────
        safe, reason = self._check_safety()
        if not safe:
            self._publish_stop()
            self._publish_obstacle(False)
            self._publish_status(active=False, reason=reason)
            return

        # ── Adım 2: Görev durumu kontrolü ───────────────────────────────
        if self._state.mission_completed:
            self._publish_stop()
            self._publish_obstacle(False)
            self._publish_status(active=False, reason='GOREV_TAMAMLANDI')
            return

        # mission_node tarafından bir hedef belirtilmiş mi?
        if not self._state.target_valid:
            self._publish_stop()
            self._publish_obstacle(False)
            self._publish_status(active=False, reason='GECERLI_HEDEF_BEKLENIYOR')
            return

        # ── Adım 3: Engel tespit kontrolü (her modda çalışır) ───────
        if self._obstacle_detection_enabled:
            obstacle_ahead = self._check_obstacle_ahead()
            self._publish_obstacle(obstacle_ahead)
        else:
            self._publish_obstacle(False)

        # ── Adım 4: Mod kontrolü — AUTO'dayken cmd_vel üretme ─────
        current_mode = self._state.mode.upper() if hasattr(self._state, 'mode') else 'UNKNOWN'

        if current_mode == 'AUTO':
            # AUTO modda Pixhawk kendi navigasyonunu yapıyor.
            # Sadece engel izleme aktif, motor komutu üretilmez.
            self._publish_status(
                active=False,
                reason='AUTO_MOD_IZLEME',
                linear=0.0,
                angular=0.0,
            )
            return

        # ── Adım 5: GUIDED modda → Parkur tespiti ve navigasyon ────
        current_parkur = self._state.current_parkur

        if current_parkur == 1:
            cmd = self._navigate_parkur1()
        else:
            cmd = self._navigate_parkur2()

        # ── Adım 6: Hız komutu yayınlama ────────────────────────────────
        if cmd is not None:
            self._pub_cmd_vel.publish(cmd)
            self._total_commands += 1

        self._publish_status(
            active=True,
            reason=f'P{current_parkur}_NAVIGASYON',
            linear=cmd.linear.x if cmd else 0.0,
            angular=cmd.angular.z if cmd else 0.0,
        )

    # ═════════════════════════════════════════════════════════════════════
    # Parkur 1 — Saf PID Waypoint Takibi
    # ═════════════════════════════════════════════════════════════════════

    def _navigate_parkur1(self) -> Twist:
        """
        Parkur 1: Engel yok, saf PID ile waypoint'e git.

        Dönüş: state_node'dan gelen heading_error'a orantılı angular.z
        İleri: hedefe uzaklığa ve dönüş açısına göre linear.x

        NOT: heading_error_deg pozitif = hedef sağda (CW dönüş gerekli)
             ArduRover body-frame: angular.z pozitif = CCW (sola dönüş)
             Bu yüzden işaret ters çevriliyor.

        Osilasyon önleme:
          1. Deadband: ±heading_deadband_deg içindeki hatalar yoksayılır
          2. EMA filtre: IMU gürültüsü yumuşatılır
        """
        cmd = Twist()
        dist = self._state.distance_to_target_m
        err_deg = self._state.heading_error_deg

        # ── Deadband: küçük heading hatalarını yoksay ─────────────────
        if abs(err_deg) < self._heading_deadband:
            err_deg = 0.0

        # ── EMA filtre: IMU gürültüsünü yumuşat ──────────────────────
        self._filtered_heading_error = (
            self._heading_alpha * err_deg
            + (1.0 - self._heading_alpha) * self._filtered_heading_error
        )
        err_deg = self._filtered_heading_error

        # ── Dönüş komutu (P kontrolcüsü) ─────────────────────────────
        # heading_error pozitif → hedef sağda → CW dönüş → angular.z negatif
        err_rad = math.radians(err_deg)
        angular_z = self._steering_kp * err_rad
        angular_z = clamp(angular_z, -self._max_angular, self._max_angular)
        cmd.angular.z = -angular_z  # İşaret ters: heading_error → ArduRover konvansiyonu

        # ── İleri hız ─────────────────────────────────────────────────
        # Hedefe yaklaştıkça yavaşla
        if dist >= self._slowdown_dist:
            dist_factor = 1.0
        elif dist <= 0.01:
            dist_factor = 0.0
        else:
            dist_factor = dist / self._slowdown_dist

        # Büyük açı hatalarında yavaşla (önce dön, sonra git)
        abs_error = abs(err_deg)
        if abs_error <= 15.0:
            turn_factor = 1.0
        elif abs_error >= 90.0:
            turn_factor = 0.2
        else:
            turn_factor = 1.0 - 0.8 * ((abs_error - 15.0) / 75.0)

        combined_factor = min(dist_factor, turn_factor)
        linear_x = self._min_linear + (self._max_linear - self._min_linear) * combined_factor
        cmd.linear.x = linear_x

        return cmd

    # ═════════════════════════════════════════════════════════════════════
    # Parkur 2 — VFH Engel Kaçınma + PID
    # ═════════════════════════════════════════════════════════════════════

    def _navigate_parkur2(self) -> Twist:
        """
        Parkur 2: VFH ile engelden kaçınarak waypoint'e git.

        Costmap yoksa veya geçersizse Parkur 1 moduna düşer (fallback).
        """
        # Costmap kontrolü — yoksa P1 moduna düş
        if not self._has_valid_costmap():
            self.get_logger().debug(
                'P2: Costmap yok/geçersiz, P1 moduna düşülüyor.'
            )
            return self._navigate_parkur1()

        # ── VFH Pipeline ─────────────────────────────────────────────
        # Adım 1: Polar histogram oluştur
        histogram, min_dist_per_sector = self._build_polar_histogram()

        # Adım 2: Valley'leri bul
        valleys = self._find_valleys(histogram)

        # Adım 3: Dar valley'leri filtrele
        valid_valleys = self._filter_narrow_valleys(valleys, min_dist_per_sector)

        # Adım 4: Hedef yönü sektöre çevir (state_node'dan gelen ideal hedef açısı)
        goal_sector = self._angle_to_sector(self._state.heading_error_deg)

        # Valley yoksa → acil durdurma
        if not valid_valleys:
            self.get_logger().warn('P2 VFH: Geçerli valley bulunamadı! Durduruluyor.')
            self._total_stops += 1
            return Twist()  # Sıfır hız

        # Adım 5: En iyi sektörü seç
        selected_sector = self._select_best_sector(
            valid_valleys, goal_sector, min_dist_per_sector
        )
        selected_angle_deg = self._sector_to_angle(selected_sector)

        # Adım 6: En yakın engel mesafesi
        nearest_obs = self._find_nearest_obstacle_in_cone(
            selected_sector, min_dist_per_sector
        )

        # Acil durum mesafesi kontrolü
        if nearest_obs < self._emergency_dist:
            self.get_logger().warn(
                f'P2 VFH: Acil durum! Engel {nearest_obs:.2f}m mesafede!'
            )
            self._total_stops += 1
            return Twist()

        # Adım 7: Hız hesapla
        cmd = Twist()
        linear_speed, angular_speed = self._calculate_vfh_speeds(
            selected_angle_deg, nearest_obs
        )
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        self._prev_selected_sector = selected_sector
        return cmd

    def _has_valid_costmap(self) -> bool:
        """Geçerli ve taze costmap var mı kontrol eder."""
        if self._costmap is None or not self._costmap_valid:
            return False
        if self._last_costmap_time is None:
            return False
        age = (self.get_clock().now() - self._last_costmap_time).nanoseconds / 1e9
        return age < self._costmap_timeout

    def _check_obstacle_ahead(self) -> bool:
        """Costmap'te aracın önünde tehlikeli engel var mı kontrol eder.

        VFH polar histogramını kullanarak ön koni (obstacle_forward_cone_deg)
        içinde engel yoğunluğu eşiği aşılmışsa True döner.

        Bu sinyal kontrol_node'un AUTO→GUIDED mod geçişini tetiklemesi
        için kullanılır. AUTO modda bile sürekli çalışır.

        Returns:
            True: Önde engel var, GUIDED moda geçilmeli.
            False: Yol temiz, AUTO modda devam edilebilir.
        """
        if not self._has_valid_costmap():
            return False

        histogram, min_dist = self._build_polar_histogram()

        # Ön koni sektörlerini kontrol et
        cone_half_sectors = max(1, int(self._obstacle_cone_deg / (2.0 * self._sector_width)))

        for offset in range(-cone_half_sectors, cone_half_sectors + 1):
            sector = offset % self._sector_count

            if histogram[sector] >= self._vfh_threshold:
                # Bu sektörde engel yoğunluğu eşiği aşıldı
                obs_dist = min_dist[sector]
                if obs_dist < self._active_radius:
                    self.get_logger().debug(
                        f'Engel tespit: sektör={sector}, '
                        f'yoğunluk={histogram[sector]:.2f}, '
                        f'mesafe={obs_dist:.1f}m'
                    )
                    return True

        return False

    # ═════════════════════════════════════════════════════════════════════
    # VFH Algoritması
    # ═════════════════════════════════════════════════════════════════════

    def _build_polar_histogram(self):
        """
        Costmap'ten polar histogram oluşturur.

        Her sektör (360°/sector_count genişliğinde) için engel yoğunluğu
        hesaplar. Engellere yakın sektörler yüksek değer alır.
        """
        histogram = [0.0] * self._sector_count
        min_dist = [self._active_radius] * self._sector_count

        costmap = self._costmap
        res = costmap.info.resolution
        w   = costmap.info.width
        h   = costmap.info.height
        ox  = costmap.info.origin.position.x
        oy  = costmap.info.origin.position.y
        data = costmap.data

        if not data:
            return histogram, min_dist

        for r in range(h):
            for c in range(w):
                cell_val = data[r * w + c]

                if cell_val <= 0:
                    continue

                cost_factor = cell_val / 100.0

                x_m = ox + (c + 0.5) * res
                y_m = oy + (r + 0.5) * res

                dist = math.hypot(x_m, y_m)
                if dist < 0.05 or dist > self._active_radius:
                    continue

                angle_deg = math.degrees(math.atan2(y_m, x_m))
                sector = self._angle_to_sector(angle_deg)

                if dist < min_dist[sector]:
                    min_dist[sector] = dist

                c_w = cost_factor * cost_factor
                d_w = (self._active_radius - dist) / self._active_radius
                d_w = max(0.0, d_w * d_w)

                histogram[sector] += c_w * d_w

        # Normalizasyon
        max_val = max(histogram) if histogram else 0.0
        if max_val > 0.0:
            histogram = [h_val / max_val for h_val in histogram]

        return histogram, min_dist

    def _find_valleys(self, histogram):
        """
        Polar histogramdaki valley'leri (düşük yoğunluklu bölgeleri) bulur.
        Dairesel sarma kontrolü yapar.
        """
        valleys = []
        in_valley = False
        start = 0

        for i in range(self._sector_count):
            if histogram[i] < self._vfh_threshold:
                if not in_valley:
                    in_valley = True
                    start = i
            else:
                if in_valley:
                    in_valley = False
                    valleys.append((start, i - 1))

        if in_valley:
            valleys.append((start, self._sector_count - 1))

        if len(valleys) > 1:
            first_start, first_end = valleys[0]
            last_start, last_end = valleys[-1]
            if last_end == self._sector_count - 1 and first_start == 0:
                merged = (last_start, first_end)
                valleys = valleys[1:-1]
                valleys.append(merged)

        return valleys

    def _filter_narrow_valleys(self, valleys, min_dist_per_sector):
        """
        Araç genişliğinden dar valley'leri filtreler.
        """
        valid = []
        required_width = self._vehicle_width + 2.0 * self._safety_margin

        for v_start, v_end in valleys:
            if v_start <= v_end:
                span = v_end - v_start + 1
            else:
                span = (self._sector_count - v_start) + v_end + 1

            valley_min = self._active_radius
            curr = v_start
            for _ in range(span):
                if min_dist_per_sector[curr] < valley_min:
                    valley_min = min_dist_per_sector[curr]
                curr = (curr + 1) % self._sector_count

            chord = 2.0 * valley_min * math.sin(
                math.radians(span * self._sector_width / 2.0)
            )

            if chord >= required_width:
                valid.append((v_start, v_end))

        return valid

    def _select_best_sector(self, valid_valleys, goal_sector, min_dist_per_sector):
        """
        Geçerli valley'ler arasından en iyi sektörü seçer.
        """
        candidates = []

        for v_start, v_end in valid_valleys:
            if v_start <= v_end:
                sectors = list(range(v_start, v_end + 1))
            else:
                sectors = (
                    list(range(v_start, self._sector_count))
                    + list(range(0, v_end + 1))
                )

            mid_idx = len(sectors) // 2
            candidates.append(sectors[mid_idx])

            if goal_sector in sectors:
                candidates.append(goal_sector)
            else:
                candidates.append(sectors[0])
                candidates.append(sectors[-1])

        best_sector = candidates[0]
        best_cost = float('inf')

        for sector in candidates:
            cost_goal = self._sector_distance(sector, goal_sector)
            cost_current = self._sector_distance(sector, 0)  # 0=ileri
            cost_previous = self._sector_distance(sector, self._prev_selected_sector)

            clearance_dist = min_dist_per_sector[sector]
            cost_clearance = max(0.0, self._active_radius - clearance_dist)

            total = (
                self._weight_goal * cost_goal
                + self._weight_current * cost_current
                + self._weight_previous * cost_previous
                + self._weight_clearance * cost_clearance
            )

            if total < best_cost:
                best_cost = total
                best_sector = sector

        return best_sector

    def _calculate_vfh_speeds(self, selected_angle_deg, nearest_obs_dist):
        """
        VFH tarafından seçilen yöne göre hız ve dönüş komutu hesaplar.

        NOT: angular işareti ters çevriliyor — heading_error konvansiyonu
             ile ArduRover body-frame konvansiyonu arasındaki fark için.

        Osilasyon önleme:
          1. Deadband: küçük açı hataları yoksayılır
          2. EMA filtre: VFH seçim gürültüsü yumuşatılır
        """
        angle_error_deg = normalize_angle_180(selected_angle_deg)

        # ── Deadband: küçük açı hatalarını yoksay ─────────────────────
        if abs(angle_error_deg) < self._heading_deadband:
            angle_error_deg = 0.0

        # ── EMA filtre: VFH seçim gürültüsünü yumuşat ────────────────
        self._filtered_vfh_angle = (
            self._heading_alpha * angle_error_deg
            + (1.0 - self._heading_alpha) * self._filtered_vfh_angle
        )
        angle_error_deg = self._filtered_vfh_angle

        angle_error_rad = math.radians(angle_error_deg)

        angular = self._steering_kp * angle_error_rad
        angular = clamp(angular, -self._max_angular, self._max_angular)
        angular = -angular  # İşaret ters: heading_error → ArduRover konvansiyonu

        if nearest_obs_dist <= 0.01:
            dist_factor = 0.0
        elif nearest_obs_dist >= self._slowdown_dist:
            dist_factor = 1.0
        else:
            dist_factor = nearest_obs_dist / self._slowdown_dist

        abs_error = abs(angle_error_deg)
        if abs_error <= 15.0:
            turn_factor = 1.0
        elif abs_error >= 90.0:
            turn_factor = 0.2
        else:
            turn_factor = 1.0 - 0.8 * ((abs_error - 15.0) / 75.0)

        combined = min(dist_factor, turn_factor)
        linear = self._min_linear + (self._max_linear - self._min_linear) * combined

        return linear, angular

    def _find_nearest_obstacle_in_cone(self, selected_sector, min_dist_per_sector):
        """Seçilen yön etrafındaki ±10° konideki en yakın engeli bulur."""
        nearest = self._active_radius
        cone_half = 2  # ±2 sektör (~±10°)
        for offset in range(-cone_half, cone_half + 1):
            idx = (selected_sector + offset) % self._sector_count
            if min_dist_per_sector[idx] < nearest:
                nearest = min_dist_per_sector[idx]
        return nearest

    # ═════════════════════════════════════════════════════════════════════
    # Güvenlik Kontrolü
    # ═════════════════════════════════════════════════════════════════════

    def _check_safety(self):
        """
        Navigasyona başlamadan önce tüm güvenlik koşullarını kontrol eder.
        """
        now = self.get_clock().now()

        # VehicleState kontrolü
        if self._state is None:
            return False, 'STATE_YOK'

        if self._last_state_time is not None:
            state_age = (now - self._last_state_time).nanoseconds / 1e9
            if state_age > self._state_timeout:
                return False, f'STATE_ESKI ({state_age:.1f}s)'

        state = self._state

        if state.emergency_stop:
            return False, 'ACIL_DURDURMA'

        if not state.mavros_connected:
            return False, 'MAVROS_BAGLI_DEGIL'

        if not state.armed:
            return False, 'ARM_DEGIL'

        if state.mode.upper() not in ('GUIDED', 'AUTO'):
            return False, f'MOD_YANLIS ({state.mode})'

        if not state.gps_ok:
            return False, 'GPS_SAGLIK_HATASI'

        if not state.imu_ok:
            return False, 'IMU_SAGLIK_HATASI'

        if not state.control_allowed:
            return False, 'KONTROL_IZNI_YOK'

        # QGroundControl'den görev aktif edilmiş mi?
        if not state.mission_active:
            return False, 'GOREV_AKTIF_DEGIL'

        return True, 'OK'

    # ═════════════════════════════════════════════════════════════════════
    # Yayın Yardımcıları
    # ═════════════════════════════════════════════════════════════════════

    def _publish_stop(self):
        """Sıfır hız komutu gönderir (motorları durdurur)."""
        self._pub_cmd_vel.publish(Twist())
        self._total_stops += 1

    def _publish_obstacle(self, detected: bool):
        """Engel tespit durumunu /albatros/obstacle/detected topic'ine yayınlar."""
        self._obstacle_detected = detected
        msg = Bool()
        msg.data = detected
        self._pub_obstacle.publish(msg)

    def _publish_status(self, active, reason, linear=0.0, angular=0.0):
        """JSON formatında durum bilgisi yayınlar (throttled: 2 Hz)."""
        now = time.time()
        if now - self._last_status_time < 0.5:
            return
        self._last_status_time = now

        status = {
            'active':              active,
            'reason':              reason,
            'parkur':              self._state.current_parkur if self._state else 0,
            'distance_to_target':  round(self._state.distance_to_target_m, 2) if self._state else 0.0,
            'heading_error_deg':   round(self._state.heading_error_deg, 1) if self._state else 0.0,
            'linear_speed':        round(linear, 3),
            'angular_speed':       round(angular, 3),
            'obstacle_detected':   self._obstacle_detected,
            'current_mode':        self._state.mode if self._state and hasattr(self._state, 'mode') else 'UNKNOWN',
            'total_commands':      self._total_commands,
            'total_stops':         self._total_stops,
        }

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._pub_status.publish(msg)

    # ═════════════════════════════════════════════════════════════════════
    # Sektör Yardımcıları
    # ═════════════════════════════════════════════════════════════════════

    def _angle_to_sector(self, angle_deg):
        """Açıyı (derece) sektör indeksine dönüştürür."""
        normalized = angle_deg % 360.0
        return int(normalized / self._sector_width) % self._sector_count

    def _sector_to_angle(self, sector):
        """Sektör indeksini merkez açısına dönüştürür ([-180, 180) aralığı)."""
        angle = (sector + 0.5) * self._sector_width
        return normalize_angle_180(angle)

    def _sector_distance(self, sector_a, sector_b):
        """İki sektör arasındaki en kısa dairesel mesafe."""
        diff = abs(sector_a - sector_b)
        return min(diff, self._sector_count - diff)


# ═══════════════════════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = KomutNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Komut Node durduruldu. '
            f'Komut: {node._total_commands} | Durma: {node._total_stops}'
        )
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
