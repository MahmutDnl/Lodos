#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — Gelişmiş Global + Local Costmap Node (v4)
# =============================================================================
# Girişler:
#   - /albatros/gps/fix          [sensor_msgs/NavSatFix]
#   - /albatros/imu/data         [sensor_msgs/Imu]
#   - /albatros/fusion/obstacles [std_msgs/String JSON]
#
# Çıkışlar:
#   - /albatros/costmap/grid              [nav_msgs/OccupancyGrid] (base_link)
#   - /albatros/costmap/global_grid       [nav_msgs/OccupancyGrid] (map)
#   - /albatros/costmap/course_markers    [visualization_msgs/MarkerArray]
#   - /albatros/costmap/course_centerline [nav_msgs/Path] (map)
#   - /albatros/costmap/markers           [visualization_msgs/MarkerArray]
#   - /albatros/costmap/info              [std_msgs/String JSON]
#   - /albatros/costmap/valid             [std_msgs/Bool]
#   - TF transform: map -> base_link
#
# ROS2 Jazzy / Ubuntu 24.04
# =============================================================================

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import NavSatFix, Imu
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import tf2_ros

# ---------------------------------------------------------------------------
# Maliyet Sabitleri
# ---------------------------------------------------------------------------
COST_UNKNOWN       = -1
COST_FREE          =  0
COST_TARGET        = 60
COST_LETHAL        = 100
COST_INFLATION_MAX = 75

TYPE_COST = {
    'obstacle_buoy': COST_LETHAL,
    'border_buoy':   COST_LETHAL,
    'target_buoy':   COST_TARGET,
    'goal_buoy':     COST_FREE,
    'unknown':       COST_LETHAL,
}

NO_INFLATE_TYPES = {'goal_buoy', 'target_buoy'}

# Marker Renkleri (RGBA)
TYPE_COLOR = {
    'obstacle_buoy': (1.0, 1.0, 0.0, 0.90),   # Sarı
    'border_buoy':   (1.0, 0.5, 0.0, 0.90),   # Turuncu
    'target_buoy':   (0.2, 0.9, 0.2, 0.90),   # Yeşil
    'goal_buoy':     (0.2, 0.4, 1.0, 0.90),   # Mavi
    'unknown':       (0.5, 0.5, 0.5, 0.70),   # Gri
}

GPS_TOPIC     = '/albatros/gps/fix'
IMU_TOPIC     = '/albatros/imu/data'
FUSION_TOPIC  = '/albatros/fusion/obstacles'

LOCAL_GRID_TOPIC   = '/albatros/costmap/grid'
GLOBAL_GRID_TOPIC  = '/albatros/costmap/global_grid'
COURSE_MARKERS_TOPIC = '/albatros/costmap/course_markers'
CENTERLINE_TOPIC   = '/albatros/costmap/course_centerline'
MARKERS_TOPIC      = '/albatros/costmap/markers'
INFO_TOPIC         = '/albatros/costmap/info'
VALID_TOPIC        = '/albatros/costmap/valid'

FRAME_GLOBAL = 'map'
FRAME_LOCAL  = 'base_link'


# ---------------------------------------------------------------------------
# Pure Math Dönüşüm Fonksiyonları (Unit Test Edilebilir)
# ---------------------------------------------------------------------------
def latlon_to_local_enu(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """
    Kısa yarışma parkurları için hafif WGS84 -> ENU (East-North) düz-dünya dönüşümü.
    (0,0) noktası lat0, lon0 konumuna karşılık gelir.
    +x = Doğu (East / Araç Başlangıç İleri Eksen)
    +y = Kuzey (North / Araç Başlangıç Sol Eksen)
    """
    R_EARTH = 6371000.0  # metre
    lat_rad = math.radians(lat0)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    
    x = dlon * R_EARTH * math.cos(lat_rad)
    y = dlat * R_EARTH
    return x, y


def local_to_global(local_x: float, local_y: float,
                    vehicle_x: float, vehicle_y: float,
                    yaw: float) -> tuple[float, float]:
    """
    Araca göre local (base_link: +x ileri, +y sol) duba koordinatını
    harita (map) global koordinatına dönüştürür.
    """
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    global_x = vehicle_x + cos_y * local_x - sin_y * local_y
    global_y = vehicle_y + sin_y * local_x + cos_y * local_y
    return global_x, global_y


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """
    Quaternion (x,y,z,w) -> (roll, pitch, yaw) dönüşümü (radyan).
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """
    Euler (roll, pitch, yaw) -> Quaternion (x,y,z,w) dönüşümü.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


def local_xy_to_grid(x_m: float, y_m: float,
                     resolution: float,
                     width_cells: int, height_cells: int,
                     vehicle_col: int, vehicle_row: int):
    """
    Local (base_link) koordinatı OccupancyGrid hücresine dönüştürür.
    """
    col = vehicle_col + math.floor(x_m / resolution + 0.5)
    row = vehicle_row + math.floor(y_m / resolution + 0.5)
    if 0 <= col < width_cells and 0 <= row < height_cells:
        return int(col), int(row)
    return None


def global_xy_to_grid(gx_m: float, gy_m: float,
                      resolution: float,
                      width_cells: int, height_cells: int,
                      origin_x: float, origin_y: float):
    """
    Global (map) koordinatı Global OccupancyGrid hücresine dönüştürür.
    """
    col = math.floor((gx_m - origin_x) / resolution)
    row = math.floor((gy_m - origin_y) / resolution)
    if 0 <= col < width_cells and 0 <= row < height_cells:
        return int(col), int(row)
    return None


# ---------------------------------------------------------------------------
# Global Takip Edilen Duba Kaydı
# ---------------------------------------------------------------------------
class TrackedBuoy:
    """Global haritada takip edilen ve doğrulanması yapılan duba nesnesi."""
    __slots__ = ('id', 'gx_m', 'gy_m', 'radius_m', 'obs_type', 'class_name',
                 'confidence', 'detection_count', 'status', 'last_seen',
                 'range_verified', 'marker_id')

    def __init__(self, buoy_id: str, gx_m: float, gy_m: float, radius_m: float,
                 obs_type: str, class_name: str, confidence: float,
                 marker_id: int, range_verified: bool = False):
        self.id              = buoy_id
        self.gx_m            = gx_m
        self.gy_m            = gy_m
        self.radius_m       = radius_m
        self.obs_type       = obs_type
        self.class_name     = class_name
        self.confidence     = confidence
        self.detection_count = 1
        self.status         = 'TENTATIVE'  # 'TENTATIVE' veya 'CONFIRMED'
        self.last_seen      = time.time()
        self.marker_id      = marker_id
        self.range_verified = range_verified

    def update(self, gx_m: float, gy_m: float, radius_m: float,
               obs_type: str, class_name: str, confidence: float,
               range_verified: bool, alpha: float = 0.3,
               confirm_threshold: int = 2):
        # EMA (Exponential Moving Average) ile pozisyon yumuşatma
        eff_alpha = alpha * 1.5 if range_verified else alpha
        eff_alpha = min(eff_alpha, 0.8)

        self.gx_m = eff_alpha * gx_m + (1.0 - eff_alpha) * self.gx_m
        self.gy_m = eff_alpha * gy_m + (1.0 - eff_alpha) * self.gy_m

        self.radius_m       = radius_m
        self.obs_type       = obs_type
        self.class_name     = class_name
        self.confidence     = max(self.confidence, confidence)
        self.last_seen      = time.time()
        self.range_verified = self.range_verified or range_verified

        self.detection_count += 1
        if self.detection_count >= confirm_threshold:
            self.status = 'CONFIRMED'


# ---------------------------------------------------------------------------
# Costmap Node Class
# ---------------------------------------------------------------------------
class CostmapNode(Node):

    def __init__(self):
        super().__init__('costmap_node')

        # ─── Parametreler ───────────────────────────────────────────────────
        self.declare_parameter('local_resolution',            0.20)
        self.declare_parameter('local_width_cells',           80)
        self.declare_parameter('local_height_cells',          80)
        self.declare_parameter('vehicle_forward_ratio',       0.20)
        self.declare_parameter('inflation_radius',            1.5)
        self.declare_parameter('decay_time_tentative',        4.0)
        self.declare_parameter('decay_time_confirmed',        20.0)
        self.declare_parameter('publish_rate',                5.0)
        self.declare_parameter('obstacle_timeout',            1.5)
        self.declare_parameter('association_distance_m',      0.60)
        self.declare_parameter('confirm_detection_threshold', 2)
        self.declare_parameter('max_gps_jump_m',              10.0)
        self.declare_parameter('max_boundary_link_distance_m', 4.50)
        self.declare_parameter('global_resolution',           0.25)
        self.declare_parameter('global_width_m',              60.0)
        self.declare_parameter('global_height_m',             60.0)

        self.local_res             = float(self.get_parameter('local_resolution').value)
        self.local_w               = int(self.get_parameter('local_width_cells').value)
        self.local_h               = int(self.get_parameter('local_height_cells').value)
        self.forward_ratio         = float(self.get_parameter('vehicle_forward_ratio').value)
        self.inflation_radius      = float(self.get_parameter('inflation_radius').value)
        self.decay_tentative       = float(self.get_parameter('decay_time_tentative').value)
        self.decay_confirmed       = float(self.get_parameter('decay_time_confirmed').value)
        self.publish_rate          = float(self.get_parameter('publish_rate').value)
        self.obstacle_timeout      = float(self.get_parameter('obstacle_timeout').value)
        self.association_dist      = float(self.get_parameter('association_distance_m').value)
        self.confirm_thresh        = int(self.get_parameter('confirm_detection_threshold').value)
        self.max_gps_jump          = float(self.get_parameter('max_gps_jump_m').value)
        self.max_link_dist         = float(self.get_parameter('max_boundary_link_distance_m').value)
        self.global_res            = float(self.get_parameter('global_resolution').value)
        self.global_w_m            = float(self.get_parameter('global_width_m').value)
        self.global_h_m            = float(self.get_parameter('global_height_m').value)

        self.global_w              = int(self.global_w_m / self.global_res)
        self.global_h              = int(self.global_h_m / self.global_res)

        # ─── Araç Local Grid Konumu ───────────────────────────────────────
        self._local_vc = int(self.local_w * self.forward_ratio)
        self._local_vr = self.local_h // 2

        # Global grid orijini ((0,0) başlangıç noktasını ortalayacak şekilde)
        self._global_origin_x = -15.0
        self._global_origin_y = -(self.global_h_m / 2.0)

        # ─── Dahili Durum Değişkenleri ─────────────────────────────────────
        self._lat_0 = None
        self._lon_0 = None
        self._gps_valid = False
        self._last_gps_time = None

        self._vehicle_x = 0.0
        self._vehicle_y = 0.0
        self._vehicle_yaw = 0.0
        self._imu_valid = False
        self._last_imu_time = None

        self._tracked_buoys: dict[str, TrackedBuoy] = {}
        self._next_marker_id = 100
        self._last_fusion_time = None
        self._observed_fov_deg = 70.0
        self._observed_range_m = 5.0

        self._latest_fusion_obstacles: list = []

        # TF Broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ─── Subscriber'lar ────────────────────────────────────────────────
        self._sub_gps = self.create_subscription(
            NavSatFix, GPS_TOPIC, self._gps_callback, qos_profile_sensor_data)

        self._sub_imu = self.create_subscription(
            Imu, IMU_TOPIC, self._imu_callback, qos_profile_sensor_data)

        self._sub_fusion = self.create_subscription(
            String, FUSION_TOPIC, self._fusion_callback, 10)

        # ─── Publisher'lar ─────────────────────────────────────────────────
        self._pub_local_grid  = self.create_publisher(OccupancyGrid, LOCAL_GRID_TOPIC,  10)
        self._pub_global_grid = self.create_publisher(OccupancyGrid, GLOBAL_GRID_TOPIC, 10)
        self._pub_course_mrk  = self.create_publisher(MarkerArray,   COURSE_MARKERS_TOPIC, 10)
        self._pub_centerline  = self.create_publisher(Path,          CENTERLINE_TOPIC,   10)
        self._pub_markers      = self.create_publisher(MarkerArray,   MARKERS_TOPIC,      10)
        self._pub_info         = self.create_publisher(String,        INFO_TOPIC,         10)
        self._pub_valid        = self.create_publisher(Bool,          VALID_TOPIC,        10)

        # ─── Timer ─────────────────────────────────────────────────────────
        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Gelişmiş Global + Local Costmap Node (v4) Başlatıldı.')
        self.get_logger().info(f'  Local Grid     : {self.local_w}x{self.local_h}, {self.local_res}m ({FRAME_LOCAL})')
        self.get_logger().info(f'  Global Grid    : {self.global_w}x{self.global_h}, {self.global_res}m ({FRAME_GLOBAL})')
        self.get_logger().info(f'  GPS Topic      : {GPS_TOPIC}')
        self.get_logger().info(f'  IMU Topic      : {IMU_TOPIC}')
        self.get_logger().info(f'  Fusion Topic   : {FUSION_TOPIC}')
        self.get_logger().info('=' * 60)

    # =========================================================================
    # Callback'ler
    # =========================================================================

    def _gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0 or math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self._gps_valid = False
            return

        lat = msg.latitude
        lon = msg.longitude

        # İlk geçerli ölçüm → (0,0) başlangıç orijini olarak kilitle
        if self._lat_0 is None:
            self._lat_0 = lat
            self._lon_0 = lon
            self.get_logger().info(
                f'★ Başlangıç GPS Referansı Kilitlendi (0,0): Lat={lat:.6f}, Lon={lon:.6f}'
            )

        new_x, new_y = latlon_to_local_enu(lat, lon, self._lat_0, self._lon_0)

        # GPS Jump Sanity Check
        if self._gps_valid:
            dist_jump = math.hypot(new_x - self._vehicle_x, new_y - self._vehicle_y)
            if dist_jump > self.max_gps_jump:
                self.get_logger().warn(
                    f'Fiziksel olarak imkansız GPS sıçraması reddedildi: {dist_jump:.1f} m'
                )
                return

        self._vehicle_x = new_x
        self._vehicle_y = new_y
        self._gps_valid = True
        self._last_gps_time = time.time()

    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            self._imu_valid = False
            return

        _, _, raw_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Yaw Smoothing (Hafif Alçak Geçiren Filtre)
        if self._imu_valid:
            # Angle wrap-around düzeltmesi
            diff = (raw_yaw - self._vehicle_yaw + math.pi) % (2.0 * math.pi) - math.pi
            self._vehicle_yaw = self._vehicle_yaw + 0.3 * diff
            self._vehicle_yaw = (self._vehicle_yaw + math.pi) % (2.0 * math.pi) - math.pi
        else:
            self._vehicle_yaw = raw_yaw

        self._imu_valid = True
        self._last_imu_time = time.time()

    def _fusion_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        if not isinstance(payload, dict) or not payload.get('fusion_valid', False):
            return

        obstacles = payload.get('obstacles', [])
        if not isinstance(obstacles, list):
            return

        fov = payload.get('observed_fov_deg', 70.0)
        rng = payload.get('observed_range_m', 5.0)

        self._last_fusion_time = time.time()
        self._observed_fov_deg = float(fov) if fov else 70.0
        self._observed_range_m = float(rng) if rng else 5.0
        self._latest_fusion_obstacles = obstacles

        # Duba takibi ve global haritaya işleme
        for obs in obstacles:
            self._track_fusion_obstacle(obs)

    # =========================================================================
    # Duba Takibi & Nearest-Neighbour Association
    # =========================================================================

    def _track_fusion_obstacle(self, obs: dict):
        lx = obs.get('x_m')
        ly = obs.get('y_m')
        if lx is None or ly is None:
            return

        obs_type = str(obs.get('type', 'unknown'))
        class_name = str(obs.get('class_name', 'unknown'))
        confidence = float(obs.get('confidence', 0.5))
        radius_m = float(obs.get('radius_m', 0.15))
        range_verified = bool(obs.get('range_verified', False))

        # Local (base_link) -> Global (map) dönüşümü
        gx, gy = local_to_global(lx, ly, self._vehicle_x, self._vehicle_y, self._vehicle_yaw)

        # Nearest-Neighbour Association
        best_key = None
        best_dist = float('inf')

        for key, buoy in self._tracked_buoys.items():
            dist = math.hypot(gx - buoy.gx_m, gy - buoy.gy_m)
            if dist < self.association_dist and dist < best_dist:
                best_dist = dist
                best_key = key

        if best_key is not None:
            self._tracked_buoys[best_key].update(
                gx, gy, radius_m, obs_type, class_name, confidence,
                range_verified, alpha=0.30, confirm_threshold=self.confirm_thresh
            )
        else:
            mid = self._next_marker_id
            self._next_marker_id += 1
            buoy_id = f"gbuoy_{mid}"
            new_buoy = TrackedBuoy(
                buoy_id, gx, gy, radius_m, obs_type, class_name, confidence, mid, range_verified
            )
            self._tracked_buoys[buoy_id] = new_buoy

    # =========================================================================
    # Grid Üreticiler
    # =========================================================================

    def _build_local_grid(self) -> list:
        total = self.local_w * self.local_h
        grid = [COST_UNKNOWN] * total

        # FOV Free Space Ray-Tracing
        self._mark_local_fov_free(grid)

        # Anlık fusion engellerini ekle (hem tentative hem confirmed)
        lethal_list = []
        for obs in self._latest_fusion_obstacles:
            lx = obs.get('x_m')
            ly = obs.get('y_m')
            obs_type = str(obs.get('type', 'unknown'))
            radius_m = float(obs.get('radius_m', 0.15))

            if lx is None or ly is None:
                continue

            cell = local_xy_to_grid(
                lx, ly, self.local_res, self.local_w, self.local_h,
                self._local_vc, self._local_vr
            )
            if cell is None:
                continue

            col, row = cell
            base_cost = TYPE_COST.get(obs_type, COST_LETHAL)
            idx = row * self.local_w + col
            grid[idx] = max(grid[idx], base_cost)
            lethal_list.append((col, row, base_cost, obs_type, radius_m))

        # Local Inflation
        for (col, row, base_cost, obs_type, radius_m) in lethal_list:
            if obs_type in NO_INFLATE_TYPES or base_cost < COST_LETHAL:
                continue
            self._apply_inflation_local(grid, col, row, radius_m)

        return grid

    def _mark_local_fov_free(self, grid: list):
        if self._last_fusion_time is None or (time.time() - self._last_fusion_time) > self.obstacle_timeout:
            return

        half_fov = math.radians(self._observed_fov_deg / 2.0)
        range_cells = int(math.ceil(self._observed_range_m / self.local_res))

        vc = self._local_vc
        vr = self._local_vr

        for dc in range(0, range_cells + 1):
            for dr in range(-range_cells, range_cells + 1):
                if dc * dc + dr * dr > range_cells * range_cells:
                    continue
                angle = math.atan2(dr, dc)
                if abs(angle) <= half_fov:
                    col = vc + dc
                    row = vr + dr
                    if 0 <= col < self.local_w and 0 <= row < self.local_h:
                        idx = row * self.local_w + col
                        if grid[idx] == COST_UNKNOWN:
                            grid[idx] = COST_FREE

    def _apply_inflation_local(self, grid: list, ec: int, er: int, radius_m: float):
        total_r_m = radius_m + self.inflation_radius
        total_cells = int(math.ceil(total_r_m / self.local_res))

        for dr in range(-total_cells, total_cells + 1):
            for dc in range(-total_cells, total_cells + 1):
                nc = ec + dc
                nr = er + dr
                if not (0 <= nc < self.local_w and 0 <= nr < self.local_h):
                    continue
                dist_m = math.hypot(dc, dr) * self.local_res
                if dist_m > total_r_m:
                    continue

                nidx = nr * self.local_w + nc
                if dist_m <= radius_m:
                    grid[nidx] = COST_LETHAL
                else:
                    ratio = 1.0 - (dist_m - radius_m) / self.inflation_radius
                    cost = max(1, int(COST_INFLATION_MAX * ratio))
                    if grid[nidx] < cost:
                        grid[nidx] = cost

    def _build_global_grid(self) -> list:
        total = self.global_w * self.global_h
        grid = [COST_FREE] * total

        lethal_list = []
        for buoy in self._tracked_buoys.values():
            if buoy.status != 'CONFIRMED':
                continue

            cell = global_xy_to_grid(
                buoy.gx_m, buoy.gy_m, self.global_res,
                self.global_w, self.global_h,
                self._global_origin_x, self._global_origin_y
            )
            if cell is None:
                continue

            col, row = cell
            base_cost = TYPE_COST.get(buoy.obs_type, COST_LETHAL)
            idx = row * self.global_w + col
            grid[idx] = max(grid[idx], base_cost)
            lethal_list.append((col, row, base_cost, buoy.obs_type, buoy.radius_m))

        for (col, row, base_cost, obs_type, radius_m) in lethal_list:
            if obs_type in NO_INFLATE_TYPES or base_cost < COST_LETHAL:
                continue
            self._apply_inflation_global(grid, col, row, radius_m)

        return grid

    def _apply_inflation_global(self, grid: list, ec: int, er: int, radius_m: float):
        total_r_m = radius_m + self.inflation_radius
        total_cells = int(math.ceil(total_r_m / self.global_res))

        for dr in range(-total_cells, total_cells + 1):
            for dc in range(-total_cells, total_cells + 1):
                nc = ec + dc
                nr = er + dr
                if not (0 <= nc < self.global_w and 0 <= nr < self.global_h):
                    continue
                dist_m = math.hypot(dc, dr) * self.global_res
                if dist_m > total_r_m:
                    continue

                nidx = nr * self.global_w + nc
                if dist_m <= radius_m:
                    grid[nidx] = COST_LETHAL
                else:
                    ratio = 1.0 - (dist_m - radius_m) / self.inflation_radius
                    cost = max(1, int(COST_INFLATION_MAX * ratio))
                    if grid[nidx] < cost:
                        grid[nidx] = cost

    # =========================================================================
    # Mesaj Oluşturucular & Görselleştirme Marker'ları
    # =========================================================================

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FRAME_GLOBAL
        t.child_frame_id  = FRAME_LOCAL

        t.transform.translation.x = self._vehicle_x
        t.transform.translation.y = self._vehicle_y
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self._vehicle_yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(t)

    def _build_occupancy_grid_msg(self, grid: list, is_global: bool) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()

        if is_global:
            msg.header.frame_id = FRAME_GLOBAL
            msg.info.resolution = self.global_res
            msg.info.width = self.global_w
            msg.info.height = self.global_h
            origin = Pose()
            origin.position.x = self._global_origin_x
            origin.position.y = self._global_origin_y
            origin.position.z = 0.0
            origin.orientation.w = 1.0
            msg.info.origin = origin
        else:
            msg.header.frame_id = FRAME_LOCAL
            msg.info.resolution = self.local_res
            msg.info.width = self.local_w
            msg.info.height = self.local_h
            origin = Pose()
            origin.position.x = -(self._local_vc + 0.5) * self.local_res
            origin.position.y = -(self._local_vr + 0.5) * self.local_res
            origin.position.z = 0.0
            origin.orientation.w = 1.0
            msg.info.origin = origin

        msg.data = grid
        return msg

    def _build_markers_array(self, expired_ids: list) -> MarkerArray:
        ma = MarkerArray()
        now_stamp = self.get_clock().now().to_msg()

        # Silinen marker'ları sil
        for mid in expired_ids:
            dm = Marker()
            dm.header.stamp = now_stamp
            dm.header.frame_id = FRAME_GLOBAL
            dm.ns = 'costmap_buoys'
            dm.id = mid
            dm.action = Marker.DELETE
            ma.markers.append(dm)

        # 1) Sabit START (0,0) Marker'ı
        sm = Marker()
        sm.header.stamp = now_stamp
        sm.header.frame_id = FRAME_GLOBAL
        sm.ns = 'start_point'
        sm.id = 0
        sm.type = Marker.CYLINDER
        sm.action = Marker.ADD
        sm.pose.position.x = 0.0
        sm.pose.position.y = 0.0
        sm.pose.position.z = 0.05
        sm.pose.orientation.w = 1.0
        sm.scale.x = 0.6
        sm.scale.y = 0.6
        sm.scale.z = 0.1
        sm.color.r, sm.color.g, sm.color.b, sm.color.a = (0.0, 1.0, 0.0, 0.8)
        ma.markers.append(sm)

        st = Marker()
        st.header.stamp = now_stamp
        st.header.frame_id = FRAME_GLOBAL
        st.ns = 'start_point_text'
        st.id = 1
        st.type = Marker.TEXT_VIEW_FACING
        st.action = Marker.ADD
        st.pose.position.x = 0.0
        st.pose.position.y = 0.0
        st.pose.position.z = 0.6
        st.scale.z = 0.4
        st.color.r, st.color.g, st.color.b, st.color.a = (1.0, 1.0, 1.0, 1.0)
        st.text = "START (0,0)"
        ma.markers.append(st)

        # 2) Araç Ok Marker'ı (ARROW)
        vm = Marker()
        vm.header.stamp = now_stamp
        vm.header.frame_id = FRAME_GLOBAL
        vm.ns = 'vehicle_marker'
        vm.id = 2
        vm.type = Marker.ARROW
        vm.action = Marker.ADD
        vm.pose.position.x = self._vehicle_x
        vm.pose.position.y = self._vehicle_y
        vm.pose.position.z = 0.2
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self._vehicle_yaw)
        vm.pose.orientation.x = qx
        vm.pose.orientation.y = qy
        vm.pose.orientation.z = qz
        vm.pose.orientation.w = qw
        vm.scale.x = 1.5  # Ok boyu
        vm.scale.y = 0.4  # Ok genişliği
        vm.scale.z = 0.4
        vm.color.r, vm.color.g, vm.color.b, sm.color.a = (0.1, 0.7, 1.0, 0.95)
        ma.markers.append(vm)

        vt = Marker()
        vt.header.stamp = now_stamp
        vt.header.frame_id = FRAME_GLOBAL
        vt.ns = 'vehicle_text'
        vt.id = 3
        vt.type = Marker.TEXT_VIEW_FACING
        vt.action = Marker.ADD
        vt.pose.position.x = self._vehicle_x
        vt.pose.position.y = self._vehicle_y
        vt.pose.position.z = 0.9
        vt.scale.z = 0.35
        vt.color.r, vt.color.g, vt.color.b, vt.color.a = (1.0, 1.0, 0.2, 1.0)
        yaw_deg = math.degrees(self._vehicle_yaw)
        vt.text = f"ALBATROS\nx={self._vehicle_x:.1f}m\ny={self._vehicle_y:.1f}m\nyaw={yaw_deg:.0f}°"
        ma.markers.append(vt)

        # 3) Dubalar & Mesafe Etiketleri
        for buoy in self._tracked_buoys.values():
            if buoy.status != 'CONFIRMED':
                continue

            dist_from_vehicle = math.hypot(buoy.gx_m - self._vehicle_x, buoy.gy_m - self._vehicle_y)

            # Silindir Duba Marker
            bm = Marker()
            bm.header.stamp = now_stamp
            bm.header.frame_id = FRAME_GLOBAL
            bm.ns = 'costmap_buoys'
            bm.id = buoy.marker_id
            bm.type = Marker.CYLINDER
            bm.action = Marker.ADD
            bm.pose.position.x = buoy.gx_m
            bm.pose.position.y = buoy.gy_m
            bm.pose.position.z = 0.20
            bm.pose.orientation.w = 1.0
            diam = max(buoy.radius_m * 2.0, 0.30)
            bm.scale.x = diam
            bm.scale.y = diam
            bm.scale.z = 0.40

            r, g, b, a = TYPE_COLOR.get(buoy.obs_type, (0.5, 0.5, 0.5, 0.7))
            bm.color.r, bm.color.g, bm.color.b, bm.color.a = r, g, b, a
            bm.lifetime = Duration(sec=int(self.decay_confirmed) + 1)
            ma.markers.append(bm)

            # Mesafe Yazısı Marker
            bt = Marker()
            bt.header.stamp = now_stamp
            bt.header.frame_id = FRAME_GLOBAL
            bt.ns = 'buoy_text'
            bt.id = buoy.marker_id + 50000
            bt.type = Marker.TEXT_VIEW_FACING
            bt.action = Marker.ADD
            bt.pose.position.x = buoy.gx_m
            bt.pose.position.y = buoy.gy_m
            bt.pose.position.z = 0.70
            bt.scale.z = 0.30
            bt.color.r, bt.color.g, bt.color.b, bt.color.a = (1.0, 1.0, 1.0, 0.95)

            name_upper = buoy.class_name.upper().replace('_', ' ')
            bt.text = f"{name_upper}\n{dist_from_vehicle:.1f} m"
            bt.lifetime = Duration(sec=int(self.decay_confirmed) + 1)
            ma.markers.append(bt)

        return ma

    def _publish_course_markers_and_centerline(self):
        now_stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # Doğrulanmış border dubalarını topla
        confirmed_borders = [
            b for b in self._tracked_buoys.values()
            if b.status == 'CONFIRMED' and b.obs_type in ('border_buoy', 'obstacle_buoy')
        ]

        if not confirmed_borders:
            self._pub_course_mrk.publish(ma)
            return

        # Sol ve Sağ dubaları ayır (Başlangıç rotasına veya araç trajectory'sine göre)
        left_buoys = []
        right_buoys = []

        for b in confirmed_borders:
            # Araç eksenine göre yerel y pozisyonu
            lx = (b.gx_m - self._vehicle_x) * math.cos(-self._vehicle_yaw) - (b.gy_m - self._vehicle_y) * math.sin(-self._vehicle_yaw)
            ly = (b.gx_m - self._vehicle_x) * math.sin(-self._vehicle_yaw) + (b.gy_m - self._vehicle_y) * math.cos(-self._vehicle_yaw)

            if ly >= 0:
                left_buoys.append((b.gx_m, b.gy_m, lx))
            else:
                right_buoys.append((b.gx_m, b.gy_m, lx))

        # İlerleme yönüne göre (lx) sırala
        left_buoys.sort(key=lambda item: item[2])
        right_buoys.sort(key=lambda item: item[2])

        # Sol Sınır Çizgisi Marker
        if len(left_buoys) >= 2:
            lm = Marker()
            lm.header.stamp = now_stamp
            lm.header.frame_id = FRAME_GLOBAL
            lm.ns = 'left_boundary'
            lm.id = 10
            lm.type = Marker.LINE_STRIP
            lm.action = Marker.ADD
            lm.scale.x = 0.12  # Çizgi kalınlığı
            lm.color.r, lm.color.g, lm.color.b, lm.color.a = (1.0, 0.3, 0.0, 0.85)

            prev_pt = None
            for gx, gy, _ in left_buoys:
                if prev_pt is not None:
                    d = math.hypot(gx - prev_pt[0], gy - prev_pt[1])
                    if d > self.max_link_dist:
                        break  # Mesafe eşiği aşıldıysa çizgi çizme
                p = Pose().position
                p.x, p.y, p.z = gx, gy, 0.1
                lm.points.append(p)
                prev_pt = (gx, gy)
            ma.markers.append(lm)

        # Sağ Sınır Çizgisi Marker
        if len(right_buoys) >= 2:
            rm = Marker()
            rm.header.stamp = now_stamp
            rm.header.frame_id = FRAME_GLOBAL
            rm.ns = 'right_boundary'
            rm.id = 11
            rm.type = Marker.LINE_STRIP
            rm.action = Marker.ADD
            rm.scale.x = 0.12
            rm.color.r, rm.color.g, rm.color.b, rm.color.a = (1.0, 0.3, 0.0, 0.85)

            prev_pt = None
            for gx, gy, _ in right_buoys:
                if prev_pt is not None:
                    d = math.hypot(gx - prev_pt[0], gy - prev_pt[1])
                    if d > self.max_link_dist:
                        break
                p = Pose().position
                p.x, p.y, p.z = gx, gy, 0.1
                rm.points.append(p)
                prev_pt = (gx, gy)
            ma.markers.append(rm)

        self._pub_course_mrk.publish(ma)

        # Global Course Centerline Path
        path = Path()
        path.header.stamp = now_stamp
        path.header.frame_id = FRAME_GLOBAL

        min_len = min(len(left_buoys), len(right_buoys))
        if min_len >= 1:
            for i in range(min_len):
                lx, ly, _ = left_buoys[i]
                rx, ry, _ = right_buoys[i]
                cx = (lx + rx) / 2.0
                cy = (ly + ry) / 2.0

                ps = PoseStamped()
                ps.header.stamp = now_stamp
                ps.header.frame_id = FRAME_GLOBAL
                ps.pose.position.x = cx
                ps.pose.position.y = cy
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)

            self._pub_centerline.publish(path)

    def _publish_info(self, local_valid: bool):
        now = self.get_clock().now().to_msg()
        stamp = now.sec + now.nanosec * 1e-9

        confirmed_count = sum(1 for b in self._tracked_buoys.values() if b.status == 'CONFIRMED')
        tentative_count = len(self._tracked_buoys) - confirmed_count

        dist_start = math.hypot(self._vehicle_x, self._vehicle_y)

        payload = {
            'stamp': stamp,
            'local_valid': local_valid,
            'global_valid': self._gps_valid and self._imu_valid,
            'vehicle_x_m': round(self._vehicle_x, 2),
            'vehicle_y_m': round(self._vehicle_y, 2),
            'vehicle_yaw_deg': round(math.degrees(self._vehicle_yaw), 1),
            'distance_from_start_m': round(dist_start, 2),
            'active_local_obstacles': len(self._latest_fusion_obstacles),
            'confirmed_global_buoys': confirmed_count,
            'tentative_global_buoys': tentative_count,
            'gps_age_sec': round(time.time() - self._last_gps_time, 2) if self._last_gps_time else -1,
            'imu_age_sec': round(time.time() - self._last_imu_time, 2) if self._last_imu_time else -1,
            'fusion_age_sec': round(time.time() - self._last_fusion_time, 2) if self._last_fusion_time else -1,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_info.publish(msg)

    # =========================================================================
    # Ana Timer
    # =========================================================================

    def _timer_callback(self):
        now = time.time()

        # Local Validity Kontrolü
        local_valid = True
        if self._last_fusion_time is None or (now - self._last_fusion_time) > self.obstacle_timeout:
            local_valid = False

        valid_msg = Bool()
        valid_msg.data = local_valid
        self._pub_valid.publish(valid_msg)

        # Decay Süresi Dolan Dubaları Temizle
        expired_keys = []
        for key, buoy in self._tracked_buoys.items():
            decay = self.decay_confirmed if buoy.status == 'CONFIRMED' else self.decay_tentative
            if (now - buoy.last_seen) > decay:
                expired_keys.append(key)

        expired_ids = [self._tracked_buoys[k].marker_id for k in expired_keys]
        for k in expired_keys:
            del self._tracked_buoys[k]

        # Dynamic TF Broadcast (map -> base_link)
        if self._gps_valid or self._imu_valid:
            self._publish_tf()

        # Grid'leri ve Görselleştirmeleri Yayınla
        local_grid = self._build_local_grid()
        global_grid = self._build_global_grid()

        self._pub_local_grid.publish(self._build_occupancy_grid_msg(local_grid, is_global=False))
        self._pub_global_grid.publish(self._build_occupancy_grid_msg(global_grid, is_global=True))
        self._pub_markers.publish(self._build_markers_array(expired_ids))

        self._publish_course_markers_and_centerline()
        self._publish_info(local_valid)


# ---------------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Costmap Node durduruldu.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
