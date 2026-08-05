#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — Costmap Node (v3)
# Giriş : /albatros/fusion/obstacles  [std_msgs/String JSON]
# Çıkış : /albatros/costmap/grid, info, markers, valid
# Koordinat: +x araç önü, +y araç solu  (base_link)
# ROS2 Jazzy / Ubuntu 24.04
# =============================================================================
import json
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

# ---------------------------------------------------------------------------
# Maliyet sabitleri
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

# Marker renkleri
TYPE_COLOR = {
    'obstacle_buoy': (1.0, 1.0, 0.0, 0.85),
    'border_buoy':   (1.0, 0.5, 0.0, 0.85),
    'target_buoy':   (0.2, 0.9, 0.2, 0.85),
    'goal_buoy':     (0.2, 0.4, 1.0, 0.85),
    'unknown':       (0.5, 0.5, 0.5, 0.70),
}

FUSION_TOPIC  = '/albatros/fusion/obstacles'
GRID_TOPIC    = '/albatros/costmap/grid'
INFO_TOPIC    = '/albatros/costmap/info'
MARKERS_TOPIC = '/albatros/costmap/markers'
VALID_TOPIC   = '/albatros/costmap/valid'
FRAME_ID      = 'base_link'


# ---------------------------------------------------------------------------
# Engel kaydı
# ---------------------------------------------------------------------------
class ObstacleRecord:
    __slots__ = ('x_m', 'y_m', 'radius_m', 'obs_type', 'class_name',
                 'confidence', 'last_seen', 'marker_id', 'range_verified')

    def __init__(self, x_m, y_m, radius_m, obs_type, class_name,
                 confidence, marker_id, range_verified=False):
        self.x_m            = x_m
        self.y_m            = y_m
        self.radius_m       = radius_m
        self.obs_type       = obs_type
        self.class_name     = class_name
        self.confidence     = confidence
        self.last_seen      = time.time()
        self.marker_id      = marker_id
        self.range_verified = range_verified

    def update(self, x_m, y_m, radius_m, obs_type, class_name,
               confidence, range_verified):
        self.x_m            = x_m
        self.y_m            = y_m
        self.radius_m       = radius_m
        self.obs_type       = obs_type
        self.class_name     = class_name
        self.confidence     = confidence
        self.last_seen      = time.time()
        self.range_verified = range_verified

    def is_expired(self, decay_time: float) -> bool:
        return (time.time() - self.last_seen) > decay_time


# ---------------------------------------------------------------------------
# Koordinat dönüşümü  — test edilebilir bağımsız fonksiyon
# ---------------------------------------------------------------------------
def local_xy_to_grid(x_m: float, y_m: float,
                     resolution: float,
                     width_cells: int, height_cells: int,
                     vehicle_col: int, vehicle_row: int):
    """
    Araç-merkezli (x=ileri, y=sol) koordinatı OccupancyGrid hücresine çevirir.

    OccupancyGrid standardı:
      data[row * width + col]
      col → +x (araç önü)
      row → +y (araç solu)

    math.floor kullanılır; araç (0,0) kendi hücresinin merkezindedir.
    Dönüş: (col, row) veya None
    """
    col = vehicle_col + math.floor(x_m / resolution + 0.5)
    row = vehicle_row + math.floor(y_m / resolution + 0.5)
    if 0 <= col < width_cells and 0 <= row < height_cells:
        return int(col), int(row)
    return None


# ---------------------------------------------------------------------------
# Yardımcı: marker rengi target_buoy için class_name'e göre
# ---------------------------------------------------------------------------
def _target_buoy_color(class_name: str):
    cn = class_name.lower()
    if any(k in cn for k in ('kirmizi', 'red', 'kırmızı')):
        return (1.0, 0.1, 0.1, 0.85)
    if any(k in cn for k in ('yesil', 'green', 'yeşil')):
        return (0.1, 0.9, 0.1, 0.85)
    return TYPE_COLOR['target_buoy']


# ---------------------------------------------------------------------------
# Yardımcı: güvenli float dönüşümü, None döner hata durumunda
# ---------------------------------------------------------------------------
def _safe_float(val):
    try:
        f = float(val)
        if math.isfinite(f):
            return f
        return None
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# Costmap Node
# ---------------------------------------------------------------------------
class CostmapNode(Node):

    def __init__(self):
        super().__init__('costmap_node')

        # ---- Parametreler ----
        self.declare_parameter('resolution',             0.25)
        self.declare_parameter('width_cells',            100)
        self.declare_parameter('height_cells',           100)
        self.declare_parameter('vehicle_forward_ratio',  0.20)
        self.declare_parameter('inflation_radius',       1.5)
        self.declare_parameter('decay_time',             3.0)
        self.declare_parameter('publish_rate',           5.0)
        self.declare_parameter('obstacle_timeout',       1.0)

        self.resolution            = float(self.get_parameter('resolution').value)
        self.width_cells           = int(self.get_parameter('width_cells').value)
        self.height_cells          = int(self.get_parameter('height_cells').value)
        self.vehicle_forward_ratio = float(self.get_parameter('vehicle_forward_ratio').value)
        self.inflation_radius      = float(self.get_parameter('inflation_radius').value)
        self.decay_time            = float(self.get_parameter('decay_time').value)
        self.publish_rate          = float(self.get_parameter('publish_rate').value)
        self.obstacle_timeout      = float(self.get_parameter('obstacle_timeout').value)

        # ---- Parametre doğrulama ----
        errors = []
        if self.resolution <= 0:
            errors.append(f'resolution={self.resolution} > 0 olmalı')
        if self.width_cells <= 0:
            errors.append(f'width_cells={self.width_cells} > 0 olmalı')
        if self.height_cells <= 0:
            errors.append(f'height_cells={self.height_cells} > 0 olmalı')
        if not (0 < self.vehicle_forward_ratio < 1):
            errors.append(f'vehicle_forward_ratio={self.vehicle_forward_ratio} (0,1) aralığında olmalı')
        if self.inflation_radius < 0:
            errors.append(f'inflation_radius={self.inflation_radius} >= 0 olmalı')
        if self.decay_time <= 0:
            errors.append(f'decay_time={self.decay_time} > 0 olmalı')
        if self.publish_rate <= 0:
            errors.append(f'publish_rate={self.publish_rate} > 0 olmalı')
        if self.obstacle_timeout <= 0:
            errors.append(f'obstacle_timeout={self.obstacle_timeout} > 0 olmalı')
        if errors:
            for e in errors:
                self.get_logger().error(f'Parametre hatası: {e}')
            raise ValueError('Geçersiz parametre(ler): ' + '; '.join(errors))

        # ---- Araç grid pozisyonu ----
        # vehicle_forward_ratio=0.20 → aracın önünde %80, arkasında %20 alan
        # +x=ileri → col artar; araç col=vehicle_col, sağ-sol eşit (row=height//2)
        self._vehicle_col = int(self.width_cells * self.vehicle_forward_ratio)
        self._vehicle_row = self.height_cells // 2

        # Grid max menzil (m)
        self._max_range_m = (self.width_cells - self._vehicle_col - 1) * self.resolution

        # ---- Durum ----
        self._obstacles: dict         = {}
        self._next_marker_id: int     = 0
        self._last_fusion_time: float = None
        self._costmap_valid: bool     = False
        self._observed_fov_deg: float = None
        self._observed_range_m: float = None

        # ---- Subscriber ----
        self._fusion_sub = self.create_subscription(
            String, FUSION_TOPIC, self._fusion_callback, 10)

        # ---- Publisher'lar ----
        self._grid_pub    = self.create_publisher(OccupancyGrid, GRID_TOPIC,    10)
        self._info_pub    = self.create_publisher(String,        INFO_TOPIC,    10)
        self._markers_pub = self.create_publisher(MarkerArray,   MARKERS_TOPIC, 10)
        self._valid_pub   = self.create_publisher(Bool,          VALID_TOPIC,   10)

        # ---- Timer ----
        self._timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Costmap Node (v3) başlatıldı.')
        self.get_logger().info(f'  Grid          : {self.width_cells}x{self.height_cells}, {self.resolution} m/hücre')
        self.get_logger().info(f'  Araç konumu   : col={self._vehicle_col}, row={self._vehicle_row}')
        self.get_logger().info(f'  Ön alan       : {(self.width_cells - 1 - self._vehicle_col) * self.resolution:.1f} m')
        self.get_logger().info(f'  Arka alan     : {self._vehicle_col * self.resolution:.1f} m')
        self.get_logger().info(f'  Sol/sağ alan  : {self._vehicle_row * self.resolution:.1f} m (eşit)')
        self.get_logger().info(f'  Inflation     : {self.inflation_radius} m')
        self.get_logger().info(f'  Decay         : {self.decay_time} s')
        self.get_logger().info(f'  Fusion topic  : {FUSION_TOPIC}')
        self.get_logger().info('=' * 60)

    # =========================================================================
    # Fusion callback
    # =========================================================================

    def _fusion_callback(self, msg: String):
        # 1) JSON parse
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().warn(
                f'Fusion JSON parse hatası: {exc}', throttle_duration_sec=5.0)
            return

        if not isinstance(payload, dict):
            self.get_logger().warn(
                'Fusion payload dict değil.', throttle_duration_sec=5.0)
            return

        # 2) fusion_valid
        if not payload.get('fusion_valid', False):
            self.get_logger().warn(
                'fusion_valid=false — atlanıyor.', throttle_duration_sec=5.0)
            return

        # 3) frame_id
        if payload.get('frame_id') != 'base_link':
            self.get_logger().warn(
                f"Beklenen frame_id=base_link, gelen={payload.get('frame_id')} — atlanıyor.",
                throttle_duration_sec=5.0)
            return

        # 4) obstacles list
        obstacles = payload.get('obstacles')
        if not isinstance(obstacles, list):
            self.get_logger().warn(
                'obstacles listesi eksik/bozuk — atlanıyor.', throttle_duration_sec=5.0)
            return

        # 5) observed_fov_deg / observed_range_m
        fov = _safe_float(payload.get('observed_fov_deg'))
        rng = _safe_float(payload.get('observed_range_m'))
        if fov is None or not (1.0 <= fov <= 179.0):
            self.get_logger().warn(
                f'Geçersiz observed_fov_deg={payload.get("observed_fov_deg")} — atlanıyor.',
                throttle_duration_sec=5.0)
            return
        if rng is None or not (0.5 <= rng <= self._max_range_m + 50):
            self.get_logger().warn(
                f'Geçersiz observed_range_m={payload.get("observed_range_m")} — atlanıyor.',
                throttle_duration_sec=5.0)
            return

        # Tüm doğrulamalar geçti → zaman damgasını güncelle
        self._last_fusion_time = time.time()
        self._observed_fov_deg = fov
        self._observed_range_m = rng

        for obs in obstacles:
            self._process_obstacle(obs)

    def _process_obstacle(self, obs: dict):
        if not isinstance(obs, dict):
            return

        # x_m, y_m
        x_m = _safe_float(obs.get('x_m'))
        y_m = _safe_float(obs.get('y_m'))
        if x_m is None or y_m is None:
            self.get_logger().warn(
                f'Geçersiz x_m/y_m — engel atlanıyor.', throttle_duration_sec=5.0)
            return

        # confidence — NaN/inf/dışı aralık → engeli atla
        conf_raw = obs.get('confidence')
        conf = _safe_float(conf_raw)
        if conf is None or not (0.0 <= conf <= 1.0):
            self.get_logger().warn(
                f'Geçersiz confidence={conf_raw} — engel atlanıyor.', throttle_duration_sec=5.0)
            return

        obs_type   = str(obs.get('type', 'unknown'))
        class_name = str(obs.get('class_name', 'unknown'))
        range_verified = bool(obs.get('range_verified', False))

        radius_m = _safe_float(obs.get('radius_m', 0.15))
        if radius_m is None or radius_m < 0:
            radius_m = 0.15

        # Takip anahtarı
        obs_id = obs.get('id')
        if obs_id:
            key = str(obs_id)
        else:
            snap = self.resolution * 2.0
            kx = math.floor(x_m / snap + 0.5) * snap
            ky = math.floor(y_m / snap + 0.5) * snap
            key = f'{obs_type}_{kx:.2f}_{ky:.2f}'

        if key in self._obstacles:
            self._obstacles[key].update(
                x_m, y_m, radius_m, obs_type, class_name, conf, range_verified)
        else:
            mid = self._next_marker_id
            self._next_marker_id += 1
            self._obstacles[key] = ObstacleRecord(
                x_m=x_m, y_m=y_m, radius_m=radius_m,
                obs_type=obs_type, class_name=class_name,
                confidence=conf, marker_id=mid,
                range_verified=range_verified)

    # =========================================================================
    # Grid oluşturma
    # =========================================================================

    def _build_grid(self):
        total = self.width_cells * self.height_cells
        grid  = [COST_UNKNOWN] * total

        self._mark_fov_free(grid)

        # Engel merkezlerini yerleştir
        lethal_list = []
        for rec in self._obstacles.values():
            cell = local_xy_to_grid(
                rec.x_m, rec.y_m, self.resolution,
                self.width_cells, self.height_cells,
                self._vehicle_col, self._vehicle_row)
            if cell is None:
                continue
            col, row = cell
            base_cost = TYPE_COST.get(rec.obs_type, COST_LETHAL)
            idx = row * self.width_cells + col
            if grid[idx] < base_cost:
                grid[idx] = base_cost
            lethal_list.append((col, row, base_cost, rec.obs_type, rec.radius_m))

        # Inflation
        for (ec, er, base_cost, obs_type, radius_m) in lethal_list:
            if obs_type in NO_INFLATE_TYPES:
                continue
            if base_cost < COST_LETHAL:
                continue
            self._apply_inflation(grid, ec, er, radius_m)

        return grid

    def _mark_fov_free(self, grid: list):
        if not self._costmap_valid:
            return
        if self._last_fusion_time is None:
            return

        fov_deg = self._observed_fov_deg if self._observed_fov_deg else 70.0
        range_m = self._observed_range_m if self._observed_range_m else 8.0

        half_fov    = math.radians(fov_deg / 2.0)
        range_cells = int(math.ceil(range_m / self.resolution))
        vc = self._vehicle_col
        vr = self._vehicle_row

        for dc in range(0, range_cells + 1):
            for dr in range(-range_cells, range_cells + 1):
                dist2 = dc * dc + dr * dr
                if dist2 > range_cells * range_cells:
                    continue
                if dc == 0 and dr == 0:
                    idx = vr * self.width_cells + vc
                    if grid[idx] == COST_UNKNOWN:
                        grid[idx] = COST_FREE
                    continue
                angle = math.atan2(dr, dc)
                if abs(angle) <= half_fov:
                    col = vc + dc
                    row = vr + dr
                    if 0 <= col < self.width_cells and 0 <= row < self.height_cells:
                        idx = row * self.width_cells + col
                        if grid[idx] == COST_UNKNOWN:
                            grid[idx] = COST_FREE

    def _apply_inflation(self, grid: list, ec: int, er: int, radius_m: float):
        """
        radius_m içindeki hücreler → COST_LETHAL (merkez footprint).
        radius_m ile radius_m+inflation_radius arasındaki hücreler → azalan maliyet.
        Başka lethal hücrenin maliyetini düşürmez.
        """
        total_radius_m = radius_m + self.inflation_radius
        total_cells    = int(math.ceil(total_radius_m / self.resolution))

        for dr in range(-total_cells, total_cells + 1):
            for dc in range(-total_cells, total_cells + 1):
                nr = er + dr
                nc = ec + dc
                if not (0 <= nc < self.width_cells and 0 <= nr < self.height_cells):
                    continue
                dist_m = math.sqrt(dr * dr + dc * dc) * self.resolution
                if dist_m > total_radius_m:
                    continue

                nidx = nr * self.width_cells + nc

                if dist_m <= radius_m:
                    # Fiziksel footprint → lethal
                    if grid[nidx] < COST_LETHAL:
                        grid[nidx] = COST_LETHAL
                else:
                    # Inflation zone → azalan maliyet
                    ratio    = 1.0 - (dist_m - radius_m) / self.inflation_radius
                    inf_cost = int(COST_INFLATION_MAX * ratio)
                    inf_cost = max(1, inf_cost)
                    if grid[nidx] < inf_cost:
                        grid[nidx] = inf_cost

    # =========================================================================
    # Mesaj oluşturucular
    # =========================================================================

    def _build_occupancy_grid_msg(self, grid: list) -> OccupancyGrid:
        """
        origin: grid sol-alt köşesinin (col=0, row=0) araç koordinatındaki yeri.
        col=vehicle_col → x=0 (araç).  orijin_x = -(vehicle_col * res + res/2)
        Hücre merkezi: x = (col + 0.5) * res + origin.x
        Araç hücresi merkezi = (vehicle_col + 0.5)*res - (vehicle_col + 0.5)*res = 0. ✓
        """
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID

        msg.info.resolution = self.resolution
        msg.info.width      = self.width_cells
        msg.info.height     = self.height_cells

        origin = Pose()
        origin.position.x    = -(self._vehicle_col + 0.5) * self.resolution
        origin.position.y    = -(self._vehicle_row + 0.5) * self.resolution
        origin.position.z    = 0.0
        origin.orientation.w = 1.0
        msg.info.origin = origin

        msg.data = grid
        return msg

    def _grid_stats(self, grid: list) -> dict:
        unknown  = 0
        free     = 0
        lethal   = 0
        inflated = 0
        target   = 0
        for v in grid:
            if v == COST_UNKNOWN:
                unknown += 1
            elif v == COST_FREE:
                free += 1
            elif v == COST_LETHAL:
                lethal += 1
            elif v == COST_TARGET:
                target += 1
            elif 0 < v < COST_LETHAL and v != COST_TARGET:
                inflated += 1
        return {
            'unknown_cell_count':  unknown,
            'free_cell_count':     free,
            'lethal_cell_count':   lethal,
            'inflated_cell_count': inflated,
            'target_cell_count':   target,
        }

    def _build_info_msg(self, grid: list) -> String:
        now   = self.get_clock().now().to_msg()
        stamp = now.sec + now.nanosec * 1e-9
        age   = (time.time() - self._last_fusion_time) if self._last_fusion_time else -1.0
        payload = {
            'stamp':               stamp,
            'frame_id':            FRAME_ID,
            'valid':               self._costmap_valid,
            'resolution':          self.resolution,
            'width_cells':         self.width_cells,
            'height_cells':        self.height_cells,
            'active_obstacle_count': len(self._obstacles),
            'last_fusion_age_sec': round(age, 3),
            **self._grid_stats(grid),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        return msg

    def _build_marker_array(self, deleted_ids: list) -> MarkerArray:
        ma       = MarkerArray()
        now_stmp = self.get_clock().now().to_msg()

        for mid in deleted_ids:
            dm = Marker()
            dm.header.stamp    = now_stmp
            dm.header.frame_id = FRAME_ID
            dm.ns              = 'costmap_obstacles'
            dm.id              = mid
            dm.action          = Marker.DELETE
            ma.markers.append(dm)

        for rec in self._obstacles.values():
            m = Marker()
            m.header.stamp    = now_stmp
            m.header.frame_id = FRAME_ID
            m.ns              = 'costmap_obstacles'
            m.id              = rec.marker_id
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD

            m.pose.position.x    = rec.x_m
            m.pose.position.y    = rec.y_m
            m.pose.position.z    = 0.15
            m.pose.orientation.w = 1.0

            diam = max(rec.radius_m * 2.0, 0.20)
            m.scale.x = diam
            m.scale.y = diam
            m.scale.z = 0.30

            if rec.obs_type == 'target_buoy':
                r, g, b, a = _target_buoy_color(rec.class_name)
            else:
                r, g, b, a = TYPE_COLOR.get(rec.obs_type, (0.5, 0.5, 0.5, 0.7))
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a

            m.lifetime = Duration(sec=int(self.decay_time) + 1)
            ma.markers.append(m)

        return ma

    # =========================================================================
    # Ana timer
    # =========================================================================

    def _timer_callback(self):
        now = time.time()

        # Validity
        if self._last_fusion_time is None:
            self._costmap_valid = False
            self.get_logger().warn(
                'Fusion verisi henüz gelmedi — costmap geçersiz.',
                throttle_duration_sec=10.0)
        elif (now - self._last_fusion_time) > self.obstacle_timeout:
            self._costmap_valid = False
            self.get_logger().warn(
                f'Fusion kesildi ({now - self._last_fusion_time:.1f}s) — costmap_valid=false.',
                throttle_duration_sec=5.0)
        else:
            self._costmap_valid = True

        valid_msg = Bool()
        valid_msg.data = self._costmap_valid
        self._valid_pub.publish(valid_msg)

        # Decay
        expired_keys = [k for k, r in self._obstacles.items()
                        if r.is_expired(self.decay_time)]
        deleted_ids  = [self._obstacles[k].marker_id for k in expired_keys]
        for k in expired_keys:
            del self._obstacles[k]

        grid = self._build_grid()

        self._grid_pub.publish(self._build_occupancy_grid_msg(grid))
        self._info_pub.publish(self._build_info_msg(grid))
        self._markers_pub.publish(self._build_marker_array(deleted_ids))

        self.get_logger().debug(
            f'Costmap yayınlandı — engel={len(self._obstacles)}, valid={self._costmap_valid}')


# ---------------------------------------------------------------------------
# Giriş noktası
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
