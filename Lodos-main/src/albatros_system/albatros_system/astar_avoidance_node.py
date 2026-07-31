#!/usr/bin/env python3
"""
astar_avoidance_node.py — LODOS Albatros A* (A-Star) Yol Planlama ve Kaçınma Node'u
====================================================================================
Costmap (OccupancyGrid) üzerinde A* (A-Star) arama algoritmasını çalıştırarak
başlangıç noktasından (araç) hedef noktasına kadar en optimal engelsiz rotayı
hesaplar ve aracı bu rota boyunca sürecek hız/dönüş komutlarını üretir.

Algoritma Mantığı (A* Search Algorithm):
  1. GİRDİLER: Costmap ızgarası, aracın ızgara pozisyonu (0,0 araç merkezi),
     ve hedefin ızgara üzerindeki konumu.
  2. A* ARAMASI (Grid Search):
     - Öncelikli Kuyruk (Min-Heap / Priority Queue) kullanılır.
     - Formül: f(n) = g(n) + h(n) + cost_penalty(n)
       - g(n): Başlangıçtan n düğümüne kadar olan gerçek mesafe.
       - h(n): n düğümünden hedefe olan Öklid mesafesi (Heuristic).
       - cost_penalty(n): Costmap üzerindeki şişirilmiş ceza puanı (dubaya yaklaştıkça ceza artar).
     - 8 komşuluk (çapraz hareketler dahil) taranır.
  3. ROTA TAKİBİ (Lookahead / Pure Pursuit Mantığı):
     - Hesaplanan A* rotası üzerindeki ileri bir nokta (lookahead point) hedeflenir.
     - Bu noktaya olan yön farkı hesaplanarak yumuşak dönüş ve hız komutları (cmd_vel) üretilir.

Veri Akışı:
  Girişler:
    - /albatros/costmap/grid   (OccupancyGrid) ← costmap_node
    - /albatros/costmap/valid  (Bool)           ← costmap_node
    - /albatros/mission/target (MissionTarget)  ← mission_node
    - /albatros/imu/data       (Imu)            ← imu_sensor_node
  Çıkışlar:
    - /albatros/command/cmd_vel     (Twist)  → control_node
    - /albatros/avoidance/status    (String) → Debug/YKİ (JSON)

Yazar : LODOS Takımı
Araç  : Albatros İDA
"""

import math
import heapq
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

# ── Costmap Sabitleri ────────────────────────────────────────────────────────
COST_LETHAL = 100

# ── Varsayılan Parametreler ──────────────────────────────────────────────────
DEFAULT_LOOKAHEAD_DISTANCE_M = 2.0     # Yol takip mesafe mesafesi (m)
DEFAULT_LETHAL_COST_THRESHOLD = 80     # Kesinlikle engelli kabul edilecek costmap değeri
DEFAULT_COST_WEIGHT           = 0.05   # Costmap engel ceza çarpanı
DEFAULT_MAX_LINEAR_SPEED      = 1.0    # Maksimum ileri hız (m/s)
DEFAULT_MIN_LINEAR_SPEED      = 0.2    # Minimum ileri hız (m/s)
DEFAULT_MAX_ANGULAR_SPEED     = 0.8    # Maksimum dönüş hızı (rad/s)
DEFAULT_STEERING_KP           = 1.5    # Yön kontrol Oransal Kazanç (Kp)
DEFAULT_PUBLISH_RATE          = 10.0   # Çalışma frekansı (Hz)
DEFAULT_TIMEOUT_SEC           = 2.0    # Veri zaman aşımı süresi (s)


class AStarNode(Node):
    """
    A* (A-Star) tabanlı yol planlama ve engelden kaçınma ROS2 node'u.
    """

    def __init__(self):
        super().__init__('astar_avoidance_node')

        # ── Parametreler ─────────────────────────────────────────────────────
        self.declare_parameter('lookahead_distance_m', DEFAULT_LOOKAHEAD_DISTANCE_M)
        self.declare_parameter('lethal_cost_threshold', DEFAULT_LETHAL_COST_THRESHOLD)
        self.declare_parameter('cost_weight',           DEFAULT_COST_WEIGHT)
        self.declare_parameter('max_linear_speed',      DEFAULT_MAX_LINEAR_SPEED)
        self.declare_parameter('min_linear_speed',      DEFAULT_MIN_LINEAR_SPEED)
        self.declare_parameter('max_angular_speed',     DEFAULT_MAX_ANGULAR_SPEED)
        self.declare_parameter('steering_kp',           DEFAULT_STEERING_KP)
        self.declare_parameter('publish_rate',          DEFAULT_PUBLISH_RATE)
        self.declare_parameter('timeout_sec',           DEFAULT_TIMEOUT_SEC)

        self._lookahead_dist   = float(self.get_parameter('lookahead_distance_m').value)
        self._lethal_threshold = int(self.get_parameter('lethal_cost_threshold').value)
        self._cost_weight      = float(self.get_parameter('cost_weight').value)
        self._max_linear       = float(self.get_parameter('max_linear_speed').value)
        self._min_linear       = float(self.get_parameter('min_linear_speed').value)
        self._max_angular      = float(self.get_parameter('max_angular_speed').value)
        self._steering_kp      = float(self.get_parameter('steering_kp').value)
        self._publish_rate     = float(self.get_parameter('publish_rate').value)
        self._timeout_sec      = float(self.get_parameter('timeout_sec').value)

        # ── Dahili Durum ─────────────────────────────────────────────────────
        self._latest_costmap    = None
        self._costmap_valid     = False
        self._last_costmap_time  = None

        self._target_valid       = False
        self._mission_active     = False
        self._target_bearing_deg = 0.0
        self._distance_to_target = 0.0
        self._last_target_time   = None

        self._imu_valid        = False
        self._current_yaw_deg  = 0.0
        self._last_imu_time    = None

        self._current_path     = []   # A* ile bulunan yol (grid koordinatları listesi)
        self._total_commands   = 0
        self._total_stops      = 0
        self._last_status_time = 0.0

        # ── QoS ve Abonelikler ───────────────────────────────────────────────
        default_qos = QoSProfile(depth=10)

        self.create_subscription(OccupancyGrid, COSTMAP_GRID_TOPIC, self._cb_costmap, default_qos)
        self.create_subscription(Bool, COSTMAP_VALID_TOPIC, self._cb_costmap_valid, default_qos)
        self.create_subscription(MissionTarget, MISSION_TARGET_TOPIC, self._cb_mission_target, default_qos)
        self.create_subscription(Imu, IMU_TOPIC, self._cb_imu, qos_profile_sensor_data)

        # ── Yayıncılar ───────────────────────────────────────────────────────
        self._pub_cmd_vel = self.create_publisher(Twist, CMD_VEL_TOPIC, default_qos)
        self._pub_status  = self.create_publisher(String, AVOIDANCE_STATUS_TOPIC, default_qos)

        # ── Timer ───────────────────────────────────────────────────────────
        period_sec = 1.0 / max(self._publish_rate, 0.1)
        self._timer = self.create_timer(period_sec, self._timer_callback)

        self.get_logger().info('=' * 60)
        self.get_logger().info('A* Yol Planlama ve Kaçınma Node başlatıldı.')
        self.get_logger().info(f'  Lookahead mesafe : {self._lookahead_dist} m')
        self.get_logger().info(f'  Lethal Eşik      : {self._lethal_threshold}')
        self.get_logger().info(f'  Maks Hız         : {self._max_linear} m/s')
        self.get_logger().info(f'  Dönüş Hızı       : {self._max_angular} rad/s')
        self.get_logger().info('=' * 60)

    # =========================================================================
    # Callback'ler
    # =========================================================================

    def _cb_costmap(self, msg: OccupancyGrid):
        self._latest_costmap = msg
        self._last_costmap_time = self.get_clock().now()

    def _cb_costmap_valid(self, msg: Bool):
        self._costmap_valid = msg.data

    def _cb_mission_target(self, msg: MissionTarget):
        self._target_valid       = msg.target_valid
        self._mission_active     = msg.mission_active
        self._target_bearing_deg = msg.target_bearing_deg
        self._distance_to_target = msg.distance_to_target_m
        self._last_target_time   = self.get_clock().now()

    def _cb_imu(self, msg: Imu):
        yaw = self._quaternion_to_yaw_deg(msg)
        if yaw is not None:
            self._imu_valid = True
            self._current_yaw_deg = (yaw + 360.0) % 360.0
        else:
            self._imu_valid = False
        self._last_imu_time = self.get_clock().now()

    # =========================================================================
    # Ana Kontrol Döngüsü
    # =========================================================================

    def _timer_callback(self):
        # 1. Veri Tazelik Kontrolü
        if not self._is_data_valid():
            self._publish_stop()
            self._publish_status(active=False, reason='DATA_INVALID_OR_STALE')
            return

        costmap = self._latest_costmap
        res = costmap.info.resolution
        w   = costmap.info.width
        h   = costmap.info.height
        ox  = costmap.info.origin.position.x
        oy  = costmap.info.origin.position.y

        # Araç konumu (Start Cell)
        start_col = int(round(-ox / res - 0.5))
        start_row = int(round(-oy / res - 0.5))

        # Hedef açısı ve mesafesi
        heading_error_deg = self._normalize_angle_180(self._target_bearing_deg - self._current_yaw_deg)
        goal_body_deg = -heading_error_deg
        goal_rad = math.radians(goal_body_deg)

        # Hedef koordinatının grid izdüşümü (Goal Cell)
        target_dist = min(self._distance_to_target, (w * res) / 2.0 - 0.5)
        goal_x_m = target_dist * math.cos(goal_rad)
        goal_y_m = target_dist * math.sin(goal_rad)

        goal_col = start_col + int(round(goal_x_m / res))
        goal_row = start_row + int(round(goal_y_m / res))

        # Grid sınırlarına kelepçele (clamp)
        goal_col = max(0, min(w - 1, goal_col))
        goal_row = max(0, min(h - 1, goal_row))

        # 2. A* Algoritması ile Yol Hesabı
        path = self._run_astar(
            grid_data=costmap.data,
            width=w, height=h,
            start=(start_col, start_row),
            goal=(goal_col, goal_row)
        )

        self._current_path = path

        if not path:
            self.get_logger().warn('A*: Rota bulunamadı (Tıkandı) — Araç durduruluyor.', throttle_duration_sec=2.0)
            self._publish_stop()
            self._total_stops += 1
            self._publish_status(active=True, reason='NO_PATH_FOUND')
            return

        # 3. Yoldan İleri Takip Noktası Seçimi (Lookahead Point)
        target_point = self._get_lookahead_point(path, start_col, start_row, res)

        # Takip noktasına olan açı (Body frame)
        dx_m = (target_point[0] - start_col) * res
        dy_m = (target_point[1] - start_row) * res
        target_angle_deg = math.degrees(math.atan2(dy_m, dx_m))

        # 4. Hız Komutu Hesabı (cmd_vel)
        linear_speed, angular_speed = self._compute_cmd_vel(target_angle_deg, target_point, start_col, start_row, res)

        # Komutu Yayınla
        cmd = Twist()
        cmd.linear.x  = linear_speed
        cmd.angular.z = angular_speed
        self._pub_cmd_vel.publish(cmd)
        self._total_commands += 1

        self._publish_status(
            active=True,
            reason='ASTAR_ACTIVE',
            target_angle=target_angle_deg,
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            path_len=len(path)
        )

    # =========================================================================
    # A* (A-Star) Arama Algoritması
    # =========================================================================

    def _run_astar(self, grid_data, width, height, start, goal):
        """
        OccupancyGrid üzerinde A* algoritmasını çalıştırır.

        Returns:
            [(col1, row1), (col2, row2), ...] şeklinde hücre listesi.
            Yol bulunamazsa boş liste [].
        """
        start_node = (start[0], start[1])
        goal_node  = (goal[0], goal[1])

        # Başlangıç veya bitiş hücresi kesin engelli mi?
        if self._is_cell_lethal(grid_data, width, start_node[0], start_node[1]):
            return []

        # 8 Yönlü Hareket (Komşuluklar ve adımlama maliyetleri)
        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        # Priority Queue: (f_score, col, row)
        open_set = []
        heapq.heappush(open_set, (0.0, start_node[0], start_node[1]))

        came_from = {}
        g_score = {start_node: 0.0}

        def heuristic(c, r):
            # Öklid Mesafesi (Euclidean Heuristic)
            return math.sqrt((c - goal_node[0])**2 + (r - goal_node[1])**2)

        visited = set()

        while open_set:
            _, curr_c, curr_r = heapq.heappop(open_set)
            curr = (curr_c, curr_r)

            if curr in visited:
                continue
            visited.add(curr)

            # Hedeften yeterince yakına varıldı mı?
            if abs(curr_c - goal_node[0]) <= 1 and abs(curr_r - goal_node[1]) <= 1:
                # Rota oluştur
                path = []
                temp = curr
                while temp in came_from:
                    path.append(temp)
                    temp = came_from[temp]
                path.append(start_node)
                path.reverse()
                return path

            for dc, dr, move_cost in neighbors:
                nc, nr = curr_c + dc, curr_r + dr

                if not (0 <= nc < width and 0 <= nr < height):
                    continue

                if self._is_cell_lethal(grid_data, width, nc, nr):
                    continue

                # Costmap üzerindeki engel ceza puanını ekle
                idx = nr * width + nc
                cost_val = grid_data[idx]
                penalty = (cost_val if cost_val > 0 else 0) * self._cost_weight

                tentative_g = g_score[curr] + move_cost + penalty

                neighbor = (nc, nr)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = curr
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(nc, nr)
                    heapq.heappush(open_set, (f_score, nc, nr))

        return []  # Yol bulunamadı

    def _is_cell_lethal(self, grid_data, width, col, row):
        idx = row * width + col
        val = grid_data[idx]
        return val >= self._lethal_threshold

    # =========================================================================
    # Rota Takip ve Hız Komutu Hesabı
    # =========================================================================

    def _get_lookahead_point(self, path, start_col, start_row, res):
        """
        Bulunan A* yolunda araçtan itibaren 'lookahead_dist' mesafe uzaklıktaki
        noktayı bulur.
        """
        lookahead_cells = self._lookahead_dist / res

        for col, row in path:
            dist = math.sqrt((col - start_col)**2 + (row - start_row)**2)
            if dist >= lookahead_cells:
                return (col, row)

        return path[-1]  # Yeterince uzak nokta yoksa son noktayı dön

    def _compute_cmd_vel(self, target_angle_deg, target_point, start_col, start_row, res):
        angle_err_rad = math.radians(target_angle_deg)
        angular = self._clamp(self._steering_kp * angle_err_rad, -self._max_angular, self._max_angular)

        # Açısal sapma arttıkça yavaşla
        abs_err = abs(target_angle_deg)
        if abs_err < 15.0:
            linear = self._max_linear
        elif abs_err > 60.0:
            linear = self._min_linear
        else:
            linear = self._min_linear + (self._max_linear - self._min_linear) * (1.0 - (abs_err - 15.0) / 45.0)

        return linear, angular

    # =========================================================================
    # Yardımcı Fonksiyonlar
    # =========================================================================

    def _is_data_valid(self):
        now = self.get_clock().now()
        if not self._latest_costmap or not self._costmap_valid:
            return False
        if self._last_costmap_time and (now - self._last_costmap_time).nanoseconds / 1e9 > self._timeout_sec:
            return False
        if not self._imu_valid:
            return False
        if not self._target_valid or not self._mission_active:
            return False
        return True

    def _publish_stop(self):
        cmd = Twist()
        self._pub_cmd_vel.publish(cmd)

    def _publish_status(self, active, reason, target_angle=0.0, linear_speed=0.0, angular_speed=0.0, path_len=0):
        now = time.time()
        if now - self._last_status_time < 0.5:
            return
        self._last_status_time = now

        status = {
            'active': active,
            'reason': reason,
            'target_angle_deg': round(target_angle, 1),
            'linear_speed': round(linear_speed, 3),
            'angular_speed': round(angular_speed, 3),
            'path_node_count': path_len,
            'total_commands': self._total_commands,
            'total_stops': self._total_stops
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._pub_status.publish(msg)

    @staticmethod
    def _normalize_angle_180(angle_deg):
        return (angle_deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def _clamp(val, min_v, max_v):
        return max(min_v, min(max_v, val))

    @staticmethod
    def _quaternion_to_yaw_deg(msg: Imu):
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm < 1e-6:
            return None
        return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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
