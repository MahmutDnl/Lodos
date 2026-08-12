#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — VFH Karar Node (Karar & Engelden Kaçınma v4)
# =============================================================================
# Girişler:
#   - /albatros/costmap/grid   [nav_msgs/OccupancyGrid] (base_link)
#   - /albatros/costmap/valid  [std_msgs/Bool]
#   - /albatros/state          [albatros_interfaces/VehicleState]
#
# Çıkışlar:
#   - /albatros/command/cmd_vel   [geometry_msgs/Twist]
#   - /albatros/avoidance/status  [std_msgs/String JSON]
#   - /albatros/avoidance/path    [nav_msgs/Path] (base_link)
#   - /albatros/avoidance/markers [visualization_msgs/MarkerArray]
#
# ROS2 Jazzy / Ubuntu 24.04
# =============================================================================

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

from albatros_interfaces.msg import VehicleState


class ObstacleAvoidanceNode(Node):
    """
    VFH (Vector Field Histogram) tabanlı engel kaçınma node'u.
    Girdi olarak base_link frame'indeki yerel costmap'i alır.
    """

    def __init__(self):
        super().__init__('karar_node')

        # ─── Parametreler ───────────────────────────────────────────────────
        self.declare_parameter('sector_count',               72)
        self.declare_parameter('vfh_threshold',              0.35)
        self.declare_parameter('active_region_radius',       6.0)
        self.declare_parameter('vehicle_width',              0.85)
        self.declare_parameter('safety_margin',              0.50)
        self.declare_parameter('max_linear_speed',           1.0)
        self.declare_parameter('min_linear_speed',           0.2)
        self.declare_parameter('max_angular_speed',          0.8)
        self.declare_parameter('cost_goal_weight',           5.0)
        self.declare_parameter('cost_current_weight',        2.0)
        self.declare_parameter('cost_previous_weight',       2.0)
        self.declare_parameter('cost_clearance_weight',      3.0)
        self.declare_parameter('slowdown_distance',          2.0)
        self.declare_parameter('emergency_stop_distance_m',  1.0)
        self.declare_parameter('unknown_is_blocked',         False)
        self.declare_parameter('steering_kp',                1.5)
        self.declare_parameter('publish_rate',               10.0)
        self.declare_parameter('costmap_timeout_sec',        1.5)
        self.declare_parameter('state_timeout_sec',          2.0)

        self._sector_count       = int(self.get_parameter('sector_count').value)
        self._vfh_threshold      = float(self.get_parameter('vfh_threshold').value)
        self._active_radius      = float(self.get_parameter('active_region_radius').value)
        self._vehicle_width      = float(self.get_parameter('vehicle_width').value)
        self._safety_margin      = float(self.get_parameter('safety_margin').value)
        self._max_linear         = float(self.get_parameter('max_linear_speed').value)
        self._min_linear         = float(self.get_parameter('min_linear_speed').value)
        self._max_angular        = float(self.get_parameter('max_angular_speed').value)
        self._weight_goal        = float(self.get_parameter('cost_goal_weight').value)
        self._weight_current     = float(self.get_parameter('cost_current_weight').value)
        self._weight_previous    = float(self.get_parameter('cost_previous_weight').value)
        self._weight_clearance   = float(self.get_parameter('cost_clearance_weight').value)
        self._slowdown_dist      = float(self.get_parameter('slowdown_distance').value)
        self._emergency_dist     = float(self.get_parameter('emergency_stop_distance_m').value)
        self._unknown_blocked    = bool(self.get_parameter('unknown_is_blocked').value)
        self._steering_kp        = float(self.get_parameter('steering_kp').value)
        self._publish_rate       = float(self.get_parameter('publish_rate').value)
        self._costmap_timeout    = float(self.get_parameter('costmap_timeout_sec').value)
        self._state_timeout      = float(self.get_parameter('state_timeout_sec').value)

        self._sector_width       = 360.0 / self._sector_count

        # ─── Dahili Durum ───────────────────────────────────────────────────
        self._latest_costmap: OccupancyGrid = None
        self._last_costmap_time = None
        self._costmap_valid     = False

        self._latest_state: VehicleState = None
        self._last_state_time   = None

        self._prev_selected_sector = 0
        self._smoothed_angle_deg   = 0.0
        self._total_commands_sent = 0
        self._total_stops         = 0
        self._last_status_pub_time = 0.0

        # ─── Subscriber'lar ────────────────────────────────────────────────
        self._sub_costmap = self.create_subscription(
            OccupancyGrid, '/albatros/costmap/grid', self._cb_costmap, qos_profile_sensor_data)

        self._sub_costmap_valid = self.create_subscription(
            Bool, '/albatros/costmap/valid', self._cb_costmap_valid, 10)

        self._sub_state = self.create_subscription(
            VehicleState, '/albatros/state', self._cb_state, 10)

        # ─── Publisher'lar ─────────────────────────────────────────────────
        self._pub_cmd_vel = self.create_publisher(Twist,        '/albatros/command/cmd_vel', 10)
        self._pub_status  = self.create_publisher(String,       '/albatros/avoidance/status', 10)
        self._pub_path    = self.create_publisher(Path,         '/albatros/avoidance/path',   10)
        self._pub_markers = self.create_publisher(MarkerArray,  '/albatros/avoidance/markers', 10)

        # ─── Timer ─────────────────────────────────────────────────────────
        period = 1.0 / max(self._publish_rate, 0.1)
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('VFH Karar Node (v4) Başlatıldı.')
        self.get_logger().info(f'  Sektör Sayısı  : {self._sector_count} ({self._sector_width:.1f}°/sektör)')
        self.get_logger().info(f'  Acil Stop Eşiği: {self._emergency_dist} m')
        self.get_logger().info('=' * 60)

    # =========================================================================
    # Callback'ler
    # =========================================================================

    def _cb_costmap(self, msg: OccupancyGrid):
        self._latest_costmap = msg
        self._last_costmap_time = self.get_clock().now()

    def _cb_costmap_valid(self, msg: Bool):
        self._costmap_valid = msg.data

    def _cb_state(self, msg: VehicleState):
        self._latest_state = msg
        self._last_state_time = self.get_clock().now()

    # =========================================================================
    # Ana Kontrol Döngüsü
    # =========================================================================

    def _control_loop(self):
        valid, reason = self._check_data_validity()
        if not valid:
            self._publish_stop()
            self._publish_status(False, reason, 0.0, 0.0, 0.0)
            return

        # VFH Hesaplamaları
        histogram, min_dist_per_sector = self._build_polar_histogram()
        valleys = self._find_valleys(histogram)
        valid_valleys = self._filter_narrow_valleys(valleys, min_dist_per_sector)

        if not valid_valleys:
            # Geçerli vadi yoksa acil stop!
            self._publish_stop()
            self._publish_status(True, 'NO_VALID_VALLEY', 0.0, 0.0, 0.0)
            self._total_stops += 1
            return

        # Hedef Sektör (Body Frame)
        goal_heading_deg = self._latest_state.heading_error_deg
        goal_sector = self._angle_to_sector(goal_heading_deg)

        # En İyi Sektörü Seç
        selected_sector = self._select_best_sector(
            valid_valleys, goal_sector, min_dist_per_sector
        )

        selected_angle_deg = self._sector_to_angle(selected_sector)

        # Hysteresis / Yön Yumuşatma (EMA)
        diff = self._normalize_angle_180(selected_angle_deg - self._smoothed_angle_deg)
        self._smoothed_angle_deg = self._normalize_angle_180(
            self._smoothed_angle_deg + 0.35 * diff
        )

        # Seçilen Yöndeki En Yakın Engel Mesafesi
        nearest_obs_dist = self._find_nearest_obstacle_in_cone(
            selected_sector, min_dist_per_sector
        )

        # Acil Durdurma Kontrolü (Emergency Stop)
        if nearest_obs_dist <= self._emergency_dist:
            self._publish_stop()
            self._publish_status(
                True, f'EMERGENCY_OBSTACLE_TOO_CLOSE ({nearest_obs_dist:.2f}m)',
                self._smoothed_angle_deg, 0.0, 0.0
            )
            self._total_stops += 1
            return

        # Hız Komutları Hesapla (P Kontrol)
        linear_speed, angular_speed = self._calculate_speeds(
            self._smoothed_angle_deg, nearest_obs_dist
        )

        # Twist Komutu Yayınla
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self._pub_cmd_vel.publish(cmd)
        self._total_commands_sent += 1
        self._prev_selected_sector = selected_sector

        # Görselleştirmeleri Yayınla
        self._publish_path(self._smoothed_angle_deg, linear_speed)
        self._publish_avoidance_markers(
            self._smoothed_angle_deg, goal_heading_deg, nearest_obs_dist
        )

        self._publish_status(
            True, 'OK', self._smoothed_angle_deg, linear_speed, angular_speed,
            goal_sector=goal_sector, selected_sector=selected_sector,
            nearest_obstacle=nearest_obs_dist, valleys=valleys, valid_valleys=valid_valleys
        )

    # =========================================================================
    # VFH Algoritma Adımları
    # =========================================================================

    def _build_polar_histogram(self):
        histogram = [0.0] * self._sector_count
        min_dist_per_sector = [self._active_radius] * self._sector_count

        costmap = self._latest_costmap
        res = costmap.info.resolution
        w = costmap.info.width
        h = costmap.info.height
        ox = costmap.info.origin.position.x
        oy = costmap.info.origin.position.y

        data = costmap.data
        if not data:
            return histogram, min_dist_per_sector

        for r in range(h):
            for c in range(w):
                idx = r * w + c
                cell_val = data[idx]

                if cell_val == 0:
                    continue

                if cell_val < 0:
                    if not self._unknown_blocked:
                        continue
                    cost_factor = 0.5
                else:
                    cost_factor = cell_val / 100.0

                x_m = ox + (c + 0.5) * res
                y_m = oy + (r + 0.5) * res

                dist = math.hypot(x_m, y_m)
                if dist < 0.05 or dist > self._active_radius:
                    continue

                angle_deg = math.degrees(math.atan2(y_m, x_m))
                sector = self._angle_to_sector(angle_deg)

                if dist < min_dist_per_sector[sector]:
                    min_dist_per_sector[sector] = dist

                c_weight = cost_factor * cost_factor
                d_weight = (self._active_radius - dist) / self._active_radius
                d_weight = max(0.0, d_weight * d_weight)

                histogram[sector] += c_weight * d_weight

        # Normalizasyon
        max_val = max(histogram) if histogram else 0.0
        if max_val > 0.0:
            histogram = [h / max_val for h in histogram]

        return histogram, min_dist_per_sector

    def _find_valleys(self, histogram):
        valleys = []
        in_valley = False
        start_sector = 0

        for i in range(self._sector_count):
            if histogram[i] < self._vfh_threshold:
                if not in_valley:
                    in_valley = True
                    start_sector = i
            else:
                if in_valley:
                    in_valley = False
                    valleys.append((start_sector, i - 1))

        if in_valley:
            valleys.append((start_sector, self._sector_count - 1))

        # Dairesel sarma kontrolü (360° etrafında)
        if len(valleys) > 1:
            first_start, first_end = valleys[0]
            last_start, last_end = valleys[-1]
            if last_end == self._sector_count - 1 and first_start == 0:
                merged_valley = (last_start, first_end)
                valleys = valleys[1:-1]
                valleys.append(merged_valley)

        return valleys

    def _filter_narrow_valleys(self, valleys, min_dist_per_sector):
        valid_valleys = []
        required_width = self._vehicle_width + 2.0 * self._safety_margin

        for v_start, v_end in valleys:
            if v_start <= v_end:
                sector_span = v_end - v_start + 1
            else:
                sector_span = (self._sector_count - v_start) + v_end + 1

            valley_min_dist = self._active_radius
            curr = v_start
            for _ in range(sector_span):
                if min_dist_per_sector[curr] < valley_min_dist:
                    valley_min_dist = min_dist_per_sector[curr]
                curr = (curr + 1) % self._sector_count

            chord_width = 2.0 * valley_min_dist * math.sin(
                math.radians(sector_span * self._sector_width / 2.0)
            )

            if chord_width >= required_width:
                valid_valleys.append((v_start, v_end))

        return valid_valleys

    def _select_best_sector(self, valid_valleys, goal_sector, min_dist_per_sector):
        candidate_sectors = []

        for v_start, v_end in valid_valleys:
            if v_start <= v_end:
                sector_span = v_end - v_start + 1
                valley_sectors = list(range(v_start, v_end + 1))
            else:
                sector_span = (self._sector_count - v_start) + v_end + 1
                valley_sectors = (
                    list(range(v_start, self._sector_count)) + list(range(0, v_end + 1))
                )

            # Koridor Merkez Sektörü (Clearance Maximization)
            mid_idx = len(valley_sectors) // 2
            candidate_sectors.append(valley_sectors[mid_idx])

            if goal_sector in valley_sectors:
                candidate_sectors.append(goal_sector)
            else:
                candidate_sectors.append(valley_sectors[0])
                candidate_sectors.append(valley_sectors[-1])

        # En İyi Sektörü Cost Fonksiyonu ile Seç
        best_sector = candidate_sectors[0]
        best_cost = float('inf')

        for sector in candidate_sectors:
            cost_goal = self._sector_distance(sector, goal_sector)
            cost_current = self._sector_distance(sector, 0)
            cost_previous = self._sector_distance(sector, self._prev_selected_sector)

            # Clearance Maliyeti (Engellere uzak olan sektör daha ucuz)
            clearance_dist = min_dist_per_sector[sector]
            cost_clearance = max(0.0, self._active_radius - clearance_dist)

            total_cost = (
                self._weight_goal * cost_goal +
                self._weight_current * cost_current +
                self._weight_previous * cost_previous +
                self._weight_clearance * cost_clearance
            )

            if total_cost < best_cost:
                best_cost = total_cost
                best_sector = sector

        return best_sector

    def _calculate_speeds(self, selected_angle_deg, nearest_obstacle_dist):
        angle_error_deg = self._normalize_angle_180(selected_angle_deg)
        angle_error_rad = math.radians(angle_error_deg)

        angular_speed = self._steering_kp * angle_error_rad
        angular_speed = self._clamp(angular_speed, -self._max_angular, self._max_angular)

        if nearest_obstacle_dist <= 0.01:
            dist_factor = 0.0
        elif nearest_obstacle_dist >= self._slowdown_dist:
            dist_factor = 1.0
        else:
            dist_factor = nearest_obstacle_dist / self._slowdown_dist

        abs_error = abs(angle_error_deg)
        if abs_error <= 15.0:
            turn_factor = 1.0
        elif abs_error >= 90.0:
            turn_factor = 0.3
        else:
            turn_factor = 1.0 - 0.7 * ((abs_error - 15.0) / 75.0)

        combined_factor = min(dist_factor, turn_factor)
        linear_speed = self._min_linear + (self._max_linear - self._min_linear) * combined_factor

        return linear_speed, angular_speed

    def _find_nearest_obstacle_in_cone(self, selected_sector, min_dist_per_sector):
        nearest = self._active_radius
        cone_half = 2  # ±2 sektör (±10°)
        for offset in range(-cone_half, cone_half + 1):
            idx = (selected_sector + offset) % self._sector_count
            if min_dist_per_sector[idx] < nearest:
                nearest = min_dist_per_sector[idx]
        return nearest

    # =========================================================================
    # Yayın Yardımcıları & Görselleştirme
    # =========================================================================

    def _publish_path(self, selected_angle_deg: float, linear_speed: float):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'base_link'

        # 4 metrelik güvenli rotayı 8 adımla çiz
        angle_rad = math.radians(selected_angle_deg)
        num_points = 8
        step_len = 0.50  # 4m toplam

        for i in range(num_points + 1):
            d = i * step_len
            ps = PoseStamped()
            ps.header.stamp = path.header.stamp
            ps.header.frame_id = 'base_link'
            ps.pose.position.x = d * math.cos(angle_rad)
            ps.pose.position.y = d * math.sin(angle_rad)
            ps.pose.position.z = 0.05
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self._pub_path.publish(path)

    def _publish_avoidance_markers(self, selected_angle_deg: float, goal_angle_deg: float, nearest_obs_dist: float):
        ma = MarkerArray()
        now_stamp = self.get_clock().now().to_msg()

        # 1) Seçilen Yön Ok Marker'ı (Yeşil ARROW)
        sm = Marker()
        sm.header.stamp = now_stamp
        sm.header.frame_id = 'base_link'
        sm.ns = 'vfh_selected_direction'
        sm.id = 100
        sm.type = Marker.ARROW
        sm.action = Marker.ADD
        sm.scale.x = 2.5  # Ok boyu
        sm.scale.y = 0.25
        sm.scale.z = 0.25
        sm.color.r, sm.color.g, sm.color.b, sm.color.a = (0.0, 1.0, 0.2, 0.95)

        s_rad = math.radians(selected_angle_deg)
        cy = math.cos(s_rad * 0.5)
        sy = math.sin(s_rad * 0.5)
        sm.pose.orientation.z = sy
        sm.pose.orientation.w = cy
        ma.markers.append(sm)

        # 2) Hedef Yön Ok Marker'ı (Mavi ARROW)
        gm = Marker()
        gm.header.stamp = now_stamp
        gm.header.frame_id = 'base_link'
        gm.ns = 'vfh_goal_direction'
        gm.id = 101
        gm.type = Marker.ARROW
        gm.action = Marker.ADD
        gm.scale.x = 2.0
        gm.scale.y = 0.15
        gm.scale.z = 0.15
        gm.color.r, gm.color.g, gm.color.b, gm.color.a = (0.2, 0.5, 1.0, 0.70)

        g_rad = math.radians(goal_angle_deg)
        gm.pose.orientation.z = math.sin(g_rad * 0.5)
        gm.pose.orientation.w = math.cos(g_rad * 0.5)
        ma.markers.append(gm)

        # 3) En Yakın Engel Küresi Marker'ı (Kırmızı SPHERE)
        if nearest_obs_dist < self._active_radius:
            om = Marker()
            om.header.stamp = now_stamp
            om.header.frame_id = 'base_link'
            om.ns = 'vfh_nearest_obstacle'
            om.id = 102
            om.type = Marker.SPHERE
            om.action = Marker.ADD
            om.pose.position.x = nearest_obs_dist * math.cos(s_rad)
            om.pose.position.y = nearest_obs_dist * math.sin(s_rad)
            om.pose.position.z = 0.10
            om.scale.x = 0.4
            om.scale.y = 0.4
            om.scale.z = 0.4
            om.color.r, om.color.g, om.color.b, om.color.a = (1.0, 0.0, 0.0, 0.90)
            ma.markers.append(om)

        self._pub_markers.publish(ma)

    def _publish_stop(self):
        cmd = Twist()
        self._pub_cmd_vel.publish(cmd)

    def _publish_status(self, active, reason, selected_angle_deg, linear_speed, angular_speed, **kwargs):
        now = time.time()
        if now - self._last_status_pub_time < 0.5:
            return
        self._last_status_pub_time = now

        status = {
            'active': active,
            'reason': reason,
            'selected_angle_deg': round(selected_angle_deg, 1),
            'linear_speed': round(linear_speed, 3),
            'angular_speed': round(angular_speed, 3),
            'total_commands': self._total_commands_sent,
            'total_stops': self._total_stops,
        }

        if self._latest_state is not None:
            st = self._latest_state
            status['heading_error_deg'] = round(st.heading_error_deg, 1)
            status['turn_direction'] = st.turn_direction
            status['current_yaw_deg'] = round(st.current_yaw_deg, 1)

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._pub_status.publish(msg)

    # =========================================================================
    # Kontroller ve Yardımcılar
    # =========================================================================

    def _check_data_validity(self):
        now = self.get_clock().now()

        if self._latest_state is None:
            return False, 'STATE_NOT_RECEIVED'

        if self._last_state_time is not None:
            state_age = (now - self._last_state_time).nanoseconds / 1e9
            if state_age > self._state_timeout:
                return False, f'STATE_STALE ({state_age:.1f}s)'

        state = self._latest_state

        if self._latest_costmap is None:
            return False, 'COSTMAP_NOT_RECEIVED'

        if self._last_costmap_time is not None:
            costmap_age = (now - self._last_costmap_time).nanoseconds / 1e9
            if costmap_age > self._costmap_timeout:
                return False, f'COSTMAP_STALE ({costmap_age:.1f}s)'

        if not self._costmap_valid:
            return False, 'COSTMAP_INVALID'

        if not state.mission_active:
            return False, 'MISSION_NOT_ACTIVE'

        if not state.target_valid:
            return False, 'TARGET_INVALID'

        if state.emergency_stop:
            return False, 'EMERGENCY_STOP_ACTIVE'

        if not state.control_allowed:
            return False, 'CONTROL_NOT_ALLOWED'

        return True, 'OK'

    def _angle_to_sector(self, angle_deg):
        normalized = angle_deg % 360.0
        return int(normalized / self._sector_width) % self._sector_count

    def _sector_to_angle(self, sector):
        angle = (sector + 0.5) * self._sector_width
        return self._normalize_angle_180(angle)

    def _sector_distance(self, sector_a, sector_b):
        diff = abs(sector_a - sector_b)
        return min(diff, self._sector_count - diff)

    @staticmethod
    def _normalize_angle_180(angle_deg):
        return (angle_deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def _clamp(val, min_v, max_v):
        return max(min_v, min(max_v, val))


# ---------------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('VFH Karar Node durduruldu.')
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
