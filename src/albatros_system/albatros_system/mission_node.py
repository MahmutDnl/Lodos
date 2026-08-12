#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros ROS 2 Mission Node.
Manages mission status, waypoint tracking, and Parkur 3 sub-states (SCAN/APPROACH/TOUCH)
for the TEKNOFEST 2026 Albatros IDA.
"""

import math
import time
import json
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from mavros_msgs.msg import WaypointList, WaypointReached
from mavros_msgs.srv import WaypointPull
from albatros_interfaces.msg import MissionStatus, MissionTarget

try:
    from albatros_interfaces.msg import VehicleState
except ImportError:
    VehicleState = None


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # --- Parameters ---
        self.declare_parameter('gps_timeout_sec', 2.0)
        self.declare_parameter('waypoint_reached_radius_m', 2.5)
        self.declare_parameter('required_reached_samples', 3)
        self.declare_parameter('mission_pull_retry_period_sec', 5.0)
        self.declare_parameter('publish_period_sec', 0.2)  # 5 Hz status/target publishing
        self.declare_parameter('warning_throttle_sec', 5.0)
        self.declare_parameter('parkur_1_start_wp', 1)
        self.declare_parameter('parkur_2_start_wp', 4)
        self.declare_parameter('parkur_3_start_wp', 7)
        self.declare_parameter('auto_pull_mission_on_startup', True)

        # Parkur 3 Scan & Approach Parameters
        self.declare_parameter('scan_settle_time_sec', 0.5)
        self.declare_parameter('yaw_tolerance_deg', 3.0)
        self.declare_parameter('target_lost_timeout_sec', 2.0)
        self.declare_parameter('touch_distance_threshold_m', 1.5)

        self.gps_timeout_sec = float(self.get_parameter('gps_timeout_sec').value)
        self.waypoint_reached_radius_m = float(self.get_parameter('waypoint_reached_radius_m').value)
        self.required_reached_samples = int(self.get_parameter('required_reached_samples').value)
        self.mission_pull_retry_period_sec = float(self.get_parameter('mission_pull_retry_period_sec').value)
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.warning_throttle_sec = float(self.get_parameter('warning_throttle_sec').value)
        self.parkur_1_start_wp = int(self.get_parameter('parkur_1_start_wp').value)
        self.parkur_2_start_wp = int(self.get_parameter('parkur_2_start_wp').value)
        self.parkur_3_start_wp = int(self.get_parameter('parkur_3_start_wp').value)
        self.auto_pull_mission_on_startup = bool(self.get_parameter('auto_pull_mission_on_startup').value)

        self.scan_settle_time_sec = float(self.get_parameter('scan_settle_time_sec').value)
        self.yaw_tolerance_deg = float(self.get_parameter('yaw_tolerance_deg').value)
        self.target_lost_timeout_sec = float(self.get_parameter('target_lost_timeout_sec').value)
        self.touch_distance_threshold_m = float(self.get_parameter('touch_distance_threshold_m').value)

        # --- Internal State variables ---
        self.mission_state = "WAITING_FOR_MAVROS"
        self.error_code = "NONE"

        # Control Node Health Status
        self.control_connected = False
        self.control_state_ok = False
        self.control_gps_ok = False
        self.last_control_status_time = 0.0

        # Vehicle Pose/Yaw
        self.current_yaw_deg = 0.0

        # GPS Data
        self.gps_received = False
        self.gps_valid = False
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.last_gps_time = 0.0

        # Waypoints storage
        self.nav_waypoints: List[Dict] = []
        self.active_waypoint: Optional[Dict] = None
        self.total_navigation_waypoints = 0
        self.reached_waypoint_count = 0
        self.last_reached_seq = -1

        # Reached checking
        self.reached_samples_counter = 0
        self.distance_to_target = 0.0
        self.target_bearing = 0.0

        # Flags for publisher outputs
        self.mission_loaded = False
        self.mission_active = False
        self.mission_completed = False
        self.target_valid = False
        self.target_reached = False
        self.current_parkur = MissionStatus.PARKUR_UNKNOWN

        # Parkur 3 Sub-state tracking
        self.scan_angles_deg = [-35.0, -20.0, 0.0, 20.0, 35.0, 20.0, 0.0, -20.0]
        self.scan_index = 0
        self.scan_reference_yaw: Optional[float] = None
        self.scan_settle_start_time: Optional[float] = None
        self.parkur3_substate = "SCAN"

        self.p3_target_confirmed = False
        self.p3_target_angle_deg = 0.0
        self.p3_last_confirmed_time = 0.0

        self.warn_timestamps: Dict[str, float] = {}

        self.pull_in_progress = False
        self.last_pull_attempt_time = 0.0

        # --- Subscriptions ---
        self.create_subscription(
            NavSatFix,
            '/albatros/gps/fix',
            self.gps_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            String,
            '/albatros/control/status',
            self.control_status_callback,
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
            String,
            '/albatros/parkur3/target_confirmed',
            self.parkur3_target_callback,
            10
        )

        if VehicleState is not None:
            self.create_subscription(
                VehicleState,
                '/albatros/state',
                self.vehicle_state_callback,
                10
            )

        # --- Service Clients ---
        self.pull_client = self.create_client(
            WaypointPull,
            '/mavros/mission/pull'
        )

        # --- Publishers ---
        self.status_publisher = self.create_publisher(
            MissionStatus,
            '/albatros/mission/status',
            10
        )

        self.target_publisher = self.create_publisher(
            MissionTarget,
            '/albatros/mission/target',
            10
        )

        # --- Main Timer Loop ---
        self.main_timer = self.create_timer(
            self.publish_period_sec,
            self.main_loop
        )

        self.get_logger().info("Mission Node with Parkur 3 Sub-states initialized successfully.")

    def gps_callback(self, msg: NavSatFix):
        self.last_gps_time = time.monotonic()
        self.gps_received = True

        is_fix_ok = msg.status.status >= NavSatStatus.STATUS_FIX
        coords_finite = math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
        coords_in_range = (-90.0 <= msg.latitude <= 90.0) and (-180.0 <= msg.longitude <= 180.0)

        self.gps_valid = is_fix_ok and coords_finite and coords_in_range

        if self.gps_valid:
            self.current_latitude = msg.latitude
            self.current_longitude = msg.longitude
        else:
            self.warn_throttled("Received invalid GPS coordinates or no fix.", "gps_invalid_msg")

    def control_status_callback(self, msg: String):
        self.last_control_status_time = time.monotonic()
        try:
            status_data = json.loads(msg.data)
            self.control_connected = bool(status_data.get('connected', False))
            self.control_state_ok = bool(status_data.get('state_ok', False))
            self.control_gps_ok = bool(status_data.get('gps_ok', False))
        except Exception as e:
            self.warn_throttled(f"Failed to parse Control Node status JSON: {e}", "control_json_parse_err")
            self.control_connected = False
            self.control_state_ok = False
            self.control_gps_ok = False

    def vehicle_state_callback(self, msg):
        self.current_yaw_deg = float(getattr(msg, "current_yaw_deg", 0.0))

    def parkur3_target_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            confirmed = bool(data.get("target_confirmed", False))
            angle = float(data.get("target_angle_deg", 0.0))

            if confirmed:
                self.p3_target_confirmed = True
                self.p3_target_angle_deg = angle
                self.p3_last_confirmed_time = time.monotonic()
            else:
                if time.monotonic() - self.p3_last_confirmed_time > self.target_lost_timeout_sec:
                    self.p3_target_confirmed = False
        except Exception as e:
            self.warn_throttled(f"Failed to parse Parkur 3 target feedback JSON: {e}", "p3_target_parse_err")

    def waypoint_list_callback(self, msg: WaypointList):
        self.get_logger().info(f"Received waypoint list containing {len(msg.waypoints)} items.")

        new_nav_waypoints = []
        for i, wp in enumerate(msg.waypoints):
            seq = i
            if wp.command == 16:
                if seq == 0:
                    continue
                new_nav_waypoints.append({
                    'seq': seq,
                    'latitude': wp.x_lat,
                    'longitude': wp.y_long,
                    'altitude': wp.z_alt,
                    'command': wp.command,
                    'is_current': wp.is_current,
                    'autocontinue': wp.autocontinue
                })

        self.nav_waypoints = new_nav_waypoints
        self.total_navigation_waypoints = len(self.nav_waypoints)
        self.mission_loaded = (self.total_navigation_waypoints > 0)

        if not self.mission_loaded:
            self.set_error("MISSION_LIST_EMPTY")
            self.active_waypoint = None
            self.target_valid = False
            return

        if self.error_code in ("MISSION_LIST_EMPTY", "NO_VALID_WAYPOINT"):
            self.clear_error()

        self.update_active_target(msg.current_seq)

    def waypoint_reached_callback(self, msg: WaypointReached):
        wp_seq = int(msg.wp_seq)
        if self.active_waypoint is None:
            return

        if wp_seq == self.active_waypoint['seq']:
            if wp_seq != self.last_reached_seq:
                self.last_reached_seq = wp_seq
                self.confirm_waypoint_reached(wp_seq)

    def request_mission_pull(self):
        if self.pull_in_progress:
            return
        if not self.pull_client.service_is_ready():
            self.set_error("MAVROS_UNAVAILABLE")
            return

        self.pull_in_progress = True
        request = WaypointPull.Request()
        future = self.pull_client.call_async(request)
        future.add_done_callback(self.pull_response_callback)

    def pull_response_callback(self, future):
        self.pull_in_progress = False
        try:
            response = future.result()
            if response.success and self.error_code == "MISSION_PULL_FAILED":
                self.clear_error()
        except Exception:
            self.set_error("MISSION_PULL_FAILED")

    def update_active_target(self, current_seq: int):
        if not self.nav_waypoints:
            self.active_waypoint = None
            self.target_valid = False
            return

        target_wp = None
        for wp in self.nav_waypoints:
            if wp['seq'] == current_seq:
                target_wp = wp
                break

        if target_wp is None:
            for wp in self.nav_waypoints:
                if wp['seq'] > current_seq:
                    target_wp = wp
                    break

        if target_wp is None:
            target_wp = self.nav_waypoints[0]

        if self.active_waypoint is None or self.active_waypoint['seq'] != target_wp['seq']:
            self.reached_samples_counter = 0

        self.active_waypoint = target_wp
        self.target_valid = True

    def validate_gps(self) -> bool:
        if not self.gps_received or not self.gps_valid:
            self.set_error("GPS_INVALID")
            return False

        now = time.monotonic()
        if now - self.last_gps_time > self.gps_timeout_sec:
            self.set_error("GPS_STALE")
            return False

        if not self.control_gps_ok or not self.control_connected or not self.control_state_ok:
            self.set_error("CONTROL_STATUS_INVALID")
            return False

        if self.error_code in ("GPS_NOT_RECEIVED", "GPS_INVALID", "GPS_STALE", "CONTROL_STATUS_INVALID", "MAVROS_UNAVAILABLE"):
            self.clear_error()

        return True

    def main_loop(self):
        now = time.monotonic()

        if not self.nav_waypoints and not self.pull_in_progress:
            time_since_attempt = now - self.last_pull_attempt_time
            if (self.last_pull_attempt_time == 0.0 and self.auto_pull_mission_on_startup) or \
               (time_since_attempt >= self.mission_pull_retry_period_sec):
                self.last_pull_attempt_time = now
                self.request_mission_pull()

        self.update_state_machine()

        if self.mission_state.startswith("RUNNING") or self.mission_state.startswith("PARKUR3"):
            if self.active_waypoint is not None:
                self.distance_to_target = self.calculate_distance_m(
                    self.current_latitude,
                    self.current_longitude,
                    self.active_waypoint['latitude'],
                    self.active_waypoint['longitude']
                )

                self.target_bearing = self.calculate_initial_bearing_deg(
                    self.current_latitude,
                    self.current_longitude,
                    self.active_waypoint['latitude'],
                    self.active_waypoint['longitude']
                )

                self.check_gps_reached()

        self.current_parkur = self.determine_current_parkur()

        if self.current_parkur == MissionStatus.PARKUR_3 and self.mission_state != "COMPLETED":
            self.process_parkur3_substates()

        self.publish_status()
        self.publish_target()

    def process_parkur3_substates(self):
        now = time.monotonic()

        if self.scan_reference_yaw is None:
            self.scan_reference_yaw = self.current_yaw_deg

        if self.parkur3_substate == "SCAN":
            self.mission_state = "PARKUR3_SCAN"

            if self.p3_target_confirmed:
                self.parkur3_substate = "APPROACH"
                self.get_logger().info("[PARKUR 3] Target confirmed! Transitioning from SCAN -> APPROACH")
                return

            rel_angle = self.scan_angles_deg[self.scan_index]
            desired_scan_yaw = (self.scan_reference_yaw + rel_angle + 360.0) % 360.0
            self.target_bearing = desired_scan_yaw

            yaw_error = abs((desired_scan_yaw - self.current_yaw_deg + 180.0) % 360.0 - 180.0)
            if yaw_error <= self.yaw_tolerance_deg:
                if self.scan_settle_start_time is None:
                    self.scan_settle_start_time = now
                elif now - self.scan_settle_start_time >= self.scan_settle_time_sec:
                    self.scan_index = (self.scan_index + 1) % len(self.scan_angles_deg)
                    self.scan_settle_start_time = None
            else:
                self.scan_settle_start_time = None

        elif self.parkur3_substate == "APPROACH":
            self.mission_state = "PARKUR3_APPROACH"

            if not self.p3_target_confirmed and (now - self.p3_last_confirmed_time > self.target_lost_timeout_sec):
                self.parkur3_substate = "SCAN"
                self.scan_reference_yaw = self.current_yaw_deg
                self.get_logger().warn("[PARKUR 3] Target lost for > 2.0s! Reverting from APPROACH -> SCAN")
                return

            target_heading = (self.current_yaw_deg + self.p3_target_angle_deg + 360.0) % 360.0
            self.target_bearing = target_heading

            if self.distance_to_target <= self.touch_distance_threshold_m:
                self.parkur3_substate = "TOUCH"
                self.get_logger().info("[PARKUR 3] Target reached! Transitioning APPROACH -> TOUCH")

        elif self.parkur3_substate == "TOUCH":
            self.mission_state = "PARKUR3_TOUCH"
            self.confirm_waypoint_reached(self.active_waypoint['seq'] if self.active_waypoint else 7)

    def update_state_machine(self):
        if self.mission_state == "COMPLETED":
            return

        if self.error_code != "NONE" and self.error_code not in ("NONE", "MISSION_PULL_FAILED"):
            self.mission_state = "ERROR"
            self.mission_active = False
            return

        if not self.control_connected or (time.monotonic() - self.last_control_status_time > self.gps_timeout_sec):
            self.mission_state = "WAITING_FOR_MAVROS"
            self.mission_active = False
            return

        if not self.nav_waypoints:
            self.mission_state = "WAITING_FOR_MISSION"
            self.mission_active = False
            return

        if not self.validate_gps():
            self.mission_state = "WAITING_FOR_GPS"
            self.mission_active = False
            return

        if self.current_parkur != MissionStatus.PARKUR_3:
            self.mission_state = "RUNNING"

        self.mission_active = True

    def check_gps_reached(self):
        if self.active_waypoint is None:
            return

        if self.distance_to_target <= self.waypoint_reached_radius_m:
            self.reached_samples_counter += 1
            self.target_reached = True

            if self.reached_samples_counter >= self.required_reached_samples:
                seq = self.active_waypoint['seq']
                self.last_reached_seq = seq
                self.confirm_waypoint_reached(seq)
        else:
            self.reached_samples_counter = 0
            self.target_reached = False

    def confirm_waypoint_reached(self, reached_seq: int):
        if self.active_waypoint is None or reached_seq != self.active_waypoint['seq']:
            return

        self.reached_waypoint_count += 1
        self.reached_samples_counter = 0
        self.target_reached = False

        self.advance_to_next_waypoint()

    def advance_to_next_waypoint(self):
        if not self.nav_waypoints:
            return

        current_idx = -1
        for i, wp in enumerate(self.nav_waypoints):
            if wp['seq'] == self.active_waypoint['seq']:
                current_idx = i
                break

        if current_idx != -1 and current_idx + 1 < len(self.nav_waypoints):
            self.active_waypoint = self.nav_waypoints[current_idx + 1]
            self.target_valid = True
            if not (math.isfinite(self.active_waypoint['latitude']) and math.isfinite(self.active_waypoint['longitude'])):
                self.set_error("ACTIVE_WAYPOINT_INVALID")
                self.target_valid = False
        else:
            self.active_waypoint = None
            self.mission_state = "COMPLETED"
            self.mission_active = False
            self.mission_completed = True
            self.target_valid = False
            self.current_parkur = MissionStatus.PARKUR_COMPLETE

    def determine_current_parkur(self) -> int:
        if self.mission_completed:
            return MissionStatus.PARKUR_COMPLETE

        if self.active_waypoint is None:
            return MissionStatus.PARKUR_UNKNOWN

        seq = self.active_waypoint['seq']

        if seq >= self.parkur_3_start_wp:
            return MissionStatus.PARKUR_3
        elif seq >= self.parkur_2_start_wp:
            return MissionStatus.PARKUR_2
        elif seq >= self.parkur_1_start_wp:
            return MissionStatus.PARKUR_1
        else:
            return MissionStatus.PARKUR_UNKNOWN

    @staticmethod
    def calculate_distance_m(current_lat: float, current_lon: float, target_lat: float, target_lon: float) -> float:
        if math.isclose(current_lat, target_lat) and math.isclose(current_lon, target_lon):
            return 0.0

        r_earth = 6371000.0
        lat1 = math.radians(current_lat)
        lon1 = math.radians(current_lon)
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_lon)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = math.sin(dlat / 2.0)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0)**2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        distance = r_earth * c

        if not math.isfinite(distance) or distance < 0.0:
            return 0.0

        return distance

    @staticmethod
    def calculate_initial_bearing_deg(current_lat: float, current_lon: float, target_lat: float, target_lon: float) -> float:
        if math.isclose(current_lat, target_lat) and math.isclose(current_lon, target_lon):
            return 0.0

        lat1 = math.radians(current_lat)
        lat2 = math.radians(target_lat)
        delta_lon = math.radians(target_lon - current_lon)

        x = math.sin(delta_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)

        bearing = math.degrees(math.atan2(x, y))
        target_bearing_deg = (bearing + 360.0) % 360.0

        if not math.isfinite(target_bearing_deg):
            return 0.0

        return target_bearing_deg

    def set_error(self, code: str):
        if self.error_code != code:
            self.error_code = code
            self.get_logger().error(f"Mission Node entered error state: {code}")

    def clear_error(self):
        if self.error_code != "NONE":
            self.get_logger().info(f"Error cleared: {self.error_code}")
            self.error_code = "NONE"

    def warn_throttled(self, msg: str, key: str):
        now = time.monotonic()
        last_time = self.warn_timestamps.get(key, 0.0)
        if now - last_time >= self.warning_throttle_sec:
            self.get_logger().warn(msg)
            self.warn_timestamps[key] = now

    def publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.mission_state = self.mission_state
        msg.mission_loaded = self.mission_loaded
        msg.mission_active = self.mission_active
        msg.mission_completed = self.mission_completed
        msg.mission_error = (self.error_code != "NONE")
        msg.error_code = self.error_code

        msg.current_parkur = self.current_parkur
        msg.current_waypoint_seq = self.active_waypoint['seq'] if self.active_waypoint else -1
        msg.total_navigation_waypoints = self.total_navigation_waypoints
        msg.reached_waypoint_count = self.reached_waypoint_count

        msg.target_valid = self.target_valid
        msg.target_reached = self.target_reached

        if self.active_waypoint:
            msg.target_latitude = self.active_waypoint['latitude']
            msg.target_longitude = self.active_waypoint['longitude']
            msg.distance_to_target_m = self.distance_to_target
            msg.target_bearing_deg = self.target_bearing
        else:
            msg.target_latitude = 0.0
            msg.target_longitude = 0.0
            msg.distance_to_target_m = 0.0
            msg.target_bearing_deg = 0.0

        msg.gps_valid = self.gps_valid
        msg.mavros_connected = self.control_connected

        self.status_publisher.publish(msg)

    def publish_target(self):
        msg = MissionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.target_valid = self.target_valid
        msg.mission_active = self.mission_active
        msg.current_parkur = self.current_parkur
        msg.waypoint_seq = self.active_waypoint['seq'] if self.active_waypoint else -1

        if self.active_waypoint:
            msg.target_latitude = self.active_waypoint['latitude']
            msg.target_longitude = self.active_waypoint['longitude']
            msg.distance_to_target_m = self.distance_to_target
            msg.target_bearing_deg = self.target_bearing
        else:
            msg.target_latitude = 0.0
            msg.target_longitude = 0.0
            msg.distance_to_target_m = 0.0
            msg.target_bearing_deg = 0.0

        self.target_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
