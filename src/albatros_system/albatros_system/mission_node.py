#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros ROS 2 Mission Node.
Manages mission status and waypoint tracking for the TEKNOFEST 2026 Albatros IDA.
This node is purely for waypoint tracking, distance/bearing calculations, and mission status management.
It DOES NOT produce velocity/motor commands, handle arm/disarm, change control modes, or execute avoidance maneuvers.
"""

import math
import time
import json
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from mavros_msgs.msg import WaypointList, WaypointReached
from mavros_msgs.srv import WaypointPull
from albatros_interfaces.msg import MissionStatus, MissionTarget


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # --- Parameters ---
        self.declare_parameter('gps_timeout_sec', 2.0)
        self.declare_parameter('waypoint_reached_radius_m', 2.5)
        self.declare_parameter('required_reached_samples', 3)
        self.declare_parameter('mission_pull_retry_period_sec', 5.0)
        self.declare_parameter('publish_period_sec', 0.2)  # 5 Hz default status/target publishing
        self.declare_parameter('warning_throttle_sec', 5.0)
        self.declare_parameter('parkur_1_start_wp', 1)
        self.declare_parameter('parkur_2_start_wp', 4)
        self.declare_parameter('parkur_3_start_wp', 7)
        self.declare_parameter('auto_pull_mission_on_startup', True)

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

        # --- Internal State variables ---
        self.mission_state = "WAITING_FOR_MAVROS"  # WAITING_FOR_MAVROS, WAITING_FOR_MISSION, WAITING_FOR_GPS, RUNNING, COMPLETED, ERROR
        self.error_code = "NONE"

        # Control Node Health Status
        self.control_connected = False
        self.control_state_ok = False
        self.control_gps_ok = False
        self.last_control_status_time = 0.0

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

        # Throttled warnings
        self.warn_timestamps: Dict[str, float] = {}

        # MAVROS Pull state
        self.pull_in_progress = False
        self.last_pull_attempt_time = 0.0

        # --- Subscriptions ---
        # 1. GPS callback (Sensor Data QoS)
        self.create_subscription(
            NavSatFix,
            '/albatros/gps/fix',
            self.gps_callback,
            qos_profile_sensor_data
        )

        # 2. Control status callback (Standard QoS 10)
        self.create_subscription(
            String,
            '/albatros/control/status',
            self.control_status_callback,
            10
        )

        # 3. MAVROS Waypoint list callback
        self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoint_list_callback,
            10
        )

        # 4. MAVROS Waypoint reached callback
        self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
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

        self.get_logger().info("Mission Node initialized successfully.")

    # --- Callbacks ---

    def gps_callback(self, msg: NavSatFix):
        """Processes raw GPS fixes and updates vehicle location."""
        self.last_gps_time = time.monotonic()
        self.gps_received = True

        # Validation: fix status, coordinates are finite and bounded
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
        """Parses health and status JSON from the Control Node."""
        self.last_control_status_time = time.monotonic()
        try:
            status_data = json.loads(msg.data)
            self.control_connected = bool(status_data.get('connected', False))
            self.control_state_ok = bool(status_data.get('state_ok', False))
            self.control_gps_ok = bool(status_data.get('gps_ok', False))
        except Exception as e:
            self.warn_throttled(f"Failed to parse Control Node status JSON: {e}", "control_json_parse_err")
            # Default to safe False values on corrupted data
            self.control_connected = False
            self.control_state_ok = False
            self.control_gps_ok = False

    def waypoint_list_callback(self, msg: WaypointList):
        """Processes the list of waypoints downloaded from MAVROS/Pixhawk."""
        self.get_logger().info(f"Received waypoint list from MAVROS containing {len(msg.waypoints)} items. Pixhawk current seq: {msg.current_seq}")

        new_nav_waypoints = []
        for i, wp in enumerate(msg.waypoints):
            # In MAVROS/ArduPilot, index in the waypoints list is the seq ID
            seq = i

            # Filter for standard navigation commands (MAV_CMD_NAV_WAYPOINT = 16)
            if wp.command == 16:
                # ArduPilot seq 0 is always the Home Position, skip it
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
            else:
                # Throttled warning for unsupported commands (e.g. servo command, camera command)
                self.warn_throttled(
                    f"Skipped non-navigation command {wp.command} at sequence {seq}.",
                    f"unsupported_cmd_{wp.command}"
                )

        self.nav_waypoints = new_nav_waypoints
        self.total_navigation_waypoints = len(self.nav_waypoints)
        self.mission_loaded = (self.total_navigation_waypoints > 0)

        if not self.mission_loaded:
            self.set_error("MISSION_LIST_EMPTY")
            self.active_waypoint = None
            self.target_valid = False
            return

        # Clear empty mission list related errors
        if self.error_code in ("MISSION_LIST_EMPTY", "NO_VALID_WAYPOINT"):
            self.clear_error()

        # Determine active target based on MAVROS current_seq
        self.update_active_target(msg.current_seq)

    def waypoint_reached_callback(self, msg: WaypointReached):
        """Processes MAVROS reached messages."""
        wp_seq = int(msg.wp_seq)
        self.get_logger().info(f"MAVROS notified reached for waypoint seq: {wp_seq}")

        if self.active_waypoint is None:
            self.warn_throttled("MAVROS reported reached waypoint, but active waypoint is None.", "reached_no_active")
            return

        if wp_seq == self.active_waypoint['seq']:
            # Double check to prevent duplicate trigger for the same seq
            if wp_seq != self.last_reached_seq:
                self.last_reached_seq = wp_seq
                self.confirm_waypoint_reached(wp_seq)
            else:
                self.warn_throttled(f"Duplicate reached event received for waypoint {wp_seq}.", "dup_reached_event")
        else:
            # Out of order or old reached message
            self.warn_throttled(
                f"Received reached event for seq {wp_seq}, but current active target is seq {self.active_waypoint['seq']}.",
                "out_of_order_reached"
            )

    # --- Mission logic operations ---

    def request_mission_pull(self):
        """Triggers asynchronous call to MAVROS pull service."""
        if self.pull_in_progress:
            return

        if not self.pull_client.service_is_ready():
            self.warn_throttled("MAVROS waypoint pull service is not ready.", "pull_client_not_ready")
            self.set_error("MAVROS_UNAVAILABLE")
            return

        self.pull_in_progress = True
        request = WaypointPull.Request()
        self.get_logger().info("Sending asynchronous mission pull command to MAVROS...")

        future = self.pull_client.call_async(request)
        future.add_done_callback(self.pull_response_callback)

    def pull_response_callback(self, future):
        """Callback for pull service response."""
        self.pull_in_progress = False
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"MAVROS mission pull succeeded. Count: {response.wp_received}")
                if self.error_code == "MISSION_PULL_FAILED":
                    self.clear_error()
            else:
                self.get_logger().error("MAVROS mission pull service returned success=False.")
                self.set_error("MISSION_PULL_FAILED")
        except Exception as e:
            self.get_logger().error(f"MAVROS mission pull call failed: {e}")
            self.set_error("MISSION_PULL_FAILED")

    def update_active_target(self, current_seq: int):
        """Calculates and sets the next navigation target from the list."""
        if not self.nav_waypoints:
            self.active_waypoint = None
            self.target_valid = False
            return

        # Find the waypoint that matches current_seq
        target_wp = None
        for wp in self.nav_waypoints:
            if wp['seq'] == current_seq:
                target_wp = wp
                break

        # If not exact match, pick the first waypoint with seq >= current_seq
        if target_wp is None:
            for wp in self.nav_waypoints:
                if wp['seq'] > current_seq:
                    target_wp = wp
                    break

        # If still none, default to the first one in the list
        if target_wp is None:
            target_wp = self.nav_waypoints[0]

        # Reset reached check counter on waypoint change
        if self.active_waypoint is None or self.active_waypoint['seq'] != target_wp['seq']:
            self.reached_samples_counter = 0
            self.get_logger().info(f"Active navigation target set to waypoint seq: {target_wp['seq']}")

        self.active_waypoint = target_wp
        self.target_valid = True

        # Validate that the active waypoint has finite coordinates
        if not (math.isfinite(target_wp['latitude']) and math.isfinite(target_wp['longitude'])):
            self.set_error("ACTIVE_WAYPOINT_INVALID")
            self.target_valid = False

    def validate_gps(self) -> bool:
        """Validates all inputs relating to GPS availability and age."""
        if not self.gps_received:
            self.set_error("GPS_NOT_RECEIVED")
            return False

        if not self.gps_valid:
            self.set_error("GPS_INVALID")
            return False

        now = time.monotonic()
        if now - self.last_gps_time > self.gps_timeout_sec:
            self.set_error("GPS_STALE")
            return False

        if not self.control_gps_ok:
            self.set_error("GPS_INVALID")
            return False

        if not self.control_connected:
            self.set_error("MAVROS_UNAVAILABLE")
            return False

        if not self.control_state_ok:
            self.set_error("CONTROL_STATUS_INVALID")
            return False

        # Clear errors if GPS recovered
        if self.error_code in ("GPS_NOT_RECEIVED", "GPS_INVALID", "GPS_STALE", "CONTROL_STATUS_INVALID", "MAVROS_UNAVAILABLE"):
            self.clear_error()

        return True

    def main_loop(self):
        """Main timer loop execution. Evaluates states, performs math, and publishes."""
        now = time.monotonic()

        # Handle startup pull or periodic pull retries
        if not self.nav_waypoints and not self.pull_in_progress:
            # Run pull immediately if startup pull is active, or periodic retry
            time_since_attempt = now - self.last_pull_attempt_time
            if (self.last_pull_attempt_time == 0.0 and self.auto_pull_mission_on_startup) or \
               (time_since_attempt >= self.mission_pull_retry_period_sec):
                self.last_pull_attempt_time = now
                self.request_mission_pull()

        # Perform state state machine transitions
        self.update_state_machine()

        # Process active tracking calculations
        if self.mission_state == "RUNNING" and self.active_waypoint is not None:
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

            # Check if waypoint target is reached via GPS distance
            self.check_gps_reached()

        # Determine current active parkur
        self.current_parkur = self.determine_current_parkur()

        # Publish the outputs
        self.publish_status()
        self.publish_target()

    def update_state_machine(self):
        """Updates the mission_state value based on environment status."""
        # Completed state is terminal and overrides other states
        if self.mission_state == "COMPLETED":
            return

        # Check for errors first
        if self.error_code != "NONE" and self.error_code not in ("NONE", "MISSION_PULL_FAILED"):
            self.mission_state = "ERROR"
            self.mission_active = False
            return

        # Check basic MAVROS / Control node connection
        if not self.control_connected or (time.monotonic() - self.last_control_status_time > self.gps_timeout_sec):
            self.mission_state = "WAITING_FOR_MAVROS"
            self.mission_active = False
            return

        # Check if waypoints are loaded
        if not self.nav_waypoints:
            self.mission_state = "WAITING_FOR_MISSION"
            self.mission_active = False
            return

        # Validate GPS health
        if not self.validate_gps():
            self.mission_state = "WAITING_FOR_GPS"
            self.mission_active = False
            return

        # If everything is functional, mission is running
        self.mission_state = "RUNNING"
        self.mission_active = True

    def check_gps_reached(self):
        """Monitors GPS distance thresholds for waypoint reached logic."""
        if self.active_waypoint is None:
            return

        # If within radius limit
        if self.distance_to_target <= self.waypoint_reached_radius_m:
            self.reached_samples_counter += 1
            self.target_reached = True

            if self.reached_samples_counter >= self.required_reached_samples:
                seq = self.active_waypoint['seq']
                self.get_logger().info(f"WaypointReached confirmed via GPS distance check for seq {seq}.")
                self.last_reached_seq = seq
                self.confirm_waypoint_reached(seq)
        else:
            self.reached_samples_counter = 0
            self.target_reached = False

    def confirm_waypoint_reached(self, reached_seq: int):
        """Handles final actions when a waypoint is officially completed."""
        if self.active_waypoint is None or reached_seq != self.active_waypoint['seq']:
            return

        self.reached_waypoint_count += 1
        self.reached_samples_counter = 0
        self.target_reached = False

        # Advance to the next waypoint in the list
        self.advance_to_next_waypoint()

    def advance_to_next_waypoint(self):
        """Points active waypoint to the next sequence item in nav_waypoints list."""
        if not self.nav_waypoints:
            return

        current_idx = -1
        for i, wp in enumerate(self.nav_waypoints):
            if wp['seq'] == self.active_waypoint['seq']:
                current_idx = i
                break

        if current_idx != -1 and current_idx + 1 < len(self.nav_waypoints):
            self.active_waypoint = self.nav_waypoints[current_idx + 1]
            self.get_logger().info(f"Advanced to next active waypoint: seq={self.active_waypoint['seq']}")
            self.target_valid = True
            # Recheck valid coordinates on new target
            if not (math.isfinite(self.active_waypoint['latitude']) and math.isfinite(self.active_waypoint['longitude'])):
                self.set_error("ACTIVE_WAYPOINT_INVALID")
                self.target_valid = False
        else:
            # No more navigation waypoints left in the mission list
            self.active_waypoint = None
            self.mission_state = "COMPLETED"
            self.mission_active = False
            self.mission_completed = True
            self.target_valid = False
            self.current_parkur = MissionStatus.PARKUR_COMPLETE
            self.get_logger().info("All waypoints completed! Mission finished.")

    def determine_current_parkur(self) -> int:
        """Determines parkur index according to active waypoint seq."""
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

    # --- Math Helpers ---

    @staticmethod
    def calculate_distance_m(current_lat: float, current_lon: float, target_lat: float, target_lon: float) -> float:
        """Calculates distance between two global coordinate pairs using Haversine formula."""
        if math.isclose(current_lat, target_lat) and math.isclose(current_lon, target_lon):
            return 0.0

        # Earth radius: 6,371,000 meters
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
        """Calculates bearing from current position pointing towards target coordinates."""
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

    # --- Error State Helpers ---

    def set_error(self, code: str):
        """Sets internal error code, only logging changes."""
        if self.error_code != code:
            self.error_code = code
            self.get_logger().error(f"Mission Node entered error state: {code}")

    def clear_error(self):
        """Clears current active error."""
        if self.error_code != "NONE":
            self.get_logger().info(f"Error cleared: {self.error_code}")
            self.error_code = "NONE"

    def warn_throttled(self, msg: str, key: str):
        """Helper to write warnings into log, throttled to prevent spam."""
        now = time.monotonic()
        last_time = self.warn_timestamps.get(key, 0.0)
        if now - last_time >= self.warning_throttle_sec:
            self.get_logger().warn(msg)
            self.warn_timestamps[key] = now

    # --- Output Publishers ---

    def publish_status(self):
        """Fills and publishes the MissionStatus message."""
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
        """Fills and publishes the MissionTarget message."""
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
        node.get_logger().info("Mission Node shutdown via KeyboardInterrupt.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
