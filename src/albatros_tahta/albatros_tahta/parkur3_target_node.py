#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
parkur3_target_node.py — LODOS Albatros Parkur 3 Target Perception & Validation Node
=====================================================================================
Görev:
  - Parkur 3 (Kamikaze Angajman) için YOLO tespitleri (conf >= 0.30) ile OpenCV HSV
    renk analizini ROI düzeyinde birleştirir.
  - %60 YOLO / %40 OpenCV HSV ağırlıklı skor hesabı yapar (Final Eşik: 0.65).
  - Hedef renk bilgisi doğrudan İDA Pixhawk'ındaki `SCR_USER1` parametresinden pymavlink ile çekilir.
    Merkezi renk eşleşmesi:
      1 -> RED
      2 -> GREEN
      3 -> BLACK
      Diğer -> UNKNOWN
  - Kesintisiz 5 saniye zaman tabanlı doğrulama uygular (time.monotonic kullanır).
  - Kamera framelerini timestamp ile deque(maxlen=10) içinde tutar ve YOLO timestamp ile senkronize eder.
  - Hedef onaylandığında `target_confirmed = true` ve hedef açısı (`target_angle_deg`) yayınlar.

Kısıtlar:
  - Motor veya Pixhawk sürüş komutu ÜRETMEZ.
  - Tarama hareketi YAPTIRMAZ.
  - YOLO modeli YÜKLEMEZ.
  - Sadece perception & validation katmanıdır.
"""

from collections import deque
import json
import math
import threading
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

try:
    from albatros_interfaces.msg import MissionStatus, VehicleState
except ImportError:
    MissionStatus = None
    VehicleState = None

# ─── Merkezi Renk Sözlüğü ─────────────────────────────────────────────────────
TARGET_COLOR_MAP = {
    1: "RED",
    2: "GREEN",
    3: "BLACK",
}


def normalize_color_name(name: str) -> str:
    """
    Normalizes Turkish and English color/object class names to standard 'RED', 'GREEN', 'BLACK' or 'UNKNOWN'.
    Supports variations like 'Kırmızı Duba', 'kirmizi_duba', 'YEŞİL', 'yesil', 'Black', etc.
    """
    if not name or not isinstance(name, str):
        return "UNKNOWN"

    s = name.strip().lower()
    s = s.replace('ı', 'i').replace('ş', 's').replace('ğ', 'g').replace('ü', 'u').replace('ö', 'o').replace('ç', 'c')
    s = s.replace('\u0307', '')
    s = s.replace('_', ' ').replace('-', ' ')

    if 'kirmizi' in s or 'red' in s:
        return "RED"
    if 'yesil' in s or 'green' in s:
        return "GREEN"
    if 'siyah' in s or 'black' in s:
        return "BLACK"
    return "UNKNOWN"


class Parkur3TargetNode(Node):
    """
    Parkur 3 Target Validation Node for LODOS Albatros İDA.
    Fuses YOLO detections with OpenCV HSV ROI color validation and 5.0-second continuous time validation.
    Reads target color from Pixhawk SCR_USER1 parameter via pymavlink.
    """

    def __init__(self):
        super().__init__('parkur3_target_node')

        # ─── ROS Parameters ───────────────────────────────────────────────────
        self.declare_parameter('input_image_topic', '/albatros/kamera/image_raw')
        self.declare_parameter('yolo_detections_topic', '/albatros/yolo/tespitler')
        self.declare_parameter('vehicle_state_topic', '/albatros/state')
        self.declare_parameter('mission_status_topic', '/albatros/mission/status')
        self.declare_parameter('output_target_topic', '/albatros/parkur3/target_confirmed')

        # MAVLink Connection Parameters
        self.declare_parameter('mavlink_device', '/dev/ttyACM0')
        self.declare_parameter('mavlink_baud', 115200)

        # Weights & Thresholds (%60 YOLO, %40 OpenCV)
        self.declare_parameter('yolo_weight', 0.60)
        self.declare_parameter('opencv_weight', 0.40)
        self.declare_parameter('final_score_threshold', 0.65)
        self.declare_parameter('yolo_min_confidence', 0.30)
        self.declare_parameter('opencv_color_gate', 0.15)  # Min color ratio inside ROI

        # Continuous Time Validation parameters
        self.declare_parameter('required_validation_sec', 5.0)
        self.declare_parameter('bbox_center_tolerance_px', 60.0)

        # Frame Synchronization & Turning parameters
        self.declare_parameter('frame_sync_tolerance_sec', 0.15)
        self.declare_parameter('clear_history_while_turning', False)

        # Camera parameters
        self.declare_parameter('camera_fov_deg', 78.0)
        self.declare_parameter('camera_width_px', 640)
        self.declare_parameter('camera_fx_px', 0.0)

        # HSV Range Parameters
        self.declare_parameter('hsv_red_low1', [0, 100, 80])
        self.declare_parameter('hsv_red_high1', [10, 255, 255])
        self.declare_parameter('hsv_red_low2', [160, 100, 80])
        self.declare_parameter('hsv_red_high2', [180, 255, 255])

        self.declare_parameter('hsv_green_low', [35, 70, 70])
        self.declare_parameter('hsv_green_high', [85, 255, 255])

        self.declare_parameter('hsv_black_low', [0, 0, 0])
        self.declare_parameter('hsv_black_high', [180, 255, 70])

        # Read parameter values
        self.input_image_topic = str(self.get_parameter('input_image_topic').value)
        self.yolo_detections_topic = str(self.get_parameter('yolo_detections_topic').value)
        self.vehicle_state_topic = str(self.get_parameter('vehicle_state_topic').value)
        self.mission_status_topic = str(self.get_parameter('mission_status_topic').value)
        self.output_target_topic = str(self.get_parameter('output_target_topic').value)

        self.mavlink_device = str(self.get_parameter('mavlink_device').value)
        self.mavlink_baud = int(self.get_parameter('mavlink_baud').value)

        self.yolo_weight = float(self.get_parameter('yolo_weight').value)
        self.opencv_weight = float(self.get_parameter('opencv_weight').value)
        self.final_score_threshold = float(self.get_parameter('final_score_threshold').value)
        self.yolo_min_confidence = float(self.get_parameter('yolo_min_confidence').value)
        self.opencv_color_gate = float(self.get_parameter('opencv_color_gate').value)

        self.required_validation_sec = float(self.get_parameter('required_validation_sec').value)
        self.bbox_tolerance = float(self.get_parameter('bbox_center_tolerance_px').value)

        self.frame_sync_tolerance_sec = float(self.get_parameter('frame_sync_tolerance_sec').value)
        self.clear_history_while_turning = bool(self.get_parameter('clear_history_while_turning').value)

        self.camera_fov_deg = float(self.get_parameter('camera_fov_deg').value)
        self.camera_width_px = int(self.get_parameter('camera_width_px').value)
        self.camera_fx_px = float(self.get_parameter('camera_fx_px').value)

        # Normalize weights if needed
        total_weight = self.yolo_weight + self.opencv_weight
        if abs(total_weight - 1.0) > 1e-3:
            self.get_logger().warn(
                f"Sum of yolo_weight ({self.yolo_weight:.2f}) and opencv_weight ({self.opencv_weight:.2f}) "
                f"is {total_weight:.2f} != 1.0. Normalizing weights."
            )
            if total_weight > 0.0:
                self.yolo_weight /= total_weight
                self.opencv_weight /= total_weight
            else:
                self.yolo_weight = 0.60
                self.opencv_weight = 0.40

        # ─── Dahili Durum Değişkenleri ───────────────────────────────────────
        self.image_buffer = deque(maxlen=10)
        self.current_parkur = 0
        self.is_parkur3 = False
        self.has_mission_status = False
        self.vehicle_turning = False

        self.state_lock = threading.Lock()
        self.target_color = "UNKNOWN"
        self.last_scr_user1_val = None

        # Continuous time validation state
        self.validation_start_time = None
        self.last_target_center = None
        self.last_progress_log_time = 0.0
        self.confirmed_logged = False

        # ─── Publishers & Subscribers ────────────────────────────────────────
        self.pub_confirmed_target = self.create_publisher(String, self.output_target_topic, 10)

        self.sub_image = self.create_subscription(
            Image,
            self.input_image_topic,
            self.cb_image,
            qos_profile_sensor_data
        )

        self.sub_yolo = self.create_subscription(
            String,
            self.yolo_detections_topic,
            self.cb_yolo_detections,
            10
        )

        if MissionStatus is not None:
            self.sub_mission_status = self.create_subscription(
                MissionStatus,
                self.mission_status_topic,
                self.cb_mission_status,
                10
            )

        if VehicleState is not None:
            self.sub_vehicle_state = self.create_subscription(
                VehicleState,
                self.vehicle_state_topic,
                self.cb_vehicle_state,
                10
            )

        # ─── MAVLink Background Thread Start ─────────────────────────────────
        self.mavlink_thread_running = True
        self.mavlink_thread = threading.Thread(target=self._mavlink_worker, daemon=True)
        self.mavlink_thread.start()

        self.get_logger().info(
            f"Parkur3TargetNode initialized. Target color: '{self.target_color}', "
            f"MAVLink device: '{self.mavlink_device}' @ {self.mavlink_baud} baud, "
            f"Weights: YOLO={self.yolo_weight:.2f} / OpenCV={self.opencv_weight:.2f}, "
            f"Final Threshold={self.final_score_threshold:.2f}, "
            f"Required continuous validation time={self.required_validation_sec:.1f}s"
        )

    def destroy_node(self):
        self.mavlink_thread_running = False
        super().destroy_node()

    def reset_validation(self):
        """Resets time-based continuous validation state."""
        self.validation_start_time = None
        self.last_target_center = None
        self.confirmed_logged = False

    # =========================================================================
    # MAVLink SCR_USER1 Reader
    # =========================================================================

    def _mavlink_worker(self):
        """
        Background thread polling Pixhawk's SCR_USER1 parameter ~0.5s via pymavlink.
        Does NOT block main ROS callbacks. Auto-reconnects if Pixhawk connection drops.
        """
        mav_conn = None

        while rclpy.ok() and self.mavlink_thread_running:
            if mavutil is None:
                self.get_logger().error(
                    "pymavlink library is not installed! Unable to read Pixhawk SCR_USER1 parameter.",
                    throttle_duration_sec=10.0
                )
                self._update_target_color_from_scr(None)
                time.sleep(2.0)
                continue

            device = str(self.get_parameter('mavlink_device').value)
            baud = int(self.get_parameter('mavlink_baud').value)

            # Establish or re-establish MAVLink connection
            if mav_conn is None:
                try:
                    self.get_logger().info(
                        f"Connecting to Pixhawk MAVLink on {device} @ {baud}...",
                        throttle_duration_sec=10.0
                    )
                    mav_conn = mavutil.mavlink_connection(device, baud=baud, autoreconnect=True)
                    hb = mav_conn.wait_heartbeat(timeout=3.0)
                    if hb:
                        self.get_logger().info(
                            f"Pixhawk MAVLink Heartbeat received! System ID: {mav_conn.target_system}, Component ID: {mav_conn.target_component}"
                        )
                    else:
                        self.get_logger().warn(
                            f"No heartbeat from Pixhawk on {device}. Retrying... "
                            f"(If using {device}, check if MAVROS or another pymavlink process is running)",
                            throttle_duration_sec=5.0
                        )
                        try:
                            mav_conn.close()
                        except Exception:
                            pass
                        mav_conn = None
                        self._update_target_color_from_scr(None)
                        time.sleep(2.0)
                        continue
                except Exception as e:
                    self.get_logger().warn(
                        f"MAVLink connection error on {device}: {e}. "
                        f"(If device busy, verify if MAVROS or another process is using {device}). Retrying in 2.0s...",
                        throttle_duration_sec=5.0
                    )
                    mav_conn = None
                    self._update_target_color_from_scr(None)
                    time.sleep(2.0)
                    continue

            # Connected: request SCR_USER1 parameter
            try:
                mav_conn.mav.param_request_read_send(
                    mav_conn.target_system,
                    mav_conn.target_component,
                    b'SCR_USER1',
                    -1
                )

                msg = mav_conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.4)
                if msg is not None:
                    param_id = msg.param_id
                    if isinstance(param_id, bytes):
                        param_id = param_id.decode('utf-8', errors='ignore')
                    param_id = param_id.replace('\x00', '').strip()

                    if param_id == 'SCR_USER1':
                        scr_val = int(round(float(msg.param_value)))
                        self._update_target_color_from_scr(scr_val)
                else:
                    # Request timeout or no match this cycle
                    pass
            except Exception as e:
                self.get_logger().warn(
                    f"MAVLink error while reading SCR_USER1: {e}. Reconnecting...",
                    throttle_duration_sec=5.0
                )
                try:
                    mav_conn.close()
                except Exception:
                    pass
                mav_conn = None
                self._update_target_color_from_scr(None)
                time.sleep(2.0)
                continue

            time.sleep(0.5)

    def _update_target_color_from_scr(self, scr_val):
        """
        Updates self.target_color based on Pixhawk SCR_USER1 value using TARGET_COLOR_MAP.
        """
        with self.state_lock:
            if scr_val == self.last_scr_user1_val:
                return

            self.last_scr_user1_val = scr_val
            new_color = TARGET_COLOR_MAP.get(scr_val, "UNKNOWN")

            log_str = f"[HEDEF RENK] SCR_USER1={scr_val} -> {new_color}"
            self.get_logger().info(log_str)
            print(log_str, flush=True)

            if new_color != self.target_color:
                self.target_color = new_color
                self.reset_validation()

    # =========================================================================
    # Callbacks
    # =========================================================================

    def cb_image(self, msg: Image):
        """Stores camera frame with timestamp in deque(maxlen=10)."""
        try:
            stamp_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
            if stamp_sec == 0.0:
                stamp_sec = self.get_clock().now().nanoseconds / 1e9

            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "mono8":
                frame = data.reshape((msg.height, msg.step))[:, :msg.width]
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame = data.reshape((msg.height, msg.step))[:, :msg.width * 3]
                frame = frame.reshape((msg.height, msg.width, 3))
                if msg.encoding in ["rgb8", "RGB8"]:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            self.image_buffer.append({
                "stamp": stamp_sec,
                "frame": frame,
                "width": msg.width
            })
            self.camera_width_px = msg.width
        except Exception as e:
            self.get_logger().error(f"Failed to convert image in target node: {e}", throttle_duration_sec=2.0)

    def cb_mission_status(self, msg):
        self.has_mission_status = True
        self.current_parkur = msg.current_parkur
        state_str = str(msg.mission_state).upper()
        self.is_parkur3 = (self.current_parkur == 3 or "PARKUR3" in state_str or "PARKUR_3" in state_str)

    def cb_vehicle_state(self, msg):
        if not self.has_mission_status:
            self.current_parkur = msg.current_parkur
            state_str = str(msg.mission_state).upper()
            self.is_parkur3 = (self.current_parkur == 3 or "PARKUR3" in state_str or "PARKUR_3" in state_str)

        turn_dir = str(getattr(msg, "turn_direction", "UNKNOWN")).upper()
        self.vehicle_turning = (turn_dir in ["LEFT", "RIGHT"])

    def cb_yolo_detections(self, msg: String):
        """
        Main perception and validation pipeline triggered when YOLO detections arrive.
        Filters candidate detections by requested color, validates ROI via OpenCV HSV,
        applies weighted scoring, and enforces 5.0-second continuous time confirmation.
        """
        now_stamp = self.get_clock().now().nanoseconds / 1e9

        with self.state_lock:
            start_target_color = self.target_color

        # 1. Parkur 3 active state check
        if not self.is_parkur3:
            with self.state_lock:
                self.reset_validation()
            self.publish_result(now_stamp, False, start_target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
            return

        # 2. Target color set check
        if start_target_color == "UNKNOWN":
            with self.state_lock:
                self.reset_validation()
            self.publish_result(now_stamp, False, "UNKNOWN", "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
            return

        # 3. Parse JSON YOLO payload safely
        try:
            payload = json.loads(msg.data)
            yolo_stamp = payload.get("stamp", None)
            if yolo_stamp is not None:
                yolo_stamp = float(yolo_stamp)
            raw_detections = payload.get("detections", [])
            if not isinstance(raw_detections, list):
                raw_detections = []
        except Exception as e:
            self.get_logger().debug(f"Failed to parse YOLO detections JSON: {e}")
            with self.state_lock:
                self.reset_validation()
            self.publish_result(now_stamp, False, start_target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
            return

        pub_stamp = yolo_stamp if yolo_stamp is not None else now_stamp

        # 4. Frame Synchronization with Image Buffer
        current_frame = None
        if yolo_stamp is not None:
            if len(self.image_buffer) == 0:
                self.get_logger().warn("Image buffer is empty. Cannot process YOLO detection.", throttle_duration_sec=2.0)
                with self.state_lock:
                    self.reset_validation()
                self.publish_result(pub_stamp, False, start_target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
                return

            best_match = min(self.image_buffer, key=lambda item: abs(item["stamp"] - yolo_stamp))
            time_diff = abs(best_match["stamp"] - yolo_stamp)

            if time_diff > self.frame_sync_tolerance_sec:
                self.get_logger().warn(
                    f"YOLO frame sync tolerance exceeded: time_diff={time_diff:.3f}s > threshold={self.frame_sync_tolerance_sec:.3f}s. Detection rejected.",
                    throttle_duration_sec=2.0
                )
                with self.state_lock:
                    self.reset_validation()
                self.publish_result(pub_stamp, False, start_target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
                return

            current_frame = best_match["frame"]
            self.camera_width_px = best_match["width"]
        else:
            self.get_logger().warn("YOLO JSON missing 'stamp'. Falling back to latest camera frame.", throttle_duration_sec=5.0)
            if len(self.image_buffer) == 0:
                with self.state_lock:
                    self.reset_validation()
                self.publish_result(pub_stamp, False, start_target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
                return

            latest_item = self.image_buffer[-1]
            current_frame = latest_item["frame"]
            self.camera_width_px = latest_item["width"]

        if current_frame is None:
            with self.state_lock:
                self.reset_validation()
            self.publish_result(pub_stamp, False, start_target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
            return

        # 5. Filter & Evaluate Candidate Detections
        candidates = []
        for det in raw_detections:
            if not isinstance(det, dict):
                continue

            # Confidence check
            try:
                yolo_conf = float(det.get("confidence", 0.0))
            except (ValueError, TypeError):
                continue

            if yolo_conf < self.yolo_min_confidence:
                continue

            # Class name normalization & requested color match
            class_name_raw = det.get("class_name", "")
            detected_color = normalize_color_name(str(class_name_raw))
            if detected_color == "UNKNOWN" or detected_color != start_target_color:
                continue

            # BBox validation
            bbox = det.get("bbox", {})
            if not isinstance(bbox, dict):
                continue

            try:
                x1 = int(float(bbox.get("x_min")))
                y1 = int(float(bbox.get("y_min")))
                x2 = int(float(bbox.get("x_max")))
                y2 = int(float(bbox.get("y_max")))
            except (ValueError, TypeError):
                continue

            if x2 <= x1 or y2 <= y1:
                continue

            # Center validation
            center = det.get("center", {})
            if not isinstance(center, dict):
                cx = int((x1 + x2) / 2.0)
                cy = int((y1 + y2) / 2.0)
                center = {"x": cx, "y": cy}
            else:
                try:
                    cx = int(center.get("x"))
                    cy = int(center.get("y"))
                except (ValueError, TypeError):
                    cx = int((x1 + x2) / 2.0)
                    cy = int((y1 + y2) / 2.0)
                    center = {"x": cx, "y": cy}

            # OpenCV ROI HSV color verification
            opencv_conf, color_pass = self.evaluate_roi_color(current_frame, bbox, start_target_color)

            if not color_pass:
                self.get_logger().debug(
                    f"Candidate '{class_name_raw}' (color={detected_color}) rejected by OpenCV color gate."
                )
                continue

            # Calculate Weighted Final Score (%60 YOLO + %40 OpenCV)
            final_score = (self.yolo_weight * yolo_conf) + (self.opencv_weight * opencv_conf)

            if final_score >= self.final_score_threshold:
                candidates.append({
                    "class_name": class_name_raw,
                    "detected_color": detected_color,
                    "yolo_conf": yolo_conf,
                    "opencv_conf": opencv_conf,
                    "final_score": final_score,
                    "bbox": {"x_min": x1, "y_min": y1, "x_max": x2, "y_max": y2},
                    "center": {"x": cx, "y": cy}
                })

        # 6. Select Best Candidate in Current Frame
        best_candidate = None
        if candidates:
            candidates.sort(key=lambda c: c["final_score"], reverse=True)
            best_candidate = candidates[0]

        now_mono = time.monotonic()

        # 7. Check race condition: Verify SCR_USER1 target_color didn't change while processing frame
        with self.state_lock:
            if self.target_color != start_target_color or self.target_color == "UNKNOWN":
                # Target color changed mid-frame! Cancel frame and reset validation.
                self.reset_validation()
                self.publish_result(pub_stamp, False, self.target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
                return

            if self.vehicle_turning and self.clear_history_while_turning:
                self.reset_validation()
                self.publish_result(pub_stamp, False, start_target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
                return

            if best_candidate is None:
                # Target lost in this frame -> reset continuous timer
                self.reset_validation()
                self.publish_result(pub_stamp, False, start_target_color, "UNKNOWN", 0.0, 0.0, 0.0, 0, 0, 0.0, {}, 0.0)
                return

            # Check spatial center jump tolerance
            if self.last_target_center is not None:
                dist = math.hypot(
                    best_candidate["center"]["x"] - self.last_target_center["x"],
                    best_candidate["center"]["y"] - self.last_target_center["y"]
                )
                if dist > self.bbox_tolerance:
                    # Spatial shift too large -> reset timer for new physical target track
                    self.reset_validation()

            self.last_target_center = best_candidate["center"]

            if self.validation_start_time is None:
                self.validation_start_time = now_mono

            elapsed_sec = now_mono - self.validation_start_time
            confirmed = (elapsed_sec >= self.required_validation_sec)

            # Logging logic
            if confirmed:
                if not self.confirmed_logged:
                    log_msg = f"[TARGET CONFIRMED] {start_target_color} | 5 saniye doğrulandı"
                    self.get_logger().info(log_msg)
                    print(log_msg, flush=True)
                    self.confirmed_logged = True
            else:
                self.confirmed_logged = False
                if now_mono - self.last_progress_log_time >= 1.0:
                    log_msg = f"[DOĞRULANIYOR] {start_target_color} {elapsed_sec:.1f} / {self.required_validation_sec:.1f} sn"
                    self.get_logger().info(log_msg)
                    print(log_msg, flush=True)
                    self.last_progress_log_time = now_mono

        final_score = best_candidate["final_score"]
        yolo_conf = best_candidate["yolo_conf"]
        opencv_conf = best_candidate["opencv_conf"]
        cx = best_candidate["center"]["x"]
        cy = best_candidate["center"]["y"]
        bbox = best_candidate["bbox"]
        detected_color = best_candidate["detected_color"]

        target_angle_deg = self.calculate_target_angle(cx)

        self.publish_result(
            pub_stamp, confirmed, start_target_color, detected_color, final_score,
            yolo_conf, opencv_conf, cx, cy, target_angle_deg, bbox, elapsed_sec
        )

    # =========================================================================
    # Helper Functions
    # =========================================================================

    def evaluate_roi_color(self, frame: np.ndarray, bbox: dict, target_color: str):
        """
        Crops bbox ROI from image, converts to HSV, and computes normalized matching pixel ratio.
        Returns: (color_confidence [0.0..1.0], passed_gate [bool])
        """
        try:
            h_img, w_img = frame.shape[:2]
            x1 = max(0, min(int(float(bbox.get("x_min", 0))), w_img - 1))
            y1 = max(0, min(int(float(bbox.get("y_min", 0))), h_img - 1))
            x2 = max(0, min(int(float(bbox.get("x_max", 0))), w_img - 1))
            y2 = max(0, min(int(float(bbox.get("y_max", 0))), h_img - 1))

            if x2 <= x1 or y2 <= y1:
                return 0.0, False

            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                return 0.0, False

            roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            total_pixels = roi.shape[0] * roi.shape[1]
            if total_pixels == 0:
                return 0.0, False

            norm_color = normalize_color_name(target_color)

            if norm_color == "RED":
                r_low1 = np.array(self.get_parameter('hsv_red_low1').value, dtype=np.uint8)
                r_high1 = np.array(self.get_parameter('hsv_red_high1').value, dtype=np.uint8)
                r_low2 = np.array(self.get_parameter('hsv_red_low2').value, dtype=np.uint8)
                r_high2 = np.array(self.get_parameter('hsv_red_high2').value, dtype=np.uint8)

                m1 = cv2.inRange(roi_hsv, r_low1, r_high1)
                m2 = cv2.inRange(roi_hsv, r_low2, r_high2)
                mask = cv2.bitwise_or(m1, m2)

            elif norm_color == "GREEN":
                g_low = np.array(self.get_parameter('hsv_green_low').value, dtype=np.uint8)
                g_high = np.array(self.get_parameter('hsv_green_high').value, dtype=np.uint8)
                mask = cv2.inRange(roi_hsv, g_low, g_high)

            elif norm_color == "BLACK":
                b_low = np.array(self.get_parameter('hsv_black_low').value, dtype=np.uint8)
                b_high = np.array(self.get_parameter('hsv_black_high').value, dtype=np.uint8)
                mask = cv2.inRange(roi_hsv, b_low, b_high)

            else:
                return 0.0, False

            matching_pixels = cv2.countNonZero(mask)
            color_ratio = matching_pixels / float(total_pixels)

            normalized_conf = min(1.0, color_ratio / 0.30)
            passed_gate = (color_ratio >= self.opencv_color_gate)

            return normalized_conf, passed_gate
        except Exception as e:
            self.get_logger().debug(f"Error in evaluate_roi_color: {e}")
            return 0.0, False

    def calculate_target_angle(self, center_x: float) -> float:
        """
        Calculates horizontal offset angle relative to camera optical axis.
        Negative = Left, 0 = Center, Positive = Right.
        """
        try:
            camera_center_x = self.camera_width_px / 2.0
            offset_px = float(center_x) - camera_center_x

            if self.camera_fx_px > 0.0:
                angle_rad = math.atan(offset_px / self.camera_fx_px)
                return float(math.degrees(angle_rad))
            else:
                if self.camera_width_px <= 0:
                    return 0.0
                angle_deg = offset_px * (self.camera_fov_deg / self.camera_width_px)
                return float(angle_deg)
        except Exception as e:
            self.get_logger().debug(f"Error calculating target angle: {e}")
            return 0.0

    def publish_result(
        self, stamp_float: float, confirmed: bool, target_color: str,
        detected_color: str, final_score: float, yolo_conf: float,
        opencv_conf: float, cx: int, cy: int, angle_deg: float,
        bbox: dict, elapsed_sec: float = 0.0
    ):
        """Publishes validated target JSON payload to /albatros/parkur3/target_confirmed."""
        payload = {
            "stamp": stamp_float,
            "target_confirmed": confirmed,
            "target_color": target_color,
            "detected_color": detected_color,
            "final_score": float(final_score),
            "yolo_confidence": float(yolo_conf),
            "opencv_confidence": float(opencv_conf),
            "center": {"x": int(cx), "y": int(cy)},
            "target_angle_deg": float(angle_deg),
            "bbox": bbox,
            "validation_elapsed_sec": round(float(elapsed_sec), 2),
            "required_validation_sec": self.required_validation_sec
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_confirmed_target.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Parkur3TargetNode()
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
