#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
parkur3_target_node.py — LODOS Albatros Parkur 3 Target Perception & Validation Node
=====================================================================================
Görevi:
  - Parkur 3 (Kamikaze Angajman) için YOLO tespitleri (conf >= 0.30) ile OpenCV HSV
    renk analizini ROI düzeyinde birleştirir.
  - %40 YOLO / %60 OpenCV HSV ağırlıklı skor hesabı yapar (Eşik: 0.65).
  - 5 frame temporal doğrulama (en az 4/5 uyum) ve bbox fiziksel süreklilik takibi uygular.
  - Hedef onaylandığında `TARGET_CONFIRMED = true` ve hedef açısı (`target_angle_deg`) yayınlar.

Kısıtlar:
  - Motor veya Pixhawk komutu ÜRETMEZ.
  - Tarama hareketi YAPTIRMAZ.
  - YOLO modeli YÜKLEMEZ.
  - Sadece perception & validation katmanıdır.
"""

from collections import deque
import json
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from albatros_interfaces.msg import MissionStatus, VehicleState
except ImportError:
    MissionStatus = None
    VehicleState = None


class Parkur3TargetNode(Node):
    """
    Parkur 3 Target Validation Node for LODOS Albatros İDA.
    Fuses YOLO detections with OpenCV HSV ROI color validation and 5-frame temporal history.
    """

    def __init__(self):
        super().__init__('parkur3_target_node')

        # ─── ROS Parameters ───────────────────────────────────────────────────
        self.declare_parameter('input_image_topic', '/albatros/kamera/image_raw')
        self.declare_parameter('yolo_detections_topic', '/albatros/yolo/tespitler')
        self.declare_parameter('target_color_topic', '/albatros/hedef_renk')
        self.declare_parameter('vehicle_state_topic', '/albatros/state')
        self.declare_parameter('mission_status_topic', '/albatros/mission/status')
        self.declare_parameter('output_target_topic', '/albatros/parkur3/target_confirmed')

        # Weights & Thresholds
        self.declare_parameter('yolo_weight', 0.40)
        self.declare_parameter('opencv_weight', 0.60)
        self.declare_parameter('final_score_threshold', 0.65)
        self.declare_parameter('yolo_min_confidence', 0.30)
        self.declare_parameter('opencv_color_gate', 0.15)  # Min color ratio inside ROI

        # Temporal validation parameters
        self.declare_parameter('validation_window_size', 5)
        self.declare_parameter('required_confirmations', 4)
        self.declare_parameter('bbox_center_tolerance_px', 60.0)

        # Camera parameter (Logitech C920 default FOV ~78 degrees)
        self.declare_parameter('camera_fov_deg', 78.0)
        self.declare_parameter('camera_width_px', 640)

        # HSV Range Parameters
        # Red range 1
        self.declare_parameter('hsv_red_low1', [0, 100, 80])
        self.declare_parameter('hsv_red_high1', [10, 255, 255])
        # Red range 2 (wrap-around)
        self.declare_parameter('hsv_red_low2', [160, 100, 80])
        self.declare_parameter('hsv_red_high2', [180, 255, 255])

        # Green range
        self.declare_parameter('hsv_green_low', [35, 70, 70])
        self.declare_parameter('hsv_green_high', [85, 255, 255])

        # Black range
        self.declare_parameter('hsv_black_low', [0, 0, 0])
        self.declare_parameter('hsv_black_high', [180, 255, 70])

        # Read parameters
        self.input_image_topic = str(self.get_parameter('input_image_topic').value)
        self.yolo_detections_topic = str(self.get_parameter('yolo_detections_topic').value)
        self.target_color_topic = str(self.get_parameter('target_color_topic').value)
        self.vehicle_state_topic = str(self.get_parameter('vehicle_state_topic').value)
        self.mission_status_topic = str(self.get_parameter('mission_status_topic').value)
        self.output_target_topic = str(self.get_parameter('output_target_topic').value)

        self.yolo_weight = float(self.get_parameter('yolo_weight').value)
        self.opencv_weight = float(self.get_parameter('opencv_weight').value)
        self.final_score_threshold = float(self.get_parameter('final_score_threshold').value)
        self.yolo_min_confidence = float(self.get_parameter('yolo_min_confidence').value)
        self.opencv_color_gate = float(self.get_parameter('opencv_color_gate').value)

        self.window_size = int(self.get_parameter('validation_window_size').value)
        self.required_confirmations = int(self.get_parameter('required_confirmations').value)
        self.bbox_tolerance = float(self.get_parameter('bbox_center_tolerance_px').value)

        self.camera_fov_deg = float(self.get_parameter('camera_fov_deg').value)
        self.camera_width_px = int(self.get_parameter('camera_width_px').value)

        # ─── Dahili Durum Değişkenleri ───────────────────────────────────────
        self.current_frame = None
        self.current_parkur = 0
        self.is_parkur3 = False
        self.vehicle_turning = False
        self.target_color = "RED"  # Default target color until updated by drone/GCS

        # Temporal validation sliding window
        self.history = deque(maxlen=self.window_size)

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

        self.sub_target_color = self.create_subscription(
            String,
            self.target_color_topic,
            self.cb_target_color,
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

        self.get_logger().info(
            f"Parkur3TargetNode initialized. Target color: '{self.target_color}', "
            f"Weights: YOLO={self.yolo_weight:.2f} / OpenCV={self.opencv_weight:.2f}, "
            f"Final Threshold={self.final_score_threshold:.2f}"
        )

    # =========================================================================
    # Callbacks
    # =========================================================================

    def cb_image(self, msg: Image):
        """Stores latest camera frame for ROI OpenCV processing."""
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "mono8":
                frame = data.reshape((msg.height, msg.step))[:, :msg.width]
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame = data.reshape((msg.height, msg.step))[:, :msg.width * 3]
                frame = frame.reshape((msg.height, msg.width, 3))
                if msg.encoding in ["rgb8", "RGB8"]:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.current_frame = frame
            self.camera_width_px = msg.width
        except Exception as e:
            self.get_logger().error(f"Failed to convert image in target node: {e}", throttle_duration_sec=2.0)

    def cb_target_color(self, msg: String):
        """Receives target color from Drone / GCS (e.g. 'RED', 'GREEN', 'BLACK')."""
        color_str = msg.data.strip().upper()
        if color_str in ["RED", "GREEN", "BLACK", "KIRMIZI", "YESIL", "SIYAH"]:
            if color_str == "KIRMIZI":
                color_str = "RED"
            elif color_str == "YESIL":
                color_str = "GREEN"
            elif color_str == "SIYAH":
                color_str = "BLACK"
            if color_str != self.target_color:
                self.get_logger().info(f"Target color updated from topic: '{color_str}'")
                self.target_color = color_str
                self.history.clear()

    def cb_mission_status(self, msg):
        self.current_parkur = msg.current_parkur
        state_str = str(msg.mission_state).upper()
        self.is_parkur3 = (self.current_parkur == 3 or "PARKUR3" in state_str or "PARKUR_3" in state_str)

    def cb_vehicle_state(self, msg):
        self.current_parkur = msg.current_parkur
        state_str = str(msg.mission_state).upper()
        self.is_parkur3 = (self.current_parkur == 3 or "PARKUR3" in state_str or "PARKUR_3" in state_str)
        turn_dir = str(getattr(msg, "turn_direction", "UNKNOWN")).upper()
        self.vehicle_turning = (turn_dir in ["LEFT", "RIGHT"])

    def cb_yolo_detections(self, msg: String):
        """
        Main validation pipeline triggered when YOLO detections arrive.
        Evaluates ROI OpenCV color analysis, weighted scoring, and 5-frame temporal tracking.
        """
        stamp_float = self.get_clock().now().nanoseconds / 1e9

        if not self.is_parkur3:
            self.history.clear()
            self.publish_result(stamp_float, False, 0.0, 0.0, 0.0, 0, 0, 0.0, {})
            return

        try:
            payload = json.loads(msg.data)
            stamp_float = payload.get("stamp", stamp_float)
            raw_detections = payload.get("detections", [])
        except Exception as e:
            self.get_logger().warn(f"Failed to parse YOLO detections JSON: {e}")
            return

        if self.current_frame is None:
            self.publish_result(stamp_float, False, 0.0, 0.0, 0.0, 0, 0, 0.0, {})
            return

        candidates = []
        for det in raw_detections:
            yolo_conf = float(det.get("confidence", 0.0))
            if yolo_conf < self.yolo_min_confidence:
                continue

            class_name = str(det.get("class_name", "")).lower()
            bbox = det.get("bbox", {})
            center = det.get("center", {})

            opencv_conf, color_pass = self.evaluate_roi_color(self.current_frame, bbox, self.target_color)

            if not color_pass or opencv_conf < self.opencv_color_gate:
                self.get_logger().debug(
                    f"Candidate '{class_name}' rejected by OpenCV color gate (ratio={opencv_conf:.2f} < gate={self.opencv_color_gate:.2f})"
                )
                continue

            final_score = (self.yolo_weight * yolo_conf) + (self.opencv_weight * opencv_conf)

            if final_score >= self.final_score_threshold:
                candidates.append({
                    "class_name": class_name,
                    "yolo_conf": yolo_conf,
                    "opencv_conf": opencv_conf,
                    "final_score": final_score,
                    "bbox": bbox,
                    "center": center
                })

        best_candidate = None
        if candidates:
            candidates.sort(key=lambda c: c["final_score"], reverse=True)
            best_candidate = candidates[0]

        if self.vehicle_turning:
            self.history.clear()
            self.publish_result(stamp_float, False, 0.0, 0.0, 0.0, 0, 0, 0.0, {})
            return

        if best_candidate is not None:
            if len(self.history) > 0:
                last_obs = self.history[-1]
                if last_obs is not None:
                    last_cx = last_obs["center"]["x"]
                    last_cy = last_obs["center"]["y"]
                    curr_cx = best_candidate["center"]["x"]
                    curr_cy = best_candidate["center"]["y"]
                    dist = math.hypot(curr_cx - last_cx, curr_cy - last_cy)
                    if dist > self.bbox_tolerance:
                        self.history.clear()

            self.history.append(best_candidate)
        else:
            self.history.append(None)

        valid_observations = [obs for obs in self.history if obs is not None]
        match_count = len(valid_observations)

        if match_count >= self.required_confirmations and best_candidate is not None:
            target_confirmed = True
            final_score = best_candidate["final_score"]
            yolo_conf = best_candidate["yolo_conf"]
            opencv_conf = best_candidate["opencv_conf"]
            cx = best_candidate["center"]["x"]
            cy = best_candidate["center"]["y"]
            bbox = best_candidate["bbox"]

            target_angle_deg = self.calculate_target_angle(cx)

            self.get_logger().info(
                f"[TARGET CONFIRMED] Color: {self.target_color} | Final Score: {final_score:.2f} "
                f"(YOLO: {yolo_conf:.2f}, OpenCV: {opencv_conf:.2f}) | Angle: {target_angle_deg:.1f}° | History: {match_count}/{self.window_size}"
            )
            self.publish_result(
                stamp_float, target_confirmed, final_score, yolo_conf, opencv_conf, cx, cy, target_angle_deg, bbox, match_count
            )
        else:
            self.publish_result(stamp_float, False, 0.0, 0.0, 0.0, 0, 0, 0.0, {}, match_count)

    def evaluate_roi_color(self, frame: np.ndarray, bbox: dict, target_color: str):
        h_img, w_img = frame.shape[:2]
        x1 = max(0, min(int(bbox.get("x_min", 0)), w_img - 1))
        y1 = max(0, min(int(bbox.get("y_min", 0)), h_img - 1))
        x2 = max(0, min(int(bbox.get("x_max", 0)), w_img - 1))
        y2 = max(0, min(int(bbox.get("y_max", 0)), h_img - 1))

        if x2 <= x1 or y2 <= y1:
            return 0.0, False

        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return 0.0, False

        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        total_pixels = roi.shape[0] * roi.shape[1]
        if total_pixels == 0:
            return 0.0, False

        target_color = target_color.upper()

        if target_color == "RED":
            r_low1 = np.array(self.get_parameter('hsv_red_low1').value, dtype=np.uint8)
            r_high1 = np.array(self.get_parameter('hsv_red_high1').value, dtype=np.uint8)
            r_low2 = np.array(self.get_parameter('hsv_red_low2').value, dtype=np.uint8)
            r_high2 = np.array(self.get_parameter('hsv_red_high2').value, dtype=np.uint8)

            m1 = cv2.inRange(roi_hsv, r_low1, r_high1)
            m2 = cv2.inRange(roi_hsv, r_low2, r_high2)
            mask = cv2.bitwise_or(m1, m2)

        elif target_color == "GREEN":
            g_low = np.array(self.get_parameter('hsv_green_low').value, dtype=np.uint8)
            g_high = np.array(self.get_parameter('hsv_green_high').value, dtype=np.uint8)
            mask = cv2.inRange(roi_hsv, g_low, g_high)

        elif target_color == "BLACK":
            b_low = np.array(self.get_parameter('hsv_black_low').value, dtype=np.uint8)
            b_high = np.array(self.get_parameter('hsv_black_high').value, dtype=np.uint8)
            mask = cv2.inRange(roi_hsv, b_low, b_high)

        else:
            return 0.5, True

        matching_pixels = cv2.countNonZero(mask)
        color_ratio = matching_pixels / float(total_pixels)

        normalized_conf = min(1.0, color_ratio / 0.30)
        passed_gate = (color_ratio >= self.opencv_color_gate)

        return normalized_conf, passed_gate

    def calculate_target_angle(self, center_x: int) -> float:
        half_w = self.camera_width_px / 2.0
        if half_w <= 0:
            return 0.0
        offset_px = center_x - half_w
        angle_deg = offset_px * (self.camera_fov_deg / self.camera_width_px)
        return float(angle_deg)

    def publish_result(
        self, stamp_float: float, confirmed: bool, final_score: float,
        yolo_conf: float, opencv_conf: float, cx: int, cy: int, angle_deg: float,
        bbox: dict, history_matches: int = 0
    ):
        payload = {
            "stamp": stamp_float,
            "target_confirmed": confirmed,
            "target_color": self.target_color,
            "final_score": float(final_score),
            "yolo_confidence": float(yolo_conf),
            "opencv_confidence": float(opencv_conf),
            "center": {"x": cx, "y": cy},
            "target_angle_deg": float(angle_deg),
            "bbox": bbox,
            "history_matches": history_matches,
            "window_size": self.window_size
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
