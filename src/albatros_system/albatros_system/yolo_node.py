#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

try:
    from albatros_interfaces.msg import MissionStatus, VehicleState
except ImportError:
    MissionStatus = None
    VehicleState = None

# Hailo platform imports
try:
    from hailo_platform import (
        HEF, VDevice, ConfigureParams,
        HailoStreamInterface,
        InputVStreamParams, OutputVStreamParams,
        InferVStreams, FormatType
    )
except ImportError as exc:
    raise RuntimeError(
        "hailo_platform is not installed or cannot be imported. "
        "This node requires HailoRT and hailo_platform to run on the target hardware."
    ) from exc


class YoloNode(Node):
    """
    Albatros YOLO perception node using Hailo AI Kit.
    Supports dual HEF model switching based on mission state and global 0.30 confidence threshold.

    Subscribes:
        /albatros/kamera/image_raw          sensor_msgs/Image
        /albatros/mission/status            albatros_interfaces/MissionStatus
        /albatros/state                     albatros_interfaces/VehicleState

    Publishes:
        /albatros/kamera/processed          sensor_msgs/Image
        /albatros/yolo/tespitler            std_msgs/String JSON (conf >= 0.30)
        /albatros/yolo/obstacles            std_msgs/String JSON
    """

    def __init__(self):
        super().__init__("yolo_node")

        # ROS Parameters
        self.declare_parameter("input_image_topic", "/albatros/kamera/image_raw")
        self.declare_parameter("processed_image_topic", "/albatros/kamera/processed")
        self.declare_parameter("detections_topic", "/albatros/yolo/tespitler")
        self.declare_parameter("obstacles_topic", "/albatros/yolo/obstacles")

        self.declare_parameter("model_path", "models/parkur12.hef")
        self.declare_parameter("parkur12_model_path", "models/parkur12.hef")
        self.declare_parameter("parkur3_model_path", "models/parkur3.hef")

        # Global YOLO confidence threshold (0.30 common across all parkours)
        self.declare_parameter("confidence_threshold", 0.30)
        self.declare_parameter("yolo_conf_threshold", 0.30)
        self.declare_parameter("model_input_width", 640)
        self.declare_parameter("model_input_height", 640)

        self.declare_parameter("save_video", False)
        self.declare_parameter("video_output_dir", "~/albatros_outputs/videos")
        self.declare_parameter("video_fps", 10.0)

        self.declare_parameter("draw_timestamp", True)
        self.declare_parameter("draw_center", True)
        self.declare_parameter("draw_detections", True)

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.processed_image_topic = str(self.get_parameter("processed_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.obstacles_topic = str(self.get_parameter("obstacles_topic").value)

        self.parkur12_model_path = str(self.get_parameter("parkur12_model_path").value)
        self.parkur3_model_path = str(self.get_parameter("parkur3_model_path").value)

        # Ensure confidence threshold parameter falls back to yolo_conf_threshold = 0.30
        conf_val = float(self.get_parameter("yolo_conf_threshold").value)
        if conf_val <= 0.0:
            conf_val = float(self.get_parameter("confidence_threshold").value)
        self.confidence_threshold = conf_val

        self.model_input_width = int(self.get_parameter("model_input_width").value)
        self.model_input_height = int(self.get_parameter("model_input_height").value)

        self.save_video = bool(self.get_parameter("save_video").value)
        self.video_output_dir = Path(str(self.get_parameter("video_output_dir").value)).expanduser()
        self.video_fps = max(float(self.get_parameter("video_fps").value), 1.0)

        self.draw_timestamp = bool(self.get_parameter("draw_timestamp").value)
        self.draw_center = bool(self.get_parameter("draw_center").value)
        self.draw_detections = bool(self.get_parameter("draw_detections").value)

        self.class_names = {
            0: 'kirmizi_duba',
            1: 'sari_duba',
            2: 'siyah_duba',
            3: 'turuncu_duba',
            4: 'yesil_duba'
        }

        # Frame buffer and locks
        self.latest_frame = None
        self.latest_stamp = None
        self.latest_msg = None
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.running = True

        # Hailo variables
        self.vdevice = None
        self.hef = None
        self.network_group = None
        self.network_group_params = None
        self.input_vstreams_params = None
        self.output_vstreams_params = None
        self.infer_pipeline = None
        self.activation_context = None
        self.active_model_name = None

        self.video_writer = None
        self.video_path = None

        self._preprocess_scale = 1.0
        self._preprocess_dx = 0
        self._preprocess_dy = 0
        self._original_size = (640, 640)
        self._first_frame_logged = False

        # Initialize VDevice & load initial model (parkur12)
        self.init_hailo()

        # Publishers
        self.processed_image_pub = self.create_publisher(Image, self.processed_image_topic, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.obstacles_pub = self.create_publisher(String, self.obstacles_topic, 10)

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        if MissionStatus is not None:
            self.mission_status_sub = self.create_subscription(
                MissionStatus,
                "/albatros/mission/status",
                self.mission_status_callback,
                10
            )

        if VehicleState is not None:
            self.vehicle_state_sub = self.create_subscription(
                VehicleState,
                "/albatros/state",
                self.vehicle_state_callback,
                10
            )

        # Worker thread
        self.worker_thread = threading.Thread(target=self.inference_worker, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("YOLO Hailo node started with dual-HEF switching and 0.30 conf threshold.")
        self.get_logger().info(f"Subscribing: {self.input_image_topic}")
        self.get_logger().info(f"Global conf threshold: {self.confidence_threshold:.2f}")

    def resolve_model_path(self, raw_path_str: str) -> Path:
        """Resolves model path looking in share, workspace, and relative locations."""
        p = Path(raw_path_str).expanduser()
        if p.is_absolute() and p.exists():
            return p

        candidates = [p, Path.cwd() / p]

        if get_package_share_directory is not None:
            for pkg in ["albatros_tahta", "albatros_system"]:
                try:
                    share_dir = Path(get_package_share_directory(pkg))
                    candidates.append(share_dir / raw_path_str)
                    candidates.append(share_dir / "models" / p.name)
                except Exception:
                    pass

        current_file = Path(__file__).resolve()
        package_dir = current_file.parent
        workspace_src = package_dir.parents[2] if len(package_dir.parents) >= 3 else package_dir
        candidates.extend([
            package_dir / raw_path_str,
            package_dir / "models" / p.name,
            workspace_src / "albatros_tahta" / raw_path_str,
            workspace_src / "albatros_system" / raw_path_str,
            workspace_src / "models" / p.name,
            Path.cwd() / "models" / p.name,
        ])

        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()

        return p

    def init_hailo(self):
        """Initializes VDevice and loads initial parkur12 HEF model."""
        self.get_logger().info("Creating Hailo VDevice...")
        self.vdevice = VDevice()

        initial_model_path = self.resolve_model_path(self.parkur12_model_path)
        if not initial_model_path.exists():
            fallback = self.resolve_model_path(self.get_parameter("model_path").value)
            if fallback.exists():
                initial_model_path = fallback

        if not initial_model_path.exists():
            raise FileNotFoundError(f"Hailo HEF model file not found: {self.parkur12_model_path}")

        self.load_hef_pipeline(initial_model_path, "parkur12")

    def load_hef_pipeline(self, model_path: Path, model_key: str):
        """Configures HEF pipeline for given model_path and updates active_model_name."""
        self.get_logger().info(f"[YOLO] Loading Hailo HEF model ({model_key}): {model_path}")
        self.hef = HEF(str(model_path))

        input_infos = self.hef.get_input_vstream_infos()
        for i, info in enumerate(input_infos):
            self.get_logger().info(f"HEF Input [{i}]: name={info.name}, shape={info.shape}, format={info.format}")
            if i == 0 and len(info.shape) >= 3:
                self.model_input_height = int(info.shape[0])
                self.model_input_width = int(info.shape[1])
                self.get_logger().info(f"Updated model dimensions: {self.model_input_width}x{self.model_input_height}")

        output_infos = self.hef.get_output_vstream_infos()
        for i, info in enumerate(output_infos):
            self.get_logger().info(f"HEF Output [{i}]: name={info.name}, shape={info.shape}, format={info.format}")

        configure_params = ConfigureParams.create_from_hef(hef=self.hef, interface=HailoStreamInterface.PCIe)
        self.network_group = self.vdevice.configure(self.hef, configure_params)[0]
        self.network_group_params = self.network_group.create_params()

        self.input_vstreams_params = InputVStreamParams.make_from_network_group(
            self.network_group, quantized=False, format_type=FormatType.UINT8
        )
        self.output_vstreams_params = OutputVStreamParams.make_from_network_group(
            self.network_group, quantized=False, format_type=FormatType.FLOAT32
        )

        self.infer_pipeline = InferVStreams(self.network_group, self.input_vstreams_params, self.output_vstreams_params)
        self.activation_context = self.network_group.activate(self.network_group_params)
        self.activation_context.__enter__()
        self.infer_pipeline.__enter__()

        self.active_model_name = model_key
        self.get_logger().info(f"[YOLO] Hailo inference pipeline ready for active model: '{self.active_model_name}'.")

    def switch_hailo_model(self, requested_model: str):
        """Safely switches between parkur12 and parkur3 HEF models without reload per frame."""
        if requested_model == self.active_model_name:
            return

        with self.frame_lock:
            if requested_model == self.active_model_name:
                return

            self.get_logger().info(
                f"[YOLO] MODEL SWITCH REQUESTED: active='{self.active_model_name}' -> requested='{requested_model}'"
            )

            raw_path = self.parkur3_model_path if requested_model == "parkur3" else self.parkur12_model_path
            resolved_path = self.resolve_model_path(raw_path)

            if not resolved_path.exists():
                self.get_logger().error(f"[YOLO] Cannot switch HEF model: File not found at {resolved_path}")
                return

            if self.infer_pipeline is not None:
                try:
                    self.infer_pipeline.__exit__(None, None, None)
                except Exception as e:
                    self.get_logger().warn(f"[YOLO] Error releasing infer_pipeline on switch: {e}")
                self.infer_pipeline = None

            if self.activation_context is not None:
                try:
                    self.activation_context.__exit__(None, None, None)
                except Exception as e:
                    self.get_logger().warn(f"[YOLO] Error releasing activation_context on switch: {e}")
                self.activation_context = None

            try:
                self.load_hef_pipeline(resolved_path, requested_model)
            except Exception as e:
                self.get_logger().error(f"[YOLO] Model switch to '{requested_model}' failed: {e}")

    def mission_status_callback(self, msg):
        current_parkur = msg.current_parkur
        mission_state = str(msg.mission_state).upper()

        if current_parkur == 3 or "PARKUR3" in mission_state or "PARKUR_3" in mission_state:
            target_model = "parkur3"
        else:
            target_model = "parkur12"

        if target_model != self.active_model_name:
            self.switch_hailo_model(target_model)

    def vehicle_state_callback(self, msg):
        current_parkur = msg.current_parkur
        mission_state = str(msg.mission_state).upper()

        if current_parkur == 3 or "PARKUR3" in mission_state or "PARKUR_3" in mission_state:
            target_model = "parkur3"
        else:
            target_model = "parkur12"

        if target_model != self.active_model_name:
            self.switch_hailo_model(target_model)

    def ros_image_to_cv2(self, msg: Image):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "mono8":
            frame = data.reshape((msg.height, msg.step))
            frame = frame[:, :msg.width]
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            frame = data.reshape((msg.height, msg.step))
            frame = frame[:, :msg.width * 3]
            frame = frame.reshape((msg.height, msg.width, 3))
            if msg.encoding in ["rgb8", "RGB8"]:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(frame)

    def image_callback(self, msg: Image):
        try:
            frame = self.ros_image_to_cv2(msg)
        except Exception as exc:
            self.get_logger().error(f"Image conversion error: {exc}", throttle_duration_sec=2.0)
            return

        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec
        stamp_float = stamp_sec + stamp_nanosec * 1e-9
        if stamp_sec == 0 and stamp_nanosec == 0:
            now_msg = self.get_clock().now().to_msg()
            stamp_float = now_msg.sec + now_msg.nanosec * 1e-9

        with self.frame_lock:
            self.latest_frame = frame
            self.latest_stamp = stamp_float
            self.latest_msg = msg
        self.frame_event.set()

    def inference_worker(self):
        while self.running and rclpy.ok():
            if not self.frame_event.wait(timeout=0.1):
                continue

            self.frame_event.clear()

            with self.frame_lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()
                stamp = self.latest_stamp
                input_msg = self.latest_msg

            try:
                processed_frame, detections = self.run_yolo(frame, stamp)

                if self.draw_timestamp:
                    self.draw_time_label(processed_frame, stamp)

                self.publish_processed_image(processed_frame, input_msg)
                self.publish_detections(detections, stamp, input_msg.header.frame_id)
                self.publish_obstacle_candidates(detections, stamp)
                self.write_video(processed_frame)
            except Exception as e:
                self.get_logger().error(f"Inference error: {e}", throttle_duration_sec=2.0)

    def preprocess_frame(self, frame):
        ih, iw = frame.shape[:2]
        h, w = self.model_input_height, self.model_input_width

        scale = min(w / iw, h / ih)
        nw, nh = int(iw * scale), int(ih * scale)
        resized = cv2.resize(frame, (nw, nh))

        canvas = np.full((h, w, 3), 128, dtype=np.uint8)
        dx, dy = (w - nw) // 2, (h - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = resized

        canvas_rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

        self._preprocess_scale = scale
        self._preprocess_dx = dx
        self._preprocess_dy = dy
        self._original_size = (iw, ih)

        return canvas_rgb

    def run_yolo(self, frame, stamp_float):
        processed = frame.copy()
        detections = []

        if self.infer_pipeline is None:
            return processed, detections

        input_frame = self.preprocess_frame(frame)
        if not self._first_frame_logged:
            self.get_logger().info(
                f"First frame input: shape={input_frame.shape}, dtype={input_frame.dtype}, nbytes={input_frame.nbytes}"
            )
            self._first_frame_logged = True

        input_name = self.hef.get_input_vstream_infos()[0].name
        input_data = {
            input_name: np.expand_dims(input_frame, axis=0)
        }

        results = self.infer_pipeline.infer(input_data)
        detections = self.postprocess_results(results, stamp_float)

        if self.draw_detections:
            for det in detections:
                self.draw_detection(
                    processed,
                    det["bbox"]["x_min"],
                    det["bbox"]["y_min"],
                    det["bbox"]["x_max"],
                    det["bbox"]["y_max"],
                    det["center"]["x"],
                    det["center"]["y"],
                    det["class_name"],
                    det["confidence"]
                )

        return processed, detections

    def postprocess_results(self, results, stamp_float):
        detections = []
        scale = self._preprocess_scale
        dx = self._preprocess_dx
        dy = self._preprocess_dy
        orig_w, orig_h = self._original_size

        nms_output = None
        for info in self.hef.get_output_vstream_infos():
            out_tensor = results[info.name][0]
            if "nms" in info.name.lower() or out_tensor.shape[-1] == 6:
                nms_output = out_tensor
                break

        if nms_output is None and len(self.hef.get_output_vstream_infos()) > 0:
            info = self.hef.get_output_vstream_infos()[0]
            out_tensor = results[info.name][0]
            if out_tensor.shape[-1] == 6:
                nms_output = out_tensor

        if nms_output is not None:
            for box in nms_output:
                if len(box) >= 6:
                    ymin, xmin, ymax, xmax, confidence, class_id = box[:6]

                    if confidence < self.confidence_threshold:
                        self.get_logger().debug(
                            f"[YOLO] Detection rejected conf={confidence:.2f} threshold={self.confidence_threshold:.2f}"
                        )
                        continue

                    class_id = int(class_id)
                    class_name = self.class_names.get(class_id, str(class_id))

                    if xmax <= 1.0 and ymax <= 1.0:
                        xmin_px = xmin * self.model_input_width
                        xmax_px = xmax * self.model_input_width
                        ymin_px = ymin * self.model_input_height
                        ymax_px = ymax * self.model_input_height
                    else:
                        xmin_px, ymin_px, xmax_px, ymax_px = xmin, ymin, xmax, ymax

                    x1 = int((xmin_px - dx) / scale)
                    y1 = int((ymin_px - dy) / scale)
                    x2 = int((xmax_px - dx) / scale)
                    y2 = int((ymax_px - dy) / scale)

                    x1 = max(0, min(x1, orig_w - 1))
                    y1 = max(0, min(y1, orig_h - 1))
                    x2 = max(0, min(x2, orig_w - 1))
                    y2 = max(0, min(y2, orig_h - 1))

                    if x1 > x2:
                        x1, x2 = x2, x1
                    if y1 > y2:
                        y1, y2 = y2, y1

                    width = int(x2 - x1)
                    height = int(y2 - y1)

                    if width <= 0 or height <= 0:
                        continue

                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    detection = {
                        "stamp": stamp_float,
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": float(confidence),
                        "model_active": self.active_model_name,
                        "bbox": {
                            "x_min": x1,
                            "y_min": y1,
                            "x_max": x2,
                            "y_max": y2,
                            "width": width,
                            "height": height
                        },
                        "center": {
                            "x": cx,
                            "y": cy
                        }
                    }
                    detections.append(detection)
        else:
            self.get_logger().warn(
                "NMS output format mismatch. Ensure HEF was compiled with nms_postprocess.",
                throttle_duration_sec=5.0
            )

        return detections

    def draw_detection(self, frame, x1, y1, x2, y2, cx, cy, class_name, confidence):
        label = f"{class_name} {confidence:.2f} [{self.active_model_name}]"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        text_w, text_h = text_size
        label_y1 = max(y1 - text_h - 8, 0)
        cv2.rectangle(frame, (x1, label_y1), (x1 + text_w + 8, label_y1 + text_h + 8), (0, 255, 0), -1)
        cv2.putText(frame, label, (x1 + 4, label_y1 + text_h + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)

        if self.draw_center:
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"({cx},{cy})", (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

    def draw_time_label(self, frame, stamp_float):
        dt_text = datetime.fromtimestamp(stamp_float).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        label = f"stamp: {stamp_float:.3f} | model: {self.active_model_name} | {dt_text}"
        cv2.rectangle(frame, (8, 8), (620, 38), (0, 0, 0), -1)
        cv2.putText(frame, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    def publish_processed_image(self, frame, input_msg):
        if frame is None:
            return

        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif len(frame.shape) == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)

        frame = np.ascontiguousarray(frame)

        out_msg = Image()
        out_msg.header.stamp = input_msg.header.stamp
        out_msg.header.frame_id = input_msg.header.frame_id or "camera_frame"
        out_msg.height = frame.shape[0]
        out_msg.width = frame.shape[1]
        out_msg.encoding = "bgr8"
        out_msg.is_bigendian = 0
        out_msg.step = frame.shape[1] * 3
        out_msg.data = frame.tobytes()

        self.processed_image_pub.publish(out_msg)

    def publish_detections(self, detections, stamp_float, frame_id):
        payload = {
            "stamp": stamp_float,
            "frame_id": frame_id or "camera_frame",
            "active_model": self.active_model_name,
            "detection_count": len(detections),
            "detections": detections
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(msg)

    def publish_obstacle_candidates(self, detections, stamp_float):
        obstacles = []
        for det in detections:
            class_name_lower = det["class_name"].lower()
            obstacle_type = "unknown"

            if "yellow" in class_name_lower or "sari" in class_name_lower or "sarı" in class_name_lower:
                obstacle_type = "obstacle_buoy"
            elif "orange" in class_name_lower or "turuncu" in class_name_lower:
                obstacle_type = "border_buoy"
            elif "red" in class_name_lower or "kirmizi" in class_name_lower or "kırmızı" in class_name_lower:
                obstacle_type = "target_or_colored_buoy"
            elif "green" in class_name_lower or "yesil" in class_name_lower or "yeşil" in class_name_lower:
                obstacle_type = "target_or_colored_buoy"
            elif "black" in class_name_lower or "siyah" in class_name_lower:
                obstacle_type = "black_buoy"

            obstacles.append({
                "stamp": stamp_float,
                "type": obstacle_type,
                "class_name": det["class_name"],
                "confidence": det["confidence"],
                "bbox": det["bbox"],
                "center": det["center"]
            })

        payload = {
            "stamp": stamp_float,
            "source": "yolo_node",
            "active_model": self.active_model_name,
            "obstacle_count": len(obstacles),
            "obstacles": obstacles
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.obstacles_pub.publish(msg)

    def write_video(self, frame):
        if not self.save_video:
            return

        if self.video_writer is None:
            self.init_video_writer(frame)

        if self.video_writer is not None:
            try:
                self.video_writer.write(frame)
            except Exception as e:
                self.get_logger().error(f"Failed to write video frame: {e}", throttle_duration_sec=5.0)

    def init_video_writer(self, frame):
        try:
            self.video_output_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.video_path = self.video_output_dir / f"albatros_yolo_processed_{timestamp}.mp4"
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(str(self.video_path), fourcc, self.video_fps, (width, height))

            if not self.video_writer.isOpened():
                self.get_logger().error(f"Could not open video writer: {self.video_path}")
                self.video_writer = None
            else:
                self.get_logger().info(f"Recording processed video: {self.video_path}")
        except Exception as e:
            self.get_logger().error(f"Error initializing video writer: {e}")
            self.video_writer = None

    def destroy_node(self):
        self.running = False
        self.frame_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)

        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Video saved: {self.video_path}")

        if self.infer_pipeline is not None:
            try:
                self.infer_pipeline.__exit__(None, None, None)
            except Exception as e:
                self.get_logger().warn(f"Error releasing infer_pipeline: {e}")

        if self.activation_context is not None:
            try:
                self.activation_context.__exit__(None, None, None)
            except Exception as e:
                self.get_logger().warn(f"Error releasing activation context: {e}")

        if self.vdevice is not None:
            try:
                self.vdevice.release()
                self.get_logger().info("Hailo VDevice released.")
            except Exception as e:
                self.get_logger().warn(f"Error releasing VDevice: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = YoloNode()
    except RuntimeError as e:
        print(f"[FATAL] Failed to initialize YOLO Node: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()