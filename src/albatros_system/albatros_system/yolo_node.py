#!/usr/bin/env python3
#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloNode(Node):
    """
    Albatros YOLO perception node.

    Subscribes:
        /camera/image_raw          sensor_msgs/Image

    Publishes:
        /yolo/processed_image      sensor_msgs/Image
        /yolo/detections           std_msgs/String JSON
        /perception/obstacles      std_msgs/String JSON

    Saves:
        Processed camera video as mp4 with timestamp, bounding boxes,
        class names and confidence values.
    """

    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter("processed_image_topic", "/yolo/processed_image")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("obstacles_topic", "/perception/obstacles")

        self.declare_parameter("model_path", "models/best.pt")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "cpu")

        self.declare_parameter("save_video", True)
        self.declare_parameter("video_output_dir", "~/albatros_outputs/videos")
        self.declare_parameter("video_fps", 10.0)

        self.declare_parameter("draw_timestamp", True)
        self.declare_parameter("draw_center", True)
        self.declare_parameter("draw_detections", True)

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.processed_image_topic = str(self.get_parameter("processed_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.obstacles_topic = str(self.get_parameter("obstacles_topic").value)

        self.model_path = str(self.get_parameter("model_path").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.device = str(self.get_parameter("device").value)

        self.save_video = bool(self.get_parameter("save_video").value)
        self.video_output_dir = Path(
            str(self.get_parameter("video_output_dir").value)
        ).expanduser()
        self.video_fps = max(float(self.get_parameter("video_fps").value), 1.0)

        self.draw_timestamp = bool(self.get_parameter("draw_timestamp").value)
        self.draw_center = bool(self.get_parameter("draw_center").value)
        self.draw_detections = bool(self.get_parameter("draw_detections").value)

        self.bridge = CvBridge()
        self.model = self.load_model()

        self.processed_image_pub = self.create_publisher(
            Image,
            self.processed_image_topic,
            10
        )

        self.detections_pub = self.create_publisher(
            String,
            self.detections_topic,
            10
        )

        self.obstacles_pub = self.create_publisher(
            String,
            self.obstacles_topic,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.video_writer = None
        self.video_path = None

        self.get_logger().info("YOLO node started.")
        self.get_logger().info(f"Subscribing: {self.input_image_topic}")
        self.get_logger().info(f"Publishing processed image: {self.processed_image_topic}")
        self.get_logger().info(f"Publishing detections JSON: {self.detections_topic}")
        self.get_logger().info(f"Publishing obstacle candidates: {self.obstacles_topic}")

    def load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().error(
                "ultralytics is not installed. Install with: pip install ultralytics"
            )
            raise exc

        model_path = Path(self.model_path).expanduser()

        if not model_path.is_absolute():
            current_file = Path(__file__).resolve()
            package_dir = current_file.parent
            workspace_guess = package_dir.parent.parent.parent
            candidate_paths = [
                package_dir / self.model_path,
                workspace_guess / self.model_path,
                Path.cwd() / self.model_path,
            ]

            for candidate in candidate_paths:
                if candidate.exists():
                    model_path = candidate
                    break

        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLO model file not found: {model_path}. "
                "Put best.pt into models/ or pass model_path parameter."
            )

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        return YOLO(str(model_path))

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge error: {exc}")
            return

        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec
        stamp_float = stamp_sec + stamp_nanosec * 1e-9

        if stamp_sec == 0 and stamp_nanosec == 0:
            now_msg = self.get_clock().now().to_msg()
            stamp_sec = now_msg.sec
            stamp_nanosec = now_msg.nanosec
            stamp_float = stamp_sec + stamp_nanosec * 1e-9

        processed_frame, detections = self.run_yolo(frame, stamp_float)

        if self.draw_timestamp:
            self.draw_time_label(processed_frame, stamp_float)

        self.publish_processed_image(processed_frame, msg)
        self.publish_detections(detections, stamp_float, msg.header.frame_id)
        self.publish_obstacle_candidates(detections, stamp_float)
        self.write_video(processed_frame)

    def run_yolo(self, frame, stamp_float):
        processed = frame.copy()
        detections = []

        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False
        )

        if not results:
            return processed, detections

        result = results[0]
        names = result.names

        if result.boxes is None:
            return processed, detections

        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = [int(v) for v in xyxy]

            confidence = float(box.conf[0].cpu().numpy())
            class_id = int(box.cls[0].cpu().numpy())
            class_name = str(names.get(class_id, class_id))

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            width = int(x2 - x1)
            height = int(y2 - y1)

            detection = {
                "stamp": stamp_float,
                "class_id": class_id,
                "class_name": class_name,
                "confidence": confidence,
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

            if self.draw_detections:
                self.draw_detection(
                    processed,
                    x1,
                    y1,
                    x2,
                    y2,
                    cx,
                    cy,
                    class_name,
                    confidence
                )

        return processed, detections

    def draw_detection(self, frame, x1, y1, x2, y2, cx, cy, class_name, confidence):
        label = f"{class_name} {confidence:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        text_size, _ = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            2
        )

        text_w, text_h = text_size#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloNode(Node):
    """
    Albatros YOLO perception node.

    Subscribes:
        /camera/image_raw          sensor_msgs/Image

    Publishes:
        /yolo/processed_image      sensor_msgs/Image
        /yolo/detections           std_msgs/String JSON
        /perception/obstacles      std_msgs/String JSON

    Saves:
        Processed camera video as mp4 with timestamp, bounding boxes,
        class names and confidence values.
    """

    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter("processed_image_topic", "/yolo/processed_image")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("obstacles_topic", "/perception/obstacles")

        self.declare_parameter("model_path", "models/best.pt")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "cpu")

        self.declare_parameter("save_video", True)
        self.declare_parameter("video_output_dir", "~/albatros_outputs/videos")
        self.declare_parameter("video_fps", 10.0)

        self.declare_parameter("draw_timestamp", True)
        self.declare_parameter("draw_center", True)
        self.declare_parameter("draw_detections", True)

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.processed_image_topic = str(self.get_parameter("processed_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.obstacles_topic = str(self.get_parameter("obstacles_topic").value)

        self.model_path = str(self.get_parameter("model_path").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.device = str(self.get_parameter("device").value)

        self.save_video = bool(self.get_parameter("save_video").value)
        self.video_output_dir = Path(
            str(self.get_parameter("video_output_dir").value)
        ).expanduser()
        self.video_fps = max(float(self.get_parameter("video_fps").value), 1.0)

        self.draw_timestamp = bool(self.get_parameter("draw_timestamp").value)
        self.draw_center = bool(self.get_parameter("draw_center").value)
        self.draw_detections = bool(self.get_parameter("draw_detections").value)

        self.bridge = CvBridge()
        self.model = self.load_model()

        self.processed_image_pub = self.create_publisher(
            Image,
            self.processed_image_topic,
            10
        )

        self.detections_pub = self.create_publisher(
            String,
            self.detections_topic,
            10
        )

        self.obstacles_pub = self.create_publisher(
            String,
            self.obstacles_topic,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.video_writer = None
        self.video_path = None

        self.get_logger().info("YOLO node started.")
        self.get_logger().info(f"Subscribing: {self.input_image_topic}")
        self.get_logger().info(f"Publishing processed image: {self.processed_image_topic}")
        self.get_logger().info(f"Publishing detections JSON: {self.detections_topic}")
        self.get_logger().info(f"Publishing obstacle candidates: {self.obstacles_topic}")

    def load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().error(
                "ultralytics is not installed. Install with: pip install ultralytics"
            )
            raise exc

        model_path = Path(self.model_path).expanduser()

        if not model_path.is_absolute():
            current_file = Path(__file__).resolve()
            package_dir = current_file.parent
            workspace_guess = package_dir.parent.parent.parent
            candidate_paths = [
                package_dir / self.model_path,
                workspace_guess / self.model_path,
                Path.cwd() / self.model_path,
            ]

            for candidate in candidate_paths:
                if candidate.exists():
                    model_path = candidate
                    break

        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLO model file not found: {model_path}. "
                "Put best.pt into models/ or pass model_path parameter."
            )

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        return YOLO(str(model_path))

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge error: {exc}")
            return

        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec
        stamp_float = stamp_sec + stamp_nanosec * 1e-9

        if stamp_sec == 0 and stamp_nanosec == 0:
            now_msg = self.get_clock().now().to_msg()
            stamp_sec = now_msg.sec
            stamp_nanosec = now_msg.nanosec
            stamp_float = stamp_sec + stamp_nanosec * 1e-9

        processed_frame, detections = self.run_yolo(frame, stamp_float)

        if self.draw_timestamp:
            self.draw_time_label(processed_frame, stamp_float)

        self.publish_processed_image(processed_frame, msg)
        self.publish_detections(detections, stamp_float, msg.header.frame_id)
        self.publish_obstacle_candidates(detections, stamp_float)
        self.write_video(processed_frame)

    def run_yolo(self, frame, stamp_float):
        processed = frame.copy()
        detections = []

        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False
        )

        if not results:
            return processed, detections

        result = results[0]
        names = result.names

        if result.boxes is None:
            return processed, detections

        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = [int(v) for v in xyxy]

            confidence = float(box.conf[0].cpu().numpy())
            class_id = int(box.cls[0].cpu().numpy())
            class_name = str(names.get(class_id, class_id))

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            width = int(x2 - x1)
            height = int(y2 - y1)

            detection = {
                "stamp": stamp_float,
                "class_id": class_id,
                "class_name": class_name,
                "confidence": confidence,
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

            if self.draw_detections:
                self.draw_detection(
                    processed,
                    x1,
                    y1,
                    x2,
                    y2,
                    cx,
                    cy,
                    class_name,
                    confidence
                )

        return processed, detections

    def draw_detection(self, frame, x1, y1, x2, y2, cx, cy, class_name, confidence):
        label = f"{class_name} {confidence:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        text_size, _ = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            2
        )

        text_w, text_h = text_size
        label_y1 = max(y1 - text_h - 8, 0)

        cv2.rectangle(
            frame,
            (x1, label_y1),
            (x1 + text_w + 8, label_y1 + text_h + 8),
            (0, 255, 0),
            -1
        )

        cv2.putText(
            frame,
            label,
            (x1 + 4, label_y1 + text_h + 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2,
            cv2.LINE_AA
        )

        if self.draw_center:
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"({cx},{cy})",
                (cx + 8, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

    def draw_time_label(self, frame, stamp_float):
        dt_text = datetime.fromtimestamp(stamp_float).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        label = f"stamp: {stamp_float:.3f} | {dt_text}"

        cv2.rectangle(frame, (8, 8), (620, 38), (0, 0, 0), -1)
        cv2.putText(
            frame,
            label,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloNode(Node):
    """
    Albatros YOLO perception node.

    Subscribes:
        /camera/image_raw          sensor_msgs/Image

    Publishes:
        /yolo/processed_image      sensor_msgs/Image
        /yolo/detections           std_msgs/String JSON
        /perception/obstacles      std_msgs/String JSON

    Saves:
        Processed camera video as mp4 with timestamp, bounding boxes,
        class names and confidence values.
    """

    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter("processed_image_topic", "/yolo/processed_image")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("obstacles_topic", "/perception/obstacles")

        self.declare_parameter("model_path", "models/best.pt")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "cpu")

        self.declare_parameter("save_video", True)
        self.declare_parameter("video_output_dir", "~/albatros_outputs/videos")
        self.declare_parameter("video_fps", 10.0)

        self.declare_parameter("draw_timestamp", True)
        self.declare_parameter("draw_center", True)
        self.declare_parameter("draw_detections", True)

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.processed_image_topic = str(self.get_parameter("processed_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.obstacles_topic = str(self.get_parameter("obstacles_topic").value)

        self.model_path = str(self.get_parameter("model_path").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.device = str(self.get_parameter("device").value)

        self.save_video = bool(self.get_parameter("save_video").value)
        self.video_output_dir = Path(
            str(self.get_parameter("video_output_dir").value)
        ).expanduser()
        self.video_fps = max(float(self.get_parameter("video_fps").value), 1.0)

        self.draw_timestamp = bool(self.get_parameter("draw_timestamp").value)
        self.draw_center = bool(self.get_parameter("draw_center").value)
        self.draw_detections = bool(self.get_parameter("draw_detections").value)

        self.bridge = CvBridge()
        self.model = self.load_model()

        self.processed_image_pub = self.create_publisher(
            Image,
            self.processed_image_topic,
            10
        )

        self.detections_pub = self.create_publisher(
            String,
            self.detections_topic,
            10
        )

        self.obstacles_pub = self.create_publisher(
            String,
            self.obstacles_topic,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.video_writer = None
        self.video_path = None

        self.get_logger().info("YOLO node started.")
        self.get_logger().info(f"Subscribing: {self.input_image_topic}")
        self.get_logger().info(f"Publishing processed image: {self.processed_image_topic}")
        self.get_logger().info(f"Publishing detections JSON: {self.detections_topic}")
        self.get_logger().info(f"Publishing obstacle candidates: {self.obstacles_topic}")

    def load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().error(
                "ultralytics is not installed. Install with: pip install ultralytics"
            )
            raise exc

        model_path = Path(self.model_path).expanduser()

        if not model_path.is_absolute():
            current_file = Path(__file__).resolve()
            package_dir = current_file.parent
            workspace_guess = package_dir.parent.parent.parent
            candidate_paths = [
                package_dir / self.model_path,
                workspace_guess / self.model_path,
                Path.cwd() / self.model_path,
            ]

            for candidate in candidate_paths:
                if candidate.exists():
                    model_path = candidate
                    break

        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLO model file not found: {model_path}. "
                "Put best.pt into models/ or pass model_path parameter."
            )

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        return YOLO(str(model_path))

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge error: {exc}")
            return

        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec
        stamp_float = stamp_sec + stamp_nanosec * 1e-9

        if stamp_sec == 0 and stamp_nanosec == 0:
            now_msg = self.get_clock().now().to_msg()
            stamp_sec = now_msg.sec
            stamp_nanosec = now_msg.nanosec
            stamp_float = stamp_sec + stamp_nanosec * 1e-9

        processed_frame, detections = self.run_yolo(frame, stamp_float)

        if self.draw_timestamp:
            self.draw_time_label(processed_frame, stamp_float)

        self.publish_processed_image(processed_frame, msg)
        self.publish_detections(detections, stamp_float, msg.header.frame_id)
        self.publish_obstacle_candidates(detections, stamp_float)
        self.write_video(processed_frame)

    def run_yolo(self, frame, stamp_float):
        processed = frame.copy()
        detections = []

        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False
        )

        if not results:
            return processed, detections

        result = results[0]
        names = result.names

        if result.boxes is None:
            return processed, detections

        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = [int(v) for v in xyxy]

            confidence = float(box.conf[0].cpu().numpy())
            class_id = int(box.cls[0].cpu().numpy())
            class_name = str(names.get(class_id, class_id))

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            width = int(x2 - x1)
            height = int(y2 - y1)

            detection = {
                "stamp": stamp_float,
                "class_id": class_id,
                "class_name": class_name,
                "confidence": confidence,
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

            if self.draw_detections:
                self.draw_detection(
                    processed,
                    x1,
                    y1,
                    x2,
                    y2,
                    cx,
                    cy,
                    class_name,
                    confidence
                )

        return processed, detections

    def draw_detection(self, frame, x1, y1, x2, y2, cx, cy, class_name, confidence):
        label = f"{class_name} {confidence:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        text_size, _ = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            2
        )

        text_w, text_h = text_size
        label_y1 = max(y1 - text_h - 8, 0)

        cv2.rectangle(
            frame,
            (x1, label_y1),
            (x1 + text_w + 8, label_y1 + text_h + 8),
            (0, 255, 0),
            -1
        )

        cv2.putText(
            frame,
            label,
            (x1 + 4, label_y1 + text_h + 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2,
            cv2.LINE_AA
        )

        if self.draw_center:
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"({cx},{cy})",
                (cx + 8, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

    def draw_time_label(self, frame, stamp_float):
        dt_text = datetime.fromtimestamp(stamp_float).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        label = f"stamp: {stamp_float:.3f} | {dt_text}"

        cv2.rectangle(frame, (8, 8), (620, 38), (0, 0, 0), -1)
        cv2.putText(
            frame,
            label,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

    def publish_processed_image(self, frame, input_msg):
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge publish error: {exc}")
            return

        out_msg.header.stamp = input_msg.header.stamp
        out_msg.header.frame_id = input_msg.header.frame_id or "camera_frame"

        self.processed_image_pub.publish(out_msg)

    def publish_detections(self, detections, stamp_float, frame_id):
        payload = {
            "stamp": stamp_float,
            "frame_id": frame_id or "camera_frame",
            "detection_count": len(detections),
            "detections": detections
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(msg)

    def publish_obstacle_candidates(self, detections, stamp_float):
        """
        This output is for costmap / mission / decision nodes.

        It does not create the final costmap. It only sends detected objects
        in a simple JSON format so the costmap node can use them.
        """
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
            self.video_writer.write(frame)

    def init_video_writer(self, frame):
        self.video_output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = self.video_output_dir / f"albatros_yolo_processed_{timestamp}.mp4"

        height, width = frame.shape[:2]

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(
            str(self.video_path),
            fourcc,
            self.video_fps,
            (width, height)
        )

        if not self.video_writer.isOpened():
            self.get_logger().error(f"Could not open video writer: {self.video_path}")
            self.video_writer = None
            return

        self.get_logger().info(f"Recording processed video: {self.video_path}")

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Video saved: {self.video_path}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

    def publish_processed_image(self, frame, input_msg):
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge publish error: {exc}")
            return

        out_msg.header.stamp = input_msg.header.stamp
        out_msg.header.frame_id = input_msg.header.frame_id or "camera_frame"

        self.processed_image_pub.publish(out_msg)

    def publish_detections(self, detections, stamp_float, frame_id):
        payload = {
            "stamp": stamp_float,
            "frame_id": frame_id or "camera_frame",
            "detection_count": len(detections),
            "detections": detections
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(msg)

    def publish_obstacle_candidates(self, detections, stamp_float):
        """
        This output is for costmap / mission / decision nodes.

        It does not create the final costmap. It only sends detected objects
        in a simple JSON format so the costmap node can use them.
        """
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
            self.video_writer.write(frame)

    def init_video_writer(self, frame):
        self.video_output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = self.video_output_dir / f"albatros_yolo_processed_{timestamp}.mp4"

        height, width = frame.shape[:2]

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(
            str(self.video_path),
            fourcc,
            self.video_fps,
            (width, height)
        )

        if not self.video_writer.isOpened():
            self.get_logger().error(f"Could not open video writer: {self.video_path}")
            self.video_writer = None
            return

        self.get_logger().info(f"Recording processed video: {self.video_path}")

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Video saved: {self.video_path}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
        label_y1 = max(y1 - text_h - 8, 0)

        cv2.rectangle(
            frame,
            (x1, label_y1),
            (x1 + text_w + 8, label_y1 + text_h + 8),
            (0, 255, 0),
            -1
        )

        cv2.putText(
            frame,
            label,
            (x1 + 4, label_y1 + text_h + 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            2,
            cv2.LINE_AA
        )

        if self.draw_center:
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"({cx},{cy})",
                (cx + 8, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

    def draw_time_label(self, frame, stamp_float):
        dt_text = datetime.fromtimestamp(stamp_float).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        label = f"stamp: {stamp_float:.3f} | {dt_text}"

        cv2.rectangle(frame, (8, 8), (620, 38), (0, 0, 0), -1)
        cv2.putText(
            frame,
            label,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

    def publish_processed_image(self, frame, input_msg):
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge publish error: {exc}")
            return

        out_msg.header.stamp = input_msg.header.stamp
        out_msg.header.frame_id = input_msg.header.frame_id or "camera_frame"

        self.processed_image_pub.publish(out_msg)

    def publish_detections(self, detections, stamp_float, frame_id):
        payload = {
            "stamp": stamp_float,
            "frame_id": frame_id or "camera_frame",
            "detection_count": len(detections),
            "detections": detections
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(msg)

    def publish_obstacle_candidates(self, detections, stamp_float):
        """
        This output is for costmap / mission / decision nodes.

        It does not create the final costmap. It only sends detected objects
        in a simple JSON format so the costmap node can use them.
        """
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
            self.video_writer.write(frame)

    def init_video_writer(self, frame):
        self.video_output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = self.video_output_dir / f"albatros_yolo_processed_{timestamp}.mp4"

        height, width = frame.shape[:2]

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(
            str(self.video_path),
            fourcc,
            self.video_fps,
            (width, height)
        )

        if not self.video_writer.isOpened():
            self.get_logger().error(f"Could not open video writer: {self.video_path}")
            self.video_writer = None
            return

        self.get_logger().info(f"Recording processed video: {self.video_path}")

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Video saved: {self.video_path}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()