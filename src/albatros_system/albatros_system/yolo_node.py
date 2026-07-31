#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
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

# Hailo platform imports
try:
    from hailo_platform import (
        HEF, VDevice, ConfigureParams,
        InputVStreamParams, OutputVStreamParams,
        InferVStreams, FormatType
    )
except ImportError:
    # Handle the case where we are not on the deployment machine yet, 
    # but still let the node script be syntactically valid.
    pass


class YoloNode(Node):
    """
    Albatros YOLO perception node using Hailo AI Kit.

    Subscribes:
        /albatros/kamera/image_raw          sensor_msgs/Image

    Publishes:
        /albatros/kamera/processed          sensor_msgs/Image
        /albatros/yolo/tespitler            std_msgs/String JSON
        /albatros/yolo/obstacles            std_msgs/String JSON

    Saves:
        Processed camera video as mp4 with timestamp, bounding boxes,
        class names and confidence values.
    """

    def __init__(self):
        super().__init__("yolo_node")

        self.declare_parameter("input_image_topic", "/albatros/kamera/image_raw")
        self.declare_parameter("processed_image_topic", "/albatros/kamera/processed")
        self.declare_parameter("detections_topic", "/albatros/yolo/tespitler")
        self.declare_parameter("obstacles_topic", "/albatros/yolo/obstacles")

        self.declare_parameter("model_path", "models/yolov11s.hef")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("model_input_width", 640)
        self.declare_parameter("model_input_height", 640)

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
        self.model_input_width = int(self.get_parameter("model_input_width").value)
        self.model_input_height = int(self.get_parameter("model_input_height").value)

        self.save_video = bool(self.get_parameter("save_video").value)
        self.video_output_dir = Path(
            str(self.get_parameter("video_output_dir").value)
        ).expanduser()
        self.video_fps = max(float(self.get_parameter("video_fps").value), 1.0)

        self.draw_timestamp = bool(self.get_parameter("draw_timestamp").value)
        self.draw_center = bool(self.get_parameter("draw_center").value)
        self.draw_detections = bool(self.get_parameter("draw_detections").value)

        self.bridge = CvBridge()
        
        # Hardcoded class names from the YOLO model
        self.class_names = {
            0: 'kirmizi_duba',
            1: 'sari_duba',
            2: 'siyah_duba',
            3: 'turuncu_duba',
            4: 'yesil_duba'
        }
        
        self.vdevice = None
        self.infer_pipeline = None
        self.load_model()

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

        self.get_logger().info("YOLO Hailo node started.")
        self.get_logger().info(f"Subscribing: {self.input_image_topic}")
        self.get_logger().info(f"Publishing processed image: {self.processed_image_topic}")
        self.get_logger().info(f"Publishing detections JSON: {self.detections_topic}")
        self.get_logger().info(f"Publishing obstacle candidates: {self.obstacles_topic}")

    def load_model(self):
        try:
            from hailo_platform import HEF, VDevice, ConfigureParams, InputVStreamParams, OutputVStreamParams, FormatType
        except ImportError as exc:
            self.get_logger().error(
                "hailo_platform is not installed. This node requires HailoRT and hailo_platform."
            )
            # return rather than raise, to allow node to be created and compiled on dev machine
            self.get_logger().error("Model loading aborted due to missing hailo_platform.")
            return

        model_path = Path(self.model_path).expanduser()

        if not model_path.is_absolute():
            current_file = Path(__file__).resolve()
            package_dir = current_file.parent
            workspace_guess = package_dir.parent.parent.parent
            candidate_paths = [
                package_dir / self.model_path,
                package_dir.parent / self.model_path,
                Path.cwd() / self.model_path,
                Path.cwd() / "src" / "albatros_system" / self.model_path,
            ]

            for candidate in candidate_paths:
                if candidate.exists():
                    model_path = candidate
                    break

        if not model_path.exists():
            raise FileNotFoundError(
                f"Hailo HEF model file not found: {model_path}. "
                "Put yolov11s.hef into models/ or pass model_path parameter."
            )

        self.get_logger().info(f"Loading Hailo HEF model: {model_path}")
        
        hef = HEF(str(model_path))
        
        self.vdevice = VDevice()
        
        configure_params = ConfigureParams.create_from_hef(hef=hef, interface=None)
        self.network_group = self.vdevice.configure(hef, configure_params)[0]
        self.network_group_params = self.network_group.create_params()
        
        self.input_vstream_info = hef.get_input_vstream_infos()[0]
        self.output_vstream_infos = hef.get_output_vstream_infos()
        
        self.input_vstreams_params = InputVStreamParams.make_from_network_group(
            self.network_group, quantized=False, format_type=FormatType.UINT8
        )
        
        # Determine whether output is quantized or not based on typical usage, Float32 is easier to parse
        self.output_vstreams_params = OutputVStreamParams.make_from_network_group(
            self.network_group, quantized=False, format_type=FormatType.FLOAT32
        )
        
        self.get_logger().info("Hailo model configured successfully.")

    def preprocess_frame(self, frame):
        """BGR frame'i model giriş boyutuna letterbox ile ölçekle."""
        ih, iw = frame.shape[:2]
        h, w = self.model_input_height, self.model_input_width
        
        scale = min(w / iw, h / ih)
        nw, nh = int(iw * scale), int(ih * scale)
        resized = cv2.resize(frame, (nw, nh))
        
        canvas = np.full((h, w, 3), 128, dtype=np.uint8)
        dx, dy = (w - nw) // 2, (h - nh) // 2
        canvas[dy:dy+nh, dx:dx+nw] = resized
        
        # Hailo models typically expect RGB
        canvas_rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        
        self._preprocess_scale = scale
        self._preprocess_dx = dx
        self._preprocess_dy = dy
        self._original_size = (iw, ih)
        
        return canvas_rgb

    def ros_image_to_cv2(self, msg: Image):
        if msg.encoding not in ("bgr8", "rgb8", "mono8", "8UC3"):
            raise ValueError(f"Desteklenmeyen image encoding: {msg.encoding}")

        data = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.encoding == "mono8":
            frame = data.reshape((msg.height, msg.step))
            frame = frame[:, :msg.width]
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            frame = data.reshape((msg.height, msg.step))
            frame = frame[:, :msg.width * 3]
            frame = frame.reshape((msg.height, msg.width, 3))

            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return np.ascontiguousarray(frame)

    def image_callback(self, msg: Image):
        try:
            frame = self.ros_image_to_cv2(msg)
        except Exception as exc:
            self.get_logger().error(f"image conversion error: {exc}")
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
        
        if self.vdevice is None:
            # Model is not loaded (likely missing hailo_platform)
            return processed, detections
            
        from hailo_platform import InferVStreams

        input_frame = self.preprocess_frame(frame)
        input_data = {
            self.input_vstream_info.name: np.expand_dims(input_frame, axis=0)
        }

        with InferVStreams(self.network_group, self.input_vstreams_params, self.output_vstreams_params) as pipeline:
            with self.network_group.activate(self.network_group_params):
                results = pipeline.infer(input_data)

        # Post process results
        detections = self.postprocess_results(results, stamp_float)

        for det in detections:
            if self.draw_detections:
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
        
        # Birden fazla output stream olabilir, NMS olan stream genelde "nms" ismini icerir veya tek streamdir
        nms_output = None
        for info in self.output_vstream_infos:
            out_tensor = results[info.name][0] # batch 0
            if "nms" in info.name.lower() or out_tensor.shape[-1] == 6:
                nms_output = out_tensor
                break
        
        # Eger NMS ciktisi bulamadiysak, ilk stream'i deneyelim (Shape: (num_boxes, 6))
        if nms_output is None and len(self.output_vstream_infos) > 0:
            info = self.output_vstream_infos[0]
            out_tensor = results[info.name][0]
            if out_tensor.shape[-1] == 6:
                nms_output = out_tensor

        if nms_output is not None:
            # Standart Hailo NMS Output format: [ymin, xmin, ymax, xmax, confidence, class_id] 
            # ya da bazi hef'lerde [xmin, ymin, xmax, ymax, confidence, class_id] olabilir.
            # Deneyimlere gore genelde [ymin, xmin, ymax, xmax, confidence, class_id] doner.
            for box in nms_output:
                
                if len(box) >= 6:
                    # HEF NMS layer'ina gore format: ymin, xmin, ymax, xmax veya xmin, ymin, xmax, ymax
                    # Standart olarak Hailo RT on-chip NMS ymin, xmin, ymax, xmax formatindadir.
                    ymin, xmin, ymax, xmax, confidence, class_id = box[:6]
                    
                    # Confidence esik degeri kontrolu (Bazen Hailo 0 basar veya NMS sonrasi cok dusuk conf kalabilir)
                    if confidence < self.confidence_threshold:
                        continue
                    
                    class_id = int(class_id)
                    class_name = self.class_names.get(class_id, str(class_id))
                    
                    # Coordinates are usually normalized [0..1]
                    # If they are already > 1, then they are absolute coordinates, but typical hailo NMS is normalized.
                    if xmax <= 1.0 and ymax <= 1.0:
                        xmin_px = xmin * self.model_input_width
                        xmax_px = xmax * self.model_input_width
                        ymin_px = ymin * self.model_input_height
                        ymax_px = ymax * self.model_input_height
                    else:
                        xmin_px, ymin_px, xmax_px, ymax_px = xmin, ymin, xmax, ymax
                        
                    # Y ile X bazen yer degistirmis olabilir (xmin, ymin, xmax, ymax). 
                    # Sayet xmin > ymin veya ymax > xmax kontrolleri ile emin olamayiz. 
                    # Fakat Hailo'nun standart bbox parser'inda ymin, xmin, ymax, xmax gelir.
                    
                    # Letterbox ters islemi (Letterbox yapilmis 640x640 frame'den orijinal frame'e donus)
                    x1 = int((xmin_px - dx) / scale)
                    y1 = int((ymin_px - dy) / scale)
                    x2 = int((xmax_px - dx) / scale)
                    y2 = int((ymax_px - dy) / scale)
                    
                    # Sinirlari orijinal resim boyutuna kirp
                    x1 = max(0, min(x1, orig_w - 1))
                    y1 = max(0, min(y1, orig_h - 1))
                    x2 = max(0, min(x2, orig_w - 1))
                    y2 = max(0, min(y2, orig_h - 1))
                    
                    # Koordinatlarin yanlis gelme ihtimaline karsi bir fallback kontrolu
                    # (xmin ve ymin yer degistirdiyse x1 ve x2 yanlis olacaktir)
                    if x1 > x2:
                        x1, x2 = x2, x1
                    if y1 > y2:
                        y1, y2 = y2, y1
                    
                    width = int(x2 - x1)
                    height = int(y2 - y1)
                    
                    # Eger bbox gecersiz bir boyutta ise yoksay
                    if width <= 0 or height <= 0:
                        continue
                    
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    
                    detection = {
                        "stamp": stamp_float,
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": float(confidence),
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
            self.get_logger().warn("NMS output format mismatch. Ensure HEF was compiled with nms_postprocess.", throttle_duration_sec=5.0)

        return detections


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
            
        if self.vdevice is not None:
            self.vdevice.release()
            self.get_logger().info("Hailo VDevice released.")

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