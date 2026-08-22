#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — Parkur 3 Özel YOLO Düğümü (Hailo AI Kit)
# =============================================================================
# ROS2 Jazzy / Ubuntu 24.04
#
# Paket: albatros_simple
# ROS2 Düğüm Adı: yolo_node
#
# Giriş:
#   - /albatros/kamera/image_raw  [sensor_msgs/Image]
#
# Çıkışlar:
#   - /albatros/yolo/tespitler    [std_msgs/String JSON]
#   - /albatros/kamera/processed  [sensor_msgs/Image]
# =============================================================================

import json
import math
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

# Hailo Platform Import
HAILO_AVAILABLE = False
HAILO_IMPORT_ERROR = None
try:
    from hailo_platform import (
        HEF, VDevice, ConfigureParams,
        HailoStreamInterface,
        InputVStreamParams, OutputVStreamParams,
        InferVStreams, FormatType
    )
    HAILO_AVAILABLE = True
except ImportError as exc:
    HAILO_AVAILABLE = False
    HAILO_IMPORT_ERROR = str(exc)


class YoloNode(Node):
    """
    Parkur 3'e özel Hailo AI Kit YOLO tespit düğümü.
    Sadece 'kirmizi_duba', 'yesil_duba', 'siyah_duba' sınıflarını algılar,
    bounding box, center_x/center_y, mesafe ve yatay açı değerlerini hesaplayarak
    JSON formatında yayınlar.
    """

    CLASS_MAPPING = {
        0: "kirmizi_duba",
        1: "yesil_duba",
        2: "siyah_duba"
    }

    def __init__(self):
        super().__init__("yolo_node")

        # ─── ROS Parametreleri ───────────────────────────────────────────────
        self.declare_parameter("input_image_topic", "/albatros/kamera/image_raw")
        self.declare_parameter("processed_image_topic", "/albatros/kamera/processed")
        self.declare_parameter("detections_topic", "/albatros/yolo/tespitler")
        self.declare_parameter("model_path", "models/parkur_3.hef")

        self.declare_parameter("fx", 500.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("real_buoy_width", 0.3)
        self.declare_parameter("min_bbox_width_px", 20)
        self.declare_parameter("confidence_threshold", 0.30)

        self.declare_parameter("model_input_width", 640)
        self.declare_parameter("model_input_height", 640)

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.processed_image_topic = str(self.get_parameter("processed_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.raw_model_path = str(self.get_parameter("model_path").value)

        self.fx = float(self.get_parameter("fx").value)
        self.cx = float(self.get_parameter("cx").value)
        self.real_buoy_width = float(self.get_parameter("real_buoy_width").value)
        self.min_bbox_width_px = float(self.get_parameter("min_bbox_width_px").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)

        self.model_input_width = int(self.get_parameter("model_input_width").value)
        self.model_input_height = int(self.get_parameter("model_input_height").value)

        # ─── Frame Buffer ve Threading ─────────────────────────────────────────
        self.latest_frame = None
        self.latest_stamp = None
        self.latest_msg = None
        self.frame_lock = threading.Lock()
        self.inference_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.running = True

        # ─── Hailo Değişkenleri ──────────────────────────────────────────────
        self.vdevice = None
        self.hef = None
        self.network_group = None
        self.network_group_params = None
        self.input_vstreams_params = None
        self.output_vstreams_params = None
        self.infer_pipeline = None
        self.activation_context = None

        self._preprocess_scale = 1.0
        self._preprocess_dx = 0
        self._preprocess_dy = 0
        self._original_size = (640, 640)

        # Model Yolunu Çözümle ve Hailo'yu Başlat (Donanım/Model yoksa FATAL fırlatır)
        self.resolved_model_path = self.resolve_model_path(self.raw_model_path)
        self.init_hailo()

        # ─── Publisher ve Subscriber'lar ─────────────────────────────────────
        self.parkur3_active = False
        self.parkur3_sub = self.create_subscription(
            Bool,
            "/albatros/parkur3_active",
            self.parkur3_active_callback,
            10
        )

        self.processed_image_pub = self.create_publisher(Image, self.processed_image_topic, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)

        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

    def parkur3_active_callback(self, msg: Bool):
        if msg.data and not self.parkur3_active:
            self.get_logger().info("[yolo_node] Parkur-3 sinyali alındı! YOLO tespiti aktif duruma geçti.")
        self.parkur3_active = msg.data

        # Worker Thread
        self.worker_thread = threading.Thread(target=self.inference_worker, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("==================================================")
        self.get_logger().info("Parkur 3 YOLO Node Başlatıldı (yolo_node)")
        self.get_logger().info(f"  Kamera Konusu     : {self.input_image_topic}")
        self.get_logger().info(f"  Tespit Konusu     : {self.detections_topic}")
        self.get_logger().info(f"  İşlenmiş Konu     : {self.processed_image_topic}")
        self.get_logger().info(f"  Model Dosyası     : {self.resolved_model_path}")
        self.get_logger().info(f"  Confidence Eşiği  : {self.confidence_threshold:.2f}")
        self.get_logger().info(f"  Min BBox Genişlik : {self.min_bbox_width_px} px")
        self.get_logger().info(f"  Kamera (fx, cx)   : {self.fx}, {self.cx}")
        self.get_logger().info(f"  Duba Genişliği    : {self.real_buoy_width} m")
        self.get_logger().info("==================================================")

    def resolve_model_path(self, raw_path_str: str) -> Path:
        """HEF model dosyasının varlığını paket dizininde ve workspace'te kontrol eder."""
        p = Path(raw_path_str).expanduser()
        if p.is_absolute() and p.exists():
            return p.resolve()

        filename = p.name
        candidates = [
            p,
            Path.cwd() / p,
            Path.cwd() / "models" / filename,
        ]

        if get_package_share_directory is not None:
            try:
                share_dir = Path(get_package_share_directory("albatros_simple"))
                candidates.append(share_dir / raw_path_str)
                candidates.append(share_dir / "models" / filename)
            except Exception:
                pass

        current_file = Path(__file__).resolve()
        package_dir = current_file.parent
        workspace_src = package_dir.parents[2] if len(package_dir.parents) >= 3 else package_dir

        candidates.extend([
            package_dir / raw_path_str,
            package_dir / "models" / filename,
            workspace_src / "albatros_simple" / raw_path_str,
            workspace_src / "albatros_simple" / "models" / filename,
            workspace_src / "albatros_tahta" / "models" / filename,
        ])

        for cand in candidates:
            if cand.exists():
                return cand.resolve()

        return p

    def init_hailo(self):
        """Hailo VDevice ve HEF inference pipeline'ını oluşturur. Başarısız olursa FATAL hata verip durur."""
        if not HAILO_AVAILABLE:
            err_msg = f"hailo_platform kütüphanesi yüklenemedi: {HAILO_IMPORT_ERROR}"
            self.get_logger().fatal(f"[FATAL] {err_msg}")
            raise RuntimeError(err_msg)

        if not self.resolved_model_path.exists():
            err_msg = f"HEF model dosyası bulunamadı: {self.resolved_model_path}"
            self.get_logger().fatal(f"[FATAL] {err_msg}")
            raise FileNotFoundError(err_msg)

        try:
            self.get_logger().info(f"Hailo VDevice başlatılıyor ve HEF modeli yükleniyor: {self.resolved_model_path}")
            self.vdevice = VDevice()
            self.hef = HEF(str(self.resolved_model_path))

            # HEF Input ve Output Tensor Bilgilerini Logla
            input_infos = self.hef.get_input_vstream_infos()
            for i, info in enumerate(input_infos):
                self.get_logger().info(f"HEF Input  [{i}]: name={info.name}, shape={info.shape}, format={info.format}")
                if i == 0 and len(info.shape) >= 3:
                    self.model_input_height = int(info.shape[0])
                    self.model_input_width = int(info.shape[1])

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

            self.get_logger().info("Hailo AI Kit Parkur 3 inference pipeline başarıyla aktifleştirildi.")
        except Exception as e:
            err_msg = f"Hailo pipeline başlatılamadı: {e}"
            self.get_logger().fatal(f"[FATAL] {err_msg}")
            raise RuntimeError(err_msg) from e

    def image_callback(self, msg: Image):
        """Kamera görüntüsü alındığında çalışır."""
        try:
            frame = self.ros_image_to_cv2(msg)
        except Exception as exc:
            self.get_logger().error(f"Görüntü dönüştürme hatası: {exc}", throttle_duration_sec=2.0)
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

    def ros_image_to_cv2(self, msg: Image) -> np.ndarray:
        """ROS Image mesajını OpenCV BGR formatına dönüştürür."""
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "mono8":
            frame = data.reshape((msg.height, msg.step))
            frame = frame[:, :msg.width]
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif msg.encoding in ["bgra8", "BGRA8"]:
            frame = data.reshape((msg.height, msg.step))
            frame = frame[:, :msg.width * 4]
            frame = frame.reshape((msg.height, msg.width, 4))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif msg.encoding in ["rgba8", "RGBA8"]:
            frame = data.reshape((msg.height, msg.step))
            frame = frame[:, :msg.width * 4]
            frame = frame.reshape((msg.height, msg.width, 4))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        else:
            frame = data.reshape((msg.height, msg.step))
            frame = frame[:, :msg.width * 3]
            frame = frame.reshape((msg.height, msg.width, 3))
            if msg.encoding in ["rgb8", "RGB8"]:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(frame)

    def cv2_to_ros_image(self, frame: np.ndarray, header) -> Image:
        """OpenCV BGR görüntüsünü ROS Image mesajına dönüştürür."""
        msg = Image()
        msg.header = header
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    def inference_worker(self):
        """Arka planda görüntüyü işleyen ve inference koşturan worker döngüsü."""
        while self.running and rclpy.ok():
            if not self.parkur3_active:
                self.get_logger().info("[yolo_node] Parkur-3 aktif sinyali bekleniyor...", throttle_duration_sec=10.0)
                self.frame_event.wait(timeout=0.5)
                continue

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
                with self.inference_lock:
                    processed_frame, detections = self.run_yolo(frame, stamp)

                self.publish_processed_image(processed_frame, input_msg.header)
                self.publish_detections(detections, stamp, input_msg.header.frame_id)
            except Exception as e:
                self.get_logger().error(f"Inference worker hatası: {e}", throttle_duration_sec=2.0)

    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """Frame'i model girdi boyutuna göre hazırlar (letterbox)."""
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

    def run_yolo(self, frame: np.ndarray, stamp_float: float):
        """Hailo pipeline üzerinde model çalıştırır ve tespit sonuçlarını hesaplar."""
        processed = frame.copy()
        detections = []

        if self.infer_pipeline is None:
            return processed, detections

        input_frame = self.preprocess_frame(frame)
        input_name = self.hef.get_input_vstream_infos()[0].name
        input_data = {input_name: np.expand_dims(input_frame, axis=0)}

        results = self.infer_pipeline.infer(input_data)
        detections = self.postprocess_results(results, stamp_float)

        # Görsel üzerinde bounding box ve bilgileri çiz
        for det in detections:
            bbox = det["bbox"]
            class_name = det["class_name"]
            conf = det["confidence"]
            dist = det["distance"]
            angle = det["angle_deg"]
            cx = int(det["center_x"])
            cy = int(det["center_y"])

            if class_name == "kirmizi_duba":
                color = (0, 0, 255)
            elif class_name == "yesil_duba":
                color = (0, 255, 0)
            elif class_name == "siyah_duba":
                color = (50, 50, 50)
            else:
                color = (255, 255, 255)

            cv2.rectangle(processed, (bbox["x_min"], bbox["y_min"]), (bbox["x_max"], bbox["y_max"]), color, 2)
            cv2.circle(processed, (cx, cy), 4, (0, 255, 255), -1)

            label = f"{class_name} {conf:.2f} | {dist:.2f}m | {angle:+.1f}deg"
            cv2.putText(processed, label, (bbox["x_min"], max(20, bbox["y_min"] - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return processed, detections

    def postprocess_results(self, results, stamp_float: float) -> list:
        """
        Hailo NMS çıktılarını ayrıştırır ve mesafe/açı hesaplamalarını yapar.
        """
        detections = []
        scale = self._preprocess_scale
        dx = self._preprocess_dx
        dy = self._preprocess_dy
        orig_w, orig_h = self._original_size

        nms_output = None
        for info in self.hef.get_output_vstream_infos():
            out_tensor = results[info.name][0]
            # Gerçek NMS çıktısı ismi kontrolü veya (N, 6) tensor kontrolü
            if "nms" in info.name.lower() or (hasattr(out_tensor, "shape") and len(out_tensor.shape) >= 2 and out_tensor.shape[-1] == 6):
                nms_output = out_tensor
                break

        if nms_output is None:
            return detections

        raw_boxes = []

        # Hailo runtime NMS formatı ayrıştırma
        if isinstance(nms_output, list) and len(nms_output) > 0:
            for class_id, class_dets in enumerate(nms_output):
                if not isinstance(class_dets, np.ndarray) or class_dets.ndim != 2 or class_dets.shape[0] == 0:
                    continue
                for det in class_dets:
                    ymin, xmin, ymax, xmax, conf = det[:5]
                    conf = float(conf)
                    if conf >= self.confidence_threshold:
                        raw_boxes.append((class_id, float(ymin), float(xmin), float(ymax), float(xmax), conf))

        elif isinstance(nms_output, np.ndarray):
            if nms_output.ndim == 3 and nms_output.shape[1] == 5:
                num_classes = nms_output.shape[0]
                num_dets = nms_output.shape[2]
                for cls_idx in range(num_classes):
                    for det_idx in range(num_dets):
                        conf = float(nms_output[cls_idx, 4, det_idx])
                        if conf >= self.confidence_threshold:
                            ymin = float(nms_output[cls_idx, 0, det_idx])
                            xmin = float(nms_output[cls_idx, 1, det_idx])
                            ymax = float(nms_output[cls_idx, 2, det_idx])
                            xmax = float(nms_output[cls_idx, 3, det_idx])
                            raw_boxes.append((cls_idx, ymin, xmin, ymax, xmax, conf))

            elif nms_output.ndim == 2 and nms_output.shape[-1] >= 6:
                for box in nms_output:
                    ymin, xmin, ymax, xmax, conf, class_id = box[:6]
                    conf = float(conf)
                    if conf >= self.confidence_threshold:
                        raw_boxes.append((int(class_id), float(ymin), float(xmin), float(ymax), float(xmax), conf))

        for (class_id, ymin, xmin, ymax, xmax, conf) in raw_boxes:
            # Sadece hedef 3 sınıf
            if class_id not in self.CLASS_MAPPING:
                continue
            class_name = self.CLASS_MAPPING[class_id]

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

            width_px = max(1.0, float(x2 - x1))
            height_px = max(1.0, float(y2 - y1))

            # BBox Genişlik Filtresi (min_bbox_width_px altındakileri reddet)
            if width_px < self.min_bbox_width_px:
                continue

            center_x = (float(x1) + float(x2)) / 2.0
            center_y = (float(y1) + float(y2)) / 2.0

            # Mesafe Formülü: (gerçek_genişlik * fx) / bbox_width_px
            distance = (self.real_buoy_width * self.fx) / width_px

            # Yatay Açı Formülü: atan((center_x - cx) / fx) -> Derece
            angle_rad = math.atan2(center_x - self.cx, self.fx)
            angle_deg = math.degrees(angle_rad)

            detection_entry = {
                "class_id": class_id,
                "class_name": class_name,
                "confidence": round(conf, 4),
                "bbox": {
                    "x_min": int(x1),
                    "y_min": int(y1),
                    "x_max": int(x2),
                    "y_max": int(y2),
                    "width": int(width_px),
                    "height": int(height_px)
                },
                "center_x": round(center_x, 2),
                "center_y": round(center_y, 2),
                "distance": round(distance, 3),
                "angle_deg": round(angle_deg, 2)
            }
            detections.append(detection_entry)

        return detections

    def publish_detections(self, detections: list, stamp_float: float, frame_id: str):
        """Tespit sonuçlarını JSON olarak /albatros/yolo/tespitler konusuna basar."""
        payload = {
            "stamp": stamp_float,
            "frame_id": frame_id,
            "detections": detections
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.detections_pub.publish(msg)

    def publish_processed_image(self, frame: np.ndarray, header):
        """Bounding box çizilmiş görüntüyü /albatros/kamera/processed konusuna basar."""
        msg = self.cv2_to_ros_image(frame, header)
        self.processed_image_pub.publish(msg)

    def destroy_node(self):
        """Kaynakları ve worker thread'i güvenli şekilde serbest bırakır."""
        self.running = False
        self.frame_event.set()
        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)

        if self.infer_pipeline is not None:
            try:
                self.infer_pipeline.__exit__(None, None, None)
            except Exception as e:
                self.get_logger().warn(f"Infer pipeline kapatılırken hata: {e}")
            self.infer_pipeline = None

        if self.activation_context is not None:
            try:
                self.activation_context.__exit__(None, None, None)
            except Exception as e:
                self.get_logger().warn(f"Activation context kapatılırken hata: {e}")
            self.activation_context = None

        if self.vdevice is not None:
            try:
                self.vdevice.release()
                self.get_logger().info("Hailo VDevice serbest bırakıldı.")
            except Exception as e:
                self.get_logger().warn(f"VDevice serbest bırakılırken hata: {e}")
            self.vdevice = None

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YoloNode()
    except Exception as exc:
        print(f"[FATAL] yolo_node başlatılamadı: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
