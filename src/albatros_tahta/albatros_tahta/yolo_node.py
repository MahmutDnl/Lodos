#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — YOLO Perception Node with Hailo AI Kit (v4)
# =============================================================================
# ROS2 Jazzy / Ubuntu 24.04
#
# Girişler:
#   - /albatros/kamera/image_raw  [sensor_msgs/Image]
#   - /albatros/mission/status    [albatros_interfaces/MissionStatus]
#   - /albatros/state             [albatros_interfaces/VehicleState]
#
# Çıkışlar:
#   - /albatros/kamera/processed  [sensor_msgs/Image]
#   - /albatros/yolo/tespitler    [std_msgs/String JSON] (conf >= 0.30)
#   - /albatros/yolo/obstacles    [std_msgs/String JSON]
#   - /albatros/yolo/status       [std_msgs/String JSON]
# =============================================================================

import json
import os
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
    Albatros YOLO algılama node'u (Hailo AI Kit desteği ile).
    Parkur 1-2 ve Parkur 3 için çift HEF model switching ve 0.30 confidence filtreleme sunar.
    """

    def __init__(self):
        super().__init__("yolo_node")

        # ─── ROS Parametreleri ───────────────────────────────────────────────
        self.declare_parameter("input_image_topic", "/albatros/kamera/image_raw")
        self.declare_parameter("processed_image_topic", "/albatros/kamera/processed")
        self.declare_parameter("detections_topic", "/albatros/yolo/tespitler")
        self.declare_parameter("obstacles_topic", "/albatros/yolo/obstacles")
        self.declare_parameter("status_topic", "/albatros/yolo/model_status")

        self.declare_parameter("model_path", "models/parkur_1_2.hef")
        self.declare_parameter("parkur_1_2_model", "models/parkur_1_2.hef")
        self.declare_parameter("parkur12_model_path", "models/parkur12.hef")
        self.declare_parameter("parkur_3_model", "models/parkur_3.hef")
        self.declare_parameter("parkur3_model_path", "models/parkur_3.hef")
        self.declare_parameter("mission_status_timeout_sec", 2.0)

        # Global YOLO confidence threshold (0.30 canonical value across all parkours)
        self.declare_parameter("confidence_threshold", 0.30)
        self.declare_parameter("yolo_conf_threshold", 0.30)

        self.declare_parameter("model_input_width", 640)
        self.declare_parameter("model_input_height", 640)
        self.declare_parameter("model_switch_debounce_sec", 0.5)

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
        self.status_topic = str(self.get_parameter("status_topic").value)

        p12_val = str(self.get_parameter("parkur_1_2_model").value)
        p12_alt = str(self.get_parameter("parkur12_model_path").value)
        self.parkur12_model_path_param = p12_alt if p12_alt and p12_alt != "models/parkur12.hef" else p12_val

        p3_val = str(self.get_parameter("parkur_3_model").value)
        p3_alt = str(self.get_parameter("parkur3_model_path").value)
        self.parkur3_model_path_param = p3_alt if p3_alt and p3_alt != "models/parkur_3.hef" else p3_val

        self.legacy_model_path_param = str(self.get_parameter("model_path").value)
        self.mission_status_timeout_sec = float(self.get_parameter("mission_status_timeout_sec").value)

        # Confidence eşiğini 0.30 kabul et
        conf_val = float(self.get_parameter("yolo_conf_threshold").value)
        if conf_val <= 0.0:
            conf_val = float(self.get_parameter("confidence_threshold").value)
        self.confidence_threshold = max(conf_val, 0.30)

        self.model_input_width = int(self.get_parameter("model_input_width").value)
        self.model_input_height = int(self.get_parameter("model_input_height").value)
        self.model_switch_debounce_sec = float(self.get_parameter("model_switch_debounce_sec").value)

        self.save_video = bool(self.get_parameter("save_video").value)
        self.video_output_dir = Path(str(self.get_parameter("video_output_dir").value)).expanduser()
        self.video_fps = max(float(self.get_parameter("video_fps").value), 1.0)

        self.draw_timestamp = bool(self.get_parameter("draw_timestamp").value)
        self.draw_center = bool(self.get_parameter("draw_center").value)
        self.draw_detections = bool(self.get_parameter("draw_detections").value)

        # ─── Model Class Mapping ─────────────────────────────────────────────
        self.class_mappings = {
            "parkur12": {
                0: "kirmizi_duba",
                1: "sari_duba",
                2: "siyah_duba",
                3: "turuncu_duba",
                4: "yesil_duba"
            },
            "parkur3": {
                0: "kirmizi_duba",
                1: "sari_duba",
                2: "siyah_duba",
                3: "turuncu_duba",
                4: "yesil_duba"
            }
        }

        # ─── Frame Buffer ve Lock'lar ─────────────────────────────────────────
        self.latest_frame = None
        self.latest_stamp = None
        self.latest_msg = None
        self.frame_lock = threading.Lock()
        self.inference_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.running = True

        # ─── Hailo Platform Değişkenleri ────────────────────────────────────
        self.vdevice = None
        self.hef = None
        self.network_group = None
        self.network_group_params = None
        self.input_vstreams_params = None
        self.output_vstreams_params = None
        self.infer_pipeline = None
        self.activation_context = None
        self.active_model_name = None

        self.parkur12_resolved_path = None
        self.parkur3_resolved_path = None
        self.parkur12_available = False
        self.parkur3_available = False

        self.last_model_switch_request_time = 0.0
        self.pending_requested_model = None

        self.video_writer = None
        self.video_path = None

        self._preprocess_scale = 1.0
        self._preprocess_dx = 0
        self._preprocess_dy = 0
        self._original_size = (640, 640)
        self._first_frame_logged = False
        self._last_status_pub_time = 0.0
        self._last_inference_ok = False

        # ─── Hailo ve Model Başlatma ─────────────────────────────────────────
        self.init_hailo()

        # ─── Publisher'lar ──────────────────────────────────────────────────
        self.processed_image_pub = self.create_publisher(Image, self.processed_image_topic, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.obstacles_pub = self.create_publisher(String, self.obstacles_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # ─── Subscriber'lar ─────────────────────────────────────────────────
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

        # ─── Worker Thread ──────────────────────────────────────────────────
        self.worker_thread = threading.Thread(target=self.inference_worker, daemon=True)
        self.worker_thread.start()

        self.last_mission_status_time = 0.0
        self.current_parkur = 0  # PARKUR_UNKNOWN
        self.timeout_timer = self.create_timer(1.0, self._check_mission_timeout)

        self._publish_status()

    # =========================================================================
    # Path Çözümleme & Hata Loglama
    # =========================================================================

    def resolve_model_path(self, raw_path_str: str, model_key: str = ""):
        """
        HEF model dosya yolunu ROS2 package share, source tree ve workspace dizinlerinde arar.
        Geriye (resolved_path_or_None, attempted_paths_list) döndürür.
        """
        attempted = []
        p = Path(raw_path_str).expanduser()

        if p.is_absolute():
            attempted.append(p)
            if p.exists():
                return p.resolve(), attempted

        filename = p.name

        candidates = [
            p,
            Path.cwd() / p,
            Path.cwd() / "models" / filename,
        ]

        if get_package_share_directory is not None:
            for pkg in ["albatros_tahta", "albatros_system"]:
                try:
                    share_dir = Path(get_package_share_directory(pkg))
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
            workspace_src / "albatros_tahta" / raw_path_str,
            workspace_src / "albatros_tahta" / "models" / filename,
            workspace_src / "albatros_system" / raw_path_str,
            workspace_src / "albatros_system" / "models" / filename,
            workspace_src / "models" / filename,
        ])

        for cand in candidates:
            if cand not in attempted:
                attempted.append(cand)
            if cand.exists():
                return cand.resolve(), attempted

        # Fallback: Eğer aranan parkur12.hef / parkur3.hef bulunamadıysa yolov11s.hef dene
        if filename != "yolov11s.hef":
            fallback_filename = "yolov11s.hef"
            for base_cand in list(candidates):
                fb_cand = base_cand.parent / fallback_filename
                if fb_cand not in attempted:
                    attempted.append(fb_cand)
                if fb_cand.exists():
                    if self is not None:
                        self.get_logger().info(
                            f"[YOLO] '{filename}' yerine fallback model '{fallback_filename}' kullanılıyor: {fb_cand}"
                        )
                    return fb_cand.resolve(), attempted

        return None, attempted

    def _log_missing_model_error(self, model_key: str, raw_path_str: str, attempted_paths: list):
        self.get_logger().error(f"[YOLO] '{model_key}' HEF modeli bulunamadı!")
        self.get_logger().error(f"  İstenen Yol : {raw_path_str}")
        self.get_logger().error("  Denenen Yollar:")
        for idx, path_item in enumerate(attempted_paths, 1):
            self.get_logger().error(f"    {idx}. {path_item}")

    # =========================================================================
    # Hailo Başlatma ve Pipeline Yönetimi
    # =========================================================================

    def init_hailo(self):
        if not HAILO_AVAILABLE:
            err_msg = f"hailo_platform kütüphanesi bulunamadı: {HAILO_IMPORT_ERROR}"
            self.get_logger().fatal(f"[FATAL] {err_msg}")
            self.get_logger().fatal("HailoRT kurulumunu ve venv/python environment'ı kontrol edin.")
            raise RuntimeError(err_msg)

        self.get_logger().info("Hailo VDevice oluşturuluyor...")
        try:
            self.vdevice = VDevice()
        except Exception as exc:
            self.get_logger().fatal(f"[FATAL] Hailo VDevice oluşturulamadı: {exc}")
            raise RuntimeError(f"Hailo VDevice initialization failed: {exc}") from exc

        # Parkur 1-2 Model Çözümleme
        self.parkur12_resolved_path, p12_attempted = self.resolve_model_path(
            self.parkur12_model_path_param, "parkur12"
        )
        if self.parkur12_resolved_path is None:
            # Fallback legacy model_path
            self.parkur12_resolved_path, legacy_attempted = self.resolve_model_path(
                self.legacy_model_path_param, "legacy_model_path"
            )
            p12_attempted.extend(legacy_attempted)

        if self.parkur12_resolved_path is not None:
            self.parkur12_available = True
        else:
            self._log_missing_model_error("parkur12", self.parkur12_model_path_param, p12_attempted)
            self.get_logger().fatal("[FATAL] YOLO başlatılamadı: hiçbir geçerli parkur12 HEF modeli bulunamadı.")
            raise FileNotFoundError("No valid parkur12 HEF model found.")

        # Parkur 3 Model Çözümleme
        self.parkur3_resolved_path, p3_attempted = self.resolve_model_path(
            self.parkur3_model_path_param, "parkur3"
        )
        if self.parkur3_resolved_path is not None:
            self.parkur3_available = True
        else:
            self.parkur3_available = False
            self.get_logger().warn(
                f"[WARN] parkur3 HEF modeli bulunamadı ({self.parkur3_model_path_param}). "
                "Parkur 1/2 modeli ile node çalışmaya devam ediyor. Parkur3 model switching pasif."
            )

        # Standalone ve varsayılan kullanım için parkur12 modelini yükle
        if self.parkur12_available:
            self.get_logger().info("[YOLO] Varsayılan model olarak 'parkur12' yükleniyor...")
            self.load_hef_pipeline(self.parkur12_resolved_path, "parkur12")
        else:
            self.active_model_name = None
            self.get_logger().info("[YOLO] Mission bilgisi bekleniyor. Model henüz aktif değil.")

        # Startup Özeti Logla
        self._print_startup_banner()

    def _print_startup_banner(self):
        banner = [
            "============================================================",
            "YOLO Hailo Node Başlatıldı (v4)",
            "",
            f"Input Image Topic      : {self.input_image_topic}",
            f"Detection Output Topic : {self.detections_topic}",
            f"Obstacles Output Topic : {self.obstacles_topic}",
            f"Confidence Threshold   : {self.confidence_threshold:.2f}",
            "",
            f"Parkur12 HEF Model     : {self.parkur12_resolved_path}",
            f"  AVAILABLE            : {self.parkur12_available}",
            f"Parkur3 HEF Model      : {self.parkur3_resolved_path or 'YOK'}",
            f"  AVAILABLE            : {self.parkur3_available}",
            "",
            f"Active Initial Model   : {self.active_model_name}",
            f"Hailo VDevice Status   : READY",
            f"Model Input Dimensions : {self.model_input_width}x{self.model_input_height}",
            "============================================================"
        ]
        for line in banner:
            self.get_logger().info(line)

    def load_hef_pipeline(self, model_path: Path, model_key: str):
        self.get_logger().info(f"[YOLO] Hailo HEF modeli yükleniyor ({model_key}): {model_path}")
        self.hef = HEF(str(model_path))

        input_infos = self.hef.get_input_vstream_infos()
        for i, info in enumerate(input_infos):
            self.get_logger().info(f"HEF Input [{i}]: name={info.name}, shape={info.shape}, format={info.format}")
            if i == 0 and len(info.shape) >= 3:
                self.model_input_height = int(info.shape[0])
                self.model_input_width = int(info.shape[1])
                self.get_logger().info(f"Girdi boyutları güncellendi: {self.model_input_width}x{self.model_input_height}")

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
        self.get_logger().info(f"[YOLO] Hailo inference pipeline hazır. Aktif model: '{self.active_model_name}'.")

    def _cleanup_hailo_pipeline(self):
        """Pipeline ve activation kaynaklarını güvenli şekilde serbest bırakır (Idempotent)."""
        if self.infer_pipeline is not None:
            try:
                self.infer_pipeline.__exit__(None, None, None)
            except Exception as e:
                self.get_logger().warn(f"[YOLO] infer_pipeline kapatılırken uyarı: {e}")
            self.infer_pipeline = None

        if self.activation_context is not None:
            try:
                self.activation_context.__exit__(None, None, None)
            except Exception as e:
                self.get_logger().warn(f"[YOLO] activation_context kapatılırken uyarı: {e}")
            self.activation_context = None

    def switch_hailo_model(self, requested_model: str):
        """
        Thread-safe model switching.
        Parkur değiştiğinde model switch yapar. Hedef model yoksa mevcut modeli bozmaz.
        """
        if requested_model == self.active_model_name:
            return

        with self.inference_lock:
            if requested_model == self.active_model_name:
                return

            target_path = (
                self.parkur3_resolved_path if requested_model == "parkur3" else self.parkur12_resolved_path
            )

            if target_path is None or not target_path.exists():
                self.get_logger().warn(
                    f"[YOLO] Model değiştirilemedi: '{requested_model}' için HEF dosyası bulunamadı. "
                    f"Mevcut '{self.active_model_name}' modeli ile devam ediliyor."
                )
                return

            self.get_logger().info(f"[YOLO] Parkur değişimi algılandı. Model değiştiriliyor: {self.active_model_name} -> {requested_model}")

            # Eski pipeline'ı kapat
            self._cleanup_hailo_pipeline()

            try:
                self.load_hef_pipeline(target_path, requested_model)
                self.get_logger().info(f"[YOLO] Model değişimi tamamlandı. Aktif model: {self.active_model_name}")
                self._publish_status()
            except Exception as e:
                self.get_logger().error(f"[YOLO] '{requested_model}' modeline geçiş başarısız oldu: {e}")
                # Eski çalışan modele geri dönmeyi dene
                if self.parkur12_resolved_path is not None and requested_model != "parkur12":
                    self.get_logger().info("[YOLO] Güvenli geri dönme: 'parkur12' modeline geçiliyor...")
                    try:
                        self.load_hef_pipeline(self.parkur12_resolved_path, "parkur12")
                    except Exception as e2:
                        self.get_logger().fatal(f"[FATAL] Fallback model de yüklenemedi: {e2}")

    # =========================================================================
    # Mission Status / Vehicle State Callback'leri
    # =========================================================================

    def _check_mission_timeout(self):
        if self.last_mission_status_time > 0 and (time.time() - self.last_mission_status_time > self.mission_status_timeout_sec):
            if self.active_model_name != "parkur12" and self.parkur12_available:
                self.get_logger().warn("[YOLO] Mission bilgisi zaman aşımı! 'parkur12' varsayılan modeline dönülüyor.")
                self.switch_hailo_model("parkur12")

    def _evaluate_model_switch_request(self, current_parkur: int, mission_state_str: str):
        self.last_mission_status_time = time.time()
        self.current_parkur = current_parkur

        p1 = MissionStatus.PARKUR_1 if MissionStatus else 1
        p2 = MissionStatus.PARKUR_2 if MissionStatus else 2
        p3 = MissionStatus.PARKUR_3 if MissionStatus else 3

        if current_parkur == p3:
            target_model = "parkur3"
        else:
            # Parkur 1, Parkur 2 veya varsayılan/bilinmeyen (0) durumda parkur12 aktif kalır
            target_model = "parkur12"

        if target_model != self.active_model_name:
            now = time.time()
            if target_model != self.pending_requested_model:
                self.pending_requested_model = target_model
                self.last_model_switch_request_time = now
            elif now - self.last_model_switch_request_time >= self.model_switch_debounce_sec:
                if target_model is None:
                    self.get_logger().info(f"[YOLO] Bilinmeyen parkur ({current_parkur}). Inference durduruluyor.")
                    self._cleanup_hailo_pipeline()
                    self.active_model_name = None
                    self.pending_requested_model = None
                    self._publish_status()
                else:
                    self.switch_hailo_model(target_model)

    def mission_status_callback(self, msg):
        self._evaluate_model_switch_request(msg.current_parkur, msg.mission_state)

    def vehicle_state_callback(self, msg):
        self._evaluate_model_switch_request(msg.current_parkur, msg.mission_state)

    # =========================================================================
    # Görsel & Inference İşleme Döngüsü
    # =========================================================================

    def ros_image_to_cv2(self, msg: Image):
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

    def image_callback(self, msg: Image):
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
                with self.inference_lock:
                    processed_frame, detections = self.run_yolo(frame, stamp)

                if self.draw_timestamp:
                    self.draw_time_label(processed_frame, stamp)

                self.publish_processed_image(processed_frame, input_msg)
                self.publish_detections(detections, stamp, input_msg.header.frame_id)
                self.publish_obstacle_candidates(detections, stamp)
                self.write_video(processed_frame)
                self._last_inference_ok = True
            except Exception as e:
                self._last_inference_ok = False
                self.get_logger().error(f"Inference hatası: {e}", throttle_duration_sec=2.0)

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
                f"İlk frame verisi: shape={input_frame.shape}, dtype={input_frame.dtype}, nbytes={input_frame.nbytes}"
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
        """
        Hailo NMS çıkışlarını okur ve GLOBAL confidence eşiğini (>= 0.30) uygular.

        Desteklenen formatlar (öncelik sırasıyla):
          A. list-of-ndarray: [ndarray(N0,5), ndarray(N1,5), ...]
             Her liste elemanı bir sınıfa ait, class_id = index.
             Her satır: [ymin, xmin, ymax, xmax, confidence]
          B. ndarray 3D: shape=(num_classes, 5, max_detections)
          C. ndarray 2D: shape=(N, 6) — [ymin, xmin, ymax, xmax, confidence, class_id]
        """
        detections = []
        scale = self._preprocess_scale
        dx = self._preprocess_dx
        dy = self._preprocess_dy
        orig_w, orig_h = self._original_size

        class_map = self.class_mappings.get(self.active_model_name, self.class_mappings["parkur12"])

        # ── NMS çıkış tensörünü bul ──────────────────────────────────────────
        nms_output = None
        nms_output_name = None
        for info in self.hef.get_output_vstream_infos():
            out_tensor = results[info.name][0]
            if "nms" in info.name.lower():
                nms_output = out_tensor
                nms_output_name = info.name
                break

        # Eğer "nms" adında output yoksa eski 6-elemanlı format kontrolü (güvenli)
        if nms_output is None:
            for info in self.hef.get_output_vstream_infos():
                out_tensor = results[info.name][0]
                try:
                    if hasattr(out_tensor, 'ndim') and out_tensor.ndim == 2 and out_tensor.shape[-1] == 6:
                        nms_output = out_tensor
                        nms_output_name = info.name
                        break
                except Exception:
                    pass

        if nms_output is None:
            self.get_logger().warn(
                "NMS output format mismatch. Ensure HEF format is supported.",
                throttle_duration_sec=5.0
            )
            return detections

        # ══════════════════════════════════════════════════════════════════════
        # Format tespiti, diagnostik ve decode
        # ══════════════════════════════════════════════════════════════════════
        raw_boxes = []  # Her eleman: (class_id, ymin, xmin, ymax, xmax, confidence)

        try:
            is_ndarray = isinstance(nms_output, np.ndarray)

            # ── FORMAT A: list-of-ndarray (Hailo runtime gerçek çıktısı) ─────
            # nms_output = [ndarray(N0,5), ndarray(N1,5), ...]
            # Her liste elemanı bir sınıfa ait, class_id = liste index'i
            # Her satır: [ymin, xmin, ymax, xmax, confidence]
            is_list_of_ndarray = (
                isinstance(nms_output, list)
                and len(nms_output) > 0
                and all(isinstance(a, np.ndarray) for a in nms_output)
            )

            if is_list_of_ndarray:
                num_classes = len(nms_output)

                # İlk inference diagnostik (bir kez)
                if not getattr(self, '_postprocess_diag_logged', False):
                    self._postprocess_diag_logged = True
                    diag_lines = [
                        f"[YOLO-PP] list-of-ndarray NMS format: {num_classes} sınıf"
                    ]
                    for cls_idx in range(num_classes):
                        cls_arr = nms_output[cls_idx]
                        cls_label = class_map.get(cls_idx, f"unknown_{cls_idx}")
                        if cls_arr.ndim == 2 and cls_arr.shape[0] > 0:
                            max_conf = float(np.max(cls_arr[:, 4]))
                        else:
                            max_conf = 0.0
                        diag_lines.append(
                            f"  class[{cls_idx}] '{cls_label}': "
                            f"shape={cls_arr.shape}, max_conf={max_conf:.4f}"
                        )
                    for line in diag_lines:
                        self.get_logger().info(line)

                for class_id, class_detections in enumerate(nms_output):
                    if class_detections.ndim != 2 or class_detections.shape[0] == 0:
                        continue
                    for det in class_detections:
                        ymin, xmin, ymax, xmax, confidence = det[:5]
                        confidence = float(confidence)
                        if confidence < self.confidence_threshold:
                            continue
                        raw_boxes.append((class_id, float(ymin), float(xmin),
                                          float(ymax), float(xmax), confidence))

            # ── FORMAT B: ndarray 3D (C, 5, N) ──────────────────────────────
            elif is_ndarray and nms_output.ndim == 3 and nms_output.shape[1] == 5:
                num_classes = nms_output.shape[0]
                num_dets = nms_output.shape[2]

                if not getattr(self, '_postprocess_diag_logged', False):
                    self._postprocess_diag_logged = True
                    diag_lines = [f"[YOLO-PP] 3D NMS format: ({num_classes}, 5, {num_dets})"]
                    for cls_idx in range(num_classes):
                        conf_slice = nms_output[cls_idx, 4, :]
                        max_conf = float(np.max(conf_slice)) if num_dets > 0 else 0.0
                        cls_label = class_map.get(cls_idx, f"unknown_{cls_idx}")
                        diag_lines.append(
                            f"  class[{cls_idx}] '{cls_label}': max_conf={max_conf:.4f}"
                        )
                    for line in diag_lines:
                        self.get_logger().info(line)

                for cls_idx in range(num_classes):
                    for det_idx in range(num_dets):
                        confidence = float(nms_output[cls_idx, 4, det_idx])
                        if confidence < self.confidence_threshold:
                            continue
                        ymin = float(nms_output[cls_idx, 0, det_idx])
                        xmin = float(nms_output[cls_idx, 1, det_idx])
                        ymax = float(nms_output[cls_idx, 2, det_idx])
                        xmax = float(nms_output[cls_idx, 3, det_idx])
                        raw_boxes.append((cls_idx, ymin, xmin, ymax, xmax, confidence))

            # ── FORMAT C: ndarray 2D (N, 6) — eski flat NMS ─────────────────
            elif is_ndarray and nms_output.ndim == 2 and nms_output.shape[-1] >= 6:
                if not getattr(self, '_postprocess_diag_logged', False):
                    self._postprocess_diag_logged = True
                    self.get_logger().info(
                        f"[YOLO-PP] Eski 2D NMS format: shape={nms_output.shape}"
                    )

                for box in nms_output:
                    ymin, xmin, ymax, xmax, confidence, class_id = box[:6]
                    confidence = float(confidence)
                    if confidence < self.confidence_threshold:
                        continue
                    raw_boxes.append((int(class_id), float(ymin), float(xmin),
                                      float(ymax), float(xmax), confidence))

            else:
                shape_info = (nms_output.shape if is_ndarray
                              else f"<{type(nms_output).__name__}, len={len(nms_output) if hasattr(nms_output, '__len__') else '?'}>")
                self.get_logger().warn(
                    f"[YOLO-PP] Tanınmayan NMS çıktı formatı: {shape_info}. "
                    "Desteklenen: list-of-ndarray, ndarray (C,5,N), ndarray (N,6).",
                    throttle_duration_sec=5.0
                )
                return detections

        except Exception as e:
            self.get_logger().error(
                f"[YOLO-PP] NMS decode sırasında hata: {e}",
                throttle_duration_sec=2.0
            )
            return detections

        # ── Ortak bbox dönüşüm ve detection oluşturma ────────────────────────
        for (class_id, ymin, xmin, ymax, xmax, confidence) in raw_boxes:

            if class_id in class_map:
                class_name = class_map[class_id]
            else:
                self.get_logger().warn(
                    f"[WARN] Bilinmeyen class_id={class_id} (Aktif model: '{self.active_model_name}')",
                    throttle_duration_sec=5.0
                )
                class_name = f"unknown_{class_id}"

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

    def _publish_status(self):
        now = time.time()
        if now - self._last_status_pub_time < 0.5:
            return
        self._last_status_pub_time = now

        active_model_str = None
        if self.active_model_name == "parkur12":
            active_model_str = "parkur_1_2.hef"
        elif self.active_model_name == "parkur3":
            active_model_str = "parkur_3.hef"

        payload = {
            "current_parkur": getattr(self, "current_parkur", 0),
            "active_model": active_model_str,
            "model_loaded": self.infer_pipeline is not None,
            "switching": (self.pending_requested_model is not None and self.pending_requested_model != self.active_model_name),
            "ready": HAILO_AVAILABLE and self.infer_pipeline is not None,
            "last_inference_ok": self._last_inference_ok,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def write_video(self, frame):
        if not self.save_video:
            return

        if self.video_writer is None:
            self.init_video_writer(frame)

        if self.video_writer is not None:
            try:
                self.video_writer.write(frame)
            except Exception as e:
                self.get_logger().error(f"Video yazma hatası: {e}", throttle_duration_sec=5.0)

    def init_video_writer(self, frame):
        try:
            self.video_output_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.video_path = self.video_output_dir / f"albatros_yolo_processed_{timestamp}.mp4"
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(str(self.video_path), fourcc, self.video_fps, (width, height))

            if not self.video_writer.isOpened():
                self.get_logger().error(f"Video yazar açılamadı: {self.video_path}")
                self.video_writer = None
            else:
                self.get_logger().info(f"Video kaydı başlatıldı: {self.video_path}")
        except Exception as e:
            self.get_logger().error(f"Video yazar başlatma hatası: {e}")
            self.video_writer = None

    def destroy_node(self):
        self.running = False
        self.frame_event.set()
        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)

        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Video kaydedildi: {self.video_path}")

        self._cleanup_hailo_pipeline()

        if self.vdevice is not None:
            try:
                self.vdevice.release()
                self.get_logger().info("Hailo VDevice serbest bırakıldı.")
            except Exception as e:
                self.get_logger().warn(f"VDevice serbest bırakılırken uyarı: {e}")

        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry Point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    try:
        node = YoloNode()
    except (RuntimeError, FileNotFoundError) as e:
        print(f"\n[FATAL] YOLO Node Başlatılamadı: {e}\n")
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