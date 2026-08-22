#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — Parkur 3 Kamikaze Görev Yöneticisi (mission_node)
# =============================================================================
# ROS2 Jazzy / Ubuntu 24.04
# Paket: albatros_simple
# ROS2 Düğüm Adı: mission_node
#
# Girişler:
#   - /albatros/parkur3_active  [std_msgs/Bool]
#   - /albatros/yolo/tespitler   [std_msgs/String JSON]
#
# Donanım Bağlantısı:
#   - Pixhawk 2.4.8 (MAVLink - /dev/serial/by-id/usb-ArduPilot* veya /dev/ttyACM0)
# =============================================================================

import glob
import json
import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    mavutil = None
    MAVLINK_AVAILABLE = False


class MissionState(Enum):
    WAIT_PARKUR3 = auto()
    ENTER_GUIDED = auto()
    SEARCH_TARGET = auto()
    ALIGN_TARGET = auto()
    APPROACH_TARGET = auto()
    HIT_TARGET = auto()
    RETURN_AUTO = auto()
    MISSION_DONE = auto()


COLOR_MAP = {
    1: "kirmizi_duba",
    2: "yesil_duba",
    3: "siyah_duba"
}


class MissionNode(Node):
    """
    Parkur-3 Kamikaze Görev Yöneticisi.
    - /albatros/parkur3_active True olana kadar hiçbir motor komutu göndermez.
    - Pixhawk ile MAVLink üzerinden haberleşir, GUIDED moda geçer ve HEARTBEAT ile doğrular.
    - YKİ'den (Pixhawk SCR_USER1) hedef rengini okur (1=kirmizi_duba, 2=yesil_duba, 3=siyah_duba).
    - /albatros/yolo/tespitler konusundaki JSON verilerini dinler ve sadece hedef rengi takip eder.
    - 10 Hz non-blocking timer + State Machine ile angajmanı yönetir.
    - Hedefe ulaştıktan sonra STOP yapar, dönüş waypoint'ine geçer ve AUTO moduna alıp onaylar.
    """

    def __init__(self):
        super().__init__('mission_node')

        # ─── ROS Parametreleri ───────────────────────────────────────────────
        self.declare_parameter('return_wp', 6)
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('confidence_threshold', 0.60)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('hit_duration_sec', 1.2)
        self.declare_parameter('default_target_color_id', 1)

        self.return_wp = int(self.get_parameter('return_wp').value)
        self.baudrate = int(self.get_parameter('serial_baudrate').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.cx = float(self.get_parameter('camera_cx').value)
        self.hit_duration_sec = float(self.get_parameter('hit_duration_sec').value)
        self.default_target_color_id = int(self.get_parameter('default_target_color_id').value)

        # ─── Durum ve Kontrol Değişkenleri ────────────────────────────────────
        self.state = MissionState.WAIT_PARKUR3
        self.parkur3_active = False

        self.target_color_id = None
        self.target_color = None
        self.param_requested = False
        self.param_request_time = 0.0

        self.mav_conn = None
        self.target_system = 1
        self.target_component = 1
        self.current_mode = "UNKNOWN"

        self.latest_detection = None
        self.last_detection_time = 0.0
        self.last_cmd = (0.0, 0.0)  # (vx, yaw_rate)
        self.hit_start_time = None
        self.last_mode_cmd_time = 0.0

        # ─── ROS Abonelikleri ────────────────────────────────────────────────
        self.parkur3_sub = self.create_subscription(
            Bool,
            '/albatros/parkur3_active',
            self.parkur3_active_callback,
            10
        )

        self.yolo_sub = self.create_subscription(
            String,
            '/albatros/yolo/tespitler',
            self.yolo_detections_callback,
            10
        )

        # ─── 10 Hz Timer (State Machine & Control Loop) ──────────────────────
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('==================================================')
        self.get_logger().info('MissionNode Başlatıldı (Parkur-3 Kamikaze Yöneticisi)')
        self.get_logger().info(f'  Dönüş Waypoint (return_wp) : {self.return_wp}')
        self.get_logger().info(f'  Min Confidence            : {self.confidence_threshold}')
        self.get_logger().info(f'  Kamera cx                  : {self.cx}')
        self.get_logger().info('==================================================')

    def parkur3_active_callback(self, msg: Bool):
        if msg.data and not self.parkur3_active:
            self.get_logger().info('[MissionNode] Parkur-3 sinyali alındı! Kamikaze angajmanı başlatılıyor...')
            self.parkur3_active = True
        elif not msg.data and self.parkur3_active:
            self.get_logger().warn('[MissionNode] Parkur-3 sinyali kesildi/pasifleşti. Motorlar durduruluyor.')
            self.parkur3_active = False
            self.send_stop()
            self.state = MissionState.WAIT_PARKUR3

    def yolo_detections_callback(self, msg: String):
        if not self.parkur3_active or self.target_color is None:
            return

        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])

            valid_dets = []
            for det in detections:
                conf = det.get("confidence", 0.0)
                cls_name = det.get("class_name", "")
                if conf >= self.confidence_threshold and cls_name == self.target_color:
                    valid_dets.append(det)

            if valid_dets:
                # En yüksek güvenilirliğe sahip tespiti seç
                best_det = max(valid_dets, key=lambda d: d.get("confidence", 0.0))
                self.latest_detection = best_det
                self.last_detection_time = time.time()
        except Exception as e:
            self.get_logger().error(f"[MissionNode] YOLO JSON ayrıştırma hatası: {e}", throttle_duration_sec=2.0)
            self.send_stop()

    def find_pixhawk_port(self) -> str:
        ports = glob.glob('/dev/serial/by-id/usb-ArduPilot*')
        if ports:
            return ports[0]
        return '/dev/ttyACM0'

    def connect_pixhawk(self) -> bool:
        if not MAVLINK_AVAILABLE:
            self.get_logger().error("[MissionNode] pymavlink kütüphanesi eksik! Pixhawk'a bağlanılamıyor.")
            return False

        if self.mav_conn is not None:
            return True

        port = self.find_pixhawk_port()
        try:
            self.get_logger().info(f"[MissionNode] Pixhawk bağlantısı deneniyor: Port={port}, Baud={self.baudrate}")
            self.mav_conn = mavutil.mavlink_connection(port, baud=self.baudrate)
            self.get_logger().info(f"[MissionNode] Pixhawk seri portu başarıyla açıldı: {port}")
            return True
        except Exception as e:
            self.get_logger().error(f"[MissionNode] Pixhawk bağlantı hatası: {e}", throttle_duration_sec=3.0)
            self.mav_conn = None
            return False

    def update_mavlink(self):
        if self.mav_conn is None:
            return

        try:
            while True:
                msg = self.mav_conn.recv_match(blocking=False)
                if msg is None:
                    break

                msg_type = msg.get_type()
                if msg_type == 'HEARTBEAT':
                    self.target_system = msg.get_srcSystem()
                    self.target_component = msg.get_srcComponent()
                    try:
                        self.current_mode = mavutil.mode_string_v10(msg)
                    except Exception:
                        pass

                elif msg_type == 'PARAM_VALUE':
                    param_id = msg.param_id.rstrip('\x00') if isinstance(msg.param_id, str) else msg.param_id.decode('ascii', errors='ignore').rstrip('\x00')
                    if param_id == 'SCR_USER1':
                        val = int(msg.param_value)
                        self.target_color_id = val
                        self.target_color = COLOR_MAP.get(val, COLOR_MAP.get(self.default_target_color_id, "kirmizi_duba"))
                        self.get_logger().info(f"[MissionNode] Pixhawk'tan SCR_USER1 okundu: {val} -> Hedef Renk: '{self.target_color}'")
        except Exception as e:
            self.get_logger().error(f"[MissionNode] MAVLink okuma hatası: {e}", throttle_duration_sec=2.0)

    def request_scr_user1_param(self):
        if self.mav_conn is None:
            return
        now = time.time()
        if now - self.param_request_time > 1.5:
            try:
                self.mav_conn.mav.param_request_read_send(
                    self.target_system,
                    self.target_component,
                    b'SCR_USER1',
                    -1
                )
                self.param_request_time = now
                self.get_logger().info("[MissionNode] SCR_USER1 parametre isteği gönderildi.")
            except Exception as e:
                self.get_logger().error(f"[MissionNode] Parametre istek hatası: {e}")

    def set_mav_mode(self, mode_name: str):
        if self.mav_conn is None:
            return
        now = time.time()
        if now - self.last_mode_cmd_time < 1.0:
            return
        self.last_mode_cmd_time = now

        try:
            if mode_name in self.mav_conn.mode_mapping():
                mode_id = self.mav_conn.mode_mapping()[mode_name]
                self.mav_conn.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id,
                    0, 0, 0, 0, 0
                )
                self.get_logger().info(f"[MissionNode] Kip değiştirme komutu gönderildi: {mode_name}")
            else:
                self.get_logger().error(f"[MissionNode] Bilinmeyen mod adı: {mode_name}")
        except Exception as e:
            self.get_logger().error(f"[MissionNode] Mod değiştirme hatası: {e}")

    def set_mission_current(self, wp_seq: int):
        if self.mav_conn is None:
            return
        try:
            self.mav_conn.mav.mission_set_current_send(
                self.target_system,
                self.target_component,
                wp_seq
            )
            self.get_logger().info(f"[MissionNode] MISSION_SET_CURRENT gönderildi: Waypoint {wp_seq}")
        except Exception as e:
            self.get_logger().error(f"[MissionNode] Waypoint ayarlama hatası: {e}")

    def send_guided_velocity(self, vx: float, yaw_rate: float):
        self.last_cmd = (vx, yaw_rate)
        if self.mav_conn is None:
            return

        # SET_POSITION_TARGET_LOCAL_NED + MAV_FRAME_BODY_OFFSET_NED
        # Mask 1527 (0x05F7): x_vel ve yaw_rate aktif, diğerleri yoksayılır
        type_mask = 1527
        try:
            self.mav_conn.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                type_mask,
                0, 0, 0,  # x, y, z positions
                vx, 0, 0,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz
                0,  # yaw
                yaw_rate  # yaw_rate
            )
        except Exception as e:
            self.get_logger().error(f"[MissionNode] MAVLink hareket komutu hatası: {e}", throttle_duration_sec=2.0)

    def send_stop(self):
        self.send_guided_velocity(0.0, 0.0)

    def control_loop(self):
        """10 Hz periyodik state machine döngüsü."""
        # 1. Parkur-3 aktif değilse dur ve bekle
        if not self.parkur3_active:
            if self.state != MissionState.WAIT_PARKUR3:
                self.send_stop()
                self.state = MissionState.WAIT_PARKUR3
            self.get_logger().info("[MissionNode] BEKLEMEDE (Parkur-3 sinyali bekleniyor)...", throttle_duration_sec=5.0)
            return

        # 2. MAVLink mesajlarını oku ve HEARTBEAT / PARAM güncelle
        self.update_mavlink()

        now = time.time()

        # 3. State Machine Mantığı
        if self.state == MissionState.WAIT_PARKUR3:
            self.get_logger().info("[MissionNode] Parkur-3 aktif! ENTER_GUIDED durumuna geçiliyor.")
            self.state = MissionState.ENTER_GUIDED

        elif self.state == MissionState.ENTER_GUIDED:
            if not self.connect_pixhawk():
                self.send_stop()
                return

            if self.target_color is None:
                self.request_scr_user1_param()
                # Parametre hemen gelmezse varsayılan hedef belirleme (2 saniye zaman aşımı)
                if now - self.param_request_time > 2.5 and self.target_color is None:
                    self.target_color_id = self.default_target_color_id
                    self.target_color = COLOR_MAP.get(self.default_target_color_id, "kirmizi_duba")
                    self.get_logger().warn(f"[MissionNode] SCR_USER1 yanıtı gecikti, varsayılan hedef seçildi: '{self.target_color}'")

            if self.current_mode != 'GUIDED':
                self.set_mav_mode('GUIDED')
            else:
                self.get_logger().info(f"[MissionNode] GUIDED moda geçildi! (HEARTBEAT Doğrulandı) | Hedef renk: '{self.target_color}'. Hedef aranıyor...")
                self.state = MissionState.SEARCH_TARGET

        elif self.state in [MissionState.SEARCH_TARGET, MissionState.ALIGN_TARGET, MissionState.APPROACH_TARGET]:
            time_since_det = now - self.last_detection_time if self.last_detection_time > 0 else 999.0

            if time_since_det > 2.0:
                # >2 sn kayıpta tekrar hedef aramaya geç ve STOP yap
                if self.state != MissionState.SEARCH_TARGET:
                    self.get_logger().warn(f"[MissionNode] Hedef {time_since_det:.1f} sn kayboldu! SEARCH_TARGET durumuna dönülüyor.")
                    self.state = MissionState.SEARCH_TARGET
                self.send_stop()

            elif 0.5 <= time_since_det <= 2.0:
                # 0.5-2 sn kayıpta STOP
                self.send_stop()

            else:  # time_since_det < 0.5 (Hedef takibi aktif veya <0.5s kayıp)
                if self.latest_detection is None:
                    self.send_stop()
                    return

                center_x = float(self.latest_detection.get("center_x", self.cx))
                distance = float(self.latest_detection.get("distance", 99.0))
                error = center_x - self.cx

                # Dönüşte yaw_rate = clamp(error * 0.002, -0.35, +0.35)
                raw_yaw_rate = error * 0.002
                yaw_rate = max(-0.35, min(0.35, raw_yaw_rate))

                # Hedef mesafe kontrolü
                if distance <= 1.0:
                    # distance <= 1m: yaw düzeltmesini azalt, kısa son ileri itki uygula
                    self.get_logger().info(f"[MissionNode] Hedefe ulaşıldı! (Mesafe: {distance:.2f}m <= 1.0m). Son kamikaze itkisi uygulanıyor.")
                    self.hit_start_time = now
                    self.state = MissionState.HIT_TARGET
                    self.send_guided_velocity(0.8, yaw_rate * 0.2)

                elif distance > 2.0:
                    # Normal yaklaşma
                    if abs(error) <= 30.0:
                        self.state = MissionState.APPROACH_TARGET
                        self.send_guided_velocity(0.8, yaw_rate)
                    else:
                        self.state = MissionState.ALIGN_TARGET
                        self.send_guided_velocity(0.2, yaw_rate)

                else:  # 1.0 < distance <= 2.0
                    # Kontrollü / yavaş yaklaşma
                    self.state = MissionState.APPROACH_TARGET
                    if abs(error) <= 30.0:
                        self.send_guided_velocity(0.4, yaw_rate)
                    else:
                        self.send_guided_velocity(0.2, yaw_rate)

        elif self.state == MissionState.HIT_TARGET:
            elapsed = now - (self.hit_start_time or now)
            if elapsed < self.hit_duration_sec:
                # Kısa son ileri itki devam ediyor (yaw_rate 0.0)
                self.send_guided_velocity(0.8, 0.0)
            else:
                # Vuruş tamamlandı, STOP yap ve dönüş aşamasına geç
                self.send_stop()
                self.get_logger().info("[MissionNode] Temas/Vuruş tamamlandı! STOP uygulandı. AUTO moduna geçiliyor.")
                self.state = MissionState.RETURN_AUTO

        elif self.state == MissionState.RETURN_AUTO:
            self.set_mission_current(self.return_wp)
            if self.current_mode != 'AUTO':
                self.set_mav_mode('AUTO')
            else:
                self.get_logger().info(f"[MissionNode] AUTO moduna geçiş doğrulandı! Dönüş waypoint: {self.return_wp}. Görev TAMAMLANDI.")
                self.state = MissionState.MISSION_DONE

        elif self.state == MissionState.MISSION_DONE:
            self.send_stop()
            self.get_logger().info("[MissionNode] GÖREV TAMAMLANDI. Araç AUTO modda dönüşe devam ediyor.", throttle_duration_sec=10.0)

    def destroy_node(self):
        self.get_logger().warn("[MissionNode] Node kapatılıyor, Pixhawk'a güvenli STOP gönderiliyor...")
        self.send_stop()
        if self.mav_conn is not None:
            try:
                self.mav_conn.close()
            except Exception:
                pass
            self.mav_conn = None
        super().destroy_node()


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
