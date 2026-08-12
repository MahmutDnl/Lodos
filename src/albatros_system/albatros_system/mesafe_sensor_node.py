#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros Insansiz Deniz Araci — Mesafe Sensoru Publisher Node
# =============================================================================
# Dosya    : mesafe_sensor_node.py
# Node adi : mesafe_sensor_node
# Paket    : albatros_system
# Gorev    : USB seri port (/dev/ttyUSB0) uzerinden Arduino'dan gelen
#            ultrasonik mesafe sensoru verilerini okuyarak sensor_msgs/Range
#            formatinda ROS 2 topic'lerine yayinlar.
#
# Arduino Seri Cikti Formati:
#   Sensor 1: 28.19 cm | Sensor 2: 20.12 cm
#
# Yayinlanan Topic'ler (sensor_msgs/msg/Range):
#   /albatros/mesafe/on_sol   — Sensor 1 (frame_id: on_sol_sensor_link)
#   /albatros/mesafe/on_sag   — Sensor 2 (frame_id: on_sag_sensor_link)
#
# ROS Parametreleri:
#   serial_port        (string, varsayilan: '/dev/ttyUSB0')
#   baud_rate          (int,    varsayilan: 9600)
#   field_of_view      (float,  varsayilan: 0.26 rad)
#   min_range          (float,  varsayilan: 0.20 m)
#   max_range          (float,  varsayilan: 4.50 m)
#   read_rate          (float,  varsayilan: 20.0 Hz)
# =============================================================================

import re
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Regex format for Arduino serial output: "Sensor 1: 28.19 cm | Sensor 2: 20.12 cm"
SENSOR_PATTERN = re.compile(
    r"Sensor\s*1:\s*([\d.]+)\s*cm\s*\|\s*Sensor\s*2:\s*([\d.]+)\s*cm",
    re.IGNORECASE
)


class MesafeSensorNode(Node):
    """
    Arduino'dan gelen mesafe sensoru verilerini okuyan ve ROS 2 Range mesajlari yayinlayan node.
    """

    def __init__(self):
        super().__init__('mesafe_sensor_node')

        # ------------------------------------------------------------------ #
        # ROS Parametreleri
        # ------------------------------------------------------------------ #
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('field_of_view', 0.26)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 4.50)
        self.declare_parameter('read_rate', 20.0)

        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.field_of_view = float(self.get_parameter('field_of_view').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.read_rate = float(self.get_parameter('read_rate').value)

        # ------------------------------------------------------------------ #
        # Publisher'lar (Yalnizca on_sol ve on_sag)
        # ------------------------------------------------------------------ #
        self.pub_on_sol = self.create_publisher(Range, '/albatros/mesafe/on_sol', 10)
        self.pub_on_sag = self.create_publisher(Range, '/albatros/mesafe/on_sag', 10)

        # ------------------------------------------------------------------ #
        # Seri Port Değişkenleri & Buffer
        # ------------------------------------------------------------------ #
        self.ser = None
        self._buffer = b""
        self._connection_error_logged = False

        # Seri porta baglanmayi dene
        self._connect_serial()

        # ------------------------------------------------------------------ #
        # Timer (Non-blocking executor dostu periyodik okuma)
        # ------------------------------------------------------------------ #
        timer_period = 1.0 / self.read_rate if self.read_rate > 0 else 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("MesafeSensorNode baslatildi.")
        self.get_logger().info(f"  Serial Port: {self.serial_port_name} @ {self.baud_rate} baud")
        self.get_logger().info(f"  Field of View: {self.field_of_view} rad")
        self.get_logger().info(f"  Range: [{self.min_range}, {self.max_range}] m")

    def _connect_serial(self) -> bool:
        """
        Seri porta baglanir. Aciamazsa crash olmaz, ROS error log verir.
        """
        if self.ser is not None and self.ser.is_open:
            return True

        try:
            # timeout=0 -> non-blocking okuma
            self.ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0)
            self.get_logger().info(
                f"Seri port baglantisi basariyla acildi: {self.serial_port_name} ({self.baud_rate} baud)"
            )
            self._connection_error_logged = False
            return True
        except Exception as e:
            if not self._connection_error_logged:
                self.get_logger().error(
                    f"Seri port acilamadi ({self.serial_port_name}): {e}"
                )
                self._connection_error_logged = True
            self.ser = None
            return False

    def timer_callback(self):
        """
        Timer tarafindan periyodik olarak cagirilir.
        Seri port verilerini non-blocking sekilde okur ve parse eder.
        """
        if self.ser is None or not self.ser.is_open:
            # Baglanti yoksa yeniden baglanmayi dene
            self._connect_serial()
            return

        try:
            bytes_to_read = self.ser.in_waiting
            if bytes_to_read > 0:
                data = self.ser.read(bytes_to_read)
                self._buffer += data
                while b'\n' in self._buffer:
                    line_bytes, self._buffer = self._buffer.split(b'\n', 1)
                    line_str = line_bytes.decode('utf-8', errors='ignore').strip()
                    if line_str:
                        self._parse_and_publish(line_str)
        except serial.SerialException as e:
            self.get_logger().error(f"Seri port okuma hatasi: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self._connection_error_logged = False
        except Exception as e:
            self.get_logger().error(f"Beklenmedik okuma hatasi: {e}")

    def _parse_and_publish(self, line: str):
        """
        Arduino seri ciktisini parse eder ve Range mesajlarini yayinlar.
        Format: Sensor 1: 28.19 cm | Sensor 2: 20.12 cm
        Bozuk veya parse edilemeyen satirlar atlanir.
        """
        match = SENSOR_PATTERN.search(line)
        if not match:
            return

        try:
            val1_cm = float(match.group(1))
            val2_cm = float(match.group(2))
        except (ValueError, TypeError):
            # Parse edilemeyen/bozuk sayisal degerleri atla
            return

        # cm -> metre cevirimi
        val1_m = val1_cm / 100.0
        val2_m = val2_cm / 100.0

        msg_on_sol = self.create_range_message('on_sol_sensor_link', val1_m)
        msg_on_sag = self.create_range_message('on_sag_sensor_link', val2_m)

        self.pub_on_sol.publish(msg_on_sol)
        self.pub_on_sag.publish(msg_on_sag)

    def create_range_message(self, frame_id: str, distance_m: float) -> Range:
        """
        sensor_msgs/Range mesaji olusturur.
        """
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = float(max(self.min_range, min(distance_m, self.max_range)))
        return msg

    def destroy_node(self):
        """
        Node kapatilirken seri portu serbest birakir.
        """
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Seri port kapatildi.")
            except Exception as e:
                self.get_logger().warn(f"Seri port kapatma hatasi: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MesafeSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MesafeSensorNode durduruldu (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
