#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros — Hedef Renk Dinleyici ROS2 Node'u
====================================================
Parkur 3 Kamikaze Angajmanı için YKİ'den gelen hedef renk bilgisini
MAVROS üzerinden alır ve ROS2 ağında yayınlar.

Veri Akışı:
  İHA → 3DR Telemetri → YKİ → 3DR Telemetri → İDA Pixhawk → MAVROS
  → /mavros/statustext/recv → target_color_node
  → /perception/target_color (std_msgs/String)

Beklenen MAVLink Mesaj Formatı:
  STATUSTEXT: "TARGET:kirmizi" / "TARGET:yesil" / "TARGET:siyah"

Bu Node:
  ✓ MAVROS STATUSTEXT mesajlarını dinler.
  ✓ "TARGET:" prefix'li mesajları ayrıştırır.
  ✓ Geçerli renk bilgisini /perception/target_color'a yayınlar.
  ✓ Renk değişikliklerini loglar.
  ✗ Motor komutu üretmez.
  ✗ MAVROS'a komut göndermez.

Topic'ler:
  Giriş: /mavros/statustext/recv    (mavros_msgs/StatusText)
  Çıkış: /perception/target_color   (std_msgs/String)
  Çıkış: /perception/target_color/status (std_msgs/String - JSON durum)

Yazar  : LODOS Takımı
Araç   : Albatros İDA
Ortam  : Ubuntu 24.04 / ROS2 Jazzy
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from std_msgs.msg import String

# MAVROS statustext mesaj tipi
# mavros_msgs paketi kurulu ise bu import kullanılır,
# kurulu değilse std_msgs/String üzerinden çalışılır.
try:
    from mavros_msgs.msg import StatusText
    MAVROS_STATUSTEXT_AVAILABLE = True
except ImportError:
    MAVROS_STATUSTEXT_AVAILABLE = False


# ══════════════════════════════════════════════════════════════════════════════
# Sabitler
# ══════════════════════════════════════════════════════════════════════════════

# Topic tanımları
MAVROS_STATUSTEXT_TOPIC   = '/mavros/statustext/recv'
TARGET_COLOR_TOPIC        = '/perception/target_color'
TARGET_COLOR_STATUS_TOPIC = '/perception/target_color/status'

# Geçerli hedef renk değerleri
VALID_COLORS = {'kirmizi', 'yesil', 'siyah'}

# Varsayılan ayarlar
DEFAULT_PUBLISH_RATE   = 2.0    # Hz — onaylanmış rengi bu frekansta yayınla
DEFAULT_STATUS_RATE    = 1.0    # Hz — durum bilgisini yayınla
DEFAULT_COLOR_TIMEOUT  = 30.0   # saniye — bu süre içinde yeni veri gelmezse uyar


# ══════════════════════════════════════════════════════════════════════════════
# TargetColorNode
# ══════════════════════════════════════════════════════════════════════════════

class TargetColorNode(Node):
    """
    MAVROS STATUSTEXT mesajlarından İHA'nın algıladığı hedef rengi alır
    ve /perception/target_color topic'ine yayınlar.
    """

    def __init__(self):
        super().__init__('target_color_node')

        # ── Parametreler ──
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE)
        self.declare_parameter('status_rate', DEFAULT_STATUS_RATE)
        self.declare_parameter('color_timeout_sec', DEFAULT_COLOR_TIMEOUT)

        self._publish_rate  = float(self.get_parameter('publish_rate').value)
        self._status_rate   = float(self.get_parameter('status_rate').value)
        self._color_timeout = float(self.get_parameter('color_timeout_sec').value)

        # ── Dahili Durum ──
        self._confirmed_color = ""            # Onaylanmış renk adı
        self._last_receive_time = 0.0         # Son renk mesajı alınma zamanı
        self._total_received = 0              # Toplam alınan renk mesajı sayısı
        self._color_history = []              # Alınan renk geçmişi (son 10)

        # ── Subscriber ──
        if MAVROS_STATUSTEXT_AVAILABLE:
            self.create_subscription(
                StatusText,
                MAVROS_STATUSTEXT_TOPIC,
                self._cb_statustext_mavros,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f'MAVROS StatusText subscriber aktif: {MAVROS_STATUSTEXT_TOPIC}'
            )
        else:
            # mavros_msgs kurulu değilse — fallback olarak String dinle
            self.create_subscription(
                String,
                MAVROS_STATUSTEXT_TOPIC,
                self._cb_statustext_string,
                10,
            )
            self.get_logger().warn(
                'mavros_msgs bulunamadı! String tipinde dinleniyor.'
            )

        # ── Publisher'lar ──
        self._pub_color = self.create_publisher(
            String, TARGET_COLOR_TOPIC, 10
        )

        self._pub_status = self.create_publisher(
            String, TARGET_COLOR_STATUS_TOPIC, 10
        )

        # ── Timer: Onaylanmış rengi periyodik yayınla ──
        color_period = 1.0 / max(self._publish_rate, 0.1)
        self._color_timer = self.create_timer(color_period, self._timer_publish_color)

        # ── Timer: Durum bilgisini periyodik yayınla ──
        status_period = 1.0 / max(self._status_rate, 0.1)
        self._status_timer = self.create_timer(status_period, self._timer_publish_status)

        # ── Başlangıç logları ──
        sep = '=' * 60
        self.get_logger().info(sep)
        self.get_logger().info('Target Color Node başlatıldı.')
        self.get_logger().info(f'  MAVROS giriş   : {MAVROS_STATUSTEXT_TOPIC}')
        self.get_logger().info(f'  Renk çıkış     : {TARGET_COLOR_TOPIC}')
        self.get_logger().info(f'  Durum çıkış    : {TARGET_COLOR_STATUS_TOPIC}')
        self.get_logger().info(f'  Yayın frekansı : {self._publish_rate} Hz')
        self.get_logger().info(f'  Renk timeout   : {self._color_timeout} s')
        self.get_logger().info(sep)

    # ══════════════════════════════════════════════════════════════════════
    # MAVROS Callback'ler
    # ══════════════════════════════════════════════════════════════════════

    def _cb_statustext_mavros(self, msg: 'StatusText'):
        """
        MAVROS StatusText mesajı callback'i.
        mavros_msgs/StatusText: severity (uint8) + text (string)
        """
        self._process_text(msg.text)

    def _cb_statustext_string(self, msg: String):
        """
        Fallback: mavros_msgs yoksa std_msgs/String olarak dinler.
        """
        self._process_text(msg.data)

    def _process_text(self, text: str):
        """
        STATUSTEXT metnini ayrıştırır ve hedef rengi günceller.

        Beklenen format: "TARGET:kirmizi"
        """
        if not text:
            return

        # Null byte ve boşluk temizliği
        text = text.replace('\x00', '').strip()

        # "TARGET:" prefix kontrolü
        if ":" not in text:
            return

        parts = text.split(":", 1)
        if len(parts) != 2:
            return

        prefix = parts[0].strip().upper()
        color  = parts[1].strip().lower()

        if prefix != "TARGET":
            return

        if color not in VALID_COLORS:
            self.get_logger().warn(
                f'Geçersiz renk değeri alındı: "{color}" '
                f'(geçerli: {VALID_COLORS})'
            )
            return

        # ── Renk güncelleme ──
        self._total_received += 1
        self._last_receive_time = time.time()

        # Geçmiş kaydı (son 10)
        self._color_history.append({
            'color': color,
            'time': self._last_receive_time,
        })
        if len(self._color_history) > 10:
            self._color_history.pop(0)

        # Renk değişikliği logla
        if color != self._confirmed_color:
            old_color = self._confirmed_color or "yok"
            self._confirmed_color = color

            self.get_logger().info(
                f'═══════════════════════════════════════════════'
            )
            self.get_logger().info(
                f'  ✓ HEDEF RENK GÜNCELLENDİ: {old_color} → {color.upper()}'
            )
            self.get_logger().info(
                f'═══════════════════════════════════════════════'
            )
        else:
            self.get_logger().debug(
                f'Hedef renk tekrarı alındı: {color} (#{self._total_received})'
            )

    # ══════════════════════════════════════════════════════════════════════
    # Timer Callback'ler
    # ══════════════════════════════════════════════════════════════════════

    def _timer_publish_color(self):
        """Onaylanmış rengi periyodik olarak yayınlar."""
        if not self._confirmed_color:
            return

        msg = String()
        msg.data = self._confirmed_color
        self._pub_color.publish(msg)

    def _timer_publish_status(self):
        """Hedef renk node durumunu JSON olarak yayınlar."""
        now = time.time()
        time_since_last = (
            now - self._last_receive_time
            if self._last_receive_time > 0 else -1.0
        )

        color_valid = (
            bool(self._confirmed_color)
            and time_since_last >= 0
            and time_since_last < self._color_timeout
        )

        status = {
            'target_color': self._confirmed_color or 'yok',
            'color_valid': color_valid,
            'total_received': self._total_received,
            'time_since_last_sec': round(time_since_last, 1),
            'timeout_sec': self._color_timeout,
        }

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._pub_status.publish(msg)

        # Timeout uyarısı
        if (self._confirmed_color
                and time_since_last > self._color_timeout):
            self.get_logger().warn(
                f'Hedef renk verisi {self._color_timeout:.0f} saniyedir '
                f'güncellenmiyor! Son renk: {self._confirmed_color}'
            )


# ══════════════════════════════════════════════════════════════════════════════
# Entry Point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = TargetColorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Target Color Node durduruldu. '
            f'Toplam alınan mesaj: {node._total_received} | '
            f'Son renk: {node._confirmed_color or "yok"}'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
