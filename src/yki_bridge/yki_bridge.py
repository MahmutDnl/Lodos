#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros — YKİ Renk Köprü Scripti
=========================================
Yer Kontrol İstasyonu (YKİ) bilgisayarında çalışır.

Görev:
  1. İHA telemetrisinden (USB0 / 3DR Çift 1) gelen MAVLink mesajlarını dinler.
  2. İHA'nın gönderdiği hedef renk bilgisini (NAMED_VALUE_FLOAT veya STATUSTEXT)
     ayrıştırır ve ekrana basar.
  3. Renk bilgisini İDA telemetrisine (USB1 / 3DR Çift 2) MAVLink STATUSTEXT
     mesajı olarak aktarır.

Donanım Bağlantısı:
  USB0 (İHA 3DR alıcısı)  ←  İHA'dan renk verisi gelir
  USB1 (İDA 3DR alıcısı)  →  İDA'ya renk verisi gönderilir

Kullanım:
  python3 yki_bridge.py
  python3 yki_bridge.py --iha-port /dev/ttyUSB0 --ida-port /dev/ttyUSB1
  python3 yki_bridge.py --iha-port COM3 --ida-port COM4     (Windows)

Ortam  : YKİ bilgisayarı (ROS2 gerektirmez)
Yazar  : LODOS Takımı
"""

import argparse
import sys
import time
from datetime import datetime

from pymavlink import mavutil


# ══════════════════════════════════════════════════════════════════════════════
# Sabitler
# ══════════════════════════════════════════════════════════════════════════════

# MAVLink NAMED_VALUE_FLOAT parametre adı (İHA ile aynı)
MAVLINK_PARAM_NAME = "TCOLOR"

# Float → renk adı eşlemesi
FLOAT_TO_COLOR_NAME = {
    1.0: "kirmizi",
    2.0: "yesil",
    3.0: "siyah",
}

# Geçerli renk adları
VALID_COLOR_NAMES = {"kirmizi", "yesil", "siyah"}

# MAVLink STATUSTEXT severity
MAV_SEVERITY_CRITICAL = 2
MAV_SEVERITY_INFO     = 6

# Varsayılan portlar
DEFAULT_IHA_PORT = "/dev/ttyUSB0"
DEFAULT_IDA_PORT = "/dev/ttyUSB1"
DEFAULT_BAUD     = 57600

# Gönderim tekrarlama aralığı (saniye)
RESEND_INTERVAL = 2.0


# ══════════════════════════════════════════════════════════════════════════════
# Yardımcı Fonksiyonlar
# ══════════════════════════════════════════════════════════════════════════════

def timestamp() -> str:
    """Şu anki zamanı HH:MM:SS formatında döner."""
    return datetime.now().strftime("%H:%M:%S")


def parse_statustext_color(text: str) -> str:
    """
    STATUSTEXT mesajından renk bilgisini ayrıştırır.

    Beklenen format: "TARGET:kirmizi"

    Returns:
        Renk adı (str) veya boş string (geçersiz ise)
    """
    if not text:
        return ""

    # Null byte'ları temizle (MAVLink padding)
    text = text.replace('\x00', '').strip()

    if ":" not in text:
        return ""

    parts = text.split(":", 1)
    if len(parts) != 2:
        return ""

    prefix = parts[0].strip().upper()
    color  = parts[1].strip().lower()

    if prefix != "TARGET":
        return ""

    if color in VALID_COLOR_NAMES:
        return color

    return ""


def connect_port(port: str, baud: int, label: str) -> mavutil.mavlink_connection:
    """
    Belirtilen seri porta MAVLink bağlantısı kurar.

    Args:
        port: Seri port adresi
        baud: Baud rate
        label: Bağlantı etiketi (log mesajları için)

    Returns:
        mavutil.mavlink_connection nesnesi
    """
    print(f"[{label}] Bağlanılıyor: {port} @ {baud} baud...")

    connection = mavutil.mavlink_connection(port, baud=baud)

    print(f"[{label}] Heartbeat bekleniyor...")
    heartbeat = connection.wait_heartbeat(timeout=30)

    if heartbeat is None:
        print(f"[{label}] ✗ Heartbeat alınamadı! Port: {port}")
        print(f"[{label}]   Bağlantıyı kontrol edin ve tekrar deneyin.")
        sys.exit(1)

    print(
        f"[{label}] ✓ Bağlantı kuruldu! "
        f"(System: {connection.target_system}, "
        f"Component: {connection.target_component})"
    )
    return connection


# ══════════════════════════════════════════════════════════════════════════════
# Ana Köprü Döngüsü
# ══════════════════════════════════════════════════════════════════════════════

def run_bridge(iha_port: str, ida_port: str, baud: int):
    """
    Ana köprü döngüsü:
      1. İHA ve İDA telemetri portlarına bağlan.
      2. İHA'dan gelen MAVLink mesajlarını dinle.
      3. Hedef renk algılandığında İDA'ya aktar.
    """

    separator = "=" * 64

    print(separator)
    print("  LODOS YKİ — İHA → İDA Renk Köprü Sistemi")
    print(separator)
    print(f"  İHA Portu (okuma) : {iha_port}")
    print(f"  İDA Portu (yazma) : {ida_port}")
    print(f"  Baud Rate         : {baud}")
    print(separator)
    print()

    # ── Bağlantıları kur ──
    iha_conn = connect_port(iha_port, baud, "İHA")
    print()
    ida_conn = connect_port(ida_port, baud, "İDA")
    print()

    print(separator)
    print("  ✓ Her iki bağlantı hazır. Renk verisi bekleniyor...")
    print(separator)
    print()

    # ── Durum değişkenleri ──
    detected_color = ""
    last_send_time = 0.0
    total_received = 0
    total_forwarded = 0

    try:
        while True:
            # ── İHA'dan MAVLink mesajı oku ──
            msg = iha_conn.recv_match(blocking=False)

            if msg is not None:
                msg_type = msg.get_type()

                # Yöntem 1: NAMED_VALUE_FLOAT mesajı
                if msg_type == "NAMED_VALUE_FLOAT":
                    name = msg.name
                    # Null byte'ları temizle
                    if isinstance(name, bytes):
                        name = name.decode('utf-8', errors='ignore')
                    name = name.replace('\x00', '').strip()

                    if name == MAVLINK_PARAM_NAME:
                        value = msg.value
                        color = FLOAT_TO_COLOR_NAME.get(value, "")

                        if color:
                            total_received += 1
                            detected_color = color
                            print(
                                f"  [{timestamp()}] [İHA→YKİ] "
                                f"NAMED_VALUE_FLOAT: {name}={value:.1f} "
                                f"→ Renk: {color.upper()}"
                            )

                # Yöntem 2: STATUSTEXT mesajı
                elif msg_type == "STATUSTEXT":
                    text = msg.text
                    if isinstance(text, bytes):
                        text = text.decode('utf-8', errors='ignore')

                    color = parse_statustext_color(text)

                    if color:
                        total_received += 1
                        detected_color = color
                        print(
                            f"  [{timestamp()}] [İHA→YKİ] "
                            f"STATUSTEXT: \"{text.strip()}\" "
                            f"→ Renk: {color.upper()}"
                        )

            # ── Algılanan rengi İDA'ya gönder ──
            if detected_color:
                now = time.time()
                if now - last_send_time >= RESEND_INTERVAL:
                    last_send_time = now
                    total_forwarded += 1

                    target_text = f"TARGET:{detected_color}"

                    # İDA'ya STATUSTEXT olarak gönder
                    ida_conn.mav.statustext_send(
                        severity=MAV_SEVERITY_CRITICAL,
                        text=target_text.encode('utf-8')[:50],
                    )

                    print(
                        f"  [{timestamp()}] [YKİ→İDA] "
                        f"STATUSTEXT: \"{target_text}\" "
                        f"(#{total_forwarded})"
                    )

            time.sleep(0.01)  # CPU yükünü azalt

    except KeyboardInterrupt:
        print()
        print()
        print(separator)
        print("  LODOS YKİ Köprü — Oturum Özeti")
        print(separator)
        print(f"  İHA'dan alınan mesaj  : {total_received}")
        print(f"  İDA'ya aktarılan mesaj: {total_forwarded}")
        print(f"  Son algılanan renk    : {detected_color or 'Yok'}")
        print(separator)

    finally:
        iha_conn.close()
        ida_conn.close()
        print("[BİLGİ] Tüm bağlantılar kapatıldı.")


# ══════════════════════════════════════════════════════════════════════════════
# Komut Satırı Arayüzü
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="LODOS YKİ — İHA → İDA Renk Köprü Sistemi",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Örnekler:
  python3 yki_bridge.py
  python3 yki_bridge.py --iha-port /dev/ttyUSB0 --ida-port /dev/ttyUSB1
  python3 yki_bridge.py --iha-port COM3 --ida-port COM4   (Windows)
        """,
    )

    parser.add_argument(
        "--iha-port", type=str, default=DEFAULT_IHA_PORT,
        help=f"İHA telemetri alıcısı seri portu (varsayılan: {DEFAULT_IHA_PORT})"
    )
    parser.add_argument(
        "--ida-port", type=str, default=DEFAULT_IDA_PORT,
        help=f"İDA telemetri alıcısı seri portu (varsayılan: {DEFAULT_IDA_PORT})"
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD,
        help=f"Seri port baud rate (varsayılan: {DEFAULT_BAUD})"
    )

    args = parser.parse_args()
    run_bridge(args.iha_port, args.ida_port, args.baud)


if __name__ == '__main__':
    main()
