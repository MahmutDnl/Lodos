#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros — İHA MAVLink Köprü Scripti
============================================
İHA'nın Raspberry Pi 5'inde çalışır. Görevleri:

  1. Kameradan plaka rengini algılar (iha_color_detector modülü).
  2. Tespit edilen rengi Pixhawk 2.4.8'e USB üzerinden MAVLink mesajı
     olarak gönderir.
  3. Pixhawk bu mesajı otomatik olarak TELEM1 portundaki 3DR telemetri
     modülüne iletir → YKİ alır.

MAVLink Mesaj Formatı:
  - NAMED_VALUE_FLOAT : name="TCOLOR", value=1.0/2.0/3.0
  - STATUSTEXT        : text="TARGET:kirmizi" / "TARGET:yesil" / "TARGET:siyah"

Bağlantı:
  İHA Raspberry Pi 5  ──USB (sadece sinyal hatları)──  İHA Pixhawk 2.4.8

Kullanım:
  python3 iha_mavlink_bridge.py                  (varsayılan: /dev/ttyACM0)
  python3 iha_mavlink_bridge.py --port /dev/ttyUSB0 --baud 57600
  python3 iha_mavlink_bridge.py --camera 1       (farklı kamera ID)

Ortam  : İHA Raspberry Pi 5 (ROS2 gerektirmez)
Yazar  : LODOS Takımı
"""

import argparse
import sys
import time

from pymavlink import mavutil

from iha_color_detector import (
    ColorDetector,
    COLOR_UNKNOWN,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLACK,
    COLOR_NAMES,
)


# ══════════════════════════════════════════════════════════════════════════════
# Sabitler
# ══════════════════════════════════════════════════════════════════════════════

# MAVLink NAMED_VALUE_FLOAT mesaj adı (max 10 karakter)
MAVLINK_PARAM_NAME = "TCOLOR"

# Renk kodu → MAVLink float değeri
COLOR_TO_FLOAT = {
    COLOR_RED:   1.0,
    COLOR_GREEN: 2.0,
    COLOR_BLACK: 3.0,
}

# MAVLink STATUSTEXT severity seviyeleri
MAV_SEVERITY_INFO     = 6
MAV_SEVERITY_CRITICAL = 2

# Varsayılan ayarlar
DEFAULT_PORT       = "/dev/ttyACM0"
DEFAULT_BAUD       = 57600
DEFAULT_CAMERA_ID  = 0
DEFAULT_SEND_INTERVAL = 1.0   # Onaylandıktan sonra her 1 saniyede tekrar gönder
DEFAULT_CONFIRM_FRAMES = 5    # Ardışık kaç frame aynı rengi vermeli


# ══════════════════════════════════════════════════════════════════════════════
# MAVLink Bağlantı ve Gönderim Fonksiyonları
# ══════════════════════════════════════════════════════════════════════════════

def connect_pixhawk(port: str, baud: int) -> mavutil.mavlink_connection:
    """
    Pixhawk'a MAVLink bağlantısı kurar.

    Args:
        port: Seri port adresi (örn: /dev/ttyACM0)
        baud: Baud rate (örn: 57600)

    Returns:
        mavutil.mavlink_connection nesnesi
    """
    print(f"[BAĞLANTI] Pixhawk'a bağlanılıyor: {port} @ {baud} baud...")

    connection = mavutil.mavlink_connection(port, baud=baud)

    # Heartbeat beklenir — Pixhawk'ın hazır olduğunun kanıtı
    print("[BAĞLANTI] Heartbeat bekleniyor...")
    connection.wait_heartbeat(timeout=30)

    print(
        f"[BAĞLANTI] ✓ Pixhawk bağlantısı kuruldu! "
        f"(System: {connection.target_system}, "
        f"Component: {connection.target_component})"
    )
    return connection


def send_named_value_float(
    connection: mavutil.mavlink_connection,
    name: str,
    value: float,
):
    """
    MAVLink NAMED_VALUE_FLOAT mesajı gönderir.
    Pixhawk bu mesajı telemetri portuna otomatik olarak iletir.

    Args:
        connection: MAVLink bağlantı nesnesi
        name: Parametre adı (max 10 karakter)
        value: Float değer
    """
    # İsmi 10 byte'a sığdır (MAVLink spesifikasyonu)
    name_bytes = name.encode('utf-8')[:10]

    connection.mav.named_value_float_send(
        time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
        name=name_bytes,
        value=value,
    )


def send_statustext(
    connection: mavutil.mavlink_connection,
    text: str,
    severity: int = MAV_SEVERITY_INFO,
):
    """
    MAVLink STATUSTEXT mesajı gönderir.
    YKİ ve QGroundControl/Mission Planner ekranlarında görünür.

    Args:
        connection: MAVLink bağlantı nesnesi
        text: Gönderilecek metin (max 50 karakter)
        severity: MAVLink severity seviyesi
    """
    # Metni 50 byte ile sınırla
    text_bytes = text.encode('utf-8')[:50]

    connection.mav.statustext_send(
        severity=severity,
        text=text_bytes,
    )


# ══════════════════════════════════════════════════════════════════════════════
# Ana Çalışma Döngüsü
# ══════════════════════════════════════════════════════════════════════════════

def run(port: str, baud: int, camera_id: int, confirm_frames: int):
    """
    Ana döngü:
      1. Kamerayı başlat
      2. Pixhawk'a bağlan
      3. Renk algıla ve onaylanana kadar taramaya devam et
      4. Onaylandıktan sonra periyodik olarak MAVLink mesajı gönder
    """

    separator = "=" * 60

    print(separator)
    print("  LODOS İHA — MAVLink Renk Gönderim Sistemi")
    print(separator)
    print(f"  Pixhawk Portu    : {port}")
    print(f"  Baud Rate        : {baud}")
    print(f"  Kamera ID        : {camera_id}")
    print(f"  Onay Frame Sayısı: {confirm_frames}")
    print(separator)
    print()

    # ── Adım 1: Kamerayı başlat ──
    detector = ColorDetector(
        camera_id=camera_id,
        confirmation_frames=confirm_frames,
    )

    if not detector.start():
        print("[HATA] Kamera başlatılamadı! Çıkılıyor.")
        sys.exit(1)

    # ── Adım 2: Pixhawk'a bağlan ──
    connection = connect_pixhawk(port, baud)

    # ── Adım 3: Renk algılama döngüsü ──
    print()
    print("[GÖREV] Plaka rengi algılanıyor...")
    print()

    confirmed_color = COLOR_UNKNOWN
    last_send_time = 0.0

    try:
        while True:
            color_code, color_name, confirmed = detector.detect()

            # Henüz onaylanmamışsa → taramaya devam et
            if not confirmed and confirmed_color == COLOR_UNKNOWN:
                if color_code != COLOR_UNKNOWN:
                    print(
                        f"\r  [TARAMA] Algılanan: {color_name:10s} | "
                        f"Teyit bekleniyor...   ",
                        end="", flush=True,
                    )
                time.sleep(0.05)
                continue

            # İlk onay anı
            if confirmed and confirmed_color == COLOR_UNKNOWN:
                confirmed_color = color_code
                print()
                print()
                print(separator)
                print(f"  ✓ HEDEF RENK ONAYLANDI: {color_name.upper()}")
                print(f"  ✓ Renk Kodu: {color_code}")
                print(f"  ✓ MAVLink gönderimi başlıyor...")
                print(separator)
                print()

            # ── Adım 4: Onaylanmış rengi periyodik olarak gönder ──
            now = time.time()
            if now - last_send_time >= DEFAULT_SEND_INTERVAL:
                last_send_time = now

                float_value = COLOR_TO_FLOAT.get(confirmed_color, 0.0)
                target_text = f"TARGET:{COLOR_NAMES[confirmed_color]}"

                # NAMED_VALUE_FLOAT gönder
                send_named_value_float(connection, MAVLINK_PARAM_NAME, float_value)

                # STATUSTEXT gönder
                send_statustext(
                    connection,
                    target_text,
                    severity=MAV_SEVERITY_CRITICAL,
                )

                print(
                    f"  [GÖNDERİM] {target_text} | "
                    f"TCOLOR={float_value:.1f} | "
                    f"Zaman: {time.strftime('%H:%M:%S')}"
                )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print()
        print()
        print("[BİLGİ] Kullanıcı tarafından durduruldu.")

    finally:
        detector.stop()
        connection.close()
        print("[BİLGİ] MAVLink bağlantısı kapatıldı.")


# ══════════════════════════════════════════════════════════════════════════════
# Komut Satırı Arayüzü
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="LODOS İHA — Plaka renk algılama ve MAVLink gönderim sistemi",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Örnekler:
  python3 iha_mavlink_bridge.py
  python3 iha_mavlink_bridge.py --port /dev/ttyUSB0 --baud 57600
  python3 iha_mavlink_bridge.py --camera 1 --confirm 10
        """,
    )

    parser.add_argument(
        "--port", type=str, default=DEFAULT_PORT,
        help=f"Pixhawk seri port adresi (varsayılan: {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD,
        help=f"Seri port baud rate (varsayılan: {DEFAULT_BAUD})"
    )
    parser.add_argument(
        "--camera", type=int, default=DEFAULT_CAMERA_ID,
        help=f"Kamera cihaz ID'si (varsayılan: {DEFAULT_CAMERA_ID})"
    )
    parser.add_argument(
        "--confirm", type=int, default=DEFAULT_CONFIRM_FRAMES,
        help=f"Renk onay frame sayısı (varsayılan: {DEFAULT_CONFIRM_FRAMES})"
    )

    args = parser.parse_args()
    run(args.port, args.baud, args.camera, args.confirm)


if __name__ == '__main__':
    main()
