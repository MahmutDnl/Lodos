#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros — İHA-YKİ-İDA Renk Aktarım ve Algılama Sistemi
==============================================================
TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması için geliştirilmiştir.

Bu tek dosya, 3 farklı cihazda 3 farklı rol ile çalışabilir:
  1. İHA (Raspberry Pi 5) : python3 lodos_color_system.py --role iha
  2. YKİ (Yer Kontrol)    : python3 lodos_color_system.py --role yki
  3. İDA (Raspberry Pi 5) : python3 lodos_color_system.py --role ida

Mimaride ROS/ROS2 kullanılmaz. Haberleşme sadece MAVLink parametre (SCR_USER1)
aktarımı üzerinedir. Haberleşme akışı: İHA -> YKİ -> İDA şeklindedir.

Yazar: LODOS Takımı
"""

import os
import sys
import time
import argparse
import logging
from datetime import datetime
import numpy as np
import cv2

# Import pymavlink (if not in dry-run/simulation or if available)
try:
    from pymavlink import mavutil
except ImportError:
    # We'll handle this in run time or if dry-run is specified
    mavutil = None

# ==============================================================================
# CONFIG
# ==============================================================================

# ArduPilot Parametre Ayarları
COLOR_PARAM_NAME = "SCR_USER1"

# Renk Kodları Sabitleri
COLOR_NOT_READY = 0
COLOR_RED = 1
COLOR_GREEN = 2
COLOR_BLACK = 3

COLOR_NAMES = {
    COLOR_NOT_READY: "NOT_READY",
    COLOR_RED: "RED",
    COLOR_GREEN: "GREEN",
    COLOR_BLACK: "BLACK"
}

# Bağlantı ve Port Ayarları
IHA_PIXHAWK_PORT = "/dev/ttyACM0"
IHA_PIXHAWK_BAUD = 115200

IDA_PIXHAWK_PORT = "/dev/ttyACM0"
IDA_PIXHAWK_BAUD = 115200

IHA_TELEM_PORT = "/dev/ttyUSB0"
IDA_TELEM_PORT = "/dev/ttyUSB1"
TELEM_BAUD = 57600

# Kamera Ayarları
CAMERA_ID = 0
CAMERA_BACKEND = "auto" # "auto", "v4l2", "libcamera"

# Dörtgen Plaka Filtre Ayarları
BLUR_KERNEL = 5
CANNY_THRESHOLD1 = 50
CANNY_THRESHOLD2 = 150
MIN_CONTOUR_AREA = 1000       # Görüntüdeki minimum plaka alanı (piksel)
MAX_CONTOUR_AREA = 200000     # Görüntüdeki maksimum plaka alanı (piksel)
ASPECT_RATIO_MIN = 0.5        # Plaka genişlik/yükseklik oranı minimum
ASPECT_RATIO_MAX = 2.0        # Plaka genişlik/yükseklik oranı maksimum
ROI_CENTER_RATIO = 0.75       # ROI'nin merkezdeki yüzde kaçlık bölümü kullanılacak (0.75 = %75)

# HSV Renk Eşikleri (H: 0-179, S: 0-255, V: 0-255)
# Kırmızı (Çift aralık)
RED_LOW_1 = np.array([0, 70, 50])
RED_HIGH_1 = np.array([10, 255, 255])
RED_LOW_2 = np.array([170, 70, 50])
RED_HIGH_2 = np.array([179, 255, 255])

# Yeşil
GREEN_LOW = np.array([35, 50, 50])
GREEN_HIGH = np.array([85, 255, 255])

# Siyah (Düşük Value/parlaklık)
BLACK_LOW = np.array([0, 0, 0])
BLACK_HIGH = np.array([179, 255, 50])

# Renk Karar Ayarları
VOTE_WINDOW = 10
VOTE_REQUIRED = 7
DETECTION_TIMEOUT_SEC = 5.0

# Zamanlama ve Yeniden Bağlanma Ayarları
PARAM_POLL_INTERVAL = 0.5
RECONNECT_INTERVAL_SEC = 2.0

# Log Dosyası Yolu
LOG_FILE_PATH = "lodos_color_system.log"

# ==============================================================================
# LOGGING VE YARDIMCI FONKSİYONLAR
# ==============================================================================

def setup_logger():
    """Dosyaya ve konsola yazan log sistemini hazırlar."""
    logger = logging.getLogger("LodosColorSystem")
    logger.setLevel(logging.DEBUG)
    
    # Format
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    
    # Console Handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    
    # File Handler
    try:
        fh = logging.FileHandler(LOG_FILE_PATH, encoding='utf-8')
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(formatter)
        logger.addHandler(fh)
    except Exception as e:
        print(f"Log dosyası oluşturulamadı: {e}")
        
    return logger

logger = setup_logger()

# ==============================================================================
# MAVLINK YÖNETİMİ
# ==============================================================================

def connect_mavlink(port, baud, label, dry_run=False):
    """
    Belirtilen porta MAVLink bağlantısı kurar ve heartbeat bekler.
    """
    if dry_run:
        logger.info(f"[{label}] [DRY-RUN] Simüle bağlantı kuruldu: {port}")
        return None

    if mavutil is None:
        logger.error(f"[{label}] pymavlink kütüphanesi yüklü değil! Bağlantı kurulamaz.")
        sys.exit(1)

    logger.info(f"[{label}] Bağlantı başlatılıyor: {port} @ {baud} baud...")
    while True:
        try:
            conn = mavutil.mavlink_connection(port, baud=baud)
            logger.info(f"[{label}] Heartbeat bekleniyor...")
            # Wait for heartbeat
            heartbeat = conn.wait_heartbeat(timeout=10)
            if heartbeat:
                logger.info(f"[{label}] Heartbeat ALINDI. System ID: {conn.target_system}, Component ID: {conn.target_component}")
                return conn
            else:
                logger.warning(f"[{label}] Heartbeat zaman aşımı! Yeniden deneniyor...")
        except Exception as e:
            logger.error(f"[{label}] Bağlantı hatası: {e}. {RECONNECT_INTERVAL_SEC} saniye sonra tekrar denenecek...")
        time.sleep(RECONNECT_INTERVAL_SEC)

def verify_param_exists(conn, param_name, label, dry_run=False):
    """
    Seçilen parametrenin Pixhawk üzerinde gerçekten var olup olmadığını kontrol eder.
    Var değilse programı kapatır.
    """
    if dry_run:
        logger.info(f"[{label}] [DRY-RUN] Parametre '{param_name}' mevcut kabul edildi.")
        return True

    logger.info(f"[{label}] Parametre '{param_name}' kontrol ediliyor...")
    
    # Request parameter
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component,
        param_name.encode('utf-8'),
        -1
    )
    
    # Wait for response
    start_time = time.time()
    while time.time() - start_time < 5.0:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if msg:
            received_name = msg.param_id
            if isinstance(received_name, bytes):
                received_name = received_name.decode('utf-8', errors='ignore')
            received_name = received_name.replace('\x00', '').strip()
            
            if received_name == param_name:
                logger.info(f"[{label}] ✓ Doğrulandı: Parametre '{param_name}' Pixhawk üzerinde mevcut. Mevcut değer: {msg.param_value}")
                return True
                
    logger.error(f"[{label}] ERROR: COLOR PARAMETER NOT FOUND ('{param_name}'). Lütfen Pixhawk üzerinde SCR_ENABLE=1 olduğundan emin olun.")
    sys.exit(1)

def write_param(conn, param_name, param_value, label, dry_run=False):
    """
    PARAM_SET mesajı göndererek parametreyi yazar ve PARAM_VALUE ile doğrular (3 retry).
    """
    if dry_run:
        logger.info(f"[{label}] [DRY-RUN] Parametre '{param_name}' değeri {param_value} olarak yazıldı.")
        return True

    for attempt in range(1, 4):
        logger.info(f"[{label}] Parametre Yazma Denemesi {attempt}: {param_name} = {param_value}")
        
        # Set parameter
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            param_name.encode('utf-8'),
            float(param_value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        
        # Verify write
        start_time = time.time()
        while time.time() - start_time < 2.0:
            msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg:
                received_name = msg.param_id
                if isinstance(received_name, bytes):
                    received_name = received_name.decode('utf-8', errors='ignore')
                received_name = received_name.replace('\x00', '').strip()
                
                if received_name == param_name:
                    if int(msg.param_value) == int(param_value):
                        logger.info(f"[{label}] ✓ Yazma Başarılı ve Doğrulandı: {param_name} = {msg.param_value}")
                        return True
                    else:
                        logger.warning(f"[{label}] Değer uyuşmadı: beklenen {param_value}, alınan {msg.param_value}")
                        
        logger.warning(f"[{label}] Parametre doğrulama zaman aşımı veya hatalı değer. Yeniden deneniyor...")
        time.sleep(0.5)
        
    logger.error(f"[{label}] HATA: Parametre {param_name} yazılamadı veya doğrulanamadı!")
    return False

def read_param_value(conn, param_name, label, dry_run=False):
    """
    Pixhawk'tan parametre değerini okur.
    """
    if dry_run:
        return COLOR_NOT_READY

    # Request read
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component,
        param_name.encode('utf-8'),
        -1
    )
    
    msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
    if msg:
        received_name = msg.param_id
        if isinstance(received_name, bytes):
            received_name = received_name.decode('utf-8', errors='ignore')
        received_name = received_name.replace('\x00', '').strip()
        
        if received_name == param_name:
            return int(msg.param_value)
            
    return None

# ==============================================================================
# OPENCV GÖRÜNTÜ İŞLEME VE DÖRTGEN TESPİTİ
# ==============================================================================

def order_points(pts):
    """Dörtgen köşe noktalarını sırasıyla: sol-üst, sağ-üst, sağ-alt, sol-alt şeklinde dizer."""
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def get_perspective_transform(image, pts):
    """Belirtilen 4 nokta üzerinden perspective transform uygular."""
    rect = order_points(pts)
    (tl, tr, br, bl) = rect

    # Genişlik hesabı
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # Yükseklik hesabı
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # Hedef koordinatlar
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")

    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped

def detect_quadrilateral(frame):
    """
    Görüntüdeki en uygun dörtgen plakayı bulur ve perspective transform uygulanmış ROI döner.
    Bulamazsa None döner.
    """
    # 1. 640x480 Çözünürlük
    resized = cv2.resize(frame, (640, 480))
    # 2. Gaussian Blur
    blurred = cv2.GaussianBlur(resized, (BLUR_KERNEL, BLUR_KERNEL), 0)
    # 3. Gri Görüntü
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    # 4. Canny Edge Detection
    edged = cv2.Canny(gray, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
    # 5. Konturları Bul
    contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_quad = None
    max_area = 0

    for c in contours:
        # Kontur alanı filtresi
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
            continue
            
        # Contour Approximation
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        
        # 4 köşeli ve convex kontrolü
        if len(approx) == 4 and cv2.isContourConvex(approx):
            # Aspect Ratio Kontrolü
            (x, y, w, h) = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if ASPECT_RATIO_MIN <= aspect_ratio <= ASPECT_RATIO_MAX:
                if area > max_area:
                    max_area = area
                    best_quad = approx.reshape(4, 2)
                    
    if best_quad is not None:
        # Perspective Transform
        try:
            warped = get_perspective_transform(resized, best_quad)
            return warped, best_quad
        except Exception as e:
            logger.warning(f"Perspective transform hatası: {e}")
            
    return None, None

def analyze_color(roi):
    """
    Kırpılmış plaka ROI görüntüsünün rengini HSV uzayında analiz eder.
    Baskın rengi ve maske oranlarını döner.
    """
    h, w = roi.shape[:2]
    # Dış kenarların ve çerçevelerin etki etmemesi için merkez %75'lik bölümü alalım
    margin_h = int(h * (1.0 - ROI_CENTER_RATIO) / 2.0)
    margin_w = int(w * (1.0 - ROI_CENTER_RATIO) / 2.0)
    
    # Sınır güvenlikleri
    if margin_h < 1: margin_h = 0
    if margin_w < 1: margin_w = 0
    
    roi_center = roi[margin_h:h-margin_h, margin_w:w-margin_w]
    roi_pixel_count = roi_center.shape[0] * roi_center.shape[1]
    
    if roi_pixel_count <= 0:
        return COLOR_NOT_READY, 0.0
        
    hsv = cv2.cvtColor(roi_center, cv2.COLOR_BGR2HSV)
    
    # 1. Kırmızı Maskesi (Çift aralık birleşimi)
    mask_red1 = cv2.inRange(hsv, RED_LOW_1, RED_HIGH_1)
    mask_red2 = cv2.inRange(hsv, RED_LOW_2, RED_HIGH_2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    # 2. Yeşil Maskesi
    mask_green = cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)
    
    # 3. Siyah Maskesi
    mask_black = cv2.inRange(hsv, BLACK_LOW, BLACK_HIGH)
    
    # Morfolojik temizleme
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)
    
    # Piksel sayımları
    red_count = cv2.countNonZero(mask_red)
    green_count = cv2.countNonZero(mask_green)
    black_count = cv2.countNonZero(mask_black)
    
    # Oranlar
    red_ratio = red_count / roi_pixel_count
    green_ratio = green_count / roi_pixel_count
    black_ratio = black_count / roi_pixel_count
    
    ratios = {
        COLOR_RED: red_ratio,
        COLOR_GREEN: green_ratio,
        COLOR_BLACK: black_ratio
    }
    
    # En yüksek oranlı rengi seç
    best_color = COLOR_NOT_READY
    max_ratio = 0.0
    
    for color, ratio in ratios.items():
        # Minimum %5 piksel doluluğu şartı
        if ratio > 0.05 and ratio > max_ratio:
            max_ratio = ratio
            best_color = color
            
    return best_color, ratios

# ==============================================================================
# ROL FONKSİYONLARI
# ==============================================================================

def run_iha(args):
    """
    İHA Rolü:
      1. Pixhawk'a bağlanır, COLOR_PARAM_NAME parametresinin varlığını doğrular.
      2. Başlangıçta parametreyi 0 yapar (eski renk bilgisini temizlemek için).
      3. Kamerayı açar ve plakayı/rengi tespit etmeye çalışır.
      4. Voting penceresi ile teyit eder.
      5. Timeout dolarsa failsafe olarak KIRMIZI (1) yazar.
      6. Tespit edilen rengi Pixhawk parametresine tek seferlik yazar.
    """
    logger.info("==================================================")
    logger.info("               İHA ROLÜ BAŞLATILDI")
    logger.info("==================================================")
    
    # Pixhawk Bağlantısı
    conn = connect_mavlink(args.port, args.baud, "İHA", args.dry_run)
    verify_param_exists(conn, COLOR_PARAM_NAME, "İHA", args.dry_run)
    
    # Başlangıçta parametreyi 0 yapıyoruz (eski renk verisi silinsin)
    logger.info(f"Eski renk verisi sıfırlanıyor: {COLOR_PARAM_NAME} = 0")
    write_param(conn, COLOR_PARAM_NAME, COLOR_NOT_READY, "İHA", args.dry_run)
    
    # Kamera Hazırlığı
    cap = None
    if not args.dry_run and args.simulate_color is None:
        logger.info(f"Kamera açılıyor: ID {CAMERA_ID}...")
        # Auto backend / V4L2
        cap = cv2.VideoCapture(CAMERA_ID)
        if not cap.isOpened():
            logger.error("Kamera açılamadı!")
            sys.exit(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
    start_time = time.time()
    vote_queue = []
    final_color = None
    
    logger.info(f"Renk algılama başladı. Maksimum süre: {DETECTION_TIMEOUT_SEC} saniye...")
    
    try:
        while True:
            elapsed_time = time.time() - start_time
            
            # Simülasyon / Dry Run Test Modu
            if args.simulate_color is not None:
                sim_color = args.simulate_color.lower()
                if sim_color == "red":
                    detected = COLOR_RED
                elif sim_color == "green":
                    detected = COLOR_GREEN
                elif sim_color == "black":
                    detected = COLOR_BLACK
                else:
                    detected = COLOR_NOT_READY
                
                # Simülasyonda 1 saniye bekledikten sonra doğrudan sonuca gitsin
                if elapsed_time > 1.0:
                    if detected != COLOR_NOT_READY:
                        final_color = detected
                        logger.info(f"[SİMÜLE] Renk başarıyla simüle edildi: {COLOR_NAMES[final_color]}")
                        break
            
            # Gerçek Görüntü İşleme
            elif cap is not None:
                ret, frame = cap.read()
                if not ret or frame is None:
                    logger.warning("Kameradan frame alınamadı!")
                    time.sleep(0.05)
                    continue
                    
                roi, quad_pts = detect_quadrilateral(frame)
                detected = COLOR_NOT_READY
                ratios = {COLOR_RED: 0.0, COLOR_GREEN: 0.0, COLOR_BLACK: 0.0}
                
                if roi is not None:
                    detected, ratios = analyze_color(roi)
                
                # Voting penceresine ekleme
                vote_queue.append(detected)
                if len(vote_queue) > VOTE_WINDOW:
                    vote_queue.pop(0)
                    
                # Voting analizi
                if len(vote_queue) == VOTE_WINDOW:
                    for color_code in [COLOR_RED, COLOR_GREEN, COLOR_BLACK]:
                        if vote_queue.count(color_code) >= VOTE_REQUIRED:
                            final_color = color_code
                            logger.info(f"✓ Renk teyit edildi (Voting: {vote_queue.count(color_code)}/{VOTE_WINDOW}): {COLOR_NAMES[final_color]}")
                            break
                            
                if final_color is not None:
                    break
                    
                # Debug penceresi
                if args.debug:
                    debug_frame = cv2.resize(frame, (640, 480))
                    if quad_pts is not None:
                        cv2.drawContours(debug_frame, [quad_pts.astype(int)], -1, (0, 255, 0), 2)
                        
                    # Yazıları ekrana ekleyelim
                    text_y = 30
                    for c_code, ratio in ratios.items():
                        cv2.putText(debug_frame, f"{COLOR_NAMES[c_code]}: {ratio:.2f}", (10, text_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        text_y += 25
                        
                    cv2.putText(debug_frame, f"Voting: {vote_queue}", (10, text_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(debug_frame, f"Sure: {elapsed_time:.1f}/{DETECTION_TIMEOUT_SEC}s", (10, 450), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                
                    cv2.imshow("LODOS IHA DEBUG", debug_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        logger.info("Debug penceresi kullanıcı tarafından kapatıldı.")
                        break
                        
            # Timeout Failsafe Kontrolü
            if elapsed_time >= DETECTION_TIMEOUT_SEC:
                logger.warning(f"Zaman aşımı ({DETECTION_TIMEOUT_SEC}s) doldu! Renk algılanamadı veya kararsız.")
                logger.warning("Failsafe devreye giriyor: Rengi KIRMIZI kabul ediyoruz.")
                final_color = COLOR_RED
                break
                
            time.sleep(0.03) # ~30 FPS
            
    except KeyboardInterrupt:
        logger.info("İHA algılama döngüsü kullanıcı tarafından durduruldu.")
    finally:
        if cap is not None:
            cap.release()
            cv2.destroyAllWindows()
            
    # Sonucu Pixhawk'a yazıyoruz
    if final_color is not None:
        logger.info(f"Final Renk Kodu: {final_color} ({COLOR_NAMES[final_color]})")
        success = write_param(conn, COLOR_PARAM_NAME, final_color, "İHA", args.dry_run)
        if success:
            logger.info("İHA görevi başarıyla tamamlandı. Parametre Pixhawk'a yazıldı.")
        else:
            logger.error("HATA: Parametre Pixhawk'a yazılamadı!")
    else:
        logger.error("Final renk belirlenemedi!")

def run_yki(args):
    """
    YKİ Rolü (Haberleşme Rölesi):
      1. İHA telemetrisine (args.iha_port) bağlanır.
      2. İDA telemetrisine (args.ida_port) bağlanır.
      3. İHA Pixhawk'taki COLOR_PARAM_NAME değerini düzenli aralıklarla sorgular.
      4. Değer 0 ise bekler.
      5. Değer 1, 2 veya 3 olduğunda bu değeri İDA Pixhawk'taki COLOR_PARAM_NAME parametresine yazar.
      6. Yazılan değeri doğrular ve röle işlemini başarıyla sonlandırır.
    """
    logger.info("==================================================")
    logger.info("               YKİ ROLÜ BAŞLATILDI")
    logger.info("==================================================")
    
    # İHA Telemetri Bağlantısı
    iha_conn = connect_mavlink(args.iha_port, args.telem_baud, "YKİ-İHA", args.dry_run)
    logger.info("IHA TELEMETRY CONNECTED")
    
    # İDA Telemetri Bağlantısı
    ida_conn = connect_mavlink(args.ida_port, args.telem_baud, "YKİ-İDA", args.dry_run)
    logger.info("IDA TELEMETRY CONNECTED")
    
    # Parametrelerin her iki tarafta da var olduğunu doğrulayalım
    verify_param_exists(iha_conn, COLOR_PARAM_NAME, "YKİ-İHA", args.dry_run)
    verify_param_exists(ida_conn, COLOR_PARAM_NAME, "YKİ-İDA", args.dry_run)
    
    logger.info("Köprü çalışıyor. İHA'dan renk bilgisi bekleniyor...")
    
    try:
        while True:
            # İHA'dan parametreyi oku
            iha_val = read_param_value(iha_conn, COLOR_PARAM_NAME, "YKİ-İHA", args.dry_run)
            
            # Dry-run veya simülasyonda kolay test için
            if args.dry_run:
                # Dry run'da döngüyü simüle edelim
                logger.info("[DRY-RUN] İHA parametresi 2 (GREEN) olarak simüle edilip İDA'ya aktarılıyor.")
                iha_val = COLOR_GREEN
            
            if iha_val is None:
                logger.warning("İHA'dan parametre okunamadı! Tekrar deneniyor...")
            elif iha_val == COLOR_NOT_READY:
                logger.info("IHA COLOR RESULT NOT READY (0)")
            elif iha_val in [COLOR_RED, COLOR_GREEN, COLOR_BLACK]:
                logger.info(f"İHA Hedef Rengi Tespit Etti: {COLOR_NAMES[iha_val]} ({iha_val})")
                logger.info(f"Renk İDA'ya aktarılıyor...")
                
                # İDA'ya yaz
                success = write_param(ida_conn, COLOR_PARAM_NAME, iha_val, "YKİ-İDA", args.dry_run)
                if success:
                    logger.info("--------------------------------------------------")
                    logger.info("COLOR RELAY SUCCESS")
                    logger.info("IHA -> YKI -> IDA")
                    logger.info(f"TARGET COLOR: {COLOR_NAMES[iha_val]}")
                    logger.info("--------------------------------------------------")
                    break
                else:
                    logger.error("İDA'ya parametre yazılamadı! Yeniden denenecek...")
            else:
                logger.warning(f"Bilinmeyen parametre değeri alındı: {iha_val}")
                
            time.sleep(PARAM_POLL_INTERVAL)
            
    except KeyboardInterrupt:
        logger.info("YKİ röle döngüsü kullanıcı tarafından durduruldu.")

def get_target_color(conn, dry_run=False):
    """
    İDA üzerinde çalışır. Pixhawk'taki renk parametresini sorgular
    ve target_color adını döner ("red", "green", "black", "none").
    """
    val = read_param_value(conn, COLOR_PARAM_NAME, "İDA", dry_run)
    
    if val == COLOR_RED:
        return "red"
    elif val == COLOR_GREEN:
        return "green"
    elif val == COLOR_BLACK:
        return "black"
    else:
        return "none"

def run_parkur3(target_color):
    """
    Mevcut Parkur-3 otonom navigasyonuna hedef rengi besleyen stub fonksiyonu.
    Geliştiriciler kendi Parkur-3 algoritmasını buraya entegre edebilir.
    """
    logger.info(f"[PARKUR3] Otonom Parkur-3 görevi başlıyor. Hedef renk duba: {target_color.upper()}")
    # Burada motor veya dümen komutları üretilmez. Bu kısım araç üzerindeki otonom karar mekanizmasıdır.
    # Örnek logic:
    if target_color == "red":
        logger.info("[PARKUR3] KIRMIZI duba hedeflendi. İlerleniyor...")
    elif target_color == "green":
        logger.info("[PARKUR3] YEŞİL duba hedeflendi. İlerleniyor...")
    elif target_color == "black":
        logger.info("[PARKUR3] SİYAH duba hedeflendi. İlerleniyor...")
    else:
        logger.warning("[PARKUR3] Geçerli bir hedef renk yok! Beklemede kalınıyor.")

def run_ida(args):
    """
    İDA Rolü:
      1. Pixhawk'a lokal MAVLink bağlantısı kurar.
      2. COLOR_PARAM_NAME parametresinin varlığını doğrular.
      3. Değer 1, 2 veya 3 olana kadar bekler.
      4. Değer geldiğinde target_color adını belirler ve run_parkur3'e aktarır.
    """
    logger.info("==================================================")
    logger.info("               İDA ROLÜ BAŞLATILDI")
    logger.info("==================================================")
    
    # Pixhawk Bağlantısı
    conn = connect_mavlink(args.port, args.baud, "İDA", args.dry_run)
    verify_param_exists(conn, COLOR_PARAM_NAME, "İDA", args.dry_run)
    
    logger.info("İDA Renk parametresi bekleniyor...")
    
    try:
        while True:
            target_color = get_target_color(conn, args.dry_run)
            
            if args.dry_run:
                # Dry run testi için simüle renk dönelim
                target_color = "green"
                
            if target_color != "none":
                logger.info(f"✓ İDA Hedef Rengi Okudu: {target_color.upper()}")
                run_parkur3(target_color)
                break
            else:
                logger.info("Hedef renk henüz hazır değil (none). Bekleniyor...")
                
            time.sleep(PARAM_POLL_INTERVAL)
            
    except KeyboardInterrupt:
        logger.info("İDA döngüsü kullanıcı tarafından durduruldu.")

# ==============================================================================
# MAIN / CLI
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="LODOS Albatros İHA-YKİ-İDA Renk Aktarım ve Algılama Sistemi"
    )
    parser.add_argument(
        "--role", type=str, required=True, choices=["iha", "yki", "ida"],
        help="Çalışma rolü: iha, yki veya ida"
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Lokal Pixhawk seri portu (İHA/İDA için varsayılan: /dev/ttyACM0)"
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Lokal Pixhawk seri port baud rate"
    )
    parser.add_argument(
        "--iha-port", type=str, default=IHA_TELEM_PORT,
        help=f"YKİ rolünde İHA telemetri portu (varsayılan: {IHA_TELEM_PORT})"
    )
    parser.add_argument(
        "--ida-port", type=str, default=IDA_TELEM_PORT,
        help=f"YKİ rolünde İDA telemetri portu (varsayılan: {IDA_TELEM_PORT})"
    )
    parser.add_argument(
        "--telem-baud", type=int, default=TELEM_BAUD,
        help=f"Telemetri portu baud rate (varsayılan: {TELEM_BAUD})"
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="İHA rolünde OpenCV debug pencerelerini gösterir"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Gerçek Pixhawk/Kamera bağlantısı olmadan simüle çalışma gerçekleştirir"
    )
    parser.add_argument(
        "--simulate-color", type=str, default=None, choices=["red", "green", "black", "none"],
        help="Kamerasız testler için simüle renk girdisi sağlar"
    )

    args = parser.parse_args()

    # Varsayılan port atamaları (Eğer belirtilmemişse)
    if args.port is None:
        if args.role == "iha":
            args.port = IHA_PIXHAWK_PORT
        elif args.role == "ida":
            args.port = IDA_PIXHAWK_PORT

    # Rollerin başlatılması
    if args.role == "iha":
        run_iha(args)
    elif args.role == "yki":
        run_yki(args)
    elif args.role == "ida":
        run_ida(args)

if __name__ == '__main__':
    main()
