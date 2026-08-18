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

Mimaride ROS/ROS2, MAVROS veya doğrudan İHA-İDA veri bağlantısı KULLANILMAZ.
Haberleşme akışı: İHA -> YKİ -> İDA (MAVLink parametre aktarımı) şeklindedir.

Yazar: LODOS Takımı
"""

import os
import sys
import time
import argparse
import logging
from collections import deque
import numpy as np
import cv2

# Import pymavlink (if not in dry-run/simulation or if available)
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

# ==============================================================================
# 22. CONFIG BÖLÜMÜ
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
CAMERA_BACKEND = "auto"  # "auto", "v4l2", "libcamera"

# Dörtgen Plaka / Kare Filtre Ayarları (Section 6)
BLUR_KERNEL = 5
CANNY_THRESHOLD1 = 50
CANNY_THRESHOLD2 = 150
MIN_CONTOUR_AREA = 1000       # Görüntüdeki minimum plaka alanı (piksel)
MAX_CONTOUR_AREA = 200000     # Görüntüdeki maksimum plaka alanı (piksel)
SQUARE_RATIO_MIN = 0.80       # minAreaRect (min(w,h)/max(w,h)) karelik oranı
SQUARE_MIN_ANGLE = 70.0       # Köşe iç açısı minimum (derece)
SQUARE_MAX_ANGLE = 110.0      # Köşe iç açısı maksimum (derece)
SQUARE_SIDE_RATIO_MIN = 0.65  # Kenar uzunlukları minimum oranı (min_side/max_side)
ROI_CENTER_RATIO = 0.75       # ROI'nin merkezdeki yüzde kaçlık bölümü kullanılacak (%75)

# HSV Renk Eşikleri (H: 0-179, S: 0-255, V: 0-255) - (Section 7)
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

# Renk Karar, Voting ve Zamanlama Ayarları (Section 8 & 9)
VOTE_WINDOW = 10
VOTE_REQUIRED = 7
DETECTION_TIMEOUT_SEC = 5.0

# Zamanlama ve Yeniden Bağlanma Ayarları
PARAM_POLL_INTERVAL = 0.5
RECONNECT_INTERVAL_SEC = 2.0

# Log Dosyası Yolu
LOG_FILE_PATH = "lodos_color_system.log"

# ==============================================================================
# 21. LOGGING VE YARDIMCI FONKSİYONLAR
# ==============================================================================

def setup_logger():
    """Dosyaya ve konsola yazan log sistemini hazırlar."""
    logger = logging.getLogger("LodosColorSystem")
    logger.setLevel(logging.DEBUG)
    
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    
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
# 11 & 20. MAVLINK YÖNETİMİ VE BAĞLANTI KOPMASI
# ==============================================================================

def connect_mavlink(port, baud, label, dry_run=False):
    """
    Belirtilen porta MAVLink bağlantısı kurar ve heartbeat bekler.
    Bağlantı koparsa yeniden deneme yapar.
    """
    if dry_run:
        logger.info(f"[{label}] [DRY-RUN] Simüle bağlantı kuruldu: {port}")
        return None

    if mavutil is None:
        logger.error(f"[{label}] pymavlink kütüphanesi yüklü değil! 'pip install pymavlink' gereklidir.")
        sys.exit(1)

    logger.info(f"[{label}] Bağlantı başlatılıyor: {port} @ {baud} baud...")
    while True:
        try:
            conn = mavutil.mavlink_connection(port, baud=baud)
            logger.info(f"[{label}] Heartbeat bekleniyor...")
            heartbeat = conn.wait_heartbeat(timeout=10)
            if heartbeat:
                logger.info(f"[{label}] Heartbeat ALINDI. System ID: {conn.target_system}, Component ID: {conn.target_component}")
                return conn
            else:
                logger.warning(f"[{label}] Heartbeat zaman aşımı! Yeniden deneniyor...")
        except Exception as e:
            logger.error(f"[{label}] Connection lost / error: {e}. Reconnecting in {RECONNECT_INTERVAL_SEC}s...")
        time.sleep(RECONNECT_INTERVAL_SEC)

def verify_param_exists(conn, param_name, label, dry_run=False):
    """
    4. Seçilen parametrenin Pixhawk üzerinde gerçekten var olup olmadığını kontrol eder.
    Parametre yoksa ERROR: COLOR PARAMETER NOT FOUND yazıp güvenli şekilde durur.
    Uçuşu etkileyen başka bir parametreye otomatik geçiş YAPMAZ.
    """
    if dry_run:
        logger.info(f"[{label}] [DRY-RUN] Parametre '{param_name}' mevcut kabul edildi.")
        return True

    logger.info(f"[{label}] Parametre '{param_name}' varlığı Pixhawk üzerinde kontrol ediliyor...")
    
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component,
        param_name.encode('utf-8'),
        -1
    )
    
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
                
    logger.error(f"[{label}] ERROR: COLOR PARAMETER NOT FOUND ('{param_name}'). Lütfen ArduPilot parametresini kontrol edin veya SCR_ENABLE=1 olduğunu doğrulayın.")
    sys.exit(1)

def write_param(conn, param_name, param_value, label, dry_run=False):
    """
    10. PARAM_SET mesajı göndererek parametreyi yazar ve PARAM_VALUE ile doğrular (maksimum 3 deneme).
    Sonsuz döngü oluşturmaz.
    """
    if dry_run:
        logger.info(f"[{label}] [DRY-RUN] Parametre '{param_name}' değeri {param_value} olarak yazıldı.")
        return True

    for attempt in range(1, 4):
        logger.info(f"[{label}] Parametre Yazma Denemesi {attempt}/3: {param_name} = {param_value}")
        
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            param_name.encode('utf-8'),
            float(param_value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        
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
                        logger.info(f"[{label}] ✓ PARAM_VALUE Alındı ve Doğrulandı: {param_name} = {int(msg.param_value)}")
                        return True
                    else:
                        logger.warning(f"[{label}] Değer henüz güncellenmedi: beklenen {param_value}, alınan {msg.param_value}")
                        
        logger.warning(f"[{label}] PARAM_VALUE doğrulama zaman aşımı. Yeniden deneniyor...")
        time.sleep(0.5)
        
    logger.error(f"[{label}] HATA: Parametre {param_name} yazılamadı veya doğrulanamadı!")
    return False

def read_param_value(conn, param_name, label, dry_run=False):
    """Pixhawk'tan parametre değerini okur."""
    if dry_run:
        return COLOR_NOT_READY

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
# 6 & 7. OPENCV DÖRTGEN PLAKA TESPİTİ VE RENK ANALİZİ
# ==============================================================================

def calculate_angle(p1, p2, p3):
    """p2 köşe noktası olmak üzere p1-p2-p3 noktaları arasındaki iç açıyı hesaplar."""
    v1 = p1.astype(float) - p2.astype(float)
    v2 = p3.astype(float) - p2.astype(float)
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)
    if norm1 < 1e-6 or norm2 < 1e-6:
        return 0.0
    cosine = np.dot(v1, v2) / (norm1 * norm2)
    cosine = np.clip(cosine, -1.0, 1.0)
    return np.degrees(np.arccos(cosine))

def check_quad_angles(pts):
    """4 noktalı poligonun iç açılarının [SQUARE_MIN_ANGLE, SQUARE_MAX_ANGLE] aralığında olduğunu doğrular."""
    for i in range(4):
        p1 = pts[i - 1]
        p2 = pts[i]
        p3 = pts[(i + 1) % 4]
        angle = calculate_angle(p1, p2, p3)
        if not (SQUARE_MIN_ANGLE <= angle <= SQUARE_MAX_ANGLE):
            return False
    return True

def check_side_ratios(pts):
    """4 kenar uzunluğunu kontrol eder."""
    sides = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        dist = np.linalg.norm(p1.astype(float) - p2.astype(float))
        sides.append(dist)
    min_side = min(sides)
    max_side = max(sides)
    if max_side < 1e-6:
        return False
    return (min_side / max_side) >= SQUARE_SIDE_RATIO_MIN

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
    """6. Dörtgen noktaları üzerinden perspective transform uygular."""
    rect = order_points(pts)
    (tl, tr, br, bl) = rect

    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    if maxWidth < 1 or maxHeight < 1:
        raise ValueError("Geçersiz ROI boyutu")

    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")

    M = cv2.getPerspectiveTransform(rect, dst)
    return cv2.warpPerspective(image, M, (maxWidth, maxHeight))

def detect_square_plate(frame):
    """
    6. Dörtgen Plaka Tespiti İşlem Sırası:
       1. Frame al, 640x480 boyutlandır.
       2. Gaussian Blur uygula.
       3. Gri görüntü oluştur.
       4. Canny Edge Detection uygula.
       5. Contour'ları bul.
       6. Contour approximation (approxPolyDP).
       7. 4 köşeli ve convex olanları seç.
       8. Min/Max area, aspect ratio, iç açılar ve kenar oranlarını filtrele.
       9. En uygun dörtgeni seç, perspective transform yap ve ROI döndür.
    """
    resized = cv2.resize(frame, (640, 480))
    blurred = cv2.GaussianBlur(resized, (BLUR_KERNEL, BLUR_KERNEL), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edged = cv2.Canny(gray, CANNY_THRESHOLD1, CANNY_THRESHOLD2)
    contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_quad = None
    max_area = 0.0

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
            continue
            
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue
            
        pts = approx.reshape(4, 2)
        
        rect = cv2.minAreaRect(approx)
        (w, h) = rect[1]
        if w <= 0 or h <= 0:
            continue
        square_ratio = min(w, h) / max(w, h)
        if square_ratio < SQUARE_RATIO_MIN:
            continue
            
        if not check_quad_angles(pts):
            continue
            
        if not check_side_ratios(pts):
            continue
            
        if area > max_area:
            max_area = area
            best_quad = pts
                    
    if best_quad is not None:
        try:
            warped = get_perspective_transform(resized, best_quad)
            return warped, best_quad
        except Exception as e:
            logger.warning(f"Perspective transform hatası: {e}")
            
    return None, None

def analyze_color(roi):
    """
    7. Renk Tespiti (HSV):
       - cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
       - Kırmızı için çift maske (RED_LOW_1/HIGH_1 OR RED_LOW_2/HIGH_2)
       - Yeşil için GREEN_LOW/HIGH
       - Siyah için düşük V parlaklığı (BLACK_LOW/HIGH)
       - ROI merkez %70-80 bölümü kullanılır.
       - Morphology OPEN & CLOSE uygulanır.
       - Maske piksel oranları hesaplanır.
    """
    h, w = roi.shape[:2]
    margin_h = int(h * (1.0 - ROI_CENTER_RATIO) / 2.0)
    margin_w = int(w * (1.0 - ROI_CENTER_RATIO) / 2.0)
    
    if margin_h < 1: margin_h = 0
    if margin_w < 1: margin_w = 0
    
    roi_center = roi[margin_h:h-margin_h, margin_w:w-margin_w]
    roi_pixel_count = roi_center.shape[0] * roi_center.shape[1]
    
    if roi_pixel_count <= 0:
        return COLOR_NOT_READY, {COLOR_RED: 0.0, COLOR_GREEN: 0.0, COLOR_BLACK: 0.0}
        
    hsv = cv2.cvtColor(roi_center, cv2.COLOR_BGR2HSV)
    
    # 1. Kırmızı (Çift aralık)
    mask_red1 = cv2.inRange(hsv, RED_LOW_1, RED_HIGH_1)
    mask_red2 = cv2.inRange(hsv, RED_LOW_2, RED_HIGH_2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    
    # 2. Yeşil
    mask_green = cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)
    
    # 3. Siyah
    mask_black = cv2.inRange(hsv, BLACK_LOW, BLACK_HIGH)
    
    # Morfoloji Open & Close
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    
    mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)
    mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel)
    
    # Piksel Oranları (mask_pixel_count / roi_pixel_count)
    red_ratio = cv2.countNonZero(mask_red) / roi_pixel_count
    green_ratio = cv2.countNonZero(mask_green) / roi_pixel_count
    black_ratio = cv2.countNonZero(mask_black) / roi_pixel_count
    
    ratios = {
        COLOR_RED: red_ratio,
        COLOR_GREEN: green_ratio,
        COLOR_BLACK: black_ratio
    }
    
    best_color = COLOR_NOT_READY
    max_ratio = 0.0
    for color, ratio in ratios.items():
        if ratio > 0.05 and ratio > max_ratio:
            max_ratio = ratio
            best_color = color
            
    return best_color, ratios

# ==============================================================================
# 3. İHA ROLÜ (İHA RASPBERRY PI)
# ==============================================================================

def open_camera(backend="auto", camera_id=0):
    """17. Dayanıklı kamera açılışı (OpenCV VideoCapture / V4L2 / libcamera)."""
    logger.info(f"Kamera açılıyor (Backend: {backend}, ID: {camera_id})...")
    if backend == "v4l2":
        cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(camera_id)
        
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return cap
    return None

def run_iha(args):
    """
    İHA Görev Akışı (Section 24 & Prompt Şartları):
    1. Pixhawk'a bağlan.
    2. COLOR_PARAM_NAME var mı kontrol et.
    3. 5. Eski renk bilgisini engelle: COLOR_PARAM = 0 yap.
    4. Kamerayı aç, 640x480 işle.
    5. Dörtgen ara, ROI oluştur, HSV renk analizi yap.
    6. 8. Son 10 frame oylama yap (VOTE_WINDOW=10, VOTE_REQUIRED=7).
    7. 9. Belirli süre (5.0s) içinde geçerli sonuç çıkmazsa TIMEOUT -> KIRMIZI (1) kabul et.
    8. 10. Parametreyi tek seferlik (1, 2 veya 3) yap, PARAM_VALUE ile doğrula.
    """
    logger.info("==================================================")
    logger.info("               İHA ROLÜ BAŞLATILDI")
    logger.info("==================================================")
    
    # 11. MAVLink bağlantısı
    conn = connect_mavlink(args.port, args.baud, "İHA", args.dry_run)
    # 4. Parametre varlık kontrolü
    verify_param_exists(conn, COLOR_PARAM_NAME, "İHA", args.dry_run)
    
    # 5. Eski parametre değerini sıfırla (0 = NOT READY)
    logger.info(f"5. Eski renk bilgisi temizleniyor: {COLOR_PARAM_NAME} = 0")
    write_param(conn, COLOR_PARAM_NAME, COLOR_NOT_READY, "İHA", args.dry_run)
    
    # Kamera Hazırlığı
    cap = None
    if not args.dry_run and args.simulate_color is None:
        cap = open_camera(CAMERA_BACKEND, CAMERA_ID)
        if cap is None:
            logger.error("Kamera açılamadı! Program durduruluyor.")
            sys.exit(1)
            
    start_time = time.monotonic()
    vote_queue = deque(maxlen=VOTE_WINDOW)  # 8. 10-frame voting penceresi
    final_color = None
    
    logger.info(f"Renk algılama başladı. Timeout: {DETECTION_TIMEOUT_SEC}s, Voting: {VOTE_REQUIRED}/{VOTE_WINDOW} frame...")
    
    try:
        while True:
            now = time.monotonic()
            elapsed_total = now - start_time
            
            # --- 19. Simülasyon / Dry-Run Modu ---
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

                plate_detected = (detected != COLOR_NOT_READY)
                quad_pts = np.array([[100, 100], [300, 100], [300, 300], [100, 300]]) if plate_detected else None
                ratios = {COLOR_RED: 0.0, COLOR_GREEN: 0.0, COLOR_BLACK: 0.0}
                if plate_detected:
                    ratios[detected] = 1.0
                frame = None
                time.sleep(0.05)
            
            # --- Gerçek Görüntü İşleme Modu ---
            elif cap is not None:
                ret, frame = cap.read()
                if not ret or frame is None:
                    logger.warning("Kameradan frame alınamadı!")
                    time.sleep(0.05)
                    continue
                    
                roi, quad_pts = detect_square_plate(frame)
                plate_detected = (roi is not None)
                detected = COLOR_NOT_READY
                ratios = {COLOR_RED: 0.0, COLOR_GREEN: 0.0, COLOR_BLACK: 0.0}
                
                if plate_detected:
                    detected, ratios = analyze_color(roi)
            else:
                plate_detected = False
                detected = COLOR_NOT_READY
                ratios = {COLOR_RED: 0.0, COLOR_GREEN: 0.0, COLOR_BLACK: 0.0}
                frame = None
                time.sleep(0.05)

            # --- 8. 10-FRAME VOTING SİSTEMİ ---
            vote_queue.append(detected)
            
            # Oy sayılarını hesapla
            red_votes = vote_queue.count(COLOR_RED)
            green_votes = vote_queue.count(COLOR_GREEN)
            black_votes = vote_queue.count(COLOR_BLACK)
            
            # En az 7/10 oy şartı
            if red_votes >= VOTE_REQUIRED:
                final_color = COLOR_RED
                logger.info(f"✓ VOTING PASSED: RED ({red_votes}/{VOTE_WINDOW})")
                break
            elif green_votes >= VOTE_REQUIRED:
                final_color = COLOR_GREEN
                logger.info(f"✓ VOTING PASSED: GREEN ({green_votes}/{VOTE_WINDOW})")
                break
            elif black_votes >= VOTE_REQUIRED:
                final_color = COLOR_BLACK
                logger.info(f"✓ VOTING PASSED: BLACK ({black_votes}/{VOTE_WINDOW})")
                break

            # --- 18. DEBUG EKRANI (Lokal İHA ekranı) ---
            if args.debug and frame is not None:
                debug_frame = cv2.resize(frame, (640, 480))
                if quad_pts is not None:
                    cv2.drawContours(debug_frame, [quad_pts.astype(int)], -1, (0, 255, 0), 2)
                    
                text_y = 25
                cv2.putText(debug_frame, f"PLATE: {'DETECTED' if plate_detected else 'NOT DETECTED'}", (10, text_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2); text_y += 20
                cv2.putText(debug_frame, f"CURR COLOR: {COLOR_NAMES[detected]}", (10, text_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1); text_y += 20
                cv2.putText(debug_frame, f"VOTES R:{red_votes} G:{green_votes} B:{black_votes} (REQ:{VOTE_REQUIRED}/{VOTE_WINDOW})", (10, text_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2); text_y += 25

                for c_code, ratio in ratios.items():
                    cv2.putText(debug_frame, f"{COLOR_NAMES[c_code]} RATIO: {ratio:.2f}", (10, text_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1); text_y += 20

                time_rem = max(0.0, DETECTION_TIMEOUT_SEC - elapsed_total)
                cv2.putText(debug_frame, f"TIMEOUT REMAINING: {time_rem:.1f} s", (10, 465), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

                cv2.imshow("LODOS IHA DEBUG (LOCAL)", debug_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # --- 9. HİÇBİR RENK ALGILANAMAZSA TIMEOUT -> KIRMIZI (FAILSAFE) ---
            if elapsed_total >= DETECTION_TIMEOUT_SEC:
                logger.warning(f"9. DETECTION TIMEOUT ({DETECTION_TIMEOUT_SEC}s) EXPIRED! Valid color couldn't be confirmed.")
                logger.warning("Failsafe active: Setting final color to RED (1).")
                final_color = COLOR_RED
                break
                
    except KeyboardInterrupt:
        logger.info("İHA algılama kullanıcı tarafından kesildi.")
    finally:
        if cap is not None:
            cap.release()
            cv2.destroyAllWindows()
            
    # 10. Final rengi Pixhawk parametresine TEK SEFERLİK yaz
    if final_color is not None:
        logger.info(f"FINAL COLOR DETERMINED: {COLOR_NAMES[final_color]} ({final_color})")
        success = write_param(conn, COLOR_PARAM_NAME, final_color, "İHA", args.dry_run)
        if success:
            logger.info("İHA görevi tamamlandı. Parametre Pixhawk'a başarıyla işlendi.")
        else:
            logger.error("HATA: İHA parametresi Pixhawk'a yazılamadı!")

# ==============================================================================
# 12, 13, 14, 15. YKİ ROLÜ (YER KONTROL İSTASYONU RÖLESİ)
# ==============================================================================

def run_yki(args):
    """
    YKİ Rolü (Haberleşme Rölesi):
    - İHA ve İDA için iki ayrı telemetri hattına bağlanır.
    - İHA Pixhawk'taki COLOR_PARAM_NAME parametresini sorgular.
    - Değer 0 ise: IHA COLOR RESULT NOT READY yazar ve bekler.
    - Değer 1, 2 veya 3 olunca İDA'nın COLOR_PARAM_NAME parametresine yazar.
    - PARAM_VALUE ile doğrular ve COLOR RELAY SUCCESS basarak tek seferlik işlemi tamamlar.
    - KESİNLİKLE genel MAVLink bridge (paket forwarding) YAPMAZ.
    """
    logger.info("==================================================")
    logger.info("               YKİ ROLÜ BAŞLATILDI")
    logger.info("==================================================")
    
    # 12. İki ayrı telemetri portu bağlantısı
    iha_conn = connect_mavlink(args.iha_port, args.telem_baud, "YKİ-İHA", args.dry_run)
    logger.info("IHA TELEMETRY CONNECTED")
    
    ida_conn = connect_mavlink(args.ida_port, args.telem_baud, "YKİ-İDA", args.dry_run)
    logger.info("IDA TELEMETRY CONNECTED")
    
    verify_param_exists(iha_conn, COLOR_PARAM_NAME, "YKİ-İHA", args.dry_run)
    verify_param_exists(ida_conn, COLOR_PARAM_NAME, "YKİ-İDA", args.dry_run)
    
    logger.info("YKİ Haberleşme Rölesi aktif. İHA parametresi bekleniyor...")
    
    try:
        while True:
            # 13. İHA parametresini oku
            iha_val = read_param_value(iha_conn, COLOR_PARAM_NAME, "YKİ-İHA", args.dry_run)
            
            if args.dry_run:
                logger.info("[DRY-RUN] İHA parametresi 2 (GREEN) simüle edilerek İDA'ya aktarılıyor.")
                iha_val = COLOR_GREEN
            
            if iha_val is None:
                logger.warning("[YKI] Waiting for IHA color...")
            elif iha_val == COLOR_NOT_READY:
                # 13. 0 durumu
                logger.info("IHA COLOR RESULT NOT READY")
            elif iha_val in [COLOR_RED, COLOR_GREEN, COLOR_BLACK]:
                logger.info(f"IHA TARGET COLOR RECEIVED = {COLOR_NAMES[iha_val]} ({iha_val})")
                
                # 14. İDA Pixhawk parametresine aktar
                success = write_param(ida_conn, COLOR_PARAM_NAME, iha_val, "YKİ-İDA", args.dry_run)
                if success:
                    # 14. Başarılı terminal çıktısı
                    logger.info("==================================================")
                    logger.info("COLOR RELAY SUCCESS")
                    logger.info("IHA -> YKI -> IDA")
                    logger.info(f"TARGET COLOR: {COLOR_NAMES[iha_val]}")
                    logger.info("==================================================")
                    break
                else:
                    logger.error("İDA parametre yazımı başarısız, tekrar deneniyor...")
            else:
                logger.warning(f"Bilinmeyen parametre değeri alındı: {iha_val}")
                
            time.sleep(PARAM_POLL_INTERVAL)
            
    except KeyboardInterrupt:
        logger.info("YKİ röle görevi kullanıcı tarafından durduruldu.")

# ==============================================================================
# 16. İDA ROLÜ (İNSANSIZ DENİZ ARACI RASPBERRY PI)
# ==============================================================================

def get_target_color(conn, dry_run=False):
    """
    16. İDA Pixhawk'taki COLOR_PARAM_NAME parametresini okur
    ve string 'red', 'green', 'black' veya 'none' döner.
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
    16. Mevcut Parkur-3 hedef seçme ve otonom algoritmasını tetikleyen bağlantı fonksiyonu.
    """
    logger.info(f"[PARKUR3] Otonom Parkur-3 görevi başlıyor. Target Color: {target_color.upper()}")
    if target_color == "red":
        logger.info("[PARKUR3] Target = RED -> Kırmızı duba angajmanı başlatıldı.")
    elif target_color == "green":
        logger.info("[PARKUR3] Target = GREEN -> Yeşil duba angajmanı başlatıldı.")
    elif target_color == "black":
        logger.info("[PARKUR3] Target = BLACK -> Siyah duba angajmanı başlatıldı.")
    else:
        logger.warning("[PARKUR3] Geçerli renk yok (none). Bekleniyor...")

def run_ida(args):
    """
    İDA Görev Akışı:
    - Lokal Pixhawk'a bağlanır.
    - Parametre varlığını doğrular.
    - Değer 1, 2 veya 3 olana kadar bekler.
    - Okunan hedef rengi run_parkur3(target_color) fonksiyonuna aktarır.
    """
    logger.info("==================================================")
    logger.info("               İDA ROLÜ BAŞLATILDI")
    logger.info("==================================================")
    
    conn = connect_mavlink(args.port, args.baud, "İDA", args.dry_run)
    verify_param_exists(conn, COLOR_PARAM_NAME, "İDA", args.dry_run)
    
    logger.info("İDA Pixhawk renk parametresi bekleniyor...")
    
    try:
        while True:
            target_color = get_target_color(conn, args.dry_run)
            
            if args.dry_run:
                target_color = "green"
                
            if target_color != "none":
                logger.info(f"✓ İDA TARGET COLOR RECEIVED: {target_color.upper()}")
                run_parkur3(target_color)
                break
            else:
                logger.info("Waiting for target color parameter from Pixhawk...")
                
            time.sleep(PARAM_POLL_INTERVAL)
            
    except KeyboardInterrupt:
        logger.info("İDA görevi kullanıcı tarafından kesildi.")

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
        help="Gerçek Pixhawk/Kamera olmadan simüle çalışma gerçekleştirir"
    )
    parser.add_argument(
        "--simulate-color", type=str, default=None, choices=["red", "green", "black", "none"],
        help="Kamerasız testler için simüle renk girdisi sağlar"
    )

    args = parser.parse_args()

    if args.port is None:
        if args.role == "iha":
            args.port = IHA_PIXHAWK_PORT
        elif args.role == "ida":
            args.port = IDA_PIXHAWK_PORT

    if args.role == "iha":
        run_iha(args)
    elif args.role == "yki":
        run_yki(args)
    elif args.role == "ida":
        run_ida(args)

if __name__ == '__main__':
    main()
