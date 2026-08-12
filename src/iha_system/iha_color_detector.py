#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros — İHA Plaka Renk Algılama Modülü
=================================================
İHA'nın alt kamerasından alınan görüntüde yerdeki plakanın
rengini (Kırmızı, Yeşil, Siyah) tespit eder.

Yöntem:
  1. Kameradan frame alınır.
  2. İlgi alanı (ROI) merkeze kırpılır (plakanın ortalarda olması beklenir).
  3. HSV renk uzayına dönüştürülür.
  4. Kırmızı, Yeşil ve Siyah için tanımlı HSV aralıklarına göre maske oluşturulur.
  5. En büyük piksel sayısına sahip renk seçilir.
  6. Güvenilirlik: Ardışık N frame aynı rengi verene kadar teyit bekler.

Renk Kodları:
  0 = Bilinmeyen / Tespit edilemedi
  1 = Kırmızı
  2 = Yeşil
  3 = Siyah

Kullanım:
  Bu modül doğrudan çalıştırılabilir (test amaçlı) veya
  iha_mavlink_bridge.py tarafından import edilerek kullanılır.

Ortam  : İHA Raspberry Pi 5 (ROS2 gerektirmez)
Yazar  : LODOS Takımı
"""

import time
from typing import Optional, Tuple

import cv2
import numpy as np


# ══════════════════════════════════════════════════════════════════════════════
# Renk Sabitleri
# ══════════════════════════════════════════════════════════════════════════════

COLOR_UNKNOWN = 0
COLOR_RED     = 1
COLOR_GREEN   = 2
COLOR_BLACK   = 3

COLOR_NAMES = {
    COLOR_UNKNOWN: "bilinmiyor",
    COLOR_RED:     "kirmizi",
    COLOR_GREEN:   "yesil",
    COLOR_BLACK:   "siyah",
}

# ══════════════════════════════════════════════════════════════════════════════
# HSV Renk Aralıkları
# ══════════════════════════════════════════════════════════════════════════════
# OpenCV HSV: H=[0,179], S=[0,255], V=[0,255]

# Kırmızı: HSV'de 0° ve 180° civarında iki bölgede bulunur.
RED_LOWER_1 = np.array([0,   100, 80])
RED_UPPER_1 = np.array([10,  255, 255])
RED_LOWER_2 = np.array([160, 100, 80])
RED_UPPER_2 = np.array([179, 255, 255])

# Yeşil
GREEN_LOWER = np.array([35,  80,  60])
GREEN_UPPER = np.array([85,  255, 255])

# Siyah: Düşük Value (parlaklık) değeri ile tespit edilir.
# Saturation düşük, Value düşük → siyah
BLACK_LOWER = np.array([0,   0,   0])
BLACK_UPPER = np.array([179, 100, 60])

# ══════════════════════════════════════════════════════════════════════════════
# Varsayılan Konfigürasyon
# ══════════════════════════════════════════════════════════════════════════════

DEFAULT_CAMERA_ID            = 0          # Kamera cihaz ID'si
DEFAULT_ROI_RATIO            = 0.4        # Merkez ROI oranı (0.4 = %40 merkez kare)
DEFAULT_MIN_PIXEL_RATIO      = 0.05       # Minimum piksel oranı (maskedeki piksel / ROI toplam)
DEFAULT_CONFIRMATION_FRAMES  = 5          # Ardışık aynı renk frame sayısı
DEFAULT_BLUR_KERNEL          = 5          # Gürültü azaltma için Gaussian blur kernel boyutu


# ══════════════════════════════════════════════════════════════════════════════
# ColorDetector Sınıfı
# ══════════════════════════════════════════════════════════════════════════════

class ColorDetector:
    """
    Kamera görüntüsünden plaka rengini algılayan sınıf.

    Kullanım:
        detector = ColorDetector(camera_id=0)
        detector.start()

        while True:
            color_code, color_name, confirmed = detector.detect()
            if confirmed:
                print(f"Tespit edilen renk: {color_name}")
                break

        detector.stop()
    """

    def __init__(
        self,
        camera_id: int = DEFAULT_CAMERA_ID,
        roi_ratio: float = DEFAULT_ROI_RATIO,
        min_pixel_ratio: float = DEFAULT_MIN_PIXEL_RATIO,
        confirmation_frames: int = DEFAULT_CONFIRMATION_FRAMES,
        blur_kernel: int = DEFAULT_BLUR_KERNEL,
    ):
        self.camera_id = camera_id
        self.roi_ratio = roi_ratio
        self.min_pixel_ratio = min_pixel_ratio
        self.confirmation_frames = confirmation_frames
        self.blur_kernel = blur_kernel

        self._cap: Optional[cv2.VideoCapture] = None
        self._consecutive_count = 0
        self._last_detected_color = COLOR_UNKNOWN
        self._confirmed_color = COLOR_UNKNOWN

    # ──────────────────────────────────────────────────────────────────────
    # Kamera Yönetimi
    # ──────────────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Kamerayı başlatır. Başarılı ise True döner."""
        self._cap = cv2.VideoCapture(self.camera_id)

        if not self._cap.isOpened():
            print(f"[HATA] Kamera açılamadı: ID={self.camera_id}")
            return False

        # Çözünürlük ayarı (640x480 yeterli)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print(f"[BİLGİ] Kamera başlatıldı: ID={self.camera_id}")
        return True

    def stop(self):
        """Kamerayı serbest bırakır."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            print("[BİLGİ] Kamera kapatıldı.")

    def is_opened(self) -> bool:
        """Kameranın açık olup olmadığını döner."""
        return self._cap is not None and self._cap.isOpened()

    # ──────────────────────────────────────────────────────────────────────
    # Ana Algılama Fonksiyonu
    # ──────────────────────────────────────────────────────────────────────

    def detect(self) -> Tuple[int, str, bool]:
        """
        Tek bir frame alıp renk algılama yapar.

        Returns:
            (color_code, color_name, confirmed)
            - color_code: 0=Bilinmiyor, 1=Kırmızı, 2=Yeşil, 3=Siyah
            - color_name: Renk adı (string)
            - confirmed: Ardışık N frame aynı renk verildi mi?
        """
        if not self.is_opened():
            return COLOR_UNKNOWN, COLOR_NAMES[COLOR_UNKNOWN], False

        ret, frame = self._cap.read()
        if not ret or frame is None:
            return COLOR_UNKNOWN, COLOR_NAMES[COLOR_UNKNOWN], False

        # ── ROI Kırpma ──
        roi = self._extract_roi(frame)

        # ── Ön İşleme ──
        blurred = cv2.GaussianBlur(roi, (self.blur_kernel, self.blur_kernel), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # ── Renk Maskeleri ──
        red_mask   = self._create_red_mask(hsv)
        green_mask = self._create_green_mask(hsv)
        black_mask = self._create_black_mask(hsv)

        # ── Piksel Sayımı ──
        total_pixels = roi.shape[0] * roi.shape[1]
        red_pixels   = cv2.countNonZero(red_mask)
        green_pixels = cv2.countNonZero(green_mask)
        black_pixels = cv2.countNonZero(black_mask)

        min_pixels = int(total_pixels * self.min_pixel_ratio)

        # ── En Baskın Rengi Seç ──
        candidates = {
            COLOR_RED:   red_pixels,
            COLOR_GREEN: green_pixels,
            COLOR_BLACK: black_pixels,
        }

        # Minimum piksel eşiğini geçen en büyük alanı bul
        best_color = COLOR_UNKNOWN
        best_count = 0

        for color_code, pixel_count in candidates.items():
            if pixel_count > min_pixels and pixel_count > best_count:
                best_color = color_code
                best_count = pixel_count

        # ── Ardışık Frame Teyidi ──
        if best_color == self._last_detected_color and best_color != COLOR_UNKNOWN:
            self._consecutive_count += 1
        else:
            self._consecutive_count = 1 if best_color != COLOR_UNKNOWN else 0
            self._last_detected_color = best_color

        confirmed = self._consecutive_count >= self.confirmation_frames

        if confirmed:
            self._confirmed_color = best_color

        return best_color, COLOR_NAMES[best_color], confirmed

    def get_confirmed_color(self) -> Tuple[int, str]:
        """Son onaylanmış renk kodunu ve adını döner."""
        return self._confirmed_color, COLOR_NAMES[self._confirmed_color]

    # ──────────────────────────────────────────────────────────────────────
    # Yardımcı Fonksiyonlar
    # ──────────────────────────────────────────────────────────────────────

    def _extract_roi(self, frame: np.ndarray) -> np.ndarray:
        """Frame'in merkezinden ROI kırpar."""
        h, w = frame.shape[:2]
        roi_h = int(h * self.roi_ratio)
        roi_w = int(w * self.roi_ratio)
        y_start = (h - roi_h) // 2
        x_start = (w - roi_w) // 2
        return frame[y_start:y_start + roi_h, x_start:x_start + roi_w]

    @staticmethod
    def _create_red_mask(hsv: np.ndarray) -> np.ndarray:
        """Kırmızı renk için HSV maskesi oluşturur (iki aralık birleşimi)."""
        mask1 = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
        mask2 = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
        return cv2.bitwise_or(mask1, mask2)

    @staticmethod
    def _create_green_mask(hsv: np.ndarray) -> np.ndarray:
        """Yeşil renk için HSV maskesi oluşturur."""
        return cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

    @staticmethod
    def _create_black_mask(hsv: np.ndarray) -> np.ndarray:
        """Siyah renk için HSV maskesi oluşturur."""
        return cv2.inRange(hsv, BLACK_LOWER, BLACK_UPPER)


# ══════════════════════════════════════════════════════════════════════════════
# Doğrudan Çalıştırma (Test Modu)
# ══════════════════════════════════════════════════════════════════════════════

def main():
    """
    Test modu: Kamerayı açar, sürekli renk algılama yapar ve
    onaylanan rengi ekrana basar.
    """
    print("=" * 60)
    print("  LODOS İHA — Plaka Renk Algılama Test Modu")
    print("=" * 60)

    detector = ColorDetector(
        camera_id=0,
        confirmation_frames=5,
    )

    if not detector.start():
        print("[HATA] Kamera başlatılamadı. Çıkılıyor.")
        return

    print("[BİLGİ] Renk algılama başlatıldı. Çıkmak için Ctrl+C.")
    print()

    try:
        while True:
            color_code, color_name, confirmed = detector.detect()

            if color_code != COLOR_UNKNOWN:
                status = "✓ ONAYLANDI" if confirmed else "⏳ teyit bekleniyor"
                print(
                    f"\r[ALGILAMA] Renk: {color_name:10s} | "
                    f"Kod: {color_code} | "
                    f"Durum: {status}   ",
                    end="",
                    flush=True,
                )

            if confirmed:
                print()
                print()
                print(f"{'=' * 60}")
                print(f"  ✓ HEDEF RENK TESPİT EDİLDİ: {color_name.upper()}")
                print(f"  ✓ Renk Kodu: {color_code}")
                print(f"{'=' * 60}")
                break

            time.sleep(0.05)  # ~20 FPS

    except KeyboardInterrupt:
        print()
        print("[BİLGİ] Kullanıcı tarafından durduruldu.")

    finally:
        detector.stop()


if __name__ == '__main__':
    main()
