#!/usr/bin/env python3
"""
safety_states.py — LODOS Albatros Güvenlik Durumları Modülü
============================================================
Bu modül, Albatros İDA'nın güvenlik durumlarını tanımlayan
enum'lar, veri yapıları ve değerlendirici sınıf içerir.

Bu bir ROS2 node'u DEĞİLDİR. Diğer node'lar tarafından
import edilerek kullanılacak saf Python modülüdür.

Kullanım Örneği:
    from albatros_system.safety_states import (
        SafetyLevel,
        InterferenceLevel,
        SafetyState,
        SafetyEvaluator,
    )

    evaluator = SafetyEvaluator()
    level = evaluator.evaluate_interference(local_noise=85, remote_noise=90)

Yazar : LODOS Takımı
Araç  : Albatros İDA
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional
import time


# =============================================================================
# Güvenlik Seviyesi Enum'u
# =============================================================================

class SafetyLevel(Enum):
    """
    Genel güvenlik seviyesi.

    Seviyeler (düşükten yükseğe):
        NORMAL    → Tüm sistemler normal çalışıyor.
        ADVISORY  → Bilgilendirme düzeyinde uyarı; müdahale gerekmez.
        WARNING   → Dikkat gerektiren durum; operatör bilgilendirilmeli.
        CRITICAL  → Kritik durum; otomatik önlem alınabilir.
        EMERGENCY → Acil durum; görev durdurulmalı.
    """
    NORMAL    = 'NORMAL'
    ADVISORY  = 'ADVISORY'
    WARNING   = 'WARNING'
    CRITICAL  = 'CRITICAL'
    EMERGENCY = 'EMERGENCY'


# =============================================================================
# Frekans Girişim Seviyesi Enum'u
# =============================================================================

class InterferenceLevel(Enum):
    """
    Frekans girişim (interference) seviyesi.

    Noise değerlerine göre belirlenir:
        NONE     → Girişim yok; gürültü normal seviyede.
        LOW      → Düşük girişim; hafif gürültü artışı.
        MEDIUM   → Orta düzey girişim; aynı bantta olası cihaz.
        HIGH     → Yüksek girişim; aynı frekansta aktif cihaz.
        CRITICAL → Kritik girişim; haberleşme tehlikede.
    """
    NONE     = 'NONE'
    LOW      = 'LOW'
    MEDIUM   = 'MEDIUM'
    HIGH     = 'HIGH'
    CRITICAL = 'CRITICAL'


# =============================================================================
# Güvenlik Durumu Veri Yapısı
# =============================================================================

@dataclass
class SafetyState:
    """
    Tüm güvenlik bilgilerini bir arada tutan veri yapısı.

    Attributes:
        safety_level:        Genel güvenlik seviyesi.
        interference_level:  Frekans girişim seviyesi.
        interference_detected: Girişim tespit edildi mi.
        telemetry_connected: Telemetri modülü bağlı mı.
        local_noise:         Yerel gürültü seviyesi (dBm).
        remote_noise:        Uzak gürültü seviyesi (dBm).
        local_rssi:          Yerel sinyal gücü (dBm).
        remote_rssi:         Uzak sinyal gücü (dBm).
        noise_floor_avg:     Ortalama gürültü tabanı (dBm).
        description:         Duruma ilişkin açıklama metni.
        timestamp:           Durum oluşturulma zamanı (epoch).
    """
    safety_level:         SafetyLevel        = SafetyLevel.NORMAL
    interference_level:   InterferenceLevel   = InterferenceLevel.NONE
    interference_detected: bool               = False
    telemetry_connected:  bool                = False

    local_noise:          int                 = 0
    remote_noise:         int                 = 0
    local_rssi:           int                 = 0
    remote_rssi:          int                 = 0
    noise_floor_avg:      float               = 0.0

    description:          str                 = ''
    timestamp:            float               = field(default_factory=time.time)


# =============================================================================
# Güvenlik Olayı Kaydı
# =============================================================================

@dataclass
class SafetyEvent:
    """
    Güvenlik durumu değişikliği kaydı.

    Her seviye değişikliğinde bir SafetyEvent oluşturulur
    ve SafetyEvaluator'ın geçmiş listesinde saklanır.
    """
    previous_level:  InterferenceLevel
    new_level:       InterferenceLevel
    local_noise:     int
    remote_noise:    int
    description:     str
    timestamp:       float = field(default_factory=time.time)


# =============================================================================
# Güvenlik Değerlendirici Sınıfı
# =============================================================================

class SafetyEvaluator:
    """
    Telemetri verilerine göre güvenlik seviyesini hesaplayan sınıf.

    Noise (gürültü) seviyelerine ve RSSI değerlerine bakarak
    InterferenceLevel ve SafetyLevel hesaplar.

    Eşik Değerleri (varsayılan):
        noise_low      = 60  → LOW girişim başlangıcı
        noise_medium   = 70  → MEDIUM girişim başlangıcı
        noise_high     = 80  → HIGH girişim başlangıcı
        noise_critical = 100 → CRITICAL girişim başlangıcı
        rssi_weak      = 50  → Zayıf sinyal eşiği

    Bu eşik değerleri yapıcıda (constructor) parametrik olarak
    değiştirilebilir.
    """

    # Geçmiş olayların saklanacağı maksimum sayı
    MAX_EVENT_HISTORY = 50

    def __init__(
        self,
        noise_low: int = 60,
        noise_medium: int = 70,
        noise_high: int = 80,
        noise_critical: int = 100,
        rssi_weak: int = 50,
    ):
        """
        Args:
            noise_low:      LOW girişim eşiği (dBm).
            noise_medium:   MEDIUM girişim eşiği (dBm).
            noise_high:     HIGH girişim eşiği (dBm).
            noise_critical: CRITICAL girişim eşiği (dBm).
            rssi_weak:      Zayıf sinyal eşiği (dBm).
        """
        self.noise_low      = noise_low
        self.noise_medium   = noise_medium
        self.noise_high     = noise_high
        self.noise_critical = noise_critical
        self.rssi_weak      = rssi_weak

        # Gürültü geçmişi (ortalama hesabı için)
        self._noise_history: List[int] = []
        self._max_noise_history = 100

        # Olay geçmişi
        self._event_history: List[SafetyEvent] = []

        # Son hesaplanan durum
        self._last_interference_level = InterferenceLevel.NONE

    # ─── Girişim Seviyesi Hesaplama ──────────────────────────────────────

    def evaluate_interference(
        self,
        local_noise: int,
        remote_noise: int,
    ) -> InterferenceLevel:
        """
        Yerel ve uzak gürültü değerlerine göre girişim seviyesini hesaplar.

        Her iki noise değerinin MAX'ı alınarak seviye belirlenir.
        Önceki seviyeden farklıysa SafetyEvent kaydı oluşturulur.

        Args:
            local_noise:  Yerel gürültü seviyesi (dBm, pozitif tam sayı).
            remote_noise: Uzak gürültü seviyesi (dBm, pozitif tam sayı).

        Returns:
            InterferenceLevel enum değeri.
        """
        # En yüksek noise değerini referans al
        peak_noise = max(local_noise, remote_noise)

        # Gürültü geçmişine ekle
        self._noise_history.append(peak_noise)
        if len(self._noise_history) > self._max_noise_history:
            self._noise_history.pop(0)

        # Seviye belirleme
        if peak_noise >= self.noise_critical:
            level = InterferenceLevel.CRITICAL
        elif peak_noise >= self.noise_high:
            level = InterferenceLevel.HIGH
        elif peak_noise >= self.noise_medium:
            level = InterferenceLevel.MEDIUM
        elif peak_noise >= self.noise_low:
            level = InterferenceLevel.LOW
        else:
            level = InterferenceLevel.NONE

        # Seviye değişikliği kaydı
        if level != self._last_interference_level:
            event = SafetyEvent(
                previous_level=self._last_interference_level,
                new_level=level,
                local_noise=local_noise,
                remote_noise=remote_noise,
                description=(
                    f'Girişim seviyesi değişti: '
                    f'{self._last_interference_level.value} → {level.value} '
                    f'(noise: local={local_noise}, remote={remote_noise})'
                ),
            )
            self._event_history.append(event)

            # Geçmiş boyutunu sınırla
            if len(self._event_history) > self.MAX_EVENT_HISTORY:
                self._event_history.pop(0)

            self._last_interference_level = level

        return level

    # ─── Genel Güvenlik Seviyesi Hesaplama ───────────────────────────────

    def evaluate_safety(
        self,
        interference_level: InterferenceLevel,
        telemetry_connected: bool,
        local_rssi: int = 0,
        remote_rssi: int = 0,
    ) -> SafetyLevel:
        """
        Girişim seviyesi ve diğer faktörlere göre genel güvenlik
        seviyesini hesaplar.

        Kurallar:
            - Telemetri bağlı değilse → CRITICAL
            - InterferenceLevel.CRITICAL → EMERGENCY
            - InterferenceLevel.HIGH → CRITICAL
            - InterferenceLevel.MEDIUM → WARNING
            - InterferenceLevel.LOW → ADVISORY
            - RSSI çok zayıfsa (< rssi_weak) → en az WARNING
            - Aksi halde → NORMAL

        Args:
            interference_level: Frekans girişim seviyesi.
            telemetry_connected: Telemetri bağlantı durumu.
            local_rssi:  Yerel sinyal gücü (dBm).
            remote_rssi: Uzak sinyal gücü (dBm).

        Returns:
            SafetyLevel enum değeri.
        """
        # Telemetri bağlı değilse kritik durum
        if not telemetry_connected:
            return SafetyLevel.CRITICAL

        # Girişim seviyesine göre temel güvenlik seviyesi
        interference_to_safety = {
            InterferenceLevel.CRITICAL: SafetyLevel.EMERGENCY,
            InterferenceLevel.HIGH:     SafetyLevel.CRITICAL,
            InterferenceLevel.MEDIUM:   SafetyLevel.WARNING,
            InterferenceLevel.LOW:      SafetyLevel.ADVISORY,
            InterferenceLevel.NONE:     SafetyLevel.NORMAL,
        }

        safety = interference_to_safety.get(
            interference_level,
            SafetyLevel.NORMAL,
        )

        # RSSI zayıfsa güvenlik seviyesini yükselt
        min_rssi = min(local_rssi, remote_rssi) if remote_rssi > 0 else local_rssi
        if min_rssi > 0 and min_rssi < self.rssi_weak:
            if safety.value in ('NORMAL', 'ADVISORY'):
                safety = SafetyLevel.WARNING

        return safety

    # ─── Tam Güvenlik Durumu Üretme ──────────────────────────────────────

    def evaluate(
        self,
        local_noise: int,
        remote_noise: int,
        local_rssi: int,
        remote_rssi: int,
        telemetry_connected: bool,
    ) -> SafetyState:
        """
        Tüm parametreleri alarak tam bir SafetyState nesnesi üretir.

        Bu fonksiyon evaluate_interference ve evaluate_safety'yi
        sırayla çağırır ve sonuçları birleştirir.

        Args:
            local_noise:         Yerel gürültü seviyesi (dBm).
            remote_noise:        Uzak gürültü seviyesi (dBm).
            local_rssi:          Yerel sinyal gücü (dBm).
            remote_rssi:         Uzak sinyal gücü (dBm).
            telemetry_connected: Telemetri bağlantı durumu.

        Returns:
            SafetyState veri yapısı.
        """
        interference_level = self.evaluate_interference(
            local_noise, remote_noise,
        )

        safety_level = self.evaluate_safety(
            interference_level=interference_level,
            telemetry_connected=telemetry_connected,
            local_rssi=local_rssi,
            remote_rssi=remote_rssi,
        )

        interference_detected = (
            interference_level != InterferenceLevel.NONE
        )

        noise_floor_avg = self.get_noise_floor_avg()

        # Açıklama metni oluştur
        if interference_detected:
            description = (
                f'Frekans girişimi tespit edildi! '
                f'Seviye: {interference_level.value} | '
                f'Noise: local={local_noise}, remote={remote_noise} | '
                f'Güvenlik: {safety_level.value}'
            )
        else:
            description = (
                f'Frekans temiz. '
                f'Noise: local={local_noise}, remote={remote_noise} | '
                f'Güvenlik: {safety_level.value}'
            )

        return SafetyState(
            safety_level=safety_level,
            interference_level=interference_level,
            interference_detected=interference_detected,
            telemetry_connected=telemetry_connected,
            local_noise=local_noise,
            remote_noise=remote_noise,
            local_rssi=local_rssi,
            remote_rssi=remote_rssi,
            noise_floor_avg=noise_floor_avg,
            description=description,
        )

    # ─── Yardımcı Metotlar ───────────────────────────────────────────────

    def get_noise_floor_avg(self) -> float:
        """
        Birikmiş gürültü geçmişinin ortalamasını döndürür.

        Returns:
            Ortalama gürültü değeri (float). Geçmiş boşsa 0.0.
        """
        if not self._noise_history:
            return 0.0
        return sum(self._noise_history) / len(self._noise_history)

    def get_event_history(self) -> List[SafetyEvent]:
        """
        Güvenlik olayı geçmişinin kopyasını döndürür.

        Returns:
            SafetyEvent listesi (en eskiden en yeniye).
        """
        return list(self._event_history)

    def get_last_interference_level(self) -> InterferenceLevel:
        """Son hesaplanan girişim seviyesini döndürür."""
        return self._last_interference_level

    def reset(self):
        """
        Tüm dahili durumu sıfırlar.
        Gürültü geçmişi, olay geçmişi ve son seviye temizlenir.
        """
        self._noise_history.clear()
        self._event_history.clear()
        self._last_interference_level = InterferenceLevel.NONE
