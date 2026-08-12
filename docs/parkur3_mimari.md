# Parkur 3 — Kamikaze Angajman Sistem Mimarisi

Bu belge TEKNOFEST 2026 İDA Yarışması Parkur 3 (Kamikaze Angajmanı) için
İHA → YKİ → İDA veri akışının teknik dokümantasyonunu içerir.

## 1. Genel Bakış

Parkur 3'te su yüzeyinde 3 adet renkli duba (Kırmızı, Yeşil, Siyah) bulunur.
Kıyıda bir İHA, yerdeki plakanın rengini algılar ve bu renk bilgisini
telemetri üzerinden Yer Kontrol İstasyonu'na (YKİ) gönderir. YKİ bu bilgiyi
İDA'ya (Albatros) aktarır. İDA, hedef renkteki dubayı tespit edip kamikaze
angajmanı (fiziksel temas) yapar.

## 2. Donanım Mimarisi

```text
┌─────────────────────────┐
│  İHA (Drone)            │
│  ├─ Raspberry Pi 5 8GB  │
│  ├─ Alt Kamera          │
│  ├─ Pixhawk 2.4.8       │
│  │   ├─ TELEM1 → 3DR #1│
│  │   └─ GPS → M8N       │
│  └─ 4 Motor (QuadCopter)│
└──────────┬──────────────┘
           │ 915 MHz (3DR Çift 1)
           ▼
┌─────────────────────────┐
│  YKİ Bilgisayarı        │
│  ├─ USB0: 3DR Alıcı #1  │──── İHA'dan okuma
│  └─ USB1: 3DR Alıcı #2  │──── İDA'ya yazma
└──────────┬──────────────┘
           │ 915 MHz (3DR Çift 2)
           ▼
┌─────────────────────────┐
│  İDA (Albatros)         │
│  ├─ Raspberry Pi 5 8GB  │
│  ├─ Pixhawk 2.4.8       │
│  │   └─ TELEM1 → 3DR #2│
│  ├─ Kamera              │
│  └─ Hailo AI Kit        │
└─────────────────────────┘
```

## 3. Yazılım Bileşenleri

### 3.1 İHA Sistemi (`src/iha_system/`)

**Platform:** İHA Raspberry Pi 5 — Standalone Python (ROS2 gerektirmez)

| Dosya | Görev |
|---|---|
| `iha_color_detector.py` | OpenCV HSV ile plaka rengini algılar |
| `iha_mavlink_bridge.py` | pymavlink ile Pixhawk'a renk gönderir |

**Çalıştırma:**
```bash
python3 iha_mavlink_bridge.py --port /dev/ttyACM0 --camera 0
```

### 3.2 YKİ Köprü (`src/yki_bridge/`)

**Platform:** YKİ bilgisayarı — Standalone Python (ROS2 gerektirmez)

| Dosya | Görev |
|---|---|
| `yki_bridge.py` | İHA'dan renk oku → İDA'ya aktar |

**Çalıştırma:**
```bash
python3 yki_bridge.py --iha-port /dev/ttyUSB0 --ida-port /dev/ttyUSB1
```

### 3.3 İDA ROS2 Node (`albatros_tahta`)

**Platform:** İDA Raspberry Pi 5 — ROS2 Jazzy

| Node | Görev |
|---|---|
| `target_color_node` | MAVROS STATUSTEXT → `/perception/target_color` |
| `duba_fusion_node` | Hedef renk + YOLO tespiti → `goal_buoy` kilitleme |
| `karar_node` | Visual Homing PID ile kamikaze sürüşü |

## 4. MAVLink Mesaj Protokolü

### Renk Kod Tablosu

| Renk | Float Değeri | STATUSTEXT |
|---|---|---|
| Kırmızı | `1.0` | `TARGET:kirmizi` |
| Yeşil | `2.0` | `TARGET:yesil` |
| Siyah | `3.0` | `TARGET:siyah` |

### Mesaj Akışı

| Kaynak → Hedef | Mesaj Tipi | İçerik |
|---|---|---|
| İHA RPi → İHA Pixhawk | `NAMED_VALUE_FLOAT` | name=`TCOLOR`, value=`1.0/2.0/3.0` |
| İHA Pixhawk → YKİ | `NAMED_VALUE_FLOAT` + `STATUSTEXT` | Otomatik TELEM1 aktarımı |
| YKİ → İDA Pixhawk | `STATUSTEXT` | `TARGET:kirmizi` |
| İDA MAVROS → ROS2 | `/mavros/statustext/recv` | StatusText mesajı |
| `target_color_node` → ROS2 | `/perception/target_color` | `kirmizi` (String) |

## 5. USB Kablo Modifikasyonu (İHA RPi ↔ Pixhawk)

İHA'nın Raspberry Pi 5'i ile Pixhawk 2.4.8 arasındaki USB bağlantısında
güç hattı kesilmelidir:

```text
USB Kablo Hatları:
  ✂ VBUS (+5V / Kırmızı) → KESİLECEK (ters akım önlemi)
  ✓ GND  (Siyah)         → KESİLMEYECEK (sinyal referansı)
  ✓ D+   (Yeşil)         → KESİLMEYECEK (veri hattı)
  ✓ D-   (Beyaz)         → KESİLMEYECEK (veri hattı)
```

## 6. Telemetri Konfigürasyonu

2 adet 3DR telemetri çifti (toplam 4 modül) kullanılır:

- **Çift 1:** İHA Pixhawk TELEM1 ↔ YKİ USB0
- **Çift 2:** İDA Pixhawk TELEM1 ↔ YKİ USB1

Her çift fabrika ayarlarında bağımsız çalışır, frekans değişikliği gerekmez.

## 7. Parkur 3 Görev Akışı

```text
1. İHA havalanır
2. İHA alt kamera ile plakanın rengini algılar
3. İHA → MAVLink → TELEM1 → YKİ (3DR Çift 1)
4. YKİ ekranında renk görüntülenir
5. YKİ → MAVLink → TELEM1 → İDA Pixhawk (3DR Çift 2)
6. MAVROS → target_color_node → /perception/target_color
7. duba_fusion_node hedef renkteki dubayı "goal_buoy" olarak kilitler
8. karar_node Visual Homing PID ile dubaya yönelir
9. kontrol_node MAVROS üzerinden motor komutları verir
10. İDA hedefe fiziksel temas kurar → GÖREV TAMAMLANDI
```
