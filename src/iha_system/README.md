# LODOS İHA — Plaka Renk Algılama ve MAVLink Gönderim Sistemi

Bu klasör İHA'nın (Drone) Raspberry Pi 5'inde çalışan renk algılama ve MAVLink
haberleşme yazılımını içerir. **ROS2 gerektirmez** — bağımsız Python scriptleridir.

## Dosyalar

| Dosya | Görev |
|---|---|
| `iha_color_detector.py` | Kameradan plaka rengini (Kırmızı/Yeşil/Siyah) algılar |
| `iha_mavlink_bridge.py` | Algılanan rengi MAVLink ile Pixhawk'a gönderir |
| `requirements.txt` | Python bağımlılıkları |

## Donanım Bağlantısı

```text
İHA Raspberry Pi 5  ──USB (sinyal hatları)──  İHA Pixhawk 2.4.8  ──TELEM1──  3DR Telemetri
```

> **Not:** USB kablosunun +5V (VBUS) hattı kesilmelidir, sadece D+, D- ve GND kullanılır.

## Kurulum

```bash
cd ~/iha_system
pip3 install -r requirements.txt
```

## Kullanım

### Test Modu (Sadece Kamera)
```bash
python3 iha_color_detector.py
```

### Tam Görev (Kamera + MAVLink)
```bash
python3 iha_mavlink_bridge.py
```

### Parametreli Çalıştırma
```bash
python3 iha_mavlink_bridge.py --port /dev/ttyACM0 --baud 57600 --camera 0 --confirm 5
```

## MAVLink Mesaj Formatı

| Mesaj Tipi | Alan | Değer | Açıklama |
|---|---|---|---|
| `NAMED_VALUE_FLOAT` | name=`TCOLOR` | `1.0` | Kırmızı |
| `NAMED_VALUE_FLOAT` | name=`TCOLOR` | `2.0` | Yeşil |
| `NAMED_VALUE_FLOAT` | name=`TCOLOR` | `3.0` | Siyah |
| `STATUSTEXT` | text | `TARGET:kirmizi` | Okunabilir metin |

## HSV Renk Aralıkları

Farklı aydınlatma koşullarında kalibrasyona ihtiyaç duyulabilir.
`iha_color_detector.py` dosyasındaki HSV sabitleri güncellenmelidir.
