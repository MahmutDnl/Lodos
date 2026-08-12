# LODOS YKİ — İHA → İDA Renk Köprü Sistemi

Bu klasör Yer Kontrol İstasyonu (YKİ) bilgisayarında çalışan köprü scriptini içerir.
**ROS2 gerektirmez** — bağımsız Python scriptidir.

## Görev

İHA telemetrisinden gelen hedef renk verisini okur ve İDA telemetrisine aktarır.

## Donanım Bağlantısı

```text
YKİ Bilgisayarı
├── USB0  ←  3DR Çift 1 Alıcısı  ←  İHA Pixhawk (OKUMA)
└── USB1  ←  3DR Çift 2 Alıcısı  →  İDA Pixhawk (YAZMA)
```

## Kurulum

```bash
cd ~/yki_bridge
pip3 install -r requirements.txt
```

## Kullanım

```bash
# Varsayılan portlarla
python3 yki_bridge.py

# Portları belirterek
python3 yki_bridge.py --iha-port /dev/ttyUSB0 --ida-port /dev/ttyUSB1

# Windows üzerinde
python3 yki_bridge.py --iha-port COM3 --ida-port COM4
```

## Port Tespiti (Linux)

YKİ bilgisayarına iki USB telemetri modülü takıldığında portları tespit etmek için:

```bash
ls /dev/ttyUSB*
```

Hangi portun hangisine ait olduğunu görmek için:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep serial
udevadm info -a -n /dev/ttyUSB1 | grep serial
```
