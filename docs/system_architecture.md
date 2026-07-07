# Sistem Mimarisi — LODOS Albatros

Bu belge Albatros’un mekanik, elektronik, yazılım ve haberleşme mimarisini özetler.

## 1. Mimari katmanlar

```text
YKİ / Operatör Katmanı
        ↓
Görev Yönetimi Katmanı
        ↓
Algılama + Konum Kestirimi Katmanı
        ↓
Karar Verme + Kaçınma Katmanı
        ↓
Kontrol / MAVROS / Pixhawk Katmanı
        ↓
Motor Sürücüleri + İtki Sistemi
```

## 2. YKİ / Arayüz katmanı

YKİ’nin sorumlulukları:

- Görev noktalarını girmek
- Görev başlatma komutu vermek
- Araç durumunu görmek
- GPS/telemetri durumunu takip etmek
- Kamera görüntüsünü veya işlenmiş çıktıları görmek
- Parkur durumunu izlemek
- Acil durumda görevi durdurmak

Hazır MissionPlanner/QGroundControl yerine takımın özgün YKİ arayüzü hedeflenmektedir.

## 3. Görev yönetimi katmanı

Sorumlu paket:

```text
lodos_mission_manager
```

Görevleri:

- Sistem durumunu tutmak
- Parkur sırasını yönetmek
- Görev başlangıç/bitiş koşullarını kontrol etmek
- Parkurlar arası otomatik geçiş yapmak
- Hata durumunda güvenli durdurma başlatmak

Önerilen durumlar:

```text
IDLE
SYSTEM_CHECK
READY
MISSION_LOADED
ARMED
PARKUR_1
PARKUR_2
PARKUR_3
RETURN_HOME
FINISHED
ERROR
EMERGENCY_STOP
```

## 4. Algılama katmanı

Sorumlu paket:

```text
lodos_perception
```

Görevleri:

- Kamera görüntüsünü almak
- YOLO modeli ile duba/hedef tespiti yapmak
- Renk ve sınıf bilgisi çıkarmak
- Detection sonuçlarını ROS2 topic olarak yayınlamak

Hedef sınıflar:

```text
turuncu_duba
sari_duba
siyah_duba
kirmizi_duba
yesil_duba
```

Hailo AI Kit gerçek zamanlı inference için kullanılacaktır. İlk geliştirmede placeholder/sahte detection ile sistem iskeleti kurulabilir.

## 5. Konum kestirimi ve navigasyon

Kaynaklar:

- GPS
- Pixhawk IMU
- Pusula/yönelim
- MAVROS üzerinden gelen vehicle state
- Gerektiğinde kamera tabanlı görsel ipuçları

EKF yaklaşımı GPS, IMU ve pusula verilerinin daha kararlı kullanılmasını sağlar. İlk prototipte Pixhawk/MAVROS topic’leri okunup mission manager’a sade veri akışı verilmelidir.

## 6. Kaçınma katmanı

Sorumlu paket:

```text
lodos_obstacle_avoidance
```

Görevleri:

- YOLO detection sonuçlarını almak
- Engel dubasını tehlikeli bölgede değerlendirmek
- Güvenli geçiş yönü/hızı önermek
- Parkur-2 için dinamik rota düzeltmesi yapmak
- Costmap/engel haritası çıktısı üretmek

İlk aşamada basit kural tabanlı kaçınma yeterlidir. Daha sonra costmap ve local planner mantığı geliştirilebilir.

## 7. Kontrol / Pixhawk / MAVROS katmanı

Sorumlu paket:

```text
lodos_mavros_control
```

Görevleri:

- MAVROS state okumak
- GPS ve IMU verilerini almak
- Arm/disarm için güvenli interface tasarlamak
- Mode değişimi için güvenli interface tasarlamak
- Hız/yönelim komutlarını Pixhawk’a aktarmak

Not: Gerçek arm/disarm ve motor komutları yalnızca manuel onayla çalıştırılmalıdır.

## 8. Haberleşme mimarisi

Araç içi yazılım haberleşmesi:

```text
ROS2 topic/service/action
```

Raspberry Pi — Pixhawk haberleşmesi:

```text
UART + MAVLink + MAVROS
```

Araç — YKİ haberleşmesi:

```text
3DR 915 MHz telemetri + MAVLink
```

## 9. Donanım veri akışı

```text
Kamera
  -> Raspberry Pi / Hailo AI Kit
  -> lodos_perception
  -> /perception/detections
  -> lodos_mission_manager / lodos_obstacle_avoidance

GPS / IMU / Pusula
  -> Pixhawk
  -> MAVLink
  -> MAVROS
  -> lodos_mavros_control
  -> /navigation/* / /vehicle/status

Mission Manager
  -> Parkur kararı
  -> Obstacle Avoidance / Control
  -> MAVROS
  -> Pixhawk
  -> Motor sürücüleri
```

## 10. Önerilen ROS2 paketleri

```text
src/
├── lodos_mission_manager/
├── lodos_perception/
├── lodos_mavros_control/
├── lodos_obstacle_avoidance/
└── lodos_launch/
```

## 11. İlk MVP hedefi

İlk çalışan minimum sistem:

```text
mission_manager node çalışır
sahte detection publisher çalışır
sahte GPS/heading girdisi alınır
mission state yayınlanır
colcon build başarıyla alınır
launch dosyası node’ları başlatır
```

Gerçek YOLO, Hailo ve Pixhawk entegrasyonu MVP’den sonra eklenmelidir.
