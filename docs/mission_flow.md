# Uçtan Uca Sistem Akışı — LODOS Albatros

Bu belge, Albatros İDA’nın suya indirme öncesinden görev bitimine kadar izlemesi gereken yazılım ve sistem akışını açıklar.

## 1. Amaç

Amaç, aracın görev başlangıcından görev sonlandırmaya kadar:

- güvenli şekilde hazırlanması,
- görev bilgilerinin yüklenmesi,
- otonom görev sırasını takip etmesi,
- parkurlar arası geçişi kendisinin yapması,
- görev sonunda güvenli durdurma ve veri teslim hazırlığını tamamlamasıdır.

## 2. Ana akış

```text
1. Suya indirme öncesi kontroller
2. Başlangıç noktasına konumlandırma
3. Güç verilmesi
4. Sistem hazırlığı
5. YKİ üzerinden görev yükleme
6. YKİ üzerinden görev başlatma
7. Arm işlemi
8. Parkur-1: nokta takibi
9. Parkur-2: engel bulunan ortamda nokta takibi
10. Parkur-3: kamikaze angajman görevi
11. Görev tamamlandı bilgisinin YKİ’ye iletilmesi
12. Güvenli durdurma
13. Disarm
14. Veri kayıtlarının kontrolü ve teslim hazırlığı
```

## 3. Suya indirme öncesi kontroller

Kontrol edilecekler:

- Gövde sızdırmazlığı
- Kapak ve conta kontrolü
- Motor/pervane bağlantıları
- Batarya bağlantısı
- Güç dağıtım kartı
- Raspberry Pi açılışı
- Pixhawk bağlantısı
- Kamera bağlantısı
- GPS bağlantısı
- Telemetri bağlantısı
- Motor sürücüleri
- Acil durdurma/manuel müdahale planı

Bu aşamada gerçek motor komutu verilmemelidir.

## 4. Güç verilmesi

Güç verildikten sonra şu bileşenlerin aktif olması beklenir:

```text
Raspberry Pi 5
Hailo AI Kit
Pixhawk 2.4.8
Kamera
GPS
IMU / pusula
Telemetri modülü
Motor sürücüleri
Mesafe sensörleri
```

## 5. Sistem hazırlığı

Sistem hazırlığı sırasında ROS2 node’ları başlatılır:

```text
mission_manager
perception
mavros_control
obstacle_avoidance
launch/config node’ları
```

Kontrol adımları:

- ROS2 ortamı source edildi mi?
- MAVROS bağlantısı var mı?
- Pixhawk state okunuyor mu?
- GPS verisi geliyor mu?
- IMU verisi geliyor mu?
- Kamera görüntüsü geliyor mu?
- Model dosyası/algılama sistemi hazır mı?
- YKİ bağlantısı var mı?

## 6. Görev yükleme

YKİ üzerinden hakemlerin verdiği görev noktaları veya parkur bilgileri sisteme girilir.

Görev yükleme çıktısı:

- Waypoint listesi
- Başlangıç/launch bilgisi
- Parkur sıralaması
- Hedef renk bilgisi varsa hedef rengi
- Güvenlik parametreleri
- Referans yarıçapı / hedefe varış eşiği

Mission manager durumu:

```text
IDLE -> SYSTEM_CHECK -> READY -> MISSION_LOADED
```

## 7. Görev başlatma ve arm

Görev başlatma YKİ üzerinden alınır. Gerçek araçta arm işlemi mutlaka manuel onayla yapılmalıdır.

Mission manager durumu:

```text
MISSION_LOADED -> ARMED
```

## 8. Parkur-1: Nokta takibi

Amaç:

- Engel bulunmayan ortamda temel navigasyon ve bot kontrolünü göstermek.
- GPS/IMU verileriyle hedef noktaya ilerlemek.
- Kamera ile sınır dubalarını izlemek.

Girdi verileri:

- GPS konumu
- IMU/pusula yönelimi
- Kamera görüntüsü
- YKİ waypoint bilgisi

Karar:

- Hedef noktaya uzaklık hesaplanır.
- Hedef doğrultusuna göre yönelim hatası hesaplanır.
- PID kontrol ile mikro manevra üretilir.
- Hedefe varış eşiği sağlandığında sonraki noktaya geçilir.

Çıktı:

- Diferansiyel itki komutu
- Görev durum güncellemesi
- YKİ telemetri bilgisi

## 9. Parkur-2: Engel bulunan ortamda nokta takibi

Amaç:

- Görev noktalarına ilerlerken engel dubalarından kaçınmak.
- En az iki duba ikilisi arasından güvenli geçiş yapmak.
- Son görev noktasına ulaşmak.

Girdi verileri:

- YOLO detection sonuçları
- Sarı engel dubası bilgisi
- Turuncu/sınır duba bilgisi
- GPS/IMU verisi
- Hedef waypoint

Karar:

- Engel duba tehlikeli bölgede mi?
- Sınır dubalarının orta noktası nerede?
- Güvenli geçiş hattı nereden geçiyor?
- Engel aşılınca rota yeniden hesaplanmalı mı?

Çıktı:

- Kaçınma manevrası
- Güncel hedef yönelimi
- Costmap / engel haritası kaydı
- YKİ durum bilgisi

Mission manager durumu:

```text
PARKUR_1 -> PARKUR_2
```

## 10. Parkur-3: Kamikaze angajman görevi

Amaç:

- İHA veya hakem/YKİ üzerinden gelen hedef bilgisine göre doğru hedef dubayı bulmak.
- Hedef renk ile algılanan duba rengini eşleştirmek.
- Doğru hedefe yönelerek fiziksel temasla görevi tamamlamak.

Girdi verileri:

- İHA/YKİ hedef renk bilgisi
- Kamera görüntüsü
- YOLO/rengi algılanmış hedefler
- GPS/IMU yönelim verisi

Karar:

- Hedef renk geldi mi?
- Algılanan hedeflerin rengi hedef renkle eşleşiyor mu?
- Eşleşme yoksa arama davranışı devam etmeli mi?
- Eşleşme varsa hedefe yönelme komutu üretildi mi?

Çıktı:

- Hedefe yönelim komutu
- Görev tamamlandı bilgisi
- YKİ’ye durum bildirimi

Mission manager durumu:

```text
PARKUR_2 -> PARKUR_3 -> FINISHED
```

## 11. Görev sonlandırma

Görevler tamamlandığında:

- Motor komutları sıfırlanır.
- Araç konumunu koruyacak şekilde güvenli duruma alınır.
- Disarm işlemi yapılır.
- Görev tamamlandı bilgisi YKİ’ye gönderilir.
- Loglar kontrol edilir.

## 12. Veri teslim hazırlığı

Kontrol edilecek dosya/veriler:

- Telemetri kayıtları
- Kamera/görüntü işleme çıktıları
- YOLO detection kayıtları
- Costmap/engel haritası
- Görev durum logları
- Sistem hata kayıtları

Bu dosyalar yarışma sonrası şartnamedeki süre ve teslim kurallarına uygun hazırlanmalıdır.

## 13. Hata durumları

Örnek hata durumları:

```text
GPS kaybı
Telemetri kopması
Kamera görüntüsü alınamaması
MAVROS bağlantı kaybı
Batarya düşük seviyesi
Motor sürücü hatası
Algılama sistemi yanıt vermemesi
Sızdırmazlık/sıvı teması alarmı
```

Hata durumunda mission manager:

```text
ANY_STATE -> ERROR -> EMERGENCY_STOP
```

kritik durumda:

```text
Motor komutu = 0
Disarm önerisi
YKİ’ye hata bilgisi
Manuel müdahale çağrısı
```
