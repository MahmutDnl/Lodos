# Donanım ve Platform Notları — LODOS Albatros

Bu belge yazılım geliştirirken bilinmesi gereken donanım/platform bilgilerini özetler.

## 1. Gövde

- Araç trimaran gövde formuna sahiptir.
- Trimaran yapı yanal stabiliteyi artırmak ve yalpa hareketini azaltmak için tercih edilmiştir.
- Ana gövde elektronik bileşenler için güvenli hacim sağlar.
- Sağ ve sol ama gövdeleri hem denge hem de itki sistemi yerleşimi için kullanılır.

## 2. Boyutlar

KTR’de verilen genel dış boyutlar:

```text
Uzunluk: 1000 mm
Genişlik: 805 mm
Yükseklik: 440 mm
```

## 3. Üretim

- Parçalı üretim
- 3B yazıcı
- PETG filament
- Cam elyaf kumaş
- Epoksi reçine
- Macun, zımpara, astar, marin boya, vernik
- Kritik bölgelerde sıvı conta/poliüretan bazlı sızdırmazlık

## 4. İtki sistemi

- İki motorlu yapı
- Motorlar sağ/sol yan gövdelerin kıç kısmında
- Diferansiyel itki ile yönlendirme
- Dümen kullanılmadan motor devir farkıyla dönüş
- Düz seyirde iki motor eşit devir
- Dönüşte sağ/sol motor hız farkı

## 5. Ana elektronik bileşenler

```text
Raspberry Pi 5
Hailo AI Kit
Pixhawk 2.4.8
GPS modülü
Kamera
Ultrasonik mesafe sensörleri
Motor sürücüleri
Güç dağıtım kartı
3DR 915 MHz telemetri
Batarya
Sigorta / şalter
MicroSD kart
```

## 6. Yazılımı etkileyen donanım noktaları

### Raspberry Pi 5

- ROS2 node’ları çalışır.
- Kamera ve Hailo tarafı burada yönetilir.
- MAVROS köprüsü burada çalışır.

### Hailo AI Kit

- YOLO inference hızlandırma için kullanılır.
- Model dönüşümü ve runtime entegrasyonu ayrı test edilmelidir.

### Pixhawk 2.4.8

- IMU, pusula, GPS ve araç kontrol döngüsü için temel bileşendir.
- Raspberry Pi ile UART/MAVLink üzerinden haberleşir.
- MAVROS ile ROS2 tarafına veri aktarır.

### Telemetri

- YKİ ile araç arasında görev durumu ve kontrol verisi taşır.
- Yarışma kuralları açısından haberleşme frekansı ve kullanım şekli kontrol edilmelidir.

## 7. Güvenlik

Yazılım geliştirirken şu donanım riskleri göz önünde bulundurulmalıdır:

- Motorların beklenmedik çalışması
- Pixhawk arm/disarm kontrolünün yanlış tetiklenmesi
- Batarya düşük voltaj
- Su sızıntısı
- Kamera/GPS/telemetri kaybı
- Motor sürücülerinin aşırı ısınması
- Pervane çevresinde fiziksel güvenlik riski

## 8. Test önerisi

Gerçek araç üzerinde test yapmadan önce:

```text
1. Masaüstü ROS2 node testi
2. Sahte veri testi
3. Kamera ayrı test
4. YOLO ayrı test
5. MAVROS/Pixhawk bağlantı testi
6. Motor sürücüleri enerjisiz komut testi
7. Pervanesiz düşük güç motor testi
8. Suya indirmeden kuru sistem testi
9. Güvenli alanda su üstü test
```
