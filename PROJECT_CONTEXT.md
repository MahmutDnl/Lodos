# PROJECT_CONTEXT.md — LODOS Albatros Proje Bağlamı

Bu dosya, Antigravity’nin ve takım üyelerinin projeyi hızlı anlaması için hazırlanmıştır. Kaynak olarak 2026 İnsansız Deniz Aracı Şartnamesi, LODOS TYR 2026 ve KTR V1 belgeleri esas alınmıştır.

## 1. Genel proje özeti

LODOS takımı tarafından geliştirilen **Albatros**, TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması için tasarlanan otonom bir su üstü aracıdır. Araçtan beklenen ana kabiliyetler:

- Otonom seyir
- Nokta takibi
- Engel algılama ve engelden kaçınma
- Parkurlar arası görev geçişi
- İHA/YKİ üzerinden gelen hedef bilgisine göre kamikaze angajman görevi
- Görev boyunca telemetri ve sistem durumunun izlenmesi
- Görev sonunda güvenli durdurma ve veri teslim hazırlığı

## 2. Yarışma görevleri

Şartnameye göre görev, üç ana parkurdan oluşur:

1. **Parkur-1: Engel bulunmayan ortamda nokta takibi**
   - Temel bot kontrolü ve navigasyonun gösterilmesi amaçlanır.
   - Araç verilen görev noktalarını takip eder.

2. **Parkur-2: Engel bulunan ortamda nokta takibi**
   - Araç birden fazla görev noktası arasında ilerlerken engellerden sakınmalıdır.
   - En az iki duba ikilisi arasından geçme ve son görev noktasına ulaşma hedeflenir.

3. **Parkur-3: Kamikaze angajman görevi**
   - İHA veya yarışma başlangıcında verilen hedef bilgisine göre hedef duba belirlenir.
   - İDA, hedef ile fiziksel temas kurarak görevi tamamlamayı hedefler.

## 3. Araç platformu

Albatros için trimaran gövde formu tercih edilmiştir. Bunun temel nedenleri:

- Yanal stabiliteyi artırması
- Yalpa hareketini azaltması
- Dalgalı koşullarda daha kararlı seyir sağlaması
- Elektronik bileşenler için daha dengeli yerleşim alanı sağlaması
- Diferansiyel itki sisteminin daha etkili kullanılmasına yardımcı olması

KTR’de genel dış boyutlar şu şekilde belirtilmiştir:

```text
Uzunluk: 1000 mm
Genişlik: 805 mm
Yükseklik: 440 mm
```

## 4. Mekanik üretim yaklaşımı

Gövde üretiminde hibrit yaklaşım hedeflenmiştir:

- Ana gövde, ama gövdeleri, taşıyıcı kollar ve kapaklar 3B yazıcı ile PETG filament kullanılarak üretilecektir.
- Üretilen parçalar siyanoakrilat bazlı yapıştırıcılar ve mekanik geçme detaylarıyla birleştirilecektir.
- Dış yüzey cam elyaf kumaş ve epoksi reçine ile güçlendirilecektir.
- Su sızdırmazlığı için kritik bölgelerde poliüretan bazlı yapıştırıcı ve sıvı conta kullanılacaktır.
- Son yüzey işlemleri: macun, zımpara, astar, marin boya ve vernik.

## 5. İtki ve yönlendirme sistemi

- İtki sistemi iki elektrik motoru ve pervane düzeneği üzerine kuruludur.
- Motorlar sağ ve sol ama gövdelerinin kıç bölgelerine simetrik olarak yerleştirilir.
- Dümen kullanılmadan motor devir farkı ile yönlendirme yapılır.
- Bu yöntem **diferansiyel itki** olarak adlandırılır.
- Düz seyirde iki motor eşit devirde çalışır.
- Dönüşte motor hızları farklılaştırılarak yaw momenti oluşturulur.

## 6. Elektronik ve kontrol altyapısı

Temel elektronik bileşenler:

- Raspberry Pi 5
- Hailo AI Kit
- Pixhawk 2.4.8
- GPS modülü
- IMU/pusula
- Kamera
- Ultrasonik mesafe sensörleri
- Motor sürücüleri
- 3DR 915 MHz telemetri
- Güç dağıtım kartı
- Batarya ve güç modülleri

## 7. Yazılım mimarisi

Yazılım sistemi ROS2 mimarisi üzerine kuruludur. ROS2’nin modüler node yapısı sayesinde algılama, karar verme, kontrol ve haberleşme katmanları birbirinden ayrılarak geliştirilebilir.

Ana yazılım katmanları:

1. **Arayüz/YKİ katmanı**
   - Görev yükleme
   - Görev başlatma
   - Araç durumu izleme
   - Kamera/GPS/telemetri gösterimi

2. **Algılama katmanı**
   - Kamera görüntüsü
   - YOLO tabanlı duba/hedef tespiti
   - Renk sınıflandırma
   - Engel konumu çıkarımı

3. **Konum ve durum kestirimi**
   - GPS
   - IMU/pusula
   - Pixhawk verileri
   - EKF ile konum/yönelim iyileştirme

4. **Karar verme ve görev yönetimi**
   - Parkur sıralaması
   - Görev durum makinesi
   - Hedef kontrolü
   - Parkur geçiş koşulları

5. **Kontrol ve icra**
   - PID kontrol
   - Diferansiyel itki komutları
   - MAVROS üzerinden Pixhawk’a hız/yönelim komutu

6. **Haberleşme**
   - ROS2 topic/service yapısı
   - MAVROS köprüsü
   - MAVLink
   - 3DR telemetri

## 8. Algılama sistemi

Algılama sistemi şu sınıflara odaklanır:

```text
turuncu_duba
sari_duba
siyah_duba
kirmizi_duba
yesil_duba
```

Temel amaçlar:

- Parkur sınır dubalarını görmek
- Engel dubalarını algılamak
- Hedef renk/duba eşleşmesini yapmak
- Parkur-2 için güvenli geçiş hattı üretmek
- Parkur-3 için hedefe yönelmek

İlk geliştirme aşamasında YOLO sonucu basit bir detection topic’i olarak yayınlanabilir. Hailo AI Kit entegrasyonu sonraki aşamada optimize edilir.

## 9. Görev akışı

Genel görev akışı:

```text
Suya indirme öncesi kontroller
Başlangıç noktasına konumlandırma
Güç verilmesi
Raspberry Pi / Hailo / Pixhawk / sensör / telemetri başlatma
ROS2 node’larının başlatılması
YKİ üzerinden görev yükleme
YKİ üzerinden görev başlatma
Arm işlemi
Parkur-1: nokta takibi
Parkur-2: engel algılama ve kaçınma
Parkur-3: hedef/kamikaze angajmanı
Görev bitiş doğrulaması
Güvenli durdurma
Disarm
Veri kayıtlarının kontrolü
```

## 10. KTR/TYR sonrası dikkat edilmesi gerekenler

KTR’de TYF/TYR sonrası sistem akışı daha ayrıntılı hale getirilmiştir. Bu nedenle yazılım geliştirme tarafında özellikle şu başlıklar net tutulmalıdır:

- Güç verme sırası
- Sistem hazırlığı
- Görev yükleme
- Arm/disarm süreci
- Parkurlar arası otomatik geçiş
- Güvenli durdurma
- Telemetri/görüntü/costmap veri kaydı

## 11. Geliştirme önceliği

İlk hedef gerçek YOLO veya gerçek Pixhawk entegrasyonu değil, sağlam yazılım iskeletidir:

1. ROS2 workspace düzeni
2. Mission manager state machine
3. Sahte/simülasyon detection publisher
4. Sahte/simülasyon GPS/heading girdisi
5. MAVROS state okuyucu
6. Launch dosyası
7. Basit terminal testleri
8. Gerçek kamera/YOLO entegrasyonu
9. Hailo optimize inference
10. Gerçek Pixhawk/MAVROS testleri

## 12. Kısa sözlük

- İDA: İnsansız Deniz Aracı
- İHA: İnsansız Hava Aracı
- YKİ: Yer Kontrol İstasyonu
- KTR: Kritik Tasarım Raporu
- TYR/TYF: Teknik Yeterlilik Raporu/Formu
- MAVROS: ROS ile MAVLink/Pixhawk arasında köprü
- EKF: Extended Kalman Filter
- PID: Proportional-Integral-Derivative kontrol
- Costmap: Engel haritası
