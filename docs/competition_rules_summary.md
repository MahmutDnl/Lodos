# Yarışma ve Şartname Özeti — TEKNOFEST 2026 İDA

Bu belge şartnamedeki yazılımı ve görev akışını doğrudan etkileyen maddeleri pratik özet halinde tutar. Resmi karar için her zaman güncel şartname esas alınmalıdır.

## 1. Yarışma süreci

Değerlendirme ana aşamaları:

```text
Teknik Yeterlilik Raporu
Kritik Tasarım Raporu
Otonomi Kabiliyeti Gösterimi Videosu
Final değerlendirme / yarışma
```

## 2. Önemli tarihler

Şartnamedeki takvim özeti:

```text
28.02.2026          Yarışma son başvuru tarihi
24.03.2026 17:00   Teknik Yeterlilik Raporu son teslim
31.03.2026          TYR sonuçları
20.05.2026 17:00   Kritik Tasarım Raporu son teslim
08.06.2026          KTR sonuçları
21.07.2026 17:00   Sistem kabiliyeti videoları son teslim
27.07.2026          Finalist takımların açıklanması
Ağustos-Eylül 2026 Yarışma tarihi/yeri
30 Eylül-4 Ekim    TEKNOFEST
```

## 3. Tanımlar

- **Görev:** Engel bulunmayan ortamda nokta takibi, engel bulunan ortamda nokta takibi ve kamikaze angajmanı olmak üzere otonomi özelliklerinin sınandığı adımlar bütünü.
- **Parkur:** Parkur-1, Parkur-2 ve Parkur-3 alt görevlerinin icra edildiği fiziksel ortam.
- **YKİ:** Yer Kontrol İstasyonu.
- **İDA:** İnsansız Deniz Aracı.
- **İHA:** İnsansız Hava Aracı.

## 4. Parkur-1: Nokta Takip Görevi

Amaç:

- Temel bot kontrolü ve navigasyon kabiliyetinin gösterilmesi.

Yazılım açısından gerekli olanlar:

- Waypoint okuma
- GPS/IMU ile hedefe yönelme
- Hedefe varış kontrolü
- Sıradaki noktaya geçiş
- Görev durum bildirimi

## 5. Parkur-2: Engel Bulunan Ortamda Nokta Takibi

Amaç:

- Birden fazla görev noktası arasında bulunan engellerden sakınarak nihai görev noktasına ulaşmak.

Yazılım açısından gerekli olanlar:

- Sınır duba algılama
- Engel duba algılama
- Güvenli geçiş orta noktası hesaplama
- Engel tehlike kontrolü
- Kaçınma manevrası
- Son görev noktasına ulaşma doğrulaması

## 6. Parkur-3: Kamikaze Angajman Görevi

Amaç:

- Belirlenecek veya İHA tarafından tespit edilecek hedefe İDA ile fiziksel temas sağlamak.

Yazılım açısından gerekli olanlar:

- Hedef renk bilgisinin YKİ/İHA’dan alınması
- Kamera/YOLO ile hedef duba tespiti
- Renk eşleşmesi
- Hedefe yönelme
- Temas/görev tamamlandı doğrulaması

## 7. Yarışma alanı ve operatör kuralları açısından yazılıma etkiler

- Yarışma alanında takım üyesi sayısı sınırlı olabilir.
- Görev yükleme kablolu veya kablosuz olabilir.
- YKİ operatörünün konumu ve görev yükleme şekli yarışma günü kurallarıyla uyumlu olmalıdır.
- Danışmanlar yarışma alanı ve çadırında bulunamayabilir.
- Hakem brifinginde verilen bilgiler kural niteliğindedir.

## 8. Veri teslim ve ceza riski

Yarışma sonrası veri teslimi için şu dosyalar düzenli saklanmalıdır:

- Telemetri
- Görüntü işleme çıktıları
- Görev durum logları
- Costmap/engel haritası
- Video/veri teslim klasörü

Veri teslim süresi ve dosya türleri resmi şartnameye göre tekrar kontrol edilmelidir.

## 9. Yazılım ödülü açısından dikkat

Şartnamede “En Özgün Yazılım Ödülü” için işlevsellik, güvenilirlik, güncel yüksek teknolojiyle uyumlu altyapı ve sistem mimarisi gibi unsurların değerlendirileceği belirtilmiştir. Bu nedenle repo düzeni, README, modüler ROS2 mimarisi ve test belgeleri düzenli tutulmalıdır.

## 10. KTR’ye doğrudan katkı verecek yazılım belgeleri

Bu repository’de şu dosyalar KTR yazımında kullanılabilir:

```text
docs/mission_flow.md
docs/system_architecture.md
docs/software_requirements.md
docs/ros2_design.md
docs/hardware_notes.md
```
