# Yazılım Gereksinimleri — LODOS Albatros

Bu belge yazılım tarafında neyin yapılması gerektiğini netleştirmek için hazırlanmıştır.

## 1. Genel gereksinimler

| Kod | Gereksinim | Öncelik |
|---|---|---|
| SW-001 | Sistem ROS2 tabanlı modüler node yapısıyla geliştirilecek. | Yüksek |
| SW-002 | Ubuntu 24.04 ortamında ROS2 Jazzy uyumluluğu korunacak. | Yüksek |
| SW-003 | Her node tek sorumluluk ilkesine göre yazılacak. | Yüksek |
| SW-004 | Görev akışı mission manager tarafından yönetilecek. | Yüksek |
| SW-005 | Gerçek araç komutları manuel onay olmadan çalıştırılmayacak. | Kritik |
| SW-006 | Build çıktıları ve büyük model/dataset dosyaları GitHub’a eklenmeyecek. | Yüksek |

## 2. Mission manager gereksinimleri

| Kod | Gereksinim | Öncelik |
|---|---|---|
| MM-001 | Görev durum makinesi bulunacak. | Kritik |
| MM-002 | `/mission/start` komutunu alabilecek. | Kritik |
| MM-003 | `/mission/stop` veya acil durdurma komutunu alabilecek. | Kritik |
| MM-004 | `/mission/state` üzerinden anlık durum yayınlayacak. | Kritik |
| MM-005 | Parkur-1, Parkur-2, Parkur-3 geçişlerini takip edecek. | Yüksek |
| MM-006 | Hata durumunda `ERROR` veya `EMERGENCY_STOP` durumuna geçecek. | Kritik |
| MM-007 | Parkur tamamlandı koşullarını ayrı fonksiyonlarda tutacak. | Orta |

## 3. Perception gereksinimleri

| Kod | Gereksinim | Öncelik |
|---|---|---|
| PR-001 | Kamera görüntüsünü alabilecek. | Yüksek |
| PR-002 | YOLO model çıktısını okuyabilecek. | Yüksek |
| PR-003 | Duba sınıflarını ayırt edecek. | Yüksek |
| PR-004 | `/perception/detections` topic’i yayınlayacak. | Yüksek |
| PR-005 | Parkur-2 için sarı engel dubalarını tespit edecek. | Kritik |
| PR-006 | Parkur-3 için kırmızı/yeşil hedef renk eşleşmesini destekleyecek. | Kritik |
| PR-007 | Gerçek model yokken sahte detection test modu sunacak. | Orta |

## 4. MAVROS / Pixhawk gereksinimleri

| Kod | Gereksinim | Öncelik |
|---|---|---|
| MV-001 | `/mavros/state` dinlenecek. | Kritik |
| MV-002 | GPS verisi okunacak. | Kritik |
| MV-003 | IMU verisi okunacak. | Kritik |
| MV-004 | Vehicle status sadeleştirilmiş topic olarak yayınlanacak. | Yüksek |
| MV-005 | Arm/disarm fonksiyonu güvenli arayüzle hazırlanacak. | Kritik |
| MV-006 | Mode değiştirme fonksiyonu güvenli arayüzle hazırlanacak. | Yüksek |
| MV-007 | Hız/konum komutu göndermek için interface tasarlanacak. | Yüksek |

## 5. Obstacle avoidance gereksinimleri

| Kod | Gereksinim | Öncelik |
|---|---|---|
| OA-001 | Detection verilerini dinleyecek. | Yüksek |
| OA-002 | Engel dubasının tehlikeli bölgede olup olmadığını belirleyecek. | Kritik |
| OA-003 | Basit kaçınma komutu üretecek. | Kritik |
| OA-004 | `/obstacle_avoidance/cmd` yayınlayacak. | Yüksek |
| OA-005 | Costmap/engel haritası kaydı için çıktı yapısı tasarlanacak. | Orta |

## 6. Launch ve config gereksinimleri

| Kod | Gereksinim | Öncelik |
|---|---|---|
| LC-001 | Sistemi başlatan `mission.launch.py` dosyası olacak. | Yüksek |
| LC-002 | Parametreler config dosyalarında tutulacak. | Orta |
| LC-003 | Simülasyon/test modu için ayrı launch opsiyonu olacak. | Orta |
| LC-004 | Gerçek araç modunda güvenlik uyarısı bulunacak. | Yüksek |

## 7. Veri kayıt gereksinimleri

| Kod | Gereksinim | Öncelik |
|---|---|---|
| LOG-001 | Görev durum logları tutulacak. | Yüksek |
| LOG-002 | Telemetri kayıtları saklanacak. | Yüksek |
| LOG-003 | Görüntü işleme/detection kayıtları saklanacak. | Yüksek |
| LOG-004 | Costmap/engel haritası çıktısı saklanacak. | Orta |
| LOG-005 | Yarışma sonrası teslim için dosyalar düzenli klasörde tutulacak. | Yüksek |

## 8. Güvenlik gereksinimleri

| Kod | Gereksinim | Öncelik |
|---|---|---|
| SAFE-001 | Motor komutu gerçek araçta manuel onaysız gönderilmeyecek. | Kritik |
| SAFE-002 | Arm/disarm işlemi manuel onay gerektirecek. | Kritik |
| SAFE-003 | GPS/telemetri kaybında güvenli duruma geçilecek. | Kritik |
| SAFE-004 | Batarya düşük seviyesinde görev durdurma önerilecek. | Kritik |
| SAFE-005 | Kamera veya perception kapanırsa mission manager uyarı verecek. | Yüksek |
| SAFE-006 | Sıvı teması/aşırı ısınma gibi durumlarda acil durdurma mantığı bulunacak. | Kritik |

## 9. Geliştirme ilkesi

Önce iskelet, sonra entegrasyon:

```text
1. Sahte veriyle çalışan node iskeleti
2. Topic/service bağlantısı
3. Launch dosyası
4. Build ve basit test
5. Gerçek sensör bağlantısı
6. Gerçek Pixhawk/MAVROS
7. Gerçek YOLO/Hailo
8. Araç üstü test
```
