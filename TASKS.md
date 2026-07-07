# TASKS.md — LODOS Albatros Yazılım Yol Haritası

Bu dosya yapılacak işleri öncelik sırasına göre tutar. Antigravity her görev başlangıcında bu dosyayı kontrol etmelidir.

## Öncelik 0 — Proje düzeni

- [ ] `src/` klasörünün varlığını kontrol et.
- [ ] `.gitignore` dosyasını ekle.
- [ ] `AGENTS.md`, `PROJECT_CONTEXT.md`, `README.md`, `docs/` dosyalarını repo içinde tut.
- [ ] GitHub bağlantısını yap.
- [ ] İlk commit’i oluştur.

Önerilen commit:

```text
docs: add initial lodos project documentation
```

## Öncelik 1 — ROS2 workspace doğrulama

- [ ] Ubuntu 24.04 üzerinde ROS2 Jazzy kurulu mu kontrol et.
- [ ] `source /opt/ros/jazzy/setup.bash` çalışıyor mu kontrol et.
- [ ] `colcon` kurulu mu kontrol et.
- [ ] Boş workspace build alınabiliyor mu kontrol et.

Komutlar:

```bash
ls /opt/ros
source /opt/ros/jazzy/setup.bash
ros2 --version
colcon build
```

## Öncelik 2 — Mission manager paketi

Paket adı:

```text
lodos_mission_manager
```

Yapılacaklar:

- [ ] Ament Python ROS2 paketi oluştur.
- [ ] `mission_node.py` oluştur.
- [ ] Basit görev durum makinesi ekle.
- [ ] `/mission/start` topic/service dinle.
- [ ] `/mission/state` topic’i yayınla.
- [ ] Terminalden test edilebilir hale getir.
- [ ] README içine açıklama ekle.

İlk durumlar:

```text
IDLE
SYSTEM_CHECK
READY
MISSION_LOADED
ARMED
PARKUR_1
PARKUR_2
PARKUR_3
FINISHED
ERROR
EMERGENCY_STOP
```

## Öncelik 3 — Perception paketi

Paket adı:

```text
lodos_perception
```

Yapılacaklar:

- [ ] Kamera görüntüsü için subscriber taslağı oluştur.
- [ ] YOLO inference için placeholder fonksiyon yaz.
- [ ] `/perception/detections` topic’i yayınla.
- [ ] İlk aşamada gerçek model zorunlu olmasın; sahte detection test edilebilsin.
- [ ] Duba sınıfları sabit bir config içinde tutulsun.

Sınıflar:

```text
turuncu_duba
sari_duba
siyah_duba
kirmizi_duba
yesil_duba
```

## Öncelik 4 — MAVROS control paketi

Paket adı:

```text
lodos_mavros_control
```

Yapılacaklar:

- [ ] MAVROS state subscriber oluştur.
- [ ] GPS ve IMU topic’lerini dinleme taslağı oluştur.
- [ ] Arm/disarm fonksiyonlarını şimdilik güvenlik nedeniyle doğrudan çalıştırma; sadece taslak hazırla.
- [ ] `/vehicle/status` yayınla.
- [ ] Gerçek araç bağlı değilken sahte test modu ekle.

## Öncelik 5 — Obstacle avoidance paketi

Paket adı:

```text
lodos_obstacle_avoidance
```

Yapılacaklar:

- [ ] `/perception/detections` dinle.
- [ ] Sarı engel dubası tehlikeli bölgede mi kontrol et.
- [ ] Basit kaçınma kararı üret.
- [ ] `/obstacle_avoidance/cmd` topic’i yayınla.
- [ ] Parkur-2 akışı için mission manager ile bağlantı tasarla.

## Öncelik 6 — Launch paketi

Paket adı:

```text
lodos_launch
```

Yapılacaklar:

- [ ] `mission.launch.py` oluştur.
- [ ] Mission manager, perception, mavros control ve obstacle avoidance node’larını tek komutla başlat.
- [ ] Parametre dosyaları için `config/` yapısı öner.

## Öncelik 7 — Belgeler ve KTR desteği

- [ ] `docs/mission_flow.md` güncel tutulacak.
- [ ] `docs/system_architecture.md` güncel tutulacak.
- [ ] `docs/software_requirements.md` güncel tutulacak.
- [ ] Her yazılım değişikliği KTR’de anlatılabilecek kadar açık yazılacak.
- [ ] KTR için “TYF/TYR sonrası değişiklikler” notları ayrıca tutulacak.

## Öncelik 8 — Test stratejisi

- [ ] Gerçek donanım yokken sahte veri ile node testleri.
- [ ] Kamera yokken örnek görsel/video ile YOLO testi.
- [ ] Pixhawk yokken MAVROS topic mock/test.
- [ ] Gerçek araç üzerinde manuel onaylı test.
- [ ] Görev sonunda log/görüntü/costmap teslim kontrolü.

## Yapılmayacaklar / dikkat

- [ ] Gerçek araca bağlıyken otomatik arm/disarm yapılmayacak.
- [ ] `rm -rf`, `git reset --hard`, `git clean -fd` komutları kullanılmayacak.
- [ ] Dataset zipleri ve model ağırlıkları GitHub’a atılmayacak.
- [ ] Agent terminal komutu çalıştırmadan önce kullanıcıdan onay isteyecek.
