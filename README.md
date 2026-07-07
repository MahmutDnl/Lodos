# LODOS Albatros ROS2 Workspace

Bu repository, TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması için LODOS takımının **Albatros** adlı İDA yazılım çalışma alanıdır.

## Amaç

Albatros’un otonom görevleri yerine getirebilmesi için ROS2 tabanlı modüler bir yazılım altyapısı kurmak:

- Görev başlatma ve görev durum yönetimi
- Parkur-1 nokta takibi
- Parkur-2 engel algılama ve kaçınma
- Parkur-3 hedef renk/kamikaze angajmanı
- Pixhawk/MAVROS haberleşmesi
- YOLO tabanlı duba/hedef algılama
- Güvenli durdurma ve veri kayıt hazırlığı

## Hedef sistem

```text
Ubuntu 24.04
ROS2 Jazzy
Python 3
Raspberry Pi 5
Hailo AI Kit
Pixhawk 2.4.8
MAVROS / MAVLink
YOLO / OpenCV
```

## Klasör yapısı

```text
lodos_ws/
├── AGENTS.md
├── PROJECT_CONTEXT.md
├── TASKS.md
├── README.md
├── docs/
│   ├── mission_flow.md
│   ├── system_architecture.md
│   ├── software_requirements.md
│   ├── competition_rules_summary.md
│   ├── ros2_design.md
│   └── hardware_notes.md
├── prompts/
│   └── antigravity_initial_prompt.txt
├── scripts/
└── src/
```

## İlk kurulum

ROS2 Jazzy ortamını etkinleştir:

```bash
source /opt/ros/jazzy/setup.bash
```

Workspace klasörüne git:

```bash
cd ~/teknofest/lodos_ws
```

Build al:

```bash
colcon build
source install/setup.bash
```

## Antigravity ile çalışma

Antigravity açılırken grafik sorunu olursa:

```bash
antigravity --disable-gpu --ozone-platform=x11 ~/teknofest/lodos_ws
```

Agent’a ilk olarak şu komut verilebilir:

```text
Önce AGENTS.md, PROJECT_CONTEXT.md ve TASKS.md dosyalarını oku. Bu proje TEKNOFEST LODOS Albatros İDA ROS2 workspace’idir. Terminal komutu çalıştırmadan önce benden onay iste. Hiçbir dosyayı silme. Önce plan çıkar, sonra dosya değişikliği öner.
```

## Geliştirme sırası

1. `lodos_mission_manager` paketi
2. `lodos_perception` paketi
3. `lodos_mavros_control` paketi
4. `lodos_obstacle_avoidance` paketi
5. `lodos_launch` paketi
6. Basit simülasyon/sahte veri testleri
7. Gerçek kamera/YOLO testi
8. Pixhawk/MAVROS entegrasyon testi
9. Araç üstü güvenli test

## GitHub kullanımı

Önerilen temel akış:

```bash
git status
git add .
git commit -m "docs: add lodos project context"
git push
```

Build çıktıları, dataset zipleri ve model ağırlıkları GitHub’a eklenmemelidir. Bunun için `.gitignore` dosyası hazırlanmıştır.

## Önemli güvenlik notu

Gerçek araç bağlıyken Antigravity’ye kontrolsüz şekilde terminal komutu çalıştırma yetkisi verilmemelidir. Özellikle arm/disarm, motor komutu, mode değişimi, `sudo` ve dosya silme komutları manuel onay gerektirir.
