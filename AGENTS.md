# AGENTS.md — LODOS / Albatros Antigravity Çalışma Kuralları

Bu dosya Antigravity Agent için proje hafızası ve güvenli çalışma kurallarıdır. Yeni bir görevde önce bu dosya, sonra `PROJECT_CONTEXT.md`, `TASKS.md` ve `docs/` klasörü okunmalıdır.

## 1. Proje kimliği

- Yarışma: TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması
- Takım: LODOS
- Araç adı: Albatros
- Araç türü: İnsansız Deniz Aracı (İDA)
- Ana hedef: Nokta takibi, engel algılama/kaçınma ve kamikaze angajman görevlerini otonom olarak tamamlayan ROS2 tabanlı su üstü aracı geliştirmek.

## 2. Projenin temel teknik yönü

Albatros; trimaran gövde yapısına sahip, diferansiyel itki ile yönlendirilen, Pixhawk + Raspberry Pi + Hailo AI Kit + kamera/sensörler ile çalışan otonom bir İDA’dır.

Ana yazılım omurgası ROS2’dir. Ubuntu 24.04 kullanıldığı için ROS2 dağıtımı olarak varsayılan hedef **Jazzy** kabul edilmelidir. Önceki raporlarda ROS2 genel ifade veya Humble geçmişi geçebilir; ancak bu çalışma alanında Ubuntu 24.04/Jazzy uyumluluğu korunmalıdır.

## 3. Ana donanım varsayımları

- Ana bilgisayar: Raspberry Pi 5
- Yapay zeka hızlandırıcı: Hailo AI Kit
- Otopilot/kontrol kartı: Pixhawk 2.4.8
- Haberleşme: MAVLink, MAVROS, 3DR 915 MHz telemetri
- Algılama: Kamera, GPS, IMU, pusula, mesafe sensörleri
- İtki: Sağ ve sol ama gövdelerinde iki elektrik motoru
- Yönlendirme: Dümen yerine diferansiyel itki
- Gövde: Trimaran, 3B yazıcı + cam elyaf/epoksi kompozit güçlendirme

## 4. Yazılım paketleri için hedef yapı

Önerilen ROS2 workspace yapısı:

```text
lodos_ws/
├── AGENTS.md
├── PROJECT_CONTEXT.md
├── TASKS.md
├── README.md
├── docs/
├── prompts/
├── scripts/
└── src/
    ├── lodos_mission_manager/
    ├── lodos_perception/
    ├── lodos_mavros_control/
    ├── lodos_obstacle_avoidance/
    └── lodos_launch/
```

### Paket sorumlulukları

#### `lodos_mission_manager`
Görev akışını ve görev durum makinesini yönetir. Parkur geçişleri, görev başlatma, görev sonlandırma, hata durumu ve güvenli durdurma kararları burada tutulur.

#### `lodos_perception`
Kamera görüntüsünü alır, YOLO tabanlı duba/hedef tespiti yapar, tespit sonuçlarını ROS2 topic olarak yayınlar. Hailo AI Kit entegrasyonu bu paketin ileri aşamasında ele alınır.

#### `lodos_mavros_control`
Pixhawk ile MAVROS üzerinden haberleşir. Araç durumu, GPS, IMU, arm/disarm, mode değiştirme ve hız/konum komutları bu katmanda yönetilir.

#### `lodos_obstacle_avoidance`
Algılanan sınır ve engel dubalarına göre kaçınma davranışı üretir. Parkur-2 için özellikle önemlidir.

#### `lodos_launch`
Sistemi tek komutla ayağa kaldıracak launch dosyalarını içerir.

## 5. Görev durum makinesi önerisi

Mission manager içinde ilk aşamada şu durumlar kullanılabilir:

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

Durumlar önce simülasyon/terminal testleriyle doğrulanmalı, gerçek araç üzerinde denenmeden önce takım onayı alınmalıdır.

## 6. Önerilen topic, service ve veri akış adları

Topic adları sade, tutarlı ve küçük harfli olmalıdır.

```text
/mission/start
/mission/stop
/mission/state
/mission/waypoints
/mission/current_goal

/perception/camera/image_raw
/perception/detections
/perception/target_color
/perception/obstacle_status

/navigation/gps
/navigation/heading
/navigation/ekf_pose
/navigation/target_error

/obstacle_avoidance/cmd
/vehicle/cmd_vel
/vehicle/status
/vehicle/battery

/mavros/state
/mavros/global_position/global
/mavros/imu/data
/mavros/setpoint_velocity/cmd_vel_unstamped
```

İlk prototipte özel mesaj tanımı zorunlu değilse `std_msgs`, `geometry_msgs`, `sensor_msgs` ile başlanabilir. Sistem oturdukça özel mesajlar eklenebilir.

## 7. Kod yazım kuralları

- Önce mevcut dosya yapısını oku.
- Kod yazmadan önce kısa plan çıkar.
- Hangi dosyaların oluşturulacağını/değişeceğini açıkça söyle.
- Terminal komutu çalıştırmadan önce kullanıcıdan onay iste.
- Hiçbir dosyayı silme.
- `rm -rf`, `sudo rm`, `git reset --hard`, `git clean -fd`, `chmod -R 777` gibi riskli komutları çalıştırma.
- Gerçek araca bağlıyken arm/disarm veya motor komutu çalıştırma.
- ROS2 node’ları basit, okunabilir ve yorumlu yazılsın.
- Her node için README veya docs içinde kısa açıklama bulunsun.
- Geçici dosyalar, model ağırlıkları, dataset zipleri, build çıktıları GitHub’a eklenmesin.

## 8. Build ve test kuralları

Ubuntu 24.04 / ROS2 Jazzy için temel komutlar:

```bash
cd ~/teknofest/lodos_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Her önemli değişiklikten sonra önerilecek kontroller:

```bash
git status
colcon build
ros2 pkg list | grep lodos
```

Node çalıştırma örneği:

```bash
ros2 run lodos_mission_manager mission_node
```

Launch örneği:

```bash
ros2 launch lodos_launch mission.launch.py
```

## 9. GitHub kuralları

- `main`: temiz ve çalışır kabul edilen sürüm
- `dev`: geliştirme ve deneme sürümü
- Küçük ekipte karmaşık branch sistemi gerekmez; gerekirse `feature/mission-manager`, `feature/perception-yolo` gibi branch açılır.

Commit mesajı örnekleri:

```text
feat: add mission manager state machine
feat: add perception detection publisher
fix: correct mavros topic mapping
docs: update mission flow
chore: add ros2 gitignore
```

## 10. Güvenlik ve yarışma riski notları

- Gerçek araç bağlıyken motor komutu, arm/disarm ve mode değişimi kontrolsüz çalıştırılmamalıdır.
- Yarışma günü haberleşme frekansları ve veri teslim kuralları şartnameye göre kontrol edilmelidir.
- Kamera, GPS, IMU, telemetri ve batarya durumu sistem başlamadan önce ayrı ayrı doğrulanmalıdır.
- Görev sonunda telemetri, görüntü işleme çıktısı ve engel haritası/costmap verileri düzenli biçimde saklanmalıdır.

## 11. Antigravity’ye verilecek standart başlangıç talimatı

Yeni oturumda Agent’a şu talimat verilebilir:

```text
Önce AGENTS.md, PROJECT_CONTEXT.md ve TASKS.md dosyalarını oku. Bu proje TEKNOFEST LODOS Albatros İDA ROS2 workspace’idir. Terminal komutu çalıştırmadan önce benden onay iste. Hiçbir dosyayı silme. Önce plan çıkar, sonra dosya değişikliği öner.
```
