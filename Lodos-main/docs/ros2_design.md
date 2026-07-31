# ROS2 Tasarım Notları — LODOS Albatros

Bu belge ROS2 paketleri, node’lar, topic/service yapısı ve geliştirme stratejisini tanımlar.

## 1. Workspace

```text
~/teknofest/lodos_ws
```

Temel yapı:

```text
lodos_ws/
└── src/
    ├── lodos_mission_manager/
    ├── lodos_perception/
    ├── lodos_mavros_control/
    ├── lodos_obstacle_avoidance/
    └── lodos_launch/
```

## 2. Paket oluşturma komutları

Mission manager örneği:

```bash
cd ~/teknofest/lodos_ws/src
ros2 pkg create lodos_mission_manager --build-type ament_python --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

Perception örneği:

```bash
ros2 pkg create lodos_perception --build-type ament_python --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

MAVROS control örneği:

```bash
ros2 pkg create lodos_mavros_control --build-type ament_python --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

Obstacle avoidance örneği:

```bash
ros2 pkg create lodos_obstacle_avoidance --build-type ament_python --dependencies rclpy std_msgs geometry_msgs
```

Launch paketi örneği:

```bash
ros2 pkg create lodos_launch --build-type ament_python --dependencies rclpy launch launch_ros
```

## 3. Node önerileri

### `mission_node.py`

Paket:

```text
lodos_mission_manager
```

Görev:

- Mission state tutar.
- Start/stop komutlarını alır.
- Parkur geçişlerini yönetir.
- Hata durumlarını yakalar.

Topic/service:

```text
Sub: /mission/start
Sub: /mission/stop
Pub: /mission/state
Sub: /perception/detections
Sub: /vehicle/status
Pub: /mission/current_goal
```

### `detection_node.py`

Paket:

```text
lodos_perception
```

Görev:

- Kamera görüntüsünü alır.
- YOLO/Hailo inference çalıştırır.
- Detection sonuçlarını yayınlar.

Topic:

```text
Sub: /perception/camera/image_raw
Pub: /perception/detections
Pub: /perception/target_color
```

### `mavros_bridge_node.py`

Paket:

```text
lodos_mavros_control
```

Görev:

- MAVROS topic’lerini okur.
- Araç durumunu sadeleştirir.
- GPS/IMU bilgisini mission manager’a uygun hale getirir.

Topic:

```text
Sub: /mavros/state
Sub: /mavros/global_position/global
Sub: /mavros/imu/data
Pub: /vehicle/status
Pub: /navigation/gps
Pub: /navigation/heading
```

### `obstacle_node.py`

Paket:

```text
lodos_obstacle_avoidance
```

Görev:

- Detection verisinden engel durumunu çıkarır.
- Basit kaçınma komutu üretir.

Topic:

```text
Sub: /perception/detections
Pub: /obstacle_avoidance/cmd
Pub: /perception/obstacle_status
```

## 4. İlk aşama veri modeli

Özel mesaj yazmadan önce basit JSON string kullanılabilir. Örnek detection mesajı:

```json
{
  "stamp": "time",
  "detections": [
    {
      "class_name": "sari_duba",
      "confidence": 0.86,
      "bbox": [120, 80, 220, 260],
      "center": [170, 170],
      "zone": "danger"
    }
  ]
}
```

Daha sonra özel ROS2 message tanımı yapılabilir:

```text
Detection.msg
DetectionArray.msg
MissionState.msg
VehicleStatus.msg
```

## 5. Mission state mesajı

İlk aşamada `std_msgs/String`:

```text
IDLE
READY
MISSION_LOADED
ARMED
PARKUR_1
PARKUR_2
PARKUR_3
FINISHED
ERROR
```

Daha sonra özel mesaj:

```text
string state
string current_parkur
int32 current_waypoint_index
bool is_armed
bool has_error
string error_message
```

## 6. Basit mission manager mantığı

```python
if state == "IDLE" and start_received:
    state = "SYSTEM_CHECK"

if state == "SYSTEM_CHECK" and all_systems_ok:
    state = "READY"

if state == "READY" and mission_loaded:
    state = "MISSION_LOADED"

if state == "MISSION_LOADED" and arm_confirmed:
    state = "PARKUR_1"

if state == "PARKUR_1" and parkur1_done:
    state = "PARKUR_2"

if state == "PARKUR_2" and parkur2_done:
    state = "PARKUR_3"

if state == "PARKUR_3" and parkur3_done:
    state = "FINISHED"
```

## 7. Build komutları

```bash
cd ~/teknofest/lodos_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## 8. Test yaklaşımı

İlk testler gerçek donanım olmadan yapılmalı:

- `mission_node` tek başına çalışıyor mu?
- `/mission/state` yayınlanıyor mu?
- Sahte `/mission/start` gönderince durum değişiyor mu?
- Sahte detection ile obstacle avoidance tepki veriyor mu?
- Launch dosyası node’ları ayağa kaldırıyor mu?

Örnek test:

```bash
ros2 topic echo /mission/state
ros2 topic pub /mission/start std_msgs/msg/String "{data: 'start'}"
```

## 9. Gerçek donanım entegrasyon sırası

```text
1. ROS2 node iskeleti
2. Sahte veri testleri
3. Kamera bağlantısı
4. YOLO test
5. Hailo AI Kit optimizasyonu
6. MAVROS state okuma
7. GPS/IMU okuma
8. Manuel güvenli motor testi
9. Parkur senaryosu testi
```
