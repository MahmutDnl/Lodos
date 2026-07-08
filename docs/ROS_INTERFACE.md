# LODOS Albatros ROS2 Interface

Bu dosya LODOS Albatros ROS2 projesinin ortak haberleşme sözleşmesidir.

Bu dosyadaki node isimleri, topic isimleri ve mesaj tipleri ekip tarafından değiştirilmemelidir.

Her geliştirici ve her AI aracı önce bu dosyayı okumalıdır.

---

# 1. Paket Yapısı

Ana yazılım node'ları mevcut pakette kalacaktır:

- Paket adı: `albatros_system`
- Node dosyalarının yolu: `src/albatros_system/albatros_system/`

Custom mesajlar için ayrı bir interface paketi kullanılacaktır:

- Interface paket adı: `albatros_interfaces`
- Interface paket yolu: `src/albatros_interfaces/`

`albatros_interfaces` paketi yalnızca `.msg` dosyaları içerecektir. Bu paket içinde node yazılmayacaktır.

---

# 2. Node Listesi

Kullanılacak node dosyaları:

- `kamera_node.py`
- `yolo_node.py`
- `mesafe_sensor_node.py`
- `gps_node.py`
- `imu_node.py`
- `state_node.py`
- `costmap_node.py`
- `gorev_node.py`
- `karar_node.py`
- `komut_node.py`
- `data_logger_node.py`

MAVROS ayrı bir takım node'u olarak yazılmayacaktır. MAVROS hazır ROS2-MAVLink köprüsü olarak kullanılacaktır.

- `gps_node.py` MAVROS GPS topicini okuyacaktır.
- `imu_node.py` MAVROS IMU topicini okuyacaktır.
- `state_node.py` GPS, IMU, MAVROS state ve batarya verilerini birleştirecektir.
- `komut_node.py` MAVROS topic ve servislerine komut gönderecektir.

---

# 3. Custom Mesajlar

Custom mesajlar `albatros_interfaces` paketi içinde tanımlanacaktır.

Oluşturulacak mesajlar:

- `Detection.msg`
- `DetectionArray.msg`
- `VehicleState.msg`
- `Obstacle.msg`
- `ObstacleArray.msg`
- `MissionStatus.msg`

Tüm custom mesajlarda zaman bilgisi için `std_msgs/Header header` alanı kullanılmalıdır.

---

# 4. Kamera Node

Dosya adı:

- `kamera_node.py`

Publish eder:

- Topic: `/albatros/kamera/image_raw`
- Type: `sensor_msgs/msg/Image`

Açıklama:

Kameradan ham görüntü alır ve ROS2 topicine yayınlar.

---

# 5. YOLO Node

Dosya adı:

- `yolo_node.py`

Subscribe eder:

- Topic: `/albatros/kamera/image_raw`
- Type: `sensor_msgs/msg/Image`

Publish eder:

- Topic: `/albatros/yolo/tespitler`
- Type: `albatros_interfaces/msg/DetectionArray`

- Topic: `/albatros/kamera/annotated`
- Type: `sensor_msgs/msg/Image`

Açıklama:

Kamera görüntüsünü işler. YOLO modeli ile duba, engel ve hedef tespiti yapar.

JSON kullanılmayacaktır. Tespitler custom mesaj ile yayınlanacaktır.

---

# 6. Mesafe Sensör Node

Dosya adı:

- `mesafe_sensor_node.py`

Publish eder:

- Topic: `/albatros/mesafe/front`
- Type: `sensor_msgs/msg/Range`

- Topic: `/albatros/mesafe/left`
- Type: `sensor_msgs/msg/Range`

- Topic: `/albatros/mesafe/right`
- Type: `sensor_msgs/msg/Range`

- Topic: `/albatros/mesafe/back`
- Type: `sensor_msgs/msg/Range`

Açıklama:

Ultrasonik mesafe sensörlerinden gelen verileri yayınlar.

---

# 7. GPS Node

Dosya adı:

- `gps_node.py`

Subscribe eder:

- Topic: `/mavros/global_position/global`
- Type: `sensor_msgs/msg/NavSatFix`

Publish eder:

- Topic: `/albatros/gps/fix`
- Type: `sensor_msgs/msg/NavSatFix`

Açıklama:

MAVROS üzerinden gelen GPS bilgisini alır ve sistem içi standart GPS topicine aktarır.

---

# 8. IMU Node

Dosya adı:

- `imu_node.py`

Subscribe eder:

- Topic: `/mavros/imu/data`
- Type: `sensor_msgs/msg/Imu`

Publish eder:

- Topic: `/albatros/imu/data`
- Type: `sensor_msgs/msg/Imu`

Açıklama:

MAVROS üzerinden gelen IMU bilgisini alır ve sistem içi standart IMU topicine aktarır.

---

# 9. State Node

Dosya adı:

- `state_node.py`

Subscribe eder:

- Topic: `/albatros/gps/fix`
- Type: `sensor_msgs/msg/NavSatFix`

- Topic: `/albatros/imu/data`
- Type: `sensor_msgs/msg/Imu`

- Topic: `/mavros/state`
- Type: `mavros_msgs/msg/State`

- Topic: `/mavros/battery`
- Type: `sensor_msgs/msg/BatteryState`

Publish eder:

- Topic: `/albatros/state`
- Type: `albatros_interfaces/msg/VehicleState`

Açıklama:

Aracın genel durumunu toplar. GPS, IMU, mod, arm bilgisi, batarya ve yönelim verilerini tek bir araç durumu mesajı haline getirir.

---

# 10. Costmap Node

Dosya adı:

- `costmap_node.py`

Subscribe eder:

- Topic: `/albatros/yolo/tespitler`
- Type: `albatros_interfaces/msg/DetectionArray`

- Topic: `/albatros/mesafe/front`
- Type: `sensor_msgs/msg/Range`

- Topic: `/albatros/mesafe/left`
- Type: `sensor_msgs/msg/Range`

- Topic: `/albatros/mesafe/right`
- Type: `sensor_msgs/msg/Range`

- Topic: `/albatros/mesafe/back`
- Type: `sensor_msgs/msg/Range`

- Topic: `/albatros/state`
- Type: `albatros_interfaces/msg/VehicleState`

Publish eder:

- Topic: `/albatros/costmap`
- Type: `nav_msgs/msg/OccupancyGrid`

- Topic: `/albatros/engeller`
- Type: `albatros_interfaces/msg/ObstacleArray`

Açıklama:

YOLO tespitleri, mesafe sensörleri ve araç durumunu kullanarak lokal engel haritası üretir.

JSON kullanılmayacaktır. Costmap için `nav_msgs/msg/OccupancyGrid` kullanılacaktır.

---

# 11. Görev Node

Dosya adı:

- `gorev_node.py`

Görev durumları:

- `IDLE`
- `READY`
- `PARKUR_1`
- `PARKUR_2`
- `PARKUR_3`
- `FINISHED`
- `EMERGENCY_STOP`

Subscribe eder:

- Topic: `/albatros/gorev/start`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/gorev/stop`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/state`
- Type: `albatros_interfaces/msg/VehicleState`

- Topic: `/albatros/costmap`
- Type: `nav_msgs/msg/OccupancyGrid`

- Topic: `/albatros/engeller`
- Type: `albatros_interfaces/msg/ObstacleArray`

- Topic: `/albatros/yolo/tespitler`
- Type: `albatros_interfaces/msg/DetectionArray`

Publish eder:

- Topic: `/albatros/gorev/status`
- Type: `albatros_interfaces/msg/MissionStatus`

- Topic: `/albatros/gorev/current_goal`
- Type: `sensor_msgs/msg/NavSatFix`

- Topic: `/albatros/gorev/target_color`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/emergency_stop`
- Type: `std_msgs/msg/Bool`

Açıklama:

Parkur-1, Parkur-2 ve Parkur-3 görev akışını yönetir. Parkurlar arası geçiş kullanıcı müdahalesi olmadan otomatik yapılmalıdır.

Start komutu:

- `ros2 topic pub --once /albatros/gorev/start std_msgs/msg/String "{data: 'START'}"`

Stop komutu:

- `ros2 topic pub --once /albatros/gorev/stop std_msgs/msg/String "{data: 'STOP'}"`

---

# 12. Karar Node

Dosya adı:

- `karar_node.py`

Subscribe eder:

- Topic: `/albatros/gorev/status`
- Type: `albatros_interfaces/msg/MissionStatus`

- Topic: `/albatros/gorev/current_goal`
- Type: `sensor_msgs/msg/NavSatFix`

- Topic: `/albatros/gorev/target_color`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/state`
- Type: `albatros_interfaces/msg/VehicleState`

- Topic: `/albatros/costmap`
- Type: `nav_msgs/msg/OccupancyGrid`

- Topic: `/albatros/engeller`
- Type: `albatros_interfaces/msg/ObstacleArray`

- Topic: `/albatros/yolo/tespitler`
- Type: `albatros_interfaces/msg/DetectionArray`

Publish eder:

- Topic: `/albatros/karar/cmd_vel`
- Type: `geometry_msgs/msg/TwistStamped`

- Topic: `/albatros/karar/global_setpoint`
- Type: `mavros_msgs/msg/GlobalPositionTarget`

Açıklama:

Aracın nasıl hareket edeceğine karar verir.

`/albatros/karar/cmd_vel` lokal manevra, engelden kaçma ve kısa süreli hız komutları için kullanılır.

`/albatros/karar/global_setpoint` GPS hedefe yönelme ve waypoint mantığı için kullanılır.

Sadece `Twist` kullanmak yeterli değildir. Hem hız komutu hem global hedef komutu mimaride bulunmalıdır.

---

# 13. Komut Node

Dosya adı:

- `komut_node.py`

Subscribe eder:

- Topic: `/albatros/karar/cmd_vel`
- Type: `geometry_msgs/msg/TwistStamped`

- Topic: `/albatros/karar/global_setpoint`
- Type: `mavros_msgs/msg/GlobalPositionTarget`

- Topic: `/albatros/gorev/status`
- Type: `albatros_interfaces/msg/MissionStatus`

- Topic: `/albatros/emergency_stop`
- Type: `std_msgs/msg/Bool`

MAVROS'a gönderir:

- Topic: `/mavros/setpoint_velocity/cmd_vel`
- Type: `geometry_msgs/msg/TwistStamped`

- Topic: `/mavros/setpoint_raw/global`
- Type: `mavros_msgs/msg/GlobalPositionTarget`

MAVROS servisleri:

- `/mavros/cmd/arming`
- `/mavros/set_mode`

Güvenlik kuralları:

- `/albatros/emergency_stop` değeri `true` olursa motor komutu sıfırlanmalıdır.
- Görev durumu `EMERGENCY_STOP` olursa motor komutu sıfırlanmalıdır.
- `karar_node.py` üzerinden 0.5 saniye boyunca yeni komut gelmezse motor komutu sıfırlanmalıdır.
- Gerekirse araç HOLD moduna alınmalıdır.
- Bu watchdog mekanizması `komut_node.py` içinde bulunmalıdır.

---

# 14. Data Logger Node

Dosya adı:

- `data_logger_node.py`

Subscribe eder:

- Topic: `/albatros/kamera/annotated`
- Type: `sensor_msgs/msg/Image`

- Topic: `/albatros/state`
- Type: `albatros_interfaces/msg/VehicleState`

- Topic: `/albatros/costmap`
- Type: `nav_msgs/msg/OccupancyGrid`

- Topic: `/albatros/karar/cmd_vel`
- Type: `geometry_msgs/msg/TwistStamped`

- Topic: `/albatros/karar/global_setpoint`
- Type: `mavros_msgs/msg/GlobalPositionTarget`

- Topic: `/albatros/gorev/status`
- Type: `albatros_interfaces/msg/MissionStatus`

Açıklama:

Yarışma sonunda teslim edilecek verileri kaydeder.

Kaydedilecek veriler:

- İşlenmiş kamera görüntüsü
- Araç telemetri verisi
- Hız setpoint bilgisi
- Yön setpoint bilgisi
- Lokal harita / costmap / engel haritası
- Görev durumu

---

# 15. Genel Haberleşme Kuralları

Yüksek frekanslı topiclerde `std_msgs/msg/String` ve JSON kullanılmayacaktır.

JSON yalnızca debug, geçici test veya insan tarafından okunacak log çıktıları için kullanılabilir.

Ana haberleşmede standart ROS2 mesajları ve custom interface mesajları kullanılacaktır.

Kritik topiclerde zaman bilgisi korunmalıdır.

Timestamp gerektiren mesajlarda `std_msgs/Header header` alanı bulunmalıdır.

İleride `tf2` kullanılabilmesi için frame ve zaman bilgileri korunmalıdır.

---

# 16. AI Araçları İçin Kural

Her AI aracı önce bu dosyayı okumalıdır.

AI araçları:

- Yeni ROS2 paketi oluşturmamalıdır.
- Node dosya isimlerini değiştirmemelidir.
- Topic isimlerini değiştirmemelidir.
- Mesaj tiplerini değiştirmemelidir.
- Sadece kendisine verilen node veya dosya üzerinde çalışmalıdır.
- Mevcut çalışan node dosyalarını izinsiz değiştirmemelidir.