# LODOS Albatros ROS2 Interface

Bu dosya LODOS Albatros ROS2 projesinin ortak node ve topic sözleşmesidir.

Bu projede tek ROS2 paketi kullanılacaktır:

- Paket adı: `albatros_system`
- Node dosyalarının yolu: `src/albatros_system/albatros_system/`

Yeni ROS2 paketi oluşturulmayacaktır.

Bu dosyadaki node isimleri, topic isimleri ve mesaj tipleri ekip tarafından değiştirilmemelidir.

Her geliştirici ve her AI aracı önce bu dosyayı okumalıdır.

---

# 1. Kullanılacak Node Listesi

Projede yalnızca aşağıdaki node dosyaları kullanılacaktır:

- `kamera_node.py`
- `yolo_node.py`
- `imu_sensor_node.py`
- `gps_sensor_node.py`
- `state_node.py`
- `mavros_node.py`
- `costmap_node.py`
- `gorev_node.py`
- `komut_node.py`
- `karar_node.py`

Ekstra node oluşturulmayacaktır.

Kullanılmayacak yapılar:

- `data_logger_node.py` kullanılmayacak.
- `mesafe_sensor_node.py` kullanılmayacak.
- `albatros_interfaces` gibi ayrı interface paketi oluşturulmayacak.
- Custom `.msg` paketi şimdilik oluşturulmayacak.

Tahtadaki mimari esas alınmıştır.

---

# 2. Yazılım İsterleri

Yarışma sonunda üretilecek / kaydedilecek temel yazılım çıktıları:

1. Costmap / engel haritası
2. YOLO işlenmiş kamera videosu
3. Araç telemetri verisi

Bu çıktılar ayrı bir data logger node ile değil, mevcut node'ların ürettiği veriler üzerinden alınacaktır.

---

# 3. Kamera Node

Dosya adı:

- `kamera_node.py`

Node adı:

- `camera_node`

Görevi:

- Kameradan ham görüntü almak.
- Görüntüyü ROS2 topic'i olarak yayınlamak.
- YOLO node'unun kullanacağı ham kamera verisini üretmek.

Kamera node içinde kullanılan parametreler:

- `camera_index`
- `frame_width`
- `frame_height`
- `fps`

Varsayılan değerler:

- `camera_index`: `0`
- `frame_width`: `640`
- `frame_height`: `480`
- `fps`: `30.0`

Publish eder:

- Topic: `/albatros/kamera/image_raw`
- Type: `sensor_msgs/msg/Image`

Mesaj bilgileri:

- Encoding: `bgr8`
- Header stamp: ROS zamanı ile doldurulur.
- Frame ID: `camera_frame`

Kamera node davranışı:

- OpenCV ile kamera açılır.
- `cv2.VideoCapture(camera_index)` kullanılır.
- Görüntü boyutu ve FPS parametrelerden alınır.
- Görüntü `cv_bridge` ile `sensor_msgs/msg/Image` formatına çevrilir.
- Görüntü sağ-sol terslik için `cv2.flip(frame, 1)` ile çevrilir.
- Kamera açılamazsa hata mesajı verilir.
- Kameradan frame alınamazsa uyarı mesajı verilir.

Kamera node içinde publisher topic'i kesinlikle şu olmalıdır:

- `/albatros/kamera/image_raw`

Eski topic olan `/camera/image_raw` kullanılmayacaktır.

---

# 4. YOLO Node

Dosya adı:

- `yolo_node.py`

Görevi:

- Kamera node'dan gelen ham görüntüyü almak.
- YOLO modeli ile duba, hedef ve engel tespiti yapmak.
- Tespit sonuçlarını yayınlamak.
- İşlenmiş kamera görüntüsünü yayınlamak.

Subscribe eder:

- Topic: `/albatros/kamera/image_raw`
- Type: `sensor_msgs/msg/Image`

Publish eder:

- Topic: `/albatros/yolo/tespitler`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/kamera/processed`
- Type: `sensor_msgs/msg/Image`

Tespit formatı:

`/albatros/yolo/tespitler` topic'i geçici olarak `std_msgs/msg/String` ile JSON benzeri metin formatında yayınlanabilir.

Örnek içerik:

{
  "detections": [
    {
      "class": "sari_duba",
      "confidence": 0.86,
      "bbox": [120, 50, 200, 180]
    }
  ]
}

Notlar:

- `/albatros/kamera/processed` yarışma sonunda istenen işlenmiş kamera videosu için kullanılacaktır.
- YOLO node doğrudan karar vermez.
- YOLO node sadece algılama sonucunu üretir.

---

# 5. IMU Sensör Node

Dosya adı:

- `imu_sensor_node.py`

Görevi:

- IMU verisini almak.
- State node'un kullanacağı IMU topic'ini yayınlamak.

Publish eder:

- Topic: `/albatros/imu/data`
- Type: `sensor_msgs/msg/Imu`

Notlar:

- IMU verisi doğrudan sensörden veya MAVROS üzerinden alınabilir.
- Sistemin diğer node'ları IMU verisini bu standart topic üzerinden okuyacaktır.

---

# 6. GPS Sensör Node

Dosya adı:

- `gps_sensor_node.py`

Görevi:

- GPS verisini almak.
- State node'un kullanacağı GPS topic'ini yayınlamak.

Publish eder:

- Topic: `/albatros/gps/fix`
- Type: `sensor_msgs/msg/NavSatFix`

Notlar:

- GPS verisi doğrudan GPS modülünden veya MAVROS üzerinden alınabilir.
- Sistemin diğer node'ları GPS verisini bu standart topic üzerinden okuyacaktır.

---

# 7. State Node

Dosya adı:

- `state_node.py`

Görevi:

- GPS, IMU ve MAVROS üzerinden gelen araç durum bilgilerini toplamak.
- Aracın genel durumunu yayınlamak.
- Araç telemetri verisi için temel state bilgisini oluşturmak.

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
- Type: `std_msgs/msg/String`

State içeriği:

`/albatros/state` topic'i araç durumunu taşıyacaktır.

İçermesi beklenen bilgiler:

- Araç modu
- Arm / disarm durumu
- GPS latitude
- GPS longitude
- Heading / yönelim
- Batarya durumu
- IMU'dan gelen temel yönelim bilgisi

Örnek içerik:

{
  "mode": "GUIDED",
  "armed": false,
  "lat": 40.123456,
  "lon": 29.123456,
  "heading": 90.0,
  "battery": 12.1
}

Notlar:

- Araç telemetri verisi için temel kaynak `state_node.py` olacaktır.
- Daha sonra ihtiyaç olursa bu veri CSV formatında kaydedilebilir.

---

# 8. MAVROS Node

Dosya adı:

- `mavros_node.py`

Görevi:

- MAVROS ile Pixhawk bağlantısını sistem içinde takip etmek.
- MAVROS topic ve servislerinin kullanımını düzenlemek.
- MAVROS bağlantısının çalıştığını kontrol etmek.

Kullanılacak MAVROS topicleri:

- `/mavros/state`
- `/mavros/global_position/global`
- `/mavros/imu/data`
- `/mavros/battery`
- `/mavros/setpoint_velocity/cmd_vel_unstamped`

Kullanılacak MAVROS servisleri:

- `/mavros/cmd/arming`
- `/mavros/set_mode`

Notlar:

- MAVROS hazır ROS2-MAVLink köprüsüdür.
- Pixhawk ile gerçek haberleşmeyi MAVROS sağlar.
- `mavros_node.py`, MAVROS'un kendisini yeniden yazmaz.
- Gerekirse bağlantı kontrolü, durum kontrolü veya yardımcı köprü mantığı için kullanılır.
- Pixhawk'a gidecek son hareket komutları `komut_node.py` üzerinden gönderilecektir.

---

# 9. Costmap Node

Dosya adı:

- `costmap_node.py`

Görevi:

- YOLO tespitleri ve araç state bilgisini kullanarak costmap / engel haritası oluşturmak.
- Yarışma sonunda istenen costmap çıktısını üretmek.

Subscribe eder:

- Topic: `/albatros/yolo/tespitler`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/state`
- Type: `std_msgs/msg/String`

Publish eder:

- Topic: `/albatros/costmap`
- Type: `nav_msgs/msg/OccupancyGrid`

Notlar:

- Costmap, engel haritası olarak kullanılacaktır.
- İlk aşamada basit bir lokal harita mantığıyla başlanabilir.
- Daha sonra gerçek koordinat dönüşümleri ve detaylı engel konumlandırma eklenebilir.

---

# 10. Görev / Mission Node

Dosya adı:

- `gorev_node.py`

Görevi:

- Görev akışını yönetmek.
- Parkur-1, Parkur-2 ve Parkur-3 geçişlerini kontrol etmek.
- Görev başlangıç, görev bitiş ve acil durdurma durumlarını yönetmek.

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
- Type: `std_msgs/msg/String`

- Topic: `/albatros/costmap`
- Type: `nav_msgs/msg/OccupancyGrid`

Publish eder:

- Topic: `/albatros/gorev/state`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/gorev/current_goal`
- Type: `sensor_msgs/msg/NavSatFix`

- Topic: `/albatros/gorev/target_color`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/emergency_stop`
- Type: `std_msgs/msg/Bool`

Start komutu:

- `ros2 topic pub --once /albatros/gorev/start std_msgs/msg/String "{data: 'START'}"`

Stop komutu:

- `ros2 topic pub --once /albatros/gorev/stop std_msgs/msg/String "{data: 'STOP'}"`

Notlar:

- Parkurlar arası geçiş kullanıcı müdahalesi olmadan otomatik yapılmalıdır.
- Görev node karar node'a doğrudan hız komutu göndermez.
- Görev node sadece görev durumunu ve hedef bilgisini yönetir.

---

# 11. Karar Node

Dosya adı:

- `karar_node.py`

Görevi:

- Görev durumuna, costmap'e, YOLO tespitlerine ve araç state bilgisine göre aracın ne yapacağına karar vermek.
- Komut node'a hareket kararı üretmek.

Subscribe eder:

- Topic: `/albatros/gorev/state`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/gorev/current_goal`
- Type: `sensor_msgs/msg/NavSatFix`

- Topic: `/albatros/gorev/target_color`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/state`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/costmap`
- Type: `nav_msgs/msg/OccupancyGrid`

- Topic: `/albatros/yolo/tespitler`
- Type: `std_msgs/msg/String`

Publish eder:

- Topic: `/albatros/karar/cmd_vel`
- Type: `geometry_msgs/msg/Twist`

Notlar:

- Karar node doğrudan Pixhawk'a komut göndermez.
- Sadece komut node'a hareket kararı üretir.
- Engelden kaçma, hedefe yönelme, duba ortalama gibi kararlar bu node içinde verilir.

---

# 12. Komut Node

Dosya adı:

- `komut_node.py`

Görevi:

- Karar node'dan gelen hareket komutunu almak.
- Güvenlik kontrolünden geçirmek.
- MAVROS üzerinden Pixhawk'a iletmek.

Subscribe eder:

- Topic: `/albatros/karar/cmd_vel`
- Type: `geometry_msgs/msg/Twist`

- Topic: `/albatros/gorev/state`
- Type: `std_msgs/msg/String`

- Topic: `/albatros/emergency_stop`
- Type: `std_msgs/msg/Bool`

MAVROS'a gönderir:

- Topic: `/mavros/setpoint_velocity/cmd_vel_unstamped`
- Type: `geometry_msgs/msg/Twist`

Güvenlik kuralları:

- `/albatros/emergency_stop` true olursa motor komutu sıfırlanacaktır.
- Görev durumu `EMERGENCY_STOP` olursa motor komutu sıfırlanacaktır.
- Karar node'dan belirli süre yeni komut gelmezse motor komutu sıfırlanacaktır.
- Pixhawk'a gönderilecek son komut bu node üzerinden geçmelidir.

---

# 13. Takım Kuralı

Her AI aracı önce bu dosyayı okumalıdır.

AI araçları:

- Yeni ROS2 paketi oluşturmayacaktır.
- Bu listedeki node'lar dışında yeni node oluşturmayacaktır.
- Topic isimlerini değiştirmeyecektir.
- Mesaj tiplerini değiştirmeyecektir.
- Mevcut çalışan node dosyalarını izinsiz değiştirmeyecektir.
- Kamera node için referans topic `/albatros/kamera/image_raw` olacaktır.