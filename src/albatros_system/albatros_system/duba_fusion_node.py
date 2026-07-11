#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — Duba Sensör Füzyon Node'u
# =============================================================================
# Dosya    : duba_fusion_node.py
# Node adi : duba_fusion_node
# Paket    : albatros_system
#
# Veri akisi:
#
#   yolo_node
#       |
#       v
#   yolo_mesafe_node
#       |
#       | /albatros/yolo/mesafe_aci  (std_msgs/String JSON)
#       v
#   duba_fusion_node <------- mesafe_sensor_node
#       |                        |
#       |                        | /albatros/mesafe/on_sol
#       |                        | /albatros/mesafe/on_sag
#       |                        | /albatros/mesafe/yan_sol
#       |                        | /albatros/mesafe/yan_sag
#       |
#       | /albatros/fusion/duba_konumlari  (std_msgs/String JSON)
#       v
#   costmap_node
#
# Gorev:
#   YOLO goruntu geometrisinden gelen yaklasik duba mesafesi/aci bilgisini
#   dort JSN-SR04T mesafe sensorunden gelen olcumlerle birlestirerek
#   costmap noduna gonderilecek nihai duba konumlarini uretmek.
#
#   Bu node:
#     - Motor/MAVROS komutu gondermez.
#     - Karar vermez.
#     - Costmap olusturmaz; yalnizca fusyon edilmis konum yayinlar.
#
# Koordinat standardi:
#   ROS base_link: x ileri, y sol, pozitif aci sol
#   bearing_deg > 0 → sol taraf
#   bearing_deg < 0 → sag taraf
#
# ACI DONUSUMU:
#   yolo_mesafe_node icinde:
#     angle_rad = atan2(center_x - cx, fx)
#   Bu formul kamera goruntusu uzerinde SAG tarafi POZITIF verir.
#   Oysa ROS base_link standardinda POZITIF aci SOL taraftiir.
#
#   Bu nedenle _cb_yolo() icinde:
#     bearing_deg = normalize_angle_180(-camera_bearing_deg)
#   donusumu uygulanir. Bundan sonraki TUM islemler donusturulmus
#   bearing_deg degeri uzerinden yapilir.
#
# ZAMAN ESLESTIRME:
#   JSON stamp yalnizca cikti meta verisi olarak korunur.
#   Fuzyon zaman eslestirmesi ayni ROS clock uzerinden alinan
#   callback zamanlarıyla yapilir (yolo_receive_time vs measurement_time).
#
# Yazan : LODOS Yazilim Ekibi
# Tarih : 2026
# =============================================================================

import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


# =============================================================================
# Sabitler
# =============================================================================

# Fuzyon guveni hesabi agirlik katsayilari (parametrize edilmedi; yorum gerekli)
# yolo_confidence 0.7 agirligi ile, mesafe uyum puani 0.3 agirligi ile birlestirilir.
FUSION_CONF_YOLO_WEIGHT       = 0.7
FUSION_CONF_AGREEMENT_WEIGHT  = 0.3

# Mesafe araligina gore sensor/YOLO agirlik esikleri (metre)
WEIGHT_RANGE_NEAR_MAX  = 3.0   # 0.20 – 3.00 m: sensor agir
WEIGHT_RANGE_MID_MAX   = 4.0   # 3.00 – 4.00 m: orta agirlik

# class_name → costmap semantic type eslesme tablosu
# Kamera sinif adlarindan costmap'in bekledigi tip degerlerine donusum.
# Turuncu sinir dubasi border_buoy, sari engel dubasi obstacle_buoy,
# kirmizi/yesil hedef dubasi target_buoy, Parkur-3 dogru hedefi goal_buoy.
CLASS_NAME_TO_TYPE: dict[str, str] = {
    'sari_duba':          'obstacle_buoy',
    'yellow_buoy':        'obstacle_buoy',
    'sari':               'obstacle_buoy',
    'turuncu_duba':       'border_buoy',
    'orange_buoy':        'border_buoy',
    'turuncu':            'border_buoy',
    'kirmizi_duba':       'target_buoy',
    'red_buoy':           'target_buoy',
    'kirmizi':            'target_buoy',
    'yesil_duba':         'target_buoy',
    'green_buoy':         'target_buoy',
    'yesil':              'target_buoy',
    'hedef_duba':         'goal_buoy',
    'goal_buoy':          'goal_buoy',
}

# Varsayilan duba fiziksel yaricapi (metre)
DEFAULT_BUOY_RADIUS_M = 0.15

# Kameranin yatay gorus acisi (derece) — costmap FOV bilgisi icin
CAMERA_FOV_DEG = 70.0


def class_name_to_type(class_name: str) -> str:
    """class_name stringini costmap semantic tipine donusturur."""
    cn = class_name.lower().strip()
    if cn in CLASS_NAME_TO_TYPE:
        return CLASS_NAME_TO_TYPE[cn]
    # Alt dize eslestirme
    for key, typ in CLASS_NAME_TO_TYPE.items():
        if key in cn:
            return typ
    return 'unknown'


def safe_float(value, fallback=None):
    """NaN ve sonsuzlugu filtreler; gecersizse fallback dondurur."""
    try:
        f = float(value)
        if math.isnan(f) or math.isinf(f):
            return fallback
        return f
    except (TypeError, ValueError):
        return fallback


def normalize_angle_180(angle_deg: float) -> float:
    """Aciyi [-180, +180) araligina normalize eder."""
    return (angle_deg + 180.0) % 360.0 - 180.0


# =============================================================================
# Sensor Veri Yapisi
# =============================================================================

class SensorReading:
    """Tek bir mesafe sensoru olcumunu ve meta bilgilerini tutar."""

    def __init__(self):
        self.sensor_name: str       = ''
        self.range_m: float         = 0.0
        self.min_range: float       = 0.0
        self.max_range: float       = 0.0
        self.field_of_view: float   = 0.0
        self.frame_id: str          = ''
        self.measurement_time       = None   # rclpy.Time
        self.valid: bool            = False


# =============================================================================
# DubaFusionNode
# =============================================================================

class DubaFusionNode(Node):
    """
    YOLO mesafe/aci verisini dort mesafe sensoruyle birlestirerek
    costmap noduna gonderilmek uzere nihai duba konumlarini yayinlar.
    """

    def __init__(self):
        super().__init__('duba_fusion_node')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('yolo_input_topic',
                               '/albatros/yolo/mesafe_aci')
        self.declare_parameter('on_sol_topic',
                               '/albatros/mesafe/on_sol')
        self.declare_parameter('on_sag_topic',
                               '/albatros/mesafe/on_sag')
        self.declare_parameter('yan_sol_topic',
                               '/albatros/mesafe/yan_sol')
        self.declare_parameter('yan_sag_topic',
                               '/albatros/mesafe/yan_sag')
        self.declare_parameter('output_topic',
                               '/albatros/fusion/obstacles')

        self.declare_parameter('min_yolo_confidence',      0.60)
        self.declare_parameter('sensor_timeout_sec',       0.50)
        self.declare_parameter('max_time_difference_sec',  0.50)
        self.declare_parameter('max_distance_difference_m', 1.50)
        self.declare_parameter('sensor_effective_max_range_m', 5.0)
        self.declare_parameter('sensor_angle_tolerance_deg', 20.0)

        # GECICI KALIBRASYON DEGERLERI:
        # Asagidaki sensor merkez acilari gercek montaj konumuna gore
        # olculerek guncellenmelidir. Su anki degerler yaklasik tahmindir.
        self.declare_parameter('on_sol_angle_deg',   25.0)
        self.declare_parameter('on_sag_angle_deg',  -25.0)
        self.declare_parameter('yan_sol_angle_deg',  90.0)
        self.declare_parameter('yan_sag_angle_deg', -90.0)

        self.declare_parameter('publish_empty_results', True)
        self.declare_parameter('log_fusion_results',    False)

        # Parametre degerlerini oku
        self._yolo_topic      = self.get_parameter('yolo_input_topic').value
        self._output_topic    = self.get_parameter('output_topic').value
        self._min_yolo_conf   = self.get_parameter('min_yolo_confidence').value
        self._sensor_timeout  = self.get_parameter('sensor_timeout_sec').value
        self._max_time_diff   = self.get_parameter('max_time_difference_sec').value
        self._max_dist_diff   = self.get_parameter('max_distance_difference_m').value
        self._eff_max_range   = self.get_parameter('sensor_effective_max_range_m').value
        self._angle_tol       = self.get_parameter('sensor_angle_tolerance_deg').value
        self._pub_empty       = self.get_parameter('publish_empty_results').value
        self._log_fusion      = self.get_parameter('log_fusion_results').value

        # Sensor merkez acilari sozlugu
        self._sensor_angles: dict[str, float] = {
            'on_sol':  self.get_parameter('on_sol_angle_deg').value,
            'on_sag':  self.get_parameter('on_sag_angle_deg').value,
            'yan_sol': self.get_parameter('yan_sol_angle_deg').value,
            'yan_sag': self.get_parameter('yan_sag_angle_deg').value,
        }

        # Son sensor olcumleri
        self._sensor_readings: dict[str, SensorReading] = {
            name: SensorReading() for name in self._sensor_angles
        }

        # Throttle: olcum uyumsuzlugu uyarilari
        self._last_mismatch_warn: dict[str, float] = {}
        self._WARN_THROTTLE_SEC = 3.0

        # ── Subscriber'lar ────────────────────────────────────────────────────
        self._sub_yolo = self.create_subscription(
            String,
            self._yolo_topic,
            self._cb_yolo,
            10
        )

        sensor_topics = {
            'on_sol':  self.get_parameter('on_sol_topic').value,
            'on_sag':  self.get_parameter('on_sag_topic').value,
            'yan_sol': self.get_parameter('yan_sol_topic').value,
            'yan_sag': self.get_parameter('yan_sag_topic').value,
        }

        # Subscription referanslari saklanir; Python GC'nin nesneyi
        # erken serbest birakmasi onlenir.
        self._sensor_subscriptions = []
        for name, topic in sensor_topics.items():
            sub = self.create_subscription(
                Range,
                topic,
                self._make_range_callback(name),
                10
            )
            self._sensor_subscriptions.append(sub)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(String, self._output_topic, 10)

        # ── Baslangic logu ────────────────────────────────────────────────────
        sep = '=' * 64
        self.get_logger().info(sep)
        self.get_logger().info('DubaFusionNode baslatildi.')
        self.get_logger().info(f'  YOLO girdi        : {self._yolo_topic}')
        self.get_logger().info(f'  Cikti             : {self._output_topic}')
        self.get_logger().info(f'  min_yolo_conf     : {self._min_yolo_conf}')
        self.get_logger().info(f'  sensor_timeout    : {self._sensor_timeout} s')
        self.get_logger().info(f'  max_time_diff     : {self._max_time_diff} s')
        self.get_logger().info(f'  max_dist_diff     : {self._max_dist_diff} m')
        self.get_logger().info(f'  eff_max_range     : {self._eff_max_range} m')
        self.get_logger().info(f'  angle_tolerance   : {self._angle_tol} deg')
        for name, angle in self._sensor_angles.items():
            self.get_logger().info(
                f'  {name:<10}: merkez aci = {angle:+.1f} deg'
            )
        self.get_logger().info(sep)

    # =========================================================================
    # Yardimci: sensor callback fabrikasi
    # =========================================================================

    def _make_range_callback(self, sensor_name: str):
        """Her sensor icin ayri bir callback closure uretir."""
        def callback(msg: Range):
            self._cb_range(sensor_name, msg)
        return callback

    # =========================================================================
    # Callback: mesafe sensoru
    # =========================================================================

    def _cb_range(self, sensor_name: str, msg: Range):
        """Range mesajini isleyerek dahili sensor okumalarini gunceller."""
        reading = self._sensor_readings[sensor_name]
        reading.sensor_name      = sensor_name
        reading.min_range        = msg.min_range
        reading.max_range        = msg.max_range
        reading.field_of_view    = msg.field_of_view
        reading.frame_id         = msg.header.frame_id
        reading.measurement_time = self.get_clock().now()

        raw = safe_float(msg.range)
        if (raw is None
                or raw <= 0.0
                or raw < msg.min_range
                or raw > msg.max_range):
            reading.range_m = 0.0
            reading.valid   = False
        else:
            reading.range_m = raw
            reading.valid   = True

    # =========================================================================
    # Callback: YOLO mesafe/aci
    # =========================================================================

    def _cb_yolo(self, msg: String):
        """
        YOLO mesafe/aci JSON mesajini alir, fusyon yaparak sonucu yayinlar.

        Aci donusumu bu callback icinde yapilir:
          kamera_bearing (sag=pozitif) → base_link_bearing (sol=pozitif)
          bearing_deg = normalize_angle_180(-camera_bearing_deg)

        Zaman eslestirmesi: YOLO mesajinin bu node'a ulasma zamani
        (yolo_receive_time) kullanilir; JSON stamp degeri yalnizca
        cikti meta verisi olarak korunur.
        """
        # Bu callback'in cagrilma ani — sensor zamanlarıyla karsilastirilacak
        yolo_receive_time = self.get_clock().now()

        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f'YOLO JSON parse hatasi: {e} | Mesaj atlandi.')
            return

        # JSON stamp cikti payload'i icin saklanir; fuzyon zamani icin kullanilmaz
        stamp    = safe_float(data.get('stamp', 0.0), fallback=0.0)
        raw_dets = data.get('detections', [])

        if not isinstance(raw_dets, list):
            self.get_logger().warn('YOLO detections listesi gecersiz. Mesaj atlandi.')
            return

        # ── Aci donusumu ve on-filtreleme ─────────────────────────────────
        # Her detection icin base_link bearing hesapla ve sensor aci farkini bul.
        # Bu bilgi detection onceliklendirmesi icin kullanilacak.
        prepared = []
        for det in raw_dets:
            try:
                camera_bearing = safe_float(det.get('bearing_deg'), None)
                if camera_bearing is None:
                    continue
                # Kamera frame (sag=+) → base_link (sol=+) donusumu
                bl_bearing = normalize_angle_180(-camera_bearing)

                # En yakin sensor merkez acisina gore oncelik skoru hesapla
                min_angle_diff = float('inf')
                for sname, sangle in self._sensor_angles.items():
                    diff = abs(normalize_angle_180(bl_bearing - normalize_angle_180(sangle)))
                    diff = min(diff, 360.0 - diff)
                    if diff < min_angle_diff:
                        min_angle_diff = diff

                prepared.append((min_angle_diff, bl_bearing, det))
            except Exception:
                # Hatalı detection onceliklendirmeyi engellemsin
                prepared.append((float('inf'), 0.0, det))

        # Sensor merkez acisına en yakin detection'lar once islenir
        # Bu, ayni sensoru birden fazla dubaya vermemek icin used_sensors ile
        # birlikte calisir. NOT: Bu ilk surum; ileride Kuhn-Munkres ataması
        # gibi global optimizasyon uygulanabilir.
        prepared.sort(key=lambda x: x[0])

        # ── Fuzyon ───────────────────────────────────────────────────────
        # Her callback'te sifirlanir; bir sensor ayni mesajda yalnizca
        # tek bir duba icin tam fuzyon yapabilir.
        used_sensors: set[str] = set()

        fused_dets = []
        for _, bl_bearing, det in prepared:
            try:
                result = self._fuse_detection(
                    det, bl_bearing, yolo_receive_time, used_sensors
                )
                if result is not None:
                    fused_dets.append(result)
            except Exception as e:
                self.get_logger().error(
                    f'Detection fusion hatasi (atlanidi): {e}'
                )
                continue

        if not fused_dets and not self._pub_empty:
            return

        # costmap_node'un beklentisiyle uyumlu format:
        # obstacles[] icinde x_m, y_m, type, confidence, id, radius_m
        obstacles_for_costmap = []
        for idx, d in enumerate(fused_dets):
            obs_type = class_name_to_type(d.get('class_name', 'unknown'))
            obstacles_for_costmap.append({
                'id':            f"fusion_{idx}_{int(stamp * 1000) % 100000}",
                'type':          obs_type,
                'class_name':    d.get('class_name', 'unknown'),
                'confidence':    d.get('fusion_confidence', 0.0),
                'x_m':           d.get('x_body_m', 0.0),
                'y_m':           d.get('y_body_m', 0.0),
                'radius_m':      DEFAULT_BUOY_RADIUS_M,
                'range_verified': d.get('fusion_source') == 'yolo_and_distance_sensor',
            })

        payload = {
            'stamp':            stamp,
            'frame_id':         'base_link',
            'source':           'duba_fusion_node',
            'fusion_valid':     True,
            'observed_fov_deg': CAMERA_FOV_DEG,
            'observed_range_m': self._eff_max_range,
            'detection_count':  len(obstacles_for_costmap),
            'obstacles':        obstacles_for_costmap,
        }

        out_msg = String()
        out_msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub.publish(out_msg)

        if self._log_fusion and fused_dets:
            for d in fused_dets:
                self.get_logger().info(
                    f"[FUZYON] {d['class_name']} | "
                    f"kaynak={d['fusion_source']} | "
                    f"mesafe={d['final_distance_m']:.2f} m | "
                    f"bearing={d['bearing_deg']:.1f} deg | "
                    f"guven={d['fusion_confidence']:.2f}"
                )

    # =========================================================================
    # Tek bir detection icin fuzyon
    # =========================================================================

    def _fuse_detection(
        self,
        det: dict,
        bearing_deg: float,
        yolo_receive_time,
        used_sensors: set[str],
    ) -> dict | None:
        """
        Tek bir YOLO detectionini mesafe sensorleriyle birlestirerek
        fuzyon sonucu uretir.

        Args:
            det:              YOLO detection sozlugu (kamera frame bearing iceriyor).
            bearing_deg:      _cb_yolo tarafindan donusturulmus base_link bearing.
                              (kamera sag=+ → base_link sol=+)
            yolo_receive_time: YOLO mesajinin bu node'a ulasma zamani (rclpy.Time).
            used_sensors:     Bu callback'te zaten kullanilmis sensor isimleri.

        Donus:
            dict  — doldurulan fuzyon detection sozlugu
            None  — gecersiz / guvensiz detection
        """
        # ── YOLO verisi dogrulama ──────────────────────────────────────────
        yolo_conf   = safe_float(det.get('yolo_confidence', 0.0), 0.0)
        visual_dist = safe_float(det.get('visual_distance_m'), None)

        if yolo_conf < self._min_yolo_conf:
            return None
        if visual_dist is None or visual_dist <= 0.0:
            return None
        # bearing_deg _cb_yolo tarafindan dogrulandi; burada None gelmez

        class_id   = det.get('class_id',   0)
        class_name = det.get('class_name', 'unknown')

        # ── En uygun sensor secimi ─────────────────────────────────────────
        # used_sensors: bu callback'te onceki dubalar tarafindan alinan sensorler
        matched_sensor, sensor_reading = self._match_sensor(
            bearing_deg, yolo_receive_time, used_sensors
        )

        # ── Fuzyon modu belirleme ──────────────────────────────────────────
        if matched_sensor is None or sensor_reading is None:
            # Durum 2: YOLO gecerli, uygun sensor yok
            return self._build_yolo_only(
                class_id, class_name, yolo_conf,
                visual_dist, bearing_deg,
                fusion_source='yolo_only',
                sensor_name=None, sensor_reading=None,
            )

        sensor_dist = sensor_reading.range_m

        # Efektif maksimum menzil: sensor max_range ile parametre minimumunun kucugu
        effective_limit = min(self._eff_max_range, sensor_reading.max_range)

        # Sensor efektif menzil dis kontrol
        if sensor_dist > effective_limit:
            return self._build_yolo_only(
                class_id, class_name, yolo_conf,
                visual_dist, bearing_deg,
                fusion_source='yolo_only',
                sensor_name=None, sensor_reading=None,
            )

        # Mesafe uyum kontrolu
        dist_diff = abs(visual_dist - sensor_dist)
        if dist_diff > self._max_dist_diff:
            # Durum 3: olcumler cok farkli — sensor kullanilmamis sayilir
            self._throttled_warn(
                matched_sensor,
                f'[UYUMSUZLUK] {class_name} | YOLO={visual_dist:.2f} m, '
                f'sensor={sensor_dist:.2f} m | fark={dist_diff:.2f} m > '
                f'{self._max_dist_diff} m. YOLO kullanilacak.'
            )
            result = self._build_yolo_only(
                class_id, class_name, yolo_conf,
                visual_dist, bearing_deg,
                fusion_source='yolo_only_measurement_mismatch',
                sensor_name=matched_sensor,
                sensor_reading=sensor_reading,
            )
            result['fusion_confidence'] = round(yolo_conf * 0.6, 4)
            # Uyumsuzlukta sensor kullanilmamis sayilir; used_sensors'a eklenmez
            return result

        # ── Durum 1: tam fuzyon ────────────────────────────────────────────
        sensor_w, yolo_w = self._get_weights(sensor_dist)

        final_dist    = sensor_dist * sensor_w + visual_dist * yolo_w
        final_dist    = safe_float(final_dist, visual_dist)
        final_bearing = bearing_deg  # aci her zaman YOLO'dan (donusturulmus)

        # base_link koordinatlari: bearing_deg > 0 → sol → y pozitif
        angle_rad = math.radians(final_bearing)
        x_body_m  = final_dist * math.cos(angle_rad)
        y_body_m  = final_dist * math.sin(angle_rad)

        # Fuzyon guveni
        agreement   = max(0.0, 1.0 - dist_diff / self._max_dist_diff)
        fusion_conf = min(
            1.0,
            FUSION_CONF_YOLO_WEIGHT * yolo_conf
            + FUSION_CONF_AGREEMENT_WEIGHT * agreement
        )

        # Basarili tam fuzyon: sensoru kullanilmis olarak isaretle
        used_sensors.add(matched_sensor)

        return {
            'class_id':          class_id,
            'class_name':        class_name,
            'yolo_confidence':   round(yolo_conf, 4),
            'fusion_confidence': round(fusion_conf, 4),
            'visual_distance_m': round(visual_dist, 4),
            'sensor_distance_m': round(sensor_dist, 4),
            'final_distance_m':  round(final_dist, 4),
            'bearing_deg':       round(final_bearing, 4),
            'x_body_m':          round(x_body_m, 4),
            'y_body_m':          round(y_body_m, 4),
            'matched_sensor':    matched_sensor,
            'sensor_frame_id':   sensor_reading.frame_id,
            'sensor_weight':     round(sensor_w, 4),
            'yolo_weight':       round(yolo_w, 4),
            'fusion_source':     'yolo_and_distance_sensor',
            'valid':             True,
        }

    # =========================================================================
    # Sensor eslestirme
    # =========================================================================

    def _match_sensor(
        self,
        bearing_deg: float,
        yolo_receive_time,
        used_sensors: set[str],
    ) -> tuple[str | None, SensorReading | None]:
        """
        YOLO dubasinin base_link bearing_deg acisina en yakin ve uygun
        mesafe sensoru secilir.

        Kriter:
          1. Bu callback'te baska bir duba icin kullanilmamis olmali.
          2. Sensor guncel olmali (sensor_timeout_sec).
          3. Sensor verisi gecerli olmali.
          4. Zaman farki max_time_difference_sec icerisinde olmali.
             (yolo_receive_time ile measurement_time karsilastirilir;
              her ikisi de ayni ROS clock'tan alinir.)
          5. Aci farki sensor_angle_tolerance_deg icerisinde olmali.
          6. Birden fazla uygunsa aci farki en kucuk olan kazanir.
        """
        now             = self.get_clock().now()
        best_name       = None
        best_angle_diff = float('inf')

        norm_bearing = normalize_angle_180(bearing_deg)

        for name, reading in self._sensor_readings.items():

            # Bu callback'te baska bir duba tarafindan kullanildi mi?
            if name in used_sensors:
                continue

            if not reading.valid:
                continue

            if reading.measurement_time is None:
                continue

            # Sensor timeout kontrolu: son olcum ne kadar eskide?
            elapsed = (now - reading.measurement_time).nanoseconds / 1e9
            if elapsed > self._sensor_timeout:
                continue

            # Zaman uyum kontrolu: yolo_receive_time ile sensor olcum zamani
            # Her ikisi de self.get_clock().now() ile alindiginden guvenilirdir.
            time_diff = abs(
                (yolo_receive_time - reading.measurement_time).nanoseconds / 1e9
            )
            if time_diff > self._max_time_diff:
                continue

            # Aci farki kontrolu
            sensor_center = normalize_angle_180(self._sensor_angles[name])
            angle_diff    = abs(normalize_angle_180(norm_bearing - sensor_center))
            angle_diff    = min(angle_diff, 360.0 - angle_diff)

            if angle_diff > self._angle_tol:
                continue

            if angle_diff < best_angle_diff:
                best_angle_diff = angle_diff
                best_name       = name

        if best_name is None:
            return None, None
        return best_name, self._sensor_readings[best_name]

    # =========================================================================
    # Agirlik secimi
    # =========================================================================

    def _get_weights(self, sensor_dist: float) -> tuple[float, float]:
        """
        Sensor mesafesine gore sensor/YOLO agirlik cifti dondurur.
        Agirlik toplami her zaman 1.0'dir.
        """
        if sensor_dist <= WEIGHT_RANGE_NEAR_MAX:
            return 0.80, 0.20
        elif sensor_dist <= WEIGHT_RANGE_MID_MAX:
            return 0.70, 0.30
        else:
            return 0.50, 0.50

    # =========================================================================
    # Yalnizca YOLO sonucu yapisi
    # =========================================================================

    def _build_yolo_only(
        self,
        class_id: int,
        class_name: str,
        yolo_conf: float,
        visual_dist: float,
        bearing_deg: float,
        fusion_source: str,
        sensor_name: str | None,
        sensor_reading: SensorReading | None,
    ) -> dict:
        """
        Sensor eslesmesi olmayan veya uyumsuz durumlarda
        yalnizca YOLO verisiyle doldurulmus detection sozlugu uretir.
        """
        angle_rad = math.radians(bearing_deg)
        x_body_m  = visual_dist * math.cos(angle_rad)
        y_body_m  = visual_dist * math.sin(angle_rad)

        return {
            'class_id':          class_id,
            'class_name':        class_name,
            'yolo_confidence':   round(yolo_conf, 4),
            'fusion_confidence': round(yolo_conf, 4),
            'visual_distance_m': round(visual_dist, 4),
            'sensor_distance_m': (
                round(sensor_reading.range_m, 4)
                if sensor_reading else None
            ),
            'final_distance_m':  round(visual_dist, 4),
            'bearing_deg':       round(bearing_deg, 4),
            'x_body_m':          round(x_body_m, 4),
            'y_body_m':          round(y_body_m, 4),
            'matched_sensor':    sensor_name,
            'sensor_frame_id':   (
                sensor_reading.frame_id if sensor_reading else None
            ),
            'sensor_weight':     0.0,
            'yolo_weight':       1.0,
            'fusion_source':     fusion_source,
            'valid':             True,
        }

    # =========================================================================
    # Throttled uyari
    # =========================================================================

    def _throttled_warn(self, key: str, message: str):
        """Ayni anahtar icin en fazla 3 saniyede bir uyari log'u yazar."""
        now = self.get_clock().now().nanoseconds / 1e9
        last = self._last_mismatch_warn.get(key, 0.0)
        if now - last >= self._WARN_THROTTLE_SEC:
            self.get_logger().warn(message)
            self._last_mismatch_warn[key] = now


# =============================================================================
# Giris Noktasi
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = DubaFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('DubaFusionNode durduruldu (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
