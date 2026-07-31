#!/usr/bin/env python3
"""
telemetry_security_node.py — LODOS Albatros Telemetri Güvenlik Node'u
=====================================================================
3DR 915 MHz telemetri modül çiftinden AT komutlarıyla RSSI ve
gürültü (noise) verilerini okuyarak frekans çakışma tespiti yapar.

Donanım Yapılandırması:
  - 3DR Telemetri (Hava): Pixhawk 2.4.8 TELEM1 portuna bağlı
  - 3DR Telemetri (Yer):  YKİ bilgisayarına USB üzerinden bağlı
  - Firmware: SiK (varsayılan)
  - Frekans bandı: 915 MHz (değişken — yarışmada atanır)
  - Baud rate: 57600 (varsayılan)

Çalışma Mantığı:
  - simulate_mode=True  → Sentetik RSSI/noise verisi üretir (test amaçlı)
  - simulate_mode=False → Seri port üzerinden AT komutlarıyla gerçek veri okur
  - Periyodik olarak (scan_interval_sec) AT moduna geçip RSSI/noise okur
  - Okunan değerler SafetyEvaluator'a verilerek girişim seviyesi hesaplanır
  - Her publish_rate Hz'de TelemetrySecurityStatus mesajı yayınlanır
  - Girişim tespit edildiğinde /albatros/telemetry/interference_alert yayınlanır

AT Komutları (SiK Firmware):
  - +++         → AT komut moduna giriş (1s bekleme gerekli)
  - ATI5        → RSSI, remote RSSI, noise, remote noise raporu
  - ATS3?       → Net ID sorgusu
  - ATS8?       → Min frekans sorgusu (kHz)
  - ATS9?       → Max frekans sorgusu (kHz)
  - ATO         → Veri moduna dönüş

Albatros Topic'leri (Çıkış):
  - /albatros/telemetry/security          → TelemetrySecurityStatus mesajı
  - /albatros/telemetry/interference_alert → Girişim uyarısı (std_msgs/String JSON)

Yazar : LODOS Takımı
Araç  : Albatros İDA
"""

import json
import math
import random
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Header

from albatros_interfaces.msg import TelemetrySecurityStatus

from albatros_system.safety_states import (
    SafetyLevel,
    InterferenceLevel,
    SafetyEvaluator,
    SafetyState,
)


# ── Topic Tanımları ──────────────────────────────────────────────────────────
SECURITY_TOPIC       = '/albatros/telemetry/security'
ALERT_TOPIC          = '/albatros/telemetry/interference_alert'

# ── Varsayılan Parametreler ──────────────────────────────────────────────────
DEFAULT_SERIAL_PORT        = '/dev/ttyUSB0'
DEFAULT_BAUD_RATE          = 57600
DEFAULT_SCAN_INTERVAL_SEC  = 30.0
DEFAULT_PUBLISH_RATE       = 1.0
DEFAULT_NOISE_THRESHOLD_HIGH     = 80
DEFAULT_NOISE_THRESHOLD_CRITICAL = 100
DEFAULT_AT_TIMEOUT_SEC     = 3.0

# ── AT Komut Sabitleri ───────────────────────────────────────────────────────
AT_ENTER_CMD       = b'+++'
AT_EXIT_CMD        = b'ATO\r\n'
AT_RSSI_CMD        = b'ATI5\r\n'
AT_NETID_CMD       = b'ATS3?\r\n'
AT_MIN_FREQ_CMD    = b'ATS8?\r\n'
AT_MAX_FREQ_CMD    = b'ATS9?\r\n'
AT_ENTER_DELAY_SEC = 1.1   # +++ sonrası bekleme süresi (SiK spec: 1s)
AT_CMD_DELAY_SEC   = 0.3   # AT komutu sonrası bekleme süresi


class TelemetrySecurityNode(Node):
    """
    3DR 915 MHz telemetri modülünden AT komutlarıyla RSSI/noise
    okuyarak frekans çakışma tespiti yapan ROS2 node'u.

    Simülasyon modunda sentetik veri üretir.
    Gerçek sensör modunda seri port üzerinden AT komutları gönderir.
    """

    def __init__(self):
        super().__init__('telemetry_security_node')

        # ─── Parametre Tanımları ─────────────────────────────────────────
        self.declare_parameter('serial_port',        DEFAULT_SERIAL_PORT)
        self.declare_parameter('baud_rate',          DEFAULT_BAUD_RATE)
        self.declare_parameter('scan_interval_sec',  DEFAULT_SCAN_INTERVAL_SEC)
        self.declare_parameter('publish_rate',       DEFAULT_PUBLISH_RATE)
        self.declare_parameter('noise_threshold_high',     DEFAULT_NOISE_THRESHOLD_HIGH)
        self.declare_parameter('noise_threshold_critical',  DEFAULT_NOISE_THRESHOLD_CRITICAL)
        self.declare_parameter('simulate_mode',      True)
        self.declare_parameter('at_timeout_sec',     DEFAULT_AT_TIMEOUT_SEC)

        # ─── Parametre Okuma ─────────────────────────────────────────────
        self._serial_port    = self.get_parameter('serial_port').value
        self._baud_rate      = self.get_parameter('baud_rate').value
        self._scan_interval  = self.get_parameter('scan_interval_sec').value
        self._publish_rate   = self.get_parameter('publish_rate').value
        noise_high           = self.get_parameter('noise_threshold_high').value
        noise_critical       = self.get_parameter('noise_threshold_critical').value
        self.simulate_mode   = self.get_parameter('simulate_mode').value
        self._at_timeout     = self.get_parameter('at_timeout_sec').value

        # ─── Güvenlik Değerlendirici ─────────────────────────────────────
        self._evaluator = SafetyEvaluator(
            noise_high=noise_high,
            noise_critical=noise_critical,
        )

        # ─── Dahili Durum Değişkenleri ───────────────────────────────────
        self._telemetry_connected = False
        self._serial = None                # Serial port nesnesi
        self._serial_lock = threading.Lock()

        # Frekans konfigürasyonu (AT ile okunacak)
        self._net_id         = 0
        self._min_freq_khz   = 0
        self._max_freq_khz   = 0

        # Son okunan RSSI / noise değerleri
        self._local_rssi     = 0
        self._remote_rssi    = 0
        self._local_noise    = 0
        self._remote_noise   = 0

        # Son güvenlik durumu
        self._last_safety_state: SafetyState = SafetyState()

        # İstatistikler
        self._scan_count       = 0
        self._last_scan_time   = 0.0
        self._start_time       = time.time()

        # Son gönderilen girişim seviyesi (tekrar alert göndermemek için)
        self._last_alerted_level = InterferenceLevel.NONE

        # ─── Seri Port Bağlantısı ────────────────────────────────────────
        if not self.simulate_mode:
            self._connect_serial()
        else:
            self._telemetry_connected = True
            self.get_logger().info(
                'Simülasyon modu aktif — seri port bağlantısı atlanıyor.'
            )

        # ─── QoS Profili ─────────────────────────────────────────────────
        default_qos = QoSProfile(depth=10)

        # ─── Publisher'lar ───────────────────────────────────────────────
        self._pub_security = self.create_publisher(
            TelemetrySecurityStatus,
            SECURITY_TOPIC,
            default_qos,
        )

        self._pub_alert = self.create_publisher(
            String,
            ALERT_TOPIC,
            default_qos,
        )

        # ─── Timer'lar ──────────────────────────────────────────────────
        # Ana yayın timer'ı
        publish_period = 1.0 / max(self._publish_rate, 0.1)
        self._publish_timer = self.create_timer(
            publish_period,
            self._publish_callback,
        )

        # AT tarama timer'ı
        self._scan_timer = self.create_timer(
            max(self._scan_interval, 5.0),
            self._scan_callback,
        )

        # ─── İlk tarama: frekans konfigürasyonunu oku ───────────────────
        if not self.simulate_mode:
            self._read_frequency_config()
        else:
            # Simülasyon varsayılan frekans değerleri
            self._min_freq_khz = 915000
            self._max_freq_khz = 928000
            self._net_id       = 25

        # ─── Başlatma Bilgi Logları ──────────────────────────────────────
        mode_str = (
            'SİMÜLASYON (simulate_mode=True)'
            if self.simulate_mode
            else f'GERÇEK SENSÖR — {self._serial_port}@{self._baud_rate}'
        )

        freq_center = (self._min_freq_khz + self._max_freq_khz) / 2000.0

        self.get_logger().info('=' * 64)
        self.get_logger().info('Telemetri Güvenlik Node başlatıldı.')
        self.get_logger().info(f'Mod: {mode_str}')
        self.get_logger().info(f'Çıkış Topic: {SECURITY_TOPIC}')
        self.get_logger().info(f'Alert Topic: {ALERT_TOPIC}')
        self.get_logger().info(
            f'Yayın frekansı: {self._publish_rate} Hz'
        )
        self.get_logger().info(
            f'Tarama periyodu: {self._scan_interval} s'
        )
        self.get_logger().info(
            f'Frekans bandı: {self._min_freq_khz/1000.0:.1f} – '
            f'{self._max_freq_khz/1000.0:.1f} MHz '
            f'(merkez: {freq_center:.1f} MHz)'
        )
        self.get_logger().info(
            f'Net ID: {self._net_id}'
        )
        self.get_logger().info(
            f'Girişim eşikleri: HIGH={noise_high}, '
            f'CRITICAL={noise_critical}'
        )
        self.get_logger().info(
            'Donanım: 3DR Telemetri 915 MHz | '
            'SiK Firmware | '
            'Pixhawk TELEM1'
        )
        self.get_logger().info('=' * 64)

    # =====================================================================
    # Seri Port Yönetimi
    # =====================================================================

    def _connect_serial(self):
        """
        Yer telemetri modülünün seri portuna bağlanır.
        pyserial kütüphanesi gerektirir.
        """
        try:
            import serial
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=self._at_timeout,
            )
            self._telemetry_connected = True
            self.get_logger().info(
                f'Seri port bağlantısı kuruldu: '
                f'{self._serial_port}@{self._baud_rate}'
            )
        except ImportError:
            self.get_logger().error(
                'pyserial kütüphanesi bulunamadı! '
                'Kurulum: pip install pyserial'
            )
            self._telemetry_connected = False
        except Exception as e:
            self.get_logger().error(
                f'Seri port bağlantısı başarısız: {e} | '
                f'Port: {self._serial_port} | '
                f'Baud: {self._baud_rate}'
            )
            self._telemetry_connected = False

    def _send_at_command(self, command: bytes, delay: float = AT_CMD_DELAY_SEC) -> str:
        """
        Seri port üzerinden AT komutu gönderir ve yanıtı okur.

        Args:
            command: Gönderilecek AT komutu (bytes).
            delay:   Komut sonrası bekleme süresi (saniye).

        Returns:
            Modülden gelen yanıt string'i. Hata durumunda boş string.
        """
        if self._serial is None or not self._serial.is_open:
            return ''

        try:
            with self._serial_lock:
                self._serial.reset_input_buffer()
                self._serial.write(command)
                time.sleep(delay)
                response = self._serial.read(
                    self._serial.in_waiting or 256
                ).decode('ascii', errors='ignore').strip()
            return response
        except Exception as e:
            self.get_logger().warn(
                f'AT komut hatası: {e} | Komut: {command}'
            )
            return ''

    def _enter_at_mode(self) -> bool:
        """
        Telemetri modülünü AT komut moduna geçirir.

        3DR (SiK) spec'e göre:
          1. En az 1 saniye sessizlik
          2. +++ gönder (CR/LF OLMADAN)
          3. En az 1 saniye bekle
          4. 'OK' yanıtı beklenir

        Returns:
            True → AT moduna girildi, False → Giriş başarısız.
        """
        response = self._send_at_command(AT_ENTER_CMD, AT_ENTER_DELAY_SEC)
        success = 'OK' in response
        if not success:
            self.get_logger().warn(
                f'AT moduna giriş başarısız. Yanıt: "{response}"'
            )
        return success

    def _exit_at_mode(self):
        """
        AT komut modundan çıkarak veri moduna geri döner.
        ATO komutu gönderilir.
        """
        self._send_at_command(AT_EXIT_CMD, AT_CMD_DELAY_SEC)

    # =====================================================================
    # AT Komut Taramaları
    # =====================================================================

    def _read_frequency_config(self):
        """
        AT komutlarıyla frekans konfigürasyonunu okur.
        Net ID, min/max frekans değerleri sorgulanır.
        Bu fonksiyon sadece başlangıçta veya gerektiğinde çağrılır.
        """
        if not self._telemetry_connected:
            return

        if not self._enter_at_mode():
            return

        try:
            # Net ID
            response = self._send_at_command(AT_NETID_CMD)
            self._net_id = self._parse_int_response(response)

            # Min frekans (kHz)
            response = self._send_at_command(AT_MIN_FREQ_CMD)
            self._min_freq_khz = self._parse_int_response(response)

            # Max frekans (kHz)
            response = self._send_at_command(AT_MAX_FREQ_CMD)
            self._max_freq_khz = self._parse_int_response(response)

            self.get_logger().info(
                f'Frekans konfigürasyonu okundu: '
                f'NetID={self._net_id}, '
                f'Min={self._min_freq_khz/1000.0:.1f} MHz, '
                f'Max={self._max_freq_khz/1000.0:.1f} MHz'
            )
        finally:
            self._exit_at_mode()

    def _read_rssi_noise(self) -> bool:
        """
        ATI5 komutuyla RSSI ve noise değerlerini okur.

        ATI5 yanıt formatı (SiK firmware):
            L/R RSSI: <local_rssi>/<remote_rssi>
            L/R noise: <local_noise>/<remote_noise>
            pkts: <rx_pkts> <tx_pkts>
            txe: <tx_errors>

        Returns:
            True → Değerler başarıyla okundu, False → Okuma başarısız.
        """
        if not self._telemetry_connected:
            return False

        if not self._enter_at_mode():
            return False

        try:
            response = self._send_at_command(AT_RSSI_CMD, delay=0.5)

            if not response:
                self.get_logger().warn('ATI5 yanıtı boş.')
                return False

            return self._parse_ati5_response(response)

        finally:
            self._exit_at_mode()

    def _parse_ati5_response(self, response: str) -> bool:
        """
        ATI5 komutunun yanıtını parse ederek RSSI/noise değerlerini
        dahili değişkenlere yazar.

        Args:
            response: ATI5 yanıt string'i.

        Returns:
            True → Parse başarılı, False → Parse hatası.
        """
        try:
            lines = response.strip().split('\n')

            for line in lines:
                line = line.strip()

                # L/R RSSI: 120/95
                if 'RSSI' in line.upper() and '/' in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        values = parts[-1].strip().split('/')
                        if len(values) >= 2:
                            self._local_rssi  = int(values[0].strip())
                            self._remote_rssi = int(values[1].strip())

                # L/R noise: 45/50
                elif 'noise' in line.lower() and '/' in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        values = parts[-1].strip().split('/')
                        if len(values) >= 2:
                            self._local_noise  = int(values[0].strip())
                            self._remote_noise = int(values[1].strip())

            self._scan_count += 1
            self._last_scan_time = time.time()

            self.get_logger().debug(
                f'ATI5 parse OK: '
                f'RSSI={self._local_rssi}/{self._remote_rssi}, '
                f'Noise={self._local_noise}/{self._remote_noise}'
            )
            return True

        except (ValueError, IndexError) as e:
            self.get_logger().warn(
                f'ATI5 parse hatası: {e} | Yanıt: "{response}"'
            )
            return False

    @staticmethod
    def _parse_int_response(response: str) -> int:
        """
        AT komutunun sayısal yanıtını parse eder.

        AT yanıtları genellikle şu formatta gelir:
            <değer>\r\nOK
        Sadece sayısal kısmı alır.

        Args:
            response: AT yanıt string'i.

        Returns:
            Parse edilen tam sayı. Hata durumunda 0.
        """
        try:
            # 'OK' ve boşlukları temizle, ilk sayısal satırı al
            for line in response.strip().split('\n'):
                line = line.strip()
                if line and line != 'OK':
                    return int(line)
        except (ValueError, IndexError):
            pass
        return 0

    # =====================================================================
    # Simülasyon Verisi Üretimi
    # =====================================================================

    def _generate_simulated_scan(self):
        """
        Test amaçlı sentetik RSSI/noise verisi üretir.

        Normal çalışma koşullarında noise 30-60 aralığında,
        ara sıra spike üretilerek girişim senaryosu simüle edilir.
        """
        elapsed = time.time() - self._start_time

        # Temel noise seviyesi (30-55 arası normal)
        base_noise = 40 + random.gauss(0, 5)

        # Her 120 saniyede bir girişim spike'ı simüle et
        # (10 saniye süreyle noise 75-110 arası)
        cycle_pos = elapsed % 120.0
        if 60.0 <= cycle_pos <= 70.0:
            # Girişim spike'ı
            spike = random.uniform(35, 70)
            base_noise += spike

        self._local_noise  = max(0, int(base_noise + random.gauss(0, 3)))
        self._remote_noise = max(0, int(base_noise + random.gauss(0, 5)))

        # RSSI simülasyonu (normal: 80-150 arası)
        self._local_rssi  = max(0, int(120 + random.gauss(0, 10)))
        self._remote_rssi = max(0, int(100 + random.gauss(0, 15)))

        self._scan_count += 1
        self._last_scan_time = time.time()

    # =====================================================================
    # Timer Callback'leri
    # =====================================================================

    def _scan_callback(self):
        """
        Periyodik AT tarama callback'i.
        Simülasyon modunda sentetik veri üretir,
        gerçek modda ATI5 ile RSSI/noise okur.
        """
        if self.simulate_mode:
            self._generate_simulated_scan()
        else:
            if not self._telemetry_connected:
                self._connect_serial()
                if self._telemetry_connected:
                    self._read_frequency_config()

            if self._telemetry_connected:
                success = self._read_rssi_noise()
                if not success:
                    self.get_logger().warn(
                        'RSSI/noise taraması başarısız. '
                        'Seri port bağlantısı kontrol ediliyor...'
                    )
                    # Bağlantıyı yeniden dene
                    self._telemetry_connected = False

        # Güvenlik değerlendirmesi yap
        self._last_safety_state = self._evaluator.evaluate(
            local_noise=self._local_noise,
            remote_noise=self._remote_noise,
            local_rssi=self._local_rssi,
            remote_rssi=self._remote_rssi,
            telemetry_connected=self._telemetry_connected,
        )

        # Girişim tespiti logu
        state = self._last_safety_state
        if state.interference_detected:
            self.get_logger().warn(
                f'⚠ FREKANS GİRİŞİMİ TESPİT EDİLDİ! '
                f'Seviye: {state.interference_level.value} | '
                f'Noise: local={self._local_noise}, '
                f'remote={self._remote_noise} | '
                f'Güvenlik: {state.safety_level.value}'
            )
            # Yeni girişim seviyesi alert'i gönder
            self._publish_interference_alert(state)
        else:
            self.get_logger().debug(
                f'Tarama tamamlandı — Frekans temiz. '
                f'Noise: {self._local_noise}/{self._remote_noise}'
            )

    def _publish_callback(self):
        """
        Periyodik yayın callback'i.
        TelemetrySecurityStatus mesajı oluşturur ve yayınlar.
        """
        msg = TelemetrySecurityStatus()

        # ── Header ──────────────────────────────────────────────────
        msg.header           = Header()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'telemetry_link'

        # ── Bağlantı Durumu ────────────────────────────────────────
        msg.telemetry_connected = self._telemetry_connected

        # ── Frekans Konfigürasyonu ─────────────────────────────────
        if self._min_freq_khz > 0 and self._max_freq_khz > 0:
            msg.operating_frequency_mhz = (
                (self._min_freq_khz + self._max_freq_khz) / 2000.0
            )
        else:
            msg.operating_frequency_mhz = 0.0

        msg.min_frequency_mhz = self._min_freq_khz / 1000.0
        msg.max_frequency_mhz = self._max_freq_khz / 1000.0
        msg.net_id             = self._net_id

        # ── Sinyal Kalitesi ────────────────────────────────────────
        msg.local_rssi   = self._local_rssi
        msg.remote_rssi  = self._remote_rssi
        msg.local_noise  = self._local_noise
        msg.remote_noise = self._remote_noise

        # ── Güvenlik Durumu ────────────────────────────────────────
        state = self._last_safety_state
        msg.interference_detected = state.interference_detected
        msg.interference_level    = state.interference_level.value
        msg.noise_floor_avg       = state.noise_floor_avg

        # ── Tarama Bilgisi ─────────────────────────────────────────
        msg.scan_count = self._scan_count
        if self._last_scan_time > 0.0:
            msg.last_scan_age_sec = time.time() - self._last_scan_time
        else:
            msg.last_scan_age_sec = -1.0

        self._pub_security.publish(msg)

    # =====================================================================
    # Girişim Alert Yayını
    # =====================================================================

    def _publish_interference_alert(self, state: SafetyState):
        """
        Girişim tespitinde JSON formatında alert mesajı yayınlar.

        Aynı seviyede tekrar tekrar alert göndermemek için
        seviye değişikliği kontrolü yapılır.

        Args:
            state: Mevcut güvenlik durumu.
        """
        current_level = state.interference_level

        # Aynı seviyede tekrar alert gönderme
        if current_level == self._last_alerted_level:
            return

        self._last_alerted_level = current_level

        alert_data = {
            'type': 'INTERFERENCE_DETECTED',
            'interference_level': current_level.value,
            'safety_level': state.safety_level.value,
            'local_noise': state.local_noise,
            'remote_noise': state.remote_noise,
            'local_rssi': state.local_rssi,
            'remote_rssi': state.remote_rssi,
            'noise_floor_avg': round(state.noise_floor_avg, 1),
            'operating_frequency_mhz': round(
                (self._min_freq_khz + self._max_freq_khz) / 2000.0, 1
            ),
            'net_id': self._net_id,
            'scan_count': self._scan_count,
            'description': state.description,
            'timestamp': time.time(),
        }

        msg = String()
        msg.data = json.dumps(alert_data, ensure_ascii=False)
        self._pub_alert.publish(msg)

        self.get_logger().warn(
            f'🚨 Girişim alerti yayınlandı: {current_level.value} | '
            f'{state.description}'
        )

    # =====================================================================
    # Temizlik
    # =====================================================================

    def destroy_node(self):
        """
        Node yıkılırken seri portu kapatır.
        """
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
                self.get_logger().info('Seri port kapatıldı.')
            except Exception as e:
                self.get_logger().warn(f'Seri port kapatma hatası: {e}')

        super().destroy_node()


# =============================================================================
# Entry Point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = TelemetrySecurityNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Telemetri Güvenlik Node durduruldu. '
            f'Toplam tarama: {node._scan_count}'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
