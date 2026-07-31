# TelemetrySecurityStatus.msg
# LODOS Albatros — Telemetri Güvenlik Durumu Mesajı
# telemetry_security_node tarafından /albatros/telemetry/security topic'ine yayınlanır.
# 3DR 915 MHz telemetri modülünden AT komutlarıyla okunan
# RSSI/noise verilerine göre frekans çakışma tespiti bilgisi taşır.

std_msgs/Header header

# --- Telemetri Modül Bağlantı Durumu ---
bool telemetry_connected          # Modüle seri port üzerinden bağlı mı

# --- Frekans Konfigürasyonu ---
float64 operating_frequency_mhz   # Mevcut çalışma frekansı (MHz)
float64 min_frequency_mhz         # Min frekans (ATS8)
float64 max_frequency_mhz         # Max frekans (ATS9)
uint8 net_id                      # Ağ kimliği (ATS3)

# --- Sinyal Kalitesi ---
int16 local_rssi                  # Yerel RSSI (dBm)
int16 remote_rssi                 # Uzak RSSI (dBm)
int16 local_noise                 # Yerel gürültü seviyesi (dBm)
int16 remote_noise                # Uzak gürültü seviyesi (dBm)

# --- Güvenlik Durumu ---
bool interference_detected        # Aynı frekansta başka cihaz tespit edildi mi
string interference_level         # NONE / LOW / MEDIUM / HIGH / CRITICAL
float64 noise_floor_avg           # Ortalama gürültü tabanı (dBm)

# --- Tarama Bilgisi ---
uint32 scan_count                 # Toplam yapılan tarama sayısı
float64 last_scan_age_sec         # Son taramadan bu yana geçen süre (saniye)
