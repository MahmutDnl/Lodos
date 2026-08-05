# VehicleState.msg
# LODOS Albatros — Birleşik Araç Durumu Mesajı
# state_node tarafından /albatros/state topic'ine yayınlanır.
# Bu mesaj motor komutu veya görev geçiş bilgisi içermez;
# yalnızca toplanan ve hesaplanan araç durumunu taşır.

std_msgs/Header header

# --- Görev (Mission) Durumu ---
bool mission_active
bool mission_completed
bool mission_error
string mission_state
string error_code

int32 current_parkur
int32 current_waypoint_seq

# --- Hedef (Target) Bilgisi ---
bool target_valid
bool target_reached

float64 target_latitude
float64 target_longitude
float64 distance_to_target_m
float64 target_bearing_deg

# --- IMU / Yönelim Bilgisi ---
bool imu_valid

float64 current_yaw_deg
float64 heading_error_deg
string turn_direction

# --- Sistem Sağlık Bilgisi ---
bool mavros_connected
bool armed
string mode
bool emergency_stop

bool gps_ok
bool imu_ok
bool control_allowed
