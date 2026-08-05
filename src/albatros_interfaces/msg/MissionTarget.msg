std_msgs/Header header

bool target_valid
bool mission_active

uint8 current_parkur
int32 waypoint_seq

float64 target_latitude
float64 target_longitude
float64 distance_to_target_m
float64 target_bearing_deg

# Constants for current_parkur
uint8 PARKUR_UNKNOWN = 0
uint8 PARKUR_1 = 1
uint8 PARKUR_2 = 2
uint8 PARKUR_3 = 3
uint8 PARKUR_COMPLETE = 4
