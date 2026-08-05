std_msgs/Header header

string mission_state

bool mission_loaded
bool mission_active
bool mission_completed
bool mission_error

string error_code

uint8 current_parkur

int32 current_waypoint_seq
uint32 total_navigation_waypoints
uint32 reached_waypoint_count

bool target_valid
bool target_reached

float64 target_latitude
float64 target_longitude
float64 distance_to_target_m
float64 target_bearing_deg

bool gps_valid
bool mavros_connected

# Constants for current_parkur
uint8 PARKUR_UNKNOWN = 0
uint8 PARKUR_1 = 1
uint8 PARKUR_2 = 2
uint8 PARKUR_3 = 3
uint8 PARKUR_COMPLETE = 4
