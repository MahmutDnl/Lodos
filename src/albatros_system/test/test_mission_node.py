import pytest
import math
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import WaypointList, Waypoint, WaypointReached
from albatros_system.mission_node import MissionNode


@pytest.fixture(scope="module", autouse=True)
def setup_ros():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_distance_and_bearing():
    # 1. Aynı konumda mesafe 0 metre
    d = MissionNode.calculate_distance_m(40.0, 29.0, 40.0, 29.0)
    assert d == 0.0

    # 2. Kuzey hedefi için bearing yaklaşık 0 derece
    b_north = MissionNode.calculate_initial_bearing_deg(40.0, 29.0, 41.0, 29.0)
    assert pytest.approx(b_north, abs=0.5) == 0.0 or pytest.approx(b_north, abs=0.5) == 360.0

    # 3. Doğu hedefi için bearing yaklaşık 90 derece
    b_east = MissionNode.calculate_initial_bearing_deg(40.0, 29.0, 40.0, 30.0)
    assert pytest.approx(b_east, abs=0.5) == 90.0

    # 4. Güney hedefi için bearing yaklaşık 180 derece
    b_south = MissionNode.calculate_initial_bearing_deg(40.0, 29.0, 39.0, 29.0)
    assert pytest.approx(b_south, abs=0.5) == 180.0

    # 5. Batı hedefi için bearing yaklaşık 270 derece
    b_west = MissionNode.calculate_initial_bearing_deg(40.0, 29.0, 40.0, 28.0)
    assert pytest.approx(b_west, abs=0.5) == 270.0


def test_gps_validation():
    # 6. Geçersiz GPS koordinatı reddedilmeli
    node = MissionNode()
    
    # Başlangıçta GPS alınmadı hatası
    node.gps_received = False
    assert node.validate_gps() is False
    assert node.error_code == "GPS_NOT_RECEIVED"

    # Geçersiz koordinat callback üzerinden
    msg = NavSatFix()
    msg.status.status = NavSatStatus.STATUS_NO_FIX
    msg.latitude = float('nan')
    msg.longitude = 29.0
    node.gps_callback(msg)
    assert node.gps_valid is False


def test_duplicate_reached():
    # 7. Tekrarlanan reached mesajı waypoint’i iki kere ilerletmemeli
    node = MissionNode()
    
    # Örnek waypoint listesi oluştur (Home seq=0, WP1 seq=1, WP2 seq=2)
    msg = WaypointList()
    msg.current_seq = 1
    
    wp_home = Waypoint()
    wp_home.command = 16
    wp_home.x_lat = 40.0
    wp_home.y_long = 29.0
    
    wp1 = Waypoint()
    wp1.command = 16
    wp1.x_lat = 40.1
    wp1.y_long = 29.1
    
    wp2 = Waypoint()
    wp2.command = 16
    wp2.x_lat = 40.2
    wp2.y_long = 29.2
    
    msg.waypoints = [wp_home, wp1, wp2]
    node.waypoint_list_callback(msg)
    
    assert len(node.nav_waypoints) == 2
    assert node.active_waypoint['seq'] == 1
    
    # İlk reached uyarısı
    r_msg = WaypointReached()
    r_msg.wp_seq = 1
    node.waypoint_reached_callback(r_msg)
    
    assert node.reached_waypoint_count == 1
    assert node.active_waypoint['seq'] == 2
    
    # Tekrarlanan reached uyarısı (ilerlememeli, count 1 kalmalı)
    node.waypoint_reached_callback(r_msg)
    assert node.reached_waypoint_count == 1
    assert node.active_waypoint['seq'] == 2


def test_mission_completed():
    # 8. Son waypoint tamamlanınca görev COMPLETED olmalı
    node = MissionNode()
    
    # Tek waypoint'li görev (Home seq=0, WP1 seq=1)
    msg = WaypointList()
    msg.current_seq = 1
    
    wp_home = Waypoint()
    wp_home.command = 16
    wp_home.x_lat = 40.0
    wp_home.y_long = 29.0
    
    wp1 = Waypoint()
    wp1.command = 16
    wp1.x_lat = 40.1
    wp1.y_long = 29.1
    
    msg.waypoints = [wp_home, wp1]
    node.waypoint_list_callback(msg)
    
    assert len(node.nav_waypoints) == 1
    assert node.active_waypoint['seq'] == 1
    
    # Waypoint'e ulaşıldı
    r_msg = WaypointReached()
    r_msg.wp_seq = 1
    node.waypoint_reached_callback(r_msg)
    
    assert node.reached_waypoint_count == 1
    assert node.active_waypoint is None
    assert node.mission_state == "COMPLETED"
    assert node.mission_completed is True
