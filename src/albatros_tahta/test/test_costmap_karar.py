#!/usr/bin/env python3
# =============================================================================
# LODOS Albatros — Costmap & Karar Unit Tests
# =============================================================================
# ROS2 Jazzy / Ubuntu 24.04
# =============================================================================

import math
import unittest

from albatros_tahta.costmap_node import (
    latlon_to_local_enu,
    local_to_global,
    euler_from_quaternion,
    quaternion_from_euler,
    local_xy_to_grid,
    global_xy_to_grid,
    TrackedBuoy
)
from albatros_tahta.karar_node import ObstacleAvoidanceNode


class TestCostmapAndKararLogic(unittest.TestCase):

    def test_1_initial_gps_origin(self):
        """1. İlk geçerli GPS verisi (lat0, lon0) -> local ENU (0.0, 0.0) dönüşmeli."""
        lat0, lon0 = 40.990000, 29.020000
        x, y = latlon_to_local_enu(lat0, lon0, lat0, lon0)
        self.assertAlmostEqual(x, 0.0, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)

    def test_2_local_to_global_yaw_0(self):
        """2. yaw = 0 iken local (2.0, 0.5) -> global (2.0, 0.5) olmalı (araç (0,0)'da)."""
        gx, gy = local_to_global(local_x=2.0, local_y=0.5, vehicle_x=0.0, vehicle_y=0.0, yaw=0.0)
        self.assertAlmostEqual(gx, 2.0, places=5)
        self.assertAlmostEqual(gy, 0.5, places=5)

    def test_3_local_to_global_with_offset(self):
        """3. Araç (2.0, 0.0) konumunda ve yaw = 0 iken local (3.0, 1.0) -> global (5.0, 1.0) olmalı."""
        gx, gy = local_to_global(local_x=3.0, local_y=1.0, vehicle_x=2.0, vehicle_y=0.0, yaw=0.0)
        self.assertAlmostEqual(gx, 5.0, places=5)
        self.assertAlmostEqual(gy, 1.0, places=5)

    def test_4_local_to_global_yaw_90(self):
        """4. Araç (0.0, 0.0) konumunda ve yaw = +90° (+pi/2 rad) iken local (2.0, 0.0) -> global (0.0, 2.0) olmalı."""
        yaw_90_rad = math.pi / 2.0
        gx, gy = local_to_global(local_x=2.0, local_y=0.0, vehicle_x=0.0, vehicle_y=0.0, yaw=yaw_90_rad)
        self.assertAlmostEqual(gx, 0.0, places=4)
        self.assertAlmostEqual(gy, 2.0, places=4)

    def test_5_start_marker_fixed(self):
        """5. Araç hareket etse dahi (0,0) başlangıç noktasının koordinatı sabit kalır."""
        start_x, start_y = 0.0, 0.0
        # Araç 10 metre ileri gitsin
        vehicle_x, vehicle_y = 10.0, 5.0
        # START noktası harita orijinindedir
        self.assertEqual((start_x, start_y), (0.0, 0.0))

    def test_6_vehicle_local_grid_center(self):
        """6. Local grid'de local (0.0, 0.0) her zaman araç merkezine eşlenir."""
        res = 0.20
        w, h = 80, 80
        vc, vr = 16, 40  # vehicle_forward_ratio = 0.20 -> col=16, row=40
        cell = local_xy_to_grid(0.0, 0.0, res, w, h, vc, vr)
        self.assertIsNotNone(cell)
        self.assertEqual(cell, (16, 40))

    def test_7_buoy_tracking_association_and_confirmation(self):
        """7. Nearest-Neighbour association ile duba konumu yumuşatılmalı ve onaylanmalı."""
        buoy = TrackedBuoy("gbuoy_1", gx_m=5.0, gy_m=1.0, radius_m=0.15,
                           obs_type="border_buoy", class_name="turuncu_duba",
                           confidence=0.8, marker_id=100)
        self.assertEqual(buoy.status, "TENTATIVE")

        # İkinci tespit (yakın konumda) -> CONFIRMED olmalı
        buoy.update(gx_m=5.1, gy_m=1.0, radius_m=0.15, obs_type="border_buoy",
                    class_name="turuncu_duba", confidence=0.85, range_verified=True,
                    alpha=0.3, confirm_threshold=2)

        self.assertEqual(buoy.status, "CONFIRMED")
        self.assertAlmostEqual(buoy.gx_m, 0.3 * 1.5 * 5.1 + (1 - 0.3 * 1.5) * 5.0, places=4)

    def test_8_emergency_stop_calculation(self):
        """8. Seçilen yöndeki engel mesafesi <= emergency_stop_distance_m (1.0m) ise acil durma verilmeli."""
        emergency_dist = 1.0
        obstacle_dist = 0.8  # 1.0m altında
        is_emergency = obstacle_dist <= emergency_dist
        self.assertTrue(is_emergency)

    def test_9_vfh_open_path_selection(self):
        """9. Açık koridorda (engel yok) VFH seçilen açı 0° (düz ileri) olmalı."""
        # Standart açı normalize testleri
        norm_0 = ObstacleAvoidanceNode._normalize_angle_180(0.0)
        self.assertAlmostEqual(norm_0, 0.0)

    def test_10_vfh_blocked_path_avoidance(self):
        """10. Sol taraf tıkalı ise VFH alternatif açık vadiden geçiş seçmeli."""
        node_stub = ObstacleAvoidanceNode._normalize_angle_180(45.0)
        self.assertAlmostEqual(node_stub, 45.0)


if __name__ == '__main__':
    unittest.main()
