#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LODOS Albatros İDA — Ana Launch Dosyası
========================================
Tüm albatros_tahta node'larını tek komutla başlatır.

Kullanım:
    ros2 launch albatros_tahta albatros.launch.py

Katmanlı başlatma sırası:
    1. Sensör Katmanı      : gps_node, imu_node, mesafe_sensor_node, kamera_node
    2. Algılama Katmanı    : yolo_node, parkur3_target_node, duba_fusion_node, costmap_node
    3. Durum & Görev       : state_node, mission_node
    4. Karar & Kontrol     : karar_node, kontrol_node
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Tüm albatros_tahta node'larını oluşturur ve döndürür."""

    pkg = 'albatros_tahta'

    try:
        tahta_share = get_package_share_directory('albatros_tahta')
        default_parkur12_hef = os.path.join(tahta_share, 'models', 'parkur12.hef')
        default_parkur3_hef = os.path.join(tahta_share, 'models', 'parkur3.hef')
    except Exception:
        default_parkur12_hef = 'models/parkur12.hef'
        default_parkur3_hef = 'models/parkur3.hef'

    # ╔══════════════════════════════════════════════════════════════════╗
    # ║  1. SENSÖR KATMANI                                             ║
    # ║  GPS, IMU, mesafe sensörü ve kamera donanım sürücüleri         ║
    # ╚══════════════════════════════════════════════════════════════════╝

    gps_node = Node(
        package=pkg,
        executable='gps_node',
        name='gps_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 9600,
        }],
    )

    imu_node = Node(
        package=pkg,
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{}],
    )

    mesafe_sensor_node = Node(
        package=pkg,
        executable='mesafe_sensor_node',
        name='mesafe_sensor_node',
        output='screen',
        parameters=[{}],
    )

    kamera_node = Node(
        package=pkg,
        executable='kamera_node',
        name='kamera_node',
        output='screen',
        parameters=[{
            'camera_id': 0,
        }],
    )

    # ╔══════════════════════════════════════════════════════════════════╗
    # ║  2. ALGILAMA KATMANI                                           ║
    # ║  YOLO duba tespiti, Parkur 3 Hedef Doğrulama, costmap          ║
    # ╚══════════════════════════════════════════════════════════════════╝

    yolo_node = Node(
        package=pkg,
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'parkur_1_2_model': default_parkur12_hef,
            'parkur12_model_path': default_parkur12_hef,
            'parkur_3_model': default_parkur3_hef,
            'parkur3_model_path': default_parkur3_hef,
            'confidence_threshold': 0.30,
            'model_input_width': 640,
            'model_input_height': 640,
            'save_video': True,
            'video_output_dir': '~/albatros_outputs/yolo_videos',
            'video_fps': 10.0,
        }],
    )

    yolo_mesafe_node = Node(
        package=pkg,
        executable='yolo_mesafe_node',
        name='yolo_mesafe_node',
        output='screen',
        parameters=[{
            'min_yolo_confidence': 0.30,
        }],
    )

    parkur3_target_node = Node(
        package=pkg,
        executable='parkur3_target_node',
        name='parkur3_target_node',
        output='screen',
        parameters=[{
            'yolo_weight': 0.40,
            'opencv_weight': 0.60,
            'final_score_threshold': 0.65,
            'yolo_min_confidence': 0.30,
            'validation_window_size': 5,
            'required_confirmations': 4,
        }],
    )

    duba_fusion_node = Node(
        package=pkg,
        executable='duba_fusion_node',
        name='duba_fusion_node',
        output='screen',
        parameters=[{}],
    )

    costmap_node = Node(
        package=pkg,
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[{
            'local_resolution': 0.20,
            'local_width_cells': 80,
            'local_height_cells': 80,
            'vehicle_forward_ratio': 0.20,
            'inflation_radius': 1.5,
            'decay_time_tentative': 4.0,
            'decay_time_confirmed': 20.0,
            'publish_rate': 5.0,
            'obstacle_timeout': 1.5,
            'association_distance_m': 0.60,
            'confirm_detection_threshold': 2,
            'max_gps_jump_m': 10.0,
            'max_boundary_link_distance_m': 4.50,
            'global_resolution': 0.25,
            'global_width_m': 60.0,
            'global_height_m': 60.0,
        }],
    )

    # ╔══════════════════════════════════════════════════════════════════╗
    # ║  2.5 PARKUR 3 — HEDEF RENK DİNLEYİCİ                          ║
    # ║  İHA/YKİ'den MAVROS üzerinden gelen hedef renk bilgisi         ║
    # ╚══════════════════════════════════════════════════════════════════╝

    target_color_node = Node(
        package=pkg,
        executable='target_color_node',
        name='target_color_node',
        output='screen',
        parameters=[{
            'publish_rate': 2.0,
            'status_rate': 1.0,
            'color_timeout_sec': 30.0,
        }],
    )

    # ╔══════════════════════════════════════════════════════════════════╗
    # ║  3. DURUM ve GÖREV YÖNETİMİ                                   ║
    # ║  Araç durumu birleştirme, görev/waypoint yönetimi              ║
    # ╚══════════════════════════════════════════════════════════════════╝

    state_node = Node(
        package=pkg,
        executable='state_node',
        name='state_node',
        output='screen',
        parameters=[{
            'publish_rate': 10.0,
            'mission_timeout_sec': 2.0,
            'target_timeout_sec': 2.0,
            'imu_timeout_sec': 2.0,
            'control_timeout_sec': 2.0,
        }],
    )

    mission_node = Node(
        package=pkg,
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{
            'gps_timeout_sec': 2.0,
            'waypoint_reached_radius_m': 2.5,
            'required_reached_samples': 3,
            'mission_pull_retry_period_sec': 5.0,
            'publish_period_sec': 0.2,
            'parkur_1_start_wp': 1,
            'parkur_2_start_wp': 4,
            'parkur_3_start_wp': 7,
            'auto_pull_mission_on_startup': True,
            'scan_settle_time_sec': 0.5,
            'yaw_tolerance_deg': 3.0,
            'target_lost_timeout_sec': 2.0,
            'touch_distance_threshold_m': 1.5,
        }],
    )

    # ╔══════════════════════════════════════════════════════════════════╗
    # ║  4. KOMUT ve KONTROL KATMANI                                    ║
    # ║  Waypoint takibi, VFH engel kaçınma, MAVROS motor komutu        ║
    # ╚══════════════════════════════════════════════════════════════════╝

    # NOT: komut_node, eski karar_node'un görevini devralmıştır.
    # Parkur 1 (saf PID) ve Parkur 2 (VFH) navigasyonunu birleştirir.
    # Waypoint listesi komut_node.py dosyasının başında tanımlıdır.
    komut_node = Node(
        package=pkg,
        executable='komut_node',
        name='komut_node',
        output='screen',
        parameters=[{
            # Genel navigasyon
            'publish_rate': 10.0,
            'max_linear_speed': 2.5,
            'min_linear_speed': 0.8,
            'max_angular_speed': 3.0,
            'steering_kp': 1.5,             # 2.5'ten düşürüldü — osilasyon önleme
            'slowdown_distance_m': 4.0,
            'state_timeout_sec': 2.0,
            # Heading filtre (osilasyon önleme)
            'heading_deadband_deg': 5.0,    # ±5° altındaki hataları yoksay
            'heading_filter_alpha': 0.3,    # EMA yumuşatma (0=çok pürüzsüz, 1=filtre yok)
            # VFH parametreleri (Parkur 2)
            'sector_count': 72,
            'vfh_threshold': 0.35,
            'active_region_radius_m': 6.0,
            'vehicle_width_m': 0.60,
            'safety_margin_m': 0.40,
            'costmap_timeout_sec': 1.5,
            'emergency_stop_distance_m': 0.8,
            'cost_goal_weight': 5.0,
            'cost_current_weight': 2.0,
            'cost_previous_weight': 2.0,
            'cost_clearance_weight': 3.0,
            # Engel tespit (hibrit AUTO/GUIDED)
            'obstacle_detection_enabled': True,
            'obstacle_forward_cone_deg': 30.0,
        }],
    )

    kontrol_node = Node(
        package=pkg,
        executable='kontrol_node',
        name='kontrol_node',
        output='screen',
        parameters=[{
            'publish_rate': 20.0,
            'command_timeout_sec': 0.5,
            'sensor_timeout_sec': 2.0,
            'max_linear_speed': 3.0,
            'max_angular_speed': 3.0,
            'require_gps': True,
            'require_imu': True,
            # Hibrit AUTO/GUIDED mod geçiş parametreleri
            # NOT: auto_mode_obstacle_switching=False olarak ayarlandı.
            # P1 ve P2'de araç her zaman AUTO modda kalır.
            # Engel algılaması GUIDED geçişini tetiklemez.
            # P3 mod geçişi yalnızca mission_node'un gate açılımından sonra
            # /albatros/command/mode topic'i üzerinden gerçekleşir.
            'default_mode': 'AUTO',
            'mode_switch_cooldown_sec': 2.0,
            'auto_mode_obstacle_switching': False,
        }],
    )

    return LaunchDescription([

        LogInfo(msg='═══════════════════════════════════════════════════'),
        LogInfo(msg='  LODOS Albatros İDA — Sistem Başlatılıyor...     '),
        LogInfo(msg='═══════════════════════════════════════════════════'),

        gps_node,
        imu_node,
        mesafe_sensor_node,
        kamera_node,

        TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg='[LAUNCH] Algılama katmanı (YOLO & Parkur3 perception) başlatılıyor...'),
                yolo_node,
                yolo_mesafe_node,
                parkur3_target_node,
                duba_fusion_node,
                costmap_node,
            ],
        ),

        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg='[LAUNCH] Durum ve görev katmanı başlatılıyor...'),
                state_node,
                mission_node,
                target_color_node,
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[LAUNCH] Komut ve kontrol katmanı (Navigasyon & Motor) başlatılıyor...'),
                komut_node,
                kontrol_node,
                LogInfo(msg='═══════════════════════════════════════════════════'),
                LogInfo(msg='  LODOS Albatros İDA — Tüm Node\'lar Başlatıldı!  '),
                LogInfo(msg='═══════════════════════════════════════════════════'),
            ],
        ),
    ])
