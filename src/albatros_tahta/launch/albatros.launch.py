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
    2. Algılama Katmanı    : yolo_node, duba_fusion_node, costmap_node
    3. Durum & Görev       : state_node, mission_node
    4. Karar & Kontrol     : karar_node, kontrol_node

Not: MAVROS ayrı bir terminalde veya ayrı bir launch dosyasıyla başlatılmalıdır.
     ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyACM0:57600

Yazar  : LODOS Takımı
Araç   : Albatros İDA
Ortam  : Ubuntu 24.04 / ROS2 Jazzy
"""

from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Tüm albatros_tahta node'larını oluşturur ve döndürür."""

    pkg = 'albatros_tahta'

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
            # GPS seri port ayarları — donanıma göre güncelle
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 9600,
        }],
    )

    imu_node = Node(
        package=pkg,
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            # IMU/pusula verisi MAVROS üzerinden gelir
        }],
    )

    mesafe_sensor_node = Node(
        package=pkg,
        executable='mesafe_sensor_node',
        name='mesafe_sensor_node',
        output='screen',
        parameters=[{
            # Ultrasonik mesafe sensörü ayarları
        }],
    )

    kamera_node = Node(
        package=pkg,
        executable='kamera_node',
        name='kamera_node',
        output='screen',
        parameters=[{
            # Kamera cihaz ID'si
            'camera_id': 0,
        }],
    )

    # ╔══════════════════════════════════════════════════════════════════╗
    # ║  2. ALGILAMA KATMANI                                           ║
    # ║  YOLO duba tespiti, duba birleştirme, costmap oluşturma        ║
    # ╚══════════════════════════════════════════════════════════════════╝

    yolo_node = Node(
        package=pkg,
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'model_path': 'models/yolov11s.hef',
            'confidence_threshold': 0.50,
            'model_input_width': 640,
            'model_input_height': 640,
        }],
    )

    duba_fusion_node = Node(
        package=pkg,
        executable='duba_fusion_node',
        name='duba_fusion_node',
        output='screen',
        parameters=[{
            # Duba birleştirme ve hedef kilitleme parametreleri
        }],
    )

    costmap_node = Node(
        package=pkg,
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[{
            # Costmap engel haritası parametreleri
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
            # ─── Parkur waypoint sınırları ───
            # Yarışma günü verilen waypoint sayısına göre güncelle!
            # Örnek: WP1-WP3 → Parkur1, WP4-WP6 → Parkur2, WP7+ → Parkur3
            'parkur_1_start_wp': 1,
            'parkur_2_start_wp': 4,
            'parkur_3_start_wp': 7,
            'auto_pull_mission_on_startup': True,
        }],
    )

    # ╔══════════════════════════════════════════════════════════════════╗
    # ║  4. KARAR ve KONTROL KATMANI                                   ║
    # ║  VFH engel kaçınma kararı, MAVROS motor komutu icrası          ║
    # ╚══════════════════════════════════════════════════════════════════╝

    karar_node = Node(
        package=pkg,
        executable='karar_node',
        name='karar_node',
        output='screen',
        parameters=[{
            # VFH algoritma parametreleri
            'sector_count': 72,
            'vfh_threshold': 0.3,
            'active_region_radius': 6.0,
            'vehicle_width': 0.85,
            'safety_margin': 0.5,
            'max_linear_speed': 1.0,
            'min_linear_speed': 0.2,
            'max_angular_speed': 0.8,
            'cost_goal_weight': 5.0,
            'cost_current_weight': 2.0,
            'cost_previous_weight': 2.0,
            'slowdown_distance': 2.0,
            'publish_rate': 10.0,
            'steering_kp': 1.5,
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
            'max_linear_speed': 1.5,
            'max_angular_speed': 1.0,
            'require_gps': True,
            'require_imu': True,
        }],
    )

    # ╔══════════════════════════════════════════════════════════════════╗
    # ║  LAUNCH AKIŞI                                                  ║
    # ║  Sensörler → Algılama → Durum/Görev → Karar/Kontrol            ║
    # ╚══════════════════════════════════════════════════════════════════╝

    return LaunchDescription([

        # ── Başlangıç bildirimi ──
        LogInfo(msg='═══════════════════════════════════════════════════'),
        LogInfo(msg='  LODOS Albatros İDA — Sistem Başlatılıyor...     '),
        LogInfo(msg='═══════════════════════════════════════════════════'),

        # ── Katman 1: Sensörler (hemen başlat) ──
        gps_node,
        imu_node,
        mesafe_sensor_node,
        kamera_node,

        # ── Katman 2: Algılama (1 sn gecikme — sensörlerin hazır olmasını bekle) ──
        TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg='[LAUNCH] Algılama katmanı başlatılıyor...'),
                yolo_node,
                duba_fusion_node,
                costmap_node,
            ],
        ),

        # ── Katman 3: Durum & Görev (2 sn gecikme) ──
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg='[LAUNCH] Durum, görev ve hedef renk katmanı başlatılıyor...'),
                state_node,
                mission_node,
                target_color_node,
            ],
        ),

        # ── Katman 4: Karar & Kontrol (3 sn gecikme — tüm veri akışı hazır) ──
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[LAUNCH] Karar ve kontrol katmanı başlatılıyor...'),
                karar_node,
                kontrol_node,
                LogInfo(msg='═══════════════════════════════════════════════════'),
                LogInfo(msg='  LODOS Albatros İDA — Tüm Node\'lar Başlatıldı!  '),
                LogInfo(msg='═══════════════════════════════════════════════════'),
            ],
        ),
    ])
