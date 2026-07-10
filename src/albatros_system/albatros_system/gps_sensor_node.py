#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus


GPS_TOPIC = '/albatros/gps/fix'
GPS_FRAME_ID = 'gps_link'

DEFAULT_PUBLISH_RATE = 10.0

DEFAULT_LATITUDE = 40.1885
DEFAULT_LONGITUDE = 29.0610
DEFAULT_ALTITUDE = 0.0

DEFAULT_POSITION_NOISE = 0.000005
POSITION_COV_DIAG = 6.25


class GpsSensorNode(Node):

    def __init__(self):
        super().__init__('gps_sensor_node')

        self.declare_parameter('simulate_mode', True)
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE)
        self.declare_parameter('base_latitude', DEFAULT_LATITUDE)
        self.declare_parameter('base_longitude', DEFAULT_LONGITUDE)
        self.declare_parameter('base_altitude', DEFAULT_ALTITUDE)
        self.declare_parameter('position_noise', DEFAULT_POSITION_NOISE)

        self.simulate_mode = self.get_parameter('simulate_mode').value
        raw_rate = self.get_parameter('publish_rate').value
        raw_lat = self.get_parameter('base_latitude').value
        raw_lon = self.get_parameter('base_longitude').value
        self.base_altitude = self.get_parameter('base_altitude').value
        raw_noise = self.get_parameter('position_noise').value

        if raw_rate <= 0.0:
            self.get_logger().warn(
                f'publish_rate={raw_rate} geçersiz. '
                f'Varsayılan {DEFAULT_PUBLISH_RATE} Hz kullanılıyor.'
            )
            self.publish_rate = DEFAULT_PUBLISH_RATE
        else:
            self.publish_rate = raw_rate

        if not (-90.0 <= raw_lat <= 90.0):
            self.get_logger().warn(
                f'base_latitude={raw_lat} geçersiz ([-90, +90] dışında). '
                f'Varsayılan {DEFAULT_LATITUDE}° kullanılıyor.'
            )
            self.base_latitude = DEFAULT_LATITUDE
        else:
            self.base_latitude = raw_lat

        if not (-180.0 <= raw_lon <= 180.0):
            self.get_logger().warn(
                f'base_longitude={raw_lon} geçersiz ([-180, +180] dışında). '
                f'Varsayılan {DEFAULT_LONGITUDE}° kullanılıyor.'
            )
            self.base_longitude = DEFAULT_LONGITUDE
        else:
            self.base_longitude = raw_lon

        if raw_noise < 0.0:
            self.get_logger().warn(
                f'position_noise={raw_noise} negatif, geçersiz. '
                f'Varsayılan {DEFAULT_POSITION_NOISE} kullanılıyor.'
            )
            self.position_noise = DEFAULT_POSITION_NOISE
        else:
            self.position_noise = raw_noise

        self._latest_gps = None

        if not self.simulate_mode:
            self._mavros_gps_sub = self.create_subscription(
                NavSatFix,
                '/mavros/global_position/global',
                self._mavros_gps_callback,
                qos_profile_sensor_data
            )

            self.get_logger().info(
                'MAVROS GPS köprüsü aktif: '
                '/mavros/global_position/global -> '
                f'{GPS_TOPIC}'
            )

        self.gps_publisher = self.create_publisher(
            NavSatFix,
            GPS_TOPIC,
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        mode_str = (
            'SİMÜLASYON (simulate_mode=True)'
            if self.simulate_mode
            else 'GERÇEK SENSÖR (simulate_mode=False)'
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('GPS Sensör Node başlatıldı.')
        self.get_logger().info(f'Mod: {mode_str}')
        self.get_logger().info(f'Topic: {GPS_TOPIC}')
        self.get_logger().info(
            f'Yayın frekansı: {self.publish_rate} Hz'
        )
        self.get_logger().info(f'Frame ID: {GPS_FRAME_ID}')
        self.get_logger().info(
            f'Başlangıç Lat: {self.base_latitude}°'
        )
        self.get_logger().info(
            f'Başlangıç Lon: {self.base_longitude}°'
        )
        self.get_logger().info(
            f'Başlangıç Alt: {self.base_altitude} m'
        )
        self.get_logger().info('=' * 60)

    def timer_callback(self):
        if self.simulate_mode:
            gps_data = self.generate_simulated_gps()
        else:
            gps_data = self.read_gps_sensor()

        if gps_data is not None:
            self.gps_publisher.publish(gps_data)

    def generate_simulated_gps(self) -> NavSatFix:
        msg = NavSatFix()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = GPS_FRAME_ID

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = (
            self.base_latitude
            + random.gauss(0.0, self.position_noise)
        )

        msg.longitude = (
            self.base_longitude
            + random.gauss(0.0, self.position_noise)
        )

        msg.altitude = (
            self.base_altitude
            + random.gauss(0.0, 0.1)
        )

        msg.position_covariance = [
            POSITION_COV_DIAG, 0.0, 0.0,
            0.0, POSITION_COV_DIAG, 0.0,
            0.0, 0.0, POSITION_COV_DIAG * 4.0
        ]

        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        )

        return msg

    def _mavros_gps_callback(
        self,
        msg: NavSatFix
    ) -> None:
        msg.header.frame_id = GPS_FRAME_ID
        self._latest_gps = msg

    def read_gps_sensor(self):
        return self._latest_gps


def main(args=None):
    rclpy.init(args=args)

    node = GpsSensorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'GPS Sensör Node durduruldu.'
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()