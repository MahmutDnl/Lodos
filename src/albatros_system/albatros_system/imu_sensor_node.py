#!/usr/bin/env python3

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


IMU_TOPIC = '/albatros/imu/data'
IMU_FRAME_ID = 'imu_link'

DEFAULT_PUBLISH_RATE = 50.0

GRAVITY = 9.81

ORIENT_COV_DIAG = 1e-4
ANG_VEL_COV_DIAG = 1e-5
LIN_ACC_COV_DIAG = 1e-3


class ImuSensorNode(Node):

    def __init__(self):
        super().__init__('imu_sensor_node')

        self.declare_parameter(
            'simulate_mode',
            True
        )

        self.declare_parameter(
            'publish_rate',
            DEFAULT_PUBLISH_RATE
        )

        self.simulate_mode = self.get_parameter(
            'simulate_mode'
        ).value

        raw_rate = self.get_parameter(
            'publish_rate'
        ).value

        if raw_rate <= 0.0:
            self.get_logger().warn(
                f'publish_rate={raw_rate} geçersiz. '
                f'Varsayılan {DEFAULT_PUBLISH_RATE} Hz kullanılıyor.'
            )

            self.publish_rate = DEFAULT_PUBLISH_RATE

        else:
            self.publish_rate = raw_rate

        self._real_sensor_warning_printed = False
        self._latest_imu = None

        if not self.simulate_mode:
            self._mavros_imu_sub = self.create_subscription(
                Imu,
                '/mavros/imu/data',
                self._mavros_imu_callback,
                qos_profile_sensor_data
            )

            self.get_logger().info(
                'MAVROS IMU köprüsü aktif: '
                '/mavros/imu/data -> '
                f'{IMU_TOPIC}'
            )

        self.imu_publisher = self.create_publisher(
            Imu,
            IMU_TOPIC,
            10
        )

        timer_period = 1.0 / self.publish_rate

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        self._start_time = time.time()

        mode_str = (
            'SİMÜLASYON (simulate_mode=True)'
            if self.simulate_mode
            else 'GERÇEK SENSÖR (simulate_mode=False)'
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('IMU Sensör Node başlatıldı.')
        self.get_logger().info(f'Mod: {mode_str}')
        self.get_logger().info(f'Topic: {IMU_TOPIC}')

        self.get_logger().info(
            f'Yayın frekansı: {self.publish_rate} Hz'
        )

        self.get_logger().info(
            f'Frame ID: {IMU_FRAME_ID}'
        )

        self.get_logger().info('=' * 60)

    def timer_callback(self):
        if self.simulate_mode:
            imu_data = self.generate_simulated_imu()

        else:
            imu_data = self.read_imu_sensor()

        if imu_data is not None:
            self.imu_publisher.publish(imu_data)

    def generate_simulated_imu(self) -> Imu:
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = IMU_FRAME_ID

        elapsed = time.time() - self._start_time

        roll = 0.01 * math.sin(
            0.5 * elapsed
        )

        pitch = 0.005 * math.cos(
            0.3 * elapsed
        )

        yaw = 0.0

        qx, qy, qz, qw = self._euler_to_quaternion(
            roll,
            pitch,
            yaw
        )

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance = [
            ORIENT_COV_DIAG,
            0.0,
            0.0,
            0.0,
            ORIENT_COV_DIAG,
            0.0,
            0.0,
            0.0,
            ORIENT_COV_DIAG
        ]

        msg.angular_velocity.x = random.gauss(
            0.0,
            0.001
        )

        msg.angular_velocity.y = random.gauss(
            0.0,
            0.001
        )

        msg.angular_velocity.z = random.gauss(
            0.0,
            0.001
        )

        msg.angular_velocity_covariance = [
            ANG_VEL_COV_DIAG,
            0.0,
            0.0,
            0.0,
            ANG_VEL_COV_DIAG,
            0.0,
            0.0,
            0.0,
            ANG_VEL_COV_DIAG
        ]

        msg.linear_acceleration.x = random.gauss(
            0.0,
            0.01
        )

        msg.linear_acceleration.y = random.gauss(
            0.0,
            0.01
        )

        msg.linear_acceleration.z = (
            GRAVITY
            + random.gauss(0.0, 0.05)
        )

        msg.linear_acceleration_covariance = [
            LIN_ACC_COV_DIAG,
            0.0,
            0.0,
            0.0,
            LIN_ACC_COV_DIAG,
            0.0,
            0.0,
            0.0,
            LIN_ACC_COV_DIAG
        ]

        return msg

    def _mavros_imu_callback(
        self,
        msg: Imu
    ) -> None:
        msg.header.frame_id = IMU_FRAME_ID
        self._latest_imu = msg

    def read_imu_sensor(self):
        return self._latest_imu

    @staticmethod
    def _euler_to_quaternion(
        roll: float,
        pitch: float,
        yaw: float
    ):
        cr = math.cos(
            roll * 0.5
        )

        sr = math.sin(
            roll * 0.5
        )

        cp = math.cos(
            pitch * 0.5
        )

        sp = math.sin(
            pitch * 0.5
        )

        cy = math.cos(
            yaw * 0.5
        )

        sy = math.sin(
            yaw * 0.5
        )

        qw = (
            cr * cp * cy
            + sr * sp * sy
        )

        qx = (
            sr * cp * cy
            - cr * sp * sy
        )

        qy = (
            cr * sp * cy
            + sr * cp * sy
        )

        qz = (
            cr * cp * sy
            - sr * sp * cy
        )

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)

    node = ImuSensorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'IMU Sensör Node durduruldu.'
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()