#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String


GPS_TOPIC = '/albatros/gps/fix'
IMU_TOPIC = '/albatros/imu/data'

COMMAND_TOPIC = '/albatros/command/cmd_vel'
ARM_COMMAND_TOPIC = '/albatros/command/arm'
MODE_COMMAND_TOPIC = '/albatros/command/mode'
EMERGENCY_STOP_TOPIC = '/albatros/command/emergency_stop'

CONTROL_STATUS_TOPIC = '/albatros/control/status'

MAVROS_STATE_TOPIC = '/mavros/state'
MAVROS_VELOCITY_TOPIC = '/mavros/setpoint_velocity/cmd_vel'
MAVROS_ARM_SERVICE = '/mavros/cmd/arming'
MAVROS_MODE_SERVICE = '/mavros/set_mode'

DEFAULT_PUBLISH_RATE = 20.0
DEFAULT_COMMAND_TIMEOUT = 0.5
DEFAULT_SENSOR_TIMEOUT = 2.0
DEFAULT_MAX_LINEAR_SPEED = 1.5
DEFAULT_MAX_ANGULAR_SPEED = 1.0


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter(
            'publish_rate',
            DEFAULT_PUBLISH_RATE
        )

        self.declare_parameter(
            'command_timeout_sec',
            DEFAULT_COMMAND_TIMEOUT
        )

        self.declare_parameter(
            'sensor_timeout_sec',
            DEFAULT_SENSOR_TIMEOUT
        )

        self.declare_parameter(
            'max_linear_speed',
            DEFAULT_MAX_LINEAR_SPEED
        )

        self.declare_parameter(
            'max_angular_speed',
            DEFAULT_MAX_ANGULAR_SPEED
        )

        self.declare_parameter(
            'require_gps',
            True
        )

        self.declare_parameter(
            'require_imu',
            True
        )

        self.publish_rate = float(
            self.get_parameter('publish_rate').value
        )

        self.command_timeout = float(
            self.get_parameter('command_timeout_sec').value
        )

        self.sensor_timeout = float(
            self.get_parameter('sensor_timeout_sec').value
        )

        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )

        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )

        self.require_gps = bool(
            self.get_parameter('require_gps').value
        )

        self.require_imu = bool(
            self.get_parameter('require_imu').value
        )

        if self.publish_rate <= 0.0:
            self.publish_rate = DEFAULT_PUBLISH_RATE

        if self.command_timeout <= 0.0:
            self.command_timeout = DEFAULT_COMMAND_TIMEOUT

        if self.sensor_timeout <= 0.0:
            self.sensor_timeout = DEFAULT_SENSOR_TIMEOUT

        if self.max_linear_speed <= 0.0:
            self.max_linear_speed = DEFAULT_MAX_LINEAR_SPEED

        if self.max_angular_speed <= 0.0:
            self.max_angular_speed = DEFAULT_MAX_ANGULAR_SPEED

        self.connected = False
        self.armed = False
        self.mode = 'UNKNOWN'
        self.emergency_stop = False

        self.gps_received = False
        self.gps_valid = False

        self.imu_received = False
        self.imu_valid = False

        self.latest_command = Twist()

        self.last_state_time = None
        self.last_gps_time = None
        self.last_imu_time = None
        self.last_command_time = None

        self.arm_future = None
        self.mode_future = None

        self.create_subscription(
            State,
            MAVROS_STATE_TOPIC,
            self.state_callback,
            10
        )

        self.create_subscription(
            NavSatFix,
            GPS_TOPIC,
            self.gps_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Twist,
            COMMAND_TOPIC,
            self.command_callback,
            10
        )

        self.create_subscription(
            Bool,
            ARM_COMMAND_TOPIC,
            self.arm_command_callback,
            10
        )

        self.create_subscription(
            String,
            MODE_COMMAND_TOPIC,
            self.mode_command_callback,
            10
        )

        self.create_subscription(
            Bool,
            EMERGENCY_STOP_TOPIC,
            self.emergency_stop_callback,
            10
        )

        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            MAVROS_VELOCITY_TOPIC,
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            CONTROL_STATUS_TOPIC,
            10
        )

        self.arming_client = self.create_client(
            CommandBool,
            MAVROS_ARM_SERVICE
        )

        self.mode_client = self.create_client(
            SetMode,
            MAVROS_MODE_SERVICE
        )

        self.control_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.control_callback
        )

        self.status_timer = self.create_timer(
            1.0,
            self.publish_status
        )

        self.get_logger().info(
            'Control Node başlatıldı.'
        )

        self.get_logger().info(
            f'Komut girişi: {COMMAND_TOPIC}'
        )

        self.get_logger().info(
            f'MAVROS çıkışı: {MAVROS_VELOCITY_TOPIC}'
        )

        self.get_logger().info(
            f'GPS girişi: {GPS_TOPIC}'
        )

        self.get_logger().info(
            f'IMU girişi: {IMU_TOPIC}'
        )

    def state_callback(self, msg: State):
        self.connected = bool(msg.connected)
        self.armed = bool(msg.armed)
        self.mode = msg.mode
        self.last_state_time = time.monotonic()

    def gps_callback(self, msg: NavSatFix):
        self.gps_received = True
        self.last_gps_time = time.monotonic()

        self.gps_valid = (
            msg.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        )

    def imu_callback(self, msg: Imu):
        self.imu_received = True
        self.last_imu_time = time.monotonic()

        orientation_values = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        angular_velocity_values = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        )

        linear_acceleration_values = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        )

        values_valid = all(
            math.isfinite(value)
            for value in (
                orientation_values
                + angular_velocity_values
                + linear_acceleration_values
            )
        )

        quaternion_norm = math.sqrt(
            msg.orientation.x ** 2
            + msg.orientation.y ** 2
            + msg.orientation.z ** 2
            + msg.orientation.w ** 2
        )

        self.imu_valid = (
            values_valid
            and quaternion_norm > 0.000001
        )

    def command_callback(self, msg: Twist):
        self.latest_command.linear.x = self.clamp(
            self.safe_value(msg.linear.x),
            -self.max_linear_speed,
            self.max_linear_speed
        )

        self.latest_command.linear.y = self.clamp(
            self.safe_value(msg.linear.y),
            -self.max_linear_speed,
            self.max_linear_speed
        )

        self.latest_command.linear.z = self.clamp(
            self.safe_value(msg.linear.z),
            -self.max_linear_speed,
            self.max_linear_speed
        )

        self.latest_command.angular.x = self.clamp(
            self.safe_value(msg.angular.x),
            -self.max_angular_speed,
            self.max_angular_speed
        )

        self.latest_command.angular.y = self.clamp(
            self.safe_value(msg.angular.y),
            -self.max_angular_speed,
            self.max_angular_speed
        )

        self.latest_command.angular.z = self.clamp(
            self.safe_value(msg.angular.z),
            -self.max_angular_speed,
            self.max_angular_speed
        )

        self.last_command_time = time.monotonic()

    def arm_command_callback(self, msg: Bool):
        self.request_arm(bool(msg.data))

    def mode_command_callback(self, msg: String):
        requested_mode = msg.data.strip().upper()

        if requested_mode:
            self.request_mode(requested_mode)

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop = bool(msg.data)

        if self.emergency_stop:
            self.latest_command = Twist()
            self.publish_stop()
            self.request_arm(False)

            self.get_logger().warn(
                'Acil durdurma aktif edildi.'
            )

        else:
            self.get_logger().info(
                'Acil durdurma kaldırıldı.'
            )

    def control_callback(self):
        if not self.control_allowed():
            self.publish_stop()
            return

        command = TwistStamped()

        command.header.stamp = (
            self.get_clock().now().to_msg()
        )

        command.header.frame_id = 'base_link'

        command.twist.linear.x = (
            self.latest_command.linear.x
        )

        command.twist.linear.y = (
            self.latest_command.linear.y
        )

        command.twist.linear.z = (
            self.latest_command.linear.z
        )

        command.twist.angular.x = (
            self.latest_command.angular.x
        )

        command.twist.angular.y = (
            self.latest_command.angular.y
        )

        command.twist.angular.z = (
            self.latest_command.angular.z
        )

        self.velocity_publisher.publish(command)

    def control_allowed(self):
        if self.emergency_stop:
            return False

        if not self.connected:
            return False

        if not self.is_fresh(
            self.last_state_time,
            self.sensor_timeout
        ):
            return False

        if not self.is_fresh(
            self.last_command_time,
            self.command_timeout
        ):
            return False

        if self.require_gps and not self.gps_is_ok():
            return False

        if self.require_imu and not self.imu_is_ok():
            return False

        return True

    def gps_is_ok(self):
        return (
            self.gps_received
            and self.gps_valid
            and self.is_fresh(
                self.last_gps_time,
                self.sensor_timeout
            )
        )

    def imu_is_ok(self):
        return (
            self.imu_received
            and self.imu_valid
            and self.is_fresh(
                self.last_imu_time,
                self.sensor_timeout
            )
        )

    def request_arm(self, arm_value):
        if (
            self.arm_future is not None
            and not self.arm_future.done()
        ):
            return

        if not self.arming_client.service_is_ready():
            self.get_logger().warn(
                'MAVROS arm servisi hazır değil.'
            )
            return

        request = CommandBool.Request()
        request.value = bool(arm_value)

        self.arm_future = (
            self.arming_client.call_async(request)
        )

        self.arm_future.add_done_callback(
            self.arm_response_callback
        )

    def arm_response_callback(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    'Arm/Disarm isteği kabul edildi.'
                )

            else:
                self.get_logger().warn(
                    f'Arm/Disarm isteği reddedildi. '
                    f'Sonuç kodu: {response.result}'
                )

        except Exception as error:
            self.get_logger().error(
                f'Arm servis hatası: {error}'
            )

    def request_mode(self, requested_mode):
        if (
            self.mode_future is not None
            and not self.mode_future.done()
        ):
            return

        if not self.mode_client.service_is_ready():
            self.get_logger().warn(
                'MAVROS mod servisi hazır değil.'
            )
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = requested_mode

        self.mode_future = (
            self.mode_client.call_async(request)
        )

        self.mode_future.add_done_callback(
            self.mode_response_callback
        )

    def mode_response_callback(self, future):
        try:
            response = future.result()

            if response.mode_sent:
                self.get_logger().info(
                    'Mod değiştirme isteği gönderildi.'
                )

            else:
                self.get_logger().warn(
                    'Mod değiştirme isteği reddedildi.'
                )

        except Exception as error:
            self.get_logger().error(
                f'Mod servis hatası: {error}'
            )

    def publish_stop(self):
        if not rclpy.ok():
            return

        stop_command = TwistStamped()

        stop_command.header.stamp = (
            self.get_clock().now().to_msg()
        )

        stop_command.header.frame_id = 'base_link'

        self.velocity_publisher.publish(
            stop_command
        )

    def publish_status(self):
        status = {
            'connected': self.connected,
            'armed': self.armed,
            'mode': self.mode,
            'emergency_stop': self.emergency_stop,

            'state_ok': self.is_fresh(
                self.last_state_time,
                self.sensor_timeout
            ),

            'last_state_age_sec': self.message_age(
                self.last_state_time
            ),

            'gps_received': self.gps_received,
            'gps_valid': self.gps_valid,
            'gps_ok': self.gps_is_ok(),

            'last_gps_age_sec': self.message_age(
                self.last_gps_time
            ),

            'imu_received': self.imu_received,
            'imu_valid': self.imu_valid,
            'imu_ok': self.imu_is_ok(),

            'last_imu_age_sec': self.message_age(
                self.last_imu_time
            ),

            'command_ok': self.is_fresh(
                self.last_command_time,
                self.command_timeout
            ),

            'last_command_age_sec': self.message_age(
                self.last_command_time
            ),

            'control_allowed': self.control_allowed(),

            'arming_service_available':
                self.arming_client.service_is_ready(),

            'set_mode_service_available':
                self.mode_client.service_is_ready()
        }

        message = String()

        message.data = json.dumps(
            status,
            ensure_ascii=False
        )

        self.status_publisher.publish(
            message
        )

    @staticmethod
    def is_fresh(last_time, timeout):
        if last_time is None:
            return False

        return (
            time.monotonic() - last_time
            <= timeout
        )

    @staticmethod
    def message_age(last_time):
        if last_time is None:
            return None

        return round(
            time.monotonic() - last_time,
            3
        )

    @staticmethod
    def safe_value(value):
        if not math.isfinite(value):
            return 0.0

        return float(value)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(
            minimum,
            min(maximum, value)
        )


def main(args=None):
    rclpy.init(args=args)

    node = ControlNode()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if rclpy.ok():
            try:
                node.publish_stop()
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()