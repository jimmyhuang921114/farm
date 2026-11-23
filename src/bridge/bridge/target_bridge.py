"""Relay ROS 2 person targets and STM32 velocity replies over UART."""

import math
import time

from geometry_msgs.msg import PointStamped, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import serial
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from .protocol import decode_cmd_vel, encode_person_lost, encode_person_position
from .uart_rx import UartReceiver
from .uart_tx import send_packet


class TargetBridge(Node):
    """Own the serial port and connect UART messages to ROS 2 topics."""

    def __init__(self):
        """Declare bridge settings and create the subscription and UART poll."""
        super().__init__('target_bridge')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        baudrate = self.declare_parameter('baudrate', 115200).value
        target_topic = self.declare_parameter('target_topic', '/target/point').value
        self._base_frame = self.declare_parameter('base_frame', 'base_link').value
        self._person_timeout = self.declare_parameter('person_timeout', 0.3).value
        if not math.isfinite(self._person_timeout) or self._person_timeout <= 0:
            raise ValueError('person_timeout must be finite and positive')

        self._last_person = None
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self._subscription = self.create_subscription(
            PointStamped, target_topic, self._receive_target, qos_profile_sensor_data
        )
        self._timer = self.create_timer(0.01, self._poll_uart)
        self._port = serial.Serial(port, baudrate, timeout=0, write_timeout=0.05)
        self._receiver = UartReceiver(self._port)

    def _send(self, packet):
        try:
            send_packet(self._port, packet)
            return True
        except OSError as exc:
            self.get_logger().error(f'UART write failed: {exc}', throttle_duration_sec=2.0)
            return False

    def _receive_target(self, message):
        frame = message.header.frame_id
        if not frame:
            return
        if frame != self._base_frame:
            try:
                transform = self._tf.lookup_transform(
                    self._base_frame, frame, Time.from_msg(message.header.stamp)
                )
                message = do_transform_point(message, transform)
            except TransformException as exc:
                self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
                return

        point = message.point
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            return

        if self._send(encode_person_position(point.x, point.y, point.z)):
            self._last_person = time.monotonic()

    def _poll_uart(self):
        if (self._last_person is not None
                and time.monotonic() - self._last_person >= self._person_timeout):
            self._last_person = None
            self._send(encode_person_lost())

        try:
            for line in self._receiver.read_lines():
                velocity = decode_cmd_vel(line)
                if velocity is not None:
                    message = Twist()
                    message.linear.x, message.angular.z = velocity
                    self._velocity.publish(message)
        except OSError as exc:
            self.get_logger().error(f'UART read failed: {exc}', throttle_duration_sec=2.0)

    def destroy_node(self):
        """Release the serial port when the bridge shuts down."""
        self._port.close()
        return super().destroy_node()


def main(args=None):
    """Run the target bridge node."""
    rclpy.init(args=args)
    node = None
    try:
        node = TargetBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
