import math
import time
import serial
import json
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .api import RobotAPI
from .uart_transport import UARTTransport


class STM32Bridge(Node):
    def __init__(self):
        super().__init__("stm32_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("person_timeout", 0.3)
        self.rx_timer = self.create_timer(0.01,self.receive_uart)
        port = self.get_parameter("port").value
        baudrate = int(self.get_parameter("baudrate").value)
        self.person_timeout = float(
            self.get_parameter("person_timeout").value
        )
        self.rx_debug_timer = self.create_timer(
            1.0,
            self.debug_uart_rx,
        )

        if self.person_timeout <= 0:
            raise ValueError("person_timeout 必須大於 0")

        self.uart = UARTTransport(
            port=port,
            baudrate=baudrate,
        )

        self.robot = RobotAPI(send=self.uart.send)
        self.rx_buffer = bytearray()

        self.last_person_time = None
        self.person_active = False

        self.subscription = self.create_subscription(
            Pose,
            "/target/relative_position",
            self.person_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # self.timer = self.create_timer(
        #     0.05,
        #     self.check_timeout,
        # )

        self.get_logger().info(
            f"UART: {port}, {baudrate}; "
            "訂閱 /target/pose，發布 /cmd_vel"
        )

    def mark_person_lost(self):
        # 先標記失效；若 UART 傳送失敗，
        # STM32 仍需依自己的逾時機制停止。
        self.person_active = False

        try:
            self.robot.send_person_lost()
        except (OSError, ValueError) as error:
            self.get_logger().error(
                f"傳送 person_lost 失敗：{error}"
            )
    def debug_uart_rx(self):
        self.get_logger().info(
            f"UART 等待讀取的 bytes：{self.uart.uart.in_waiting}"
        )
    def person_callback(self, msg):
        x = float(msg.position.x)
        y = float(msg.position.y)
        z = float(msg.position.z)

        if not all(math.isfinite(v) for v in (x, y, z)):
            self.mark_person_lost()
            return

        # self.get_logger().info(
        #     f"收到目標 XYZ：X={x:.3f}, Y={y:.3f}, Z={z:.3f}"
        # )

        self.robot.send_person_position(x, y, z)
        
        self.get_logger().info(
            f"UART 傳送函式已執行：X={x:.3f}, Y={y:.3f}, Z={z:.3f}"
        )

        self.last_person_time = time.monotonic()
        self.person_active = True

    def check_timeout(self):
        if not self.person_active:
            return

        if (
            time.monotonic() - self.last_person_time
            >= self.person_timeout
        ):
            self.get_logger().warning(
                "人物位置逾時，通知 STM32 停止追蹤"
            )
            self.mark_person_lost()





    def receive_uart(self):
        try:
            received = self.uart.read_available()
            if received:
                self.get_logger().info(f"UART RX 原始資料：{received!r}")
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(f"UART 接收失敗：{error}")
            return

        if not received:
            return

        self.rx_buffer.extend(received)

        while b"\n" in self.rx_buffer:
            line, _, remaining = self.rx_buffer.partition(b"\n")
            self.rx_buffer[:] = remaining

            if not line or len(line) > 1024:
                continue

            try:
                packet = json.loads(line.decode("utf-8"))

                if not isinstance(packet, dict):
                    continue

                if packet.get("cmd") == "error":
                    self.get_logger().warning(
                        f"STM32 回報錯誤：{packet.get('data')}"
                    )
                    continue

                if packet.get("cmd") != "cmd_vel":
                    continue

                data = packet["data"]

                v = float(data["v"])
                w = float(data["w"])

                if not math.isfinite(v) or not math.isfinite(w):
                    continue

                msg = Twist()

                msg.linear.x = v
                msg.angular.z = w

                self.cmd_vel_pub.publish(msg)

            except (
                UnicodeDecodeError,
                json.JSONDecodeError,
                KeyError,
                TypeError,
                ValueError,
                OverflowError,
            ) as error:
                self.get_logger().warning(
                    f"無效 UART 封包：{error}"
                )
                continue


        # 防止沒有換行的異常資料持續累積
        if len(self.rx_buffer) > 1024:
            self.rx_buffer.clear()
            self.get_logger().warning("UART 接收資料過長，已清除")
    def close(self):
        try:
            self.robot.stop()
        except (OSError, ValueError):
            pass
        finally:
            self.uart.close()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = STM32Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close()
            finally:
                node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()