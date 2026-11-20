
#!/usr/bin/env python3

import time
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose


class TFCubeExtractor(Node):

    def __init__(self):
        super().__init__("tf_cube_extractor")

        self.last_print_time = 0.0

        # 訂閱 Isaac Sim 的 TF
        self.subscription = self.create_subscription(
            TFMessage,
            "/tf",
            self.tf_callback,
            qos_profile_sensor_data
        )

        # 發布篩選後的目標 XYZ
        self.publisher = self.create_publisher(
            Pose,
            "/target/relative_position",
            10
        )

        self.get_logger().info(
            "開始監聽 base_link -> TargetCube"
        )

    def tf_callback(self, msg):

        # /tf 可能同時包含多組 Transform
        for tf in msg.transforms:

            parent = tf.header.frame_id.lstrip("/")
            child = tf.child_frame_id.lstrip("/")

            # 只保留我們要的那一組
            if parent != "base_link" or child != "TargetCube":
                continue

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            if not all(math.isfinite(v) for v in (x, y, z)):
                return

            # 建立 XYZ 訊息
            output = Pose()

            output.position.x = x
            output.position.y = y
            output.position.z = z

            output.orientation = tf.transform.rotation

            self.publisher.publish(output)

            # 每秒印出一次，避免 Terminal 洗版
            now = time.monotonic()

            if now - self.last_print_time >= 0.05:

                self.get_logger().info(
                    f"TargetCube Relative Position | "
                    f"X={x:.3f} m, "
                    f"Y={y:.3f} m, "
                    f"Z={z:.3f} m"
                )

                self.last_print_time = now

            break


def main(args=None):

    rclpy.init(args=args)

    node = TFCubeExtractor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
