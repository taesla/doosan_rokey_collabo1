#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class ConveyorDebugListener(Node):
    def __init__(self):
        super().__init__('conveyor_debug_listener')

        # 상태 코드 (1,2,3...) 구독
        self.sub_code = self.create_subscription(
            Int32,
            'conveyor/status_code',
            self.code_callback,
            10
        )

        # 원본 문자열도 같이 보고 싶으면
        self.sub_raw = self.create_subscription(
            String,
            'conveyor/status',
            self.raw_callback,
            10
        )

        self.get_logger().info("ConveyorDebugListener started. Waiting for messages...")

    def code_callback(self, msg: Int32):
        code = msg.data

        if code == 1:
            state_str = "물체감지"
        elif code == 2:
            state_str = "물체이동완료"
        elif code == 3:
            state_str = "컨베이어벨트작동중"
        else:
            state_str = "알 수 없음"

        self.get_logger().info(f"[CODE] {code} ({state_str})")

    def raw_callback(self, msg: String):
        self.get_logger().info(f"[RAW] {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorDebugListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

