#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32, String


class RobotPickBridge(Node):
    """
    컨베이어 ↔ 로봇 제어 노드 사이에서
    토픽만 중계하는 노드.

    - /conveyor/detect   (UInt32): 아두이노/컨베이어에서 올라오는 DETECT 이벤트
    - /robot/pick_request (UInt32): 로봇 제어 노드에게 "이 번호 물체 집어라" 요청
    - /robot/pick_done   (UInt32): 로봇 제어 노드에서 "이 번호 작업 끝" 알림
    - /conveyor/cmd      (String): 컨베이어에 RESUME 등 명령 전송
    """

    def __init__(self):
        super().__init__('robot_pick_bridge')

        # 현재 로봇이 작업 중인지 여부
        self.busy = False
        self.current_detect_id = None

        # 컨베이어 제어 명령 publisher (아두이노 쪽으로 내려감)
        self.conv_cmd_pub = self.create_publisher(
            String, '/conveyor/cmd', 10
        )

        # 로봇에게 픽업을 요청하는 publisher
        self.robot_req_pub = self.create_publisher(
            UInt32, '/robot/pick_request', 10
        )

        # 컨베이어 DETECT 이벤트 구독
        self.detect_sub = self.create_subscription(
            UInt32,
            '/conveyor/detect',
            self.detect_callback,
            10
        )

        # 로봇이 작업 완료를 알리는 토픽 구독
        self.robot_done_sub = self.create_subscription(
            UInt32,
            '/robot/pick_done',
            self.pick_done_callback,
            10
        )

        self.get_logger().info(
            "RobotPickBridge started (only topic relay, no direct robot control)."
        )

    # ====================================
    # 1) 컨베이어에서 DETECT 들어옴
    # ====================================
    def detect_callback(self, msg: UInt32):
        detect_id = msg.data

        if self.busy:
            # 이미 다른 물체 처리 중이면 무시 (혹은 큐잉 로직 추가 가능)
            self.get_logger().info(
                f"Ignore DETECT #{detect_id}; robot busy with #{self.current_detect_id}"
            )
            return

        self.busy = True
        self.current_detect_id = detect_id

        self.get_logger().info(f"DETECT #{detect_id} → 로봇에게 픽업 요청")

        # 로봇 제어 노드로 픽업 요청 전송
        self.robot_req_pub.publish(msg)

    # ====================================
    # 2) 로봇 제어 노드에서 "작업 완료" 신호
    # ====================================
    def pick_done_callback(self, msg: UInt32):
        done_id = msg.data

        self.get_logger().info(
            f"로봇이 DETECT #{done_id} 작업 완료 알림 수신"
        )

        # 현재 처리중인 ID와 맞는지 확인 (옵션)
        if self.current_detect_id is not None and done_id != self.current_detect_id:
            self.get_logger().warn(
                f"완료 ID #{done_id}가 현재 작업 ID #{self.current_detect_id}와 다릅니다."
            )

        # 컨베이어 다시 돌리라고 명령
        cmd_msg = String()
        cmd_msg.data = "RESUME"
        self.conv_cmd_pub.publish(cmd_msg)
        self.get_logger().info("컨베이어에 RESUME 명령 전송")

        # 상태 초기화
        self.busy = False
        self.current_detect_id = None


def main(args=None):
    rclpy.init(args=args)
    node = RobotPickBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
