#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import threading
import time

# Doosan 서비스 import
from dsr_msgs2.srv import MoveJoint # (가장 기본적인 movej)
# 필요하면 다른 서비스도 추가 가능


class RobotConveyorControl(Node):
    def __init__(self):
        super().__init__('robot_conveyor_control')

        # 컨베이어 상태 구독
        self.sub_status = self.create_subscription(
            Int32,
            'conveyor/status_code',
            self.cb_conveyor_status,
            10
        )

        # 컨베이어 명령 발행 (RESUME)
        self.pub_cmd = self.create_publisher(String, 'conveyor/cmd', 10)

        # 로봇 명령 서비스 클라이언트
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/m0609/move_joint')

        self.busy = False   # 로봇이 작업 중인지 체크

        self.get_logger().info("로봇팔 컨베이어 제어 노드 시작됨")

    def cb_conveyor_status(self, msg: Int32):
        code = msg.data

        # 1 = DETECT
        if code == 1 and not self.busy:
            self.busy = True
            self.get_logger().info("컨베이어: DETECT → 로봇팔 작업 시작")

            # 별도 스레드에서 로봇 작업 실행 (ROS2 스레드 방해하지 않음)
            threading.Thread(target=self.robot_job, daemon=True).start()

    def robot_job(self):
        # 실제 로봇 동작 예시
        self.get_logger().info("로봇: 작업 중… (pick → place)")

        # 1) 원하는 포즈로 movej
        req = MoveJoint.Request()
        req.pos = [0, -20, 90, 0, 70, 0]  # 예시 포즈
        req.vel = 20
        req.acc = 20
        req.time = 0
        req.mode = 0

        future = self.cli_movej.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("로봇: movej 완료")

        # 2) 실제라면 그리퍼 열기/닫기, 집기/놓기 동작 수행

        time.sleep(1.0)

        # 3) 작업 완료 → 컨베이어 재가동 신호 보내기
        msg = String()
        msg.data = "RESUME"
        self.pub_cmd.publish(msg)
        self.get_logger().info("로봇: 컨베이어에 RESUME 신호 보냄")

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = RobotConveyorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
