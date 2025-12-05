#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import threading


class RobotConveyorDemo(Node):
    def __init__(self):
        super().__init__('robot_conveyor_demo')

        # 컨베이어 상태 코드 구독
        self.sub_status = self.create_subscription(
            Int32,
            'conveyor/status_code',
            self.status_callback,
            10
        )

        # 컨베이어 명령 발행 (RESUME, SPD1~3, DIR_FWD/REV 등)
        self.pub_cmd = self.create_publisher(String, 'conveyor/cmd', 10)

        self.get_logger().info("RobotConveyorDemo started.")
        self.get_logger().info("키 입력 가이드:")
        self.get_logger().info("  1: 느린 속도 (SPD1)")
        self.get_logger().info("  2: 중간 속도 (SPD2)")
        self.get_logger().info("  3: 빠른 속도 (SPD3)")
        self.get_logger().info("  f: 정방향 (DIR_FWD)")
        self.get_logger().info("  b: 역방향 (DIR_REV)")
        self.get_logger().info("  r: RESUME (컨베이어 재가동)")
        self.get_logger().info("  q: 종료")

        # 키보드 입력을 별도 스레드에서 처리
        self._running = True
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    # -----------------------------
    # 컨베이어 상태 콜백
    # -----------------------------
    def status_callback(self, msg: Int32):
        code = msg.data

        if code == 1:
            # DETECT
            self.get_logger().info("[STATUS] DETECT (물체 감지, 컨베이어 정지 상태)")
        elif code == 3:
            # RUNNING
            # 너무 자주 찍히면 시끄러우니까 필요하면 debug 로
            # self.get_logger().debug("[STATUS] RUNNING (컨베이어 가동중)")
            pass
        else:
            # BOOT, 기타
            # self.get_logger().info(f"[STATUS] code={code}")
            pass

    # -----------------------------
    # 키보드 입력 루프
    # -----------------------------
    def keyboard_loop(self):
        while rclpy.ok() and self._running:
            try:
                user_input = input("> 명령 입력 (1/2/3/f/b/r/q): ").strip().lower()
            except EOFError:
                break

            if not user_input:
                continue

            if user_input == 'q':
                self.get_logger().info("종료 명령(q) 입력됨. 노드 종료 준비.")
                self._running = False
                # rclpy.shutdown() 은 main 쪽에서 처리
                break

            cmd_msg = String()

            if user_input == '1':
                cmd_msg.data = "SPD1"
            elif user_input == '2':
                cmd_msg.data = "SPD2"
            elif user_input == '3':
                cmd_msg.data = "SPD3"
            elif user_input == 'f':
                cmd_msg.data = "DIR_FWD"
            elif user_input == 'b':
                cmd_msg.data = "DIR_REV"
            elif user_input == 'r':
                # 헴이 말한 resume 기능
                cmd_msg.data = "RESUME"
            else:
                self.get_logger().warn(f"알 수 없는 입력: {user_input}")
                continue

            self.pub_cmd.publish(cmd_msg)
            self.get_logger().info(f"[SEND CMD] {cmd_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotConveyorDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
