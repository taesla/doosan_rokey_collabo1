#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32, Bool
import serial
import threading

class ConveyorSerialNode(Node):
    def __init__(self):
        super().__init__('conveyor_serial_node')

        # === 파라미터 ===
        port = self.declare_parameter('port', '/dev/ttyACM0').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 9600).get_parameter_value().integer_value

        self.get_logger().info(f"Opening serial: {port} @ {baud}")
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)

        # === 퍼블리셔 ===
        self.detect_pub = self.create_publisher(Int32, '/conveyor/detect', 10)
        self.running_pub = self.create_publisher(Bool, '/conveyor/is_running', 10)
        self.status_pub = self.create_publisher(String, '/conveyor/status', 10)
        self.log_pub = self.create_publisher(String, '/conveyor/log', 10)

        # === 구독자 (웹/로봇 → 시리얼 명령) ===
        self.cmd_sub = self.create_subscription(
            String, '/conveyor/cmd', self.on_cmd, 10
        )

        # === 시리얼 리더 스레드 시작 ===
        self._stop_event = threading.Event()
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def on_cmd(self, msg: String):
        """웹/로봇에서 /conveyor/cmd 에 문자열 올리면 그대로 아두이노로 전송"""
        cmd = msg.data.strip()
        if not cmd:
            return
        self.get_logger().info(f"[TX] {cmd}")
        try:
            self.ser.write((cmd + '\n').encode())
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def read_loop(self):
        """시리얼에서 한 줄씩 읽어서 ROS 토픽으로 변환"""
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                continue

            if not line:
                continue

            self.get_logger().info(f"[RX] {line}")
            self.handle_line(line)

    def handle_line(self, line: str):
        # 1) DETECT#N → /conveyor/detect
        if line.startswith("DETECT#"):
            try:
                n = int(line.split('#')[1])
            except:
                n = -1
            msg = Int32()
            msg.data = n
            self.detect_pub.publish(msg)

            # 이벤트 로그용
            log = String()
            log.data = f"DETECT,{n}"
            self.log_pub.publish(log)
            return

        # 2) RUNNING → /conveyor/is_running
        if line == "RUNNING":
            msg = Bool()
            msg.data = True
            self.running_pub.publish(msg)
            return

        # 3) STATE:... → /conveyor/status
        if line.startswith("STATE:"):
            s = String()
            # "STATE:" 부분을 떼고 나머지 문자열만 전달
            s.data = line[len("STATE:"):]
            self.status_pub.publish(s)
            return

        # 4) 기타 응답들(ESTOP_OK, SPD1_OK 등) → /conveyor/log
        msg = String()
        msg.data = line
        self.log_pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        try:
            self.ser.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
