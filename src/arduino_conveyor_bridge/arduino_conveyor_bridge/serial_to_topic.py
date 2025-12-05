#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

import serial


class SerialToTopicNode(Node):
    def __init__(self):
        super().__init__('serial_to_topic_node')

        # === 시리얼 포트 설정 ===
        # 여러 포트 시도 (환경에 따라 다를 수 있음)
        ports_to_try = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        baud = 9600
        
        self.ser = None
        self.serial_connected = False
        
        for port in ports_to_try:
            try:
                self.ser = serial.Serial(port, baudrate=baud, timeout=0.01)
                self.get_logger().info(f"✅ Serial opened on {port} (baud {baud})")
                self.serial_connected = True
                break
            except Exception as e:
                self.get_logger().debug(f"포트 {port} 열기 실패: {e}")
        
        if not self.serial_connected:
            self.get_logger().warn("⚠️ 시리얼 포트를 찾을 수 없습니다. 시뮬레이션 모드로 실행합니다.")
            self.get_logger().warn("   Arduino를 연결하면 자동으로 감지됩니다.")

        # === 퍼블리셔: 상태 ===
        self.pub_raw = self.create_publisher(String, '/conveyor/status', 10)
        self.pub_code = self.create_publisher(Int32, '/conveyor/status_code', 10)

        # === 서브스크라이버: 명령 ===
        self.sub_cmd = self.create_subscription(
            String,
            '/conveyor/cmd',
            self.cmd_callback,
            10
        )

        # 주기적으로 시리얼 읽기
        self.timer = self.create_timer(0.01, self.read_serial)
        
        # 재연결 시도 타이머 (시리얼 연결 안됐을 때)
        if not self.serial_connected:
            self.reconnect_timer = self.create_timer(5.0, self.try_reconnect)

    def try_reconnect(self):
        """시리얼 포트 재연결 시도"""
        if self.serial_connected:
            return
            
        ports_to_try = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        
        for port in ports_to_try:
            try:
                self.ser = serial.Serial(port, baudrate=9600, timeout=0.01)
                self.get_logger().info(f"✅ Serial 재연결 성공: {port}")
                self.serial_connected = True
                break
            except:
                pass

    def read_serial(self):
        if not self.serial_connected or self.ser is None:
            return
            
        if not self.ser.in_waiting:
            return

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        if not line:
            return

        raw_msg = String()
        raw_msg.data = line
        self.pub_raw.publish(raw_msg)

        code_msg = Int32()
        code_msg.data = self.map_status_code(line)
        self.pub_code.publish(code_msg)

        self.get_logger().info(f"[FROM ARDUINO] {line} (code={code_msg.data})")

    def map_status_code(self, line: str) -> int:
        line_up = line.upper()

        if "DETECT" in line_up:
            return 1
        if "RUNNING" in line_up:
            return 3
        if "BOOT" in line_up:
            return 0
        return 0

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip()
        if not cmd:
            return

        if not self.serial_connected or self.ser is None:
            self.get_logger().warn(f"[TO ARDUINO] 시리얼 미연결 - 명령 무시: {cmd}")
            return

        try:
            self.ser.write((cmd + '\n').encode('utf-8'))
            self.get_logger().info(f"[TO ARDUINO] {cmd}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialToTopicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
