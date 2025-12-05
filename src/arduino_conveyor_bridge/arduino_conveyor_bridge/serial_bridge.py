#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import serial
import threading
class ArduinoConveyorBridge(Node):
    def __init__(self):
        super().__init__('arduino_conveyor_bridge')
        # --- 파라미터 (포트/보드레이트) ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        # --- 시리얼 오픈 ---
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1.0)
            self.get_logger().info(f"시리얼 포트 열림: {port}, {baud}bps")
        except Exception as e:
            self.get_logger().error(f"시리얼 포트 열기 실패: {e}")
            raise
        # --- ROS2 Pub/Sub ---
        self.pub_status = self.create_publisher(Int32, 'conveyor/status_code', 10)
        self.sub_cmd = self.create_subscription(
            String,
            'conveyor/cmd',
            self.cb_cmd,
            10
        )
        # 시리얼 수신 스레드
        self._running = True
        self.thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.thread.start()
    def read_serial_loop(self):
        while rclpy.ok() and self._running:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                # 디버깅용 로그
                self.get_logger().info(f"[Arduino] {line}")
                # ---- 아두이노 → ROS2 이벤트 매핑 ----
                if line == "DETECT":
                    msg = Int32()
                    msg.data = 1   # 1 = DETECT
                    self.pub_status.publish(msg)
                    self.get_logger().info("conveyor/status_code = 1 (DETECT) 발행")
                # RUNNING, BOOT, RESUME_OK, SPD*_OK 등은
                # 필요하면 별도 토픽으로 보내거나, 지금처럼 로그만 찍어도 됨.
            except Exception as e:
                self.get_logger().error(f"시리얼 수신 에러: {e}")
    def cb_cmd(self, msg: String):
        """
        ROS2 → 아두이노 명령 전달
        예: "RESUME", "SPD0", "SPD1", "SPD2", "DIR_FWD", "DIR_REV"
        """
        cmd = msg.data.strip()
        if not cmd:
            return
        try:
            send_str = cmd + "\n"
            self.ser.write(send_str.encode())
            self.get_logger().info(f"[ROS→Arduino] '{send_str.strip()}' 전송")
        except Exception as e:
            self.get_logger().error(f"시리얼 전송 에러: {e}")
    def destroy_node(self):
        self._running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = ArduinoConveyorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()