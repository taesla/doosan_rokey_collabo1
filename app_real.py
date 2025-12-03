"""
물류 로봇 모니터링 웹 서버 (실제 로봇 연동)
두산 로봇 + ROS2 + 실시간 대시보드 + Firebase 연동
- /dsr01/joint_states 토픽 구독
- 서비스 호출로 추가 데이터 가져오기
- Firebase Realtime Database 연동 (외부 접속용)
"""

from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO
import threading
import time
import math
import os
import json
from datetime import datetime, timezone, timedelta
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
import subprocess
import io

# Firebase 연동 (선택적)
firebase_enabled = False
firebase_ref = None
firebase_cmd_ref = None

try:
    import firebase_admin
    from firebase_admin import credentials, db
    
    # config 폴더 경로
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    CONFIG_DIR = os.path.join(SCRIPT_DIR, 'config')
    SERVICE_ACCOUNT_KEY_PATH = os.path.join(CONFIG_DIR, 'serviceAccountKey.json')
    WEB_CONFIG_PATH = os.path.join(CONFIG_DIR, 'firebase_web_config.json')
    
    if os.path.exists(SERVICE_ACCOUNT_KEY_PATH) and os.path.exists(WEB_CONFIG_PATH):
        # Web Config에서 databaseURL 읽기
        with open(WEB_CONFIG_PATH, 'r') as f:
            web_config = json.load(f)
        DATABASE_URL = web_config.get('databaseURL')
        
        # Firebase 초기화
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        
        # DB 참조
        firebase_ref = db.reference('/robot_status')
        firebase_cmd_ref = db.reference('/robot_commands')
        
        firebase_enabled = True
        print("✅ Firebase 연동 활성화")
        print(f"   DB URL: {DATABASE_URL}")
    else:
        print("⚠️ Firebase 설정 파일 없음 - Firebase 비활성화")
        
except ImportError:
    print("⚠️ firebase-admin 패키지 없음 - Firebase 비활성화")
except Exception as e:
    print(f"⚠️ Firebase 초기화 실패: {e}")

app = Flask(__name__)
app.config['SECRET_KEY'] = 'logistics_monitor_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 전역 변수 - RobotState 전체 필드
robot_data = {
    'connected': False,
    'timestamp': 0,
    
    # ============== 조인트 데이터 (Actual) ==============
    'actual_joint_position': [0.0] * 6,
    'actual_joint_velocity': [0.0] * 6,
    'actual_joint_torque': [0.0] * 6,
    'joint_temperature': [25.0] * 6,
    'actual_motor_torque': [0.0] * 6,
    'actual_je': [0.0] * 6,
    
    # ============== 조인트 데이터 (Target) ==============
    'target_joint_position': [0.0] * 6,
    'target_joint_velocity': [0.0] * 6,
    
    # ============== 조인트 데이터 (기타) ==============
    'actual_joint_position_abs': [0.0] * 6,
    'actual_bk': [0] * 6,
    
    # ============== TCP 데이터 (Actual) ==============
    'actual_tcp_position': [0.0] * 6,
    'actual_tcp_velocity': [0.0] * 6,
    
    # ============== TCP 데이터 (Target) ==============
    'target_tcp_position': [0.0] * 6,
    'target_tcp_velocity': [0.0] * 6,
    
    # ============== 힘/토크 데이터 ==============
    'external_tcp_force': [0.0] * 6,
    'target_force': [0.0] * 6,
    'actual_tool_force': [0.0] * 6,
    
    # ============== Controller I/O ==============
    'controller_digital_input': 0,
    'controller_digital_output': 0,
    'controller_analog_input': [0.0, 0.0],
    'controller_analog_output': [0.0, 0.0],
    
    # ============== Flange I/O ==============
    'flange_digital_input': 0,
    'flange_digital_output': 0,
    'flange_analog_input': [0.0] * 4,
    
    # ============== 시스템 상태 ==============
    'robot_mode': 1,
    'robot_state': 1,
    'operation_speed_rate': 100,
    'singularity': 0.0,
    'solution_space': 0,
    'program_running': False,
    'access_control': 0,
    'running_time': 0.0,
    'safe_stop_reset_req': False,
    
    # ============== 툴/좌표계 ==============
    'tool_index': 0,
    'tcp_index': 0,
    'gravity_dir': [0.0, 0.0, -9.81],
    
    # ============== 에러 정보 ==============
    'error_level': 0,
    'error_code': 0,
    'error_msg': '',
    
    # ============== 툴/TCP 정보 ==============
    'current_tool_name': '',
    'current_tcp_name': '',
    'tool_flange_posx': [0.0] * 6,
    
    # ============== 목표(Desired) 위치 ==============
    'desired_tcp_position': [0.0] * 6,
    'desired_joint_position': [0.0] * 6,
    'desired_tcp_velocity': [0.0] * 6,
    'desired_joint_velocity': [0.0] * 6,
}

task_queue = {
    'current_task': None,
    'queue': [],
    'completed': 0,
    'errors': 0,
    'rate_per_hour': 0,
}

logs = []


class RobotStateSubscriber(Node):
    """두산 로봇 ROS2 토픽/서비스 연동 노드"""
    
    def __init__(self):
        super().__init__('logistics_monitor')
        self.get_logger().info('Logistics Monitor Node Started')
        
        # 토픽 이름 (실제 로봇에서 사용하는 네임스페이스)
        self.robot_ns = '/dsr01'
        
        self.callback_group = ReentrantCallbackGroup()
        
        # JointState 구독
        self.joint_sub = self.create_subscription(
            JointState,
            f'{self.robot_ns}/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        self.get_logger().info(f'Subscribed to {self.robot_ns}/joint_states')
        
        # 서비스 클라이언트 생성 시도
        self.setup_service_clients()
        
        # RobotError 토픽 구독 시도
        try:
            from dsr_msgs2.msg import RobotError
            self.error_sub = self.create_subscription(
                RobotError,
                f'{self.robot_ns}/error',
                self.error_callback,
                10,
                callback_group=self.callback_group
            )
            self.get_logger().info(f'Subscribed to {self.robot_ns}/error')
        except Exception as e:
            self.get_logger().warn(f'RobotError subscription failed: {e}')
        
        # 주기적으로 서비스 호출해서 추가 데이터 가져오기
        self.service_timer = self.create_timer(0.2, self.call_services, callback_group=self.callback_group)
        
        robot_data['connected'] = True
        self.start_time = time.time()
        
        self.add_log('INFO', '로봇 모니터링 시작')
    
    def setup_service_clients(self):
        """서비스 클라이언트 설정"""
        try:
            from dsr_msgs2.srv import GetCurrentPosx, GetCurrentPosj, GetCurrentVelx, GetCurrentVelj
            from dsr_msgs2.srv import GetDesiredPosx, GetDesiredPosj, GetDesiredVelx, GetDesiredVelj
            from dsr_msgs2.srv import GetToolForce, GetExternalTorque, GetJointTorque
            from dsr_msgs2.srv import GetCtrlBoxDigitalInput, GetCtrlBoxDigitalOutput
            from dsr_msgs2.srv import GetToolDigitalInput, GetToolDigitalOutput
            from dsr_msgs2.srv import GetCtrlBoxAnalogInput
            
            # TCP 위치
            self.cli_posx = self.create_client(
                GetCurrentPosx, 
                f'{self.robot_ns}/aux_control/get_current_posx',
                callback_group=self.callback_group
            )
            
            # TCP 속도
            self.cli_velx = self.create_client(
                GetCurrentVelx,
                f'{self.robot_ns}/aux_control/get_current_velx',
                callback_group=self.callback_group
            )
            
            # 조인트 위치
            self.cli_posj = self.create_client(
                GetCurrentPosj,
                f'{self.robot_ns}/aux_control/get_current_posj',
                callback_group=self.callback_group
            )
            
            # 조인트 속도
            self.cli_velj = self.create_client(
                GetCurrentVelj,
                f'{self.robot_ns}/aux_control/get_current_velj',
                callback_group=self.callback_group
            )
            
            # 툴 힘
            self.cli_tool_force = self.create_client(
                GetToolForce,
                f'{self.robot_ns}/aux_control/get_tool_force',
                callback_group=self.callback_group
            )
            
            # 외부 토크
            self.cli_ext_torque = self.create_client(
                GetExternalTorque,
                f'{self.robot_ns}/aux_control/get_external_torque',
                callback_group=self.callback_group
            )
            
            # 조인트 토크
            self.cli_joint_torque = self.create_client(
                GetJointTorque,
                f'{self.robot_ns}/aux_control/get_joint_torque',
                callback_group=self.callback_group
            )
            
            # Desired(목표) TCP 위치
            self.cli_desired_posx = self.create_client(
                GetDesiredPosx,
                f'{self.robot_ns}/aux_control/get_desired_posx',
                callback_group=self.callback_group
            )
            
            # Desired(목표) 조인트 위치
            self.cli_desired_posj = self.create_client(
                GetDesiredPosj,
                f'{self.robot_ns}/aux_control/get_desired_posj',
                callback_group=self.callback_group
            )
            
            # Desired(목표) TCP 속도
            self.cli_desired_velx = self.create_client(
                GetDesiredVelx,
                f'{self.robot_ns}/aux_control/get_desired_velx',
                callback_group=self.callback_group
            )
            
            # Desired(목표) 조인트 속도
            self.cli_desired_velj = self.create_client(
                GetDesiredVelj,
                f'{self.robot_ns}/aux_control/get_desired_velj',
                callback_group=self.callback_group
            )
            
            # Digital I/O
            self.cli_din = self.create_client(
                GetCtrlBoxDigitalInput,
                f'{self.robot_ns}/io/get_ctrl_box_digital_input',
                callback_group=self.callback_group
            )
            self.cli_dout = self.create_client(
                GetCtrlBoxDigitalOutput,
                f'{self.robot_ns}/io/get_ctrl_box_digital_output',
                callback_group=self.callback_group
            )
            
            # Tool Digital I/O
            self.cli_tool_din = self.create_client(
                GetToolDigitalInput,
                f'{self.robot_ns}/io/get_tool_digital_input',
                callback_group=self.callback_group
            )
            self.cli_tool_dout = self.create_client(
                GetToolDigitalOutput,
                f'{self.robot_ns}/io/get_tool_digital_output',
                callback_group=self.callback_group
            )
            
            # 그리퍼 제어용 서비스 클라이언트
            from dsr_msgs2.srv import SetToolDigitalOutput, SetCtrlBoxDigitalOutput
            self.cli_set_tool_dout = self.create_client(
                SetToolDigitalOutput,
                f'{self.robot_ns}/io/set_tool_digital_output',
                callback_group=self.callback_group
            )
            self.cli_set_ctrl_dout = self.create_client(
                SetCtrlBoxDigitalOutput,
                f'{self.robot_ns}/io/set_ctrl_box_digital_output',
                callback_group=self.callback_group
            )
            self.get_logger().info('Gripper service clients created')
            
            # 홈 이동 서비스 클라이언트
            from dsr_msgs2.srv import MoveHome
            self.cli_move_home = self.create_client(
                MoveHome,
                f'{self.robot_ns}/motion/move_home',
                callback_group=self.callback_group
            )
            self.get_logger().info('MoveHome service client created')
            
            # MoveJoint 서비스 클라이언트 (진자운동 테스트용)
            from dsr_msgs2.srv import MoveJoint
            self.cli_move_joint = self.create_client(
                MoveJoint,
                f'{self.robot_ns}/motion/move_joint',
                callback_group=self.callback_group
            )
            self.get_logger().info('MoveJoint service client created')
            
            # 진자운동 테스트 상태
            self.pendulum_running = False
            self.pendulum_thread = None
            self.pendulum_paused = False  # 일시정지 상태
            self.pendulum_params = None   # 마지막 진자운동 파라미터 저장
            
            # 작업 속도 변경 서비스 클라이언트
            from dsr_msgs2.srv import ChangeOperationSpeed
            self.cli_change_speed = self.create_client(
                ChangeOperationSpeed,
                f'{self.robot_ns}/motion/change_operation_speed',
                callback_group=self.callback_group
            )
            self.get_logger().info('ChangeOperationSpeed service client created')
            
            # 긴급정지 서비스 클라이언트
            from dsr_msgs2.srv import MoveStop
            self.cli_move_stop = self.create_client(
                MoveStop,
                f'{self.robot_ns}/motion/move_stop',
                callback_group=self.callback_group
            )
            self.get_logger().info('MoveStop service client created')
            
            # 일시정지/재개 서비스 클라이언트
            from dsr_msgs2.srv import MovePause, MoveResume
            self.cli_move_pause = self.create_client(
                MovePause,
                f'{self.robot_ns}/motion/move_pause',
                callback_group=self.callback_group
            )
            self.cli_move_resume = self.create_client(
                MoveResume,
                f'{self.robot_ns}/motion/move_resume',
                callback_group=self.callback_group
            )
            self.get_logger().info('MovePause/MoveResume service clients created')
            
            # Tool/TCP 정보 서비스 클라이언트
            from dsr_msgs2.srv import GetCurrentTool, GetCurrentTcp, GetCurrentToolFlangePosx
            self.cli_current_tool = self.create_client(
                GetCurrentTool,
                f'{self.robot_ns}/tool/get_current_tool',
                callback_group=self.callback_group
            )
            self.cli_current_tcp = self.create_client(
                GetCurrentTcp,
                f'{self.robot_ns}/tcp/get_current_tcp',
                callback_group=self.callback_group
            )
            self.cli_tool_flange_posx = self.create_client(
                GetCurrentToolFlangePosx,
                f'{self.robot_ns}/aux_control/get_current_tool_flange_posx',
                callback_group=self.callback_group
            )
            self.get_logger().info('Tool/TCP service clients created')
            
            # Tool/TCP 정보 조회 플래그 (초기화 시 한번만 확인)
            self.tool_tcp_checked = False
            
            self.services_available = True
            self.get_logger().info('Service clients created successfully')
            
        except ImportError as e:
            self.get_logger().warn(f'dsr_msgs2.srv import failed: {e}')
            self.services_available = False
        except Exception as e:
            self.get_logger().warn(f'Service client setup failed: {e}')
            self.services_available = False
    
    def joint_state_callback(self, msg):
        """JointState 메시지 콜백"""
        global robot_data
        
        # 조인트 순서 정렬 (joint_1 ~ joint_6)
        joint_order = {'joint_1': 0, 'joint_2': 1, 'joint_3': 2, 
                       'joint_4': 3, 'joint_5': 4, 'joint_6': 5}
        
        positions = [0.0] * 6
        velocities = [0.0] * 6
        
        for i, name in enumerate(msg.name):
            if name in joint_order:
                idx = joint_order[name]
                # 라디안 -> 도 변환
                positions[idx] = math.degrees(msg.position[i]) if i < len(msg.position) else 0.0
                velocities[idx] = math.degrees(msg.velocity[i]) if i < len(msg.velocity) else 0.0
        
        robot_data['actual_joint_position'] = positions
        robot_data['actual_joint_velocity'] = velocities
        robot_data['timestamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        robot_data['running_time'] = time.time() - self.start_time
        robot_data['connected'] = True
    
    def error_callback(self, msg):
        """RobotError 메시지 콜백"""
        global robot_data
        
        robot_data['error_level'] = msg.level
        robot_data['error_code'] = msg.code
        robot_data['error_msg'] = msg.msg1 if hasattr(msg, 'msg1') else str(msg)
        
        if msg.level > 0:
            level_str = ['INFO', 'WARN', 'ERROR'][min(msg.level - 1, 2)]
            self.add_log(level_str, f'[{msg.code}] {robot_data["error_msg"]}')
    
    def call_services(self):
        """서비스 호출로 추가 데이터 가져오기"""
        global robot_data
        
        if not hasattr(self, 'services_available') or not self.services_available:
            return
        
        try:
            # TCP 위치 (현재)
            if self.cli_posx.service_is_ready():
                from dsr_msgs2.srv import GetCurrentPosx
                req = GetCurrentPosx.Request()
                req.ref = 0  # DR_BASE
                future = self.cli_posx.call_async(req)
                future.add_done_callback(self.posx_callback)
            
            # TCP 위치 (목표)
            if hasattr(self, 'cli_desired_posx') and self.cli_desired_posx.service_is_ready():
                from dsr_msgs2.srv import GetDesiredPosx
                req = GetDesiredPosx.Request()
                req.ref = 0  # DR_BASE
                future = self.cli_desired_posx.call_async(req)
                future.add_done_callback(self.desired_posx_callback)
            
            # 조인트 위치 (목표)
            if hasattr(self, 'cli_desired_posj') and self.cli_desired_posj.service_is_ready():
                from dsr_msgs2.srv import GetDesiredPosj
                req = GetDesiredPosj.Request()
                future = self.cli_desired_posj.call_async(req)
                future.add_done_callback(self.desired_posj_callback)
            
            # 툴 힘
            if self.cli_tool_force.service_is_ready():
                from dsr_msgs2.srv import GetToolForce
                req = GetToolForce.Request()
                req.ref = 0
                future = self.cli_tool_force.call_async(req)
                future.add_done_callback(self.tool_force_callback)
            
            # 조인트 토크
            if self.cli_joint_torque.service_is_ready():
                from dsr_msgs2.srv import GetJointTorque
                req = GetJointTorque.Request()
                future = self.cli_joint_torque.call_async(req)
                future.add_done_callback(self.joint_torque_callback)
            
            # Digital Input (각 핀별로 호출)
            if self.cli_din.service_is_ready():
                self.call_digital_io()
            
            # Tool/TCP 정보 주기적 조회
            self.call_tool_tcp_info()
                
        except Exception as e:
            self.get_logger().debug(f'Service call error: {e}')
    
    def posx_callback(self, future):
        """TCP 위치 서비스 응답"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                # task_pos_info는 리스트, 첫 번째 요소의 data 추출
                pos_data = result.task_pos_info
                if isinstance(pos_data, list) and len(pos_data) > 0:
                    first_item = pos_data[0]
                    if hasattr(first_item, 'data'):
                        robot_data['actual_tcp_position'] = list(first_item.data)[:6]
                elif hasattr(pos_data, 'data'):
                    robot_data['actual_tcp_position'] = list(pos_data.data)[:6]
        except Exception as e:
            self.get_logger().debug(f'posx_callback error: {e}')
    
    def desired_posx_callback(self, future):
        """목표 TCP 위치 서비스 응답"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                pos_data = result.pos
                if hasattr(pos_data, 'tolist'):
                    robot_data['desired_tcp_position'] = pos_data.tolist()[:6]
                else:
                    robot_data['desired_tcp_position'] = list(pos_data)[:6]
        except Exception as e:
            self.get_logger().debug(f'desired_posx_callback error: {e}')
    
    def desired_posj_callback(self, future):
        """목표 조인트 위치 서비스 응답"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                pos_data = result.pos
                if hasattr(pos_data, 'tolist'):
                    robot_data['desired_joint_position'] = pos_data.tolist()[:6]
                else:
                    robot_data['desired_joint_position'] = list(pos_data)[:6]
        except Exception as e:
            self.get_logger().debug(f'desired_posj_callback error: {e}')
    
    def tool_force_callback(self, future):
        """툴 힘 서비스 응답"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                # numpy array나 list 형태로 올 수 있음
                force_data = result.tool_force
                if hasattr(force_data, 'tolist'):
                    force_list = force_data.tolist()[:6]
                elif hasattr(force_data, 'data'):
                    force_list = [float(x) for x in list(force_data.data)[:6]]
                else:
                    force_list = [float(x) for x in list(force_data)[:6]]
                robot_data['external_tcp_force'] = force_list
                robot_data['actual_tool_force'] = force_list
        except Exception as e:
            self.get_logger().debug(f'tool_force_callback error: {e}')
    
    def joint_torque_callback(self, future):
        """조인트 토크 서비스 응답"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                # numpy array나 list 형태로 올 수 있음
                torque_data = result.jts
                if hasattr(torque_data, 'tolist'):
                    robot_data['actual_joint_torque'] = torque_data.tolist()[:6]
                elif hasattr(torque_data, 'data'):
                    robot_data['actual_joint_torque'] = [float(x) for x in list(torque_data.data)[:6]]
                else:
                    robot_data['actual_joint_torque'] = [float(x) for x in list(torque_data)[:6]]
        except Exception as e:
            self.get_logger().debug(f'joint_torque_callback error: {e}')
    
    def call_digital_io(self):
        """Digital I/O 서비스 호출 - DO1, DO2 상태 읽기"""
        global robot_data
        try:
            from dsr_msgs2.srv import GetCtrlBoxDigitalOutput
            
            # DO1, DO2 상태 읽기 (그리퍼 상태)
            for pin in [1, 2]:
                if self.cli_dout.service_is_ready():
                    req = GetCtrlBoxDigitalOutput.Request()
                    req.index = pin
                    future = self.cli_dout.call_async(req)
                    future.add_done_callback(lambda f, p=pin: self.dio_callback(f, p))
                    
        except Exception as e:
            self.get_logger().debug(f'call_digital_io error: {e}')
    
    def dio_callback(self, future, pin):
        """Digital Output 콜백"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                # 비트 플래그로 저장
                current = robot_data.get('controller_digital_output', 0)
                if result.value == 0:  # ON
                    current |= (1 << (pin - 1))
                else:  # OFF
                    current &= ~(1 << (pin - 1))
                robot_data['controller_digital_output'] = current
        except Exception as e:
            pass
    
    def call_tool_tcp_info(self):
        """Tool/TCP 정보 서비스 호출"""
        global robot_data
        try:
            from dsr_msgs2.srv import GetCurrentTool, GetCurrentTcp, GetCurrentToolFlangePosx
            
            # Current Tool
            if hasattr(self, 'cli_current_tool') and self.cli_current_tool.service_is_ready():
                req = GetCurrentTool.Request()
                future = self.cli_current_tool.call_async(req)
                future.add_done_callback(self.current_tool_callback)
            
            # Current TCP
            if hasattr(self, 'cli_current_tcp') and self.cli_current_tcp.service_is_ready():
                req = GetCurrentTcp.Request()
                future = self.cli_current_tcp.call_async(req)
                future.add_done_callback(self.current_tcp_callback)
            
            # Tool Flange Position
            if hasattr(self, 'cli_tool_flange_posx') and self.cli_tool_flange_posx.service_is_ready():
                req = GetCurrentToolFlangePosx.Request()
                req.ref = 0  # DR_BASE
                future = self.cli_tool_flange_posx.call_async(req)
                future.add_done_callback(self.tool_flange_posx_callback)
                
        except Exception as e:
            self.get_logger().debug(f'call_tool_tcp_info error: {e}')
    
    def current_tool_callback(self, future):
        """Current Tool 서비스 응답"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                robot_data['current_tool_name'] = result.info if result.info else ''
        except Exception as e:
            self.get_logger().debug(f'current_tool_callback error: {e}')
    
    def current_tcp_callback(self, future):
        """Current TCP 서비스 응답"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                robot_data['current_tcp_name'] = result.info if result.info else ''
        except Exception as e:
            self.get_logger().debug(f'current_tcp_callback error: {e}')
    
    def tool_flange_posx_callback(self, future):
        """Tool Flange Position 서비스 응답"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                pos_data = result.pos
                if hasattr(pos_data, 'tolist'):
                    robot_data['tool_flange_posx'] = pos_data.tolist()[:6]
                else:
                    robot_data['tool_flange_posx'] = list(pos_data)[:6]
        except Exception as e:
            self.get_logger().debug(f'tool_flange_posx_callback error: {e}')
    
    def add_log(self, level, message):
        """로그 추가 (한국시간 KST)"""
        global logs
        kst = timezone(timedelta(hours=9))
        now_kst = datetime.now(kst)
        log_entry = {
            'time': now_kst.strftime('%H:%M:%S'),
            'level': level,
            'message': message
        }
        logs.insert(0, log_entry)
        logs = logs[:100]
    
    def set_gripper(self, open_gripper):
        """그리퍼 제어 (Controller Digital Output 1, 2번 사용)
        OnRobot RG2 그리퍼 제어:
        - 열기: DO1=ON(0), DO2=OFF(1)
        - 닫기: DO1=OFF(1), DO2=ON(0)
        """
        try:
            from dsr_msgs2.srv import SetCtrlBoxDigitalOutput
            
            if not hasattr(self, 'cli_set_ctrl_dout'):
                self.add_log('ERROR', '그리퍼 서비스 클라이언트 없음')
                return False
            
            if not self.cli_set_ctrl_dout.service_is_ready():
                self.add_log('WARN', '그리퍼 서비스 연결 대기중...')
                return False
            
            # DO1 설정
            req1 = SetCtrlBoxDigitalOutput.Request()
            req1.index = 1
            req1.value = 0 if open_gripper else 1  # 열기: ON(0), 닫기: OFF(1)
            
            # DO2 설정
            req2 = SetCtrlBoxDigitalOutput.Request()
            req2.index = 2
            req2.value = 1 if open_gripper else 0  # 열기: OFF(1), 닫기: ON(0)
            
            # 비동기 호출
            future1 = self.cli_set_ctrl_dout.call_async(req1)
            future2 = self.cli_set_ctrl_dout.call_async(req2)
            
            self.get_logger().info(f'Gripper {"OPEN" if open_gripper else "CLOSE"} command sent (DO1={req1.value}, DO2={req2.value})')
            self.add_log('INFO', f'그리퍼 {"열기" if open_gripper else "닫기"} 명령 전송')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Gripper control error: {e}')
            self.add_log('ERROR', f'그리퍼 제어 오류: {e}')
            return False
    
    def move_home(self, target=1):
        """홈 위치로 이동
        target: 0=기계적 홈(0,0,0,0,0,0), 1=사용자 정의 홈(그리퍼 아래 방향)
        """
        try:
            from dsr_msgs2.srv import MoveHome
            
            if not hasattr(self, 'cli_move_home') or not self.cli_move_home.service_is_ready():
                self.add_log('WARN', '홈 이동 서비스가 준비되지 않음')
                return False
            
            req = MoveHome.Request()
            req.target = target  # 0=기계적 홈, 1=사용자 홈
            
            future = self.cli_move_home.call_async(req)
            home_type = "기계적 홈" if target == 0 else "사용자 홈"
            self.get_logger().info(f'MoveHome command sent (target={target}: {home_type})')
            self.add_log('INFO', f'홈 이동 명령 전송 ({home_type})')
            return True
            
        except Exception as e:
            self.get_logger().error(f'MoveHome error: {e}')
            self.add_log('ERROR', f'홈 이동 오류: {e}')
            return False
    
    def set_ctrl_digital_output(self, index, value):
        """Controller Digital Output 제어"""
        try:
            from dsr_msgs2.srv import SetCtrlBoxDigitalOutput
            
            if not hasattr(self, 'cli_set_ctrl_dout') or not self.cli_set_ctrl_dout.service_is_ready():
                return False
            
            req = SetCtrlBoxDigitalOutput.Request()
            req.index = index  # 1~16
            req.value = value  # 0=ON, 1=OFF
            
            future = self.cli_set_ctrl_dout.call_async(req)
            self.add_log('INFO', f'Digital Output {index} = {"ON" if value == 0 else "OFF"}')
            return True
            
        except Exception as e:
            self.add_log('ERROR', f'Digital Output 제어 오류: {e}')
            return False
    
    def move_joint(self, pos, vel=30.0, acc=30.0):
        """조인트 이동 (작업 속도에 영향받음)
        pos: 목표 조인트 각도 [deg] 6개
        vel: 속도 [deg/sec]
        acc: 가속도 [deg/sec2]
        """
        try:
            from dsr_msgs2.srv import MoveJoint
            
            if not hasattr(self, 'cli_move_joint') or not self.cli_move_joint.service_is_ready():
                self.add_log('WARN', 'MoveJoint 서비스가 준비되지 않음')
                return False
            
            req = MoveJoint.Request()
            req.pos = pos
            req.vel = vel
            req.acc = acc
            req.time = 0.0
            req.radius = 0.0
            req.mode = 0  # ABSOLUTE
            req.blend_type = 0
            req.sync_type = 0  # SYNC (완료까지 대기)
            
            future = self.cli_move_joint.call_async(req)
            # 동기 대기 (최대 30초)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None:
                return future.result().success
            return False
            
        except Exception as e:
            self.get_logger().error(f'MoveJoint error: {e}')
            self.add_log('ERROR', f'조인트 이동 오류: {e}')
            return False
    
    def start_pendulum_test(self, joint_index=0, amplitude=15.0, vel=30.0):
        """진자운동 테스트 시작
        joint_index: 움직일 조인트 (0~5)
        amplitude: 진폭 [deg]
        vel: 속도 [deg/sec] - 작업 속도에 영향받음
        """
        if self.pendulum_running:
            self.add_log('WARN', '진자운동 테스트가 이미 실행 중')
            return False
        
        # 파라미터 저장 (재개 시 사용)
        self.pendulum_params = {
            'joint_index': joint_index,
            'amplitude': amplitude,
            'vel': vel
        }
        self.pendulum_paused = False
        
        self.pendulum_running = True
        self.pendulum_thread = threading.Thread(
            target=self._pendulum_loop,
            args=(joint_index, amplitude, vel),
            daemon=True
        )
        self.pendulum_thread.start()
        self.add_log('INFO', f'진자운동 테스트 시작 (J{joint_index+1}, 진폭 ±{amplitude}°, 속도 {vel}°/s)')
        return True
    
    def stop_pendulum_test(self):
        """진자운동 테스트 정지"""
        self.pendulum_running = False
        self.pendulum_paused = False
        self.pendulum_params = None  # 파라미터도 초기화
        self.add_log('INFO', '진자운동 테스트 정지')
        return True
    
    def pause_pendulum_test(self):
        """진자운동 테스트 일시정지 (재개 가능)"""
        if self.pendulum_running:
            self.pendulum_running = False
            self.pendulum_paused = True
            self.add_log('INFO', '진자운동 테스트 일시정지')
            return True
        return False
    
    def resume_pendulum_test(self):
        """진자운동 테스트 재개"""
        if self.pendulum_paused and self.pendulum_params:
            self.pendulum_paused = False
            params = self.pendulum_params
            self.start_pendulum_test(
                params['joint_index'],
                params['amplitude'],
                params['vel']
            )
            return True
        return False
    
    def _pendulum_loop(self, joint_index, amplitude, vel):
        """진자운동 루프 (별도 스레드에서 실행)"""
        try:
            from dsr_msgs2.srv import MoveJoint
            
            # 사용자 홈 위치를 중심으로 사용 (현재 위치 대신 고정된 홈 위치)
            # 사용자 홈: [0, 0, 90, 0, 90, 0] - 그리퍼가 아래를 바라보는 자세
            user_home_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
            center_pos = user_home_pos.copy()
            center_value = center_pos[joint_index]
            
            direction = 1  # 1: 양방향, -1: 음방향
            cycle_count = 0
            
            self.add_log('INFO', f'진자운동 중심 (사용자 홈): J{joint_index+1}={center_value:.1f}°')
            
            while self.pendulum_running:
                # 목표 위치 계산
                target_pos = center_pos.copy()
                target_pos[joint_index] = center_value + (amplitude * direction)
                
                # 이동 명령
                if not hasattr(self, 'cli_move_joint'):
                    break
                    
                req = MoveJoint.Request()
                req.pos = [float(p) for p in target_pos]  # float 타입으로 변환
                req.vel = float(vel)
                req.acc = float(vel)  # 가속도는 속도와 동일하게
                req.time = 0.0
                req.radius = 0.0
                req.mode = 0  # ABSOLUTE
                req.blend_type = 0
                req.sync_type = 0  # SYNC
                
                future = self.cli_move_joint.call_async(req)
                # 동기 대기
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                
                if not self.pendulum_running:
                    break
                
                # 방향 반전
                direction *= -1
                cycle_count += 1
                
                if cycle_count % 2 == 0:
                    self.add_log('INFO', f'진자운동 {cycle_count//2}회 완료')
                
                # 잠시 대기
                time.sleep(0.1)
                
        except Exception as e:
            self.get_logger().error(f'Pendulum test error: {e}')
            self.add_log('ERROR', f'진자운동 오류: {e}')
        finally:
            self.pendulum_running = False


# 전역 ROS 노드 참조
ros_node = None


def ros2_spin_thread(node):
    """ROS2 노드 스핀 스레드"""
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except Exception as e:
        print(f"ROS2 spin error: {e}")
    finally:
        executor.shutdown()


def emit_robot_data():
    """웹소켓으로 로봇 데이터 전송"""
    global current_task, ui_state
    last_robot_state = None
    
    while True:
        if robot_data['connected']:
            # 로봇 상태 변화 감지 (MOVING -> STANDBY)
            current_robot_state = robot_data.get('robot_state', 1)
            
            # 이동 작업(move_home)이 완료되었을 때 작업 상태 초기화
            if (last_robot_state == 2 and current_robot_state == 1 and 
                current_task.get('type') in ['move_home_user', 'move_home_mech'] and
                not current_task.get('paused')):
                print(f'✅ Move home completed: {current_task["type"]}')
                clear_current_task()
            
            last_robot_state = current_robot_state
            
            socketio.emit('robot_state', robot_data)
            socketio.emit('task_status', task_queue)
            socketio.emit('logs', logs[:20])
        time.sleep(0.1)


# UI 상태 (로컬/외부 웹 동기화용)
ui_state = {
    'pendulum_running': False,
    'is_moving': False,
    'is_stopped': False,
    'paused_task': None  # 일시정지된 작업 타입 ('pendulum', 'move_home_user', 'move_home_mech', None)
}

# 현재 작업 상태 (일시정지/재개용)
current_task = {
    'type': None,       # 'pendulum', 'move_home_user', 'move_home_mech', None
    'params': {},       # 작업 파라미터
    'paused': False     # 일시정지 상태
}


def clear_current_task():
    """현재 작업 초기화"""
    global current_task
    current_task = {'type': None, 'params': {}, 'paused': False}


def set_current_task(task_type, params=None):
    """현재 작업 설정 (이전 작업 취소)"""
    global current_task, ros_node, ui_state
    
    # 이전 작업이 있으면 중지
    if current_task['type'] == 'pendulum' and ros_node:
        ros_node.stop_pendulum_test()
    
    # UI 상태 초기화 - 새 작업 시작 시 정지 상태 해제
    ui_state['is_stopped'] = False
    ui_state['paused_task'] = None
    
    current_task = {
        'type': task_type,
        'params': params or {},
        'paused': False
    }
    
    # UI 상태 전송
    socketio.emit('ui_state', ui_state)


def pause_current_task():
    """현재 작업 일시정지"""
    global current_task, ros_node
    
    if current_task['type'] == 'pendulum' and ros_node:
        ros_node.pause_pendulum_test()
        current_task['paused'] = True
        return True
    elif current_task['type'] in ['move_home_user', 'move_home_mech']:
        current_task['paused'] = True
        return True
    return False


def resume_current_task():
    """현재 작업 재개"""
    global current_task, ros_node
    
    if not current_task['paused'] or current_task['type'] is None:
        return False
    
    current_task['paused'] = False
    
    if current_task['type'] == 'pendulum' and ros_node:
        params = current_task['params']
        ros_node.start_pendulum_test(
            params.get('joint_index', 4),
            params.get('amplitude', 15),
            params.get('velocity', 30)
        )
        return True
    elif current_task['type'] == 'move_home_user' and ros_node:
        ros_node.move_home(target=1)
        return True
    elif current_task['type'] == 'move_home_mech' and ros_node:
        ros_node.move_home(target=0)
        return True
    
    return False


def firebase_upload_thread():
    """Firebase에 로봇 상태 주기적 업로드 (1초마다)"""
    global firebase_ref, ros_node, ui_state
    
    if not firebase_enabled or firebase_ref is None:
        return
    
    print("🔥 Firebase 업로드 스레드 시작")
    
    while True:
        try:
            if robot_data['connected']:
                # 진자운동 상태 동기화
                if ros_node:
                    ui_state['pendulum_running'] = ros_node.pendulum_running
                
                # 업로드할 데이터 (필수 정보만)
                upload_data = {
                    'timestamp': time.time(),
                    'connected': robot_data['connected'],
                    'joint_position': robot_data['actual_joint_position'],
                    'tcp_position': robot_data['actual_tcp_position'],
                    'robot_state': robot_data['robot_state'],
                    'robot_mode': robot_data['robot_mode'],
                    'operation_speed': robot_data['operation_speed_rate'],
                    'access_control': robot_data['access_control'],
                    'gripper': {
                        'do1': (robot_data['controller_digital_output'] >> 0) & 1,
                        'do2': (robot_data['controller_digital_output'] >> 1) & 1,
                    },
                    'ui_state': ui_state
                }
                firebase_ref.update(upload_data)
        except Exception as e:
            print(f"Firebase 업로드 오류: {e}")
        
        time.sleep(1)  # 1초마다 업로드


def firebase_command_listener():
    """Firebase에서 제어 명령 수신 (리스너)"""
    global firebase_cmd_ref, ros_node, ui_state, current_task
    
    if not firebase_enabled or firebase_cmd_ref is None:
        return
    
    def on_command(event):
        """Firebase 명령 수신 콜백"""
        global ui_state, current_task
        
        if event.data is None:
            return
        
        try:
            cmd = event.data.get('command') if isinstance(event.data, dict) else None
            if cmd is None:
                return
            
            print(f"🔥 Firebase 명령 수신: {cmd}")
            
            # 명령 처리
            if cmd == 'gripper_open':
                if ros_node:
                    ros_node.set_gripper(open_gripper=True)
                    
            elif cmd == 'gripper_close':
                if ros_node:
                    ros_node.set_gripper(open_gripper=False)
                    
            elif cmd == 'move_home_user':
                if ros_node:
                    # 작업 상태 관리 시스템 사용
                    set_current_task('move_home_user', {'target': 1})
                    ros_node.move_home(target=1)
                    print('🔥 Firebase: Move to user home')
                    
            elif cmd == 'move_home_mech':
                if ros_node:
                    # 작업 상태 관리 시스템 사용
                    set_current_task('move_home_mech', {'target': 0})
                    ros_node.move_home(target=0)
                    print('🔥 Firebase: Move to mechanical home')
                    
            elif cmd == 'emergency_stop':
                if ros_node:
                    # 작업 상태 관리 시스템 사용
                    pause_current_task()
                    
                    from dsr_msgs2.srv import MoveStop
                    if hasattr(ros_node, 'cli_move_stop') and ros_node.cli_move_stop.service_is_ready():
                        req = MoveStop.Request()
                        req.stop_mode = 3  # DR_HOLD: HOLD stop (재개 가능)
                        ros_node.cli_move_stop.call_async(req)
                    
                    ui_state['is_stopped'] = True
                    ui_state['pendulum_running'] = False
                    ui_state['paused_task'] = current_task['type']
                    print(f'🔥 Firebase: Emergency stop (paused task: {current_task["type"]})')
                        
            elif cmd == 'move_resume':
                # 작업 상태 관리 시스템 사용
                resumed = resume_current_task()
                if resumed:
                    if current_task['type'] == 'pendulum':
                        ui_state['pendulum_running'] = True
                    print(f'🔥 Firebase: Task resumed ({current_task["type"]})')
                else:
                    print('🔥 Firebase: Resume - no paused task')
                ui_state['is_stopped'] = False
                ui_state['paused_task'] = None
                        
            elif cmd == 'pendulum_start':
                if ros_node:
                    joint = event.data.get('joint', 4)
                    amplitude = event.data.get('amplitude', 15)
                    velocity = event.data.get('velocity', 30)
                    # 작업 상태 관리 시스템 사용
                    set_current_task('pendulum', {
                        'joint_index': joint,
                        'amplitude': amplitude,
                        'velocity': velocity
                    })
                    ros_node.start_pendulum_test(joint, amplitude, velocity)
                    ui_state['pendulum_running'] = True
                    print(f'🔥 Firebase: Pendulum started J{joint+1}, ±{amplitude}°, {velocity}°/s')
                    
            elif cmd == 'pendulum_stop':
                if ros_node:
                    ros_node.stop_pendulum_test()
                    clear_current_task()
                    ui_state['pendulum_running'] = False
                    print('🔥 Firebase: Pendulum stopped')
                        
            elif cmd == 'speed_change':
                speed = event.data.get('value', 50)
                if ros_node:
                    from dsr_msgs2.srv import ChangeOperationSpeed
                    if hasattr(ros_node, 'cli_change_speed') and ros_node.cli_change_speed.service_is_ready():
                        req = ChangeOperationSpeed.Request()
                        req.speed = int(speed)
                        ros_node.cli_change_speed.call_async(req)
            
            # 명령 처리 완료 후 삭제 및 UI 상태 업데이트
            firebase_cmd_ref.update({'command': None, 'value': None, 'processed': time.time()})
            
            # 로컬 웹소켓으로도 상태 전파
            socketio.emit('ui_state', ui_state)
            if current_task['type'] == 'pendulum' or cmd in ['pendulum_start', 'pendulum_stop', 'emergency_stop', 'move_resume']:
                running = ui_state.get('pendulum_running', False)
                paused = current_task.get('paused', False)
                socketio.emit('pendulum_status', {'running': running, 'paused': paused})
            
        except Exception as e:
            print(f"Firebase 명령 처리 오류: {e}")
            import traceback
            traceback.print_exc()
    
    print("🔥 Firebase 명령 리스너 시작")
    firebase_cmd_ref.listen(on_command)


# Flask 라우트
@app.route('/')
def index():
    return render_template('index_full.html')


@app.route('/external')
def external():
    """외부 접속용 웹페이지 (Firebase 연동)"""
    return render_template('index_external.html')


@app.route('/api/status')
def get_status():
    return jsonify(robot_data)


@app.route('/api/tasks')
def get_tasks():
    return jsonify(task_queue)


@app.route('/api/logs')
def get_logs():
    return jsonify(logs[:50])


# RViz 스크린샷 스트리밍
def get_rviz_screenshot():
    """RViz 창 스크린샷 캡처 (gnome-screenshot 사용)"""
    try:
        # RViz 창 ID 찾기
        result = subprocess.run(
            ['xdotool', 'search', '--name', 'RViz'],
            capture_output=True, text=True, timeout=2
        )
        window_ids = result.stdout.strip().split('\n')
        
        if window_ids and window_ids[0]:
            # 첫 번째 RViz 창 캡처
            window_id = window_ids[0]
            
            # gnome-screenshot으로 창 캡처
            import tempfile
            with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as tmp:
                tmp_path = tmp.name
            
            # import 사용하여 특정 창 캡처
            subprocess.run(
                ['import', '-window', window_id, tmp_path],
                timeout=3, capture_output=True
            )
            
            with open(tmp_path, 'rb') as f:
                img_data = f.read()
            
            import os
            os.unlink(tmp_path)
            
            return img_data
    except Exception as e:
        print(f"Screenshot error: {e}")
    
    return None


def generate_rviz_stream():
    """RViz 스크린샷 MJPEG 스트림 생성"""
    while True:
        img_data = get_rviz_screenshot()
        if img_data:
            yield (b'--frame\r\n'
                   b'Content-Type: image/png\r\n\r\n' + img_data + b'\r\n')
        time.sleep(0.1)  # 10 FPS


@app.route('/rviz_stream')
def rviz_stream():
    """RViz 스트리밍 엔드포인트"""
    return Response(
        generate_rviz_stream(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/rviz_snapshot')
def rviz_snapshot():
    """RViz 단일 스크린샷"""
    img_data = get_rviz_screenshot()
    if img_data:
        return Response(img_data, mimetype='image/png')
    return "RViz not found", 404


# SocketIO 이벤트
@socketio.on('connect')
def handle_connect(auth=None):
    print('✅ Client connected')
    # 연결 시 초기 데이터는 emit_robot_data 스레드에서 전송


@socketio.on('disconnect')
def handle_disconnect():
    print('❌ Client disconnected')


def get_kst_time():
    """한국시간(KST) 반환"""
    kst = timezone(timedelta(hours=9))
    return datetime.now(kst).strftime('%H:%M:%S')


@socketio.on('gripper_command')
def handle_gripper(data):
    global ros_node
    command = data.get('command', 'close')
    print(f'🤖 Gripper command: {command}')
    
    # 실제 그리퍼 제어
    if ros_node is not None:
        open_gripper = (command == 'open')
        success = ros_node.set_gripper(open_gripper)
        if success:
            log_entry = {
                'time': get_kst_time(),
                'level': 'INFO',
                'message': f'그리퍼 {command} 명령 성공'
            }
        else:
            log_entry = {
                'time': get_kst_time(),
                'level': 'WARN',
                'message': f'그리퍼 {command} 명령 실패 (서비스 미연결)'
            }
    else:
        log_entry = {
            'time': get_kst_time(),
            'level': 'ERROR',
            'message': 'ROS 노드가 초기화되지 않음'
        }
    
    logs.insert(0, log_entry)


@socketio.on('speed_change')
def handle_speed(data):
    global ros_node
    speed = data.get('speed', 50)
    print(f'🚀 Speed change: {speed}%')
    
    success = False
    if ros_node is not None:
        try:
            from dsr_msgs2.srv import ChangeOperationSpeed
            
            if hasattr(ros_node, 'cli_change_speed') and ros_node.cli_change_speed.service_is_ready():
                req = ChangeOperationSpeed.Request()
                req.speed = int(speed)
                future = ros_node.cli_change_speed.call_async(req)
                success = True
                print(f'✅ Operation speed changed to {speed}%')
            else:
                print('⚠️ ChangeOperationSpeed service not ready')
        except Exception as e:
            print(f'❌ Speed change error: {e}')
    
    log_entry = {
        'time': get_kst_time(),
        'level': 'INFO' if success else 'WARN',
        'message': f'작업 속도 변경: {speed}%' + (' ✅' if success else ' (서비스 미연결)')
    }
    logs.insert(0, log_entry)


@socketio.on('move_home')
def handle_move_home(data):
    global ros_node, current_task
    target = data.get('target', 1)  # 1=사용자 홈 (그리퍼 아래 방향)
    home_type = "기계적 홈" if target == 0 else "사용자 홈"
    task_type = 'move_home_mech' if target == 0 else 'move_home_user'
    print(f'🏠 Move Home command: {home_type}')
    
    if ros_node is not None:
        # 새 작업 설정 (이전 작업 취소)
        set_current_task(task_type, {'target': target})
        
        success = ros_node.move_home(target)
        if success:
            log_entry = {
                'time': get_kst_time(),
                'level': 'INFO',
                'message': f'홈 이동 명령 전송 ({home_type})'
            }
        else:
            clear_current_task()
            log_entry = {
                'time': get_kst_time(),
                'level': 'WARN',
                'message': f'홈 이동 명령 실패 (서비스 미연결)'
            }
    else:
        log_entry = {
            'time': get_kst_time(),
            'level': 'ERROR',
            'message': 'ROS 노드가 초기화되지 않음'
        }
    
    logs.insert(0, log_entry)


@socketio.on('emergency_stop')
def handle_estop():
    global ros_node, current_task
    print('🛑 EMERGENCY STOP')
    
    success = False
    had_task = current_task['type'] is not None
    
    if ros_node is not None:
        try:
            # 현재 작업 일시정지
            pause_current_task()
            
            from dsr_msgs2.srv import MoveStop
            
            if hasattr(ros_node, 'cli_move_stop') and ros_node.cli_move_stop.service_is_ready():
                req = MoveStop.Request()
                req.stop_mode = 3  # DR_HOLD: HOLD stop (재개 가능)
                future = ros_node.cli_move_stop.call_async(req)
                success = True
                print('✅ MoveStop command sent (HOLD - resumable)')
            else:
                print('⚠️ MoveStop service not ready')
        except Exception as e:
            print(f'❌ Emergency stop error: {e}')
    
    # UI 상태 업데이트
    ui_state['pendulum_running'] = False
    ui_state['is_stopped'] = True
    ui_state['paused_task'] = current_task['type']  # 일시정지된 작업 타입
    
    # UI 상태 전송
    socketio.emit('pendulum_status', {'running': False, 'paused': current_task['paused']})
    socketio.emit('ui_state', ui_state)
    
    task_info = f" ({current_task['type']} 일시정지)" if current_task['type'] else ""
    log_entry = {
        'time': get_kst_time(),
        'level': 'ERROR',
        'message': f'🛑 긴급정지 실행' + (' ✅' if success else ' (서비스 미연결)') + task_info
    }
    logs.insert(0, log_entry)


@socketio.on('move_pause')
def handle_move_pause():
    """더 이상 사용하지 않음 - emergency_stop으로 통합"""
    pass


@socketio.on('move_resume')
def handle_move_resume():
    global ros_node, ui_state, current_task
    print('▶️ MOVE RESUME')
    
    resumed = False
    task_type = current_task.get('type')
    
    # 일시정지된 작업이 있다면 재개
    if current_task['paused'] and task_type:
        resumed = resume_current_task()
        if resumed:
            print(f'✅ Task resumed: {task_type}')
            if task_type == 'pendulum':
                ui_state['pendulum_running'] = True
    
    # UI 상태 업데이트 - 정지 상태 해제
    ui_state['is_stopped'] = False
    ui_state['paused_task'] = None
    socketio.emit('ui_state', ui_state)
    
    if resumed:
        if task_type == 'pendulum' and ros_node and ros_node.pendulum_params:
            socketio.emit('pendulum_status', {
                'running': True, 
                'joint': ros_node.pendulum_params['joint_index'],
                'amplitude': ros_node.pendulum_params['amplitude'],
                'velocity': ros_node.pendulum_params['vel']
            })
        
        task_names = {
            'pendulum': '진자운동',
            'move_home_user': '사용자 홈 이동',
            'move_home_mech': '기계적 홈 이동'
        }
        log_entry = {
            'time': get_kst_time(),
            'level': 'INFO',
            'message': f'▶️ 재개 - {task_names.get(task_type, task_type)} 이어서 실행'
        }
    else:
        socketio.emit('pendulum_status', {'running': False})
        log_entry = {
            'time': get_kst_time(),
            'level': 'INFO',
            'message': '▶️ 재개 - 새 명령 대기 상태'
        }
    
    logs.insert(0, log_entry)
    print('✅ Resume completed')


@socketio.on('pendulum_start')
def handle_pendulum_start(data):
    global ros_node, ui_state, current_task
    joint_index = data.get('joint', 0)  # 0~5
    amplitude = data.get('amplitude', 15.0)  # 진폭 (deg)
    velocity = data.get('velocity', 30.0)  # 속도 (deg/s)
    
    print(f'🔄 Pendulum test start: J{joint_index+1}, ±{amplitude}°, {velocity}°/s')
    
    if ros_node is not None:
        # 새 작업 설정 (이전 작업 취소)
        set_current_task('pendulum', {
            'joint_index': joint_index,
            'amplitude': amplitude,
            'velocity': velocity
        })
        
        success = ros_node.start_pendulum_test(joint_index, amplitude, velocity)
        if success:
            ui_state['pendulum_running'] = True
            log_entry = {
                'time': get_kst_time(),
                'level': 'INFO',
                'message': f'진자운동 테스트 시작 (J{joint_index+1}, ±{amplitude}°, {velocity}°/s)'
            }
            socketio.emit('pendulum_status', {'running': True, 'joint': joint_index, 'amplitude': amplitude, 'velocity': velocity})
            socketio.emit('ui_state', ui_state)
        else:
            log_entry = {
                'time': get_kst_time(),
                'level': 'WARN',
                'message': '진자운동 테스트가 이미 실행 중'
            }
    else:
        log_entry = {
            'time': get_kst_time(),
            'level': 'ERROR',
            'message': 'ROS 노드가 초기화되지 않음'
        }
    
    logs.insert(0, log_entry)


@socketio.on('pendulum_stop')
def handle_pendulum_stop():
    global ros_node, ui_state
    print('⏹️ Pendulum test stop')
    
    if ros_node is not None:
        ros_node.stop_pendulum_test()
        clear_current_task()  # 작업 상태 초기화
        ui_state['pendulum_running'] = False
        log_entry = {
            'time': get_kst_time(),
            'level': 'INFO',
            'message': '진자운동 테스트 정지'
        }
        socketio.emit('pendulum_status', {'running': False})
        socketio.emit('ui_state', ui_state)
    else:
        log_entry = {
            'time': get_kst_time(),
            'level': 'ERROR',
            'message': 'ROS 노드가 초기화되지 않음'
        }
    
    logs.insert(0, log_entry)


@socketio.on('get_ui_state')
def handle_get_ui_state():
    """클라이언트 연결시 현재 UI 상태 전송"""
    global ui_state, ros_node
    if ros_node:
        ui_state['pendulum_running'] = ros_node.pendulum_running
    socketio.emit('ui_state', ui_state)


# ===== 분류 제어 기능 (ROS2 서비스 연동) =====
sort_state = {
    'is_running': False,
    'is_paused': False,
    'cycle_count': 0,
    'current_phase': '-',
    'last_classification': '-',
    'completed': 0,
    'errors': 0,
    'small': 0,
    'medium': 0,
    'large': 0
}

# 분류 서비스 클라이언트들 (ROS2 노드에서 생성)
sort_start_client = None
sort_stop_client = None
sort_pause_client = None
sort_status_subscriber = None


def init_sort_service_clients():
    """분류 서비스 클라이언트 초기화 (ROS2 노드 시작 후 호출)"""
    global ros_node, sort_start_client, sort_stop_client, sort_pause_client, sort_status_subscriber
    
    if ros_node is None:
        print("⚠️ ROS 노드가 없어 분류 서비스 클라이언트를 생성할 수 없습니다")
        return
    
    try:
        from std_srvs.srv import Trigger, SetBool
        
        sort_start_client = ros_node.create_client(Trigger, '/dlar/start_sort')
        sort_stop_client = ros_node.create_client(Trigger, '/dlar/stop_sort')
        sort_pause_client = ros_node.create_client(SetBool, '/dlar/pause_sort')
        
        # 분류 상태 구독
        from std_msgs.msg import String
        sort_status_subscriber = ros_node.create_subscription(
            String,
            '/dlar/status',
            sort_status_callback,
            10
        )
        
        print("✅ 분류 서비스 클라이언트 초기화 완료")
    except Exception as e:
        print(f"⚠️ 분류 서비스 클라이언트 초기화 실패: {e}")


def sort_status_callback(msg):
    """분류 상태 업데이트 콜백"""
    global sort_state
    try:
        import json
        data = json.loads(msg.data)
        sort_state.update(data)
        socketio.emit('sort_status', sort_state)
    except Exception as e:
        print(f"분류 상태 파싱 오류: {e}")


@socketio.on('sort_start')
def handle_sort_start():
    """분류 시작"""
    global sort_start_client, sort_state
    
    if sort_start_client is None:
        socketio.emit('sort_log', {'message': '❌ 분류 서비스가 연결되지 않았습니다'})
        return
    
    try:
        from std_srvs.srv import Trigger
        
        if not sort_start_client.wait_for_service(timeout_sec=1.0):
            socketio.emit('sort_log', {'message': '❌ 분류 서비스 응답 없음'})
            return
        
        request = Trigger.Request()
        future = sort_start_client.call_async(request)
        
        # 비동기 결과 처리
        def on_result(future):
            try:
                result = future.result()
                if result.success:
                    socketio.emit('sort_log', {'message': '✅ 분류 시작됨'})
                else:
                    socketio.emit('sort_log', {'message': f'❌ 시작 실패: {result.message}'})
            except Exception as e:
                socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})
        
        future.add_done_callback(on_result)
        
    except Exception as e:
        socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})


@socketio.on('sort_stop')
def handle_sort_stop():
    """분류 정지"""
    global sort_stop_client
    
    if sort_stop_client is None:
        socketio.emit('sort_log', {'message': '❌ 분류 서비스가 연결되지 않았습니다'})
        return
    
    try:
        from std_srvs.srv import Trigger
        
        if not sort_stop_client.wait_for_service(timeout_sec=1.0):
            socketio.emit('sort_log', {'message': '❌ 분류 서비스 응답 없음'})
            return
        
        request = Trigger.Request()
        future = sort_stop_client.call_async(request)
        
        def on_result(future):
            try:
                result = future.result()
                if result.success:
                    socketio.emit('sort_log', {'message': '✅ 분류 정지됨'})
                else:
                    socketio.emit('sort_log', {'message': f'❌ 정지 실패: {result.message}'})
            except Exception as e:
                socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})
        
        future.add_done_callback(on_result)
        
    except Exception as e:
        socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})


@socketio.on('sort_pause')
def handle_sort_pause():
    """분류 일시정지"""
    global sort_pause_client
    
    if sort_pause_client is None:
        socketio.emit('sort_log', {'message': '❌ 분류 서비스가 연결되지 않았습니다'})
        return
    
    try:
        from std_srvs.srv import SetBool
        
        if not sort_pause_client.wait_for_service(timeout_sec=1.0):
            socketio.emit('sort_log', {'message': '❌ 분류 서비스 응답 없음'})
            return
        
        request = SetBool.Request()
        request.data = True  # 일시정지
        future = sort_pause_client.call_async(request)
        
        def on_result(future):
            try:
                result = future.result()
                if result.success:
                    socketio.emit('sort_log', {'message': '⏸️ 분류 일시정지됨'})
                else:
                    socketio.emit('sort_log', {'message': f'❌ 일시정지 실패: {result.message}'})
            except Exception as e:
                socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})
        
        future.add_done_callback(on_result)
        
    except Exception as e:
        socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})


@socketio.on('sort_resume')
def handle_sort_resume():
    """분류 재개"""
    global sort_pause_client
    
    if sort_pause_client is None:
        socketio.emit('sort_log', {'message': '❌ 분류 서비스가 연결되지 않았습니다'})
        return
    
    try:
        from std_srvs.srv import SetBool
        
        if not sort_pause_client.wait_for_service(timeout_sec=1.0):
            socketio.emit('sort_log', {'message': '❌ 분류 서비스 응답 없음'})
            return
        
        request = SetBool.Request()
        request.data = False  # 재개
        future = sort_pause_client.call_async(request)
        
        def on_result(future):
            try:
                result = future.result()
                if result.success:
                    socketio.emit('sort_log', {'message': '▶️ 분류 재개됨'})
                else:
                    socketio.emit('sort_log', {'message': f'❌ 재개 실패: {result.message}'})
            except Exception as e:
                socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})
        
        future.add_done_callback(on_result)
        
    except Exception as e:
        socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})


def emit_sort_status():
    """주기적으로 분류 상태를 emit (dlar_sort_node 없을 때 대비)"""
    global sort_state
    socketio.emit('sort_status', sort_state)


# ===== 컨베이어 연동 기능 =====
conveyor_mode_client = None


def init_conveyor_service_clients():
    """컨베이어 서비스 클라이언트 초기화"""
    global ros_node, conveyor_mode_client
    
    if ros_node is None:
        return
    
    try:
        from std_srvs.srv import SetBool
        
        conveyor_mode_client = ros_node.create_client(SetBool, '/dlar/conveyor_mode')
        print("✅ 컨베이어 모드 서비스 클라이언트 초기화 완료")
    except Exception as e:
        print(f"⚠️ 컨베이어 서비스 클라이언트 초기화 실패: {e}")


@socketio.on('conveyor_mode')
def handle_conveyor_mode(data):
    """컨베이어 자동 모드 설정"""
    global conveyor_mode_client
    
    enabled = data.get('enabled', False)
    
    if conveyor_mode_client is None:
        socketio.emit('sort_log', {'message': '❌ 컨베이어 서비스가 연결되지 않았습니다'})
        return
    
    try:
        from std_srvs.srv import SetBool
        
        if not conveyor_mode_client.wait_for_service(timeout_sec=1.0):
            socketio.emit('sort_log', {'message': '❌ 컨베이어 서비스 응답 없음'})
            return
        
        request = SetBool.Request()
        request.data = enabled
        future = conveyor_mode_client.call_async(request)
        
        def on_result(future):
            try:
                result = future.result()
                if result.success:
                    mode_text = '활성화' if enabled else '비활성화'
                    socketio.emit('sort_log', {'message': f'✅ 컨베이어 자동 모드 {mode_text}'})
                else:
                    socketio.emit('sort_log', {'message': f'❌ 모드 변경 실패: {result.message}'})
            except Exception as e:
                socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})
        
        future.add_done_callback(on_result)
        
    except Exception as e:
        socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})


@socketio.on('conveyor_resume')
def handle_conveyor_resume():
    """컨베이어 수동 재시작"""
    global ros_node
    
    if ros_node is None:
        socketio.emit('sort_log', {'message': '❌ ROS 노드가 초기화되지 않았습니다'})
        return
    
    try:
        from std_msgs.msg import String
        
        # conveyor/cmd 토픽에 RESUME 발행
        if not hasattr(ros_node, 'conveyor_cmd_pub'):
            ros_node.conveyor_cmd_pub = ros_node.create_publisher(String, 'conveyor/cmd', 10)
        
        msg = String()
        msg.data = 'RESUME'
        ros_node.conveyor_cmd_pub.publish(msg)
        socketio.emit('sort_log', {'message': '✅ 컨베이어 재시작 명령 전송'})
        
    except Exception as e:
        socketio.emit('sort_log', {'message': f'❌ 오류: {str(e)}'})


if __name__ == '__main__':
    print("=" * 70)
    print("📦 물류 로봇 모니터링 시스템 (Real Robot)")
    print("   Namespace: /dsr01")
    print("   그리퍼 제어: Tool Digital Output 1")
    print("=" * 70)
    
    # ROS2 초기화
    try:
        rclpy.init()
        ros_node = RobotStateSubscriber()
        
        ros_thread = threading.Thread(target=ros2_spin_thread, args=(ros_node,), daemon=True)
        ros_thread.start()
        print("✅ ROS2 Node started")
        print("✅ Gripper control enabled")
        
        # 분류 서비스 클라이언트 초기화
        init_sort_service_clients()
        
        # 컨베이어 서비스 클라이언트 초기화
        init_conveyor_service_clients()
        
    except Exception as e:
        print(f"⚠️ ROS2 initialization failed: {e}")
        import traceback
        traceback.print_exc()
        ros_node = None
    
    # 로컬 웹소켓 전송 스레드
    emit_thread = threading.Thread(target=emit_robot_data, daemon=True)
    emit_thread.start()
    
    # Firebase 스레드 (활성화된 경우에만)
    if firebase_enabled:
        firebase_upload = threading.Thread(target=firebase_upload_thread, daemon=True)
        firebase_upload.start()
        
        firebase_listener = threading.Thread(target=firebase_command_listener, daemon=True)
        firebase_listener.start()
        
        print("✅ Firebase 연동 시작")
    
    print(f"📍 Web UI: http://localhost:5000")
    print(f"📍 REST API: http://localhost:5000/api/status")
    print("=" * 70)
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
