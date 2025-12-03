#!/usr/bin/env python3
"""
물류 로봇 모니터링 웹 서버 노드 (ROS2 래핑)
- Flask + SocketIO 웹 서버
- 분류 작업 제어 (dlar_sort_node 연동)
- Firebase 연동
"""

import os
import sys
import threading
import time
import json
from datetime import datetime, timezone, timedelta

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import (
    GetCurrentTool, GetCurrentTcp,
    GetCurrentPosx, GetCurrentPosj, GetCurrentVelx, GetCurrentVelj,
    GetDesiredPosx, GetDesiredPosj, GetDesiredVelx, GetDesiredVelj,
    GetCurrentToolFlangePosx,
    GetToolForce, GetExternalTorque, GetJointTorque,
    GetCtrlBoxDigitalOutput, SetCtrlBoxDigitalOutput,
    GetCtrlBoxDigitalInput, GetToolDigitalInput, GetToolDigitalOutput,
    GetCtrlBoxAnalogInput,
    SetToolDigitalOutput,
    MoveHome, ChangeOperationSpeed, MoveStop,
    MovePause, MoveResume, MoveJoint
)
from dsr_msgs2.msg import RobotError

# Flask 관련
from flask import Flask, render_template, jsonify, Response
from flask_socketio import SocketIO

# 경로 설정 (logistics_monitor 템플릿 사용)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 절대 경로로 설정
LOGISTICS_MONITOR_DIR = os.path.expanduser('~/cobot1_ws/src/logistics_monitor')
TEMPLATE_DIR = os.path.join(LOGISTICS_MONITOR_DIR, 'templates')
CONFIG_DIR = os.path.join(LOGISTICS_MONITOR_DIR, 'config')

# Flask 앱 생성
app = Flask(__name__, template_folder=TEMPLATE_DIR)
app.config['SECRET_KEY'] = 'dsr_integrated_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Firebase 연동 (선택적)
firebase_enabled = False
firebase_ref = None
firebase_cmd_ref = None

try:
    import firebase_admin
    from firebase_admin import credentials, db
    
    SERVICE_ACCOUNT_KEY_PATH = os.path.join(CONFIG_DIR, 'serviceAccountKey.json')
    WEB_CONFIG_PATH = os.path.join(CONFIG_DIR, 'firebase_web_config.json')
    
    if os.path.exists(SERVICE_ACCOUNT_KEY_PATH) and os.path.exists(WEB_CONFIG_PATH):
        with open(WEB_CONFIG_PATH, 'r') as f:
            web_config = json.load(f)
        DATABASE_URL = web_config.get('databaseURL')
        
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        
        firebase_ref = db.reference('/robot_status')
        firebase_cmd_ref = db.reference('/robot_commands')
        
        firebase_enabled = True
        print("✅ Firebase 연동 활성화")
except Exception as e:
    print(f"⚠️ Firebase 초기화 실패: {e}")

# 전역 변수 - app_real.py와 동일한 구조
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

sort_status = {
    'running': False,
    'paused': False,
    'phase': 'PICK',
    'cycle_count': 0,
    'last_width': None,
    'dsr_ready': False,
}

conveyor_status = {
    'connected': False,
    'status': 'IDLE',
    'status_code': 0,
}

# UI 상태 (로컬/외부 웹 동기화용)
ui_state = {
    'pendulum_running': False,
    'is_moving': False,
    'is_stopped': False,
    'paused_task': None  # 일시정지된 작업 타입
}

# 현재 작업 상태 (일시정지/재개용)
current_task = {
    'type': None,       # 'pendulum', 'move_home_user', 'move_home_mech', None
    'params': {},       # 작업 파라미터
    'paused': False     # 일시정지 상태
}

logs = []
ros_node = None


class WebServerNode(Node):
    """ROS2 웹 서버 노드"""
    
    def __init__(self):
        super().__init__('web_server_node')
        self.get_logger().info('Web Server Node 시작')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 조인트 상태 구독
        self.joint_sub = self.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 분류 상태 구독
        self.sort_status_sub = self.create_subscription(
            String,
            '/dlar/status',
            self.sort_status_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 컨베이어 상태 구독
        self.conveyor_status_sub = self.create_subscription(
            String,
            '/conveyor/status',
            self.conveyor_status_callback,
            10,
            callback_group=self.callback_group
        )
        self.conveyor_code_sub = self.create_subscription(
            Int32,
            '/conveyor/status_code',
            self.conveyor_code_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 컨베이어 명령 퍼블리셔
        self.conveyor_cmd_pub = self.create_publisher(
            String,
            '/conveyor/cmd',
            10
        )
        
        # 분류 제어 서비스 클라이언트
        self.cli_start_sort = self.create_client(
            Trigger, '/dlar/start_sort',
            callback_group=self.callback_group
        )
        self.cli_stop_sort = self.create_client(
            Trigger, '/dlar/stop_sort',
            callback_group=self.callback_group
        )
        self.cli_pause_sort = self.create_client(
            SetBool, '/dlar/pause_sort',
            callback_group=self.callback_group
        )
        self.cli_reset_state = self.create_client(
            Trigger, '/dlar/reset_state',
            callback_group=self.callback_group
        )
        
        # 컨베이어 자동 모드 서비스 클라이언트
        self.cli_conveyor_mode = self.create_client(
            SetBool, '/dlar/conveyor_mode',
            callback_group=self.callback_group
        )
        
        # Tool/TCP 서비스 클라이언트
        self.cli_get_tool = self.create_client(
            GetCurrentTool, '/dsr01/tool/get_current_tool',
            callback_group=self.callback_group
        )
        self.cli_get_tcp = self.create_client(
            GetCurrentTcp, '/dsr01/tcp/get_current_tcp',
            callback_group=self.callback_group
        )
        
        # 추가 상태 서비스 클라이언트
        self.cli_get_posx = self.create_client(
            GetCurrentPosx, '/dsr01/aux_control/get_current_posx',
            callback_group=self.callback_group
        )
        self.cli_get_flange_posx = self.create_client(
            GetCurrentToolFlangePosx, '/dsr01/aux_control/get_current_tool_flange_posx',
            callback_group=self.callback_group
        )
        self.cli_get_digital_output = self.create_client(
            GetCtrlBoxDigitalOutput, '/dsr01/io/get_ctrl_box_digital_output',
            callback_group=self.callback_group
        )
        
        # 추가 상태 조회 서비스 클라이언트 (app_real.py 방식)
        self.cli_posj = self.create_client(
            GetCurrentPosj, '/dsr01/aux_control/get_current_posj',
            callback_group=self.callback_group
        )
        self.cli_velx = self.create_client(
            GetCurrentVelx, '/dsr01/aux_control/get_current_velx',
            callback_group=self.callback_group
        )
        self.cli_velj = self.create_client(
            GetCurrentVelj, '/dsr01/aux_control/get_current_velj',
            callback_group=self.callback_group
        )
        self.cli_tool_force = self.create_client(
            GetToolForce, '/dsr01/aux_control/get_tool_force',
            callback_group=self.callback_group
        )
        self.cli_ext_torque = self.create_client(
            GetExternalTorque, '/dsr01/aux_control/get_external_torque',
            callback_group=self.callback_group
        )
        self.cli_joint_torque = self.create_client(
            GetJointTorque, '/dsr01/aux_control/get_joint_torque',
            callback_group=self.callback_group
        )
        self.cli_desired_posx = self.create_client(
            GetDesiredPosx, '/dsr01/aux_control/get_desired_posx',
            callback_group=self.callback_group
        )
        self.cli_desired_posj = self.create_client(
            GetDesiredPosj, '/dsr01/aux_control/get_desired_posj',
            callback_group=self.callback_group
        )
        self.cli_desired_velx = self.create_client(
            GetDesiredVelx, '/dsr01/aux_control/get_desired_velx',
            callback_group=self.callback_group
        )
        self.cli_desired_velj = self.create_client(
            GetDesiredVelj, '/dsr01/aux_control/get_desired_velj',
            callback_group=self.callback_group
        )
        self.cli_din = self.create_client(
            GetCtrlBoxDigitalInput, '/dsr01/io/get_ctrl_box_digital_input',
            callback_group=self.callback_group
        )
        self.cli_dout = self.create_client(
            GetCtrlBoxDigitalOutput, '/dsr01/io/get_ctrl_box_digital_output',
            callback_group=self.callback_group
        )
        self.cli_tool_din = self.create_client(
            GetToolDigitalInput, '/dsr01/io/get_tool_digital_input',
            callback_group=self.callback_group
        )
        self.cli_tool_dout = self.create_client(
            GetToolDigitalOutput, '/dsr01/io/get_tool_digital_output',
            callback_group=self.callback_group
        )
        
        # 제어 서비스 클라이언트
        self.cli_set_digital_output = self.create_client(
            SetCtrlBoxDigitalOutput, '/dsr01/io/set_ctrl_box_digital_output',
            callback_group=self.callback_group
        )
        self.cli_set_tool_dout = self.create_client(
            SetToolDigitalOutput, '/dsr01/io/set_tool_digital_output',
            callback_group=self.callback_group
        )
        self.cli_move_home = self.create_client(
            MoveHome, '/dsr01/motion/move_home',
            callback_group=self.callback_group
        )
        self.cli_move_joint = self.create_client(
            MoveJoint, '/dsr01/motion/move_joint',
            callback_group=self.callback_group
        )
        self.cli_change_speed = self.create_client(
            ChangeOperationSpeed, '/dsr01/motion/change_operation_speed',
            callback_group=self.callback_group
        )
        self.cli_move_stop = self.create_client(
            MoveStop, '/dsr01/motion/move_stop',
            callback_group=self.callback_group
        )
        self.cli_move_pause = self.create_client(
            MovePause, '/dsr01/motion/move_pause',
            callback_group=self.callback_group
        )
        self.cli_move_resume = self.create_client(
            MoveResume, '/dsr01/motion/move_resume',
            callback_group=self.callback_group
        )
        
        # RobotError 구독 시도
        try:
            self.error_sub = self.create_subscription(
                RobotError,
                '/dsr01/error',
                self.error_callback,
                10,
                callback_group=self.callback_group
            )
            self.get_logger().info('RobotError 구독 완료')
        except Exception as e:
            self.get_logger().warn(f'RobotError 구독 실패: {e}')
        
        self.start_time = time.time()
        
        # 로봇 상태 업데이트 타이머 (0.2초마다 - 비동기 호출)
        self.robot_status_timer = self.create_timer(0.2, self.update_robot_status)
        
        # 진자운동 테스트 상태
        self.pendulum_running = False
        self.pendulum_thread = None
        self.pendulum_paused = False
        self.pendulum_params = None
        
        robot_data['connected'] = True
        self.get_logger().info('Web Server Node 준비 완료')
    
    def joint_state_callback(self, msg):
        """조인트 상태 콜백 - app_real.py 방식"""
        global robot_data
        import math
        
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
        robot_data['connected'] = True
    
    def sort_status_callback(self, msg):
        """분류 상태 콜백"""
        global sort_status
        try:
            sort_status = json.loads(msg.data)
        except:
            pass
    
    def conveyor_status_callback(self, msg):
        """컨베이어 상태 콜백"""
        global conveyor_status
        conveyor_status['status'] = msg.data
        conveyor_status['connected'] = True
    
    def conveyor_code_callback(self, msg):
        """컨베이어 상태 코드 콜백"""
        global conveyor_status
        conveyor_status['status_code'] = msg.data
    
    def error_callback(self, msg):
        """RobotError 메시지 콜백"""
        global robot_data, logs
        
        robot_data['error_level'] = msg.level
        robot_data['error_code'] = msg.code
        robot_data['error_msg'] = msg.msg1 if hasattr(msg, 'msg1') else str(msg)
        
        if msg.level > 0:
            level_str = ['INFO', 'WARN', 'ERROR'][min(msg.level - 1, 2)]
            # 전역 add_log 함수 호출
            kst = timezone(timedelta(hours=9))
            log_entry = {
                'time': datetime.now(kst).strftime('%H:%M:%S'),
                'level': level_str,
                'message': f'[{msg.code}] {robot_data["error_msg"]}'
            }
            logs.insert(0, log_entry)
            if len(logs) > 100:
                logs.pop()
    
    def update_robot_status(self):
        """로봇 상태 정보 업데이트 (비동기 호출) - app_real.py 방식"""
        global robot_data
        
        # 런닝 타임 업데이트
        robot_data['running_time'] = time.time() - self.start_time
        
        # Get current TCP position (posx) - 비동기 + 콜백
        if self.cli_get_posx.service_is_ready():
            try:
                req = GetCurrentPosx.Request()
                req.ref = 0  # DR_BASE
                future = self.cli_get_posx.call_async(req)
                future.add_done_callback(self.posx_callback)
            except Exception as e:
                self.get_logger().debug(f'Posx 조회 실패: {e}')
        
        # Get desired TCP position
        if self.cli_desired_posx.service_is_ready():
            try:
                req = GetDesiredPosx.Request()
                req.ref = 0  # DR_BASE
                future = self.cli_desired_posx.call_async(req)
                future.add_done_callback(self.desired_posx_callback)
            except Exception as e:
                self.get_logger().debug(f'Desired posx 조회 실패: {e}')
        
        # Get desired joint position
        if self.cli_desired_posj.service_is_ready():
            try:
                req = GetDesiredPosj.Request()
                future = self.cli_desired_posj.call_async(req)
                future.add_done_callback(self.desired_posj_callback)
            except Exception as e:
                self.get_logger().debug(f'Desired posj 조회 실패: {e}')
        
        # Get tool flange position - 비동기 + 콜백
        if self.cli_get_flange_posx.service_is_ready():
            try:
                req = GetCurrentToolFlangePosx.Request()
                req.ref = 0  # DR_BASE
                future = self.cli_get_flange_posx.call_async(req)
                future.add_done_callback(self.flange_posx_callback)
            except Exception as e:
                self.get_logger().debug(f'Flange posx 조회 실패: {e}')
        
        # Get tool force
        if self.cli_tool_force.service_is_ready():
            try:
                req = GetToolForce.Request()
                req.ref = 0
                future = self.cli_tool_force.call_async(req)
                future.add_done_callback(self.tool_force_callback)
            except Exception as e:
                self.get_logger().debug(f'Tool force 조회 실패: {e}')
        
        # Get joint torque
        if self.cli_joint_torque.service_is_ready():
            try:
                req = GetJointTorque.Request()
                future = self.cli_joint_torque.call_async(req)
                future.add_done_callback(self.joint_torque_callback)
            except Exception as e:
                self.get_logger().debug(f'Joint torque 조회 실패: {e}')
        
        # Get current tool name - 비동기 + 콜백
        if self.cli_get_tool.service_is_ready():
            try:
                req = GetCurrentTool.Request()
                future = self.cli_get_tool.call_async(req)
                future.add_done_callback(self.tool_callback)
            except Exception as e:
                self.get_logger().debug(f'Tool 정보 조회 실패: {e}')
        
        # Get current TCP name - 비동기 + 콜백
        if self.cli_get_tcp.service_is_ready():
            try:
                req = GetCurrentTcp.Request()
                future = self.cli_get_tcp.call_async(req)
                future.add_done_callback(self.tcp_callback)
            except Exception as e:
                self.get_logger().debug(f'TCP 정보 조회 실패: {e}')
        
        # Get digital output status (DO1, DO2만 - 그리퍼 상태용)
        if self.cli_get_digital_output.service_is_ready():
            try:
                for pin in [1, 2]:
                    req = GetCtrlBoxDigitalOutput.Request()
                    req.index = pin
                    future = self.cli_get_digital_output.call_async(req)
                    future.add_done_callback(lambda f, p=pin: self.dio_callback(f, p))
            except Exception as e:
                self.get_logger().debug(f'Digital output 조회 실패: {e}')
    
    def posx_callback(self, future):
        """TCP 위치 서비스 응답 콜백"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                pos_data = result.task_pos_info
                # task_pos_info는 리스트, 첫 번째 요소의 data 추출
                if isinstance(pos_data, list) and len(pos_data) > 0:
                    first_item = pos_data[0]
                    if hasattr(first_item, 'data'):
                        robot_data['actual_tcp_position'] = list(first_item.data)[:6]
                elif hasattr(pos_data, 'data'):
                    robot_data['actual_tcp_position'] = list(pos_data.data)[:6]
        except Exception as e:
            self.get_logger().debug(f'posx_callback error: {e}')
    
    def flange_posx_callback(self, future):
        """Flange 위치 서비스 응답 콜백"""
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
            self.get_logger().debug(f'flange_posx_callback error: {e}')
    
    def tool_callback(self, future):
        """Tool 정보 서비스 응답 콜백"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                robot_data['current_tool_name'] = result.info if result.info else ''
        except Exception as e:
            self.get_logger().debug(f'tool_callback error: {e}')
    
    def tcp_callback(self, future):
        """TCP 정보 서비스 응답 콜백"""
        global robot_data
        try:
            result = future.result()
            if result.success:
                robot_data['current_tcp_name'] = result.info if result.info else ''
        except Exception as e:
            self.get_logger().debug(f'tcp_callback error: {e}')
    
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
                torque_data = result.jts
                if hasattr(torque_data, 'tolist'):
                    robot_data['actual_joint_torque'] = torque_data.tolist()[:6]
                elif hasattr(torque_data, 'data'):
                    robot_data['actual_joint_torque'] = [float(x) for x in list(torque_data.data)[:6]]
                else:
                    robot_data['actual_joint_torque'] = [float(x) for x in list(torque_data)[:6]]
        except Exception as e:
            self.get_logger().debug(f'joint_torque_callback error: {e}')
    
    def call_start_sort(self):
        """분류 시작 서비스 호출"""
        if not self.cli_start_sort.service_is_ready():
            return False, '분류 노드가 준비되지 않았습니다'
        
        req = Trigger.Request()
        future = self.cli_start_sort.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_stop_sort(self):
        """분류 정지 서비스 호출"""
        if not self.cli_stop_sort.service_is_ready():
            return False, '분류 노드가 준비되지 않았습니다'
        
        req = Trigger.Request()
        future = self.cli_stop_sort.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_pause_sort(self, pause):
        """분류 일시정지 서비스 호출"""
        if not self.cli_pause_sort.service_is_ready():
            return False, '분류 노드가 준비되지 않았습니다'
        
        req = SetBool.Request()
        req.data = pause
        future = self.cli_pause_sort.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_reset_state(self):
        """상태 초기화 서비스 호출"""
        if not self.cli_reset_state.service_is_ready():
            return False, '분류 노드가 준비되지 않았습니다'
        
        req = Trigger.Request()
        future = self.cli_reset_state.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_conveyor_mode(self, enabled):
        """컨베이어 자동 모드 설정 서비스 호출"""
        if not self.cli_conveyor_mode.service_is_ready():
            return False, '분류 노드가 준비되지 않았습니다'
        
        req = SetBool.Request()
        req.data = enabled
        future = self.cli_conveyor_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def set_gripper(self, open_gripper):
        """그리퍼 제어 (Controller Digital Output 1, 2번)
        OnRobot RG2:
        - 열기: DO1=ON(0), DO2=OFF(1)
        - 닫기: DO1=OFF(1), DO2=ON(0)
        """
        if not self.cli_set_digital_output.service_is_ready():
            return False
        
        # DO1 설정
        req1 = SetCtrlBoxDigitalOutput.Request()
        req1.index = 1
        req1.value = 0 if open_gripper else 1
        
        # DO2 설정
        req2 = SetCtrlBoxDigitalOutput.Request()
        req2.index = 2
        req2.value = 1 if open_gripper else 0
        
        self.cli_set_digital_output.call_async(req1)
        self.cli_set_digital_output.call_async(req2)
        
        self.get_logger().info(f'Gripper {"OPEN" if open_gripper else "CLOSE"}')
        return True
    
    def move_home(self, target=1):
        """홈 위치로 이동
        target: 0=기계적 홈, 1=사용자 홈
        """
        if not self.cli_move_home.service_is_ready():
            return False
        
        req = MoveHome.Request()
        req.target = target
        self.cli_move_home.call_async(req)
        
        self.get_logger().info(f'MoveHome target={target}')
        return True
    
    def change_speed(self, speed):
        """작업 속도 변경 (%)"""
        if not self.cli_change_speed.service_is_ready():
            return False
        
        req = ChangeOperationSpeed.Request()
        req.speed = int(speed)
        self.cli_change_speed.call_async(req)
        
        self.get_logger().info(f'ChangeSpeed {speed}%')
        return True
    
    def emergency_stop(self):
        """긴급정지"""
        if not self.cli_move_stop.service_is_ready():
            return False
        
        req = MoveStop.Request()
        req.stop_mode = 3  # DR_HOLD
        self.cli_move_stop.call_async(req)
        
        self.get_logger().info('Emergency Stop')
        return True
    
    def send_conveyor_cmd(self, command):
        """컨베이어 명령 전송"""
        msg = String()
        msg.data = command
        self.conveyor_cmd_pub.publish(msg)
        self.get_logger().info(f'Conveyor command: {command}')
        return True
    
    def move_joint(self, pos, vel=30.0, acc=30.0):
        """조인트 이동 (동기)"""
        if not self.cli_move_joint.service_is_ready():
            return False
        
        req = MoveJoint.Request()
        req.pos = [float(p) for p in pos]
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0  # ABSOLUTE
        req.blend_type = 0
        req.sync_type = 0  # SYNC
        
        future = self.cli_move_joint.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def start_pendulum_test(self, joint_index=0, amplitude=15.0, vel=30.0):
        """진자운동 테스트 시작"""
        if self.pendulum_running:
            add_log('WARN', '진자운동 테스트가 이미 실행 중')
            return False
        
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
        add_log('INFO', f'진자운동 테스트 시작 (J{joint_index+1}, 진폭 ±{amplitude}°, 속도 {vel}°/s)')
        return True
    
    def stop_pendulum_test(self):
        """진자운동 테스트 정지"""
        self.pendulum_running = False
        self.pendulum_paused = False
        self.pendulum_params = None
        add_log('INFO', '진자운동 테스트 정지')
        return True
    
    def pause_pendulum_test(self):
        """진자운동 테스트 일시정지"""
        if self.pendulum_running:
            self.pendulum_running = False
            self.pendulum_paused = True
            add_log('INFO', '진자운동 테스트 일시정지')
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
            # 사용자 홈 위치를 중심으로 사용
            user_home_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
            center_pos = user_home_pos.copy()
            center_value = center_pos[joint_index]
            
            direction = 1  # 1: 양방향, -1: 음방향
            cycle_count = 0
            
            add_log('INFO', f'진자운동 중심 (사용자 홈): J{joint_index+1}={center_value:.1f}°')
            
            while self.pendulum_running:
                # 목표 위치 계산
                target_pos = center_pos.copy()
                target_pos[joint_index] = center_value + (amplitude * direction)
                
                # 이동 명령
                if not self.cli_move_joint.service_is_ready():
                    break
                
                req = MoveJoint.Request()
                req.pos = [float(p) for p in target_pos]
                req.vel = float(vel)
                req.acc = float(vel)
                req.time = 0.0
                req.radius = 0.0
                req.mode = 0  # ABSOLUTE
                req.blend_type = 0
                req.sync_type = 0  # SYNC
                
                future = self.cli_move_joint.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                
                if not self.pendulum_running:
                    break
                
                # 방향 반전
                direction *= -1
                cycle_count += 1
                
                if cycle_count % 2 == 0:
                    add_log('INFO', f'진자운동 {cycle_count//2}회 완료')
                
                time.sleep(0.1)
                
        except Exception as e:
            self.get_logger().error(f'Pendulum test error: {e}')
            add_log('ERROR', f'진자운동 오류: {e}')
        finally:
            self.pendulum_running = False


def ros2_spin_thread(node):
    """ROS2 스핀 스레드"""
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except:
        pass
    finally:
        executor.shutdown()


def emit_robot_data():
    """웹소켓 데이터 전송"""
    global ui_state
    while True:
        if robot_data['connected']:
            # 진자운동 상태 동기화
            if ros_node:
                ui_state['pendulum_running'] = ros_node.pendulum_running
            
            socketio.emit('robot_state', robot_data)
            socketio.emit('sort_status', sort_status)
            socketio.emit('conveyor_status', conveyor_status)
            socketio.emit('logs', logs[:20])
            socketio.emit('ui_state', ui_state)
        time.sleep(0.1)


def firebase_upload_thread():
    """Firebase에 로봇 상태 주기적 업로드 (1초마다)"""
    global firebase_ref, ros_node, ui_state, logs, sort_status
    
    if not firebase_enabled or firebase_ref is None:
        return
    
    print("🔥 Firebase 업로드 스레드 시작")
    
    while True:
        try:
            if robot_data['connected']:
                # 진자운동 상태 동기화
                if ros_node:
                    ui_state['pendulum_running'] = ros_node.pendulum_running
                
                # 최근 로그 20개만 추출
                recent_logs = logs[-20:] if logs else []
                
                # 업로드할 데이터 (필수 정보 + 로그 + 분류 상태)
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
                    'ui_state': ui_state,
                    # 분류 작업 상태 추가
                    'sort_status': {
                        'running': sort_status.get('running', False),
                        'paused': sort_status.get('paused', False),
                        'phase': sort_status.get('phase', 'IDLE'),
                        'cycle_count': sort_status.get('cycle_count', 0),
                        'last_width': sort_status.get('last_width', None),
                        'dsr_ready': sort_status.get('dsr_ready', False),
                    },
                    # 최근 로그 추가
                    'logs': recent_logs
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
                    ros_node.move_home(target=1)
                    print('🔥 Firebase: Move to user home')
                    
            elif cmd == 'move_home_mech':
                if ros_node:
                    ros_node.move_home(target=0)
                    print('🔥 Firebase: Move to mechanical home')
                    
            elif cmd == 'emergency_stop':
                if ros_node:
                    ros_node.emergency_stop()
                    if ros_node.pendulum_running:
                        ros_node.pause_pendulum_test()
                    ui_state['is_stopped'] = True
                    ui_state['pendulum_running'] = False
                    print('🔥 Firebase: Emergency stop')
                        
            elif cmd == 'move_resume':
                if ros_node:
                    if ros_node.pendulum_paused:
                        ros_node.resume_pendulum_test()
                        ui_state['pendulum_running'] = True
                ui_state['is_stopped'] = False
                print('🔥 Firebase: Resume')
                        
            elif cmd == 'pendulum_start':
                if ros_node:
                    joint = event.data.get('joint', 4)
                    amplitude = event.data.get('amplitude', 15)
                    velocity = event.data.get('velocity', 30)
                    ros_node.start_pendulum_test(joint, amplitude, velocity)
                    ui_state['pendulum_running'] = True
                    print(f'🔥 Firebase: Pendulum started J{joint+1}, ±{amplitude}°, {velocity}°/s')
                    
            elif cmd == 'pendulum_stop':
                if ros_node:
                    ros_node.stop_pendulum_test()
                    ui_state['pendulum_running'] = False
                    print('🔥 Firebase: Pendulum stopped')
                        
            elif cmd == 'speed_change':
                speed = event.data.get('value', 50)
                if ros_node:
                    ros_node.change_speed(speed)
            
            # 명령 처리 완료 후 삭제 및 UI 상태 업데이트
            firebase_cmd_ref.update({'command': None, 'value': None, 'processed': time.time()})
            
            # 로컬 웹소켓으로도 상태 전파
            socketio.emit('ui_state', ui_state)
            if cmd in ['pendulum_start', 'pendulum_stop', 'emergency_stop', 'move_resume']:
                running = ui_state.get('pendulum_running', False)
                socketio.emit('pendulum_status', {'running': running})
            
        except Exception as e:
            print(f"Firebase 명령 처리 오류: {e}")
            import traceback
            traceback.print_exc()
    
    print("🔥 Firebase 명령 리스너 시작")
    firebase_cmd_ref.listen(on_command)


def get_kst_time():
    """한국시간 반환"""
    kst = timezone(timedelta(hours=9))
    return datetime.now(kst).strftime('%H:%M:%S')


def add_log(level, message):
    """로그 추가"""
    logs.insert(0, {
        'time': get_kst_time(),
        'level': level,
        'message': message
    })
    if len(logs) > 100:
        logs.pop()


# Flask 라우트
@app.route('/')
def index():
    return render_template('index_full.html')


@app.route('/external')
def external():
    return render_template('index_tabbed.html')


@app.route('/api/status')
def get_status():
    return jsonify(robot_data)


@app.route('/api/sort_status')
def get_sort_status():
    return jsonify(sort_status)


@app.route('/api/conveyor_status')
def get_conveyor_status():
    return jsonify(conveyor_status)


@app.route('/firebase_config')
def get_firebase_config():
    """Firebase 웹 설정 반환 (클라이언트용)"""
    try:
        with open(WEB_CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return jsonify(config)
    except Exception as e:
        print(f"⚠️ Firebase 설정 로드 실패: {e}")
        return jsonify({'error': 'Firebase config not found'}), 404


# SocketIO 이벤트
@socketio.on('connect')
def handle_connect():
    print('✅ Client connected')


@socketio.on('disconnect')
def handle_disconnect():
    print('❌ Client disconnected')


@socketio.on('sort_start')
def handle_sort_start():
    """분류 시작"""
    global ros_node
    if ros_node:
        success, message = ros_node.call_start_sort()
        add_log('INFO' if success else 'ERROR', f'분류 시작: {message}')
        socketio.emit('sort_result', {'success': success, 'message': message})


@socketio.on('sort_stop')
def handle_sort_stop():
    """분류 정지"""
    global ros_node
    if ros_node:
        success, message = ros_node.call_stop_sort()
        add_log('INFO' if success else 'ERROR', f'분류 정지: {message}')
        socketio.emit('sort_result', {'success': success, 'message': message})


@socketio.on('sort_pause')
def handle_sort_pause(data=None):
    """분류 일시정지"""
    global ros_node
    if data is None:
        data = {}
    pause = data.get('pause', True)
    if ros_node:
        success, message = ros_node.call_pause_sort(pause)
        add_log('INFO' if success else 'ERROR', f'분류 {"일시정지" if pause else "재개"}: {message}')
        socketio.emit('sort_result', {'success': success, 'message': message})


@socketio.on('sort_resume')
def handle_sort_resume():
    """분류 재개"""
    global ros_node
    if ros_node:
        success, message = ros_node.call_pause_sort(False)  # pause=False로 재개
        add_log('INFO' if success else 'ERROR', f'분류 재개: {message}')
        socketio.emit('sort_result', {'success': success, 'message': message})


@socketio.on('sort_reset')
def handle_sort_reset():
    """상태 초기화"""
    global ros_node
    if ros_node:
        success, message = ros_node.call_reset_state()
        add_log('INFO' if success else 'ERROR', f'상태 초기화: {message}')
        socketio.emit('sort_result', {'success': success, 'message': message})


@socketio.on('conveyor_mode')
def handle_conveyor_mode(data):
    """컨베이어 자동 모드 설정"""
    global ros_node
    enabled = data.get('enabled', False)
    if ros_node:
        success, message = ros_node.call_conveyor_mode(enabled)
        add_log('INFO' if success else 'ERROR', f'컨베이어 자동 모드 {"활성화" if enabled else "비활성화"}: {message}')
        socketio.emit('conveyor_result', {'success': success, 'message': message, 'enabled': enabled})
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')
        socketio.emit('conveyor_result', {'success': False, 'message': 'ROS 노드 초기화 안됨', 'enabled': False})


@socketio.on('gripper_command')
def handle_gripper(data):
    """그리퍼 제어"""
    global ros_node
    command = data.get('command', 'close')
    print(f'🤖 Gripper command: {command}')
    
    if ros_node:
        open_gripper = (command == 'open')
        success = ros_node.set_gripper(open_gripper)
        if success:
            add_log('INFO', f'그리퍼 {command} 명령 전송')
        else:
            add_log('WARN', f'그리퍼 서비스 준비 안됨')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('move_home')
def handle_move_home(data):
    """홈 이동"""
    global ros_node
    target = data.get('target', 1)
    home_type = "기계적 홈" if target == 0 else "사용자 홈"
    print(f'🏠 Move Home: {home_type}')
    
    if ros_node:
        success = ros_node.move_home(target)
        if success:
            add_log('INFO', f'홈 이동 명령 전송 ({home_type})')
        else:
            add_log('WARN', f'홈 이동 서비스 준비 안됨')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('speed_change')
def handle_speed(data):
    """속도 변경"""
    global ros_node
    speed = data.get('speed', 50)
    print(f'🚀 Speed change: {speed}%')
    
    if ros_node:
        success = ros_node.change_speed(speed)
        if success:
            add_log('INFO', f'작업 속도 변경: {speed}%')
        else:
            add_log('WARN', f'속도 변경 서비스 준비 안됨')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('emergency_stop')
def handle_estop():
    """긴급정지"""
    global ros_node
    print('🛑 EMERGENCY STOP')
    
    if ros_node:
        success = ros_node.emergency_stop()
        if success:
            add_log('ERROR', '🛑 긴급정지 실행')
        else:
            add_log('WARN', f'긴급정지 서비스 준비 안됨')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('move_pause')
def handle_move_pause():
    """일시정지"""
    global ros_node
    print('⏸️ MOVE PAUSE')
    
    if ros_node:
        if hasattr(ros_node, 'cli_move_pause') and ros_node.cli_move_pause.service_is_ready():
            req = MovePause.Request()
            ros_node.cli_move_pause.call_async(req)
            add_log('INFO', '⏸️ 일시정지 실행')
        else:
            add_log('WARN', '일시정지 서비스 준비 안됨')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('move_resume')
def handle_move_resume():
    """재개"""
    global ros_node
    print('▶️ MOVE RESUME')
    
    if ros_node:
        if hasattr(ros_node, 'cli_move_resume') and ros_node.cli_move_resume.service_is_ready():
            req = MoveResume.Request()
            ros_node.cli_move_resume.call_async(req)
            add_log('INFO', '▶️ 재개 실행')
        else:
            add_log('WARN', '재개 서비스 준비 안됨')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('conveyor_resume')
def handle_conveyor_resume():
    """컨베이어 재시작"""
    global ros_node
    print('🚚 Conveyor resume requested')
    
    if ros_node:
        success = ros_node.send_conveyor_cmd('RESUME')
        if success:
            add_log('INFO', '🚚 컨베이어 재시작')
        else:
            add_log('WARN', '컨베이어 명령 전송 실패')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('conveyor_command')
def handle_conveyor(data):
    """컨베이어 제어"""
    global ros_node
    command = data.get('command', '')
    print(f'🚚 Conveyor command: {command}')
    
    if ros_node:
        success = ros_node.send_conveyor_cmd(command)
        if success:
            add_log('INFO', f'컨베이어 명령: {command}')
        else:
            add_log('WARN', '컨베이어 명령 전송 실패')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('pendulum_start')
def handle_pendulum_start(data):
    """진자운동 테스트 시작"""
    global ros_node, ui_state
    joint_index = data.get('joint', 0)
    amplitude = data.get('amplitude', 15.0)
    velocity = data.get('velocity', 30.0)
    
    print(f'🔄 Pendulum test start: J{joint_index+1}, ±{amplitude}°, {velocity}°/s')
    
    if ros_node:
        success = ros_node.start_pendulum_test(joint_index, amplitude, velocity)
        if success:
            ui_state['pendulum_running'] = True
            socketio.emit('pendulum_status', {'running': True, 'joint': joint_index, 'amplitude': amplitude, 'velocity': velocity})
            socketio.emit('ui_state', ui_state)
        else:
            add_log('WARN', '진자운동 테스트가 이미 실행 중')
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('pendulum_stop')
def handle_pendulum_stop():
    """진자운동 테스트 정지"""
    global ros_node, ui_state
    print('⏹️ Pendulum test stop')
    
    if ros_node:
        ros_node.stop_pendulum_test()
        ui_state['pendulum_running'] = False
        socketio.emit('pendulum_status', {'running': False})
        socketio.emit('ui_state', ui_state)
    else:
        add_log('ERROR', 'ROS 노드 초기화 안됨')


@socketio.on('get_ui_state')
def handle_get_ui_state():
    """클라이언트 연결시 현재 UI 상태 전송"""
    global ui_state, ros_node
    if ros_node:
        ui_state['pendulum_running'] = ros_node.pendulum_running
    socketio.emit('ui_state', ui_state)


def main(args=None):
    global ros_node
    
    print("=" * 70)
    print("📦 물류 로봇 통합 시스템 - 웹 서버 노드")
    print("=" * 70)
    
    # ROS2 초기화
    rclpy.init(args=args)
    ros_node = WebServerNode()
    
    # ROS2 스핀 스레드
    ros_thread = threading.Thread(target=ros2_spin_thread, args=(ros_node,), daemon=True)
    ros_thread.start()
    print("✅ ROS2 Node started")
    
    # 웹소켓 데이터 전송 스레드
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
    print("=" * 70)
    
    # Flask 서버 실행 (allow_unsafe_werkzeug 추가)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    
    # 정리
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
