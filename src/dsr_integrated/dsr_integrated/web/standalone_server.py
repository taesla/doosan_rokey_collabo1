#!/usr/bin/env python3
"""
독립 웹 서버 (Standalone Flask Server)

ROS와 독립적으로 실행되어 ROS가 죽어도 웹 서버는 유지됩니다.
Firebase DB를 통해 작업 상태를 저장/복원합니다.

실행 방법:
    python3 -m dsr_integrated.web.standalone_server
    또는
    cd ~/cobot1_ws/src/dsr_integrated/dsr_integrated/web && python3 standalone_server.py
"""

import os
import sys
import json
import time
import threading
from datetime import datetime
from typing import Optional, Dict, Any

# ROS2 경로를 Python path에 추가 (백그라운드 실행 시 환경변수 손실 방지)
ros_paths = [
    '/opt/ros/humble/local/lib/python3.10/dist-packages',
    '/opt/ros/humble/lib/python3.10/site-packages',
    os.path.expanduser('~/cobot1_ws/install/dsr_msgs2/local/lib/python3.10/dist-packages'),
]
for path in ros_paths:
    if os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)

# ROS 워크스페이스의 모든 패키지를 Python path에 추가
install_dir = os.path.expanduser('~/cobot1_ws/install')
if os.path.exists(install_dir):
    for pkg in os.listdir(install_dir):
        pkg_python_path = os.path.join(install_dir, pkg, 'local/lib/python3.10/dist-packages')
        if os.path.exists(pkg_python_path) and pkg_python_path not in sys.path:
            sys.path.insert(0, pkg_python_path)

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit

# Firebase 연동
try:
    import firebase_admin
    from firebase_admin import credentials, db as firebase_db
    FIREBASE_AVAILABLE = True
except ImportError:
    FIREBASE_AVAILABLE = False
    print("⚠️ firebase_admin 모듈 없음 - pip install firebase-admin")

# ROS2 연동 (선택적)
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger, SetBool
    from std_msgs.msg import String, Bool
    from sensor_msgs.msg import JointState
    from dsr_msgs2.srv import (
        GetRobotState, GetCurrentPosx, GetToolForce,
        GetCurrentTool, GetCurrentTcp, MoveHome,
        ChangeOperationSpeed, Stop, SetToolDigitalOutput
    )
    ROS_AVAILABLE = True
except ImportError:
    print("⚠️ rclpy 없음 - ROS 기능 비활성화")


# =========================================
# 경로 설정
# =========================================
LOGISTICS_MONITOR_DIR = os.path.expanduser('~/cobot1_ws/src/logistics_monitor')
TEMPLATE_DIR = os.path.join(LOGISTICS_MONITOR_DIR, 'templates')
CONFIG_DIR = os.path.join(LOGISTICS_MONITOR_DIR, 'config')

# =========================================
# Flask 앱 설정
# =========================================
app = Flask(__name__, template_folder=TEMPLATE_DIR)
app.config['SECRET_KEY'] = 'dlar_secret_key_2024'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# =========================================
# 데이터 저장소
# =========================================
class DataStore:
    """웹 서버 데이터 저장소"""
    
    def __init__(self):
        # 로봇 상태 데이터
        self.robot_data = {
            'joint_states': [0.0] * 6,
            'tcp_position': [0.0] * 6,
            'tool_force': [0.0] * 6,
            'robot_state': 'UNKNOWN',
            'robot_state_code': -1,
        }
        
        # 분류 작업 상태
        self.sort_status = {
            'is_running': False,
            'is_paused': False,
            'current_cycle': 0,
            'total_cycles': 9,
            'current_phase': 'IDLE',  # IDLE, PICK, PLACE
            'box_type': None,
            'small_count': 0,
            'medium_count': 0,
            'long_count': 0,
            'total_sorted': 0,
            'last_updated': None,
        }
        
        # 컨베이어 상태
        self.conveyor_status = {
            'is_running': False,
            'status_code': 0,
            'status_text': '정지',
        }
        
        # 로그
        self.logs = []
        self.max_logs = 100
        
        # ROS 연결 상태
        self.ros_connected = False
        
    def add_log(self, level: str, message: str):
        """로그 추가"""
        log_entry = {
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'level': level,
            'message': message
        }
        self.logs.insert(0, log_entry)
        if len(self.logs) > self.max_logs:
            self.logs = self.logs[:self.max_logs]
        
        # 실시간 전송
        socketio.emit('log', log_entry)
        
    def update_sort_status(self, data: dict):
        """분류 상태 업데이트"""
        self.sort_status.update(data)
        self.sort_status['last_updated'] = datetime.now().isoformat()
        socketio.emit('sort_status', self.sort_status)


data_store = DataStore()

# =========================================
# Firebase 연동
# =========================================
class FirebaseManager:
    """Firebase 연동 관리자"""
    
    def __init__(self):
        self.db = None
        self.enabled = False
        self._init_firebase()
    
    def _init_firebase(self):
        """Firebase 초기화"""
        if not FIREBASE_AVAILABLE:
            print("⚠️ Firebase 비활성화")
            return
        
        try:
            service_account_key = os.path.join(CONFIG_DIR, 'serviceAccountKey.json')
            web_config_path = os.path.join(CONFIG_DIR, 'firebase_web_config.json')
            
            if not os.path.exists(service_account_key):
                print(f"⚠️ Firebase serviceAccountKey.json 없음: {service_account_key}")
                return
            
            if not os.path.exists(web_config_path):
                print(f"⚠️ Firebase web_config.json 없음: {web_config_path}")
                return
            
            with open(web_config_path, 'r') as f:
                web_config = json.load(f)
            database_url = web_config.get('databaseURL')
            
            if not database_url:
                print("⚠️ Firebase databaseURL 없음")
                return
            
            # 이미 초기화되어 있는지 확인
            try:
                firebase_admin.get_app()
                print("✅ Firebase 이미 초기화됨")
            except ValueError:
                cred = credentials.Certificate(service_account_key)
                firebase_admin.initialize_app(cred, {'databaseURL': database_url})
                print("✅ Firebase 초기화 완료")
            
            self.db = firebase_db
            self.enabled = True
            
        except Exception as e:
            print(f"⚠️ Firebase 초기화 실패: {e}")
    
    def save_work_state(self, state: dict):
        """
        작업 상태 저장 (충돌 복구용)
        
        저장 데이터:
            - current_cycle: 현재 사이클 (1-9)
            - total_cycles: 총 사이클 수
            - current_phase: 현재 단계 (PICK/PLACE)
            - box_type: 현재 박스 타입
            - stack_counts: 적재 카운트 {SMALL, MEDIUM, LONG}
            - gripper_state: 그리퍼 상태 (open/closed)
            - last_position: 마지막 위치 [x, y, z, rx, ry, rz]
            - timestamp: 저장 시간
            - interrupted: 중단 여부
        """
        if not self.enabled:
            return False
        
        try:
            state['timestamp'] = datetime.now().isoformat()
            state['interrupted'] = True
            
            ref = self.db.reference('/work_state')
            ref.set(state)
            print(f"💾 작업 상태 저장: 사이클 {state.get('current_cycle')}/{state.get('total_cycles')}, 단계: {state.get('current_phase')}")
            return True
        except Exception as e:
            print(f"⚠️ 작업 상태 저장 실패: {e}")
            return False
    
    def get_work_state(self) -> Optional[dict]:
        """저장된 작업 상태 조회"""
        if not self.enabled:
            return None
        
        try:
            ref = self.db.reference('/work_state')
            state = ref.get()
            if state and state.get('interrupted'):
                print(f"📂 저장된 작업 상태 발견: 사이클 {state.get('current_cycle')}/{state.get('total_cycles')}")
                return state
            return None
        except Exception as e:
            print(f"⚠️ 작업 상태 조회 실패: {e}")
            return None
    
    def clear_work_state(self):
        """작업 상태 삭제 (정상 완료 시)"""
        if not self.enabled:
            return
        
        try:
            ref = self.db.reference('/work_state')
            ref.delete()
            print("🗑️ 작업 상태 삭제 완료")
        except Exception as e:
            print(f"⚠️ 작업 상태 삭제 실패: {e}")
    
    def save_sort_result(self, box_type: str, position: list, force_value: float, success: bool = True):
        """분류 결과 저장"""
        if not self.enabled:
            return
        
        try:
            # 히스토리 저장
            history_ref = self.db.reference('/sorting_history')
            history_ref.push({
                'timestamp': datetime.now().isoformat(),
                'box_type': box_type,
                'position': position,
                'force': force_value,
                'success': success
            })
            
            # 통계 업데이트
            stats_ref = self.db.reference('/statistics')
            current_stats = stats_ref.get() or {}
            
            total = current_stats.get('total_sorted', 0) + 1
            small = current_stats.get('small_count', 0)
            medium = current_stats.get('medium_count', 0)
            long_count = current_stats.get('long_count', 0)
            
            if box_type == 'SMALL':
                small += 1
            elif box_type == 'MEDIUM':
                medium += 1
            elif box_type in ('LONG', 'LARGE'):
                long_count += 1
            
            stats_ref.update({
                'total_sorted': total,
                'small_count': small,
                'medium_count': medium,
                'long_count': long_count,
                'last_updated': datetime.now().isoformat()
            })
            
            print(f"💾 Firebase 저장: {box_type}")
            
        except Exception as e:
            print(f"⚠️ Firebase 저장 실패: {e}")
    
    def get_statistics(self) -> Optional[dict]:
        """통계 조회"""
        if not self.enabled:
            return None
        
        try:
            stats_ref = self.db.reference('/statistics')
            return stats_ref.get()
        except Exception as e:
            print(f"⚠️ 통계 조회 실패: {e}")
            return None


firebase_manager = FirebaseManager()

# =========================================
# ROS 브릿지 (선택적)
# =========================================
class ROSBridge:
    """ROS2 서비스 브릿지 + 토픽 구독"""
    
    def __init__(self):
        self.node = None
        self.connected = False
        self.clients = {}
        self.subscribers = {}
        
        if ROS_AVAILABLE:
            self._init_ros()
    
    def _init_ros(self):
        """ROS2 초기화"""
        try:
            rclpy.init()
            self.node = rclpy.create_node('web_ros_bridge')
            
            # 서비스 클라이언트 생성
            self.clients['start_sort'] = self.node.create_client(Trigger, '/dlar/start_sort')
            self.clients['stop_sort'] = self.node.create_client(Trigger, '/dlar/stop_sort')
            self.clients['pause_sort'] = self.node.create_client(SetBool, '/dlar/pause_sort')
            self.clients['reset_state'] = self.node.create_client(Trigger, '/dlar/reset_state')
            self.clients['conveyor_mode'] = self.node.create_client(SetBool, '/dlar/conveyor_mode')
            
            # DSR 상태 조회 서비스
            self.clients['get_robot_state'] = self.node.create_client(GetRobotState, '/dsr01/system/get_robot_state')
            self.clients['get_current_posx'] = self.node.create_client(GetCurrentPosx, '/dsr01/aux_control/get_current_posx')
            self.clients['get_tool_force'] = self.node.create_client(GetToolForce, '/dsr01/aux_control/get_tool_force')
            self.clients['get_current_tool'] = self.node.create_client(GetCurrentTool, '/dsr01/tool/get_current_tool')
            self.clients['get_current_tcp'] = self.node.create_client(GetCurrentTcp, '/dsr01/tcp/get_current_tcp')
            
            # DSR 제어 서비스
            self.clients['move_home'] = self.node.create_client(MoveHome, '/dsr01/motion/move_home')
            self.clients['stop'] = self.node.create_client(Stop, '/dsr01/stop')
            self.clients['change_operation_speed'] = self.node.create_client(ChangeOperationSpeed, '/dsr01/system/change_operation_speed')
            self.clients['set_digital_output'] = self.node.create_client(SetToolDigitalOutput, '/dsr01/tool/set_tool_digital_output')
            
            # 토픽 구독 - 로봇 상태
            self.subscribers['joint_states'] = self.node.create_subscription(
                JointState, '/dsr01/joint_states',
                self._on_joint_states, 10
            )
            
            # 토픽 구독 - 분류 상태
            self.subscribers['dlar_status'] = self.node.create_subscription(
                String, '/dlar/status',
                self._on_dlar_status, 10
            )
            
            # 토픽 구독 - 분류 실행 상태
            self.subscribers['dlar_running'] = self.node.create_subscription(
                Bool, '/dlar/is_running',
                self._on_dlar_running, 10
            )
            
            # 백그라운드 스핀
            self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self.spin_thread.start()
            
            # 상태 브로드캐스트 타이머
            self.broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
            self.broadcast_thread.start()
            
            self.connected = True
            print("✅ ROS2 브릿지 연결 완료")
            
        except Exception as e:
            print(f"⚠️ ROS2 브릿지 연결 실패: {e}")
            import traceback
            traceback.print_exc()
            self.connected = False
    
    def _on_joint_states(self, msg):
        """조인트 상태 콜백"""
        try:
            data_store.robot_data['joint_states'] = list(msg.position)
        except:
            pass
    
    def _on_dlar_status(self, msg):
        """분류 상태 콜백 - /dlar/status 토픽에서 수신"""
        try:
            status = json.loads(msg.data)
            
            # 분류 작업 상태
            data_store.sort_status['is_running'] = status.get('is_running', False)
            data_store.sort_status['is_paused'] = status.get('is_paused', False)
            data_store.sort_status['current_cycle'] = status.get('cycle_count', 0)
            data_store.sort_status['current_phase'] = status.get('current_phase', 'IDLE')
            data_store.sort_status['box_type'] = status.get('last_classification')
            
            # 통계 데이터
            data_store.sort_status['total_sorted'] = status.get('completed', 0)
            data_store.sort_status['small_count'] = status.get('small', 0)
            data_store.sort_status['medium_count'] = status.get('medium', 0)
            data_store.sort_status['long_count'] = status.get('large', 0)
            
            # 로봇/컨베이어 상태
            data_store.robot_data['dsr_ready'] = status.get('dsr_ready', False)
            data_store.conveyor_status['is_running'] = status.get('conveyor_mode', False)
            data_store.conveyor_status['detected'] = status.get('conveyor_detected', False)
            data_store.conveyor_status['waiting'] = status.get('waiting_for_object', False)
            
            # 컨베이어 상태 텍스트
            if status.get('conveyor_detected'):
                data_store.conveyor_status['status_text'] = '물체 감지됨'
            elif status.get('waiting_for_object'):
                data_store.conveyor_status['status_text'] = '물체 대기 중'
            elif status.get('conveyor_mode'):
                data_store.conveyor_status['status_text'] = '자동 모드'
            else:
                data_store.conveyor_status['status_text'] = '정지'
                
        except Exception as e:
            print(f'⚠️ dlar_status 파싱 에러: {e}')
    
    def _on_dlar_running(self, msg):
        """분류 실행 상태 콜백"""
        data_store.sort_status['is_running'] = msg.data
    
    def _spin_loop(self):
        """ROS2 스핀 루프"""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
    
    def _call_service_quick(self, client_name, req_class, timeout=0.3):
        """빠른 서비스 호출 (브로드캐스트용)"""
        try:
            client = self.clients.get(client_name)
            if not client or not client.service_is_ready():
                return None
            
            req = req_class()
            future = client.call_async(req)
            
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.02)
            
            if future.done():
                return future.result()
        except:
            pass
        return None
    
    def _get_robot_state(self) -> int:
        """로봇 상태 조회 (1=STANDBY, 6=SAFE_STOP 등)"""
        result = self._call_service_quick('get_robot_state', GetRobotState.Request)
        return result.robot_state if result else -1
    
    def _get_tcp_position(self) -> list:
        """TCP 위치 조회"""
        try:
            client = self.clients.get('get_current_posx')
            if not client or not client.service_is_ready():
                return [0.0] * 6
            
            req = GetCurrentPosx.Request()
            req.ref = 0  # DR_BASE
            future = client.call_async(req)
            
            start = time.time()
            while not future.done() and (time.time() - start) < 0.3:
                time.sleep(0.02)
            
            if future.done() and future.result():
                # task_pos_info는 Float64MultiArray[] 타입
                # 첫 번째 요소의 data 필드에서 값을 추출
                pos_info = future.result().task_pos_info
                if pos_info and len(pos_info) > 0:
                    return list(pos_info[0].data)[:6]
        except:
            pass
        return [0.0] * 6
    
    def _get_tool_force(self) -> list:
        """툴 힘 조회"""
        try:
            client = self.clients.get('get_tool_force')
            if not client or not client.service_is_ready():
                return [0.0] * 6
            
            req = GetToolForce.Request()
            req.ref = 0  # DR_BASE
            future = client.call_async(req)
            
            start = time.time()
            while not future.done() and (time.time() - start) < 0.3:
                time.sleep(0.02)
            
            if future.done() and future.result():
                return list(future.result().tool_force)[:6]
        except:
            pass
        return [0.0] * 6
    
    def _get_tool_name(self) -> str:
        """현재 툴 이름 조회"""
        result = self._call_service_quick('get_current_tool', GetCurrentTool.Request)
        if result:
            return result.info  # tool name은 info 필드
        return ''
    
    def _get_tcp_name(self) -> str:
        """현재 TCP 이름 조회"""
        result = self._call_service_quick('get_current_tcp', GetCurrentTcp.Request)
        if result:
            return result.info  # tcp name은 info 필드
        return ''
    
    def _state_code_to_name(self, code: int) -> str:
        """상태 코드를 이름으로 변환"""
        names = {
            -1: 'UNKNOWN',
            0: 'INITIALIZING',
            1: 'STANDBY',
            2: 'MOVING',
            3: 'SAFE_OFF',
            4: 'TEACHING',
            5: 'SAFE_STOP2',
            6: 'SAFE_STOP',
            7: 'EMERGENCY',
            8: 'HOMMING',
            9: 'RECOVERY',
        }
        return names.get(code, f'STATE_{code}')
    
    def _broadcast_loop(self):
        """상태 브로드캐스트 루프 (0.5초마다)"""
        broadcast_count = 0
        while True:
            try:
                broadcast_count += 1
                if broadcast_count % 10 == 1:  # 5초마다 로그
                    print(f"📡 브로드캐스트 #{broadcast_count}")
                
                # 로봇 상태 조회 (서비스 호출)
                robot_state_code = self._get_robot_state()
                tcp_position = self._get_tcp_position()
                tool_force = self._get_tool_force()
                tool_name = self._get_tool_name()
                tcp_name = self._get_tcp_name()
                
                # 데이터 저장
                data_store.robot_data['robot_state_code'] = robot_state_code
                data_store.robot_data['robot_state'] = robot_state_code  # 웹은 숫자 기대
                data_store.robot_data['tcp_position'] = tcp_position
                data_store.robot_data['tool_force'] = tool_force
                
                # 조인트 상태 가져오기 (6개 보장)
                joint_states = data_store.robot_data.get('joint_states', [])
                if len(joint_states) < 6:
                    joint_states = [0.0] * 6
                
                # robot_state 이벤트 (웹 UI가 기대하는 형식)
                socketio.emit('robot_state', {
                    # 개요 탭용
                    'robot_mode': 1,  # AUTONOMOUS
                    'robot_state': robot_state_code if robot_state_code >= 0 else 0,
                    'speed_rate': 100,
                    'current_tool_name': tool_name or 'Tool0',
                    'current_tcp_name': tcp_name or 'TCP0',
                    
                    # 모니터링 탭용 - 조인트
                    'actual_joint_position': list(joint_states[:6]),
                    'actual_joint_velocity': [0.0] * 6,
                    'actual_joint_torque': [0.0] * 6,
                    'joint_temperature': [25.0] * 6,
                    
                    # 모니터링 탭용 - TCP
                    'actual_tcp_position': tcp_position,
                    
                    # 모니터링 탭용 - 힘
                    'external_tcp_force': tool_force,
                    
                    # 기타
                    'dsr_ready': data_store.robot_data.get('dsr_ready', False),
                })
                
                # sort_status 이벤트 (웹 UI가 기대하는 필드명으로 변환)
                socketio.emit('sort_status', {
                    'running': data_store.sort_status.get('is_running', False),
                    'paused': data_store.sort_status.get('is_paused', False),
                    'phase': data_store.sort_status.get('current_phase', 'IDLE'),
                    'cycle_count': data_store.sort_status.get('current_cycle', 0),
                    'dsr_ready': data_store.robot_data.get('dsr_ready', False),
                    'last_width': None,
                    'box_type': data_store.sort_status.get('box_type'),
                })
                
                # conveyor_status 이벤트 (웹 UI 형식)
                socketio.emit('conveyor_status', {
                    'connected': data_store.conveyor_status.get('is_running', False) or data_store.conveyor_status.get('detected', False),
                    'status': data_store.conveyor_status.get('status_text', 'IDLE'),
                    'is_running': data_store.conveyor_status.get('is_running', False),
                    'detected': data_store.conveyor_status.get('detected', False),
                })
                
                # logistics_status 이벤트
                socketio.emit('logistics_status', {
                    'stack_count': {
                        'SMALL': data_store.sort_status.get('small_count', 0),
                        'MEDIUM': data_store.sort_status.get('medium_count', 0),
                        'LONG': data_store.sort_status.get('long_count', 0),
                    },
                    'total_count': data_store.sort_status.get('total_sorted', 0),
                    'z_touch': 0,
                    'pick_ok': False,
                    'placed_boxes': [],
                })
                
            except Exception as e:
                print(f"⚠️ 브로드캐스트 에러: {e}")
                import traceback
                traceback.print_exc()
            
            time.sleep(0.5)
    
    def call_service(self, service_name: str, request=None) -> tuple:
        """서비스 호출"""
        if not self.connected:
            return False, "ROS 연결 안됨"
        
        client = self.clients.get(service_name)
        if not client:
            return False, f"서비스 없음: {service_name}"
        
        if not client.wait_for_service(timeout_sec=2.0):
            return False, "서비스 준비 안됨"
        
        if request is None:
            request = Trigger.Request()
        
        future = client.call_async(request)
        
        # 타임아웃 대기
        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            time.sleep(0.1)
        
        if future.done() and future.result():
            result = future.result()
            return result.success, getattr(result, 'message', 'OK')
        
        return False, "서비스 호출 실패"
    
    def is_connected(self) -> bool:
        """ROS 연결 상태"""
        if not self.connected:
            return False
        
        # 서비스 가용성 체크
        try:
            return self.clients['start_sort'].wait_for_service(timeout_sec=0.5)
        except:
            return False


ros_bridge = ROSBridge()

# =========================================
# Flask 라우트
# =========================================
@app.route('/')
def index():
    """메인 페이지"""
    return render_template('index.html')

@app.route('/api/status')
def api_status():
    """상태 API"""
    return jsonify({
        'robot': data_store.robot_data,
        'sort': data_store.sort_status,
        'conveyor': data_store.conveyor_status,
        'ros_connected': ros_bridge.is_connected(),
        'firebase_enabled': firebase_manager.enabled,
    })

@app.route('/api/logistics_status')
def api_logistics_status():
    """물류 상태 API (기존 웹 호환)"""
    return jsonify({
        'stack_count': {
            'SMALL': data_store.sort_status.get('small_count', 0),
            'MEDIUM': data_store.sort_status.get('medium_count', 0),
            'LONG': data_store.sort_status.get('long_count', 0),
        },
        'total_count': data_store.sort_status.get('total_sorted', 0),
        'z_touch': 0,
        'pick_ok': False,
        'placed_boxes': [],
    })

@app.route('/firebase_config')
def firebase_config():
    """Firebase 설정 API (기존 웹 호환)"""
    try:
        web_config_path = os.path.join(CONFIG_DIR, 'firebase_web_config.json')
        if os.path.exists(web_config_path):
            with open(web_config_path, 'r') as f:
                return jsonify(json.load(f))
    except Exception as e:
        print(f"Firebase 설정 로드 실패: {e}")
    return jsonify({})

@app.route('/api/statistics')
def api_statistics():
    """통계 API"""
    stats = firebase_manager.get_statistics()
    return jsonify(stats or {})

@app.route('/api/work_state')
def api_work_state():
    """저장된 작업 상태 API"""
    state = firebase_manager.get_work_state()
    return jsonify(state or {})

@app.route('/api/logs')
def api_logs():
    """로그 API"""
    return jsonify(data_store.logs)

# =========================================
# SocketIO 이벤트
# =========================================
@socketio.on('connect')
def handle_connect():
    """클라이언트 연결"""
    print(f"🔗 클라이언트 연결")
    emit('connection_status', {
        'connected': True,
        'ros_connected': ros_bridge.is_connected(),
        'firebase_enabled': firebase_manager.enabled,
    })

@socketio.on('disconnect')
def handle_disconnect():
    """클라이언트 연결 해제"""
    print(f"🔌 클라이언트 연결 해제")

@socketio.on('sort_start')
def handle_sort_start(data=None):
    """분류 시작"""
    data_store.add_log('INFO', '분류 시작 요청')
    
    # 저장된 작업 상태 확인
    saved_state = firebase_manager.get_work_state()
    resume = data.get('resume', False) if data else False
    
    if saved_state and not resume:
        # 이전 작업 있음 - 확인 필요
        emit('confirm_resume', {
            'has_saved_state': True,
            'saved_state': saved_state,
            'message': f"이전 작업이 중단되었습니다. (사이클 {saved_state.get('current_cycle')}/{saved_state.get('total_cycles')})\n이어서 진행하시겠습니까?"
        })
        return
    
    # ROS 서비스 호출
    success, message = ros_bridge.call_service('start_sort')
    
    if success:
        data_store.add_log('INFO', '분류 작업 시작됨')
        data_store.update_sort_status({'is_running': True, 'is_paused': False})
    else:
        data_store.add_log('ERROR', f'분류 시작 실패: {message}')
    
    emit('sort_result', {'success': success, 'message': message})

@socketio.on('sort_stop')
def handle_sort_stop():
    """분류 정지"""
    data_store.add_log('INFO', '분류 정지 요청')
    
    success, message = ros_bridge.call_service('stop_sort')
    
    if success:
        data_store.add_log('INFO', '분류 작업 정지됨')
        data_store.update_sort_status({'is_running': False, 'is_paused': False})
    else:
        data_store.add_log('ERROR', f'분류 정지 실패: {message}')
    
    emit('sort_result', {'success': success, 'message': message})

@socketio.on('sort_pause')
def handle_sort_pause(data):
    """분류 일시정지/재개"""
    pause = data.get('pause', True)
    action = '일시정지' if pause else '재개'
    data_store.add_log('INFO', f'분류 {action} 요청')
    
    request = SetBool.Request()
    request.data = pause
    
    # ROS 서비스 호출
    if ros_bridge.connected:
        client = ros_bridge.clients.get('pause_sort')
        if client and client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(request)
            start = time.time()
            while not future.done() and (time.time() - start) < 5.0:
                time.sleep(0.1)
            
            if future.done() and future.result():
                success = future.result().success
                message = future.result().message
            else:
                success, message = False, '서비스 호출 실패'
        else:
            success, message = False, '서비스 준비 안됨'
    else:
        success, message = False, 'ROS 연결 안됨'
    
    if success:
        data_store.add_log('INFO', f'분류 {action} 완료')
        data_store.update_sort_status({'is_paused': pause})
    else:
        data_store.add_log('ERROR', f'분류 {action} 실패: {message}')
    
    emit('sort_result', {'success': success, 'message': message})

@socketio.on('clear_work_state')
def handle_clear_work_state():
    """저장된 작업 상태 삭제"""
    firebase_manager.clear_work_state()
    data_store.add_log('INFO', '저장된 작업 상태 삭제됨')
    emit('work_state_cleared', {'success': True})

@socketio.on('collision_recovery')
def handle_collision_recovery():
    """충돌 복구 안내"""
    data_store.add_log('INFO', '복구는 별도 터미널의 recovery_node가 자동으로 수행합니다.')
    emit('recovery_result', {
        'success': True,
        'message': '복구는 별도 터미널의 recovery_node가 자동으로 수행합니다. 터미널을 확인하세요.'
    })

@socketio.on('sort_reset')
def handle_sort_reset():
    """상태 초기화"""
    data_store.add_log('INFO', '상태 초기화 요청')
    success, message = ros_bridge.call_service('reset_state')
    if success:
        data_store.add_log('INFO', '상태 초기화 완료')
    else:
        data_store.add_log('ERROR', f'상태 초기화 실패: {message}')
    emit('sort_result', {'success': success, 'message': message})

@socketio.on('sort_resume')
def handle_sort_resume():
    """분류 재개"""
    data_store.add_log('INFO', '분류 재개 요청')
    
    if ROS_AVAILABLE and ros_bridge.connected:
        request = SetBool.Request()
        request.data = False  # pause=False = 재개
        
        client = ros_bridge.clients.get('pause_sort')
        if client and client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(request)
            start = time.time()
            while not future.done() and (time.time() - start) < 5.0:
                time.sleep(0.1)
            
            if future.done() and future.result():
                success = future.result().success
                message = future.result().message
            else:
                success, message = False, '서비스 호출 실패'
        else:
            success, message = False, '서비스 준비 안됨'
    else:
        success, message = False, 'ROS 연결 안됨'
    
    if success:
        data_store.add_log('INFO', '분류 재개 완료')
        data_store.update_sort_status({'is_paused': False})
    else:
        data_store.add_log('ERROR', f'분류 재개 실패: {message}')
    
    emit('sort_result', {'success': success, 'message': message})

@socketio.on('conveyor_mode')
def handle_conveyor_mode(data):
    """컨베이어 자동 모드 설정"""
    enabled = data.get('enabled', False)
    data_store.add_log('INFO', f'컨베이어 모드 {"활성화" if enabled else "비활성화"} 요청')
    
    if ROS_AVAILABLE and ros_bridge.connected:
        # /dlar/conveyor_mode 서비스 호출
        try:
            from std_srvs.srv import SetBool
            if 'conveyor_mode' not in ros_bridge.clients:
                ros_bridge.clients['conveyor_mode'] = ros_bridge.node.create_client(
                    SetBool, '/dlar/conveyor_mode')
            
            client = ros_bridge.clients['conveyor_mode']
            if client.wait_for_service(timeout_sec=2.0):
                request = SetBool.Request()
                request.data = enabled
                future = client.call_async(request)
                
                start = time.time()
                while not future.done() and (time.time() - start) < 5.0:
                    time.sleep(0.1)
                
                if future.done() and future.result():
                    success = future.result().success
                    message = future.result().message
                else:
                    success, message = False, '서비스 호출 실패'
            else:
                success, message = False, '서비스 준비 안됨'
        except Exception as e:
            success, message = False, str(e)
    else:
        success, message = False, 'ROS 연결 안됨'
    
    emit('conveyor_result', {'success': success, 'message': message, 'enabled': enabled})

@socketio.on('conveyor_command')
def handle_conveyor_command(data):
    """컨베이어 명령"""
    command = data.get('command', 'STOP')
    data_store.add_log('INFO', f'컨베이어 명령: {command}')
    # 컨베이어 명령은 시리얼 노드를 통해 처리됨
    emit('conveyor_result', {'success': True, 'message': f'명령 전송: {command}'})

@socketio.on('emergency_stop')
def handle_emergency_stop():
    """긴급정지"""
    data_store.add_log('ERROR', '🛑 긴급정지 요청')
    
    success = False
    
    # DSR stop 서비스 호출
    if ROS_AVAILABLE and ros_bridge.connected:
        client = ros_bridge.clients.get('stop')
        if client and client.wait_for_service(timeout_sec=2.0):
            try:
                req = Stop.Request()
                req.stop_mode = 0  # STOP_TYPE_QUICK
                
                future = client.call_async(req)
                start = time.time()
                while not future.done() and (time.time() - start) < 3.0:
                    time.sleep(0.1)
                
                if future.done() and future.result():
                    success = future.result().success
            except:
                pass
        
        # sort_node도 정지
        stop_client = ros_bridge.clients.get('stop_sort')
        if stop_client:
            try:
                stop_client.call_async(Trigger.Request())
            except:
                pass
    
    emit('safety_state', {
        'state': 'emergency_stop',
        'is_safe': False
    })
    emit('estop_result', {'success': success, 'message': '긴급정지 실행'})

@socketio.on('emergency_stop_release')
def handle_emergency_stop_release():
    """긴급정지 해제"""
    data_store.add_log('INFO', '▶️ 긴급정지 해제 요청')
    
    success = False
    
    # pause_sort(false) 서비스 호출 - 작업 재개
    if ROS_AVAILABLE and ros_bridge.connected:
        request = SetBool.Request()
        request.data = False
        
        client = ros_bridge.clients.get('pause_sort')
        if client and client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(request)
            start = time.time()
            while not future.done() and (time.time() - start) < 3.0:
                time.sleep(0.1)
            
            if future.done() and future.result():
                success = True
    
    emit('safety_state', {
        'state': 'normal',
        'is_safe': True
    })
    emit('estop_release_result', {'success': success, 'message': '긴급정지 해제'})

@socketio.on('gripper_command')
def handle_gripper_command(data):
    """그리퍼 제어"""
    command = data.get('command', 'close')
    data_store.add_log('INFO', f'그리퍼 명령: {command}')
    
    if not ROS_AVAILABLE or not ros_bridge.connected:
        emit('gripper_result', {'success': False, 'message': 'ROS 연결 안됨'})
        return
    
    try:
        client = ros_bridge.clients.get('set_digital_output')
        if not client or not client.wait_for_service(timeout_sec=2.0):
            emit('gripper_result', {'success': False, 'message': '서비스 준비 안됨'})
            return
        
        req = SetToolDigitalOutput.Request()
        req.index = 1  # DO 번호
        req.val = 1 if command == 'open' else 0  # 1=열기, 0=닫기
        
        future = client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.1)
        
        if future.done() and future.result():
            result = future.result()
            emit('gripper_result', {'success': result.success, 'message': f'그리퍼 {command} 완료'})
        else:
            emit('gripper_result', {'success': False, 'message': '타임아웃'})
    except Exception as e:
        emit('gripper_result', {'success': False, 'message': f'에러: {e}'})

@socketio.on('move_home')
def handle_move_home(data):
    """홈 이동"""
    home_type = data.get('type', 'user')  # 'user' 또는 'mechanical'
    data_store.add_log('INFO', f'홈 이동 요청: {home_type}')
    
    if not ROS_AVAILABLE or not ros_bridge.connected:
        emit('move_result', {'success': False, 'message': 'ROS 연결 안됨'})
        return
    
    try:
        client = ros_bridge.clients.get('move_home')
        if not client or not client.wait_for_service(timeout_sec=2.0):
            emit('move_result', {'success': False, 'message': '서비스 준비 안됨'})
            return
        
        req = MoveHome.Request()
        req.target = 0 if home_type == 'user' else 1  # 0=user home, 1=mechanical home
        req.vel = 60.0
        req.acc = 30.0
        
        future = client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            time.sleep(0.1)
        
        if future.done() and future.result():
            result = future.result()
            emit('move_result', {'success': result.success, 'message': f'{home_type} 홈 이동 완료'})
        else:
            emit('move_result', {'success': False, 'message': '타임아웃'})
    except Exception as e:
        emit('move_result', {'success': False, 'message': f'에러: {e}'})

@socketio.on('speed_change')
def handle_speed_change(data):
    """속도 변경"""
    speed = data.get('speed', 50)
    data_store.add_log('INFO', f'속도 변경: {speed}%')
    
    if not ROS_AVAILABLE or not ros_bridge.connected:
        emit('speed_result', {'success': False, 'message': 'ROS 연결 안됨'})
        return
    
    try:
        client = ros_bridge.clients.get('change_operation_speed')
        if not client or not client.wait_for_service(timeout_sec=2.0):
            emit('speed_result', {'success': False, 'speed': speed, 'message': '서비스 준비 안됨'})
            return
        
        req = ChangeOperationSpeed.Request()
        req.speed = float(speed)
        
        future = client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.1)
        
        if future.done() and future.result():
            result = future.result()
            emit('speed_result', {'success': result.success, 'speed': speed})
        else:
            emit('speed_result', {'success': False, 'speed': speed, 'message': '타임아웃'})
    except Exception as e:
        emit('speed_result', {'success': False, 'speed': speed, 'message': f'에러: {e}'})

@socketio.on('pendulum_start')
def handle_pendulum_start(data):
    """진자 운동 시작"""
    amplitude = data.get('amplitude', 10)
    cycle = data.get('cycle', 10)
    data_store.add_log('INFO', f'진자 운동 시작: 진폭={amplitude}, 주기={cycle}')
    emit('pendulum_result', {'success': True, 'message': '진자 운동 시작'})

@socketio.on('pendulum_stop')
def handle_pendulum_stop():
    """진자 운동 정지"""
    data_store.add_log('INFO', '진자 운동 정지')
    emit('pendulum_result', {'success': True, 'message': '진자 운동 정지'})

@socketio.on('one_take_start')
def handle_one_take_start():
    """원 테이크 시작"""
    data_store.add_log('INFO', '원 테이크 시작')
    emit('one_take_result', {'success': True, 'message': '원 테이크 시작'})

@socketio.on('one_take_stop')
def handle_one_take_stop():
    """원 테이크 정지"""
    data_store.add_log('INFO', '원 테이크 정지')
    emit('one_take_result', {'success': True, 'message': '원 테이크 정지'})

@socketio.on('logistics_reset')
def handle_logistics_reset():
    """물류 데이터 초기화"""
    data_store.sort_status.update({
        'small_count': 0,
        'medium_count': 0,
        'long_count': 0,
        'total_sorted': 0,
        'current_cycle': 0,
    })
    data_store.add_log('INFO', '물류 데이터 초기화')
    emit('logistics_status', {
        'stack_count': {'SMALL': 0, 'MEDIUM': 0, 'LONG': 0},
        'total_count': 0,
        'z_touch': 0,
        'pick_ok': False,
        'placed_boxes': [],
    })

# =========================================
# 메인 실행
# =========================================
def main():
    """메인 함수"""
    print("\n" + "=" * 60)
    print("🌐 독립 웹 서버 시작")
    print("=" * 60)
    print(f"  • 템플릿 경로: {TEMPLATE_DIR}")
    print(f"  • Firebase: {'활성화' if firebase_manager.enabled else '비활성화'}")
    print(f"  • ROS 브릿지: {'연결됨' if ros_bridge.connected else '연결 안됨'}")
    print("=" * 60)
    print("  ★ 이 서버는 ROS가 죽어도 유지됩니다!")
    print("  ★ 작업 상태는 Firebase에 저장됩니다.")
    print("=" * 60)
    print("\n🔗 http://localhost:5000 에서 접속 가능\n")
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
