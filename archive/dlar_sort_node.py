#!/usr/bin/env python3
"""
물류 분류 로봇 노드 (DLAR Sort Node)
- ROS2 서비스 클라이언트를 통해 DSR 로봇 제어
- 컨베이어 연동 자동 분류
- ROS2 서비스로 분류 작업 시작/정지 제어
- 작업 상태를 토픽으로 발행
- Compliance Control + Force 센서 기반 높이 측정
"""

import os
import json
import time
import threading
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Bool, Int32

# DSR 메시지/서비스 타입
from dsr_msgs2.srv import (
    MoveLine,
    MoveJoint,
    MoveStop,
    GetCurrentPosx,
    GetToolForce,
    SetCtrlBoxDigitalOutput,
    GetCtrlBoxDigitalInput,
    TaskComplianceCtrl,
    ReleaseComplianceCtrl,
    SetDesiredForce,
    ReleaseForce,
)

# =========================================
# 로봇 ID/모델 설정
# =========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# =========================================
# 전역 상수 (원본 알고리즘과 동일)
# =========================================
HOME_C_VAL   = [367.53,   4.02, 203.18, 164.67, -179.96, 164.99]

# ===== 픽업 위치 =====
PICK_POS_VAL = [401.99, -235.44, 153.98, 53.98, 179.23, 56.08]

# 팔레트 배치 위치
LARGE_POS_VAL  = [589.98, 180.15, 143.57, 161.18, -179.97, 161.50]
SMALL_POS_VAL  = [208.56, 180.09, 213.74, 133.74, -179.98, 134.06]  # Z: 133.74 → 213.74 (+80mm)
MEDIUM_POS_VAL = [399.27, 180.15, 143.57, 161.18, -179.97, 161.50]

# Force 센서 설정
FORCE_THRESH = 30.0  # 접촉 감지 임계값
FORCE_PUSH   = 50.0  # Compliance 모드에서 인가할 힘
MAX_DOWN     = 120.0

# 픽업/배치 오프셋
UP_OFFSET   = 30.0
GRIP_OFFSET = 20.0
PICK_EXTRA_DOWN  = 30.0
PLACE_EXTRA_DOWN = 50.0
FINAL_PUSH       = 15.0
PLACE_REDUCE     = 30.0

# ===== 컨베이어 높이 보정 (7cm = 70mm) =====
CONVEYOR_HEIGHT_OFFSET = 70.0

# 안전 Z 한계
SAFE_Z_LIMIT = 103.0  # pick 시 안전 Z 한계
SAFE_Z_PLACE = 12.5   # place 시 안전 Z 한계

# 속도/가속도
V_MOVE       = 200.0
A_MOVE       = 400.0
V_PICK_DOWN  = 60.0
A_PICK_DOWN  = 200.0

# 작업 단계
PHASE_PICK  = 0
PHASE_PLACE = 1

STATE_FILE = os.path.expanduser("~/.dlar_sort_state.json")

# 컨베이어 상태 코드
CONVEYOR_DETECT = 1
CONVEYOR_WAITING = 2
CONVEYOR_RUNNING = 3

# DSR 상수
DR_BASE = 0
DR_TOOL = 1
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1
DR_FC_MOD_ABS = 0


class DlarSortNode(Node):
    """물류 분류 로봇 ROS2 노드"""
    
    def __init__(self):
        super().__init__('dlar_sort_node')
        self.get_logger().info('DLAR Sort Node 시작')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 상태 변수
        self.is_running = False
        self.is_paused = False
        self.stop_requested = False
        self.current_phase = PHASE_PICK
        self.z_touch = HOME_C_VAL[2]
        self.cycle_count = 0
        self.last_width_class = None
        
        # 컨베이어 연동 상태
        self.conveyor_mode = False
        self.conveyor_detected = False
        self.waiting_for_object = False
        self.auto_start_on_detection = False
        
        # 통계
        self.stats = {
            'completed': 0,
            'errors': 0,
            'small': 0,
            'medium': 0,
            'large': 0
        }
        
        # Firebase 초기화
        self.firebase_enabled = False
        self.firebase_db = None
        self.init_firebase()
        
        # DSR 서비스 클라이언트 초기화
        self.dsr_ready = False
        self.init_dsr_clients()
        
        # 서비스 생성
        self.srv_start = self.create_service(
            Trigger, '/dlar/start_sort', 
            self.start_sort_callback,
            callback_group=self.callback_group
        )
        self.srv_stop = self.create_service(
            Trigger, '/dlar/stop_sort', 
            self.stop_sort_callback,
            callback_group=self.callback_group
        )
        self.srv_pause = self.create_service(
            SetBool, '/dlar/pause_sort', 
            self.pause_sort_callback,
            callback_group=self.callback_group
        )
        self.srv_reset = self.create_service(
            Trigger, '/dlar/reset_state', 
            self.reset_state_callback,
            callback_group=self.callback_group
        )
        
        # 상태 발행 토픽
        self.pub_status = self.create_publisher(String, '/dlar/status', 10)
        self.pub_running = self.create_publisher(Bool, '/dlar/is_running', 10)
        
        # 컨베이어 연동 토픽
        self.sub_conveyor = self.create_subscription(
            Int32,
            'conveyor/status_code',
            self.conveyor_status_callback,
            10,
            callback_group=self.callback_group
        )
        self.pub_conveyor_cmd = self.create_publisher(String, 'conveyor/cmd', 10)
        
        # 컨베이어 모드 설정 서비스
        self.srv_conveyor_mode = self.create_service(
            SetBool, '/dlar/conveyor_mode',
            self.conveyor_mode_callback,
            callback_group=self.callback_group
        )
        
        # 상태 발행 타이머
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        # DSR 연결 확인 타이머
        self.check_dsr_timer = self.create_timer(2.0, self.check_dsr_connection)
        
        self.get_logger().info('DLAR Sort Node 준비 완료')
        self.get_logger().info('  서비스: /dlar/start_sort, /dlar/stop_sort, /dlar/pause_sort, /dlar/reset_state')
        self.get_logger().info('  서비스: /dlar/conveyor_mode (컨베이어 자동 모드)')
        self.get_logger().info('  구독: conveyor/status_code')
        self.get_logger().info('  발행: conveyor/cmd')
    
    def init_firebase(self):
        """Firebase 초기화 (선택적)"""
        try:
            import firebase_admin
            from firebase_admin import credentials, db
            
            config_dir = os.path.expanduser('~/cobot1_ws/src/logistics_monitor/config')
            service_account_key = os.path.join(config_dir, 'serviceAccountKey.json')
            web_config_path = os.path.join(config_dir, 'firebase_web_config.json')
            
            if os.path.exists(service_account_key) and os.path.exists(web_config_path):
                with open(web_config_path, 'r') as f:
                    web_config = json.load(f)
                database_url = web_config.get('databaseURL')
                
                try:
                    firebase_admin.get_app()
                    self.get_logger().info('Firebase 이미 초기화됨')
                except ValueError:
                    cred = credentials.Certificate(service_account_key)
                    firebase_admin.initialize_app(cred, {'databaseURL': database_url})
                    self.get_logger().info('✅ Firebase 초기화 완료')
                
                self.firebase_db = db
                self.firebase_enabled = True
            else:
                self.get_logger().warn('⚠️ Firebase 설정 파일 없음')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Firebase 초기화 실패: {e}')
    
    def save_to_firebase(self, box_type, position, force_value):
        """Firebase에 분류 결과 저장"""
        if not self.firebase_enabled or not self.firebase_db:
            return
        
        try:
            history_ref = self.firebase_db.reference('/sorting_history')
            history_ref.push({
                'timestamp': datetime.now().isoformat(),
                'box_type': box_type,
                'position': position,
                'force': force_value,
                'success': True
            })
            
            stats_ref = self.firebase_db.reference('/statistics')
            current_stats = stats_ref.get() or {}
            
            total = current_stats.get('total_sorted', 0) + 1
            small = current_stats.get('small_count', 0)
            medium = current_stats.get('medium_count', 0)
            long = current_stats.get('long_count', 0)
            
            if box_type == 'SMALL':
                small += 1
            elif box_type == 'MEDIUM':
                medium += 1
            elif box_type == 'LONG':
                long += 1
            
            stats_ref.update({
                'total_sorted': total,
                'small_count': small,
                'medium_count': medium,
                'long_count': long,
                'last_updated': datetime.now().isoformat()
            })
            
            self.get_logger().info(f'💾 Firebase 저장 완료: {box_type}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Firebase 저장 실패: {e}')
    
    def init_dsr_clients(self):
        """DSR ROS2 서비스 클라이언트 초기화"""
        prefix = f'/{ROBOT_ID}'
        
        # 모션 서비스 클라이언트
        self.cli_move_line = self.create_client(
            MoveLine, f'{prefix}/motion/move_line',
            callback_group=self.callback_group
        )
        self.cli_move_joint = self.create_client(
            MoveJoint, f'{prefix}/motion/move_joint',
            callback_group=self.callback_group
        )
        self.cli_move_stop = self.create_client(
            MoveStop, f'{prefix}/motion/move_stop',
            callback_group=self.callback_group
        )
        
        # 보조 제어 서비스 클라이언트
        self.cli_get_current_posx = self.create_client(
            GetCurrentPosx, f'{prefix}/aux_control/get_current_posx',
            callback_group=self.callback_group
        )
        self.cli_get_tool_force = self.create_client(
            GetToolForce, f'{prefix}/aux_control/get_tool_force',
            callback_group=self.callback_group
        )
        
        # IO 서비스 클라이언트
        self.cli_set_digital_output = self.create_client(
            SetCtrlBoxDigitalOutput, f'{prefix}/io/set_ctrl_box_digital_output',
            callback_group=self.callback_group
        )
        self.cli_get_digital_input = self.create_client(
            GetCtrlBoxDigitalInput, f'{prefix}/io/get_ctrl_box_digital_input',
            callback_group=self.callback_group
        )
        
        # Force 제어 서비스 클라이언트
        self.cli_task_compliance = self.create_client(
            TaskComplianceCtrl, f'{prefix}/force/task_compliance_ctrl',
            callback_group=self.callback_group
        )
        self.cli_release_compliance = self.create_client(
            ReleaseComplianceCtrl, f'{prefix}/force/release_compliance_ctrl',
            callback_group=self.callback_group
        )
        self.cli_set_desired_force = self.create_client(
            SetDesiredForce, f'{prefix}/force/set_desired_force',
            callback_group=self.callback_group
        )
        self.cli_release_force = self.create_client(
            ReleaseForce, f'{prefix}/force/release_force',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('DSR 서비스 클라이언트 생성 완료')
    
    def check_dsr_connection(self):
        """DSR 연결 상태 확인"""
        if self.cli_move_line.service_is_ready() and self.cli_get_current_posx.service_is_ready():
            if not self.dsr_ready:
                self.dsr_ready = True
                self.get_logger().info('✅ DSR 로봇 연결됨')
        else:
            if self.dsr_ready:
                self.dsr_ready = False
                self.get_logger().warn('⚠️ DSR 로봇 연결 끊김')
    
    # =========================================
    # DSR 래퍼 함수들
    # =========================================
    def movel(self, pos, vel=None, acc=None, time_val=0.0, radius=0.0, ref=DR_BASE, mode=DR_MV_MOD_ABS):
        """MoveLine 서비스 호출 래퍼"""
        if not self.cli_move_line.service_is_ready():
            self.get_logger().error('move_line 서비스 준비 안됨')
            return False
        
        req = MoveLine.Request()
        req.pos = np.array(pos, dtype=np.float64)
        
        if vel is not None:
            req.vel = np.array([vel, vel], dtype=np.float64)
        else:
            req.vel = np.array([V_MOVE, V_MOVE], dtype=np.float64)
        
        if acc is not None:
            req.acc = np.array([acc, acc], dtype=np.float64)
        else:
            req.acc = np.array([A_MOVE, A_MOVE], dtype=np.float64)
        
        req.time = time_val
        req.radius = radius
        req.ref = ref
        req.mode = mode
        req.blend_type = 0
        req.sync_type = 0
        
        future = self.cli_move_line.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def get_current_posx(self, ref=DR_BASE):
        """GetCurrentPosx 서비스 호출 래퍼"""
        if not self.cli_get_current_posx.service_is_ready():
            self.get_logger().error('get_current_posx 서비스 준비 안됨')
            return None
        
        req = GetCurrentPosx.Request()
        req.ref = ref
        
        future = self.cli_get_current_posx.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                pos_data = result.task_pos_info
                if isinstance(pos_data, list) and len(pos_data) > 0:
                    first_item = pos_data[0]
                    if hasattr(first_item, 'data'):
                        return list(first_item.data)[:6]
                elif hasattr(pos_data, 'data'):
                    return list(pos_data.data)[:6]
        return None
    
    def get_tool_force(self, ref=DR_TOOL):
        """GetToolForce 서비스 호출 래퍼"""
        if not self.cli_get_tool_force.service_is_ready():
            self.get_logger().error('get_tool_force 서비스 준비 안됨')
            return None
        
        req = GetToolForce.Request()
        req.ref = ref
        
        future = self.cli_get_tool_force.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                force_data = result.tool_force
                if hasattr(force_data, 'tolist'):
                    return force_data.tolist()[:6]
                elif hasattr(force_data, 'data'):
                    return [float(x) for x in list(force_data.data)[:6]]
                else:
                    return [float(x) for x in list(force_data)[:6]]
        return None
    
    def set_digital_output(self, index, value):
        """SetCtrlBoxDigitalOutput 서비스 호출 래퍼"""
        if not self.cli_set_digital_output.service_is_ready():
            self.get_logger().error('set_digital_output 서비스 준비 안됨')
            return False
        
        req = SetCtrlBoxDigitalOutput.Request()
        req.index = index
        req.value = value
        
        future = self.cli_set_digital_output.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def get_digital_input(self, index):
        """GetCtrlBoxDigitalInput 서비스 호출 래퍼"""
        if not self.cli_get_digital_input.service_is_ready():
            self.get_logger().error('get_digital_input 서비스 준비 안됨')
            return None
        
        req = GetCtrlBoxDigitalInput.Request()
        req.index = index
        
        future = self.cli_get_digital_input.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().value
        return None
    
    def task_compliance_ctrl(self, stiffness=None):
        """TaskComplianceCtrl 서비스 호출 래퍼 (Z축 부드럽게)"""
        if not self.cli_task_compliance.service_is_ready():
            self.get_logger().error('task_compliance_ctrl 서비스 준비 안됨')
            return False
        
        req = TaskComplianceCtrl.Request()
        if stiffness is None:
            # Z축만 부드럽게 설정 (원본 알고리즘과 동일)
            req.stx = [1000.0, 1000.0, 200.0, 500.0, 500.0, 500.0]
        else:
            req.stx = stiffness
        req.ref = 0  # DR_BASE
        req.time = 0.0
        
        future = self.cli_task_compliance.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def release_compliance_ctrl(self):
        """ReleaseComplianceCtrl 서비스 호출 래퍼"""
        if not self.cli_release_compliance.service_is_ready():
            self.get_logger().error('release_compliance_ctrl 서비스 준비 안됨')
            return False
        
        req = ReleaseComplianceCtrl.Request()
        
        future = self.cli_release_compliance.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def set_desired_force(self, force, direction, time_val=0.0, mod=DR_FC_MOD_ABS):
        """SetDesiredForce 서비스 호출 래퍼"""
        if not self.cli_set_desired_force.service_is_ready():
            self.get_logger().error('set_desired_force 서비스 준비 안됨')
            return False
        
        req = SetDesiredForce.Request()
        req.fd = np.array(force, dtype=np.float64)
        req.dir = np.array(direction, dtype=np.int8)
        req.time = time_val
        req.mod = mod
        
        future = self.cli_set_desired_force.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def release_force(self, time_val=0.0):
        """ReleaseForce 서비스 호출 래퍼"""
        if not self.cli_release_force.service_is_ready():
            self.get_logger().error('release_force 서비스 준비 안됨')
            return False
        
        req = ReleaseForce.Request()
        req.time = time_val
        
        future = self.cli_release_force.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    # =========================================
    # 상태 관리
    # =========================================
    def publish_status(self):
        """상태 발행"""
        status = {
            'is_running': self.is_running,
            'is_paused': self.is_paused,
            'current_phase': 'PICK' if self.current_phase == PHASE_PICK else 'PLACE',
            'cycle_count': self.cycle_count,
            'last_classification': self.last_width_class,
            'dsr_ready': self.dsr_ready,
            'conveyor_mode': self.conveyor_mode,
            'conveyor_detected': self.conveyor_detected,
            'waiting_for_object': self.waiting_for_object,
            'completed': self.stats['completed'],
            'errors': self.stats['errors'],
            'small': self.stats['small'],
            'medium': self.stats['medium'],
            'large': self.stats['large'],
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)
        
        running_msg = Bool()
        running_msg.data = self.is_running
        self.pub_running.publish(running_msg)
    
    # =========================================
    # 서비스 콜백
    # =========================================
    def start_sort_callback(self, request, response):
        """분류 작업 시작"""
        if self.is_running:
            response.success = False
            response.message = '이미 실행 중입니다'
            return response
        
        if not self.dsr_ready:
            response.success = False
            response.message = 'DSR 로봇이 준비되지 않았습니다'
            return response
        
        self.is_running = True
        self.stop_requested = False
        self.is_paused = False
        
        # 별도 스레드에서 분류 작업 실행
        self.sort_thread = threading.Thread(target=self.run_sort_loop, daemon=True)
        self.sort_thread.start()
        
        response.success = True
        response.message = '분류 작업 시작'
        self.get_logger().info('분류 작업 시작')
        return response
    
    def stop_sort_callback(self, request, response):
        """분류 작업 정지"""
        if not self.is_running:
            response.success = False
            response.message = '실행 중이 아닙니다'
            return response
        
        self.stop_requested = True
        self.is_running = False
        
        # ===== 정지 시 현재 상태 저장 =====
        self.save_state()
        self.get_logger().info(f'[STOP] 상태 저장됨 (phase={self.current_phase}, z_touch={self.z_touch:.2f})')
        
        response.success = True
        response.message = '분류 작업 정지 요청됨'
        self.get_logger().info('분류 작업 정지 요청')
        return response
    
    def pause_sort_callback(self, request, response):
        """분류 작업 일시정지/재개"""
        if not self.is_running:
            response.success = False
            response.message = '실행 중이 아닙니다'
            return response
        
        self.is_paused = request.data
        
        # ===== 일시정지 시 현재 상태 저장 및 로봇 모션 즉시 중단 =====
        if self.is_paused:
            # 로봇 모션 즉시 중단 (Quick Stop)
            self.stop_robot_motion()
            self.save_state()
            self.get_logger().info(f'[PAUSE] 로봇 정지 및 상태 저장됨 (phase={self.current_phase}, z_touch={self.z_touch:.2f})')
        
        response.success = True
        response.message = '일시정지' if self.is_paused else '재개'
        self.get_logger().info(f'분류 작업 {"일시정지" if self.is_paused else "재개"}')
        return response
    
    def stop_robot_motion(self):
        """로봇 모션 즉시 중단 (MoveStop 서비스 호출)"""
        try:
            if not self.cli_move_stop.service_is_ready():
                self.get_logger().warn('[STOP] MoveStop 서비스 준비 안됨')
                return False
            
            req = MoveStop.Request()
            req.stop_mode = 1  # DR_QSTOP (Quick Stop - Category 2)
            
            future = self.cli_move_stop.call_async(req)
            # 비동기 호출이지만 즉시 정지를 위해 짧은 대기
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.done() and future.result().success:
                self.get_logger().info('[STOP] 🛑 로봇 모션 즉시 정지!')
                return True
            else:
                self.get_logger().warn('[STOP] 로봇 정지 실패')
                return False
        except Exception as e:
            self.get_logger().error(f'[STOP] 오류: {e}')
            return False
    
    def reset_state_callback(self, request, response):
        """상태 초기화"""
        self.current_phase = PHASE_PICK
        self.z_touch = HOME_C_VAL[2]
        self.cycle_count = 0
        self.last_width_class = None
        self.stats = {'completed': 0, 'errors': 0, 'small': 0, 'medium': 0, 'large': 0}
        self.save_state()
        
        response.success = True
        response.message = '상태 초기화 완료'
        self.get_logger().info('상태 초기화')
        return response
    
    def conveyor_status_callback(self, msg):
        """컨베이어 상태 수신 콜백"""
        code = msg.data
        
        if code == CONVEYOR_DETECT:
            self.conveyor_detected = True
            self.get_logger().info('[CONVEYOR] 물체 감지!')
            self.get_logger().info(f'[CONVEYOR] 상태 체크: conveyor_mode={self.conveyor_mode}, waiting_for_object={self.waiting_for_object}, is_running={self.is_running}')
            
            if self.conveyor_mode and self.waiting_for_object and not self.is_running:
                self.get_logger().info('[CONVEYOR] ✅ 조건 충족 - 자동 분류 시작!')
                self.waiting_for_object = False
                self.start_single_cycle()
            else:
                if not self.conveyor_mode:
                    self.get_logger().info('[CONVEYOR] ❌ 자동 모드 비활성화 상태')
                if not self.waiting_for_object:
                    self.get_logger().info('[CONVEYOR] ❌ 물체 대기 상태 아님 (HOME 이동 중?)')
                if self.is_running:
                    self.get_logger().info('[CONVEYOR] ❌ 이미 분류 작업 실행 중')
        
        elif code == CONVEYOR_RUNNING:
            self.conveyor_detected = False
    
    def conveyor_mode_callback(self, request, response):
        """컨베이어 자동 모드 설정"""
        self.conveyor_mode = request.data
        
        if self.conveyor_mode:
            # ===== 자동 모드 활성화 시 컨베이어 시작 =====
            self.send_conveyor_resume()
            
            # ===== 로봇을 HOME 위치로 이동 (대기 자세) - 별도 스레드에서 =====
            if self.dsr_ready and not self.is_running:
                def move_home_and_wait():
                    self.get_logger().info('[CONVEYOR] HOME 위치로 이동 (대기 자세)')
                    self.grip_on()  # 그리퍼 닫기
                    time.sleep(0.3)
                    home = HOME_C_VAL.copy()
                    self.movel(home, vel=V_MOVE, acc=A_MOVE)
                    # 이동 완료 후 물체 대기 상태로 전환
                    self.waiting_for_object = True
                    self.get_logger().info('[CONVEYOR] HOME 도착 - 물체 감지 대기 중')
                    
                    # ===== HOME 도착 시 이미 물체가 감지된 상태면 바로 분류 시작 =====
                    if self.conveyor_detected and self.conveyor_mode and not self.is_running:
                        self.get_logger().info('[CONVEYOR] ✅ 이미 물체 감지됨 - 자동 분류 시작!')
                        self.waiting_for_object = False
                        self.start_single_cycle()
                
                import threading
                threading.Thread(target=move_home_and_wait, daemon=True).start()
            else:
                # DSR 미연결 시 바로 대기 상태로
                self.waiting_for_object = True
            
            response.message = '컨베이어 자동 모드 활성화 - 컨베이어 시작'
            self.get_logger().info('[CONVEYOR] 자동 모드 ON - 컨베이어 시작, HOME으로 이동 중')
        else:
            self.waiting_for_object = False
            response.message = '컨베이어 자동 모드 비활성화'
            self.get_logger().info('[CONVEYOR] 자동 모드 OFF')
        
        response.success = True
        return response
    
    def send_conveyor_resume(self):
        """컨베이어에 RESUME 명령 전송"""
        msg = String()
        msg.data = 'RESUME'
        self.pub_conveyor_cmd.publish(msg)
        self.get_logger().info('[CONVEYOR] RESUME 명령 전송')
    
    def start_single_cycle(self):
        """단일 분류 사이클 시작 (컨베이어 모드용)"""
        if self.is_running:
            return
        
        if not self.dsr_ready:
            self.get_logger().warn('DSR 로봇이 준비되지 않았습니다')
            return
        
        self.is_running = True
        self.stop_requested = False
        self.is_paused = False
        
        self.sort_thread = threading.Thread(target=self.run_single_cycle, daemon=True)
        self.sort_thread.start()
    
    def load_state(self):
        """상태 로드"""
        if not os.path.exists(STATE_FILE):
            return
        
        try:
            with open(STATE_FILE, 'r') as f:
                data = json.load(f)
                self.current_phase = data.get("phase", PHASE_PICK)
                self.z_touch = data.get("z_touch", HOME_C_VAL[2])
                self.cycle_count = data.get("cycle_count", 0)
        except Exception as e:
            self.get_logger().warn(f'상태 로드 실패: {e}')
    
    def save_state(self):
        """상태 저장"""
        data = {
            "phase": int(self.current_phase),
            "z_touch": float(self.z_touch),
            "cycle_count": int(self.cycle_count),
        }
        try:
            with open(STATE_FILE, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            self.get_logger().warn(f'상태 저장 실패: {e}')
    
    # =========================================
    # 그리퍼 제어
    # =========================================
    def grip_off(self):
        """그리퍼 열기"""
        self.set_digital_output(1, 0)  # OFF
        self.set_digital_output(2, 1)  # ON
        time.sleep(0.3)
    
    def grip_on(self):
        """그리퍼 닫기"""
        self.set_digital_output(1, 1)  # ON
        self.set_digital_output(2, 0)  # OFF
        time.sleep(0.3)
    
    def get_width_class(self):
        """RG2 그리퍼 폭 구간 읽기"""
        di1 = self.get_digital_input(1)
        di2 = self.get_digital_input(2)
        
        if di1 is None or di2 is None:
            return "UNKNOWN"
        
        if di1 == 1 and di2 == 0:
            return "SMALL"
        elif di1 == 0 and di2 == 1:
            return "LONG"
        elif di1 == 0 and di2 == 0:
            return "MEDIUM"
        else:
            return "ERROR"
    
    # =========================================
    # 분류 작업 루프
    # =========================================
    def run_sort_loop(self):
        """분류 작업 메인 루프"""
        self.load_state()
        
        home = HOME_C_VAL.copy()
        
        # 초기 설정 (원본 알고리즘과 동일)
        self.grip_on()
        time.sleep(0.3)
        
        # 홈 위치로 이동
        self.get_logger().info('STEP: HOME 위치로 이동')
        self.movel(home, vel=V_MOVE, acc=A_MOVE)
        
        while self.is_running and not self.stop_requested:
            # 일시정지 대기
            if self.is_paused:
                self.get_logger().info('[PAUSE] 일시정지 중... 재개 대기')
            while self.is_paused and self.is_running:
                time.sleep(0.1)
            if not self.is_paused and self.is_running and not self.stop_requested:
                self.get_logger().info('[RESUME] 작업 재개')
            
            if self.stop_requested:
                break
            
            try:
                # PICK 단계
                if self.current_phase == PHASE_PICK:
                    self.get_logger().info(f'[CYCLE {self.cycle_count + 1}] PICK 단계')
                    self.pick_and_measure()
                    self.current_phase = PHASE_PLACE
                    self.save_state()
                
                if self.stop_requested:
                    break
                
                # PLACE 단계
                if self.current_phase == PHASE_PLACE:
                    self.get_logger().info(f'[CYCLE {self.cycle_count + 1}] PLACE 단계')
                    self.last_width_class = self.get_width_class()
                    self.place_to_box(self.last_width_class)
                    self.current_phase = PHASE_PICK
                    self.cycle_count += 1
                    self.save_state()
                
                # 홈으로 복귀
                self.movel(home, vel=V_MOVE, acc=A_MOVE)
                
                # ===== 홈 복귀 후 그리퍼 닫기 (다음 사이클 준비) =====
                self.grip_on()
                time.sleep(0.3)
                self.get_logger().info('[CYCLE] 그리퍼 닫힘 - 다음 사이클 준비 완료')
                
                # 통계 업데이트
                self.stats['completed'] += 1
                if self.last_width_class == 'SMALL':
                    self.stats['small'] += 1
                elif self.last_width_class == 'MEDIUM':
                    self.stats['medium'] += 1
                elif self.last_width_class == 'LONG':
                    self.stats['large'] += 1
                
                self.get_logger().info(f'[CYCLE {self.cycle_count}] 완료 (분류: {self.last_width_class})')
                
            except Exception as e:
                self.get_logger().error(f'분류 작업 오류: {e}')
                self.stats['errors'] += 1
                self.is_running = False
                break
        
        # 정리
        self.is_running = False
        self.get_logger().info('분류 작업 종료')
    
    def run_single_cycle(self):
        """단일 분류 사이클 실행 (컨베이어 모드용)"""
        self.load_state()
        
        home = HOME_C_VAL.copy()
        
        # 초기 설정
        self.grip_on()
        time.sleep(0.3)
        
        # HOME 위치로 이동
        self.movel(home, vel=V_MOVE, acc=A_MOVE)
        
        try:
            # PICK 단계
            self.current_phase = PHASE_PICK
            self.get_logger().info(f'[SINGLE CYCLE] PICK 단계')
            self.pick_and_measure()
            
            # PLACE 단계
            self.current_phase = PHASE_PLACE
            self.last_width_class = self.get_width_class()
            self.get_logger().info(f'[SINGLE CYCLE] PLACE 단계 ({self.last_width_class})')
            self.place_to_box(self.last_width_class)
            
            # 홈으로 복귀
            self.movel(home, vel=V_MOVE, acc=A_MOVE)
            
            # ===== 홈 복귀 후 그리퍼 닫기 (다음 사이클 준비) =====
            self.grip_on()
            time.sleep(0.3)
            self.get_logger().info('[SINGLE CYCLE] 그리퍼 닫힘 - 다음 사이클 준비 완료')
            
            # 통계 업데이트
            self.cycle_count += 1
            self.stats['completed'] += 1
            if self.last_width_class == 'SMALL':
                self.stats['small'] += 1
            elif self.last_width_class == 'MEDIUM':
                self.stats['medium'] += 1
            elif self.last_width_class == 'LONG':
                self.stats['large'] += 1
            
            self.save_state()
            self.get_logger().info(f'[SINGLE CYCLE] 완료 (분류: {self.last_width_class})')
            
        except Exception as e:
            self.get_logger().error(f'단일 사이클 오류: {e}')
            self.stats['errors'] += 1
        
        finally:
            self.is_running = False
            
            # 컨베이어 모드면 RESUME 전송 후 다음 물체 대기
            if self.conveyor_mode:
                self.send_conveyor_resume()
                self.waiting_for_object = True
                self.get_logger().info('[CONVEYOR] 다음 물체 대기 중...')
    
    def pick_and_measure(self):
        """
        컨베이어 위에서 물체 픽업 + Compliance Control + Force 센서로 높이 측정
        원본 알고리즘의 pick_and_measure 함수와 동일한 시퀀스
        """
        home = HOME_C_VAL.copy()
        pick = PICK_POS_VAL.copy()
        
        hx, hy, hz, hrx, hry, hrz = home
        px, py, pz, prx, pry, prz = pick
        
        # 1) HOME_Z 높이에서 픽업 위치로 이동
        self.get_logger().info('STEP: HOME_Z 높이에서 컨베이어 픽업 위치로 이동')
        self.movel([px, py, hz, prx, pry, prz], vel=V_MOVE, acc=A_MOVE)
        
        # 2) Compliance Control 활성화
        self.get_logger().info('STEP: Force 센서 초기화 및 하강 시작')
        
        # ===== Compliance 강성 설정 (Z축만 부드럽게) =====
        self.get_logger().info('STEP: Compliance Control 활성화 중...')
        self.task_compliance_ctrl(stiffness=[1000.0, 1000.0, 200.0, 500.0, 500.0, 500.0])
        self.get_logger().info('STEP: Compliance Control 활성화 완료')
        
        # ===== Force 센서 초기 상태 확인 =====
        time.sleep(0.2)
        initial_force = self.get_tool_force(ref=DR_TOOL)
        if initial_force:
            self.get_logger().info(
                f'[FORCE_INIT] 초기 힘: Fx={initial_force[0]:.2f}, '
                f'Fy={initial_force[1]:.2f}, Fz={initial_force[2]:.2f}N'
            )
        
        # Z축으로 -FORCE_PUSH만큼 힘 인가 (연속 모드)
        self.get_logger().info(f'STEP: Desired Force 설정 (Fz=-{FORCE_PUSH}N)')
        self.set_desired_force(
            force=[0.0, 0.0, -FORCE_PUSH, 0.0, 0.0, 0.0],
            direction=[0, 0, 1, 0, 0, 0],
            time_val=0.0,  # 무한 지속
            mod=DR_FC_MOD_ABS,
        )
        
        # ===== Force 적용 확인 =====
        time.sleep(0.5)
        after_force = self.get_tool_force(ref=DR_TOOL)
        if after_force:
            self.get_logger().info(
                f'[FORCE_SET] Force 적용 후: Fx={after_force[0]:.2f}, '
                f'Fy={after_force[1]:.2f}, Fz={after_force[2]:.2f}N'
            )
        
        # ===== 힘만 체크하면서 대기 (실시간 모니터링) =====
        start_z = hz
        contact = False
        z_touch = hz
        
        max_wait_time = 10.0  # 최대 10초
        start_time = time.time()
        
        self.get_logger().info('='*60)
        self.get_logger().info('Force 센서 실시간 모니터링 시작')
        self.get_logger().info(f'시작 높이: {start_z:.2f}mm, 목표 Force: {FORCE_THRESH}N')
        self.get_logger().info('='*60)
        
        loop_count = 0
        last_print_time = time.time()
        
        while (time.time() - start_time) < max_wait_time:
            if self.stop_requested:
                break
            
            # ===== get_current_posx 안전하게 호출 =====
            cur_pos_result = self.get_current_posx(ref=DR_BASE)
            if cur_pos_result is None:
                self.get_logger().error('[MONITOR] get_current_posx 실패')
                break
            
            cur_z = cur_pos_result[2]
            
            # Force 센서 읽기
            force_data = self.get_tool_force(ref=DR_TOOL)
            if force_data is None:
                self.get_logger().error('[MONITOR] get_tool_force 실패')
                break
            
            fx, fy, fz = force_data[0], force_data[1], force_data[2]
            
            # ===== 0.1초마다 상태 출력 (10Hz) =====
            current_time = time.time()
            if (current_time - last_print_time) >= 0.1:
                moved_dist = start_z - cur_z
                self.get_logger().info(
                    f'[MONITOR] Z={cur_z:6.2f}mm (▼{moved_dist:5.2f}mm) | '
                    f'Fx={fx:6.2f}N, Fy={fy:6.2f}N, Fz={fz:6.2f}N | '
                    f'경과={current_time - start_time:.1f}s'
                )
                last_print_time = current_time
            
            # 안전 높이 체크
            if cur_z < SAFE_Z_LIMIT:
                self.get_logger().info('='*60)
                self.get_logger().info(f'[SAFE_Z] SAFE_Z_LIMIT {SAFE_Z_LIMIT} 도달')
                self.get_logger().info('='*60)
                z_touch = SAFE_Z_LIMIT
                break
            
            # 최대 하강 거리 체크
            if (start_z - cur_z) > MAX_DOWN:
                self.get_logger().info('='*60)
                self.get_logger().info(f'[MAX_DOWN] {MAX_DOWN}mm 하강 완료')
                self.get_logger().info('='*60)
                break
            
            # Force 센서 체크 (접촉 감지)
            if abs(fz) >= FORCE_THRESH:
                z_touch = cur_z
                self.get_logger().info('='*60)
                self.get_logger().info(f'[CONTACT] ✓ 컨베이어 물체 접촉 감지!')
                self.get_logger().info(f'         접촉 높이: z={z_touch:.2f}mm')
                self.get_logger().info(f'         접촉 힘: Fz={fz:.2f}N (임계값: {FORCE_THRESH}N)')
                self.get_logger().info(f'         하강 거리: {start_z - z_touch:.2f}mm')
                self.get_logger().info('='*60)
                contact = True
                break
            
            loop_count += 1
            time.sleep(0.01)  # 10ms 대기 (100Hz 체크)
        
        # ===== 최종 상태 출력 =====
        elapsed = time.time() - start_time
        final_force = self.get_tool_force(ref=DR_TOOL)
        if final_force:
            self.get_logger().info('='*60)
            self.get_logger().info(f'Force 모니터링 종료 (총 {loop_count}회 체크, {elapsed:.2f}초)')
            self.get_logger().info(f'최종 위치: Z={cur_z:.2f}mm')
            self.get_logger().info(
                f'최종 힘: Fx={final_force[0]:.2f}, Fy={final_force[1]:.2f}, Fz={final_force[2]:.2f}N'
            )
            self.get_logger().info(f'접촉 여부: {"YES" if contact else "NO"}')
            self.get_logger().info('='*60)
        
        # Force 제어 해제
        self.release_force(0.0)
        self.release_compliance_ctrl()
        
        if not contact:
            self.get_logger().info('[NO_CONTACT] 물체 미감지, HOME으로 복귀')
            self.movel(home, vel=V_MOVE, acc=A_MOVE)
            self.current_phase = PHASE_PICK
            self.save_state()
            return
        
        # 안전 보정
        if z_touch < SAFE_Z_LIMIT:
            self.get_logger().info(f'[SAFE_Z] z_touch={z_touch:.2f} -> {SAFE_Z_LIMIT}로 보정')
            z_touch = SAFE_Z_LIMIT
        
        # 3) 위로 올라갔다가 그리퍼 열고 다시 내려가 잡기
        self.get_logger().info(f'STEP: z_touch+UP_OFFSET={z_touch + UP_OFFSET:.2f}로 올라감')
        self.movel([px, py, z_touch + UP_OFFSET, prx, pry, prz], vel=V_MOVE, acc=A_MOVE)
        
        self.grip_off()
        
        target_pick_z = z_touch - GRIP_OFFSET - PICK_EXTRA_DOWN
        if target_pick_z < SAFE_Z_LIMIT:
            self.get_logger().info(f'[SAFE_Z] pick_z={target_pick_z:.2f} -> {SAFE_Z_LIMIT}로 보정')
            target_pick_z = SAFE_Z_LIMIT
        
        self.get_logger().info(f'STEP: 컨베이어 물체 집기 (z={target_pick_z:.2f}mm)')
        self.movel([px, py, target_pick_z, prx, pry, prz], vel=V_PICK_DOWN, acc=A_PICK_DOWN)
        
        self.grip_on()
        time.sleep(0.5)
        
        # 4) 다시 HOME_Z 높이까지 올리기
        self.get_logger().info('STEP: HOME_Z 높이까지 올리기')
        self.movel([px, py, hz, prx, pry, prz], vel=V_MOVE, acc=A_MOVE)
        
        # 5) RG2 폭 구간 읽기
        width_class = self.get_width_class()
        self.get_logger().info(f'측정 완료: {width_class}, 컨베이어 z_touch={z_touch:.2f}mm')
        
        # ===== 중요: z_touch 저장 (원본 알고리즘의 w_z_touch = z_touch) =====
        self.z_touch = z_touch
    
    def place_to_box(self, width_class):
        """
        컨베이어에서 측정한 z_touch를 기준으로 팔레트에 적재
        - 컨베이어 높이 70mm 보정 적용
        원본 알고리즘의 place_to_box 함수와 동일한 시퀀스
        """
        home = HOME_C_VAL.copy()
        hx, hy, hz, hrx, hry, hrz = home
        
        if width_class == "SMALL":
            dst_val = SMALL_POS_VAL.copy()
        elif width_class == "MEDIUM":
            dst_val = MEDIUM_POS_VAL.copy()
        elif width_class == "LONG":
            dst_val = LARGE_POS_VAL.copy()
        else:
            self.get_logger().warn(f'[WARN] width_class={width_class} -> 알 수 없는 폭, HOME으로 복귀')
            self.movel(home, vel=V_MOVE, acc=A_MOVE)
            return
        
        dx, dy, dz, drx, dry, drz = dst_val
        
        self.get_logger().info(f'STEP: HOME_Z에서 팔레트 위치로 이동 ({width_class})')
        self.movel([dx, dy, hz, drx, dry, drz], vel=V_MOVE, acc=A_MOVE)
        
        # ===== z_touch 기준으로 적재 높이 계산 (컨베이어 70mm 보정) =====
        # z_touch - PLACE_EXTRA_DOWN(50) - FINAL_PUSH(15) + PLACE_REDUCE(30) - CONVEYOR_HEIGHT_OFFSET(70)
        # = z_touch - 50 - 15 + 30 - 70 = z_touch - 105mm
        target_place_z = self.z_touch - PLACE_EXTRA_DOWN - FINAL_PUSH + PLACE_REDUCE - CONVEYOR_HEIGHT_OFFSET
        
        if target_place_z < SAFE_Z_PLACE:
            self.get_logger().info(f'[SAFE_Z_PLACE] place_z={target_place_z:.2f} -> {SAFE_Z_PLACE}로 보정')
            target_place_z = SAFE_Z_PLACE
        
        self.get_logger().info(
            f'STEP: 팔레트 적재 (z_touch={self.z_touch:.2f}mm → place_z={target_place_z:.2f}mm, '
            f'컨베이어 보정 70mm)'
        )
        
        self.movel([dx, dy, target_place_z, drx, dry, drz], vel=V_PICK_DOWN, acc=A_PICK_DOWN)
        
        self.grip_off()
        time.sleep(0.3)
        
        self.movel([dx, dy, hz, drx, dry, drz], vel=V_MOVE, acc=A_MOVE)
        
        # Firebase에 저장
        final_position = [dx, dy, target_place_z]
        force_at_contact = FORCE_THRESH
        self.save_to_firebase(width_class, final_position, force_at_contact)


def main(args=None):
    rclpy.init(args=args)
    node = DlarSortNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
