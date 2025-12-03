#!/usr/bin/env python3
"""
물류 로봇 모니터링 웹 서버 노드 (리팩토링)
- Flask + SocketIO 웹 서버
- 분류 작업 제어 (sort_node 연동)
- Firebase 연동
"""

import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import (
    GetCurrentTool, GetCurrentTcp,
    GetCurrentPosx, GetCurrentToolFlangePosx,
    GetToolForce, GetJointTorque,
    GetCtrlBoxDigitalOutput, SetCtrlBoxDigitalOutput,
    GetDesiredPosx, GetDesiredPosj,
    MoveHome, ChangeOperationSpeed, MoveStop,
    MovePause, MoveResume, MoveJoint
)
from dsr_msgs2.msg import RobotError

from flask import Flask
from flask_socketio import SocketIO

# SafetyManager 임포트
from ..safety import SafetyManager
from ..safety.safety_manager import is_safe_to_move, check_safety

# 웹 모듈 임포트
from ..web.data_store import (
    robot_data, sort_status, conveyor_status,
    ui_state, logs, add_log, set_logistics_broadcast_callback
)
from ..web.routes import routes_bp
from ..web.socket_handlers import register_socket_handlers
from ..web import firebase_threads

# Tasks 모듈 임포트
from ..tasks.pendulum import PendulumController

# 상태 모니터링 모듈 임포트
from ..monitoring.state_monitor import RobotStateMonitor
from ..web.robot_monitor import (
    RobotStatusMonitor,
    joint_state_callback,
    sort_status_callback,
    conveyor_status_callback,
    conveyor_code_callback,
    error_callback,
)

# 경로 설정
LOGISTICS_MONITOR_DIR = os.path.expanduser('~/cobot1_ws/src/logistics_monitor')
TEMPLATE_DIR = os.path.join(LOGISTICS_MONITOR_DIR, 'templates')


def wait_for_future(future, timeout=5.0):
    """
    Future 완료 대기 (폴링 방식)
    
    MultiThreadedExecutor와 함께 사용 시 spin_until_future_complete 대신 사용
    (spin_until_future_complete는 데드락 발생 가능)
    
    Args:
        future: rclpy Future 객체
        timeout: 타임아웃 (초)
        
    Returns:
        bool: 완료 여부 (True=완료, False=타임아웃)
    """
    start_time = time.time()
    while not future.done():
        if time.time() - start_time > timeout:
            return False
        time.sleep(0.01)
    return True

# Flask 앱 생성
app = Flask(__name__, template_folder=TEMPLATE_DIR)
app.config['SECRET_KEY'] = 'dsr_integrated_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Blueprint 등록
app.register_blueprint(routes_bp)

# 전역 ROS 노드 참조
ros_node = None


def get_ros_node():
    """ROS 노드 반환 (콜백에서 사용)"""
    return ros_node


# Socket 핸들러 등록
register_socket_handlers(socketio, get_ros_node)


# 물류 상태 브로드캐스트 콜백 등록
def _broadcast_logistics_status(data):
    """물류 상태 변경 시 소켓으로 브로드캐스트"""
    socketio.emit('logistics_status', data)

set_logistics_broadcast_callback(_broadcast_logistics_status)


class WebServerNode(Node):
    """ROS2 웹 서버 노드"""
    
    def __init__(self):
        super().__init__('web_server_node')
        self.get_logger().info('Web Server Node 시작 (Refactored)')
        
        self.callback_group = ReentrantCallbackGroup()
        self.start_time = time.time()
        
        # 구독자/퍼블리셔 생성
        self._create_subscribers()
        self._create_publishers()
        self._create_service_clients()
        
        # 로봇 상태 모니터 (STANDBY 체크용)
        self.state_monitor = RobotStateMonitor(self, self.callback_group)
        
        # 진자운동 컨트롤러 초기화 (BaseTask 인자 추가)
        self.pendulum = PendulumController(
            self, 
            self.cli_move_joint,
            state_monitor=self.state_monitor
            # recovery_checker는 server_node에서 사용하지 않음
        )
        
        # 상태 모니터 초기화
        self.status_monitor = RobotStatusMonitor(self, {
            'get_posx': self.cli_get_posx,
            'tool_force': self.cli_tool_force,
            'get_tool': self.cli_get_tool,
            'get_tcp': self.cli_get_tcp,
            'get_flange_posx': self.cli_get_flange_posx,
            'joint_torque': self.cli_joint_torque,
            'get_digital_output': self.cli_get_digital_output,
        })
        
        # 상태 업데이트 타이머
        self.robot_status_timer = self.create_timer(0.2, self._update_robot_status)
        
        # SafetyManager 초기화
        SafetyManager.initialize(self)
        
        robot_data['connected'] = True
        self.get_logger().info('Web Server Node 준비 완료')
    
    def _create_subscribers(self):
        """구독자 생성"""
        # 조인트 상태
        self.joint_sub = self.create_subscription(
            JointState, '/dsr01/joint_states',
            lambda msg: joint_state_callback(msg), 10,
            callback_group=self.callback_group
        )
        
        # 분류 상태
        self.sort_status_sub = self.create_subscription(
            String, '/dlar/status',
            lambda msg: sort_status_callback(msg), 10,
            callback_group=self.callback_group
        )
        
        # 복구 상태 (Web UI 전달용)
        self.recovery_status_sub = self.create_subscription(
            String, '/dlar/recovery_status',
            self._recovery_status_callback, 10,
            callback_group=self.callback_group
        )
        
        # 컨베이어 상태
        self.conveyor_status_sub = self.create_subscription(
            String, '/conveyor/status',
            lambda msg: conveyor_status_callback(msg), 10,
            callback_group=self.callback_group
        )
        self.conveyor_code_sub = self.create_subscription(
            Int32, '/conveyor/status_code',
            lambda msg: conveyor_code_callback(msg), 10,
            callback_group=self.callback_group
        )
        
        # 로봇 에러
        try:
            self.error_sub = self.create_subscription(
                RobotError, '/dsr01/error',
                lambda msg: error_callback(msg), 10,
                callback_group=self.callback_group
            )
        except Exception as e:
            self.get_logger().warn(f'RobotError 구독 실패: {e}')
    
    def _create_publishers(self):
        """퍼블리셔 생성"""
        self.conveyor_cmd_pub = self.create_publisher(String, '/conveyor/cmd', 10)
    
    def _create_service_clients(self):
        """서비스 클라이언트 생성"""
        prefix = '/dsr01'
        cb = self.callback_group
        
        # 분류 제어
        self.cli_start_sort = self.create_client(Trigger, '/dlar/start_sort', callback_group=cb)
        self.cli_stop_sort = self.create_client(Trigger, '/dlar/stop_sort', callback_group=cb)
        self.cli_pause_sort = self.create_client(SetBool, '/dlar/pause_sort', callback_group=cb)
        self.cli_reset_state = self.create_client(Trigger, '/dlar/reset_state', callback_group=cb)
        self.cli_conveyor_mode = self.create_client(SetBool, '/dlar/conveyor_mode', callback_group=cb)
        self.cli_collision_recovery = self.create_client(Trigger, '/dlar/collision_recovery', callback_group=cb)
        
        # Tool/TCP
        self.cli_get_tool = self.create_client(GetCurrentTool, f'{prefix}/tool/get_current_tool', callback_group=cb)
        self.cli_get_tcp = self.create_client(GetCurrentTcp, f'{prefix}/tcp/get_current_tcp', callback_group=cb)
        
        # 상태 조회
        self.cli_get_posx = self.create_client(GetCurrentPosx, f'{prefix}/aux_control/get_current_posx', callback_group=cb)
        self.cli_get_flange_posx = self.create_client(GetCurrentToolFlangePosx, f'{prefix}/aux_control/get_current_tool_flange_posx', callback_group=cb)
        self.cli_tool_force = self.create_client(GetToolForce, f'{prefix}/aux_control/get_tool_force', callback_group=cb)
        self.cli_joint_torque = self.create_client(GetJointTorque, f'{prefix}/aux_control/get_joint_torque', callback_group=cb)
        self.cli_desired_posx = self.create_client(GetDesiredPosx, f'{prefix}/aux_control/get_desired_posx', callback_group=cb)
        self.cli_desired_posj = self.create_client(GetDesiredPosj, f'{prefix}/aux_control/get_desired_posj', callback_group=cb)
        
        # IO
        self.cli_get_digital_output = self.create_client(GetCtrlBoxDigitalOutput, f'{prefix}/io/get_ctrl_box_digital_output', callback_group=cb)
        self.cli_set_digital_output = self.create_client(SetCtrlBoxDigitalOutput, f'{prefix}/io/set_ctrl_box_digital_output', callback_group=cb)
        
        # 모션
        self.cli_move_home = self.create_client(MoveHome, f'{prefix}/motion/move_home', callback_group=cb)
        self.cli_move_joint = self.create_client(MoveJoint, f'{prefix}/motion/move_joint', callback_group=cb)
        self.cli_change_speed = self.create_client(ChangeOperationSpeed, f'{prefix}/motion/change_operation_speed', callback_group=cb)
        self.cli_move_stop = self.create_client(MoveStop, f'{prefix}/motion/move_stop', callback_group=cb)
        self.cli_move_pause = self.create_client(MovePause, f'{prefix}/motion/move_pause', callback_group=cb)
        self.cli_move_resume = self.create_client(MoveResume, f'{prefix}/motion/move_resume', callback_group=cb)
    
    def _update_robot_status(self):
        """로봇 상태 정보 업데이트 (RobotStatusMonitor 위임)"""
        self.status_monitor.update_robot_status()
    
    def _recovery_status_callback(self, msg):
        """복구 상태 콜백 → SocketIO로 전달"""
        import json
        try:
            data = json.loads(msg.data)
            event = data.get('event', '')
            
            if event == 'detected':
                socketio.emit('collision_detected', data)
            elif event == 'progress':
                socketio.emit('recovery_progress', {
                    'step': data.get('step', ''),
                    'percent': data.get('percent', 0)
                })
            elif event == 'complete':
                socketio.emit('recovery_complete', {
                    'success': data.get('success', False),
                    'message': data.get('step', '')
                })
        except Exception as e:
            self.get_logger().error(f'복구 상태 파싱 오류: {e}')
    
    # =========================================
    # 서비스 호출 메서드
    # =========================================
    def call_start_sort(self):
        """분류 시작"""
        if not self.cli_start_sort.service_is_ready():
            return False, '분류 노드 준비 안됨'
        req = Trigger.Request()
        future = self.cli_start_sort.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False, '서비스 호출 타임아웃'
        if future.result():
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_stop_sort(self):
        """분류 정지"""
        if not self.cli_stop_sort.service_is_ready():
            return False, '분류 노드 준비 안됨'
        req = Trigger.Request()
        future = self.cli_stop_sort.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False, '서비스 호출 타임아웃'
        if future.result():
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_pause_sort(self, pause):
        """분류 일시정지/재개"""
        if not self.cli_pause_sort.service_is_ready():
            return False, '분류 노드 준비 안됨'
        req = SetBool.Request()
        req.data = pause
        future = self.cli_pause_sort.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False, '서비스 호출 타임아웃'
        if future.result():
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_reset_state(self):
        """상태 초기화"""
        if not self.cli_reset_state.service_is_ready():
            return False, '분류 노드 준비 안됨'
        req = Trigger.Request()
        future = self.cli_reset_state.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False, '서비스 호출 타임아웃'
        if future.result():
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_conveyor_mode(self, enabled):
        """컨베이어 모드 설정"""
        if not self.cli_conveyor_mode.service_is_ready():
            return False, '분류 노드 준비 안됨'
        req = SetBool.Request()
        req.data = enabled
        future = self.cli_conveyor_mode.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False, '서비스 호출 타임아웃'
        if future.result():
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def call_collision_recovery(self):
        """충돌 복구"""
        if not self.cli_collision_recovery.service_is_ready():
            return False, '분류 노드 준비 안됨'
        req = Trigger.Request()
        future = self.cli_collision_recovery.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False, '서비스 호출 타임아웃'
        if future.result():
            return future.result().success, future.result().message
        return False, '서비스 호출 실패'
    
    def set_gripper(self, open_gripper):
        """그리퍼 제어"""
        if not self.cli_set_digital_output.service_is_ready():
            return False
        req1 = SetCtrlBoxDigitalOutput.Request()
        req1.index = 1
        req1.value = 0 if open_gripper else 1
        req2 = SetCtrlBoxDigitalOutput.Request()
        req2.index = 2
        req2.value = 1 if open_gripper else 0
        self.cli_set_digital_output.call_async(req1)
        self.cli_set_digital_output.call_async(req2)
        return True
    
    def move_home(self, target=1):
        """홈 이동"""
        if not self.cli_move_home.service_is_ready():
            return False
        req = MoveHome.Request()
        req.target = target
        self.cli_move_home.call_async(req)
        return True
    
    def change_speed(self, speed):
        """속도 변경"""
        if not self.cli_change_speed.service_is_ready():
            return False
        req = ChangeOperationSpeed.Request()
        req.speed = int(speed)
        self.cli_change_speed.call_async(req)
        return True
    
    # ========== 내부 헬퍼 메서드 (SafetyManager에서 호출) ==========
    
    def _call_move_pause(self):
        """비상정지: MoveStop으로 즉시 정지 (목표 위치는 진자운동 루프에서 관리)"""
        # MoveStop - 현재 모션 즉시 정지
        if self.cli_move_stop.service_is_ready():
            req = MoveStop.Request()
            req.stop_mode = 1  # STOP_TYPE_QUICK (즉시 정지)
            self.cli_move_stop.call_async(req)
            print("✅ MoveStop 호출됨 (즉시 정지)")
            return True
        else:
            print("⚠️ MoveStop 서비스 준비 안됨")
            return False
    
    def _get_desired_posj(self):
        """GetDesiredPosj 서비스 호출 - 목표 관절 위치 반환"""
        if not self.cli_desired_posj.service_is_ready():
            print("⚠️ GetDesiredPosj 서비스 준비 안됨")
            return None
        
        try:
            req = GetDesiredPosj.Request()
            future = self.cli_desired_posj.call_async(req)
            
            # 타임아웃 내에 결과 대기
            timeout = 1.0  # 1초
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout:
                time.sleep(0.01)
            
            if future.done():
                result = future.result()
                if hasattr(result, 'pos') and len(result.pos) >= 6:
                    target_posj = list(result.pos[:6])
                    print(f"✅ GetDesiredPosj: {[f'{p:.1f}' for p in target_posj]}")
                    return target_posj
                else:
                    print(f"⚠️ GetDesiredPosj 결과 형식 오류: {result}")
                    return None
            else:
                print("⚠️ GetDesiredPosj 타임아웃")
                return None
        except Exception as e:
            print(f"⚠️ GetDesiredPosj 예외: {e}")
            return None
    
    def _call_move_to_saved_target(self, target_posj, vel=30.0):
        """저장된 목표 위치로 movej 호출 (비상정지 해제 후 이어서 재개)"""
        if not target_posj or len(target_posj) < 6:
            print("⚠️ 저장된 목표 위치가 없음")
            return False
        
        if not self.cli_move_joint.service_is_ready():
            print("⚠️ MoveJoint 서비스 준비 안됨")
            return False
        
        try:
            req = MoveJoint.Request()
            req.pos = target_posj
            req.time = 0.0
            req.vel = vel
            req.acc = vel
            req.radius = 0.0
            req.mode = 0
            req.blend_type = 0
            req.sync_type = 0
            
            print(f"▶️ 저장된 목표로 이어서 재개: {[f'{p:.1f}' for p in target_posj]}")
            future = self.cli_move_joint.call_async(req)
            return True
        except Exception as e:
            print(f"⚠️ MoveJoint 예외: {e}")
            return False
    
    def _call_move_resume(self):
        """비상정지 해제: 상태만 변경 (스레드가 자동으로 다음 동작 진행)"""
        # MoveStop 후에는 MoveResume이 의미 없음
        # SafetyManager 상태 변경만으로 스레드가 다음 동작 진행
        print("✅ 비상정지 해제됨 (다음 동작부터 재개)")
        return True
    
    # ========== 공개 API (SafetyManager 위임) ==========
    
    def emergency_stop(self):
        """긴급정지 - SafetyManager를 통해 처리"""
        return SafetyManager.emergency_stop("웹 UI 비상정지")
    
    def emergency_stop_release(self):
        """긴급정지 해제 - SafetyManager를 통해 처리"""
        return SafetyManager.emergency_release()
    
    def pause_motion(self):
        """일시정지"""
        return SafetyManager.pause("웹 UI 일시정지")
    
    def resume_motion(self):
        """재개"""
        return SafetyManager.resume()
    
    def send_conveyor_cmd(self, command):
        """컨베이어 명령"""
        msg = String()
        msg.data = command
        self.conveyor_cmd_pub.publish(msg)
        self.get_logger().info(f"[CONVEYOR CMD] Published: {command}")
        return True
    
    # =========================================
    # 진자운동 테스트 (PendulumController 위임)
    # =========================================
    @property
    def pendulum_running(self) -> bool:
        """진자운동 실행 상태"""
        return self.pendulum.is_running
    
    @property
    def pendulum_paused(self) -> bool:
        """진자운동 일시정지 상태"""
        return self.pendulum.is_paused
    
    def start_pendulum_test(self, joint_index=0, amplitude=15.0, vel=30.0):
        """진자운동 시작"""
        return self.pendulum.start(joint_index, amplitude, vel)
    
    def stop_pendulum_test(self):
        """진자운동 정지"""
        return self.pendulum.stop()
    
    def pause_pendulum_test(self):
        """진자운동 일시정지"""
        return self.pendulum.pause()
    
    def resume_pendulum_test(self):
        """진자운동 재개"""
        return self.pendulum.resume()


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
    while True:
        if robot_data['connected']:
            if ros_node:
                ui_state['pendulum_running'] = ros_node.pendulum_running
            
            socketio.emit('robot_state', robot_data)
            socketio.emit('sort_status', sort_status)
            socketio.emit('conveyor_status', conveyor_status)
            socketio.emit('logs', logs[:20])
            socketio.emit('ui_state', ui_state)
        time.sleep(0.1)


def main(args=None):
    global ros_node
    
    print("=" * 70)
    print("📦 물류 로봇 통합 시스템 - 웹 서버 노드 (Refactored)")
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
    
    # Firebase 초기화
    firebase_threads.init_firebase()
    if firebase_threads.is_firebase_enabled():
        fb_upload = threading.Thread(
            target=firebase_threads.firebase_upload_thread,
            args=(get_ros_node,),
            daemon=True
        )
        fb_upload.start()
        
        fb_listener = threading.Thread(
            target=firebase_threads.firebase_command_listener,
            args=(get_ros_node, socketio),
            daemon=True
        )
        fb_listener.start()
        print("✅ Firebase 연동 시작")
    
    print(f"📍 Web UI: http://localhost:5000")
    print("=" * 70)
    
    # Flask 서버 실행
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    
    # 정리
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
