#!/usr/bin/env python3
"""
DSR 시스템 관리자 노드 (System Manager)

역할:
1. 시스템 시작 시 full_system.launch.py 자동 실행
2. 드라이버 상태 주기적 감시
3. 드라이버 죽음 감지 시 전체 시스템 완전 재시작 (pkill + launch)
4. RViz 포함 모든 관련 프로세스 종료 후 재시작
5. 재시작 후 복구 로직 수행 (그리퍼 열기, 사이클 카운트 복원)

복구 로직:
- 재시작 시 물체를 놓음 (gripping=True일 경우 그리퍼 열기)
- 재시도는 최대 3번
- 사이클 카운트는 유지 (이어서 작업)

사용법:
    ros2 run dsr_recovery_watcher watcher_node

    # 자동 재시작 활성화 (기본값: true)
    ros2 run dsr_recovery_watcher watcher_node --ros-args -p auto_restart:=true
    
    # 시작 시 launch 자동 실행 (기본값: true)
    ros2 run dsr_recovery_watcher watcher_node --ros-args -p auto_launch:=true

Firebase 상태:
- 웹이 꺼져도 Firebase DB에 마지막 작업 상태 저장됨
- sort_status.phase, cycle_count, running 등 확인 가능
"""

import os
import subprocess
import signal
import time
import threading
import json

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

# DSR 서비스 (동적 import)
DSR_AVAILABLE = False
try:
    from dsr_msgs2.srv import GetRobotState
    DSR_AVAILABLE = True
except ImportError:
    print("⚠️ dsr_msgs2 없음 - 더미 모드로 실행")

# Firebase (동적 import)
FIREBASE_AVAILABLE = False
try:
    import firebase_admin
    from firebase_admin import credentials, db
    FIREBASE_AVAILABLE = True
except ImportError:
    print("⚠️ firebase_admin 없음 - Firebase 복구 기능 비활성화")


class SystemManagerNode(Node):
    """시스템 관리자 노드 - Launch 실행 및 드라이버 감시"""
    
    def __init__(self):
        super().__init__('system_manager_node')
        self.get_logger().info('🚀 시스템 관리자 노드 시작')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # ========== 파라미터 ==========
        self.declare_parameter('auto_launch', False)     # 시작 시 자동 launch (False: 감시만)
        self.declare_parameter('auto_restart', True)     # 드라이버 죽으면 자동 재시작
        self.declare_parameter('restart_cooldown', 30.0) # 재시작 쿨다운 (초)
        self.declare_parameter('check_interval', 2.0)    # 드라이버 체크 주기 (초)
        self.declare_parameter('max_failures', 3)        # 연속 실패 허용 횟수
        self.declare_parameter('startup_delay', 10.0)    # launch 후 체크 시작 대기 (초)
        self.declare_parameter('workspace_path', os.path.expanduser('~/cobot1_ws'))
        self.declare_parameter('max_retry_count', 3)     # 최대 재시도 횟수
        self.declare_parameter('recovery_delay', 5.0)    # 복구 후 대기 시간 (초)
        
        self._auto_launch = self.get_parameter('auto_launch').value
        self._auto_restart = self.get_parameter('auto_restart').value
        self._restart_cooldown = self.get_parameter('restart_cooldown').value
        self._check_interval = self.get_parameter('check_interval').value
        self._max_failures = self.get_parameter('max_failures').value
        self._startup_delay = self.get_parameter('startup_delay').value
        self._workspace_path = self.get_parameter('workspace_path').value
        self._max_retry_count = self.get_parameter('max_retry_count').value
        self._recovery_delay = self.get_parameter('recovery_delay').value
        
        # ========== 상태 변수 ==========
        self._driver_alive = False  # 시작 시 False (launch 안됐으므로)
        self._consecutive_failures = 0
        self._last_restart_time = 0
        self._launch_process = None
        self._system_ready = False  # launch 후 startup_delay 지나면 True
        self._restart_count = 0     # 총 재시작 횟수
        self._retry_count = 0       # 현재 재시도 횟수
        self._last_recovery_state = None  # 마지막 복구 상태 (Firebase에서 읽음)
        self._firebase_initialized = False
        
        # ========== 서비스 클라이언트 ==========
        if DSR_AVAILABLE:
            self.cli_get_state = self.create_client(
                GetRobotState,
                '/dsr01/system/get_robot_state',
                callback_group=self.callback_group
            )
        else:
            self.cli_get_state = None
        
        # ========== 퍼블리셔 ==========
        self.pub_restart_needed = self.create_publisher(
            Bool, '/dlar/driver_restart_needed', 10
        )
        self.pub_driver_status = self.create_publisher(
            String, '/dlar/driver_status', 10
        )
        self.pub_system_status = self.create_publisher(
            String, '/dlar/system_status', 10
        )
        
        # ========== 서비스 ==========
        self.srv_restart = self.create_service(
            Trigger, '/dlar/restart_system',
            self._handle_restart_request,
            callback_group=self.callback_group
        )
        self.srv_shutdown = self.create_service(
            Trigger, '/dlar/shutdown_system',
            self._handle_shutdown_request,
            callback_group=self.callback_group
        )
        
        # ========== 타이머 ==========
        self.check_timer = self.create_timer(
            self._check_interval,
            self._check_driver,
            callback_group=self.callback_group
        )
        
        # ========== 설정 출력 ==========
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'📦 워크스페이스: {self._workspace_path}')
        self.get_logger().info(f'🚀 자동 Launch: {self._auto_launch}')
        self.get_logger().info(f'🔄 자동 재시작: {self._auto_restart}')
        self.get_logger().info(f'⏱️ 체크 주기: {self._check_interval}초')
        self.get_logger().info(f'⚠️ 실패 허용: {self._max_failures}회')
        self.get_logger().info(f'⏳ 시작 대기: {self._startup_delay}초')
        self.get_logger().info(f'🕐 쿨다운: {self._restart_cooldown}초')
        self.get_logger().info(f'🔁 최대 재시도: {self._max_retry_count}회')
        self.get_logger().info(f'💾 Firebase: {FIREBASE_AVAILABLE}')
        self.get_logger().info('=' * 50)
        
        # ========== Firebase 초기화 ==========
        if FIREBASE_AVAILABLE:
            self._init_firebase()
        
        # ========== 자동 Launch 실행 ==========
        if self._auto_launch:
            self.get_logger().info('🚀 시스템 자동 시작...')
            threading.Thread(target=self._initial_launch, daemon=True).start()
        else:
            # auto_launch=False면 외부에서 launch 실행했다고 가정
            # startup_delay 후에 system_ready 활성화
            self.get_logger().info(f'⏳ 외부 Launch 감지 모드 - {self._startup_delay}초 후 감시 시작')
            def enable_monitoring():
                time.sleep(self._startup_delay)
                self._system_ready = True
                self.get_logger().info('✅ 시스템 준비 완료 - 드라이버 감시 시작')
            threading.Thread(target=enable_monitoring, daemon=True).start()
    
    # =========================================
    # Firebase 관련 메서드
    # =========================================
    def _init_firebase(self):
        """Firebase 초기화"""
        try:
            # 이미 초기화되어 있는지 확인
            if firebase_admin._apps:
                self._firebase_initialized = True
                self.get_logger().info('💾 Firebase 이미 초기화됨')
                return
            
            # 인증 파일 경로
            cred_path = os.path.join(
                self._workspace_path, 'src', 'dsr_integrated', 'config',
                'serviceAccountKey.json'
            )
            
            if not os.path.exists(cred_path):
                self.get_logger().warn(f'⚠️ Firebase 인증 파일 없음: {cred_path}')
                return
            
            cred = credentials.Certificate(cred_path)
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://logistics-robot-dfb91-default-rtdb.firebaseio.com/'
            })
            
            self._firebase_initialized = True
            self.get_logger().info('💾 Firebase 초기화 완료')
            
        except Exception as e:
            self.get_logger().error(f'❌ Firebase 초기화 실패: {e}')
    
    def _read_recovery_state(self) -> dict:
        """Firebase에서 복구 상태 읽기"""
        if not self._firebase_initialized:
            return {}
        
        try:
            ref = db.reference('sort_status')
            data = ref.get()
            
            if data:
                self._last_recovery_state = data
                self.get_logger().info(f'💾 복구 상태 읽기: {json.dumps(data, indent=2)}')
                return data
            
        except Exception as e:
            self.get_logger().error(f'❌ 복구 상태 읽기 실패: {e}')
        
        return {}
    
    def _update_restart_count_firebase(self, count: int, reason: str):
        """Firebase에 재시작 카운트 업데이트"""
        if not self._firebase_initialized:
            return
        
        try:
            ref = db.reference('sort_status/recovery_state')
            ref.update({
                'system_restart_count': count,
                'last_crash_reason': reason,
                'last_restart_time': time.strftime('%Y-%m-%d %H:%M:%S')
            })
            self.get_logger().info(f'💾 Firebase 재시작 카운트 업데이트: {count}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Firebase 업데이트 실패: {e}')
    
    def _perform_recovery(self):
        """
        시스템 재시작 후 복구 수행
        - 복구 상태 읽기
        - 로봇 상태를 STANDBY로 전환 (노란불 → 녹색불)
        - gripping=True이면 그리퍼 열기
        - cycle_count 유지 (이미 Firebase에서 관리)
        """
        self.get_logger().info('🔧 복구 로직 시작...')
        
        # 1. 복구 상태 읽기
        recovery_state = self._read_recovery_state()
        
        if not recovery_state:
            self.get_logger().info('💾 복구 상태 없음 - 새로운 시작')
            # 상태 없어도 로봇 STANDBY 전환은 시도
            self._ensure_robot_standby()
            return
        
        # 2. 재시도 횟수 확인
        if self._retry_count >= self._max_retry_count:
            self.get_logger().error(f'🛑 최대 재시도 횟수 초과 ({self._max_retry_count}회) - 복구 중단')
            self._publish_system_status("RECOVERY_FAILED:MAX_RETRY")
            return
        
        self._retry_count += 1
        self.get_logger().info(f'🔁 재시도 {self._retry_count}/{self._max_retry_count}')
        
        # 3. 마지막 작업 상태 확인
        last_action = recovery_state.get('recovery_state', {}).get('last_action', 'idle')
        gripping = recovery_state.get('recovery_state', {}).get('gripping', False)
        cycle_count = recovery_state.get('cycle_count', 0)
        
        self.get_logger().info(f'📊 마지막 상태: action={last_action}, gripping={gripping}, cycle={cycle_count}')
        
        # 4. 로봇 상태를 STANDBY로 전환 (노란불 → 녹색불)
        self._ensure_robot_standby()
        
        # 5. gripping=True이면 그리퍼 열기 명령 발행
        if gripping:
            self.get_logger().warn('🖐️ 물체를 잡고 있었음 - 그리퍼 열기 요청')
            # 그리퍼 열기는 sort_node가 시작된 후에 수행해야 함
            # 여기서는 상태만 기록하고, sort_node에서 처리하도록 토픽 발행
            msg = String()
            msg.data = json.dumps({
                'action': 'open_gripper',
                'reason': 'recovery_restart',
                'last_action': last_action,
                'cycle_count': cycle_count
            })
            # 복구 명령 발행 (sort_node가 구독)
            self.pub_system_status.publish(String(data="RECOVERY:OPEN_GRIPPER"))
        
        # 6. 복구 완료
        self._publish_system_status(f"RECOVERY_DONE:cycle={cycle_count}")
        self.get_logger().info(f'✅ 복구 완료 - cycle_count={cycle_count}부터 이어서 작업')
    
    def _ensure_robot_standby(self):
        """
        로봇을 STANDBY 상태로 전환 (노란불 → 녹색불)
        
        LED 상태:
        - 노란색: 중단상태 또는 복구상태 (RECOVERY=9)
        - 녹색: 자동 모드 실행 대기 상태 (STANDBY=1)
        """
        if not DSR_AVAILABLE:
            self.get_logger().warn('⚠️ DSR 서비스 없음 - STANDBY 전환 불가')
            return
        
        self.get_logger().info('🔄 로봇 STANDBY 전환 시도 (노란불 → 녹색불)...')
        
        try:
            from dsr_msgs2.srv import SetRobotControl, SetSafetyMode
            
            # 서비스 클라이언트 생성 (없으면)
            if not hasattr(self, 'cli_control_recovery'):
                self.cli_control_recovery = self.create_client(
                    SetRobotControl,
                    '/dsr01/system/set_robot_control',
                    callback_group=self.callback_group
                )
                self.cli_safety_recovery = self.create_client(
                    SetSafetyMode,
                    '/dsr01/system/set_safety_mode',
                    callback_group=self.callback_group
                )
            
            # 서비스 대기
            if not self.cli_control_recovery.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('⚠️ SetRobotControl 서비스 없음')
                return
            
            # STANDBY 전환 시퀀스 (최대 3회 시도)
            for attempt in range(3):
                self.get_logger().info(f'  → STANDBY 전환 시도 {attempt+1}/3')
                
                # 1. RECOVERY 모드 해제 (control=7)
                req = SetRobotControl.Request()
                req.robot_control = 7  # CTRL_RESET_RECOVERY
                future = self.cli_control_recovery.call_async(req)
                time.sleep(0.5)
                
                # 2. 서보 ON (control=3)
                req = SetRobotControl.Request()
                req.robot_control = 3  # CTRL_SERVO_ON
                future = self.cli_control_recovery.call_async(req)
                time.sleep(0.5)
                
                # 3. 상태 확인
                if self.cli_get_state and self.cli_get_state.service_is_ready():
                    req = GetRobotState.Request()
                    future = self.cli_get_state.call_async(req)
                    time.sleep(0.5)
                    
                    if future.done() and future.result():
                        state = future.result().robot_state
                        state_names = {
                            0: 'INIT', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF',
                            4: 'TEACHING', 5: 'SAFE_STOP2', 6: 'SAFE_STOP',
                            7: 'EMERGENCY', 8: 'HOMMING', 9: 'RECOVERY'
                        }
                        state_name = state_names.get(state, f'UNKNOWN({state})')
                        
                        if state == 1:  # STANDBY
                            self.get_logger().info(f'✅ 로봇 STANDBY 전환 성공! (녹색불)')
                            return
                        else:
                            self.get_logger().warn(f'  → 현재 상태: {state_name}, 재시도...')
                
                time.sleep(1.0)
            
            self.get_logger().warn('⚠️ STANDBY 전환 실패 - 수동 개입 필요할 수 있음')
            
        except Exception as e:
            self.get_logger().error(f'❌ STANDBY 전환 예외: {e}')
    
    def _initial_launch(self):
        """초기 launch 실행 (스레드)"""
        time.sleep(2.0)  # 노드 초기화 완료 대기
        self._start_system()
    
    def _check_driver(self):
        """드라이버 상태 체크"""
        # 시스템 준비 안됐으면 스킵
        if not self._system_ready:
            return
        
        if not DSR_AVAILABLE or self.cli_get_state is None:
            return
        
        # 서비스 존재 확인
        if not self.cli_get_state.service_is_ready():
            self._handle_failure("서비스 없음")
            return
        
        # 서비스 호출
        try:
            req = GetRobotState.Request()
            future = self.cli_get_state.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.done() and future.result() is not None:
                self._handle_success(future.result().robot_state)
            else:
                self._handle_failure("응답 타임아웃")
        except Exception as e:
            self._handle_failure(f"예외: {e}")
    
    def _handle_success(self, robot_state: int):
        """드라이버 정상 응답"""
        if not self._driver_alive:
            self.get_logger().info('✅ 드라이버 연결 확인!')
            self._publish_status("RECOVERED")
        
        self._driver_alive = True
        self._consecutive_failures = 0
        
        state_names = {
            0: 'INIT', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF',
            4: 'TEACHING', 5: 'SAFE_STOP2', 6: 'SAFE_STOP',
            7: 'EMERGENCY', 8: 'HOMMING', 9: 'RECOVERY'
        }
        state_name = state_names.get(robot_state, f'UNKNOWN({robot_state})')
        self._publish_status(f"ALIVE:{state_name}")
    
    def _handle_failure(self, reason: str):
        """드라이버 응답 실패"""
        self._consecutive_failures += 1
        
        self.get_logger().warn(
            f'⚠️ 드라이버 응답 실패 ({self._consecutive_failures}/{self._max_failures}): {reason}'
        )
        
        if self._consecutive_failures >= self._max_failures:
            if self._driver_alive:
                self.get_logger().error('🔴 드라이버 죽음 감지!')
                self._driver_alive = False
                self._notify_restart_needed()
                
                # 자동 재시작
                if self._auto_restart:
                    threading.Thread(target=self._restart_system, daemon=True).start()
            
            self._publish_status("DEAD")
        else:
            self._publish_status(f"UNSTABLE:{reason}")
    
    def _notify_restart_needed(self):
        """드라이버 재시작 필요 알림"""
        msg = Bool()
        msg.data = True
        self.pub_restart_needed.publish(msg)
    
    def _handle_restart_request(self, request, response):
        """수동 재시작 서비스"""
        self.get_logger().info('🔄 수동 재시작 요청')
        threading.Thread(target=self._restart_system, daemon=True).start()
        response.success = True
        response.message = '시스템 재시작 시작됨'
        return response
    
    def _handle_shutdown_request(self, request, response):
        """시스템 종료 서비스"""
        self.get_logger().info('🛑 시스템 종료 요청')
        self._kill_all_processes()
        response.success = True
        response.message = '시스템 종료됨'
        return response
    
    def _restart_system(self):
        """전체 시스템 재시작"""
        current_time = time.time()
        
        # 쿨다운 체크
        if current_time - self._last_restart_time < self._restart_cooldown:
            remaining = self._restart_cooldown - (current_time - self._last_restart_time)
            self.get_logger().warn(f'⏳ 쿨다운 중... {remaining:.1f}초 남음')
            return False
        
        self._last_restart_time = current_time
        self._restart_count += 1
        self._system_ready = False
        
        self.get_logger().error(f'🔄 시스템 재시작 #{self._restart_count}')
        self._publish_system_status(f"RESTARTING:#{self._restart_count}")
        
        # 0. Firebase에 재시작 기록
        if self._firebase_initialized:
            self._update_restart_count_firebase(
                self._restart_count, 
                "driver_death"
            )
        
        # 1. 완전 종료
        self._kill_all_processes()
        
        # 2. 대기
        self.get_logger().info('⏳ 프로세스 정리 대기 (5초)...')
        time.sleep(5.0)
        
        # 3. 새로 시작
        self._start_system()
        
        # 4. 복구 로직 수행 (시스템 시작 후)
        def do_recovery():
            time.sleep(self._startup_delay + self._recovery_delay)
            self._perform_recovery()
        
        threading.Thread(target=do_recovery, daemon=True).start()
        
        return True
    
    def _kill_all_processes(self):
        """모든 관련 프로세스 완전 종료"""
        self.get_logger().info('🛑 모든 프로세스 종료 중...')
        self._publish_system_status("KILLING")
        
        # 종료할 프로세스 패턴 목록
        kill_patterns = [
            # ROS2 노드들
            'dsr_bringup2',
            'dsr01',
            'web_server_node',
            'sort_node',
            'dlar_sort_node',
            'serial_to_topic',
            'arduino_conveyor',
            # Launch 관련
            'full_system.launch',
            'dsr_bringup2_rviz.launch',
            # RViz
            'rviz2',
            'rviz',
            # Robot State Publisher
            'robot_state_publisher',
            'joint_state_publisher',
        ]
        
        for pattern in kill_patterns:
            try:
                # SIGTERM으로 먼저 시도
                subprocess.run(
                    f'pkill -15 -f "{pattern}"',
                    shell=True, timeout=2, capture_output=True
                )
            except:
                pass
        
        time.sleep(1.0)
        
        # 남은 프로세스 강제 종료 (SIGKILL)
        for pattern in kill_patterns:
            try:
                subprocess.run(
                    f'pkill -9 -f "{pattern}"',
                    shell=True, timeout=2, capture_output=True
                )
            except:
                pass
        
        self.get_logger().info('🛑 프로세스 종료 완료')
    
    def _start_system(self):
        """시스템 시작"""
        self.get_logger().info('🚀 시스템 시작 중...')
        self._publish_system_status("STARTING")
        
        # Launch 명령
        launch_cmd = f'''
            cd {self._workspace_path} && \
            source /opt/ros/humble/setup.bash && \
            source install/setup.bash && \
            ros2 launch dsr_integrated full_system.launch.py
        '''
        
        try:
            self._launch_process = subprocess.Popen(
                ['bash', '-c', launch_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                bufsize=1,
                universal_newlines=True
            )
            
            self.get_logger().info(f'🚀 Launch 시작됨 (PID: {self._launch_process.pid})')
            
            # Launch 출력 로깅 스레드
            threading.Thread(
                target=self._log_launch_output,
                daemon=True
            ).start()
            
            # startup_delay 후 체크 시작
            self.get_logger().info(f'⏳ {self._startup_delay}초 후 드라이버 체크 시작...')
            
            def enable_check():
                time.sleep(self._startup_delay)
                self._system_ready = True
                self._consecutive_failures = 0
                self.get_logger().info('✅ 시스템 준비 완료 - 드라이버 감시 시작')
                self._publish_system_status("READY")
            
            threading.Thread(target=enable_check, daemon=True).start()
            
        except Exception as e:
            self.get_logger().error(f'❌ Launch 실패: {e}')
            self._publish_system_status(f"FAILED:{e}")
    
    def _log_launch_output(self):
        """Launch 출력 로깅"""
        if self._launch_process is None:
            return
        
        try:
            for line in self._launch_process.stdout:
                line = line.strip()
                if line:
                    # 중요 메시지만 출력
                    if any(kw in line.lower() for kw in ['error', 'warn', 'fail', 'exception']):
                        self.get_logger().warn(f'[LAUNCH] {line}')
        except:
            pass
    
    def _publish_status(self, status: str):
        """드라이버 상태 발행"""
        msg = String()
        msg.data = status
        self.pub_driver_status.publish(msg)
    
    def _publish_system_status(self, status: str):
        """시스템 상태 발행"""
        msg = String()
        msg.data = status
        self.pub_system_status.publish(msg)
        self.get_logger().info(f'📊 시스템 상태: {status}')


def main(args=None):
    rclpy.init(args=args)
    
    node = SystemManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 요청...')
        node._kill_all_processes()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
