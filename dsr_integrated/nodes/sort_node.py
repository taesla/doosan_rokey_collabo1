#!/usr/bin/env python3
"""
물류 분류 로봇 노드 (DLAR Sort Node) - Refactored
- ROS2 서비스 클라이언트를 통해 DSR 로봇 제어
- 컨베이어 연동 자동 분류
- Compliance Control + Force 센서 기반 높이 측정
- YAML 설정 파일 기반 구성
- robot_pick_node/dlar_sort 기반 9사이클 적재 로직
- 충돌 감지 및 자동 복구
"""

import json
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Bool

# YAML 설정 로더
from ..config.yaml_loader import get_config

# 상수 임포트 (YAML에 없는 것들만)
from ..config.constants import (
    PHASE_PICK, PHASE_PLACE,
    DR_BASE, DR_TOOL, DR_FC_MOD_ABS,
    STATE_STANDBY,
)
from ..config.positions import HOME_POSITION

from ..core.robot_controller import RobotController
from ..monitoring.state_manager import StateManager
from ..integration.conveyor import ConveyorHandler
from ..integration.firebase import FirebaseHandler
from ..tasks.pick_place import PickPlaceTask

# 충돌 복구 모듈
from ..monitoring import RobotStateMonitor
from ..safety import CollisionRecovery

# 1차 적재 사이클 수 (robot_pick_node 기준)
MAX_CYCLES = 9


class DlarSortNode(Node):
    """물류 분류 로봇 ROS2 노드 (리팩토링)"""
    
    def __init__(self):
        super().__init__('dlar_sort_node')
        self.get_logger().info('DLAR Sort Node 시작 (Refactored + YAML Config)')
        
        # 콜백 그룹 분리 - 서비스는 항상 응답할 수 있도록
        self.callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()  # 서비스 전용
        
        # YAML 설정 로드
        self.config = get_config()
        self._load_config_values()
        
        # 모듈 초기화
        self.robot = RobotController(self, self.callback_group)
        self.state = StateManager(self)
        self.firebase = FirebaseHandler(self)
        
        # 충돌 복구 모듈 초기화
        self.state_monitor = RobotStateMonitor(self, self.callback_group)
        self.recovery = CollisionRecovery(
            self, self.state_monitor, self.callback_group,
            robot_controller=self.robot  # 홈 이동용
        )
        self._setup_recovery_callbacks()
        
        # Pick/Place 태스크 초기화 (BaseTask 인자 추가)
        self.pick_place = PickPlaceTask(
            self, self.robot, self.state, self.firebase, self.config,
            state_monitor=self.state_monitor,
            recovery_checker=lambda: self.recovery.is_recovering
        )
        
        # 컨베이어 핸들러 (감지 콜백 연결)
        self.conveyor = ConveyorHandler(
            self,
            self.callback_group,
            on_detect=self._on_conveyor_detect
        )
        
        # ★ Place 완료 시 컨베이어 즉시 재시작 콜백 등록
        self.pick_place.on_place_complete = self._on_place_complete
        
        # ROS2 서비스 생성
        self._create_services()
        
        # 상태 발행 토픽
        self.pub_status = self.create_publisher(String, '/dlar/status', 10)
        self.pub_running = self.create_publisher(Bool, '/dlar/is_running', 10)
        self.pub_recovery = self.create_publisher(String, '/dlar/recovery_status', 10)
        
        # 타이머
        self.status_timer = self.create_timer(0.5, self._publish_status)
        self.check_dsr_timer = self.create_timer(2.0, self._check_dsr_connection)
        
        self.get_logger().info('DLAR Sort Node 준비 완료')
        self.get_logger().info('  서비스: /dlar/start_sort, /dlar/stop_sort, /dlar/pause_sort')
        self.get_logger().info('  서비스: /dlar/reset_state, /dlar/conveyor_mode, /dlar/collision_recovery')
    
    def _load_config_values(self):
        """YAML 설정값 로드"""
        # 위치 설정
        self.HOME_POSITION = self.config.get_home_position()
        self.PICK_POSITION = self.config.get_pick_position()
        
        # Force 설정
        force = self.config.get_force_settings()
        self.FORCE_THRESHOLD = force['threshold']
        self.FORCE_PUSH = force['push_force']
        self.MAX_DESCENT = force['max_descent']
        
        # 오프셋 설정
        offsets = self.config.get_offsets()
        self.UP_OFFSET = offsets['up']
        self.GRIP_OFFSET = offsets['grip']
        self.PICK_EXTRA_DOWN = offsets['pick_extra_down']
        self.PLACE_EXTRA_DOWN = offsets['place_extra_down']
        self.FINAL_PUSH = offsets['final_push']
        self.PLACE_REDUCE = offsets['place_reduce']
        
        # 컨베이어 설정
        conveyor = self.config.get_conveyor_settings()
        self.CONVEYOR_HEIGHT_OFFSET = conveyor['height_offset']
        
        # 안전 한계
        safety = self.config.get_safety_limits()
        self.SAFE_Z_PICK = safety['z_pick']
        self.SAFE_Z_PLACE = safety['z_place']
        
        # 모션 설정
        motion = self.config.get_motion_settings()
        self.VELOCITY_MOVE = motion['velocity_move']
        self.ACCEL_MOVE = motion['accel_move']
        self.VELOCITY_PICK = motion['velocity_pick']
        self.ACCEL_PICK = motion['accel_pick']
        
        self.get_logger().info('YAML 설정 로드 완료')
    
    def _create_services(self):
        """ROS2 서비스 생성 - 서비스 전용 콜백 그룹 사용"""
        self.srv_start = self.create_service(
            Trigger, '/dlar/start_sort',
            self._start_sort_callback,
            callback_group=self.service_callback_group
        )
        self.srv_stop = self.create_service(
            Trigger, '/dlar/stop_sort',
            self._stop_sort_callback,
            callback_group=self.service_callback_group
        )
        self.srv_pause = self.create_service(
            SetBool, '/dlar/pause_sort',
            self._pause_sort_callback,
            callback_group=self.service_callback_group
        )
        self.srv_reset = self.create_service(
            Trigger, '/dlar/reset_state',
            self._reset_state_callback,
            callback_group=self.service_callback_group
        )
        self.srv_conveyor_mode = self.create_service(
            SetBool, '/dlar/conveyor_mode',
            self._conveyor_mode_callback,
            callback_group=self.service_callback_group
        )
        
        # 충돌 복구 서비스
        self.srv_collision_recovery = self.create_service(
            Trigger, '/dlar/collision_recovery',
            self._collision_recovery_callback,
            callback_group=self.service_callback_group
        )
    
    # =========================================
    # 타이머 콜백
    # =========================================
    def _publish_status(self):
        """상태 발행"""
        self.state.state.dsr_ready = self.robot.is_ready
        self.state.state.conveyor_detected = self.conveyor.is_detected
        
        status = self.state.get_status_dict()
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)
        
        running_msg = Bool()
        running_msg.data = self.state.state.is_running
        self.pub_running.publish(running_msg)
    
    def _check_dsr_connection(self):
        """DSR 연결 상태 확인"""
        self.robot.check_connection()
    
    # =========================================
    # 서비스 콜백
    # =========================================
    def _start_sort_callback(self, request, response):
        """분류 작업 시작"""
        if self.state.state.is_running:
            response.success = False
            response.message = '이미 실행 중입니다'
            return response
        
        if not self.robot.is_ready:
            response.success = False
            response.message = 'DSR 로봇이 준비되지 않았습니다'
            return response
        
        # 복구 중인지 확인
        if self.recovery.is_recovering:
            response.success = False
            response.message = '복구 진행 중입니다. 완료 후 시도하세요.'
            self.get_logger().warn('[SORT] 분류 시작 거부 - 복구 진행 중')
            return response
        
        # 로봇 상태가 STANDBY인지 확인
        current_state = self.state_monitor.get_robot_state()
        if current_state != 1:  # 1 = STANDBY
            response.success = False
            response.message = f'로봇이 준비 상태가 아닙니다 (현재: {current_state})'
            self.get_logger().warn(f'[SORT] 분류 시작 거부 - 로봇 상태: {current_state}')
            return response
        
        self.state.start()
        
        # 별도 스레드에서 분류 작업 실행
        thread = threading.Thread(target=self._run_sort_loop, daemon=True)
        thread.start()
        
        response.success = True
        response.message = '분류 작업 시작'
        self.get_logger().info('분류 작업 시작')
        return response
    
    def _stop_sort_callback(self, request, response):
        """분류 작업 비상정지 (즉시 정지, 이어서 재개 가능)"""
        # 1. 로봇 즉시 정지 (MoveStop)
        self.robot.stop_motion()
        
        # 2. Force Control 해제 (Force 모니터링 중일 수 있음)
        self.robot.release_force()
        self.robot.disable_compliance()
        
        # 3. 비상정지 상태로 전환 (루프 종료 아님, 대기 상태)
        self.state.emergency_stop()
        self.get_logger().info(f'[EMERGENCY STOP] 로봇 정지 - 이어서 재개 대기 중')
        
        response.success = True
        response.message = '비상정지 활성화'
        return response
    
    def _pause_sort_callback(self, request, response):
        """분류 작업 일시정지/재개 (비상정지 해제도 포함)"""
        self.get_logger().info(f'📩 pause_sort 콜백 진입: data={request.data}')
        if request.data:
            # 일시정지 (pause=true)
            self.robot.stop_motion()
            self.state.pause()
            self.get_logger().info('[PAUSE] 로봇 정지 및 상태 저장됨')
            response.message = '일시정지'
        else:
            # 재개 (pause=false) - 비상정지 해제도 포함
            if self.state.is_emergency_stopped():
                self.state.emergency_release()
                self.get_logger().info('[RESUME] 비상정지 해제 - 작업 재개')
            else:
                self.state.resume()
                self.get_logger().info('[RESUME] 작업 재개')
            response.message = '재개'
        
        response.success = True
        return response
    
    def _reset_state_callback(self, request, response):
        """상태 초기화"""
        self.state.reset()
        response.success = True
        response.message = '상태 초기화 완료'
        self.get_logger().info('상태 초기화')
        return response

    def _collision_recovery_callback(self, request, response):
        """수동 충돌 복구 서비스"""
        if self.recovery.is_recovering:
            response.success = False
            response.message = '이미 복구 진행 중'
            return response
        
        self.get_logger().info('[RECOVERY] 수동 복구 요청')
        
        # 현재 작업 상태 저장
        work_state = self._get_current_work_state()
        
        # 별도 스레드에서 복구 실행
        thread = threading.Thread(
            target=self._execute_recovery,
            args=(work_state,),
            daemon=True
        )
        thread.start()
        
        response.success = True
        response.message = '복구 시작'
        return response

    # =========================================
    # 충돌 복구 관련
    # =========================================
    def _setup_recovery_callbacks(self):
        """복구 모듈 콜백 설정"""
        # 충돌 감지 콜백
        self.state_monitor.set_collision_callback(self._on_collision_detected)
        # 드라이버 죽음 콜백
        self.state_monitor.set_driver_dead_callback(self._on_driver_dead)
        # ★ 드라이버 복구 콜백 (하트비트 기반 자동 복구)
        self.state_monitor.set_driver_recovered_callback(self._on_driver_recovered)
        # 복구 진행 콜백 (Web UI용)
        self.recovery.set_progress_callback(self._on_recovery_progress)
        # 복구 완료 콜백
        self.recovery.set_complete_callback(self._on_recovery_complete)
        # 상태 모니터링 시작
        self.state_monitor.start_monitoring()
    
    def _on_driver_dead(self):
        """드라이버 죽음 감지 콜백"""
        self.get_logger().error('💀 [SORT] DSR 드라이버가 응답하지 않습니다!')
        
        # 비상정지 상태로 전환
        self.state.emergency_stop()
        
        # 작업 강제 종료
        self.state.request_stop()
        
        # Web UI에 알림
        self._publish_recovery_status(
            event='driver_dead',
            step='DSR 드라이버 응답 없음! 재시작 시도 중...',
            percent=10,
            success=False
        )
        
        # 드라이버 재시작 시도
        self.get_logger().warn('🔄 [SORT] 드라이버 재시작 시도...')
        self.recovery.restart_driver(on_restart_complete=self._on_driver_restart_complete)
    
    def _on_driver_restart_complete(self, success: bool):
        """드라이버 재시작 완료 콜백"""
        if success:
            self.get_logger().info('✅ [SORT] 드라이버 재시작 성공!')
            self._publish_recovery_status(
                event='driver_restarted',
                step='드라이버 재시작 성공! 홈 위치로 복귀 중...',
                percent=80,
                success=True
            )
            # TODO: 홈 위치로 이동 (드라이버 완전 초기화 후)
        else:
            self.get_logger().error('❌ [SORT] 드라이버 재시작 실패!')
            self._publish_recovery_status(
                event='driver_restart_failed',
                step='드라이버 재시작 실패! 수동 복구 필요',
                percent=0,
                success=False
            )
    
    def _on_driver_recovered(self):
        """
        ★ 하트비트 기반 자동 복구 콜백
        드라이버가 죽었다가 다시 살아났을 때 호출
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('💚 [SORT] 하트비트 기반 자동 복구 시작!')
        self.get_logger().info('=' * 60)
        
        # Web UI에 알림
        self._publish_recovery_status(
            event='driver_recovered',
            step='드라이버 복구 감지! 자동 복구 진행 중...',
            percent=50,
            success=True
        )
        
        # 별도 스레드에서 복구 진행
        def auto_recovery_sequence():
            try:
                # 1. 서비스 안정화 대기
                self.get_logger().info('[HEARTBEAT] 서비스 안정화 대기 (3초)...')
                time.sleep(3.0)
                
                # 2. 로봇 상태 확인
                state = self.state_monitor.get_robot_state()
                self.get_logger().info(f'[HEARTBEAT] 현재 로봇 상태: {state}')
                
                # 3. STANDBY가 아니면 충돌 복구 시퀀스 실행
                if state != STATE_STANDBY:
                    self.get_logger().info('[HEARTBEAT] STANDBY 아님 → 충돌 복구 시퀀스 실행')
                    self.recovery.auto_recover()
                else:
                    # 4. STANDBY면 바로 홈 이동
                    self.get_logger().info('[HEARTBEAT] STANDBY 상태 → 홈 위치로 이동')
                    self._publish_recovery_status(
                        event='driver_recovered',
                        step='홈 위치로 이동 중...',
                        percent=80,
                        success=True
                    )
                    
                    # 홈 이동
                    self.robot.grip_close()
                    success = self.robot.movel(HOME_POSITION, vel=self.VELOCITY_MOVE, acc=self.ACCEL_MOVE)
                    
                    if success:
                        self.get_logger().info('✅ [HEARTBEAT] 홈 이동 완료 - 자동 복구 성공!')
                        self._publish_recovery_status(
                            event='driver_recovered_complete',
                            step='자동 복구 완료! 작업 재개 가능',
                            percent=100,
                            success=True
                        )
                        
                        # 비상정지 해제
                        self.state.emergency_release()
                    else:
                        self.get_logger().warn('⚠️ [HEARTBEAT] 홈 이동 실패')
                        self._publish_recovery_status(
                            event='driver_recovered_partial',
                            step='홈 이동 실패 - 수동 확인 필요',
                            percent=90,
                            success=False
                        )
                        
            except Exception as e:
                self.get_logger().error(f'[HEARTBEAT] 자동 복구 오류: {e}')
                self._publish_recovery_status(
                    event='driver_recovered_failed',
                    step=f'자동 복구 오류: {e}',
                    percent=0,
                    success=False
                )
        
        threading.Thread(target=auto_recovery_sequence, daemon=True).start()
    
    def _get_current_work_state(self) -> dict:
        """현재 작업 상태 저장"""
        try:
            stack_counts = self.pick_place.get_summary()['stack_count'].copy()
        except Exception:
            stack_counts = {'SMALL': 0, 'MEDIUM': 0, 'LONG': 0}
        
        return {
            'cycle_count': self.state.state.cycle_count,
            'is_running': self.state.state.is_running,
            'phase': self.state.state.current_phase,
            'last_width_class': self.state.state.last_width_class,
            'stack_counts': stack_counts,
        }
    
    def _publish_recovery_status(self, event: str, step: str = '', percent: int = 0, success: bool = True):
        """복구 상태 발행 (Web UI 전달용)"""
        msg = String()
        data = {
            'event': event,  # 'detected', 'progress', 'complete'
            'step': step,
            'percent': percent,
            'success': success,
        }
        msg.data = json.dumps(data)
        self.pub_recovery.publish(msg)
    
    def _on_collision_detected(self):
        """충돌 감지 이벤트 핸들러"""
        # ★ 이미 복구 중이면 새 충돌 콜백 무시 (복구 완료 후에만 처리)
        if self.recovery.is_recovering:
            self.get_logger().info('[SORT] 🔄 복구 중 - 충돌 콜백 무시')
            return
        
        self.get_logger().warn('⚠️ [SORT] 충돌 감지됨!')
        
        # ★★★ 즉시 Force Control 해제 (pick_place와 충돌 방지) ★★★
        try:
            self.robot.release_force()
            self.robot.disable_compliance()
            self.get_logger().info('[SORT] Force Control 해제 완료')
        except Exception as e:
            self.get_logger().warn(f'[SORT] Force Control 해제 실패: {e}')
        
        # Web UI에 충돌 감지 알림
        self._publish_recovery_status('detected', '충돌 감지 - 자동 복구 시작', 0)
        
        # 비상정지 상태로 전환
        self.state.emergency_stop()
        
        # 현재 작업 상태 저장
        work_state = self._get_current_work_state()
        
        # 자동 복구 실행 (별도 스레드)
        thread = threading.Thread(
            target=self._execute_recovery,
            args=(work_state,),
            daemon=True
        )
        thread.start()
    
    def _execute_recovery(self, work_state: dict):
        """복구 실행"""
        try:
            # 작업 상태 저장 (나중에 이어서 하기 위해)
            self.recovery.save_work_state(work_state)
            
            success = self.recovery.auto_recover()
            if success:
                self.get_logger().info('✅ [SORT] 복구 성공 - 작업 재개 준비')
            else:
                self.get_logger().error('❌ [SORT] 복구 실패')
        except Exception as e:
            self.get_logger().error(f'[SORT] 복구 중 오류: {e}')
    
    def _on_recovery_progress(self, step: str, percent: int):
        """복구 진행 상태 (Web UI 전송용)"""
        self.get_logger().info(f'[RECOVERY] {step} ({percent}%)')
        # Web UI에 진행 상태 발행
        self._publish_recovery_status('progress', step, percent)
    
    def _on_recovery_complete(self, success: bool, was_gripping: bool = False):
        """
        복구 완료 이벤트
        
        Args:
            success: 복구 성공 여부
            was_gripping: 복구 전 그립 상태였는지 (사이클 카운트 스킵 판단용)
        """
        # Web UI에 완료 상태 발행
        if was_gripping:
            step_msg = '복구 완료 (물체 반납됨)' if success else '복구 실패'
        else:
            step_msg = '복구 완료' if success else '복구 실패'
        
        self._publish_recovery_status('complete', step_msg, 100 if success else 0, success)
        
        if success:
            # 비상정지 해제
            self.state.emergency_release()
            self.get_logger().info('✅ [SORT] 비상정지 해제 - 작업 재개 가능')
            
            # 저장된 작업 상태 확인 (사이클 카운트 스킵 플래그 설정)
            saved_state = self.recovery.get_saved_work_state()
            if saved_state:
                self.get_logger().info(f'[SORT] 저장된 상태: {saved_state}')
                # ★ 충돌 복구 시 사이클 카운트 스킵 표시
                if self.recovery.was_collision_recovery:
                    self.get_logger().info('[SORT] ⚠️ 충돌 복구로 인해 현재 사이클 카운트 스킵됨')
                self.recovery.clear_saved_work_state()
            
            # 충돌 복구 플래그 클리어
            self.recovery.clear_collision_flag()
        else:
            self.get_logger().warn('[SORT] 복구 실패 - 수동 개입 필요')

    def _conveyor_mode_callback(self, request, response):
        """컨베이어 자동 모드 설정"""
        self.state.set_conveyor_mode(request.data)
        
        if request.data:
            self.conveyor.send_resume()
            
            if self.robot.is_ready and not self.state.state.is_running:
                def move_home_and_wait():
                    self.get_logger().info('[CONVEYOR] HOME 위치로 이동')
                    self.robot.grip_close()
                    self.robot.movel(HOME_POSITION, vel=self.VELOCITY_MOVE, acc=self.ACCEL_MOVE)
                    self.state.set_waiting_for_object(True)
                    self.get_logger().info('[CONVEYOR] HOME 도착 - 물체 감지 대기 중')
                    
                    if self.conveyor.is_detected and self.state.state.conveyor_mode:
                        self._start_single_cycle()
                
                threading.Thread(target=move_home_and_wait, daemon=True).start()
            else:
                self.state.set_waiting_for_object(True)
            
            response.message = '컨베이어 자동 모드 활성화'
            self.get_logger().info('[CONVEYOR] 자동 모드 ON')
        else:
            response.message = '컨베이어 자동 모드 비활성화'
            self.get_logger().info('[CONVEYOR] 자동 모드 OFF')
        
        response.success = True
        return response
    
    # =========================================
    # 컨베이어 이벤트
    # =========================================
    def _on_conveyor_detect(self):
        """컨베이어 물체 감지 이벤트"""
        self.state.set_conveyor_detected(True)
        
        self.get_logger().info(
            f'[CONVEYOR] 상태: mode={self.state.state.conveyor_mode}, '
            f'waiting={self.state.state.waiting_for_object}, '
            f'running={self.state.state.is_running}'
        )
        
        if self.state.can_start_auto_cycle():
            self.get_logger().info('[CONVEYOR] ✅ 자동 분류 시작!')
            self._start_single_cycle()
    
    def _on_place_complete(self):
        """Place 완료 콜백 - 그리퍼 열자마자 컨베이어 재시작"""
        if self.state.state.conveyor_mode:
            self.get_logger().info('[CONVEYOR] ★ Place 완료 - 컨베이어 즉시 재시작!')
            self.conveyor.send_resume()
            self.state.set_conveyor_detected(False)  # 감지 상태 리셋
    
    def _start_single_cycle(self):
        """단일 분류 사이클 시작"""
        if self.state.state.is_running:
            return
        
        if not self.robot.is_ready:
            self.get_logger().warn('DSR 로봇이 준비되지 않았습니다')
            return
        
        self.state.start()
        self.state.set_waiting_for_object(False)
        
        thread = threading.Thread(target=self._run_single_cycle, daemon=True)
        thread.start()
    
    # =========================================
    # 분류 작업 로직
    # =========================================
    def _run_sort_loop(self):
        """
        분류 작업 메인 루프 - robot_pick_node 기반 9사이클 적재
        
        충돌 복구 시:
        - 그립 상태: 물체 반납 후 홈으로 → 현재 사이클 스킵 (카운트 안함)
        - 비그립 상태: 홈 직행 → 현재 사이클 재시도
        """
        self.state.load()
        home = HOME_POSITION.copy()
        
        # 카운트 리셋
        self.pick_place.reset_counts()
        
        self.robot.grip_close()
        self.get_logger().info('STEP: HOME 위치로 이동')
        self._movel_with_estop_check(home, vel=self.VELOCITY_MOVE, acc=self.ACCEL_MOVE)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"1차 적재 시작 (총 {MAX_CYCLES}개 박스)")
        self.get_logger().info("=" * 60)
        
        cycle_count = 0
        
        while cycle_count < MAX_CYCLES and self.state.state.is_running and not self.state.state.stop_requested:
            # 비상정지 상태 체크 - 비상정지 중이면 대기
            if self.state.is_emergency_stopped():
                time.sleep(0.1)
                continue
            
            # ★ 충돌 복구 후 체크: 복구가 완료되면 카운트 스킵 여부 확인
            if self.recovery.was_collision_recovery:
                # 복구 완료 - 충돌 플래그 클리어하고 다음 사이클로
                self.get_logger().info("[RECOVERY] 충돌 복구 완료 - 현재 사이클 스킵하고 다음 진행")
                self.recovery.clear_collision_flag()
                # 사이클 카운트 증가 없이 다음 루프로
                continue
            
            # 일시정지 대기
            if self.state.state.is_paused:
                time.sleep(0.1)
                continue
            
            cycle_count += 1
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"[CYCLE {cycle_count}/{MAX_CYCLES}] 시작")
            self.get_logger().info("=" * 60)
            
            try:
                # ===== PICK =====
                self.get_logger().info("[PHASE] PICK 단계 시작 (컨베이어)")
                pick_ok = self.pick_place.pick_and_measure()
                
                # 비상정지/충돌 체크
                if self.state.is_emergency_stopped():
                    cycle_count -= 1  # 사이클 재시도 (복구 후 다시 시도)
                    continue
                
                if not pick_ok:
                    self.get_logger().info("[SKIP] PICK 실패 감지 → 사이클 재시도")
                    self._movel_with_estop_check(home, vel=self.VELOCITY_MOVE, acc=self.ACCEL_MOVE)
                    cycle_count -= 1
                    continue
                
                # ===== PLACE =====
                self.get_logger().info("[PHASE] PLACE 단계 시작 (팔레트)")
                
                # 비상정지/충돌 체크
                if self.state.is_emergency_stopped():
                    cycle_count -= 1
                    continue
                
                width_class = self.robot.get_width_class()
                self.get_logger().info(f'[CYCLE] PLACE ({width_class})')
                self.pick_place.place_to_box(width_class)
                
                # 비상정지/충돌 체크
                if self.state.is_emergency_stopped():
                    # ★ PLACE 중 충돌: 물체는 이미 반납됐으므로 카운트만 유지
                    continue
                
                # 그리퍼 닫고 홈으로 복귀
                self.robot.grip_close()
                self._movel_with_estop_check(home, vel=self.VELOCITY_MOVE, acc=self.ACCEL_MOVE)
                
                # 상태 업데이트
                self.state.complete_cycle(width_class)
                
                summary = self.pick_place.get_summary()
                self.get_logger().info("=" * 60)
                self.get_logger().info(f"[CYCLE {cycle_count}/{MAX_CYCLES}] 완료")
                self.get_logger().info(
                    f"현재까지 적재: S={summary['stack_count']['SMALL']}, "
                    f"M={summary['stack_count']['MEDIUM']}, L={summary['stack_count']['LONG']} "
                    f"(총 {summary['total_boxes']}개)"
                )
                self.get_logger().info("=" * 60)
                
            except Exception as e:
                self.get_logger().error(f'분류 작업 오류: {e}')
                self.state.stats.add_error()
                if not self.state.is_emergency_stopped():
                    break
        
        # ===== 최종 요약 =====
        self._print_final_summary()
        
        self.state.finish()
        self.get_logger().info('분류 작업 종료')
    
    def _print_final_summary(self):
        """최종 적재 결과 출력"""
        summary = self.pick_place.get_summary()
        
        self.get_logger().info("")
        self.get_logger().info("#" * 60)
        self.get_logger().info("#" + " " * 58 + "#")
        self.get_logger().info("#" + "  1차 적재 완료 - 최종 결과".center(56) + "  #")
        self.get_logger().info("#" + " " * 58 + "#")
        self.get_logger().info("#" * 60)
        self.get_logger().info("")
        self.get_logger().info(f"  총 사이클 수: {self.state.state.cycle_count}회")
        self.get_logger().info(f"  총 적재 박스: {summary['total_boxes']}개")
        self.get_logger().info("")
        self.get_logger().info("  [분류별 개수]")
        self.get_logger().info(f"    - SMALL  : {summary['stack_count']['SMALL']}개")
        self.get_logger().info(f"    - MEDIUM : {summary['stack_count']['MEDIUM']}개")
        self.get_logger().info(f"    - LONG   : {summary['stack_count']['LONG']}개")
        self.get_logger().info("")
        self.get_logger().info("#" * 60)
    
    def _wait_for_estop_release(self):
        """비상정지 해제될 때까지 대기 (Event 기반 - 비블로킹)"""
        if self.state.is_emergency_stopped():
            self.get_logger().info('⏸️ 비상정지 대기 중...')
            # Event.wait()는 비블로킹 - 다른 스레드의 set() 호출로 깨어남
            while not self.state.wait_for_estop_release(timeout=0.5):
                if self.state.state.stop_requested:
                    return False  # 완전 중단 요청
            self.get_logger().info('▶️ 비상정지 해제 - 작업 재개')
        return True
    
    def _movel_with_estop_check(self, pos, vel, acc):
        """movel 실행 + 비상정지 시 대기"""
        # 비상정지 중이면 해제될 때까지 대기
        if not self._wait_for_estop_release():
            return False
        
        # 동기 movel 호출 (MoveStop으로 중단될 수 있음)
        result = self.robot.movel(pos, vel=vel, acc=acc)
        
        # movel 후 비상정지 상태 체크 - MoveStop으로 중단됐으면 해제될 때까지 대기
        if self.state.is_emergency_stopped():
            if not self._wait_for_estop_release():
                return False
            # 해제됐으면 다시 이동 시도
            result = self.robot.movel(pos, vel=vel, acc=acc)
        
        return result
    
    def _run_single_cycle(self):
        """단일 분류 사이클 실행 (컨베이어 자동 모드용)"""
        self.state.load()
        home = HOME_POSITION.copy()
        
        self.robot.grip_close()
        self._movel_with_estop_check(home, vel=self.VELOCITY_MOVE, acc=self.ACCEL_MOVE)
        
        try:
            # PICK
            self.get_logger().info('[SINGLE] PICK 단계')
            pick_ok = self.pick_place.pick_and_measure()
            
            if not pick_ok:
                self.get_logger().info('[SINGLE] PICK 실패 - 사이클 종료')
                self._movel_with_estop_check(home, vel=self.VELOCITY_MOVE, acc=self.ACCEL_MOVE)
                return
            
            # PLACE
            width_class = self.robot.get_width_class()
            self.get_logger().info(f'[SINGLE] PLACE ({width_class})')
            self.pick_place.place_to_box(width_class)
            
            # 완료
            self.robot.grip_close()
            self._movel_with_estop_check(home, vel=self.VELOCITY_MOVE, acc=self.ACCEL_MOVE)
            self.state.complete_cycle(width_class)
            
            summary = self.pick_place.get_summary()
            self.get_logger().info(
                f'[SINGLE] 완료 ({width_class}) - '
                f'S={summary["stack_count"]["SMALL"]}, '
                f'M={summary["stack_count"]["MEDIUM"]}, '
                f'L={summary["stack_count"]["LONG"]}'
            )
            
        except Exception as e:
            self.get_logger().error(f'단일 사이클 오류: {e}')
            self.state.stats.add_error()
        
        finally:
            # ★ waiting 상태를 finish() 전에 먼저 설정 (타이밍 이슈 방지)
            if self.state.state.conveyor_mode:
                self.state.set_waiting_for_object(True)
                self.get_logger().info('[CONVEYOR] 다음 물체 대기 상태 설정')
            
            self.state.finish()
            self.get_logger().info(f'[SINGLE] 사이클 종료 - conveyor_mode={self.state.state.conveyor_mode}')
            
            if self.state.state.conveyor_mode:
                # ★ 사이클 중에 이미 감지된 물체가 있으면 바로 다음 사이클 시작
                if self.conveyor.is_detected:
                    self.get_logger().info('[CONVEYOR] ★ 이미 감지된 물체 있음 - 즉시 다음 사이클!')
                    self._start_single_cycle()
                else:
                    self.get_logger().info('[CONVEYOR] 다음 물체 대기 중...')
            else:
                self.get_logger().warn('[CONVEYOR] conveyor_mode가 꺼져있어 재시작 안함')


def main(args=None):
    rclpy.init(args=args)
    node = DlarSortNode()
    
    # app_real.py와 동일한 패턴 - executor를 별도 스레드에서 실행
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    
    # executor를 별도 스레드에서 spin (서비스 콜백이 항상 처리됨)
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    node.get_logger().info('✅ Executor 스레드 시작됨')
    
    try:
        # 메인 스레드는 대기
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
