#!/usr/bin/env python3
"""
충돌 복구 모듈 (단순화 버전)

복구 시퀀스 (원본 recovery.py 기반):
    1. SAFE_STOP 리셋 (control=2)
    2. RECOVERY ENTER (mode=2, event=0)
    3. Jog Z+ (바닥 충돌 시)
    4. RECOVERY COMPLETE (mode=2, event=2)
    5. RECOVERY 해제 (control=7)
    6. Servo ON (control=3)
    
드라이버 재시작 (서비스 응답 없을 때):
    1. 현재 런치 프로세스 종료
    2. 새 런치 프로세스 시작
    3. 서비스 연결 대기
    4. 홈 위치로 이동
"""

import time
import subprocess
import signal
import os
from typing import Callable, Optional

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from dsr_msgs2.srv import SetRobotControl, SetSafetyMode, Jog

from ..config.constants import (
    ROBOT_ID,
    CTRL_RESET_SAFE_STOP, CTRL_SERVO_ON, CTRL_RESET_RECOVERY,
    RECOVERY_Z_THRESHOLD, RECOVERY_JOG_TIME, RECOVERY_JOG_SPEED, RECOVERY_JOG_AXIS_Z,
    VELOCITY_MOVE, ACCEL_MOVE,
)
from ..config.positions import HOME_POSITION, CONVEY_START_POINT
from ..monitoring.state_monitor import RobotStateMonitor, state_name


class CollisionRecovery:
    """
    충돌 복구 클래스
    
    복구 시나리오:
    1. 그립 상태 (물체 잡고 있음):
       - 복구 → 컨베이어 시작점으로 이동 → Place → 홈으로 이동
       - 사이클 카운트 증가 안함
    2. 비그립 상태:
       - 복구 → 홈으로 직행
       - 사이클 카운트 증가 안함
    """
    
    def __init__(
        self, 
        node: Node, 
        state_monitor: RobotStateMonitor,
        callback_group: ReentrantCallbackGroup = None,
        robot_controller = None
    ):
        self.node = node
        self.state_monitor = state_monitor
        self.callback_group = callback_group
        self.robot = robot_controller  # RobotController 인스턴스
        
        # 서비스 클라이언트
        self._init_clients()
        
        # 복구 상태
        self._is_recovering = False
        self._saved_work_state = None
        self._recovery_caused_by_collision = False  # 충돌로 인한 복구인지
        
        # 콜백
        self._on_progress: Optional[Callable[[str, int], None]] = None
        self._on_complete: Optional[Callable[[bool, bool], None]] = None  # (success, was_gripping)
        
        self.node.get_logger().info('[Recovery] 초기화 완료')
    
    def _init_clients(self):
        """서비스 클라이언트 초기화"""
        prefix = f'/{ROBOT_ID}'
        
        self.cli_control = self.node.create_client(
            SetRobotControl, f'{prefix}/system/set_robot_control',
            callback_group=self.callback_group
        )
        self.cli_safety = self.node.create_client(
            SetSafetyMode, f'{prefix}/system/set_safety_mode',
            callback_group=self.callback_group
        )
        self.cli_jog = self.node.create_client(
            Jog, f'{prefix}/motion/jog',
            callback_group=self.callback_group
        )
    
    # =========================================
    # 콜백 설정
    # =========================================
    def set_progress_callback(self, callback: Callable[[str, int], None]):
        """진행 상태 콜백 설정"""
        self._on_progress = callback
    
    def set_complete_callback(self, callback: Callable[[bool], None]):
        """완료 콜백 설정"""
        self._on_complete = callback
    
    def _notify_progress(self, message: str, percent: int):
        """진행 상태 알림"""
        self.node.get_logger().info(f'[Recovery] {message} ({percent}%)')
        if self._on_progress:
            self._on_progress(message, percent)
    
    # =========================================
    # 서비스 호출
    # =========================================
    def _call_control(self, control_code: int) -> bool:
        """SetRobotControl 서비스 호출"""
        if not self.cli_control.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('[Recovery] SetRobotControl 서비스 없음')
            return False
        
        req = SetRobotControl.Request()
        req.robot_control = control_code
        
        future = self.cli_control.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.05)
        
        if future.done() and future.result():
            return future.result().success
        return False
    
    def _call_safety(self, mode: int, event: int) -> bool:
        """SetSafetyMode 서비스 호출"""
        if not self.cli_safety.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('[Recovery] SetSafetyMode 서비스 없음')
            return False
        
        req = SetSafetyMode.Request()
        req.safety_mode = mode
        req.safety_event = event
        
        future = self.cli_safety.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.05)
        
        if future.done() and future.result():
            return future.result().success
        return False
    
    def _call_jog(self, axis: int, speed: float, duration: float) -> bool:
        """Jog 서비스 호출 (시작 → 대기 → 정지)"""
        if not self.cli_jog.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('[Recovery] Jog 서비스 없음')
            return False
        
        # Jog 시작
        req = Jog.Request()
        req.jog_axis = axis
        req.move_reference = 0  # BASE
        req.speed = speed
        
        future = self.cli_jog.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 2.0:
            time.sleep(0.05)
        
        if not (future.done() and future.result() and future.result().success):
            return False
        
        # 대기
        time.sleep(duration)
        
        # Jog 정지
        req.speed = 0.0
        future = self.cli_jog.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 2.0:
            time.sleep(0.05)
        
        return True
    
    # =========================================
    # 복구 단계
    # =========================================
    def reset_safe_stop(self) -> bool:
        """1단계: SAFE_STOP 리셋"""
        self._notify_progress('SAFE_STOP 리셋', 10)
        result = self._call_control(CTRL_RESET_SAFE_STOP)
        time.sleep(0.5)
        return result
    
    def enter_recovery(self) -> bool:
        """2단계: RECOVERY 모드 진입"""
        self._notify_progress('복구 모드 진입', 25)
        result = self._call_safety(2, 0)  # mode=2 (RECOVERY), event=0 (ENTER)
        time.sleep(0.3)
        return result
    
    def jog_up(self) -> bool:
        """3단계: Z축 상승 (바닥 충돌 시)"""
        self._notify_progress('Z축 상승', 50)
        result = self._call_jog(RECOVERY_JOG_AXIS_Z, RECOVERY_JOG_SPEED, RECOVERY_JOG_TIME)
        time.sleep(0.3)
        return result
    
    def complete_recovery(self) -> bool:
        """4단계: RECOVERY 완료"""
        self._notify_progress('복구 완료 처리', 70)
        result = self._call_safety(2, 2)  # mode=2 (RECOVERY), event=2 (COMPLETE)
        time.sleep(0.5)
        return result
    
    def exit_recovery(self) -> bool:
        """5단계: RECOVERY 모드 해제"""
        self._notify_progress('복구 모드 종료', 85)
        result = self._call_control(CTRL_RESET_RECOVERY)
        time.sleep(0.5)
        return result
    
    def servo_on(self) -> bool:
        """6단계: 서보 ON"""
        self._notify_progress('서보 ON', 95)
        result = self._call_control(CTRL_SERVO_ON)
        time.sleep(1.0)
        return result
    
    def _move_to_home(self) -> bool:
        """홈 위치로 이동 (복구 완료 후)"""
        if self.robot is None:
            self.node.get_logger().warn('[Recovery] robot_controller가 없어서 홈 이동 불가')
            return False
        
        try:
            self.node.get_logger().info('[Recovery] 홈 위치로 이동 시작...')
            success = self.robot.movel(HOME_POSITION, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
            
            if success:
                self.node.get_logger().info('[Recovery] 홈 위치 도착')
            else:
                self.node.get_logger().warn('[Recovery] 홈 이동 실패')
            
            return success
        except Exception as e:
            self.node.get_logger().error(f'[Recovery] 홈 이동 예외: {e}')
            return False
    
    def _place_and_go_home(self) -> bool:
        """
        그립 상태에서 복구: 컨베이어 시작점에 물체 내려놓고 홈으로 이동
        
        Returns:
            성공 여부
        """
        if self.robot is None:
            self.node.get_logger().warn('[Recovery] robot_controller가 없어서 place 불가')
            return False
        
        try:
            # 1. 먼저 안전 높이로 올리기 (현재 위치에서 Z+100)
            self.node.get_logger().info('[Recovery] 안전 높이로 상승...')
            current_pos = self.robot.get_current_posx()
            if current_pos:
                safe_pos = list(current_pos)
                safe_pos[2] = max(safe_pos[2], HOME_POSITION[2])  # HOME Z 높이로
                self.robot.movel(safe_pos, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
            
            # 2. 컨베이어 시작점 위로 이동 (안전 높이 유지)
            self._notify_progress('컨베이어 위치로 이동 중...', 70)
            approach_pos = CONVEY_START_POINT.copy()
            approach_pos[2] = HOME_POSITION[2]  # 안전 높이
            self.robot.movel(approach_pos, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
            
            # 3. 컨베이어 시작점으로 하강
            self._notify_progress('물체 내려놓기...', 80)
            self.robot.movel(CONVEY_START_POINT, vel=VELOCITY_MOVE/2, acc=ACCEL_MOVE/2)
            
            # 4. 그리퍼 열기 (물체 내려놓기)
            self.robot.grip_open()
            time.sleep(0.5)
            
            # 5. 위로 복귀
            self.robot.movel(approach_pos, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
            
            # 6. 그리퍼 닫기
            self.robot.grip_close()
            
            # 7. 홈으로 이동
            self._notify_progress('홈 위치로 이동 중...', 90)
            success = self.robot.movel(HOME_POSITION, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
            
            if success:
                self.node.get_logger().info('[Recovery] ✅ 물체 반납 후 홈 도착')
            
            return success
            
        except Exception as e:
            self.node.get_logger().error(f'[Recovery] place_and_go_home 예외: {e}')
            return False
    
    # =========================================
    # 자동 복구
    # =========================================
    @property
    def is_recovering(self) -> bool:
        return self._is_recovering
    
    def save_work_state(self, state: dict):
        """작업 상태 저장 (복구 후 이어서 하기 위해)"""
        self._saved_work_state = state
        self.node.get_logger().info(f'[Recovery] 작업 상태 저장: {state}')
    
    def get_saved_work_state(self) -> Optional[dict]:
        """저장된 작업 상태 반환"""
        return self._saved_work_state
    
    def clear_saved_work_state(self):
        """저장된 작업 상태 삭제"""
        self._saved_work_state = None
    
    def auto_recover(self, max_attempts: int = 3) -> bool:
        """
        자동 복구 시퀀스 실행
        
        복구 시나리오:
        1. 그립 상태 → 컨베이어 시작점에 물체 반납 → 홈 이동 (사이클 카운트 X)
        2. 비그립 상태 → 홈 직행 (사이클 카운트 X)
        
        Returns:
            복구 성공 여부
        """
        if self._is_recovering:
            self.node.get_logger().warn('[Recovery] 이미 복구 중')
            return False
        
        self._is_recovering = True
        self._recovery_caused_by_collision = True  # 충돌로 인한 복구 표시
        success = False
        was_gripping = False
        
        try:
            # ===== 그립 상태 확인 (복구 전) =====
            if self.robot:
                was_gripping = self.robot.is_gripping()
                grip_status = "🔴 물체 잡고 있음" if was_gripping else "⚪ 빈 손"
                self.node.get_logger().info(f'[Recovery] 그립 상태: {grip_status}')
            
            # 현재 Z 높이 확인 (바닥 충돌 판단용)
            current_z = self.state_monitor.get_current_z()
            needs_jog = current_z is not None and current_z < RECOVERY_Z_THRESHOLD
            
            z_str = f'{current_z:.1f}mm' if current_z else 'N/A'
            case_type = '바닥 충돌' if needs_jog else '외부 충돌'
            
            self.node.get_logger().info('=' * 50)
            self.node.get_logger().info(f'[Recovery] 자동 복구 시작 - {case_type}, Z={z_str}')
            if was_gripping:
                self.node.get_logger().info('[Recovery] → 물체 반납 후 홈으로 이동 예정')
            else:
                self.node.get_logger().info('[Recovery] → 홈으로 직행 예정')
            self.node.get_logger().info('=' * 50)
            
            for attempt in range(max_attempts):
                self.node.get_logger().info(f'[Recovery] 시도 {attempt + 1}/{max_attempts}')
                
                # 현재 상태 확인
                state = self.state_monitor.get_robot_state()
                self.node.get_logger().info(f'[Recovery] 현재 상태: {state_name(state)}')
                
                # 이미 STANDBY면 성공
                if self.state_monitor.is_standby(state):
                    self.node.get_logger().info('✅ [Recovery] 이미 STANDBY!')
                    success = True
                    break
                
                # 1. SAFE_STOP 리셋
                if self.state_monitor.is_safe_stop(state):
                    if not self.reset_safe_stop():
                        continue
                
                # 2. RECOVERY 진입
                self.enter_recovery()
                
                # 3. Jog Z+ (바닥 충돌 시)
                if needs_jog:
                    self.jog_up()
                else:
                    self._notify_progress('Jog 생략', 50)
                
                # 4. RECOVERY 완료
                self.complete_recovery()
                
                # 5. RECOVERY 해제
                self.exit_recovery()
                
                # 6. 서보 ON
                state = self.state_monitor.get_robot_state()
                self.node.get_logger().info(f'[Recovery] 해제 후 상태: {state_name(state)}')
                
                if not self.state_monitor.is_standby(state):
                    self.servo_on()
                
                # 결과 확인
                state = self.state_monitor.get_robot_state()
                if self.state_monitor.is_standby(state):
                    self.node.get_logger().info('✅ [Recovery] 상태 복구 성공!')
                    
                    # 서비스 안정화 대기
                    self.node.get_logger().info('[Recovery] 서비스 안정화 대기 (2초)...')
                    time.sleep(2.0)
                    
                    # ===== 그립 상태에 따른 분기 =====
                    if was_gripping:
                        # 그립 상태: 물체 반납 후 홈으로
                        self._notify_progress('물체 반납 및 홈 이동 중...', 75)
                        home_success = self._place_and_go_home()
                    else:
                        # 비그립 상태: 홈 직행
                        self._notify_progress('홈 위치로 이동 중...', 85)
                        home_success = self._move_to_home()
                    
                    if home_success:
                        self._notify_progress('복구 완료', 100)
                        self.node.get_logger().info('✅ [Recovery] 홈 이동 완료 - 복구 100% 완료!')
                    else:
                        self._notify_progress('복구 완료 (홈 이동 실패)', 95)
                        self.node.get_logger().warn('⚠️ [Recovery] 홈 이동 실패 - 수동 홈 이동 필요')
                    
                    success = True
                    break
                
                self.node.get_logger().warn(f'[Recovery] 재시도... 상태: {state_name(state)}')
                time.sleep(0.5)
            
            if not success:
                self.node.get_logger().error('[Recovery] 복구 실패')
                self._notify_progress('복구 실패', 0)
            
        except Exception as e:
            self.node.get_logger().error(f'[Recovery] 예외: {e}')
            success = False
        
        finally:
            self._is_recovering = False
            if self._on_complete:
                # 콜백에 그립 상태 정보도 전달
                self._on_complete(success, was_gripping)
        
        return success
    
    @property
    def was_collision_recovery(self) -> bool:
        """마지막 복구가 충돌로 인한 것인지 반환 (사이클 카운트 스킵용)"""
        return self._recovery_caused_by_collision
    
    def clear_collision_flag(self):
        """충돌 복구 플래그 클리어"""
        self._recovery_caused_by_collision = False

    # =========================================
    # 드라이버 재시작 (서비스 응답 없을 때)
    # =========================================
    def restart_driver(self, on_restart_complete: Optional[Callable] = None) -> bool:
        """
        DSR 드라이버 재시작
        
        주의: 이 함수는 현재 런치 파일의 DSR 관련 노드만 재시작합니다.
              전체 시스템을 재시작하려면 런치 파일을 다시 실행해야 합니다.
        
        Args:
            on_restart_complete: 재시작 완료 콜백 (성공 여부 전달)
            
        Returns:
            재시작 시도 성공 여부
        """
        self.node.get_logger().warn('=' * 60)
        self.node.get_logger().warn('🔄 [Recovery] DSR 드라이버 재시작 시도')
        self.node.get_logger().warn('=' * 60)
        
        self._notify_progress('드라이버 재시작 중...', 10)
        
        try:
            # 1. DSR 관련 노드 종료 시도
            self.node.get_logger().info('[Recovery] DSR 노드 종료 시도...')
            
            # ros2 node list에서 dsr 관련 노드 찾기
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            
            dsr_nodes = [n.strip() for n in result.stdout.split('\n') 
                        if 'dsr' in n.lower() or 'controller_manager' in n.lower()]
            
            self.node.get_logger().info(f'[Recovery] 발견된 DSR 노드: {dsr_nodes}')
            
            # 2. 노드 강제 종료는 위험하므로 안내 메시지만 출력
            self.node.get_logger().error('=' * 60)
            self.node.get_logger().error('❌ [Recovery] 자동 드라이버 재시작 불가')
            self.node.get_logger().error('   DSR 드라이버는 ros2_control의 일부로 실행되어')
            self.node.get_logger().error('   개별 재시작이 어렵습니다.')
            self.node.get_logger().error('')
            self.node.get_logger().error('📋 수동 복구 절차:')
            self.node.get_logger().error('   1. 터미널에서 Ctrl+C로 현재 런치 종료')
            self.node.get_logger().error('   2. 로봇 상태 확인 (물리적 안전)')
            self.node.get_logger().error('   3. 런치 파일 다시 실행:')
            self.node.get_logger().error('      ros2 launch dsr_integrated full_system.launch.py')
            self.node.get_logger().error('=' * 60)
            
            self._notify_progress('수동 재시작 필요', 0)
            
            if on_restart_complete:
                on_restart_complete(False)
            
            return False
            
        except Exception as e:
            self.node.get_logger().error(f'[Recovery] 드라이버 재시작 실패: {e}')
            self._notify_progress('드라이버 재시작 실패', 0)
            
            if on_restart_complete:
                on_restart_complete(False)
            
            return False
    
    def check_driver_health(self) -> bool:
        """
        드라이버 건강 상태 확인
        
        Returns:
            True: 드라이버 정상
            False: 드라이버 응답 없음
        """
        # StateMonitor의 드라이버 상태 확인
        return self.state_monitor.is_driver_alive
