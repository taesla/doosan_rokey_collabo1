#!/usr/bin/env python3
"""
충돌 복구 모듈
- Case 1: 외부 힘 충돌 (손으로 밀림) → Jog 불필요
- Case 2: 바닥 충돌 (하강 중) → Jog Z+ 필요
- 블럭 잡고 있으면: 복구 후 컨베이어 시작점으로 이동 → 순응제어 하강 → 그리퍼 열기 → 홈 복귀

복구 시퀀스:
SAFE_STOP → reset_safe_stop → enter_recovery → [jog_up] → complete_recovery → exit_recovery → servo_on → STANDBY
→ [블럭 있으면] convey_start_point → 순응제어 하강 → grip_open → 상승 → HOME
"""

import time
from typing import Callable, Optional

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from dsr_msgs2.srv import (
    SetRobotControl, SetSafetyMode, Jog,
    MoveLine, TaskComplianceCtrl, ReleaseComplianceCtrl,
    SetCtrlBoxDigitalOutput,
)

from ..config.constants import (
    ROBOT_ID,
    CTRL_RESET_SAFE_STOP, CTRL_SERVO_ON, CTRL_RESET_RECOVERY,
    RECOVERY_Z_THRESHOLD, RECOVERY_JOG_TIME, RECOVERY_JOG_SPEED, RECOVERY_JOG_AXIS_Z,
    DR_BASE, DR_MV_MOD_ABS, VELOCITY_MOVE, ACCEL_MOVE,
    DEFAULT_STIFFNESS,
)
from ..config.positions import HOME_POSITION, CONVEY_START_POINT
from .state_monitor import RobotStateMonitor, state_name


class CollisionRecovery:
    """충돌 복구 클래스"""
    
    def __init__(
        self, 
        node: Node, 
        state_monitor: RobotStateMonitor,
        callback_group: ReentrantCallbackGroup = None
    ):
        """
        Args:
            node: ROS2 노드 인스턴스
            state_monitor: 상태 모니터 인스턴스
            callback_group: 콜백 그룹
        """
        self.node = node
        self.state_monitor = state_monitor
        self.callback_group = callback_group
        
        # 서비스 클라이언트
        self._init_clients()
        
        # 복구 상태
        self._is_recovering = False
        self._recovery_step = ""
        self._saved_work_state = None
        
        # 콜백
        self._on_progress_callback: Optional[Callable[[str, int], None]] = None
        self._on_complete_callback: Optional[Callable[[bool], None]] = None
        
        self.node.get_logger().info('[CollisionRecovery] 초기화 완료')
    
    def _init_clients(self):
        """서비스 클라이언트 초기화"""
        prefix = f'/{ROBOT_ID}'
        
        # 복구 관련
        self.cli_set_control = self.node.create_client(
            SetRobotControl, f'{prefix}/system/set_robot_control',
            callback_group=self.callback_group
        )
        self.cli_set_safety = self.node.create_client(
            SetSafetyMode, f'{prefix}/system/set_safety_mode',
            callback_group=self.callback_group
        )
        self.cli_jog = self.node.create_client(
            Jog, f'{prefix}/motion/jog',
            callback_group=self.callback_group
        )
        
        # 모션 관련 (블럭 반납용)
        self.cli_move_line = self.node.create_client(
            MoveLine, f'{prefix}/motion/move_line',
            callback_group=self.callback_group
        )
        
        # 순응제어 관련
        self.cli_task_compliance = self.node.create_client(
            TaskComplianceCtrl, f'{prefix}/force/task_compliance_ctrl',
            callback_group=self.callback_group
        )
        self.cli_release_compliance = self.node.create_client(
            ReleaseComplianceCtrl, f'{prefix}/force/release_compliance_ctrl',
            callback_group=self.callback_group
        )
        
        # 그리퍼 관련
        self.cli_set_digital_output = self.node.create_client(
            SetCtrlBoxDigitalOutput, f'{prefix}/io/set_ctrl_box_digital_output',
            callback_group=self.callback_group
        )
    
    # =========================================
    # 콜백 설정
    # =========================================
    def set_progress_callback(self, callback: Callable[[str, int], None]):
        """진행 상태 콜백 (step_name, progress_percent)"""
        self._on_progress_callback = callback
    
    def set_complete_callback(self, callback: Callable[[bool], None]):
        """복구 완료 콜백 (success)"""
        self._on_complete_callback = callback
    
    def _notify_progress(self, step: str, percent: int):
        """진행 상태 알림"""
        self._recovery_step = step
        self.node.get_logger().info(f'[Recovery] {step} ({percent}%)')
        if self._on_progress_callback:
            self._on_progress_callback(step, percent)
    
    def _get_external_force(self) -> float:
        """
        현재 외력의 크기를 반환
        원본 collision_recovery에는 이 기능이 없었음 - 사용하지 않음
        """
        return 0.0
    
    # =========================================
    # 작업 상태 저장/복원
    # =========================================
    def save_work_state(self, state: dict):
        """충돌 전 작업 상태 저장"""
        self._saved_work_state = state.copy() if state else None
        self.node.get_logger().info(f'[Recovery] 작업 상태 저장: {state}')
    
    def get_saved_work_state(self) -> Optional[dict]:
        """저장된 작업 상태 반환"""
        return self._saved_work_state
    
    def clear_saved_work_state(self):
        """저장된 작업 상태 삭제"""
        self._saved_work_state = None
    
    # =========================================
    # 복구 속성
    # =========================================
    @property
    def is_recovering(self) -> bool:
        return self._is_recovering
    
    @property
    def recovery_step(self) -> str:
        return self._recovery_step
    
    # =========================================
    # 개별 복구 단계
    # =========================================
    def _call_control(self, control_cmd: int, timeout: float = 10.0) -> bool:
        """제어 명령 호출"""
        self.node.get_logger().info(f'[Recovery] _call_control({control_cmd}) 시작')
        
        if not self.cli_set_control.service_is_ready():
            self.node.get_logger().warn('[Recovery] set_robot_control 서비스 없음')
            return False
        
        req = SetRobotControl.Request()
        req.robot_control = control_cmd
        
        self.node.get_logger().info(f'[Recovery] call_async 호출 중...')
        future = self.cli_set_control.call_async(req)
        
        # MultiThreadedExecutor가 별도 스레드에서 돌고 있으므로 단순 폴링
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        
        if future.done():
            result = future.result()
            if result:
                self.node.get_logger().info(f'[Recovery] set_robot_control({control_cmd}) = {result.success}')
                return result.success
            else:
                self.node.get_logger().warn(f'[Recovery] set_robot_control({control_cmd}) result=None')
        else:
            self.node.get_logger().warn(f'[Recovery] set_robot_control({control_cmd}) 타임아웃 ({timeout}초)')
        return False
    
    def _call_safety(self, mode: int, event: int, timeout: float = 10.0) -> bool:
        """안전 모드 호출"""
        self.node.get_logger().info(f'[Recovery] _call_safety({mode},{event}) 시작')
        
        if not self.cli_set_safety.service_is_ready():
            self.node.get_logger().warn('[Recovery] set_safety_mode 서비스 없음')
            return False
        
        req = SetSafetyMode.Request()
        req.safety_mode = mode
        req.safety_event = event
        
        self.node.get_logger().info(f'[Recovery] call_async 호출 중...')
        future = self.cli_set_safety.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        
        if future.done():
            result = future.result()
            if result:
                self.node.get_logger().info(f'[Recovery] set_safety_mode({mode},{event}) = {result.success}')
                return result.success
            else:
                self.node.get_logger().warn(f'[Recovery] set_safety_mode({mode},{event}) result=None')
        else:
            self.node.get_logger().warn(f'[Recovery] set_safety_mode({mode},{event}) 타임아웃 ({timeout}초)')
        return False
    
    def _call_jog(self, axis: int, speed: float, time_sec: float) -> bool:
        """Jog 호출 (복구 모드에서 Z축 상승)"""
        if not self.cli_jog.service_is_ready():
            self.node.get_logger().warn('[Recovery] jog 서비스 없음')
            return False
        
        self.node.get_logger().info(f'[Recovery] Jog 시작: axis={axis}, speed={speed}, time={time_sec}s')
        
        req = Jog.Request()
        req.jog_axis = axis
        req.move_reference = 0  # base 기준
        req.speed = speed
        
        # Jog 시작 호출 및 결과 대기
        future = self.cli_jog.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 2.0:
            time.sleep(0.05)
        
        if future.done():
            result = future.result()
            self.node.get_logger().info(f'[Recovery] Jog 시작 결과: {result.success if result else "None"}')
        else:
            self.node.get_logger().warn('[Recovery] Jog 시작 타임아웃')
            return False
        
        # Jog 시작 후 지정된 시간만큼 대기 (실제 이동 시간)
        self.node.get_logger().info(f'[Recovery] Jog 이동 중 ({time_sec}초)...')
        time.sleep(time_sec)
        
        # Jog 정지
        self.node.get_logger().info('[Recovery] Jog 정지 요청')
        req_stop = Jog.Request()
        req_stop.jog_axis = axis  # 같은 축
        req_stop.move_reference = 0
        req_stop.speed = 0.0  # 속도 0으로 정지
        
        future_stop = self.cli_jog.call_async(req_stop)
        start = time.time()
        while not future_stop.done() and (time.time() - start) < 2.0:
            time.sleep(0.05)
        
        if future_stop.done():
            result_stop = future_stop.result()
            self.node.get_logger().info(f'[Recovery] Jog 정지 결과: {result_stop.success if result_stop else "None"}')
        
        time.sleep(0.5)  # 정지 후 안정화 대기
        return True
    
    def reset_safe_stop(self) -> bool:
        """1단계: SAFE_STOP 리셋"""
        self._notify_progress('SAFE_STOP 리셋', 10)
        result = self._call_control(CTRL_RESET_SAFE_STOP)
        time.sleep(0.3)
        return result
    
    def enter_recovery_mode(self) -> bool:
        """
        2단계: 복구 모드 진입 (ENTER만)
        원본 recovery.py와 동일하게 ENTER(event=0)만 호출
        """
        self._notify_progress('복구 모드 진입', 25)
        
        # ENTER (event=0)만 호출 - 원본과 동일
        enter_result = self._call_safety(2, 0)
        if not enter_result:
            self.node.get_logger().warn('[Recovery] RECOVERY ENTER 실패')
            return False
        
        time.sleep(0.3)
        
        # 상태 확인
        state = self.state_monitor.get_robot_state()
        self.node.get_logger().info(f'[Recovery] ENTER 후 상태: {state_name(state)}')
        
        return True
    
    def jog_up(self) -> bool:
        """3단계: Z축 상승 Jog (바닥 충돌 시)"""
        self._notify_progress('Z축 상승 중', 50)
        result = self._call_jog(RECOVERY_JOG_AXIS_Z, RECOVERY_JOG_SPEED, RECOVERY_JOG_TIME)
        time.sleep(0.2)
        return result
    
    def complete_recovery(self) -> bool:
        """4단계: 복구 완료 (mode=2, event=2)"""
        self._notify_progress('복구 완료 처리', 70)
        result = self._call_safety(2, 2)
        time.sleep(0.3)
        return result
    
    def exit_recovery_mode(self) -> bool:
        """5단계: 복구 모드 종료"""
        self._notify_progress('복구 모드 종료', 85)
        result = self._call_control(CTRL_RESET_RECOVERY)
        time.sleep(0.3)
        return result
    
    def servo_on(self) -> bool:
        """6단계: 서보 ON"""
        self._notify_progress('서보 ON', 95)
        result = self._call_control(CTRL_SERVO_ON)
        time.sleep(0.3)
        return result
    
    # =========================================
    # 자동 복구
    # =========================================
    def auto_recovery(self, work_state: dict = None) -> bool:
        """
        자동 복구 실행
        - Z 높이 기준으로 Case 판별
        - Case 1 (Z >= 100mm): 외부 충돌 → Jog 생략
        - Case 2 (Z < 100mm): 바닥 충돌 → Jog 필요
        
        Args:
            work_state: 저장할 작업 상태
            
        Returns:
            복구 성공 여부
        """
        self.node.get_logger().info('[Recovery] ========== auto_recovery 진입 ==========')
        
        if self._is_recovering:
            self.node.get_logger().warn('[Recovery] 이미 복구 중')
            return False
        
        self._is_recovering = True
        success = False
        
        try:
            # 작업 상태 저장
            if work_state:
                self.save_work_state(work_state)
            
            self.node.get_logger().info('[Recovery] Z 높이 조회 중...')
            
            # 현재 Z 높이로 Case 판별
            current_z = self.state_monitor.get_current_z()
            
            self.node.get_logger().info(f'[Recovery] Z 높이 = {current_z}')
            
            needs_jog = (current_z is not None and current_z < RECOVERY_Z_THRESHOLD)
            
            case_type = "Case 2 (바닥 충돌)" if needs_jog else "Case 1 (외부 충돌)"
            z_str = f'{current_z:.1f}mm' if current_z else 'None'
            self.node.get_logger().info(f'[Recovery] 자동 복구 시작 - {case_type}, Z={z_str}')
            
            self._notify_progress(f'{case_type} 감지', 5)
            
            # 1단계: SAFE_STOP 리셋
            if not self.reset_safe_stop():
                self.node.get_logger().error('[Recovery] SAFE_STOP 리셋 실패')
                return False
            
            # 2단계: 복구 모드 진입 (ENTER + EXECUTE)
            if not self.enter_recovery_mode():
                self.node.get_logger().error('[Recovery] 복구 모드 진입 실패')
                return False
            
            # 3단계: Z축 상승 (바닥 충돌 시)
            if needs_jog:
                if not self.jog_up():
                    self.node.get_logger().warn('[Recovery] Jog 실패, 계속 진행')
            else:
                self._notify_progress('Jog 생략', 50)
            
            # 4단계: 복구 완료
            if not self.complete_recovery():
                self.node.get_logger().error('[Recovery] 복구 완료 처리 실패')
                return False
            
            # 5단계: 복구 모드 종료
            if not self.exit_recovery_mode():
                self.node.get_logger().error('[Recovery] 복구 모드 종료 실패')
                return False
            
            # 6단계: 서보 ON
            if not self.servo_on():
                self.node.get_logger().error('[Recovery] 서보 ON 실패')
                return False
            
            # 최종 상태 확인 (상태 전환까지 최대 3초 대기)
            final_state = None
            for attempt in range(15):  # 0.2초 * 15 = 3초
                time.sleep(0.2)
                final_state = self.state_monitor.get_robot_state()
                self.node.get_logger().info(f'[Recovery] 상태 확인 {attempt+1}/15: {state_name(final_state)}')
                if self.state_monitor.is_standby(final_state):
                    break
            
            if self.state_monitor.is_standby(final_state):
                self._notify_progress('서보 복구 완료', 80)
                self.node.get_logger().info('✅ [Recovery] 서보 복구 성공!')
                
                # 복구 모드 완전 종료 대기 (로봇이 정상 동작 가능해질 때까지)
                # 복구 후 바로 이동하면 SAFE_OFF가 발생할 수 있으므로 충분히 대기
                self.node.get_logger().info('[Recovery] 복구 모드 완전 종료 대기 중 (3초)...')
                time.sleep(3.0)
                
                # 상태가 여전히 STANDBY인지 재확인
                check_state = self.state_monitor.get_robot_state()
                if not self.state_monitor.is_standby(check_state):
                    self.node.get_logger().warn(f'[Recovery] 대기 후 상태 변경됨: {state_name(check_state)}')
                    self._notify_progress('복구 완료 (이동 생략)', 100)
                    success = True
                else:
                    self.node.get_logger().info('[Recovery] 상태 안정 확인됨, 이동 시작')
                    
                    # 7단계: 블럭 잡고 있으면 반납 후 홈 복귀
                    if self._is_gripping():
                        self.node.get_logger().info('[Recovery] 블럭 감지 → 컨베이어로 반납')
                        self._return_block_to_conveyor()
                    else:
                        self.node.get_logger().info('[Recovery] 블럭 없음 → 홈 복귀')
                        self._move_to_home()
                    
                    self._notify_progress('복구 완료', 100)
                    success = True
            else:
                self.node.get_logger().warn(f'[Recovery] 최종 상태: {state_name(final_state)}')
                success = False
            
        except Exception as e:
            self.node.get_logger().error(f'[Recovery] 오류: {e}')
            success = False
            
        finally:
            self._is_recovering = False
            if self._on_complete_callback:
                self._on_complete_callback(success)
        
        return success
    
    def manual_recovery(self) -> bool:
        """수동 복구 (Web UI에서 버튼 클릭 시)"""
        return self.auto_recovery()
    
    # =========================================
    # 블럭 반납 관련
    # =========================================
    def _is_gripping(self) -> bool:
        """
        그리퍼가 블럭을 잡고 있는지 확인
        DO1=1(닫힘 명령 상태) + DI로 유효 폭 감지 시 블럭 잡고 있음
        """
        try:
            from dsr_msgs2.srv import GetCtrlBoxDigitalInput, GetCtrlBoxDigitalOutput
            
            # DO 클라이언트 생성
            cli_do = self.node.create_client(
                GetCtrlBoxDigitalOutput, f'/{ROBOT_ID}/io/get_ctrl_box_digital_output',
                callback_group=self.callback_group
            )
            
            cli_di = self.node.create_client(
                GetCtrlBoxDigitalInput, f'/{ROBOT_ID}/io/get_ctrl_box_digital_input',
                callback_group=self.callback_group
            )
            
            # DO1 확인 (닫힘 명령 여부)
            if cli_do.service_is_ready():
                req_do = GetCtrlBoxDigitalOutput.Request()
                req_do.index = 1
                future_do = cli_do.call_async(req_do)
                
                start = time.time()
                while not future_do.done() and (time.time() - start) < 2.0:
                    time.sleep(0.01)
                
                if future_do.done() and future_do.result():
                    do1 = future_do.result().value
                    if do1 != 1:
                        # 그리퍼 열림 명령 상태 → 블럭 없음
                        self.node.get_logger().info(f'[Recovery] 그리퍼 상태: DO1={do1} (열림) → 블럭 없음')
                        return False
            
            # DI1, DI2로 그리퍼 폭 확인
            if not cli_di.service_is_ready():
                return False
            
            req1 = GetCtrlBoxDigitalInput.Request()
            req1.index = 1
            future1 = cli_di.call_async(req1)
            
            start = time.time()
            while not future1.done() and (time.time() - start) < 2.0:
                time.sleep(0.01)
            
            req2 = GetCtrlBoxDigitalInput.Request()
            req2.index = 2
            future2 = cli_di.call_async(req2)
            
            start = time.time()
            while not future2.done() and (time.time() - start) < 2.0:
                time.sleep(0.01)
            
            if future1.done() and future2.done():
                di1 = future1.result().value if future1.result() else 0
                di2 = future2.result().value if future2.result() else 0
                
                # DI1=1,DI2=0 → SMALL, DI1=0,DI2=1 → LONG, DI1=0,DI2=0 → MEDIUM
                # DI1=1,DI2=1 → 완전히 열림 (블럭 없음)
                if di1 == 1 and di2 == 1:
                    self.node.get_logger().info(f'[Recovery] 그리퍼 상태: DI1={di1}, DI2={di2} (완전열림) → 블럭 없음')
                    return False
                
                self.node.get_logger().info(f'[Recovery] 그리퍼 상태: DI1={di1}, DI2={di2} → 블럭 있음')
                return True
            
            return False
        except Exception as e:
            self.node.get_logger().warn(f'[Recovery] 그리퍼 상태 확인 실패: {e}')
            return False
    
    def _movel(self, pos, vel=None, acc=None, timeout=30.0) -> bool:
        """직선 이동 (이동 완료까지 대기)"""
        import numpy as np
        
        # 이동 전 상태 확인 - STANDBY가 아니면 이동 불가
        current_state = self.state_monitor.get_robot_state()
        if not self.state_monitor.is_standby(current_state):
            self.node.get_logger().warn(f'[Recovery] 이동 불가 - 현재 상태: {state_name(current_state)}')
            return False
        
        if not self.cli_move_line.service_is_ready():
            self.node.get_logger().warn('[Recovery] move_line 서비스 없음')
            return False
        
        req = MoveLine.Request()
        req.pos = np.array(pos, dtype=np.float64)
        req.vel = np.array([vel or VELOCITY_MOVE] * 2, dtype=np.float64)
        req.acc = np.array([acc or ACCEL_MOVE] * 2, dtype=np.float64)
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0
        
        future = self.cli_move_line.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)
        
        if future.done() and future.result() and future.result().success:
            # 이동 완료까지 대기 (MOVING → STANDBY)
            self._wait_for_motion_complete(timeout=timeout)
            return True
        return False
    
    def _wait_for_motion_complete(self, timeout: float = 30.0):
        """로봇이 STANDBY 상태가 될 때까지 대기"""
        start = time.time()
        while (time.time() - start) < timeout:
            state = self.state_monitor.get_robot_state()
            if self.state_monitor.is_standby(state):
                return True
            time.sleep(0.1)
        return False
    
    def _grip_open(self):
        """그리퍼 열기"""
        if not self.cli_set_digital_output.service_is_ready():
            return False
        
        req1 = SetCtrlBoxDigitalOutput.Request()
        req1.index = 1
        req1.value = 0
        self.cli_set_digital_output.call_async(req1)
        
        req2 = SetCtrlBoxDigitalOutput.Request()
        req2.index = 2
        req2.value = 1
        self.cli_set_digital_output.call_async(req2)
        
        time.sleep(0.3)
        return True
    
    def _enable_compliance(self) -> bool:
        """순응제어 활성화"""
        if not self.cli_task_compliance.service_is_ready():
            return False
        
        req = TaskComplianceCtrl.Request()
        req.stx = DEFAULT_STIFFNESS
        req.ref = DR_BASE
        req.time = 0.0
        
        future = self.cli_task_compliance.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 2.0:
            time.sleep(0.01)
        
        return future.done() and future.result() and future.result().success
    
    def _disable_compliance(self) -> bool:
        """순응제어 해제"""
        if not self.cli_release_compliance.service_is_ready():
            return False
        
        req = ReleaseComplianceCtrl.Request()
        future = self.cli_release_compliance.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 2.0:
            time.sleep(0.01)
        
        return True
    
    def _return_block_to_conveyor(self):
        """블럭을 컨베이어 시작점으로 반납"""
        self._notify_progress('블럭 반납 중', 85)
        
        try:
            # 1. 컨베이어 시작점으로 이동 (50mm 위)
            self.node.get_logger().info('[Recovery] 컨베이어 시작점으로 이동')
            convey_above = CONVEY_START_POINT.copy()
            convey_above[2] += 50.0  # 50mm 위로
            
            if not self._movel(convey_above, vel=150.0, acc=300.0):
                self.node.get_logger().warn('[Recovery] 컨베이어 위 이동 실패 - 홈 복귀 시도')
                self._move_to_home()
                return
            
            # 2. 순응제어로 부드럽게 하강
            self.node.get_logger().info('[Recovery] 순응제어로 하강')
            self._enable_compliance()
            time.sleep(0.2)
            
            # 컨베이어 시작점으로 천천히 하강
            if not self._movel(CONVEY_START_POINT, vel=50.0, acc=100.0):
                self.node.get_logger().warn('[Recovery] 하강 실패')
                self._disable_compliance()
                self._move_to_home()
                return
            
            time.sleep(0.3)
            self._disable_compliance()
            
            # 3. 그리퍼 열기 (블럭 놓기)
            self.node.get_logger().info('[Recovery] 그리퍼 열기')
            self._grip_open()
            time.sleep(0.5)
            
            # 4. 상승
            self.node.get_logger().info('[Recovery] 상승')
            if not self._movel(convey_above, vel=150.0, acc=300.0):
                self.node.get_logger().warn('[Recovery] 상승 실패')
            
            # 5. 홈 복귀
            self._move_to_home()
            
            self.node.get_logger().info('✅ [Recovery] 블럭 반납 완료')
            
        except Exception as e:
            self.node.get_logger().error(f'[Recovery] 블럭 반납 오류: {e}')
            self._disable_compliance()
    
    def _move_to_home(self):
        """홈 위치로 이동"""
        self._notify_progress('홈 복귀 중', 95)
        self.node.get_logger().info('[Recovery] 홈 위치로 이동')
        
        # 이동 전 상태 재확인
        current_state = self.state_monitor.get_robot_state()
        if not self.state_monitor.is_standby(current_state):
            self.node.get_logger().warn(f'[Recovery] 홈 이동 불가 - 현재 상태: {state_name(current_state)}')
            return
        
        # 추가 안정화 대기
        time.sleep(0.5)
        
        if not self._movel(HOME_POSITION, vel=200.0, acc=400.0):
            self.node.get_logger().warn('[Recovery] 홈 이동 실패')
        else:
            self.node.get_logger().info('✅ [Recovery] 홈 도착')
