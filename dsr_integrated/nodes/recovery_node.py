#!/usr/bin/env python3
"""
독립 복구 노드 (Recovery Node)

별도 터미널에서 실행하여 로봇 상태를 모니터링하고
충돌 감지 시 자동으로 복구를 수행합니다.

사용법:
    터미널 1: ros2 launch dsr_integrated full_system.launch.py
    터미널 2: ros2 run dsr_integrated recovery_node

동작:
    1. 로봇 상태를 0.2초마다 모니터링
    2. SAFE_STOP 또는 SAFE_OFF 감지 시 자동 복구 시작
    3. 복구 진행률을 터미널에 출력
    4. 복구 완료 후 STANDBY 상태 확인
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time
import sys

from dsr_msgs2.srv import (
    GetRobotState, 
    SetRobotControl, 
    SetSafetyMode, 
    Jog,
    GetCurrentPosx,
    SetCtrlBoxDigitalOutput,
    GetCtrlBoxDigitalInput
)

# =========================================
# 상수 정의
# =========================================
ROBOT_ID = 'dsr01'

# 로봇 상태
STATE_STANDBY = 1
STATE_MOVING = 2
STATE_SAFE_OFF = 3
STATE_SAFE_STOP = 5
STATE_RECOVERY = 8
STATE_SAFE_OFF2 = 10

# 제어 코드
CTRL_RESET_SAFE_STOP = 2
CTRL_SERVO_ON = 3
CTRL_RESET_RECOVERY = 7

# 복구 설정
RECOVERY_Z_THRESHOLD = 100.0  # mm - 이 높이 이하면 바닥 충돌로 판단
RECOVERY_JOG_SPEED = 10.0
RECOVERY_JOG_TIME = 0.5
RECOVERY_JOG_AXIS_Z = 2  # Z축

# 상태 이름
STATE_NAMES = {
    0: 'INITIALIZING',
    1: 'STANDBY',
    2: 'MOVING',
    3: 'SAFE_OFF',
    4: 'TEACHING',
    5: 'SAFE_STOP',
    6: 'EMERGENCY_STOP',
    7: 'HOMING',
    8: 'RECOVERY',
    9: 'SAFE_STOP2',
    10: 'SAFE_OFF2',
    15: 'NOT_READY',
}


def state_name(state) -> str:
    if state is None:
        return 'UNKNOWN(None)'
    return STATE_NAMES.get(state, f'UNKNOWN({state})')


class RecoveryNode(Node):
    """독립 복구 노드"""
    
    def __init__(self):
        super().__init__('recovery_node')
        self.callback_group = ReentrantCallbackGroup()
        
        prefix = f'/{ROBOT_ID}'
        
        # 서비스 클라이언트
        self.cli_state = self.create_client(
            GetRobotState, f'{prefix}/system/get_robot_state',
            callback_group=self.callback_group
        )
        self.cli_control = self.create_client(
            SetRobotControl, f'{prefix}/system/set_robot_control',
            callback_group=self.callback_group
        )
        self.cli_safety = self.create_client(
            SetSafetyMode, f'{prefix}/system/set_safety_mode',
            callback_group=self.callback_group
        )
        self.cli_jog = self.create_client(
            Jog, f'{prefix}/motion/jog',
            callback_group=self.callback_group
        )
        self.cli_posx = self.create_client(
            GetCurrentPosx, f'{prefix}/aux_control/get_current_posx',
            callback_group=self.callback_group
        )
        self.cli_set_do = self.create_client(
            SetCtrlBoxDigitalOutput, f'{prefix}/io/set_ctrl_box_digital_output',
            callback_group=self.callback_group
        )
        self.cli_get_di = self.create_client(
            GetCtrlBoxDigitalInput, f'{prefix}/io/get_ctrl_box_digital_input',
            callback_group=self.callback_group
        )
        
        # 상태 변수
        self._previous_state = None
        self._is_recovering = False
        self._monitor_interval = 0.2  # 200ms
        self._consecutive_failures = 0  # 드라이버 응답 실패 카운터
        
        self._print_banner()
        
        # 서비스 연결 대기
        self._wait_for_services()
        
        # 모니터링 타이머 시작
        self.create_timer(self._monitor_interval, self._monitor_callback)
    
    def _print_banner(self):
        """시작 배너 출력"""
        print('\n' + '=' * 60)
        print('🔧 독립 복구 노드 (Recovery Node)')
        print('=' * 60)
        print('  • 로봇 상태를 실시간 모니터링합니다')
        print('  • SAFE_STOP/SAFE_OFF 감지 시 자동 복구를 시작합니다')
        print('  • 복구 진행률이 터미널에 표시됩니다')
        print('  • Ctrl+C로 종료합니다')
        print('=' * 60 + '\n')
    
    def _wait_for_services(self):
        """서비스 연결 대기"""
        print('⏳ DSR 드라이버 서비스 연결 대기 중...')
        
        services = [
            (self.cli_state, 'get_robot_state'),
            (self.cli_control, 'set_robot_control'),
            (self.cli_safety, 'set_safety_mode'),
        ]
        
        for cli, name in services:
            if not cli.wait_for_service(timeout_sec=10.0):
                print(f'❌ {name} 서비스 연결 실패!')
                print('   → DSR 드라이버가 실행 중인지 확인하세요')
                sys.exit(1)
        
        print('✅ 서비스 연결 완료!')
        print('👀 상태 모니터링 시작...\n')
    
    # =========================================
    # 서비스 호출
    # =========================================
    def _get_robot_state(self) -> int:
        """로봇 상태 조회"""
        if not self.cli_state.service_is_ready():
            return None
        
        req = GetRobotState.Request()
        future = self.cli_state.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 1.0:
            time.sleep(0.02)
        
        if future.done() and future.result():
            return future.result().robot_state
        return None
    
    def _get_current_z(self) -> float:
        """현재 Z 좌표 조회"""
        if not self.cli_posx.service_is_ready():
            return None
        
        req = GetCurrentPosx.Request()
        req.ref = 0  # DR_BASE
        future = self.cli_posx.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 1.0:
            time.sleep(0.02)
        
        if future.done() and future.result():
            result = future.result()
            if hasattr(result, 'task_pos_info') and len(result.task_pos_info) > 0:
                pos = result.task_pos_info[0].data
                if len(pos) >= 3:
                    return pos[2]
        return None
    
    def _call_control(self, control_code: int) -> bool:
        """SetRobotControl 호출"""
        if not self.cli_control.wait_for_service(timeout_sec=2.0):
            return False
        
        req = SetRobotControl.Request()
        req.robot_control = control_code
        
        future = self.cli_control.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.05)
        
        return future.done() and future.result() and future.result().success
    
    def _call_safety(self, mode: int, event: int) -> bool:
        """SetSafetyMode 호출"""
        if not self.cli_safety.wait_for_service(timeout_sec=2.0):
            return False
        
        req = SetSafetyMode.Request()
        req.safety_mode = mode
        req.safety_event = event
        
        future = self.cli_safety.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.05)
        
        return future.done() and future.result() and future.result().success
    
    def _call_jog(self, axis: int, speed: float, duration: float) -> bool:
        """Jog 호출 (시작 → 대기 → 정지)"""
        if not self.cli_jog.wait_for_service(timeout_sec=1.0):
            return False
        
        # Jog 시작
        req = Jog.Request()
        req.jog_axis = axis
        req.move_reference = 0
        req.speed = speed
        
        future = self.cli_jog.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 2.0:
            time.sleep(0.05)
        
        if not (future.done() and future.result() and future.result().success):
            return False
        
        time.sleep(duration)
        
        # Jog 정지
        req.speed = 0.0
        future = self.cli_jog.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 2.0:
            time.sleep(0.05)
        
        return True
    
    def _set_digital_output(self, index: int, value: int) -> bool:
        """Digital Output 설정"""
        if not self.cli_set_do.service_is_ready():
            return False
        
        req = SetCtrlBoxDigitalOutput.Request()
        req.index = index
        req.value = value
        
        future = self.cli_set_do.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 1.0:
            time.sleep(0.02)
        
        return future.done() and future.result() and future.result().success
    
    def _get_digital_input(self, index: int) -> int:
        """Digital Input 읽기"""
        if not self.cli_get_di.service_is_ready():
            return None
        
        req = GetCtrlBoxDigitalInput.Request()
        req.index = index
        
        future = self.cli_get_di.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 1.0:
            time.sleep(0.02)
        
        if future.done() and future.result():
            return future.result().value
        return None
    
    def _grip_open(self):
        """그리퍼 열기"""
        self._set_digital_output(1, 0)
        self._set_digital_output(2, 1)
        time.sleep(0.3)
    
    def _is_gripping(self) -> bool:
        """
        그리퍼가 물체를 잡고 있는지 확인
        DI1/DI2 조합으로 폭 구간 판단
        
        Returns:
            True: 물체를 잡고 있음 (SMALL/MEDIUM/LONG)
        """
        di1 = self._get_digital_input(1)
        di2 = self._get_digital_input(2)
        
        if di1 is None or di2 is None:
            return False
        
        # SMALL: DI1=1, DI2=0
        # MEDIUM: DI1=0, DI2=0
        # LONG: DI1=0, DI2=1
        # 셋 중 하나면 물체 잡고 있음
        if (di1 == 1 and di2 == 0) or (di1 == 0 and di2 == 0) or (di1 == 0 and di2 == 1):
            return True
        return False
    
    # =========================================
    # 상태 체크
    # =========================================
    def _is_safe_stop(self, state) -> bool:
        return state == STATE_SAFE_STOP
    
    def _is_safe_off(self, state) -> bool:
        return state in (STATE_SAFE_OFF, STATE_SAFE_OFF2)
    
    def _is_standby(self, state) -> bool:
        return state == STATE_STANDBY
    
    def _needs_recovery(self, state) -> bool:
        return self._is_safe_stop(state) or self._is_safe_off(state)
    
    def _estimate_collision_type(self, state, z_pos, is_gripping) -> str:
        """
        추정 충돌 유형 판단
        
        판단 기준:
        - Z < 150mm + 그리핑 → Force Control 중 바닥 충돌 (moveL 추정)
        - Z < 200mm + 비그리핑 → Pick 시도 중 충돌 (moveL 추정)
        - SAFE_OFF → 심각한 충돌 (moveL/moveJ 고속 이동 추정)
        - Z > 300mm → 이동 중 장애물 (moveJ 추정)
        - 그 외 → 외력/손 밀기 (Jog/외력 추정)
        """
        if z_pos is None:
            return '❓ 알 수 없음 (Z 조회 실패)'
        
        # SAFE_OFF는 심각한 충돌
        if self._is_safe_off(state):
            if z_pos < 150:
                return '🔴 Force Control 충돌 (moveL 추정) - 바닥 접촉'
            else:
                return '🔴 고속 이동 중 충돌 (moveL/moveJ 추정)'
        
        # SAFE_STOP 기준
        if z_pos < 150:
            if is_gripping:
                return '🟠 Place 중 바닥 충돌 (moveL + Force 추정)'
            else:
                return '🟠 Pick 중 바닥 충돌 (moveL + Force 추정)'
        elif z_pos < 200:
            return '🟡 저고도 이동 중 충돌 (moveL 추정)'
        elif z_pos < 350:
            return '🟢 외력/손 밀기 (Jog/외력 추정)'
        else:
            return '🔵 고고도 이동 중 충돌 (moveJ 추정)'
    
    # =========================================
    # 모니터링 콜백
    # =========================================
    def _monitor_callback(self):
        """상태 모니터링 (타이머 콜백)"""
        if self._is_recovering:
            return
        
        state = self._get_robot_state()
        
        # 드라이버 죽음 감지 (연속 실패)
        if state is None:
            self._consecutive_failures += 1
            if self._consecutive_failures >= 5:
                if self._consecutive_failures == 5:
                    print('\n💀 DSR 드라이버 응답 없음!')
                    print('   → 런치 파일 재시작 필요 (Ctrl+C 후 다시 실행)')
            return
        else:
            self._consecutive_failures = 0
        
        # 상태 변경 감지
        if state != self._previous_state:
            print(f'📊 상태 변경: {state_name(self._previous_state)} → {state_name(state)}')
            self._previous_state = state
        
        # 복구 필요 감지
        if self._needs_recovery(state):
            collision_type = "SAFE_STOP (노란링)" if self._is_safe_stop(state) else "SAFE_OFF (빨간링)"
            print(f'\n⚠️  충돌 감지! {collision_type}')
            self._execute_recovery(state)
    
    # =========================================
    # 복구 실행
    # =========================================
    def _execute_recovery(self, initial_state):
        """자동 복구 실행"""
        self._is_recovering = True
        
        print('\n' + '=' * 60)
        print('🔧 자동 복구 시작')
        print('=' * 60)
        
        # Z 좌표 확인
        current_z = self._get_current_z()
        needs_jog = current_z is not None and current_z < RECOVERY_Z_THRESHOLD
        
        # 그리퍼 상태 확인
        was_gripping = self._is_gripping()
        
        # 추정 충돌 유형 판단
        collision_type = self._estimate_collision_type(initial_state, current_z, was_gripping)
        
        z_str = f'{current_z:.1f}mm' if current_z else 'N/A'
        grip_status = '🔴 물체 잡음' if was_gripping else '⚪ 빈 손'
        
        print(f'  현재 Z: {z_str}')
        print(f'  추정 충돌: {collision_type}')
        print(f'  그리퍼 상태: {grip_status}')
        print('=' * 60)
        
        max_attempts = 3
        success = False
        
        for attempt in range(max_attempts):
            print(f'\n[시도 {attempt + 1}/{max_attempts}]')
            
            state = self._get_robot_state()
            print(f'  현재 상태: {state_name(state)}')
            
            # 이미 STANDBY면 성공
            if self._is_standby(state):
                print('  ✅ 이미 STANDBY 상태!')
                success = True
                break
            
            # 1. SAFE_STOP 리셋
            if self._is_safe_stop(state):
                print('  [10%] SAFE_STOP 리셋...')
                self._call_control(CTRL_RESET_SAFE_STOP)
                time.sleep(0.5)
            
            # 2. RECOVERY 모드 진입
            print('  [25%] 복구 모드 진입...')
            self._call_safety(2, 0)  # mode=RECOVERY, event=ENTER
            time.sleep(0.3)
            
            # 3. 바닥 충돌 시 그리퍼 열기 + Jog Z+
            if needs_jog:
                # 바닥 충돌 + 물체 잡고 있으면 먼저 그리퍼 열기 (끼임 방지)
                if was_gripping:
                    print('  [40%] 그리퍼 열기 (물체 끼임 방지)...')
                    self._grip_open()
                    was_gripping = False  # 더 이상 잡고 있지 않음
                
                print('  [50%] Z축 상승 (Jog)...')
                self._call_jog(RECOVERY_JOG_AXIS_Z, RECOVERY_JOG_SPEED, RECOVERY_JOG_TIME)
                time.sleep(0.3)
            else:
                print('  [50%] Jog 생략 (외부 충돌)')
            
            # 4. RECOVERY 완료
            print('  [70%] 복구 완료 처리...')
            self._call_safety(2, 2)  # mode=RECOVERY, event=COMPLETE
            time.sleep(0.5)
            
            # 5. RECOVERY 모드 해제
            print('  [85%] 복구 모드 종료...')
            self._call_control(CTRL_RESET_RECOVERY)
            time.sleep(0.5)
            
            # 6. 상태 확인
            state = self._get_robot_state()
            print(f'  해제 후 상태: {state_name(state)}')
            
            # 7. 서보 ON (필요시)
            if not self._is_standby(state):
                print('  [95%] 서보 ON...')
                self._call_control(CTRL_SERVO_ON)
                time.sleep(1.0)
            
            # 8. 최종 확인
            state = self._get_robot_state()
            if self._is_standby(state):
                success = True
                break
            
            print(f'  ⚠️ 아직 복구 안됨: {state_name(state)}')
            time.sleep(0.5)
        
        # 결과 출력
        print('\n' + '=' * 60)
        if success:
            print('✅ 복구 완료! [100%]')
            print('   → 웹에서 "분류 시작" 또는 "재개"를 눌러주세요')
        else:
            print('❌ 복구 실패!')
            print('   → 수동 개입이 필요합니다')
            print('   → 티치 펜던트로 복구하거나 런치 파일을 재시작하세요')
        print('=' * 60 + '\n')
        
        self._is_recovering = False


def main(args=None):
    rclpy.init(args=args)
    
    node = RecoveryNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\n\n👋 복구 노드 종료')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
