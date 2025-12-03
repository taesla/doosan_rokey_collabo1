#!/usr/bin/env python3
"""
리커버리 모듈 테스트 스크립트

사용법:
    ros2 run dsr_integrated test_recovery

테스트 방법:
    1. 로봇을 밀어서 SAFE_STOP 상태로 만들기
    2. 이 스크립트 실행
    3. 복구 과정 확인
    
메뉴:
    1. 현재 상태 확인
    2. 자동 복구 실행
    3. 단계별 수동 복구
    4. 종료
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import threading
import time
import signal
import sys

from dsr_msgs2.srv import GetRobotState

# 상대 경로 import 대신 직접 정의
ROBOT_ID = 'dsr01'  # 실제 서비스 prefix

# 상수 정의 (constants.py에서 가져옴)
CTRL_RESET_SAFE_STOP = 2
CTRL_SERVO_ON = 3
CTRL_RESET_RECOVERY = 7

RECOVERY_Z_THRESHOLD = 100.0
RECOVERY_JOG_TIME = 0.5
RECOVERY_JOG_SPEED = 10.0
RECOVERY_JOG_AXIS_Z = 2

# 상태 이름
STATE_NAMES = {
    0: 'INITIALIZING',
    1: 'STANDBY',
    2: 'MOVING',
    3: 'SAFE_OFF',
    4: 'TEACHING',
    5: 'SAFE_STOP',
    6: 'EMERGENCY_STOP',
    7: 'HOMMING',
    8: 'RECOVERY',
    9: 'SAFE_STOP2',
    10: 'SAFE_OFF2',
    11: 'RESERVED1',
    12: 'RESERVED2',
    13: 'RESERVED3',
    14: 'RESERVED4',
    15: 'NOT_READY',
}

def state_name(state: int) -> str:
    return STATE_NAMES.get(state, f'UNKNOWN({state})')


class RecoveryTestNode(Node):
    """리커버리 테스트 노드"""
    
    def __init__(self):
        super().__init__('recovery_test_node')
        self.callback_group = ReentrantCallbackGroup()
        
        prefix = f'/{ROBOT_ID}'
        
        # 서비스 클라이언트
        from dsr_msgs2.srv import SetRobotControl, SetSafetyMode, Jog
        
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
        
        # GetCurrentPosx 서비스 (Z 좌표 확인용)
        from dsr_msgs2.srv import GetCurrentPosx
        self.cli_posx = self.create_client(
            GetCurrentPosx, f'{prefix}/aux_control/get_current_posx',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('리커버리 테스트 노드 시작')
        self.get_logger().info('=' * 50)
    
    # =========================================
    # 서비스 호출
    # =========================================
    def get_robot_state(self) -> int:
        """로봇 상태 조회"""
        if not self.cli_state.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('GetRobotState 서비스 없음')
            return -1
        
        req = GetRobotState.Request()
        future = self.cli_state.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.05)
        
        if future.done() and future.result():
            return future.result().robot_state
        return -1
    
    def get_current_z(self) -> float:
        """현재 Z 좌표 조회"""
        from dsr_msgs2.srv import GetCurrentPosx
        
        if not self.cli_posx.wait_for_service(timeout_sec=2.0):
            return None
        
        req = GetCurrentPosx.Request()
        req.ref = 0  # DR_BASE
        future = self.cli_posx.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.05)
        
        if future.done() and future.result():
            result = future.result()
            if hasattr(result, 'task_pos_info') and len(result.task_pos_info) > 0:
                pos = result.task_pos_info[0].data
                if len(pos) >= 3:
                    return pos[2]
        return None
    
    def call_control(self, control_code: int, name: str) -> bool:
        """SetRobotControl 호출"""
        from dsr_msgs2.srv import SetRobotControl
        
        self.get_logger().info(f'  → SetRobotControl({control_code}) : {name}')
        
        if not self.cli_control.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('서비스 없음')
            return False
        
        req = SetRobotControl.Request()
        req.robot_control = control_code
        
        future = self.cli_control.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.05)
        
        if future.done() and future.result():
            result = future.result().success
            self.get_logger().info(f'    결과: {"성공" if result else "실패"}')
            return result
        
        self.get_logger().error('    결과: 타임아웃')
        return False
    
    def call_safety(self, mode: int, event: int, name: str) -> bool:
        """SetSafetyMode 호출"""
        from dsr_msgs2.srv import SetSafetyMode
        
        self.get_logger().info(f'  → SetSafetyMode(mode={mode}, event={event}) : {name}')
        
        if not self.cli_safety.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('서비스 없음')
            return False
        
        req = SetSafetyMode.Request()
        req.safety_mode = mode
        req.safety_event = event
        
        future = self.cli_safety.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 3.0:
            time.sleep(0.05)
        
        if future.done() and future.result():
            result = future.result().success
            self.get_logger().info(f'    결과: {"성공" if result else "실패"}')
            return result
        
        self.get_logger().error('    결과: 타임아웃')
        return False
    
    def call_jog(self, axis: int, speed: float, duration: float) -> bool:
        """Jog 호출"""
        from dsr_msgs2.srv import Jog
        
        self.get_logger().info(f'  → Jog(axis={axis}, speed={speed}, time={duration}s)')
        
        if not self.cli_jog.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('서비스 없음')
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
            self.get_logger().error('    Jog 시작 실패')
            return False
        
        self.get_logger().info(f'    Jog 시작됨, {duration}초 대기...')
        time.sleep(duration)
        
        # Jog 정지
        req.speed = 0.0
        future = self.cli_jog.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 2.0:
            time.sleep(0.05)
        
        self.get_logger().info('    Jog 정지')
        return True
    
    # =========================================
    # 테스트 함수
    # =========================================
    def show_status(self):
        """현재 상태 표시"""
        print('\n' + '=' * 50)
        print('📊 현재 로봇 상태')
        print('=' * 50)
        
        state = self.get_robot_state()
        z = self.get_current_z()
        
        print(f'  상태: {state_name(state)} ({state})')
        print(f'  Z 좌표: {z:.1f}mm' if z else '  Z 좌표: N/A')
        
        if state == 5 or state == 9:  # SAFE_STOP or SAFE_STOP2
            print('  ⚠️  SAFE_STOP 상태 - 복구 필요!')
        elif state == 8:  # RECOVERY
            print('  🔧 RECOVERY 모드')
        elif state == 1:  # STANDBY
            print('  ✅ 정상 (STANDBY)')
        
        print('=' * 50)
    
    def auto_recover(self):
        """자동 복구 시퀀스"""
        print('\n' + '=' * 50)
        print('🔄 자동 복구 시작')
        print('=' * 50)
        
        # 1. 현재 상태 확인
        state = self.get_robot_state()
        z = self.get_current_z()
        
        print(f'\n현재 상태: {state_name(state)}')
        print(f'현재 Z: {z:.1f}mm' if z else '현재 Z: N/A')
        
        # 이미 STANDBY면 종료
        if state == 1:
            print('✅ 이미 STANDBY 상태입니다!')
            return True
        
        # 바닥 충돌 판단
        needs_jog = z is not None and z < RECOVERY_Z_THRESHOLD
        print(f'바닥 충돌 여부: {"예 (Jog 필요)" if needs_jog else "아니오"}')
        
        input('\n[Enter]를 눌러 복구 시작...')
        
        # 2. 복구 시퀀스
        print('\n[1/6] SAFE_STOP 리셋')
        if state == 5 or state == 9:
            self.call_control(CTRL_RESET_SAFE_STOP, 'RESET_SAFE_STOP')
            time.sleep(0.5)
        else:
            print('  → 생략 (SAFE_STOP 아님)')
        
        print('\n[2/6] RECOVERY 모드 진입')
        self.call_safety(2, 0, 'RECOVERY ENTER')
        time.sleep(0.3)
        
        print('\n[3/6] Z축 상승 (Jog)')
        if needs_jog:
            self.call_jog(RECOVERY_JOG_AXIS_Z, RECOVERY_JOG_SPEED, RECOVERY_JOG_TIME)
            time.sleep(0.3)
        else:
            print('  → 생략 (바닥 충돌 아님)')
        
        print('\n[4/6] RECOVERY 완료')
        self.call_safety(2, 2, 'RECOVERY COMPLETE')
        time.sleep(0.5)
        
        print('\n[5/6] RECOVERY 해제')
        self.call_control(CTRL_RESET_RECOVERY, 'RESET_RECOVERY')
        time.sleep(0.5)
        
        # 중간 상태 확인
        state = self.get_robot_state()
        print(f'\n중간 상태: {state_name(state)}')
        
        print('\n[6/6] 서보 ON')
        if state != 1:
            self.call_control(CTRL_SERVO_ON, 'SERVO_ON')
            time.sleep(1.0)
        else:
            print('  → 생략 (이미 STANDBY)')
        
        # 3. 결과 확인
        state = self.get_robot_state()
        print('\n' + '=' * 50)
        if state == 1:
            print('✅ 복구 성공! 상태: STANDBY')
        else:
            print(f'❌ 복구 실패. 상태: {state_name(state)}')
        print('=' * 50)
        
        return state == 1
    
    def manual_step(self):
        """단계별 수동 복구"""
        print('\n' + '=' * 50)
        print('🔧 단계별 수동 복구')
        print('=' * 50)
        print('1. SAFE_STOP 리셋 (control=2)')
        print('2. RECOVERY 진입 (mode=2, event=0)')
        print('3. Jog Z+ 상승')
        print('4. RECOVERY 완료 (mode=2, event=2)')
        print('5. RECOVERY 해제 (control=7)')
        print('6. 서보 ON (control=3)')
        print('0. 취소')
        print('=' * 50)
        
        try:
            choice = int(input('선택: '))
        except:
            return
        
        if choice == 1:
            self.call_control(CTRL_RESET_SAFE_STOP, 'RESET_SAFE_STOP')
        elif choice == 2:
            self.call_safety(2, 0, 'RECOVERY ENTER')
        elif choice == 3:
            duration = float(input('Jog 시간(초, 기본 0.5): ') or '0.5')
            self.call_jog(RECOVERY_JOG_AXIS_Z, RECOVERY_JOG_SPEED, duration)
        elif choice == 4:
            self.call_safety(2, 2, 'RECOVERY COMPLETE')
        elif choice == 5:
            self.call_control(CTRL_RESET_RECOVERY, 'RESET_RECOVERY')
        elif choice == 6:
            self.call_control(CTRL_SERVO_ON, 'SERVO_ON')
        
        # 현재 상태 표시
        time.sleep(0.5)
        self.show_status()


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryTestNode()
    
    # Ctrl+C 핸들러
    def signal_handler(sig, frame):
        print('\n\n👋 종료합니다...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # executor 스레드
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    time.sleep(1.0)  # 서비스 연결 대기
    
    try:
        while True:
            print('\n' + '=' * 50)
            print('🤖 리커버리 테스트 메뉴')
            print('=' * 50)
            print('1. 현재 상태 확인')
            print('2. 자동 복구 실행')
            print('3. 단계별 수동 복구')
            print('4. 종료')
            print('=' * 50)
            
            try:
                choice = input('선택: ').strip()
                if not choice:
                    continue
                choice = int(choice)
            except ValueError:
                continue
            except EOFError:
                break
            
            if choice == 1:
                node.show_status()
            elif choice == 2:
                node.auto_recover()
            elif choice == 3:
                node.manual_step()
            elif choice == 4:
                break
    
    except KeyboardInterrupt:
        pass
    
    finally:
        print('\n👋 종료합니다...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
