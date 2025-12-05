#!/usr/bin/env python3
"""
로봇 상태 모니터링 모듈
- GetRobotState 서비스로 상태 조회
- 별도 스레드에서 주기적 모니터링
- 충돌 감지 시 콜백 호출
"""

import time
import threading
from typing import Callable, Optional

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from dsr_msgs2.srv import GetRobotState, GetCurrentPosx

from ..config.constants import (
    ROBOT_ID,
    STATE_STANDBY, STATE_SAFE_STOP, STATE_SAFE_STOP2,
    STATE_SAFE_OFF, STATE_SAFE_OFF2, STATE_RECOVERY,
    DR_BASE,
)


def state_name(code: int) -> str:
    """상태 코드를 이름으로 변환"""
    STATE_NAMES = {
        0: "INITIALIZING", 1: "STANDBY", 2: "MOVING",
        3: "SAFE_OFF", 4: "TEACHING", 5: "SAFE_STOP",
        6: "EMERGENCY_STOP", 7: "HOMING", 8: "RECOVERY",
        9: "SAFE_STOP2", 10: "SAFE_OFF2", 15: "NOT_READY",
    }
    return STATE_NAMES.get(code, f"UNKNOWN({code})")


class RobotStateMonitor:
    """로봇 상태 모니터링 클래스"""
    
    def __init__(self, node: Node, callback_group: ReentrantCallbackGroup = None):
        """
        Args:
            node: ROS2 노드 인스턴스
            callback_group: 콜백 그룹
        """
        self.node = node
        self.callback_group = callback_group
        
        # 서비스 클라이언트
        self._init_clients()
        
        # 모니터링 상태
        self._monitoring = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._monitor_interval = 0.5  # 0.5초 주기
        
        # 현재 상태
        self._current_state: Optional[int] = None
        self._previous_state: Optional[int] = None
        
        # 드라이버 건강 상태
        self._consecutive_failures = 0
        self._max_failures = 10  # 연속 10회 실패 시 드라이버 죽음 판정
        self._driver_alive = True
        self._on_driver_dead_callback: Optional[Callable] = None
        
        # 하트비트 기반 자동 복구
        self._driver_was_dead = False  # 드라이버가 죽었다가 살아났는지
        self._on_driver_recovered_callback: Optional[Callable] = None  # 드라이버 복구 콜백
        
        # 콜백
        self._on_collision_callback: Optional[Callable] = None
        self._on_recovery_complete_callback: Optional[Callable] = None
        
        self.node.get_logger().info('[StateMonitor] 초기화 완료')
    
    def _init_clients(self):
        """서비스 클라이언트 초기화"""
        prefix = f'/{ROBOT_ID}'
        
        self.cli_get_state = self.node.create_client(
            GetRobotState, f'{prefix}/system/get_robot_state',
            callback_group=self.callback_group
        )
        self.cli_get_posx = self.node.create_client(
            GetCurrentPosx, f'{prefix}/aux_control/get_current_posx',
            callback_group=self.callback_group
        )
        
        # 서비스 연결 상태
        self._services_connected = False
    
    def _ensure_services_connected(self) -> bool:
        """서비스 연결 확인 (처음 한 번만 대기)"""
        if self._services_connected:
            return True
        
        # 처음 연결 시에만 대기
        if self.cli_get_state.wait_for_service(timeout_sec=5.0):
            self._services_connected = True
            self.node.get_logger().info('[StateMonitor] 서비스 연결 완료')
            return True
        
        self.node.get_logger().warn('[StateMonitor] 서비스 연결 실패')
        return False
    
    def set_collision_callback(self, callback: Callable):
        """충돌 감지 콜백 설정"""
        self._on_collision_callback = callback
    
    def set_recovery_complete_callback(self, callback: Callable):
        """복구 완료 콜백 설정"""
        self._on_recovery_complete_callback = callback
    
    def set_driver_dead_callback(self, callback: Callable):
        """드라이버 죽음 콜백 설정"""
        self._on_driver_dead_callback = callback
    
    def set_driver_recovered_callback(self, callback: Callable):
        """드라이버 복구 콜백 설정 (하트비트 기반 자동 복구용)"""
        self._on_driver_recovered_callback = callback
    
    @property
    def is_driver_alive(self) -> bool:
        """드라이버 생존 여부"""
        return self._driver_alive
    
    # =========================================
    # 상태 조회
    # =========================================
    def get_robot_state(self) -> Optional[int]:
        """현재 로봇 상태 조회"""
        # 서비스 연결 확인 (처음 한 번만 대기)
        if not self._ensure_services_connected():
            return None
        
        # 이후에는 즉시 체크
        if not self.cli_get_state.service_is_ready():
            self.node.get_logger().warn('[StateMonitor] get_robot_state 서비스 준비 안됨')
            return None
        
        req = GetRobotState.Request()
        future = self.cli_get_state.call_async(req)
        
        # 폴링 대기 (최대 1초)
        start = time.time()
        while not future.done() and (time.time() - start) < 1.0:
            time.sleep(0.01)
        
        if future.done() and future.result():
            result = future.result()
            if result.success:
                return result.robot_state
        return None
    
    def get_current_z(self) -> Optional[float]:
        """현재 Z 높이 조회"""
        # 서비스 연결 확인 (처음 한 번만 대기)
        if not self._ensure_services_connected():
            return None
        
        # 이후에는 즉시 체크
        if not self.cli_get_posx.service_is_ready():
            self.node.get_logger().warn('[StateMonitor] get_current_posx 서비스 준비 안됨')
            return None
        
        req = GetCurrentPosx.Request()
        req.ref = DR_BASE
        future = self.cli_get_posx.call_async(req)
        
        start = time.time()
        while not future.done() and (time.time() - start) < 1.0:
            time.sleep(0.01)
        
        if future.done() and future.result():
            result = future.result()
            if result.success:
                pos_data = result.task_pos_info
                if isinstance(pos_data, list) and len(pos_data) > 0:
                    first_item = pos_data[0]
                    if hasattr(first_item, 'data') and len(first_item.data) >= 3:
                        z = float(first_item.data[2])
                        self.node.get_logger().info(f'[StateMonitor] Z = {z:.2f}mm')
                        return z
        
        self.node.get_logger().warn('[StateMonitor] get_current_z 실패 또는 타임아웃')
        return None
    
    # =========================================
    # 상태 판별
    # =========================================
    def is_safe_stop(self, state: int = None) -> bool:
        """SAFE_STOP 상태인지 (충돌 감지 - 노란 링)"""
        if state is None:
            state = self._current_state
        return state in (STATE_SAFE_STOP, STATE_SAFE_STOP2)
    
    def is_safe_off(self, state: int = None) -> bool:
        """SAFE_OFF 상태인지 (서보 OFF - 빨간 링)"""
        if state is None:
            state = self._current_state
        return state in (STATE_SAFE_OFF, STATE_SAFE_OFF2)
    
    def needs_recovery(self, state: int = None) -> bool:
        """복구가 필요한 상태인지 (SAFE_STOP 또는 SAFE_OFF)"""
        if state is None:
            state = self._current_state
        return self.is_safe_stop(state) or self.is_safe_off(state)
    
    def is_standby(self, state: int = None) -> bool:
        """STANDBY 상태인지 (정상)"""
        if state is None:
            state = self._current_state
        return state == STATE_STANDBY
    
    def is_recovery_mode(self, state: int = None) -> bool:
        """RECOVERY 모드인지"""
        if state is None:
            state = self._current_state
        return state == STATE_RECOVERY
    
    @property
    def current_state(self) -> Optional[int]:
        return self._current_state
    
    @property
    def current_state_name(self) -> str:
        return state_name(self._current_state) if self._current_state else "UNKNOWN"
    
    # =========================================
    # 모니터링
    # =========================================
    def start_monitoring(self):
        """모니터링 스레드 시작"""
        if self._monitoring:
            return
        
        self._monitoring = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        self.node.get_logger().info('[StateMonitor] 모니터링 시작')
    
    def stop_monitoring(self):
        """모니터링 스레드 정지"""
        self._monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
        self.node.get_logger().info('[StateMonitor] 모니터링 중지')
    
    def _monitor_loop(self):
        """모니터링 루프"""
        while self._monitoring:
            try:
                self._previous_state = self._current_state
                self._current_state = self.get_robot_state()
                
                # 드라이버 건강 체크
                if self._current_state is None:
                    self._consecutive_failures += 1
                    if self._consecutive_failures >= self._max_failures:
                        if self._driver_alive:
                            self._driver_alive = False
                            self._driver_was_dead = True  # 드라이버 죽음 기록
                            self.node.get_logger().error(
                                f'💀 [StateMonitor] DSR 드라이버 응답 없음! (연속 {self._consecutive_failures}회 실패)'
                            )
                            self.node.get_logger().error(
                                '💀 [StateMonitor] 런치 파일 재시작 필요!'
                            )
                            if self._on_driver_dead_callback:
                                self._on_driver_dead_callback()
                    time.sleep(self._monitor_interval)
                    continue
                else:
                    # 응답 성공 시 카운터 리셋
                    if self._consecutive_failures > 0:
                        # ★ 하트비트 기반 자동 복구: 드라이버가 죽었다가 살아났을 때
                        if not self._driver_alive and self._driver_was_dead:
                            self.node.get_logger().info('=' * 50)
                            self.node.get_logger().info('💚 [StateMonitor] DSR 드라이버 복구 감지!')
                            self.node.get_logger().info('💚 [StateMonitor] 하트비트 기반 자동 복구 시작')
                            self.node.get_logger().info('=' * 50)
                            
                            # 드라이버 복구 콜백 호출 (홈 이동 등)
                            if self._on_driver_recovered_callback:
                                self._on_driver_recovered_callback()
                            
                            self._driver_was_dead = False  # 플래그 리셋
                        
                        self._driver_alive = True
                    self._consecutive_failures = 0
                
                # 상태 전이 감지
                if self._previous_state != self._current_state:
                    self.node.get_logger().info(
                        f'[StateMonitor] 상태 변경: {state_name(self._previous_state)} → {state_name(self._current_state)}'
                    )
                    
                    # 충돌 감지 (SAFE_STOP 또는 SAFE_OFF)
                    if self.needs_recovery(self._current_state):
                        state_type = "SAFE_STOP (노란링)" if self.is_safe_stop(self._current_state) else "SAFE_OFF (빨간링)"
                        self.node.get_logger().warn(f'⚠️ [StateMonitor] 충돌 감지! {state_type}')
                        if self._on_collision_callback:
                            self._on_collision_callback()
                    
                    # 복구 완료 (→ STANDBY)
                    if self.is_standby(self._current_state) and self._previous_state in (STATE_RECOVERY, STATE_SAFE_OFF, STATE_SAFE_OFF2):
                        self.node.get_logger().info('✅ [StateMonitor] 복구 완료! STANDBY')
                        if self._on_recovery_complete_callback:
                            self._on_recovery_complete_callback()
                
            except Exception as e:
                self.node.get_logger().error(f'[StateMonitor] 오류: {e}')
            
            time.sleep(self._monitor_interval)
