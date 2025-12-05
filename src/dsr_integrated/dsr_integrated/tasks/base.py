#!/usr/bin/env python3
"""
Task 베이스 클래스
- 모든 태스크에 공통적인 STANDBY 상태 체크
- 복구 중 태스크 실행 방지
"""

from abc import ABC, abstractmethod
from typing import Optional, Callable

from rclpy.node import Node

from ..monitoring.state_monitor import RobotStateMonitor, state_name
from ..safety.safety_manager import SafetyManager


class BaseTask(ABC):
    """태스크 베이스 클래스"""
    
    # 상태 코드 상수
    STATE_STANDBY = 1
    STATE_MOVING = 2
    
    def __init__(
        self,
        node: Node,
        state_monitor: Optional[RobotStateMonitor] = None,
        recovery_checker: Optional[Callable[[], bool]] = None
    ):
        """
        Args:
            node: ROS2 노드 인스턴스
            state_monitor: 로봇 상태 모니터 (None이면 상태 체크 스킵)
            recovery_checker: 복구 중인지 확인하는 콜백 함수 (예: lambda: recovery.is_recovering)
        """
        self.node = node
        self.state_monitor = state_monitor
        self._recovery_checker = recovery_checker
    
    def is_ready(self) -> tuple[bool, str]:
        """
        태스크 시작 가능 상태인지 확인
        
        Returns:
            (ready, reason): 준비 상태 여부와 불가능한 경우 이유
        """
        # 1. 비상정지 상태 체크
        if SafetyManager.is_emergency_stopped():
            return False, "비상정지 상태입니다"
        
        # 2. 복구 진행 중 체크
        if self._recovery_checker and self._recovery_checker():
            return False, "복구 진행 중입니다"
        
        # 3. STANDBY 상태 체크 (모니터가 있는 경우)
        if self.state_monitor:
            state = self.state_monitor.get_robot_state()
            if state is None:
                return False, "로봇 상태를 확인할 수 없습니다"
            
            if state not in (self.STATE_STANDBY, self.STATE_MOVING):
                return False, f"로봇이 준비 상태가 아닙니다 (현재: {state_name(state)})"
        
        return True, ""
    
    def _check_ready_or_log(self) -> bool:
        """
        준비 상태 체크 후 로그 출력
        
        Returns:
            준비 상태 여부
        """
        ready, reason = self.is_ready()
        if not ready:
            self._log('WARNING', f'태스크 시작 불가: {reason}')
        return ready
    
    def _log(self, level: str, message: str):
        """로그 출력 헬퍼"""
        logger = self.node.get_logger()
        if level == 'INFO':
            logger.info(f'[{self.__class__.__name__}] {message}')
        elif level == 'WARNING':
            logger.warn(f'[{self.__class__.__name__}] {message}')
        elif level == 'ERROR':
            logger.error(f'[{self.__class__.__name__}] {message}')
        else:
            logger.debug(f'[{self.__class__.__name__}] {message}')
    
    @abstractmethod
    def execute(self, *args, **kwargs) -> bool:
        """
        태스크 실행 (서브클래스에서 구현)
        
        Returns:
            성공 여부
        """
        pass
    
    def safe_execute(self, *args, **kwargs) -> tuple[bool, str]:
        """
        안전한 태스크 실행 (상태 체크 후 실행)
        
        Returns:
            (success, message): 성공 여부와 메시지
        """
        ready, reason = self.is_ready()
        if not ready:
            return False, reason
        
        try:
            success = self.execute(*args, **kwargs)
            return success, "" if success else "태스크 실행 실패"
        except Exception as e:
            self._log('ERROR', f'태스크 실행 중 예외: {e}')
            return False, str(e)
