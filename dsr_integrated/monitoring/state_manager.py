#!/usr/bin/env python3
"""
분류 작업 상태 관리 모듈
작업 상태, 통계, 파일 저장/로드 기능 제공
"""

import os
import json
import threading
from dataclasses import dataclass, field, asdict
from typing import Optional
from rclpy.node import Node

from ..config.constants import PHASE_PICK, STATE_FILE
from ..config.positions import HOME_POSITION


@dataclass
class SortStatistics:
    """분류 작업 통계"""
    completed: int = 0
    errors: int = 0
    small: int = 0
    medium: int = 0
    large: int = 0
    
    def reset(self):
        """통계 초기화"""
        self.completed = 0
        self.errors = 0
        self.small = 0
        self.medium = 0
        self.large = 0
    
    def increment(self, width_class: str):
        """분류 완료 시 통계 업데이트"""
        self.completed += 1
        if width_class == 'SMALL':
            self.small += 1
        elif width_class == 'MEDIUM':
            self.medium += 1
        elif width_class in ('LONG', 'LARGE'):
            self.large += 1
    
    def add_error(self):
        """에러 카운트 증가"""
        self.errors += 1
    
    def to_dict(self) -> dict:
        """딕셔너리로 변환"""
        return asdict(self)


@dataclass
class SortState:
    """분류 작업 상태"""
    # 작업 실행 상태
    is_running: bool = False
    is_paused: bool = False
    stop_requested: bool = False
    emergency_stopped: bool = False  # 비상정지 상태
    
    # 작업 단계
    current_phase: int = PHASE_PICK
    z_touch: float = field(default_factory=lambda: HOME_POSITION[2])
    cycle_count: int = 0
    last_width_class: Optional[str] = None
    
    # 컨베이어 연동 상태
    conveyor_mode: bool = False
    conveyor_detected: bool = False
    waiting_for_object: bool = False
    
    # DSR 연결 상태
    dsr_ready: bool = False


class StateManager:
    """분류 작업 상태 관리자"""
    
    def __init__(self, node: Node = None, state_file: str = STATE_FILE):
        """
        Args:
            node: ROS2 노드 (로깅용)
            state_file: 상태 저장 파일 경로
        """
        self.node = node
        self.state_file = state_file
        self.state = SortState()
        self.stats = SortStatistics()
        
        # 비상정지 해제 이벤트 (set=해제됨, clear=비상정지중)
        self.estop_event = threading.Event()
        self.estop_event.set()  # 초기상태: 비상정지 아님
    
    def log(self, msg: str, level: str = 'info'):
        """로깅 헬퍼"""
        if self.node:
            if level == 'info':
                self.node.get_logger().info(msg)
            elif level == 'warn':
                self.node.get_logger().warn(msg)
            elif level == 'error':
                self.node.get_logger().error(msg)
    
    # =========================================
    # 작업 상태 제어
    # =========================================
    def start(self) -> bool:
        """작업 시작"""
        if self.state.is_running:
            return False
        
        self.state.is_running = True
        self.state.stop_requested = False
        self.state.is_paused = False
        return True
    
    def stop(self):
        """작업 정지 (완전 종료)"""
        self.state.stop_requested = True
        self.state.is_running = False
        self.save()
    
    def emergency_stop(self):
        """비상정지 (일시 정지, 이어서 재개 가능)"""
        self.state.emergency_stopped = True
        self.estop_event.clear()  # 비상정지 상태로 설정
        self.log('🛑 [StateManager] 비상정지 활성화')
        self.save()
    
    def emergency_release(self):
        """비상정지 해제 (이어서 재개)"""
        self.state.emergency_stopped = False
        self.estop_event.set()  # 비상정지 해제 - 대기중인 스레드 깨움
        self.log('▶️ [StateManager] 비상정지 해제')
    
    def wait_for_estop_release(self, timeout: float = None) -> bool:
        """비상정지 해제될 때까지 대기 (비블로킹)
        
        Args:
            timeout: 타임아웃 (초), None이면 무한 대기
            
        Returns:
            True: 비상정지 해제됨
            False: 타임아웃 또는 중단 요청
        """
        return self.estop_event.wait(timeout=timeout)
    
    def is_emergency_stopped(self) -> bool:
        """비상정지 상태 확인"""
        return self.state.emergency_stopped
    
    def pause(self):
        """작업 일시정지"""
        self.state.is_paused = True
        self.save()
    
    def resume(self):
        """작업 재개"""
        self.state.is_paused = False
    
    def finish(self):
        """작업 완료"""
        self.state.is_running = False
    
    def reset(self):
        """상태 초기화"""
        self.state = SortState()
        self.state.z_touch = HOME_POSITION[2]
        self.stats.reset()
        self.save()
    
    # =========================================
    # 컨베이어 상태 제어
    # =========================================
    def set_conveyor_mode(self, enabled: bool):
        """컨베이어 모드 설정"""
        self.state.conveyor_mode = enabled
        if not enabled:
            self.state.waiting_for_object = False
    
    def set_waiting_for_object(self, waiting: bool):
        """물체 대기 상태 설정"""
        self.state.waiting_for_object = waiting
    
    def set_conveyor_detected(self, detected: bool):
        """컨베이어 물체 감지 상태 설정"""
        self.state.conveyor_detected = detected
    
    def can_start_auto_cycle(self) -> bool:
        """자동 사이클 시작 가능 여부"""
        return (
            self.state.conveyor_mode and
            self.state.waiting_for_object and
            not self.state.is_running and
            self.state.conveyor_detected
        )
    
    # =========================================
    # 작업 단계 관리
    # =========================================
    def set_phase(self, phase: int):
        """작업 단계 설정"""
        self.state.current_phase = phase
        self.save()
    
    def set_z_touch(self, z: float):
        """접촉 높이 설정"""
        self.state.z_touch = z
    
    def complete_cycle(self, width_class: str):
        """사이클 완료 처리"""
        self.state.cycle_count += 1
        self.state.last_width_class = width_class
        self.stats.increment(width_class)
        self.save()
    
    # =========================================
    # 파일 저장/로드
    # =========================================
    def save(self):
        """상태 파일 저장"""
        data = {
            "phase": int(self.state.current_phase),
            "z_touch": float(self.state.z_touch),
            "cycle_count": int(self.state.cycle_count),
        }
        try:
            with open(self.state_file, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            self.log(f'상태 저장 실패: {e}', 'warn')
    
    def load(self):
        """상태 파일 로드"""
        if not os.path.exists(self.state_file):
            return
        
        try:
            with open(self.state_file, 'r') as f:
                data = json.load(f)
                self.state.current_phase = data.get("phase", PHASE_PICK)
                self.state.z_touch = data.get("z_touch", HOME_POSITION[2])
                self.state.cycle_count = data.get("cycle_count", 0)
        except Exception as e:
            self.log(f'상태 로드 실패: {e}', 'warn')
    
    # =========================================
    # 상태 조회
    # =========================================
    def get_status_dict(self) -> dict:
        """현재 상태를 딕셔너리로 반환"""
        return {
            'is_running': self.state.is_running,
            'is_paused': self.state.is_paused,
            'current_phase': 'PICK' if self.state.current_phase == PHASE_PICK else 'PLACE',
            'cycle_count': self.state.cycle_count,
            'last_classification': self.state.last_width_class,
            'dsr_ready': self.state.dsr_ready,
            'conveyor_mode': self.state.conveyor_mode,
            'conveyor_detected': self.state.conveyor_detected,
            'waiting_for_object': self.state.waiting_for_object,
            **self.stats.to_dict()
        }
