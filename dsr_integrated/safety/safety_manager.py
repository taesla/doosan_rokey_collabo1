"""
Safety Manager - 로봇 안전 상태 중앙 관리
==========================================
모든 로봇 제어 모듈에서 이 모듈을 통해 안전 상태를 확인/제어합니다.

사용법:
    from dsr_integrated.safety_manager import SafetyManager
    
    # 비상정지 상태 확인
    if SafetyManager.is_emergency_stopped():
        return  # 명령 차단
    
    # 비상정지 활성화/해제
    SafetyManager.emergency_stop()
    SafetyManager.emergency_release()
    
    # 모션 컨텍스트 저장/복원 (이어서 재개용)
    SafetyManager.save_motion_context("pendulum", {"target": [0,0,90,0,105,0], "vel": 30})
    context = SafetyManager.get_motion_context("pendulum")
"""

import threading
import time
from typing import Callable, List, Optional, Dict, Any
from dataclasses import dataclass, field
from enum import IntEnum


class SafetyState(IntEnum):
    """안전 상태 열거형"""
    NORMAL = 0          # 정상 운영
    EMERGENCY_STOP = 1  # 비상정지
    PAUSED = 2          # 일시정지
    ERROR = 3           # 에러 상태


@dataclass
class MotionContext:
    """모션 컨텍스트 - 비상정지 시 저장, 재개 시 복원"""
    motion_type: str = ""           # "pendulum", "sort_pick", "sort_place", etc.
    target_position: List[float] = field(default_factory=list)  # 목표 위치
    velocity: float = 0.0           # 속도
    additional_data: Dict[str, Any] = field(default_factory=dict)  # 추가 데이터
    timestamp: float = 0.0          # 저장 시간


@dataclass
class SafetyStatus:
    """안전 상태 정보"""
    state: SafetyState = SafetyState.NORMAL
    reason: str = ""
    timestamp: float = 0.0


class SafetyManager:
    """
    싱글톤 패턴의 안전 관리자
    모든 로봇 제어에서 공유되는 전역 안전 상태 관리
    
    비상정지 시:
    1. MoveStop으로 즉시 정지
    2. GetDesiredPosj로 목표 위치 저장
    
    비상정지 해제 시:
    1. 저장된 목표 위치로 movej 호출 → 이어서 재개
    """
    _instance = None
    _lock = threading.Lock()
    
    # 상태
    _state: SafetyState = SafetyState.NORMAL
    _reason: str = ""
    
    # 비상정지 시 저장된 목표 위치 (GetDesiredPosj로 읽어옴)
    _saved_target_posj: Optional[List[float]] = None
    _saved_velocity: float = 30.0  # 기본 속도
    
    # 모션 컨텍스트 저장소 (이어서 재개용) - 레거시 호환
    _motion_contexts: Dict[str, MotionContext] = {}
    
    # 콜백 리스트
    _on_emergency_stop_callbacks: List[Callable] = []
    _on_emergency_release_callbacks: List[Callable] = []
    _on_state_change_callbacks: List[Callable[[SafetyState, SafetyState], None]] = []
    
    # ROS 노드 참조 (서비스 호출용)
    _ros_node = None
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance
    
    @classmethod
    def initialize(cls, ros_node=None):
        """초기화 - ROS 노드 연결"""
        cls._ros_node = ros_node
        cls._state = SafetyState.NORMAL
        cls._reason = ""
        print("✅ SafetyManager 초기화 완료")
    
    @classmethod
    def set_ros_node(cls, ros_node):
        """ROS 노드 설정"""
        cls._ros_node = ros_node
    
    # ========== 상태 조회 ==========
    
    @classmethod
    def get_state(cls) -> SafetyState:
        """현재 안전 상태 반환"""
        return cls._state
    
    @classmethod
    def get_status(cls) -> SafetyStatus:
        """상세 상태 정보 반환"""
        import time
        return SafetyStatus(
            state=cls._state,
            reason=cls._reason,
            timestamp=time.time()
        )
    
    @classmethod
    def is_emergency_stopped(cls) -> bool:
        """비상정지 상태인지 확인"""
        return cls._state == SafetyState.EMERGENCY_STOP
    
    @classmethod
    def is_safe_to_move(cls) -> bool:
        """로봇 이동이 안전한지 확인"""
        return cls._state == SafetyState.NORMAL
    
    @classmethod
    def is_paused(cls) -> bool:
        """일시정지 상태인지 확인"""
        return cls._state == SafetyState.PAUSED
    
    # ========== 상태 제어 ==========
    
    @classmethod
    def emergency_stop(cls, reason: str = "사용자 비상정지") -> bool:
        """
        비상정지 활성화
        - 모든 로봇 모션 즉시 일시정지
        - 새로운 모션 명령 차단
        """
        with cls._lock:
            if cls._state == SafetyState.EMERGENCY_STOP:
                return True  # 이미 비상정지 상태
            
            old_state = cls._state
            cls._state = SafetyState.EMERGENCY_STOP
            cls._reason = reason
            
            print(f"🛑 [SafetyManager] 비상정지 활성화: {reason}")
            
            # ROS 서비스 호출 (MovePause)
            if cls._ros_node:
                cls._ros_node._call_move_pause()
            
            # 콜백 실행
            cls._notify_state_change(old_state, cls._state)
            for callback in cls._on_emergency_stop_callbacks:
                try:
                    callback()
                except Exception as e:
                    print(f"⚠️ 비상정지 콜백 에러: {e}")
            
            return True
    
    @classmethod
    def emergency_release(cls) -> bool:
        """
        비상정지 해제
        - 상태를 NORMAL로 변경 (각 모션 루프가 알아서 이어서 재개)
        """
        with cls._lock:
            if cls._state != SafetyState.EMERGENCY_STOP:
                return True  # 비상정지 상태가 아님
            
            old_state = cls._state
            cls._state = SafetyState.NORMAL
            cls._reason = ""
            
            print(f"▶️ [SafetyManager] 비상정지 해제")
            
            # 콜백 실행
            cls._notify_state_change(old_state, cls._state)
            for callback in cls._on_emergency_release_callbacks:
                try:
                    callback()
                except Exception as e:
                    print(f"⚠️ 비상정지 해제 콜백 에러: {e}")
            
            return True
            
            return True
    
    @classmethod
    def set_saved_target(cls, target_posj: List[float], vel: float = 30.0):
        """비상정지 시 저장된 목표 위치 설정 (외부에서 호출)"""
        cls._saved_target_posj = list(target_posj)
        cls._saved_velocity = vel
        print(f"💾 [SafetyManager] 목표 위치 저장: {[f'{p:.1f}' for p in target_posj]}")
    
    @classmethod
    def get_saved_target(cls) -> Optional[List[float]]:
        """저장된 목표 위치 반환"""
        return cls._saved_target_posj
    
    @classmethod
    def clear_saved_target(cls):
        """저장된 목표 위치 초기화"""
        cls._saved_target_posj = None
    
    @classmethod
    def pause(cls, reason: str = "일시정지") -> bool:
        """일시정지 (비상정지보다 가벼운 정지)"""
        with cls._lock:
            if cls._state == SafetyState.EMERGENCY_STOP:
                return False  # 비상정지 중에는 불가
            
            old_state = cls._state
            cls._state = SafetyState.PAUSED
            cls._reason = reason
            
            print(f"⏸️ [SafetyManager] 일시정지: {reason}")
            
            if cls._ros_node:
                cls._ros_node._call_move_pause()
            
            cls._notify_state_change(old_state, cls._state)
            return True
    
    @classmethod
    def resume(cls) -> bool:
        """일시정지 해제"""
        with cls._lock:
            if cls._state == SafetyState.EMERGENCY_STOP:
                return False  # 비상정지 중에는 resume 불가, emergency_release 사용
            
            if cls._state != SafetyState.PAUSED:
                return True
            
            old_state = cls._state
            cls._state = SafetyState.NORMAL
            cls._reason = ""
            
            print(f"▶️ [SafetyManager] 재개")
            
            if cls._ros_node:
                cls._ros_node._call_move_resume()
            
            cls._notify_state_change(old_state, cls._state)
            return True
    
    @classmethod
    def reset(cls) -> bool:
        """상태 리셋 (에러 복구용)"""
        with cls._lock:
            old_state = cls._state
            cls._state = SafetyState.NORMAL
            cls._reason = ""
            
            print(f"🔄 [SafetyManager] 상태 리셋")
            cls._notify_state_change(old_state, cls._state)
            return True
    
    # ========== 모션 명령 가드 ==========
    
    @classmethod
    def check_before_motion(cls, motion_name: str = "") -> bool:
        """
        모션 명령 전 안전 체크
        False 반환 시 모션 명령 차단해야 함
        
        사용 예:
            if not SafetyManager.check_before_motion("movej"):
                return False
        """
        if cls._state == SafetyState.EMERGENCY_STOP:
            print(f"⛔ [SafetyManager] 비상정지 중 - '{motion_name}' 명령 차단")
            return False
        
        if cls._state == SafetyState.PAUSED:
            print(f"⏸️ [SafetyManager] 일시정지 중 - '{motion_name}' 명령 대기")
            return False
        
        if cls._state == SafetyState.ERROR:
            print(f"❌ [SafetyManager] 에러 상태 - '{motion_name}' 명령 차단")
            return False
        
        return True
    
    # ========== 콜백 등록 ==========
    
    @classmethod
    def on_emergency_stop(cls, callback: Callable):
        """비상정지 시 호출될 콜백 등록"""
        if callback not in cls._on_emergency_stop_callbacks:
            cls._on_emergency_stop_callbacks.append(callback)
    
    @classmethod
    def on_emergency_release(cls, callback: Callable):
        """비상정지 해제 시 호출될 콜백 등록"""
        if callback not in cls._on_emergency_release_callbacks:
            cls._on_emergency_release_callbacks.append(callback)
    
    @classmethod
    def on_state_change(cls, callback: Callable[[SafetyState, SafetyState], None]):
        """상태 변경 시 호출될 콜백 등록 (old_state, new_state)"""
        if callback not in cls._on_state_change_callbacks:
            cls._on_state_change_callbacks.append(callback)
    
    @classmethod
    def remove_callback(cls, callback: Callable):
        """콜백 제거"""
        for cb_list in [cls._on_emergency_stop_callbacks, 
                        cls._on_emergency_release_callbacks,
                        cls._on_state_change_callbacks]:
            if callback in cb_list:
                cb_list.remove(callback)
    
    @classmethod
    def _notify_state_change(cls, old_state: SafetyState, new_state: SafetyState):
        """상태 변경 알림"""
        for callback in cls._on_state_change_callbacks:
            try:
                callback(old_state, new_state)
            except Exception as e:
                print(f"⚠️ 상태 변경 콜백 에러: {e}")
    
    # ========== 모션 컨텍스트 관리 (이어서 재개용) ==========
    
    @classmethod
    def save_motion_context(cls, motion_id: str, target_position: List[float], 
                           velocity: float = 0.0, **kwargs):
        """
        모션 컨텍스트 저장 - movej 호출 전에 저장
        비상정지 후 재개 시 이 정보로 목표 위치까지 이어서 이동
        
        Args:
            motion_id: 모션 식별자 ("pendulum", "sort_pick", "sort_place" 등)
            target_position: 목표 위치 (조인트 또는 TCP)
            velocity: 속도
            **kwargs: 추가 데이터
        """
        cls._motion_contexts[motion_id] = MotionContext(
            motion_type=motion_id,
            target_position=list(target_position),
            velocity=velocity,
            additional_data=kwargs,
            timestamp=time.time()
        )
    
    @classmethod
    def get_motion_context(cls, motion_id: str) -> Optional[MotionContext]:
        """저장된 모션 컨텍스트 반환"""
        return cls._motion_contexts.get(motion_id)
    
    @classmethod
    def clear_motion_context(cls, motion_id: str):
        """모션 컨텍스트 삭제 - 모션 완료 후 호출"""
        if motion_id in cls._motion_contexts:
            del cls._motion_contexts[motion_id]
    
    @classmethod
    def has_pending_motion(cls, motion_id: str) -> bool:
        """재개 대기 중인 모션이 있는지 확인"""
        return motion_id in cls._motion_contexts
    
    @classmethod
    def clear_all_contexts(cls):
        """모든 모션 컨텍스트 삭제"""
        cls._motion_contexts.clear()


# 편의 함수 (모듈 레벨에서 바로 사용 가능)
def is_emergency_stopped() -> bool:
    """비상정지 상태 확인"""
    return SafetyManager.is_emergency_stopped()

def is_safe_to_move() -> bool:
    """이동 가능 상태 확인"""
    return SafetyManager.is_safe_to_move()

def check_safety(motion_name: str = "") -> bool:
    """모션 전 안전 체크"""
    return SafetyManager.check_before_motion(motion_name)
