#!/usr/bin/env python3
"""
전역 데이터 저장소 모듈
웹 서버와 ROS2 노드 간 공유되는 데이터
"""

from datetime import datetime, timezone, timedelta
from typing import Callable, Optional

# 물류 상태 변경 시 호출될 콜백 (소켓 브로드캐스트용)
_logistics_broadcast_callback: Optional[Callable] = None


def set_logistics_broadcast_callback(callback: Callable):
    """물류 상태 브로드캐스트 콜백 등록"""
    global _logistics_broadcast_callback
    _logistics_broadcast_callback = callback

# ============================================
# 로봇 상태 데이터
# ============================================
robot_data = {
    'connected': False,
    'timestamp': 0,
    
    # 조인트 데이터 (Actual)
    'actual_joint_position': [0.0] * 6,
    'actual_joint_velocity': [0.0] * 6,
    'actual_joint_torque': [0.0] * 6,
    'joint_temperature': [25.0] * 6,
    'actual_motor_torque': [0.0] * 6,
    'actual_je': [0.0] * 6,
    
    # 조인트 데이터 (Target)
    'target_joint_position': [0.0] * 6,
    'target_joint_velocity': [0.0] * 6,
    
    # 조인트 데이터 (기타)
    'actual_joint_position_abs': [0.0] * 6,
    'actual_bk': [0] * 6,
    
    # TCP 데이터 (Actual)
    'actual_tcp_position': [0.0] * 6,
    'actual_tcp_velocity': [0.0] * 6,
    
    # TCP 데이터 (Target)
    'target_tcp_position': [0.0] * 6,
    'target_tcp_velocity': [0.0] * 6,
    
    # 힘/토크 데이터
    'external_tcp_force': [0.0] * 6,
    'target_force': [0.0] * 6,
    'actual_tool_force': [0.0] * 6,
    
    # Controller I/O
    'controller_digital_input': 0,
    'controller_digital_output': 0,
    'controller_analog_input': [0.0, 0.0],
    'controller_analog_output': [0.0, 0.0],
    
    # Flange I/O
    'flange_digital_input': 0,
    'flange_digital_output': 0,
    'flange_analog_input': [0.0] * 4,
    
    # 시스템 상태
    'robot_mode': 1,
    'robot_state': 1,
    'operation_speed_rate': 100,
    'singularity': 0.0,
    'solution_space': 0,
    'program_running': False,
    'access_control': 0,
    'running_time': 0.0,
    'safe_stop_reset_req': False,
    
    # 툴/좌표계
    'tool_index': 0,
    'tcp_index': 0,
    'gravity_dir': [0.0, 0.0, -9.81],
    
    # 에러 정보
    'error_level': 0,
    'error_code': 0,
    'error_msg': '',
    
    # 툴/TCP 정보
    'current_tool_name': '',
    'current_tcp_name': '',
    'tool_flange_posx': [0.0] * 6,
    
    # 목표(Desired) 위치
    'desired_tcp_position': [0.0] * 6,
    'desired_joint_position': [0.0] * 6,
    'desired_tcp_velocity': [0.0] * 6,
    'desired_joint_velocity': [0.0] * 6,
}

# ============================================
# 분류 상태 데이터
# ============================================
sort_status = {
    'running': False,
    'paused': False,
    'phase': 'PICK',
    'cycle_count': 0,          # 현재 사이클 (session)
    'last_width': None,
    'dsr_ready': False,
}

# ============================================
# 물류 적재 상태 데이터 (robot_pick_node 호환)
# ============================================
logistics_status = {
    # 분류별 적재 개수
    'stack_count': {
        'SMALL': 0,
        'MEDIUM': 0,
        'LONG': 0
    },
    # 적재 이력
    'placed_boxes': [],
    # Pick 상태
    'z_touch': 0.0,
    'pick_ok': False,
    # 총 개수
    'total_count': 0,
}

# ============================================
# 컨베이어 상태 데이터
# ============================================
conveyor_status = {
    'connected': False,
    'status': 'IDLE',
    'status_code': 0,
}

# ============================================
# UI 상태 (로컬/외부 웹 동기화용)
# ============================================
ui_state = {
    'pendulum_running': False,
    'is_moving': False,
    'is_stopped': False,
    'paused_task': None
}

# ============================================
# 원테이크 시나리오 상태
# ============================================
one_take_status = {
    'running': False,
    'phase': 'IDLE',           # IDLE, SORTING, STACKING, COMPLETE, ERROR
    'sorting_complete': False,  # 1차 분류 9개 완료
    'stacking_complete': False, # 2차 적재 6개 완료
    'stacking_step': 0,        # 현재 적재 단계 (0-6)
    'total_sorted': 0,         # 분류된 박스 개수
    'target_count': 9,         # 목표 분류 개수
    'start_stacking': False,   # 2차 적재 시작 트리거 (내부용)
}

# ============================================
# 현재 작업 상태
# ============================================
current_task = {
    'type': None,
    'params': {},
    'paused': False
}

# ============================================
# 로그 저장소
# ============================================
logs = []


def get_kst_time() -> str:
    """한국시간 문자열 반환"""
    kst = timezone(timedelta(hours=9))
    return datetime.now(kst).strftime('%H:%M:%S')


def add_log(level: str, message: str):
    """로그 추가"""
    logs.insert(0, {
        'time': get_kst_time(),
        'level': level,
        'message': message
    })
    if len(logs) > 100:
        logs.pop()


def reset_robot_data():
    """로봇 데이터 초기화"""
    robot_data['connected'] = False
    robot_data['timestamp'] = 0
    robot_data['actual_joint_position'] = [0.0] * 6
    robot_data['actual_tcp_position'] = [0.0] * 6


def update_sort_status(data: dict):
    """분류 상태 업데이트"""
    sort_status.update(data)


def update_logistics_status(
    stack_count: dict = None,
    placed_boxes: list = None,
    z_touch: float = None,
    pick_ok: bool = None
):
    """물류 적재 상태 업데이트"""
    if stack_count is not None:
        logistics_status['stack_count'] = stack_count.copy()
        logistics_status['total_count'] = sum(stack_count.values())
    if placed_boxes is not None:
        logistics_status['placed_boxes'] = placed_boxes.copy()
    if z_touch is not None:
        logistics_status['z_touch'] = z_touch
    if pick_ok is not None:
        logistics_status['pick_ok'] = pick_ok
    
    # 콜백 호출 (소켓 브로드캐스트)
    if _logistics_broadcast_callback:
        try:
            _logistics_broadcast_callback(logistics_status)
        except Exception as e:
            print(f"[data_store] logistics broadcast error: {e}")


def reset_logistics_status():
    """물류 적재 상태 초기화"""
    logistics_status['stack_count'] = {'SMALL': 0, 'MEDIUM': 0, 'LONG': 0}
    logistics_status['placed_boxes'] = []
    logistics_status['z_touch'] = 0.0
    logistics_status['pick_ok'] = False
    logistics_status['total_count'] = 0


def reset_all_status():
    """전체 상태 초기화 (새로 시작)"""
    # 분류 상태 초기화
    sort_status['running'] = False
    sort_status['paused'] = False
    sort_status['phase'] = 'PICK'
    sort_status['cycle_count'] = 0
    sort_status['last_width'] = None
    
    # 물류 적재 상태 초기화
    reset_logistics_status()
    
    # 원테이크 상태 초기화
    one_take_status['running'] = False
    one_take_status['phase'] = 'IDLE'
    one_take_status['sorting_complete'] = False
    one_take_status['stacking_complete'] = False
    one_take_status['stacking_step'] = 0
    one_take_status['total_sorted'] = 0
    one_take_status['start_stacking'] = False
    
    add_log('INFO', '전체 상태 초기화 완료')


def update_conveyor_status(status: str = None, code: int = None):
    """컨베이어 상태 업데이트"""
    if status is not None:
        conveyor_status['status'] = status
        conveyor_status['connected'] = True
    if code is not None:
        conveyor_status['status_code'] = code
