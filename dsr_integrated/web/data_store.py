#!/usr/bin/env python3
"""
전역 데이터 저장소 모듈
웹 서버와 ROS2 노드 간 공유되는 데이터
"""

from datetime import datetime, timezone, timedelta

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
    'cycle_count': 0,
    'last_width': None,
    'dsr_ready': False,
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


def update_conveyor_status(status: str = None, code: int = None):
    """컨베이어 상태 업데이트"""
    if status is not None:
        conveyor_status['status'] = status
        conveyor_status['connected'] = True
    if code is not None:
        conveyor_status['status_code'] = code
