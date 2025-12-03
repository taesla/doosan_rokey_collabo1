#!/usr/bin/env python3
"""
로봇 제어 상수 모듈
Force 센서, 속도, 오프셋 등 모든 상수값 관리

※ robot_pick_node/dlar_sort/config.py 기준으로 업데이트됨
"""
import os

# =========================================
# 로봇 ID/모델 설정
# =========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# =========================================
# Force 센서 설정 (robot_pick_node 기준)
# =========================================
FORCE_THRESHOLD = 30.0      # 접촉 감지 임계값 (N) - robot_pick_node: 30.0
FORCE_PUSH = 50.0           # Compliance 모드에서 인가할 힘 (N)
MAX_DESCENT = 100.0         # 최대 하강 거리 (mm)
STEP_Z = 0.5                # Force 감지 시 스텝 (mm)

# Place 시 Force 탐색 설정
PLACE_FORCE_THRESHOLD = 5.0     # 배치 시 접촉 감지 임계값 (N)
PLACE_FORCE_THRESHOLD_ML = 2.5  # MEDIUM/LONG 배치 시 감도 (N)
SAFE_FORCE_LIMIT = 10.0         # Force 탐색 시 안전 상한 (N)

# =========================================
# 픽업/배치 오프셋 (mm) - robot_pick_node 기준
# =========================================
UP_OFFSET = 30.0            # 위로 올라가는 오프셋
GRIP_OFFSET = 20.0          # 그리퍼 오프셋
PICK_EXTRA_DOWN = 30.0      # 픽업 시 추가 하강
PLACE_EXTRA_DOWN = 50.0     # 배치 시 추가 하강
FINAL_PUSH = 15.0           # 최종 밀어넣기
PLACE_REDUCE = 30.0         # 배치 높이 감소
Z_OFFSET_DOWN = 10.0        # 배치 시 추가 하강 오프셋

# =========================================
# 층별 적재 높이 (MEDIUM/LONG)
# =========================================
LAYER_HEIGHT_MEDIUM = 45.0  # MEDIUM 한 층 높이 + 여유
LAYER_HEIGHT_LONG = 65.0    # LONG 한 층 높이 + 여유

# =========================================
# 컨베이어 높이 보정
# =========================================
CONVEYOR_HEIGHT_OFFSET = 90.0  # 컨베이어 높이 (mm) - robot_pick_node: 90.0

# =========================================
# 안전 Z 한계 (mm)
# =========================================
SAFE_Z_PICK = 103.0         # 픽업 시 안전 Z 한계
SAFE_Z_PLACE = 12.5         # 배치 시 안전 Z 한계

# =========================================
# 속도/가속도 설정
# =========================================
VELOCITY_MOVE = 200.0       # 이동 속도 (mm/s)
ACCEL_MOVE = 400.0          # 이동 가속도 (mm/s²)
VELOCITY_PICK = 75.0        # 픽업 하강 속도 (mm/s) - robot_pick_node: 75.0
ACCEL_PICK = 200.0          # 픽업 하강 가속도 (mm/s²)
VELOCITY_FORCE = 200.0      # Force 하강 속도 (mm/s)
ACCEL_FORCE = 400.0         # Force 하강 가속도 (mm/s²)

# =========================================
# 작업 단계 상수
# =========================================
PHASE_PICK = 0
PHASE_PLACE = 1

# =========================================
# 컨베이어 상태 코드
# =========================================
CONVEYOR_DETECT = 1         # 물체 감지
CONVEYOR_WAITING = 2        # 대기 중
CONVEYOR_RUNNING = 3        # 동작 중

# =========================================
# DSR 상수
# =========================================
DR_BASE = 0
DR_TOOL = 1
DR_MV_MOD_ABS = 0           # 절대 좌표 모드
DR_MV_MOD_REL = 1           # 상대 좌표 모드
DR_FC_MOD_ABS = 0           # Force Control 절대 모드

# =========================================
# 로봇 상태 코드 (GetRobotState)
# =========================================
STATE_INITIALIZING = 0
STATE_STANDBY = 1
STATE_MOVING = 2
STATE_SAFE_OFF = 3
STATE_TEACHING = 4
STATE_SAFE_STOP = 5
STATE_EMERGENCY_STOP = 6
STATE_HOMING = 7
STATE_RECOVERY = 8
STATE_SAFE_STOP2 = 9
STATE_SAFE_OFF2 = 10
STATE_NOT_READY = 15

# =========================================
# 로봇 제어 명령 (SetRobotControl)
# =========================================
CONTROL_INIT_CONFIG = 0
CONTROL_ENABLE_OPERATION = 1
CONTROL_RESET_SAFE_STOP = 2    # SAFE_STOP → STANDBY
CONTROL_RESET_SAFE_OFF = 3     # SAFE_OFF → STANDBY (SERVO ON)
CONTROL_RECOVERY_SAFE_STOP = 4 # SAFE_STOP2 → RECOVERY
CONTROL_RECOVERY_SAFE_OFF = 5  # SAFE_OFF2 → RECOVERY
CONTROL_RECOVERY_BACKDRIVE = 6 # H/W 기반 복구
CONTROL_RESET_RECOVERY = 7     # RECOVERY → STANDBY

# =========================================
# 안전 모드 (SetSafetyMode)
# =========================================
SAFETY_MODE_MANUAL = 0
SAFETY_MODE_AUTONOMOUS = 1
SAFETY_MODE_RECOVERY = 2
SAFETY_MODE_BACKDRIVE = 3
SAFETY_MODE_MEASURE = 4
SAFETY_MODE_INITIALIZE = 5

# 안전 모드 이벤트
SAFETY_EVENT_ENTER = 0
SAFETY_EVENT_MOVE = 1
SAFETY_EVENT_STOP = 2
SAFETY_EVENT_LAST = 3

# =========================================
# Jog 축 인덱스
# =========================================
JOG_JOINT_1 = 0
JOG_JOINT_2 = 1
JOG_JOINT_3 = 2
JOG_JOINT_4 = 3
JOG_JOINT_5 = 4
JOG_JOINT_6 = 5
JOG_TASK_X = 6
JOG_TASK_Y = 7
JOG_TASK_Z = 8
JOG_TASK_RX = 9
JOG_TASK_RY = 10
JOG_TASK_RZ = 11

# =========================================
# 파일 경로
# =========================================
STATE_FILE = os.path.expanduser("~/.dlar_sort_state.json")

# =========================================
# Compliance Control 기본 강성
# =========================================
DEFAULT_STIFFNESS = [1000.0, 1000.0, 200.0, 500.0, 500.0, 500.0]

# =========================================
# 충돌 복구 설정
# =========================================
RECOVERY_Z_THRESHOLD = 100.0    # 바닥 충돌 판별 기준 Z 높이 (mm)
RECOVERY_JOG_TIME = 1.5         # Jog 상승 시간 (초)
RECOVERY_JOG_SPEED = 20.0       # Jog 상승 속도 (mm/s)
RECOVERY_JOG_AXIS_Z = 8         # Z축 Jog (JOG_TASK_Z)

# 제어 명령 단축 alias
CTRL_RESET_SAFE_STOP = CONTROL_RESET_SAFE_STOP   # 2
CTRL_SERVO_ON = CONTROL_RESET_SAFE_OFF           # 3
CTRL_RESET_RECOVERY = CONTROL_RESET_RECOVERY     # 7
