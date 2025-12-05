"""
DSR Integrated - 물류 분류 로봇 시스템

Doosan M0609 로봇을 사용한 물류 자동 분류 시스템

Package Structure:
    core/           - 로봇 제어 기반 시스템
    monitoring/     - 상태 모니터링 및 관리
    safety/         - 안전 관리 및 충돌 복구
    tasks/          - 작업 기능 (pick_place, pendulum)
    integration/    - 외부 시스템 연동 (conveyor, firebase)
    web/            - 웹 UI 서버
    nodes/          - ROS2 노드 진입점
    config/         - 설정 파일
"""

__version__ = '1.0.0'

# Core
from .core import RobotController

# Monitoring
from .monitoring import RobotStateMonitor, StateManager

# Safety
from .safety import SafetyManager, CollisionRecovery

# Integration
from .integration import ConveyorHandler, FirebaseHandler

# Nodes
from .nodes import DlarSortNode, WebServerNode

__all__ = [
    'RobotController',
    'RobotStateMonitor',
    'StateManager', 
    'SafetyManager',
    'CollisionRecovery',
    'ConveyorHandler',
    'FirebaseHandler',
    'DlarSortNode',
    'WebServerNode',
]
