"""
DSR Integrated - 물류 분류 로봇 시스템

Doosan M0609 로봇을 사용한 물류 자동 분류 시스템

Modules:
    sort_node: 메인 분류 노드 (ROS2)
    server_node: 웹서버 노드 (Flask + SocketIO)
    robot_controller: DSR 로봇 제어 래퍼
    state_manager: 상태/통계 관리
    conveyor_handler: 컨베이어 통신
    firebase_handler: Firebase 연동
"""

__version__ = '1.0.0'
