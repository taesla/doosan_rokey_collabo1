#!/usr/bin/env python3
"""
노드 전용 Launch 파일 (드라이버 제외)
- 로봇 드라이버는 외부에서 실행 (real 또는 virtual 모드)
- 분류 작업 노드
- 웹 서버 노드
- (선택) 컨베이어 브릿지

사용법:
  # 1. 외부 터미널에서 드라이버 먼저 실행
  ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual
  
  # 2. 이 launch 파일로 노드들 실행
  ros2 launch dsr_integrated nodes_only.launch.py
  
  # 컨베이어 비활성화:
  ros2 launch dsr_integrated nodes_only.launch.py use_conveyor:=false
"""

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Launch 인자 선언
    use_conveyor_arg = DeclareLaunchArgument(
        'use_conveyor', default_value='false',
        description='Enable conveyor bridge node'
    )
    
    # 1. 컨베이어 시리얼 브릿지 노드 (선택적)
    serial_to_topic_node = Node(
        package='arduino_conveyor_bridge',
        executable='serial_to_topic',
        name='serial_to_topic_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_conveyor'))
    )
    
    # 2. 분류 작업 노드 (1초 대기 후 시작)
    sort_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='dsr_integrated',
                executable='sort_node',
                name='dlar_sort_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    # 3. 웹 서버 노드 (2초 대기 후 시작)
    web_server_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='dsr_integrated',
                executable='server_node',
                name='web_server_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    return LaunchDescription([
        use_conveyor_arg,
        serial_to_topic_node,
        sort_node,
        web_server_node,
    ])
