#!/usr/bin/env python3
"""
통합 시스템 Launch 파일
- 로봇 드라이버 (dsr_bringup2)
- 시리얼-토픽 브릿지 (컨베이어)
- 분류 작업 노드
- 웹 서버 노드
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch 인자 선언
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='real',
        description='Robot mode: real or virtual'
    )
    host_arg = DeclareLaunchArgument(
        'host', default_value='192.168.1.100',
        description='Robot IP address'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='12345',
        description='Robot port'
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='m0609',
        description='Robot model'
    )
    name_arg = DeclareLaunchArgument(
        'name', default_value='dsr01',
        description='Robot namespace'
    )
    use_conveyor_arg = DeclareLaunchArgument(
        'use_conveyor', default_value='true',
        description='Enable conveyor bridge node'
    )
    
    # 1. 로봇 드라이버 Launch (dsr_bringup2)
    dsr_bringup2_dir = get_package_share_directory('dsr_bringup2')
    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dsr_bringup2_dir, 'launch', 'dsr_bringup2_rviz.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'mode': LaunchConfiguration('mode'),
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'model': LaunchConfiguration('model'),
        }.items()
    )
    
    # 2. 컨베이어 시리얼 브릿지 노드 (로봇 드라이버와 동시 시작)
    serial_to_topic_node = Node(
        package='arduino_conveyor_bridge',
        executable='serial_to_topic',
        name='serial_to_topic_node',
        output='screen',
        emulate_tty=True,
    )
    
    # 3. 분류 작업 노드 (로봇 드라이버 시작 후 3초 대기)
    # 리팩토링된 sort_node 사용 (기존: dlar_sort_node)
    sort_node = TimerAction(
        period=3.0,
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
    
    # 4. 웹 서버 노드 (로봇 드라이버 시작 후 5초 대기)
    web_server_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='dsr_integrated',
                executable='web_server_node',
                name='web_server_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    return LaunchDescription([
        mode_arg,
        host_arg,
        port_arg,
        model_arg,
        name_arg,
        use_conveyor_arg,
        robot_driver,
        serial_to_topic_node,
        sort_node,
        web_server_node,
    ])
