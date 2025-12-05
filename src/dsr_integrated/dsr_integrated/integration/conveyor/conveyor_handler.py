#!/usr/bin/env python3
"""
컨베이어 연동 핸들러 모듈
아두이노 컨베이어와의 통신 및 이벤트 처리
"""

from typing import Callable, Optional
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Int32

from ...config.constants import CONVEYOR_DETECT, CONVEYOR_RUNNING


class ConveyorHandler:
    """컨베이어 연동 핸들러"""
    
    def __init__(
        self,
        node: Node,
        callback_group: ReentrantCallbackGroup = None,
        on_detect: Callable = None
    ):
        """
        Args:
            node: ROS2 노드 인스턴스
            callback_group: 콜백 그룹
            on_detect: 물체 감지 시 호출할 콜백 함수
        """
        self.node = node
        self.callback_group = callback_group
        self.on_detect_callback = on_detect
        
        # 상태
        self.detected = False
        self.running = False
        
        # Publisher/Subscriber 생성
        self._init_topics()
        
        self.node.get_logger().info('ConveyorHandler 초기화 완료')
    
    def _init_topics(self):
        """토픽 초기화"""
        # 컨베이어 상태 수신
        self.sub_status = self.node.create_subscription(
            Int32,
            '/conveyor/status_code',
            self._status_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 컨베이어 명령 송신
        self.pub_cmd = self.node.create_publisher(String, '/conveyor/cmd', 10)
    
    def _status_callback(self, msg):
        """컨베이어 상태 수신 콜백"""
        code = msg.data
        
        if code == CONVEYOR_DETECT:
            self.detected = True
            self.running = False
            self.node.get_logger().info('[CONVEYOR] 물체 감지!')
            
            # 감지 콜백 호출
            if self.on_detect_callback:
                self.on_detect_callback()
        
        elif code == CONVEYOR_RUNNING:
            self.detected = False
            self.running = True
    
    def set_on_detect(self, callback: Callable):
        """감지 콜백 설정"""
        self.on_detect_callback = callback
    
    def send_resume(self):
        """컨베이어 RESUME 명령 전송"""
        msg = String()
        msg.data = 'RESUME'
        self.pub_cmd.publish(msg)
        self.node.get_logger().info('[CONVEYOR] RESUME 명령 전송')
    
    def send_command(self, cmd: str):
        """컨베이어 명령 전송"""
        msg = String()
        msg.data = cmd
        self.pub_cmd.publish(msg)
        self.node.get_logger().info(f'[CONVEYOR] {cmd} 명령 전송')
    
    @property
    def is_detected(self) -> bool:
        """물체 감지 상태"""
        return self.detected
    
    @property
    def is_running(self) -> bool:
        """컨베이어 동작 상태"""
        return self.running
    
    def clear_detection(self):
        """감지 상태 초기화"""
        self.detected = False
