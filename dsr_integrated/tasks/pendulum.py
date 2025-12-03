#!/usr/bin/env python3
"""
진자운동 테스트 모듈
server_node.py에서 분리
"""

import time
import threading
from typing import Optional, Dict, Any

from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint

from ..safety import SafetyManager
from ..web.data_store import add_log


class PendulumController:
    """진자운동 테스트 컨트롤러"""
    
    def __init__(self, node: Node, cli_move_joint):
        """
        Args:
            node: ROS2 노드 인스턴스
            cli_move_joint: MoveJoint 서비스 클라이언트
        """
        self.node = node
        self.cli_move_joint = cli_move_joint
        
        # 상태
        self.running = False
        self.paused = False
        self.params: Optional[Dict[str, Any]] = None
        self._thread: Optional[threading.Thread] = None
    
    @property
    def is_running(self) -> bool:
        """진자운동 실행 중 여부"""
        return self.running
    
    @property
    def is_paused(self) -> bool:
        """진자운동 일시정지 여부"""
        return self.paused
    
    def start(self, joint_index: int = 0, amplitude: float = 15.0, vel: float = 30.0) -> bool:
        """
        진자운동 시작
        
        Args:
            joint_index: 대상 조인트 인덱스 (0-5)
            amplitude: 진폭 (도)
            vel: 속도 (도/초)
            
        Returns:
            성공 여부
        """
        if self.running:
            return False
        
        self.params = {
            'joint_index': joint_index,
            'amplitude': amplitude,
            'vel': vel
        }
        self.paused = False
        self.running = True
        
        self._thread = threading.Thread(
            target=self._pendulum_loop,
            args=(joint_index, amplitude, vel),
            daemon=True
        )
        self._thread.start()
        
        add_log('INFO', f'진자운동 시작 (J{joint_index+1}, ±{amplitude}°)')
        return True
    
    def stop(self) -> bool:
        """진자운동 정지"""
        self.running = False
        self.paused = False
        add_log('INFO', '진자운동 정지')
        return True
    
    def pause(self) -> bool:
        """진자운동 일시정지"""
        if self.running:
            self.running = False
            self.paused = True
            return True
        return False
    
    def resume(self) -> bool:
        """진자운동 재개"""
        if self.paused and self.params:
            self.paused = False
            p = self.params
            self.start(p['joint_index'], p['amplitude'], p['vel'])
            return True
        return False
    
    def _pendulum_loop(self, joint_index: int, amplitude: float, vel: float):
        """
        진자운동 루프 - SafetyManager 상태 체크
        
        Args:
            joint_index: 대상 조인트 인덱스
            amplitude: 진폭 (도)
            vel: 속도 (도/초)
        """
        try:
            user_home = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
            center_pos = user_home.copy()
            center_value = center_pos[joint_index]
            direction = 1
            
            # 현재 목표 위치 (비상정지 시 이어서 재개용)
            current_target_pos = None
            
            while self.running:
                # 비상정지 상태 체크 - 비상정지 중이면 대기
                if SafetyManager.is_emergency_stopped():
                    time.sleep(0.1)
                    continue
                
                # 일시정지 상태 체크
                if SafetyManager.is_paused():
                    time.sleep(0.1)
                    continue
                
                # 비상정지 해제 후: 저장된 목표가 있으면 이어서 재개
                if current_target_pos is not None:
                    target_pos = current_target_pos
                    print(f"🔄 [Pendulum] 이어서 재개: J{joint_index+1} → {target_pos[joint_index]:.1f}°")
                else:
                    # 새 목표 계산
                    target_pos = center_pos.copy()
                    target_pos[joint_index] = center_value + (amplitude * direction)
                
                if not self.cli_move_joint.service_is_ready():
                    break
                
                # 현재 목표 저장 (비상정지 시 이어서 재개용)
                current_target_pos = target_pos.copy()
                
                req = MoveJoint.Request()
                req.pos = [float(p) for p in target_pos]
                req.vel = float(vel)
                req.acc = float(vel)
                req.time = 0.0
                req.radius = 0.0
                req.mode = 0
                req.blend_type = 0
                req.sync_type = 0
                
                future = self.cli_move_joint.call_async(req)
                
                # 모션 완료 대기 (비상정지 체크하면서)
                while not future.done():
                    if SafetyManager.is_emergency_stopped():
                        print(f"⏸️ [Pendulum] 비상정지 - 목표 유지: J{joint_index+1} → {current_target_pos[joint_index]:.1f}°")
                        break
                    time.sleep(0.05)
                
                # 비상정지 상태면 목표 유지하고 다음 루프로
                if SafetyManager.is_emergency_stopped():
                    continue
                
                # 모션 정상 완료 - 목표 초기화하고 방향 전환
                current_target_pos = None
                
                if not self.running:
                    break
                
                direction *= -1
                time.sleep(0.1)
                
        except Exception as e:
            self.node.get_logger().error(f'Pendulum error: {e}')
        finally:
            self.running = False
            SafetyManager.clear_motion_context("pendulum")
