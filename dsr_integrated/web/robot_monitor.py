#!/usr/bin/env python3
"""
로봇 상태 모니터링 모듈
server_node.py에서 분리된 상태 업데이트 로직
"""

import math
import json
import time

from rclpy.node import Node

from dsr_msgs2.srv import (
    GetCurrentTool, GetCurrentTcp,
    GetCurrentPosx, GetCurrentToolFlangePosx,
    GetToolForce, GetJointTorque,
    GetCtrlBoxDigitalOutput,
)

from .data_store import robot_data, sort_status, conveyor_status, add_log


class RobotStatusMonitor:
    """로봇 상태 모니터링 클래스"""
    
    def __init__(self, node: Node, service_clients: dict):
        """
        Args:
            node: ROS2 노드 인스턴스
            service_clients: 서비스 클라이언트 딕셔너리
        """
        self.node = node
        self.clients = service_clients
        self.start_time = time.time()
    
    def update_robot_status(self):
        """로봇 상태 정보 업데이트 (타이머 콜백에서 호출)"""
        robot_data['running_time'] = time.time() - self.start_time
        
        # TCP 위치
        cli = self.clients.get('get_posx')
        if cli and cli.service_is_ready():
            try:
                req = GetCurrentPosx.Request()
                req.ref = 0
                future = cli.call_async(req)
                future.add_done_callback(self._posx_callback)
            except:
                pass
        
        # Tool Force
        cli = self.clients.get('tool_force')
        if cli and cli.service_is_ready():
            try:
                req = GetToolForce.Request()
                req.ref = 0
                future = cli.call_async(req)
                future.add_done_callback(self._tool_force_callback)
            except:
                pass
        
        # Current Tool Name
        cli = self.clients.get('get_tool')
        if cli and cli.service_is_ready():
            try:
                req = GetCurrentTool.Request()
                future = cli.call_async(req)
                future.add_done_callback(self._tool_callback)
            except:
                pass
        
        # Current TCP Name
        cli = self.clients.get('get_tcp')
        if cli and cli.service_is_ready():
            try:
                req = GetCurrentTcp.Request()
                future = cli.call_async(req)
                future.add_done_callback(self._tcp_callback)
            except:
                pass
        
        # Tool Flange Position
        cli = self.clients.get('get_flange_posx')
        if cli and cli.service_is_ready():
            try:
                req = GetCurrentToolFlangePosx.Request()
                req.ref = 0
                future = cli.call_async(req)
                future.add_done_callback(self._flange_posx_callback)
            except:
                pass
        
        # Joint Torque
        cli = self.clients.get('joint_torque')
        if cli and cli.service_is_ready():
            try:
                req = GetJointTorque.Request()
                future = cli.call_async(req)
                future.add_done_callback(self._joint_torque_callback)
            except:
                pass
        
        # Digital Output
        cli = self.clients.get('get_digital_output')
        if cli and cli.service_is_ready():
            try:
                for pin in [1, 2]:
                    req = GetCtrlBoxDigitalOutput.Request()
                    req.index = pin
                    future = cli.call_async(req)
                    future.add_done_callback(lambda f, p=pin: self._dio_callback(f, p))
            except:
                pass
    
    # =========================================
    # 비동기 콜백 함수들
    # =========================================
    def _posx_callback(self, future):
        """TCP 위치 콜백"""
        try:
            result = future.result()
            if result.success:
                pos_data = result.task_pos_info
                if isinstance(pos_data, list) and len(pos_data) > 0:
                    first_item = pos_data[0]
                    if hasattr(first_item, 'data'):
                        robot_data['actual_tcp_position'] = list(first_item.data)[:6]
        except:
            pass
    
    def _tool_force_callback(self, future):
        """Tool Force 콜백"""
        try:
            result = future.result()
            if result.success:
                force_data = result.tool_force
                if hasattr(force_data, 'tolist'):
                    force_list = force_data.tolist()[:6]
                elif hasattr(force_data, 'data'):
                    force_list = [float(x) for x in list(force_data.data)[:6]]
                else:
                    force_list = [float(x) for x in list(force_data)[:6]]
                robot_data['external_tcp_force'] = force_list
                robot_data['actual_tool_force'] = force_list
        except:
            pass
    
    def _dio_callback(self, future, pin):
        """Digital Output 콜백"""
        try:
            result = future.result()
            if result.success:
                current = robot_data.get('controller_digital_output', 0)
                if result.value == 0:
                    current |= (1 << (pin - 1))
                else:
                    current &= ~(1 << (pin - 1))
                robot_data['controller_digital_output'] = current
        except:
            pass
    
    def _tool_callback(self, future):
        """Tool 이름 콜백"""
        try:
            result = future.result()
            if result.success:
                robot_data['current_tool_name'] = result.info if result.info else ''
        except:
            pass
    
    def _tcp_callback(self, future):
        """TCP 이름 콜백"""
        try:
            result = future.result()
            if result.success:
                robot_data['current_tcp_name'] = result.info if result.info else ''
        except:
            pass
    
    def _flange_posx_callback(self, future):
        """Tool Flange Position 콜백"""
        try:
            result = future.result()
            if result.success:
                pos_data = result.task_pos_info
                if isinstance(pos_data, list) and len(pos_data) > 0:
                    first_item = pos_data[0]
                    if hasattr(first_item, 'data'):
                        robot_data['tool_flange_posx'] = list(first_item.data)[:6]
        except:
            pass
    
    def _joint_torque_callback(self, future):
        """조인트 토크 콜백"""
        try:
            result = future.result()
            if result.success:
                torque_data = result.jts
                if hasattr(torque_data, 'tolist'):
                    torque_list = torque_data.tolist()[:6]
                else:
                    torque_list = [float(x) for x in list(torque_data)[:6]]
                robot_data['actual_joint_torque'] = torque_list
        except:
            pass


# =========================================
# 구독 콜백 함수들 (노드에서 직접 사용)
# =========================================
def joint_state_callback(msg):
    """조인트 상태 콜백"""
    joint_order = {'joint_1': 0, 'joint_2': 1, 'joint_3': 2,
                  'joint_4': 3, 'joint_5': 4, 'joint_6': 5}
    
    positions = [0.0] * 6
    velocities = [0.0] * 6
    
    for i, name in enumerate(msg.name):
        if name in joint_order:
            idx = joint_order[name]
            positions[idx] = math.degrees(msg.position[i]) if i < len(msg.position) else 0.0
            velocities[idx] = math.degrees(msg.velocity[i]) if i < len(msg.velocity) else 0.0
    
    robot_data['actual_joint_position'] = positions
    robot_data['actual_joint_velocity'] = velocities
    robot_data['timestamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    robot_data['connected'] = True


def sort_status_callback(msg):
    """분류 상태 콜백"""
    try:
        data = json.loads(msg.data)
        sort_status.update(data)
    except:
        pass


def conveyor_status_callback(msg):
    """컨베이어 상태 콜백"""
    conveyor_status['status'] = msg.data
    conveyor_status['connected'] = True


def conveyor_code_callback(msg):
    """컨베이어 코드 콜백"""
    conveyor_status['status_code'] = msg.data


def error_callback(msg):
    """로봇 에러 콜백"""
    robot_data['error_level'] = msg.level
    robot_data['error_code'] = msg.code
    robot_data['error_msg'] = msg.msg1 if hasattr(msg, 'msg1') else str(msg)
    
    if msg.level > 0:
        level_str = ['INFO', 'WARN', 'ERROR'][min(msg.level - 1, 2)]
        add_log(level_str, f'[{msg.code}] {robot_data["error_msg"]}')
