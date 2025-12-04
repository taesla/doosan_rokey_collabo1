#!/usr/bin/env python3
"""
DSR 로봇 제어 래퍼 클래스
ROS2 서비스 클라이언트를 통한 로봇 제어 기능 제공
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from dsr_msgs2.srv import (
    MoveLine,
    MoveJoint,
    MoveStop,
    GetCurrentPosx,
    GetToolForce,
    SetCtrlBoxDigitalOutput,
    GetCtrlBoxDigitalInput,
    TaskComplianceCtrl,
    ReleaseComplianceCtrl,
    SetDesiredForce,
    ReleaseForce,
)

from ..config.constants import (
    ROBOT_ID,
    DR_BASE, DR_TOOL,
    DR_MV_MOD_ABS, DR_FC_MOD_ABS,
    VELOCITY_MOVE, ACCEL_MOVE,
    DEFAULT_STIFFNESS,
)


def wait_for_future(future, timeout=5.0):
    """
    Future 완료 대기 (폴링 방식)
    
    이미 executor가 별도 스레드에서 spinning 중이므로
    spin_until_future_complete 대신 이 함수를 사용해야 함
    
    Args:
        future: rclpy Future 객체
        timeout: 타임아웃 (초)
        
    Returns:
        bool: 완료 여부 (True=완료, False=타임아웃)
    """
    start_time = time.time()
    while not future.done():
        if time.time() - start_time > timeout:
            return False
        time.sleep(0.01)  # CPU 사용량 낮추기
    return True


class RobotController:
    """DSR 로봇 제어 래퍼 클래스"""
    
    def __init__(self, node: Node, callback_group: ReentrantCallbackGroup = None):
        """
        Args:
            node: ROS2 노드 인스턴스
            callback_group: 콜백 그룹 (None이면 기본 그룹 사용)
        """
        self.node = node
        self.callback_group = callback_group
        self.dsr_ready = False
        
        self._init_clients()
        self.node.get_logger().info('RobotController 초기화 완료')
    
    def _init_clients(self):
        """DSR ROS2 서비스 클라이언트 초기화"""
        prefix = f'/{ROBOT_ID}'
        
        # 모션 서비스 클라이언트
        self.cli_move_line = self.node.create_client(
            MoveLine, f'{prefix}/motion/move_line',
            callback_group=self.callback_group
        )
        self.cli_move_joint = self.node.create_client(
            MoveJoint, f'{prefix}/motion/move_joint',
            callback_group=self.callback_group
        )
        self.cli_move_stop = self.node.create_client(
            MoveStop, f'{prefix}/motion/move_stop',
            callback_group=self.callback_group
        )
        
        # 보조 제어 서비스 클라이언트
        self.cli_get_current_posx = self.node.create_client(
            GetCurrentPosx, f'{prefix}/aux_control/get_current_posx',
            callback_group=self.callback_group
        )
        self.cli_get_tool_force = self.node.create_client(
            GetToolForce, f'{prefix}/aux_control/get_tool_force',
            callback_group=self.callback_group
        )
        
        # IO 서비스 클라이언트
        self.cli_set_digital_output = self.node.create_client(
            SetCtrlBoxDigitalOutput, f'{prefix}/io/set_ctrl_box_digital_output',
            callback_group=self.callback_group
        )
        self.cli_get_digital_input = self.node.create_client(
            GetCtrlBoxDigitalInput, f'{prefix}/io/get_ctrl_box_digital_input',
            callback_group=self.callback_group
        )
        
        # Force 제어 서비스 클라이언트
        self.cli_task_compliance = self.node.create_client(
            TaskComplianceCtrl, f'{prefix}/force/task_compliance_ctrl',
            callback_group=self.callback_group
        )
        self.cli_release_compliance = self.node.create_client(
            ReleaseComplianceCtrl, f'{prefix}/force/release_compliance_ctrl',
            callback_group=self.callback_group
        )
        self.cli_set_desired_force = self.node.create_client(
            SetDesiredForce, f'{prefix}/force/set_desired_force',
            callback_group=self.callback_group
        )
        self.cli_release_force = self.node.create_client(
            ReleaseForce, f'{prefix}/force/release_force',
            callback_group=self.callback_group
        )
    
    def check_connection(self) -> bool:
        """DSR 연결 상태 확인"""
        connected = (
            self.cli_move_line.service_is_ready() and
            self.cli_get_current_posx.service_is_ready()
        )
        
        if connected and not self.dsr_ready:
            self.dsr_ready = True
            self.node.get_logger().info('✅ DSR 로봇 연결됨')
        elif not connected and self.dsr_ready:
            self.dsr_ready = False
            self.node.get_logger().warn('⚠️ DSR 로봇 연결 끊김')
        
        return self.dsr_ready
    
    @property
    def is_ready(self) -> bool:
        """로봇 준비 상태"""
        return self.dsr_ready
    
    # =========================================
    # 모션 제어
    # =========================================
    def movel(self, pos, vel=None, acc=None, time_val=0.0, radius=0.0,
              ref=DR_BASE, mode=DR_MV_MOD_ABS) -> bool:
        """
        직선 이동 (MoveLine) - 동기 호출
        
        Args:
            pos: 목표 위치 [x, y, z, rx, ry, rz]
            vel: 속도 (mm/s)
            acc: 가속도 (mm/s²)
            time_val: 이동 시간 (0이면 vel/acc 사용)
            radius: 블렌딩 반경
            ref: 좌표계 (DR_BASE or DR_TOOL)
            mode: 이동 모드 (DR_MV_MOD_ABS or DR_MV_MOD_REL)
            
        Returns:
            성공 여부
        """
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_move_line.service_is_ready():
            if not self.cli_move_line.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('move_line 서비스 준비 안됨')
                return False
        
        req = MoveLine.Request()
        req.pos = np.array(pos, dtype=np.float64)
        req.vel = np.array([vel or VELOCITY_MOVE] * 2, dtype=np.float64)
        req.acc = np.array([acc or ACCEL_MOVE] * 2, dtype=np.float64)
        req.time = time_val
        req.radius = radius
        req.ref = ref
        req.mode = mode
        req.blend_type = 0
        req.sync_type = 0
        
        future = self.cli_move_line.call_async(req)
        
        # 이미 executor가 별도 스레드에서 spinning 중이므로 폴링으로 대기
        if not wait_for_future(future, timeout=30.0):
            self.node.get_logger().warn('movel 타임아웃')
            return False
        
        if future.result() is not None:
            return future.result().success
        else:
            self.node.get_logger().warn('movel 실패')
        return False
    
    def movel_async(self, pos, vel=None, acc=None, time_val=0.0, radius=0.0,
                    ref=DR_BASE, mode=DR_MV_MOD_ABS):
        """
        직선 이동 (MoveLine) - 비동기 호출, future 반환 (비상정지 체크용)
        
        Args:
            pos: 목표 위치 [x, y, z, rx, ry, rz]
            vel: 속도 (mm/s)
            acc: 가속도 (mm/s²)
            time_val: 이동 시간 (0이면 vel/acc 사용)
            radius: 블렌딩 반경
            ref: 좌표계 (DR_BASE or DR_TOOL)
            mode: 이동 모드 (DR_MV_MOD_ABS or DR_MV_MOD_REL)
            
        Returns:
            Future 객체 (future.done()으로 완료 체크, future.result()로 결과 확인)
        """
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_move_line.service_is_ready():
            if not self.cli_move_line.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('move_line 서비스 준비 안됨')
                # 즉시 완료되는 더미 future 반환
                import concurrent.futures
                dummy_future = concurrent.futures.Future()
                dummy_future.set_result(None)
                return dummy_future
        
        req = MoveLine.Request()
        req.pos = np.array(pos, dtype=np.float64)
        req.vel = np.array([vel or VELOCITY_MOVE] * 2, dtype=np.float64)
        req.acc = np.array([acc or ACCEL_MOVE] * 2, dtype=np.float64)
        req.time = time_val
        req.radius = radius
        req.ref = ref
        req.mode = mode
        req.blend_type = 0
        req.sync_type = 0
        
        return self.cli_move_line.call_async(req)
    
    def stop_motion(self, stop_mode: int = 1) -> bool:
        """
        로봇 모션 즉시 중단
        
        Args:
            stop_mode: 0=DR_SSTOP (Smooth), 1=DR_QSTOP (Quick)
            
        Returns:
            성공 여부
        """
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_move_stop.service_is_ready():
            if not self.cli_move_stop.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().warn('MoveStop 서비스 준비 안됨')
                return False
        
        req = MoveStop.Request()
        req.stop_mode = stop_mode
        
        future = self.cli_move_stop.call_async(req)
        if not wait_for_future(future, timeout=1.0):
            return False
        
        if future.done() and future.result() and future.result().success:
            self.node.get_logger().info('🛑 로봇 모션 정지')
            return True
        return False
    
    def movej(self, pos, vel=30, acc=60, time_val=0.0, radius=0.0) -> bool:
        """
        조인트 이동 (MoveJoint) - 동기 호출
        
        Args:
            pos: 목표 조인트 각도 [j1, j2, j3, j4, j5, j6] (deg)
            vel: 속도 (deg/s)
            acc: 가속도 (deg/s²)
            time_val: 이동 시간 (0이면 vel/acc 사용)
            radius: 블렌딩 반경
            
        Returns:
            성공 여부
        """
        if not self.cli_move_joint.service_is_ready():
            if not self.cli_move_joint.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('move_joint 서비스 준비 안됨')
                return False
        
        req = MoveJoint.Request()
        req.pos = np.array(pos, dtype=np.float64)
        req.vel = float(vel)
        req.acc = float(acc)
        req.time = float(time_val)
        req.radius = float(radius)
        req.mode = 0  # 절대 좌표
        req.blend_type = 0
        req.sync_type = 0
        
        future = self.cli_move_joint.call_async(req)
        
        if not wait_for_future(future, timeout=30.0):
            self.node.get_logger().warn('movej 타임아웃')
            return False
        
        if future.result() is not None:
            return future.result().success
        else:
            self.node.get_logger().warn('movej 실패')
        return False
    
    @staticmethod
    def posj(*args):
        """
        조인트 좌표 생성 헬퍼
        
        Args:
            *args: j1, j2, j3, j4, j5, j6 또는 (j1, j2, j3, j4, j5, j6)
            
        Returns:
            list: [j1, j2, j3, j4, j5, j6]
        """
        if len(args) == 1 and hasattr(args[0], '__iter__'):
            return list(args[0])
        return list(args)
    
    @staticmethod
    def posx(*args):
        """
        직교 좌표 생성 헬퍼
        
        Args:
            *args: x, y, z, rx, ry, rz 또는 (x, y, z, rx, ry, rz)
            
        Returns:
            list: [x, y, z, rx, ry, rz]
        """
        if len(args) == 1 and hasattr(args[0], '__iter__'):
            return list(args[0])
        return list(args)
    
    # =========================================
    # 위치/센서 조회
    # =========================================
    def get_current_posx(self, ref=DR_BASE) -> list:
        """
        현재 TCP 위치 조회
        
        Args:
            ref: 좌표계 (DR_BASE or DR_TOOL)
            
        Returns:
            위치 리스트 [x, y, z, rx, ry, rz] 또는 None
        """
        # 서비스 준비 대기 (최대 0.5초)
        if not self.cli_get_current_posx.service_is_ready():
            if not self.cli_get_current_posx.wait_for_service(timeout_sec=0.5):
                return None
        
        req = GetCurrentPosx.Request()
        req.ref = ref
        
        future = self.cli_get_current_posx.call_async(req)
        # 타임아웃을 0.5초로 줄임 (빠른 실패, 재시도 허용)
        if not wait_for_future(future, timeout=0.5):
            return None
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                pos_data = result.task_pos_info
                if isinstance(pos_data, list) and len(pos_data) > 0:
                    first_item = pos_data[0]
                    if hasattr(first_item, 'data'):
                        return list(first_item.data)[:6]
                elif hasattr(pos_data, 'data'):
                    return list(pos_data.data)[:6]
        return None
    
    def get_tool_force(self, ref=DR_TOOL) -> list:
        """
        Tool Force 센서값 조회
        
        Args:
            ref: 좌표계 (DR_BASE or DR_TOOL)
            
        Returns:
            힘 리스트 [Fx, Fy, Fz, Mx, My, Mz] 또는 None
        """
        # 서비스 준비 대기 (최대 0.5초)
        if not self.cli_get_tool_force.service_is_ready():
            if not self.cli_get_tool_force.wait_for_service(timeout_sec=0.5):
                return None
        
        req = GetToolForce.Request()
        req.ref = ref
        
        future = self.cli_get_tool_force.call_async(req)
        # 타임아웃을 0.5초로 줄임 (빠른 실패, 재시도 허용)
        if not wait_for_future(future, timeout=0.5):
            return None
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                force_data = result.tool_force
                if hasattr(force_data, 'tolist'):
                    return force_data.tolist()[:6]
                elif hasattr(force_data, 'data'):
                    return [float(x) for x in list(force_data.data)[:6]]
                else:
                    return [float(x) for x in list(force_data)[:6]]
        return None
    
    # =========================================
    # 그리퍼 제어
    # =========================================
    def set_digital_output(self, index: int, value: int) -> bool:
        """디지털 출력 설정"""
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_set_digital_output.service_is_ready():
            if not self.cli_set_digital_output.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('set_digital_output 서비스 준비 안됨')
                return False
        
        req = SetCtrlBoxDigitalOutput.Request()
        req.index = index
        req.value = value
        
        future = self.cli_set_digital_output.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def get_digital_input(self, index: int) -> int:
        """디지털 입력 읽기"""
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_get_digital_input.service_is_ready():
            if not self.cli_get_digital_input.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('get_digital_input 서비스 준비 안됨')
                return None
        
        req = GetCtrlBoxDigitalInput.Request()
        req.index = index
        
        future = self.cli_get_digital_input.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return None
        
        if future.result() is not None:
            return future.result().value
        return None
    
    def grip_open(self):
        """그리퍼 열기"""
        self.set_digital_output(1, 0)
        self.set_digital_output(2, 1)
        time.sleep(0.3)
    
    def grip_close(self):
        """그리퍼 닫기"""
        self.set_digital_output(1, 1)
        self.set_digital_output(2, 0)
        time.sleep(0.3)
    
    # Alias for compatibility with stacking task
    def grip_off(self):
        """그리퍼 열기 (alias)"""
        self.grip_open()
    
    def grip_on(self):
        """그리퍼 닫기 (alias)"""
        self.grip_close()
    
    def is_gripping(self) -> bool:
        """
        그리퍼가 물체를 잡고 있는지 확인
        DO1=1 (close 명령 상태)이면 잡고 있는 것으로 판단
        
        Returns:
            True: 물체를 잡고 있음, False: 잡고 있지 않음
        """
        # DO1=1이면 그리퍼 닫힘 상태 (물체 잡고 있을 가능성)
        # 실제로는 DI로 확인하는게 정확하지만, DO 상태로 판단
        # grip_close() → DO1=1, DO2=0
        # grip_open() → DO1=0, DO2=1
        width_class = self.get_width_class()
        # width_class가 SMALL/MEDIUM/LONG이면 물체 잡고 있음
        return width_class in ('SMALL', 'MEDIUM', 'LONG')
    
    def get_width_class(self) -> str:
        """
        RG2 그리퍼 폭 구간 읽기
        
        Returns:
            'SMALL', 'MEDIUM', 'LONG', 'UNKNOWN', 'ERROR'
        """
        di1 = self.get_digital_input(1)
        di2 = self.get_digital_input(2)
        
        if di1 is None or di2 is None:
            return "UNKNOWN"
        
        if di1 == 1 and di2 == 0:
            return "SMALL"
        elif di1 == 0 and di2 == 1:
            return "LONG"
        elif di1 == 0 and di2 == 0:
            return "MEDIUM"
        else:
            return "ERROR"
    
    # =========================================
    # Force/Compliance 제어
    # =========================================
    def enable_compliance(self, stiffness=None) -> bool:
        """
        Compliance Control 활성화
        
        Args:
            stiffness: 강성 값 [x, y, z, rx, ry, rz] (None이면 기본값)
            
        Returns:
            성공 여부
        """
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_task_compliance.service_is_ready():
            if not self.cli_task_compliance.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('task_compliance_ctrl 서비스 준비 안됨')
                return False
        
        req = TaskComplianceCtrl.Request()
        req.stx = stiffness or DEFAULT_STIFFNESS
        req.ref = 0
        req.time = 0.0
        
        future = self.cli_task_compliance.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def disable_compliance(self) -> bool:
        """Compliance Control 비활성화"""
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_release_compliance.service_is_ready():
            if not self.cli_release_compliance.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('release_compliance_ctrl 서비스 준비 안됨')
                return False
        
        req = ReleaseComplianceCtrl.Request()
        future = self.cli_release_compliance.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def set_desired_force(self, force, direction, time_val=0.0, mod=DR_FC_MOD_ABS) -> bool:
        """
        목표 힘 설정
        
        Args:
            force: 힘 값 [Fx, Fy, Fz, Mx, My, Mz]
            direction: 방향 마스크 [x, y, z, rx, ry, rz]
            time_val: 지속 시간 (0=무한)
            mod: 모드 (DR_FC_MOD_ABS)
            
        Returns:
            성공 여부
        """
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_set_desired_force.service_is_ready():
            if not self.cli_set_desired_force.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('set_desired_force 서비스 준비 안됨')
                return False
        
        req = SetDesiredForce.Request()
        req.fd = np.array(force, dtype=np.float64)
        req.dir = np.array(direction, dtype=np.int8)
        req.time = time_val
        req.mod = mod
        
        future = self.cli_set_desired_force.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def release_force(self, time_val=0.0) -> bool:
        """Force 제어 해제"""
        # 서비스 준비 대기 (최대 1초)
        if not self.cli_release_force.service_is_ready():
            if not self.cli_release_force.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('release_force 서비스 준비 안됨')
                return False
        
        req = ReleaseForce.Request()
        req.time = time_val
        
        future = self.cli_release_force.call_async(req)
        if not wait_for_future(future, timeout=5.0):
            return False
        
        if future.result() is not None:
            return future.result().success
        return False
