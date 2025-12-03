#!/usr/bin/env python3
"""
Pick/Place 작업 로직 모듈
robot_pick_node/dlar_sort/pick.py + place.py 완전 병합

- Force 센서 기반 컨베이어 물체 접촉
- 그리퍼 제어 및 분류 로직
- 층별 적재 (SMALL: 줄세우기, MEDIUM/LONG: 위로 쌓기)
- Force 탐색을 통한 정밀 배치
"""

import time
from typing import Optional, Tuple, Callable

from rclpy.node import Node

from .base import BaseTask
from ..monitoring.state_monitor import RobotStateMonitor
from ..web.data_store import update_logistics_status, reset_logistics_status
from ..config.constants import (
    PHASE_PICK, PHASE_PLACE,
    DR_BASE, DR_TOOL, DR_FC_MOD_ABS,
    FORCE_THRESHOLD, FORCE_PUSH, MAX_DESCENT,
    PLACE_FORCE_THRESHOLD_ML, SAFE_FORCE_LIMIT,
    UP_OFFSET, GRIP_OFFSET, PICK_EXTRA_DOWN,
    PLACE_EXTRA_DOWN, FINAL_PUSH, PLACE_REDUCE,
    Z_OFFSET_DOWN, SAFE_Z_PICK, SAFE_Z_PLACE,
    VELOCITY_MOVE, ACCEL_MOVE, VELOCITY_PICK, ACCEL_PICK,
    CONVEYOR_HEIGHT_OFFSET,
    LAYER_HEIGHT_MEDIUM, LAYER_HEIGHT_LONG,
)
from ..config.positions import (
    HOME_POSITION, PICK_POSITION,
    PLACE_SMALL, PLACE_MEDIUM, PLACE_LARGE,
    CLASS_FOOTPRINT, PLACE_ROTATE_ENABLED, PLACE_ROTATE_DEG,
    PLACE_POS_TABLE,
    normalize_angle_deg, get_footprint,
)


class PickPlaceTask(BaseTask):
    """Pick/Place 작업 로직 (robot_pick_node 완전 병합)"""
    
    def __init__(
        self, 
        node: Node, 
        robot, 
        state, 
        firebase, 
        config,
        state_monitor: Optional[RobotStateMonitor] = None,
        recovery_checker: Optional[Callable[[], bool]] = None
    ):
        """
        Args:
            node: ROS2 노드 인스턴스
            robot: RobotController 인스턴스
            state: StateManager 인스턴스
            firebase: FirebaseHandler 인스턴스
            config: YAML Config 인스턴스
            state_monitor: 로봇 상태 모니터 (STANDBY 체크용)
            recovery_checker: 복구 중인지 확인하는 콜백 함수
        """
        super().__init__(node, state_monitor, recovery_checker)
        
        self.robot = robot
        self.state = state
        self.firebase = firebase
        self.config = config
        
        # 런타임 상태
        self._last_good_z = HOME_POSITION[2]
        
        # 배치 이력 및 카운트
        self.placed_boxes = []
        self.stack_count = {"SMALL": 0, "MEDIUM": 0, "LONG": 0}
        
        # 콜백: Place 완료 후 호출 (컨베이어 재시작용)
        self.on_place_complete: Optional[Callable[[], None]] = None
        
        # Pick 상태
        self.w_z_touch = HOME_POSITION[2]
        self.w_pick_ok = False
    
    def execute(self, width_class: str = None, *args, **kwargs) -> bool:
        """
        Pick & Place 전체 사이클 실행 (BaseTask 구현)
        
        Args:
            width_class: 분류 클래스 (SMALL/MEDIUM/LONG). None이면 pick만 수행
            
        Returns:
            성공 여부
        """
        if not self.pick_and_measure():
            return False
        
        if width_class:
            return self.place_to_box(width_class)
        
        return True
    
    def reset_counts(self):
        """카운트 리셋"""
        self.placed_boxes = []
        self.stack_count = {"SMALL": 0, "MEDIUM": 0, "LONG": 0}
        self.w_z_touch = HOME_POSITION[2]
        self.w_pick_ok = False
        
        # 웹 UI 상태 동기화
        reset_logistics_status()
        
        self._log('카운트 리셋')
    
    def _log(self, msg: str):
        """DLAR 포맷 로그 출력"""
        self.node.get_logger().info(f'[DLAR] {msg}')
    
    def _wait_for_estop_release(self) -> bool:
        """비상정지 해제될 때까지 대기"""
        if self.state.is_emergency_stopped():
            self._log('⏸️ 비상정지 대기 중...')
            while not self.state.wait_for_estop_release(timeout=0.5):
                if self.state.state.stop_requested:
                    return False
            self._log('▶️ 비상정지 해제 - 작업 재개')
        return True
    
    def _movel_with_estop_check(self, pos, vel, acc) -> bool:
        """movel 실행 + 비상정지 시 대기"""
        if not self._wait_for_estop_release():
            return False
        
        result = self.robot.movel(pos, vel=vel, acc=acc)
        
        if self.state.is_emergency_stopped():
            if not self._wait_for_estop_release():
                return False
            result = self.robot.movel(pos, vel=vel, acc=acc)
        
        return result
    
    def _safe_get_current_z(self) -> Tuple[bool, float]:
        """
        현재 Z값 안전하게 조회 (robot_pick_node 호환)
        Returns: (성공여부, Z값)
        """
        try:
            px = self.robot.get_current_posx(ref=DR_BASE)
            
            if px is None:
                self._log("[SAFE_Z] get_current_posx → None 반환")
                return False, self._last_good_z
            
            if len(px) >= 3:
                cur_z = float(px[2])
                self._last_good_z = cur_z
                return True, cur_z
            
            self._log(f"[SAFE_Z] 알 수 없는 posx 형식: {px}")
            return False, self._last_good_z
            
        except Exception as e:
            self._log(f"[SAFE_Z] 예외 발생: {e}")
            return False, self._last_good_z
    
    # =========================================
    # PICK 로직 (robot_pick_node/pick.py 기반)
    # =========================================
    
    def pick_and_measure(self) -> bool:
        """
        Force 센서로 컨베이어 물체 접촉 높이 측정 + 픽업
        (robot_pick_node/dlar_sort/pick.py 완전 병합)
        
        Returns:
            성공 여부
        """
        home = HOME_POSITION.copy()
        pick = PICK_POSITION.copy()
        
        hx, hy, hz, hrx, hry, hrz = home
        px, py, pz, prx, pry, prz = pick
        
        # 1) HOME_Z 높이에서 픽업 XY로 이동
        self._log("STEP: HOME_Z 높이에서 컨베이어 픽업 위치로 이동")
        if not self._movel_with_estop_check([px, py, hz, prx, pry, prz], vel=VELOCITY_MOVE, acc=ACCEL_MOVE):
            self._log("[ERROR] 픽업 위치 이동 실패")
            return False
        
        # ★ 충돌/비상정지 확인 - Force Control 활성화 전에 체크
        if self.state.is_emergency_stopped():
            self._log("[ABORT] 비상정지 상태 - Force Control 건너뜀")
            return False
        
        # 2) Compliance 모드 설정
        self._log("STEP: Compliance Control 활성화 중...")
        self.robot.enable_compliance()
        self._log("STEP: Compliance Control 활성화 완료")
        
        time.sleep(0.2)
        initial_force = self.robot.get_tool_force(ref=DR_TOOL)
        if initial_force:
            self._log(f"[FORCE_INIT] 초기 힘: Fx={initial_force[0]:.2f}, Fy={initial_force[1]:.2f}, Fz={initial_force[2]:.2f}N")
        
        self._log(f"STEP: Desired Force 설정 (Fz=-{FORCE_PUSH}N)")
        self.robot.set_desired_force(
            force=[0.0, 0.0, -FORCE_PUSH, 0.0, 0.0, 0.0],
            direction=[0, 0, 1, 0, 0, 0],
            time_val=0.0,
            mod=DR_FC_MOD_ABS
        )
        
        time.sleep(0.5)
        after_force = self.robot.get_tool_force(ref=DR_TOOL)
        if after_force:
            self._log(f"[FORCE_SET] Force 적용 후: Fx={after_force[0]:.2f}, Fy={after_force[1]:.2f}, Fz={after_force[2]:.2f}N")
        
        ok_start, start_z = self._safe_get_current_z()
        if not ok_start:
            start_z = hz
            self._log(f"[SAFE_Z] 시작 Z를 읽지 못해 HOME_Z={hz:.2f} 사용")
        
        contact = False
        z_touch = start_z
        max_wait = 10.0
        start_t = time.time()
        last_print = start_t
        loop_count = 0
        
        self._log("=" * 60)
        self._log("Force 센서 실시간 모니터링 시작")
        self._log(f"시작 높이: {start_z:.2f}mm, 목표 Force: {FORCE_THRESHOLD}N")
        self._log("=" * 60)
        
        while (time.time() - start_t) < max_wait:
            # 정지 요청 체크
            if self.state.state.stop_requested:
                break
            
            # 비상정지 체크
            if self.state.is_emergency_stopped():
                self.robot.release_force()
                self.robot.disable_compliance()
                self._log('⏸️ Force 모니터링 중 비상정지 - 대기')
                if not self._wait_for_estop_release():
                    self.w_pick_ok = False
                    return False
                self._log('▶️ 비상정지 해제 - Force 모니터링 재개')
                
                # 복구 후 서비스 안정화 대기
                self._log('[RECOVERY] 서비스 안정화 대기 (3초)...')
                time.sleep(3.0)
                
                # 서비스 연결 확인
                for retry in range(5):
                    test_pos = self.robot.get_current_posx(ref=DR_BASE)
                    if test_pos is not None:
                        self._log(f'[RECOVERY] 서비스 복구 확인: Z={test_pos[2]:.2f}mm')
                        break
                    self._log(f'[RECOVERY] 서비스 재연결 대기... ({retry+1}/5)')
                    time.sleep(1.0)
                else:
                    self._log('[ERROR] 서비스 복구 실패 - 작업 중단')
                    self.w_pick_ok = False
                    return False
                
                self.robot.enable_compliance()
                time.sleep(0.3)
                self.robot.set_desired_force(
                    force=[0.0, 0.0, -FORCE_PUSH, 0.0, 0.0, 0.0],
                    direction=[0, 0, 1, 0, 0, 0],
                    time_val=0.0,
                    mod=DR_FC_MOD_ABS
                )
                time.sleep(0.5)
                start_t = time.time()
                continue
            
            ok_z, cur_z = self._safe_get_current_z()
            if not ok_z:
                time.sleep(0.05)
                continue
            
            force = self.robot.get_tool_force(ref=DR_TOOL)
            if force is None:
                time.sleep(0.05)
                continue
            
            fx, fy, fz = force[:3]
            
            now = time.time()
            if (now - last_print) >= 0.1:
                moved = start_z - cur_z
                self._log(f"[MONITOR] Z={cur_z:6.2f}mm (▼{moved:5.2f}mm) | Fx={fx:6.2f}N, Fy={fy:6.2f}N, Fz={fz:6.2f}N | 경과={now - start_t:.1f}s")
                last_print = now
            
            # 안전 Z
            if cur_z < SAFE_Z_PICK:
                self._log("=" * 60)
                self._log(f"[SAFE_Z] SAFE_Z_LIMIT {SAFE_Z_PICK} 도달")
                self._log("=" * 60)
                z_touch = SAFE_Z_PICK
                break
            
            # 최대 하강 거리
            if (start_z - cur_z) > MAX_DESCENT:
                self._log("=" * 60)
                self._log(f"[MAX_DOWN] {MAX_DESCENT}mm 하강 완료")
                self._log("=" * 60)
                z_touch = cur_z
                break
            
            # Force 임계값
            if abs(fz) >= FORCE_THRESHOLD:
                z_touch = cur_z
                self._log("=" * 60)
                self._log("[CONTACT] ✓ 컨베이어 물체 접촉 감지!")
                self._log(f"         접촉 높이: z={z_touch:.2f}mm")
                self._log(f"         접촉 힘: Fz={fz:.2f}N (임계값: {FORCE_THRESHOLD}N)")
                self._log(f"         하강 거리: {start_z - z_touch:.2f}mm")
                self._log("=" * 60)
                contact = True
                break
            
            loop_count += 1
            time.sleep(0.05)
        
        elapsed = time.time() - start_t
        final_force = self.robot.get_tool_force(ref=DR_TOOL)
        self._log("=" * 60)
        self._log(f"Force 모니터링 종료 (총 {loop_count}회 체크, {elapsed:.2f}초)")
        if final_force:
            self._log(f"최종 위치: Z={z_touch:.2f}mm | 최종 힘: Fx={final_force[0]:.2f}, Fy={final_force[1]:.2f}, Fz={final_force[2]:.2f}N")
        self._log(f"접촉 여부: {'YES' if contact else 'NO'}")
        self._log("=" * 60)
        
        self.robot.release_force()
        self.robot.disable_compliance()
        
        if not contact:
            self._log("[NO_CONTACT] 물체 미감지, HOME으로 복귀")
            self.w_pick_ok = False
            self._movel_with_estop_check(home, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
            return False
        
        if z_touch < SAFE_Z_PICK:
            self._log(f"[SAFE_Z] z_touch={z_touch:.2f} -> {SAFE_Z_PICK}로 보정")
            z_touch = SAFE_Z_PICK
        
        # 위로 올렸다가 그리퍼 열고 다시 내려가서 집기
        self._movel_with_estop_check(
            [px, py, z_touch + UP_OFFSET, prx, pry, prz],
            vel=VELOCITY_MOVE, acc=ACCEL_MOVE
        )
        
        self.robot.grip_open()
        
        target_pick_z = z_touch - GRIP_OFFSET - PICK_EXTRA_DOWN
        if target_pick_z < SAFE_Z_PICK:
            self._log(f"[SAFE_Z] pick_z={target_pick_z:.2f} -> {SAFE_Z_PICK}로 보정")
            target_pick_z = SAFE_Z_PICK
        
        self._log(f"STEP: 컨베이어 물체 집기 (z={target_pick_z:.2f}mm)")
        self._movel_with_estop_check(
            [px, py, target_pick_z, prx, pry, prz],
            vel=VELOCITY_PICK, acc=ACCEL_PICK
        )
        
        self.robot.grip_close()
        time.sleep(0.5)
        
        # HOME_Z 높이까지 복귀
        self._movel_with_estop_check(
            [px, py, hz, prx, pry, prz],
            vel=VELOCITY_MOVE, acc=ACCEL_MOVE
        )
        
        # z_touch 저장
        self.w_z_touch = z_touch
        self.w_pick_ok = True
        self.state.set_z_touch(z_touch)
        
        # 웹 UI 상태 동기화
        update_logistics_status(z_touch=z_touch, pick_ok=True)
        
        self._log(f'픽업 완료: z_touch={z_touch:.2f}mm')
        return True
    
    # =========================================
    # Force 탐색 (robot_pick_node/place.py 기반)
    # =========================================
    
    def _force_search_down(
        self,
        x: float, y: float, start_z: float,
        rx: float, ry: float, rz: float,
        force_threshold: float = 5.0,
        max_travel: float = 50.0,
        step_down: float = 1.0,
    ) -> float:
        """
        그립을 쥔 상태에서 Z축으로 조금씩 내려가며
        외력(Fz)을 읽다가 임계값을 초과하면 그 위치를 contact_z로 리턴.
        
        - Force 모드(set_desired_force) 사용 안 함
        - position 모드로 천천히 내려가면서 get_tool_force()로 힘만 읽음
        
        Args:
            x, y, start_z: 시작 위치
            rx, ry, rz: 자세
            force_threshold: 접촉 감지 임계값 (N)
            max_travel: 최대 탐색 거리 (mm)
            step_down: 스텝 크기 (mm)
            
        Returns:
            contact_z: 접촉 감지된 Z 위치
        """
        # 시작 위치로 이동
        self._movel_with_estop_check(
            [x, y, start_z, rx, ry, rz],
            vel=VELOCITY_MOVE, acc=ACCEL_MOVE
        )
        
        current_z = start_z
        contact_z = start_z
        
        while True:
            current_z -= step_down
            
            self._movel_with_estop_check(
                [x, y, current_z, rx, ry, rz],
                vel=VELOCITY_PICK, acc=ACCEL_PICK
            )
            
            force = self.robot.get_tool_force(ref=DR_TOOL)
            if force is None:
                self._log(f"[ForceSearch] get_tool_force 실패")
                break
            
            fx, fy, fz = force[:3]
            
            # 1) 정상적인 임계값 도달
            if abs(fz) >= force_threshold:
                self._log(f"[ForceSearch] 감지됨! Z={current_z:.2f}, fz={fz:.2f}")
                contact_z = current_z
                break
            
            # 2) 안전 Force 상한 초과
            if abs(fz) >= SAFE_FORCE_LIMIT:
                self._log(f"[ForceSearch] SAFE_FORCE_LIMIT 초과! Z={current_z:.2f}, fz={fz:.2f}")
                contact_z = current_z
                break
            
            # 3) 최대 이동거리 초과
            if (start_z - current_z) > max_travel:
                self._log("[ForceSearch] 최대 탐색 거리 도달 - 접촉 없음")
                contact_z = current_z
                break
            
            time.sleep(0.02)
        
        return contact_z
    
    # =========================================
    # PLACE 로직 (robot_pick_node/place.py 기반)
    # =========================================
    
    def place_to_box(self, width_class: str) -> bool:
        """
        팔레트 적재 (robot_pick_node/dlar_sort/place.py 완전 병합)
        
        - SMALL(초록): 바닥에 한 층만, 각각 다른 XY 위치
        - MEDIUM(노랑) / LONG(파랑): 같은 자리에서 위로 계속 쌓기
        
        Args:
            width_class: 크기 분류 ('SMALL', 'MEDIUM', 'LONG')
            
        Returns:
            성공 여부
        """
        home = HOME_POSITION.copy()
        hx, hy, hz, hrx, hry, hrz = home
        
        self._log("[PLACE] HOME 위치로 경유 이동")
        self._movel_with_estop_check(home, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
        
        if width_class not in ["SMALL", "MEDIUM", "LONG"]:
            self._log(f"[WARN] width_class={width_class} -> 알 수 없는 분류, PLACE 스킵")
            return False
        
        # z_touch 가져오기
        z_touch = self.w_z_touch
        
        # ==============================
        # 1) 바닥 기준 Z 계산 (공통)
        # ==============================
        base_z = (
            z_touch
            - PLACE_EXTRA_DOWN
            - FINAL_PUSH
            + PLACE_REDUCE
            - CONVEYOR_HEIGHT_OFFSET
        )
        place_z_bottom = max(SAFE_Z_PLACE, base_z)
        
        bw, bd = CLASS_FOOTPRINT.get(width_class, (30.0, 30.0))
        h_est = max(20.0, min(200.0, abs(z_touch - SAFE_Z_PLACE)))
        
        # 자세: MEDIUM_POS_VAL의 RPY 사용
        rx, ry, rz = PLACE_MEDIUM[3:6]
        
        # ==============================
        # 2) 몇 번째 박스인지 (층 번호)
        # ==============================
        current_count = self.stack_count[width_class]
        if current_count >= 3:
            self._log(f"[PLACE] {width_class}는 이미 3개 적재됨 -> 추가 적재 스킵")
            return False
        
        slot_idx = current_count  # 0,1,2
        
        # ==============================
        # SMALL: 1,2,3번 위치 각각 다른 XY
        # MEDIUM/LONG: 같은 자리(0번) 위로 쌓기
        # ==============================
        if width_class == "SMALL":
            idx_xy = slot_idx
        else:
            idx_xy = 0  # MEDIUM/LONG은 0번 자리 고정
        
        try:
            target_x, target_y = PLACE_POS_TABLE[width_class][idx_xy]
        except Exception as e:
            self._log(f"[PLACE] {width_class} 좌표 없음 (idx_xy={idx_xy}, error={e})")
            return False
        
        # ==============================
        # 3) 층별 target_z / approach_z 계산
        # ==============================
        if width_class == "SMALL":
            layer_idx = 0
            target_z = place_z_bottom
            approach_z = target_z + 200.0
            
            self._log(f"[PLACE-Z] SMALL: target_z={target_z:.1f}, approach_z={approach_z:.1f}")
        
        else:
            # MEDIUM / LONG: 위로 쌓기
            layer_idx = current_count
            
            if width_class == "MEDIUM":
                if layer_idx == 0:
                    target_z_est = place_z_bottom + 15.0
                elif layer_idx == 1:
                    target_z_est = place_z_bottom + LAYER_HEIGHT_MEDIUM + 15.0
                else:
                    target_z_est = place_z_bottom + 2 * LAYER_HEIGHT_MEDIUM + 15.0
            else:  # "LONG"
                if layer_idx == 0:
                    target_z_est = place_z_bottom
                elif layer_idx == 1:
                    target_z_est = place_z_bottom + LAYER_HEIGHT_LONG
                else:
                    target_z_est = place_z_bottom + 2 * LAYER_HEIGHT_LONG
            
            # Z 상한 제한
            MAX_Z = HOME_POSITION[2] + 80.0
            if target_z_est > MAX_Z:
                self._log(f"[CLAMP] target_z_est {target_z_est:.1f} → {MAX_Z:.1f}")
                target_z_est = MAX_Z
            
            target_z = target_z_est
            approach_z = target_z + 100.0
            if approach_z > MAX_Z + 50.0:
                self._log(f"[CLAMP] approach_z {approach_z:.1f} → {MAX_Z+50.0:.1f}")
                approach_z = MAX_Z + 50.0
            
            self._log(f"[PLACE-Z] {width_class}: layer={layer_idx+1}, target_z_est={target_z_est:.1f}, approach_z={approach_z:.1f}, base_z={place_z_bottom:.1f}")
        
        # ==============================
        # 4) 접근 (공통)
        # ==============================
        self._movel_with_estop_check(
            [target_x, target_y, approach_z, rx, ry, rz],
            vel=VELOCITY_MOVE, acc=ACCEL_MOVE
        )
        
        rz_current = rz
        place_z_final = target_z  # 기본값
        
        # ==============================
        # 5) 실제 내려가서 놓는 동작
        # ==============================
        if width_class == "SMALL":
            # SMALL: 위치 기반으로 살짝 누르고
            target_z_small = target_z - 1.0
            self._log(f"[PLACE] SMALL 내려가기: target_z={target_z:.2f} → target_z_small={target_z_small:.2f}")
            
            self._movel_with_estop_check(
                [target_x, target_y, target_z_small, rx, ry, rz_current],
                vel=VELOCITY_PICK, acc=ACCEL_PICK
            )
            place_z_final = target_z_small
        
        elif width_class == "MEDIUM":
            # MEDIUM 전용 Force-only 내려가기
            start_z_force = target_z
            if start_z_force > approach_z:
                start_z_force = approach_z
            
            self._log(f"[PLACE] MEDIUM Force-only 시작: layer={layer_idx+1}, start_z_force={start_z_force:.2f}, target_z={target_z:.2f}")
            
            contact_z = self._force_search_down(
                target_x, target_y,
                start_z_force,
                rx, ry, rz_current,
                force_threshold=PLACE_FORCE_THRESHOLD_ML,
                max_travel=60.0,
                step_down=0.3,
            )
            
            MEDIUM_PRESS_Z_THRESH = 21.0
            EXTRA_PRESS_MM = 3.0
            
            if contact_z < MEDIUM_PRESS_Z_THRESH:
                place_z_final = contact_z - EXTRA_PRESS_MM
                self._log(f"[PLACE] MEDIUM 바닥층: contact_z={contact_z:.2f} → press {EXTRA_PRESS_MM:.1f}mm → place_z_final={place_z_final:.2f}")
            else:
                place_z_final = contact_z
                self._log(f"[PLACE] MEDIUM 상부층: contact_z={contact_z:.2f} → 추가 press 없이 place_z_final={place_z_final:.2f}")
            
            if place_z_final < SAFE_Z_PLACE:
                place_z_final = SAFE_Z_PLACE
            
            self._movel_with_estop_check(
                [target_x, target_y, place_z_final, rx, ry, rz_current],
                vel=VELOCITY_PICK, acc=ACCEL_PICK
            )
        
        elif width_class == "LONG":
            # LONG 전용 Force-only 내려가기
            start_z_force = target_z
            if start_z_force > approach_z - 20.0:
                start_z_force = approach_z - 20.0
            
            self._log(f"[PLACE] LONG Force-only 시작: layer={layer_idx+1}, start_z_force={start_z_force:.2f}, target_z={target_z:.2f}")
            
            contact_z = self._force_search_down(
                target_x, target_y,
                start_z_force,
                rx, ry, rz_current,
                force_threshold=PLACE_FORCE_THRESHOLD_ML,
                max_travel=60.0,
                step_down=0.5,
            )
            
            # LONG: contact_z 기준으로 조금 더 아래로 눌러서 놓기
            place_z_final = contact_z - 60.0
            if place_z_final < SAFE_Z_PLACE:
                place_z_final = SAFE_Z_PLACE
            
            self._log(f"[PLACE] LONG contact_z={contact_z:.2f} → place_z_final={place_z_final:.2f}")
        
        # ==============================
        # 6) 적재 수행 + 복귀 (공통)
        # ==============================
        self.robot.grip_open()
        time.sleep(0.3)
        
        self._movel_with_estop_check(
            [target_x, target_y, approach_z, rx, ry, rz_current],
            vel=VELOCITY_MOVE, acc=ACCEL_MOVE
        )
        
        # ★ 컨베이어 재시작 콜백
        if self.on_place_complete:
            self._log("[PLACE] 위로 복귀 완료 - 3초 후 컨베이어 재시작")
            time.sleep(3.0)
            self.on_place_complete()
        
        # ==============================
        # 7) 상태 저장 (공통)
        # ==============================
        self.placed_boxes.append({
            "x": target_x,
            "y": target_y,
            "z": place_z_final,
            "class": width_class,
            "bw": bw,
            "bd": bd,
            "h": h_est,
        })
        self.stack_count[width_class] += 1
        
        # 웹 UI 상태 동기화
        update_logistics_status(
            stack_count=self.stack_count,
            placed_boxes=self.placed_boxes
        )
        
        self._log(f"[COUNT] S={self.stack_count['SMALL']}, M={self.stack_count['MEDIUM']}, L={self.stack_count['LONG']} (총 {len(self.placed_boxes)}개)")
        
        # Firebase 저장
        self.firebase.save_sort_result(
            box_type=width_class,
            position=[target_x, target_y, place_z_final],
            force_value=FORCE_THRESHOLD
        )
        
        return True
    
    def get_summary(self) -> dict:
        """현재 적재 상태 요약 반환"""
        return {
            "total_boxes": len(self.placed_boxes),
            "stack_count": self.stack_count.copy(),
            "placed_boxes": self.placed_boxes.copy(),
        }
