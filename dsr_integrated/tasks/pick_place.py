#!/usr/bin/env python3
"""
Pick/Place 작업 로직 모듈
robot_pick_node/dlar_sort/pick.py + place.py 기반 ROS2 서비스 방식으로 변환
"""

import time
from typing import Optional, Tuple, Callable

from rclpy.node import Node

from ..config.constants import (
    PHASE_PICK, PHASE_PLACE,
    DR_BASE, DR_TOOL, DR_FC_MOD_ABS,
    FORCE_THRESHOLD, FORCE_PUSH, MAX_DESCENT,
    UP_OFFSET, GRIP_OFFSET, PICK_EXTRA_DOWN,
    PLACE_EXTRA_DOWN, FINAL_PUSH, PLACE_REDUCE,
    Z_OFFSET_DOWN, SAFE_Z_PICK, SAFE_Z_PLACE,
    VELOCITY_MOVE, ACCEL_MOVE, VELOCITY_PICK, ACCEL_PICK,
)
from ..config.positions import (
    HOME_POSITION, PICK_POSITION,
    PLACE_SMALL, PLACE_MEDIUM, PLACE_LARGE,
    CLASS_FOOTPRINT, PLACE_ROTATE_ENABLED, PLACE_ROTATE_DEG,
    normalize_angle_deg, get_footprint,
)


class PickPlaceTask:
    """Pick/Place 작업 로직 (robot_pick_node 기반)"""
    
    def __init__(self, node: Node, robot, state, firebase, config):
        """
        Args:
            node: ROS2 노드 인스턴스
            robot: RobotController 인스턴스
            state: StateManager 인스턴스
            firebase: FirebaseHandler 인스턴스
            config: YAML Config 인스턴스
        """
        self.node = node
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
    
    def reset_counts(self):
        """카운트 리셋"""
        self.placed_boxes = []
        self.stack_count = {"SMALL": 0, "MEDIUM": 0, "LONG": 0}
        self.node.get_logger().info('[PickPlace] 카운트 리셋')
    
    def _log(self, msg: str):
        """로그 출력"""
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
        현재 Z값 안전하게 조회
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
    
    def pick_and_measure(self) -> bool:
        """
        Force 센서로 컨베이어 물체 접촉 높이 측정 + 픽업
        (robot_pick_node/dlar_sort/pick.py::pick_and_measure 기반)
        
        Returns:
            성공 여부
        """
        home = HOME_POSITION.copy()
        pick = PICK_POSITION.copy()
        
        hx, hy, hz, hrx, hry, hrz = home
        px, py, pz, prx, pry, prz = pick
        
        # 1) HOME_Z 높이에서 픽업 XY로 이동
        self._log("STEP: HOME_Z 높이에서 컨베이어 픽업 위치로 이동")
        self._movel_with_estop_check([px, py, hz, prx, pry, prz], vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
        
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
                    return False
                self._log('▶️ 비상정지 해제 - Force 모니터링 재개')
                
                # 복구 후 서비스 안정화 대기 (Force 제어 재개 전)
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
                    return False
                
                self.robot.enable_compliance()
                time.sleep(0.3)
                self.robot.set_desired_force(
                    force=[0.0, 0.0, -FORCE_PUSH, 0.0, 0.0, 0.0],
                    direction=[0, 0, 1, 0, 0, 0],
                    time_val=0.0,
                    mod=DR_FC_MOD_ABS
                )
                time.sleep(0.5)  # Force 제어 안정화
                start_t = time.time()
                continue
            
            ok_z, cur_z = self._safe_get_current_z()
            if not ok_z:
                # 서비스 실패 시 더 긴 대기 (병목 방지)
                time.sleep(0.05)
                continue
            
            force = self.robot.get_tool_force(ref=DR_TOOL)
            if force is None:
                # Force 서비스 실패 시에도 대기 후 재시도
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
            # 루프 주기 조절 (50ms = 초당 20회, 병목 방지)
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
        self.state.set_z_touch(z_touch)
        self._log(f'픽업 완료: z_touch={z_touch:.2f}mm')
        return True
    
    def place_to_box(self, width_class: str) -> bool:
        """
        측정된 z_touch 기준으로 팔레트 적재
        (robot_pick_node/dlar_sort/place.py::place_to_box 기반)
        
        - SMALL : X- 방향으로 20mm 간격 줄세우기
        - MEDIUM/LONG : 각 위치에 위로 쌓기 (층별 높이 h_est + 5mm)
        
        Args:
            width_class: 크기 분류 ('SMALL', 'MEDIUM', 'LONG')
            
        Returns:
            성공 여부
        """
        home = HOME_POSITION.copy()
        hz = home[2]
        
        self._log("[PLACE] HOME 위치로 경유 이동")
        self._movel_with_estop_check(home, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)
        
        if width_class not in ["SMALL", "MEDIUM", "LONG"]:
            self._log(f"[WARN] width_class={width_class} -> 알 수 없는 분류, PLACE 스킵")
            return False
        
        # z_touch 가져오기
        z_touch = self.state.state.z_touch
        
        # 바닥 기준 높이 계산
        conveyor_height_offset = 70.0  # mm
        base_z = z_touch - PLACE_EXTRA_DOWN - FINAL_PUSH + PLACE_REDUCE - conveyor_height_offset
        place_z_bottom = max(SAFE_Z_PLACE, base_z)
        
        bw, bd = get_footprint(width_class)
        h_est = max(20.0, min(200.0, abs(z_touch - SAFE_Z_PLACE)))
        
        # 회전용 기준 자세 (MEDIUM_POS_VAL의 RPY 사용)
        rx, ry, rz = PLACE_MEDIUM[3:6]
        
        # ===== SMALL : X- 방향으로 줄세우기 =====
        if width_class == "SMALL":
            base_x, base_y = PLACE_SMALL[0], PLACE_SMALL[1]
            current_count = self.stack_count["SMALL"]
            idx = current_count % 3  # 0,1,2
            
            offset_x = -20.0 * (idx + 1)
            target_x = base_x + offset_x
            target_y = base_y
            target_z = place_z_bottom
            
            self._log(f"[PLACE] SMALL {current_count+1}번째: X={base_x:.1f}->{target_x:.1f} ({offset_x:.1f}mm), Y={target_y:.1f}, Z={target_z:.1f}")
        
        # ===== MEDIUM / LONG : 위로 쌓기 =====
        else:
            if width_class == "MEDIUM":
                base_x, base_y = PLACE_MEDIUM[0], PLACE_MEDIUM[1]
            else:
                base_x, base_y = PLACE_LARGE[0], PLACE_LARGE[1]
            
            current_count = self.stack_count[width_class]
            layer_idx = current_count % 3  # 0,1,2
            
            layer_height = h_est + 5.0
            target_x = base_x
            target_y = base_y
            target_z = place_z_bottom + layer_idx * layer_height
            
            self._log(f"[PLACE] {width_class} {current_count+1}번째 (층 {layer_idx}): ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) h_est={h_est:.1f}, layer_h={layer_height:.1f}")
        
        # Z 오프셋 추가 적용
        target_z = target_z - Z_OFFSET_DOWN
        
        # ===== 실제 모션 =====
        # 1) 위에서 XY 위치 맞추기
        self._movel_with_estop_check(
            [target_x, target_y, hz, rx, ry, rz],
            vel=VELOCITY_MOVE, acc=ACCEL_MOVE
        )
        
        rz_current = rz
        
        # 2) J6 회전 (PLACE_ROTATE_ENABLED) - movej 사용
        # TODO: movej 서비스 추가 필요시 구현
        # 현재는 movel만 사용
        
        # 3) 실제 내려가서 놓는 동작
        self._movel_with_estop_check(
            [target_x, target_y, target_z, rx, ry, rz_current],
            vel=VELOCITY_PICK, acc=ACCEL_PICK
        )
        
        self.robot.grip_open()
        time.sleep(0.3)
        
        # 4) 위로 복귀
        self._movel_with_estop_check(
            [target_x, target_y, hz, rx, ry, rz_current],
            vel=VELOCITY_MOVE, acc=ACCEL_MOVE
        )
        
        # ★ 위로 복귀 후 3초 대기 → 컨베이어 재시작
        if self.on_place_complete:
            self._log("[PLACE] 위로 복귀 완료 - 3초 후 컨베이어 재시작")
            time.sleep(3.0)
            self.on_place_complete()
        self._movel_with_estop_check(
            [target_x, target_y, hz, rx, ry, rz_current],
            vel=VELOCITY_MOVE, acc=ACCEL_MOVE
        )
        
        # ===== 이력 저장 =====
        self.placed_boxes.append({
            "x": target_x,
            "y": target_y,
            "z": target_z,
            "class": width_class,
            "bw": bw,
            "bd": bd,
            "h": h_est,
        })
        self.stack_count[width_class] += 1
        
        self._log(f"[COUNT] S={self.stack_count['SMALL']}, M={self.stack_count['MEDIUM']}, L={self.stack_count['LONG']} (총 {len(self.placed_boxes)}개)")
        
        # Firebase 저장
        self.firebase.save_sort_result(
            box_type=width_class,
            position=[target_x, target_y, target_z],
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
