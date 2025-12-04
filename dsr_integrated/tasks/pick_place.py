#!/usr/bin/env python3
"""
Pick/Place 작업 로직 모듈
robot_pick_node/dlar_sort/pick.py + place.py 완전 병합

- DI 기반 분류 (RG2 그리퍼 폭 구간)
- 그리퍼 제어 및 분류 로직
- 고정 좌표 적재 (PLACE_POS_TABLE에서 티칭된 좌표 사용)
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
    FORCE_THRESHOLD,
    SAFE_Z_PICK, SAFE_Z_PLACE,
    VELOCITY_MOVE, ACCEL_MOVE, VELOCITY_PICK, ACCEL_PICK,
    PICK_EXTRA_DOWN,
)
from ..config.positions import (
    HOME_POSITION, PICK_POSITION,
    PLACE_SMALL, PLACE_MEDIUM, PLACE_LARGE,
    CLASS_FOOTPRINT, PLACE_ROTATE_ENABLED, PLACE_ROTATE_DEG,
    PLACE_POS_TABLE,
    normalize_angle_deg, get_footprint,
)


class PickPlaceTask(BaseTask):
    """Pick/Place 작업 로직 (고정 좌표 버전 - Force 탐색 제거)"""
    
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
            state: RobotStateManager
            firebase: FirebaseManager
            config: 설정 객체 (예: Config)
            state_monitor: 실시간 상태 모니터
            recovery_checker: 외부에서 호출되는 복구 가능 여부 체크 함수
        """
        super().__init__(node, robot, state, firebase, config)
        
        self.state_monitor = state_monitor
        self.recovery_checker = recovery_checker
        
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
        
        # Pick 상태
        self.current_width = None
        self.current_class = None
        
        # 로깅용
        reset_logistics_status()
    
    # =========================================================
    # 내부 유틸 함수들
    # =========================================================
    def _log(self, msg: str):
        """공통 로깅 함수"""
        self.node.get_logger().info(f"[PickPlace] {msg}")
    
    def _set_phase(self, phase: str):
        """현재 Pick/Place phase 설정"""
        self.state.set_phase(phase)
        self._log(f"=== PHASE 변경: {phase} ===")
    
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
            self._log("⏸️ movel 중 비상정지 감지 - 정지 상태로 전환")
            while not self.state.wait_for_estop_release(timeout=0.5):
                if self.state.state.stop_requested:
                    return False
        
        return result
    
    # =========================================================
    # PICK 관련 유틸
    # =========================================================
    def _safe_get_current_z(self) -> Tuple[bool, float]:
        """
        현재 TCP Z를 읽어오되, 실패 시 마지막 정상값 반환
        """
        try:
            pos = self.robot.get_current_posx()
            if pos and len(pos) >= 3:
                self._last_good_z = pos[2]
                return True, pos[2]
        except Exception as e:
            self._log(f"[WARN] 현재 Z 읽기 실패: {e}")
        return False, self._last_good_z
    
    def _approach_pick(self) -> bool:
        """
        픽업 위치 상공으로 접근
        """
        self._log("[PICK] 픽업 상공으로 이동")
        
        pick = PICK_POSITION.copy()
        px, py, pz, prx, pry, prz = pick
        
        # 안전 높이로 이동 (SAFE_Z_PICK or HOME 기반)
        safe_z = max(SAFE_Z_PICK, HOME_POSITION[2] + 50.0)
        approach = [px, py, safe_z, prx, pry, prz]
        
        if not self._movel_with_estop_check(approach, VELOCITY_MOVE, ACCEL_MOVE):
            return False
        
        return True
    
    def _down_to_pick_height(self) -> Tuple[bool, float]:
        """
        픽업 위치로 내려가기
        """
        pick = PICK_POSITION.copy()
        px, py, pz, prx, pry, prz = pick
        
        # 티칭된 픽업 Z 사용 (PICK_EXTRA_DOWN 적용)
        target_z = pz - PICK_EXTRA_DOWN
        if target_z < SAFE_Z_PICK:
            target_z = SAFE_Z_PICK
        
        self._log(f"[PICK] 픽업 Z로 이동: target_z={target_z:.2f}")
        if not self._movel_with_estop_check(
            [px, py, target_z, prx, pry, prz],
            VELOCITY_PICK, ACCEL_PICK
        ):
            return False, target_z
        
        ok, z_now = self._safe_get_current_z()
        if not ok:
            z_now = target_z
        
        return True, z_now
    
    def _grip_and_classify(self) -> Tuple[bool, Optional[str]]:
        """
        그리퍼로 집고, DI 기반 분류 수행
        
        Returns:
            (성공 여부, 분류 결과 class)
        """
        self._log("[PICK] 그리퍼 클로즈 (집기)")
        self.robot.grip_close()
        time.sleep(0.5)
        
        # DI 기반 분류 (RG2 그리퍼)
        try:
            box_class = self.robot.get_width_class()
            self._log(f"[PICK] DI 분류 결과: {box_class}")
        except Exception as e:
            self._log(f"[ERROR] 분류 실패: {e}")
            box_class = "UNKNOWN"
        
        self.current_class = box_class
        
        # UNKNOWN/ERROR는 분류 실패로 처리
        if box_class not in ("SMALL", "MEDIUM", "LONG"):
            self._log(f"[PICK] 유효하지 않은 분류: {box_class}")
            return False, None
        
        return True, box_class
    
    def _lift_after_pick(self) -> bool:
        """
        픽업 후 안전 높이까지 들어올리기
        """
        pick = PICK_POSITION.copy()
        px, py, pz, prx, pry, prz = pick
        
        safe_z = max(SAFE_Z_PICK, HOME_POSITION[2] + 50.0)
        target = [px, py, safe_z, prx, pry, prz]
        
        self._log(f"[PICK] 픽업 후 상승: safe_z={safe_z:.2f}")
        return self._movel_with_estop_check(target, VELOCITY_MOVE, ACCEL_MOVE)
    
    # =========================================================
    # MAIN PICK ENTRY
    # =========================================================
    def do_pick(self) -> Tuple[bool, Optional[str]]:
        """
        전체 Pick 시퀀스 수행
        - 픽업 위치 접근 → 내려가기 → 집기 → DI 분류 → 상승
        
        Returns:
            (성공 여부, 분류 결과 class)
        """
        self._set_phase(PHASE_PICK)
        self._set_action('approaching_pick')
        
        # 1) 픽업 상공 접근
        if not self._approach_pick():
            return False, None
        
        # 2) 그리퍼 열기 (준비)
        self._log("[PICK] 그리퍼 오픈 (준비)")
        self.robot.grip_open()
        time.sleep(0.3)
        
        # 3) 내려가기
        self._set_action('picking')
        ok, z_touch = self._down_to_pick_height()
        if not ok:
            return False, None
        
        # 4) 집기 + DI 분류
        ok, box_class = self._grip_and_classify()
        if not ok or box_class is None:
            return False, None
        
        # 5) 상승
        self._set_action('picked')
        if not self._lift_after_pick():
            return False, None
        
        return True, box_class
    
    # =========================================
    # PLACE 로직 (고정 좌표 버전)
    # =========================================
    def place_to_box(self, width_class: str) -> bool:
        """
        팔레트 적재 (고정 좌표 버전)

        - Force 탐색 전부 제거
        - PLACE_POS_TABLE에 있는 (x, y, z, rx, ry, rz) 좌표로만 이동
        - 플로우: pick → HOME → (x, y, 200) → (x, y, z) → 놓기

        Args:
            width_class: 크기 분류 ('SMALL', 'MEDIUM', 'LONG')

        Returns:
            성공 여부
        """
        # HOME 좌표
        home = HOME_POSITION.copy()
        hx, hy, hz, hrx, hry, hrz = home

        # 0) HOME으로 먼저 이동
        self._log("[PLACE] HOME 위치로 이동")
        self._movel_with_estop_check(home, vel=VELOCITY_MOVE, acc=ACCEL_MOVE)

        # 1) 분류 체크
        if width_class not in ["SMALL", "MEDIUM", "LONG"]:
            self._log(f"[WARN] width_class={width_class} -> 알 수 없는 분류, PLACE 스킵")
            return False

        # 2) 지금까지 적재한 개수 기준으로 몇 번째 슬롯 쓸지 결정
        current_count = self.stack_count.get(width_class, 0)

        # 사용할 수 있는 슬롯 개수 (PLACE_POS_TABLE에 몇 개 들어있는지)
        max_slots = len(PLACE_POS_TABLE.get(width_class, []))
        if max_slots == 0:
            self._log(f"[PLACE] {width_class}에 대한 PLACE_POS_TABLE 좌표가 없습니다.")
            return False

        if current_count >= max_slots:
            self._log(
                f"[PLACE] {width_class}는 이미 {max_slots}개 적재됨 -> 추가 적재 스킵"
            )
            return False

        slot_idx = current_count  # 0 → 첫 번째, 1 → 두 번째 ...

        # 3) PLACE_POS_TABLE에서 이번에 쓸 좌표 꺼내기
        try:
            target_x, target_y, target_z, rx, ry, rz = PLACE_POS_TABLE[width_class][slot_idx]
        except Exception as e:
            self._log(
                f"[PLACE] {width_class} 좌표 없음 (idx={slot_idx}, error={e})"
            )
            return False

        # 4) 접근 높이(경유 Z) 설정: z = 200 고정
        approach_z = 200.0
        # 만약 target_z보다 너무 낮으면 살짝 올려주기 (안전)
        if approach_z <= target_z + 10.0:
            approach_z = target_z + 50.0

        self._log(
            f"[PLACE] class={width_class}, slot={slot_idx+1}, "
            f"approach_z={approach_z:.1f}, target_z={target_z:.1f}"
        )

        # ==============================
        # 5) 접근 단계
        # ==============================
        # ★ 작업 단계 설정 (복구용) - Place 접근 중
        self._set_action('approaching_place')

        # HOME → (x, y, approach_z)
        self._log("[PLACE] 접근 위치로 이동: (x, y, z=approach_z)")
        self._movel_with_estop_check(
            [target_x, target_y, approach_z, rx, ry, rz],
            vel=VELOCITY_MOVE,
            acc=ACCEL_MOVE,
        )

        # ==============================
        # 6) 실제 놓는 위치로 이동 + 파지 해제
        # ==============================
        # ★ 작업 단계 설정 (복구용) - Place 중
        self._set_action('placing')

        # (x, y, target_z) 로 이동
        self._log("[PLACE] 최종 적재 위치로 이동: (x, y, z=target_z)")
        self._movel_with_estop_check(
            [target_x, target_y, target_z, rx, ry, rz],
            vel=VELOCITY_PICK,
            acc=ACCEL_PICK,
        )

        # 그리퍼 오픈 (놓기)
        self.robot.grip_open()
        time.sleep(0.3)

        # ==============================
        # 7) 위로 복귀 + 콜백 호출
        # ==============================
        # ★ 작업 단계 설정 (복구용) - Place 완료
        self._set_action('placed')

        # 다시 (x, y, approach_z)로 위로 올라가기
        self._log("[PLACE] 적재 후 위로 복귀")
        self._movel_with_estop_check(
            [target_x, target_y, approach_z, rx, ry, rz],
            vel=VELOCITY_MOVE,
            acc=ACCEL_MOVE,
        )

        # ★ 작업 단계 설정 (복구용) - Home으로 복귀 중
        self._set_action('returning_home')

        # 컨베이어 재시작 콜백 (있으면 실행)
        if self.on_place_complete:
            self._log("[PLACE] 위로 복귀 완료 - 컨베이어 재시작 콜백 호출")
            time.sleep(0.3)  # 최소 안정화 딜레이
            self.on_place_complete()

        # ==============================
        # 8) 상태/카운트/로그 업데이트
        # ==============================
        # 발자국 정보 (웹 로그용)
        bw, bd = get_footprint(width_class)
        # 대충 높이 추정 값 (UI용, 크게 중요하진 않음)
        h_est = max(20.0, target_z - 0.0)

        # 적재 기록 추가
        self.placed_boxes.append(
            {
                "x": target_x,
                "y": target_y,
                "z": target_z,
                "class": width_class,
                "bw": bw,
                "bd": bd,
                "h": h_est,
            }
        )
        self.stack_count[width_class] = current_count + 1

        # 웹 UI 상태 동기화
        update_logistics_status(
            stack_count=self.stack_count,
            placed_boxes=self.placed_boxes,
        )

        self._log(
            f"[COUNT] S={self.stack_count['SMALL']}, "
            f"M={self.stack_count['MEDIUM']}, "
            f"L={self.stack_count['LONG']} "
            f"(총 {len(self.placed_boxes)}개)"
        )

        # Firebase 저장
        self.firebase.save_sort_result(
            box_type=width_class,
            position=[target_x, target_y, target_z],
            force_value=FORCE_THRESHOLD,
        )

        return True
    
    def get_summary(self) -> dict:
        """현재 적재 상태 요약 반환"""
        return {
            "total_boxes": len(self.placed_boxes),
            "stack_count": self.stack_count.copy(),
            "placed_boxes": self.placed_boxes.copy(),
        }
