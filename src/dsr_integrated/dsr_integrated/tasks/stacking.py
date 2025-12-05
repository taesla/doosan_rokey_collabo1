#!/usr/bin/env python3
"""
2차 적재(테트리스) 태스크 모듈
분류 완료된 박스를 2차 구역으로 재배치

- 1차 적재(분류) 완료 후 실행
- MEDIUM 2개, LONG 2개, SMALL 2개를 2차 구역으로 이동
- 조인트 기반 티칭 좌표 사용
- 충돌 발생 시 CollisionRecovery 연동
- 웹 UI로 진행 상태 실시간 전송
"""

import time
from typing import Optional, Callable

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from .base import BaseTask
from ..monitoring.state_monitor import RobotStateMonitor
from ..safety.collision_recovery import CollisionRecovery
from ..web.data_store import add_log
from ..config.constants import (
    STACKING_V_MOVE, STACKING_A_MOVE,
    STACKING_V_JOINT, STACKING_A_JOINT,
    STACKING_V_PLACE, STACKING_A_PLACE,
)
from ..config.stacking_positions import (
    HOME_POSITION_TUPLE,
    LARGE_GRIP_POSJ, MEDIUM_GRIP_POSJ, SMALL_GRIP_POSJ,
    LARGE_GRIP_APPROACH_POSJ, MEDIUM_GRIP_APPROACH_POSJ,
    LARGE_GRIP_VIA_POSJ, MEDIUM_GRIP_VIA_POSJ,
    LARGE_PLACE_POSJ, MEDIUM_PLACE_POSJ, SMALL_PLACE_POSJ,
    LARGE_PLACE_ABOVE_POSJ, MEDIUM_PLACE_ABOVE_POSJ, SMALL_PLACE_ABOVE_POSJ,
    LARGE_2_PLACE_VIA_POSJ,
    MEDIUM_2_GRIP_POSX, SMALL_GRIP_XY, SMALL_GRIP_Z, SMALL_PLACE_POSX,
    PLACEMENT_ORDER,
)

# SocketIO emit을 위한 전역 함수
_socketio_emit = None

def set_socketio_emit(emit_func):
    """SocketIO emit 함수 설정"""
    global _socketio_emit
    _socketio_emit = emit_func

def _emit_stacking_progress(step: int, total: int, message: str, box_info: dict = None):
    """웹 UI로 적재 진행 상태 전송"""
    global _socketio_emit
    if _socketio_emit:
        try:
            _socketio_emit('stacking_progress', {
                'step': step,
                'total': total,
                'percent': int((step / total) * 100) if total > 0 else 0,
                'message': message,
                'box_info': box_info
            })
        except Exception as e:
            print(f"[Stacking] emit error: {e}")


class StackingTask(BaseTask):
    """2차 적재(테트리스) 태스크 - 충돌 복구 지원"""
    
    def __init__(
        self,
        node: Node,
        robot,
        state_monitor: Optional[RobotStateMonitor] = None,
        recovery_checker: Optional[Callable[[], bool]] = None,
        collision_recovery: Optional[CollisionRecovery] = None,
        callback_group: Optional[ReentrantCallbackGroup] = None
    ):
        super().__init__(node, state_monitor, recovery_checker)
        self.robot = robot
        self.collision_recovery = collision_recovery
        self.callback_group = callback_group
        
        # 진행 상태
        self.current_step = 0
        self.total_steps = len(PLACEMENT_ORDER)
        self.is_running = False
        
        # 복구용 작업 상태
        self._current_action = 'idle'  # idle, picking, placing
        self._current_box_info = None  # (width_class, pick_idx, place_idx)
        self._is_gripping = False
    
    def execute(self) -> bool:
        """6개 박스 2차 적재 실행"""
        if not self._check_ready_or_log():
            return False
        
        self.is_running = True
        self.current_step = 0
        self._current_action = 'idle'
        
        self._log('INFO', '=' * 50)
        self._log('INFO', '[2차 적재] 테트리스 재배치 시작 (총 6개)')
        self._log('INFO', '순서: MEDIUM1 → LONG1 → LONG2 → MEDIUM2 → SMALL1 → SMALL2')
        self._log('INFO', '=' * 50)
        
        # 웹 UI 알림
        add_log('INFO', '📦 [2차 적재] 테트리스 재배치 시작')
        _emit_stacking_progress(0, 6, '2차 적재 시작')
        
        # HOME으로 이동
        home = self.robot.posx(*HOME_POSITION_TUPLE)
        self.robot.movel(home, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
        
        for idx, (width_class, pick_idx, place_idx) in enumerate(PLACEMENT_ORDER):
            self.current_step = idx + 1
            self._current_box_info = (width_class, pick_idx, place_idx)
            
            step_msg = f'{width_class} 그립{pick_idx+1} → 적재{place_idx+1}'
            self._log('INFO', f'[사이클 {self.current_step}/6] {step_msg}')
            
            # 웹 UI 진행 상태 전송
            add_log('INFO', f'📦 [2차 적재 {self.current_step}/6] {step_msg}')
            _emit_stacking_progress(self.current_step, 6, step_msg, {
                'width_class': width_class,
                'pick_idx': pick_idx,
                'place_idx': place_idx
            })
            
            # 비상정지/충돌 체크
            if not self._check_ready_or_log():
                # 충돌 발생 시 복구 시도
                if self.collision_recovery and not self.collision_recovery.is_recovering:
                    self._log('WARNING', '[2차 적재] 충돌 감지 - 복구 시도')
                    add_log('WARN', '⚠️ [2차 적재] 충돌 감지 - 복구 시도')
                    if self._attempt_recovery():
                        self._log('INFO', '[2차 적재] 복구 성공 - 현재 사이클 재시도')
                        add_log('INFO', '✅ [2차 적재] 복구 성공 - 재시도')
                        # 현재 사이클 재시도 (idx 유지)
                        continue
                    else:
                        self._log('ERROR', '[2차 적재] 복구 실패 - 작업 중단')
                        add_log('ERROR', '❌ [2차 적재] 복구 실패')
                        self.is_running = False
                        return False
                else:
                    self.is_running = False
                    return False
            
            # 1. 분류 구역에서 집기
            self._current_action = 'picking'
            if not self._pick_from_sorting(width_class, pick_idx):
                if self._attempt_recovery():
                    continue  # 복구 후 재시도
                self.is_running = False
                return False
            
            self._is_gripping = True
            
            # 2. HOME 경유
            self.robot.movel(home, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
            
            # 3. 적재 구역에 배치
            self._current_action = 'placing'
            if not self._place_to_stacking(width_class, place_idx):
                if self._attempt_recovery():
                    continue  # 복구 후 재시도
                self.is_running = False
                return False
            
            self._is_gripping = False
            self._current_action = 'idle'
            
            # 4. HOME 복귀
            self.robot.movel(home, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
        
        self._log('INFO', '=' * 50)
        self._log('INFO', '[2차 적재] 테트리스 재배치 완료 (총 6개)')
        self._log('INFO', '=' * 50)
        
        # 웹 UI 완료 알림
        add_log('INFO', '✅ [2차 적재] 테트리스 재배치 완료!')
        _emit_stacking_progress(6, 6, '2차 적재 완료')
        
        self.is_running = False
        self._current_action = 'idle'
        return True
    
    def _attempt_recovery(self) -> bool:
        """충돌 복구 시도"""
        if not self.collision_recovery:
            self._log('WARNING', '[2차 적재] CollisionRecovery 없음 - 복구 불가')
            return False
        
        try:
            self._log('INFO', '[2차 적재] 충돌 복구 시작...')
            
            # 작업 상태 저장
            work_state = {
                'phase': 'stacking',
                'step': self.current_step,
                'action': self._current_action,
                'box_info': self._current_box_info,
                'gripping': self._is_gripping
            }
            self.collision_recovery.save_work_state(work_state)
            
            # 자동 복구 실행
            success = self.collision_recovery.auto_recover(max_attempts=3)
            
            if success:
                self._log('INFO', '[2차 적재] 충돌 복구 성공')
                # 그립 상태 동기화
                self._is_gripping = False  # 복구 후에는 그리퍼 열림 상태
                return True
            else:
                self._log('ERROR', '[2차 적재] 충돌 복구 실패')
                return False
                
        except Exception as e:
            self._log('ERROR', f'[2차 적재] 복구 예외: {e}')
            return False
    
    def _pick_from_sorting(self, width_class: str, pick_index: int) -> bool:
        """분류 구역에서 박스 집기"""
        home = self.robot.posx(*HOME_POSITION_TUPLE)
        
        # 조인트 값 선택
        if width_class == "LARGE":
            grip_posj = LARGE_GRIP_POSJ[pick_index]
            approach_posj = LARGE_GRIP_APPROACH_POSJ[pick_index]
            via_posj = LARGE_GRIP_VIA_POSJ[pick_index]
        elif width_class == "MEDIUM":
            grip_posj = MEDIUM_GRIP_POSJ[pick_index]
            approach_posj = MEDIUM_GRIP_APPROACH_POSJ[pick_index]
            via_posj = MEDIUM_GRIP_VIA_POSJ[pick_index]
        elif width_class == "SMALL":
            grip_posj = None
            approach_posj = None
            via_posj = None
        else:
            self._log('ERROR', f'잘못된 박스 클래스: {width_class}')
            return False
        
        self._log('INFO', f'[PICK] {width_class} {pick_index+1}번째 박스 집기')
        
        # 1. HOME 이동 + 그리퍼 열기
        self.robot.movel(home, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
        self.robot.grip_off()
        
        # 2. 그립 전 접근
        if width_class == "MEDIUM" and pick_index == 1:
            # MEDIUM 2번: XY 이동 → Z 하강
            target = self.robot.posx(*MEDIUM_2_GRIP_POSX)
            self._log('INFO', '[MEDIUM 2] XY 이동 후 Z 하강')
            self.robot.movel(
                self.robot.posx(target[0], target[1], home[2], target[3], target[4], target[5]),
                vel=STACKING_V_MOVE, acc=STACKING_A_MOVE
            )
            time.sleep(0.2)
            self.robot.movel(target, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
            time.sleep(0.2)
            
        elif width_class == "SMALL":
            # SMALL: XY 이동 → Z 하강
            target_x, target_y = SMALL_GRIP_XY[pick_index]
            self._log('INFO', f'[SMALL {pick_index+1}] XY 이동 후 Z 하강')
            self.robot.movel(
                self.robot.posx(target_x, target_y, home[2], home[3], home[4], home[5]),
                vel=STACKING_V_MOVE, acc=STACKING_A_MOVE
            )
            time.sleep(0.2)
            self.robot.movel(
                self.robot.posx(target_x, target_y, SMALL_GRIP_Z, home[3], home[4], home[5]),
                vel=STACKING_V_MOVE, acc=STACKING_A_MOVE
            )
            time.sleep(0.2)
            
        elif approach_posj:
            self._log('INFO', f'[GRIP] 경유지점 이동')
            self.robot.movej(self.robot.posj(*approach_posj), vel=STACKING_V_JOINT, acc=STACKING_A_JOINT)
            time.sleep(0.2)
        
        # 3. 최종 그립 자세
        if width_class not in ["SMALL"] and not (width_class == "MEDIUM" and pick_index == 1):
            if grip_posj:
                self._log('INFO', f'[GRIP] 그립 위치 이동')
                self.robot.movej(self.robot.posj(*grip_posj), vel=STACKING_V_JOINT, acc=STACKING_A_JOINT)
                time.sleep(0.3)
        
        # 4. 그리퍼 닫기
        self.robot.grip_on()
        self._log('INFO', f'[GRIP] {width_class} {pick_index+1}번 그립 완료')
        
        # 5. 안전 상승
        if width_class == "MEDIUM" and pick_index == 1:
            target = MEDIUM_2_GRIP_POSX
            self.robot.movel(
                self.robot.posx(target[0], target[1], home[2], target[3], target[4], target[5]),
                vel=STACKING_V_MOVE, acc=STACKING_A_MOVE
            )
            time.sleep(0.2)
        elif width_class == "SMALL":
            target_x, target_y = SMALL_GRIP_XY[pick_index]
            self.robot.movel(
                self.robot.posx(target_x, target_y, home[2], home[3], home[4], home[5]),
                vel=STACKING_V_MOVE, acc=STACKING_A_MOVE
            )
            time.sleep(0.2)
        elif via_posj:
            self.robot.movej(self.robot.posj(*via_posj), vel=STACKING_V_JOINT, acc=STACKING_A_JOINT)
        
        # 6. HOME 복귀
        self.robot.movel(home, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
        
        return True
    
    def _place_to_stacking(self, width_class: str, place_index: int) -> bool:
        """적재 구역에 박스 배치"""
        home = self.robot.posx(*HOME_POSITION_TUPLE)
        
        # 적재 조인트 선택
        if width_class == "LARGE":
            place_above_posj = LARGE_PLACE_ABOVE_POSJ[place_index]
            place_posj_val = LARGE_PLACE_POSJ[place_index]
        elif width_class == "MEDIUM":
            place_above_posj = MEDIUM_PLACE_ABOVE_POSJ[place_index]
            place_posj_val = MEDIUM_PLACE_POSJ[place_index]
        elif width_class == "SMALL":
            place_above_posj = None
            place_posj_val = None
        else:
            self._log('ERROR', f'{width_class} 적재 좌표 없음')
            return False
        
        self._log('INFO', f'[PLACE] {width_class} {place_index+1}번째 적재')
        
        # 1. HOME 경유
        self.robot.movel(home, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
        
        # 2. 적재 상공 이동
        if width_class == "SMALL":
            # SMALL: XY 이동 → Z 하강
            target = self.robot.posx(*SMALL_PLACE_POSX[place_index])
            self._log('INFO', f'[SMALL {place_index+1}] XY 이동 후 Z 하강')
            self.robot.movel(
                self.robot.posx(target[0], target[1], home[2], home[3], home[4], home[5]),
                vel=STACKING_V_MOVE, acc=STACKING_A_MOVE
            )
            time.sleep(0.2)
            self.robot.movel(target, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
            time.sleep(0.2)
            
        elif width_class == "LARGE" and place_index == 1:
            # LARGE 2번: 2단계 경유
            self._log('INFO', '[LARGE 2] 1차 상공 → 2차 상공')
            self.robot.movej(self.robot.posj(*LARGE_2_PLACE_VIA_POSJ[0]), vel=STACKING_V_JOINT, acc=STACKING_A_JOINT)
            time.sleep(0.3)
            self.robot.movej(self.robot.posj(*LARGE_2_PLACE_VIA_POSJ[1]), vel=STACKING_V_JOINT, acc=STACKING_A_JOINT)
            time.sleep(0.3)
        else:
            if place_above_posj:
                self._log('INFO', f'[{width_class}] 적재 상공 대기')
                self.robot.movej(self.robot.posj(*place_above_posj), vel=STACKING_V_JOINT, acc=STACKING_A_JOINT)
                time.sleep(0.3)
        
        # 3. 적재 위치 하강
        if width_class != "SMALL" and place_posj_val:
            place_speed = STACKING_V_PLACE if width_class == "MEDIUM" else 15
            place_acc = STACKING_A_PLACE if width_class == "MEDIUM" else 20
            self._log('INFO', f'[{width_class}] 적재 위치 하강')
            self.robot.movej(self.robot.posj(*place_posj_val), vel=place_speed, acc=place_acc)
            time.sleep(0.3)
        
        # 4. 그리퍼 열기
        self.robot.grip_off()
        self._log('INFO', f'[PLACE] {width_class} {place_index+1}번 적재 완료')
        
        # 5. HOME 복귀
        self.robot.movel(home, vel=STACKING_V_MOVE, acc=STACKING_A_MOVE)
        
        return True
    
    def get_progress(self) -> dict:
        """현재 진행 상태 반환"""
        return {
            'is_running': self.is_running,
            'current_step': self.current_step,
            'total_steps': self.total_steps,
            'progress_percent': int((self.current_step / self.total_steps) * 100) if self.total_steps > 0 else 0
        }
