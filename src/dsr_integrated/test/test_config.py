#!/usr/bin/env python3
"""
Config 모듈 단위 테스트 (positions.py, constants.py)
"""

import pytest


class TestPositions:
    """positions.py 테스트"""
    
    def test_import(self):
        """모듈 임포트 테스트"""
        from dsr_integrated.config.positions import (
            HOME_POSITION, PICK_POSITION,
            PLACE_LARGE, PLACE_MEDIUM, PLACE_SMALL,
            get_place_position
        )
        assert HOME_POSITION is not None
        assert PICK_POSITION is not None
    
    def test_home_position_format(self):
        """HOME_POSITION 형식 테스트"""
        from dsr_integrated.config.positions import HOME_POSITION
        
        assert isinstance(HOME_POSITION, list)
        assert len(HOME_POSITION) == 6
        
        for val in HOME_POSITION:
            assert isinstance(val, (int, float))
    
    def test_pick_position_format(self):
        """PICK_POSITION 형식 테스트"""
        from dsr_integrated.config.positions import PICK_POSITION
        
        assert isinstance(PICK_POSITION, list)
        assert len(PICK_POSITION) == 6
    
    def test_place_positions_format(self):
        """PLACE 위치들 형식 테스트"""
        from dsr_integrated.config.positions import (
            PLACE_LARGE, PLACE_MEDIUM, PLACE_SMALL
        )
        
        for pos in [PLACE_LARGE, PLACE_MEDIUM, PLACE_SMALL]:
            assert isinstance(pos, list)
            assert len(pos) == 6
    
    def test_get_place_position_large(self):
        """get_place_position('LARGE') 테스트"""
        from dsr_integrated.config.positions import get_place_position, PLACE_LARGE
        
        result = get_place_position('LARGE')
        assert result == PLACE_LARGE
    
    def test_get_place_position_medium(self):
        """get_place_position('MEDIUM') 테스트"""
        from dsr_integrated.config.positions import get_place_position, PLACE_MEDIUM
        
        result = get_place_position('MEDIUM')
        assert result == PLACE_MEDIUM
    
    def test_get_place_position_small(self):
        """get_place_position('SMALL') 테스트"""
        from dsr_integrated.config.positions import get_place_position, PLACE_SMALL
        
        result = get_place_position('SMALL')
        assert result == PLACE_SMALL
    
    def test_get_place_position_long(self):
        """get_place_position('LONG') - LARGE와 동일 테스트"""
        from dsr_integrated.config.positions import get_place_position, PLACE_LARGE
        
        result = get_place_position('LONG')
        assert result == PLACE_LARGE
    
    def test_get_place_position_unknown(self):
        """알 수 없는 크기 시 HOME 반환 테스트"""
        from dsr_integrated.config.positions import get_place_position, HOME_POSITION
        
        result = get_place_position('unknown')
        assert result == HOME_POSITION


class TestConstants:
    """constants.py 테스트"""
    
    def test_import(self):
        """모듈 임포트 테스트"""
        from dsr_integrated.config.constants import (
            PHASE_PICK, PHASE_PLACE,
            FORCE_THRESHOLD, FORCE_PUSH, MAX_DESCENT,
            VELOCITY_MOVE, ACCEL_MOVE,
            DR_BASE, DR_TOOL
        )
        assert PHASE_PICK == 0
        assert PHASE_PLACE == 1
    
    def test_phase_constants(self):
        """페이즈 상수 테스트"""
        from dsr_integrated.config.constants import PHASE_PICK, PHASE_PLACE
        
        assert isinstance(PHASE_PICK, int)
        assert isinstance(PHASE_PLACE, int)
        assert PHASE_PICK != PHASE_PLACE
    
    def test_force_constants(self):
        """Force 상수 테스트"""
        from dsr_integrated.config.constants import (
            FORCE_THRESHOLD, FORCE_PUSH, MAX_DESCENT
        )
        
        assert isinstance(FORCE_THRESHOLD, (int, float))
        assert isinstance(FORCE_PUSH, (int, float))
        assert isinstance(MAX_DESCENT, (int, float))
        
        assert FORCE_THRESHOLD > 0
        assert FORCE_PUSH > 0
        assert MAX_DESCENT > 0
    
    def test_offset_constants(self):
        """오프셋 상수 테스트"""
        from dsr_integrated.config.constants import (
            UP_OFFSET, GRIP_OFFSET, PICK_EXTRA_DOWN, PLACE_EXTRA_DOWN
        )
        
        for val in [UP_OFFSET, GRIP_OFFSET, PICK_EXTRA_DOWN, PLACE_EXTRA_DOWN]:
            assert isinstance(val, (int, float))
    
    def test_velocity_constants(self):
        """속도 상수 테스트"""
        from dsr_integrated.config.constants import (
            VELOCITY_MOVE, ACCEL_MOVE, VELOCITY_PICK, ACCEL_PICK
        )
        
        assert VELOCITY_MOVE > 0
        assert ACCEL_MOVE > 0
        assert VELOCITY_PICK > 0
        assert ACCEL_PICK > 0
    
    def test_coordinate_ref_constants(self):
        """좌표 레퍼런스 상수 테스트"""
        from dsr_integrated.config.constants import DR_BASE, DR_TOOL
        
        assert isinstance(DR_BASE, int)
        assert isinstance(DR_TOOL, int)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
