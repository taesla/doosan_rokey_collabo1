#!/usr/bin/env python3
"""
State Manager 단위 테스트
"""

import pytest
from unittest.mock import MagicMock


class TestStateData:
    """StateData 테스트"""
    
    def test_import(self):
        """모듈 임포트 테스트"""
        from dsr_integrated.state_manager import SortState, SortStatistics, StateManager
        assert SortState is not None
        assert SortStatistics is not None
        assert StateManager is not None
    
    def test_state_data_default_values(self):
        """SortState 기본값 테스트"""
        from dsr_integrated.state_manager import SortState
        state = SortState()
        
        assert state.is_running == False
        assert state.is_paused == False
        assert state.stop_requested == False
        assert state.cycle_count == 0
        assert state.dsr_ready == False
        assert state.conveyor_mode == False
        assert state.conveyor_detected == False
        assert state.waiting_for_object == False


class TestSortStatistics:
    """SortStatistics 테스트"""
    
    def test_default_values(self):
        """기본값 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        assert stats.completed == 0
        assert stats.large == 0
        assert stats.medium == 0
        assert stats.small == 0
        assert stats.errors == 0
    
    def test_increment_large(self):
        """대형 카운트 증가 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        stats.increment('LARGE')
        
        assert stats.large == 1
        assert stats.completed == 1
    
    def test_increment_medium(self):
        """중형 카운트 증가 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        stats.increment('MEDIUM')
        
        assert stats.medium == 1
        assert stats.completed == 1
    
    def test_increment_small(self):
        """소형 카운트 증가 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        stats.increment('SMALL')
        
        assert stats.small == 1
        assert stats.completed == 1
    
    def test_increment_long_maps_to_large(self):
        """LONG은 large로 매핑 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        stats.increment('LONG')
        
        assert stats.large == 1
        assert stats.completed == 1
    
    def test_increment_multiple(self):
        """여러 번 증가 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        stats.increment('LARGE')
        stats.increment('LARGE')
        stats.increment('MEDIUM')
        stats.increment('SMALL')
        
        assert stats.large == 2
        assert stats.medium == 1
        assert stats.small == 1
        assert stats.completed == 4
    
    def test_add_error(self):
        """에러 추가 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        stats.add_error()
        stats.add_error()
        
        assert stats.errors == 2
    
    def test_reset(self):
        """리셋 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        stats.increment('LARGE')
        stats.increment('MEDIUM')
        stats.add_error()
        
        stats.reset()
        
        assert stats.completed == 0
        assert stats.large == 0
        assert stats.medium == 0
        assert stats.small == 0
        assert stats.errors == 0
    
    def test_to_dict(self):
        """딕셔너리 변환 테스트"""
        from dsr_integrated.state_manager import SortStatistics
        stats = SortStatistics()
        
        stats.increment('LARGE')
        stats.increment('MEDIUM')
        
        d = stats.to_dict()
        
        assert d['completed'] == 2
        assert d['large'] == 1
        assert d['medium'] == 1
        assert d['small'] == 0
        assert d['errors'] == 0


class TestStateManager:
    """StateManager 테스트"""
    
    def create_mock_node(self):
        """Mock ROS2 노드 생성"""
        node = MagicMock()
        node.get_logger.return_value = MagicMock()
        return node
    
    def test_init(self):
        """초기화 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        
        manager = StateManager(node)
        
        assert manager.state is not None
        assert manager.stats is not None
        assert manager.state.is_running == False
    
    def test_start(self):
        """시작 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.start()
        
        assert manager.state.is_running == True
        assert manager.state.stop_requested == False
        assert manager.state.is_paused == False
    
    def test_finish(self):
        """종료 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.start()
        manager.finish()
        
        assert manager.state.is_running == False
    
    def test_pause_resume(self):
        """일시정지/재개 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.start()
        
        manager.pause()
        assert manager.state.is_paused == True
        
        manager.resume()
        assert manager.state.is_paused == False
    
    def test_stop(self):
        """정지 요청 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.start()
        manager.stop()
        
        assert manager.state.stop_requested == True
    
    def test_reset(self):
        """리셋 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.start()
        manager.state.cycle_count = 5
        
        manager.reset()
        
        assert manager.state.is_running == False
        assert manager.state.cycle_count == 0
    
    def test_set_phase(self):
        """페이즈 설정 테스트"""
        from dsr_integrated.state_manager import StateManager
        from dsr_integrated.config.constants import PHASE_PLACE
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.set_phase(PHASE_PLACE)
        
        assert manager.state.current_phase == PHASE_PLACE
    
    def test_set_z_touch(self):
        """z_touch 설정 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.set_z_touch(150.5)
        
        assert manager.state.z_touch == 150.5
    
    def test_complete_cycle(self):
        """사이클 완료 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.start()
        manager.complete_cycle('LARGE')
        
        assert manager.state.cycle_count == 1
        assert manager.stats.large == 1
        assert manager.stats.completed == 1
    
    def test_can_start_auto_cycle_true(self):
        """자동 사이클 시작 가능 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.state.conveyor_mode = True
        manager.state.waiting_for_object = True
        manager.state.conveyor_detected = True
        manager.state.is_running = False
        
        assert manager.can_start_auto_cycle() == True
    
    def test_can_start_auto_cycle_false(self):
        """자동 사이클 시작 불가 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        # 이미 실행 중
        manager.state.conveyor_mode = True
        manager.state.waiting_for_object = True
        manager.state.conveyor_detected = True
        manager.state.is_running = True
        
        assert manager.can_start_auto_cycle() == False
    
    def test_get_status_dict(self):
        """상태 딕셔너리 테스트"""
        from dsr_integrated.state_manager import StateManager
        node = self.create_mock_node()
        manager = StateManager(node)
        
        manager.start()
        status = manager.get_status_dict()
        
        assert 'is_running' in status
        assert 'cycle_count' in status
        assert status['is_running'] == True


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
