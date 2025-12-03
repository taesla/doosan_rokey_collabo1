#!/usr/bin/env python3
"""
YAML 설정 로더 단위 테스트
"""

import pytest
import os
import tempfile
import yaml


class TestConfigLoader:
    """ConfigLoader 테스트"""
    
    def test_import(self):
        """모듈 임포트 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader, get_config
        assert ConfigLoader is not None
        assert get_config is not None
    
    def test_config_loader_init(self):
        """ConfigLoader 초기화 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        assert loader.package_name == 'dsr_integrated'
        assert loader.config_dir is not None
    
    def test_positions_loading(self):
        """positions.yaml 로드 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        positions = loader.positions
        
        # 필수 키 확인
        assert 'home' in positions
        assert 'pick' in positions
        assert 'place' in positions
        
        # home 위치 형식 확인
        home = positions['home']
        assert 'x' in home
        assert 'y' in home
        assert 'z' in home
        assert 'rx' in home
        assert 'ry' in home
        assert 'rz' in home
    
    def test_settings_loading(self):
        """settings.yaml 로드 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        settings = loader.settings
        
        # 필수 키 확인
        assert 'robot' in settings
        assert 'force' in settings
        assert 'offsets' in settings
        assert 'motion' in settings
        assert 'safety' in settings
    
    def test_get_home_position(self):
        """홈 위치 반환 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        home = loader.get_home_position()
        
        # 리스트 형식 및 길이 확인
        assert isinstance(home, list)
        assert len(home) == 6
        
        # 모든 값이 숫자인지 확인
        for val in home:
            assert isinstance(val, (int, float))
    
    def test_get_pick_position(self):
        """픽업 위치 반환 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        pick = loader.get_pick_position()
        
        assert isinstance(pick, list)
        assert len(pick) == 6
    
    def test_get_place_position_large(self):
        """배치 위치 (large) 반환 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        place = loader.get_place_position('large')
        
        assert isinstance(place, list)
        assert len(place) == 6
    
    def test_get_place_position_medium(self):
        """배치 위치 (medium) 반환 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        place = loader.get_place_position('medium')
        
        assert isinstance(place, list)
        assert len(place) == 6
    
    def test_get_place_position_small(self):
        """배치 위치 (small) 반환 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        place = loader.get_place_position('small')
        
        assert isinstance(place, list)
        assert len(place) == 6
    
    def test_get_place_position_long_maps_to_large(self):
        """long은 large로 매핑되는지 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        place_long = loader.get_place_position('long')
        place_large = loader.get_place_position('large')
        
        assert place_long == place_large
    
    def test_get_place_position_invalid(self):
        """잘못된 크기 요청 시 예외 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        
        with pytest.raises(ValueError):
            loader.get_place_position('invalid_size')
    
    def test_get_robot_id(self):
        """로봇 ID 반환 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        robot_id = loader.get_robot_id()
        
        assert isinstance(robot_id, str)
        assert len(robot_id) > 0
    
    def test_get_force_settings(self):
        """Force 설정 반환 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        force = loader.get_force_settings()
        
        assert 'threshold' in force
        assert 'push_force' in force
        assert 'max_descent' in force
        
        assert isinstance(force['threshold'], (int, float))
        assert force['threshold'] > 0
    
    def test_get_offsets(self):
        """오프셋 설정 반환 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        offsets = loader.get_offsets()
        
        assert 'up' in offsets
        assert 'grip' in offsets
        assert 'pick_extra_down' in offsets
        assert 'place_extra_down' in offsets
    
    def test_get_safety_limits(self):
        """안전 한계 설정 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        safety = loader.get_safety_limits()
        
        assert 'z_pick' in safety
        assert 'z_place' in safety
    
    def test_get_motion_settings(self):
        """모션 설정 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        motion = loader.get_motion_settings()
        
        assert 'velocity_move' in motion
        assert 'accel_move' in motion
        assert 'velocity_pick' in motion
        assert 'accel_pick' in motion
    
    def test_singleton_get_config(self):
        """get_config 싱글톤 테스트"""
        from dsr_integrated.config.yaml_loader import get_config
        
        config1 = get_config()
        config2 = get_config()
        
        # 동일 인스턴스여야 함
        assert config1 is config2
    
    def test_reload(self):
        """설정 리로드 테스트"""
        from dsr_integrated.config.yaml_loader import ConfigLoader
        loader = ConfigLoader()
        
        # 처음 로드
        positions1 = loader.positions
        
        # 리로드
        loader.reload()
        
        # 캐시가 초기화되었는지 확인
        assert loader._positions is None
        assert loader._settings is None
        
        # 다시 로드
        positions2 = loader.positions
        
        # 같은 값이어야 함
        assert positions1 == positions2


class TestModuleFunctions:
    """모듈 레벨 함수 테스트"""
    
    def test_load_positions(self):
        """load_positions 함수 테스트"""
        from dsr_integrated.config.yaml_loader import load_positions
        positions = load_positions()
        
        assert 'home' in positions
        assert 'pick' in positions
    
    def test_load_settings(self):
        """load_settings 함수 테스트"""
        from dsr_integrated.config.yaml_loader import load_settings
        settings = load_settings()
        
        assert 'robot' in settings
        assert 'force' in settings


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
