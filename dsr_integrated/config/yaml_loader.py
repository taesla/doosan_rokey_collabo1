#!/usr/bin/env python3
"""
YAML 설정 로더 모듈
positions.yaml, settings.yaml 파일을 로드하여 설정값 제공
"""

import os
import yaml
from typing import Dict, List, Optional, Any
from ament_index_python.packages import get_package_share_directory


class ConfigLoader:
    """YAML 설정 파일 로더"""
    
    def __init__(self, package_name: str = 'dsr_integrated'):
        """
        Args:
            package_name: ROS2 패키지 이름
        """
        self.package_name = package_name
        self._positions = None
        self._settings = None
        self._config_dir = None
    
    @property
    def config_dir(self) -> str:
        """설정 파일 디렉토리 경로"""
        if self._config_dir is None:
            try:
                share_dir = get_package_share_directory(self.package_name)
                self._config_dir = os.path.join(share_dir, 'config')
            except Exception:
                # 개발 환경에서는 소스 디렉토리 사용
                self._config_dir = os.path.expanduser(
                    f'~/cobot1_ws/src/{self.package_name}/config'
                )
        return self._config_dir
    
    def _load_yaml(self, filename: str) -> dict:
        """YAML 파일 로드"""
        filepath = os.path.join(self.config_dir, filename)
        
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"설정 파일 없음: {filepath}")
        
        with open(filepath, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    
    @property
    def positions(self) -> dict:
        """위치 설정 로드"""
        if self._positions is None:
            self._positions = self._load_yaml('positions.yaml')
        return self._positions
    
    @property
    def settings(self) -> dict:
        """일반 설정 로드"""
        if self._settings is None:
            self._settings = self._load_yaml('settings.yaml')
        return self._settings
    
    def reload(self):
        """설정 다시 로드"""
        self._positions = None
        self._settings = None
    
    # =========================================
    # 위치 값 접근자
    # =========================================
    def get_home_position(self) -> List[float]:
        """홈 위치 [x, y, z, rx, ry, rz]"""
        p = self.positions['home']
        return [p['x'], p['y'], p['z'], p['rx'], p['ry'], p['rz']]
    
    def get_pick_position(self) -> List[float]:
        """픽업 위치 [x, y, z, rx, ry, rz]"""
        p = self.positions['pick']
        return [p['x'], p['y'], p['z'], p['rx'], p['ry'], p['rz']]
    
    def get_place_position(self, size: str) -> List[float]:
        """
        배치 위치 반환
        
        Args:
            size: 'small', 'medium', 'large', 'long'
        """
        # long은 large로 매핑
        size_key = 'large' if size.lower() == 'long' else size.lower()
        
        if size_key not in self.positions['place']:
            raise ValueError(f"알 수 없는 크기: {size}")
        
        p = self.positions['place'][size_key]
        return [p['x'], p['y'], p['z'], p['rx'], p['ry'], p['rz']]
    
    def get_joint_home(self) -> List[float]:
        """조인트 홈 위치"""
        return self.positions.get('joint_home', [0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    
    # =========================================
    # 설정 값 접근자
    # =========================================
    def get_robot_id(self) -> str:
        """로봇 ID"""
        return self.settings['robot']['id']
    
    def get_robot_model(self) -> str:
        """로봇 모델"""
        return self.settings['robot']['model']
    
    def get_force_settings(self) -> Dict[str, float]:
        """Force 센서 설정"""
        return self.settings['force']
    
    def get_offsets(self) -> Dict[str, float]:
        """오프셋 설정"""
        return self.settings['offsets']
    
    def get_safety_limits(self) -> Dict[str, float]:
        """안전 한계 설정"""
        safety = self.settings['safety']
        return {
            'z_pick': safety.get('z_pick_limit', safety.get('z_pick', 103.0)),
            'z_place': safety.get('z_place_limit', safety.get('z_place', 12.5))
        }
    
    def get_motion_settings(self) -> Dict[str, float]:
        """모션 설정"""
        return self.settings['motion']
    
    def get_compliance_stiffness(self) -> List[float]:
        """Compliance 강성 설정"""
        return self.settings['compliance']['stiffness']
    
    def get_conveyor_settings(self) -> Dict[str, Any]:
        """컨베이어 설정"""
        return self.settings['conveyor']
    
    def get_web_server_settings(self) -> Dict[str, Any]:
        """웹서버 설정"""
        return self.settings['web_server']


# 싱글톤 인스턴스
_config_loader = None


def get_config() -> ConfigLoader:
    """설정 로더 싱글톤 반환"""
    global _config_loader
    if _config_loader is None:
        _config_loader = ConfigLoader()
    return _config_loader


# 편의 함수들
def load_positions() -> dict:
    """위치 설정 로드"""
    return get_config().positions


def load_settings() -> dict:
    """일반 설정 로드"""
    return get_config().settings
