#!/usr/bin/env python3
"""
Firebase 연동 모듈
분류 결과를 Firebase Realtime Database에 저장
"""

import os
import json
from datetime import datetime
from typing import Optional
from rclpy.node import Node


class FirebaseHandler:
    """Firebase 연동 핸들러"""
    
    def __init__(self, node: Node = None):
        """
        Args:
            node: ROS2 노드 (로깅용)
        """
        self.node = node
        self.enabled = False
        self.db = None
        
        self._init_firebase()
    
    def log(self, msg: str, level: str = 'info'):
        """로깅 헬퍼"""
        if self.node:
            if level == 'info':
                self.node.get_logger().info(msg)
            elif level == 'warn':
                self.node.get_logger().warn(msg)
            elif level == 'error':
                self.node.get_logger().error(msg)
    
    def _init_firebase(self):
        """Firebase 초기화"""
        try:
            import firebase_admin
            from firebase_admin import credentials, db
            
            config_dir = os.path.expanduser('~/cobot1_ws/src/logistics_monitor/config')
            service_account_key = os.path.join(config_dir, 'serviceAccountKey.json')
            web_config_path = os.path.join(config_dir, 'firebase_web_config.json')
            
            if not os.path.exists(service_account_key):
                self.log('⚠️ Firebase serviceAccountKey.json 없음', 'warn')
                return
            
            if not os.path.exists(web_config_path):
                self.log('⚠️ Firebase web_config.json 없음', 'warn')
                return
            
            with open(web_config_path, 'r') as f:
                web_config = json.load(f)
            database_url = web_config.get('databaseURL')
            
            if not database_url:
                self.log('⚠️ Firebase databaseURL 없음', 'warn')
                return
            
            # 이미 초기화되어 있는지 확인
            try:
                firebase_admin.get_app()
                self.log('Firebase 이미 초기화됨')
            except ValueError:
                cred = credentials.Certificate(service_account_key)
                firebase_admin.initialize_app(cred, {'databaseURL': database_url})
                self.log('✅ Firebase 초기화 완료')
            
            self.db = db
            self.enabled = True
            
        except ImportError:
            self.log('⚠️ firebase_admin 모듈 없음', 'warn')
        except Exception as e:
            self.log(f'⚠️ Firebase 초기화 실패: {e}', 'warn')
    
    @property
    def is_enabled(self) -> bool:
        """Firebase 활성화 상태"""
        return self.enabled and self.db is not None
    
    def save_sort_result(
        self,
        box_type: str,
        position: list,
        force_value: float,
        success: bool = True
    ):
        """
        분류 결과 저장
        
        Args:
            box_type: 박스 타입 ('SMALL', 'MEDIUM', 'LONG')
            position: 배치 위치 [x, y, z]
            force_value: 접촉 시 힘 값
            success: 성공 여부
        """
        if not self.is_enabled:
            return
        
        try:
            # 히스토리 저장
            history_ref = self.db.reference('/sorting_history')
            history_ref.push({
                'timestamp': datetime.now().isoformat(),
                'box_type': box_type,
                'position': position,
                'force': force_value,
                'success': success
            })
            
            # 통계 업데이트
            stats_ref = self.db.reference('/statistics')
            current_stats = stats_ref.get() or {}
            
            total = current_stats.get('total_sorted', 0) + 1
            small = current_stats.get('small_count', 0)
            medium = current_stats.get('medium_count', 0)
            long_count = current_stats.get('long_count', 0)
            
            if box_type == 'SMALL':
                small += 1
            elif box_type == 'MEDIUM':
                medium += 1
            elif box_type in ('LONG', 'LARGE'):
                long_count += 1
            
            stats_ref.update({
                'total_sorted': total,
                'small_count': small,
                'medium_count': medium,
                'long_count': long_count,
                'last_updated': datetime.now().isoformat()
            })
            
            self.log(f'💾 Firebase 저장: {box_type}')
            
        except Exception as e:
            self.log(f'⚠️ Firebase 저장 실패: {e}', 'warn')
    
    def get_statistics(self) -> Optional[dict]:
        """통계 조회"""
        if not self.is_enabled:
            return None
        
        try:
            stats_ref = self.db.reference('/statistics')
            return stats_ref.get()
        except Exception as e:
            self.log(f'⚠️ Firebase 조회 실패: {e}', 'warn')
            return None
