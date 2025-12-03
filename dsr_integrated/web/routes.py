#!/usr/bin/env python3
"""
Flask HTTP 라우트 모듈
"""

import os
import json
from flask import Blueprint, render_template, jsonify

from .data_store import robot_data, sort_status, conveyor_status, logistics_status

# 경로 설정
LOGISTICS_MONITOR_DIR = os.path.expanduser('~/cobot1_ws/src/logistics_monitor')
CONFIG_DIR = os.path.join(LOGISTICS_MONITOR_DIR, 'config')
WEB_CONFIG_PATH = os.path.join(CONFIG_DIR, 'firebase_web_config.json')

# Blueprint 생성
routes_bp = Blueprint('routes', __name__)


@routes_bp.route('/')
def index():
    """메인 페이지"""
    return render_template('index.html')


@routes_bp.route('/external')
def external():
    """외부 접속용 페이지"""
    return render_template('index_tabbed.html')


@routes_bp.route('/api/status')
def get_status():
    """로봇 상태 API"""
    return jsonify(robot_data)


@routes_bp.route('/api/sort_status')
def get_sort_status():
    """분류 상태 API"""
    return jsonify(sort_status)


@routes_bp.route('/api/conveyor_status')
def get_conveyor_status():
    """컨베이어 상태 API"""
    return jsonify(conveyor_status)


@routes_bp.route('/api/logistics_status')
def get_logistics_status():
    """물류 적재 상태 API"""
    return jsonify(logistics_status)


@routes_bp.route('/firebase_config')
def get_firebase_config():
    """Firebase 웹 설정 반환"""
    try:
        with open(WEB_CONFIG_PATH, 'r') as f:
            config = json.load(f)
        return jsonify(config)
    except Exception as e:
        print(f"⚠️ Firebase 설정 로드 실패: {e}")
        return jsonify({'error': 'Firebase config not found'}), 404
