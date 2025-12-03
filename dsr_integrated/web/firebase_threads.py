#!/usr/bin/env python3
"""
Firebase 연동 스레드 모듈
"""

import os
import json
import time

from .data_store import (
    robot_data, sort_status, ui_state, logs
)


# Firebase 설정
LOGISTICS_MONITOR_DIR = os.path.expanduser('~/cobot1_ws/src/logistics_monitor')
CONFIG_DIR = os.path.join(LOGISTICS_MONITOR_DIR, 'config')
SERVICE_ACCOUNT_KEY_PATH = os.path.join(CONFIG_DIR, 'serviceAccountKey.json')
WEB_CONFIG_PATH = os.path.join(CONFIG_DIR, 'firebase_web_config.json')

# Firebase 상태
firebase_enabled = False
firebase_ref = None
firebase_cmd_ref = None


def init_firebase():
    """Firebase 초기화"""
    global firebase_enabled, firebase_ref, firebase_cmd_ref
    
    try:
        import firebase_admin
        from firebase_admin import credentials, db
        
        if not os.path.exists(SERVICE_ACCOUNT_KEY_PATH):
            print("⚠️ Firebase serviceAccountKey.json 없음")
            return False
        
        if not os.path.exists(WEB_CONFIG_PATH):
            print("⚠️ Firebase web_config.json 없음")
            return False
        
        with open(WEB_CONFIG_PATH, 'r') as f:
            web_config = json.load(f)
        DATABASE_URL = web_config.get('databaseURL')
        
        if not DATABASE_URL:
            print("⚠️ Firebase databaseURL 없음")
            return False
        
        # 이미 초기화 확인
        try:
            firebase_admin.get_app()
        except ValueError:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        
        firebase_ref = db.reference('/robot_status')
        firebase_cmd_ref = db.reference('/robot_commands')
        firebase_enabled = True
        
        print("✅ Firebase 연동 활성화")
        return True
        
    except ImportError:
        print("⚠️ firebase_admin 모듈 없음")
    except Exception as e:
        print(f"⚠️ Firebase 초기화 실패: {e}")
    
    return False


def firebase_upload_thread(get_ros_node):
    """Firebase에 로봇 상태 주기적 업로드 (1초마다)"""
    global firebase_ref
    
    if not firebase_enabled or firebase_ref is None:
        return
    
    print("🔥 Firebase 업로드 스레드 시작")
    
    while True:
        try:
            if robot_data['connected']:
                ros_node = get_ros_node()
                if ros_node:
                    ui_state['pendulum_running'] = ros_node.pendulum_running
                
                recent_logs = logs[-20:] if logs else []
                
                upload_data = {
                    'timestamp': time.time(),
                    'connected': robot_data['connected'],
                    'joint_position': robot_data['actual_joint_position'],
                    'tcp_position': robot_data['actual_tcp_position'],
                    'robot_state': robot_data['robot_state'],
                    'robot_mode': robot_data['robot_mode'],
                    'operation_speed': robot_data['operation_speed_rate'],
                    'access_control': robot_data['access_control'],
                    'gripper': {
                        'do1': (robot_data['controller_digital_output'] >> 0) & 1,
                        'do2': (robot_data['controller_digital_output'] >> 1) & 1,
                    },
                    'ui_state': ui_state,
                    'sort_status': {
                        'running': sort_status.get('running', False),
                        'paused': sort_status.get('paused', False),
                        'phase': sort_status.get('phase', 'IDLE'),
                        'cycle_count': sort_status.get('cycle_count', 0),
                        'last_width': sort_status.get('last_width', None),
                        'dsr_ready': sort_status.get('dsr_ready', False),
                    },
                    'logs': recent_logs
                }
                firebase_ref.update(upload_data)
        except Exception as e:
            print(f"Firebase 업로드 오류: {e}")
        
        time.sleep(1)


def firebase_command_listener(get_ros_node, socketio):
    """Firebase에서 제어 명령 수신"""
    global firebase_cmd_ref
    
    if not firebase_enabled or firebase_cmd_ref is None:
        return
    
    def on_command(event):
        if event.data is None:
            return
        
        try:
            cmd = event.data.get('command') if isinstance(event.data, dict) else None
            if cmd is None:
                return
            
            print(f"🔥 Firebase 명령 수신: {cmd}")
            ros_node = get_ros_node()
            
            if cmd == 'gripper_open':
                if ros_node:
                    ros_node.set_gripper(open_gripper=True)
                    
            elif cmd == 'gripper_close':
                if ros_node:
                    ros_node.set_gripper(open_gripper=False)
                    
            elif cmd == 'move_home_user':
                if ros_node:
                    ros_node.move_home(target=1)
                    
            elif cmd == 'move_home_mech':
                if ros_node:
                    ros_node.move_home(target=0)
                    
            elif cmd == 'emergency_stop':
                if ros_node:
                    ros_node.emergency_stop()
                    if ros_node.pendulum_running:
                        ros_node.pause_pendulum_test()
                    ui_state['is_stopped'] = True
                    ui_state['pendulum_running'] = False
                        
            elif cmd == 'move_resume':
                if ros_node:
                    if ros_node.pendulum_paused:
                        ros_node.resume_pendulum_test()
                        ui_state['pendulum_running'] = True
                ui_state['is_stopped'] = False
                        
            elif cmd == 'pendulum_start':
                if ros_node:
                    joint = event.data.get('joint', 4)
                    amplitude = event.data.get('amplitude', 15)
                    velocity = event.data.get('velocity', 30)
                    ros_node.start_pendulum_test(joint, amplitude, velocity)
                    ui_state['pendulum_running'] = True
                    
            elif cmd == 'pendulum_stop':
                if ros_node:
                    ros_node.stop_pendulum_test()
                    ui_state['pendulum_running'] = False
                        
            elif cmd == 'speed_change':
                speed = event.data.get('value', 50)
                if ros_node:
                    ros_node.change_speed(speed)
            
            firebase_cmd_ref.update({
                'command': None, 
                'value': None, 
                'processed': time.time()
            })
            
            socketio.emit('ui_state', ui_state)
            if cmd in ['pendulum_start', 'pendulum_stop', 'emergency_stop', 'move_resume']:
                running = ui_state.get('pendulum_running', False)
                socketio.emit('pendulum_status', {'running': running})
            
        except Exception as e:
            print(f"Firebase 명령 처리 오류: {e}")
            import traceback
            traceback.print_exc()
    
    print("🔥 Firebase 명령 리스너 시작")
    firebase_cmd_ref.listen(on_command)


def is_firebase_enabled() -> bool:
    """Firebase 활성화 상태"""
    return firebase_enabled
