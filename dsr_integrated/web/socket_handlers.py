#!/usr/bin/env python3
"""
SocketIO 이벤트 핸들러 모듈
"""

from .data_store import (
    robot_data, sort_status, conveyor_status, logistics_status,
    ui_state, logs, add_log, one_take_status
)
from ..safety import SafetyManager


def register_socket_handlers(socketio, get_ros_node):
    """SocketIO 이벤트 핸들러 등록
    
    Args:
        socketio: SocketIO 인스턴스
        get_ros_node: ROS 노드를 반환하는 callable
    """
    
    @socketio.on('connect')
    def handle_connect():
        print('✅ Client connected')

    @socketio.on('disconnect')
    def handle_disconnect():
        print('❌ Client disconnected')

    @socketio.on('sort_start')
    def handle_sort_start():
        """분류 시작"""
        ros_node = get_ros_node()
        if ros_node:
            success, message = ros_node.call_start_sort()
            add_log('INFO' if success else 'ERROR', f'분류 시작: {message}')
            socketio.emit('sort_result', {'success': success, 'message': message})

    @socketio.on('sort_stop')
    def handle_sort_stop():
        """분류 정지"""
        ros_node = get_ros_node()
        if ros_node:
            success, message = ros_node.call_stop_sort()
            add_log('INFO' if success else 'ERROR', f'분류 정지: {message}')
            socketio.emit('sort_result', {'success': success, 'message': message})

    @socketio.on('sort_pause')
    def handle_sort_pause(data=None):
        """분류 일시정지"""
        ros_node = get_ros_node()
        if data is None:
            data = {}
        pause = data.get('pause', True)
        if ros_node:
            success, message = ros_node.call_pause_sort(pause)
            add_log('INFO' if success else 'ERROR', f'분류 {"일시정지" if pause else "재개"}: {message}')
            socketio.emit('sort_result', {'success': success, 'message': message})

    @socketio.on('sort_resume')
    def handle_sort_resume():
        """분류 재개"""
        ros_node = get_ros_node()
        if ros_node:
            success, message = ros_node.call_pause_sort(False)
            add_log('INFO' if success else 'ERROR', f'분류 재개: {message}')
            socketio.emit('sort_result', {'success': success, 'message': message})

    @socketio.on('sort_reset')
    def handle_sort_reset():
        """상태 초기화"""
        ros_node = get_ros_node()
        if ros_node:
            success, message = ros_node.call_reset_state()
            add_log('INFO' if success else 'ERROR', f'상태 초기화: {message}')
            socketio.emit('sort_result', {'success': success, 'message': message})

    @socketio.on('conveyor_mode')
    def handle_conveyor_mode(data):
        """컨베이어 자동 모드 설정"""
        ros_node = get_ros_node()
        enabled = data.get('enabled', False)
        if ros_node:
            success, message = ros_node.call_conveyor_mode(enabled)
            add_log('INFO' if success else 'ERROR', 
                   f'컨베이어 자동 모드 {"활성화" if enabled else "비활성화"}: {message}')
            socketio.emit('conveyor_result', 
                         {'success': success, 'message': message, 'enabled': enabled})
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')
            socketio.emit('conveyor_result', 
                         {'success': False, 'message': 'ROS 노드 초기화 안됨', 'enabled': False})

    @socketio.on('collision_recovery')
    def handle_collision_recovery():
        """충돌 복구 요청"""
        ros_node = get_ros_node()
        print('🔧 COLLISION RECOVERY')
        
        if ros_node:
            success, message = ros_node.call_collision_recovery()
            add_log('INFO' if success else 'ERROR', f'충돌 복구: {message}')
            socketio.emit('recovery_result', {'success': success, 'message': message})
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')
            socketio.emit('recovery_result', {'success': False, 'message': 'ROS 노드 초기화 안됨'})

    @socketio.on('gripper_command')
    def handle_gripper(data):
        """그리퍼 제어"""
        ros_node = get_ros_node()
        command = data.get('command', 'close')
        print(f'🤖 Gripper command: {command}')
        
        if ros_node:
            open_gripper = (command == 'open')
            success = ros_node.set_gripper(open_gripper)
            if success:
                add_log('INFO', f'그리퍼 {command} 명령 전송')
            else:
                add_log('WARN', f'그리퍼 서비스 준비 안됨')
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('move_home')
    def handle_move_home(data):
        """홈 이동"""
        ros_node = get_ros_node()
        # target: 0=기계적홈, 1=사용자홈 (직접 전달)
        # 또는 type: 'mechanical'/'user' (레거시 호환)
        if 'target' in data:
            target = data.get('target', 1)
        else:
            home_type = data.get('type', 'user')
            target = 0 if home_type == 'mechanical' else 1
        
        home_name = "기계적 홈" if target == 0 else "사용자 홈"
        print(f'🏠 Move Home: {home_name} (target={target})')
        
        if ros_node:
            success = ros_node.move_home(target)
            if success:
                add_log('INFO', f'홈 이동 명령 전송 ({home_name})')
            else:
                add_log('WARN', f'홈 이동 서비스 준비 안됨')
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('speed_change')
    def handle_speed(data):
        """속도 변경"""
        ros_node = get_ros_node()
        speed = data.get('speed', 50)
        print(f'🚀 Speed change: {speed}%')
        
        if ros_node:
            success = ros_node.change_speed(speed)
            if success:
                add_log('INFO', f'작업 속도 변경: {speed}%')
            else:
                add_log('WARN', f'속도 변경 서비스 준비 안됨')
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('emergency_stop')
    def handle_estop():
        """긴급정지 - SafetyManager + ROS 서비스 호출"""
        print('🛑 EMERGENCY STOP')
        
        # 1. SafetyManager 통해 전역 상태 설정
        success = SafetyManager.emergency_stop("웹 UI 비상정지")
        
        # 2. ROS 서비스 호출 (sort_node의 비상정지)
        ros_node = get_ros_node()
        if ros_node:
            try:
                ros_success, ros_msg = ros_node.call_stop_sort()
                print(f'🛑 ROS 비상정지: {ros_success} - {ros_msg}')
            except Exception as e:
                print(f'⚠️ ROS 비상정지 호출 실패: {e}')
        
        if success:
            add_log('ERROR', '🛑 긴급정지 실행')
            # 비상정지 상태 브로드캐스트
            socketio.emit('safety_state', {
                'state': 'emergency_stop',
                'is_safe': False
            })
        else:
            add_log('WARN', '긴급정지 실패')

    @socketio.on('emergency_stop_release')
    def handle_estop_release():
        """긴급정지 해제 - SafetyManager를 통해 전역 처리"""
        print('▶️ EMERGENCY STOP RELEASE')
        
        success = SafetyManager.emergency_release()
        if success:
            add_log('INFO', '▶️ 긴급정지 해제')
            # 상태 브로드캐스트
            socketio.emit('safety_state', {
                'state': 'normal',
                'is_safe': True
            })
        else:
            add_log('WARN', '긴급정지 해제 실패')

    @socketio.on('move_pause')
    def handle_move_pause():
        """일시정지"""
        ros_node = get_ros_node()
        print('⏸️ MOVE PAUSE')
        
        if ros_node:
            success = ros_node.pause_motion()
            if success:
                add_log('INFO', '⏸️ 일시정지 실행')
            else:
                add_log('WARN', '일시정지 서비스 준비 안됨')
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('move_resume')
    def handle_move_resume():
        """재개"""
        ros_node = get_ros_node()
        print('▶️ MOVE RESUME')
        
        if ros_node:
            success = ros_node.resume_motion()
            if success:
                add_log('INFO', '▶️ 재개 실행')
            else:
                add_log('WARN', '재개 서비스 준비 안됨')
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('conveyor_resume')
    def handle_conveyor_resume():
        """컨베이어 재시작"""
        ros_node = get_ros_node()
        print('🚚 Conveyor resume requested')
        
        if ros_node:
            success = ros_node.send_conveyor_cmd('RESUME')
            if success:
                add_log('INFO', '🚚 컨베이어 재시작')
            else:
                add_log('WARN', '컨베이어 명령 전송 실패')
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('conveyor_command')
    def handle_conveyor(data):
        """컨베이어 제어"""
        ros_node = get_ros_node()
        command = data.get('command', '')
        print(f'🚚 Conveyor command: {command}')
        
        if ros_node:
            success = ros_node.send_conveyor_cmd(command)
            if success:
                add_log('INFO', f'컨베이어 명령: {command}')
            else:
                add_log('WARN', '컨베이어 명령 전송 실패')
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('pendulum_start')
    def handle_pendulum_start(data):
        """진자운동 테스트 시작"""
        ros_node = get_ros_node()
        # joint_index 또는 joint 키 모두 지원
        joint_index = data.get('joint_index', data.get('joint', 0))
        amplitude = data.get('amplitude', 15.0)
        velocity = data.get('velocity', 30.0)
        
        print(f'🔄 Pendulum test start: J{joint_index+1}, ±{amplitude}°, {velocity}°/s')
        
        if ros_node:
            success = ros_node.start_pendulum_test(joint_index, amplitude, velocity)
            if success:
                ui_state['pendulum_running'] = True
                socketio.emit('pendulum_status', {
                    'running': True, 
                    'joint': joint_index, 
                    'amplitude': amplitude, 
                    'velocity': velocity
                })
                socketio.emit('ui_state', ui_state)
            else:
                add_log('WARN', '진자운동 테스트가 이미 실행 중')
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('pendulum_stop')
    def handle_pendulum_stop():
        """진자운동 테스트 정지"""
        ros_node = get_ros_node()
        print('⏹️ Pendulum test stop')
        
        if ros_node:
            ros_node.stop_pendulum_test()
            ui_state['pendulum_running'] = False
            socketio.emit('pendulum_status', {'running': False})
            socketio.emit('ui_state', ui_state)
        else:
            add_log('ERROR', 'ROS 노드 초기화 안됨')

    @socketio.on('get_ui_state')
    def handle_get_ui_state():
        """현재 UI 상태 전송"""
        ros_node = get_ros_node()
        if ros_node:
            ui_state['pendulum_running'] = ros_node.pendulum_running
        socketio.emit('ui_state', ui_state)

    @socketio.on('one_take_start')
    def handle_one_take_start():
        """
        원테이크 시나리오 시작
        1. 1차 분류: 컨베이어 → 분류 구역 (9개)
        2. 2차 적재: 분류 구역 → 적재 구역 (6개 테트리스)
        """
        ros_node = get_ros_node()
        print('🚀 ONE TAKE SCENARIO START')
        
        if not ros_node:
            add_log('ERROR', 'ROS 노드 초기화 안됨')
            socketio.emit('one_take_result', {'success': False, 'message': 'ROS 노드 초기화 안됨'})
            return
        
        try:
            # 원테이크 상태 초기화
            one_take_status['running'] = True
            one_take_status['phase'] = 'SORTING'
            one_take_status['sorting_complete'] = False
            one_take_status['stacking_complete'] = False
            one_take_status['stacking_step'] = 0
            one_take_status['total_sorted'] = 0
            
            # Step 1: 컨베이어 자동 모드 활성화 (내부에서 홈 이동 + 분류 시작)
            success, message = ros_node.call_conveyor_mode(True)
            
            if success:
                add_log('INFO', '🚀 원테이크 시나리오 시작!')
                add_log('INFO', '  [1단계] 1차 분류 시작 (목표: 9개)')
                add_log('INFO', '  → 컨베이어 자동 모드 활성화')
                add_log('INFO', '  → 물체 감지 대기 중...')
                socketio.emit('one_take_result', {
                    'success': True, 
                    'message': '원테이크 시나리오 시작됨 - 1차 분류 진행 중'
                })
                socketio.emit('one_take_status', one_take_status)
            else:
                one_take_status['running'] = False
                one_take_status['phase'] = 'IDLE'
                add_log('ERROR', f'원테이크 시나리오 시작 실패: {message}')
                socketio.emit('one_take_result', {'success': False, 'message': message})
                
        except Exception as e:
            one_take_status['running'] = False
            one_take_status['phase'] = 'IDLE'
            add_log('ERROR', f'원테이크 시나리오 오류: {e}')
            socketio.emit('one_take_result', {'success': False, 'message': str(e)})

    @socketio.on('one_take_stop')
    def handle_one_take_stop():
        """원테이크 시나리오 중지"""
        ros_node = get_ros_node()
        print('⏹️ ONE TAKE SCENARIO STOP')
        
        if not ros_node:
            add_log('ERROR', 'ROS 노드 초기화 안됨')
            socketio.emit('one_take_result', {'success': False, 'message': 'ROS 노드 초기화 안됨'})
            return
        
        try:
            # 1. 분류 정지
            ros_node.call_stop_sort()
            
            # 2. 컨베이어 자동 모드 비활성화
            ros_node.call_conveyor_mode(False)
            
            # 3. 원테이크 상태 초기화
            one_take_status['running'] = False
            one_take_status['phase'] = 'IDLE'
            
            add_log('INFO', '⏹️ 원테이크 시나리오 중지됨')
            socketio.emit('one_take_result', {'success': True, 'message': '원테이크 시나리오 중지됨'})
            socketio.emit('one_take_status', one_take_status)
            
        except Exception as e:
            add_log('ERROR', f'원테이크 시나리오 중지 오류: {e}')
            socketio.emit('one_take_result', {'success': False, 'message': str(e)})
    
    @socketio.on('stacking_start')
    def handle_stacking_start():
        """2차 적재 수동 시작"""
        ros_node = get_ros_node()
        print('📦 STACKING START')
        
        if not ros_node:
            add_log('ERROR', 'ROS 노드 초기화 안됨')
            socketio.emit('stacking_result', {'success': False, 'message': 'ROS 노드 초기화 안됨'})
            return
        
        try:
            # 분류 먼저 정지
            ros_node.call_stop_sort()
            ros_node.call_conveyor_mode(False)
            
            add_log('INFO', '📦 2차 적재(테트리스) 시작')
            add_log('INFO', '  순서: MEDIUM→LARGE→LARGE→MEDIUM→SMALL→SMALL')
            
            # 2차 적재 실행 (백그라운드 스레드)
            import threading
            def run_stacking():
                try:
                    success = ros_node.run_stacking_task()
                    if success:
                        add_log('INFO', '✅ 2차 적재 완료!')
                        socketio.emit('stacking_result', {'success': True, 'message': '2차 적재 완료'})
                    else:
                        add_log('ERROR', '2차 적재 실패')
                        socketio.emit('stacking_result', {'success': False, 'message': '2차 적재 실패'})
                except Exception as e:
                    add_log('ERROR', f'2차 적재 오류: {e}')
                    socketio.emit('stacking_result', {'success': False, 'message': str(e)})
            
            threading.Thread(target=run_stacking, daemon=True).start()
            socketio.emit('stacking_result', {'success': True, 'message': '2차 적재 시작됨'})
            
        except Exception as e:
            add_log('ERROR', f'2차 적재 시작 오류: {e}')
            socketio.emit('stacking_result', {'success': False, 'message': str(e)})
    
    @socketio.on('get_one_take_status')
    def handle_get_one_take_status():
        """원테이크 상태 조회"""
        socketio.emit('one_take_status', one_take_status)

    @socketio.on('logistics_reset')
    def handle_logistics_reset():
        """물류 데이터 초기화"""
        from .data_store import reset_logistics_status, logistics_status
        
        print('🗑️ LOGISTICS DATA RESET')
        reset_logistics_status()
        add_log('INFO', '물류 데이터 초기화됨')
        
        # 초기화된 상태 브로드캐스트
        socketio.emit('logistics_status', logistics_status)
        socketio.emit('logistics_result', {'success': True, 'message': '물류 데이터 초기화 완료'})

    @socketio.on('reset_all')
    def handle_reset_all():
        """전체 상태 초기화 (새로 시작)"""
        from .data_store import reset_all_status, logistics_status, sort_status, one_take_status
        
        print('🗑️ RESET ALL - 전체 초기화')
        reset_all_status()
        add_log('INFO', '전체 상태 초기화 - 새로 시작')
        
        # 초기화된 상태 브로드캐스트
        socketio.emit('logistics_status', logistics_status)
        socketio.emit('sort_status', sort_status)
        socketio.emit('one_take_status', one_take_status)
        socketio.emit('reset_all_result', {'success': True, 'message': '전체 초기화 완료'})
