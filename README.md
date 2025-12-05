# DSR Integrated - 물류 분류 로봇 시스템

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

> **📚 [Interactive Documentation](https://taesla.github.io/doosan_rokey_collabo1/)** - 인터랙티브 슬라이드 문서 보기

Doosan M0609 로봇을 사용한 물류 자동 분류 시스템입니다.  
**Flask + SocketIO** 기반 웹 UI와 **ROS2 Humble**을 통합하여 실시간 로봇 제어 및 모니터링을 제공합니다.

---

## 📖 Documentation

| 문서 | 설명 |
|------|------|
| 🎯 [Interactive Slides](https://taesla.github.io/doosan_rokey_collabo1/) | 11장 인터랙티브 아키텍처 슬라이드 |
| 🔧 [Collision Recovery](https://taesla.github.io/collision_recovery/) | 충돌 복구 시스템 문서 |
| 📄 [Web Integration Report](docs/WEB_INTEGRATION_REPORT.md) | ROS2-Web 통합 상세 리포트 |

---

## ✨ 주요 기능

- 🤖 **자동 분류**: 컨베이어 감지 → 픽업 → 크기별 분류 → 적재
- 🌐 **웹 기반 제어**: 브라우저에서 로봇 상태 모니터링 및 제어
- 🔄 **충돌 자동 복구**: SAFE_STOP 감지 시 6단계 자동 복구
- 🔥 **Firebase 연동**: 작업 상태 클라우드 동기화
- 👀 **시스템 감시**: 드라이버 크래시 감지 및 자동 재시작

## 📁 프로젝트 구조

```
dsr_integrated/
├── config/                         # YAML 설정 파일
│   ├── positions.yaml              # 로봇 위치 좌표 (HOME, PICK, PLACE)
│   └── settings.yaml               # 로봇 설정값 (Force, Speed, Offset 등)
│
├── dsr_integrated/                 # Python 패키지
│   ├── nodes/                      # 🔵 ROS2 노드
│   │   ├── sort_node.py            # 메인 분류 노드
│   │   ├── server_node.py          # 웹서버 노드 (Flask + SocketIO)
│   │   └── recovery_node.py        # 충돌 복구 노드
│   │
│   ├── core/                       # 핵심 로직
│   │   └── robot_controller.py     # 로봇 제어 래퍼 클래스
│   │
│   ├── tasks/                      # 작업 모듈
│   │   ├── pick_place.py           # 픽업/플레이스 시퀀스
│   │   ├── pendulum.py             # 진자 운동 (데모)
│   │   └── base_task.py            # 태스크 베이스 클래스
│   │
│   ├── safety/                     # 안전 모듈
│   │   ├── collision_recovery.py   # 6단계 충돌 복구
│   │   └── safety_manager.py       # 안전 상태 관리
│   │
│   ├── monitoring/                 # 상태 모니터링
│   │   ├── state_monitor.py        # 로봇 상태 감시
│   │   └── state_manager.py        # 작업 상태/통계 관리
│   │
│   ├── integration/                # 외부 연동
│   │   ├── conveyor/               # 컨베이어 핸들러
│   │   └── firebase/               # Firebase 핸들러
│   │
│   ├── web/                        # 🌐 웹 서버 모듈
│   │   ├── routes.py               # Flask HTTP 라우트
│   │   ├── socket_handlers.py      # SocketIO 이벤트 핸들러
│   │   ├── data_store.py           # 전역 데이터 저장소
│   │   ├── robot_monitor.py        # 로봇 상태 주기적 조회
│   │   ├── firebase_threads.py     # Firebase 백그라운드 스레드
│   │   └── standalone_server.py    # 독립 웹 서버 (ROS 없이)
│   │
│   └── config/                     # 설정 모듈
│       ├── positions.py            # 좌표 상수
│       ├── constants.py            # 일반 상수
│       └── yaml_loader.py          # YAML 설정 로더
│
├── launch/                         # Launch 파일
│   ├── full_system.launch.py       # 전체 시스템 (드라이버 + 노드)
│   └── nodes_only.launch.py        # 노드만 (드라이버 별도)
│
├── test/                           # 단위 테스트
│   ├── test_yaml_loader.py
│   ├── test_state_manager.py
│   └── test_config.py
│
└── docs/                           # 문서
    └── architecture.md             # 아키텍처 설명
```

## 🚀 빠른 시작

### 1. 의존성 설치
```bash
# ROS2 Humble 설치 필요
# Doosan Robot ROS2 패키지 필요 (doosan-robot2)

# Python 패키지
pip install flask flask-socketio pyyaml firebase-admin pyserial
```

### 2. 빌드
```bash
cd ~/cobot1_ws
colcon build --packages-select dsr_integrated arduino_conveyor_bridge --symlink-install
source install/setup.bash
```

### 3. 실행

**전체 시스템 (로봇 드라이버 + 모든 노드):**
```bash
ros2 launch dsr_integrated full_system.launch.py mode:=real host:=192.168.137.100
```

**개별 노드 실행 (드라이버 별도 실행 시):**
```bash
# 분류 노드
ros2 run dsr_integrated sort_node

# 웹서버 노드
ros2 run dsr_integrated web_server_node

# 컨베이어 브릿지
ros2 run arduino_conveyor_bridge serial_to_topic
```

**시스템 감시자 (자동 재시작 포함):**
```bash
ros2 run dsr_recovery_watcher watcher_node --ros-args -p auto_launch:=true
```

### 4. 웹 UI 접속
```
http://<로봇PC_IP>:5000
```

## 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                     웹 브라우저 (UI)                        │
│              index_full.html + SocketIO                     │
└─────────────────────────────────────────────────────────────┘
                              │ WebSocket
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   web_server_node                           │
│              Flask + SocketIO + ROS2 Node                   │
└─────────────────────────────────────────────────────────────┘
                              │ ROS2 Services
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                     sort_node                               │
│      PickPlaceTask + ConveyorHandler + StateManager        │
└─────────────────────────────────────────────────────────────┘
                              │ ROS2 Services
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  doosan-robot2 드라이버                      │
│         dsr_controller2 + dsr_hardware2 + DRFL             │
└─────────────────────────────────────────────────────────────┘
                              │ TCP/IP
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                 Doosan M0609 Robot                          │
└─────────────────────────────────────────────────────────────┘
```

## ⚙️ 설정 변경

### 로봇 위치 수정 (`config/positions.yaml`)
```yaml
home:
  x: 367.53
  y: 4.02
  z: 203.18
  rx: 164.67
  ry: -179.96
  rz: 164.99
```

### 동작 파라미터 수정 (`config/settings.yaml`)
```yaml
force:
  threshold: 30.0       # 접촉 감지 임계값 (N)
  push_force: 50.0      # Compliance 인가 힘 (N)
  
motion:
  velocity_move: 200.0  # 이동 속도 (mm/s)
  accel_move: 400.0     # 이동 가속도 (mm/s²)
```

> ⚠️ 설정 변경 후 노드 재시작 필요

## 🔌 ROS2 인터페이스

### 서비스 (Services)
| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/dlar/start_sort` | Trigger | 분류 작업 시작 |
| `/dlar/stop_sort` | Trigger | 분류 작업 중지 |
| `/dlar/pause_sort` | SetBool | 일시정지/재개 |
| `/dlar/reset_state` | Trigger | 상태 초기화 |
| `/dlar/conveyor_mode` | SetBool | 컨베이어 자동 모드 |
| `/dlar/collision_recovery` | Trigger | 충돌 복구 실행 |

### 토픽 (Topics)
| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| `/dlar/status` | String | sort_node | 분류 상태 JSON |
| `/dlar/is_running` | Bool | sort_node | 실행 중 여부 |
| `/dlar/recovery_status` | String | sort_node | 복구 상태 |
| `/conveyor/status` | String | serial_bridge | 컨베이어 상태 |
| `/conveyor/status_code` | Int32 | serial_bridge | 박스 크기 코드 (1=S, 2=M, 3=L) |
| `/conveyor/cmd` | String | sort_node | 컨베이어 명령 |

### 서비스 호출 예시
```bash
# 분류 시작
ros2 service call /dlar/start_sort std_srvs/srv/Trigger

# 컨베이어 자동 모드 활성화
ros2 service call /dlar/conveyor_mode std_srvs/srv/SetBool "{data: true}"

# 충돌 복구
ros2 service call /dlar/collision_recovery std_srvs/srv/Trigger

# 상태 확인
ros2 topic echo /dlar/status --once
```

## 🔄 충돌 복구 시스템

로봇이 SAFE_STOP 상태에 진입하면 자동으로 6단계 복구를 수행합니다:

```
1. SAFE_STOP 리셋 (set_robot_control: 2)
2. RECOVERY 모드 진입 (set_safety_mode: mode=2, event=2)
3. Z축 Jog 상승 (100mm)
4. RECOVERY 완료 (set_safety_mode: event=0)
5. RECOVERY 모드 해제
6. 서보 ON (set_robot_control: 3)
```

웹 UI에서 "복구" 버튼으로 수동 실행도 가능합니다.

## 🔄 동작 흐름

```
1. 컨베이어에서 물체 감지 (DETECTED:S/M/L)
       ↓
2. HOME 위치에서 PICK 위치로 이동
       ↓
3. Compliance Control + Force 센서로 높이 측정
       ↓
4. 물체 집기 (그리퍼)
       ↓
5. 물체 크기에 따라 팔레트 선택 (S/M/L)
       ↓
6. 해당 팔레트에 적재 (2층 스태킹)
       ↓
7. HOME으로 복귀, 다음 물체 대기
```

**9사이클 적재 패턴:**
- 1차: S(2개) → M(2개) → L(2개) = 6개
- 충돌 시 자동 복구 후 계속

## 🧪 테스트

```bash
cd ~/cobot1_ws/src/dsr_integrated
python3 -m pytest test/ -v

# 특정 테스트만
python3 -m pytest test/test_state_manager.py -v
```

## 📦 의존성

### 필수
- ROS2 Humble
- Doosan Robot ROS2 패키지 (`doosan-robot2`)
- Python 3.10+

### Python 패키지
```bash
pip install flask flask-socketio pyyaml pyserial
pip install firebase-admin  # Firebase 사용 시
```

### 관련 패키지 (src/ 내)
- `arduino_conveyor_bridge`: 컨베이어 시리얼 브릿지
- `dsr_recovery_watcher`: 시스템 감시 및 자동 재시작
- `logistics_monitor`: 웹 UI 템플릿 (HTML/CSS/JS)

## 🛠️ 트러블슈팅

### DSR 로봇 연결 실패
```bash
# 로봇 IP 확인
ping 192.168.137.100

# DSR 서비스 확인
ros2 service list | grep dsr01

# 드라이버 재시작
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100
```

### 컨베이어 연결 실패
```bash
# 시리얼 포트 확인
ls -la /dev/ttyACM* /dev/ttyUSB*

# 권한 설정
sudo chmod 666 /dev/ttyACM0
sudo usermod -aG dialout $USER
```

### 충돌 후 로봇 멈춤
```bash
# 수동 복구
ros2 service call /dlar/collision_recovery std_srvs/srv/Trigger

# 또는 웹 UI에서 "복구" 버튼 클릭
```

### 웹 서버 접속 안됨
```bash
# 포트 확인
ss -tlnp | grep 5000

# 방화벽 허용
sudo ufw allow 5000
```

## 📝 변경 이력

- **2025-12-05**: 충돌 복구 시스템 고도화
  - 6단계 자동 복구 알고리즘
  - 드라이버 크래시 원인 분석 및 문서화
  - ROS2 웹 통합 아키텍처 문서 추가

- **2025-12-04**: 시스템 감시자 추가
  - watcher_node: 드라이버 상태 감시
  - 자동 재시작 및 Firebase 상태 복원

- **2025-12-02**: 코드 리팩토링 완료
  - 모듈 분리 (1204줄 → 16개 모듈)
  - YAML 설정 파일 도입
  - 단위 테스트 추가

## 📄 라이선스

Apache 2.0

## 👥 기여자

- Doosan Rokey Collaboration Team
