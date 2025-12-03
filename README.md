# DSR Integrated - 물류 분류 로봇 시스템

Doosan M0609 로봇을 사용한 물류 자동 분류 시스템입니다.

## 📁 프로젝트 구조

```
dsr_integrated/
├── config/                     # YAML 설정 파일
│   ├── positions.yaml          # 로봇 위치 좌표 (HOME, PICK, PLACE)
│   └── settings.yaml           # 로봇 설정값 (Force, Speed, Offset 등)
│
├── dsr_integrated/             # Python 패키지
│   ├── config/                 # 설정 모듈
│   │   ├── positions.py        # 좌표 상수 (Python)
│   │   ├── constants.py        # 일반 상수
│   │   └── yaml_loader.py      # YAML 설정 로더
│   │
│   ├── web/                    # 웹서버 모듈
│   │   ├── routes.py           # Flask HTTP 라우트
│   │   ├── socket_handlers.py  # SocketIO 이벤트 핸들러
│   │   ├── data_store.py       # 전역 데이터 저장소
│   │   └── firebase_threads.py # Firebase 백그라운드 스레드
│   │
│   ├── sort_node.py            # 🤖 메인 분류 노드 (ROS2)
│   ├── server_node.py          # 🌐 웹서버 노드 (Flask + SocketIO)
│   ├── robot_controller.py     # 로봇 제어 래퍼 클래스
│   ├── state_manager.py        # 상태/통계 관리
│   ├── conveyor_handler.py     # 컨베이어 통신 핸들러
│   └── firebase_handler.py     # Firebase 연동
│
├── launch/                     # Launch 파일
│   └── full_system.launch.py   # 전체 시스템 실행
│
├── test/                       # 단위 테스트
│   ├── test_yaml_loader.py
│   ├── test_state_manager.py
│   └── test_config.py
│
└── archive/                    # 백업 (기존 파일)
    ├── dlar_sort_node.py       # 원본 분류 노드
    └── web_server_node.py      # 원본 웹서버 노드
```

## 🚀 빠른 시작

### 1. 빌드
```bash
cd ~/cobot1_ws
colcon build --packages-select dsr_integrated --symlink-install
source install/setup.bash
```

### 2. 실행

**전체 시스템 (로봇 + 웹서버):**
```bash
ros2 launch dsr_integrated full_system.launch.py
```

**개별 노드 실행:**
```bash
# 분류 노드만
ros2 run dsr_integrated sort_node

# 웹서버만
ros2 run dsr_integrated server_node
```

### 3. 웹 UI 접속
- URL: `http://localhost:5000`
- 로봇 제어, 상태 모니터링, 분류 작업 관리

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

### 서비스
| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/dlar/start_sort` | Trigger | 분류 작업 시작 |
| `/dlar/stop_sort` | Trigger | 분류 작업 중지 |
| `/dlar/pause_sort` | SetBool | 일시정지/재개 |
| `/dlar/reset_state` | Trigger | 상태 초기화 |
| `/dlar/conveyor_mode` | SetBool | 컨베이어 자동 모드 |

### 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/dlar/status` | String (JSON) | 로봇 상태 정보 |
| `/dlar/is_running` | Bool | 실행 중 여부 |
| `/conveyor/detect` | Int32 | 컨베이어 물체 감지 |

### 서비스 호출 예시
```bash
# 분류 시작
ros2 service call /dlar/start_sort std_srvs/srv/Trigger

# 컨베이어 자동 모드 활성화
ros2 service call /dlar/conveyor_mode std_srvs/srv/SetBool "{data: true}"

# 상태 확인
ros2 topic echo /dlar/status --once
```

## 🔄 동작 흐름

```
1. 컨베이어에서 물체 감지
       ↓
2. HOME 위치에서 PICK 위치로 이동
       ↓
3. Compliance Control + Force 센서로 높이 측정
       ↓
4. 물체 집기 (그리퍼)
       ↓
5. 물체 크기 분류 (SMALL/MEDIUM/LARGE)
       ↓
6. 해당 팔레트에 배치
       ↓
7. HOME으로 복귀, 다음 물체 대기
```

## 🧪 테스트

```bash
cd ~/cobot1_ws/src/dsr_integrated
python3 -m pytest test/ -v
```

## 📦 의존성

- ROS2 Humble
- Doosan Robot ROS2 패키지 (`doosan-robot2`)
- Python 패키지: `flask`, `flask-socketio`, `pyyaml`, `firebase-admin` (선택)

## 🛠️ 트러블슈팅

### DSR 로봇 연결 실패
```bash
# 로봇 IP 확인
ping 192.168.137.100

# DSR 서비스 확인
ros2 service list | grep dsr01
```

### 컨베이어 연결 실패
```bash
# 시리얼 포트 확인
ls -la /dev/ttyUSB*

# 권한 설정
sudo chmod 666 /dev/ttyUSB0
```

## 📝 변경 이력

- **2025-12-02**: 코드 리팩토링 완료
  - 모듈 분리 (1204줄 → 16개 모듈)
  - YAML 설정 파일 도입
  - 단위 테스트 추가 (58개)
